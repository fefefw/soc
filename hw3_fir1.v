module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready, // address write ready
    output  wire                     wready,  // data write ready
    input   wire                     awvalid, 
    input   wire [(pADDR_WIDTH-1):0] awaddr,  // => Write the Address of Coefficient
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,   // => Write the Coefficient

    output  wire                     arready, // address read ready
    input   wire                     rready,  // data read ready
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,  // => Send the Address of Coefficient to Read
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,   // => Coefficient of that Address

    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, // AXI-Stream In X[n]
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 

    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, // AXI-Stream Out Y[t]
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

// Parameter Declaration --------------------------------------------------------------------------------------------------

parameter AXIL_IDLE  = 2'b00;
parameter AXIL_WRITE = 2'b01;
parameter AXIL_RADDR = 2'b10;
parameter AXIL_RDATA = 2'b11;

parameter AXIS_IDLE  = 2'b00;
parameter AXIS_LOAD  = 2'b01;
parameter AXIS_COMP  = 2'b10;
parameter AXIS_OUT   = 2'b11;


// reg/wire Declaration --------------------------------------------------------------------------------------------------
reg [1:0] axis_state_r, axis_state_w;
reg [1:0] axil_state_r, axil_state_w;


reg [2:0] ap_config_reg_r, ap_config_reg_w; // [0] ap_start, [1] ap_done, [2] ap_idle
reg [9:0] data_length_r, data_length_w;

reg [3:0] fir_count_r, fir_count_w;
reg [3:0] write_pointer_r, write_pointer_w;
reg [3:0] read_pointer_r, read_pointer_w;


reg signed [31:0] accumulation_r, accumulation_w;

reg last_input_r, last_input_w;

reg [3:0] rst_data_count_r, rst_data_count_w;
// Continuous Assignments  --------------------------------------------------------------------------------------------------


assign tap_WE = (axil_state_r == AXIL_WRITE && awaddr >= 32'h0000_0020 && awaddr <= 32'h0000_0020 + Tape_Num<<2)? 4'b1111: 4'b0000;  // only WE = 1 when write to tap, not to ap_start or data_length
assign tap_EN = 1'b1;
assign tap_Di = wdata;
assign tap_A  = (awready)? awaddr - 32'h0000_0020 : 
                (arvalid && axis_state_r == AXIS_IDLE)? araddr - 32'h0000_0020 : 
                fir_count_r << 2; 


assign data_WE = (axis_state_r == AXIS_LOAD || axis_state_r == AXIS_IDLE)? 4'b1111 : 4'b0000;
assign data_EN = 1'b1;
assign data_Di = (axis_state_r == AXIS_IDLE)? 32'b0 : ss_tdata;
assign data_A  = (axis_state_r == AXIS_COMP)? read_pointer_r << 2 : 
                 (axis_state_r == AXIS_LOAD)? write_pointer_r << 2 :
                 (rst_data_count_r <= 4'd11)? rst_data_count_r << 2: fir_count_r << 2;

assign awready = (axil_state_r == AXIL_WRITE)? 1'b1 : 1'b0;
assign wready  = (axil_state_r == AXIL_WRITE)? 1'b1 : 1'b0;

assign arready = (axil_state_r == AXIL_RADDR)? 1'b1 : 1'b0;
assign rvalid  = (axil_state_r == AXIL_RDATA)? 1'b1 : 1'b0;
assign rdata   = (axil_state_r == AXIL_RDATA)?  ((araddr == 32'h0000_0000)? {29'b0, ap_config_reg_r} : 
                                                (araddr == 32'h0000_0010)? data_length_r : 
                                                (araddr >= 32'h0000_0020 && (araddr <= 32'h0000_0020 + Tape_Num<<2))? tap_Do : 32'b0 ): 
                                                32'b0;


assign ss_tready = (axis_state_r == AXIS_LOAD)? 1'b1 : 1'b0;


//assign read_pointer = (write_pointer_r < fir_count_r)? (4'b1011 - (fir_count_r - write_pointer_r)) : (write_pointer_r - fir_count_r);

assign sm_tvalid = (axis_state_r == AXIS_OUT)? 1'b1 : 1'b0;
assign sm_tdata = accumulation_r;
assign sm_tlast = (axis_state_r == AXIS_OUT)? last_input_r : 1'b0;

// Procedure Assignment --------------------------------------------------------------------------------------------------

always @(*) begin
    ap_config_reg_w = ap_config_reg_r;
    if(axil_state_r == AXIL_WRITE && awaddr == 32'h0000_0000) begin
        ap_config_reg_w = wdata[2:0];
    end
    else if(rready == 1'b1 && araddr == 32'h0000_0000) begin
        ap_config_reg_w[1] = 1'b0; // write-one-clear 
    end
    else begin
        if(axis_state_r == AXIS_LOAD) ap_config_reg_w[0] = 1'b0;  // reset ap_start

        if(last_input_r == 1'b1 && axis_state_r == AXIS_OUT) begin
            ap_config_reg_w[1] = 1'b1;  // set ap_done
            ap_config_reg_w[2] = 1'b1;  // set ap_idle
        end
    end

end

always @(posedge axis_clk or negedge axis_rst_n ) begin
    if(~axis_rst_n) begin
        ap_config_reg_r <= 3'b100;  // only ap_idle is asserted when reset
    end else begin
        ap_config_reg_r <= ap_config_reg_w;
    end
end

always @(*) begin
    write_pointer_w = write_pointer_r;
    if(axis_state_r == AXIS_LOAD && ss_tvalid) begin
        if(write_pointer_r == 4'd10) write_pointer_w = 4'd0;
        else write_pointer_w = write_pointer_r + 4'd1;
    end
    else if(axis_state_r == AXIS_OUT &&  last_input_r == 1'b1) begin
        write_pointer_w = 4'd0;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        write_pointer_r <= 4'b0;
    end else begin
        write_pointer_r <= write_pointer_w;
    end
end

always @(*) begin
    read_pointer_w = read_pointer_r;
    if(axis_state_r == AXIS_LOAD) begin
        if(write_pointer_r == 4'd0) begin
            read_pointer_w = 4'd10;
        end
        else begin
            read_pointer_w = write_pointer_r - 1'd1;
        end
    end
    else if(axis_state_r == AXIS_COMP) begin
        if(read_pointer_r == 4'd0) begin
            read_pointer_w = 4'd10;
        end
        else begin
            read_pointer_w = read_pointer_r - 1'd1;
        end
    end
    else read_pointer_w = 4'd0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        read_pointer_r <= 4'b0;
    end else begin
        read_pointer_r <= read_pointer_w;
    end
end


always @(*) begin
    fir_count_w = fir_count_r;
    
    if(axis_state_r == AXIS_LOAD || axis_state_r == AXIS_COMP) begin
        if(fir_count_r == 4'd10) begin
            fir_count_w = 4'd0;
        end
        else begin
            fir_count_w = fir_count_r + 1'd1;
        end
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        fir_count_r <= 4'b0;
    end else begin
        fir_count_r <= fir_count_w;
    end
end



always @(*) begin
    accumulation_w = accumulation_r;
    
    if(axis_state_r == AXIS_COMP) begin
        accumulation_w = accumulation_r + $signed(data_Do) * $signed(tap_Do);
    end
    else if(axis_state_r == AXIS_OUT) begin
        accumulation_w = 32'd0;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        accumulation_r <= 32'd0;
    end
    else begin
        accumulation_r <= accumulation_w;
    end
end


always @(*) begin
    last_input_w = last_input_r;
    if(axis_state_r == AXIS_LOAD) last_input_w = ss_tlast;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        last_input_r <= 1'b0;
    end
    else begin
        last_input_r <= last_input_w;
    end
end

always @(*) begin
    rst_data_count_w = rst_data_count_r;
    
    if(ap_config_reg_r == 3'b001) begin
        rst_data_count_w = rst_data_count_r + 1'd1;
    end
    else if(ap_config_reg_r == 3'b100) begin
        rst_data_count_w = 4'd0;
    end
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        rst_data_count_r <= 4'b0;
    end else begin
        rst_data_count_r <= rst_data_count_w;
    end
end

always @(*) begin
    data_length_w = data_length_r;
    if(axil_state_r == AXIL_WRITE && awaddr == 32'h0010) begin
        data_length_w = wdata;
    end
end



always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        data_length_r <= 0;
    end else begin
        data_length_r <= data_length_w;
    end
end



// ----------------------------------------------------------------------------------------------------------------------
// FSM
always @(*) begin
    axis_state_w = axis_state_r;
    case(axis_state_r)
        AXIS_IDLE: begin
            if( rst_data_count_r == 4'd10 && ss_tvalid == 1'b1) begin
                axis_state_w = AXIS_LOAD;
            end else begin
                axis_state_w = AXIS_IDLE;
            end
        end
        AXIS_LOAD: begin
            axis_state_w = AXIS_COMP;
        end
        AXIS_COMP: begin
            if(fir_count_r == 4'd10) begin
                axis_state_w = AXIS_OUT;
            end
            else begin
                axis_state_w = AXIS_COMP;
            end
        end
        AXIS_OUT: begin
            if(last_input_r == 1'b1) begin
                axis_state_w = AXIS_IDLE;
            end
            else begin
                axis_state_w = AXIS_LOAD;
            end
        end
    endcase
end

always @(*) begin
    axil_state_w = axil_state_r;
    case(axil_state_r)
        AXIL_IDLE: begin
            if(awvalid && wvalid) begin
                axil_state_w = AXIL_WRITE;
            end else if(arvalid) begin
                axil_state_w = AXIL_RADDR;
            end
        end
        AXIL_WRITE: begin
            axil_state_w = AXIL_IDLE;
        end
        AXIL_RADDR: begin
            if(arready) begin
                axil_state_w = AXIL_RDATA;
            end
            else begin
                axil_state_w = AXIL_RADDR;
            end
        end
        AXIL_RDATA: begin
            if(rready) begin
                axil_state_w = AXIL_IDLE;
            end
            else begin
                axil_state_w = AXIL_RDATA;
            end
        end
    endcase
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(~axis_rst_n) begin
        axis_state_r <= AXIS_IDLE;
        axil_state_r <= AXIL_IDLE;
    end 
    else begin
        axis_state_r <= axis_state_w;
        axil_state_r <= axil_state_w;
    end
end



endmodule