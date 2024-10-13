module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    // AXI-Lite
    output  reg                     awready,
    output  reg                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  reg                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  reg                     rvalid,
    output  reg [(pDATA_WIDTH-1):0] rdata, 

    // AXI-Stream   
    // Input data
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                     ss_tready, 
    // Output data
    input   wire                     sm_tready, 
    output  reg                     sm_tvalid, 
    output  reg [(pDATA_WIDTH-1):0] sm_tdata, 
    output  reg                     sm_tlast, 
    
    // bram for tap RAM
    output  reg [3:0]               tap_WE,
    output  reg                     tap_EN,
    output  reg [(pDATA_WIDTH-1):0] tap_Di,
    output  reg [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  reg [3:0]               data_WE,
    output  reg                     data_EN,
    output  reg [(pDATA_WIDTH-1):0] data_Di,
    output  reg [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);


//FIR
localparam   IDLE      = 0,
             INPUT_1   = 1 ,
             INPUT_2   = 2 ,
             INPUT_3   = 3 ,
             INPUT_4   = 4 ,
             INPUT_5   = 5 ,
             INPUT_6   = 6 ,
             INPUT_7   = 7 ,
             INPUT_8   = 8 ,
             INPUT_9   = 9 ,
             INPUT_10  = 10,
             INPUT_11  = 11,

             WAIT      = 12,
             ADD_1     = 13,
             ADD_2     = 14,
             ADD_3     = 15,
             ADD_4     = 16,
             ADD_5     = 17,
             ADD_6     = 18,
             ADD_7     = 19,
             ADD_8     = 20,
             ADD_9     = 21,
             ADD_10    = 22,
             ADD_11    = 23,
             OUTPUT    = 24,
             RESET     = 25;

//AXILITE WRITE
localparam  WRITE_IDLE    = 0,
            WRITE_ADDR    = 1,
            WRITE_DATA_AP = 3,
            WRITE_DATA    = 2;

//AXILITE READ
localparam  READ_IDLE  = 0,
            READ_ADDR  = 1,
            READ_DATA  = 2,
            READ_DATA_AP = 3;            
reg [2:0] n_state_write, c_state_write, n_state_read, c_state_read;

reg [(pDATA_WIDTH-1):0] input_data, tap_data, n_tap_data;

reg [4:0] c_state, n_state;
reg done;
reg [(pDATA_WIDTH-1):0] temp;

reg [10:0] data_counter, n_data_counter;
reg [10:0] counter, n_counter;


reg [10:0] pointer, n_pointer;

reg [2:0] ap;

//ap signal
always @( posedge axis_clk or negedge axis_rst_n ) begin
    if(!axis_rst_n) begin
       ap[0] <= 0;
       ap[1] <= 0;
       ap[2] <= 1;
    end
    else begin
        ap[0] <= (c_state_write == WRITE_DATA_AP && wvalid == 1) ? wdata : ( c_state == INPUT_1 ) ? 0 : ap[0];
        ap[1] <= (done == 1 && n_state == RESET) ? 1 : (c_state_write == WRITE_DATA_AP && wvalid == 1) ? 0 : ap [1];
        ap[2] <= (done == 1 && n_state == RESET) ? 1 : (c_state_write == WRITE_DATA_AP && wvalid == 1) ? 0 : ap [2];
    end
end



always @(*) begin
    case (c_state_read)
        READ_IDLE : n_state_read = ( arvalid ) ? READ_ADDR : READ_IDLE;
        READ_ADDR : n_state_read = (araddr == 0) ? READ_DATA_AP : READ_DATA;
        READ_DATA : n_state_read = ( rvalid ) ? READ_IDLE : READ_DATA;
        READ_DATA_AP : n_state_read = ( rvalid ) ? READ_IDLE : READ_DATA_AP;
        default: n_state_read = c_state_read;
    endcase
end

always @( posedge axis_clk or negedge axis_rst_n ) begin
    if(!axis_rst_n) c_state_read <= READ_IDLE;
    else            c_state_read <= n_state_read;
end

always @(*) begin
    arready = ( c_state_read == READ_ADDR ) ? 1 : 0;
    rvalid  = ( (c_state_read == READ_DATA || c_state_read == READ_DATA_AP) && rready == 1 ) ? 1 : 0;
    if(rvalid == 1)begin
        if( c_state_read == READ_DATA )     rdata = tap_Do;
        else if ( c_state_read == READ_DATA_AP )begin
            rdata = ap;
        end
    end 
    else rdata = 0;
end




always @(*) begin
    case (c_state_write)
        WRITE_IDLE    : n_state_write = ( awvalid ) ? WRITE_ADDR : WRITE_IDLE;
        WRITE_ADDR    : n_state_write = (awaddr == 0) ? WRITE_DATA_AP :WRITE_DATA;
        WRITE_DATA    : n_state_write = ( wvalid ) ? WRITE_IDLE : WRITE_DATA;
        WRITE_DATA_AP : n_state_write = ( wvalid ) ? WRITE_IDLE : WRITE_DATA_AP;
        default: n_state_write = c_state_write;
    endcase
end

always @( posedge axis_clk or negedge axis_rst_n ) begin
    if(!axis_rst_n) c_state_write <= WRITE_IDLE;
    else            c_state_write <= n_state_write;
end

always @(*) begin
    awready = ( c_state_write == WRITE_ADDR ) ? 1 : 0;
    wready  = ( c_state_write == WRITE_DATA || c_state_write == WRITE_DATA_AP) ? 1 : 0;
end


//SRAM write in and read out according to AXI addr
reg [11:0] addr_save;


always @( posedge axis_clk or negedge axis_rst_n ) begin
    if(!axis_rst_n)begin
        tap_WE <= 0;
        tap_EN <= 0;
        tap_Di <= 0;
        tap_A  <= 0;
        addr_save <= 0;
    end
    else begin
        tap_EN    <= 1;
        addr_save <= ( awvalid )  ? awaddr : addr_save; 
        tap_A     <= ( wvalid && addr_save >= 32 ) ? addr_save - 'h20 : ( arvalid && c_state == RESET || c_state == IDLE) ? araddr - 'h20 : (n_state >= 12 && n_state <= 22) ? (n_state - 12)<<2 : tap_A ;
        tap_Di    <= ( wvalid ) ? wdata  : 0;
        tap_WE    <= ( wvalid && addr_save >= 32 ) ? 15 : 0;
    end  
end







always @(*) begin
    case (c_state)
        RESET    : n_state = (counter == 10) ? IDLE : RESET;
        IDLE     : n_state = ( ss_tvalid == 1 ) ? INPUT_1 : IDLE;
        INPUT_1  : n_state = WAIT;
        WAIT     : n_state = ADD_1 ;
        ADD_1    : n_state = ADD_2 ;
        ADD_2    : n_state = ADD_3 ;
        ADD_3    : n_state = ADD_4 ;
        ADD_4    : n_state = ADD_5 ;
        ADD_5    : n_state = ADD_6 ;
        ADD_6    : n_state = ADD_7 ;
        ADD_7    : n_state = ADD_8 ;
        ADD_8    : n_state = ADD_9 ;
        ADD_9    : n_state = ADD_10;
        ADD_10   : n_state = ADD_11;
        ADD_11   : n_state = OUTPUT;
        OUTPUT   : n_state = (done == 1 && sm_tready == 1) ? RESET : (done == 0 && sm_tready == 1) ? INPUT_1 : OUTPUT;
        default: n_state = c_state;
    endcase
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) done <= 0;
    else            done <= (c_state == IDLE && n_state == INPUT_1) ? 0 : (ss_tlast == 1 && ss_tready == 1) ? 1 : done;
end

always @( posedge axis_clk or negedge axis_rst_n ) begin
    if(!axis_rst_n) c_state <= RESET;
    else            c_state <= n_state;
end


always @( posedge axis_clk or negedge axis_rst_n ) begin
    if(!axis_rst_n)begin
        data_A  <= 0;
        data_EN <= 0;
        data_Di <= 0;
        counter <= 0;
        data_WE <= 0;
    end
    else begin
        counter <= ( c_state == RESET ) ? counter + 1 : counter;
        data_A  <= ( c_state == RESET ) ? counter<<2 : n_pointer<<2;
        data_EN <= 1;
        data_Di <= ( c_state == RESET ) ? 0 : (c_state == INPUT_1) ? ss_tdata : 0;
        data_WE <= ( c_state == INPUT_1 || c_state == RESET ) ? 15 : 0;
    end
end

//axi stream input
always @(*) begin
    ss_tready = (c_state == INPUT_1) ? 1 : 0;
end

//pointer 
always @(*) begin
    n_pointer = pointer;
    if(c_state == OUTPUT && n_state == INPUT_1)begin
        if (pointer == 10) n_pointer = 0;
        else n_pointer = pointer + 1;
    end
    else if (c_state >= 12 && c_state <= 22)begin
        if (pointer == 0) n_pointer = 10;
        else              n_pointer = pointer - 1;
    end                 
end

always @( posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) pointer <= 0;
    else           pointer <= n_pointer;
end






//MA
always @(posedge axis_clk or negedge axis_rst_n) begin
    if(!axis_rst_n) temp <= 0;
    else            temp <= (c_state >= 13 && c_state <= 23 ) ? temp + data_Do*tap_Do : ( c_state == OUTPUT && n_state == INPUT_1 ) ? 0 : temp;
end

//stream out
always @(*) begin
    sm_tlast  = ( done == 1 && c_state == OUTPUT ) ? 1 : 0;
    sm_tvalid = ( c_state == OUTPUT ) ? 1 : 0;
    sm_tdata  = ( c_state == OUTPUT ) ? temp : 0;
end



endmodule