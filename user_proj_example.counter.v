// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype wire
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */
`define MPRJ_IO_PADS 38
module user_proj_example #(
    parameter BITS = 32,
    parameter DELAYS=10
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    //wire [`MPRJ_IO_PADS-1:0] io_in;
    //wire [`MPRJ_IO_PADS-1:0] io_out;
    //wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [31: 0] addr;
    reg  [3: 0]  counter_r;
    wire [3: 0]  counter_w;
    wire [3: 0]  WE;
    wire         EN;
    wire [31: 0] data_i;
    wire [31: 0] data_o;
    wire         n_ready,  valid, s_start;
    reg          ready;
    assign irq = 3'b000;
    assign valid = (wbs_adr_i[31: 24] == 32'h38)? 1'b1 : 1'b0;
    assign s_start = valid & wbs_stb_i & wbs_cyc_i;
    
    assign WE = wbs_sel_i & {4{wbs_we_i}};
    assign data_i = wbs_dat_i;
    assign wbs_dat_o = data_o;
    assign addr = wbs_adr_i;
    assign wbs_ack_o = ready;
    assign n_ready = (counter_r == DELAYS-1)? 1'b1 : 1'b0;
    assign counter_w = (counter_r == DELAYS-1)? 4'b0000 : counter_r + 1'b1;

    always@(posedge wb_clk_i or posedge wb_rst_i) begin
        if(wb_rst_i) begin
            ready <= 1'b0;
        end
        else begin
            ready <= n_ready;
        end
    end
    
    always@(posedge wb_clk_i or posedge wb_rst_i) begin
        if(wb_rst_i) begin
            counter_r <= 0;
        end
        else begin
            if(s_start && !ready) begin
                counter_r <= counter_w;
            end
            else begin
                counter_r <= 0;
            end
            
        end
    end

    
        
    bram user_bram (
        .CLK(wb_clk_i),
        .WE0(WE),
        .EN0(s_start),
        .Di0(data_i),
        .Do0(data_o),
        .A0(addr)
    );

endmodule



`default_nettype wire
