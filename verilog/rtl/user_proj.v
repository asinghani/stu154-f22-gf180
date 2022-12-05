// SPDX-License-Identifier: Apache-2.0
`default_nettype none

`include "beepboop_fast.sv"
`include "beepboop.sv"
`include "tt.sv"

module user_proj (
`ifdef USE_POWER_PINS
    inout vdd,	// User area 1 1.8V supply
    inout vss,	// User area 1 digital ground
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
    input  [63:0] la_data_in,
    output [63:0] la_data_out,
    input  [63:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk = wb_clk_i;
    wire rst = wb_rst_i;
    assign irq = 3'b000;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    logic [7:0] ttA_out[0:7];
    assign io_out[28:21] = ttA_out[io_in[20:18]];
    assign io_oeb[28:21] = 0;
    assign io_oeb[20:18] = 1;
    assign io_oeb[17:10] = 1;
    wire [7:0] ttA_in = io_in[17:10];

    // Instantiate TT projects
    tt_ngaertne ttA_0 ( .io_in(ttA_in), .io_out(ttA_out[0]) );
    tt_jrecta ttA_1 ( .io_in(ttA_in), .io_out(ttA_out[1]) );
    tt_sophiali ttA_2 ( .io_in(ttA_in), .io_out(ttA_out[2]) );
    tt_mgee3 ttA_3 ( .io_in(ttA_in), .io_out(ttA_out[3]) );
    tt_jxlu ttA_4 ( .io_in(ttA_in), .io_out(ttA_out[4]) );
    tt_qilins ttA_5 ( .io_in(ttA_in), .io_out(ttA_out[5]) );
    beepboop ttA_6 ( .io_in(ttA_in), .io_out(ttA_out[6]) );

    beepboop_fast beepboop (
        .clock(clk),
        .reset(io_in[30]),
        .btn(io_in[31]),
        .io_out(io_out[37:32])
    );

    assign io_oeb[37:32] = 0;
    assign io_oeb[31:30] = 1;

endmodule

`default_nettype wire
