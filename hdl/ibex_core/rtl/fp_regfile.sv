// Copyright lowRISC contributors.
// Copyright 2018 ETH Zurich and University of Bologna, see also CREDITS.md.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

`ifdef RISCV_FORMAL
  `define RVFI
`endif

`include "prim_assert.sv"

/*
Modified ibex register file to support 3 read ports
*/
module fp_register_file #(
    parameter regfile_e             RegFile             = RegFileFF,
    parameter bit                   RV32E               = 0,
    parameter int unsigned          DataWidth           = 32,
    parameter bit                   DummyInstructions   = 0,
    parameter bit                   WrenCheck           = 0,
    parameter logic [DataWidth-1:0] WordZeroVal         = '0
) (
    input logic clk,
    input logic rst_ni,

    input logic test_en_i,
    input logic dummy_instr_id,

    // Read port R1
    input logic [4:0] raddr_a_i,
    output logic [DataWidth-1:0] rdata_a_o,
    
    // Read port R2
    input logic [4:0] raddr_b_i,
    output logic [DataWidth-1:0] rdata_b_o,
    
    // Read port R3
    input logic [4:0] raddr_c_i,
    output logic [DataWidth-1:0] rdata_c_o,
    
    // Write port W1
    input logic [4:0] waddr_a_i,
    input logic [DataWidth-1:0] wdata_a_i,
    input logic we_a_i_fprf, 
    
    // This indicates whether spurious WE are detected.
    output logic err_o
);
    if (RegFile == RegFileFF) begin : gen_regfile_ff
        fp_register_file_ff #(
        .RV32E            (RV32E),
        .DataWidth        (RegFileDataWidth),
        .DummyInstructions(DummyInstructions),
        // SEC_CM: DATA_REG_SW.GLITCH_DETECT
        .WrenCheck        (RegFileWrenCheck),
        .WordZeroVal      (RegFileDataWidth'(prim_secded_pkg::SecdedInv3932ZeroWord))
        ) register_file_i (
        .clk_i (clk),
        .rst_ni(rst_ni),

        .test_en_i       (test_en_i),
        .dummy_instr_id_i(dummy_instr_id),

        .raddr_a_i(raddr_a_i),
        .rdata_a_o(rdata_a_o),
        .raddr_b_i(raddr_b_i),
        .rdata_b_o(rdata_b_o),
        .raddr_c_i(raddr_c_i),
        .rdata_c_o(rdata_c_o),
        .waddr_a_i(waddr_a_i),
        .wdata_a_i(wdata_a_i),
        .we_a_i   (we_a_i_fprf),
        .err_o    (err_o)
        );
    end else if (RegFile == RegFileFPGA) begin : gen_regfile_fpga
        fp_register_file_fpga #(
        .RV32E            (RV32E),
        .DataWidth        (RegFileDataWidth),
        .DummyInstructions(DummyInstructions),
        // SEC_CM: DATA_REG_SW.GLITCH_DETECT
        .WrenCheck        (RegFileWrenCheck),
        .WordZeroVal      (RegFileDataWidth'(prim_secded_pkg::SecdedInv3932ZeroWord))
        ) register_file_i (
        .clk_i (clk),
        .rst_ni(rst_ni),

        .test_en_i       (test_en_i),
        .dummy_instr_id_i(dummy_instr_id),

        .raddr_a_i(raddr_a_i),
        .rdata_a_o(rdata_a_o),
        .raddr_b_i(raddr_b_i),
        .rdata_b_o(rdata_b_o),
        .raddr_c_i(raddr_c_i),
        .rdata_c_o(rdata_c_o),
        .waddr_a_i(waddr_a_i),
        .wdata_a_i(wdata_a_i),
        .we_a_i   (we_a_i_fprf),
        .err_o    (err_o)
        );
    end else if (RegFile == RegFileLatch) begin : gen_regfile_latch
        fp_register_file_latch #(
        .RV32E            (RV32E),
        .DataWidth        (RegFileDataWidth),
        .DummyInstructions(DummyInstructions),
        // SEC_CM: DATA_REG_SW.GLITCH_DETECT
        .WrenCheck        (RegFileWrenCheck),
        .WordZeroVal      (RegFileDataWidth'(prim_secded_pkg::SecdedInv3932ZeroWord))
        ) register_file_i (
        .clk_i (clk),
        .rst_ni(rst_ni),

        .test_en_i       (test_en_i),
        .dummy_instr_id_i(dummy_instr_id),

        .raddr_a_i(raddr_a_i),
        .rdata_a_o(rdata_a_o),
        .raddr_b_i(raddr_b_i),
        .rdata_b_o(rdata_b_o),
        .raddr_c_i(raddr_c_i),
        .rdata_c_o(rdata_c_o),
        .waddr_a_i(waddr_a_i),
        .wdata_a_i(wdata_a_i),
        .we_a_i   (we_a_i_fprf),
        .err_o    (err_o)
        );
    end
endmodule

/*
Adapted from ibex_register_file_ff
*/
module fp_register_file_ff #(
    parameter int unsigned          DataWidth           = 32,
    parameter bit                   DummyInstructions   = 0,
    parameter bit                   WrenCheck           = 0,
    parameter logic [DataWidth-1:0] WordZeroVal         = '0
) (
    // Clock and Reset
    input  logic                    clk_i,
    input  logic                    rst_ni,

    input  logic                    test_en_i,
    input  logic                    dummy_instr_id_i,

    //Read port R1
    input  logic [4:0]              raddr_a_i,
    output logic [DataWidth-1:0]    rdata_a_o,

    //Read port R2
    input  logic [4:0]              raddr_b_i,
    output logic [DataWidth-1:0]    rdata_b_o,

    //Read port R3
    input  logic [4:0]              raddr_c_i,
    output logic [DataWidth-1:0]    rdata_c_o,

    // Write port W1
    input  logic [4:0]              waddr_a_i,
    input  logic [DataWidth-1:0]    wdata_a_i,
    input  logic                    we_a_i,

    // This indicates whether spurious WE are detected.
    output logic                    err_o
);

    localparam int unsigned ADDR_WIDTH = RV32E ? 4 : 5;
    localparam int unsigned NUM_WORDS  = 2**ADDR_WIDTH;

    logic [NUM_WORDS-1:0][DataWidth-1:0] rf_reg;
    logic [NUM_WORDS-1:1][DataWidth-1:0] rf_reg_q;
    logic [NUM_WORDS-1:0]                we_a_dec;

    always_comb begin : we_a_decoder
        for (int unsigned i = 0; i < NUM_WORDS; i++) begin
        we_a_dec[i] = (waddr_a_i == 5'(i)) ? we_a_i : 1'b0;
        end
    end

    // SEC_CM: DATA_REG_SW.GLITCH_DETECT
    // This checks for spurious WE strobes on the regfile.
    if (WrenCheck) begin : gen_wren_check
        // Buffer the decoded write enable bits so that the checker
        // is not optimized into the address decoding logic.
        logic [NUM_WORDS-1:0] we_a_dec_buf;
        prim_buf #(
        .Width(NUM_WORDS)
        ) u_prim_buf (
        .in_i(we_a_dec),
        .out_o(we_a_dec_buf)
        );

        prim_onehot_check #(
        .AddrWidth(ADDR_WIDTH),
        .AddrCheck(1),
        .EnableCheck(1)
        ) u_prim_onehot_check (
        .clk_i,
        .rst_ni,
        .oh_i(we_a_dec_buf),
        .addr_i(waddr_a_i),
        .en_i(we_a_i),
        .err_o
        );
    end else begin : gen_no_wren_check
        logic unused_strobe;
        assign unused_strobe = we_a_dec[0]; // this is never read from in this case
        assign err_o = 1'b0;
    end

    // No flops for R0 as it's hard-wired to 0
    for (genvar i = 1; i < NUM_WORDS; i++) begin : g_rf_flops
        always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rf_reg_q[i] <= WordZeroVal;
        end else if (we_a_dec[i]) begin
            rf_reg_q[i] <= wdata_a_i;
        end
        end
    end

    // With dummy instructions enabled, R0 behaves as a real register but will always return 0 for
    // real instructions.
    if (DummyInstructions) begin : g_dummy_r0
        // SEC_CM: CTRL_FLOW.UNPREDICTABLE
        logic                 we_r0_dummy;
        logic [DataWidth-1:0] rf_r0_q;

        // Write enable for dummy R0 register (waddr_a_i will always be 0 for dummy instructions)
        assign we_r0_dummy = we_a_i & dummy_instr_id_i;

        always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rf_r0_q <= WordZeroVal;
        end else if (we_r0_dummy) begin
            rf_r0_q <= wdata_a_i;
        end
        end

        // Output the dummy data for dummy instructions, otherwise R0 reads as zero
        assign rf_reg[0] = dummy_instr_id_i ? rf_r0_q : WordZeroVal;

    end else begin : g_normal_r0
        logic unused_dummy_instr_id;
        assign unused_dummy_instr_id = dummy_instr_id_i;

        // R0 is nil
        assign rf_reg[0] = WordZeroVal;
    end

    assign rf_reg[NUM_WORDS-1:1] = rf_reg_q[NUM_WORDS-1:1];

    assign rdata_a_o = rf_reg[raddr_a_i];
    assign rdata_b_o = rf_reg[raddr_b_i];
    assign rdata_c_o = rf_reg[raddr_c_i];

    // Signal not used in FF register file
    logic unused_test_en;
    assign unused_test_en = test_en_i;

endmodule


/*
Adapted from ibex_register_file_fpga
*/

module fp_register_file_fpga #(
    parameter bit                   RV32E               = 0,
    parameter int unsigned          DataWidth           = 32,
    parameter bit                   DummyInstructions   = 0,
    parameter bit                   WrenCheck           = 0,
    parameter logic [DataWidth-1:0] WordZeroVal         = '0
    ) (
    // Clock and Reset
    input  logic                    clk_i,
    input  logic                    rst_ni,

    input  logic                    test_en_i,
    input  logic                    dummy_instr_id_i,

    //Read port R1
    input  logic [          4:0]    raddr_a_i,
    output logic [DataWidth-1:0]    rdata_a_o,
    //Read port R2
    input  logic [          4:0]    raddr_b_i,
    output logic [DataWidth-1:0]    rdata_b_o,
    //Read port R3
    input  logic [          4:0]    raddr_c_i,
    output logic [DataWidth-1:0]    rdata_c_o,
    // Write port W1
    input  logic [          4:0]    waddr_a_i,
    input  logic [DataWidth-1:0]    wdata_a_i,
    input  logic                    we_a_i,

    // This indicates whether spurious WE are detected.
    output logic                    err_o
);

    localparam int ADDR_WIDTH = RV32E ? 4 : 5;
    localparam int NUM_WORDS = 2 ** ADDR_WIDTH;

    logic [DataWidth-1:0] mem[NUM_WORDS];
    logic we; // write enable if writing to any register other than R0

    // async_read a
    assign rdata_a_o = (raddr_a_i == '0) ? '0 : mem[raddr_a_i];

    // async_read b
    assign rdata_b_o = (raddr_b_i == '0) ? '0 : mem[raddr_b_i];


    // async_read c
    assign rdata_c_o = (raddr_c_i == '0) ? '0 : mem[raddr_c_i];

    // we select
    assign we = (waddr_a_i == '0) ? 1'b0 : we_a_i;

    // SEC_CM: DATA_REG_SW.GLITCH_DETECT
    // This checks for spurious WE strobes on the regfile.
    if (WrenCheck) begin : gen_wren_check
        // Since the FPGA uses a memory macro, there is only one write-enable strobe to check.
        assign err_o = we && !we_a_i;
    end else begin : gen_no_wren_check
        assign err_o = 1'b0;
    end

    // Note that the SystemVerilog LRM requires variables on the LHS of assignments within
    // "always_ff" to not be written to by any other process. However, to enable the initialization
    // of the inferred RAM32M primitives with non-zero values, below "initial" procedure is needed.
    // Therefore, we use "always" instead of the generally preferred "always_ff" for the synchronous
    // write procedure.
    always @(posedge clk_i) begin : sync_write
        if (we == 1'b1) begin
        mem[waddr_a_i] <= wdata_a_i;
        end
    end : sync_write

    // Make sure we initialize the BRAM with the correct register reset value.
    initial begin
        for (int k = 0; k < NUM_WORDS; k++) begin
        mem[k] = WordZeroVal;
        end
    end

    // Reset not used in this register file version
    logic unused_rst_ni;
    assign unused_rst_ni = rst_ni;

    // Dummy instruction changes not relevant for FPGA implementation
    logic unused_dummy_instr;
    assign unused_dummy_instr = dummy_instr_id_i;
    // Test enable signal not used in FPGA implementation
    logic unused_test_en;
    assign unused_test_en = test_en_i;

endmodule

/*
Adapted from ibex_register_file_latch
*/
module fp_register_file_latch #(
    parameter bit                   RV32E               = 0,
    parameter int unsigned          DataWidth           = 32,
    parameter bit                   DummyInstructions   = 0,
    parameter bit                   WrenCheck           = 0,
    parameter logic [DataWidth-1:0] WordZeroVal         = '0
) (
    // Clock and Reset
    input  logic                    clk_i,
    input  logic                    rst_ni,

    input  logic                    test_en_i,
    input  logic                    dummy_instr_id_i,

    //Read port R1
    input  logic [4:0]              raddr_a_i,
    output logic [DataWidth-1:0]    rdata_a_o,

    //Read port R2
    input  logic [4:0]              raddr_b_i,
    output logic [DataWidth-1:0]    rdata_b_o,

    //Read port R3
    input  logic [4:0]              raddr_c_i,
    output logic [DataWidth-1:0]    rdata_c_o,

    // Write port W1
    input  logic [4:0]              waddr_a_i,
    input  logic [DataWidth-1:0]    wdata_a_i,
    input  logic                    we_a_i,

    // This indicates whether spurious WE are detected.
    output logic                    err_o
);

    localparam int unsigned ADDR_WIDTH = RV32E ? 4 : 5;
    localparam int unsigned NUM_WORDS  = 2**ADDR_WIDTH;

    logic [DataWidth-1:0] mem[NUM_WORDS];

    logic [NUM_WORDS-1:0] waddr_onehot_a;

    logic [NUM_WORDS-1:1] mem_clocks;
    logic [DataWidth-1:0] wdata_a_q;

    // internal addresses
    logic [ADDR_WIDTH-1:0] raddr_a_int, raddr_b_int, raddr_c_int, waddr_a_int;

    assign raddr_a_int = raddr_a_i[ADDR_WIDTH-1:0];
    assign raddr_b_int = raddr_b_i[ADDR_WIDTH-1:0];
    assign raddr_c_int = raddr_c_i[ADDR_WIDTH-1:0];
    assign waddr_a_int = waddr_a_i[ADDR_WIDTH-1:0];

    logic clk_int;

    //////////
    // READ //
    //////////
    assign rdata_a_o = mem[raddr_a_int];
    assign rdata_b_o = mem[raddr_b_int];
    assign rdata_c_o = mem[raddr_c_int];

    ///////////
    // WRITE //
    ///////////
    // Global clock gating
    prim_clock_gating cg_we_global (
        .clk_i     ( clk_i     ),
        .en_i      ( we_a_i    ),
        .test_en_i ( test_en_i ),
        .clk_o     ( clk_int   )
    );

    // Sample input data
    // Use clk_int here, since otherwise we don't want to write anything anyway.
    always_ff @(posedge clk_int or negedge rst_ni) begin : sample_wdata
        if (!rst_ni) begin
        wdata_a_q   <= WordZeroVal;
        end else begin
        if (we_a_i) begin
            wdata_a_q <= wdata_a_i;
        end
        end
    end

    // Write address decoding
    always_comb begin : wad
        for (int i = 0; i < NUM_WORDS; i++) begin : wad_word_iter
        if (we_a_i && (waddr_a_int == 5'(i))) begin
            waddr_onehot_a[i] = 1'b1;
        end else begin
            waddr_onehot_a[i] = 1'b0;
        end
        end
    end

    // SEC_CM: DATA_REG_SW.GLITCH_DETECT
    // This checks for spurious WE strobes on the regfile.
    if (WrenCheck) begin : gen_wren_check
        // Buffer the decoded write enable bits so that the checker
        // is not optimized into the address decoding logic.
        logic [NUM_WORDS-1:0] waddr_onehot_a_buf;
        prim_buf #(
        .Width(NUM_WORDS)
        ) u_prim_buf (
        .in_i(waddr_onehot_a),
        .out_o(waddr_onehot_a_buf)
        );

        prim_onehot_check #(
        .AddrWidth(ADDR_WIDTH),
        .AddrCheck(1),
        .EnableCheck(1)
        ) u_prim_onehot_check (
        .clk_i,
        .rst_ni,
        .oh_i(waddr_onehot_a_buf),
        .addr_i(waddr_a_i),
        .en_i(we_a_i),
        .err_o
        );
    end else begin : gen_no_wren_check
        logic unused_strobe;
        assign unused_strobe = waddr_onehot_a[0]; // this is never read from in this case
        assign err_o = 1'b0;
    end

    // Individual clock gating (if integrated clock-gating cells are available)
    for (genvar x = 1; x < NUM_WORDS; x++) begin : gen_cg_word_iter
        prim_clock_gating cg_i (
            .clk_i     ( clk_int           ),
            .en_i      ( waddr_onehot_a[x] ),
            .test_en_i ( test_en_i         ),
            .clk_o     ( mem_clocks[x]     )
        );
    end

    // Actual write operation:
    // Generate the sequential process for the NUM_WORDS words of the memory.
    // The process is synchronized with the clocks mem_clocks[i], i = 1, ..., NUM_WORDS-1.
    for (genvar i = 1; i < NUM_WORDS; i++) begin : g_rf_latches
        always_latch begin
        if (mem_clocks[i]) begin
            mem[i] = wdata_a_q;
        end
        end
    end

    // With dummy instructions enabled, R0 behaves as a real register but will always return 0 for
    // real instructions.
    if (DummyInstructions) begin : g_dummy_r0
        // SEC_CM: CTRL_FLOW.UNPREDICTABLE
        logic                 we_r0_dummy;
        logic                 r0_clock;
        logic [DataWidth-1:0] mem_r0;

        // Write enable for dummy R0 register (waddr_a_i will always be 0 for dummy instructions)
        assign we_r0_dummy = we_a_i & dummy_instr_id_i;

        // R0 clock gate
        prim_clock_gating cg_i (
            .clk_i     ( clk_int     ),
            .en_i      ( we_r0_dummy ),
            .test_en_i ( test_en_i   ),
            .clk_o     ( r0_clock    )
        );

        always_latch begin : latch_wdata
        if (r0_clock) begin
            mem_r0 = wdata_a_q;
        end
        end

        // Output the dummy data for dummy instructions, otherwise R0 reads as zero
        assign mem[0] = dummy_instr_id_i ? mem_r0 : WordZeroVal;

    end else begin : g_normal_r0
        logic unused_dummy_instr_id;
        assign unused_dummy_instr_id = dummy_instr_id_i;

        assign mem[0] = WordZeroVal;
    end

`ifdef VERILATOR
    initial begin
        $display("Latch-based register file not supported for Verilator simulation");
        $fatal;
    end
`endif

endmodule