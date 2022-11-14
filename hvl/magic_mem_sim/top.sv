
module swamy_tb;

timeunit 1ns;
timeprecision 1ns;
/********************* Do not touch for proper compilation *******************/
// Instantiate Interfaces
tb_itf itf();

// Instantiate Testbench
source_tb tb(
    .magic_mem_itf(itf),
    .mem_itf(itf),
    .sm_itf(itf),
    .tb_itf(itf)
);

// For local simulation, add signal for Modelsim to display by default
// Note that this signal does nothing and is not used for anything
bit f;

/****************************** End do not touch *****************************/

/************************ Signals necessary for monitor **********************/
// This section not required until CP2


/*
The following signals need to be set:
Instruction and trap:
    rvfi.inst
    rvfi.trap

Regfile:
    rvfi.rs1_addr
    rvfi.rs2_add
    rvfi.rs1_rdata
    rvfi.rs2_rdata
    rvfi.load_regfile
    rvfi.rd_addr
    rvfi.rd_wdata

PC:
    rvfi.pc_rdata
    rvfi.pc_wdata

Memory:
    rvfi.mem_addr
    rvfi.mem_rmask
    rvfi.mem_wmask
    rvfi.mem_rdata
    rvfi.mem_wdata

Please refer to rvfi_itf.sv for more information.
*/

/**************************** End RVFIMON signals ****************************/

/********************* Assign Shadow Memory Signals Here *********************/
// This section not required until CP2
/*
The following signals need to be set:
icache signals:
    itf.inst_read
    itf.inst_addr
    itf.inst_resp
    itf.inst_rdata

dcache signals:
    itf.data_read
    itf.data_write
    itf.data_mbe
    itf.data_addr
    itf.data_wdata
    itf.data_resp
    itf.data_rdata

Please refer to tb_itf.sv for more information.
*/

/*********************** End Shadow Memory Assignments ***********************/

// Set this to the proper value
assign itf.registers = '{default: '0};

/*********************** Instantiate your design here ************************/
/*
The following signals need to be connected to your top level:
Clock and reset signals:
    itf.clk
    itf.rst

Burst Memory Ports:
    itf.mem_read
    itf.mem_write
    itf.mem_wdata
    itf.mem_rdata
    itf.mem_addr
    itf.mem_resp

Please refer to tb_itf.sv for more information.
*/

ibex_top #(
    .PMPEnable        ( 0                                ),
    .PMPGranularity   ( 0                                ),
    .PMPNumRegions    ( 4                                ),
    .MHPMCounterNum   ( 0                                ),
    .MHPMCounterWidth ( 40                               ),
    .RV32E            ( 0                                ),
    .RV32M            ( ibex_pkg::RV32MFast              ),
    .RV32B            ( ibex_pkg::RV32BNone              ),
    .RegFile          ( ibex_pkg::RegFileFF              ),
    .ICache           ( 0                                ),
    .ICacheECC        ( 0                                ),
    .ICacheScramble   ( 0                                ),
    .BranchPrediction ( 0                                ),
    .SecureIbex       ( 0                                ),
    .RndCnstLfsrSeed  ( ibex_pkg::RndCnstLfsrSeedDefault ),
    .RndCnstLfsrPerm  ( ibex_pkg::RndCnstLfsrPermDefault ),
    .DbgTriggerEn     ( 0                                ),
    .DmHaltAddr       ( 32'h1A110800                     ),
    .DmExceptionAddr  ( 32'h1A110808                     )
) dut (
    // Clock and reset
    .clk_i                  (itf.clk),
    .rst_ni                 (itf.rst),
    .test_en_i              (1'b1), //this makes sure that the chip doesnt do any clock gating
    .scan_rst_ni            (1'b1), // document told to tie this off to 1 as we don't have dft
    .ram_cfg_i              ('0), //all the DV files seem to be setting this to 0

    // Configuration
    .hart_id_i              ('0), //setting to 0 as that's what everyone else seems to be doing
    .boot_addr_i            ('0), // this should make it boot at address 0x80

    // Instruction memory interface
    .instr_req_o            (itf.inst_read),
    .instr_gnt_i            (itf.inst_resp),
    .instr_rvalid_i         (itf.inst_resp),
    .instr_addr_o           (itf.inst_addr),
    .instr_rdata_i          (itf.inst_rdata),
    .instr_rdata_intg_i     ('0), //that's what a bunch of examples did, so doesn't seem too unreasonable
    .instr_err_i            ('b0),

    // Data memory interface
    .data_req_o             (itf.data_read),
    .data_gnt_i             (itf.data_resp),
    .data_rvalid_i          (itf.data_resp),
    .data_we_o              (itf.data_write),
    .data_be_o              (itf.data_mbe),
    .data_addr_o            (itf.data_addr),
    .data_wdata_o           (itf.data_wdata),
    .data_wdata_intg_o      ('0), //the FPGA example sets this to '0, so so am I
    .data_rdata_i           (itf.data_rdata),
    .data_rdata_intg_i      ('0), 
    .data_err_i             ('b0),

    // Interrupt inputs
    //All of these were copied from the artya example
    .irq_software_i         (1'b0),
    .irq_timer_i            (1'b0),
    .irq_external_i         (1'b0),
    .irq_fast_i             (15'b0),
    .irq_nm_i               (1'b0),

    // Debug interface
    //again values copied from the FPGA example
    .debug_req_i            ('b0),
    .crash_dump_o           (),

    // Special control signals
    .fetch_enable_i         ('b1),
    .alert_minor_o          (),
    .alert_major_internal_o (),
    .alert_major_bus_o      (),
    .core_sleep_o           (),
    .double_fault_seen_o    (),

    // scramble 
    .scramble_key_valid_i   ('0),
    .scramble_key_i         ('0),
    .scramble_nonce_i       ('0),
    .scramble_req_o         ()

);
/***************************** End Instantiation *****************************/

endmodule
