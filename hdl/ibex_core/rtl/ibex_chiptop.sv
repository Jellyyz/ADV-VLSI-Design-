module ibex_chiptop(
    input vss,
    input vdd,
    input clk_i,
    input rst_ni,
    input logic [7:0] instr_rdata_i, //concatenated instr data in
    input logic [7:0] data_rdata_i //total 20 pins only

);


logic inst_read;
logic inst_resp;
logic delay_iresp;
logic inst_addr;

logic data_req;
logic data_resp;
logic delay_dresp;
logic write_enable;
logic data_mbe;
logic data_addr;
logic data_wdata;




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
) ibex_c0re (
    // Clock and reset
    .clk_i                  (clk_i),
    .rst_ni                 (rst_ni),
    .test_en_i              (1'b1), //this makes sure that the chip doesnt do any clock gating
    .scan_rst_ni            (1'b1), // document told to tie this off to 1 as we don't have dft
    .ram_cfg_i              ('0), //all the DV files seem to be setting this to 0

    // Configuration
    .hart_id_i              ('0), //setting to 0 as that's what everyone else seems to be doing
    .boot_addr_i            ('0), // this should make it boot at address 0x80

    // Instruction memory interface
    .instr_req_o            (inst_read),
    .instr_gnt_i            (inst_resp),
    .instr_rvalid_i         (delay_iresp),
    .instr_addr_o           (inst_addr),
    .instr_rdata_i          ({24'h0,instr_rdata_i}),
    .instr_rdata_intg_i     ('0), //that's what a bunch of examples did, so doesn't seem too unreasonable
    .instr_err_i            ('b0),

    // Data memory interface
    .data_req_o             (data_req),
    .data_gnt_i             (data_resp),
    .data_rvalid_i          (delay_dresp),
    .data_we_o              (write_enable),
    .data_be_o              (data_mbe),
    .data_addr_o            (data_addr),
    .data_wdata_o           (data_wdata),
    .data_wdata_intg_o      ('0), //the FPGA example sets this to '0, so so am I
    .data_rdata_i           ({24'h0,data_rdata_i}),
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