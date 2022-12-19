module ibex_chiptop(
    input    vss,
    input    vdd,
    input    clk_i,
    input    rst_ni,
    input    logic [7:0] ext_sram_rdata,
    output   logic [7:0] ext_sram_wdata,
    output   logic [15:0] ext_sram_addr,
    output   ext_sram_read,
    output   ext_sram_write

);


logic [31:0] ext_sram_addr_32;

assign ext_sram_addr = ext_sram_addr_32[15:0];

logic instr_req;
logic instr_gnt;
logic instr_rvalid;
logic [31:0] instr_addr;
logic [31:0] instr_rdata;

logic data_req;
logic data_rvalid;
logic data_gnt;
logic data_we;
logic [3:0] data_be;
logic [31:0] data_addr;
logic [31:0] data_wdata;
logic [31:0] data_rdata;



mmu mmu(
    .clk                (clk_i),
    .rst_ni             (rst_ni),
    .instr_req_i        (instr_req),
    .instr_gnt_o        (instr_gnt),
    .instr_rvalid_o     (instr_rvalid),
    .instr_addr_i       (instr_addr),
    .instr_rdata_o      (instr_rdata),

    .data_req_i         (data_req),
    .data_gnt_o         (data_gnt),
    .data_rvalid_o      (data_rvalid),
    .data_we_i          (data_we),
    .data_be_i          (data_be),
    .data_addr_i        (data_addr),
    .data_wdata_i       (data_wdata),
    .data_rdata_o       (data_rdata),

    .ext_sram_rdata_i   (ext_sram_rdata),
    .ext_sram_wdata_o   (ext_sram_wdata),
    .ext_sram_addr_o    (ext_sram_addr_32),
    .ext_sram_read_o    (ext_sram_read),
    .ext_sram_write_o   (ext_sram_write)
);

// // sqrt 
// logic [31:0] dw_sqrt_z;
// logic [7:0] dw_sqrt_status; 

// // add sub 
// logic [31:0] dw_addsub_z; 
// logic [7:0] dw_addsub_status; 

// // cmp 
// logic altb, agtb, aeqb, unordered; 
// logic [31:0] z0, z1; 
// logic [7:0] status; 

// // div 
// logic [31:0] dw_div_z; 
// logic [7:0] dw_div_status; 

// // mult 
// logic [31:0] dw_mult_z; 
// logic [7:0] dw_mult_status; 

// // flt2i
// logic [31:0] dw_flt2i_z; 
// logic [7:0] dw_flt2i_status; 

// // i2flt 
// logic [31:0] dw_i2flt_z; 
// logic [7:0] dw_i2flt_status; 

// // mac 
// logic [31:0] dw_mac_z; 
// logic [7:0] dw_mac_status; 


// // Designware Section 
// DW_fp_sqrt_inst DW_fp_sqrt_inst(
//     .inst_a(32'b11111111111111111111111111111111), 
//     .inst_rnd(3'b111), 
//     .z_inst(dw_sqrt_z), 
//     .status_inst(dw_sqrt_status)
// ); 

// DW_fp_addsub_inst DW_fp_addsub_inst(
//     .inst_a(32'b11111111111111111111111111111111),
//     .inst_b(32'b11111111111111111111111111111111), 
//     .inst_rnd(3'b111), 
//     .inst_op(1'b0),
//     .z_inst(dw_addsub_z), 
//     .status_inst(dw_addsub_status) 
// ); 

// DW_fp_cmp_inst DW_fp_cmp_inst(
//     .inst_a(32'b11111111111111111111111111111111),
//     .inst_b(32'b11111111111111111111111111111111), 
//     .inst_zctr(1'b0), 
//     .altb_inst(altb), .agtb_inst(agtb), .aeqb_inst(aeqb), .unordered_inst(unordered),
//     .z0_inst(z0), .z1_inst(z1), 
//     .status0_inst(status) 
// );

// DW_fp_div_inst DW_fp_div_inst(
//     .inst_a(32'b11111111111111111111111111111111), 
//     .inst_b(32'b11111111111111111111111111111111), 
//     .inst_rnd(3'b111), 
//     .z_inst(dw_div_z), 
//     .status_inst(dw_div_status)
// ); 

// DW_fp_mult_inst DW_fp_mult_inst(
//     .inst_a(32'b11111111111111111111111111111111), 
//     .inst_b(32'b11111111111111111111111111111111), 
//     .inst_rnd(3'b111), 
//     .z_inst(dw_mult_z), 
//     .status_inst(dw_mult_status)
// ); 

// DW_fp_flt2i_inst DW_fp_flt2i_inst(
//     .inst_a(32'b11111111111111111111111111111111), 
//     .inst_rnd(3'b111), 
//     .z_inst(dw_flt2i_z), 
//     .status_inst(dw_flt2i_status)
// );

// DW_fp_i2flt_inst DW_fp_i2flt_inst(
//     .inst_a(32'b11111111111111111111111111111111), 
//     .inst_rnd(3'b111), 
//     .z_inst(dw_i2flt_z), 
//     .status_inst(dw_i2flt_status)
// ); 

// DW_fp_mac_inst DW_fp_mac_inst(
//     .inst_a(32'b11111111111111111111111111111111), 
//     .inst_b(32'b11111111111111111111111111111111), 
//     .inst_c(32'b11111111111111111111111111111111), 
//     .inst_rnd(3'b111), 
//     .z_inst(dw_mac_z), 
//     .status_inst(dw_mac_status)


// ); 

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
    .SecureIbex       ( 0                                ),
    .RndCnstLfsrSeed  ( ibex_pkg::RndCnstLfsrSeedDefault ),
    .RndCnstLfsrPerm  ( ibex_pkg::RndCnstLfsrPermDefault ),
    .DbgTriggerEn     ( 0                                ),
    .DmHaltAddr       ( 32'h1A110800                     ),
    .DmExceptionAddr  ( 32'h1A110808                     )
) ibex_core (
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
    .instr_req_o            (instr_req),
    .instr_gnt_i            (instr_gnt),
    .instr_rvalid_i         (instr_rvalid),
    .instr_addr_o           (instr_addr),
    .instr_rdata_i          (instr_rdata),
    .instr_rdata_intg_i     (7'b0), //that's what a bunch of examples did, so doesn't seem too unreasonable
    .instr_err_i            (1'b0),

    // Data memory interface
    .data_req_o             (data_req),
    .data_gnt_i             (data_gnt),
    .data_rvalid_i          (data_rvalid),
    .data_we_o              (data_we),
    .data_be_o              (data_be),
    .data_addr_o            (data_addr),
    .data_wdata_o           (data_wdata),
    .data_wdata_intg_o      (7'b0), //the FPGA example sets this to '0, so so am I
    .data_rdata_i           (data_rdata),
    .data_rdata_intg_i      (7'b0), 
    .data_err_i             (1'b0),

    // Interrupt inputs
    //All of these were copied from the artya example
    .irq_software_i         (1'b0),
    .irq_timer_i            (1'b0),
    .irq_external_i         (1'b0),
    .irq_fast_i             (15'b0),
    .irq_nm_i               (1'b0),

    // Debug interface
    //again values copied from the FPGA example
    .debug_req_i            (1'b0),
    .crash_dump_o           (),

    // Special control signals
    .fetch_enable_i         (4'b1),
    .alert_minor_o          (),
    .alert_major_internal_o (),
    .alert_major_bus_o      (),
    .core_sleep_o           (),
    .double_fault_seen_o    (),

    // scramble 
    .scramble_key_valid_i   (1'b0),
    .scramble_key_i         (128'h0),
    .scramble_nonce_i       (64'h0),
    .scramble_req_o         ()

);

endmodule