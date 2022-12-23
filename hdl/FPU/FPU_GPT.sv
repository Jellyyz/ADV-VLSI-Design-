import ibex_pkg::*;

module ibex_FPU(
    // Clock input
    input                   clk_i,
    // Floating point rounding mode
    input   logic [2:0]     fp_rounding_mode,
    // Floating point operation to be performed
    input   fpu_op_e        fp_op,
    // First operand for floating point operation
    input   logic [31:0]    rs1_i,
    // First operand for integer operation
    input   logic [31:0]    rs1_int_i,
    // Second operand for floating point operation
    input   logic [31:0]    rs2_i,
    // Third operand for floating point operation
    input   logic [31:0]    rs3_i,
    // Address of register to write result to
    input   logic [4:0]     rd_addr_i,
    // Result of floating point operation to be written to floating point register file
    output  logic [31:0]    fp_regfile_wdata_o,
    // Address of register in floating point register file to write result to
    output  logic [4:0]     fp_regfile_addr_o,
    // Write enable signal for floating point register file
    output                  fp_regfile_write_o,
    // Result of floating point operation to be written to integer register file
    output  logic [31:0]    int_regfile_wdata_o,
    // Address of register in integer register file to write result to
    output  logic [4:0]     int_regfile_addr_o,
    // Write enable signal for integer register file
    output                  int_regfile_write_o
);

// Intermediate signals and status flags for square root operation
logic [31:0] sqrt_op_a;
logic [31:0] sqrt_result;
logic [7:0] sqrt_status;

// Intermediate signals and status flags for add/subtract operation
logic [31:0] add_sub_op_a;
logic [31:0] add_sub_op_b;
// Selects between add and subtract
logic        add_sub_select;
logic [31:0] add_sub_result;
logic [7:0] add_sub_status;

// Intermediate signals and status flags for floating point compare operation
logic [31:0] fpcmp_op_a;
logic [31:0] fpcmp_op_b;
// Comparison results: rs1 > rs2, rs1 = rs2, rs1 < rs2
logic fp_cmp_rs1_gt_rs2, fp_cmp_rs1_eq_rs2, fp_cmp_rs1_lt_rs2;
logic [31:0] min_of_rs1_rs2;
logic [31:0] max_of_rs1_rs2;
// Indicates if one of the operands is NaN
logic fp_cmp_NaN;
logic [7:0] fpcmp_status;

// Intermediate signals and status flags for multiply operation
logic [31:0] mult_op_a;
logic [31:0] mult_op_b;
logic [31:0] mult_result;
logic [7:0] mult_status;

// Intermediate signals and status flags for divide operation
logic [31:0] div_op_a;
logic [31:0] div_op_b;
logic [31:0] div_result;
logic [7:0] div_status;

// Intermediate signals and status flags for floating point to integer conversion operation
logic [31:0] fp2int_op_a;
logic [31:0] fp2int_result;
logic [7:0] fp2int_status;

// Intermediate signals and status flags for integer to floating point conversion operation
logic [31:0] int2fp_op_a;
logic [31:0] int2fp_result;
logic [31:0] int2fp_result_unsigned;
logic [7:0] int2fp_status;
logic [7:0] int2fp_status_unsigned;

// Intermediate signals and status flags for multiply-accumulate operation
logic [31:0] mac_op_a;
logic [31:0] mac_op_b;
logic [31:0] mac_op_c;
logic [31:0] mac_result;
logic [7:0] mac_status;

// Intermediate signal and result for floating point class operation
logic [31:0] fpclass_op_a;
logic [31:0] fpclass_result;

// Assignment statements for all the input operands
assign sqrt_op_a    = rs1_i;
assign add_sub_op_a = rs1_i;
assign add_sub_op_b = rs2_i;
assign fpcmp_op_a   = rs1_i;
assign fpcmp_op_b   = rs2_i;
assign mult_op_a    = rs1_i;
assign mult_op_b    = rs2_i;
assign div_op_a     = rs1_i;
assign div_op_b     = rs2_i;
assign mac_op_a     = rs1_i;
assign mac_op_b     = rs2_i;
assign mac_op_c     = rs3_i;
assign fpclass_op_a = rs1_i;
assign fp2int_op_a  = rs1_i;
assign int2fp_op_a  = rs1_int_i;

