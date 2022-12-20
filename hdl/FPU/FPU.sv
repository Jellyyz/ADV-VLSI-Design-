import ibex_pkg::*


module ibex_FPU(

    input                   clk_i,
    input   logic [2:0]     fp_rounding_mode,          
    input   fpu_op_e        fp_op
    input   logic [31:0]    rs1_i,
    input   logic [31:0]    rs1_int_i,
    input   logic [31:0]    rs2_i,
    input   logic [31:0]    rs3_i,
    input   logic [4:0]     rd_addr_i
    output  logic [31:0]    fp_regfile_wdata_o,
    output  logic [4:0]     fp_regfile_addr_o,
    output                  fp_regfile_write_o,
    output  logic [31:0]    int_regfile_wdata_o,
    output  logic [4:0]     int_regfile_addr_o,
    output                  int_regfile_write_o,
);

logic [31:0] sqrt_op_a;
logic [31:0] sqrt_result;
logic [7:0] sqrt_status;

logic [31:0] add_sub_op_a;
logic [31:0] add_sub_op_b;
logic        add_sub_select; //select between add and sub
logic [31:0] add_sub_result;
logic [7:0] add_sub_status;

logic [31:0] fpcmp_op_a;
logic [31:0] fpcmp_op_b;
logic fp_cmp_rs1_gt_rs2, fp_cmp_rs1_eq_rs2, fp_cmp_rs1_lt_rs2;
logic [31:0] min_of_rs1_rs2;
logic [31:0] max_of_rs1_rs2;
logic fp_cmp_NaN;
logic [7:0] fpcmp_status;

logic [31:0] mult_op_a;
logic [31:0] mult_op_b;
logic [31:0] mult_result;
logic [7:0] mult_status;

logic [31:0] div_op_a;
logic [31:0] div_op_b;
logic [31:0] div_result;
logic [7:0] div_status;

logic [31:0] fp2int_op_a;
logic [31:0] fp2int_result;
logic [7:0] fp2int_status;

logic [31:0] int2fp_op_a;
logic [31:0] int2fp_result;
logic [31:0] int2fp_result_unsigned;
logic [7:0] int2fp_status;
logic [7:0] int2fp_status_unsigned;

logic [31:0] mac_op_a;
logic [31:0] mac_op_b;
logic [31:0] mac_op_c;
logic [31:0] mac_result;
logic [7:0] mac_status;


logic [31:0] fpclass_op_a;
logic [31:0] fpclass_result;

//assignment statements for all the input operands

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
assign mac_op_c     = rs3_i
assign fp2int_op_a  = rs1_i;
assign int2fp_op_a  = rs1_int_i;
assign fpclass_op_a = rs1_i;

//default functions

function void set_defaults();
    fp_regfile_wdata_o      = 32'b0;
    fp_regfile_addr_o       = rd_addr_i;
    fp_regfile_write_o      = 1'b0;
    int_regfile_wdata_o     = 32'b0;
    int_regfile_addr_o      = rd_addr_i;
    int_regfile_write_o     = 1'b0;
    add_sub_select          = 1'b0;

    mac_op_a                = rs1;
    mac_op_b                = rs2;
    mac_op_c                = rs3;
endfunction


always_comb begin
    set_defaults();
    case(fp_op)

        FPU_ADD: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = add_sub_result;
        end

        FPU_SUB: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = add_sub_result;
        end

        FPU_MUL: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = mult_result;
        end

        FPU_DIV: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = div_result;
        end

        FPU_SQRT: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = sqrt_result;
        end
 
        FPU_MIN: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = min_of_rs1_rs2;
        end

        FPU_MAX: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = max_of_rs1_rs2;
        end

    
        FPU_MADD: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = mac_result;
        end

        FPU_NMADD: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = {~mac_result[31], mac_result[30:0]};
        end

        FPU_MSUB: begin
            fp_regfile_write_o      = 1'b1;
            mac_op_c                = {~rs3[31], rs3[30:0]};
            fp_regfile_wdata_o      = mac_result;
        end

        FPU_NMSUB: begin
            fp_regfile_write_o      = 1'b1;
            mac_op_c                = {~rs3[31], rs3[30:0]};
            fp_regfile_wdata_o      = {~mac_result[31], mac_result[30:0]};
        end

    
        FPU_INT2FLOAT: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = int2fp_result;
        end

        FPU_FLOAT2INT: begin
            int_regfile_wdata_o     = 1'b1;
            int_regfile_wdata_o     = fp2int_result;
        end


        FPU_INT2FLOAT_U: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = int2fp_result_unsigned;
        end

        //TODO: Figure out what the best way to do unsigned flt2int conversion - or not do it at all lol
        FPU_FLOAT2INT_U: begin
            int_regfile_wdata_o     = 1'b1;
            int_regfile_wdata_o     = fp2int_result;
        end
    
        FPU_SGNJ: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = {rs2[31], rs1[30:0]};
        end

        FPU_SGNJ_N: begin//negated sign-injection
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = {~rs2[31], rs1[30:0]};
        end 
        
        FPU_SGNJ_X: begin//xor sign-injection
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = {rs2[31]^rs1[31], rs1[30:0]};
        end

        FPU_MOVE_INT2FLOAT: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = rs1_int_i;
        end 

        FPU_MOVE_FLOAT2INT: begin
            int_regfile_write_o     = 1'b1;
            int_regfile_wdata_o     = rs1_i;
        end

        FPU_CMP_EQ: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = {31{1'b0}, fp_cmp_rs1_eq_rs2};
        end

        FPU_CMP_LT: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = {31{1'b0}, fp_cmp_rs1_lt_rs2};
        end

        FPU_CMP_LE: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = {31{1'b0}, fp_cmp_rs1_lt_rs2|fp_cmp_rs1_eq_rs2};
        end

        FCLASS: begin
            fp_regfile_write_o      = 1'b1;
            fp_regfile_wdata_o      = fpclass_result;
        end

        FPNOP: begin
            set_defaults();
        end

    endcase
end


//TODO: Double check the parameter values on all of these - DONE

DW_fp_sqrt_inst DW_fp_sqrt_inst(
    .inst_a(sqrt_op_a), 
    .inst_rnd(fp_rounding_mode), 
    .z_inst(sqrt_result), 
    .status_inst(sqrt_status)
); 

DW_fp_addsub_inst DW_fp_addsub_inst(
    .inst_a(add_sub_op_a),
    .inst_b(add_sub_op_b), 
    .inst_rnd(fp_rounding_mode), 
    .inst_op(add_sub_select),
    .z_inst(add_sub_result),
    .status_inst(add_sub_status)
);

DW_fp_cmp_inst DW_fp_cmp_inst(
    .inst_a(fpcmp_op_a),
    .inst_b(fpcmp_op_b), 
    .inst_zctr(1'b0), 
    .altb_inst(fp_cmp_rs1_lt_rs2), 
    .agtb_inst(fp_cmp_rs1_gt_rs2), 
    .aeqb_inst(fp_cmp_rs1_eq_rs2), 
    .unordered_inst(fp_cmp_NaN),
    .z0_inst(min_of_rs1_rs2), .z1_inst(max_of_rs1_rs2), 
    .status0_inst(fpcmp_status)
);

DW_fp_mult_inst DW_fp_mult_inst(
    .inst_a(mult_op_a), 
    .inst_b(mult_op_b), 
    .inst_rnd(fp_rounding_mode), 
    .z_inst(mult_result), 
    .status_inst(mult_status)
); 

DW_fp_div_inst DW_fp_div_inst(
    .inst_a(div_op_a), 
    .inst_b(div_op_b), 
    .inst_rnd(fp_rounding_mode), 
    .z_inst(div_result), 
    .status_inst(div_status)
); 

DW_fp_flt2i_inst DW_fp_flt2i_inst(
    .inst_a(fp2int_op_a), 
    .inst_rnd(fp_rounding_mode), 
    .z_inst(fp2int_result), 
    .status_inst(fp2int_status)
);

DW_fp_i2flt_inst DW_fp_i2flt_inst(
    .inst_a(int2fp_op_a), 
    .inst_rnd(fp_rounding_mode), 
    .z_inst(int2fp_result), 
    .status_inst(int2fp_status)
); 

DW_fp_i2flt_inst DW_fp_i2flt_inst_unsigned #(
    .isign(1'b0)
)
(
    .inst_a(int2fp_op_a), 
    .inst_rnd(fp_rounding_mode), 
    .z_inst(int2fp_result_unsigned), 
    .status_inst(int2fp_status)
);

DW_fp_mac_inst DW_fp_mac_inst(
    .inst_a(mac_op_a), 
    .inst_b(mac_op_b), 
    .inst_c(mac_op_c), 
    .inst_rnd(fp_rounding_mode), 
    .z_inst(mac_result), 
    .status_inst(mac_status)
);

FPU_classifer FPU_classifer_inst(
    .f_num (rs1)
    .f_num_class (fpclass_result)
);


endmodule
