module FPU_arth_tb;

import ibex_fp_pkg::*;

logic clk_i;

always #1 clk_i = clk_i === 1'b0;

logic [15:0] [31:0] int_regfile;
logic [15:0] [31:0] fp_regfile; 
logic [2:0]     fp_rounding_mode;
fpu_op_e        fp_op;
logic [31:0]    rs1_i;
logic [31:0]    rs1_int_i;
logic [31:0]    rs2_i;
logic [31:0]    rs3_i;
logic [4:0]     rd_addr_i;
logic [31:0]    fp_regfile_wdata_o;
logic [4:0]     fp_regfile_addr_o;
logic [31:0]    int_regfile_wdata_o;
logic [4:0]     int_regfile_addr_o;
logic int_regfile_write_o, fp_regfile_write_o;
logic [31:0]    instr; 
logic [31:0]    instr_rdata;

ibex_FPU F_PUXY (.*);

fpu_op_e fp_op_input; 

assign fp_op = fp_op_input;
FPU_decoder FPU_decoder(

    .instr(instr), 
    .f_opcode(fp_op_input) 
); 


initial begin

    $fsdbDumpfile("ibex_fpu_comp");
    $fsdbDumpvars();

    clk_i = 1'b1;

    fp_rounding_mode = 3'b001;

    fp_regfile[0] = shortreal'(0.0);
    fp_regfile[1] = 32'h4023d70a; //2.56
    fp_regfile[2] = 32'h41200000; //10
    fp_regfile[3] = 32'h0;
    fp_regfile[4] = 32'h0;
    fp_regfile[5] = 32'h0;
    fp_regfile[6] = 32'h0;
    fp_regfile[7] = 32'h0;
    fp_regfile[8] = 32'h0;
    fp_regfile[9] = 32'h0;
    fp_regfile[10] = 32'h41200000;
    fp_regfile[11] = 32'h41200000;
    fp_regfile[12] = 32'h41200000;
    fp_regfile[13] = 32'h41200000;
    fp_regfile[14] = 32'h41200000;
    fp_regfile[15] = 32'h41200000;

    int_regfile[0] = 32'd0; 
    int_regfile[1] = 32'd23; 
    int_regfile[2] = 32'd250; 
    int_regfile[3] = 32'd120; 
    int_regfile[4] = 32'd30; 
    int_regfile[5] = 32'd40; 
    int_regfile[6] = 32'd45; 
    int_regfile[7] = 32'd46; 
    int_regfile[8] = 32'd23; 
    int_regfile[9] = 32'd23; 
    int_regfile[10] = 32'd23;
    int_regfile[11] = 32'd23;
    int_regfile[12] = 32'd23;
    int_regfile[13] = 32'hc001c0de;
    int_regfile[14] = 32'd23;
    int_regfile[15] = 32'd23;

    /*
    fp_operation(FPU_ADD, 5'h2, 5'h1, 5'h0, 5'h3);
    fp_operation(FPU_SUB, 5'h2, 5'h1, 5'h0, 5'h4);
    fp_operation(FPU_ADD, 5'h3, 5'h4, 5'h0, 5'h5); //h5 = 20
    fp_operation(FPU_MUL, 5'h2, 5'h2, 5'h0, 5'h6);
    fp_operation(FPU_DIV, 5'h5, 5'h2, 5'h0, 5'h7);
    fp_operation(FPU_SQRT, 5'h6, 5'h0, 5'h0, 5'h8);
    fp_operation(FPU_MIN, 5'h1, 5'h2, 5'h0, 5'h9);
    fp_operation(FPU_MAX, 5'h1, 5'h2, 5'h0, 5'ha);

    fp_operation(FPU_MADD, 5'h2, 5'h2, 5'h5, 5'hb);
    fp_operation(FPU_MSUB, 5'h2, 5'h2, 5'h5, 5'hc);
    fp_operation(FPU_NMADD, 5'h2, 5'h2, 5'h5, 5'hd); //-120
    fp_operation(FPU_NMSUB, 5'h2, 5'h2, 5'h5, 5'he);
            
    fp_operation(FPU_INT2FLOAT, 5'h2, 5'h2, 5'h5, 5'hf);
    fp_operation(FPU_FLOAT2INT, 5'hb, 5'h2, 5'h5, 5'hf);

    fp_operation(FPU_SGNJ, 5'hd, 5'h2, 5'h0, 5'h7);
    fp_operation(FPU_SGNJ_N, 5'hb, 5'h2, 5'h0, 5'h8);
    fp_operation(FPU_SGNJ_X, 5'hd, 5'hd, 5'h0, 5'h9);

    fp_operation(FPU_MOVE_INT2FLOAT, 5'hd, 5'h2, 5'h5, 5'h6);
    fp_operation(FPU_MOVE_FLOAT2INT, 5'h2, 5'h2, 5'h5, 5'he);

    fp_operation(FPU_CMP_EQ, 5'ha, 5'h2, 5'h5, 5'h3);
    fp_operation(FPU_CMP_LT, 5'hb, 5'h2, 5'h5, 5'h4);
    fp_operation(FPU_CMP_LE, 5'ha, 5'h2, 5'h5, 5'h5);


    fp_operation(FCLASS, 5'hb, 5'h2, 5'h5, 5'hf);
    */
    #1;
    execute_fp_instructions("fpu_instr.lst");

    $finish;
end

task fp_operation(input fpu_op_e fp_op_in, input [4:0] rs1_addr, input [4:0] rs2_addr, input [4:0] rs3_addr, input [4:0] rd_addr);
    rs1_i = fp_regfile[rs1_addr];
    rs1_int_i = int_regfile[rs1_addr];
    rs2_i = fp_regfile[rs2_addr];
    rs3_i = fp_regfile[rs3_addr];
    rd_addr_i = rd_addr;

    @(posedge clk_i); 
    if(fp_regfile_write_o) begin
        $display("Writing to %f register file", shortreal'(fp_regfile_wdata_o));
        fp_regfile[fp_regfile_addr_o] <=  fp_regfile_wdata_o;
    end
    if(int_regfile_write_o) begin
        $display("Writing to int reg file");
        int_regfile[int_regfile_addr_o]<= int_regfile_wdata_o;
    end
    // fp_op=FPU_NOP;
    @(posedge clk_i);
endtask



int fd;
task execute_fp_instructions( input string file_name);
    fd = $fopen(file_name, "r");
    while ($fscanf(fd,"%h",instr_rdata)==1) begin
        $display("executing instruction: 0x%h",instr_rdata);
        instr = instr_rdata;
        fp_operation(fp_op_input, instr_rdata[19:15], instr_rdata[24:20], instr_rdata[31:27], instr_rdata[11:7]);
    end
endtask
endmodule