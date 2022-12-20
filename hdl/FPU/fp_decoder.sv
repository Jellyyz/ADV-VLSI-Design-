
`include "ibex_fp_package.sv"
`include "../ibex_core/rtl/ibex_pkg.sv"
import ibex_fp_pkg::*; 
import ibex_pkg::*; 


module FPU_decoder(
    input logic [31:0] instr, 
    output fpu_op_e f_opcode 



);



// declaration of glue logic variables 
// Source/Destination register instruction index
logic [4:0] instr_rs1;
logic [4:0] instr_rs2;
logic [4:0] instr_rs3;
logic [4:0] instr_rd;

assign instr_rs1 = instr[19:15];
assign instr_rs2 = instr[24:20];
assign instr_rs3 = instr[31:27];

opcode_e opcode; 
logic [6:0] funct7; 
logic [2:0] funct3; 
logic [4:0] op_rs2;

opcode = opcode_e'(instr[6:0]); 
funct7 = instr[31:25]; 
funct3 = instr[14:12];
op_rs2 = instr[24:20]; 

always_comb begin

    unique case(opcode)

    OPCODE_F_LOAD:begin
        ;
    end 
    OPCODE_F_STORE:begin 
        ; 
    end 
    OPCODE_F_ADD:begin
        f_opcode = FPU_MADD; 
    end 
    OPCODE_F_SUB:begin 
        f_opcode = FPU_MSUB;
    end 
    OPCODE_F_N_SUB:begin
        f_opcode = FPU_NMADD;
    end 
    OPCODE_F_N_ADD:begin
        f_opcode = FPU_NMADD;
    end 
    OPCODE_F:begin 
        unique case(funct7)
            // F_ADD
            7'b0000000:begin 
                f_opcode = FPU_ADD; 
            end 
            // F_SUB
            7'b0000100:begin
                f_opcode = FPU_SUB; 
            end 
            // F_MUL 
            7'b0001000:begin
                f_opcode = FPU_MUL; 
            end 
            // F_DIV 
            7'b0001100:begin
                f_opcode = F_DIV; 
            end 
            // F_SQRT
            7'b0101100:begin
                f_opcode = F_SQRT; 
            end 
            // F_SGNJ types
            7'b0010000:begin 
                unique case(funct3)
                //FSGNJ.S 
                3'b000:begin 
                    f_opcode = F_SGNJ; 
                end 
                //FSGNJN.S
                3'b001:begin 
                    f_opcode = F_SGNJ_N; 
                end 
                //FSGNJX.S 
                3'b010:begin 
                    f_opcode = F_SGNJ_X; 
                end 

                endcase 
            end 
            // F MIN/MAX types
            7'b0010100:begin 
                unique case(funct3)
                // fMIN
                3'b000:begin
                    f_opcode = FPU_MIN;
                end
                // fMAX
                3'b001:begin
                    f_opcode = FPU_MAX; 
                end 
                endcase
            end 
            // FCVT.w types
            7'b1100000:begin 
                unique case(op_rs2)
                // FCVT.W.S
                3'b00000:begin 
                    f_opcode = FPU_FLOAT2INT; 
                end 
                // FCVT.WU.S
                3'b00001:begin 
                    f_opcode = FPU_FLOAT2INT_U;
                end 
                endcase 
            end 
            // FMV FCLASS types
            7'b1110000:begin 
                unique case(funct3)
                // FMV.X.W
                3'b000:begin 
                    f_opcode = FPU_MOVE_FLOAT2INT;
                end 
                // FCLASS.S
                3'b001:begin 
                    f_opcode = FCLASS; 
                end 
                endcase
            end 
            // FEQ/FLT/FLE
            7'b1010000:begin 
                unique case (funct3)
                // FLE.S
                3'b000:begin 
                    f_opcode = FPU_CMP_LE; 
                end 
                // FLT.S
                3'b001:begin 
                    f_opcode = FPU_CMP_LT; 
                end 
                // FEQ.S
                3'b010:begin 
                    f_opcode = FPU_CMP_EQ; 
                end 
                endcase 
            end 
            // FCVT.S types
            7'b1101000:begin 
                unique case(op_rs2)
                    // FCVT.S.W
                    5'b00000:begin 
                        f_opcode = FPU_INT2FLOAT;
                    end 
                    // FCVT.S.WU
                    5'b00001:begin 
                        f_opcode = FPU_INT2FLOAT_Ul
                    end 
                
                endcase
            end 

            // FMV.w 
            7'b1111000:begin 
                f_opcode = FPU_MOVE_INT2FLOAT; 
            end 
            default: begin 
                f_opcode = FPU_NOP; 
            end 
        endcase 
    end 
    default: begin 
        f_opcode = FPU_NOP; 
    end 
    endcase     


end 


endmodule