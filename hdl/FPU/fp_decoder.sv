import ibex_fp_pkg::*; 
import ibex_pkg::*; 

module FPU_decoder(
    input logic [31:0] instr, 
    output fpu_op_e f_opcode 
);


typedef enum logic [6:0] {
    OPCODE_LOAD     = 7'h03,
    OPCODE_MISC_MEM = 7'h0f,
    OPCODE_OP_IMM   = 7'h13,
    OPCODE_AUIPC    = 7'h17,
    OPCODE_STORE    = 7'h23,
    OPCODE_OP       = 7'h33,
    OPCODE_LUI      = 7'h37,
    OPCODE_BRANCH   = 7'h63,
    OPCODE_JALR     = 7'h67,
    OPCODE_JAL      = 7'h6f,
    OPCODE_SYSTEM   = 7'h73,
    OPCODE_F_LOAD   = 7'b0000111,  // 'h7 
    OPCODE_F_STORE  = 7'b0100111,  // 'h27
    OPCODE_F_ADD    = 7'b1000011,  // 'h43 
    OPCODE_F_SUB    = 7'b1000111,  // 'h47
    OPCODE_F_N_SUB  = 7'b1001011,  // 'h4B
    OPCODE_F_N_ADD  = 7'b1001111,  // 'h4F 
    OPCODE_F        = 7'b1010011   // 'h53

  } opcode_e;

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

assign opcode = opcode_e'(instr[6:0]); 
assign funct7 = instr[31:25]; 
assign funct3 = instr[14:12];
assign op_rs2 = instr[24:20]; 

always_comb begin

    unique case(opcode)

    OPCODE_F_LOAD:begin
        f_opcode = FPU_NOP;
    end 
    OPCODE_F_STORE:begin 
        f_opcode = FPU_NOP; 
    end 
    OPCODE_F_ADD:begin
        f_opcode = FPU_MADD; 
    end 
    OPCODE_F_SUB:begin 
        f_opcode = FPU_MSUB;
    end 
    OPCODE_F_N_SUB:begin
        f_opcode = FPU_NMSUB;
    end 
    OPCODE_F_N_ADD:begin
        f_opcode = FPU_NMADD;
    end 
    OPCODE_F:begin 
        unique case(funct7)
            // FPUADD
            7'b0000000:begin 
                f_opcode = FPU_ADD; 
            end 
            // FPU_SUB
            7'b0000100:begin
                f_opcode = FPU_SUB; 
            end 
            // FPU_MUL 
            7'b0001000:begin
                f_opcode = FPU_MUL; 
            end 
            // FPU_DIV 
            7'b0001100:begin
                f_opcode = FPU_DIV; 
            end 
            // FPU_SQRT
            7'b0101100:begin
                f_opcode = FPU_SQRT; 
            end 
            // FPU_SGNJ types
            7'b0010000:begin 
                unique case(funct3)
                //FSGNJ.S 
                3'b000:begin 
                    f_opcode = FPU_SGNJ; 
                end 
                //FSGNJN.S
                3'b001:begin 
                    f_opcode = FPU_SGNJ_N; 
                end 
                //FSGNJX.S 
                3'b010:begin 
                    f_opcode = FPU_SGNJ_X; 
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
                5'b00000:begin 
                    f_opcode = FPU_FLOAT2INT; 
                end 
                // FCVT.WU.S
                5'b00001:begin 
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
            7'b1010000: begin 
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
                        f_opcode = FPU_INT2FLOAT_U;
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