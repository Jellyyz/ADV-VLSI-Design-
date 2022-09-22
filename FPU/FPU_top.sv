module FPU_top(
    input logic Clk, rst, 
    input logic [31:0] rs1, rs2, 
    input logic FPU_sel, 
    output logic [31:0] alu_out 

); 

logic [31:0] rs1_class, rs2_class; 
logic [31:0] f_compare; 
// can use the classifer for helping other modules 
FPU_classifer FPU_classifer_rs1(
    //input 
    .f_num(rs1), 
    
    //output 
    .f_num_class(rs1_class)
); 
FPU_classifer FPU_classifer_rs2(
    //input 
    .f_num(rs2), 
    
    //output 
    .f_num_class(rs2_class)
); 
FPU_compare FPU_compare(
    //input 
    .f_num_a(rs1), .f_num_b(rs2),
    .f_class_a(rs2_class), .f_class_b(rs2_class), 
    //output 
    .f_compare(f_compare)
);

always_comb begin 

    unique case(FPU_sel)
    ADD:begin 
    end
    MULT:begin 
    end 
    SUB:begin 
    end 
    DIV:begin 
    end  



    endcase  


end 






endmodule 