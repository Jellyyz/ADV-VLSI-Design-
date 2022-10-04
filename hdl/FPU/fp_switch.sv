import fp_types::*; 

module fp_switch(
    input IEEE_fp x1,
    input IEEE_fp x2,
    input logic x1_gt_x2, //is exponent of x1 (strictly) greater than that of x2 ?
    input logic exp1_eq_exp2,
    output IEEE_fp new_x1,
    output IEEE_fp new_x2
);

unique case(x1_gt_x2, exp1_eq_exp2):
    1'b1x: begin
        new_x1 = x1;
        new_x2 = x2;
    end
    1'b01: begin //if the two exponents aren't equal, then new x1 is the bigger number
        unique case (x1.mant>x2.mant): begin
            1'b1:
                new_x1 = x1;
                new_x2 = x2;
            1'b0:
                new_x1 = x2;
                new_x2 = x1;
        endcase
    1'b00: begin
        new_x1 = x2;
        new_x2 = x1;    
    end
endcase


endmodule