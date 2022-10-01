import fp_types::*; 

module fp_switch(
    input IEEE_fp x1,
    input IEEE_fp x2,
    input logic x1_gt_x2, //difference between x1 and x's exponent
    output IEEE_fp new_x1,
    output IEEE_fp new_x2
);

unique case(x1_gt_x2):
    1'b1: begin
        new_x1 = x1;
        new_x2 = x2;
    end
    1'b0: begin
        new_x1 = x2;
        new_x2  = x1;
    end
endcase


endmodule