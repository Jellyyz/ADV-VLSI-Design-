module fp_add_sub #( parameter PRECISION = 3)
(
    input IEEE_extended_fp x1,
    input IEEE_extended_fp x2,
    input fp_op_t op,
    input switched, //if the operands were switched during normalization
    output IEEE_extended_fp ans
)

logic [22 + PRECISION:0] add_op_1; //the first operand that needs to be added
logic [22 + PRECISION:0] add_op_2; //the second operand that needs to be added
logic [22 + PRECISION:0] add_result; //add extra bit to put sign
logic [24 + PRECISION:0] add_raw_result; //add extra bit for overflow

logic [5:0] idx_of_first_one;
logic [5:0] out_stage [0:24 + PRECISION]; //temporary array to store intermediates for finding position of first one. 

logic[7:0] add_overflow;

logic is_add = op == fp_add;
logic is_sub = op == fp_sub;
logic signs_equal = x1.sign_bit == x2.sign_bit;

always_comb begin
    ans.man = add_res[23:0];
    ans.exp = x1.exp + add_overflow;
    ans.sign_bit = 1'b0;
    unique case({fp_add, fp_sub, x1.sign_bit, x2.sign_bit}) begin
           
        4'b00xx, 4'b11xx: //it's not an add or subtract operation, so don't do anything
            begin
                $warn("got an operation that wasn't add or subtract");
                ans.man = 24'hbadbad;
                ans.exp = '0;
                ans.sign_bit = 0;
            end
        4'b0100, 4'b1001: //subtraction of two positive numbers has same operands as addition of positive and negative number
            begin
                add_op_1 = {1'b1, x1.mant};
                add_op_2 = ~{switched, x2.mant} + 1; //if the two operands were switched, then they have different exponents
                ans.sign_bit = (~switched) & (x1.mant<x2.mant);
            end
        4'b0101, 4'b1000://subtraction of positive with negative, so just addition
            begin:
                add_op_1 = {1'b1, x1.mant};
                add_op_2 = {~switched, x2.mant};
            end
        4'b0111, 4'b1010: //subtraction of negative and negative
            begin:
                add_op_1 = ~{1'b1, x1.mant} +1;
                add_op_2 = {~switched, x2.mant};
            end
        4'b0110, 4'b1001: //subtractin of negative and positive
            begin:
                add_op_1 = {1'b1, x1.mant};
                add_op_2 = ~{switched, x2.mant} + 1;
            end    
    endcase
end



always_comb begin :add_logic
    add_raw_result = {1'b0, add_op_1} + {1'b0, add_op_2};
    unique case ({add_raw_result[24 + PRECISION], add_raw_result[23 + PRECISION]||add_raw_result[24 + PRECISION]}) begin
        //the first bit is to check if there was overflow, and the second bit checks if there was a zero before the binary point in the answer
        2'b1x: begin
            add_overflow = 1'b1;
            add_result = add_overflow[23+ PRECISION:1];
        end
        2'b00: begin // these is some underflow
            //check that the first index of 1 is not -1 to give a zero result
            add_result = add_raw_result[22+PRECISION:0]<<(22+PRECISION - idx_of_first_one);
            add_overflow = ~(22+PRECISION - idx_of_first_one) + 1
        end
        2'b01: begin //there is no underflow or overflow
            add_result = add_raw_result[22+PRECISION:0];
            add_overflow = '0;
        end
    end
end

assign out_stage[0] = ~0;

generate
    genvar i;
    for(i=0; i<24 + PRECISION; i=i+1)
        assign out_stage[i+1] = in[i] ? i : out_stage[i];
endgenerate

always_comb begin
    idx_of_first_one = out_stage[24 + PRECISION]
end

endmodule