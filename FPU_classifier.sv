// University of Illinois @ Urbana-Champaign ECE498HK
// BLOCK NAME : FPU_Classifer 
// This block allows for a 32 bit input to be classified into different 
// classes based on the type of the 32 bit input 
// Key for f_num_class :
// 0 - negative infinity 
// 1 - negative normal 
// 2 - negative subnormal 
// 3 - negative zero 
// 4 - positive zero
// 5 - positive subnormal 
// 6 - positive normal 
// 7 - positive infinity 
// 8 - sNaN
// 9 - qNaN

module FPU_C(
    input logic [31:0] f_num,
    output logic [31:0] f_num_class
); 


enum logic [7:0]{
    N_INF = 10'b0000000001, 
    N_NORMAL= 10'b0000000010, 
    N_SUBNORMAL = 10'b0000000100,
    N_ZERO = 10'b0000001000,
    P_ZERO = 10'b0000010000, 
    P_SUBNORMAL = 10'b0000100000,
    P_NORMAL = 10'b0001000000, 
    P_INF = 10'b0010000000, 
    SNAN = 10'b0100000000,
    QNAN = 10'b1000000000

} f_num_class; 

logic sign; 
logic [7:0] exponent; 
logic [24:0] mantissa; 

// bits [31:10] are hard coded to 0 
assign f_num_class[31:10] = 21'b0; 
// spread out the input number to start classifying them 
assign sign = f_num[31]; 
assign exponent = f_num[30:24]; 
assign mantissa = f_num[24:0];

always_comb begin 
    f_num_class[8:0] = 8'h0; 
    
    if(sign)begin 
        // covers if exponent is all 1 and sign is 1
        if(exponent == 8'hFF)begin 
            f_num_class = (mantissa == 25'h0) ? SNAN : N_INF; 
        end
        // covers if exponent is NOT all 1 and sign is 1 
        else begin
            // covers case of sign 1 , exponent not all 1, and exponent is not all 0 
            if(exponent != 8'h0)begin 
                f_num_class = N_NORMAL; 
                // covers the two alt cases that exponent is all 0
                else begin 
                    f_num_class = (mantissa == 25'h0 ) ? N_SUBNORMAL : N_ZERO; 
                end
            end 
        end   
    end 
    // other cases where sign is not 0, basically opposite as above
    else if(~sign)begin 
        if(exponent == 8'hFF)begin 
            // check for QNan
            f_num_class = (mantissa == 25'h0) ? QNAN : P_INF; 
        end 
        // covers if exponent is NOT all 1 and sign is 1 
        else begin
            // covers case of sign 1 , exponent not all 1, and exponent is not all 0 
            if(exponent != 8'h0)begin 
                f_num_class = P_NORMAL; 
                // covers the two alt cases that exponent is all 0
                else begin 
                    f_num_class = (mantissa == 25'h0 ) ? P_SUBNORMAL : P_ZERO; 
                end
            end 
        end    
    end 
end 

endmodule 
