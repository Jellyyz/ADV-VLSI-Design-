// import rv32i_types::*;

// module divider
// (
//     input clk,
//     input rst,
//     input start,
//     input [31:0] a, b,
//     input mul_ops mulop,
//     input pipeline_stalled,
//     output logic ready,
//     output logic [31:0] f,
//     output logic done
// );

//     enum logic [3:0] {
//         BEGIN,
//         FIND_QUOTIENT,
//         END
//     } state, next_state;

//     logic invert_result, result_needs_inverting;

//     logic [31:0] pos_a, pos_b;
//     logic [31:0] curr_quotient, next_quotient;
//     logic [31:0] dividend, next_dividend, remainder;

//     assign ready = (state == BEGIN);
//     assign remainder = pos_a - dividend;

//     always_ff @(posedge clk) begin
//         if (rst) begin
//             state <= BEGIN;
//         end else begin
//             state <= next_state;
//         end
//     end

//     always_ff @(posedge clk) begin
//         if (rst) begin
//             dividend <= 0;
//             curr_quotient <= 0;
//             invert_result <= 0;
//         end else if (next_dividend > pos_a) begin
//             dividend <= dividend;
//             curr_quotient <= curr_quotient;
//             invert_result <= result_needs_inverting;
//         end else begin
//             dividend <= next_dividend;
//             curr_quotient <= next_quotient;
//             invert_result <= result_needs_inverting;
//         end
//     end

//     always_comb begin
//         unique case (mulop)
//             m_divu, m_div:
//             begin
//                 if (invert_result)      invert(f, curr_quotient);
//                 else                    f = curr_quotient;
//             end

//             m_rem, m_remu:
//             begin
//                 if (invert_result)      invert(f, remainder);
//                 else                    f = remainder;
//             end

//             default:                    f = curr_quotient;
//         endcase
//     end

//     function void invert;
//         output logic [31:0] inverted_a;
//         input logic [31:0] a;

//         inverted_a = (~a) + 32'd1;
//     endfunction

//     always_comb begin : POS_CONVERSION
//         pos_a = a;
//         pos_b = b;

//         // if (state == BEGIN) begin
//             result_needs_inverting = 0;

//             if (mulop == m_div || mulop == m_rem) begin
//                 if (a[31] & b[31]) begin
//                     result_needs_inverting = 0;
//                     invert(pos_a, a);
//                     invert(pos_b, b);
//                 end else if (a[31]) begin
//                     result_needs_inverting = 1;
//                     invert(pos_a, a);
//                     pos_b = b;
//                 end else if (b[31]) begin
//                     result_needs_inverting = 1;
//                     pos_a = a;
//                     invert(pos_b, b);
//                 end
//             end
//         // end else begin
//         //     result_needs_inverting = invert_result;
//         // end
//     end

//     always_comb begin : NEXT_STATE_LOGIC
//         unique case (state)
//             BEGIN:
//                 if (start)      next_state = FIND_QUOTIENT;
//                 else            next_state = BEGIN;

//             FIND_QUOTIENT:
//                 if (next_dividend > pos_a)  next_state = END;
//                 else                        next_state = FIND_QUOTIENT;

//             END:
//                 if (pipeline_stalled)       next_state = END;
//                 else                        next_state = BEGIN;
//         endcase
//     end

//     always_comb begin
//         done = 0;

//         unique case (state)
//             BEGIN:
//             begin
//                 next_dividend = 0;
//                 next_quotient = 0;
//             end
            
//             FIND_QUOTIENT:
//             begin
//                 if (pos_b == 0) begin
//                     next_dividend = dividend;
//                     next_quotient = curr_quotient;
//                 end else if (dividend <= pos_a) begin
//                     next_dividend = dividend + pos_b;
//                     next_quotient = curr_quotient + 1;
//                 end else begin
//                     next_dividend = dividend + pos_b;
//                     next_quotient = curr_quotient;
//                 end
//             end

//             END:
//             begin
//                 next_dividend = dividend;
//                 next_quotient = curr_quotient;
//                 done = 1;
//             end
//         endcase
//     end

// endmodule