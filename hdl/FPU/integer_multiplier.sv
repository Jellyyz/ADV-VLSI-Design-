// import rv32i_types::*;

// module multiplier
// (
//     input clk,
//     input rst,
//     input start,
//     input [31:0] a, b,
//     input mul_ops mulop,
//     input load_bubble,
//     input pipeline_stalled,
//     output logic ready,
//     output logic [31:0] f,
//     output logic done
// );

//     logic [63:0] mul_f;

//     // logic [31:0] partial_products [31:0];

//     logic [63:0] product_additions_0 [31:0];
//     logic [63:0] product_additions_1 [31:0];

//     // logic [31:0] div_f;
//     // logic [31:0] rem_f;

//     logic [31:0] pos_a, pos_b;

//     logic invert_result, result_needs_inverting;
//     logic ready_for_done;
//     logic multiple_rows_not_done;
//     // logic products_reg_idx, next_products_reg_idx;

//     enum logic [3:0] {
//         BEGIN,
//         ADD_PRODUCTS,
//         // ADD_PRODUCTS_1,
//         END
//     } state, next_state;
    


//     // TODO: Fix this
//     assign ready_for_done = (product_additions_0[2] == 0) & ~multiple_rows_not_done & (state == ADD_PRODUCTS);
//     // assign f = product_additions_0[0] + product_additions_0[1];
//     assign ready = (state == BEGIN & !load_bubble);

//     always_ff @(posedge clk) begin
//         if (rst) begin
//             state <= BEGIN;
//         end else begin
//             state <= next_state;
//         end
//     end

//     // always_ff @(posedge clk) begin
//     //     if (rst) begin
//     //         products_reg_idx <= 0;
//     //     end else begin
//     //         products_reg_idx <= next_products_reg_idx;
//     //     end
//     // end


//     always_ff @(posedge clk) begin
//         if (rst) begin
//             invert_result <= 0;
//             for (int i=0; i<32; i++)
//                 product_additions_0[i] <= 0;
//         end
//         else begin
//             product_additions_0 <= product_additions_1;
//             invert_result <= result_needs_inverting;
//         end
//     end


//     function void invert;
//         output logic [31:0] inverted_a;
//         input logic [31:0] a;

//         inverted_a = (~a) + 32'd1;
//     endfunction

//     function void invert_64;
//         output logic [63:0] inverted_a;
//         input logic [63:0] a;

//         inverted_a = (~a) + 64'd1;
//     endfunction

    

//     always_comb begin
//         pos_a = a;
//         pos_b = b;

//         if (state == BEGIN)
//         begin
//             unique case ({a[31], b[31]})
//                 2'b00:
//                 begin
//                     result_needs_inverting = 0;
//                     pos_a = a;
//                     pos_b = b;
//                 end

//                 2'b01:
//                 begin
//                     pos_a = a;
//                     if (mulop == m_mulhsu || mulop == m_mulhu) begin
//                         result_needs_inverting = 0;
//                         pos_b = b;
//                     end else begin
//                         result_needs_inverting = 1;
//                         invert(pos_b, b);
//                     end
//                 end

//                 2'b10:
//                 begin
//                     pos_b = b;
//                     if (mulop == m_mulhu) begin
//                         result_needs_inverting = 0;
//                         pos_a = a;
//                     end else begin
//                         result_needs_inverting = 1;
//                         invert(pos_a, a);
//                     end
//                 end

//                 2'b11:
//                 begin
//                     if (mulop == m_mul || mulop == m_mulh || mulop == m_mulhu) begin
//                         result_needs_inverting = 0;
//                         invert(pos_a, a);
//                         invert(pos_b, b);
//                     end else begin
//                         result_needs_inverting = 1;
//                         invert(pos_a, a);
//                         pos_b = b;
//                     end
//                 end

//                 default:
//                 begin
//                     result_needs_inverting = 0;
//                     pos_a = a;
//                     pos_b = b;
//                 end
//             endcase
//         end
//         else begin
//             result_needs_inverting = invert_result;
//         end
//     end

//     always_comb begin
//         if (invert_result)              invert_64(mul_f, product_additions_0[0] + product_additions_0[1]);
//         else                            mul_f = product_additions_0[0] + product_additions_0[1];
//     end

//     always_comb begin
//         unique case (mulop)
//             m_mul:      f = mul_f[31:0];
//             m_mulh:     f = mul_f[63:32];
//             m_mulhsu:   f = mul_f[63:32];
//             m_mulhu:    f = mul_f[63:32];
//             // m_div:      f = div_f;
//             // m_divu:     f = div_f;
//             // m_rem:      f = rem_f;
//             // m_remu:     f = rem_f;

//             default:    f = mul_f[31:0];
//         endcase
//     end

//     always_comb begin
//         unique case (state)
//             BEGIN:
//                 if (start & !load_bubble)              next_state = ADD_PRODUCTS;
//                 else                    next_state = BEGIN;

//             ADD_PRODUCTS:
//                 if (~ready_for_done)    next_state = ADD_PRODUCTS;
//                 else                    next_state = END;
            
//             // ADD_PRODUCTS_1:
//             //     if (~ready_for_done)    next_state = ADD_PRODUCTS_0;
//             //     else                    next_state = END;
            
//             END:
//                 if (pipeline_stalled)   next_state = END;
//                 else                    next_state = BEGIN;
            
//             default:
//                 ;
                
//         endcase
//     end

//     function void set_defaults();
//         done = 0;
//         // next_products_reg_idx = ~next_products_reg_idx;
//         multiple_rows_not_done = 0;
//         product_additions_1 = product_additions_0;
//     endfunction


//     function void half_add;
//         output logic [63:0] f;
//         output logic [63:0] carry_outs;
//         input logic [63:0] a;
//         input logic [63:0] b;
//         input logic [63:0] c;
//         input logic debug;

//         f = (a&b&c) | ((a&~b&~c) | (~a&b&~c) | (~a&~b&c));
//         // TODO: fix carry_outs
//         carry_outs  = ((a&b) | (a&c) | (b&c)) << 1;
//         // if (debug)
//         //     $display("f=%x, carry_outs=%x", ((a&b&c) | ((a&~b&~c) | (~a&b&c) | (~a&~b&c))), carry_outs);
//     endfunction



//     always_comb begin
//         set_defaults();

//         unique case (state)
//             BEGIN:
//             begin
//                 // next_products_reg_idx = 1;
//                 multiple_rows_not_done = 1;
//                 for (int i=0; i<32; i++)
//                 begin
//                     if(mulop == m_mulhu)
//                         product_additions_1[i] = {{96{1'b0}}, pos_a & {32{pos_b[i]}}} << i;
//                     else begin
//                         product_additions_1[i] = {{96{pos_a[31] & pos_b[i]}}, pos_a & {32{pos_b[i]}}} << i;
//                     end
//                     // product_additions_1[i] = 64'd0;
//                 end

//                 // for (int i=32; i<64; i++)
//                 // begin
//                 //     // product_additions_0[i] = 64'd0;
//                 //     product_additions_1[i] = 0;
//                 // end
//             end

//             ADD_PRODUCTS:
//             begin
//                 // if (products_reg_idx)
//                 // begin
//                 //     half_add(product_additions_1[0], product_additions_1[1], product_additions_0[0], product_additions_0[1], product_additions_0[2], 1);
//                 //     for (int i=0, j=0; i<30; i=i+3, j=j+2)
//                 //     begin
//                 //         half_add(product_additions_1[j], product_additions_1[j+1], product_additions_0[i], product_additions_0[i+1], product_additions_0[i+2]);
//                 //         if (product_additions_0[i] != 0 || product_additions_0[i+1] != 0 || product_additions_0[i+2] != 0)
//                 //         begin
//                 //             multiple_rows_not_done = 1;
//                 //         end
//                 //     end
//                 //     half_add(product_additions_1[20], product_additions_1[21], product_additions_0[30], product_additions_0[31], 32'b0);
//                 //     for (int j=22; j<32; j++)
//                 //     begin
//                 //         product_additions_1[j] = 0'd0;
//                 //     end
//                 // end
//                 // else
//                 // begin
//                 //     half_add(product_additions_0[0], product_additions_0[1], product_additions_1[0], product_additions_1[1], product_additions_1[2]);
//                 //     for (int i=3, j=2; i<30; i=i+3, j=j+2)
//                 //     begin
//                 //         half_add(product_additions_0[j], product_additions_0[j+1], product_additions_1[i], product_additions_1[i+1], product_additions_1[i+2]);
//                 //         if (product_additions_0[i] != 0 || product_additions_0[i+1] != 0 || product_additions_0[i+2] != 0)
//                 //         begin
//                 //             multiple_rows_not_done = 1;
//                 //         end
//                 //     end
//                 //     half_add(product_additions_0[20], product_additions_0[21], product_additions_1[30], product_additions_1[31], 32'b0);
//                 //     for (int j=22; j<32; j++)
//                 //     begin
//                 //         product_additions_0[j] = 0'd0;
//                 //     end
//                 // end

//                 half_add(product_additions_1[0], product_additions_1[1], product_additions_0[0], product_additions_0[1], product_additions_0[2], 1);
//                 half_add(product_additions_1[20], product_additions_1[21], product_additions_0[30], product_additions_0[31], 0, 0);
//                 for (int i=3, j=2; i<30; i=i+3, j=j+2)
//                 begin
//                     half_add(product_additions_1[j], product_additions_1[j+1], product_additions_0[i], product_additions_0[i+1], product_additions_0[i+2], 0);
//                     if (product_additions_0[i] != 0 || product_additions_0[i+1] != 0 || product_additions_0[i+2] != 0)
//                     begin
//                         multiple_rows_not_done = 1;
//                     end
//                 end
//                 for (int j=22; j<32; j++)
//                 begin
//                     product_additions_1[j] = 0;
//                 end
//             end
            
//             END:
//             begin
//                 for (int i=0; i<32; i++)
//                 done = 1;
//             end
            
//             default:
//                 ;
//         endcase
//     end

// endmodule
