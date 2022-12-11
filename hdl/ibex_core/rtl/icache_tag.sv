// FE Release Version: 2.4.24 
//
//       CONFIDENTIAL AND PROPRIETARY SOFTWARE OF ARM PHYSICAL IP, INC.
//      
//       Copyright (c) 1993 - 2022 ARM Physical IP, Inc.  All Rights Reserved.
//      
//       Use of this Software is subject to the terms and conditions of the
//       applicable license agreement with ARM Physical IP, Inc.
//       In addition, this Software is protected by patents, copyright law 
//       and international treaties.
//      
//       The copyright notice(s) in this Software does not indicate actual or
//       intended publication of this Software.
//
//      Verilog model for Synchronous Single-Port Ram
//
//      Instance Name:              icache_tag
//      Words:                      512
//      Bits:                       21
//      Mux:                        4
//      Drive:                      6
//      Write Mask:                 On
//      Extra Margin Adjustment:    On
//      Accelerated Retention Test: Off
//      Redundant Rows:             0
//      Redundant Columns:          0
//      Test Muxes                  Off
//
//      Creation Date:  Sat Dec 10 23:58:34 2022
//      Version: 	r0p0-00eac0
//
//      Modeling Assumptions: This model supports full gate level simulation
//          including proper x-handling and timing check behavior.  Unit
//          delay timing is included in the model. Back-annotation of SDF
//          (v2.1) is supported.  SDF can be created utilyzing the delay
//          calculation views provided with this generator and supported
//          delay calculators.  All buses are modeled [MSB:LSB].  All 
//          ports are padded with Verilog primitives.
//
//      Modeling Limitations: None.
//
//      Known Bugs: None.
//
//      Known Work Arounds: N/A
//
`ifdef ARM_UD_MODEL

`timescale 1 ns/1 ps

`ifdef ARM_UD_DP
`else
`define ARM_UD_DP #0.001
`endif
`ifdef ARM_UD_CP
`else
`define ARM_UD_CP
`endif
`ifdef ARM_UD_SEQ
`else
`define ARM_UD_SEQ #0.01
`endif

`celldefine
`ifdef POWER_PINS
module icache_tag (Q, CLK, CEN, WEN, A, D, EMA, GWEN, RETN, VSSE, VDDPE, VDDCE);
`else
module icache_tag (Q, CLK, CEN, WEN, A, D, EMA, GWEN, RETN);
`endif

  parameter BITS = 21;
  parameter WORDS = 512;
  parameter MUX = 4;
  parameter MEM_WIDTH = 84; // redun block size 4, 40 on left, 44 on right
  parameter MEM_HEIGHT = 128;
  parameter WP_SIZE = 8 ;
  parameter UPM_WIDTH = 3;

  output [20:0] Q;
  input  CLK;
  input  CEN;
  input [2:0] WEN;
  input [8:0] A;
  input [20:0] D;
  input [2:0] EMA;
  input  GWEN;
  input  RETN;
`ifdef POWER_PINS
  inout VSSE;
  inout VDDPE;
  inout VDDCE;
`endif

  integer row_address;
  integer mux_address;
  reg [83:0] mem [0:127];
  reg [83:0] row;
  reg LAST_CLK;
  reg [83:0] data_out;
  reg [83:0] row_mask;
  reg [83:0] new_data;
  reg [20:0] Q_int;
  reg [20:0] writeEnable;
  reg clk0_int;
  reg CREN_legal;
  initial CREN_legal = 1'b1;

  wire [20:0] Q_;
 wire  CLK_;
  wire  CEN_;
  reg  CEN_int;
  wire [2:0] WEN_;
  reg [2:0] WEN_int;
  wire [8:0] A_;
  reg [8:0] A_int;
  wire [20:0] D_;
  reg [20:0] D_int;
  wire [2:0] EMA_;
  reg [2:0] EMA_int;
  wire  GWEN_;
  reg  GWEN_int;
  wire  RETN_;
  reg  RETN_int;

  assign Q[0] = Q_[0]; 
  assign Q[1] = Q_[1]; 
  assign Q[2] = Q_[2]; 
  assign Q[3] = Q_[3]; 
  assign Q[4] = Q_[4]; 
  assign Q[5] = Q_[5]; 
  assign Q[6] = Q_[6]; 
  assign Q[7] = Q_[7]; 
  assign Q[8] = Q_[8]; 
  assign Q[9] = Q_[9]; 
  assign Q[10] = Q_[10]; 
  assign Q[11] = Q_[11]; 
  assign Q[12] = Q_[12]; 
  assign Q[13] = Q_[13]; 
  assign Q[14] = Q_[14]; 
  assign Q[15] = Q_[15]; 
  assign Q[16] = Q_[16]; 
  assign Q[17] = Q_[17]; 
  assign Q[18] = Q_[18]; 
  assign Q[19] = Q_[19]; 
  assign Q[20] = Q_[20]; 
  assign CLK_ = CLK;
  assign CEN_ = CEN;
  assign WEN_[0] = WEN[0];
  assign WEN_[1] = WEN[1];
  assign WEN_[2] = WEN[2];
  assign A_[0] = A[0];
  assign A_[1] = A[1];
  assign A_[2] = A[2];
  assign A_[3] = A[3];
  assign A_[4] = A[4];
  assign A_[5] = A[5];
  assign A_[6] = A[6];
  assign A_[7] = A[7];
  assign A_[8] = A[8];
  assign D_[0] = D[0];
  assign D_[1] = D[1];
  assign D_[2] = D[2];
  assign D_[3] = D[3];
  assign D_[4] = D[4];
  assign D_[5] = D[5];
  assign D_[6] = D[6];
  assign D_[7] = D[7];
  assign D_[8] = D[8];
  assign D_[9] = D[9];
  assign D_[10] = D[10];
  assign D_[11] = D[11];
  assign D_[12] = D[12];
  assign D_[13] = D[13];
  assign D_[14] = D[14];
  assign D_[15] = D[15];
  assign D_[16] = D[16];
  assign D_[17] = D[17];
  assign D_[18] = D[18];
  assign D_[19] = D[19];
  assign D_[20] = D[20];
  assign EMA_[0] = EMA[0];
  assign EMA_[1] = EMA[1];
  assign EMA_[2] = EMA[2];
  assign GWEN_ = GWEN;
  assign RETN_ = RETN;

  assign `ARM_UD_SEQ Q_ = RETN_ ? (Q_int) : {21{1'b0}};

`ifdef INITIALIZE_MEMORY
  integer i;
  initial
    for (i = 0; i < MEM_HEIGHT; i = i + 1)
      mem[i] = {MEM_WIDTH{1'b0}};
`endif

  task failedWrite;
  input port;
  integer i;
  begin
    for (i = 0; i < MEM_HEIGHT; i = i + 1)
      mem[i] = {MEM_WIDTH{1'bx}};
  end
  endtask

  function isBitX;
    input bitIn;
    begin
      isBitX = ( bitIn===1'bx || bitIn===1'bz ) ? 1'b1 : 1'b0;
    end
  endfunction


  task readWrite;
  begin
    if (RETN_int === 1'bx) begin
      failedWrite(0);
      Q_int = {21{1'bx}};
    end else if (RETN_int === 1'b0 && CEN_int === 1'b0) begin
      failedWrite(0);
      Q_int = {21{1'bx}};
    end else if (RETN_int === 1'b0) begin
      // no cycle in retention mode
    end else if (^{CEN_int, EMA_int} === 1'bx) begin
      failedWrite(0);
      Q_int = {21{1'bx}};
    end else if ((A_int >= WORDS) && (CEN_int === 1'b0)) begin
      writeEnable = ~( {21{GWEN_int}} | {WEN_int[2], WEN_int[2], WEN_int[2], WEN_int[2],
         WEN_int[2], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1],
         WEN_int[1], WEN_int[1], WEN_int[0], WEN_int[0], WEN_int[0], WEN_int[0], WEN_int[0],
         WEN_int[0], WEN_int[0], WEN_int[0]});
      Q_int = ((writeEnable & D_int) | (~writeEnable & {21{1'bx}}));
    end else if (CEN_int === 1'b0 && (^A_int) === 1'bx) begin
      if (GWEN_int !== 1'b1 && (& WEN_int) !== 1'b1) failedWrite(0);
      Q_int = {21{1'bx}};
    end else if (CEN_int === 1'b0) begin
      mux_address = (A_int & 2'b11);
      row_address = (A_int >> 2);
      if (row_address >= 128)
        row = {84{1'bx}};
      else
        row = mem[row_address];
      if( isBitX(GWEN_int) )
        writeEnable = {21{1'bx}};
      else
        writeEnable = ~( {21{GWEN_int}} | {WEN_int[2], WEN_int[2], WEN_int[2], WEN_int[2],
         WEN_int[2], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1],
         WEN_int[1], WEN_int[1], WEN_int[0], WEN_int[0], WEN_int[0], WEN_int[0], WEN_int[0],
         WEN_int[0], WEN_int[0], WEN_int[0]});
      row_mask =  ( {3'b000, writeEnable[20], 3'b000, writeEnable[19], 3'b000, writeEnable[18],
          3'b000, writeEnable[17], 3'b000, writeEnable[16], 3'b000, writeEnable[15],
          3'b000, writeEnable[14], 3'b000, writeEnable[13], 3'b000, writeEnable[12],
          3'b000, writeEnable[11], 3'b000, writeEnable[10], 3'b000, writeEnable[9],
          3'b000, writeEnable[8], 3'b000, writeEnable[7], 3'b000, writeEnable[6], 3'b000, writeEnable[5],
          3'b000, writeEnable[4], 3'b000, writeEnable[3], 3'b000, writeEnable[2], 3'b000, writeEnable[1],
          3'b000, writeEnable[0]} << mux_address);
      new_data =  ( {3'b000, D_int[20], 3'b000, D_int[19], 3'b000, D_int[18], 3'b000, D_int[17],
          3'b000, D_int[16], 3'b000, D_int[15], 3'b000, D_int[14], 3'b000, D_int[13],
          3'b000, D_int[12], 3'b000, D_int[11], 3'b000, D_int[10], 3'b000, D_int[9],
          3'b000, D_int[8], 3'b000, D_int[7], 3'b000, D_int[6], 3'b000, D_int[5], 3'b000, D_int[4],
          3'b000, D_int[3], 3'b000, D_int[2], 3'b000, D_int[1], 3'b000, D_int[0]} << mux_address);
      row = (row & ~row_mask) | (row_mask & (~row_mask | new_data));
      mem[row_address] = row;
      data_out = (row >> mux_address);
      if (GWEN_int !== 1'b0)
        Q_int = {data_out[80], data_out[76], data_out[72], data_out[68], data_out[64],
          data_out[60], data_out[56], data_out[52], data_out[48], data_out[44], data_out[40],
          data_out[36], data_out[32], data_out[28], data_out[24], data_out[20], data_out[16],
          data_out[12], data_out[8], data_out[4], data_out[0]};
      else
        Q_int = {(writeEnable[20]?data_out[80]:Q_int[20]), (writeEnable[19]?data_out[76]:Q_int[19]),
          (writeEnable[18]?data_out[72]:Q_int[18]), (writeEnable[17]?data_out[68]:Q_int[17]),
          (writeEnable[16]?data_out[64]:Q_int[16]), (writeEnable[15]?data_out[60]:Q_int[15]),
          (writeEnable[14]?data_out[56]:Q_int[14]), (writeEnable[13]?data_out[52]:Q_int[13]),
          (writeEnable[12]?data_out[48]:Q_int[12]), (writeEnable[11]?data_out[44]:Q_int[11]),
          (writeEnable[10]?data_out[40]:Q_int[10]), (writeEnable[9]?data_out[36]:Q_int[9]),
          (writeEnable[8]?data_out[32]:Q_int[8]), (writeEnable[7]?data_out[28]:Q_int[7]),
          (writeEnable[6]?data_out[24]:Q_int[6]), (writeEnable[5]?data_out[20]:Q_int[5]),
          (writeEnable[4]?data_out[16]:Q_int[4]), (writeEnable[3]?data_out[12]:Q_int[3]),
          (writeEnable[2]?data_out[8]:Q_int[2]), (writeEnable[1]?data_out[4]:Q_int[1]),
          (writeEnable[0]?data_out[0]:Q_int[0])};
    end
  end
  endtask

  always @ RETN_ begin
    if (RETN_ == 1'b0) begin
      Q_int = {21{1'b0}};
      CEN_int = 1'b0;
      WEN_int = {3{1'b0}};
      A_int = {9{1'b0}};
      D_int = {21{1'b0}};
      EMA_int = {3{1'b0}};
      GWEN_int = 1'b0;
      RETN_int = 1'b0;
    end else begin
      Q_int = {21{1'bx}};
      CEN_int = 1'bx;
      WEN_int = {3{1'bx}};
      A_int = {9{1'bx}};
      D_int = {21{1'bx}};
      EMA_int = {3{1'bx}};
      GWEN_int = 1'bx;
      RETN_int = 1'bx;
    end
    RETN_int = RETN_;
  end

  always @ CLK_ begin
`ifdef POWER_PINS
    if (VSSE === 1'bx || VSSE === 1'bz)
      $display("ERROR: Illegal value for VSSE %b", VSSE);
    if (VDDPE === 1'bx || VDDPE === 1'bz)
      $display("ERROR: Illegal value for VDDPE %b", VDDPE);
    if (VDDCE === 1'bx || VDDCE === 1'bz)
      $display("ERROR: Illegal value for VDDCE %b", VDDCE);
`endif
    if (CLK_ === 1'bx && (CEN_ !== 1'b1)) begin
      failedWrite(0);
      Q_int = {21{1'bx}};
    end else if (CLK_ === 1'b1 && LAST_CLK === 1'b0) begin
      CEN_int = CEN_;
      WEN_int = WEN_;
      A_int = A_;
      D_int = D_;
      EMA_int = EMA_;
      GWEN_int = GWEN_;
      RETN_int = RETN_;
      clk0_int = 1'b0;
      readWrite;
    end
    LAST_CLK = CLK_;
  end


endmodule
`endcelldefine
`else
`timescale 1 ns/1 ps
`celldefine
`ifdef POWER_PINS
module icache_tag (Q, CLK, CEN, WEN, A, D, EMA, GWEN, RETN, VSSE, VDDPE, VDDCE);
`else
module icache_tag (Q, CLK, CEN, WEN, A, D, EMA, GWEN, RETN);
`endif

  parameter BITS = 21;
  parameter WORDS = 512;
  parameter MUX = 4;
  parameter MEM_WIDTH = 84; // redun block size 4, 40 on left, 44 on right
  parameter MEM_HEIGHT = 128;
  parameter WP_SIZE = 8 ;
  parameter UPM_WIDTH = 3;

  output [20:0] Q;
  input  CLK;
  input  CEN;
  input [2:0] WEN;
  input [8:0] A;
  input [20:0] D;
  input [2:0] EMA;
  input  GWEN;
  input  RETN;
`ifdef POWER_PINS
  inout VSSE;
  inout VDDPE;
  inout VDDCE;
`endif

  integer row_address;
  integer mux_address;
  reg [83:0] mem [0:127];
  reg [83:0] row;
  reg LAST_CLK;
  reg [83:0] data_out;
  reg [83:0] row_mask;
  reg [83:0] new_data;
  reg [20:0] Q_int;
  reg [20:0] writeEnable;

  reg NOT_A0, NOT_A1, NOT_A2, NOT_A3, NOT_A4, NOT_A5, NOT_A6, NOT_A7, NOT_A8, NOT_CEN;
  reg NOT_CLK_MINH, NOT_CLK_MINL, NOT_CLK_PER, NOT_D0, NOT_D1, NOT_D10, NOT_D11, NOT_D12;
  reg NOT_D13, NOT_D14, NOT_D15, NOT_D16, NOT_D17, NOT_D18, NOT_D19, NOT_D2, NOT_D20;
  reg NOT_D3, NOT_D4, NOT_D5, NOT_D6, NOT_D7, NOT_D8, NOT_D9, NOT_EMA0, NOT_EMA1, NOT_EMA2;
  reg NOT_GWEN, NOT_RETN, NOT_WEN0, NOT_WEN1, NOT_WEN2;
  reg clk0_int;
  reg CREN_legal;
  initial CREN_legal = 1'b1;

  wire [20:0] Q_;
 wire  CLK_;
  wire  CEN_;
  reg  CEN_int;
  wire [2:0] WEN_;
  reg [2:0] WEN_int;
  wire [8:0] A_;
  reg [8:0] A_int;
  wire [20:0] D_;
  reg [20:0] D_int;
  wire [2:0] EMA_;
  reg [2:0] EMA_int;
  wire  GWEN_;
  reg  GWEN_int;
  wire  RETN_;
  reg  RETN_int;

  buf B0(Q[0], Q_[0]);
  buf B1(Q[1], Q_[1]);
  buf B2(Q[2], Q_[2]);
  buf B3(Q[3], Q_[3]);
  buf B4(Q[4], Q_[4]);
  buf B5(Q[5], Q_[5]);
  buf B6(Q[6], Q_[6]);
  buf B7(Q[7], Q_[7]);
  buf B8(Q[8], Q_[8]);
  buf B9(Q[9], Q_[9]);
  buf B10(Q[10], Q_[10]);
  buf B11(Q[11], Q_[11]);
  buf B12(Q[12], Q_[12]);
  buf B13(Q[13], Q_[13]);
  buf B14(Q[14], Q_[14]);
  buf B15(Q[15], Q_[15]);
  buf B16(Q[16], Q_[16]);
  buf B17(Q[17], Q_[17]);
  buf B18(Q[18], Q_[18]);
  buf B19(Q[19], Q_[19]);
  buf B20(Q[20], Q_[20]);
  buf B21(CLK_, CLK);
  buf B22(CEN_, CEN);
  buf B23(WEN_[0], WEN[0]);
  buf B24(WEN_[1], WEN[1]);
  buf B25(WEN_[2], WEN[2]);
  buf B26(A_[0], A[0]);
  buf B27(A_[1], A[1]);
  buf B28(A_[2], A[2]);
  buf B29(A_[3], A[3]);
  buf B30(A_[4], A[4]);
  buf B31(A_[5], A[5]);
  buf B32(A_[6], A[6]);
  buf B33(A_[7], A[7]);
  buf B34(A_[8], A[8]);
  buf B35(D_[0], D[0]);
  buf B36(D_[1], D[1]);
  buf B37(D_[2], D[2]);
  buf B38(D_[3], D[3]);
  buf B39(D_[4], D[4]);
  buf B40(D_[5], D[5]);
  buf B41(D_[6], D[6]);
  buf B42(D_[7], D[7]);
  buf B43(D_[8], D[8]);
  buf B44(D_[9], D[9]);
  buf B45(D_[10], D[10]);
  buf B46(D_[11], D[11]);
  buf B47(D_[12], D[12]);
  buf B48(D_[13], D[13]);
  buf B49(D_[14], D[14]);
  buf B50(D_[15], D[15]);
  buf B51(D_[16], D[16]);
  buf B52(D_[17], D[17]);
  buf B53(D_[18], D[18]);
  buf B54(D_[19], D[19]);
  buf B55(D_[20], D[20]);
  buf B56(EMA_[0], EMA[0]);
  buf B57(EMA_[1], EMA[1]);
  buf B58(EMA_[2], EMA[2]);
  buf B59(GWEN_, GWEN);
  buf B60(RETN_, RETN);

  assign Q_ = RETN_ ? (Q_int) : {21{1'b0}};

`ifdef INITIALIZE_MEMORY
  integer i;
  initial
    for (i = 0; i < MEM_HEIGHT; i = i + 1)
      mem[i] = {MEM_WIDTH{1'b0}};
`endif

  task failedWrite;
  input port;
  integer i;
  begin
    for (i = 0; i < MEM_HEIGHT; i = i + 1)
      mem[i] = {MEM_WIDTH{1'bx}};
  end
  endtask

  function isBitX;
    input bitIn;
    begin
      isBitX = ( bitIn===1'bx || bitIn===1'bz ) ? 1'b1 : 1'b0;
    end
  endfunction


  task readWrite;
  begin
    if (RETN_int === 1'bx) begin
      failedWrite(0);
      Q_int = {21{1'bx}};
    end else if (RETN_int === 1'b0 && CEN_int === 1'b0) begin
      failedWrite(0);
      Q_int = {21{1'bx}};
    end else if (RETN_int === 1'b0) begin
      // no cycle in retention mode
    end else if (^{CEN_int, EMA_int} === 1'bx) begin
      failedWrite(0);
      Q_int = {21{1'bx}};
    end else if ((A_int >= WORDS) && (CEN_int === 1'b0)) begin
      writeEnable = ~( {21{GWEN_int}} | {WEN_int[2], WEN_int[2], WEN_int[2], WEN_int[2],
         WEN_int[2], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1],
         WEN_int[1], WEN_int[1], WEN_int[0], WEN_int[0], WEN_int[0], WEN_int[0], WEN_int[0],
         WEN_int[0], WEN_int[0], WEN_int[0]});
      Q_int = ((writeEnable & D_int) | (~writeEnable & {21{1'bx}}));
    end else if (CEN_int === 1'b0 && (^A_int) === 1'bx) begin
      if (GWEN_int !== 1'b1 && (& WEN_int) !== 1'b1) failedWrite(0);
      Q_int = {21{1'bx}};
    end else if (CEN_int === 1'b0) begin
      mux_address = (A_int & 2'b11);
      row_address = (A_int >> 2);
      if (row_address >= 128)
        row = {84{1'bx}};
      else
        row = mem[row_address];
      if( isBitX(GWEN_int) )
        writeEnable = {21{1'bx}};
      else
        writeEnable = ~( {21{GWEN_int}} | {WEN_int[2], WEN_int[2], WEN_int[2], WEN_int[2],
         WEN_int[2], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1], WEN_int[1],
         WEN_int[1], WEN_int[1], WEN_int[0], WEN_int[0], WEN_int[0], WEN_int[0], WEN_int[0],
         WEN_int[0], WEN_int[0], WEN_int[0]});
      row_mask =  ( {3'b000, writeEnable[20], 3'b000, writeEnable[19], 3'b000, writeEnable[18],
          3'b000, writeEnable[17], 3'b000, writeEnable[16], 3'b000, writeEnable[15],
          3'b000, writeEnable[14], 3'b000, writeEnable[13], 3'b000, writeEnable[12],
          3'b000, writeEnable[11], 3'b000, writeEnable[10], 3'b000, writeEnable[9],
          3'b000, writeEnable[8], 3'b000, writeEnable[7], 3'b000, writeEnable[6], 3'b000, writeEnable[5],
          3'b000, writeEnable[4], 3'b000, writeEnable[3], 3'b000, writeEnable[2], 3'b000, writeEnable[1],
          3'b000, writeEnable[0]} << mux_address);
      new_data =  ( {3'b000, D_int[20], 3'b000, D_int[19], 3'b000, D_int[18], 3'b000, D_int[17],
          3'b000, D_int[16], 3'b000, D_int[15], 3'b000, D_int[14], 3'b000, D_int[13],
          3'b000, D_int[12], 3'b000, D_int[11], 3'b000, D_int[10], 3'b000, D_int[9],
          3'b000, D_int[8], 3'b000, D_int[7], 3'b000, D_int[6], 3'b000, D_int[5], 3'b000, D_int[4],
          3'b000, D_int[3], 3'b000, D_int[2], 3'b000, D_int[1], 3'b000, D_int[0]} << mux_address);
      row = (row & ~row_mask) | (row_mask & (~row_mask | new_data));
      mem[row_address] = row;
      data_out = (row >> mux_address);
      if (GWEN_int !== 1'b0)
        Q_int = {data_out[80], data_out[76], data_out[72], data_out[68], data_out[64],
          data_out[60], data_out[56], data_out[52], data_out[48], data_out[44], data_out[40],
          data_out[36], data_out[32], data_out[28], data_out[24], data_out[20], data_out[16],
          data_out[12], data_out[8], data_out[4], data_out[0]};
      else
        Q_int = {(writeEnable[20]?data_out[80]:Q_int[20]), (writeEnable[19]?data_out[76]:Q_int[19]),
          (writeEnable[18]?data_out[72]:Q_int[18]), (writeEnable[17]?data_out[68]:Q_int[17]),
          (writeEnable[16]?data_out[64]:Q_int[16]), (writeEnable[15]?data_out[60]:Q_int[15]),
          (writeEnable[14]?data_out[56]:Q_int[14]), (writeEnable[13]?data_out[52]:Q_int[13]),
          (writeEnable[12]?data_out[48]:Q_int[12]), (writeEnable[11]?data_out[44]:Q_int[11]),
          (writeEnable[10]?data_out[40]:Q_int[10]), (writeEnable[9]?data_out[36]:Q_int[9]),
          (writeEnable[8]?data_out[32]:Q_int[8]), (writeEnable[7]?data_out[28]:Q_int[7]),
          (writeEnable[6]?data_out[24]:Q_int[6]), (writeEnable[5]?data_out[20]:Q_int[5]),
          (writeEnable[4]?data_out[16]:Q_int[4]), (writeEnable[3]?data_out[12]:Q_int[3]),
          (writeEnable[2]?data_out[8]:Q_int[2]), (writeEnable[1]?data_out[4]:Q_int[1]),
          (writeEnable[0]?data_out[0]:Q_int[0])};
    end
  end
  endtask

  always @ RETN_ begin
    if (RETN_ == 1'b0) begin
      Q_int = {21{1'b0}};
      CEN_int = 1'b0;
      WEN_int = {3{1'b0}};
      A_int = {9{1'b0}};
      D_int = {21{1'b0}};
      EMA_int = {3{1'b0}};
      GWEN_int = 1'b0;
      RETN_int = 1'b0;
    end else begin
      Q_int = {21{1'bx}};
      CEN_int = 1'bx;
      WEN_int = {3{1'bx}};
      A_int = {9{1'bx}};
      D_int = {21{1'bx}};
      EMA_int = {3{1'bx}};
      GWEN_int = 1'bx;
      RETN_int = 1'bx;
    end
    RETN_int = RETN_;
  end

  always @ CLK_ begin
`ifdef POWER_PINS
    if (VSSE === 1'bx || VSSE === 1'bz)
      $display("ERROR: Illegal value for VSSE %b", VSSE);
    if (VDDPE === 1'bx || VDDPE === 1'bz)
      $display("ERROR: Illegal value for VDDPE %b", VDDPE);
    if (VDDCE === 1'bx || VDDCE === 1'bz)
      $display("ERROR: Illegal value for VDDCE %b", VDDCE);
`endif
    if (CLK_ === 1'bx && (CEN_ !== 1'b1)) begin
      failedWrite(0);
      Q_int = {21{1'bx}};
    end else if (CLK_ === 1'b1 && LAST_CLK === 1'b0) begin
      CEN_int = CEN_;
      WEN_int = WEN_;
      A_int = A_;
      D_int = D_;
      EMA_int = EMA_;
      GWEN_int = GWEN_;
      RETN_int = RETN_;
      clk0_int = 1'b0;
      readWrite;
    end
    LAST_CLK = CLK_;
  end

  reg globalNotifier0;
  initial globalNotifier0 = 1'b0;

  always @ globalNotifier0 begin
    if ($realtime == 0) begin
    end else if (CEN_int === 1'bx || EMA_int[0] === 1'bx || EMA_int[1] === 1'bx || 
      EMA_int[2] === 1'bx || RETN_int === 1'bx || clk0_int === 1'bx) begin
      Q_int = {21{1'bx}};
      failedWrite(0);
    end else begin
      readWrite;
   end
    globalNotifier0 = 1'b0;
  end

  always @ NOT_A0 begin
    A_int[0] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A1 begin
    A_int[1] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A2 begin
    A_int[2] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A3 begin
    A_int[3] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A4 begin
    A_int[4] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A5 begin
    A_int[5] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A6 begin
    A_int[6] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A7 begin
    A_int[7] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_A8 begin
    A_int[8] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_CEN begin
    CEN_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D0 begin
    D_int[0] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D10 begin
    D_int[10] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D11 begin
    D_int[11] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D12 begin
    D_int[12] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D13 begin
    D_int[13] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D14 begin
    D_int[14] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D15 begin
    D_int[15] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D16 begin
    D_int[16] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D17 begin
    D_int[17] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D18 begin
    D_int[18] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D19 begin
    D_int[19] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D1 begin
    D_int[1] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D20 begin
    D_int[20] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D2 begin
    D_int[2] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D3 begin
    D_int[3] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D4 begin
    D_int[4] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D5 begin
    D_int[5] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D6 begin
    D_int[6] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D7 begin
    D_int[7] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D8 begin
    D_int[8] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_D9 begin
    D_int[9] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_EMA0 begin
    EMA_int[0] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_EMA1 begin
    EMA_int[1] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_EMA2 begin
    EMA_int[2] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_GWEN begin
    GWEN_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_RETN begin
    RETN_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_WEN0 begin
    WEN_int[0] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_WEN1 begin
    WEN_int[1] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_WEN2 begin
    WEN_int[2] = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_CLK_MINH begin
    clk0_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_CLK_MINL begin
    clk0_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end
  always @ NOT_CLK_PER begin
    clk0_int = 1'bx;
    if ( globalNotifier0 === 1'b0 ) globalNotifier0 = 1'bx;
  end

  wire CEN_flag;
  wire flag;
  wire D_flag0;
  wire D_flag1;
  wire D_flag2;
  wire D_flag3;
  wire D_flag4;
  wire D_flag5;
  wire D_flag6;
  wire D_flag7;
  wire D_flag8;
  wire D_flag9;
  wire D_flag10;
  wire D_flag11;
  wire D_flag12;
  wire D_flag13;
  wire D_flag14;
  wire D_flag15;
  wire D_flag16;
  wire D_flag17;
  wire D_flag18;
  wire D_flag19;
  wire D_flag20;
  wire cyc_flag;
  wire EMA2eq0andEMA1eq0andEMA0eq0;
  wire EMA2eq0andEMA1eq0andEMA0eq1;
  wire EMA2eq0andEMA1eq1andEMA0eq0;
  wire EMA2eq0andEMA1eq1andEMA0eq1;
  wire EMA2eq1andEMA1eq0andEMA0eq0;
  wire EMA2eq1andEMA1eq0andEMA0eq1;
  wire EMA2eq1andEMA1eq1andEMA0eq0;
  wire EMA2eq1andEMA1eq1andEMA0eq1;
  assign CEN_flag = 1'b1;
  assign flag = !CEN_;
  assign D_flag0 = !(CEN_ || WEN_[0] || GWEN_);
  assign D_flag1 = !(CEN_ || WEN_[0] || GWEN_);
  assign D_flag2 = !(CEN_ || WEN_[0] || GWEN_);
  assign D_flag3 = !(CEN_ || WEN_[0] || GWEN_);
  assign D_flag4 = !(CEN_ || WEN_[0] || GWEN_);
  assign D_flag5 = !(CEN_ || WEN_[0] || GWEN_);
  assign D_flag6 = !(CEN_ || WEN_[0] || GWEN_);
  assign D_flag7 = !(CEN_ || WEN_[0] || GWEN_);
  assign D_flag8 = !(CEN_ || WEN_[1] || GWEN_);
  assign D_flag9 = !(CEN_ || WEN_[1] || GWEN_);
  assign D_flag10 = !(CEN_ || WEN_[1] || GWEN_);
  assign D_flag11 = !(CEN_ || WEN_[1] || GWEN_);
  assign D_flag12 = !(CEN_ || WEN_[1] || GWEN_);
  assign D_flag13 = !(CEN_ || WEN_[1] || GWEN_);
  assign D_flag14 = !(CEN_ || WEN_[1] || GWEN_);
  assign D_flag15 = !(CEN_ || WEN_[1] || GWEN_);
  assign D_flag16 = !(CEN_ || WEN_[2] || GWEN_);
  assign D_flag17 = !(CEN_ || WEN_[2] || GWEN_);
  assign D_flag18 = !(CEN_ || WEN_[2] || GWEN_);
  assign D_flag19 = !(CEN_ || WEN_[2] || GWEN_);
  assign D_flag20 = !(CEN_ || WEN_[2] || GWEN_);
  assign cyc_flag = !CEN_;
  assign EMA2eq0andEMA1eq0andEMA0eq0 = !EMA_[2] && !EMA_[1] && !EMA_[0] && cyc_flag;
  assign EMA2eq0andEMA1eq0andEMA0eq1 = !EMA_[2] && !EMA_[1] && EMA_[0] && cyc_flag;
  assign EMA2eq0andEMA1eq1andEMA0eq0 = !EMA_[2] && EMA_[1] && !EMA_[0] && cyc_flag;
  assign EMA2eq0andEMA1eq1andEMA0eq1 = !EMA_[2] && EMA_[1] && EMA_[0] && cyc_flag;
  assign EMA2eq1andEMA1eq0andEMA0eq0 = EMA_[2] && !EMA_[1] && !EMA_[0] && cyc_flag;
  assign EMA2eq1andEMA1eq0andEMA0eq1 = EMA_[2] && !EMA_[1] && EMA_[0] && cyc_flag;
  assign EMA2eq1andEMA1eq1andEMA0eq0 = EMA_[2] && EMA_[1] && !EMA_[0] && cyc_flag;
  assign EMA2eq1andEMA1eq1andEMA0eq1 = EMA_[2] && EMA_[1] && EMA_[0] && cyc_flag;

  specify
      $hold(posedge CEN, negedge RETN, 1.000, NOT_RETN);
      $setuphold(posedge CLK &&& CEN_flag, posedge CEN, 1.000, 0.500, NOT_CEN);
      $setuphold(posedge CLK &&& CEN_flag, negedge CEN, 1.000, 0.500, NOT_CEN);
      $setuphold(posedge CLK &&& flag, posedge WEN[2], 1.000, 0.500, NOT_WEN2);
      $setuphold(posedge CLK &&& flag, negedge WEN[2], 1.000, 0.500, NOT_WEN2);
      $setuphold(posedge CLK &&& flag, posedge WEN[1], 1.000, 0.500, NOT_WEN1);
      $setuphold(posedge CLK &&& flag, negedge WEN[1], 1.000, 0.500, NOT_WEN1);
      $setuphold(posedge CLK &&& flag, posedge WEN[0], 1.000, 0.500, NOT_WEN0);
      $setuphold(posedge CLK &&& flag, negedge WEN[0], 1.000, 0.500, NOT_WEN0);
      $setuphold(posedge CLK &&& flag, posedge GWEN, 1.000, 0.500, NOT_GWEN);
      $setuphold(posedge CLK &&& flag, negedge GWEN, 1.000, 0.500, NOT_GWEN);
      $setuphold(posedge CLK &&& flag, posedge A[8], 1.000, 0.500, NOT_A8);
      $setuphold(posedge CLK &&& flag, negedge A[8], 1.000, 0.500, NOT_A8);
      $setuphold(posedge CLK &&& flag, posedge A[7], 1.000, 0.500, NOT_A7);
      $setuphold(posedge CLK &&& flag, negedge A[7], 1.000, 0.500, NOT_A7);
      $setuphold(posedge CLK &&& flag, posedge A[6], 1.000, 0.500, NOT_A6);
      $setuphold(posedge CLK &&& flag, negedge A[6], 1.000, 0.500, NOT_A6);
      $setuphold(posedge CLK &&& flag, posedge A[5], 1.000, 0.500, NOT_A5);
      $setuphold(posedge CLK &&& flag, negedge A[5], 1.000, 0.500, NOT_A5);
      $setuphold(posedge CLK &&& flag, posedge A[4], 1.000, 0.500, NOT_A4);
      $setuphold(posedge CLK &&& flag, negedge A[4], 1.000, 0.500, NOT_A4);
      $setuphold(posedge CLK &&& flag, posedge A[3], 1.000, 0.500, NOT_A3);
      $setuphold(posedge CLK &&& flag, negedge A[3], 1.000, 0.500, NOT_A3);
      $setuphold(posedge CLK &&& flag, posedge A[2], 1.000, 0.500, NOT_A2);
      $setuphold(posedge CLK &&& flag, negedge A[2], 1.000, 0.500, NOT_A2);
      $setuphold(posedge CLK &&& flag, posedge A[1], 1.000, 0.500, NOT_A1);
      $setuphold(posedge CLK &&& flag, negedge A[1], 1.000, 0.500, NOT_A1);
      $setuphold(posedge CLK &&& flag, posedge A[0], 1.000, 0.500, NOT_A0);
      $setuphold(posedge CLK &&& flag, negedge A[0], 1.000, 0.500, NOT_A0);
      $setuphold(posedge CLK &&& D_flag20, posedge D[20], 1.000, 0.500, NOT_D20);
      $setuphold(posedge CLK &&& D_flag20, negedge D[20], 1.000, 0.500, NOT_D20);
      $setuphold(posedge CLK &&& D_flag19, posedge D[19], 1.000, 0.500, NOT_D19);
      $setuphold(posedge CLK &&& D_flag19, negedge D[19], 1.000, 0.500, NOT_D19);
      $setuphold(posedge CLK &&& D_flag18, posedge D[18], 1.000, 0.500, NOT_D18);
      $setuphold(posedge CLK &&& D_flag18, negedge D[18], 1.000, 0.500, NOT_D18);
      $setuphold(posedge CLK &&& D_flag17, posedge D[17], 1.000, 0.500, NOT_D17);
      $setuphold(posedge CLK &&& D_flag17, negedge D[17], 1.000, 0.500, NOT_D17);
      $setuphold(posedge CLK &&& D_flag16, posedge D[16], 1.000, 0.500, NOT_D16);
      $setuphold(posedge CLK &&& D_flag16, negedge D[16], 1.000, 0.500, NOT_D16);
      $setuphold(posedge CLK &&& D_flag15, posedge D[15], 1.000, 0.500, NOT_D15);
      $setuphold(posedge CLK &&& D_flag15, negedge D[15], 1.000, 0.500, NOT_D15);
      $setuphold(posedge CLK &&& D_flag14, posedge D[14], 1.000, 0.500, NOT_D14);
      $setuphold(posedge CLK &&& D_flag14, negedge D[14], 1.000, 0.500, NOT_D14);
      $setuphold(posedge CLK &&& D_flag13, posedge D[13], 1.000, 0.500, NOT_D13);
      $setuphold(posedge CLK &&& D_flag13, negedge D[13], 1.000, 0.500, NOT_D13);
      $setuphold(posedge CLK &&& D_flag12, posedge D[12], 1.000, 0.500, NOT_D12);
      $setuphold(posedge CLK &&& D_flag12, negedge D[12], 1.000, 0.500, NOT_D12);
      $setuphold(posedge CLK &&& D_flag11, posedge D[11], 1.000, 0.500, NOT_D11);
      $setuphold(posedge CLK &&& D_flag11, negedge D[11], 1.000, 0.500, NOT_D11);
      $setuphold(posedge CLK &&& D_flag10, posedge D[10], 1.000, 0.500, NOT_D10);
      $setuphold(posedge CLK &&& D_flag10, negedge D[10], 1.000, 0.500, NOT_D10);
      $setuphold(posedge CLK &&& D_flag9, posedge D[9], 1.000, 0.500, NOT_D9);
      $setuphold(posedge CLK &&& D_flag9, negedge D[9], 1.000, 0.500, NOT_D9);
      $setuphold(posedge CLK &&& D_flag8, posedge D[8], 1.000, 0.500, NOT_D8);
      $setuphold(posedge CLK &&& D_flag8, negedge D[8], 1.000, 0.500, NOT_D8);
      $setuphold(posedge CLK &&& D_flag7, posedge D[7], 1.000, 0.500, NOT_D7);
      $setuphold(posedge CLK &&& D_flag7, negedge D[7], 1.000, 0.500, NOT_D7);
      $setuphold(posedge CLK &&& D_flag6, posedge D[6], 1.000, 0.500, NOT_D6);
      $setuphold(posedge CLK &&& D_flag6, negedge D[6], 1.000, 0.500, NOT_D6);
      $setuphold(posedge CLK &&& D_flag5, posedge D[5], 1.000, 0.500, NOT_D5);
      $setuphold(posedge CLK &&& D_flag5, negedge D[5], 1.000, 0.500, NOT_D5);
      $setuphold(posedge CLK &&& D_flag4, posedge D[4], 1.000, 0.500, NOT_D4);
      $setuphold(posedge CLK &&& D_flag4, negedge D[4], 1.000, 0.500, NOT_D4);
      $setuphold(posedge CLK &&& D_flag3, posedge D[3], 1.000, 0.500, NOT_D3);
      $setuphold(posedge CLK &&& D_flag3, negedge D[3], 1.000, 0.500, NOT_D3);
      $setuphold(posedge CLK &&& D_flag2, posedge D[2], 1.000, 0.500, NOT_D2);
      $setuphold(posedge CLK &&& D_flag2, negedge D[2], 1.000, 0.500, NOT_D2);
      $setuphold(posedge CLK &&& D_flag1, posedge D[1], 1.000, 0.500, NOT_D1);
      $setuphold(posedge CLK &&& D_flag1, negedge D[1], 1.000, 0.500, NOT_D1);
      $setuphold(posedge CLK &&& D_flag0, posedge D[0], 1.000, 0.500, NOT_D0);
      $setuphold(posedge CLK &&& D_flag0, negedge D[0], 1.000, 0.500, NOT_D0);
      $setuphold(posedge CLK &&& cyc_flag, posedge EMA[2], 1.000, 0.500, NOT_EMA2);
      $setuphold(posedge CLK &&& cyc_flag, negedge EMA[2], 1.000, 0.500, NOT_EMA2);
      $setuphold(posedge CLK &&& cyc_flag, posedge EMA[1], 1.000, 0.500, NOT_EMA1);
      $setuphold(posedge CLK &&& cyc_flag, negedge EMA[1], 1.000, 0.500, NOT_EMA1);
      $setuphold(posedge CLK &&& cyc_flag, posedge EMA[0], 1.000, 0.500, NOT_EMA0);
      $setuphold(posedge CLK &&& cyc_flag, negedge EMA[0], 1.000, 0.500, NOT_EMA0);
      $setuphold(posedge CLK, posedge RETN, 1.000, 0.500, NOT_RETN);
      $setuphold(posedge CLK, negedge RETN, 1.000, 0.500, NOT_RETN);
      $hold(posedge RETN, negedge CEN, 1.000, NOT_RETN);

      $width(posedge CLK &&& cyc_flag, 1.000, 0, NOT_CLK_MINH);
      $width(negedge CLK &&& cyc_flag, 1.000, 0, NOT_CLK_MINL);
`ifdef NO_SDTC
      $period(posedge CLK  &&& cyc_flag, 3.000, NOT_CLK_PER);
`else
      $period(posedge CLK &&& EMA2eq0andEMA1eq0andEMA0eq0, 3.000, NOT_CLK_PER);
      $period(posedge CLK &&& EMA2eq0andEMA1eq0andEMA0eq1, 3.000, NOT_CLK_PER);
      $period(posedge CLK &&& EMA2eq0andEMA1eq1andEMA0eq0, 3.000, NOT_CLK_PER);
      $period(posedge CLK &&& EMA2eq0andEMA1eq1andEMA0eq1, 3.000, NOT_CLK_PER);
      $period(posedge CLK &&& EMA2eq1andEMA1eq0andEMA0eq0, 3.000, NOT_CLK_PER);
      $period(posedge CLK &&& EMA2eq1andEMA1eq0andEMA0eq1, 3.000, NOT_CLK_PER);
      $period(posedge CLK &&& EMA2eq1andEMA1eq1andEMA0eq0, 3.000, NOT_CLK_PER);
      $period(posedge CLK &&& EMA2eq1andEMA1eq1andEMA0eq1, 3.000, NOT_CLK_PER);
`endif

      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[20]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[20]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[20]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[20]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[20]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[20]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[20]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[20]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[19]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[19]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[19]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[19]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[19]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[19]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[19]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[19]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[18]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[18]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[18]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[18]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[18]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[18]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[18]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[18]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[17]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[17]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[17]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[17]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[17]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[17]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[17]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[17]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[16]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[16]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[16]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[16]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[16]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[16]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[16]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[16]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[15]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[15]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[15]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[15]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[15]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[15]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[15]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[15]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[14]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[14]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[14]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[14]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[14]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[14]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[14]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[14]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[13]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[13]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[13]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[13]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[13]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[13]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[13]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[13]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[12]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[12]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[12]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[12]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[12]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[12]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[12]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[12]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[11]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[11]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[11]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[11]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[11]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[11]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[11]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[11]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[10]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[10]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[10]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[10]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[10]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[10]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[10]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[10]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[9]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[9]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[9]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[9]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[9]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[9]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[9]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[9]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[8]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[8]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[8]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[8]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[8]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[8]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[8]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[8]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[7]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[7]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[7]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[7]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[7]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[7]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[7]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[7]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[6]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[6]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[6]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[6]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[6]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[6]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[6]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[6]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[5]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[5]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[5]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[5]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[5]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[5]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[5]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[5]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[4]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[4]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[4]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[4]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[4]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[4]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[4]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[4]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[3]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[3]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[3]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[3]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[3]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[3]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[3]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[3]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[2]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[2]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[2]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[2]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[2]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[2]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[2]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[2]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[1]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[1]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[1]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[1]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[1]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[1]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[1]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[1]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[0]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[0]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[0]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b0) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[0]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[0]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b0) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[0]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b0))
        (posedge CLK => (Q[0]:1'b0))=(1.000, 1.000);
      if ((EMA[2] == 1'b1) && (EMA[1] == 1'b1) && (EMA[0] == 1'b1))
        (posedge CLK => (Q[0]:1'b0))=(1.000, 1.000);

      (RETN => (Q[20] +: 1'b0)) = (1.000);
      (RETN => (Q[19] +: 1'b0)) = (1.000);
      (RETN => (Q[18] +: 1'b0)) = (1.000);
      (RETN => (Q[17] +: 1'b0)) = (1.000);
      (RETN => (Q[16] +: 1'b0)) = (1.000);
      (RETN => (Q[15] +: 1'b0)) = (1.000);
      (RETN => (Q[14] +: 1'b0)) = (1.000);
      (RETN => (Q[13] +: 1'b0)) = (1.000);
      (RETN => (Q[12] +: 1'b0)) = (1.000);
      (RETN => (Q[11] +: 1'b0)) = (1.000);
      (RETN => (Q[10] +: 1'b0)) = (1.000);
      (RETN => (Q[9] +: 1'b0)) = (1.000);
      (RETN => (Q[8] +: 1'b0)) = (1.000);
      (RETN => (Q[7] +: 1'b0)) = (1.000);
      (RETN => (Q[6] +: 1'b0)) = (1.000);
      (RETN => (Q[5] +: 1'b0)) = (1.000);
      (RETN => (Q[4] +: 1'b0)) = (1.000);
      (RETN => (Q[3] +: 1'b0)) = (1.000);
      (RETN => (Q[2] +: 1'b0)) = (1.000);
      (RETN => (Q[1] +: 1'b0)) = (1.000);
      (RETN => (Q[0] +: 1'b0)) = (1.000);
  endspecify

endmodule
`endcelldefine
`endif