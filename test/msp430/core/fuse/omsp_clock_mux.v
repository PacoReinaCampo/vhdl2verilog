// Converted from msp430/core/fuse/omsp_clock_mux.vhd
// by verilog2vhdl - QueenField


//============================================================================--
//==                                          __ _      _     _             ==--
//==                                         / _(_)    | |   | |            ==--
//==              __ _ _   _  ___  ___ _ __ | |_ _  ___| | __| |            ==--
//==             / _` | | | |/ _ \/ _ \ '_ \|  _| |/ _ \ |/ _` |            ==--
//==            | (_| | |_| |  __/  __/ | | | | | |  __/ | (_| |            ==--
//==             \__, |\__,_|\___|\___|_| |_|_| |_|\___|_|\__,_|            ==--
//==                | |                                                     ==--
//==                |_|                                                     ==--
//==                                                                        ==--
//==                                                                        ==--
//==            MSP430 CPU                                                  ==--
//==            Processing Unit                                             ==--
//==                                                                        ==--
//============================================================================--
// Copyright (c) 2015-2016 by the author(s)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// =============================================================================
// Author(s):
//   Francisco Javier Reina Campo <frareicam@gmail.com>
//
module omsp_clock_mux(clk_out, clk_in0, clk_in1, reset, scan_mode, selection);
  output   clk_out;
  input   clk_in0;
  input   clk_in1;
  input   reset;
  input   scan_mode;
  input   selection;

  wire   in0_select;
  reg    in0_select_s;
  reg    in0_select_ss;
  wire   in0_enable;
  wire   in1_select;
  reg    in1_select_s;
  reg    in1_select_ss;
  wire   in1_enable;
  wire   clk_in0_inv;
  wire   clk_in1_inv;
  wire   gated_clk_in0;
  wire   gated_clk_in1;

  //CLK_IN0 Selection
  assign in0_select =  ~selection &  ~in1_select_ss;
  always @(clk_in0_inv or reset) begin
    if (reset == 1'b1) begin
      in0_select_s <= 1'b1;
    end else if (rising_edge[clk_in0_inv]) begin
      in0_select_s <= in0_select;
    end
  end
  always @(clk_in0 or reset) begin
    if (reset == 1'b1) begin
      in0_select_ss <= 1'b1;
    end else if (rising_edge[clk_in0]) begin
      in0_select_ss <= in0_select_s;
    end
  end
  assign in0_enable = in0_select_ss | scan_mode;
  //CLK_IN1 Selection   
  assign in1_select = selection &  ~in0_select_ss;
  always @(clk_in1_inv or reset) begin
    if (reset == 1'b1) begin
      in1_select_s <= 1'b0;
    end else if (rising_edge[clk_in1_inv]) begin
      in1_select_s <= in1_select;
    end
  end
  always @(clk_in1 or reset) begin
    if (reset == 1'b1) begin
      in1_select_ss <= 1'b0;
    end else if (rising_edge[clk_in1]) begin
      in1_select_ss <= in1_select_s;
    end
  end
  assign in1_enable = in1_select_ss &  ~scan_mode;
  //Clock MUX
  assign clk_in0_inv =  ~clk_in0;
  assign clk_in1_inv =  ~clk_in1;
  assign gated_clk_in0 =  ~(clk_in0_inv & in0_enable);
  assign gated_clk_in1 =  ~(clk_in1_inv & in1_enable);
  assign clk_out = gated_clk_in0 & gated_clk_in1;
endmodule
