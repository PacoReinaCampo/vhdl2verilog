--============================================================================--
--==                                          __ _      _     _             ==--
--==                                         / _(_)    | |   | |            ==--
--==              __ _ _   _  ___  ___ _ __ | |_ _  ___| | __| |            ==--
--==             / _` | | | |/ _ \/ _ \ '_ \|  _| |/ _ \ |/ _` |            ==--
--==            | (_| | |_| |  __/  __/ | | | | | |  __/ | (_| |            ==--
--==             \__, |\__,_|\___|\___|_| |_|_| |_|\___|_|\__,_|            ==--
--==                | |                                                     ==--
--==                |_|                                                     ==--
--==                                                                        ==--
--==                                                                        ==--
--==            MSP430 CPU                                                  ==--
--==            Processing Unit                                             ==--
--==                                                                        ==--
--============================================================================--

-- Copyright (c) 2015-2016 by the author(s)
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in
-- all copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
-- THE SOFTWARE.
--
-- =============================================================================
-- Author(s):
--   Francisco Javier Reina Campo <pacoreinacampo@queenfield.tech>
--

library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity omsp_clock_mux is
  port (
    clk_out   : out std_logic;
    clk_in0   : in  std_logic;
    clk_in1   : in  std_logic;
    reset     : in  std_logic;
    scan_mode : in  std_logic;
    selection : in  std_logic);
end omsp_clock_mux;

architecture omsp_clock_mux_ARQ of omsp_clock_mux is

  signal in0_select    : std_logic;
  signal in0_select_s  : std_logic;
  signal in0_select_ss : std_logic;
  signal in0_enable    : std_logic;

  signal in1_select    : std_logic;
  signal in1_select_s  : std_logic;
  signal in1_select_ss : std_logic;
  signal in1_enable    : std_logic;

  signal clk_in0_inv : std_logic;
  signal clk_in1_inv : std_logic;

  signal gated_clk_in0 : std_logic;
  signal gated_clk_in1 : std_logic;

begin
  --CLK_IN0 Selection
  in0_select <= not selection and not in1_select_ss;

  process (clk_in0_inv, reset)
  begin
    if (reset = '1') then
      in0_select_s <= '1';
    elsif (rising_edge(clk_in0_inv)) then
      in0_select_s <= in0_select;
    end if;
  end process;

  process (clk_in0, reset)
  begin
    if (reset = '1') then
      in0_select_ss <= '1';
    elsif (rising_edge(clk_in0)) then
      in0_select_ss <= in0_select_s;
    end if;
  end process;

  in0_enable <= in0_select_ss or scan_mode;

  --CLK_IN1 Selection   
  in1_select <= selection and not in0_select_ss;

  process (clk_in1_inv, reset)
  begin
    if (reset = '1') then
      in1_select_s <= '0';
    elsif (rising_edge(clk_in1_inv)) then
      in1_select_s <= in1_select;
    end if;
  end process;

  process (clk_in1, reset)
  begin
    if (reset = '1') then
      in1_select_ss <= '0';
    elsif (rising_edge(clk_in1)) then
      in1_select_ss <= in1_select_s;
    end if;
  end process;

  in1_enable <= in1_select_ss and not scan_mode;

  --Clock MUX
  clk_in0_inv   <= not clk_in0;
  clk_in1_inv   <= not clk_in1;
  gated_clk_in0 <= not (clk_in0_inv and in0_enable);
  gated_clk_in1 <= not (clk_in1_inv and in1_enable);
  clk_out       <= gated_clk_in0 and gated_clk_in1;
end omsp_clock_mux_ARQ;
