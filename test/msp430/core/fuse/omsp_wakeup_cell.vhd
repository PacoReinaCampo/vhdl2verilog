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
--   Francisco Javier Reina Campo <frareicam@gmail.com>
--

library IEEE;
use IEEE.STD_LOGIC_1164 .all;
use WORK.MSP430_PACK .all;

entity omsp_wakeup_cell is
  port (
    wkup_out   : out std_logic;
    scan_clk   : in  std_logic;
    scan_mode  : in  std_logic;
    scan_rst   : in  std_logic;
    wkup_clear : in  std_logic;
    wkup_event : in  std_logic);
end omsp_wakeup_cell;

architecture omsp_wakeup_cell_ARQ of omsp_wakeup_cell is

  signal wkup_rst : std_logic;
  signal wkup_clk : std_logic;

begin
  --Scan stuff for the ASIC mode
  asic_on : if (ASIC = '1') generate
    scan_mux_rst : omsp_scan_mux
      port map (
        data_out     => wkup_rst,
        data_in_scan => scan_rst,
        data_in_func => wkup_clear,
        scan_mode    => scan_mode);

    scan_mux_clk : omsp_scan_mux
      port map (
        data_out     => wkup_clk,
        data_in_scan => scan_clk,
        data_in_func => wkup_event,
        scan_mode    => scan_mode);
  end generate asic_on;

  asic_off : if (ASIC = '0') generate
    wkup_rst <= wkup_clear;
    wkup_clk <= wkup_event;
  end generate asic_off;

  --Wakeup capture
  process (wkup_clk, wkup_rst)
  begin
    if (wkup_rst = '1') then
      wkup_out <= '0';
    elsif (rising_edge(wkup_clk)) then
      wkup_out <= '1';
    end if;
  end process;
end omsp_wakeup_cell_ARQ;
