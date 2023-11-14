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
use IEEE.STD_LOGIC_1164 .all;
use IEEE.NUMERIC_STD .all;
use WORK.MSP430_PACK .all;

entity omsp_dbg_hwbrk is
  port (
    brk_halt : out std_logic;
    brk_pnd  : out std_logic;
    brk_dout : out std_logic_vector (15 downto 0);

    dbg_clk      : in std_logic;
    dbg_rst      : in std_logic;
    decode_noirq : in std_logic;
    eu_mb_en     : in std_logic;
    eu_mb_wr     : in std_logic_vector (1 downto 0);
    brk_reg_rd   : in std_logic_vector (3 downto 0);
    brk_reg_wr   : in std_logic_vector (3 downto 0);
    dbg_din      : in std_logic_vector (15 downto 0);
    eu_mab       : in std_logic_vector (15 downto 0);
    pc           : in std_logic_vector (15 downto 0));
end omsp_dbg_hwbrk;

architecture omsp_dbg_hwbrk_ARQ of omsp_dbg_hwbrk is

  -- 0. WIRE & PARAMETER DECLARATION
  signal range_wr_set : std_logic;
  signal range_rd_set : std_logic;
  signal addr1_wr_set : std_logic;
  signal addr1_rd_set : std_logic;
  signal addr0_wr_set : std_logic;
  signal addr0_rd_set : std_logic;

  constant BRK_CTLC   : integer := 0;
  constant BRK_STATC  : integer := 1;
  constant BRK_ADDR0C : integer := 2;
  constant BRK_ADDR1C : integer := 3;

  -- 1.CONFIGURATION REGISTERS
  -- BRK_CTL Register
  signal brk_ctl_wr   : std_logic;
  signal brk_ctl      : std_logic_vector (4 downto 0);
  signal brk_ctl_full : std_logic_vector (7 downto 0);

  -- BRK_STAT Register
  signal brk_stat_wr   : std_logic;
  signal brk_stat_set  : std_logic_vector (5 downto 0);
  signal brk_stat      : std_logic_vector (5 downto 0);
  signal brk_stat_clr  : std_logic_vector (5 downto 0);
  signal brk_stat_full : std_logic_vector (7 downto 0);

  -- BRK_ADDR0 Register
  signal brk_addr_wr : std_logic_vector (1 downto 0);
  signal brk_addr    : std_logic_matrix (1 downto 0)(15 downto 0);

  -- 2.DATA OUTPUT GENERATION
  signal brk_ctl_rd   : std_logic_vector (15 downto 0);
  signal brk_stat_rd  : std_logic_vector (15 downto 0);
  signal brk_addr0_rd : std_logic_vector (15 downto 0);
  signal brk_addr1_rd : std_logic_vector (15 downto 0);

  -- 3.BREAKPOINT / WATCHPOINT GENERATION
  -- Comparators
  signal equ_d_addr0 : std_logic;
  signal equ_d_addr1 : std_logic;
  signal equ_d_range : std_logic;

  signal equ_i_addr0 : std_logic;
  signal equ_i_addr1 : std_logic;
  signal equ_i_range : std_logic;

  -- Detect accesses
  signal i_addr0_rd : std_logic;
  signal i_addr1_rd : std_logic;
  signal i_range_rd : std_logic;

  signal d_addr0_wr : std_logic;
  signal d_addr1_wr : std_logic;
  signal d_range_wr : std_logic;

  signal d_addr0_rd : std_logic;
  signal d_addr1_rd : std_logic;
  signal d_range_rd : std_logic;

begin
  CONFIGURATION_REGISTERS : block
  begin
    -- BRK_CTL Register
    brk_ctl_wr <= brk_reg_wr(BRK_CTLC);

    process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        brk_ctl <= (others => '0');
      elsif (rising_edge(dbg_clk)) then
        if (brk_ctl_wr = '1') then
          brk_ctl <= (HWBRK_RANGE and dbg_din(BRK_RANGE)) & dbg_din (BRK_RANGE - 1 downto 0);
        end if;
      end if;
    end process;

    brk_ctl_full <= "000" & brk_ctl;

    -- BRK_STAT Register
    brk_stat_wr  <= brk_reg_wr(BRK_STATC);
    brk_stat_set <= (range_wr_set and HWBRK_RANGE) & (range_rd_set and HWBRK_RANGE) & addr1_wr_set &
                    addr1_rd_set & addr0_wr_set & addr0_rd_set;
    brk_stat_clr <= not dbg_din(5 downto 0);

    process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        brk_stat <= (others => '0');
      elsif (rising_edge(dbg_clk)) then
        if (brk_stat_wr = '1') then
          brk_stat <= (brk_stat and brk_stat_clr) or brk_stat_set;
        else
          brk_stat <= brk_stat or brk_stat_set;
        end if;
      end if;
    end process;

    brk_stat_full <= "00" & brk_stat;
    brk_pnd       <= reduce_or(brk_stat);

    -- BRK_ADDR0 Register
    brk_addr_wr(0) <= brk_reg_wr(BRK_ADDR0C);

    process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        brk_addr(0) <= (others => '0');
      elsif (rising_edge(dbg_clk)) then
        if (brk_addr_wr(0) = '1') then
          brk_addr(0) <= dbg_din;
        end if;
      end if;
    end process;

    -- BRK_ADDR1/DATA0 Register  
    brk_addr_wr(1) <= brk_reg_wr(BRK_ADDR1C);

    process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        brk_addr(1) <= (others => '0');
      elsif (rising_edge(dbg_clk)) then
        if (brk_addr_wr(1) = '1') then
          brk_addr(1) <= dbg_din;
        end if;
      end if;
    end process;
  end block CONFIGURATION_REGISTERS;

  DATA_OUTPUT_GENERATION : block
  begin
    brk_ctl_rd   <= ("00000000" & brk_ctl_full)  and (0 to 15 => brk_reg_rd(BRK_CTLC));
    brk_stat_rd  <= ("00000000" & brk_stat_full) and (0 to 15 => brk_reg_rd(BRK_STATC));
    brk_addr0_rd <= brk_addr(0)                  and (0 to 15 => brk_reg_rd(BRK_ADDR0C));
    brk_addr1_rd <= brk_addr(1)                  and (0 to 15 => brk_reg_rd(BRK_ADDR1C));
    brk_dout     <= brk_ctl_rd or brk_stat_rd or brk_addr0_rd or brk_addr1_rd;
  end block DATA_OUTPUT_GENERATION;

  BREAKPOINT_WATCHPOINT_GENERATION : block
  begin
    -- Comparators
    equ_d_addr0 <= eu_mb_en and to_stdlogic(eu_mab = brk_addr(0)) and not brk_ctl(BRK_RANGE);
    equ_d_addr1 <= eu_mb_en and to_stdlogic(eu_mab = brk_addr(1)) and not brk_ctl(BRK_RANGE);
    equ_d_range <= eu_mb_en and (to_stdlogic(eu_mab >= brk_addr(0)) and to_stdlogic(eu_mab <= brk_addr(1))) and brk_ctl(BRK_RANGE) and HWBRK_RANGE;

    equ_i_addr0 <= decode_noirq and to_stdlogic(pc = brk_addr(0)) and not brk_ctl(BRK_RANGE);
    equ_i_addr1 <= decode_noirq and to_stdlogic(pc = brk_addr(1)) and not brk_ctl(BRK_RANGE);
    equ_i_range <= decode_noirq and (to_stdlogic(pc >= brk_addr(0)) and to_stdlogic(pc <= brk_addr(1))) and brk_ctl(BRK_RANGE) and HWBRK_RANGE;

    -- Detect accesses   
    i_addr0_rd <= equ_i_addr0 and brk_ctl(BRK_I_EN);
    i_addr1_rd <= equ_i_addr1 and brk_ctl(BRK_I_EN);
    i_range_rd <= equ_i_range and brk_ctl(BRK_I_EN);

    d_addr0_wr <= equ_d_addr0 and not brk_ctl(BRK_I_EN) and (reduce_or(eu_mb_wr));
    d_addr1_wr <= equ_d_addr1 and not brk_ctl(BRK_I_EN) and (reduce_or(eu_mb_wr));
    d_range_wr <= equ_d_range and not brk_ctl(BRK_I_EN) and (reduce_or(eu_mb_wr));

    d_addr0_rd <= equ_d_addr0 and not brk_ctl(BRK_I_EN) and not (reduce_or(eu_mb_wr));
    d_addr1_rd <= equ_d_addr1 and not brk_ctl(BRK_I_EN) and not (reduce_or(eu_mb_wr));
    d_range_rd <= equ_d_range and not brk_ctl(BRK_I_EN) and not (reduce_or(eu_mb_wr));

    addr0_rd_set <= brk_ctl(BRK_MODE_RD) and (d_addr0_rd or i_addr0_rd);
    addr0_wr_set <= brk_ctl(BRK_MODE_WR) and d_addr0_wr;
    addr1_rd_set <= brk_ctl(BRK_MODE_RD) and (d_addr1_rd or i_addr1_rd);
    addr1_wr_set <= brk_ctl(BRK_MODE_WR) and d_addr1_wr;
    range_rd_set <= brk_ctl(BRK_MODE_RD) and (d_range_rd or i_range_rd);
    range_wr_set <= brk_ctl(BRK_MODE_WR) and d_range_wr;

    brk_halt <= brk_ctl(BRK_EN) and reduce_or(brk_stat_set);
  end block BREAKPOINT_WATCHPOINT_GENERATION;
end omsp_dbg_hwbrk_ARQ;
