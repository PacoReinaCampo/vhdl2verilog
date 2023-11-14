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

entity omsp_register_file is
  port (
    r0  : out std_logic_vector (15 downto 0);
    r1  : out std_logic_vector (15 downto 0);
    r2  : out std_logic_vector (15 downto 0);
    r3  : out std_logic_vector (15 downto 0);
    r4  : out std_logic_vector (15 downto 0);
    r5  : out std_logic_vector (15 downto 0);
    r6  : out std_logic_vector (15 downto 0);
    r7  : out std_logic_vector (15 downto 0);
    r8  : out std_logic_vector (15 downto 0);
    r9  : out std_logic_vector (15 downto 0);
    r10 : out std_logic_vector (15 downto 0);
    r11 : out std_logic_vector (15 downto 0);
    r12 : out std_logic_vector (15 downto 0);
    r13 : out std_logic_vector (15 downto 0);
    r14 : out std_logic_vector (15 downto 0);
    r15 : out std_logic_vector (15 downto 0);

    cpuoff   : out std_logic;
    gie      : out std_logic;
    oscoff   : out std_logic;
    pc_sw_wr : out std_logic;
    scg0     : out std_logic;
    scg1     : out std_logic;
    status   : out std_logic_vector (3 downto 0);
    pc_sw    : out std_logic_vector (15 downto 0);
    reg_dest : out std_logic_vector (15 downto 0);
    reg_src  : out std_logic_vector (15 downto 0);

    inst_bw      : in std_logic;
    mclk         : in std_logic;
    puc_rst      : in std_logic;
    reg_dest_wr  : in std_logic;
    reg_pc_call  : in std_logic;
    reg_sp_wr    : in std_logic;
    reg_sr_wr    : in std_logic;
    reg_sr_clr   : in std_logic;
    reg_incr     : in std_logic;
    scan_enable  : in std_logic;
    alu_stat     : in std_logic_vector (3 downto 0);
    alu_stat_wr  : in std_logic_vector (3 downto 0);
    inst_dest    : in std_logic_vector (15 downto 0);
    inst_src     : in std_logic_vector (15 downto 0);
    pc           : in std_logic_vector (15 downto 0);
    reg_dest_val : in std_logic_vector (15 downto 0);
    reg_sp_val   : in std_logic_vector (15 downto 0));
end omsp_register_file;

architecture omsp_register_file_ARQ of omsp_register_file is

  -- SIGNAL INOUT
  signal reg_src_omsp : std_logic_vector (15 downto 0);

  -- 1.REGISTER FILE
  -- 1.1.AUTOINCREMENT UNIT
  signal inst_src_in     : std_logic_vector (15 downto 0);
  signal incr_op         : std_logic_vector (15 downto 0);
  signal reg_incr_val    : std_logic_vector (15 downto 0);
  signal reg_dest_val_in : std_logic_vector (15 downto 0);

  -- 1.2.SPECIAL REGISTERS (R1/R2/R3)
  -- R0: Program counter
  signal re : std_logic_matrix (3 downto 0)(15 downto 0);

  -- R1: Stack pointer
  signal mclk_r1 : std_logic;
  signal r1_en   : std_logic;
  signal r1_inc  : std_logic;
  signal r1_wr   : std_logic;

  -- R2: Status register
  signal mclk_r2     : std_logic;
  signal r2_c        : std_logic;
  signal r2_en       : std_logic;
  signal r2_n        : std_logic;
  signal r2_v        : std_logic;
  signal r2_wr       : std_logic;
  signal r2_z        : std_logic;
  signal r2_nxt      : std_logic_vector (7 downto 3);
  signal cpuoff_mask : std_logic_vector (15 downto 0);
  signal oscoff_mask : std_logic_vector (15 downto 0);
  signal r2_mask     : std_logic_vector (15 downto 0);
  signal scg_mask    : std_logic_matrix (1 downto 0)(15 downto 0);

  -- R3: Constant generator
  signal mclk_r3 : std_logic;
  signal r3_en   : std_logic;
  signal r3_wr   : std_logic;

  -- 1.3.GENERAL PURPOSE REGISTERS (R4...R15)
  signal rg      : std_logic_matrix (15 downto 0)(15 downto 0);
  signal mclk_rg : std_logic_vector (15 downto 4);
  signal rg_en   : std_logic_vector (15 downto 4);
  signal rg_inc  : std_logic_vector (15 downto 4);
  signal rg_wr   : std_logic_vector (15 downto 4);

begin
  -- 1.1.AUTOINCREMENT UNIT
  incr_op         <= X"0001"                            when (inst_bw and not inst_src_in(1)) = '1' else X"0002";
  reg_incr_val    <= std_logic_vector(unsigned(reg_src_omsp) + unsigned(incr_op));
  reg_dest_val_in <= (X"00" & reg_dest_val(7 downto 0)) when inst_bw = '1'                          else reg_dest_val;

  -- 1.2.SPECIAL REGISTERS (R1/R2/R3)
  -- Source input selection mask (for interrupt support)
  inst_src_in <= X"0004" when reg_sr_clr = '1' else inst_src;

  -- R0: Program counter
  re(0)    <= pc;
  pc_sw    <= reg_dest_val_in;
  pc_sw_wr <= (inst_dest(0) and reg_dest_wr) or reg_pc_call;

  -- R1: Stack pointer
  r1_wr  <= inst_dest(1) and reg_dest_wr;
  r1_inc <= inst_src_in(1) and reg_incr;

  clock_gating_on : if (CLOCK_GATING = '1') generate
    r1_en <= r1_wr or reg_sp_wr or r1_inc;

    clock_gate_r1 : omsp_clock_gate
      port map (
        gclk        => mclk_r1,
        clk         => mclk,
        enable      => r1_en,
        scan_enable => scan_enable);
  end generate clock_gating_on;

  clock_gating_off : if (CLOCK_GATING = '0') generate
    mclk_r1 <= mclk;
  end generate clock_gating_off;

  R_1c_2c_3i_4ci : process (mclk_r1, puc_rst)
  begin
    if (puc_rst = '1') then
      re(1) <= X"0000";
    elsif (rising_edge(mclk_r1)) then
      if (r1_wr = '1') then
        re(1) <= reg_dest_val_in and X"FFFE";
      elsif (reg_sp_wr = '1') then
        re(1) <= reg_sp_val and X"FFFE";
      elsif (CLOCK_GATING = '1') then
        re(1) <= reg_incr_val and X"FFFE";
      elsif (r1_inc = '1' and CLOCK_GATING = '0') then
        re(1) <= reg_incr_val and X"FFFE";
      end if;
    end if;
  end process R_1c_2c_3i_4ci;

  -- R2: Status register
  r2_wr <= (inst_dest(2) and reg_dest_wr) or reg_sr_wr;

  clock_gating_1_on : if (CLOCK_GATING = '1') generate
    r2_c   <= alu_stat(0)                 when alu_stat_wr(0) = '1' else reg_dest_val_in(0);
    r2_z   <= alu_stat(1)                  when alu_stat_wr(1) = '1' else reg_dest_val_in(1);
    r2_n   <= alu_stat(2)                 when alu_stat_wr(2) = '1' else reg_dest_val_in(2);
    r2_nxt <= reg_dest_val_in(7 downto 3) when r2_wr = '1'          else re(2)(7 downto 3);
    r2_v   <= alu_stat(3)                 when alu_stat_wr(3) = '1' else reg_dest_val_in(8);
    r2_en  <= reduce_or(alu_stat_wr) or r2_wr or reg_sr_clr;

    clock_gate_r2 : omsp_clock_gate
      port map (
        gclk        => mclk_r2,
        clk         => mclk,
        enable      => r2_en,
        scan_enable => scan_enable);
  end generate clock_gating_1_on;

  clock_gating_1_off : if (CLOCK_GATING = '0') generate
    r2_c <= alu_stat(0) when alu_stat_wr(0) = '1' else reg_dest_val_in(0)
            when r2_wr = '1' else re(2)(0);
    r2_z <= alu_stat(1) when alu_stat_wr(1) = '1' else reg_dest_val_in(1)
            when r2_wr = '1' else re(2)(1);
    r2_n <= alu_stat(2) when alu_stat_wr(2) = '1' else reg_dest_val_in(2)
            when r2_wr = '1' else re(2)(2);
    r2_nxt <= reg_dest_val_in(7 downto 3) when r2_wr = '1'          else re(2)(7 downto 3);
    r2_v   <= alu_stat(3)                 when alu_stat_wr(3) = '1' else reg_dest_val_in(8)
            when r2_wr = '1' else re(2)(8);
    mclk_r2 <= mclk;
  end generate clock_gating_1_off;

  asic_clocking_r2_on : if (ASIC_CLOCKING = '1') generate
    cpuoff_en_on  : if (CPUOFF_EN = '1') generate cpuoff_mask <= X"0010"; end generate cpuoff_en_on;
    cpuoff_en_off : if (CPUOFF_EN = '0') generate cpuoff_mask <= X"0000"; end generate cpuoff_en_off;

    oscoff_en_on  : if (OSCOFF_EN = '1') generate oscoff_mask <= X"0020"; end generate oscoff_en_on;
    oscoff_en_off : if (OSCOFF_EN = '0') generate oscoff_mask <= X"0000"; end generate oscoff_en_off;

    scg0_en_on  : if (SCG_EN_0 = '1') generate scg_mask(0) <= X"0040"; end generate scg0_en_on;
    scg0_en_off : if (SCG_EN_0 = '0') generate scg_mask(0) <= X"0000"; end generate scg0_en_off;

    scg1_en_on  : if (SCG_EN_1 = '1') generate scg_mask(1) <= X"0080"; end generate scg1_en_on;
    scg1_en_off : if (SCG_EN_1 = '0') generate scg_mask(1) <= X"0000"; end generate scg1_en_off;
  end generate asic_clocking_r2_on;

  asic_clocking_r2_off : if (ASIC_CLOCKING = '0') generate
    cpuoff_mask <= X"0010";
    oscoff_mask <= X"0020";
    scg_mask(0) <= X"0000";
    scg_mask(1) <= X"0080";
  end generate asic_clocking_r2_off;

  r2_mask <= cpuoff_mask or oscoff_mask or scg_mask(0) or scg_mask(1) or "0000000100001111";

  R_1c_2 : process (mclk_r2, puc_rst)
  begin
    if (puc_rst = '1') then
      re(2) <= X"0000";
    elsif (rising_edge(mclk_r2)) then
      if (reg_sr_clr = '1') then
        re(2) <= X"0000";
      else
        re(2) <= ("0000000" & r2_v & r2_nxt & r2_n & r2_z & r2_c) and r2_mask;
      end if;
    end if;
  end process R_1c_2;

  status <= re(2)(8) & re(2)(2 downto 0);
  gie    <= re(2)(3);
  cpuoff <= re(2)(4) or (r2_nxt(4) and r2_wr and cpuoff_mask(4));
  oscoff <= re(2)(5);
  scg0   <= re(2)(6);
  scg1   <= re(2)(7);

  -- R3: Constant generator
  r3_wr <= inst_dest(3) and reg_dest_wr;

  clock_gating_2_on : if (CLOCK_GATING = '1') generate
    r3_en <= r3_wr;

    clock_gate_r3 : omsp_clock_gate
      port map (
        gclk        => mclk_r3,
        clk         => mclk,
        enable      => r3_en,
        scan_enable => scan_enable);
  end generate clock_gating_2_on;

  clock_gating_2_off : if (CLOCK_GATING = '0') generate
    mclk_r3 <= mclk;
  end generate clock_gating_2_off;

  R_1i_2ci : process (mclk_r3, puc_rst)
  begin
    if (puc_rst = '1') then
      re(3) <= X"0000";
    elsif (rising_edge(mclk_r3)) then
      if (CLOCK_GATING = '1') then
        re(3) <= reg_dest_val_in;
      elsif (r3_wr = '1' and CLOCK_GATING = '0') then
        re(3) <= reg_dest_val_in;
      end if;
    end if;
  end process R_1i_2ci;

  -- 1.3.GENERAL PURPOSE REGISTERS (R4...R15)
  GPR : for i in 15 downto 4 generate
    rg_wr(i)  <= inst_dest(i) and reg_dest_wr;
    rg_inc(i) <= inst_src_in(i) and reg_incr;

    clock_gating_rgt_on : if (CLOCK_GATING = '1') generate
      rg_en(i) <= rg_wr(i) or rg_inc(i);

      clock_gate_rgt : omsp_clock_gate
        port map (
          gclk        => mclk_rg(i),
          clk         => mclk,
          enable      => rg_en(i),
          scan_enable => scan_enable);
    end generate clock_gating_rgt_on;

    clock_gating_rgt_off : if (CLOCK_GATING = '0') generate
      mclk_rg(i) <= mclk;
    end generate clock_gating_rgt_off;

    R_1c_2i_3ci : process (mclk_rg(i), puc_rst)
    begin
      if (puc_rst = '1') then
        rg(i) <= X"0000";
      elsif (rising_edge(mclk_rg(i))) then
        if (rg_wr(i) = '1') then
          rg(i) <= reg_dest_val_in;
        elsif (CLOCK_GATING = '1') then
          rg(i) <= reg_incr_val;
        elsif (rg_inc(i) = '1' and CLOCK_GATING = '0') then
          rg(i) <= reg_incr_val;
        end if;
      end if;
    end process R_1c_2i_3ci;
  end generate GPR;

  -- 1.4.READ MUX
  reg_src_omsp <= (re(0) and (0 to 15 => inst_src_in(0))) or
                  (re(1) and (0 to 15  => inst_src_in(1))) or
                  (re(2) and (0 to 15  => inst_src_in(2))) or
                  (re(3) and (0 to 15  => inst_src_in(3))) or
                  (rg(4) and (0 to 15  => inst_src_in(4))) or
                  (rg(5) and (0 to 15  => inst_src_in(5))) or
                  (rg(6) and (0 to 15  => inst_src_in(6))) or
                  (rg(7) and (0 to 15  => inst_src_in(7))) or
                  (rg(8) and (0 to 15  => inst_src_in(8))) or
                  (rg(9) and (0 to 15  => inst_src_in(9))) or
                  (rg(10) and (0 to 15 => inst_src_in(10))) or
                  (rg(11) and (0 to 15 => inst_src_in(11))) or
                  (rg(12) and (0 to 15 => inst_src_in(12))) or
                  (rg(13) and (0 to 15 => inst_src_in(13))) or
                  (rg(14) and (0 to 15 => inst_src_in(14))) or
                  (rg(15) and (0 to 15 => inst_src_in(15)));

  reg_dest <= (re(0) and (0 to 15 => inst_dest(0))) or
              (re(1) and (0 to 15  => inst_dest(1))) or
              (re(2) and (0 to 15  => inst_dest(2))) or
              (re(3) and (0 to 15  => inst_dest(3))) or
              (rg(4) and (0 to 15  => inst_dest(4))) or
              (rg(5) and (0 to 15  => inst_dest(5))) or
              (rg(6) and (0 to 15  => inst_dest(6))) or
              (rg(7) and (0 to 15  => inst_dest(7))) or
              (rg(8) and (0 to 15  => inst_dest(8))) or
              (rg(9) and (0 to 15  => inst_dest(9))) or
              (rg(10) and (0 to 15 => inst_dest(10))) or
              (rg(11) and (0 to 15 => inst_dest(11))) or
              (rg(12) and (0 to 15 => inst_dest(12))) or
              (rg(13) and (0 to 15 => inst_dest(13))) or
              (rg(14) and (0 to 15 => inst_dest(14))) or
              (rg(15) and (0 to 15 => inst_dest(15)));

  SIGNAL_INOUT : block
  begin
    reg_src <= reg_src_omsp;

    r0 <= re(0);
    r1 <= re(1);
    r2 <= re(2);
    r3 <= re(3);

    r4  <= rg(4);
    r5  <= rg(5);
    r6  <= rg(6);
    r7  <= rg(7);
    r8  <= rg(8);
    r9  <= rg(9);
    r10 <= rg(10);
    r11 <= rg(11);
    r12 <= rg(12);
    r13 <= rg(13);
    r14 <= rg(14);
    r15 <= rg(15);
  end block SIGNAL_INOUT;
end omsp_register_file_ARQ;
