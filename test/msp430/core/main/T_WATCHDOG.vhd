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
use IEEE.NUMERIC_STD .all;
use WORK.MSP430_PACK .all;

entity T_WATCHDOG is
  port (
    wdt_reset      : out std_logic;
    wdt_wkup       : out std_logic;
    wdtifg         : out std_logic;
    wdtnmies       : out std_logic;
    aclk           : in  std_logic;
    aclk_en        : in  std_logic;
    dbg_freeze     : in  std_logic;
    por            : in  std_logic;
    scan_enable    : in  std_logic;
    scan_mode      : in  std_logic;
    smclk          : in  std_logic;
    smclk_en       : in  std_logic;
    wdtie          : in  std_logic;
    wdtifg_irq_clr : in  std_logic;
    wdtifg_sw_clr  : in  std_logic;
    wdtifg_sw_set  : in  std_logic;

    wdt_irq : out std_logic;

    per_dout : out std_logic_vector (15 downto 0);
    mclk     : in  std_logic;
    per_en   : in  std_logic;
    puc_rst  : in  std_logic;
    per_we   : in  std_logic_vector (1 downto 0);
    per_addr : in  std_logic_vector (13 downto 0);
    per_din  : in  std_logic_vector (15 downto 0));
end T_WATCHDOG;

architecture T_WATCHDOG_ARQ of T_WATCHDOG is

  --SIGNAL INOUT
  signal wdtifg_omsp : std_logic;

  --0.  PARAMETER DECLARATION
  --0.1.        Register base address (must be aligned to decoder bit width)
  constant BASE_ADDR_W : std_logic_vector (14 downto 0) := "000000100100000";

  --0.2.                Decoder bit width (defines how many bits are considered for address decoding)
  constant DEC_WD_W : integer := 2;

  --0.3.        Register addresses offset
  constant WDTCTLB : std_logic_vector (DEC_WD_W - 1 downto 0) := (others => '0');
  constant WDTCTLC : integer                                   := to_integer(unsigned(WDTCTLB));

  --0.4.        Register one-hot decoder utilities
  constant DEC_SZ_W   : integer                                   := 2**DEC_WD_W;
  constant BASE_REG_W : std_logic_vector (DEC_SZ_W - 1 downto 0) := std_logic_vector(to_unsigned(1, DEC_SZ_W));

  --0.5.        Register one-hot decoder
  constant WDTCTLC_D : std_logic_vector (DEC_SZ_W - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_W) sll WDTCTLC);

  --1.  REGISTER DECODER
  --1.1.        Local register selection
  signal reg_sel_w : std_logic;

  --1.2.        Register local address
  signal reg_addr_w : std_logic_vector (DEC_WD_W - 1 downto 0);

  --1.3.        Register address decode
  signal reg_dec_w : std_logic_vector (DEC_SZ_W - 1 downto 0);

  --1.4.        Read/Write probes
  signal reg_write_w : std_logic;
  signal reg_read_w  : std_logic;

  --1.5.        Read/Write vectors
  signal reg_wr_w : std_logic_vector (DEC_SZ_W - 1 downto 0);
  signal reg_rd_w : std_logic_vector (DEC_SZ_W - 1 downto 0);

  --2.  REGISTERS
  --2.1.        WDTCTLC Register
  signal wdtctl_wr   : std_logic;
  signal mclk_wdtctl : std_logic;
  signal wdtpw_error : std_logic;
  signal wdttmsel    : std_logic;
  signal wdtctl      : std_logic_vector (7 downto 0);

  --3.  DATA OUTPUT GENERATION
  --3.1.        Data output mux

  --4.  WATCHDOG TIMER (ASIC IMPLEMENTATION)    
  --4.1.        Watchdog clock source selection
  signal wdt_clk : std_logic;

  --4.2.        Reset synchronizer for the watchdog local clock domain
  signal wdt_rst_noscan : std_logic;
  signal wdt_rst        : std_logic;

  --4.3.        Watchog counter clear (synchronization)
  signal wdtcnt_clr_toggle   : std_logic;
  signal wdtcnt_clr_detect   : std_logic;
  signal wdtcnt_clr_sync     : std_logic;
  signal wdtcnt_clr_sync_dly : std_logic;
  signal wdtqn_edge          : std_logic;
  signal wdtcnt_clr          : std_logic;

  --4.4.        Watchog counter increment (synchronization)
  signal wdtctl_dbg : std_logic;

  --4.5.        Watchdog 16 bit counter
  signal wdtcnt_en   : std_logic;
  signal wdt_clk_cnt : std_logic;
  signal wdtisx_s    : std_logic_vector (1 downto 0);
  signal wdtisx_ss   : std_logic_vector (1 downto 0);
  signal wdtcnt      : std_logic_vector (15 downto 0);
  signal wdtcnt_nxt  : std_logic_vector (15 downto 0);

  --4.6.        Interval selection mux
  --4.7.        Watchdog event detection        
  signal wdt_evt_toggle          : std_logic;
  signal wdt_evt_toggle_sync     : std_logic;
  signal wdt_evt_toggle_sync_dly : std_logic;
  signal wdtifg_evt              : std_logic;

  --4.8.        Watchdog wakeup generation
  signal wdtifg_clr_reg : std_logic;
  signal wdtqn_edge_reg : std_logic;
  signal wdt_wkup_pre   : std_logic;
  signal wdt_wkup_en    : std_logic;

  --4.9.        Watchdog interrupt flag
  signal wdtifg_set : std_logic;

  --4.10.       Watchdog interrupt generation
  --4.11.       Watchdog reset generation
  --5.  WATCHDOG TIMER (FPGA IMPLEMENTATION)
  --5.1.        Watchdog clock source selection
  signal clk_src_en : std_logic;

  --5.2.        Watchdog 16 bit counter
  signal wdtcnt_incr : std_logic;

  --5.3.        Interval selection mux
  signal wdtqn : std_logic;

  --5.4.        Watchdog event detection
  --5.5.        Watchdog interrupt flag
  signal wdtifg_clr : std_logic;

  --5.6.        Watchdog interrupt generation   
  --5.7.        Watchdog reset generation

begin
  REGISTER_DECODER : block
  begin
    --1.1.      Local register selection
    reg_sel_w  <= per_en and to_stdlogic(per_addr(13 downto DEC_WD_W-1) = BASE_ADDR_W(14 downto DEC_WD_W));
    --1.2.      Register local address
    reg_addr_w <= (per_addr(DEC_WD_W - 2 downto 0) & '0');

    --1.3.      Register address decode
    reg_dec_w   <= WDTCTLC_D and (0 to DEC_SZ_W - 1 => to_stdlogic(reg_addr_w = WDTCTLB));
    --1.4.      Read/Write probes       
    reg_write_w <= reduce_or(per_we) and reg_sel_w;
    reg_read_w  <= not reduce_or(per_we) and reg_sel_w;

    --1.5.      Read/Write vectors
    reg_wr_w <= reg_dec_w and (0 to DEC_SZ_W - 1 => reg_write_w);
    reg_rd_w <= reg_dec_w and (0 to DEC_SZ_W - 1 => reg_read_w);
  end block REGISTER_DECODER;

  REGISTERS : block
  begin
    --2.1.      WDTCTLC Register
    wdtctl_wr <= reg_wr_w(WDTCTLC);

    clock_gating_1_on : if (CLOCK_GATING = '1') generate
      clock_gate_wdtctl : omsp_clock_gate
        port map (
          gclk        => mclk_wdtctl,
          clk         => mclk,
          enable      => wdtctl_wr,
          scan_enable => scan_enable);
    end generate clock_gating_1_on;

    clock_gating_1_off : if (CLOCK_GATING = '0') generate
      mclk_wdtctl <= mclk;
    end generate clock_gating_1_off;

    process (mclk_wdtctl, puc_rst)
      variable WDTNMIES_MASK : std_logic_vector (7 downto 0);
      variable WDTSSEL_MASK  : std_logic_vector (7 downto 0);
      variable WDTCTL_MASK   : std_logic_vector (7 downto 0);
    begin
      if (NMI_EN = '1') then
        WDTNMIES_MASK := "01000000";
      elsif (NMI_EN = '0') then
        WDTNMIES_MASK := "00000000";
      end if;

      if (ASIC_CLOCKING = '1') then
        if (WATCHDOG_MUX = '1') then
          WDTSSEL_MASK := "00000100";
        elsif (WATCHDOG_MUX = '0') then
          WDTSSEL_MASK := "00000000";
        end if;
      elsif (ASIC_CLOCKING = '0') then
        WDTSSEL_MASK := "00000100";
      end if;

      WDTCTL_MASK := "10010011" or WDTSSEL_MASK or WDTNMIES_MASK;

      if (puc_rst = '1') then
        wdtctl <= X"00";
      elsif (rising_edge(mclk_wdtctl)) then
        if (CLOCK_GATING = '1') then
          wdtctl <= per_din(7 downto 0) and WDTCTL_MASK;
        elsif (wdtctl_wr = '1' and CLOCK_GATING = '0') then
          wdtctl <= per_din(7 downto 0) and WDTCTL_MASK;
        end if;
      end if;
    end process;

    wdtpw_error <= wdtctl_wr and to_stdlogic(per_din(15 downto 8) /= X"5A");
    wdttmsel    <= wdtctl(4);
    wdtnmies    <= wdtctl(6);
  end block REGISTERS;

  DATA_OUTPUT_GENERATION : block
  begin
    data_output_gen_00 : if (NMI_EN = '0' and WATCHDOG_MUX = '0') generate
      data_output_gen_a : if (WATCHDOG_NOMUX_ACLK = '0') generate
        per_dout <= (X"69" & (wdtctl or "00000000")) and (0 to 15 => reg_rd_w(WDTCTLC));
      end generate data_output_gen_a;

      data_output_gen_b : if (WATCHDOG_NOMUX_ACLK = '1') generate
        per_dout <= (X"69" & (wdtctl or "00000100")) and (0 to 15 => reg_rd_w(WDTCTLC));
      end generate data_output_gen_b;
    end generate data_output_gen_00;

    data_output_gen_01 : if (NMI_EN = '0' and WATCHDOG_MUX = '1') generate
      per_dout <= (X"69" & (wdtctl or "00000000")) and (0 to 15 => reg_rd_w(WDTCTLC));
    end generate data_output_gen_01;

    data_output_gen_10 : if (NMI_EN = '1' and WATCHDOG_MUX = '0') generate
      data_output_gen_a : if (WATCHDOG_NOMUX_ACLK = '0') generate
        per_dout <= (X"69" & (wdtctl or "00100000")) and (0 to 15 => reg_rd_w(WDTCTLC));
      end generate data_output_gen_a;

      data_output_gen_b : if (WATCHDOG_NOMUX_ACLK = '1') generate
        per_dout <= (X"69" & (wdtctl or "00100100")) and (0 to 15 => reg_rd_w(WDTCTLC));
      end generate data_output_gen_b;
    end generate data_output_gen_10;

    data_output_gen_11 : if (NMI_EN = '1' and WATCHDOG_MUX = '1') generate
      per_dout <= (X"69" & (wdtctl or "00100000")) and (0 to 15 => reg_rd_w(WDTCTLC));
    end generate data_output_gen_11;
  end block DATA_OUTPUT_GENERATION;

  WATCHDOG_TIMER_ASIC_IMPLEMENTATION : block
  begin
    asic_clocking_on : if (ASIC_CLOCKING = '1') generate

      --4.1.    Watchdog clock source selection
      watchdog_mux_on : if (WATCHDOG_MUX = '1') generate
        clock_mux_watchdog : omsp_clock_mux
          port map (
            clk_out   => wdt_clk,
            clk_in0   => smclk,
            clk_in1   => aclk,
            reset     => puc_rst,
            scan_mode => scan_mode,
            selection => wdtctl(2));
      end generate watchdog_mux_on;

      watchdog_mux_off : if (WATCHDOG_MUX = '0') generate
        watchdog_nomux_aclk_on : if (WATCHDOG_NOMUX_ACLK = '1') generate
          wdt_clk <= aclk;
        end generate watchdog_nomux_aclk_on;

        watchdog_nomux_aclk_off : if (WATCHDOG_NOMUX_ACLK = '0') generate
          wdt_clk <= smclk;
        end generate watchdog_nomux_aclk_off;
      end generate watchdog_mux_off;

      --4.2.    Reset synchronizer for the watchdog local clock domain
      sync_reset_por : omsp_sync_reset
        port map (
          rst_s => wdt_rst_noscan,
          clk   => wdt_clk,
          rst_a => puc_rst);

      scan_mux_wdt_rst : omsp_scan_mux
        port map (
          data_out     => wdt_rst,
          data_in_scan => puc_rst,
          data_in_func => wdt_rst_noscan,
          scan_mode    => scan_mode);

      --4.3.    Watchog counter clear (synchronization)
      wdtcnt_clr_detect <= (wdtctl_wr and per_din(3));

      R1_1c_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          wdtcnt_clr_toggle <= '0';
        elsif (rising_edge(mclk)) then
          if (wdtcnt_clr_detect = '1') then
            wdtcnt_clr_toggle <= not wdtcnt_clr_toggle;
          end if;
        end if;
      end process R1_1c_e;

      sync_cell_wdtcnt_clr : omsp_sync_cell
        port map (
          data_out => wdtcnt_clr_sync,
          data_in  => wdtcnt_clr_toggle,
          clk      => wdt_clk,
          rst      => wdt_rst);

      R_1_e : process (wdt_clk, wdt_rst)
      begin
        if (wdt_rst = '1') then
          wdtcnt_clr_sync_dly <= '0';
        elsif (rising_edge(wdt_clk)) then
          wdtcnt_clr_sync_dly <= wdtcnt_clr_sync;
        end if;
      end process R_1_e;

      wdtcnt_clr <= (wdtcnt_clr_sync xor wdtcnt_clr_sync_dly) or wdtqn_edge;

      --4.4.    Watchog counter increment (synchronization)
      sync_cell_wdtcnt_incr : omsp_sync_cell
        port map (
          data_out => wdtcnt_incr,
          data_in  => wdtctl_dbg,
          clk      => wdt_clk,
          rst      => wdt_rst);

      wdtctl_dbg <= not wdtctl(7) and not dbg_freeze;

      --4.5.    Watchdog 16 bit counter
      wdtcnt_nxt <= std_logic_vector(unsigned(wdtcnt) + "0000000000000001");

      clock_gating_2_on : if (CLOCK_GATING = '1') generate
        wdtcnt_en <= wdtcnt_clr or wdtcnt_incr;

        clock_gate_wdtcnt : omsp_clock_gate
          port map (
            gclk        => wdt_clk_cnt,
            clk         => wdt_clk,
            enable      => wdtcnt_en,
            scan_enable => scan_enable);
      end generate clock_gating_2_on;

      clock_gating_2_off : if (CLOCK_GATING = '0') generate
        wdt_clk_cnt <= wdt_clk;
      end generate clock_gating_2_off;

      R_1c_2i_3ci : process (wdt_clk_cnt, wdt_rst)
      begin
        if (wdt_rst = '1') then
          wdtcnt <= X"0000";
        elsif (rising_edge(wdt_clk_cnt)) then
          if (wdtcnt_clr = '1') then
            wdtcnt <= X"0000";
          elsif (CLOCK_GATING = '1') then
            wdtcnt <= wdtcnt_nxt;
          elsif (wdtcnt_incr = '1' and CLOCK_GATING = '0') then
            wdtcnt <= wdtcnt_nxt;
          end if;
        end if;
      end process R_1c_2i_3ci;

      R_1_s2 : process (wdt_clk_cnt, wdt_rst)
      begin
        if (wdt_rst = '1') then
          wdtisx_s  <= "00";
          wdtisx_ss <= "00";
        elsif (rising_edge(wdt_clk_cnt)) then
          wdtisx_s  <= wdtctl(1 downto 0);
          wdtisx_ss <= wdtisx_s;
        end if;
      end process R_1_s2;

      --4.6.    Interval selection mux
      process(wdtisx_ss, wdtcnt_nxt)
      begin
        case wdtisx_ss is
          when "00"   => wdtqn <= wdtcnt_nxt(15);
          when "01"   => wdtqn <= wdtcnt_nxt(13);
          when "10"   => wdtqn <= wdtcnt_nxt(9);
          when others => wdtqn <= wdtcnt_nxt(6);
        end case;
      end process;

      --4.7.    Watchdog event detection
      wdtqn_edge <= wdtqn and wdtcnt_incr;

      R2_1c_e : process (wdt_clk_cnt, wdt_rst)
      begin
        if (wdt_rst = '1') then
          wdt_evt_toggle <= '0';
        elsif (rising_edge(wdt_clk_cnt)) then
          if (wdtqn_edge = '1') then
            wdt_evt_toggle <= not wdt_evt_toggle;
          end if;
        end if;
      end process R2_1c_e;

      sync_cell_wdt_evt : omsp_sync_cell
        port map (
          data_out => wdt_evt_toggle_sync,
          data_in  => wdt_evt_toggle,
          clk      => mclk,
          rst      => puc_rst);

      R2_1_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          wdt_evt_toggle_sync_dly <= '0';
        elsif (rising_edge(mclk)) then
          wdt_evt_toggle_sync_dly <= wdt_evt_toggle_sync;
        end if;
      end process R2_1_e;

      wdtifg_evt <= (wdt_evt_toggle_sync_dly xor wdt_evt_toggle_sync) or wdtpw_error;

      --4.8.    Watchdog wakeup generation
      R3_1_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          wdtifg_clr_reg <= '1';
        elsif (rising_edge(mclk)) then
          wdtifg_clr_reg <= wdtifg_clr;
        end if;
      end process R3_1_e;

      R4_1_e : process (wdt_clk_cnt, wdt_rst)
      begin
        if (wdt_rst = '1') then
          wdtqn_edge_reg <= '0';
        elsif (rising_edge(wdt_clk_cnt)) then
          wdtqn_edge_reg <= wdtqn_edge;
        end if;
      end process R4_1_e;

      wakeup_cell_wdog : omsp_wakeup_cell
        port map (
          wkup_out   => wdt_wkup_pre,
          scan_clk   => mclk,
          scan_mode  => scan_mode,
          scan_rst   => puc_rst,
          wkup_clear => wdtifg_clr_reg,
          wkup_event => wdtqn_edge_reg);

      R5_1_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          wdt_wkup_en <= '0';
        elsif (rising_edge(mclk)) then
          wdt_wkup_en <= not wdtctl(7) and (not wdttmsel or (wdttmsel and wdtie));
        end if;
      end process R5_1_e;

      and_wdt_wkup : omsp_and_gate
        port map (
          y => wdt_wkup,
          a => wdt_wkup_pre,
          b => wdt_wkup_en);

      --4.9.    Watchdog interrupt flag
      wdtifg_set <= wdtifg_evt or wdtifg_sw_set;
      wdtifg_clr <= (wdtifg_irq_clr and wdttmsel) or wdtifg_sw_clr;

      R_1c_2c_e : process (mclk, por)
      begin
        if (por = '1') then
          wdtifg_omsp <= '0';
        elsif (rising_edge(mclk)) then
          if (wdtifg_set = '1') then
            wdtifg_omsp <= '1';
          elsif (wdtifg_clr = '1') then
            wdtifg_omsp <= '0';
          end if;
        end if;
      end process R_1c_2c_e;

      wdtifg <= wdtifg_omsp;

      --4.10.   Watchdog interrupt generation
      wdt_irq <= wdttmsel and wdtifg_omsp and wdtie;

      --4.11.   Watchdog reset generation
      R6_1_e : process (mclk, por)
      begin
        if (por = '1') then
          wdt_reset <= '0';
        elsif (rising_edge(mclk)) then
          wdt_reset <= wdtpw_error or (wdtifg_set and not wdttmsel);
        end if;
      end process R6_1_e;
    end generate asic_clocking_on;
  end block WATCHDOG_TIMER_ASIC_IMPLEMENTATION;

  WATCHDOG_TIMER_FPGA_IMPLEMENTATION : block
  begin
    asic_clocking_off : if (ASIC_CLOCKING = '0') generate

      --5.1.    Watchdog clock source selection
      clk_src_en <= aclk_en when wdtctl(2) = '1' else smclk_en;

      --5.2.    Watchdog 16 bit counter
      wdtcnt_clr  <= (wdtctl_wr and per_din(3)) or wdtifg_evt;
      wdtcnt_incr <= not wdtctl(7) and clk_src_en and not dbg_freeze;
      wdtcnt_nxt  <= std_logic_vector(unsigned(wdtcnt) + "0000000000000001");

      R_1c_2c : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          wdtcnt <= X"0000";
        elsif (rising_edge(mclk)) then
          if (wdtcnt_clr = '1') then
            wdtcnt <= X"0000";
          elsif (wdtcnt_incr = '1') then
            wdtcnt <= wdtcnt_nxt;
          end if;
        end if;
      end process R_1c_2c;

      --5.3.    Interval selection mux
      process(wdtctl, wdtcnt_nxt)
      begin
        case wdtctl (1 downto 0) is
          when "00"   => wdtqn <= wdtcnt_nxt(15);
          when "01"   => wdtqn <= wdtcnt_nxt(13);
          when "10"   => wdtqn <= wdtcnt_nxt(9);
          when others => wdtqn <= wdtcnt_nxt(6);
        end case;
      end process;

      --5.4.    Watchdog event detection
      wdtifg_evt <= (wdtqn and wdtcnt_incr) or wdtpw_error;

      --5.5.    Watchdog interrupt flag
      wdtifg_set <= wdtifg_evt or wdtifg_sw_set;
      wdtifg_clr <= (wdtifg_irq_clr and wdttmsel) or wdtifg_sw_clr;

      R1_1c_2c_e : process (mclk, por)
      begin
        if (por = '1') then
          wdtifg_omsp <= '0';
        elsif (rising_edge(mclk)) then
          if (wdtifg_set = '1') then
            wdtifg_omsp <= '1';
          elsif (wdtifg_clr = '1') then
            wdtifg_omsp <= '0';
          end if;
        end if;
      end process R1_1c_2c_e;

      wdtifg <= wdtifg_omsp;

      --5.6.    Watchdog interrupt generation
      wdt_irq  <= wdttmsel and wdtifg_omsp and wdtie;
      wdt_wkup <= '0';

      --5.7.    Watchdog reset generation               
      R_1_e : process (mclk, por)
      begin
        if (por = '1') then
          wdt_reset <= '0';
        elsif (rising_edge(mclk)) then
          wdt_reset <= wdtpw_error or (wdtifg_set and not wdttmsel);
        end if;
      end process R_1_e;
    end generate asic_clocking_off;
  end block WATCHDOG_TIMER_FPGA_IMPLEMENTATION;
end T_WATCHDOG_ARQ;
