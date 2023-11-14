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

library IEEE;
use IEEE.STD_LOGIC_1164 .all;
use IEEE.NUMERIC_STD .all;
use WORK.MSP430_PACK .all;

entity BCM is
  port (
    aclk          : out std_logic;
    aclk_en       : out std_logic;
    cpu_en_s      : out std_logic;
    dbg_clk       : out std_logic;
    dbg_en_s      : out std_logic;
    dbg_rst       : out std_logic;
    dco_enable    : out std_logic;
    dco_wkup      : out std_logic;
    lfxt_enable   : out std_logic;
    lfxt_wkup     : out std_logic;
    por           : out std_logic;
    puc_pnd_set   : out std_logic;
    smclk         : out std_logic;
    smclk_en      : out std_logic;
    cpu_en        : in  std_logic;
    cpuoff        : in  std_logic;
    dbg_cpu_reset : in  std_logic;
    dbg_en        : in  std_logic;
    dco_clk       : in  std_logic;
    lfxt_clk      : in  std_logic;
    mclk_enable   : in  std_logic;
    mclk_wkup     : in  std_logic;
    oscoff        : in  std_logic;
    reset_n       : in  std_logic;
    scan_enable   : in  std_logic;
    scan_mode     : in  std_logic;
    wdt_reset     : in  std_logic;
    scg0          : in  std_logic;
    scg1          : in  std_logic;

    nodiv_smclk : out std_logic;

    per_dout : out std_logic_vector (15 downto 0);

    mclk     : out std_logic;
    puc_rst  : out std_logic;
    per_en   : in  std_logic;
    per_we   : in  std_logic_vector (1 downto 0);
    per_addr : in  std_logic_vector (13 downto 0);
    per_din  : in  std_logic_vector (15 downto 0));
end BCM;

architecture BCM_ARQ of BCM is

  -- SIGNAL INOUT
  signal cpu_en_s_omsp    : std_logic;
  signal dbg_en_s_omsp    : std_logic;
  signal dbg_rst_omsp     : std_logic;
  signal dco_enable_omsp  : std_logic;
  signal lfxt_enable_omsp : std_logic;
  signal mclk_omsp        : std_logic;
  signal por_omsp         : std_logic;
  signal puc_rst_omsp     : std_logic;

  -- 0.  PARAMETER DECLARATION
  -- 0.1.        Register base address (must be aligned to decoder bit width)
  constant BASE_ADDR_B : std_logic_vector (14 downto 0) := "000000001010000";

  -- 0.2.        Decoder bit width (defines how many bits are considered for address decoding)
  constant DEC_WD_B : integer := 4;

  -- 0.3.        Register addresses offset
  constant BCSCTL1B : std_logic_vector (DEC_WD_B - 1 downto 0) := X"7";
  constant BCSCTL2B : std_logic_vector (DEC_WD_B - 1 downto 0) := X"8";

  constant BCSCTL1C : integer := to_integer(unsigned(BCSCTL1B));
  constant BCSCTL2C : integer := to_integer(unsigned(BCSCTL2B));

  -- 0.4.        Register one-hot decoder utilities
  constant DEC_SZ_B   : integer                                   := 2**DEC_WD_B;
  constant BASE_REG_B : std_logic_vector (DEC_SZ_B - 1 downto 0) := std_logic_vector(to_unsigned(1, DEC_SZ_B));

  -- 0.5.        Register one-hot decoder        
  constant BCSCTL1C_D : std_logic_vector (DEC_SZ_B - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_B) sll BCSCTL1C);
  constant BCSCTL2C_D : std_logic_vector (DEC_SZ_B - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_B) sll BCSCTL2C);

  -- 0.6.        Local wire declarations
  signal nodiv_mclk       : std_logic;
  signal nodiv_mclk_n     : std_logic;
  signal nodiv_smclk_omsp : std_logic;

  -- 1.  REGISTER DECODER
  -- 1.1.        Local register selection
  signal reg_sel_b : std_logic;

  -- 1.2.        Register local address
  signal reg_addr_b : std_logic_vector (DEC_WD_B - 1 downto 0);

  -- 1.3.        Register address decode
  signal reg_dec_b : std_logic_vector (DEC_SZ_B - 1 downto 0);

  -- 1.4.        Read/Write probes
  signal reg_lo_write_b : std_logic;
  signal reg_hi_write_b : std_logic;
  signal reg_read_b     : std_logic;

  -- 1.5.        Read/Write vectors
  signal reg_hi_wr_b : std_logic_vector (DEC_SZ_B - 1 downto 0);
  signal reg_lo_wr_b : std_logic_vector (DEC_SZ_B - 1 downto 0);
  signal reg_rd_b    : std_logic_vector (DEC_SZ_B - 1 downto 0);

  -- 2.  REGISTERS
  -- 2.1.        BCSCTL1C Register
  signal bcsctl_wr  : std_logic_vector (1 downto 0);
  signal divax_mask : std_logic_vector (7 downto 0);
  signal bcsctl     : std_logic_matrix (1 downto 0)(7 downto 0);
  signal bcsctl_nxt : std_logic_matrix (1 downto 0)(7 downto 0);

  -- 2.2.        BCSCTL2C Register
  signal selmx_mask : std_logic_vector (7 downto 0);
  signal divmx_mask : std_logic_vector (7 downto 0);
  signal sels_mask  : std_logic_vector (7 downto 0);
  signal divsx_mask : std_logic_vector (7 downto 0);

  -- 3.  DATA OUTPUT GENERATION
  -- 3.1.        Data output mux
  signal bcsctl_rd : std_logic_matrix (1 downto 0)(15 downto 0);

  -- 4.  DCO_CLK / LFXT_CLK INTERFACES (WAKEUP, ENABLE, ...)
  signal cpuoff_and_mclk_enable : std_logic;

  -- 4.1.        HIGH SPEED SYSTEM CLOCK GENERATOR (DCO_CLK)
  signal por_a                  : std_logic;
  signal cpu_en_wkup            : std_logic;
  signal cpu_enabled_with_dco   : std_logic;
  signal dco_not_enabled_by_dbg : std_logic;
  signal dco_disable_by_scg0    : std_logic;
  signal dco_disable_by_cpu_en  : std_logic;
  signal dco_enable_nxt         : std_logic;
  signal dco_disable            : std_logic;
  signal dco_clk_n              : std_logic;
  signal dco_mclk_wkup          : std_logic;
  signal dco_en_wkup            : std_logic;
  signal dco_wkup_set           : std_logic;
  signal dco_wkup_set_scan      : std_logic;
  signal dco_wkup_clear         : std_logic;
  signal dco_wkup_n             : std_logic;
  signal not_dco_disable        : std_logic;

  -- 4.2.        LOW FREQUENCY CRYSTAL CLOCK GENERATOR (LFXT_CLK)
  --            ASIC MODE
  signal cpu_enabled_with_lfxt   : std_logic;
  signal lfxt_not_enabled_by_dbg : std_logic;
  signal lfxt_disable_by_oscoff  : std_logic;
  signal lfxt_disable_by_cpu_en  : std_logic;
  signal lfxt_enable_nxt         : std_logic;
  signal lfxt_disable            : std_logic;
  signal lfxt_clk_n              : std_logic;
  signal lfxt_mclk_wkup          : std_logic;
  signal lfxt_en_wkup            : std_logic;
  signal lfxt_wkup_set           : std_logic;
  signal lfxt_wkup_set_scan      : std_logic;
  signal lfxt_wkup_clear         : std_logic;
  signal lfxt_wkup_n             : std_logic;
  signal not_lfxt_disable        : std_logic;

  --            FPGA MODE
  signal lfxt_clk_s   : std_logic;
  signal lfxt_clk_dly : std_logic;
  signal lfxt_clk_en  : std_logic;

  -- 5.  CLOCK GENERATION
  -- 5.1.        GLOBAL CPU ENABLE
  --            Synchronize CPU_EN signal to the MCLK domain
  --            Synchronize CPU_EN signal to the ACLK domain
  signal cpu_en_aux_s : std_logic;

  --            Synchronize CPU_EN signal to the SMCLK domain
  signal cpu_en_sm_s : std_logic;

  -- 5.2.        MCLK GENERATION
  --            Clock MUX
  --            Wakeup synchronizer
  signal mclk_wkup_s : std_logic;

  --            Clock Divider
  signal mclk_active     : std_logic;
  signal mclk_div_en     : std_logic;
  signal mclk_div_en_and : std_logic;
  signal mclk_div        : std_logic_vector (2 downto 0);

  --            Generate main system clock
  -- 5.3.        ACLK GENERATION
  --            ASIC MODE
  signal nodiv_aclk        : std_logic;
  signal puc_lfxt_rst      : std_logic;
  signal puc_lfxt_noscan_n : std_logic;
  signal oscoff_s          : std_logic;
  signal aclk_div_en       : std_logic;
  signal aclk_div_en_and   : std_logic;
  signal divax_s           : std_logic_vector (1 downto 0);
  signal divax_ss          : std_logic_vector (1 downto 0);

  --            FPGA MODE
  signal aclk_en_nxt     : std_logic;
  signal aclk_en_nxt_and : std_logic;
  signal aclk_div        : std_logic_vector (2 downto 0);

  -- 5.4.        SMCLK GENERATION
  --            ASIC MODE
  signal puc_sm_noscan_n  : std_logic;
  signal puc_sm_rst       : std_logic;
  signal scg1_s           : std_logic;
  signal smclk_div_en     : std_logic;
  signal smclk_div_en_and : std_logic;
  signal divsx_s          : std_logic_vector (1 downto 0);
  signal divsx_ss         : std_logic_vector (1 downto 0);

  --            FPGA MODE
  signal smclk_in         : std_logic;
  signal smclk_in_and     : std_logic;
  signal smclk_en_nxt     : std_logic;
  signal smclk_en_nxt_and : std_logic;
  signal smclk_div        : std_logic_vector (2 downto 0);

  -- 5.5.        DEBUG INTERFACE CLOCK GENERATION (DBG_CLK)
  --            Synchronize DBG_EN signal to MCLK domain
  signal dbg_en_n_s  : std_logic;
  signal dbg_rst_nxt : std_logic;
  signal not_dbg_en  : std_logic;

  --            Serial Debug Interface Clock gate

  -- 6.  RESET GENERATION
  -- 6.1.        Generate synchronized POR to MCLK domain
  signal por_noscan : std_logic;

  -- 6.2.        Generate synchronized reset for the SDI
  signal dbg_rst_noscan : std_logic;

  -- 6.3.        Generate main system reset (PUC_RST)
  signal puc_noscan_n : std_logic;
  signal puc_a_scan   : std_logic;
  signal puc_a        : std_logic;
  signal puc_s        : std_logic;
  signal not_puc_s    : std_logic;

begin
  REGISTER_DECODER : block
  begin
    -- 1.1.      Local register selection
    reg_sel_b <= per_en and to_stdlogic(per_addr(13 downto DEC_WD_B - 1) = BASE_ADDR_B(14 downto DEC_WD_B));

    -- 1.2.      Register local address
    reg_addr_b <= '0' & per_addr(DEC_WD_B - 2 downto 0);

    -- 1.3.      Register address decode
    reg_dec_b <= (BCSCTL1C_D and (0 to DEC_SZ_B - 1 => to_stdlogic(reg_addr_b = std_logic_vector(unsigned(BCSCTL1B) srl 1)))) or
                 (BCSCTL2C_D and (0 to DEC_SZ_B - 1 => to_stdlogic(reg_addr_b = std_logic_vector(unsigned(BCSCTL2B) srl 1))));

    -- 1.4.      Read/Write probes
    reg_lo_write_b <= per_we(0) and reg_sel_b;
    reg_hi_write_b <= per_we(1) and reg_sel_b;
    reg_read_b     <= not (per_we(0) or per_we(1)) and reg_sel_b;

    -- 1.5.      Read/Write vectors
    reg_hi_wr_b <= reg_dec_b and (0 to DEC_SZ_B - 1 => reg_hi_write_b);
    reg_lo_wr_b <= reg_dec_b and (0 to DEC_SZ_B - 1 => reg_lo_write_b);
    reg_rd_b    <= reg_dec_b and (0 to DEC_SZ_B - 1 => reg_read_b);
  end block REGISTER_DECODER;

  REGISTERS : block
  begin
    -- 2.1.      BCSCTL1C Register
    bcsctl_wr(0)  <= reg_hi_wr_b(BCSCTL1C) when BCSCTL1B(0) = '1' else reg_lo_wr_b(BCSCTL1C);
    bcsctl_nxt(0) <= per_din(15 downto 8)  when BCSCTL1B(0) = '1' else per_din(7 downto 0);

    asic_clocking_1_on : if (ASIC_CLOCKING = '1') generate
      divax_mask <= X"30" when ACLK_DIVIDER = '1' else X"00";
    end generate asic_clocking_1_on;

    asic_clocking_1_off : if (ASIC_CLOCKING = '0') generate
      divax_mask <= X"30";
    end generate asic_clocking_1_off;

    R1_1c : process (mclk_omsp, puc_rst_omsp)
    begin
      if (puc_rst_omsp = '1') then
        bcsctl(0) <= X"00";
      elsif (rising_edge(mclk_omsp)) then
        if (bcsctl_wr(0) = '1') then
          bcsctl(0) <= bcsctl_nxt(0) and divax_mask;
        end if;
      end if;
    end process R1_1c;

    -- 2.2.      BCSCTL2C Register
    bcsctl_wr(1)  <= reg_hi_wr_b(BCSCTL2C) when BCSCTL2B(0) = '1' else reg_lo_wr_b(BCSCTL2C);
    bcsctl_nxt(1) <= per_din(15 downto 8)  when BCSCTL2B(0) = '1' else per_din(7 downto 0);

    selmx_mask <= X"80" when MCLK_MUX = '1'     else X"00";
    divmx_mask <= X"30" when MCLK_DIVIDER = '1' else X"00";

    asic_clocking_2_on : if (ASIC_CLOCKING = '1') generate
      sels_mask  <= X"08" when SMCLK_MUX = '1'     else X"00";
      divsx_mask <= X"06" when SMCLK_DIVIDER = '1' else X"00";
    end generate asic_clocking_2_on;

    asic_clocking_2_off : if (ASIC_CLOCKING = '0') generate
      sels_mask  <= X"08";
      divsx_mask <= X"06";
    end generate asic_clocking_2_off;

    R2_1c : process (mclk_omsp, puc_rst_omsp)
    begin
      if (puc_rst_omsp = '1') then
        bcsctl(1) <= X"00";
      elsif (rising_edge(mclk_omsp)) then
        if (bcsctl_wr(1) = '1') then
          bcsctl(1) <= bcsctl_nxt(1) and (sels_mask or divsx_mask or selmx_mask or divmx_mask);
        end if;
      end if;
    end process R2_1c;
  end block REGISTERS;

  DATA_OUTPUT_GENERATION : block
  begin
    -- 3.1.      Data output mux
    bcsctl_rd(0) <= std_logic_vector((X"00" & (unsigned(bcsctl(0)) and (0 to 7 => reg_rd_b(BCSCTL1C))))
                                      sll to_integer((0 to 3                    => BCSCTL1B(0)) and to_unsigned(8, 4)));
    bcsctl_rd(1) <= std_logic_vector((X"00" & (unsigned(bcsctl(1)) and (0 to 7 => reg_rd_b(BCSCTL2C))))
                                      sll to_integer((0 to 3                    => BCSCTL2B(0)) and to_unsigned(8, 4)));
    per_dout <= bcsctl_rd(0) or bcsctl_rd(1);
  end block DATA_OUTPUT_GENERATION;

  DCO_CLK_LFXT_CLK_INTERFACES : block
  begin
    asic_clocking_1_on : if (ASIC_CLOCKING = '1') generate
      cpuoff_and_mclk_enable <= cpuoff and mclk_enable;
    end generate asic_clocking_1_on;

    -- 4.1.      HIGH SPEED SYSTEM CLOCK GENERATOR (DCO_CLK)
    scg0_en_on : if (SCG_EN_0 = '1') generate
      cpu_enabled_with_dco   <= not bcsctl(1)(SELMX) and cpuoff_and_mclk_enable;
      dco_not_enabled_by_dbg <= not dbg_en_s_omsp and not cpu_enabled_with_dco;
      dco_disable_by_scg0    <= scg0 and dco_not_enabled_by_dbg;
      dco_disable_by_cpu_en  <= not cpu_en_s_omsp and not mclk_enable;
      dco_enable_nxt         <= not dco_disable_by_scg0 and not dco_disable_by_cpu_en;

      R_1_e : process (nodiv_mclk_n, por_omsp)
      begin
        if (por_omsp = '1') then
          dco_disable <= '1';
        elsif (rising_edge(nodiv_mclk_n)) then
          dco_disable <= not dco_enable_nxt;
        end if;
      end process R_1_e;

      dco_clk_n <= not dco_clk;

      mclk_mux_on : if (MCLK_MUX = '1') generate
        sync_cell_dco_disable : omsp_sync_cell
          port map (
            data_out => dco_enable_omsp,
            data_in  => not_dco_disable,
            clk      => dco_clk_n,
            rst      => por_omsp);

        not_dco_disable <= not dco_disable;
      end generate mclk_mux_on;

      mclk_mux_off : if (MCLK_MUX = '0') generate
        dco_enable_omsp <= not dco_disable;
      end generate mclk_mux_off;

      dco_mclk_wkup <= mclk_wkup and not bcsctl(1)(SELMX);
      dco_en_wkup   <= not dco_enable_omsp and dco_enable_nxt;

      dco_wkup_set      <= dco_mclk_wkup or dco_en_wkup or cpu_en_wkup;
      dco_wkup_set_scan <= por_a when scan_mode = '1' else (dco_wkup_set or por_omsp);

      scan_mux_dco_wkup_clear : omsp_scan_mux
        port map (
          data_out     => dco_wkup_clear,
          data_in_scan => dco_wkup_set,
          data_in_func => '1',
          scan_mode    => scan_mode);

      sync_cell_dco_wkup : omsp_sync_cell
        port map (
          data_out => dco_wkup_n,
          data_in  => dco_wkup_clear,
          clk      => dco_clk_n,
          rst      => dco_wkup_set_scan);

      dco_wkup <= not dco_wkup_n and cpu_en;
    end generate scg0_en_on;

    scg0_en_off : if (SCG_EN_0 = '0') generate
      dco_enable_omsp <= '1';
      dco_wkup        <= '1';
    end generate scg0_en_off;

    -- 4.2.      LOW FREQUENCY CRYSTAL CLOCK GENERATOR (LFXT_CLK)        
    --          ASIC MODE
    asic_clocking_2_on : if (ASIC_CLOCKING = '1') generate
      oscoff_en_on : if (OSCOFF_EN = '1') generate
        cpu_enabled_with_lfxt   <= bcsctl(1)(SELMX) and cpuoff_and_mclk_enable;
        lfxt_not_enabled_by_dbg <= not dbg_en_s_omsp and not cpu_enabled_with_lfxt;
        lfxt_disable_by_oscoff  <= oscoff and lfxt_not_enabled_by_dbg;
        lfxt_disable_by_cpu_en  <= not cpu_en_s_omsp and not mclk_enable;
        lfxt_enable_nxt         <= not lfxt_disable_by_oscoff and not lfxt_disable_by_cpu_en;

        R1_2_e : process (nodiv_mclk_n, por_omsp)
        begin
          if (por_omsp = '1') then
            lfxt_disable <= '1';
          elsif (rising_edge(nodiv_mclk_n)) then
            lfxt_disable <= not lfxt_enable_nxt;
          end if;
        end process R1_2_e;

        lfxt_clk_n <= not lfxt_clk;

        sync_cell_lfxt_disable : omsp_sync_cell
          port map (
            data_out => lfxt_enable_omsp,
            data_in  => not_lfxt_disable,
            clk      => lfxt_clk_n,
            rst      => por_omsp);

        lfxt_mclk_wkup     <= mclk_wkup and bcsctl(1)(SELMX);
        lfxt_en_wkup       <= not lfxt_enable_omsp and lfxt_enable_nxt;
        lfxt_wkup_set      <= lfxt_mclk_wkup or lfxt_en_wkup or cpu_en_wkup;
        lfxt_wkup_set_scan <= por_a when scan_mode = '1' else (lfxt_wkup_set or por_omsp);
        not_lfxt_disable   <= not lfxt_disable;

        scan_mux_lfxt_wkup_clear : omsp_scan_mux
          port map (
            data_out     => lfxt_wkup_clear,
            data_in_scan => lfxt_wkup_set,
            data_in_func => '1',
            scan_mode    => scan_mode);

        sync_cell_lfxt_wkup : omsp_sync_cell
          port map (
            data_out => lfxt_wkup_n,
            data_in  => lfxt_wkup_clear,
            clk      => lfxt_clk_n,
            rst      => lfxt_wkup_set_scan);

        lfxt_wkup <= not lfxt_wkup_n and cpu_en;
      end generate oscoff_en_on;

      oscoff_en_off : if (OSCOFF_EN = '0') generate
        lfxt_enable_omsp <= '1';
        lfxt_wkup        <= '0';
      end generate oscoff_en_off;
    end generate asic_clocking_2_on;

    --          FPGA MODE
    asic_clocking_off : if (ASIC_CLOCKING = '0') generate
      sync_cell_lfxt_clk : omsp_sync_cell
        port map (
          data_out => lfxt_clk_s,
          data_in  => lfxt_clk,
          clk      => mclk_omsp,
          rst      => por_omsp);

      R1_e : process (mclk_omsp, por_omsp)
      begin
        if (por_omsp = '1') then
          lfxt_clk_dly <= '0';
        elsif (rising_edge(mclk_omsp)) then
          lfxt_clk_dly <= lfxt_clk_s;
        end if;
      end process;

      lfxt_clk_en      <= (lfxt_clk_s and not lfxt_clk_dly) and not (oscoff and not bcsctl(1)(SELS));
      lfxt_enable_omsp <= '1';
      lfxt_wkup        <= '0';
    end generate asic_clocking_off;
  end block DCO_CLK_LFXT_CLK_INTERFACES;

  CLOCK_GENERATION : block
  begin
    -- 5.1.      GLOBAL CPU ENABLE
    --          Synchronize CPU_EN signal to the MCLK domain
    sync_cpu_en_on : if (SYNC_CPU_EN = '1') generate
      sync_cell_cpu_en : omsp_sync_cell
        port map (
          data_out => cpu_en_s_omsp,
          data_in  => cpu_en,
          clk      => nodiv_mclk,
          rst      => por_omsp);

      cpu_en_wkup <= cpu_en and not cpu_en_s_omsp;
    end generate sync_cpu_en_on;

    sync_cpu_en_off : if (SYNC_CPU_EN = '0') generate
      cpu_en_s_omsp <= cpu_en;
      cpu_en_wkup   <= '0';
    end generate sync_cpu_en_off;

    --          Synchronize CPU_EN signal to the ACLK domain
    lfxt_domain_on : if (LFXT_DOMAIN = '1') generate
      sync_cell_cpu_aux_en : omsp_sync_cell
        port map (
          data_out => cpu_en_aux_s,
          data_in  => cpu_en,
          clk      => lfxt_clk,
          rst      => por_omsp);
    end generate lfxt_domain_on;

    lfxt_domain_off : if (LFXT_DOMAIN = '0') generate
      cpu_en_aux_s <= cpu_en_s_omsp;
    end generate lfxt_domain_off;

    --          Synchronize CPU_EN signal to the SMCLK domain
    asic_clocking_on : if (ASIC_CLOCKING = '1') generate
      smclk_mux_on : if (SMCLK_MUX = '1') generate
        sync_cell_cpu_sm_en : omsp_sync_cell
          port map (
            data_out => cpu_en_sm_s,
            data_in  => cpu_en,
            clk      => nodiv_smclk_omsp,
            rst      => por_omsp);
      end generate smclk_mux_on;

      smclk_mux_off : if (SMCLK_MUX = '0') generate
        cpu_en_sm_s <= cpu_en_s_omsp;
      end generate smclk_mux_off;
    end generate asic_clocking_on;

    -- 5.2.      MCLK GENERATION
    --          Clock MUX
    mclk_mux_on : if (MCLK_MUX = '1') generate
      clock_mux_mclk : omsp_clock_mux
        port map (
          clk_out   => nodiv_mclk,
          clk_in0   => dco_clk,
          clk_in1   => lfxt_clk,
          reset     => por_omsp,
          scan_mode => scan_mode,
          selection => bcsctl(1)(SELMX));
    end generate mclk_mux_on;

    mclk_mux_off : if (MCLK_MUX = '0') generate
      nodiv_mclk <= dco_clk;
    end generate mclk_mux_off;

    nodiv_mclk_n <= not nodiv_mclk;

    --          Wakeup synchronizer
    cpuoff_en_1_on : if (CPUOFF_EN = '1') generate
      sync_cell_mclk_wkup : omsp_sync_cell
        port map (
          data_out => mclk_wkup_s,
          data_in  => mclk_wkup,
          clk      => nodiv_mclk,
          rst      => puc_rst_omsp);
    end generate cpuoff_en_1_on;

    cpuoff_en_1_off : if (CPUOFF_EN = '0') generate
      mclk_wkup_s <= '0';
    end generate cpuoff_en_1_off;

    --          Clock Divider
    cpuoff_en_2_on : if (CPUOFF_EN = '1') generate
      mclk_active <= mclk_enable or mclk_wkup_s or (dbg_en_s_omsp and cpu_en_s_omsp);
    end generate cpuoff_en_2_on;

    cpuoff_en_2_off : if (CPUOFF_EN = '0') generate
      mclk_active <= '1';
    end generate cpuoff_en_2_off;

    mclk_divider_on : if (MCLK_DIVIDER = '1') generate

      R1_1c : process (nodiv_mclk, puc_rst_omsp)
      begin
        if (puc_rst_omsp = '1') then
          mclk_div <= "000";
        elsif (rising_edge(nodiv_mclk)) then
          if (bcsctl(1)(5 downto 4) /= "00") then
            mclk_div <= std_logic_vector(unsigned(mclk_div) + "001");
          end if;
        end if;
      end process R1_1c;

      mclk_div_en <= mclk_active and mclk_div_en_and;

      mclk_div_en_and <= '1'
                         when bcsctl(1)(5 downto 4) = "00" else mclk_div(0)
                         when bcsctl(1)(5 downto 4) = "01" else mclk_div(0) and mclk_div(1)
                         when bcsctl(1)(5 downto 4) = "10" else mclk_div(0) and mclk_div(1) and mclk_div(2);
    end generate mclk_divider_on;

    mclk_divider_off : if (MCLK_DIVIDER = '0') generate
      mclk_div_en <= mclk_active;
    end generate mclk_divider_off;

    --          Generate main system clock
    mclk_cgate_on : if (MCLK_CGATE = '1') generate
      clock_gate_mclk : omsp_clock_gate
        port map (
          gclk        => mclk_omsp,
          clk         => nodiv_mclk,
          enable      => mclk_div_en,
          scan_enable => scan_enable);
    end generate mclk_cgate_on;

    mclk_cgate_off : if (MCLK_CGATE = '0') generate
      mclk_omsp <= nodiv_mclk;
    end generate mclk_cgate_off;

    -- 5.3.      ACLK GENERATION
    --          ASIC MODE
    asic_on : if (ASIC_CLOCKING = '1') generate
      aclk_divider_on : if (ACLK_DIVIDER = '1') generate
        lfxt_domain_on : if (LFXT_DOMAIN = '1') generate
          nodiv_aclk <= lfxt_clk;

          sync_cell_puc_lfxt : omsp_sync_cell
            port map (
              data_out => puc_lfxt_noscan_n,
              data_in  => '1',
              clk      => nodiv_aclk,
              rst      => puc_rst_omsp);

          puc_lfxt_rst <= por_a when scan_mode = '1' else not puc_lfxt_noscan_n;

          R1_1_s2 : process (nodiv_aclk, puc_lfxt_rst)
          begin
            if (puc_lfxt_rst = '1') then
              divax_s  <= "00";
              divax_ss <= "00";
            elsif (rising_edge(nodiv_aclk)) then
              divax_s  <= bcsctl(0)(5 downto 4);
              divax_ss <= divax_s;
            end if;
          end process R1_1_s2;

          oscoff_en_on : if (OSCOFF_EN = '1') generate
            sync_cell_oscoff : omsp_sync_cell
              port map (
                data_out => oscoff_s,
                data_in  => oscoff,
                clk      => nodiv_aclk,
                rst      => puc_lfxt_rst);
          end generate oscoff_en_on;

          oscoff_en_off : if (OSCOFF_EN = '0') generate
            oscoff_s <= '0';
          end generate oscoff_en_off;
        end generate lfxt_domain_on;

        lfxt_domain_off : if (LFXT_DOMAIN = '0') generate
          puc_lfxt_rst <= puc_rst_omsp;
          nodiv_aclk   <= dco_clk;
          divax_ss     <= bcsctl(0)(5 downto 4);
          oscoff_s     <= oscoff;
        end generate lfxt_domain_off;

        R1_1c : process (nodiv_aclk, puc_lfxt_rst)
        begin
          if (puc_lfxt_rst = '1') then
            aclk_div <= "000";
          elsif (rising_edge(nodiv_aclk)) then
            if (divax_ss /= "00") then
              aclk_div <= std_logic_vector(unsigned(aclk_div) + "001");
            end if;
          end if;
        end process R1_1c;

        aclk_div_en <= cpu_en_aux_s and not oscoff_s and aclk_div_en_and;

        aclk_div_en_and <= '1'
                           when divax_ss = "00" else aclk_div(0)
                           when divax_ss = "01" else aclk_div(0) and aclk_div(1)
                           when divax_ss = "10" else aclk_div(0) and aclk_div(1) and aclk_div(2);

        clock_gate_aclk : omsp_clock_gate
          port map (
            gclk        => aclk,
            clk         => nodiv_aclk,
            enable      => aclk_div_en,
            scan_enable => scan_enable);
      end generate aclk_divider_on;

      aclk_divider_off : if (ACLK_DIVIDER = '0') generate
        lfxt_domain_on : if (LFXT_DOMAIN = '1') generate
          aclk <= lfxt_clk;
        end generate lfxt_domain_on;

        lfxt_domain_off : if (LFXT_DOMAIN = '0') generate
          aclk <= dco_clk;
        end generate lfxt_domain_off;
      end generate aclk_divider_off;

      aclk_en <= '1';
    end generate asic_on;

    --          FPGA MODE
    asic_off : if (ASIC_CLOCKING = '0') generate
      aclk_en_nxt <= lfxt_clk_en and aclk_en_nxt_and;

      aclk_en_nxt_and <= '1'
                         when bcsctl(0)(5 downto 4) = "00" else aclk_div(0)
                         when bcsctl(0)(5 downto 4) = "01" else aclk_div(0) and aclk_div(1)
                         when bcsctl(0)(5 downto 4) = "10" else aclk_div(0) and aclk_div(1) and aclk_div(2);

      R1_1c : process (mclk_omsp, puc_rst_omsp)
      begin
        if (puc_rst_omsp = '1') then
          aclk_div <= "000";
        elsif (rising_edge(mclk_omsp)) then
          if (bcsctl(0)(5 downto 4) /= "00" and lfxt_clk_en = '1') then
            aclk_div <= std_logic_vector(unsigned(aclk_div) + "001");
          end if;
        end if;
      end process R1_1c;

      R1_1 : process (mclk_omsp, puc_rst_omsp)
      begin
        if (puc_rst_omsp = '1') then
          aclk_en <= '0';
        elsif (rising_edge(mclk_omsp)) then
          aclk_en <= aclk_en_nxt and cpu_en_s_omsp;
        end if;
      end process R1_1;

      aclk <= mclk_omsp;
    end generate asic_off;

    -- 5.4.      SMCLK GENERATION
    smclk_mux_on : if (SMCLK_MUX = '1') generate
      clock_mux_smclk : omsp_clock_mux
        port map (
          clk_out   => nodiv_smclk_omsp,
          clk_in0   => dco_clk,
          clk_in1   => lfxt_clk,
          reset     => por_omsp,
          scan_mode => scan_mode,
          selection => bcsctl(1)(SELS));
    end generate smclk_mux_on;

    smclk_mux_off : if (SMCLK_MUX = '0') generate
      nodiv_smclk_omsp <= dco_clk;
    end generate smclk_mux_off;

    --          ASIC MODE
    asic_clocking_1_on : if (ASIC_CLOCKING = '1') generate
      smclk_mux_on : if (SMCLK_MUX = '1') generate
        sync_cell_puc_sm : omsp_sync_cell
          port map (
            data_out => puc_sm_noscan_n,
            data_in  => '1',
            clk      => nodiv_smclk_omsp,
            rst      => puc_rst_omsp);

        puc_sm_rst <= por_a when scan_mode = '1' else not puc_sm_noscan_n;

        scg1_en_on : if (SCG_EN_1 = '1') generate
          sync_cell_scg1 : omsp_sync_cell
            port map (
              data_out => scg1_s,
              data_in  => scg1,
              clk      => nodiv_smclk_omsp,
              rst      => puc_sm_rst);
        end generate scg1_en_on;

        scg1_en_off : if (SCG_EN_1 = '0') generate
          scg1_s <= '0';
        end generate scg1_en_off;

        smclk_divider_on : if (SMCLK_DIVIDER = '1') generate
          R2_1_s2 : process (nodiv_smclk_omsp, puc_sm_rst)
          begin
            if (puc_sm_rst = '1') then
              divsx_s  <= "00";
              divsx_ss <= "00";
            elsif (rising_edge(nodiv_smclk_omsp)) then
              divsx_s  <= bcsctl(1)(2 downto 1);
              divsx_ss <= divsx_s;
            end if;
          end process R2_1_s2;
        end generate smclk_divider_on;
      end generate smclk_mux_on;

      smclk_mux_off : if (SMCLK_MUX = '0') generate
        puc_sm_rst <= puc_rst_omsp;
        divsx_ss   <= bcsctl(1)(2 downto 1);
        scg1_s     <= scg1;
      end generate smclk_mux_off;

      smclk_divider_on : if (SMCLK_DIVIDER = '1') generate
        R1_1c : process (nodiv_smclk_omsp, puc_sm_rst)
        begin
          if (puc_sm_rst = '1') then
            smclk_div <= "000";
          elsif (rising_edge(nodiv_smclk_omsp)) then
            if (divsx_ss /= "00") then
              smclk_div <= std_logic_vector(unsigned(smclk_div) + "001");
            end if;
          end if;
        end process R1_1c;

        smclk_div_en <= cpu_en_sm_s and not scg1_s and smclk_div_en_and;

        smclk_div_en_and <= '1'
                            when divsx_ss = "00" else smclk_div(0)
                            when divsx_ss = "01" else smclk_div(0) and smclk_div(1)
                            when divsx_ss = "10" else smclk_div(0) and smclk_div(1) and smclk_div(2);
      end generate smclk_divider_on;

      smclk_divider_off : if (SMCLK_DIVIDER = '0') generate
        scg1_en_on : if (SCG_EN_1 = '1') generate
          smclk_div_en <= cpu_en_sm_s and not scg1_s;
        end generate scg1_en_on;

        scg1_en_off : if (SCG_EN_1 = '0') generate
          smclk_div_en <= cpu_en_sm_s;
        end generate scg1_en_off;
      end generate smclk_divider_off;

      smclk_cgate_on : if (SMCLK_CGATE = '1') generate
        clock_gate_smclk : omsp_clock_gate
          port map (
            gclk        => smclk,
            clk         => nodiv_smclk_omsp,
            enable      => smclk_div_en,
            scan_enable => scan_enable);
      end generate smclk_cgate_on;

      smclk_cgate_off : if (SMCLK_CGATE = '0') generate
        smclk <= nodiv_smclk_omsp;
      end generate smclk_cgate_off;

      smclk_en <= '1';
    end generate asic_clocking_1_on;

    --          FPGA MODE
    asic_clocking_off : if (ASIC_CLOCKING = '0') generate
      smclk_in <= not scg1 and smclk_in_and;

      smclk_in_and <= lfxt_clk_en
                      when bcsctl(1)(SELS) = '1' else '1';

      smclk_en_nxt <= smclk_in and smclk_en_nxt_and;

      smclk_en_nxt_and <= '1'
                          when bcsctl(1)(2 downto 1) = "00" else smclk_div(0)
                          when bcsctl(1)(2 downto 1) = "01" else smclk_div(0) and smclk_div(1)
                          when bcsctl(1)(2 downto 1) = "10" else smclk_div(0) and smclk_div(1) and smclk_div(2);

      R1_1_e : process (mclk_omsp, puc_rst_omsp)
      begin
        if (puc_rst_omsp = '1') then
          smclk_en <= '0';
        elsif (rising_edge(mclk_omsp)) then
          smclk_en <= smclk_en_nxt and cpu_en_s_omsp;
        end if;
      end process R1_1_e;

      R1_1c : process (mclk_omsp, puc_rst_omsp)
      begin
        if (puc_rst_omsp = '1') then
          smclk_div <= "000";
        elsif (rising_edge(mclk_omsp)) then
          if (bcsctl(1)(2 downto 1) /= "00" and smclk_in = '1') then
            smclk_div <= std_logic_vector(unsigned(smclk_div) + "001");
          end if;
        end if;
      end process R1_1c;

      smclk <= mclk_omsp;
    end generate asic_clocking_off;

    -- 5.5.      DEBUG INTERFACE CLOCK GENERATION (DBG_CLK)
    --          Synchronize DBG_EN signal to MCLK domain
    dbg_en_1_on : if (DBG_ON = '1') generate
      sync_dbg_en_1_on : if (SYNC_DBG_EN = '1') generate
        sync_cell_dbg_en : omsp_sync_cell
          port map (
            data_out => dbg_en_n_s,
            data_in  => not_dbg_en,
            clk      => mclk_omsp,
            rst      => por_omsp);

        dbg_en_s_omsp <= not dbg_en_n_s;
        dbg_rst_nxt   <= dbg_en_n_s;
        not_dbg_en    <= not dbg_en;
      end generate sync_dbg_en_1_on;

      sync_dbg_en_1_off : if (SYNC_DBG_EN = '0') generate
        dbg_en_s_omsp <= dbg_en;
        dbg_rst_nxt   <= not dbg_en;
      end generate sync_dbg_en_1_off;
    end generate dbg_en_1_on;

    dbg_en_1_off : if (DBG_ON = '0') generate
      dbg_en_s_omsp <= '0';
      dbg_rst_nxt   <= '0';
    end generate dbg_en_1_off;

    --          Serial Debug Interface Clock gate
    dbg_en_2_on : if (DBG_ON = '1') generate
      asic_clocking_4_on : if (ASIC_CLOCKING = '1') generate
        clock_gate_dbg_clk : omsp_clock_gate
          port map (
            gclk        => dbg_clk,
            clk         => mclk_omsp,
            enable      => dbg_en_s_omsp,
            scan_enable => scan_enable);
      end generate asic_clocking_4_on;

      asic_clocking_4_off : if (ASIC_CLOCKING = '0') generate
        dbg_clk <= dco_clk;
      end generate asic_clocking_4_off;
    end generate dbg_en_2_on;

    dbg_en_2_off : if (DBG_ON = '0') generate
      dbg_clk <= '0';
    end generate dbg_en_2_off;
  end block CLOCK_GENERATION;

  RESET_GENERATION : block
  begin
    -- 6.1.      Generate synchronized POR to MCLK domain
    por_a <= not reset_n;

    sync_reset_por : omsp_sync_reset
      port map (
        rst_s => por_noscan,
        clk   => nodiv_mclk,
        rst_a => por_a);

    asic_1_on : if (ASIC = '1') generate
      scan_mux_por : omsp_scan_mux
        port map (
          data_out     => por_omsp,
          data_in_scan => por_a,
          data_in_func => por_noscan,
          scan_mode    => scan_mode);
    end generate asic_1_on;

    asic_1_off : if (ASIC = '0') generate
      por_omsp <= por_noscan;
    end generate asic_1_off;

    -- 6.2.      Generate synchronized reset for the SDI
    dbg_en_on : if (DBG_ON = '1') generate

      R2_3_e : process (mclk_omsp, por_omsp)
      begin
        if (por_omsp = '1') then
          dbg_rst_noscan <= '1';
        elsif (rising_edge(mclk_omsp)) then
          dbg_rst_noscan <= dbg_rst_nxt;
        end if;
      end process R2_3_e;

      asic_2_on : if (ASIC = '1') generate
        scan_mux_dbg_rst : omsp_scan_mux
          port map (
            data_out     => dbg_rst_omsp,
            data_in_scan => por_a,
            data_in_func => dbg_rst_noscan,
            scan_mode    => scan_mode);
      end generate asic_2_on;

      asic_2_off : if (ASIC = '0') generate
        dbg_rst_omsp <= dbg_rst_noscan;
      end generate asic_2_off;
    end generate dbg_en_on;

    dbg_en_off : if (DBG_ON = '0') generate
      dbg_rst_noscan <= '1';
      dbg_rst_omsp   <= '1';
    end generate dbg_en_off;

    -- 6.3.      Generate main system reset (PUC_RST)
    puc_a <= por_omsp or wdt_reset;
    puc_s <= dbg_cpu_reset or (dbg_en_s_omsp and dbg_rst_noscan and not puc_noscan_n);

    asic_2_on : if (ASIC = '1') generate
      scan_mux_puc_rst_a : omsp_scan_mux
        port map (
          data_out     => puc_a_scan,
          data_in_scan => por_a,
          data_in_func => puc_a,
          scan_mode    => scan_mode);
    end generate asic_2_on;

    asic_2_off : if (ASIC = '0') generate
      puc_a_scan <= puc_a;
    end generate asic_2_off;

    sync_cell_puc : omsp_sync_cell
      port map (
        data_out => puc_noscan_n,
        data_in  => not_puc_s,
        clk      => mclk_omsp,
        rst      => puc_a_scan);

    not_puc_s <= not puc_s;

    asic_3_on : if (ASIC = '1') generate
      puc_rst_omsp <= por_a when scan_mode = '1' else not puc_noscan_n;
    end generate asic_3_on;

    asic_3_off : if (ASIC = '0') generate
      puc_rst_omsp <= not puc_noscan_n;
    end generate asic_3_off;

    puc_pnd_set <= not puc_noscan_n;
  end block RESET_GENERATION;

  SIGNAL_INOUT : block
  begin
    cpu_en_s    <= cpu_en_s_omsp;
    dbg_en_s    <= dbg_en_s_omsp;
    dbg_rst     <= dbg_rst_omsp;
    dco_enable  <= dco_enable_omsp;
    lfxt_enable <= lfxt_enable_omsp;
    mclk        <= mclk_omsp;
    por         <= por_omsp;
    puc_rst     <= puc_rst_omsp;
    nodiv_smclk <= nodiv_smclk_omsp;
  end block SIGNAL_INOUT;
end BCM_ARQ;
