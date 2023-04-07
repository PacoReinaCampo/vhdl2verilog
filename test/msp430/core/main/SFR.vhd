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

entity SFR is
  port (
    nmi_pnd       : out std_logic;
    nmi_wkup      : out std_logic;
    wdtie         : out std_logic;
    wdtifg_sw_clr : out std_logic;
    wdtifg_sw_set : out std_logic;
    cpu_id        : out std_logic_vector (31 downto 0);
    nmi           : in  std_logic;
    nmi_acc       : in  std_logic;
    scan_mode     : in  std_logic;
    wdtifg        : in  std_logic;
    wdtnmies      : in  std_logic;
    cpu_nr_inst   : in  std_logic_vector (7 downto 0);
    cpu_nr_total  : in  std_logic_vector (7 downto 0);

    per_dout : out std_logic_vector (15 downto 0);
    mclk     : in  std_logic;
    per_en   : in  std_logic;
    puc_rst  : in  std_logic;
    per_we   : in  std_logic_vector (1 downto 0);
    per_addr : in  std_logic_vector (13 downto 0);
    per_din  : in  std_logic_vector (15 downto 0));
end SFR;

architecture SFR_ARQ of SFR is

  --SIGNAL INOUT
  signal wdtie_omsp : std_logic;
  signal cpu_id_hi  : std_logic_vector (15 downto 0);
  signal cpu_id_lo  : std_logic_vector (15 downto 0);

  --0.  PARAMETER_DECLARATION
  --0.1.        Register base address (must be aligned to decoder bit width)
  constant BASE_ADDR_S : std_logic_vector (14 downto 0) := (others => '0');

  --0.2.                Decoder bit width (defines how many bits are considered for address decoding)
  constant DEC_WD_S : integer := 4;

  --0.3.        Register addresses offset
  constant IE1C       : integer := 0;
  constant IFG1C      : integer := 2;
  constant CPU_ID_LOC : integer := 4;
  constant CPU_ID_HIC : integer := 6;
  constant CPU_NRC    : integer := 8;

  constant IE1B       : std_logic_vector (DEC_WD_S - 1 downto 0) := std_logic_vector(to_unsigned(IE1C, DEC_WD_S));
  constant IFG1B      : std_logic_vector (DEC_WD_S - 1 downto 0) := std_logic_vector(to_unsigned(IFG1C, DEC_WD_S));
  constant CPU_ID_LOB : std_logic_vector (DEC_WD_S - 1 downto 0) := std_logic_vector(to_unsigned(CPU_ID_LOC, DEC_WD_S));
  constant CPU_ID_HIB : std_logic_vector (DEC_WD_S - 1 downto 0) := std_logic_vector(to_unsigned(CPU_ID_HIC, DEC_WD_S));
  constant CPU_NRB    : std_logic_vector (DEC_WD_S - 1 downto 0) := std_logic_vector(to_unsigned(CPU_NRC, DEC_WD_S));

  --0.4.        Register one-hot decoder utilities
  constant DEC_SZ_S   : integer                                   := 2**DEC_WD_S;
  constant BASE_REG_S : std_logic_vector (DEC_SZ_S - 1 downto 0) := std_logic_vector(to_unsigned(1, DEC_SZ_S));

  --0.5.        Register one-hot decoder
  constant IE1_D       : std_logic_vector (DEC_SZ_S - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_S) sll IE1C);
  constant IFG1_D      : std_logic_vector (DEC_SZ_S - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_S) sll IFG1C);
  constant CPU_ID_LO_D : std_logic_vector (DEC_SZ_S - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_S) sll CPU_ID_LOC);
  constant CPU_ID_HI_D : std_logic_vector (DEC_SZ_S - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_S) sll CPU_ID_HIC);
  constant CPU_NRC_D   : std_logic_vector (DEC_SZ_S - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_S) sll CPU_NRC);

  --1.  REGISTER_DECODER
  --1.1.        Local register selection
  signal reg_sel_s : std_logic;

  --1.2.        Register local address
  signal reg_addr_s : std_logic_vector (DEC_WD_S - 1 downto 0);

  --1.3.        Register address decode
  signal reg_dec_s : std_logic_vector (DEC_SZ_S - 1 downto 0);

  --1.4.        Read/Write probes
  signal reg_lo_write_s : std_logic;
  signal reg_hi_write_s : std_logic;
  signal reg_read_s     : std_logic;

  --1.5.        Read/Write vectors
  signal reg_hi_wr_s : std_logic_vector (DEC_SZ_S - 1 downto 0);
  signal reg_lo_wr_s : std_logic_vector (DEC_SZ_S - 1 downto 0);
  signal reg_rd_s    : std_logic_vector (DEC_SZ_S - 1 downto 0);

  --2.  REGISTERS
  --2.1.        IE1 Register
  signal ie1_wr  : std_logic;
  signal nmie    : std_logic;
  signal ie1     : std_logic_vector (7 downto 0);
  signal ie1_nxt : std_logic_vector (7 downto 0);

  --2.2.        IFG1 Register
  signal ifg1_wr  : std_logic;
  signal nmiifg   : std_logic;
  signal nmi_edge : std_logic;

  signal ifg1     : std_logic_vector (7 downto 0);
  signal ifg1_nxt : std_logic_vector (7 downto 0);

  --2.3.        CPU_ID Register (READ ONLY)
  signal cpu_asic       : std_logic;
  signal mpy_info       : std_logic;
  signal cpu_version_s  : std_logic_vector (2 downto 0);
  signal user_version_s : std_logic_vector (4 downto 0);
  signal pmem_size_s    : std_logic_vector (5 downto 0);
  signal per_space      : std_logic_vector (6 downto 0);
  signal dmem_size_s    : std_logic_vector (8 downto 0);

  --2.4.        CPU_NRC Register (READ ONLY)
  signal cpu_nr_s : std_logic_vector (15 downto 0);

  --3.  DATA_OUTPUT_GENERATION
  --3.1.        Data output mux
  signal ie1_rd         : std_logic_vector (15 downto 0);
  signal ifg1_rd        : std_logic_vector (15 downto 0);
  signal cpu_id_lo_rd_s : std_logic_vector (15 downto 0);
  signal cpu_id_hi_rd_s : std_logic_vector (15 downto 0);
  signal cpu_nr_rd_s    : std_logic_vector (15 downto 0);

  --4.  NMI_GENERATION
  --4.1.        Edge selection
  signal nmi_pol : std_logic;

  --4.2.        Pulse capture and synchronization
  signal nmi_capture_rst : std_logic;
  signal nmi_capture     : std_logic;
  signal nmi_s           : std_logic;

  --4.3.        NMI Pending flag
  signal nmi_dly : std_logic;

begin
  REGISTER_DECODER : block
  begin
    --1.1.      Local register selection
    reg_sel_s <= per_en and to_stdlogic(per_addr(13 downto DEC_WD_S - 1) = BASE_ADDR_S(14 downto DEC_WD_S));

    --1.2.      Register local address
    reg_addr_s <= '0' & per_addr(DEC_WD_S - 2 downto 0);

    --1.3.      Register address decode
    reg_dec_s <= (IE1_D and (0 to DEC_SZ_S - 1 => to_stdlogic(reg_addr_s =
                                                            std_logic_vector(unsigned(IE1B) srl 1)))) or
                 (IFG1_D and (0 to DEC_SZ_S - 1 => to_stdlogic(reg_addr_s =
                                                             std_logic_vector(unsigned(IFG1B) srl 1)))) or
                 (CPU_ID_LO_D and (0 to DEC_SZ_S - 1 => to_stdlogic(reg_addr_s =
                                                                  std_logic_vector(unsigned(CPU_ID_LOB) srl 1)))) or
                 (CPU_ID_HI_D and (0 to DEC_SZ_S - 1 => to_stdlogic(reg_addr_s =
                                                                  std_logic_vector(unsigned(CPU_ID_HIB) srl 1)))) or
                 (CPU_NRC_D and (0 to DEC_SZ_S - 1 => to_stdlogic(reg_addr_s =
                                                                std_logic_vector(unsigned(CPU_NRB) srl 1))));

    --1.4.      Read/Write probes
    reg_lo_write_s <= per_we(0) and reg_sel_s;
    reg_hi_write_s <= per_we(1) and reg_sel_s;
    reg_read_s     <= not reduce_or(per_we) and reg_sel_s;

    --1.5.      Read/Write vectors
    reg_hi_wr_s <= reg_dec_s and (0 to DEC_SZ_S - 1 => reg_hi_write_s);
    reg_lo_wr_s <= reg_dec_s and (0 to DEC_SZ_S - 1 => reg_lo_write_s);
    reg_rd_s    <= reg_dec_s and (0 to DEC_SZ_S - 1 => reg_read_s);
  end block REGISTER_DECODER;

  REGISTERS : block
  begin
    --2.1.      IE1 Register
    ie1_wr  <= reg_hi_wr_s(IE1C)    when (IE1B(0) = '1') else reg_lo_wr_s(IE1C);
    ie1_nxt <= per_din(15 downto 8) when (IE1B(0) = '1') else per_din(7 downto 0);

    nmi_en_on : if (NMI_EN = '1') generate
      R_1c_2c_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          nmie <= '0';
        elsif (rising_edge(mclk)) then
          if (nmi_acc = '1') then
            nmie <= '0';
          elsif (ie1_wr = '1') then
            nmie <= ie1_nxt(4);
          end if;
        end if;
      end process R_1c_2c_e;
    end generate nmi_en_on;

    nmi_en_off : if (NMI_EN = '0') generate
      nmie <= '0';
    end generate nmi_en_off;

    watchdog_on : if (WATCHDOG = '1') generate
      R_1c_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          wdtie_omsp <= '0';
        elsif (rising_edge(mclk)) then
          if (ie1_wr = '1') then
            wdtie_omsp <= ie1_nxt(0);
          end if;
        end if;
      end process R_1c_e;
    end generate watchdog_on;

    watchdog_off : if (WATCHDOG = '0') generate
      wdtie_omsp <= '0';
    end generate watchdog_off;

    wdtie <= wdtie_omsp;
    ie1   <= "000" & nmie & "000" & wdtie_omsp;

    --2.2.      IFG1 Register
    ifg1_wr  <= reg_hi_wr_s(IFG1C)   when (IFG1B(0) = '1') else reg_lo_wr_s(IFG1C);
    ifg1_nxt <= per_din(15 downto 8) when (IFG1B(0) = '1') else per_din(7 downto 0);

    nmi_en_1_on : if (NMI_EN = '1') generate
      R_1c_2c_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          nmiifg <= '0';
        elsif (rising_edge(mclk)) then
          if (nmi_edge = '1') then
            nmiifg <= '1';
          elsif (ifg1_wr = '1') then
            nmiifg <= ifg1_nxt(4);
          end if;
        end if;
      end process R_1c_2c_e;
    end generate nmi_en_1_on;

    nmi_en_1_off : if (NMI_EN = '0') generate
      nmiifg <= '0';
    end generate nmi_en_1_off;

    watchdog_1_on : if (WATCHDOG = '1') generate
      wdtifg_sw_clr <= ifg1_wr and not ifg1_nxt(0);
      wdtifg_sw_set <= ifg1_wr and ifg1_nxt(0);
    end generate watchdog_1_on;

    watchdog_1_off : if (WATCHDOG = '0') generate
      wdtifg_sw_clr <= '0';
      wdtifg_sw_set <= '0';
    end generate watchdog_1_off;

    ifg1 <= "000" & nmiifg & "000" & wdtifg;

    --2.3.      CPU_ID Register (READ ONLY)
    cpu_version_s <= CPU_VERSION;

    asic_on : if (ASIC = '1') generate
      cpu_asic <= '1';
    end generate asic_on;

    asic_off : if (ASIC = '0') generate
      cpu_asic <= '0';
    end generate asic_off;

    user_version_s <= USER_VERSION;
    per_space      <= std_logic_vector(to_unsigned(PER_SIZE/512, 7));

    multiplier_on : if (MULTIPLYING = '1') generate
      mpy_info <= '1';
    end generate multiplier_on;

    multiplier_off : if (MULTIPLYING = '0') generate
      mpy_info <= '0';
    end generate multiplier_off;

    dmem_size_s <= std_logic_vector(to_unsigned(DMEM_SIZE/128, 9));
    pmem_size_s <= std_logic_vector(to_unsigned(PMEM_SIZE/1024, 6));
    cpu_id_hi   <= pmem_size_s & dmem_size_s & mpy_info;
    cpu_id_lo   <= per_space & user_version_s & cpu_asic & cpu_version_s;
    cpu_id      <= cpu_id_hi & cpu_id_lo;

    --2.4.      CPU_NRC Register (READ ONLY)
    cpu_nr_s <= cpu_nr_total & cpu_nr_inst;
  end block REGISTERS;

  DATA_OUTPUT_GENERATION : block
  begin
    --3.1.      Data output mux
    ie1_rd <= std_logic_vector((X"00" & (unsigned(ie1) and (0 to 7 => reg_rd_s(IE1C))))
                                sll to_integer((0 to 3              => IE1B(0)) and to_unsigned(8, 4)));
    ifg1_rd <= std_logic_vector((X"00" & (unsigned(ifg1) and (0 to 7 => reg_rd_s(IFG1C))))
                                 sll to_integer((0 to 3               => IFG1B(0)) and to_unsigned(8, 4)));
    cpu_id_lo_rd_s <= cpu_id_lo and (0 to 15 => reg_rd_s(CPU_ID_LOC));
    cpu_id_hi_rd_s <= cpu_id_hi and (0 to 15 => reg_rd_s(CPU_ID_HIC));
    cpu_nr_rd_s    <= cpu_nr_s and (0 to 15  => reg_rd_s(CPU_NRC));
    per_dout       <= ie1_rd or ifg1_rd or cpu_id_lo_rd_s or cpu_id_hi_rd_s or cpu_nr_rd_s;
  end block DATA_OUTPUT_GENERATION;

  NMI_GENERATION : block
  begin
    nmi_en_on : if (NMI_EN = '1') generate
      --4.1.    Edge selection
      nmi_pol <= nmi xor wdtnmies;

      --4.2.    Pulse capture and synchronization
      sync_nmi_on : if (SYNC_NMI = '1') generate
        asic_clocking_on : if (ASIC_CLOCKING = '1') generate
          R_1_e : process (mclk, puc_rst)
          begin
            if (puc_rst = '1') then
              nmi_capture_rst <= '1';
            elsif (rising_edge(mclk)) then
              nmi_capture_rst <= ifg1_wr and not ifg1_nxt(4);
            end if;
          end process R_1_e;

          wakeup_cell_nmi : omsp_wakeup_cell
            port map (
              wkup_out   => nmi_capture,
              scan_clk   => mclk,
              scan_mode  => scan_mode,
              scan_rst   => puc_rst,
              wkup_clear => nmi_capture_rst,
              wkup_event => nmi_pol);
        end generate asic_clocking_on;

        asic_clocking_off : if (ASIC_CLOCKING = '0') generate
          nmi_capture <= nmi_pol;
        end generate asic_clocking_off;

        sync_cell_nmi : omsp_sync_cell
          port map (
            data_out => nmi_s,
            data_in  => nmi_capture,
            clk      => mclk,
            rst      => puc_rst);
      end generate sync_nmi_on;

      sync_nmi_off : if (SYNC_NMI = '0') generate
        nmi_capture <= nmi_pol;
        nmi_s       <= nmi_pol;
      end generate sync_nmi_off;

      --4.3.    NMI Pending flag
      R_1_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          nmi_dly <= '0';
        elsif (rising_edge(mclk)) then
          nmi_dly <= nmi_s;
        end if;
      end process R_1_e;

      nmi_edge <= not nmi_dly and nmi_s;
      nmi_pnd  <= nmiifg and nmie;

      asic_clocking_on : if (ASIC_CLOCKING = '1') generate
        nmi_wkup <= (nmi_capture xor nmi_dly) and nmie;
      end generate asic_clocking_on;

      asic_clocking_off : if (ASIC_CLOCKING = '0') generate
        nmi_wkup <= '0';
      end generate asic_clocking_off;
    end generate nmi_en_on;

    nmi_en_off : if (NMI_EN = '0') generate
      nmi_pnd  <= '0';
      nmi_wkup <= '0';
    end generate nmi_en_off;
  end block NMI_GENERATION;
end SFR_ARQ;
