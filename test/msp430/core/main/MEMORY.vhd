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

entity MEMORY is
  port (
    dmem_cen     : out std_logic;
    fe_pmem_wait : out std_logic;
    per_en       : out std_logic;
    pmem_cen     : out std_logic;
    dmem_wen     : out std_logic_vector (1 downto 0);
    per_we       : out std_logic_vector (1 downto 0);
    pmem_wen     : out std_logic_vector (1 downto 0);
    per_addr     : out std_logic_vector (13 downto 0);
    dbg_mem_din  : out std_logic_vector (15 downto 0);
    dmem_din     : out std_logic_vector (15 downto 0);
    eu_mdb_in    : out std_logic_vector (15 downto 0);
    fe_mdb_in    : out std_logic_vector (15 downto 0);
    per_din      : out std_logic_vector (15 downto 0);
    pmem_din     : out std_logic_vector (15 downto 0);
    dmem_addr    : out std_logic_vector (DMEM_MSB downto 0);
    pmem_addr    : out std_logic_vector (PMEM_MSB downto 0);

    dbg_halt_st  : in std_logic;
    dbg_mem_en   : in std_logic;
    eu_mb_en     : in std_logic;
    fe_mb_en     : in std_logic;
    mclk         : in std_logic;
    puc_rst      : in std_logic;
    scan_enable  : in std_logic;
    dbg_mem_wr   : in std_logic_vector (1 downto 0);
    eu_mb_wr     : in std_logic_vector (1 downto 0);
    eu_mab       : in std_logic_vector (14 downto 0);
    fe_mab       : in std_logic_vector (14 downto 0);
    dbg_mem_addr : in std_logic_vector (15 downto 0);
    dbg_mem_dout : in std_logic_vector (15 downto 0);
    dmem_dout    : in std_logic_vector (15 downto 0);
    eu_mdb_out   : in std_logic_vector (15 downto 0);
    per_dout     : in std_logic_vector (15 downto 0);
    pmem_dout    : in std_logic_vector (15 downto 0));
end MEMORY;

architecture MEMORY_ARQ of MEMORY is

  constant PMEM_OFFSET : std_logic_vector (15 downto 0) := std_logic_vector(to_unsigned(65536 - PMEM_SIZE, 16) srl 1);

  -- SIGNAL INOUT
  signal per_en_omsp : std_logic;

  -- 1.RAM INTERFACE
  -- Execution unit access
  signal eu_dmem_cen  : std_logic;
  signal eu_dmem_addr : std_logic_vector (15 downto 0);

  -- Debug interface access
  signal dbg_dmem_cen  : std_logic;
  signal dbg_dmem_addr : std_logic_vector (15 downto 0);

  -- 2.ROM INTERFACE
  -- Execution unit access (only read access are accepted)
  signal eu_pmem_cen  : std_logic;
  signal eu_pmem_addr : std_logic_vector (15 downto 0);

  -- Front-end access
  signal fe_pmem_cen  : std_logic;
  signal fe_pmem_addr : std_logic_vector (15 downto 0);

  -- Debug interface access
  signal dbg_pmem_cen  : std_logic;
  signal dbg_pmem_addr : std_logic_vector (15 downto 0);

  -- 3.PERIPHERALS
  signal dbg_per_en   : std_logic;
  signal eu_per_en    : std_logic;
  signal per_addr_ful : std_logic_vector (14 downto 0);
  signal per_dout_val : std_logic_vector (15 downto 0);
  signal per_addr_mux : std_logic_vector (PER_MSB downto 0);

  -- 4.FRONTEND DATA MUX
  -- Detect whenever the data should be backuped and restored
  signal fe_pmem_cen_dly : std_logic;
  signal fe_pmem_save    : std_logic;
  signal fe_pmem_restore : std_logic;
  signal mclk_bckup      : std_logic;
  signal pmem_dout_bckup : std_logic_vector (15 downto 0);

  -- Mux between the ROM data and the backup
  signal pmem_dout_bckup_sel : std_logic;

  -- 5.EXECUTION - UNIT DATA MUX
  -- Select between peripherals, RAM and ROM     
  signal eu_mdb_in_sel : std_logic_vector (1 downto 0);

  -- 6.DEBUG INTERFACE DATA MUX
  -- Select between peripherals, RAM and ROM
  signal dbg_mem_din_sel : std_logic_vector (1 downto 0);

begin
  M1_RAM_INTERFACE : block
  begin
    -- Execution unit access
    eu_dmem_cen <= not (eu_mb_en and to_stdlogic(eu_mab >= std_logic_vector(to_unsigned(DMEM_BASE, 15) srl 1))
                        and to_stdlogic(eu_mab < std_logic_vector(to_unsigned(DMEM_BASE + DMEM_SIZE, 15) srl 1)));
    eu_dmem_addr <= std_logic_vector(('0' & unsigned(eu_mab)) - (to_unsigned(DMEM_BASE, 15) srl 1));

    -- Debug interface access    
    dbg_dmem_cen <= not (dbg_mem_en and to_stdlogic(dbg_mem_addr(15 downto 1) >= std_logic_vector(to_unsigned(DMEM_BASE, 15) srl 1))
                         and to_stdlogic(dbg_mem_addr(15 downto 1) < std_logic_vector(to_unsigned(DMEM_BASE + DMEM_SIZE, 15) srl 1)));
    dbg_dmem_addr <= std_logic_vector(('0' & unsigned(dbg_mem_addr(15 downto 1))) - (to_unsigned(DMEM_BASE, 15) srl 1));

    -- RAM Interface
    dmem_addr <= dbg_dmem_addr(DMEM_MSB downto 0) when dbg_dmem_cen = '0' else eu_dmem_addr(DMEM_MSB downto 0);
    dmem_cen  <= dbg_dmem_cen and eu_dmem_cen;
    dmem_wen  <= not (dbg_mem_wr or eu_mb_wr);
    dmem_din  <= dbg_mem_dout                     when dbg_dmem_cen = '0' else eu_mdb_out;
  end block M1_RAM_INTERFACE;

  M2_ROM_INTERFACE : block
  begin
    -- Execution unit access (only read access are accepted)     
    eu_pmem_cen  <= not (eu_mb_en and not reduce_or(eu_mb_wr) and to_stdlogic(eu_mab >= PMEM_OFFSET));
    eu_pmem_addr <= std_logic_vector(unsigned(eu_mab) - unsigned(PMEM_OFFSET));

    -- Front-end access
    fe_pmem_cen  <= not (fe_mb_en and to_stdlogic(fe_mab >= PMEM_OFFSET));
    fe_pmem_addr <= std_logic_vector(unsigned(fe_mab) - unsigned(PMEM_OFFSET));

    -- Debug interface access
    dbg_pmem_cen  <= not (dbg_mem_en and to_stdlogic(dbg_mem_addr(15 downto 1) >= PMEM_OFFSET));
    dbg_pmem_addr <= std_logic_vector(('0' & unsigned(dbg_mem_addr(15 downto 1))) - unsigned(PMEM_OFFSET));

    -- ROM Interface (Execution unit has priority)
    pmem_addr <= dbg_pmem_addr(PMEM_MSB downto 0)
                 when dbg_pmem_cen = '0' else
                 eu_pmem_addr(PMEM_MSB downto 0)
                 when (eu_mb_en and not reduce_or(eu_mb_wr) and to_stdlogic(eu_mab >= PMEM_OFFSET)) = '1' else
                 fe_pmem_addr(PMEM_MSB downto 0);
    pmem_cen <= not (fe_mb_en and to_stdlogic(fe_mab >= PMEM_OFFSET)) and
                not (eu_mb_en and not reduce_or(eu_mb_wr) and to_stdlogic(eu_mab >= PMEM_OFFSET)) and dbg_pmem_cen;
    pmem_wen     <= not dbg_mem_wr;
    pmem_din     <= dbg_mem_dout;
    fe_pmem_wait <= (fe_mb_en and to_stdlogic(fe_mab >= PMEM_OFFSET)) and
                    (eu_mb_en and not reduce_or(eu_mb_wr) and to_stdlogic(eu_mab >= PMEM_OFFSET));
  end block M2_ROM_INTERFACE;

  M3_PERIPHERALS : block
  begin
    dbg_per_en   <= dbg_mem_en and to_stdlogic(dbg_mem_addr(15 downto 1) < std_logic_vector(to_unsigned(PER_SIZE, 15) srl 1));
    eu_per_en    <= eu_mb_en and to_stdlogic(eu_mab < std_logic_vector(to_unsigned(PER_SIZE, 15) srl 1));
    per_din      <= dbg_mem_dout                       when dbg_mem_en = '1' else eu_mdb_out;
    per_we       <= dbg_mem_wr                         when dbg_mem_en = '1' else eu_mb_wr;
    per_en_omsp  <= dbg_per_en                         when dbg_mem_en = '1' else eu_per_en;
    per_addr_mux <= dbg_mem_addr(PER_MSB + 1 downto 1) when dbg_mem_en = '1' else eu_mab(PER_MSB downto 0);
    per_addr_ful <= (PER_AWIDTH to 14 => '0') & per_addr_mux;
    per_addr     <= per_addr_ful(13 downto 0);
    per_en       <= per_en_omsp;

    R_1 : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        per_dout_val <= X"0000";
      elsif (rising_edge(mclk)) then
        per_dout_val <= per_dout;
      end if;
    end process R_1;
  end block M3_PERIPHERALS;

  M4_FRONTEND_DATA_MUX : block
  begin
    -- Detect whenever the data should be backuped and restored
    R_1_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        fe_pmem_cen_dly <= '0';
      elsif (rising_edge(mclk)) then
        fe_pmem_cen_dly <= fe_pmem_cen;
      end if;
    end process R_1_e;

    fe_pmem_save    <= (fe_pmem_cen and not fe_pmem_cen_dly) and not dbg_halt_st;
    fe_pmem_restore <= (not fe_pmem_cen and fe_pmem_cen_dly) or dbg_halt_st;

    clock_gating_on : if (CLOCK_GATING = '1') generate
      clock_gate_bckup : omsp_clock_gate
        port map (
          gclk        => mclk_bckup,
          clk         => mclk,
          enable      => fe_pmem_save,
          scan_enable => scan_enable);
    end generate clock_gating_on;

    clock_gating_off : if (CLOCK_GATING = '0') generate
      mclk_bckup <= mclk;
    end generate clock_gating_off;

    R_1i_2ci : process (mclk_bckup, puc_rst)
    begin
      if (puc_rst = '1') then
        pmem_dout_bckup <= X"0000";
      elsif (rising_edge(mclk_bckup)) then
        if (CLOCK_GATING = '1') then
          pmem_dout_bckup <= pmem_dout;
        elsif (fe_pmem_save = '1' and CLOCK_GATING = '0') then
          pmem_dout_bckup <= pmem_dout;
        end if;
      end if;
    end process R_1i_2ci;

    -- Mux between the ROM data and the backup
    R_1c_2c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        pmem_dout_bckup_sel <= '0';
      elsif (rising_edge(mclk)) then
        if (fe_pmem_save = '1') then
          pmem_dout_bckup_sel <= '1';
        elsif (fe_pmem_restore = '1') then
          pmem_dout_bckup_sel <= '0';
        end if;
      end if;
    end process R_1c_2c_e;

    fe_mdb_in <= pmem_dout_bckup when pmem_dout_bckup_sel = '1' else pmem_dout;
  end block M4_FRONTEND_DATA_MUX;

  M5_EXECUTION_UNIT_DATA_MUX : block
  begin
    -- Select between peripherals, RAM and ROM
    R_1 : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        eu_mdb_in_sel <= "00";
      elsif (rising_edge(mclk)) then
        eu_mdb_in_sel <= not eu_pmem_cen & per_en_omsp;
      end if;
    end process R_1;

    -- Mux
    eu_mdb_in <= pmem_dout
                 when eu_mdb_in_sel(1) = '1' else per_dout_val
                 when eu_mdb_in_sel(0) = '1' else dmem_dout;
  end block M5_EXECUTION_UNIT_DATA_MUX;

  M6_DEBUG_INTERFACE_DATA_MUX : block
  begin
    -- Select between peripherals, RAM and ROM
    dbg_en_on : if (DBG_ON = '1') generate
      R_1 : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          dbg_mem_din_sel <= "00";
        elsif (rising_edge(mclk)) then
          dbg_mem_din_sel <= not dbg_pmem_cen & dbg_per_en;
        end if;
      end process R_1;
    end generate dbg_en_on;

    dbg_en_off : if (DBG_ON = '0') generate
      dbg_mem_din_sel <= (others => '0');
    end generate dbg_en_off;

    -- Mux
    dbg_mem_din <= pmem_dout
                   when dbg_mem_din_sel(1) = '1' else per_dout_val
                   when dbg_mem_din_sel(0) = '1' else dmem_dout;
  end block M6_DEBUG_INTERFACE_DATA_MUX;
end MEMORY_ARQ;
