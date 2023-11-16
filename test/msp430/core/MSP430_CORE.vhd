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

entity MSP430_CORE is
  port (
    -- FRONTEND - SCAN
    scan_enable : in std_logic;
    scan_mode   : in std_logic;

    -- FRONTEND - INTERRUPTION
    irq_acc : out std_logic_vector (IRQ_NR - 3 downto 0);
    nmi     : in  std_logic;
    irq     : in  std_logic_vector (IRQ_NR - 3 downto 0);

    -- FRONTEND - RESET
    puc_rst : out std_logic;
    reset_n : in  std_logic;

    -- DATA MEMORY
    dmem_cen  : out std_logic;
    dmem_wen  : out std_logic_vector (1 downto 0);
    dmem_din  : out std_logic_vector (15 downto 0);
    dmem_addr : out std_logic_vector (DMEM_MSB downto 0);
    dmem_dout : in  std_logic_vector (15 downto 0);

    -- INSTRUCTION MEMORY
    pmem_cen  : out std_logic;
    pmem_wen  : out std_logic_vector (1 downto 0);
    pmem_din  : out std_logic_vector (15 downto 0);
    pmem_addr : out std_logic_vector (PMEM_MSB downto 0);
    pmem_dout : in  std_logic_vector (15 downto 0);

    -- PERIPHERAL MEMORY
    per_en   : out std_logic;
    per_we   : out std_logic_vector (1 downto 0);
    per_addr : out std_logic_vector (13 downto 0);
    per_din  : out std_logic_vector (15 downto 0);
    per_dout : in  std_logic_vector (15 downto 0);

    -- EXECUTION - REGISTERS
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

    dbg_clk    : out std_logic;
    dbg_rst    : out std_logic;
    irq_detect : out std_logic;
    nmi_detect : out std_logic;

    i_state : out std_logic_vector (2 downto 0);
    e_state : out std_logic_vector (3 downto 0);
    decode  : out std_logic;
    ir      : out std_logic_vector (15 downto 0);
    irq_num : out std_logic_vector (5 downto 0);
    pc      : out std_logic_vector (15 downto 0);

    nodiv_smclk : out std_logic;

    -- DBG
    dbg_freeze : out std_logic;
    dbg_en     : in  std_logic;

    -- DBG - I2C
    dbg_i2c_sda_out   : out std_logic;
    dbg_i2c_scl       : in  std_logic;
    dbg_i2c_sda_in    : in  std_logic;
    dbg_i2c_addr      : in  std_logic_vector (6 downto 0);
    dbg_i2c_broadcast : in  std_logic_vector (6 downto 0);

    -- DBG - UART
    dbg_uart_txd : out std_logic;
    dbg_uart_rxd : in  std_logic;

    -- BCM
    aclk        : out std_logic;
    aclk_en     : out std_logic;
    dco_enable  : out std_logic;
    dco_wkup    : out std_logic;
    lfxt_enable : out std_logic;
    lfxt_wkup   : out std_logic;
    mclk        : out std_logic;
    smclk       : out std_logic;
    smclk_en    : out std_logic;
    cpu_en      : in  std_logic;
    dco_clk     : in  std_logic;
    lfxt_clk    : in  std_logic;
    wkup        : in  std_logic);
end MSP430_CORE;

architecture MSP430_CORE_ARQ of MSP430_CORE is

  -- SIGNAL INOUT--
  -- FRONTEND - INTERRUPTION
  signal irq_acc_omsp : std_logic_vector (IRQ_NR - 3 downto 0);

  -- FRONTEND - RESET
  signal puc_rst_omsp : std_logic;

  -- PERIPHERAL MEMORY   
  signal per_en_omsp   : std_logic;
  signal per_we_omsp   : std_logic_vector (1 downto 0);
  signal per_addr_omsp : std_logic_vector (13 downto 0);
  signal per_din_omsp  : std_logic_vector (15 downto 0);
  signal per_dout_omsp : std_logic_vector (15 downto 0);

  -- DBG
  signal dbg_freeze_omsp : std_logic;

  -- BCM
  signal aclk_omsp     : std_logic;
  signal aclk_en_omsp  : std_logic;
  signal dbg_clk_omsp  : std_logic;
  signal dbg_rst_omsp  : std_logic;
  signal mclk_omsp     : std_logic;
  signal smclk_omsp    : std_logic;
  signal smclk_en_omsp : std_logic;

  -- OMSP--
  -- FRONTEND
  signal dbg_halt_st  : std_logic;
  signal decode_noirq : std_logic;
  signal exec_done    : std_logic;
  signal fe_mb_en     : std_logic;
  signal inst_bw      : std_logic;
  signal inst_irq_rst : std_logic;
  signal inst_mov     : std_logic;
  signal mclk_enable  : std_logic;
  signal mclk_wkup    : std_logic;
  signal nmi_acc      : std_logic;
  signal inst_type    : std_logic_vector (2 downto 0);
  signal e_state_omsp : std_logic_vector (3 downto 0);
  signal inst_ad      : std_logic_vector (7 downto 0);
  signal inst_as      : std_logic_vector (7 downto 0);
  signal inst_jmp     : std_logic_vector (7 downto 0);
  signal inst_so      : std_logic_vector (7 downto 0);
  signal inst_alu     : std_logic_vector (11 downto 0);
  signal fe_mab       : std_logic_vector (15 downto 0);
  signal inst_dest    : std_logic_vector (15 downto 0);
  signal inst_dext    : std_logic_vector (15 downto 0);
  signal inst_sext    : std_logic_vector (15 downto 0);
  signal inst_src     : std_logic_vector (15 downto 0);
  signal pc_omsp      : std_logic_vector (15 downto 0);
  signal pc_nxt       : std_logic_vector (15 downto 0);

  -- MEMORY
  signal fe_pmem_wait : std_logic;
  signal dbg_mem_din  : std_logic_vector (15 downto 0);
  signal eu_mdb_in    : std_logic_vector (15 downto 0);
  signal fe_mdb_in    : std_logic_vector (15 downto 0);

  -- EXECUTION
  signal cpuoff      : std_logic;
  signal eu_mb_en    : std_logic;
  signal gie         : std_logic;
  signal pc_sw_wr    : std_logic;
  signal oscoff      : std_logic;
  signal scg0        : std_logic;
  signal scg1        : std_logic;
  signal eu_mb_wr    : std_logic_vector (1 downto 0);
  signal dbg_reg_din : std_logic_vector (15 downto 0);
  signal eu_mab      : std_logic_vector (15 downto 0);
  signal eu_mdb_out  : std_logic_vector (15 downto 0);
  signal pc_sw       : std_logic_vector (15 downto 0);

  -- DBG
  signal dbg_cpu_reset : std_logic;
  signal dbg_halt_cmd  : std_logic;
  signal dbg_mem_en    : std_logic;
  signal dbg_reg_wr    : std_logic;
  signal dbg_mem_wr    : std_logic_vector (1 downto 0);
  signal dbg_mem_addr  : std_logic_vector (15 downto 0);
  signal dbg_mem_dout  : std_logic_vector (15 downto 0);

  -- BCM 
  signal cpu_en_s     : std_logic;
  signal dbg_en_s     : std_logic;
  signal por          : std_logic;
  signal puc_pnd_set  : std_logic;
  signal per_dout_clk : std_logic_vector (15 downto 0);

  -- MULTIPLIER
  signal per_dout_mpy : std_logic_vector (15 downto 0);

  -- SFR
  signal nmi_pnd       : std_logic;
  signal nmi_wkup      : std_logic;
  signal wdtie         : std_logic;
  signal wdtifg        : std_logic;
  signal wdtifg_sw_clr : std_logic;
  signal wdtifg_sw_set : std_logic;
  signal wdtnmies      : std_logic;
  signal per_dout_sfr  : std_logic_vector (15 downto 0);
  signal cpu_id        : std_logic_vector (31 downto 0);

  -- T_WATCHDOG  
  signal wdt_irq       : std_logic;
  signal wdt_reset     : std_logic;
  signal wdt_wkup      : std_logic;
  signal per_dout_wdog : std_logic_vector (15 downto 0);

  signal cpu_nr_inst  : std_logic_vector (7 downto 0);
  signal cpu_nr_total : std_logic_vector (7 downto 0);

  signal per_dout_or : std_logic_vector (15 downto 0);

  -- MAIN        
  component FRONTEND
    port (
      dbg_halt_st  : out std_logic;
      decode_noirq : out std_logic;
      exec_done    : out std_logic;
      inst_bw      : out std_logic;
      inst_irq_rst : out std_logic;
      inst_mov     : out std_logic;
      mb_en        : out std_logic;
      mclk_enable  : out std_logic;
      mclk_wkup    : out std_logic;
      nmi_acc      : out std_logic;
      inst_type    : out std_logic_vector (2 downto 0);
      e_state      : out std_logic_vector (3 downto 0);
      inst_ad      : out std_logic_vector (7 downto 0);
      inst_as      : out std_logic_vector (7 downto 0);
      inst_jmp     : out std_logic_vector (7 downto 0);
      inst_so      : out std_logic_vector (7 downto 0);
      inst_alu     : out std_logic_vector (11 downto 0);
      inst_dest    : out std_logic_vector (15 downto 0);
      inst_dext    : out std_logic_vector (15 downto 0);
      inst_sext    : out std_logic_vector (15 downto 0);
      inst_src     : out std_logic_vector (15 downto 0);
      mab          : out std_logic_vector (15 downto 0);
      pc           : out std_logic_vector (15 downto 0);
      pc_nxt       : out std_logic_vector (15 downto 0);
      irq_acc      : out std_logic_vector (IRQ_NR - 3 downto 0);

      decode     : out std_logic;
      irq_detect : out std_logic;
      i_state    : out std_logic_vector (2 downto 0);
      irq_num    : out std_logic_vector (5 downto 0);
      ir         : out std_logic_vector (15 downto 0);

      cpu_en_s     : in std_logic;
      cpuoff       : in std_logic;
      dbg_halt_cmd : in std_logic;
      fe_pmem_wait : in std_logic;
      gie          : in std_logic;
      mclk         : in std_logic;
      nmi_pnd      : in std_logic;
      nmi_wkup     : in std_logic;
      pc_sw_wr     : in std_logic;
      puc_rst      : in std_logic;
      scan_enable  : in std_logic;
      wdt_irq      : in std_logic;
      wdt_wkup     : in std_logic;
      wkup         : in std_logic;
      dbg_reg_sel  : in std_logic_vector (3 downto 0);
      mdb_in       : in std_logic_vector (15 downto 0);
      pc_sw        : in std_logic_vector (15 downto 0);
      irq          : in std_logic_vector (IRQ_NR - 3 downto 0));
  end component FRONTEND;

  component MEMORY
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
  end component MEMORY;

  component EXECUTION
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

      cpuoff      : out std_logic;
      gie         : out std_logic;
      mb_en       : out std_logic;
      oscoff      : out std_logic;
      pc_sw_wr    : out std_logic;
      scg0        : out std_logic;
      scg1        : out std_logic;
      mb_wr       : out std_logic_vector (1 downto 0);
      dbg_reg_din : out std_logic_vector (15 downto 0);
      mab         : out std_logic_vector (15 downto 0);
      mdb_out     : out std_logic_vector (15 downto 0);
      pc_sw       : out std_logic_vector (15 downto 0);

      dbg_halt_st  : in std_logic;
      dbg_reg_wr   : in std_logic;
      exec_done    : in std_logic;
      inst_bw      : in std_logic;
      inst_irq_rst : in std_logic;
      inst_mov     : in std_logic;
      mclk         : in std_logic;
      puc_rst      : in std_logic;
      scan_enable  : in std_logic;
      inst_type    : in std_logic_vector (2 downto 0);
      e_state      : in std_logic_vector (3 downto 0);
      inst_ad      : in std_logic_vector (7 downto 0);
      inst_as      : in std_logic_vector (7 downto 0);
      inst_jmp     : in std_logic_vector (7 downto 0);
      inst_so      : in std_logic_vector (7 downto 0);
      inst_alu     : in std_logic_vector (11 downto 0);
      dbg_mem_dout : in std_logic_vector (15 downto 0);
      inst_dest    : in std_logic_vector (15 downto 0);
      inst_dext    : in std_logic_vector (15 downto 0);
      inst_sext    : in std_logic_vector (15 downto 0);
      inst_src     : in std_logic_vector (15 downto 0);
      mdb_in       : in std_logic_vector (15 downto 0);
      pc           : in std_logic_vector (15 downto 0);
      pc_nxt       : in std_logic_vector (15 downto 0));
  end component EXECUTION;

  component DBG
    port (
      dbg_cpu_reset   : out std_logic;
      dbg_freeze      : out std_logic;
      dbg_halt_cmd    : out std_logic;
      dbg_i2c_sda_out : out std_logic;
      dbg_mem_en      : out std_logic;
      dbg_reg_wr      : out std_logic;
      dbg_uart_txd    : out std_logic;
      dbg_mem_wr      : out std_logic_vector (1 downto 0);
      dbg_mem_addr    : out std_logic_vector (15 downto 0);
      dbg_mem_dout    : out std_logic_vector (15 downto 0);

      cpu_en_s          : in std_logic;
      dbg_clk           : in std_logic;
      dbg_en_s          : in std_logic;
      dbg_halt_st       : in std_logic;
      dbg_i2c_scl       : in std_logic;
      dbg_i2c_sda_in    : in std_logic;
      dbg_rst           : in std_logic;
      dbg_uart_rxd      : in std_logic;
      decode_noirq      : in std_logic;
      eu_mb_en          : in std_logic;
      puc_pnd_set       : in std_logic;
      eu_mb_wr          : in std_logic_vector (1 downto 0);
      dbg_i2c_addr      : in std_logic_vector (6 downto 0);
      dbg_i2c_broadcast : in std_logic_vector (6 downto 0);
      cpu_nr_inst       : in std_logic_vector (7 downto 0);
      cpu_nr_total      : in std_logic_vector (7 downto 0);
      dbg_mem_din       : in std_logic_vector (15 downto 0);
      dbg_reg_din       : in std_logic_vector (15 downto 0);
      eu_mab            : in std_logic_vector (15 downto 0);
      fe_mdb_in         : in std_logic_vector (15 downto 0);
      pc                : in std_logic_vector (15 downto 0);
      cpu_id            : in std_logic_vector (31 downto 0));
  end component DBG;

  -- INTERNAL PERIPHERAL
  component BCM
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
  end component BCM;

  component MULTIPLIER
    port (
      scan_enable : in std_logic;

      per_dout : out std_logic_vector (15 downto 0);
      mclk     : in  std_logic;
      per_en   : in  std_logic;
      puc_rst  : in  std_logic;
      per_we   : in  std_logic_vector (1 downto 0);
      per_addr : in  std_logic_vector (13 downto 0);
      per_din  : in  std_logic_vector (15 downto 0));
  end component MULTIPLIER;

  component SFR
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
  end component SFR;

  component T_WATCHDOG
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
  end component T_WATCHDOG;

begin
  MULTICORE : block
  begin
    cpu_nr_inst  <= std_logic_vector(to_unsigned(INST_NR, 8));
    cpu_nr_total <= std_logic_vector(to_unsigned(TOTAL_NR, 8));
  end block MULTICORE;

  MAIN : block
  begin
    FRONTEND430 : FRONTEND
      port map (
        dbg_halt_st  => dbg_halt_st,
        decode_noirq => decode_noirq,
        exec_done    => exec_done,
        inst_bw      => inst_bw,
        inst_irq_rst => inst_irq_rst,
        inst_mov     => inst_mov,
        mb_en        => fe_mb_en,
        mclk_enable  => mclk_enable,
        mclk_wkup    => mclk_wkup,
        nmi_acc      => nmi_acc,
        inst_type    => inst_type,
        e_state      => e_state_omsp,
        inst_ad      => inst_ad,
        inst_as      => inst_as,
        inst_jmp     => inst_jmp,
        inst_so      => inst_so,
        inst_alu     => inst_alu,
        inst_dest    => inst_dest,
        inst_dext    => inst_dext,
        inst_sext    => inst_sext,
        inst_src     => inst_src,
        mab          => fe_mab,
        pc           => pc_omsp,
        pc_nxt       => pc_nxt,
        irq_acc      => irq_acc_omsp,

        decode     => decode,
        irq_detect => irq_detect,
        i_state    => i_state,
        irq_num    => irq_num,
        ir         => ir,

        cpu_en_s     => cpu_en_s,
        cpuoff       => cpuoff,
        dbg_halt_cmd => dbg_halt_cmd,
        fe_pmem_wait => fe_pmem_wait,
        gie          => gie,
        mclk         => mclk_omsp,
        nmi_pnd      => nmi_pnd,
        nmi_wkup     => nmi_wkup,
        pc_sw_wr     => pc_sw_wr,
        puc_rst      => puc_rst_omsp,
        scan_enable  => scan_enable,
        wdt_irq      => wdt_irq,
        wdt_wkup     => wdt_wkup,
        wkup         => wkup,
        dbg_reg_sel  => dbg_mem_addr (3 downto 0),
        mdb_in       => fe_mdb_in,
        pc_sw        => pc_sw,
        irq          => irq);

    nmi_detect <= nmi_pnd;

    MEMORY430 : MEMORY
      port map (
        dmem_cen     => dmem_cen,
        fe_pmem_wait => fe_pmem_wait,
        per_en       => per_en_omsp,
        pmem_cen     => pmem_cen,
        dmem_wen     => dmem_wen,
        per_we       => per_we_omsp,
        pmem_wen     => pmem_wen,
        per_addr     => per_addr_omsp,
        dbg_mem_din  => dbg_mem_din,
        dmem_din     => dmem_din,
        eu_mdb_in    => eu_mdb_in,
        fe_mdb_in    => fe_mdb_in,
        per_din      => per_din_omsp,
        pmem_din     => pmem_din,
        dmem_addr    => dmem_addr,
        pmem_addr    => pmem_addr,

        dbg_halt_st  => dbg_halt_st,
        dbg_mem_en   => dbg_mem_en,
        eu_mb_en     => eu_mb_en,
        fe_mb_en     => fe_mb_en,
        mclk         => mclk_omsp,
        puc_rst      => puc_rst_omsp,
        scan_enable  => scan_enable,
        dbg_mem_wr   => dbg_mem_wr,
        eu_mb_wr     => eu_mb_wr,
        eu_mab       => eu_mab (15 downto 1),
        fe_mab       => fe_mab (15 downto 1),
        dbg_mem_addr => dbg_mem_addr,
        dbg_mem_dout => dbg_mem_dout,
        dmem_dout    => dmem_dout,
        eu_mdb_out   => eu_mdb_out,
        per_dout     => per_dout_or,
        pmem_dout    => pmem_dout);

    EXECUTION430 : EXECUTION
      port map (
        r0  => r0,
        r1  => r1,
        r2  => r2,
        r3  => r3,
        r4  => r4,
        r5  => r5,
        r6  => r6,
        r7  => r7,
        r8  => r8,
        r9  => r9,
        r10 => r10,
        r11 => r11,
        r12 => r12,
        r13 => r13,
        r14 => r14,
        r15 => r15,

        cpuoff      => cpuoff,
        gie         => gie,
        mb_en       => eu_mb_en,
        oscoff      => oscoff,
        pc_sw_wr    => pc_sw_wr,
        mb_wr       => eu_mb_wr,
        scg0        => scg0,
        scg1        => scg1,
        dbg_reg_din => dbg_reg_din,
        mab         => eu_mab,
        mdb_out     => eu_mdb_out,
        pc_sw       => pc_sw,

        dbg_halt_st  => dbg_halt_st,
        dbg_reg_wr   => dbg_reg_wr,
        exec_done    => exec_done,
        inst_bw      => inst_bw,
        inst_irq_rst => inst_irq_rst,
        inst_mov     => inst_mov,
        mclk         => mclk_omsp,
        puc_rst      => puc_rst_omsp,
        scan_enable  => scan_enable,
        inst_type    => inst_type,
        e_state      => e_state_omsp,
        inst_ad      => inst_ad,
        inst_as      => inst_as,
        inst_jmp     => inst_jmp,
        inst_so      => inst_so,
        inst_alu     => inst_alu,
        dbg_mem_dout => dbg_mem_dout,
        inst_dest    => inst_dest,
        inst_dext    => inst_dext,
        inst_sext    => inst_sext,
        inst_src     => inst_src,
        mdb_in       => eu_mdb_in,
        pc           => pc_omsp,
        pc_nxt       => pc_nxt);

    dbgc_en_on : if (DBG_ON = '1') generate
      DBG430 : DBG
        port map (
          dbg_cpu_reset   => dbg_cpu_reset,
          dbg_freeze      => dbg_freeze_omsp,
          dbg_halt_cmd    => dbg_halt_cmd,
          dbg_i2c_sda_out => dbg_i2c_sda_out,
          dbg_mem_en      => dbg_mem_en,
          dbg_reg_wr      => dbg_reg_wr,
          dbg_uart_txd    => dbg_uart_txd,
          dbg_mem_wr      => dbg_mem_wr,
          dbg_mem_addr    => dbg_mem_addr,
          dbg_mem_dout    => dbg_mem_dout,

          cpu_en_s          => cpu_en_s,
          dbg_clk           => dbg_clk_omsp,
          dbg_en_s          => dbg_en_s,
          dbg_halt_st       => dbg_halt_st,
          dbg_i2c_scl       => dbg_i2c_scl,
          dbg_i2c_sda_in    => dbg_i2c_sda_in,
          dbg_rst           => dbg_rst_omsp,
          dbg_uart_rxd      => dbg_uart_rxd,
          decode_noirq      => decode_noirq,
          eu_mb_en          => eu_mb_en,
          puc_pnd_set       => puc_pnd_set,
          eu_mb_wr          => eu_mb_wr,
          dbg_i2c_addr      => dbg_i2c_addr,
          dbg_i2c_broadcast => dbg_i2c_broadcast,
          cpu_nr_inst       => cpu_nr_inst,
          cpu_nr_total      => cpu_nr_total,
          dbg_mem_din       => dbg_mem_din,
          dbg_reg_din       => dbg_reg_din,
          eu_mab            => eu_mab,
          fe_mdb_in         => fe_mdb_in,
          pc                => pc_omsp,
          cpu_id            => cpu_id);
    end generate dbgc_en_on;

    dbgc_en_off : if (DBG_ON = '0') generate
      dbg_cpu_reset   <= '0';
      dbg_freeze_omsp <= not cpu_en_s;
      dbg_halt_cmd    <= '0';
      dbg_i2c_sda_out <= '1';
      dbg_mem_addr    <= X"0000";
      dbg_mem_dout    <= X"0000";
      dbg_mem_en      <= '0';
      dbg_mem_wr      <= "00";
      dbg_reg_wr      <= '0';
      dbg_uart_txd    <= '1';
    end generate dbgc_en_off;
  end block MAIN;

  INTERNAL_PERIPHERAL : block
  begin
    BCM430 : BCM
      port map (
        aclk        => aclk_omsp,
        aclk_en     => aclk_en_omsp,
        cpu_en_s    => cpu_en_s,
        dbg_clk     => dbg_clk_omsp,
        dbg_en_s    => dbg_en_s,
        dbg_rst     => dbg_rst_omsp,
        dco_enable  => dco_enable,
        dco_wkup    => dco_wkup,
        lfxt_enable => lfxt_enable,
        lfxt_wkup   => lfxt_wkup,
        mclk        => mclk_omsp,
        por         => por,
        puc_pnd_set => puc_pnd_set,
        puc_rst     => puc_rst_omsp,
        smclk       => smclk_omsp,
        smclk_en    => smclk_en_omsp,
        per_dout    => per_dout_clk,
        nodiv_smclk => nodiv_smclk,

        cpu_en        => cpu_en,
        cpuoff        => cpuoff,
        dbg_cpu_reset => dbg_cpu_reset,
        dbg_en        => dbg_en,
        dco_clk       => dco_clk,
        lfxt_clk      => lfxt_clk,
        mclk_enable   => mclk_enable,
        mclk_wkup     => mclk_wkup,
        oscoff        => oscoff,
        per_en        => per_en_omsp,
        reset_n       => reset_n,
        scan_enable   => scan_enable,
        scan_mode     => scan_mode,
        wdt_reset     => wdt_reset,
        scg0          => scg0,
        scg1          => scg1,
        per_we        => per_we_omsp,
        per_addr      => per_addr_omsp,
        per_din       => per_din_omsp);

    multiplier_on : if (MULTIPLYING = '1') generate
      MULTIPLIER430 : MULTIPLIER
        port map (
          per_dout => per_dout_mpy,

          mclk        => mclk_omsp,
          per_en      => per_en_omsp,
          puc_rst     => puc_rst_omsp,
          scan_enable => scan_enable,
          per_we      => per_we_omsp,
          per_addr    => per_addr_omsp,
          per_din     => per_din_omsp);
    end generate multiplier_on;

    multiplier_off : if (MULTIPLYING = '0') generate
      per_dout_mpy <= X"0000";
    end generate multiplier_off;

    SFR430 : SFR
      port map (
        nmi_pnd       => nmi_pnd,
        nmi_wkup      => nmi_wkup,
        wdtie         => wdtie,
        wdtifg_sw_clr => wdtifg_sw_clr,
        wdtifg_sw_set => wdtifg_sw_set,
        per_dout      => per_dout_sfr,
        cpu_id        => cpu_id,

        mclk         => mclk_omsp,
        nmi          => nmi,
        nmi_acc      => nmi_acc,
        per_en       => per_en_omsp,
        puc_rst      => puc_rst_omsp,
        scan_mode    => scan_mode,
        wdtifg       => wdtifg,
        wdtnmies     => wdtnmies,
        per_we       => per_we_omsp,
        cpu_nr_inst  => cpu_nr_inst,
        cpu_nr_total => cpu_nr_total,
        per_addr     => per_addr_omsp,
        per_din      => per_din_omsp);

    t_watchdog_on : if (WATCHDOG = '1') generate
      T_WATCHDOG430 : T_WATCHDOG
        port map (
          wdt_irq   => wdt_irq,
          wdt_reset => wdt_reset,
          wdt_wkup  => wdt_wkup,
          wdtifg    => wdtifg,
          wdtnmies  => wdtnmies,
          per_dout  => per_dout_wdog,

          aclk           => aclk_omsp,
          aclk_en        => aclk_en_omsp,
          dbg_freeze     => dbg_freeze_omsp,
          mclk           => mclk_omsp,
          per_en         => per_en_omsp,
          por            => por,
          puc_rst        => puc_rst_omsp,
          scan_enable    => scan_enable,
          scan_mode      => scan_mode,
          smclk          => smclk_omsp,
          smclk_en       => smclk_en_omsp,
          wdtie          => wdtie,
          wdtifg_irq_clr => irq_acc_omsp (IRQ_NR - 6),
          wdtifg_sw_clr  => wdtifg_sw_clr,
          wdtifg_sw_set  => wdtifg_sw_set,
          per_we         => per_we_omsp,
          per_addr       => per_addr_omsp,
          per_din        => per_din_omsp);
    end generate t_watchdog_on;

    t_watchdog_off : if (WATCHDOG = '0') generate
      per_dout_wdog <= X"0000";
      wdt_irq       <= '0';
      wdt_reset     <= '0';
      wdt_wkup      <= '0';
      wdtifg        <= '0';
      wdtnmies      <= '0';
    end generate t_watchdog_off;
  end block INTERNAL_PERIPHERAL;

  SIGNAL_INOUT : block
  begin
    per_dout_or <= per_dout_omsp or
                   per_dout_clk or
                   per_dout_sfr or
                   per_dout_wdog or
                   per_dout_mpy;

    -- FRONTEND - INTERRUPTION
    irq_acc <= irq_acc_omsp;

    -- FRONTEND - RESET
    puc_rst <= puc_rst_omsp;

    e_state <= e_state_omsp;
    pc      <= pc_omsp;

    -- PERIPHERAL MEMORY 
    per_en        <= per_en_omsp;
    per_we        <= per_we_omsp;
    per_din       <= per_din_omsp;
    per_addr      <= per_addr_omsp;
    per_dout_omsp <= per_dout;

    -- DBG
    dbg_freeze <= dbg_freeze_omsp;

    -- BCM
    aclk     <= aclk_omsp;
    aclk_en  <= aclk_en_omsp;
    dbg_clk  <= dbg_clk_omsp;
    dbg_rst  <= dbg_rst_omsp;
    mclk     <= mclk_omsp;
    smclk    <= smclk_omsp;
    smclk_en <= smclk_en_omsp;
  end block SIGNAL_INOUT;
end MSP430_CORE_ARQ;

