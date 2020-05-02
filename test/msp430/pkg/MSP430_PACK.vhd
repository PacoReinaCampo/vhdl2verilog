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

package MSP430_PACK is

  function reduce_or (reduce_or_in : std_logic_vector) return std_logic;
  function to_stdlogic (input : boolean) return std_logic;

  type std_logic_matrix is array (natural range <>) of std_logic_vector;
  type std_logic_3array is array (natural range <>) of std_logic_matrix;
  type std_logic_4array is array (natural range <>) of std_logic_3array;
  type std_logic_5array is array (natural range <>) of std_logic_4array;
  type std_logic_6array is array (natural range <>) of std_logic_5array;
  type std_logic_7array is array (natural range <>) of std_logic_6array;
  type std_logic_8array is array (natural range <>) of std_logic_7array;
  type std_logic_9array is array (natural range <>) of std_logic_8array;

  type xy_std_logic        is array (natural range <>, natural range <>) of std_logic;
  type xy_std_logic_vector is array (natural range <>, natural range <>) of std_logic_vector;
  type xy_std_logic_matrix is array (natural range <>, natural range <>) of std_logic_matrix;
  type xy_std_logic_3array is array (natural range <>, natural range <>) of std_logic_3array;
  type xy_std_logic_4array is array (natural range <>, natural range <>) of std_logic_4array;
  type xy_std_logic_5array is array (natural range <>, natural range <>) of std_logic_5array;
  type xy_std_logic_6array is array (natural range <>, natural range <>) of std_logic_6array;
  type xy_std_logic_7array is array (natural range <>, natural range <>) of std_logic_7array;
  type xy_std_logic_8array is array (natural range <>, natural range <>) of std_logic_8array;
  type xy_std_logic_9array is array (natural range <>, natural range <>) of std_logic_9array;

  type xyz_std_logic        is array (natural range <>, natural range <>, natural range <>) of std_logic;
  type xyz_std_logic_vector is array (natural range <>, natural range <>, natural range <>) of std_logic_vector;
  type xyz_std_logic_matrix is array (natural range <>, natural range <>, natural range <>) of std_logic_matrix;
  type xyz_std_logic_3array is array (natural range <>, natural range <>, natural range <>) of std_logic_3array;
  type xyz_std_logic_4array is array (natural range <>, natural range <>, natural range <>) of std_logic_4array;
  type xyz_std_logic_5array is array (natural range <>, natural range <>, natural range <>) of std_logic_5array;
  type xyz_std_logic_6array is array (natural range <>, natural range <>, natural range <>) of std_logic_6array;
  type xyz_std_logic_7array is array (natural range <>, natural range <>, natural range <>) of std_logic_7array;
  type xyz_std_logic_8array is array (natural range <>, natural range <>, natural range <>) of std_logic_8array;
  type xyz_std_logic_9array is array (natural range <>, natural range <>, natural range <>) of std_logic_9array;

  constant INST_NR  : integer := 0;
  constant TOTAL_NR : integer := 0;
  constant TOTAL_BP : integer := 4;

  --BASIC SYSTEM CONFIGURATION

  --Program Memory Size
  constant PMEM_SIZE : integer := 16384;

  --Data Memory Size
  constant DMEM_SIZE : integer := 4096;

  --Include/Exclude Hardware Multiplier
  constant MULTIPLYING : std_logic := '1';

  --Include/Exclude Serial Debug interface      
  constant DBG_ON : std_logic := '1';

  --ADVANCED SYSTEM CONFIGURATION (FOR EXPERIENCED USERS)

  --Custom user version number
  constant USER_VERSION : std_logic_vector (4 downto 0) := (others => '0');

  --Include/Exclude Watchdog timer
  constant WATCHDOG : std_logic := '1';

  --Include/Exclude Non-Maskable-Interrupt support
  constant NMI_EN : std_logic := '1';

  --Number of available IRQs
  constant IRQ_16 : std_logic := '1';
  constant IRQ_32 : std_logic := '0';
  constant IRQ_64 : std_logic := '0';

  --Input synchronizers
  constant SYNC_NMI    : std_logic := '1';
  constant SYNC_CPU_EN : std_logic := '0';
  constant SYNC_DBG_EN : std_logic := '0';

  --Peripheral Memory Space
  constant PER_SIZE : integer := 512;

  --Defines the debugger CPU_CTL.RST_BRK_EN reset value
  constant DBG_RST_BRK_EN : std_logic := '0';

  --EXPERT SYSTEM CONFIGURATION (EXPERTS ONLY)

  --Select serial debug interface protocol
  constant DBG_UART : std_logic := '0';
  constant DBG_I2C  : std_logic := '1';

  --Enable the I2C broadcast address
  constant DBG_I2C_BROADCASTC : std_logic := '1';

  --Number of hardware breakpoint/watchpoint units
  constant DBG_HWBRK : std_logic_vector (TOTAL_BP - 1 downto 0) := (others => '1');

  --Enable/Disable the hardware breakpoint RANGE mode
  constant HWBRK_RANGE : std_logic := '1';

  --ASIC version
  constant ASIC : std_logic := '1';

  --ASIC SYSTEM CONFIGURATION (EXPERTS/PROFESSIONALS ONLY)

  --FINE GRAINED CLOCK GATING
  constant CLOCK_GATING : std_logic := '1';

  -- ASIC CLOCKING
  constant ASIC_CLOCKING : std_logic := '1';

  --LFXT CLOCK DOMAIN
  constant LFXT_DOMAIN : std_logic := '1';

  --CLOCK MUXES
  --MCLK: Clock Mux
  constant MCLK_MUX : std_logic := '1';

  --SMCLK: Clock Mux
  constant SMCLK_MUX : std_logic := '1';

  --WATCHDOG: Clock Mux
  constant WATCHDOG_MUX        : std_logic := '1';
  constant WATCHDOG_NOMUX_ACLK : std_logic := '0';

  --CLOCK DIVIDERS
  --MCLK: Clock divider
  constant MCLK_DIVIDER : std_logic := '1';

  --SMCLK: Clock divider (/1/2/4/8)
  constant SMCLK_DIVIDER : std_logic := '1';

  --ACLK: Clock divider (/1/2/4/8)
  constant ACLK_DIVIDER : std_logic := '1';

  --LOW POWER MODES
  --LOW POWER MODE: CPUOFF
  constant CPUOFF_EN : std_logic := '1';

  --LOW POWER MODE: SCG
  constant SCG_EN_0 : std_logic := '1';
  constant SCG_EN_1 : std_logic := '1';

  --LOW POWER MODE: OSCOFF
  constant OSCOFF_EN : std_logic := '1';

  --SYSTEM CONSTANTS (DO NOT EDIT)

  --PROGRAM, DATA & PERIPHERAL MEMORY CONFIGURATION
  --Program Memory Size
  constant PMEM_AWIDTH : integer := 13;

  --Data Memory Size
  constant DMEM_AWIDTH : integer := 11;

  --Peripheral Memory Size
  constant PER_AWIDTH : integer := 8;

  --Data Memory Base Adresses
  constant DMEM_BASE : integer := PER_SIZE;

  --Program & Data Memory most significant address bit (for 16 bit words)
  constant PMEM_MSB : integer := PMEM_AWIDTH - 1;
  constant DMEM_MSB : integer := DMEM_AWIDTH - 1;
  constant PER_MSB  : integer := PER_AWIDTH - 1;

  --Number of available IRQs
  constant IRQ_NR : integer := 16;

  --STATES-REGISTER FIELDS
  --Instructions type
  constant INST_SOC  : integer := 0;
  constant INST_JMPC : integer := 1;
  constant INST_TOC  : integer := 2;

  --Single-operand arithmetic
  constant RRC  : integer := 0;
  constant SWPB : integer := 1;
  constant RRA  : integer := 2;
  constant SXTC : integer := 3;
  constant PUSH : integer := 4;
  constant CALL : integer := 5;
  constant RETI : integer := 6;
  constant IRQX : integer := 7;

  --Conditional jump
  constant JNE : integer := 0;
  constant JEQ : integer := 1;
  constant JNC : integer := 2;
  constant JC  : integer := 3;
  constant JN  : integer := 4;
  constant JGE : integer := 5;
  constant JL  : integer := 6;
  constant JMP : integer := 7;

  --Two-operand arithmetic
  constant MOV  : integer := 0;
  constant ADD  : integer := 1;
  constant ADDC : integer := 2;
  constant SUBC : integer := 3;
  constant SUBB : integer := 4;
  constant CMP  : integer := 5;
  constant DADD : integer := 6;
  constant BITC : integer := 7;
  constant BIC  : integer := 8;
  constant BIS  : integer := 9;
  constant XORX : integer := 10;
  constant ANDX : integer := 11;

  --Addressing modes
  constant DIR     : integer := 0;
  constant IDX     : integer := 1;
  constant INDIR   : integer := 2;
  constant INDIR_I : integer := 3;
  constant SYMB    : integer := 4;
  constant IMM     : integer := 5;
  constant ABSC    : integer := 6;
  constant CONST   : integer := 7;

  --Instruction state machine
  constant I_IRQ_FETCH : std_logic_vector (2 downto 0) := "000";
  constant I_IRQ_DONE  : std_logic_vector (2 downto 0) := "001";
  constant I_DEC       : std_logic_vector (2 downto 0) := "010";
  constant I_EXT1      : std_logic_vector (2 downto 0) := "011";
  constant I_EXT2      : std_logic_vector (2 downto 0) := "100";
  constant I_IDLE      : std_logic_vector (2 downto 0) := "101";

  --Execution state machine     
  constant E_SRC_AD : std_logic_vector (3 downto 0) := X"5";
  constant E_SRC_RD : std_logic_vector (3 downto 0) := X"6";
  constant E_SRC_WR : std_logic_vector (3 downto 0) := X"7";
  constant E_DST_AD : std_logic_vector (3 downto 0) := X"8";
  constant E_DST_RD : std_logic_vector (3 downto 0) := X"9";
  constant E_DST_WR : std_logic_vector (3 downto 0) := X"A";
  constant E_EXEC   : std_logic_vector (3 downto 0) := X"B";
  constant E_JUMP   : std_logic_vector (3 downto 0) := X"C";
  constant E_IDLE   : std_logic_vector (3 downto 0) := X"D";

  constant E_IRQ_0 : std_logic_vector (3 downto 0) := X"2";
  constant E_IRQ_1 : std_logic_vector (3 downto 0) := X"1";
  constant E_IRQ_2 : std_logic_vector (3 downto 0) := X"0";
  constant E_IRQ_3 : std_logic_vector (3 downto 0) := X"3";
  constant E_IRQ_4 : std_logic_vector (3 downto 0) := X"4";

  constant E_IRQ : std_logic_matrix(4 downto 0)(3 downto 0) := (X"4", X"3", X"0", X"1", X"2");

  --ALU control signals
  constant ALU_SRC_INV : integer := 0;
  constant ALU_INC     : integer := 1;
  constant ALU_INC_C   : integer := 2;
  constant ALU_ADD     : integer := 3;
  constant ALU_AND     : integer := 4;
  constant ALU_OR      : integer := 5;
  constant ALU_XOR     : integer := 6;
  constant ALU_DADD    : integer := 7;
  constant ALU_STAT_7  : integer := 8;
  constant ALU_STAT_F  : integer := 9;
  constant ALU_SHIFT   : integer := 10;
  constant EXEC_NO_WR  : integer := 11;

  --Debug interface
  constant DBG_UART_WR : integer := 18;
  constant DBG_UART_BW : integer := 17;

  --Debug interface CPU_CTL register
  constant HALT       : integer := 0;
  constant RUN        : integer := 1;
  constant ISTEP      : integer := 2;
  constant SW_BRK_EN  : integer := 3;
  constant FRZ_BRK_EN : integer := 4;
  constant RST_BRK_EN : integer := 5;
  constant CPU_RST    : integer := 6;

  --Debug interface BRKx_CTL register
  constant BRK_MODE_RD : integer := 0;
  constant BRK_MODE_WR : integer := 1;
  constant BRK_EN      : integer := 2;
  constant BRK_I_EN    : integer := 3;
  constant BRK_RANGE   : integer := 4;

  --Basic clock module: BCSCTL2 Control Register
  constant SELMX : integer := 7;
  constant SELS  : integer := 3;

  -- MCLK Clock gate
  constant MCLK_CGATE : std_logic := '1';

  --SMCLK Clock gate
  constant SMCLK_CGATE : std_logic := '1';

  --DEBUG INTERFACE EXTRA CONFIGURATION
  --Debug interface: CPU version
  constant CPU_VERSION : std_logic_vector (2 downto 0) := "010";

  --Debug interface: Software breakpoint opcode
  constant DBG_SWBRK_OP : std_logic_vector (15 downto 0) := X"4343";

  --Debug UART interface auto data synchronization
  constant DBG_UART_AUTO_SYNC : std_logic := '1';

  --Counter width for the debug interface UART
  constant DBG_UART_XFER_CNT_W : integer := 16;

  --Debug UART interface data rate
  constant DBG_UART_BAUD : integer                                              := 2000000;
  constant DBG_DCO_FREQ  : integer                                              := 20000000;
  constant DBG_UART_CNT  : integer                                              := (DBG_DCO_FREQ / DBG_UART_BAUD) - 1;
  constant DBG_UART_CNTB : std_logic_vector (DBG_UART_XFER_CNT_W - 1 downto 0) := std_logic_vector(to_unsigned(DBG_UART_CNT, DBG_UART_XFER_CNT_W));

  --Debug interface input synchronizer
  constant SYNC_DBG_UART_RXD : std_logic := '1';

  --MULTIPLIER CONFIGURATION
  constant MPY_16X16 : std_logic := '1';

  --COMPONENT
  --omsp
  component omsp_alu
    port (
      alu_stat    : out std_logic_vector (3 downto 0);
      alu_stat_wr : out std_logic_vector (3 downto 0);
      alu_out     : out std_logic_vector (15 downto 0);
      alu_out_add : out std_logic_vector (15 downto 0);

      dbg_halt_st : in std_logic;
      exec_cycle  : in std_logic;
      inst_bw     : in std_logic;
      status      : in std_logic_vector (3 downto 0);
      inst_jmp    : in std_logic_vector (7 downto 0);
      inst_so     : in std_logic_vector (7 downto 0);
      inst_alu    : in std_logic_vector (11 downto 0);
      op_dst      : in std_logic_vector (15 downto 0);
      op_src      : in std_logic_vector (15 downto 0));
  end component omsp_alu;

  component omsp_dbg_hwbrk
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
  end component omsp_dbg_hwbrk;

  component omsp_dbg_i2c
    port (
      dbg_i2c_sda_out : out std_logic;
      dbg_rd          : out std_logic;
      dbg_wr          : out std_logic;
      dbg_addr        : out std_logic_vector (5 downto 0);
      dbg_din         : out std_logic_vector (15 downto 0);

      dbg_clk           : in std_logic;
      dbg_i2c_scl       : in std_logic;
      dbg_i2c_sda_in    : in std_logic;
      dbg_rd_rdy        : in std_logic;
      dbg_rst           : in std_logic;
      mem_burst         : in std_logic;
      mem_burst_end     : in std_logic;
      mem_burst_rd      : in std_logic;
      mem_burst_wr      : in std_logic;
      mem_bw            : in std_logic;
      dbg_i2c_addr      : in std_logic_vector (6 downto 0);
      dbg_i2c_broadcast : in std_logic_vector (6 downto 0);
      dbg_dout          : in std_logic_vector (15 downto 0));
  end component omsp_dbg_i2c;

  component omsp_dbg_uart
    port (
      dbg_uart_txd : out std_logic;
      dbg_rd       : out std_logic;
      dbg_wr       : out std_logic;
      dbg_addr     : out std_logic_vector (5 downto 0);
      dbg_din      : out std_logic_vector (15 downto 0);

      dbg_clk       : in std_logic;
      dbg_rd_rdy    : in std_logic;
      dbg_rst       : in std_logic;
      dbg_uart_rxd  : in std_logic;
      mem_burst     : in std_logic;
      mem_burst_end : in std_logic;
      mem_burst_rd  : in std_logic;
      mem_burst_wr  : in std_logic;
      mem_bw        : in std_logic;
      dbg_dout      : in std_logic_vector (15 downto 0));
  end component omsp_dbg_uart;

  component omsp_interrupt is
    port (
      inst_irq_rst : out std_logic;
      irq_detect   : out std_logic;
      nmi_acc      : out std_logic;
      irq_num      : out std_logic_vector (5 downto 0);
      irq_addr     : out std_logic_vector (15 downto 0);
      irq_acc      : out std_logic_vector (IRQ_NR - 3 downto 0);

      mclk         : in std_logic;
      puc_rst      : in std_logic;
      exec_done    : in std_logic;
      nmi_pnd      : in std_logic;
      dbg_halt_st  : in std_logic;
      gie          : in std_logic;
      scan_enable  : in std_logic;
      wdt_irq      : in std_logic;
      cpu_halt_cmd : in std_logic;
      i_state      : in std_logic_vector (2 downto 0);
      irq          : in std_logic_vector (IRQ_NR - 3 downto 0));
  end component omsp_interrupt;

  component omsp_register_file
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
  end component omsp_register_file;

  component omsp_state_machine is
    port (
      dbg_halt_st  : out std_logic;
      decode_noirq : out std_logic;
      fetch        : out std_logic;
      decode       : out std_logic;
      cpu_halt_cmd : out std_logic;
      i_state      : out std_logic_vector (2 downto 0);
      i_state_nxt  : out std_logic_vector (2 downto 0);

      exec_done    : in std_logic;
      cpu_en_s     : in std_logic;
      cpuoff       : in std_logic;
      irq_detect   : in std_logic;
      dbg_halt_cmd : in std_logic;
      mclk         : in std_logic;
      pc_sw_wr     : in std_logic;
      puc_rst      : in std_logic;
      inst_sz      : in std_logic_vector (1 downto 0);
      inst_sz_nxt  : in std_logic_vector (1 downto 0);
      e_state      : in std_logic_vector (3 downto 0);
      e_state_nxt  : in std_logic_vector (3 downto 0));
  end component omsp_state_machine;

  --fusibles
  component omsp_and_gate
    port (
      y : out std_logic;
      a : in  std_logic;
      b : in  std_logic);
  end component omsp_and_gate;

  component omsp_clock_gate
    port (
      gclk        : out std_logic;
      clk         : in  std_logic;
      enable      : in  std_logic;
      scan_enable : in  std_logic);
  end component omsp_clock_gate;

  component omsp_clock_mux
    port (
      clk_out   : out std_logic;
      clk_in0   : in  std_logic;
      clk_in1   : in  std_logic;
      reset     : in  std_logic;
      scan_mode : in  std_logic;
      selection : in  std_logic);
  end component omsp_clock_mux;

  component omsp_scan_mux
    port (
      data_out     : out std_logic;
      data_in_scan : in  std_logic;
      data_in_func : in  std_logic;
      scan_mode    : in  std_logic);
  end component omsp_scan_mux;

  component omsp_sync_cell
    port (
      data_out : out std_logic;
      data_in  : in  std_logic;
      clk      : in  std_logic;
      rst      : in  std_logic);
  end component omsp_sync_cell;

  component omsp_sync_reset
    port (
      rst_s : out std_logic;
      clk   : in  std_logic;
      rst_a : in  std_logic);
  end component omsp_sync_reset;

  component omsp_wakeup_cell
    port (
      wkup_out   : out std_logic;
      scan_clk   : in  std_logic;
      scan_mode  : in  std_logic;
      scan_rst   : in  std_logic;
      wkup_clear : in  std_logic;
      wkup_event : in  std_logic);
  end component omsp_wakeup_cell;

end MSP430_PACK;

package body MSP430_PACK is

  function reduce_or (
    reduce_or_in : std_logic_vector
    ) return std_logic is
    variable reduce_or_out : std_logic := '0';
  begin
    for i in reduce_or_in'range loop
      reduce_or_out := reduce_or_out or reduce_or_in(i);
    end loop;
    return reduce_or_out;
  end reduce_or;

  function to_stdlogic (
    input : boolean
    ) return std_logic is
  begin
    if input then
      return('1');
    else
      return('0');
    end if;
  end function to_stdlogic;

end MSP430_PACK;
