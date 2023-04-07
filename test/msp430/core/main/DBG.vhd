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

entity DBG is
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
end DBG;

architecture DBG_ARQ of DBG is

  type M_TOTAL_BP1_3 is array (TOTAL_BP - 1 downto 0) of std_logic_vector (3 downto 0);
  type M_TOTAL_BP1_15 is array (TOTAL_BP - 1 downto 0) of std_logic_vector (15 downto 0);
  type M_TOTAL_BP1_I is array (TOTAL_BP - 1 downto 0) of natural;

  --SIGNAL INOUT
  signal dbg_mem_en_omsp : std_logic;
  signal dbg_reg_wr_omsp : std_logic;
  signal dbg_mem_wr_omsp : std_logic_vector (1 downto 0);

  --0.          PARAMETER_DECLARATION
  --0.1.                Diverse wires and registers
  signal dbg_wr         : std_logic;
  signal mem_burst      : std_logic;
  signal dbg_reg_rd     : std_logic;
  signal dbg_mem_rd_dly : std_logic;
  signal dbg_swbrk      : std_logic;
  signal dbg_rd         : std_logic;
  signal dbg_rd_rdy     : std_logic;
  signal mem_burst_rd   : std_logic;
  signal mem_burst_wr   : std_logic;
  signal dbg_addr       : std_logic_vector (5 downto 0);
  signal dbg_din        : std_logic_vector (15 downto 0);
  signal brk_halt       : std_logic_vector (TOTAL_BP - 1 downto 0);
  signal brk_pnd        : std_logic_vector (TOTAL_BP - 1 downto 0);
  signal brk_dout       : M_TOTAL_BP1_15;

  --0.2.                Number of registers
  constant NR_REG : integer := 25;

  --0.3.                Register addresses      
  constant CPU_ID_LO : integer := 00;
  constant CPU_ID_HI : integer := 01;
  constant CPU_CTL   : integer := 02;
  constant CPU_STAT  : integer := 03;
  constant MEM_CTL   : integer := 04;
  constant MEM_ADDR  : integer := 05;
  constant MEM_DATA  : integer := 06;
  constant MEM_CNT   : integer := 07;

  constant BRK0_CTL   : integer := 08;
  constant BRK0_STAT  : integer := 09;
  constant BRK0_ADDR0 : integer := 10;
  constant BRK0_ADDR1 : integer := 11;

  constant BRK1_CTL   : integer := 12;
  constant BRK1_STAT  : integer := 13;
  constant BRK1_ADDR0 : integer := 14;
  constant BRK1_ADDR1 : integer := 15;

  constant BRK2_CTL   : integer := 16;
  constant BRK2_STAT  : integer := 17;
  constant BRK2_ADDR0 : integer := 18;
  constant BRK2_ADDR1 : integer := 19;

  constant BRK3_CTL   : integer := 20;
  constant BRK3_STAT  : integer := 21;
  constant BRK3_ADDR0 : integer := 22;
  constant BRK3_ADDR1 : integer := 23;

  constant CPU_NR : integer := 24;

  constant CPU_ID_LOB : std_logic_vector (5 downto 0) := "000000";
  constant CPU_ID_HIB : std_logic_vector (5 downto 0) := "000001";
  constant CPU_CTLB   : std_logic_vector (5 downto 0) := "000010";
  constant CPU_STATB  : std_logic_vector (5 downto 0) := "000011";
  constant MEM_CTLB   : std_logic_vector (5 downto 0) := "000100";
  constant MEM_ADDRB  : std_logic_vector (5 downto 0) := "000101";
  constant MEM_DATAB  : std_logic_vector (5 downto 0) := "000110";
  constant MEM_CNTB   : std_logic_vector (5 downto 0) := "000111";

  constant BRK0_CTLB   : std_logic_vector (5 downto 0) := "001000";
  constant BRK0_STATB  : std_logic_vector (5 downto 0) := "001001";
  constant BRK0_ADDR0B : std_logic_vector (5 downto 0) := "001010";
  constant BRK0_ADDR1B : std_logic_vector (5 downto 0) := "001011";

  constant BRK1_CTLB   : std_logic_vector (5 downto 0) := "001100";
  constant BRK1_STATB  : std_logic_vector (5 downto 0) := "001101";
  constant BRK1_ADDR0B : std_logic_vector (5 downto 0) := "001110";
  constant BRK1_ADDR1B : std_logic_vector (5 downto 0) := "001111";

  constant BRK2_CTLB   : std_logic_vector (5 downto 0) := "010000";
  constant BRK2_STATB  : std_logic_vector (5 downto 0) := "010001";
  constant BRK2_ADDR0B : std_logic_vector (5 downto 0) := "010010";
  constant BRK2_ADDR1B : std_logic_vector (5 downto 0) := "010011";

  constant BRK3_CTLB   : std_logic_vector (5 downto 0) := "010100";
  constant BRK3_STATB  : std_logic_vector (5 downto 0) := "010101";
  constant BRK3_ADDR0B : std_logic_vector (5 downto 0) := "010110";
  constant BRK3_ADDR1B : std_logic_vector (5 downto 0) := "010111";

  constant CPU_NRB : std_logic_vector (5 downto 0) := "011000";

  constant BRK_CTL : M_TOTAL_BP1_I := (BRK3_CTL,
                                       BRK2_CTL,
                                       BRK1_CTL,
                                       BRK0_CTL);

  constant BRK_STAT : M_TOTAL_BP1_I := (BRK3_STAT,
                                        BRK2_STAT,
                                        BRK1_STAT,
                                        BRK0_STAT);

  constant BRK_ADDR0 : M_TOTAL_BP1_I := (BRK3_ADDR0,
                                         BRK2_ADDR0,
                                         BRK1_ADDR0,
                                         BRK0_ADDR0);

  constant BRK_ADDR1 : M_TOTAL_BP1_I := (BRK3_ADDR1,
                                         BRK2_ADDR1,
                                         BRK1_ADDR1,
                                         BRK0_ADDR1);

  --0.4.                Register one-hot decoder
  constant BASE_D : std_logic_vector (NR_REG - 1 downto 0) := (0 => '1', others => '0');

  constant CPU_ID_LO_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) srl CPU_ID_LO);
  constant CPU_ID_HI_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll CPU_ID_HI);
  constant CPU_CTL_D   : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll CPU_CTL);
  constant CPU_STAT_D  : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll CPU_STAT);
  constant MEM_CTL_D   : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll MEM_CTL);
  constant MEM_ADDR_D  : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll MEM_ADDR);
  constant MEM_DATA_D  : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll MEM_DATA);
  constant MEM_CNT_D   : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll MEM_CNT);

  constant BRK0_CTL_D   : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK0_CTL);
  constant BRK0_STAT_D  : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK0_STAT);
  constant BRK0_ADDR0_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK0_ADDR0);
  constant BRK0_ADDR1_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK0_ADDR1);

  constant BRK1_CTL_D   : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK1_CTL);
  constant BRK1_STAT_D  : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK1_STAT);
  constant BRK1_ADDR0_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK1_ADDR0);
  constant BRK1_ADDR1_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK1_ADDR1);

  constant BRK2_CTL_D   : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK2_CTL);
  constant BRK2_STAT_D  : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK2_STAT);
  constant BRK2_ADDR0_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK2_ADDR0);
  constant BRK2_ADDR1_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK2_ADDR1);

  constant BRK3_CTL_D   : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK3_CTL);
  constant BRK3_STAT_D  : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK3_STAT);
  constant BRK3_ADDR0_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK3_ADDR0);
  constant BRK3_ADDR1_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll BRK3_ADDR1);

  constant CPU_NR_D : std_logic_vector (NR_REG - 1 downto 0) := std_logic_vector(unsigned(BASE_D) sll CPU_NR);

  --1.          REGISTER_DECODER
  --1.1.                Select Data register during a burst
  signal dbg_addr_in : std_logic_vector (5 downto 0);

  --1.2.                Register address decode
  signal reg_dec : std_logic_vector (NR_REG - 1 downto 0);

  --1.3.                Read/Write probes
  signal reg_write : std_logic;
  signal reg_read  : std_logic;

  --1.4.                Read/Write vectors
  signal reg_wr : std_logic_vector (NR_REG - 1 downto 0);
  signal reg_rd : std_logic_vector (NR_REG - 1 downto 0);

  --2.          REGISTER_CORE_INTERFACE
  --2.1.                CPU_NR Register
  signal cpu_nr_s : std_logic_vector (15 downto 0);

  --2.2.                CPU_CTL Register
  signal cpu_ctl_wr   : std_logic;
  signal halt_cpu     : std_logic;
  signal run_cpu      : std_logic;
  signal istep_s      : std_logic;
  signal cpu_ctl_s    : std_logic_vector (6 downto 3);
  signal cpu_ctl_full : std_logic_vector (7 downto 0);

  --2.3.                CPU_STAT Register
  signal cpu_stat_wr   : std_logic;
  signal cpu_stat_s    : std_logic_vector (3 downto 2);
  signal cpu_stat_set  : std_logic_vector (3 downto 2);
  signal cpu_stat_clr  : std_logic_vector (3 downto 2);
  signal cpu_stat_full : std_logic_vector (7 downto 0);

  --3.          REGISTER_MEMORY_INTERFACE
  --3.1.                MEM_CTL Register
  signal mem_ctl_wr   : std_logic;
  signal mem_bw       : std_logic;
  signal mem_start    : std_logic;
  signal mem_ctl_s    : std_logic_vector (3 downto 1);
  signal mem_ctl_full : std_logic_vector (7 downto 0);

  --3.2.                MEM_DATA Register
  signal mem_access     : std_logic;
  signal mem_data_wr    : std_logic;
  signal mem_data_s     : std_logic_vector (15 downto 0);
  signal mem_addr_s     : std_logic_vector (15 downto 0);
  signal dbg_mem_din_bw : std_logic_vector (15 downto 0);

  --3.3.                MEM_ADDR Register
  signal mem_addr_wr  : std_logic;
  signal dbg_mem_acc  : std_logic;
  signal dbg_reg_acc  : std_logic;
  signal mem_cnt_s    : std_logic_vector (15 downto 0);
  signal mem_addr_inc : std_logic_vector (15 downto 0);

  --3.4.                MEM_CNT Register
  signal mem_cnt_wr  : std_logic;
  signal mem_cnt_dec : std_logic_vector (15 downto 0);

  --4.          BREAKPOINTS_/_WATCHPOINTS
  --4.1.                Hardware Breakpoint/Watchpoint Register write/read select
  signal brk_reg_rd : M_TOTAL_BP1_3;
  signal brk_reg_wr : M_TOTAL_BP1_3;

  --5.          DATA_OUTPUT_GENERATION
  signal cpu_id_lo_rd : std_logic_vector (15 downto 0);
  signal cpu_id_hi_rd : std_logic_vector (15 downto 0);
  signal cpu_ctl_rd   : std_logic_vector (15 downto 0);
  signal cpu_stat_rd  : std_logic_vector (15 downto 0);
  signal mem_ctl_rd   : std_logic_vector (15 downto 0);
  signal mem_data_rd  : std_logic_vector (15 downto 0);
  signal mem_addr_rd  : std_logic_vector (15 downto 0);
  signal mem_cnt_rd   : std_logic_vector (15 downto 0);
  signal cpu_nr_rd    : std_logic_vector (15 downto 0);
  signal dbg_dout     : std_logic_vector (15 downto 0);

  signal cpu_id_lo_s : std_logic_vector (15 downto 0);
  signal cpu_id_hi_s : std_logic_vector (15 downto 0);

  --5.1.                Tell UART/I2C interface that the data is ready to be read

  --6.          CPU_CONTROL
  --6.1.                Reset CPU       
  --6.2.                Beak after reset
  signal halt_rst : std_logic;

  --6.3.                Freeze peripherals      
  --6.4.                Software break
  signal inc_step : std_logic_vector (1 downto 0);

  --6.5.                Single step
  --6.6.                Run / Halt
  signal halt_flag     : std_logic;
  signal mem_halt_cpu  : std_logic;
  signal mem_run_cpu   : std_logic;
  signal halt_flag_clr : std_logic;
  signal halt_flag_set : std_logic;

  --7.          MEMORY_CONTROL
  --7.1.                Control Memory bursts
  signal mem_burst_start : std_logic;
  signal mem_burst_end   : std_logic;

  --7.1.1.      Detect when burst is on going 
  --7.1.2.      Control signals for UART/I2C interface 
  --7.1.3.      Trigger CPU Register or memory access during a burst
  signal mem_startb    : std_logic;
  signal mem_seq_start : std_logic;

  --7.1.4.      Combine single and burst memory start of sequence 
  --7.2.                Memory access state machine
  signal mem_state     : std_logic_vector (1 downto 0);
  signal mem_state_nxt : std_logic_vector (1 downto 0);

  --7.2.1.      State machine definition
  signal re_m_idle    : std_logic_vector (1 downto 0);
  signal re_m_set_brk : std_logic_vector (1 downto 0);

  constant M_IDLE       : std_logic_vector (1 downto 0) := "00";
  constant M_SET_BRK    : std_logic_vector (1 downto 0) := "01";
  constant M_ACCESS_BRK : std_logic_vector (1 downto 0) := "10";
  constant M_ACCESS     : std_logic_vector (1 downto 0) := "11";

  --7.2.2.      State transition 
  --7.2.3.      State machine 
  --7.2.4.      Utility signals 
  --7.3.                Interface to CPU Registers and Memory bacbkone
  signal dbg_mem_rd       : std_logic;
  signal dbg_mem_wr_msk_s : std_logic_vector (1 downto 0);

  --7.3.1.      It takes one additional cycle to read from Memory as from registers

  function matrix_or (matrix : M_TOTAL_BP1_15) return std_logic_vector is
    variable RESULT : std_logic_vector (15 downto 0) := (others => '0');
  begin
    for i in matrix'range loop
      RESULT := RESULT or matrix(i);
    end loop;
    return RESULT;
  end matrix_or;

begin
  P1_REGISTER_DECODER : block
  begin
    --1.1.              Select Data register during a burst
    dbg_addr_in <= std_logic_vector(to_unsigned(MEM_DATA, 6)) when mem_burst = '1' else dbg_addr;

    --1.2.              Register address decode
    address_decode_01 : if (TOTAL_BP = 1) generate
      process(dbg_addr_in)
      begin
        case dbg_addr_in is
          when CPU_ID_LOB => reg_dec <= CPU_ID_LO_D;
          when CPU_ID_HIB => reg_dec <= CPU_ID_HI_D;
          when CPU_CTLB   => reg_dec <= CPU_CTL_D;
          when CPU_STATB  => reg_dec <= CPU_STAT_D;
          when MEM_CTLB   => reg_dec <= MEM_CTL_D;
          when MEM_ADDRB  => reg_dec <= MEM_ADDR_D;
          when MEM_DATAB  => reg_dec <= MEM_DATA_D;
          when MEM_CNTB   => reg_dec <= MEM_CNT_D;

          when BRK0_CTLB   => reg_dec <= BRK0_CTL_D;
          when BRK0_STATB  => reg_dec <= BRK0_STAT_D;
          when BRK0_ADDR0B => reg_dec <= BRK0_ADDR0_D;
          when BRK0_ADDR1B => reg_dec <= BRK0_ADDR1_D;

          when CPU_NRB => reg_dec <= CPU_NR_D;
          when others  => reg_dec <= (others => '0');
        end case;
      end process;
    end generate address_decode_01;

    address_decode_02 : if (TOTAL_BP = 2) generate
      process(dbg_addr_in)
      begin
        case dbg_addr_in is
          when CPU_ID_LOB => reg_dec <= CPU_ID_LO_D;
          when CPU_ID_HIB => reg_dec <= CPU_ID_HI_D;
          when CPU_CTLB   => reg_dec <= CPU_CTL_D;
          when CPU_STATB  => reg_dec <= CPU_STAT_D;
          when MEM_CTLB   => reg_dec <= MEM_CTL_D;
          when MEM_ADDRB  => reg_dec <= MEM_ADDR_D;
          when MEM_DATAB  => reg_dec <= MEM_DATA_D;
          when MEM_CNTB   => reg_dec <= MEM_CNT_D;

          when BRK0_CTLB   => reg_dec <= BRK0_CTL_D;
          when BRK0_STATB  => reg_dec <= BRK0_STAT_D;
          when BRK0_ADDR0B => reg_dec <= BRK0_ADDR0_D;
          when BRK0_ADDR1B => reg_dec <= BRK0_ADDR1_D;

          when BRK1_CTLB   => reg_dec <= BRK1_CTL_D;
          when BRK1_STATB  => reg_dec <= BRK1_STAT_D;
          when BRK1_ADDR0B => reg_dec <= BRK1_ADDR0_D;
          when BRK1_ADDR1B => reg_dec <= BRK1_ADDR1_D;

          when CPU_NRB => reg_dec <= CPU_NR_D;
          when others  => reg_dec <= (others => '0');
        end case;
      end process;
    end generate address_decode_02;

    address_decode_03 : if (TOTAL_BP = 3) generate
      process(dbg_addr_in)
      begin
        case dbg_addr_in is
          when CPU_ID_LOB => reg_dec <= CPU_ID_LO_D;
          when CPU_ID_HIB => reg_dec <= CPU_ID_HI_D;
          when CPU_CTLB   => reg_dec <= CPU_CTL_D;
          when CPU_STATB  => reg_dec <= CPU_STAT_D;
          when MEM_CTLB   => reg_dec <= MEM_CTL_D;
          when MEM_ADDRB  => reg_dec <= MEM_ADDR_D;
          when MEM_DATAB  => reg_dec <= MEM_DATA_D;
          when MEM_CNTB   => reg_dec <= MEM_CNT_D;

          when BRK0_CTLB   => reg_dec <= BRK0_CTL_D;
          when BRK0_STATB  => reg_dec <= BRK0_STAT_D;
          when BRK0_ADDR0B => reg_dec <= BRK0_ADDR0_D;
          when BRK0_ADDR1B => reg_dec <= BRK0_ADDR1_D;

          when BRK1_CTLB   => reg_dec <= BRK1_CTL_D;
          when BRK1_STATB  => reg_dec <= BRK1_STAT_D;
          when BRK1_ADDR0B => reg_dec <= BRK1_ADDR0_D;
          when BRK1_ADDR1B => reg_dec <= BRK1_ADDR1_D;


          when BRK2_CTLB   => reg_dec <= BRK2_CTL_D;
          when BRK2_STATB  => reg_dec <= BRK2_STAT_D;
          when BRK2_ADDR0B => reg_dec <= BRK2_ADDR0_D;
          when BRK2_ADDR1B => reg_dec <= BRK2_ADDR1_D;

          when CPU_NRB => reg_dec <= CPU_NR_D;
          when others  => reg_dec <= (others => '0');
        end case;
      end process;
    end generate address_decode_03;

    address_decode_04 : if (TOTAL_BP = 4) generate
      process(dbg_addr_in)
      begin
        case dbg_addr_in is
          when CPU_ID_LOB => reg_dec <= CPU_ID_LO_D;
          when CPU_ID_HIB => reg_dec <= CPU_ID_HI_D;
          when CPU_CTLB   => reg_dec <= CPU_CTL_D;
          when CPU_STATB  => reg_dec <= CPU_STAT_D;
          when MEM_CTLB   => reg_dec <= MEM_CTL_D;
          when MEM_ADDRB  => reg_dec <= MEM_ADDR_D;
          when MEM_DATAB  => reg_dec <= MEM_DATA_D;
          when MEM_CNTB   => reg_dec <= MEM_CNT_D;

          when BRK0_CTLB   => reg_dec <= BRK0_CTL_D;
          when BRK0_STATB  => reg_dec <= BRK0_STAT_D;
          when BRK0_ADDR0B => reg_dec <= BRK0_ADDR0_D;
          when BRK0_ADDR1B => reg_dec <= BRK0_ADDR1_D;

          when BRK1_CTLB   => reg_dec <= BRK1_CTL_D;
          when BRK1_STATB  => reg_dec <= BRK1_STAT_D;
          when BRK1_ADDR0B => reg_dec <= BRK1_ADDR0_D;
          when BRK1_ADDR1B => reg_dec <= BRK1_ADDR1_D;

          when BRK2_CTLB   => reg_dec <= BRK2_CTL_D;
          when BRK2_STATB  => reg_dec <= BRK2_STAT_D;
          when BRK2_ADDR0B => reg_dec <= BRK2_ADDR0_D;
          when BRK2_ADDR1B => reg_dec <= BRK2_ADDR1_D;

          when BRK3_CTLB   => reg_dec <= BRK3_CTL_D;
          when BRK3_STATB  => reg_dec <= BRK3_STAT_D;
          when BRK3_ADDR0B => reg_dec <= BRK3_ADDR0_D;
          when BRK3_ADDR1B => reg_dec <= BRK3_ADDR1_D;

          when CPU_NRB => reg_dec <= CPU_NR_D;
          when others  => reg_dec <= (others => '0');
        end case;
      end process;
    end generate address_decode_04;

    --1.3.              Read/Write probes
    reg_write <= dbg_wr;
    reg_read  <= '1';

    --1.4.              Read/Write vectors
    reg_wr <= reg_dec and (0 to NR_REG - 1 => reg_write);
    reg_rd <= reg_dec and (0 to NR_REG - 1 => reg_read);
  end block P1_REGISTER_DECODER;

  P2_REGISTER_CORE_INTERFACE : block
  begin
    --2.1.              CPU_NR Register
    cpu_nr_s <= cpu_nr_total & cpu_nr_inst;

    --2.2.              CPU_CTL Register
    cpu_ctl_wr <= reg_wr(CPU_CTL);

    R_0i_1c : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        if (DBG_RST_BRK_EN = '1') then
          cpu_ctl_s <= std_logic_vector(to_unsigned(6, 4));
        elsif (DBG_RST_BRK_EN = '0') then
          cpu_ctl_s <= std_logic_vector(to_unsigned(2, 4));
        end if;
      elsif (rising_edge(dbg_clk)) then
        if (cpu_ctl_wr = '1') then
          cpu_ctl_s <= dbg_din(6 downto 3);
        end if;
      end if;
    end process R_0i_1c;

    cpu_ctl_full <= '0' & cpu_ctl_s & "000";
    halt_cpu     <= cpu_ctl_wr and dbg_din(HALT) and not dbg_halt_st;
    run_cpu      <= cpu_ctl_wr and dbg_din(RUN) and dbg_halt_st;
    istep_s      <= cpu_ctl_wr and dbg_din(ISTEP) and dbg_halt_st;

    --2.3.              CPU_STAT Register
    cpu_stat_wr  <= reg_wr(CPU_STAT);
    cpu_stat_set <= dbg_swbrk & puc_pnd_set;
    cpu_stat_clr <= not dbg_din(3 downto 2);

    R_1c_2 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        cpu_stat_s <= "00";
      elsif (rising_edge(dbg_clk)) then
        if (cpu_stat_wr = '1') then
          cpu_stat_s <= (cpu_stat_s and cpu_stat_clr) or cpu_stat_set;
        else
          cpu_stat_s <= cpu_stat_s or cpu_stat_set;
        end if;
      end if;
    end process R_1c_2;

    cpu_stat_full <= brk_pnd & cpu_stat_s & '0' & dbg_halt_st;
  end block P2_REGISTER_CORE_INTERFACE;

  P3_REGISTER_MEMORY_INTERFACE : block
  begin
    --3.1.              MEM_CTL Register
    mem_ctl_wr <= reg_wr(MEM_CTL);

    R_1c : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        mem_ctl_s <= "000";
      elsif (rising_edge(dbg_clk)) then
        if (mem_ctl_wr = '1') then
          mem_ctl_s <= dbg_din(3 downto 1);
        end if;
      end if;
    end process R_1c;

    mem_ctl_full <= (X"0" & mem_ctl_s & '0');

    R_1_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        mem_start <= '0';
      elsif (rising_edge(dbg_clk)) then
        mem_start <= mem_ctl_wr and dbg_din(0);
      end if;
    end process R_1_e;

    mem_bw <= mem_ctl_s(3);

    --3.2.              MEM_DATA Register
    mem_data_wr    <= reg_wr(MEM_DATA);
    dbg_mem_din_bw <= dbg_mem_din
                      when mem_bw = '0'        else X"00" & dbg_mem_din(15 downto 8)
                      when mem_addr_s(0) = '1' else X"00" & dbg_mem_din(7 downto 0);

    R1_1c_2c_3c : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        mem_data_s <= X"0000";
      elsif (rising_edge(dbg_clk)) then
        if (mem_data_wr = '1') then
          mem_data_s <= dbg_din;
        elsif (dbg_reg_rd = '1') then
          mem_data_s <= dbg_reg_din;
        elsif (dbg_mem_rd_dly = '1') then
          mem_data_s <= dbg_mem_din_bw;
        end if;
      end if;
    end process R1_1c_2c_3c;

    --3.3.              MEM_ADDR Register
    mem_addr_wr <= reg_wr(MEM_ADDR);
    dbg_mem_acc <= (dbg_mem_wr_omsp(0) or dbg_mem_wr_omsp(1)) or (dbg_rd_rdy and not mem_ctl_s(2));
    dbg_reg_acc <= dbg_reg_wr_omsp or (dbg_rd_rdy and mem_ctl_s(2));

    mem_addr_inc <= X"0000"
                    when (mem_cnt_s = X"0000")                              else X"0002"
                    when (mem_burst and dbg_mem_acc and not mem_bw) = '1'   else X"0001"
                    when (mem_burst and (dbg_mem_acc or dbg_reg_acc)) = '1' else X"0000";

    R1_1c_2 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        mem_addr_s <= X"0000";
      elsif (rising_edge(dbg_clk)) then
        if (mem_addr_wr = '1') then
          mem_addr_s <= dbg_din;
        else
          mem_addr_s <= std_logic_vector(unsigned(mem_addr_s) + unsigned(mem_addr_inc));
        end if;
      end if;
    end process R1_1c_2;

    --3.4.              MEM_CNT Register
    mem_cnt_wr  <= reg_wr(MEM_CNT);
    mem_cnt_dec <= X"0000"
                   when (mem_cnt_s = X"0000")                              else X"FFFF"
                   when (mem_burst and (dbg_mem_acc or dbg_reg_acc)) = '1' else X"0000";

    R2_1c_2 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        mem_cnt_s <= X"0000";
      elsif (rising_edge(dbg_clk)) then
        if (mem_cnt_wr = '1') then
          mem_cnt_s <= dbg_din;
        else
          mem_cnt_s <= std_logic_vector(unsigned(mem_cnt_s) + unsigned(mem_cnt_dec));
        end if;
      end if;
    end process R2_1c_2;
  end block P3_REGISTER_MEMORY_INTERFACE;

  P4_BREAKPOINTS_WATCHPOINTS : block
  begin
    --4.1.              Hardware Breakpoint/Watchpoint Register write/read select
    dbg_hwbr_on : for i in TOTAL_BP - 1 downto 0 generate
      dbg_hwbr_i : if (DBG_HWBRK(i) = '1') generate

        brk_reg_rd(i) <= reg_rd(BRK_ADDR1(i)) &
                         reg_rd(BRK_ADDR0(i)) &
                         reg_rd(BRK_STAT(i)) &
                         reg_rd(BRK_CTL(i));

        brk_reg_wr(i) <= reg_wr(BRK_ADDR1(i)) &
                         reg_wr(BRK_ADDR0(i)) &
                         reg_wr(BRK_STAT(i)) &
                         reg_wr(BRK_CTL(i));

        omsp_dbg_hwbrk_wp : omsp_dbg_hwbrk
          port map (
            brk_halt => brk_halt(i),
            brk_pnd  => brk_pnd(i),
            brk_dout => brk_dout(i),

            dbg_clk      => dbg_clk,
            dbg_rst      => dbg_rst,
            decode_noirq => decode_noirq,
            eu_mb_en     => eu_mb_en,
            eu_mb_wr     => eu_mb_wr,
            brk_reg_rd   => brk_reg_rd(i),
            brk_reg_wr   => brk_reg_wr(i),
            dbg_din      => dbg_din,
            eu_mab       => eu_mab,
            pc           => pc);
      end generate dbg_hwbr_i;
    end generate dbg_hwbr_on;

    dbg_hwbr_off : for i in TOTAL_BP - 1 downto 0 generate
      dbg_hwbr_i : if (DBG_HWBRK(i) = '0') generate
        brk_halt(i) <= '0';
        brk_pnd(i)  <= '0';
        brk_dout(i) <= (others => '0');
      end generate dbg_hwbr_i;
    end generate dbg_hwbr_off;
  end block P4_BREAKPOINTS_WATCHPOINTS;

  P5_DATA_OUTPUT_GENERATION : block
  begin
    cpu_id_lo_s <= cpu_id(15 downto 0);
    cpu_id_hi_s <= cpu_id(31 downto 16);

    cpu_id_lo_rd <= cpu_id_lo_s and (0 to 15             => reg_rd(CPU_ID_LO));
    cpu_id_hi_rd <= cpu_id_hi_s and (0 to 15             => reg_rd(CPU_ID_HI));
    cpu_ctl_rd   <= (X"00" & cpu_ctl_full) and (0 to 15  => reg_rd(CPU_CTL));
    cpu_stat_rd  <= (X"00" & cpu_stat_full) and (0 to 15 => reg_rd(CPU_STAT));
    mem_ctl_rd   <= (X"00" & mem_ctl_full) and (0 to 15  => reg_rd(MEM_CTL));
    mem_data_rd  <= mem_data_s and (0 to 15              => reg_rd(MEM_DATA));
    mem_addr_rd  <= mem_addr_s and (0 to 15              => reg_rd(MEM_ADDR));
    mem_cnt_rd   <= mem_cnt_s and (0 to 15               => reg_rd(MEM_CNT));
    cpu_nr_rd    <= cpu_nr_s and (0 to 15                => reg_rd(CPU_NR));

    dbg_dout <= cpu_id_lo_rd or
                cpu_id_hi_rd or
                cpu_ctl_rd or
                cpu_stat_rd or
                mem_ctl_rd or
                mem_data_rd or
                mem_addr_rd or
                mem_cnt_rd or
                cpu_nr_rd or
                matrix_or(brk_dout);

    --5.1.              Tell UART/I2C interface that the data is ready to be read
    R_1c_2_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        dbg_rd_rdy <= '0';
      elsif (rising_edge(dbg_clk)) then
        if ((mem_burst or mem_burst_rd) = '1') then
          dbg_rd_rdy <= dbg_reg_rd or dbg_mem_rd_dly;
        else
          dbg_rd_rdy <= dbg_rd;
        end if;
      end if;
    end process R_1c_2_e;
  end block P5_DATA_OUTPUT_GENERATION;

  P6_CPU_CONTROL : block
  begin
    --6.1.              Reset CPU
    dbg_cpu_reset <= cpu_ctl_s(CPU_RST);

    --6.2.              Beak after reset
    halt_rst <= cpu_ctl_s(RST_BRK_EN) and dbg_en_s and puc_pnd_set;

    --6.3.              Freeze peripherals
    dbg_freeze <= dbg_halt_st and (cpu_ctl_s(FRZ_BRK_EN) or not cpu_en_s);

    --6.4.              Software break
    dbg_swbrk <= to_stdlogic(fe_mdb_in = DBG_SWBRK_OP) and decode_noirq and cpu_ctl_s(SW_BRK_EN);

    --6.5.              Single step
    R_1c_2 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        inc_step <= "00";
      elsif (rising_edge(dbg_clk)) then
        if (istep_s = '1') then
          inc_step <= "11";
        else
          inc_step <= inc_step(0) & '0';
        end if;
      end if;
    end process R_1c_2;

    --6.6.              Run / Halt
    halt_flag_clr <= run_cpu or mem_run_cpu;
    halt_flag_set <= halt_cpu or halt_rst or dbg_swbrk or mem_halt_cpu or reduce_or(brk_halt);

    R_1c_2c_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        halt_flag <= '0';
      elsif (rising_edge(dbg_clk)) then
        if (halt_flag_clr = '1') then
          halt_flag <= '0';
        elsif (halt_flag_set = '1') then
          halt_flag <= '1';
        end if;
      end if;
    end process R_1c_2c_e;

    dbg_halt_cmd <= (halt_flag or halt_flag_set) and not inc_step(1);
  end block P6_CPU_CONTROL;

  P7_MEMORY_CONTROL : block
  begin
    --7.1.              Control Memory bursts
    mem_burst_start <= mem_start and reduce_or(mem_cnt_s);
    mem_burst_end   <= (dbg_wr or dbg_rd_rdy) and not reduce_or(mem_cnt_s);

    --7.1.1.    Detect when burst is on going
    R_1c_2c_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        mem_burst <= '0';
      elsif (rising_edge(dbg_clk)) then
        if (mem_burst_start = '1') then
          mem_burst <= '1';
        elsif (mem_burst_end = '1') then
          mem_burst <= '0';
        end if;
      end if;
    end process R_1c_2c_e;

    --7.1.2.    Control signals for UART/I2C interface 
    mem_burst_rd <= mem_burst_start and not mem_ctl_s(1);
    mem_burst_wr <= mem_burst_start and mem_ctl_s(1);

    --7.1.3.    Trigger CPU Register or memory access during a burst 
    R1_1_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        mem_startb <= '0';
      elsif (rising_edge(dbg_clk)) then
        mem_startb <= (mem_burst and (dbg_wr or dbg_rd)) or mem_burst_rd;
      end if;
    end process R1_1_e;

    --7.1.4.    Combine single and burst memory start of sequence 
    mem_seq_start <= (mem_start and not reduce_or(mem_cnt_s)) or mem_startb;

    --7.2.              Memory access state machine
    --7.2.1.    State machine definition 
    process(mem_state, re_m_idle, re_m_set_brk)
    begin
      case mem_state is
        when M_IDLE       => mem_state_nxt <= re_m_idle;
        when M_SET_BRK    => mem_state_nxt <= re_m_set_brk;
        when M_ACCESS_BRK => mem_state_nxt <= M_IDLE;
        when M_ACCESS     => mem_state_nxt <= M_IDLE;
        when others       => mem_state_nxt <= M_IDLE;
      end case;
    end process;

    re_m_idle <= M_IDLE
                 when mem_seq_start = '0' else M_ACCESS
                 when dbg_halt_st = '1'   else M_SET_BRK;

    re_m_set_brk <= M_ACCESS_BRK
                    when dbg_halt_st = '1' else M_SET_BRK;

    --7.2.2.    State transition 
    --7.2.3.    State machine
    R_1 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        mem_state <= M_IDLE;
      elsif (rising_edge(dbg_clk)) then
        mem_state <= mem_state_nxt;
      end if;
    end process R_1;

    --7.2.4.    Utility signals 
    mem_halt_cpu <= to_stdlogic(mem_state = M_IDLE) and to_stdlogic(mem_state_nxt = M_SET_BRK);
    mem_run_cpu  <= to_stdlogic(mem_state = M_ACCESS_BRK) and to_stdlogic(mem_state_nxt = M_IDLE);
    mem_access   <= to_stdlogic(mem_state = M_ACCESS) or to_stdlogic(mem_state = M_ACCESS_BRK);

    --7.3.              Interface to CPU Registers and Memory bacbkone
    dbg_mem_addr <= mem_addr_s;
    dbg_mem_dout <= mem_data_s
                    when mem_bw = '0'        else (mem_data_s(7 downto 0) & X"00")
                    when mem_addr_s(0) = '1' else (X"00" & mem_data_s(7 downto 0));
    dbg_reg_wr_omsp  <= mem_access and mem_ctl_s(1) and mem_ctl_s(2);
    dbg_reg_rd       <= mem_access and not mem_ctl_s(1) and mem_ctl_s(2);
    dbg_mem_en_omsp  <= mem_access and not mem_ctl_s(2);
    dbg_mem_rd       <= dbg_mem_en_omsp and not mem_ctl_s(1);
    dbg_mem_wr_msk_s <= "11"
                        when mem_bw = '0'        else "10"
                        when mem_addr_s(0) = '1' else "01";
    dbg_mem_wr_omsp <= (0 to 1 => (dbg_mem_en_omsp and mem_ctl_s(1))) and dbg_mem_wr_msk_s;

    --7.3.1.    It takes one additional cycle to read from Memory as from registers
    R2_1_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        dbg_mem_rd_dly <= '0';
      elsif (rising_edge(dbg_clk)) then
        dbg_mem_rd_dly <= dbg_mem_rd;
      end if;
    end process R2_1_e;
  end block P7_MEMORY_CONTROL;

  P8_UART_COMMUNICATION : block
  begin
    dbg_uart_on : if (DBG_UART = '1') generate
      omsp_dbg_uart_0 : omsp_dbg_uart
        port map (
          dbg_uart_txd => dbg_uart_txd,
          dbg_rd       => dbg_rd,
          dbg_wr       => dbg_wr,
          dbg_addr     => dbg_addr,
          dbg_din      => dbg_din,

          dbg_clk       => dbg_clk,
          dbg_rd_rdy    => dbg_rd_rdy,
          dbg_rst       => dbg_rst,
          dbg_uart_rxd  => dbg_uart_rxd,
          mem_burst     => mem_burst,
          mem_burst_end => mem_burst_end,
          mem_burst_rd  => mem_burst_rd,
          mem_burst_wr  => mem_burst_wr,
          mem_bw        => mem_bw,
          dbg_dout      => dbg_dout);
    end generate dbg_uart_on;

    dbg_uart_off : if (DBG_UART = '0') generate
      dbg_uart_txd <= '1';
      dbg_i2c_on : if (DBG_I2C = '0') generate
        dbg_addr <= "000000";
        dbg_din  <= X"0000";
        dbg_rd   <= '0';
        dbg_wr   <= '0';
      end generate dbg_i2c_on;
    end generate dbg_uart_off;
  end block P8_UART_COMMUNICATION;

  P9_I2C_COMMUNICATION : block
  begin
    dbg_i2c_on : if (DBG_I2C = '1') generate
      omsp_dbg_i2c_0 : omsp_dbg_i2c
        port map (
          dbg_i2c_sda_out => dbg_i2c_sda_out,
          dbg_rd          => dbg_rd,
          dbg_wr          => dbg_wr,
          dbg_addr        => dbg_addr,
          dbg_din         => dbg_din,

          dbg_clk           => dbg_clk,
          dbg_i2c_scl       => dbg_i2c_scl,
          dbg_i2c_sda_in    => dbg_i2c_sda_in,
          dbg_rd_rdy        => dbg_rd_rdy,
          dbg_rst           => dbg_rst,
          mem_burst         => mem_burst,
          mem_burst_end     => mem_burst_end,
          mem_burst_rd      => mem_burst_rd,
          mem_burst_wr      => mem_burst_wr,
          mem_bw            => mem_bw,
          dbg_i2c_addr      => dbg_i2c_addr,
          dbg_i2c_broadcast => dbg_i2c_broadcast,
          dbg_dout          => dbg_dout);
    end generate dbg_i2c_on;

    dbg_i2c_off : if (DBG_I2C = '0') generate
      dbg_i2c_sda_out <= '1';
    end generate dbg_i2c_off;
  end block P9_I2C_COMMUNICATION;

  SIGNAL_INOUT : block
  begin
    dbg_mem_en <= dbg_mem_en_omsp;
    dbg_reg_wr <= dbg_reg_wr_omsp;
    dbg_mem_wr <= dbg_mem_wr_omsp;
  end block SIGNAL_INOUT;
end DBG_ARQ;
