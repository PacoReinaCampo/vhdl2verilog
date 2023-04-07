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

entity omsp_state_machine is
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
end omsp_state_machine;

architecture FRONTEND_B1_ARQ of omsp_state_machine is

  --SIGNAL INOUT
  signal dbg_halt_st_omsp  : std_logic;
  signal decode_noirq_omsp : std_logic;
  signal decode_omsp       : std_logic;
  signal exec_done_omsp    : std_logic;
  signal irq_detect_omsp   : std_logic;
  signal i_state_omsp      : std_logic_vector (2 downto 0);
  signal e_state_omsp      : std_logic_vector (3 downto 0);

  --1.FRONTEND STATE MACHINE
  --The wire "conv" is used as state bits to calculate the next response
  signal i_state_nxt_omsp : std_logic_vector (2 downto 0);

  --CPU on/off through the debug interface or cpu_en port
  signal cpu_halt_cmd_omsp : std_logic;

  signal re_i_idle : std_logic_vector (2 downto 0);
  signal re_i_dec  : std_logic_vector (2 downto 0);
  signal re_i_ext1 : std_logic_vector (2 downto 0);

begin
  C1_FRONTEND_STATE_MACHINE : block
  begin
    --CPU on/off through the debug interface or cpu_en port
    cpu_halt_cmd_omsp <= dbg_halt_cmd or not cpu_en_s;

    --States Transitions
    process(i_state_omsp, re_i_dec, re_i_ext1, re_i_idle)
    begin
      case i_state_omsp is
        when I_IDLE      => i_state_nxt_omsp <= re_i_idle;
        when I_IRQ_FETCH => i_state_nxt_omsp <= I_IRQ_DONE;
        when I_IRQ_DONE  => i_state_nxt_omsp <= I_DEC;
        when I_DEC       => i_state_nxt_omsp <= re_i_dec;
        when I_EXT1      => i_state_nxt_omsp <= re_i_ext1;
        when I_EXT2      => i_state_nxt_omsp <= I_DEC;
        when others      => i_state_nxt_omsp <= I_IRQ_FETCH;
      end case;
    end process;

    re_i_idle <= I_IRQ_FETCH
                 when (irq_detect_omsp and not cpu_halt_cmd_omsp) = '1' else I_DEC
                 when (not cpuoff and not cpu_halt_cmd_omsp) = '1'      else I_IDLE;

    re_i_dec <= I_IRQ_FETCH
                when irq_detect_omsp = '1'                                    else I_IDLE
                when ((cpuoff or cpu_halt_cmd_omsp) and exec_done_omsp) = '1' else I_IDLE
                when cpu_halt_cmd_omsp = '1' and (e_state_omsp = E_IDLE)      else I_DEC
                when pc_sw_wr = '1'                                           else I_DEC
                when exec_done_omsp = '0' and not (e_state_omsp = E_IDLE)     else I_EXT1
                when inst_sz_nxt /= "00"                                      else I_DEC;

    re_i_ext1 <= I_DEC
                 when pc_sw_wr = '1'  else I_EXT2
                 when inst_sz /= "01" else I_DEC;

    --State machine
    R_1 : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        i_state_omsp <= I_IRQ_FETCH;
      elsif (rising_edge(mclk)) then
        i_state_omsp <= i_state_nxt_omsp;
      end if;
    end process R_1;

    --Utility signals   
    decode_noirq_omsp <= to_stdlogic(i_state_omsp = I_DEC) and (exec_done_omsp or to_stdlogic(e_state_omsp = E_IDLE));
    decode_omsp       <= decode_noirq_omsp or irq_detect_omsp;
    fetch             <= not (to_stdlogic(i_state_omsp = I_DEC) and not (exec_done_omsp or to_stdlogic(e_state_omsp = E_IDLE))) and not to_stdlogic(e_state_nxt = E_IDLE);

    --Debug interface cpu status
    R_1_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        dbg_halt_st_omsp <= '0';
      elsif (rising_edge(mclk)) then
        dbg_halt_st_omsp <= cpu_halt_cmd_omsp and to_stdlogic(i_state_nxt_omsp = I_IDLE);
      end if;
    end process R_1_e;
  end block C1_FRONTEND_STATE_MACHINE;

  SIGNAL_INOUT : block
  begin
    cpu_halt_cmd <= cpu_halt_cmd_omsp;
    dbg_halt_st  <= dbg_halt_st_omsp;
    decode       <= decode_omsp;
    decode_noirq <= decode_noirq_omsp;
    i_state      <= i_state_omsp;
    i_state_nxt  <= i_state_nxt_omsp;

    exec_done_omsp  <= exec_done;
    irq_detect_omsp <= irq_detect;
    e_state_omsp    <= e_state;
  end block SIGNAL_INOUT;
end FRONTEND_B1_ARQ;
