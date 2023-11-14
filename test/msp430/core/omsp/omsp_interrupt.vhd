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

entity omsp_interrupt is
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
end omsp_interrupt;

architecture FRONTEND_B2_ARQ of omsp_interrupt is

  -- SIGNAL INOUT
  signal inst_irq_rst_omsp : std_logic;
  signal irq_detect_omsp   : std_logic;
  signal irq_num_omsp      : std_logic_vector (5 downto 0);

  -- 2.INTERRUPT HANDLING & SYSTEM WAKEUP
  -- 2.1.INTERRUPT HANDLING
  -- Detect other interrupts
  signal mclk_irq_num : std_logic;

  -- Combine all IRQs
  signal irq_all : std_logic_vector (62 downto 0);

  -- Interrupt request accepted
  signal irq_acc_all : std_logic_vector (63 downto 0);

  function to_natural (entrada : unsigned) return natural is
    constant ARG_LEFT : integer := entrada'length - 1;
    alias XXARG       : unsigned (ARG_LEFT downto 0) is entrada;
    variable XARG     : unsigned (ARG_LEFT downto 0);
    variable RESULT   : natural := 0;
    variable w        : integer := 1;
  begin
    XARG := TO_01(XXARG);
    if (XARG(XARG'left) = 'X') then
      return 0;
    end if;
    for i in ARG_LEFT downto 0 loop
      if (XARG(i) = '1') then
        RESULT := RESULT + w;
      end if;
      if (i /= XARG'left) then
        w := w + w;
      end if;
    end loop;
    return RESULT;
  end to_natural;

  function one_hot64 (binary : std_logic_vector (5 downto 0)) return std_logic_vector is
    variable v : std_logic_vector (63 downto 0) := (others => '0');
  begin
    v(to_natural(unsigned(binary))) := '1';
    return v;
  end one_hot64;

  function one_hot16 (binary : std_logic_vector (3 downto 0)) return std_logic_vector is
    variable v : std_logic_vector (15 downto 0) := (others => '0');
  begin
    v(to_natural(unsigned(binary))) := '1';
    return v;
  end one_hot16;

  function one_hot8 (binary : std_logic_vector (2 downto 0)) return std_logic_vector is
    variable v : std_logic_vector (7 downto 0) := (others => '0');
  begin
    v(to_natural(unsigned(binary))) := '1';
    return v;
  end one_hot8;

  function get_irq_num (irq_all : std_logic_vector (62 downto 0)) return std_logic_vector is
    variable v : std_logic_vector (5 downto 0) := (others => '1');
  begin
    for i in 62 downto 0 loop
      if((v(0) and v(1) and v(2) and v(3) and v(4) and v(5) and irq_all(i)) = '1') then
        v := std_logic_vector(to_unsigned(i, 6));
      end if;
    end loop;
    return v;
  end get_irq_num;

begin
  C2_INTERRUPT_HANDLING_AND_SYSTEM_WAKEUP : block
  begin
    -- 2.1.INTERRUPT HANDLING
    -- Detect reset interrupt
    R_1c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_irq_rst_omsp <= '1';
      elsif (rising_edge(mclk)) then
        if (exec_done = '1') then
          inst_irq_rst_omsp <= '0';
        end if;
      end if;
    end process R_1c_e;

    -- Detect other interrupts
    irq_detect_omsp <= (nmi_pnd or ((reduce_or(irq) or wdt_irq) and gie))
                       and not cpu_halt_cmd and not dbg_halt_st and (exec_done or to_stdlogic(i_state = I_IDLE));

    irq_detect <= irq_detect_omsp;

    clock_gating_on : if (CLOCK_GATING = '1') generate
      clock_gate_irq_num : omsp_clock_gate
        port map (
          gclk        => mclk_irq_num,
          clk         => mclk,
          enable      => irq_detect_omsp,
          scan_enable => scan_enable);
    end generate clock_gating_on;

    clock_gating_off : if (CLOCK_GATING = '0') generate
      mclk_irq_num <= mclk;
    end generate clock_gating_off;

    -- Combine all IRQs
    combine_irq_16 : if (IRQ_16 = '1' and IRQ_32 = '0' and IRQ_64 = '0') generate
      irq_all <= (nmi_pnd & irq & (0 to 47 => '0')) or ("0000" & wdt_irq & (0 to 57 => '0'));
    end generate combine_irq_16;

    combine_irq_32 : if (IRQ_16 = '0' and IRQ_32 = '1' and IRQ_64 = '0') generate
      irq_all <= (nmi_pnd & irq & (0 to 31 => '0')) or ("0000" & wdt_irq & (0 to 57 => '0'));
    end generate combine_irq_32;

    combine_irq_64 : if (IRQ_16 = '0' and IRQ_32 = '0' and IRQ_64 = '1') generate
      irq_all <= (nmi_pnd & irq) or ("0000" & wdt_irq & (57 downto 0 => '0'));
    end generate combine_irq_64;

    -- Select highest priority IRQ
    R_1i_2ci : process (mclk_irq_num, puc_rst)
    begin
      if (puc_rst = '1') then
        irq_num_omsp <= "111111";
      elsif (rising_edge(mclk_irq_num)) then
        if (irq_detect_omsp = '1') then
          irq_num_omsp <= get_irq_num(irq_all);
        end if;
      end if;
    end process R_1i_2ci;

    irq_num <= irq_num_omsp;

    -- Generate selected IRQ vector address
    irq_addr <= (7 to 15 => '1') & irq_num_omsp & '0';

    -- Interrupt request accepted
    irq_acc_all <= one_hot64(irq_num_omsp) and (0 to 63 => to_stdlogic(i_state = I_IRQ_FETCH));
    irq_acc     <= irq_acc_all(61 downto 64 - IRQ_NR);
    nmi_acc     <= irq_acc_all(62);
  end block C2_INTERRUPT_HANDLING_AND_SYSTEM_WAKEUP;

  SIGNAL_INOUT : block
  begin
    inst_irq_rst <= inst_irq_rst_omsp;
  end block SIGNAL_INOUT;
end FRONTEND_B2_ARQ;
