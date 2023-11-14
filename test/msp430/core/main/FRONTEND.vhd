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

entity FRONTEND is
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
end FRONTEND;

architecture FRONTEND_ARQ of FRONTEND is

  -- SIGNAL INOUT
  signal dbg_halt_st_omsp  : std_logic;
  signal decode_noirq_omsp : std_logic;
  signal exec_done_omsp    : std_logic;
  signal inst_irq_rst_omsp : std_logic;
  signal inst_type_omsp    : std_logic_vector (2 downto 0);
  signal e_state_omsp      : std_logic_vector (3 downto 0);
  signal inst_ad_omsp      : std_logic_vector (7 downto 0);
  signal inst_as_omsp      : std_logic_vector (7 downto 0);
  signal inst_so_omsp      : std_logic_vector (7 downto 0);
  signal pc_omsp           : std_logic_vector (15 downto 0);
  signal pc_nxt_omsp       : std_logic_vector (15 downto 0);

  signal irq_detect_omsp : std_logic;
  signal i_state_omsp    : std_logic_vector (2 downto 0);
  signal decode_omsp     : std_logic;
  signal ir_omsp         : std_logic_vector (15 downto 0);
  signal irq_num_omsp    : std_logic_vector (5 downto 0);

  -- 1.FRONTEND STATE MACHINE
  -- The wire "conv" is used as state bits to calculate the next response
  signal is_const      : std_logic;
  signal inst_sz       : std_logic_vector (1 downto 0);
  signal inst_sz_nxt   : std_logic_vector (1 downto 0);
  signal inst_sz_nxt_a : std_logic_vector (1 downto 0);
  signal inst_sz_nxt_b : std_logic_vector (1 downto 0);
  signal i_state_nxt   : std_logic_vector (2 downto 0);
  signal inst_type_nxt : std_logic_vector (2 downto 0);
  signal e_state_nxt   : std_logic_vector (3 downto 0);
  signal sconst_nxt    : std_logic_vector (15 downto 0);

  -- CPU on/off through the debug interface or cpu_en port
  signal cpu_halt_cmd : std_logic;

  signal re_i_idle : std_logic_vector (2 downto 0);
  signal re_i_dec  : std_logic_vector (2 downto 0);
  signal re_i_ext1 : std_logic_vector (2 downto 0);

  -- Utility signals     
  signal fetch : std_logic;

  -- 2.INTERRUPT HANDLING & SYSTEM WAKEUP
  -- 2.1.INTERRUPT HANDLING
  -- Detect other interrupts
  signal mclk_irq_num : std_logic;

  -- Combine all IRQs
  signal irq_all : std_logic_vector (62 downto 0);

  -- Select highest priority IRQ

  -- Generate selected IRQ vector address
  signal irq_addr : std_logic_vector (15 downto 0);

  -- Interrupt request accepted
  signal irq_acc_all : std_logic_vector (63 downto 0);

  -- 2.2.SYSTEM WAKEUP
  -- Wakeup condition from maskable interrupts
  signal mirq_wkup : std_logic;

  -- 3.FETCH INSTRUCTION
  -- 3.1.PROGRAM COUNTER & MEMORY INTERFACE
  -- Compute next PC value
  signal mclk_pc : std_logic;
  signal pc_en   : std_logic;
  signal pc_incr : std_logic_vector (15 downto 0);

  -- Check if ROM has been busy in order to retry ROM access
  signal pmem_busy : std_logic;

  -- 3.2.INSTRUCTION REGISTER
  -- Instruction register

  -- Detect if source extension word is required
  signal is_sext : std_logic;

  -- For the Symbolic addressing mode, add -2 to the extension word in order to make up for the PC address
  signal ext_incr : std_logic_vector (15 downto 0);
  signal ext_nxt  : std_logic_vector (15 downto 0);

  -- Store source extension word
  signal inst_sext_en   : std_logic;
  signal mclk_inst_sext : std_logic;

  -- Source extension word is ready
  signal inst_sext_rdy : std_logic;

  -- Store destination extension word
  signal inst_dext_en   : std_logic;
  signal mclk_inst_dext : std_logic;

  -- Destination extension word is ready 
  signal inst_dext_rdy : std_logic;

  -- 4.DECODE INSTRUCTIONS
  signal mclk_decode : std_logic;

  -- 4.2.OPCODE: SINGLE-OPERAND ARITHMETIC 
  -- Instructions are encoded in a one hot fashion as following:
  signal inst_so_nxt : std_logic_vector (7 downto 0);

  -- 4.3.OPCODE: CONDITIONAL JUMP 
  -- Instructions are encoded in a one hot fashion as following: 
  signal inst_jmp_bin : std_logic_vector (2 downto 0);

  -- 4.4.OPCODE: TWO-OPERAND ARITHMETIC
  -- Instructions are encoded in a one hot fashion as following:
  signal inst_to_nxt  : std_logic_vector (11 downto 0);
  signal inst_to_1hot : std_logic_vector (15 downto 0);

  -- 4.5.SOURCE AND DESTINATION REGISTERS
  -- Destination register
  signal inst_dest_bin : std_logic_vector (3 downto 0);

  -- Source register
  signal inst_src_bin : std_logic_vector (3 downto 0);

  -- 4.6.SOURCE ADDRESSING MODES
  -- Source addressing modes are encoded in a one hot fashion as following:
  signal src_reg     : std_logic_vector (3 downto 0);
  signal inst_as_nxt : std_logic_vector (12 downto 0);

  -- 4.7.DESTINATION ADDRESSING MODES
  -- Source addressing modes are encoded in a one hot fashion as following:
  signal dest_reg    : std_logic_vector (3 downto 0);
  signal inst_ad_nxt : std_logic_vector (7 downto 0);

  -- 5.EXECUTION-UNIT STATE MACHINE
  signal re_e_src_ad : std_logic_vector (3 downto 0);
  signal re_e_src_rd : std_logic_vector (3 downto 0);
  signal re_e_dst_ad : std_logic_vector (3 downto 0);
  signal re_e_exec   : std_logic_vector (3 downto 0);
  signal re_e_dst_wr : std_logic_vector (3 downto 0);

  -- State machine control signals
  signal src_acalc_pre : std_logic;
  signal src_rd_pre    : std_logic;
  signal dst_acalc_pre : std_logic;
  signal dst_acalc     : std_logic;
  signal dst_rd_pre    : std_logic;
  signal dst_rd        : std_logic;
  signal inst_branch   : std_logic;
  signal exec_jmp      : std_logic;
  signal exec_dst_wr   : std_logic;
  signal exec_src_wr   : std_logic;
  signal exec_dext_rdy : std_logic;

  -- Execution first state
  signal e_first_state : std_logic_vector (3 downto 0);

  -- 6.EXECUTION-UNIT STATE FRONTEND
  -- 6.1.ALU FRONTEND SIGNALS
  signal alu_src_inv_s  : std_logic;
  signal alu_inc_s      : std_logic;
  signal alu_inc_c_s    : std_logic;
  signal alu_add_s      : std_logic;
  signal alu_and_s      : std_logic;
  signal alu_or_s       : std_logic;
  signal alu_xor_s      : std_logic;
  signal alu_dadd_s     : std_logic;
  signal alu_stat_7_s   : std_logic;
  signal alu_stat_f_s   : std_logic;
  signal alu_shift_s    : std_logic;
  signal exec_no_wr_s   : std_logic;
  signal inst_alu_nxt_s : std_logic_vector (11 downto 0);

  function one_hot64 (binary : std_logic_vector (5 downto 0)) return std_logic_vector is
    variable v : std_logic_vector (63 downto 0) := (others => '0');
  begin
    v(to_integer(unsigned(binary))) := '1';
    return v;
  end one_hot64;

  function one_hot16 (binary : std_logic_vector (3 downto 0)) return std_logic_vector is
    variable v : std_logic_vector (15 downto 0) := (others => '0');
  begin
    v(to_integer(unsigned(binary))) := '1';
    return v;
  end one_hot16;

  function one_hot8 (binary : std_logic_vector (2 downto 0)) return std_logic_vector is
    variable v : std_logic_vector (7 downto 0) := (others => '0');
  begin
    v(to_integer(unsigned(binary))) := '1';
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
  C1_FRONTEND_STATE_MACHINE : block
  begin
    omsp_state_machine_0 : omsp_state_machine
      port map(
        dbg_halt_st  => dbg_halt_st_omsp,
        decode_noirq => decode_noirq_omsp,
        fetch        => fetch,
        decode       => decode_omsp,
        cpu_halt_cmd => cpu_halt_cmd,
        i_state      => i_state_omsp,
        i_state_nxt  => i_state_nxt,

        exec_done    => exec_done_omsp,
        cpu_en_s     => cpu_en_s,
        cpuoff       => cpuoff,
        irq_detect   => irq_detect_omsp,
        dbg_halt_cmd => dbg_halt_cmd,
        mclk         => mclk,
        pc_sw_wr     => pc_sw_wr,
        puc_rst      => puc_rst,
        inst_sz      => inst_sz,
        inst_sz_nxt  => inst_sz_nxt,
        e_state      => e_state_omsp,
        e_state_nxt  => e_state_nxt);
  end block C1_FRONTEND_STATE_MACHINE;

  C2_INTERRUPT_HANDLING_AND_SYSTEM_WAKEUP : block
  begin
    omsp_interrupt_0 : omsp_interrupt
      port map(
        inst_irq_rst => inst_irq_rst_omsp,
        irq_detect   => irq_detect_omsp,
        nmi_acc      => nmi_acc,
        irq_num      => irq_num,
        irq_addr     => irq_addr,
        irq_acc      => irq_acc,

        mclk         => mclk,
        puc_rst      => puc_rst,
        exec_done    => exec_done_omsp,
        nmi_pnd      => nmi_pnd,
        dbg_halt_st  => dbg_halt_st_omsp,
        gie          => gie,
        scan_enable  => scan_enable,
        wdt_irq      => wdt_irq,
        cpu_halt_cmd => cpu_halt_cmd,
        i_state      => i_state_omsp,
        irq          => irq);

    -- 2.2.SYSTEM WAKEUP
    -- Generate the main system clock enable signal
    cpuoff_en_on : if (CPUOFF_EN = '1') generate
      mclk_enable <= cpu_en_s
                     when inst_irq_rst_omsp = '1' else
                     not ((cpuoff or not cpu_en_s) and to_stdlogic(i_state_omsp = I_IDLE) and to_stdlogic(e_state_omsp = E_IDLE));

      mirq_wkup <= (wkup or wdt_wkup) and gie;
      mclk_wkup <= (nmi_wkup or mirq_wkup) and cpu_en_s;
    end generate cpuoff_en_on;

    cpuoff_en_off : if (CPUOFF_EN = '0') generate
      mclk_wkup   <= '1';
      mclk_enable <= '1';
    end generate cpuoff_en_off;
  end block C2_INTERRUPT_HANDLING_AND_SYSTEM_WAKEUP;

  C3_FETCH_INSTRUCTION : block
  begin
    -- 3.1.PROGRAM COUNTER & MEMORY INTERFACE
    -- Compute next PC value
    pc_incr     <= std_logic_vector(unsigned(pc_omsp) + ((1 to 14 => '0') & fetch & '0'));
    pc_nxt_omsp <= pc_sw
                   when pc_sw_wr = '1'             else irq_addr
                   when i_state_omsp = I_IRQ_FETCH else mdb_in
                   when i_state_omsp = I_IRQ_DONE  else pc_incr;

    clock_gating_on : if (CLOCK_GATING = '1') generate
      pc_en <= fetch or pc_sw_wr or to_stdlogic(i_state_omsp = I_IRQ_FETCH) or to_stdlogic(i_state_omsp = I_IRQ_DONE);

      clock_gate_pc : omsp_clock_gate
        port map (
          gclk        => mclk_pc,
          clk         => mclk,
          enable      => pc_en,
          scan_enable => scan_enable);
    end generate clock_gating_on;

    clock_gating_off : if (CLOCK_GATING = '0') generate
      mclk_pc <= mclk;
    end generate clock_gating_off;

    R_1 : process (mclk_pc, puc_rst)
    begin
      if (puc_rst = '1') then
        pc_omsp <= X"0000";
      elsif (rising_edge(mclk_pc)) then
        pc_omsp <= pc_nxt_omsp;
      end if;
    end process;

    -- Check if ROM has been busy in order to retry ROM access
    R_1_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        pmem_busy <= '0';
      elsif (rising_edge(mclk)) then
        pmem_busy <= fe_pmem_wait;
      end if;
    end process R_1_e;

    -- Memory interface
    mab   <= pc_nxt_omsp;
    mb_en <= fetch or pc_sw_wr or to_stdlogic(i_state_omsp = I_IRQ_FETCH) or pmem_busy or (dbg_halt_st_omsp and not cpu_halt_cmd);

    -- 3.2.INSTRUCTION REGISTER
    -- Instruction register
    ir_omsp <= mdb_in;

    -- Detect if source extension word is required
    is_sext <= inst_as_omsp(IDX) or inst_as_omsp(SYMB) or inst_as_omsp(ABSC) or inst_as_omsp(IMM);

    -- For the Symbolic addressing mode, add -2 to the extension word in order to make up for the PC address
    ext_incr <= X"FFFE"
                when (i_state_omsp = I_EXT1 and inst_as_omsp(SYMB) = '1') or
                (i_state_omsp = I_EXT2 and inst_ad_omsp(SYMB) = '1') or
                (i_state_omsp = I_EXT1 and inst_as_omsp(SYMB) = '0' and
                 i_state_nxt /= I_EXT2 and inst_ad_omsp(SYMB) = '1')
                else X"0000";
    ext_nxt <= std_logic_vector(unsigned(ir_omsp) + unsigned(ext_incr));

    -- Store source extension word 
    clock_gating_1_on : if (CLOCK_GATING = '1') generate
      inst_sext_en <= (decode_omsp and is_const) or (decode_omsp and inst_type_nxt(INST_JMPC)) or (to_stdlogic(i_state_omsp = I_EXT1) and is_sext);

      clock_gate_inst_sext : omsp_clock_gate
        port map (
          gclk        => mclk_inst_sext,
          clk         => mclk,
          enable      => inst_sext_en,
          scan_enable => scan_enable);
    end generate clock_gating_1_on;

    clock_gating_1_off : if (CLOCK_GATING = '0') generate
      mclk_inst_sext <= mclk;
    end generate clock_gating_1_off;

    R_1c_2c_3i_4ci : process (mclk_inst_sext, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_sext <= X"0000";
      elsif (rising_edge(mclk_inst_sext)) then
        if ((decode_omsp and is_const) = '1') then
          inst_sext <= sconst_nxt;
        elsif ((decode_omsp and inst_type_nxt(INST_JMPC)) = '1') then
          inst_sext <= (11 to 15 => ir_omsp(9)) & ir_omsp(9 downto 0) & '0';
        elsif (CLOCK_GATING = '1') then
          inst_sext <= ext_nxt;
        elsif ((i_state_omsp = I_EXT1) and is_sext = '1' and CLOCK_GATING = '0') then
          inst_sext <= ext_nxt;
        end if;
      end if;
    end process R_1c_2c_3i_4ci;

    -- Source extension word is ready
    inst_sext_rdy <= to_stdlogic(i_state_omsp = I_EXT1) and is_sext;

    -- Store destination extension word
    clock_gating_2_on : if (CLOCK_GATING = '1') generate
      inst_dext_en <= (to_stdlogic(i_state_omsp = I_EXT1) and not is_sext) or to_stdlogic(i_state_omsp = I_EXT2);

      clock_gate_inst_dext : omsp_clock_gate
        port map (
          gclk        => mclk_inst_dext,
          clk         => mclk,
          enable      => inst_dext_en,
          scan_enable => scan_enable);
    end generate clock_gating_2_on;

    clock_gating_2_off : if (CLOCK_GATING = '0') generate
      mclk_inst_dext <= mclk;
    end generate clock_gating_2_off;

    R_1c_2i_3ci : process (mclk_inst_dext, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_dext <= X"0000";
      elsif (rising_edge(mclk_inst_dext)) then
        if ((i_state_omsp = I_EXT1) and is_sext = '0') then
          inst_dext <= ext_nxt;
        elsif (CLOCK_GATING = '1') then
          inst_dext <= ext_nxt;
        elsif (i_state_omsp = I_EXT2 and CLOCK_GATING = '0') then
          inst_dext <= ext_nxt;
        end if;
      end if;
    end process R_1c_2i_3ci;

    -- Destination extension word is ready
    inst_dext_rdy <= (to_stdlogic(i_state_omsp = I_EXT1) and not is_sext) or to_stdlogic(i_state_omsp = I_EXT2);
  end block C3_FETCH_INSTRUCTION;

  C4_DECODE_INSTRUCTIONS : block
  begin
    clock_gating_on : if (CLOCK_GATING = '1') generate

      clock_gate_decode : omsp_clock_gate
        port map (
          gclk        => mclk_decode,
          clk         => mclk,
          enable      => decode_omsp,
          scan_enable => scan_enable);
    end generate clock_gating_on;

    clock_gating_off : if (CLOCK_GATING = '0') generate
      mclk_decode <= mclk;
    end generate clock_gating_off;

    -- 4.1.OPCODE: INSTRUCTION TYPE
    -- Instructions type is encoded in a one hot fashion as following:   
    inst_type_nxt <= (to_stdlogic(ir_omsp(15 downto 14) /= "00") &
                      to_stdlogic(ir_omsp(15 downto 13) = "001") &
                      to_stdlogic(ir_omsp(15 downto 13) = "000"))
                     and (0 to 2 => not irq_detect_omsp);

    R_1_1i_2ci : process (mclk_decode, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_type_omsp <= "000";
      elsif (rising_edge(mclk_decode)) then
        if (CLOCK_GATING = '1') then
          inst_type_omsp <= inst_type_nxt;
        elsif (decode_omsp = '1' and CLOCK_GATING = '0') then
          inst_type_omsp <= inst_type_nxt;
        end if;
      end if;
    end process R_1_1i_2ci;

    -- 4.2.OPCODE: SINGLE-OPERAND ARITHMETIC
    -- Instructions are encoded in a one hot fashion as following:
    inst_so_nxt <= "10000000"
                   when irq_detect_omsp = '1' else (one_hot8(ir_omsp(9 downto 7)) and (0 to 7 => inst_type_nxt(INST_SOC)));

    R_2_1i_2ci : process (mclk_decode, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_so_omsp <= X"00";
      elsif (rising_edge(mclk_decode)) then
        if (CLOCK_GATING = '1') then
          inst_so_omsp <= inst_so_nxt;
        elsif (decode_omsp = '1' and CLOCK_GATING = '0') then
          inst_so_omsp <= inst_so_nxt;
        end if;
      end if;
    end process R_2_1i_2ci;

    -- 4.3.OPCODE: CONDITIONAL JUMP inst_jmp_bin
    -- Instructions are encoded in a one hot fashion as following:
    R_3_1i_2ci : process (mclk_decode, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_jmp_bin <= "000";
      elsif (rising_edge(mclk_decode)) then
        if (CLOCK_GATING = '1') then
          inst_jmp_bin <= ir_omsp(12 downto 10);
        elsif (decode_omsp = '1' and CLOCK_GATING = '0') then
          inst_jmp_bin <= ir_omsp(12 downto 10);
        end if;
      end if;
    end process R_3_1i_2ci;

    inst_jmp <= one_hot8(inst_jmp_bin) and (0 to 7 => inst_type_omsp(INST_JMPC));

    -- 4.4.OPCODE: TWO-OPERAND ARITHMETIC
    -- Instructions are encoded in a one hot fashion as following:
    inst_to_1hot <= one_hot16(ir_omsp(15 downto 12)) and (0 to 15 => inst_type_nxt(INST_TOC));
    inst_to_nxt  <= inst_to_1hot(15 downto 4);

    R_1_1i_2ci_e : process (mclk_decode, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_mov <= '0';
      elsif (rising_edge(mclk_decode)) then
        if (CLOCK_GATING = '1') then
          inst_mov <= inst_to_nxt(MOV);
        elsif (decode_omsp = '1' and CLOCK_GATING = '0') then
          inst_mov <= inst_to_nxt(MOV);
        end if;
      end if;
    end process R_1_1i_2ci_e;

    -- 4.5.SOURCE AND DESTINATION REGISTERS
    -- Destination register
    R_4_1i_2ci : process (mclk_decode, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_dest_bin <= "0000";
      elsif (rising_edge(mclk_decode)) then
        if (CLOCK_GATING = '1') then
          inst_dest_bin <= ir_omsp(3 downto 0);
        elsif (decode_omsp = '1' and CLOCK_GATING = '0') then
          inst_dest_bin <= ir_omsp(3 downto 0);
        end if;
      end if;
    end process R_4_1i_2ci;

    inst_dest <= one_hot16(dbg_reg_sel)
                 when dbg_halt_st_omsp = '1'                                                 else X"0001"
                 when inst_type_omsp(INST_JMPC) = '1'                                        else X"0002"
                 when (inst_so_omsp(IRQX) or inst_so_omsp(PUSH) or inst_so_omsp(CALL)) = '1' else one_hot16(inst_dest_bin);

    -- Source register
    R_5_1i_2ci : process (mclk_decode, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_src_bin <= "0000";
      elsif (rising_edge(mclk_decode)) then
        if (CLOCK_GATING = '1') then
          inst_src_bin <= ir_omsp(11 downto 8);
        elsif (decode_omsp = '1' and CLOCK_GATING = '0') then
          inst_src_bin <= ir_omsp(11 downto 8);
        end if;
      end if;
    end process R_5_1i_2ci;

    inst_src <= one_hot16(inst_src_bin)
                when inst_type_omsp(INST_TOC) = '1' else X"0002"
                when inst_so_omsp(RETI) = '1'       else X"0001"
                when inst_so_omsp(IRQX) = '1'       else one_hot16(inst_dest_bin)
                when inst_type_omsp(INST_SOC) = '1' else X"0000";

    -- 4.6.SOURCE ADDRESSING MODES
    -- Source addressing modes are encoded in a one hot fashion as following:    
    src_reg <= ir_omsp(3 downto 0) when inst_type_nxt(INST_SOC) = '1' else ir_omsp(11 downto 8);

    process (src_reg, ir_omsp, inst_type_nxt)
    begin
      if (inst_type_nxt(INST_JMPC) = '1') then
        inst_as_nxt <= "0000000000001";
      elsif (src_reg = X"3") then
        case ir_omsp(5 downto 4) is
          when "11"   => inst_as_nxt <= "1000000000000";
          when "10"   => inst_as_nxt <= "0100000000000";
          when "01"   => inst_as_nxt <= "0010000000000";
          when others => inst_as_nxt <= "0001000000000";
        end case;
      elsif (src_reg = X"2") then
        case ir_omsp(5 downto 4) is
          when "11"   => inst_as_nxt <= "0000100000000";
          when "10"   => inst_as_nxt <= "0000010000000";
          when "01"   => inst_as_nxt <= "0000001000000";
          when others => inst_as_nxt <= "0000000000001";
        end case;
      elsif (src_reg = X"0") then
        case ir_omsp(5 downto 4) is
          when "11"   => inst_as_nxt <= "0000000100000";
          when "10"   => inst_as_nxt <= "0000000000100";
          when "01"   => inst_as_nxt <= "0000000010000";
          when others => inst_as_nxt <= "0000000000001";
        end case;
      else
        case ir_omsp(5 downto 4) is
          when "11"   => inst_as_nxt <= "0000000001000";
          when "10"   => inst_as_nxt <= "0000000000100";
          when "01"   => inst_as_nxt <= "0000000000010";
          when others => inst_as_nxt <= "0000000000001";
        end case;
      end if;
    end process;

    is_const <= reduce_or(inst_as_nxt(12 downto 7));

    R_6_1i_2ci : process (mclk_decode, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_as_omsp <= X"00";
      elsif (rising_edge(mclk_decode)) then
        if (CLOCK_GATING = '1') then
          inst_as_omsp <= is_const & inst_as_nxt(6 downto 0);
        elsif (decode_omsp = '1' and CLOCK_GATING = '0') then
          inst_as_omsp <= is_const & inst_as_nxt(6 downto 0);
        end if;
      end if;
    end process R_6_1i_2ci;

    process (inst_as_nxt)
    begin
      if (inst_as_nxt(07) = '1') then sconst_nxt    <= X"0004";
      elsif (inst_as_nxt(08) = '1') then sconst_nxt <= X"0008";
      elsif (inst_as_nxt(09) = '1') then sconst_nxt <= X"0000";
      elsif (inst_as_nxt(10) = '1') then sconst_nxt <= X"0001";
      elsif (inst_as_nxt(11) = '1') then sconst_nxt <= X"0002";
      elsif (inst_as_nxt(12) = '1') then sconst_nxt <= X"FFFF";
      else sconst_nxt                               <= X"0000";
      end if;
    end process;

    -- 4.7.DESTINATION ADDRESSING MODES
    -- Destination addressing modes are encoded in a one hot fashion as following:       
    dest_reg <= ir_omsp(3 downto 0);

    process (inst_type_nxt, dest_reg, ir_omsp)
    begin
      if (inst_type_nxt(INST_TOC) = '0') then
        inst_ad_nxt <= "00000000";
      elsif (dest_reg = X"2") then
        case ir_omsp(7) is
          when '1'    => inst_ad_nxt <= "01000000";
          when others => inst_ad_nxt <= "00000001";
        end case;
      elsif (dest_reg = X"0") then
        case ir_omsp(7) is
          when '1'    => inst_ad_nxt <= "00010000";
          when others => inst_ad_nxt <= "00000001";
        end case;
      else
        case ir_omsp(7) is
          when '1'    => inst_ad_nxt <= "00000010";
          when others => inst_ad_nxt <= "00000001";
        end case;
      end if;
    end process;

    R_7_1i_2ci : process (mclk_decode, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_ad_omsp <= X"00";
      elsif (rising_edge(mclk_decode)) then
        if (CLOCK_GATING = '1') then
          inst_ad_omsp <= inst_ad_nxt;
        elsif (decode_omsp = '1' and CLOCK_GATING = '0') then
          inst_ad_omsp <= inst_ad_nxt;
        end if;
      end if;
    end process R_7_1i_2ci;

    -- 4.8.REMAINING INSTRUCTION DECODING
    -- Operation size
    R_1_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_bw <= '0';
      elsif (rising_edge(mclk)) then
        if (decode_omsp = '1') then
          inst_bw <= ir_omsp(6) and not inst_type_nxt(INST_JMPC) and not irq_detect_omsp and not cpu_halt_cmd;
        end if;
      end if;
    end process;

    -- Extended instruction size
    inst_sz_nxt   <= std_logic_vector(unsigned(inst_sz_nxt_a) + unsigned(inst_sz_nxt_b));
    inst_sz_nxt_a <= '0' & (inst_as_nxt(IDX) or inst_as_nxt(SYMB) or inst_as_nxt(ABSC) or inst_as_nxt(IMM));
    inst_sz_nxt_b <= ('0' & ((inst_ad_nxt(IDX) or inst_ad_nxt(SYMB) or inst_ad_nxt(ABSC)) and not inst_type_nxt(INST_SOC)));

    R_8_1i_2ci : process (mclk_decode, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_sz <= "00";
      elsif (rising_edge(mclk_decode)) then
        if (CLOCK_GATING = '1') then
          inst_sz <= inst_sz_nxt;
        elsif (decode_omsp = '1' and CLOCK_GATING = '0') then
          inst_sz <= inst_sz_nxt;
        end if;
      end if;
    end process R_8_1i_2ci;
  end block C4_DECODE_INSTRUCTIONS;

  C5_EXECUTION_UNIT_STATE_MACHINE : block
  begin
    -- 5.1.State machine control signals
    src_acalc_pre <= inst_as_nxt(IDX) or inst_as_nxt(SYMB) or inst_as_nxt(ABSC);
    src_rd_pre    <= inst_as_nxt(INDIR) or inst_as_nxt(INDIR_I) or inst_as_nxt(IMM) or inst_so_nxt(RETI);
    dst_acalc_pre <= inst_ad_nxt(IDX) or inst_ad_nxt(SYMB) or inst_ad_nxt(ABSC);
    dst_acalc     <= inst_ad_omsp(IDX) or inst_ad_omsp(SYMB) or inst_ad_omsp(ABSC);
    dst_rd_pre    <= inst_ad_nxt(IDX) or inst_so_nxt(PUSH) or inst_so_nxt(CALL) or inst_so_nxt(RETI);
    dst_rd        <= inst_ad_omsp(IDX) or inst_so_omsp(PUSH) or inst_so_omsp(CALL) or inst_so_omsp(RETI);

    inst_branch <= (inst_ad_nxt(DIR) and to_stdlogic(ir_omsp(3 downto 0) = X"0")) or inst_type_nxt(INST_JMPC) or inst_so_nxt(RETI);

    R_1_1c_2c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        exec_jmp <= '0';
      elsif (rising_edge(mclk)) then
        if ((inst_branch and decode_omsp) = '1') then
          exec_jmp <= '1';
        elsif (e_state_omsp = E_JUMP) then
          exec_jmp <= '0';
        end if;
      end if;
    end process R_1_1c_2c_e;

    R_2_1c_2c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        exec_dst_wr <= '0';
      elsif (rising_edge(mclk)) then
        if (e_state_omsp = E_DST_RD) then
          exec_dst_wr <= '1';
        elsif (e_state_omsp = E_DST_WR) then
          exec_dst_wr <= '0';
        end if;
      end if;
    end process R_2_1c_2c_e;

    R_3_1c_2c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        exec_src_wr <= '0';
      elsif (rising_edge(mclk)) then
        if (inst_type_omsp(INST_SOC) = '1' and e_state_omsp = E_SRC_RD) then
          exec_src_wr <= '1';
        elsif ((e_state_omsp = E_SRC_WR) or (e_state_omsp = E_DST_WR)) then
          exec_src_wr <= '0';
        end if;
      end if;
    end process R_3_1c_2c_e;

    R_4_1c_2c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        exec_dext_rdy <= '0';
      elsif (rising_edge(mclk)) then
        if (e_state_omsp = E_DST_RD) then
          exec_dext_rdy <= '0';
        elsif (inst_dext_rdy = '1') then
          exec_dext_rdy <= '1';
        end if;
      end if;
    end process R_4_1c_2c_e;

    -- Execution first state
    e_first_state <= E_IRQ(0)
                     when (not dbg_halt_st_omsp and inst_so_nxt(IRQX)) = '1' else E_IDLE
                     when cpu_halt_cmd = '1' or (i_state_omsp = I_IDLE)      else E_IDLE
                     when cpuoff = '1'                                       else E_SRC_AD
                     when src_acalc_pre = '1'                                else E_SRC_RD
                     when src_rd_pre = '1'                                   else E_DST_AD
                     when dst_acalc_pre = '1'                                else E_DST_RD
                     when dst_rd_pre = '1'                                   else E_EXEC;

    -- 5.2.State machine
    -- States Transitions
    process (e_state_omsp, e_first_state, re_e_dst_ad, re_e_dst_wr, re_e_exec, re_e_src_ad, re_e_src_rd)
    begin
      case e_state_omsp is
        when E_IDLE   => e_state_nxt <= e_first_state;
        when E_IRQ_0  => e_state_nxt <= E_IRQ_1;
        when E_IRQ_1  => e_state_nxt <= E_IRQ_2;
        when E_IRQ_2  => e_state_nxt <= E_IRQ_3;
        when E_IRQ_3  => e_state_nxt <= E_IRQ_4;
        when E_IRQ_4  => e_state_nxt <= E_EXEC;
        when E_SRC_AD => e_state_nxt <= re_e_src_ad;
        when E_SRC_RD => e_state_nxt <= re_e_src_rd;
        when E_DST_AD => e_state_nxt <= re_e_dst_ad;
        when E_DST_RD => e_state_nxt <= E_EXEC;
        when E_EXEC   => e_state_nxt <= re_e_exec;
        when E_JUMP   => e_state_nxt <= e_first_state;
        when E_DST_WR => e_state_nxt <= re_e_dst_wr;
        when E_SRC_WR => e_state_nxt <= e_first_state;
        when others   => e_state_nxt <= E_IRQ(0);
      end case;
    end process;

    re_e_src_ad <= E_SRC_RD
                   when inst_sext_rdy = '1' else E_SRC_AD;

    re_e_src_rd <= E_DST_AD
                   when dst_acalc = '1' else E_DST_RD
                   when dst_rd = '1'    else E_EXEC;

    re_e_dst_ad <= E_DST_RD
                   when (inst_dext_rdy or exec_dext_rdy) = '1' else E_DST_AD;

    re_e_exec <= E_DST_WR
                 when exec_dst_wr = '1' else E_JUMP
                 when exec_jmp = '1'    else E_SRC_WR
                 when exec_src_wr = '1' else e_first_state;

    re_e_dst_wr <= E_JUMP
                   when exec_jmp = '1' else e_first_state;

    -- State machine
    R_1 : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        e_state_omsp <= E_IRQ(1);
      elsif (rising_edge(mclk)) then
        e_state_omsp <= e_state_nxt;
      end if;
    end process R_1;

    -- 5.3.Frontend State machine control signals
    exec_done_omsp <= to_stdlogic(e_state_omsp = E_JUMP)
                      when exec_jmp = '1'    else to_stdlogic(e_state_omsp = E_DST_WR)
                      when exec_dst_wr = '1' else to_stdlogic(e_state_omsp = E_SRC_WR)
                      when exec_src_wr = '1' else to_stdlogic(e_state_omsp = E_EXEC);
  end block C5_EXECUTION_UNIT_STATE_MACHINE;

  C6_EXECUTION_UNIT_STATE_FRONTEND_B6 : block
  begin
    -- ALU FRONTEND_B6 SIGNALS
    alu_src_inv_s <= inst_to_nxt(SUBB) or inst_to_nxt(SUBC) or
                     inst_to_nxt(CMP) or inst_to_nxt(BIC);

    alu_inc_s <= inst_to_nxt(SUBB) or inst_to_nxt(CMP);

    alu_inc_c_s <= inst_to_nxt(ADDC) or inst_to_nxt(DADD) or
                   inst_to_nxt(SUBC);

    alu_add_s <= inst_to_nxt(ADD) or inst_to_nxt(ADDC) or
                 inst_to_nxt(SUBB) or inst_to_nxt(SUBC) or
                 inst_to_nxt(CMP) or inst_type_nxt(INST_JMPC) or
                 inst_so_nxt(RETI);

    alu_and_s <= inst_to_nxt(ANDX) or inst_to_nxt(BIC) or
                 inst_to_nxt(BITC);

    alu_or_s <= inst_to_nxt(BIS);

    alu_xor_s <= inst_to_nxt(XORX);

    alu_dadd_s <= inst_to_nxt(DADD);

    alu_stat_7_s <= inst_to_nxt(BITC) or inst_to_nxt(ANDX) or
                    inst_so_nxt(SXTC);

    alu_stat_f_s <= inst_to_nxt(ADD) or inst_to_nxt(ADDC) or
                    inst_to_nxt(SUBB) or inst_to_nxt(SUBC) or
                    inst_to_nxt(CMP) or inst_to_nxt(DADD) or
                    inst_to_nxt(BITC) or inst_to_nxt(XORX) or
                    inst_to_nxt(ANDX) or
                    inst_so_nxt(RRC) or inst_so_nxt(RRA) or
                    inst_so_nxt(SXTC);

    alu_shift_s <= inst_so_nxt(RRC) or inst_so_nxt(RRA);

    exec_no_wr_s <= inst_to_nxt(CMP) or inst_to_nxt(BITC);

    inst_alu_nxt_s <= exec_no_wr_s & alu_shift_s & alu_stat_f_s & alu_stat_7_s &
                      alu_dadd_s & alu_xor_s & alu_or_s & alu_and_s &
                      alu_add_s & alu_inc_c_s & alu_inc_s & alu_src_inv_s;

    R_1i_2ci : process (mclk_decode, puc_rst)
    begin
      if (puc_rst = '1') then
        inst_alu <= "000000000000";
      elsif (rising_edge(mclk_decode)) then
        if (CLOCK_GATING = '1') then
          inst_alu <= inst_alu_nxt_s;
        elsif (decode_omsp = '1' and CLOCK_GATING = '0') then
          inst_alu <= inst_alu_nxt_s;
        end if;
      end if;
    end process R_1i_2ci;
  end block C6_EXECUTION_UNIT_STATE_FRONTEND_B6;

  SIGNAL_INOUT : block
  begin
    dbg_halt_st  <= dbg_halt_st_omsp;
    decode       <= decode_omsp;
    decode_noirq <= decode_noirq_omsp;
    e_state      <= e_state_omsp;
    exec_done    <= exec_done_omsp;
    i_state      <= i_state_omsp;
    inst_ad      <= inst_ad_omsp;
    inst_as      <= inst_as_omsp;
    inst_irq_rst <= inst_irq_rst_omsp;
    inst_so      <= inst_so_omsp;
    inst_type    <= inst_type_omsp;
    ir           <= ir_omsp;
    irq_detect   <= irq_detect_omsp;
    pc           <= pc_omsp;
    pc_nxt       <= pc_nxt_omsp;
  end block SIGNAL_INOUT;
end FRONTEND_ARQ;
