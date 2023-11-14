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

entity omsp_alu is
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

end omsp_alu;

architecture omsp_alu_ARQ of omsp_alu is

  -- SIGNAL INOUT
  signal alu_out_omsp : std_logic_vector (15 downto 0);

  -- 4.ALU       
  -- 4.1.INSTRUCTION FETCH/DECODE CONTROL STATE MACHINE
  -- Invert source for substract and compare instructions
  signal op_src_inv_cmd : std_logic;
  signal op_src_inv     : std_logic_vector (15 downto 0);

  -- Mask the bit 8 for the Byte instructions for correct flags generation
  signal op_bit8_msk : std_logic;
  signal op_src_in   : std_logic_vector (16 downto 0);
  signal op_dst_in   : std_logic_vector (16 downto 0);

  -- Clear the source operand (= jump offset) for conditional jumps
  signal jmp_not_taken : std_logic;
  signal op_src_in_jmp : std_logic_vector (16 downto 0);

  -- Adder / AND / OR / XOR
  signal alu_add_s : std_logic_vector (16 downto 0);
  signal alu_and_s : std_logic_vector (16 downto 0);
  signal alu_or_s  : std_logic_vector (16 downto 0);
  signal alu_xor_s : std_logic_vector (16 downto 0);

  -- Incrementer
  signal alu_inc_s   : std_logic;
  signal alu_add_inc : std_logic_vector (16 downto 0);

  -- Decimal adder (DADD) 
  signal alu_dadd_s   : std_logic_vector (16 downto 0);
  signal alu_dadd_v_s : std_logic_matrix (3 downto 0)(4 downto 0);

  -- Shifter for rotate instructions (RRC & RRA) 
  signal alu_shift_msb : std_logic;
  signal alu_shift_7_s : std_logic;
  signal alu_shift_s   : std_logic_vector (16 downto 0);

  -- Swap bytes / Extend Sign 
  signal alu_swpb : std_logic_vector (16 downto 0);
  signal alu_sxt  : std_logic_vector (16 downto 0);

  -- Combine short paths toghether to simplify final ALU mux 
  signal alu_short_thro : std_logic;
  signal alu_short      : std_logic_vector (16 downto 0);

  -- ALU output mux
  signal alu_out_nxt : std_logic_vector (16 downto 0);

  -- 4.2.STATUS FLAG GENERATION
  signal V_xor : std_logic;
  signal V     : std_logic;
  signal N     : std_logic;
  signal Z     : std_logic;
  signal C     : std_logic;

  function bcd_add (X : std_logic_vector (3 downto 0);
                    N : std_logic_vector (3 downto 0);
                    C : std_logic) return std_logic_vector is

    variable U : unsigned (4 downto 0);
    variable V : unsigned (4 downto 0);
    variable A : unsigned (4 downto 0);

    variable UVA : std_logic_vector (4 downto 0);

  begin
    U := ('0' & unsigned(X));
    V := ('0' & unsigned(N));
    A := ("0000" & C);

    UVA := std_logic_vector(U+V+A);

    if (UVA < "01010") then
      return UVA;
    else
      return std_logic_vector(unsigned(UVA) + "00110");
    end if;
  end bcd_add;

begin

  -- 4.1.INSTRUCTION FETCH/DECODE CONTROL STATE MACHINE
  -- Invert source for substract and compare instructions.
  op_src_inv_cmd <= exec_cycle and (inst_alu(ALU_SRC_INV));
  op_src_inv     <= (0 to 15 => op_src_inv_cmd) xor op_src;

  -- Mask the bit 8 for the Byte instructions for correct flags generation
  op_bit8_msk <= not exec_cycle or not inst_bw;
  op_src_in   <= '0' & (op_src_inv (15 downto 8) and (8 to 15 => op_bit8_msk)) & op_src_inv (7 downto 0);
  op_dst_in   <= '0' & (op_dst     (15 downto 8) and (8 to 15 => op_bit8_msk)) & op_dst (7 downto 0);

  -- Clear the source operand (= jump offset) for conditional jumps
  jmp_not_taken <= (inst_jmp(JL) and not (status(3) xor status(2))) or
                   (inst_jmp(JGE) and (status(3) xor status(2))) or
                   (inst_jmp(JN) and not status(2)) or
                   (inst_jmp(JC) and not status(0)) or
                   (inst_jmp(JNC) and status(0)) or
                   (inst_jmp(JEQ) and not status(1)) or
                   (inst_jmp(JNE) and status(1));
  op_src_in_jmp <= op_src_in and (0 to 16 => not jmp_not_taken);

  -- Adder / AND / OR / XOR
  alu_add_s <= std_logic_vector(unsigned(op_src_in_jmp) + unsigned(op_dst_in));
  alu_and_s <= op_src_in and op_dst_in;
  alu_or_s  <= op_src_in or op_dst_in;
  alu_xor_s <= op_src_in xor op_dst_in;

  -- Incrementer
  alu_inc_s   <= exec_cycle and ((inst_alu(ALU_INC_C) and status(0)) or inst_alu(ALU_INC));
  alu_add_inc <= std_logic_vector(unsigned(alu_add_s) + ((1 to 16 => '0') & alu_inc_s));

  -- Decimal adder (DADD)
  alu_dadd_v_s(0) <= bcd_add(op_src_in(03 downto 00), op_dst_in(03 downto 00), status(0));
  alu_dadd_v_s(1) <= bcd_add(op_src_in(07 downto 04), op_dst_in(07 downto 04), alu_dadd_v_s(0)(4));
  alu_dadd_v_s(2) <= bcd_add(op_src_in(11 downto 08), op_dst_in(11 downto 08), alu_dadd_v_s(1)(4));
  alu_dadd_v_s(3) <= bcd_add(op_src_in(15 downto 12), op_dst_in(15 downto 12), alu_dadd_v_s(2)(4));

  alu_dadd_s <= (alu_dadd_v_s(3) & alu_dadd_v_s(2)(3 downto 0) & alu_dadd_v_s(1)(3 downto 0) & alu_dadd_v_s(0)(3 downto 0));

  -- Shifter for rotate instructions (RRC AND RRA)
  alu_shift_msb <= status(0)
                   when inst_so(RRC) = '1' else op_src(07)
                   when inst_bw = '1'      else op_src(15);
  alu_shift_7_s <= alu_shift_msb
                   when inst_bw = '1' else op_src(08);

  alu_shift_s <= ('0' & alu_shift_msb & op_src(15 downto 9) & alu_shift_7_s & op_src(7 downto 1));

  -- Swap bytes / Extend Sign
  alu_swpb <= ('0' & op_src(7 downto 0) & op_src(15 downto 8));
  alu_sxt  <= ('0' & (8 to 15 => op_src(7)) & op_src(07 downto 0));

  -- Combine short paths toghether to simplify final ALU mux
  alu_short_thro <= not (inst_alu(ALU_AND) or inst_alu(ALU_OR) or inst_alu(ALU_XOR) or
                         inst_alu(ALU_SHIFT) or inst_so(SWPB) or inst_so(SXTC));

  alu_short <= ((0 to 16 => inst_alu(ALU_AND)) and alu_and_s) or
               ((0 to 16 => inst_alu(ALU_OR)) and alu_or_s) or
               ((0 to 16 => inst_alu(ALU_XOR)) and alu_xor_s) or
               ((0 to 16 => inst_alu(ALU_SHIFT)) and alu_shift_s) or
               ((0 to 16 => inst_so(SWPB)) and alu_swpb) or
               ((0 to 16 => inst_so(SXTC)) and alu_sxt) or
               ((0 to 16 => alu_short_thro) and op_src_in);

  -- ALU output mux
  alu_out_nxt <= alu_add_inc
                 when (inst_so(IRQX) or dbg_halt_st or inst_alu(ALU_ADD)) = '1' else alu_dadd_s
                 when inst_alu(ALU_DADD) = '1'                                  else alu_short;

  alu_out_omsp <= alu_out_nxt (15 downto 0);
  alu_out_add  <= alu_add_s (15 downto 0);
  alu_out      <= alu_out_omsp;

  -- 4.2.STATUS FLAG GENERATION
  V_xor <= (op_src_in(7) and op_dst_in(7)) when inst_bw = '1' else (op_src_in(15) and op_dst_in(15));

  V <= ((not op_src_in(07) and not op_dst_in(07) and alu_out_omsp(07)) or
        (op_src_in(07) and op_dst_in(07) and not alu_out_omsp(07)))
       when inst_bw = '1' else
       ((not op_src_in(15) and not op_dst_in(15) and alu_out_omsp(15)) or
        (op_src_in(15) and op_dst_in(15) and not alu_out_omsp(15)));

  N <= alu_out_omsp(7)
       when inst_bw = '1' else alu_out_omsp(15);

  Z <= to_stdlogic(alu_out_omsp(7 downto 0) = X"00")
       when inst_bw = '1' else to_stdlogic(alu_out_omsp = X"0000");

  C <= alu_out_omsp(8)
       when inst_bw = '1' else alu_out_nxt(16);

  alu_stat <= ('0' & N & Z & op_src_in(0)) when inst_alu(ALU_SHIFT) = '1'  else
              ('0' & N & Z & not Z)        when inst_alu(ALU_STAT_7) = '1' else
              (V_xor & N & Z & not Z)      when inst_alu(ALU_XOR) = '1'    else (V & N & Z & C);

  alu_stat_wr <= "1111" when (inst_alu(ALU_STAT_F) and exec_cycle) = '1' else "0000";
end omsp_alu_ARQ;
