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
use WORK.MSP430_PACK .all;

entity MULTIPLIER is
  port (
    scan_enable : in std_logic;

    per_dout : out std_logic_vector (15 downto 0);
    mclk     : in  std_logic;
    per_en   : in  std_logic;
    puc_rst  : in  std_logic;
    per_we   : in  std_logic_vector (1 downto 0);
    per_addr : in  std_logic_vector (13 downto 0);
    per_din  : in  std_logic_vector (15 downto 0));
end MULTIPLIER;

architecture MULTIPLIER_ARQ of MULTIPLIER is

  --0.          PARAMETER_DECLARATION
  --0.1.                Register base address (must be aligned to decoder bit width)
  constant BASE_ADDR_M : std_logic_vector (14 downto 0) := "000000100110000";

  --0.2.                Decoder bit width (defines how many bits are considered for address decoding)
  constant DEC_WD_M : integer := 4;

  --0.3.                Register addresses offset
  constant OP1_MPY  : integer := 0;
  constant OP1_MPYS : integer := 2;
  constant OP1_MAC  : integer := 4;
  constant OP1_MACS : integer := 6;
  constant OP2C     : integer := 8;
  constant RESLOC   : integer := 10;
  constant RESHIC   : integer := 12;
  constant SUMEXTC  : integer := 14;

  constant OP1_MPYB  : std_logic_vector (DEC_WD_M - 1 downto 0) := std_logic_vector(to_unsigned(OP1_MPY, DEC_WD_M));
  constant OP1_MPYSB : std_logic_vector (DEC_WD_M - 1 downto 0) := std_logic_vector(to_unsigned(OP1_MPYS, DEC_WD_M));
  constant OP1_MACB  : std_logic_vector (DEC_WD_M - 1 downto 0) := std_logic_vector(to_unsigned(OP1_MAC, DEC_WD_M));
  constant OP1_MACSB : std_logic_vector (DEC_WD_M - 1 downto 0) := std_logic_vector(to_unsigned(OP1_MACS, DEC_WD_M));
  constant OP2CB     : std_logic_vector (DEC_WD_M - 1 downto 0) := std_logic_vector(to_unsigned(OP2C, DEC_WD_M));
  constant RESLOCB   : std_logic_vector (DEC_WD_M - 1 downto 0) := std_logic_vector(to_unsigned(RESLOC, DEC_WD_M));
  constant RESHICB   : std_logic_vector (DEC_WD_M - 1 downto 0) := std_logic_vector(to_unsigned(RESHIC, DEC_WD_M));
  constant SUMEXTCB  : std_logic_vector (DEC_WD_M - 1 downto 0) := std_logic_vector(to_unsigned(SUMEXTC, DEC_WD_M));

  --0.4.                Register one-hot decoder utilities
  constant DEC_SZ_M   : integer                                   := 2**DEC_WD_M;
  constant BASE_REG_M : std_logic_vector (DEC_SZ_M - 1 downto 0) := std_logic_vector(to_unsigned(1, DEC_SZ_M));

  --0.5.                Register one-hot decoder
  constant OP1_MPY_D  : std_logic_vector (DEC_SZ_M - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_M) sll OP1_MPY);
  constant OP1_MPYS_D : std_logic_vector (DEC_SZ_M - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_M) sll OP1_MPYS);
  constant OP1_MAC_D  : std_logic_vector (DEC_SZ_M - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_M) sll OP1_MAC);
  constant OP1_MACS_D : std_logic_vector (DEC_SZ_M - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_M) sll OP1_MACS);
  constant OP2C_D     : std_logic_vector (DEC_SZ_M - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_M) sll OP2C);
  constant RESLOC_D   : std_logic_vector (DEC_SZ_M - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_M) sll RESLOC);
  constant RESHIC_D   : std_logic_vector (DEC_SZ_M - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_M) sll RESHIC);
  constant SUMEXTC_D  : std_logic_vector (DEC_SZ_M - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_M) sll SUMEXTC);

  --0.6.        Wire pre-declarations
  signal result_wr  : std_logic;
  signal result_clr : std_logic;
  signal early_read : std_logic;

  --1.  REGISTER_DECODER
  --1.1.        Local register selection
  signal reg_sel_m : std_logic;

  --1.2.        Register local address
  signal reg_addr_m : std_logic_vector (DEC_WD_M - 1 downto 0);

  --1.3.        Register address decode
  signal reg_dec_m : std_logic_vector (DEC_SZ_M - 1 downto 0);

  --1.4.        Read/Write probes
  signal reg_write_m : std_logic;
  signal reg_read_m  : std_logic;

  --1.5.        Read/Write vectors
  signal reg_wr_m : std_logic_vector (DEC_SZ_M - 1 downto 0);
  signal reg_rd_m : std_logic_vector (DEC_SZ_M - 1 downto 0);

  --1.6.        Masked input data for byte access
  signal per_din_msk : std_logic_vector (15 downto 0);

  --2.  REGISTERS
  --2.1.        OP1 Register    
  signal op_wr   : std_logic_vector (1 downto 0);
  signal mclk_op : std_logic_vector (1 downto 0);
  signal op      : std_logic_matrix (1 downto 0)(15 downto 0);
  signal op_rd   : std_logic_matrix (1 downto 0)(15 downto 0);

  --2.2.        OP2C Register

  --2.3.        RESLOC Register
  signal reslo_en   : std_logic;
  signal reslo_wr   : std_logic;
  signal mclk_reslo : std_logic;
  signal reslo      : std_logic_vector (15 downto 0);
  signal reslo_nxt  : std_logic_vector (15 downto 0);
  signal reslo_rd   : std_logic_vector (15 downto 0);

  --2.4.        RESHIC Register
  signal reshi_en   : std_logic;
  signal reshi_wr   : std_logic;
  signal mclk_reshi : std_logic;
  signal reshi      : std_logic_vector (15 downto 0);

  signal reshi_nxt : std_logic_vector (15 downto 0);
  signal reshi_rd  : std_logic_vector (15 downto 0);

  --2.5.        SUMEXTC Register
  signal sumext_s     : std_logic_vector (1 downto 0);
  signal sumext_s_nxt : std_logic_vector (1 downto 0);
  signal sumext_nxt   : std_logic_vector (15 downto 0);
  signal sumext       : std_logic_vector (15 downto 0);
  signal sumext_rd    : std_logic_vector (15 downto 0);

  --3.  DATA_OUTPUT_GENERATION
  --3.1.        Data output mux
  signal op1_mux    : std_logic_vector (15 downto 0);
  signal op2_mux    : std_logic_vector (15 downto 0);
  signal reslo_mux  : std_logic_vector (15 downto 0);
  signal reshi_mux  : std_logic_vector (15 downto 0);
  signal sumext_mux : std_logic_vector (15 downto 0);

  --4.  HARDWARE_MULTIPLIER_FUNCTIONAL_LOGIC
  --4.1.        Multiplier configuration        
  --Detect signed mode
  signal sign_sel : std_logic;

  --Detect accumulate mode
  signal acc_sel : std_logic;

  --Detect whenever the RESHIC and RESLOC registers should be cleared

  --Combine RESHIC & RESLOC
  signal result : std_logic_vector (31 downto 0);

  --4.2.        16x16 Multiplier (result computed in 1 clock cycle1)
  --Detect start of a multiplication
  signal cycle1 : std_logic;

  --Expand the operands to support signed & unsigned operations
  signal op1_xp : std_logic_vector (16 downto 0);
  signal op2_xp : std_logic_vector (16 downto 0);

  --17x17 signed multiplication
  signal product1 : std_logic_vector (33 downto 0);

  --Accumulate
  signal result_nxt : std_logic_vector (32 downto 0);

  --Next register values        
  --Since the MAC is completed within 1 clock cycle1, an early read can't happen

  --4.3.        16x8 Multiplier (result computed in 2 clock cycle1s)
  --Detect start of a multiplication
  signal cycle2 : std_logic_vector (1 downto 0);

  --Expand the operands to support signed & unsigned operations
  signal op2_hi_xp : std_logic_vector (8 downto 0);
  signal op2_lo_xp : std_logic_vector (8 downto 0);
  signal op2_xp9   : std_logic_vector (8 downto 0);

  --17x9 signed multiplication
  signal product2   : std_logic_vector (25 downto 0);
  signal product_xp : std_logic_vector (31 downto 0);

  --Accumulate
  --Next register values
  --Since the MAC is completed within 2 clock cycle1, an early read can happen during the second cycle1

begin
  REGISTER_DECODER : block
  begin
    --1.1.      Local register selection
    reg_sel_m <= per_en and to_stdlogic(per_addr(13 downto DEC_WD_M - 1) = BASE_ADDR_M(14 downto DEC_WD_M));

    --1.2.      Register local address
    reg_addr_m <= per_addr(DEC_WD_M - 2 downto 0) & '0';

    --1.3.      Register address decode
    reg_dec_m <= (OP1_MPY_D and (0 to DEC_SZ_M - 1 => to_stdlogic(reg_addr_m = OP1_MPYB))) or
                 (OP1_MPYS_D and (0 to DEC_SZ_M - 1 => to_stdlogic(reg_addr_m = OP1_MPYSB))) or
                 (OP1_MAC_D and (0 to DEC_SZ_M - 1  => to_stdlogic(reg_addr_m = OP1_MACB))) or
                 (OP1_MACS_D and (0 to DEC_SZ_M - 1 => to_stdlogic(reg_addr_m = OP1_MACSB))) or
                 (OP2C_D and (0 to DEC_SZ_M - 1     => to_stdlogic(reg_addr_m = OP2CB))) or
                 (RESLOC_D and (0 to DEC_SZ_M - 1   => to_stdlogic(reg_addr_m = RESLOCB))) or
                 (RESHIC_D and (0 to DEC_SZ_M - 1   => to_stdlogic(reg_addr_m = RESHICB))) or
                 (SUMEXTC_D and (0 to DEC_SZ_M - 1  => to_stdlogic(reg_addr_m = SUMEXTCB)));

    --1.4.      Read/Write probes
    reg_write_m <= reduce_or(per_we) and reg_sel_m;
    reg_read_m  <= not reduce_or(per_we) and reg_sel_m;

    --1.5.      Read/Write vectors
    reg_wr_m <= reg_dec_m and (0 to DEC_SZ_M - 1 => reg_write_m);
    reg_rd_m <= reg_dec_m and (0 to DEC_SZ_M - 1 => reg_read_m);

    --1.6.      Masked input data for byte access
    per_din_msk <= per_din and (8 to 15 => per_we(1), 0 to 7 => '1');
  end block REGISTER_DECODER;

  REGISTERS : block
  begin
    --2.1.      OP1 Register
    op_wr(0) <= reg_wr_m(OP1_MPY) or reg_wr_m(OP1_MPYS) or reg_wr_m(OP1_MAC) or reg_wr_m(OP1_MACS);

    clock_gating_1_on : if (CLOCK_GATING = '1') generate
      clock_gate_op1 : omsp_clock_gate
        port map (
          gclk        => mclk_op(0),
          clk         => mclk,
          enable      => op_wr(0),
          scan_enable => scan_enable);
    end generate clock_gating_1_on;

    clock_gating_1_off : if (CLOCK_GATING = '0') generate
      mclk_op(0) <= mclk;
    end generate clock_gating_1_off;

    R1_1i_2ci : process (mclk_op(0), puc_rst)
    begin
      if (puc_rst = '1') then
        op(0) <= X"0000";
      elsif (rising_edge(mclk_op(0))) then
        if (CLOCK_GATING = '1') then
          op(0) <= per_din_msk;
        elsif (op_wr(0) = '1' and CLOCK_GATING = '0') then
          op(0) <= per_din_msk;
        end if;
      end if;
    end process R1_1i_2ci;

    op_rd(0) <= op(0);

    --2.2.      OP2C Register
    op_wr(1) <= reg_wr_m(OP2C);

    clock_gating_2_on : if (CLOCK_GATING = '1') generate
      clock_gate_op2 : omsp_clock_gate
        port map (
          gclk        => mclk_op(1),
          clk         => mclk,
          enable      => op_wr(1),
          scan_enable => scan_enable);
    end generate clock_gating_2_on;

    clock_gating_2_off : if (CLOCK_GATING = '0') generate
      mclk_op(1) <= mclk;
    end generate clock_gating_2_off;

    R2_1i_2ci : process (mclk_op(1), puc_rst)
    begin
      if (puc_rst = '1') then
        op(1) <= X"0000";
      elsif (rising_edge(mclk_op(1))) then
        if (CLOCK_GATING = '1') then
          op(1) <= per_din_msk;
        elsif (op_wr(1) = '1' and CLOCK_GATING = '0') then
          op(1) <= per_din_msk;
        end if;
      end if;
    end process R2_1i_2ci;

    op_rd(1) <= op(1);

    --2.3.      RESLO Register
    reslo_wr <= reg_wr_m(RESLOC);

    clock_gating_3_on : if (CLOCK_GATING = '1') generate
      reslo_en <= reslo_wr or result_clr or result_wr;

      clock_gate_reslo : omsp_clock_gate
        port map (
          gclk        => mclk_reslo,
          clk         => mclk,
          enable      => reslo_en,
          scan_enable => scan_enable);
    end generate clock_gating_3_on;

    clock_gating_3_off : if (CLOCK_GATING = '0') generate
      mclk_reslo <= mclk;
    end generate clock_gating_3_off;

    R1_1c_2c_3i_4ci : process (mclk_reslo, puc_rst)
    begin
      if (puc_rst = '1') then
        reslo <= X"0000";
      elsif (rising_edge(mclk_reslo)) then
        if (reslo_wr = '1') then
          reslo <= per_din_msk;
        elsif (result_clr = '1') then
          reslo <= X"0000";
        elsif (CLOCK_GATING = '1') then
          reslo <= reslo_nxt;
        elsif (result_wr = '1' and CLOCK_GATING = '0') then
          reslo <= reslo_nxt;
        end if;
      end if;
    end process R1_1c_2c_3i_4ci;

    reslo_rd <= reslo_nxt when early_read = '1' else reslo;

    --2.4.      RESHI Register
    reshi_wr <= reg_wr_m(RESHIC);

    clock_gating_4_on : if (CLOCK_GATING = '1') generate
      reshi_en <= reshi_wr or result_clr or result_wr;

      clock_gate_reshi : omsp_clock_gate
        port map (
          gclk        => mclk_reshi,
          clk         => mclk,
          enable      => reshi_en,
          scan_enable => scan_enable);
    end generate clock_gating_4_on;

    clock_gating_4_off : if (CLOCK_GATING = '0') generate
      mclk_reshi <= mclk;
    end generate clock_gating_4_off;

    R2_1c_2c_3i_4ci : process (mclk_reshi, puc_rst)
    begin
      if (puc_rst = '1') then
        reshi <= X"0000";
      elsif (rising_edge(mclk_reshi)) then
        if (reshi_wr = '1') then
          reshi <= per_din_msk;
        elsif (result_clr = '1') then
          reshi <= X"0000";
        elsif (CLOCK_GATING = '1') then
          reshi <= reshi_nxt;
        elsif (result_wr = '1' and CLOCK_GATING = '0') then
          reshi <= reshi_nxt;
        end if;
      end if;
    end process R2_1c_2c_3i_4ci;

    reshi_rd <= reshi_nxt when early_read = '1' else reshi;

    --2.5.      SUMEXTC Register
    R_1c_2c : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        sumext_s <= "00";
      elsif (rising_edge(mclk)) then
        if (op_wr(1) = '1') then
          sumext_s <= "00";
        elsif (result_wr = '1') then
          sumext_s <= sumext_s_nxt;
        end if;
      end if;
    end process R_1c_2c;

    sumext_nxt <= (2 to 15 => sumext_s_nxt (1)) & sumext_s_nxt;
    sumext     <= (2 to 15 => sumext_s (1)) & sumext_s;
    sumext_rd  <= sumext_nxt when early_read = '1' else sumext;
  end block REGISTERS;

  DATA_OUTPUT_GENERATION : block
  begin
    --3.1.      Data output mux
    op1_mux    <= op_rd(0) and (0 to 15  => reg_rd_m(OP1_MPY) or reg_rd_m(OP1_MPYS) or reg_rd_m(OP1_MAC) or reg_rd_m(OP1_MACS));
    op2_mux    <= op_rd(1) and (0 to 15  => reg_rd_m(OP2C));
    reslo_mux  <= reslo_rd and (0 to 15  => reg_rd_m(RESLOC));
    reshi_mux  <= reshi_rd and (0 to 15  => reg_rd_m(RESHIC));
    sumext_mux <= sumext_rd and (0 to 15 => reg_rd_m(SUMEXTC));
    per_dout   <= op1_mux or op2_mux or reslo_mux or reshi_mux or sumext_mux;
  end block DATA_OUTPUT_GENERATION;

  HARDWARE_MULTIPLIER_FUNCTIONAL_LOGIC : block
  begin
    --4.1.      Multiplier configuration
    --Detect signed mode
    R1_1i_2ci_e : process (mclk_op(0), puc_rst)
    begin
      if (puc_rst = '1') then
        sign_sel <= '0';
      elsif (rising_edge(mclk_op(0))) then
        if (CLOCK_GATING = '1') then
          sign_sel <= reg_wr_m(OP1_MPYS) or reg_wr_m(OP1_MACS);
        elsif (op_wr(0) = '1' and CLOCK_GATING = '0') then
          sign_sel <= reg_wr_m(OP1_MPYS) or reg_wr_m(OP1_MACS);
        end if;
      end if;
    end process R1_1i_2ci_e;

    --Detect accumulate mode
    R2_1i_2ci_e : process (mclk_op(0), puc_rst)
    begin
      if (puc_rst = '1') then
        acc_sel <= '0';
      elsif (rising_edge(mclk_op(0))) then
        if (CLOCK_GATING = '1') then
          acc_sel <= reg_wr_m(OP1_MAC) or reg_wr_m(OP1_MACS);
        elsif (op_wr(0) = '1' and CLOCK_GATING = '0') then
          acc_sel <= reg_wr_m(OP1_MAC) or reg_wr_m(OP1_MACS);
        end if;
      end if;
    end process R2_1i_2ci_e;

    --Detect whenever the RESHIC and RESLOC registers should be cleared
    result_clr <= op_wr(1) and not acc_sel;

    --Combine RESHIC & RESLOC
    result <= reshi & reslo;

    --4.2.      16x16 Multiplier (result computed in 1 clock cycle)
    mpy_16x16_on : if (MPY_16x16 = '1') generate

      --Detect start of a multiplication
      R_1_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          cycle1 <= '0';
        elsif (rising_edge(mclk)) then
          cycle1 <= op_wr(1);
        end if;
      end process R_1_e;

      result_wr <= cycle1;

      --Expand the operands to support signed & unsigned operations
      op1_xp <= (sign_sel and op(0)(15)) & op(0);
      op2_xp <= (sign_sel and op(1)(15)) & op(1);

      --17x17 signed multiplication
      product1 <= std_logic_vector(signed(op1_xp) * signed(op2_xp));

      --Accumulate
      result_nxt <= std_logic_vector(('0' & unsigned(result)) + ('0' & unsigned(product1(31 downto 0))));

      --Next register values
      reslo_nxt    <= result_nxt(15 downto 0);
      reshi_nxt    <= result_nxt(31 downto 16);
      sumext_s_nxt <= (others => result_nxt(31)) when (sign_sel = '1') else ('0' & result_nxt(32));

      --Since the MAC is completed within 1 clock cycle1, an early read can't happen
      early_read <= '0';
    end generate mpy_16x16_on;

    --4.3.      16x8 Multiplier (result computed in 2 clock cycles)
    mpy_16x16_off : if (MPY_16x16 = '0') generate

      --Detect start of a multiplication
      R_1 : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          cycle2 <= "00";
        elsif (rising_edge(mclk)) then
          cycle2 <= cycle2(0) & op_wr(1);
        end if;
      end process R_1;

      result_wr <= reduce_or(cycle2);

      --Expand the operands to support signed & unsigned operations
      op1_xp    <= (sign_sel and op(0)(15)) & op(0);
      op2_hi_xp <= (sign_sel and op(1)(15)) & op(1)(15 downto 8);
      op2_lo_xp <= '0' & op(1)(7 downto 0);
      op2_xp9   <= op2_hi_xp when cycle2(0) = '1' else op2_lo_xp;

      --17x9 signed multiplication
      product2   <= std_logic_vector(signed(op1_xp) * signed(op2_xp9));
      product_xp <= product2(23 downto 0) & X"00"
                    when cycle2(0) = '1' else (31 downto 24 => (sign_sel and product2(23))) & product2(23 downto 0);

      --Accumulate
      result_nxt <= std_logic_vector(('0' & unsigned(result)) + ('0' & unsigned(product_xp(31 downto 0))));

      --Next register values
      reslo_nxt    <= result_nxt(15 downto 0);
      reshi_nxt    <= result_nxt(31 downto 16);
      sumext_s_nxt <= (others => result_nxt(31)) when sign_sel = '1' else ('0' & (result_nxt(32) or sumext_s(0)));

      --Since the MAC is completed within 2 clock cycle1, an early read can happen during the second cycle1     
      early_read <= cycle2(1);
    end generate mpy_16x16_off;
  end block HARDWARE_MULTIPLIER_FUNCTIONAL_LOGIC;
end MULTIPLIER_ARQ;
