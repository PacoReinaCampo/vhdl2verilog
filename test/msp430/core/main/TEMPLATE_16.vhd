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

entity TEMPLATE_16 is
  port (
    per_dout   : out std_logic_vector (15 downto 0);
    cntrl2_16b : out std_logic_vector (15 downto 0);
    cntrl4_16b : out std_logic_vector (15 downto 0);

    mclk     : in std_logic;
    per_en   : in std_logic;
    puc_rst  : in std_logic;
    per_we   : in std_logic_vector (1 downto 0);
    per_addr : in std_logic_vector (13 downto 0);
    per_din  : in std_logic_vector (15 downto 0));
end TEMPLATE_16;

architecture TEMPLATE_16_ARQ of TEMPLATE_16 is

  constant SIZE_P16 : integer := 4;

  -- 0.  PARAMETER_DECLARATION
  -- 0.1.        Register base address (must be aligned to decoder bit width)
  constant BASE_ADDR_P16 : std_logic_vector (14 downto 0) := "000000110010000";

  -- 0.2.        Decoder bit width (defines how many bits are considered for address decoding)
  constant DEC_WD_P16 : integer := 3;

  -- 0.3.        Register addresses offset
  constant CNTRL1B16 : std_logic_vector (DEC_WD_P16 - 1 downto 0) := std_logic_vector(to_unsigned(0, DEC_WD_P16));
  constant CNTRL2B16 : std_logic_vector (DEC_WD_P16 - 1 downto 0) := std_logic_vector(to_unsigned(2, DEC_WD_P16));
  constant CNTRL3B16 : std_logic_vector (DEC_WD_P16 - 1 downto 0) := std_logic_vector(to_unsigned(4, DEC_WD_P16));
  constant CNTRL4B16 : std_logic_vector (DEC_WD_P16 - 1 downto 0) := std_logic_vector(to_unsigned(6, DEC_WD_P16));

  -- 0.4.        Register one-hot decoder utilities
  constant DEC_SZ_P16 : integer := 2**DEC_WD_P16;

  constant BASE_REG_P16 : std_logic_vector (DEC_SZ_P16 - 1 downto 0) := std_logic_vector(to_unsigned(1, DEC_SZ_P16));

  -- 0.5.        Register one-hot decoder
  constant CNTRL1_D16 : std_logic_vector (DEC_SZ_P16 - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_P16) sll to_integer(unsigned(CNTRL1B16)));
  constant CNTRL2_D16 : std_logic_vector (DEC_SZ_P16 - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_P16) sll to_integer(unsigned(CNTRL2B16)));
  constant CNTRL3_D16 : std_logic_vector (DEC_SZ_P16 - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_P16) sll to_integer(unsigned(CNTRL3B16)));
  constant CNTRL4_D16 : std_logic_vector (DEC_SZ_P16 - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_P16) sll to_integer(unsigned(CNTRL4B16)));

  type M_SIZE_P16_I is array (SIZE_P16 - 1 downto 0) of integer;

  constant CNTRLB16 : std_logic_matrix (SIZE_P16 - 1 downto 0)(DEC_WD_P16 - 1 downto 0) := (CNTRL4B16,
                                                                                            CNTRL3B16,
                                                                                            CNTRL2B16,
                                                                                            CNTRL1B16);

  constant CNTRLI16 : M_SIZE_P16_I := (to_integer(unsigned(CNTRL4B16)),
                                       to_integer(unsigned(CNTRL3B16)),
                                       to_integer(unsigned(CNTRL2B16)),
                                       to_integer(unsigned(CNTRL1B16)));

  constant CNTRL_D16 : std_logic_matrix (SIZE_P16 - 1 downto 0)(DEC_SZ_P16 - 1 downto 0) := (CNTRL4_D16,
                                                                                             CNTRL3_D16,
                                                                                             CNTRL2_D16,
                                                                                             CNTRL1_D16);

  -- 1.  REGISTER_DECODER
  -- 1.1.        Local register selection
  signal reg_sel_p16 : std_logic;

  -- 1.2.        Register local address
  signal reg_addr_p16 : std_logic_vector (DEC_WD_P16 - 1 downto 0);

  -- 1.3.        Register address decode
  signal reg_dec_p16 : std_logic_vector (DEC_SZ_P16 - 1 downto 0);

  -- 1.4.        Read/Write probes
  signal reg_write_p : std_logic;

  signal reg_read_p16 : std_logic;

  -- 1.5.        Read/Write vectors
  signal reg_wr_p : std_logic_vector (DEC_SZ_P16 - 1 downto 0);

  signal reg_rd_p16 : std_logic_vector (DEC_SZ_P16 - 1 downto 0);

  -- 2.  REGISTERS       
  signal cntrl_wr_p16 : std_logic_vector (SIZE_P16 - 1 downto 0);

  signal cntrl_p16 : std_logic_matrix (SIZE_P16 - 1 downto 0)(15 downto 0);

  -- 4.  DATA_OUTPUT_GENERATION
  -- 4.1.        Data output mux
  signal cntrl_rd16 : std_logic_matrix (SIZE_P16 - 1 downto 0)(15 downto 0);

  function matrixAP_or (matrix : std_logic_matrix) return std_logic_vector is
    variable RESULT : std_logic_vector (15 downto 0) := (others => '0');
  begin
    for i in 0 to SIZE_P16-1 loop
      RESULT := RESULT or matrix(i);
    end loop;
    return RESULT;
  end matrixAP_or;

  function matrixBP_or (matrix : std_logic_matrix) return std_logic_vector is
    variable RESULT : std_logic_vector (DEC_SZ_P16 - 1 downto 0) := (others => '0');
  begin
    for i in 0 to SIZE_P16-1 loop
      RESULT := RESULT or matrix(i);
    end loop;
    return RESULT;
  end matrixBP_or;

begin
  REGISTER_DECODER : block
  begin
    -- 1.1.      Local register selection
    reg_sel_p16 <= per_en and
                   to_stdlogic(per_addr(13 downto DEC_WD_P16 - 1) = BASE_ADDR_P16(14 downto DEC_WD_P16));

    -- 1.2.      Register local address
    reg_addr_p16 <= per_addr(DEC_WD_P16 - 2 downto 0) & '0';

    -- 1.3.      Register address decode 
    address_decode : process (reg_addr_p16)
      variable decode : std_logic_matrix (SIZE_P16 - 1 downto 0)(DEC_SZ_P16 - 1 downto 0);
    begin
      for i in SIZE_P16 - 1 downto 0 loop
        decode(i) := CNTRL_D16(i) and (0 to DEC_SZ_P16 - 1 =>
                                       to_stdlogic(reg_addr_p16 = CNTRLB16(i)));
      end loop;

      reg_dec_p16 <= matrixBP_or(decode);
    end process address_decode;

    -- 1.4.      Read/Write probes
    reg_write_p <= reduce_or(per_we) and reg_sel_p16;

    reg_read_p16 <= not reduce_or(per_we) and reg_sel_p16;

    -- 1.5.      Read/Write vectors
    reg_wr_p <= reg_dec_p16 and (0 to DEC_SZ_P16 - 1 => reg_write_p);

    reg_rd_p16 <= reg_dec_p16 and (0 to DEC_SZ_P16 - 1 => reg_read_p16);
  end block REGISTER_DECODER;

  REGISTERS : block
  begin
    PCNTRL_Register : for i in SIZE_P16 - 1 downto 0 generate
      cntrl_wr_p16(i) <= reg_wr_p(CNTRLI16(i));

      R_1c_1 : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          cntrl_p16(i) <= X"0000";
        elsif (rising_edge(mclk)) then
          if (cntrl_wr_p16(i) = '1') then
            cntrl_p16(i) <= per_din;
          end if;
        end if;
      end process R_1c_1;
    end generate PCNTRL_Register;
  end block REGISTERS;

  DATA_OUTPUT_GENERATION : block
  begin
    data_output_mux : for i in SIZE_P16 - 1 downto 0 generate
      cntrl_rd16 (i) <= cntrl_p16(i) and (0 to 15 => reg_rd_p16(CNTRLI16(i)));
    end generate data_output_mux;

    per_dout   <= matrixAP_or(cntrl_rd16);
    cntrl2_16b <= cntrl_p16(1);
    cntrl4_16b <= cntrl_p16(3);
  end block DATA_OUTPUT_GENERATION;
end TEMPLATE_16_ARQ;
