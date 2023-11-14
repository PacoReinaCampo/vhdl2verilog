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

entity TEMPLATE_08 is
  port (
    per_dout : out std_logic_vector (15 downto 0);

    mclk     : in std_logic;
    per_en   : in std_logic;
    puc_rst  : in std_logic;
    per_we   : in std_logic_vector (1 downto 0);
    per_addr : in std_logic_vector (13 downto 0);
    per_din  : in std_logic_vector (15 downto 0));
end TEMPLATE_08;

architecture TEMPLATE_08_ARQ of TEMPLATE_08 is

  constant SIZE_P08 : integer := 4;

  -- 0.  PARAMETER_DECLARATION
  -- 0.1.        Register base address (must be aligned to decoder bit width)
  constant BASE_ADDR_P08 : std_logic_vector (14 downto 0) := "000000010010000";

  -- 0.2.        Decoder bit width (defines how many bits are considered for address decoding)
  constant DEC_WD_P08 : integer := 2;

  -- 0.3.        Register addresses offset
  constant CNTRL1B08 : std_logic_vector (DEC_WD_P08 - 1 downto 0) := std_logic_vector(to_unsigned(0, DEC_WD_P08));
  constant CNTRL2B08 : std_logic_vector (DEC_WD_P08 - 1 downto 0) := std_logic_vector(to_unsigned(1, DEC_WD_P08));
  constant CNTRL3B08 : std_logic_vector (DEC_WD_P08 - 1 downto 0) := std_logic_vector(to_unsigned(2, DEC_WD_P08));
  constant CNTRL4B08 : std_logic_vector (DEC_WD_P08 - 1 downto 0) := std_logic_vector(to_unsigned(3, DEC_WD_P08));

  -- 0.4.        Register one-hot decoder utilities
  constant DEC_SZ_P08 : integer := 2**DEC_WD_P08;

  constant BASE_REG_P08 : std_logic_vector (0 to DEC_SZ_P08 - 1) := std_logic_vector(to_unsigned(1, DEC_SZ_P08));

  -- 0.5.        Register one-hot decoder
  constant CNTRL1_D08 : std_logic_vector (0 to DEC_SZ_P08 - 1) := std_logic_vector(unsigned(BASE_REG_P08) sll to_integer(unsigned(CNTRL1B08)));
  constant CNTRL2_D08 : std_logic_vector (0 to DEC_SZ_P08 - 1) := std_logic_vector(unsigned(BASE_REG_P08) sll to_integer(unsigned(CNTRL2B08)));
  constant CNTRL3_D08 : std_logic_vector (0 to DEC_SZ_P08 - 1) := std_logic_vector(unsigned(BASE_REG_P08) sll to_integer(unsigned(CNTRL3B08)));
  constant CNTRL4_D08 : std_logic_vector (0 to DEC_SZ_P08 - 1) := std_logic_vector(unsigned(BASE_REG_P08) sll to_integer(unsigned(CNTRL4B08)));

  type M_SIZE_P081_I is array (SIZE_P08 - 1 downto 0) of integer;

  constant CNTRLB08 : std_logic_matrix (SIZE_P08 - 1 downto 0)(DEC_WD_P08 - 1 downto 0) := (CNTRL4B08,
                                                                                            CNTRL3B08,
                                                                                            CNTRL2B08,
                                                                                            CNTRL1B08);

  constant CNTRLI08 : M_SIZE_P081_I := (to_integer(unsigned(CNTRL4B08)),
                                        to_integer(unsigned(CNTRL3B08)),
                                        to_integer(unsigned(CNTRL2B08)),
                                        to_integer(unsigned(CNTRL1B08)));

  constant CNTRL_D08 : std_logic_matrix (SIZE_P08 - 1 downto 0)(0 to DEC_SZ_P08 - 1) := (CNTRL4_D08,
                                                                                         CNTRL3_D08,
                                                                                         CNTRL2_D08,
                                                                                         CNTRL1_D08);

  -- 1.  REGISTER_DECODER
  -- 1.1.        Local register selection
  signal reg_sel_p08 : std_logic;

  -- 1.2.        Register local address
  signal reg_addr_p08 : std_logic_vector (DEC_WD_P08 - 1 downto 0);

  -- 1.3.        Register address decode
  signal reg_dec_p08 : std_logic_vector (0 to DEC_SZ_P08 - 1);

  -- 1.4.        Read/Write probes
  signal reg_lo_write_p : std_logic;
  signal reg_hi_write_p : std_logic;
  signal reg_read_p08   : std_logic;

  -- 1.5.        Read/Write vectors
  signal reg_lo_wr_p : std_logic_vector (0 to DEC_SZ_P08 - 1);
  signal reg_hi_wr_p : std_logic_vector (0 to DEC_SZ_P08 - 1);
  signal reg_rd_p08  : std_logic_vector (0 to DEC_SZ_P08 - 1);

  -- 2.  REGISTERS       
  signal cntrl_wr_p08   : std_logic_vector (SIZE_P08 - 1 downto 0);
  signal cntrl_next_p08 : std_logic_matrix (SIZE_P08 - 1 downto 0)(7 downto 0);
  signal cntrl_p08      : std_logic_matrix (SIZE_P08 - 1 downto 0)(7 downto 0);

  -- 4.  DATA_OUTPUT_GENERATION
  -- 4.1.        Data output mux
  signal cntrl_rd08 : std_logic_matrix (SIZE_P08 - 1 downto 0)(15 downto 0);

  function matrixAP_or (matrix : std_logic_matrix) return std_logic_vector is
    variable RESULT : std_logic_vector (15 downto 0) := (others => '0');
  begin
    for i in 0 to SIZE_P08-1 loop
      RESULT := RESULT or matrix(i);
    end loop;
    return RESULT;
  end matrixAP_or;

  function matrixBP_or (matrix : std_logic_matrix) return std_logic_vector is
    variable RESULT : std_logic_vector (0 to DEC_SZ_P08 - 1) := (others => '0');
  begin
    for i in 0 to SIZE_P08-1 loop
      RESULT := RESULT or matrix(i);
    end loop;
    return RESULT;
  end matrixBP_or;

begin
  REGISTER_DECODER : block
  begin
    -- 1.1.      Local register selection
    reg_sel_p08 <= per_en and
                   to_stdlogic(per_addr(13 downto DEC_WD_P08 - 1) = BASE_ADDR_P08(14 downto DEC_WD_P08));

    -- 1.2.      Register local address
    reg_addr_p08 <= '0' & per_addr(DEC_WD_P08 - 2 downto 0);

    -- 1.3.      Register address decode 
    address_decode : process (reg_addr_p08)
      variable decode : std_logic_matrix (SIZE_P08 - 1 downto 0)(0 to DEC_SZ_P08 - 1);
    begin
      for i in SIZE_P08 - 1 downto 0 loop
        decode(i) := CNTRL_D08(i) and (0 to DEC_SZ_P08 - 1 =>
                                       to_stdlogic(reg_addr_p08 = std_logic_vector(unsigned(CNTRLB08(i)) srl 1)));
      end loop;

      reg_dec_p08 <= matrixBP_or(decode);
    end process address_decode;

    -- 1.4.      Read/Write probes
    reg_lo_write_p <= per_we(0) and reg_sel_p08;
    reg_hi_write_p <= per_we(1) and reg_sel_p08;
    reg_read_p08   <= not reduce_or(per_we) and reg_sel_p08;

    -- 1.5.      Read/Write vectors
    reg_lo_wr_p <= reg_dec_p08 and (0 to DEC_SZ_P08 - 1 => reg_lo_write_p);
    reg_hi_wr_p <= reg_dec_p08 and (0 to DEC_SZ_P08 - 1 => reg_hi_write_p);
    reg_rd_p08  <= reg_dec_p08 and (0 to DEC_SZ_P08 - 1 => reg_read_p08);
  end block REGISTER_DECODER;

  REGISTERS : block
  begin
    PCNTRL_Register : for i in SIZE_P08 - 1 downto 0 generate
      cntrl_wr_p08(i) <= reg_hi_wr_p(CNTRLI08(i))
                         when CNTRLB08(i)(0) = '1' else reg_lo_wr_p(CNTRLI08(i));
      cntrl_next_p08(i) <= per_din(15 downto 8)
                           when CNTRLB08(i)(0) = '1' else per_din(7 downto 0);

      R_1c_1 : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          cntrl_p08(i) <= X"00";
        elsif (rising_edge(mclk)) then
          if (cntrl_wr_p08(i) = '1') then
            cntrl_p08(i) <= cntrl_next_p08(i);
          end if;
        end if;
      end process R_1c_1;
    end generate PCNTRL_Register;
  end block REGISTERS;

  DATA_OUTPUT_GENERATION : block
  begin
    data_output_mux : for i in SIZE_P08 - 1 downto 0 generate
      cntrl_rd08 (i) <= std_logic_vector((X"00" & (unsigned(cntrl_p08(i)) and (0 to 7 => reg_rd_p08(CNTRLI08(i)))))
                        sll to_integer((0 to 3 => CNTRLB08(i)(0)) and to_unsigned(8, 4)));
    end generate data_output_mux;

    per_dout <= matrixAP_or(cntrl_rd08);
  end block DATA_OUTPUT_GENERATION;
end TEMPLATE_08_ARQ;
