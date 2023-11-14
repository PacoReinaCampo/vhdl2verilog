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

entity GPIO is

  port (
    p1_dout : out std_logic_vector (7 downto 0);
    p2_dout : out std_logic_vector (7 downto 0);
    p3_dout : out std_logic_vector (7 downto 0);
    p4_dout : out std_logic_vector (7 downto 0);
    p5_dout : out std_logic_vector (7 downto 0);
    p6_dout : out std_logic_vector (7 downto 0);

    p1_dout_en : out std_logic_vector (7 downto 0);
    p2_dout_en : out std_logic_vector (7 downto 0);
    p3_dout_en : out std_logic_vector (7 downto 0);
    p4_dout_en : out std_logic_vector (7 downto 0);
    p5_dout_en : out std_logic_vector (7 downto 0);
    p6_dout_en : out std_logic_vector (7 downto 0);

    p1_sel : out std_logic_vector (7 downto 0);
    p2_sel : out std_logic_vector (7 downto 0);
    p3_sel : out std_logic_vector (7 downto 0);
    p4_sel : out std_logic_vector (7 downto 0);
    p5_sel : out std_logic_vector (7 downto 0);
    p6_sel : out std_logic_vector (7 downto 0);

    p1dir : out std_logic_vector (7 downto 0);
    p1ifg : out std_logic_vector (7 downto 0);

    p1_din : in std_logic_vector (7 downto 0);
    p2_din : in std_logic_vector (7 downto 0);
    p3_din : in std_logic_vector (7 downto 0);
    p4_din : in std_logic_vector (7 downto 0);
    p5_din : in std_logic_vector (7 downto 0);
    p6_din : in std_logic_vector (7 downto 0);

    irq_port1 : out std_logic;
    irq_port2 : out std_logic;

    per_dout : out std_logic_vector (15 downto 0);
    mclk     : in  std_logic;
    per_en   : in  std_logic;
    puc_rst  : in  std_logic;
    per_we   : in  std_logic_vector (1 downto 0);
    per_addr : in  std_logic_vector (13 downto 0);
    per_din  : in  std_logic_vector (15 downto 0));
end GPIO;

architecture GPIO_ARQ of GPIO is

  -- 0.  PARAMETER DECLARATION
  -- 0.1.        Register base address (must be aligned to decoder bit width)
  constant BASE_ADDR_G : std_logic_vector (14 downto 0) := (others => '0');

  -- 0.2.        Register addresses offset
  constant DEC_WD_G : integer := 6;

  -- 0.3.        Register one-hot decoder utilities
  constant DEC_SZ_G   : integer := 2**DEC_WD_G;

  constant BASE_REG_G : std_logic_vector (DEC_SZ_G - 1 downto 0) := std_logic_vector(to_unsigned(1, DEC_SZ_G));

  -- 0.7.        Masks
  constant P_EN : std_logic_vector (DEC_WD_G - 1 downto 0) := "111111";

  constant P_IN   : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_WD_G - 1 downto 0) := ("110100", "110000", "011100", "011000", "101000", "100000");
  constant P_OUT  : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_WD_G - 1 downto 0) := ("110101", "110001", "011101", "011001", "101001", "100001");
  constant P_DIR  : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_WD_G - 1 downto 0) := ("110110", "110010", "011110", "011010", "101010", "100010");
  constant P_SELC : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_WD_G - 1 downto 0) := ("110111", "110011", "011111", "011011", "101110", "100110");

  constant P_IFG : std_logic_matrix (DEC_WD_G - 5 downto 0)(DEC_WD_G - 1 downto 0) := ("101011", "100011");
  constant P_IES : std_logic_matrix (DEC_WD_G - 5 downto 0)(DEC_WD_G - 1 downto 0) := ("101100", "100100");
  constant P_IE  : std_logic_matrix (DEC_WD_G - 5 downto 0)(DEC_WD_G - 1 downto 0) := ("101101", "100101");

  -- SIGNAL INOUT
  signal p_dout    : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);
  signal p_dout_en : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);
  signal p_sel     : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);
  signal p_din     : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);
  signal p_din_en  : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);

  -- 1.  REGISTER_DECODER
  -- 1.1.        Local register selection
  signal reg_sel_g : std_logic;

  -- 1.2.        Register local address
  signal reg_addr_g : std_logic_vector (DEC_WD_G - 1 downto 0);

  -- 1.3.        Register address decode
  signal reg_dec_g : std_logic_vector (DEC_SZ_G - 1 downto 0);

  -- 1.4.        Read/Write probes
  signal reg_lo_write_g : std_logic;
  signal reg_hi_write_g : std_logic;
  signal reg_read_g     : std_logic;

  -- 1.5.        Read/Write vectors
  signal reg_hi_wr_g : std_logic_vector (DEC_SZ_G - 1 downto 0);
  signal reg_lo_wr_g : std_logic_vector (DEC_SZ_G - 1 downto 0);
  signal reg_rd_g    : std_logic_vector (DEC_SZ_G - 1 downto 0);

  -- 2.  REGISTERS       
  -- 2.1.        PIN Register
  signal pin : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);

  -- 2.2.        POUT Register
  signal pout_wr  : std_logic_vector (DEC_SZ_G - 1 downto 0);
  signal pout     : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);
  signal pout_nxt : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);

  -- 2.3.        PDIR Register
  signal pdir_wr  : std_logic_vector (DEC_SZ_G - 1 downto 0);
  signal pdir     : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);
  signal pdir_nxt : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);

  -- 2.4.        PIFG Register
  signal pifg_wr  : std_logic_vector (DEC_SZ_G - 5 downto 0);
  signal pifg     : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);
  signal pifg_nxt : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);
  signal pifg_set : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);

  -- 2.5.        PIES Register
  signal pies_wr  : std_logic_vector (DEC_SZ_G - 5 downto 0);
  signal pies     : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);
  signal pies_nxt : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);

  -- 2.6.        PIE Register
  signal pie_wr  : std_logic_vector (DEC_SZ_G - 5 downto 0);
  signal pie     : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);
  signal pie_nxt : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);

  -- 2.7.        PSEL Register
  signal psel_wr  : std_logic_vector (DEC_SZ_G - 5 downto 0);
  signal psel     : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);
  signal psel_nxt : std_logic_matrix (DEC_WD_G - 1 downto 0)(7 downto 0);

  -- 3.  INTERRRUPT_GENERATION
  signal irq_port : std_logic_vector (1 downto 0);

  -- 3.1.        Delay input
  signal p_in_dly : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);

  -- 3.2.        Edge detection
  signal p_in_re : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);
  signal p_in_fe : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);

  -- 3.3.        Set interrupt flag
  -- 3.4.        Generate CPU interrupt
  -- 4.  DATA_OUTPUT_GENERATION
  -- 4.1.        Data output mux
  signal p_in_rd  : std_logic_matrix (DEC_WD_G - 1 downto 0)(15 downto 0);
  signal p_out_rd : std_logic_matrix (DEC_WD_G - 1 downto 0)(15 downto 0);
  signal p_dir_rd : std_logic_matrix (DEC_WD_G - 1 downto 0)(15 downto 0);
  signal p_sel_rd : std_logic_matrix (DEC_WD_G - 1 downto 0)(15 downto 0);

  signal p_ifg_rd : std_logic_matrix (DEC_WD_G - 5 downto 0)(15 downto 0);
  signal p_ies_rd : std_logic_matrix (DEC_WD_G - 5 downto 0)(15 downto 0);
  signal p_ie_rd  : std_logic_matrix (DEC_WD_G - 5 downto 0)(15 downto 0);

  function matrixA5G_or (matrix : std_logic_matrix) return std_logic_vector is
    variable RESULT : std_logic_vector (15 downto 0) := (others => '0');
  begin
    for i in matrix'range loop
      RESULT := RESULT or matrix(i);
    end loop;
    return RESULT;
  end matrixA5G_or;

  function matrixA1G_or (matrix : std_logic_matrix) return std_logic_vector is
    variable RESULT : std_logic_vector (15 downto 0) := (others => '0');
  begin
    for i in matrix'range loop
      RESULT := RESULT or matrix(i);
    end loop;
    return RESULT;
  end matrixA1G_or;

  function matrixB5G_or (matrix : std_logic_matrix) return std_logic_vector is
    variable RESULT : std_logic_vector (DEC_SZ_G - 1 downto 0) := (others => '0');
  begin
    for i in matrix'range loop
      RESULT := RESULT or matrix(i);
    end loop;
    return RESULT;
  end matrixB5G_or;

  function matrixB1G_or (matrix : std_logic_matrix) return std_logic_vector is
    variable RESULT : std_logic_vector (DEC_SZ_G - 1 downto 0) := (others => '0');
  begin
    for i in matrix'range loop
      RESULT := RESULT or matrix(i);
    end loop;
    return RESULT;
  end matrixB1G_or;

begin
  REGISTER_DECODER : block
  begin
    -- 1.1.      Local register selection
    reg_sel_g <= per_en and to_stdlogic(per_addr(13 downto DEC_WD_G - 1) = BASE_ADDR_G(14 downto DEC_WD_G));

    -- 1.2.      Register local address
    reg_addr_g <= '0' & per_addr(DEC_WD_G-2 downto 0);

    -- 1.3.      Register address decode
    process (reg_addr_g)
      variable P_IN_D   : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_SZ_G - 1 downto 0);
      variable P_OUT_D  : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_SZ_G - 1 downto 0);
      variable P_DIR_D  : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_SZ_G - 1 downto 0);
      variable P_SELC_D : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_SZ_G - 1 downto 0);

      variable P_IFG_D : std_logic_matrix (DEC_WD_G - 5 downto 0)(DEC_SZ_G - 1 downto 0);
      variable P_IES_D : std_logic_matrix (DEC_WD_G - 5 downto 0)(DEC_SZ_G - 1 downto 0);
      variable P_IE_D  : std_logic_matrix (DEC_WD_G - 5 downto 0)(DEC_SZ_G - 1 downto 0);

      variable P_IN_DX   : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_SZ_G - 1 downto 0);
      variable P_OUT_DX  : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_SZ_G - 1 downto 0);
      variable P_DIR_DX  : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_SZ_G - 1 downto 0);
      variable P_SELC_DX : std_logic_matrix (DEC_WD_G - 1 downto 0)(DEC_SZ_G - 1 downto 0);

      variable P_IFG_DX : std_logic_matrix (DEC_WD_G - 5 downto 0)(DEC_SZ_G - 1 downto 0);
      variable P_IES_DX : std_logic_matrix (DEC_WD_G - 5 downto 0)(DEC_SZ_G - 1 downto 0);
      variable P_IE_DX  : std_logic_matrix (DEC_WD_G - 5 downto 0)(DEC_SZ_G - 1 downto 0);

    begin
      for i in DEC_WD_G - 1 downto 0 loop
        P_IN_D (i)   := std_logic_vector(unsigned(BASE_REG_G) sll to_integer(unsigned(P_IN (i))));
        P_OUT_D (i)  := std_logic_vector(unsigned(BASE_REG_G) sll to_integer(unsigned(P_OUT (i))));
        P_DIR_D (i)  := std_logic_vector(unsigned(BASE_REG_G) sll to_integer(unsigned(P_DIR (i))));
        P_SELC_D (i) := std_logic_vector(unsigned(BASE_REG_G) sll to_integer(unsigned(P_SELC (i))));

        P_IN_DX (i) := (P_IN_D (i) and (0 to DEC_SZ_G - 1 =>
                                        (to_stdlogic(reg_addr_g = std_logic_vector(unsigned(P_IN (i)) srl 1)) and P_EN(i))));
        P_OUT_DX (i) := (P_OUT_D (i) and (0 to DEC_SZ_G - 1 =>
                                          (to_stdlogic(reg_addr_g = std_logic_vector(unsigned(P_OUT (i)) srl 1)) and P_EN(i))));
        P_DIR_DX (i) := (P_DIR_D (i) and (0 to DEC_SZ_G - 1 =>
                                          (to_stdlogic(reg_addr_g = std_logic_vector(unsigned(P_DIR (i)) srl 1)) and P_EN(i))));
        P_SELC_DX (i) := (P_SELC_D (i) and (0 to DEC_SZ_G - 1 =>
                                            (to_stdlogic(reg_addr_g = std_logic_vector(unsigned(P_SELC (i)) srl 1)) and P_EN(i))));
      end loop;

      for i in DEC_WD_G - 5 downto 0 loop
        P_IFG_D (i) := std_logic_vector(unsigned(BASE_REG_G) sll to_integer(unsigned(P_IFG (i))));
        P_IES_D (i) := std_logic_vector(unsigned(BASE_REG_G) sll to_integer(unsigned(P_IES (i))));
        P_IE_D (i)  := std_logic_vector(unsigned(BASE_REG_G) sll to_integer(unsigned(P_IE (i))));

        P_IFG_DX (i) := (P_IFG_D (i) and (0 to DEC_SZ_G - 1 =>
                                          (to_stdlogic(reg_addr_g = std_logic_vector(unsigned(P_IFG (i)) srl 1)) and P_EN(i))));
        P_IES_DX (i) := (P_IES_D (i) and (0 to DEC_SZ_G - 1 =>
                                          (to_stdlogic(reg_addr_g = std_logic_vector(unsigned(P_IES (i)) srl 1)) and P_EN(i))));
        P_IE_DX (i) := (P_IE_D (i) and (0 to DEC_SZ_G - 1 =>
                                        (to_stdlogic(reg_addr_g = std_logic_vector(unsigned(P_IE (i)) srl 1)) and P_EN(i))));
      end loop;

      reg_dec_g <= matrixB1G_or(P_IN_DX) or
                   matrixB1G_or(P_OUT_DX) or
                   matrixB1G_or(P_DIR_DX) or
                   matrixB1G_or(P_SELC_DX) or

                   matrixB5G_or(P_IFG_DX) or
                   matrixB5G_or(P_IES_DX) or
                   matrixB5G_or(P_IE_DX);
    end process;

    -- 1.4.      Read/Write probes
    reg_lo_write_g <= per_we(0) and reg_sel_g;
    reg_hi_write_g <= per_we(1) and reg_sel_g;
    reg_read_g     <= not reduce_or(per_we) and reg_sel_g;

    -- 1.5.      Read/Write vectors
    reg_hi_wr_g <= reg_dec_g and (0 to DEC_SZ_G - 1 => reg_hi_write_g);
    reg_lo_wr_g <= reg_dec_g and (0 to DEC_SZ_G - 1 => reg_lo_write_g);
    reg_rd_g    <= reg_dec_g and (0 to DEC_SZ_G - 1 => reg_read_g);
  end block REGISTER_DECODER;

  REGISTERS : block
  begin
    RD_DEC_WD_G1 : for i in DEC_WD_G - 1 downto 0 generate
      -- 2.1.    PIN Register
      sync_cell_pin_j : for j in 7 downto 0 generate
        sync_cell_pin : omsp_sync_cell
          port map (
            data_out => pin(i)(j),
            data_in  => p_din_en(i)(j),
            clk      => mclk,
            rst      => puc_rst);

        p_din_en(i)(j) <= p_din(i)(j) and P_EN(i);
      end generate sync_cell_pin_j;

      -- 2.2.    POUT Register           
      pout_wr (i) <= reg_hi_wr_g(to_integer(unsigned(P_OUT(i))))
                     when P_OUT(i)(0) = '1' else reg_lo_wr_g(to_integer(unsigned(P_OUT(i))));
      pout_nxt (i) <= per_din(15 downto 8)
                      when P_OUT(i)(0) = '1' else per_din(7 downto 0);

      process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          pout(i) <= X"00";
        elsif (rising_edge(mclk)) then
          if (pout_wr(i) = '1') then
            pout(i) <= pout_nxt(i) and (0 to 7 => P_EN(i));
          end if;
        end if;
      end process;

      p_dout(i) <= pout(i);

      -- 2.3.    PDIR Register                   
      pdir_wr (i) <= reg_hi_wr_g(to_integer(unsigned(P_DIR(i))))
                     when P_DIR(i)(0) = '1' else reg_lo_wr_g(to_integer(unsigned(P_DIR(i))));
      pdir_nxt (i) <= per_din(15 downto 8)
                      when P_DIR(i)(0) = '1' else per_din(7 downto 0);

      process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          pdir(i) <= X"00";
        elsif (rising_edge(mclk)) then
          if (pdir_wr(i) = '1') then
            pdir(i) <= pdir_nxt(i) and (0 to 7 => P_EN(i));
          end if;
        end if;
      end process;

      p_dout_en(i) <= pdir(i);

      -- 2.7.    PSEL Register                   
      psel_wr (i) <= reg_hi_wr_g(to_integer(unsigned(P_SELC(i))))
                     when P_SELC(i)(0) = '1' else reg_lo_wr_g(to_integer(unsigned(P_SELC(i))));
      psel_nxt (i) <= per_din(15 downto 8)
                      when P_SELC(i)(0) = '1' else per_din(7 downto 0);

      process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          psel(i) <= X"00";
        elsif (rising_edge(mclk)) then
          if (psel_wr(i) = '1') then
            psel(i) <= psel_nxt(i) and (0 to 7 => P_EN(i));
          end if;
        end if;
      end process;

      p_sel(i) <= psel(i);
    end generate RD_DEC_WD_G1;

    RD_DEC_WD_G5 : for i in DEC_WD_G - 5 downto 0 generate
      -- 2.4.    PIFG Register
      pifg_wr (i) <= reg_hi_wr_g(to_integer(unsigned(P_IFG(i))))
                     when P_IFG(i)(0) = '1' else reg_lo_wr_g(to_integer(unsigned(P_IFG(i))));
      pifg_nxt (i) <= per_din(15 downto 8)
                      when P_IFG(i)(0) = '1' else per_din(7 downto 0);

      process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          pifg(i) <= X"00";
        elsif (rising_edge(mclk)) then
          if (pifg_wr(i) = '1') then
            pifg(i) <= (pifg_nxt(i) or pifg_set(i)) and (0 to 7 => P_EN(i));
          else
            pifg(i) <= (pifg(i) or pifg_set(i)) and (0 to 7 => P_EN(i));
          end if;
        end if;
      end process;

      -- 2.5.    PIES Register                   
      pies_wr (i) <= reg_hi_wr_g(to_integer(unsigned(P_IES(i))))
                     when P_IES(i)(0) = '1' else reg_lo_wr_g(to_integer(unsigned(P_IES(i))));
      pies_nxt (i) <= per_din(15 downto 8)
                      when P_IES(i)(0) = '1' else per_din(7 downto 0);

      process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          pies(i) <= X"00";
        elsif (rising_edge(mclk)) then
          if (pies_wr(i) = '1') then
            pies(i) <= pies_nxt(i) and (0 to 7 => P_EN(i));
          end if;
        end if;
      end process;

      -- 2.6.    PIE Register                    
      pie_wr (i) <= reg_hi_wr_g(to_integer(unsigned(P_IE(i))))
                    when P_IE(i)(0) = '1' else reg_lo_wr_g(to_integer(unsigned(P_IE(i))));
      pie_nxt (i) <= per_din(15 downto 8)
                     when P_IE(i)(0) = '1' else per_din(7 downto 0);

      process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          pie(i) <= X"00";
        elsif (rising_edge(mclk)) then
          if (pie_wr(i) = '1') then
            pie(i) <= pie_nxt(i) and (0 to 7 => P_EN(i));
          end if;
        end if;
      end process;
    end generate RD_DEC_WD_G5;
  end block REGISTERS;

  INTERRRUPT_GENERATION : block
  begin
    IG_DEC_WD_G5 : for i in DEC_WD_G - 5 downto 0 generate
      -- 3.1.    Delay input
      process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          p_in_dly(i) <= X"00";
        elsif (rising_edge(mclk)) then
          p_in_dly(i) <= pin(i) and (0 to 7 => P_EN(i));
        end if;
      end process;

      -- 3.2.    Edge detection
      p_in_re(i) <= pin(i) and not p_in_dly(i);
      p_in_fe(i) <= not pin(i) and p_in_dly(i);

      -- 3.3.    Set interrupt flag
      process (p_in_fe, p_in_re, pies)
        variable dout : std_logic_matrix (DEC_WD_G - 5 downto 0)(7 downto 0);
      begin
        for j in 7 downto 0 loop
          if pies(i)(j) = '1' then
            dout(i)(j) := p_in_fe(i)(j);
          else
            dout(i)(j) := p_in_re(i)(j);
          end if;
        end loop;

        pifg_set(i) <= dout(i) and (0 to 7 => P_EN(i));
      end process;

      -- 3.4.    Generate CPU interrupt
      irq_port(i) <= ((pie(i)(0) and pifg(i)(0)) or
                      (pie(i)(1) and pifg(i)(1)) or
                      (pie(i)(2) and pifg(i)(2)) or
                      (pie(i)(3) and pifg(i)(3)) or
                      (pie(i)(4) and pifg(i)(4)) or
                      (pie(i)(5) and pifg(i)(5)) or
                      (pie(i)(6) and pifg(i)(6)) or
                      (pie(i)(7) and pifg(i)(7))) and P_EN(i);
    end generate IG_DEC_WD_G5;

    irq_port1 <= irq_port(0);
    irq_port2 <= irq_port(1);
  end block INTERRRUPT_GENERATION;

  DATA_OUTPUT_GENERATION : block
  begin
    -- 4.1.      Data output mux
    data_output_mux_1 : for i in DEC_WD_G - 1 downto 0 generate
      p_in_rd (i) <= std_logic_vector((X"00" & (unsigned(pin(i)) and
                                                 (0 to 7 => reg_rd_g(to_integer(unsigned(P_IN (i)))))))
                                       sll to_integer((0 to 3 => P_IN (i)(0)) and to_unsigned(8, 4)));
      p_out_rd (i) <= std_logic_vector((X"00" & (unsigned(pout(i)) and
                                                  (0 to 7 => reg_rd_g(to_integer(unsigned(P_OUT (i)))))))
                                        sll to_integer((0 to 3 => P_OUT (i)(0)) and to_unsigned(8, 4)));
      p_dir_rd (i) <= std_logic_vector((X"00" & (unsigned(pdir(i)) and
                                                  (0 to 7 => reg_rd_g(to_integer(unsigned(P_DIR (i)))))))
                                        sll to_integer((0 to 3 => P_DIR (i)(0)) and to_unsigned(8, 4)));
      p_sel_rd (i) <= std_logic_vector((X"00" & (unsigned(psel(i)) and
                                                  (0 to 7 => reg_rd_g(to_integer(unsigned(P_SELC (i)))))))
                                        sll to_integer((0 to 3 => P_SELC (i)(0)) and to_unsigned(8, 4)));
    end generate data_output_mux_1;

    data_output_mux_5 : for i in DEC_WD_G - 5 downto 0 generate
      p_ifg_rd (i) <= std_logic_vector((X"00" & (unsigned(pifg(i)) and
                                                  (0 to 7 => reg_rd_g(to_integer(unsigned(P_IFG (i)))))))
                                        sll to_integer((0 to 3 => P_IFG (i)(0)) and to_unsigned(8, 4)));
      p_ies_rd (i) <= std_logic_vector((X"00" & (unsigned(pies(i)) and
                                                  (0 to 7 => reg_rd_g(to_integer(unsigned(P_IES (i)))))))
                                        sll to_integer((0 to 3 => P_IES (i)(0)) and to_unsigned(8, 4)));
      p_ie_rd (i) <= std_logic_vector((X"00" & (unsigned(pie(i)) and
                                                 (0 to 7 => reg_rd_g(to_integer(unsigned(P_IE (i)))))))
                                       sll to_integer((0 to 3 => P_IE (i)(0)) and to_unsigned(8, 4)));
    end generate data_output_mux_5;

    per_dout <= matrixA1G_or(p_in_rd) or
                matrixA1G_or(p_out_rd) or
                matrixA1G_or(p_dir_rd) or
                matrixA1G_or(p_sel_rd) or

                matrixA5G_or(p_ifg_rd) or
                matrixA5G_or(p_ies_rd) or
                matrixA5G_or(p_ie_rd);
  end block DATA_OUTPUT_GENERATION;

  SIGNAL_INOUT : block
  begin
    p1_dout <= p_dout(0);
    p2_dout <= p_dout(1);
    p3_dout <= p_dout(2);
    p4_dout <= p_dout(3);
    p5_dout <= p_dout(4);
    p6_dout <= p_dout(5);

    p1_dout_en <= p_dout_en(0);
    p2_dout_en <= p_dout_en(1);
    p3_dout_en <= p_dout_en(2);
    p4_dout_en <= p_dout_en(3);
    p5_dout_en <= p_dout_en(4);
    p6_dout_en <= p_dout_en(5);

    p1_sel <= p_sel(0);
    p2_sel <= p_sel(1);
    p3_sel <= p_sel(2);
    p4_sel <= p_sel(3);
    p5_sel <= p_sel(4);
    p6_sel <= p_sel(5);

    p_din(0) <= p1_din;
    p_din(1) <= p2_din;
    p_din(2) <= p3_din;
    p_din(3) <= p4_din;
    p_din(4) <= p5_din;
    p_din(5) <= p6_din;

    p1dir <= pdir(0);
    p1ifg <= pifg(0);
  end block SIGNAL_INOUT;
end GPIO_ARQ;
