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

entity UART is
  port (
    uart_txd : out std_logic;
    uart_rxd : in  std_logic;
    smclk_en : in  std_logic;

    irq_uart_rx : out std_logic;
    irq_uart_tx : out std_logic;

    per_dout : out std_logic_vector (15 downto 0);
    mclk     : in  std_logic;
    per_en   : in  std_logic;
    puc_rst  : in  std_logic;
    per_we   : in  std_logic_vector (1 downto 0);
    per_addr : in  std_logic_vector (13 downto 0);
    per_din  : in  std_logic_vector (15 downto 0));
end UART;

architecture UART_ARQ of UART is

  --0.  PARAMETER_DECLARATION
  --0.1.        Register base address (must be aligned to decoder bit width)
  constant BASE_ADDR_U : std_logic_vector (14 downto 0) := "000000010000000";

  --0.2.        Decoder bit width (defines how many bits are considered for address decoding)
  constant DEC_WD_U : integer := 3;

  --0.3.        Register addresses offset
  constant CTRLB    : std_logic_vector (DEC_WD_U - 1 downto 0) := std_logic_vector(to_unsigned(0, DEC_WD_U));
  constant STATUSB  : std_logic_vector (DEC_WD_U - 1 downto 0) := std_logic_vector(to_unsigned(1, DEC_WD_U));
  constant BAUD_LOB : std_logic_vector (DEC_WD_U - 1 downto 0) := std_logic_vector(to_unsigned(2, DEC_WD_U));
  constant BAUD_HIB : std_logic_vector (DEC_WD_U - 1 downto 0) := std_logic_vector(to_unsigned(3, DEC_WD_U));
  constant DATA_TXB : std_logic_vector (DEC_WD_U - 1 downto 0) := std_logic_vector(to_unsigned(4, DEC_WD_U));
  constant DATA_RXB : std_logic_vector (DEC_WD_U - 1 downto 0) := std_logic_vector(to_unsigned(5, DEC_WD_U));

  constant CTRLC    : integer := to_integer(unsigned(CTRLB));
  constant STATUSC  : integer := to_integer(unsigned(STATUSB));
  constant BAUD_LOC : integer := to_integer(unsigned(BAUD_LOB));
  constant BAUD_HIC : integer := to_integer(unsigned(BAUD_HIB));
  constant DATA_TXC : integer := to_integer(unsigned(DATA_TXB));
  constant DATA_RXC : integer := to_integer(unsigned(DATA_RXB));

  --0.4.        Register one-hot decoder utilities
  constant DEC_SZ_U   : integer                                   := 2**DEC_WD_U;
  constant BASE_REG_U : std_logic_vector (DEC_SZ_U - 1 downto 0) := std_logic_vector(to_unsigned(1, DEC_SZ_U));

  --0.5.        Register one-hot decoder
  constant CTRL_D    : std_logic_vector (DEC_SZ_U - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_U) sll CTRLC);
  constant STATUS_D  : std_logic_vector (DEC_SZ_U - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_U) sll STATUSC);
  constant BAUD_LO_D : std_logic_vector (DEC_SZ_U - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_U) sll BAUD_LOC);
  constant BAUD_HI_D : std_logic_vector (DEC_SZ_U - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_U) sll BAUD_HIC);
  constant DATA_TX_D : std_logic_vector (DEC_SZ_U - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_U) sll DATA_TXC);
  constant DATA_RX_D : std_logic_vector (DEC_SZ_U - 1 downto 0) := std_logic_vector(unsigned(BASE_REG_U) sll DATA_RXC);

  --1.  REGISTER_DECODER
  --1.1.        Local register selection
  signal reg_sel_u : std_logic;

  --1.2.        Register local address
  signal reg_addr_u : std_logic_vector (DEC_WD_U - 1 downto 0);

  --1.3.        Register address decode
  signal reg_dec_u : std_logic_vector (DEC_SZ_U - 1 downto 0);

  --1.4.        Read/Write probes
  signal reg_read_u     : std_logic;
  signal reg_hi_write_u : std_logic;
  signal reg_lo_write_u : std_logic;

  --1.5.        Read/Write vectors
  signal reg_rd_u    : std_logic_vector (DEC_SZ_U - 1 downto 0);
  signal reg_hi_wr_u : std_logic_vector (DEC_SZ_U - 1 downto 0);
  signal reg_lo_wr_u : std_logic_vector (DEC_SZ_U - 1 downto 0);

  --2.  REGISTERS
  --2.1.        CTRL Register
  signal ctrl_wr           : std_logic;
  signal ctrl_ien_tx_empty : std_logic;
  signal ctrl_ien_tx       : std_logic;
  signal ctrl_ien_rx_ovflw : std_logic;
  signal ctrl_ien_rx       : std_logic;
  signal ctrl_smclk_sel    : std_logic;
  signal ctrl_en           : std_logic;
  signal ctrl              : std_logic_vector (7 downto 0);
  signal ctrl_nxt          : std_logic_vector (7 downto 0);

  --2.2.        STATUS Register
  signal status_tx_empty_pnd     : std_logic;
  signal status_tx_pnd           : std_logic;
  signal status_rx_ovflw_pnd     : std_logic;
  signal status_rx_pnd           : std_logic;
  signal status_tx_full          : std_logic;
  signal status_tx_busy          : std_logic;
  signal status_rx_busy          : std_logic;
  signal status_wr               : std_logic;
  signal status_tx_empty_pnd_clr : std_logic;
  signal status_tx_pnd_clr       : std_logic;
  signal status_rx_ovflw_pnd_clr : std_logic;
  signal status_rx_pnd_clr       : std_logic;
  signal status_tx_empty_pnd_set : std_logic;
  signal status_tx_pnd_set       : std_logic;
  signal status_rx_ovflw_pnd_set : std_logic;
  signal status_rx_pnd_set       : std_logic;
  signal status_u                : std_logic_vector (7 downto 0);
  signal status_nxt              : std_logic_vector (7 downto 0);

  --2.3.        BAUD_LO Register
  signal baud_lo_wr  : std_logic;
  signal baud_lo     : std_logic_vector (7 downto 0);
  signal baud_lo_nxt : std_logic_vector (7 downto 0);

  --2.4.        BAUD_HI Register
  signal baud_hi_wr  : std_logic;
  signal baud_hi     : std_logic_vector (7 downto 0);
  signal baud_hi_nxt : std_logic_vector (7 downto 0);
  signal baudrate    : std_logic_vector (15 downto 0);

  --2.5.        DATA_TX Register
  signal data_tx_wr  : std_logic;
  signal data_tx     : std_logic_vector (7 downto 0);
  signal data_tx_nxt : std_logic_vector (7 downto 0);

  --2.6.        DATA_RX Register
  signal data_rx   : std_logic_vector (7 downto 0);
  signal rxfer_buf : std_logic_vector (7 downto 0);

  --3.  DATA_OUTPUT_GENERATION
  --3.1.        Data output mux
  signal ctrl_rd    : std_logic_vector (15 downto 0);
  signal status_rd  : std_logic_vector (15 downto 0);
  signal baud_lo_rd : std_logic_vector (15 downto 0);
  signal baud_hi_rd : std_logic_vector (15 downto 0);
  signal data_tx_rd : std_logic_vector (15 downto 0);
  signal data_rx_rd : std_logic_vector (15 downto 0);

  --4.  UART_CLOCK_SELECTION
  signal uclk_en : std_logic;

  --5.  LINE_SYNCHRONIZTION_FILTERING
  --5.1.        Synchronize RXD input
  signal uart_rxd_sync_n : std_logic;
  signal not_uart_rxd    : std_logic;
  signal uart_rxd_sync   : std_logic;

  --5.2.        RXD input buffer
  signal rxd_buf_u : std_logic_vector (1 downto 0);

  --5.3.        Majority decision
  signal rxd_maj_u     : std_logic;
  signal rxd_maj_nxt_u : std_logic;
  signal rxd_s_u       : std_logic;
  signal rxd_fe_u      : std_logic;
  signal rxd_maj_cnt   : std_logic_vector (1 downto 0);
  signal zero_sync     : std_logic_vector (1 downto 0);
  signal zero_buf_u_0  : std_logic_vector (1 downto 0);
  signal zero_buf_u_1  : std_logic_vector (1 downto 0);

  --6.  UART_RECEIVE
  --6.1.        RX Transfer counter
  signal rxfer_start   : std_logic;
  signal rxfer_bit_inc : std_logic;
  signal rxfer_done    : std_logic;
  signal rxfer_bit     : std_logic_vector (3 downto 0);
  signal rxfer_cnt     : std_logic_vector (15 downto 0);

  --6.2.        Receive buffer
  signal rxfer_buf_nxt : std_logic_vector (7 downto 0);

  --6.3.        Status flags
  signal rxfer_done_dly : std_logic;

  --7.  UART_TRANSMIT
  --7.1.        TX Transfer start detection
  signal txfer_triggered : std_logic;
  signal txfer_start     : std_logic;

  --7.2.        TX Transfer counter
  signal txfer_bit_inc : std_logic;
  signal txfer_done    : std_logic;
  signal txfer_bit     : std_logic_vector (3 downto 0);
  signal txfer_cnt     : std_logic_vector (15 downto 0);

  --7.3.        Transmit buffer
  signal txfer_buf     : std_logic_vector (8 downto 0);
  signal txfer_buf_nxt : std_logic_vector (8 downto 0);

  --7.4.        Status flags
  signal txfer_done_dly : std_logic;

  --8.  INTERRUPTS
  --8.1.        Receive interrupt
  --8.2.        Transmit interrupt

begin
  REGISTER_DECODER : block
  begin
    --1.1.      Local register selection
    reg_sel_u <= per_en and to_stdlogic(per_addr(13 downto DEC_WD_U - 1) = BASE_ADDR_U(14 downto DEC_WD_U));

    --1.2.      Register local address
    reg_addr_u <= '0' & per_addr(DEC_WD_U - 2 downto 0);

    --1.3.      Register address decode
    reg_dec_u <= (CTRL_D and (0 to DEC_SZ_U - 1 => to_stdlogic(reg_addr_u =
                                                             std_logic_vector(unsigned(CTRLB) srl 1)))) or
                 (STATUS_D and (0 to DEC_SZ_U - 1 => to_stdlogic(reg_addr_u =
                                                               std_logic_vector(unsigned(STATUSB) srl 1)))) or
                 (BAUD_LO_D and (0 to DEC_SZ_U - 1 => to_stdlogic(reg_addr_u =
                                                                std_logic_vector(unsigned(BAUD_LOB) srl 1)))) or
                 (BAUD_HI_D and (0 to DEC_SZ_U - 1 => to_stdlogic(reg_addr_u =
                                                                std_logic_vector(unsigned(BAUD_HIB) srl 1)))) or
                 (DATA_TX_D and (0 to DEC_SZ_U - 1 => to_stdlogic(reg_addr_u =
                                                                std_logic_vector(unsigned(DATA_TXB) srl 1)))) or
                 (DATA_RX_D and (0 to DEC_SZ_U - 1 => to_stdlogic(reg_addr_u =
                                                                std_logic_vector(unsigned(DATA_RXB) srl 1))));

    --1.4.      Read/Write probes
    reg_hi_write_u <= per_we(0) and reg_sel_u;
    reg_lo_write_u <= per_we(1) and reg_sel_u;
    reg_read_u     <= not reduce_or(per_we) and reg_sel_u;

    --1.5.      Read/Write vectors
    reg_hi_wr_u <= reg_dec_u and (0 to DEC_SZ_U - 1 => reg_hi_write_u);
    reg_lo_wr_u <= reg_dec_u and (0 to DEC_SZ_U - 1 => reg_lo_write_u);
    reg_rd_u    <= reg_dec_u and (0 to DEC_SZ_U - 1 => reg_read_u);
  end block REGISTER_DECODER;

  REGISTERS : block
  begin
    --2.1.      CTRL Register
    ctrl_wr  <= reg_hi_wr_u(CTRLC)   when CTRLB(0) = '1' else reg_lo_wr_u(CTRLC);
    ctrl_nxt <= per_din(15 downto 8) when CTRLB(0) = '1' else per_din(7 downto 0);

    R_1c : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        ctrl <= X"00";
      elsif (rising_edge(mclk)) then
        if (ctrl_wr = '1') then
          ctrl <= ctrl_nxt and X"73";
        end if;
      end if;
    end process R_1c;

    ctrl_ien_tx_empty <= ctrl(7);
    ctrl_ien_tx       <= ctrl(6);
    ctrl_ien_rx_ovflw <= ctrl(5);
    ctrl_ien_rx       <= ctrl(4);
    ctrl_smclk_sel    <= ctrl(1);
    ctrl_en           <= ctrl(0);

    --2.2.      STATUS Register
    status_wr               <= reg_hi_wr_u(STATUSC) when STATUSB(0) = '1' else reg_lo_wr_u(STATUSC);
    status_nxt              <= per_din(15 downto 8) when STATUSB(0) = '1' else per_din(7 downto 0);
    status_tx_empty_pnd_clr <= status_wr and status_nxt(7);
    status_tx_pnd_clr       <= status_wr and status_nxt(6);
    status_rx_ovflw_pnd_clr <= status_wr and status_nxt(5);
    status_rx_pnd_clr       <= status_wr and status_nxt(4);

    R1_1c_2c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        status_tx_empty_pnd <= '0';
      elsif (rising_edge(mclk)) then
        if (status_tx_empty_pnd_set = '1') then
          status_tx_empty_pnd <= '1';
        elsif (status_tx_empty_pnd_clr = '1') then
          status_tx_empty_pnd <= '0';
        end if;
      end if;
    end process R1_1c_2c_e;

    R2_1c_2c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        status_tx_pnd <= '0';
      elsif (rising_edge(mclk)) then
        if (status_tx_pnd_set = '1') then
          status_tx_pnd <= '1';
        elsif (status_tx_pnd_clr = '1') then
          status_tx_pnd <= '0';
        end if;
      end if;
    end process R2_1c_2c_e;

    R3_1c_2c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        status_rx_ovflw_pnd <= '0';
      elsif (rising_edge(mclk)) then
        if (status_rx_ovflw_pnd_set = '1') then
          status_rx_ovflw_pnd <= '1';
        elsif (status_rx_ovflw_pnd_clr = '1') then
          status_rx_ovflw_pnd <= '0';
        end if;
      end if;
    end process R3_1c_2c_e;

    R4_1c_2c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        status_rx_pnd <= '0';
      elsif (rising_edge(mclk)) then
        if (status_rx_pnd_set = '1') then
          status_rx_pnd <= '1';
        elsif (status_rx_pnd_clr = '1') then
          status_rx_pnd <= '0';
        end if;
      end if;
    end process R4_1c_2c_e;

    status_u <= status_tx_empty_pnd & status_tx_pnd & status_rx_ovflw_pnd & status_rx_pnd &
                status_tx_full & status_tx_busy & '0' & status_rx_busy;
    --2.3.      BAUD_LO Register
    baud_lo_wr  <= reg_hi_wr_u(BAUD_LOC) when BAUD_LOB(0) = '1' else reg_lo_wr_u(BAUD_LOC);
    baud_lo_nxt <= per_din(15 downto 8)  when BAUD_LOB(0) = '1' else per_din(7 downto 0);

    R1_1c : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        baud_lo <= X"00";
      elsif (rising_edge(mclk)) then
        if (baud_lo_wr = '1') then
          baud_lo <= baud_lo_nxt;
        end if;
      end if;
    end process R1_1c;

    --2.4.      BAUD_HI Register
    baud_hi_wr  <= reg_hi_wr_u(BAUD_LOC) when BAUD_HIB(0) = '1' else reg_hi_wr_u(BAUD_HIC);
    baud_hi_nxt <= per_din(15 downto 8)  when BAUD_HIB(0) = '1' else per_din(7 downto 0);

    R2_1c : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        baud_hi <= X"00";
      elsif (rising_edge(mclk)) then
        if (baud_hi_wr = '1') then
          baud_hi <= baud_hi_nxt;
        end if;
      end if;
    end process R2_1c;

    baudrate <= baud_hi & baud_lo;

    --2.5.      DATA_TX Register
    data_tx_wr  <= reg_hi_wr_u(DATA_TXC) when DATA_TXB(0) = '1' else reg_lo_wr_u(DATA_TXC);
    data_tx_nxt <= per_din(15 downto 8)  when DATA_TXB(0) = '1' else per_din(7 downto 0);

    R3_1c : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        data_tx <= X"00";
      elsif (rising_edge(mclk)) then
        if (data_tx_wr = '1') then
          data_tx <= data_tx_nxt;
        end if;
      end if;
    end process R3_1c;

    --2.6.      DATA_RX Register
    R4_1c : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        data_rx <= X"00";
      elsif (rising_edge(mclk)) then
        if (status_rx_pnd_set = '1') then
          data_rx <= rxfer_buf;
        end if;
      end if;
    end process R4_1c;
  end block REGISTERS;

  DATA_OUTPUT_GENERATION : block
  begin
    --3.1.      Data output mux
    ctrl_rd <= std_logic_vector((X"00" & (unsigned(ctrl) and (0 to 7 => reg_rd_u(CTRLC))))
                                 sll to_integer((0 to 3 => CTRLB (0)) and to_unsigned(8, 4)));

    status_rd <= std_logic_vector((X"00" & (unsigned(status_u) and (0 to 7 => reg_rd_u(STATUSC))))
                                   sll to_integer((0 to 3 => STATUSB (0)) and to_unsigned(8, 4)));

    baud_lo_rd <= std_logic_vector((X"00" & (unsigned(baud_lo) and (0 to 7 => reg_rd_u(BAUD_LOC))))
                                    sll to_integer((0 to 3 => BAUD_LOB (0)) and to_unsigned(8, 4)));

    baud_hi_rd <= std_logic_vector((X"00" & (unsigned(baud_hi) and (0 to 7 => reg_rd_u(BAUD_HIC))))
                                    sll to_integer((0 to 3 => BAUD_HIB (0)) and to_unsigned(8, 4)));

    data_tx_rd <= std_logic_vector((X"00" & (unsigned(data_tx) and (0 to 7 => reg_rd_u(DATA_TXC))))
                                    sll to_integer((0 to 3 => DATA_TXB (0)) and to_unsigned(8, 4)));

    data_rx_rd <= std_logic_vector((X"00" & (unsigned(data_rx) and (0 to 7 => reg_rd_u(DATA_RXC))))
                                    sll to_integer((0 to 3 => DATA_RXB (0)) and to_unsigned(8, 4)));


    per_dout <= ctrl_rd or status_rd or baud_lo_rd or baud_hi_rd or data_tx_rd or data_rx_rd;
  end block DATA_OUTPUT_GENERATION;

  UART_CLOCK_SELECTION : block
  begin
    uclk_en <= smclk_en when ctrl_smclk_sel = '1' else '1';
  end block UART_CLOCK_SELECTION;

  LINE_SYNCHRONIZTION_FILTERING : block
  begin
    --5.1.      Synchronize RXD input
    sync_cell_uart_rxd : omsp_sync_cell
      port map (
        data_out => uart_rxd_sync_n,
        data_in  => not_uart_rxd,
        clk      => mclk,
        rst      => puc_rst);

    not_uart_rxd  <= not uart_rxd;
    uart_rxd_sync <= not uart_rxd_sync_n;

    --5.2.      RXD input buffer
    R_1 : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        rxd_buf_u <= "11";
      elsif (rising_edge(mclk)) then
        rxd_buf_u <= rxd_buf_u(0) & uart_rxd_sync;
      end if;
    end process R_1;

    --5.3.      Majority decision
    zero_sync    <= '0' & uart_rxd_sync;
    zero_buf_u_0 <= '0' & rxd_buf_u(0);
    zero_buf_u_1 <= '0' & rxd_buf_u(1);

    rxd_maj_cnt <= std_logic_vector(unsigned(zero_sync) +
                                     unsigned(zero_buf_u_0) +
                                     unsigned(zero_buf_u_1));

    rxd_maj_nxt_u <= to_stdlogic(rxd_maj_cnt >= "10");

    R_1_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        rxd_maj_u <= '1';
      elsif (rising_edge(mclk)) then
        rxd_maj_u <= rxd_maj_nxt_u;
      end if;
    end process R_1_e;

    rxd_s_u  <= rxd_maj_u;
    rxd_fe_u <= rxd_maj_u and not rxd_maj_nxt_u;
  end block LINE_SYNCHRONIZTION_FILTERING;

  UART_RECEIVE : block
  begin
    --6.1.      RX Transfer counter
    rxfer_start   <= to_stdlogic(rxfer_bit = X"0") and rxd_fe_u;
    rxfer_bit_inc <= to_stdlogic(rxfer_bit /= X"0") and to_stdlogic(rxfer_cnt = X"0000");
    rxfer_done    <= to_stdlogic(rxfer_bit = X"A");

    process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        rxfer_bit <= X"0";
      elsif (rising_edge(mclk)) then
        if (ctrl_en = '0') then
          rxfer_bit <= X"0";
        elsif (rxfer_start = '1') then
          rxfer_bit <= X"1";
        elsif (uclk_en = '1') then
          if (rxfer_done = '1') then
            rxfer_bit <= X"0";
          elsif (rxfer_bit_inc = '1') then
            rxfer_bit <= std_logic_vector(unsigned(rxfer_bit) + X"1");
          end if;
        end if;
      end if;
    end process;

    process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        rxfer_cnt <= X"0000";
      elsif (rising_edge(mclk)) then
        if (ctrl_en = '0') then
          rxfer_cnt <= X"0000";
        elsif (rxfer_start = '1') then
          rxfer_cnt <= '0' & baudrate(15 downto 1);
        elsif (uclk_en = '1') then
          if (rxfer_bit_inc = '1') then
            rxfer_cnt <= baudrate;
          elsif (reduce_or(rxfer_cnt) = '1') then
            rxfer_cnt <= std_logic_vector(unsigned(rxfer_cnt) + X"FFFF");
          end if;
        end if;
      end if;
    end process;

    --6.2.      Receive buffer
    rxfer_buf_nxt <= rxd_s_u & rxfer_buf(7 downto 1);

    process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        rxfer_buf <= X"00";
      elsif (rising_edge(mclk)) then
        if (ctrl_en = '0') then
          rxfer_buf <= X"00";
        elsif (uclk_en = '1') then
          if (rxfer_bit_inc = '1') then
            rxfer_buf <= rxfer_buf_nxt;
          end if;
        end if;
      end if;
    end process;

    --6.3.      Status flags    
    R_1_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        rxfer_done_dly <= '0';
      elsif (rising_edge(mclk)) then
        rxfer_done_dly <= rxfer_done;
      end if;
    end process R_1_e;

    status_rx_pnd_set       <= rxfer_done and not rxfer_done_dly;
    status_rx_ovflw_pnd_set <= status_rx_pnd_set and status_rx_pnd;
    status_rx_busy          <= to_stdlogic(rxfer_bit /= X"0");
  end block UART_RECEIVE;

  UART_TRANSMIT : block
  begin
    --7.1.      TX Transfer start detection
    R_1c_2c_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        txfer_triggered <= '0';
      elsif (rising_edge(mclk)) then
        if (data_tx_wr = '1') then
          txfer_triggered <= '1';
        elsif (txfer_start = '1') then
          txfer_triggered <= '0';
        end if;
      end if;
    end process R_1c_2c_e;

    --7.2.      TX Transfer counter
    txfer_start   <= to_stdlogic(txfer_bit = X"0") and txfer_triggered;
    txfer_bit_inc <= to_stdlogic(txfer_bit /= X"0") and to_stdlogic(txfer_cnt = X"0000");
    txfer_done    <= to_stdlogic(txfer_bit = X"B");

    process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        txfer_bit <= X"0";
      elsif (rising_edge(mclk)) then

        if (ctrl_en = '0') then
          txfer_bit <= X"0";
        elsif (txfer_start = '1') then
          txfer_bit <= X"1";
        elsif (uclk_en = '1') then
          if (txfer_done = '1') then
            txfer_bit <= X"0";
          elsif (txfer_bit_inc = '1') then
            txfer_bit <= std_logic_vector(unsigned(txfer_bit) + X"1");
          end if;
        end if;
      end if;
    end process;

    process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        txfer_cnt <= X"0000";
      elsif (rising_edge(mclk)) then
        if (ctrl_en = '0') then
          txfer_cnt <= X"0000";
        elsif (txfer_start = '1') then
          txfer_cnt <= baudrate;
        elsif (uclk_en = '1') then
          if (txfer_bit_inc = '1') then
            txfer_cnt <= baudrate;
          elsif (reduce_or(txfer_cnt) = '1') then
            txfer_cnt <= std_logic_vector(unsigned(txfer_cnt) + X"FFFF");
          end if;
        end if;
      end if;
    end process;

    --7.3.      Transmit buffer
    txfer_buf_nxt <= '1' & txfer_buf(8 downto 1);

    process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        txfer_buf <= "111111111";
      elsif (rising_edge(mclk)) then
        if (ctrl_en = '0') then
          txfer_buf <= "111111111";
        elsif (txfer_start = '1') then
          txfer_buf <= data_tx & '0';
        elsif (uclk_en = '1') then
          if (txfer_bit_inc = '1') then
            txfer_buf <= txfer_buf_nxt;
          end if;
        end if;
      end if;
    end process;

    uart_txd <= txfer_buf(0);

    --7.4.      Status flags
    R_1_e : process (mclk, puc_rst)
    begin
      if (puc_rst = '1') then
        txfer_done_dly <= '0';
      elsif (rising_edge(mclk)) then
        txfer_done_dly <= txfer_done;
      end if;
    end process R_1_e;

    status_tx_pnd_set       <= txfer_done and not txfer_done_dly;
    status_tx_empty_pnd_set <= status_tx_pnd_set and not txfer_triggered;
    status_tx_busy          <= to_stdlogic(txfer_bit /= X"0") or txfer_triggered;
    status_tx_full          <= status_tx_busy and txfer_triggered;
  end block UART_TRANSMIT;

  INTERRUPTS : block
  begin
    --8.1.      Receive interrupt
    irq_uart_rx <= (status_rx_pnd and ctrl_ien_rx) or
                   (status_rx_ovflw_pnd and ctrl_ien_rx_ovflw);
    --8.2.      Transmit interrupt
    irq_uart_tx <= (status_tx_pnd and ctrl_ien_tx) or
                   (status_tx_empty_pnd and ctrl_ien_tx_empty);
  end block INTERRUPTS;
end UART_ARQ;
