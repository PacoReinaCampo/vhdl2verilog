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

library IEEE;
use IEEE.STD_LOGIC_1164 .all;
use IEEE.NUMERIC_STD .all;
use WORK.MSP430_PACK .all;

entity omsp_dbg_uart is
  port (
    dbg_rd       : out std_logic;
    dbg_uart_txd : out std_logic;
    dbg_wr       : out std_logic;
    dbg_addr     : out std_logic_vector (5 downto 0);
    dbg_din      : out std_logic_vector (15 downto 0);

    dbg_clk       : in std_logic;
    dbg_rd_rdy    : in std_logic;
    dbg_rst       : in std_logic;
    dbg_uart_rxd  : in std_logic;
    mem_burst     : in std_logic;
    mem_burst_end : in std_logic;
    mem_burst_rd  : in std_logic;
    mem_burst_wr  : in std_logic;
    mem_bw        : in std_logic;
    dbg_dout      : in std_logic_vector (15 downto 0));
end omsp_dbg_uart;

architecture omsp_dbg_uart_ARQ of omsp_dbg_uart is

  -- 8.          UART_COMMUNICATION      
  -- 8.2.                UART STATE MACHINE      
  -- 8.2.2.      State machine definition
  constant RX_SYNC  : std_logic_vector (2 downto 0) := "000";
  constant RX_CMD   : std_logic_vector (2 downto 0) := "001";
  constant RX_DATA1 : std_logic_vector (2 downto 0) := "010";
  constant RX_DATA2 : std_logic_vector (2 downto 0) := "011";
  constant TX_DATA1 : std_logic_vector (2 downto 0) := "100";
  constant TX_DATA2 : std_logic_vector (2 downto 0) := "101";

  -- 8.          UART_COMMUNICATION                                              
  -- 8.1.                UART RECEIVE LINE SYNCHRONIZTION & FILTERING
  -- 8.1.1.      Synchronize RXD input
  signal uart_rxd         : std_logic;
  signal uart_rxd_n       : std_logic;
  signal not_dbg_uart_rxd : std_logic;

  -- 8.1.2.      RXD input buffer
  signal rxd_buf : std_logic_vector (1 downto 0);

  -- 8.1.3.      Majority decision
  signal rxd_maj     : std_logic;
  signal rxd_maj_nxt : std_logic;
  signal rxd_s       : std_logic;
  signal rxd_fe      : std_logic;
  signal rxd_re      : std_logic;
  signal rxd_edge    : std_logic;

  -- 8.2.                UART STATE MACHINE
  -- 8.2.1.      Receive state
  signal sync_done      : std_logic;
  signal xfer_done      : std_logic;
  signal uart_state     : std_logic_vector (2 downto 0);
  signal uart_state_nxt : std_logic_vector (2 downto 0);
  signal xfer_buf       : std_logic_vector (19 downto 0);
  signal xfer_buf_nxt   : std_logic_vector (19 downto 0);

  -- 8.2.2.      State machine definition
  signal re_rx_cmd : std_logic_vector (2 downto 0);

  signal re_0_rx_cmd : std_logic_vector (2 downto 0);
  signal re_0_tx_cmd : std_logic_vector (2 downto 0);

  signal re_1_rx_cmd : std_logic_vector (2 downto 0);
  signal re_1_tx_cmd : std_logic_vector (2 downto 0);

  signal re_rx_data2 : std_logic_vector (2 downto 0);
  signal re_tx_data2 : std_logic_vector (2 downto 0);

  signal re_0_rx_data2 : std_logic_vector (2 downto 0);
  signal re_0_tx_data2 : std_logic_vector (2 downto 0);

  -- 8.2.3.      State transition
  -- 8.2.4.      State machine
  -- 8.2.5.      Utility signals
  signal cmd_valid : std_logic;
  signal rx_active : std_logic;
  signal tx_active : std_logic;

  -- 8.3.                UART SYNCHRONIZATION
  signal sync_busy   : std_logic;
  signal sync_cnt    : std_logic_vector (DBG_UART_XFER_CNT_W+2 downto 0);
  signal bit_cnt_max : std_logic_vector (DBG_UART_XFER_CNT_W-1 downto 0);

  -- 8.4.                UART RECEIVE / TRANSMIT
  -- 8.4.1.      Transfer counter
  signal txd_start    : std_logic;
  signal rxd_start    : std_logic;
  signal xfer_bit_inc : std_logic;

  signal xfer_bit : std_logic_vector (3 downto 0);
  signal xfer_cnt : std_logic_vector (DBG_UART_XFER_CNT_W-1 downto 0);

  -- 8.4.2.      Receive/Transmit buffer
  -- 8.4.3.      Generate TXD output
  -- 8.5.                INTERFACE TO DEBUG REGISTERS
  signal dbg_bw     : std_logic;
  signal dbg_din_bw : std_logic;

begin
  P8_UART_COMMUNICATION : block
  begin
    -- 8.1.              UART RECEIVE LINE SYNCHRONIZTION & FILTERING
    -- 8.1.1.    Synchronize RXD input
    sync_dbg_uart_rxd_on : if (SYNC_DBG_UART_RXD = '1') generate
      sync_cell_uart_rxd : omsp_sync_cell
        port map (
          data_out => uart_rxd_n,
          data_in  => not_dbg_uart_rxd,
          clk      => dbg_clk,
          rst      => dbg_rst);

      not_dbg_uart_rxd <= not dbg_uart_rxd;
      uart_rxd         <= not uart_rxd_n;
    end generate sync_dbg_uart_rxd_on;

    sync_dbg_uart_rxd_off : if (SYNC_DBG_UART_RXD = '0') generate
      uart_rxd <= dbg_uart_rxd;
    end generate sync_dbg_uart_rxd_off;

    -- 8.1.2.    RXD input buffer
    R1_2 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        rxd_buf <= "11";
      elsif (rising_edge(dbg_clk)) then
        rxd_buf <= rxd_buf(0) & uart_rxd;
      end if;
    end process R1_2;

    -- 8.1.3.    Majority decision
    rxd_maj_nxt <= (uart_rxd and rxd_buf(0)) or (uart_rxd and rxd_buf(1)) or (rxd_buf(0) and rxd_buf(1));

    R1_2_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        rxd_maj <= '1';
      elsif (rising_edge(dbg_clk)) then
        rxd_maj <= rxd_maj_nxt;
      end if;
    end process R1_2_e;

    rxd_s    <= rxd_maj;
    rxd_fe   <= rxd_maj and not rxd_maj_nxt;
    rxd_re   <= not rxd_maj and rxd_maj_nxt;
    rxd_edge <= rxd_maj xor rxd_maj_nxt;

    -- 8.2.              UART STATE MACHINE
    -- 8.2.1.    Receive state
    -- 8.2.2.    State machine definition
    -- 8.2.3.    State transition
    process(uart_state, re_rx_cmd, re_rx_data2, re_tx_data2)
    begin
      case uart_state is
        when RX_SYNC => uart_state_nxt <= RX_CMD;
        when RX_CMD  => uart_state_nxt <= re_rx_cmd;

        when RX_DATA1 => uart_state_nxt <= RX_DATA2;
        when RX_DATA2 => uart_state_nxt <= re_rx_data2;

        when TX_DATA1 => uart_state_nxt <= TX_DATA2;
        when TX_DATA2 => uart_state_nxt <= re_tx_data2;

        when others => uart_state_nxt <= RX_CMD;
      end case;
    end process;

    re_rx_cmd <= re_0_rx_cmd
                 when mem_burst_wr = '1'              else re_0_tx_cmd
                 when mem_burst_rd = '1'              else re_1_rx_cmd
                 when xfer_buf_nxt(DBG_UART_WR) = '1' else re_1_tx_cmd;

    re_0_rx_cmd <= RX_DATA2
                   when mem_bw = '1' else RX_DATA1;
    re_0_tx_cmd <= TX_DATA2
                   when mem_bw = '1' else TX_DATA1;

    re_1_rx_cmd <= RX_DATA2
                   when xfer_buf_nxt(DBG_UART_BW) = '1' else RX_DATA1;
    re_1_tx_cmd <= TX_DATA2
                   when xfer_buf_nxt(DBG_UART_BW) = '1' else TX_DATA1;

    re_rx_data2 <= re_0_rx_data2
                   when (mem_burst and not mem_burst_end) = '1' else RX_CMD;
    re_tx_data2 <= re_0_tx_data2
                   when (mem_burst and not mem_burst_end) = '1' else RX_CMD;

    re_0_rx_data2 <= RX_DATA2
                     when mem_bw = '1' else RX_DATA1;
    re_0_tx_data2 <= TX_DATA2
                     when mem_bw = '1' else TX_DATA1;

    -- 8.2.4.    State machine
    R_1c : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        uart_state <= RX_SYNC;
      elsif (rising_edge(dbg_clk)) then
        if ((xfer_done or sync_done or mem_burst_wr or mem_burst_rd) = '1') then
          uart_state <= uart_state_nxt;
        end if;
      end if;
    end process R_1c;

    -- 8.2.5.    Utility signals
    cmd_valid <= to_stdlogic(uart_state = RX_CMD) and xfer_done;
    rx_active <= to_stdlogic(uart_state = RX_DATA1) or to_stdlogic(uart_state = RX_DATA2) or to_stdlogic(uart_state = RX_CMD);
    tx_active <= to_stdlogic(uart_state = TX_DATA1) or to_stdlogic(uart_state = TX_DATA2);

    -- 8.3.              UART SYNCHRONIZATION
    R_1c_2c_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        sync_busy <= '0';
      elsif (rising_edge(dbg_clk)) then
        if (uart_state = RX_SYNC and rxd_fe = '1') then
          sync_busy <= '1';
        elsif (uart_state = RX_SYNC and rxd_re = '1') then
          sync_busy <= '0';
        end if;
      end if;
    end process R_1c_2c_e;

    sync_done <= to_stdlogic(uart_state = RX_SYNC) and rxd_re and sync_busy;

    dbg_uart_auto_sync_on : if (DBG_UART_AUTO_SYNC = '1') generate
      R_1c : process (dbg_clk, dbg_rst)
      begin
        if (dbg_rst = '1') then
          sync_cnt <= (DBG_UART_XFER_CNT_W + 2 downto 3 => '1', 2 downto 0 => '0');
        elsif (rising_edge(dbg_clk)) then
          if ((sync_busy or (not sync_busy and sync_cnt(2))) = '1') then
            sync_cnt <= std_logic_vector(unsigned(sync_cnt) + to_unsigned(1, DBG_UART_XFER_CNT_W+3));
          end if;
        end if;
      end process R_1c;

      bit_cnt_max <= sync_cnt(DBG_UART_XFER_CNT_W + 2 downto 3);
    end generate dbg_uart_auto_sync_on;

    dbg_uart_auto_sync_off : if (DBG_UART_AUTO_SYNC = '0') generate
      bit_cnt_max <= DBG_UART_CNTB;
    end generate dbg_uart_auto_sync_off;

    -- 8.4.              UART RECEIVE / TRANSMIT
    -- 8.4.1.    Transfer counter
    txd_start    <= dbg_rd_rdy or (xfer_done and to_stdlogic(uart_state = TX_DATA1));
    rxd_start    <= to_stdlogic(xfer_bit = X"0") and rxd_fe and to_stdlogic(uart_state /= RX_SYNC);
    xfer_bit_inc <= to_stdlogic(xfer_bit /= X"0") and to_stdlogic(xfer_cnt = (0 to DBG_UART_XFER_CNT_W - 1 => '0'));
    xfer_done    <= to_stdlogic(xfer_bit = "1010") when rx_active = '1' else to_stdlogic(xfer_bit = "1011");

    R1_1c_2c_3c : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        xfer_bit <= "0000";
      elsif (rising_edge(dbg_clk)) then
        if ((txd_start or rxd_start) = '1') then
          xfer_bit <= "0001";
        elsif (xfer_done = '1') then
          xfer_bit <= "0000";
        elsif (xfer_bit_inc = '1') then
          xfer_bit <= std_logic_vector(unsigned(xfer_bit) + "0001");
        end if;
      end if;
    end process R1_1c_2c_3c;

    R2_1c_2c_3c : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        xfer_cnt <= (DBG_UART_XFER_CNT_W - 1 downto 0 => '0');
      elsif (rising_edge(dbg_clk)) then
        if ((rx_active and rxd_edge) = '1') then
          xfer_cnt <= ('0' & bit_cnt_max(DBG_UART_XFER_CNT_W - 1 downto 1));
        elsif ((txd_start or xfer_bit_inc) = '1') then
          xfer_cnt <= bit_cnt_max;
        elsif (reduce_or(xfer_cnt) = '1') then
          xfer_cnt <= std_logic_vector(unsigned(xfer_cnt) + (0 to DBG_UART_XFER_CNT_W - 1 => '1'));
        end if;
      end if;
    end process R2_1c_2c_3c;

    -- 8.4.2.    Receive/Transmit buffer
    xfer_buf_nxt <= rxd_s & xfer_buf(19 downto 1);

    R_1c_2c : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        xfer_buf <= (19 downto 0 => '0');
      elsif (rising_edge(dbg_clk)) then
        if (dbg_rd_rdy = '1') then
          xfer_buf <= ('1' & dbg_dout(15 downto 8) & "01" & dbg_dout(7 downto 0) & '0');
        elsif (xfer_bit_inc = '1') then
          xfer_buf <= xfer_buf_nxt;
        end if;
      end if;
    end process R_1c_2c;

    -- 8.4.3.    Generate TXD output
    R_1c_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        dbg_uart_txd <= '1';
      elsif (rising_edge(dbg_clk)) then
        if ((xfer_bit_inc and tx_active) = '1') then
          dbg_uart_txd <= xfer_buf(0);
        end if;
      end if;
    end process R_1c_e;

    -- 8.5.              INTERFACE TO DEBUG REGISTERS
    R2_1c : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        dbg_addr <= "000000";
      elsif (rising_edge(dbg_clk)) then
        if (cmd_valid = '1') then
          dbg_addr <= xfer_buf_nxt(16 downto 11);
        end if;
      end if;
    end process R2_1c;

    R2_1c_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        dbg_bw <= '0';
      elsif (rising_edge(dbg_clk)) then
        if (cmd_valid = '1') then
          dbg_bw <= xfer_buf_nxt(DBG_UART_BW);
        end if;
      end if;
    end process R2_1c_e;

    dbg_din_bw <= mem_bw
                  when mem_burst = '1' else dbg_bw;
    dbg_din <= X"00" & xfer_buf_nxt(18 downto 11)
               when dbg_din_bw = '1' else xfer_buf_nxt(18 downto 11) & xfer_buf_nxt(9 downto 2);
    dbg_wr <= xfer_done and to_stdlogic(uart_state = RX_DATA2);
    dbg_rd <= xfer_done and to_stdlogic(uart_state = TX_DATA2)
              when mem_burst = '1' else (cmd_valid and not xfer_buf_nxt(DBG_UART_WR)) or mem_burst_rd;
  end block P8_UART_COMMUNICATION;
end omsp_dbg_uart_ARQ;
