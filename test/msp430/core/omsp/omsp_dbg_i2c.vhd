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

entity omsp_dbg_i2c is
  port (
    dbg_i2c_sda_out : out std_logic;
    dbg_rd          : out std_logic;
    dbg_wr          : out std_logic;
    dbg_addr        : out std_logic_vector (5 downto 0);
    dbg_din         : out std_logic_vector (15 downto 0);

    dbg_clk           : in std_logic;
    dbg_i2c_scl       : in std_logic;
    dbg_i2c_sda_in    : in std_logic;
    dbg_rd_rdy        : in std_logic;
    dbg_rst           : in std_logic;
    mem_burst         : in std_logic;
    mem_burst_end     : in std_logic;
    mem_burst_rd      : in std_logic;
    mem_burst_wr      : in std_logic;
    mem_bw            : in std_logic;
    dbg_i2c_addr      : in std_logic_vector (6 downto 0);
    dbg_i2c_broadcast : in std_logic_vector (6 downto 0);
    dbg_dout          : in std_logic_vector (15 downto 0));
end omsp_dbg_i2c;

architecture omsp_dbg_i2c_ARQ of omsp_dbg_i2c is

  -- 9.          I2C_COMMUNICATION       
  -- 9.3.                I2C STATE MACHINE       
  -- 9.3.3.      State machine definition
  constant RX_ADDR     : std_logic_vector (2 downto 0) := "000";
  constant RX_ADDR_ACK : std_logic_vector (2 downto 0) := "001";
  constant RX_DATA     : std_logic_vector (2 downto 0) := "010";
  constant RX_DATA_ACK : std_logic_vector (2 downto 0) := "011";
  constant TX_DATA     : std_logic_vector (2 downto 0) := "100";
  constant TX_DATA_ACK : std_logic_vector (2 downto 0) := "101";
  -- 9.4.                I2C SHIFT REGISTER (FOR RECEIVING & TRANSMITING)
  -- 9.6.3.      State machine definition
  constant RX_CMD      : std_logic_vector (2 downto 0) := "000";
  constant RX_BYTE_LO  : std_logic_vector (2 downto 0) := "001";
  constant RX_BYTE_HI  : std_logic_vector (2 downto 0) := "010";
  constant TX_BYTE_LO  : std_logic_vector (2 downto 0) := "011";
  constant TX_BYTE_HI  : std_logic_vector (2 downto 0) := "100";

  -- 9.7.                REGISTER READ/WRITE ACCESS
  constant MEM_DATA : std_logic_vector (5 downto 0) := "000110";

  -- 9.          I2C_COMMUNICATION
  -- 9.1.                I2C RECEIVE LINE SYNCHRONIZTION & FILTERING
  -- 9.1.1.      Synchronize SCL/SDA inputs
  signal scl_sync_n         : std_logic;
  signal scl_sync           : std_logic;
  signal sda_in_sync_n      : std_logic;
  signal sda_in_sync        : std_logic;
  signal not_dbg_i2c_scl    : std_logic;
  signal not_dbg_i2c_sda_in : std_logic;

  -- 9.1.2.      SCL/SDA input buffers
  signal scl_buf    : std_logic_vector (1 downto 0);
  signal sda_in_buf : std_logic_vector (1 downto 0);

  -- 9.1.3.      SCL/SDA Majority decision
  signal scl    : std_logic;
  signal sda_in : std_logic;

  -- 9.1.4.      SCL/SDA Edge detection
  signal sda_in_dly : std_logic;
  signal sda_in_fe  : std_logic;
  signal sda_in_re  : std_logic;
  signal scl_dly    : std_logic;
  signal scl_fe     : std_logic;
  signal scl_re     : std_logic;
  signal scl_sample : std_logic;
  signal scl_re_dly : std_logic_vector (1 downto 0);

  -- 9.2.                I2C START & STOP CONDITION DETECTION
  -- 9.2.1.      Start condition
  signal start_detect : std_logic;

  -- 9.2.2.      Stop condition
  signal stop_detect : std_logic;

  -- 9.2.3.      I2C Slave Active
  signal i2c_addr_not_valid : std_logic;
  signal i2c_active_seq     : std_logic;
  signal i2c_active         : std_logic;
  signal i2c_init           : std_logic;

  -- 9.3.                I2C STATE MACHINE
  signal re_rx_addr     : std_logic_vector (2 downto 0);
  signal re_rx_addr_ack : std_logic_vector (2 downto 0);
  signal re_rx_data     : std_logic_vector (2 downto 0);
  signal re_rx_data_ack : std_logic_vector (2 downto 0);
  signal re_tx_data     : std_logic_vector (2 downto 0);
  signal re_tx_data_ack : std_logic_vector (2 downto 0);

  -- 9.3.1.      State register/wires
  signal i2c_state     : std_logic_vector (2 downto 0);
  signal i2c_state_nxt : std_logic_vector (2 downto 0);

  -- 9.3.2.      Utility signals
  signal shift_rx_done : std_logic;
  signal shift_tx_done : std_logic;
  signal shift_buf     : std_logic_vector (8 downto 0);

  -- 9.3.3.      State machine definition        
  -- 9.3.4.      State transition
  -- 9.3.5.      State machine
  -- 9.4.                I2C SHIFT REGISTER (FOR RECEIVING & TRANSMITING)
  signal shift_rx_en       : std_logic;
  signal shift_tx_en       : std_logic;
  signal shift_tx_en_pre   : std_logic;
  signal shift_buf_rx_init : std_logic;
  signal shift_buf_rx_en   : std_logic;
  signal shift_buf_tx_en   : std_logic;
  signal shift_buf_tx_init : std_logic;
  signal shift_tx_val      : std_logic_vector (7 downto 0);
  signal shift_buf_nxt     : std_logic_vector (8 downto 0);

  -- 9.4.1.      Detect when the received I2C device address is not valid
  -- 9.4.2.      Utility signals
  signal shift_rx_data_done : std_logic;
  signal shift_tx_data_done : std_logic;

  -- 9.5.                I2C TRANSMIT BUFFER
  -- 9.6.                DEBUG INTERFACE STATE MACHINE
  signal re_rx_cmd     : std_logic_vector (2 downto 0);
  signal re_rx_byte_lo : std_logic_vector (2 downto 0);
  signal re_rx_byte_hi : std_logic_vector (2 downto 0);
  signal re_tx_byte_lo : std_logic_vector (2 downto 0);
  signal re_tx_byte_hi : std_logic_vector (2 downto 0);
  signal re_0_rx_cmd   : std_logic_vector (2 downto 0);

  -- 9.6.1.      State register/wires
  signal dbg_state     : std_logic_vector (2 downto 0);
  signal dbg_state_nxt : std_logic_vector (2 downto 0);

  -- 9.6.2.      Utility signals
  signal dbg_bw : std_logic;

  -- 9.6.3.      State machine definition
  -- 9.6.4.      State transition
  -- 9.6.5.      State machine
  -- 9.6.6.      Utility signals
  signal cmd_valid   : std_logic;
  signal rx_lo_valid : std_logic;
  signal rx_hi_valid : std_logic;

  -- 9.7.                REGISTER READ/WRITE ACCESS      
  -- 9.7.1.      Debug register address & bit width      
  -- 9.7.2.      Debug register data input
  signal dbg_din_lo : std_logic_vector (7 downto 0);
  signal dbg_din_hi : std_logic_vector (7 downto 0);

  -- 9.7.3.      Debug register data write command       
  signal data_write_command : std_logic;

  -- 9.7.4.      Debug register data read command
  signal data_read_command : std_logic;

  -- 9.7.5.      Debug register data read value

begin
  P9_I2C_COMMUNICATION : block
  begin
    -- 9.1.              I2C RECEIVE LINE SYNCHRONIZTION & FILTERING
    -- 9.1.1.    Synchronize SCL/SDA inputs
    sync_cell_i2c_scl : omsp_sync_cell
      port map (
        data_out => scl_sync_n,
        data_in  => not_dbg_i2c_scl,
        clk      => dbg_clk,
        rst      => dbg_rst);

    not_dbg_i2c_scl <= not dbg_i2c_scl;
    scl_sync        <= not scl_sync_n;

    sync_cell_i2c_sda : omsp_sync_cell
      port map (
        data_out => sda_in_sync_n,
        data_in  => not_dbg_i2c_sda_in,
        clk      => dbg_clk,
        rst      => dbg_rst);

    not_dbg_i2c_sda_in <= not dbg_i2c_sda_in;
    sda_in_sync        <= not sda_in_sync_n;

    -- 9.1.2.    SCL/SDA input buffers
    R1_1 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        scl_buf <= "11";
      elsif (rising_edge(dbg_clk)) then
        scl_buf <= scl_buf(0) & scl_sync;
      end if;
    end process R1_1;

    R1_2 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        sda_in_buf <= "11";
      elsif (rising_edge(dbg_clk)) then
        sda_in_buf <= sda_in_buf(0) & sda_in_sync;
      end if;
    end process R1_2;

    -- 9.1.3.    SCL/SDA Majority decisi\F3n
    scl    <= (scl_sync and scl_buf(0)) or (scl_sync and scl_buf(1)) or (scl_buf(0) and scl_buf(1));
    sda_in <= (sda_in_sync and sda_in_buf(0)) or (sda_in_sync and sda_in_buf(1)) or (sda_in_buf(0) and sda_in_buf(1));

    -- 9.1.4.    SCL/SDA Edge detection
    R1_1_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        sda_in_dly <= '1';
      elsif (rising_edge(dbg_clk)) then
        sda_in_dly <= sda_in;
      end if;
    end process R1_1_e;

    sda_in_fe <= sda_in_dly and not sda_in;
    sda_in_re <= not sda_in_dly and sda_in;

    R2_1_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        scl_dly <= '1';
      elsif (rising_edge(dbg_clk)) then
        scl_dly <= scl;
      end if;
    end process R2_1_e;

    scl_fe <= scl_dly and not scl;
    scl_re <= not scl_dly and scl;

    R3_2 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        scl_re_dly <= "00";
      elsif (rising_edge(dbg_clk)) then
        scl_re_dly <= scl_re_dly(0) & scl_re;
      end if;
    end process R3_2;

    scl_sample <= scl_re_dly(1);

    -- 9.2.              I2C START & STOP CONDITION DETECTION
    -- 9.2.1.    Start condition
    start_detect <= sda_in_fe and scl;

    -- 9.2.2.    Stop condition
    stop_detect <= sda_in_re and scl;

    -- 9.2.3.    I2C Slave Active
    R_1c_2c_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        i2c_active_seq <= '0';
      elsif (rising_edge(dbg_clk)) then
        if (start_detect = '1') then
          i2c_active_seq <= '1';
        elsif ((stop_detect or i2c_addr_not_valid) = '1') then
          i2c_active_seq <= '0';
        end if;
      end if;
    end process R_1c_2c_e;

    i2c_active <= i2c_active_seq and not stop_detect;
    i2c_init   <= not i2c_active or start_detect;

    -- 9.3.              I2C STATE MACHINE
    -- 9.3.1.    State register/wires
    -- 9.3.2.    Utility signals
    -- 9.3.3.    State machine definition
    -- 9.3.4.    State transition
    process(i2c_state, re_rx_addr, re_rx_addr_ack, re_rx_data, re_rx_data_ack, re_tx_data, re_tx_data_ack)
    begin
      case i2c_state is
        when RX_ADDR     => i2c_state_nxt <= re_rx_addr;
        when RX_ADDR_ACK => i2c_state_nxt <= re_rx_addr_ack;
        when RX_DATA     => i2c_state_nxt <= re_rx_data;
        when RX_DATA_ACK => i2c_state_nxt <= re_rx_data_ack;
        when TX_DATA     => i2c_state_nxt <= re_tx_data;
        when TX_DATA_ACK => i2c_state_nxt <= re_tx_data_ack;
        when others      => i2c_state_nxt <= RX_ADDR;
      end case;
    end process;

    re_rx_addr <= RX_ADDR
                  when i2c_init = '1'           else RX_ADDR
                  when shift_rx_done = '0'      else RX_ADDR
                  when i2c_addr_not_valid = '1' else RX_ADDR_ACK;

    re_rx_addr_ack <= RX_ADDR
                      when i2c_init = '1'     else RX_ADDR_ACK
                      when scl_fe = '0'       else TX_DATA
                      when shift_buf(0) = '1' else RX_DATA;

    re_rx_data <= RX_ADDR
                  when i2c_init = '1'      else RX_DATA
                  when shift_rx_done = '0' else RX_DATA_ACK;

    re_rx_data_ack <= RX_ADDR
                      when i2c_init = '1' else RX_DATA_ACK
                      when scl_fe = '0'   else RX_DATA;

    re_tx_data <= RX_ADDR
                  when i2c_init = '1'      else TX_DATA
                  when shift_tx_done = '0' else TX_DATA_ACK;

    re_tx_data_ack <= RX_ADDR
                      when i2c_init = '1' else TX_DATA_ACK
                      when scl_fe = '0'   else TX_DATA
                      when sda_in = '0'   else RX_ADDR;

    -- 9.3.5.    State machine   
    R4_2 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        i2c_state <= RX_ADDR;
      elsif (rising_edge(dbg_clk)) then
        i2c_state <= i2c_state_nxt;
      end if;
    end process R4_2;

    -- 9.4.              I2C SHIFT REGISTER (FOR RECEIVING & TRANSMITING)
    shift_rx_en       <= (to_stdlogic(i2c_state = RX_ADDR) or to_stdlogic(i2c_state = RX_DATA) or to_stdlogic(i2c_state = RX_DATA_ACK));
    shift_tx_en       <= to_stdlogic(i2c_state = TX_DATA) or to_stdlogic(i2c_state = TX_DATA_ACK);
    shift_tx_en_pre   <= to_stdlogic(i2c_state_nxt = TX_DATA) or to_stdlogic(i2c_state_nxt = TX_DATA_ACK);
    shift_rx_done     <= shift_rx_en and scl_fe and shift_buf(8);
    shift_tx_done     <= shift_tx_en and scl_fe and to_stdlogic(shift_buf = "100000000");
    shift_buf_rx_init <= i2c_init or (to_stdlogic(i2c_state = RX_ADDR_ACK) and scl_fe and not shift_buf(0))
                         or (to_stdlogic(i2c_state = RX_DATA_ACK) and scl_fe);
    shift_buf_rx_en   <= shift_rx_en and scl_sample;
    shift_buf_tx_init <= (to_stdlogic(i2c_state = RX_ADDR_ACK) and scl_re and shift_buf(0)) or (to_stdlogic(i2c_state = TX_DATA_ACK) and scl_re);
    shift_buf_tx_en   <= shift_tx_en_pre and scl_fe and not to_stdlogic(shift_buf = "100000000");
    shift_buf_nxt     <= "000000001"
                     when shift_buf_rx_init = '1' else shift_tx_val & '1'
                     when shift_buf_tx_init = '1' else shift_buf(7 downto 0) & sda_in
                     when shift_buf_rx_en = '1'   else shift_buf(7 downto 0) & '0'
                     when shift_buf_tx_en = '1'   else shift_buf(8 downto 0);

    R5_2 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        shift_buf <= "000000001";
      elsif (rising_edge(dbg_clk)) then
        shift_buf <= shift_buf_nxt;
      end if;
    end process R5_2;

    -- 9.4.1.    Detect when the received I2C device address is not valid                        
    dbg_i2c_broadcastc_on : if (DBG_I2C_BROADCASTC = '1') generate
      i2c_addr_not_valid <= to_stdlogic(i2c_state = RX_ADDR) and shift_rx_done and
                            to_stdlogic(shift_buf(7 downto 1) /= dbg_i2c_addr(6 downto 0)) and
                            to_stdlogic(shift_buf(7 downto 1) /= dbg_i2c_broadcast(6 downto 0));
    end generate dbg_i2c_broadcastc_on;

    dbg_i2c_broadcastc_off : if (DBG_I2C_BROADCASTC = '0') generate
      i2c_addr_not_valid <= to_stdlogic(i2c_state = RX_ADDR) and shift_rx_done and
                            to_stdlogic(shift_buf(7 downto 1) /= dbg_i2c_addr(6 downto 0));
    end generate dbg_i2c_broadcastc_off;

    -- 9.4.2.    Utility signals
    shift_rx_data_done <= shift_rx_done and to_stdlogic(i2c_state = RX_DATA);
    shift_tx_data_done <= shift_tx_done;

    -- 9.5.              I2C TRANSMIT BUFFER
    R3_1c_e : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        dbg_i2c_sda_out <= '1';
      elsif (rising_edge(dbg_clk)) then
        if (scl_fe = '1') then
          dbg_i2c_sda_out <= not
                             ((to_stdlogic(i2c_state_nxt = RX_ADDR_ACK)) or
                              (to_stdlogic(i2c_state_nxt = RX_DATA_ACK)) or
                              (shift_buf_tx_en and not shift_buf(8)));
        end if;
      end if;
    end process R3_1c_e;

    -- 9.6.              DEBUG INTERFACE STATE MACHINE
    -- 9.6.1.    State register/wires
    -- 9.6.2.    Utility signals
    -- 9.6.3.    State machine definition
    -- 9.6.4.    State transition
    process(dbg_state, re_rx_byte_hi, re_rx_byte_lo, re_rx_cmd, re_tx_byte_hi, re_tx_byte_lo)
    begin
      case dbg_state is
        when RX_CMD     => dbg_state_nxt <= re_rx_cmd;
        when RX_BYTE_LO => dbg_state_nxt <= re_rx_byte_lo;
        when RX_BYTE_HI => dbg_state_nxt <= re_rx_byte_hi;
        when TX_BYTE_LO => dbg_state_nxt <= re_tx_byte_lo;
        when TX_BYTE_HI => dbg_state_nxt <= re_tx_byte_hi;
        when others     => dbg_state_nxt <= RX_CMD;
      end case;
    end process;

    re_rx_cmd <= RX_BYTE_LO
                 when mem_burst_wr = '1'       else TX_BYTE_LO
                 when mem_burst_rd = '1'       else RX_CMD
                 when shift_rx_data_done = '0' else RX_BYTE_LO
                 when shift_buf(7) = '1'       else TX_BYTE_LO;

    re_rx_byte_lo <= RX_CMD
                     when (mem_burst and mem_burst_end) = '1'     else RX_BYTE_LO
                     when shift_rx_data_done = '0'                else re_0_rx_cmd
                     when (mem_burst and not mem_burst_end) = '1' else RX_CMD
                     when dbg_bw = '1'                            else RX_BYTE_HI;

    re_rx_byte_hi <= RX_BYTE_HI
                     when shift_rx_data_done = '0'                else RX_BYTE_LO
                     when (mem_burst and not mem_burst_end) = '1' else RX_CMD;

    re_tx_byte_lo <= TX_BYTE_LO
                     when shift_tx_data_done = '0'         else TX_BYTE_LO
                     when (mem_burst and mem_bw) = '1'     else TX_BYTE_HI
                     when (mem_burst and not mem_bw) = '1' else TX_BYTE_HI
                     when dbg_bw = '0'                     else RX_CMD;

    re_tx_byte_hi <= TX_BYTE_HI
                     when shift_tx_data_done = '0' else TX_BYTE_LO
                     when mem_burst = '1'          else RX_CMD;

    re_0_rx_cmd <= RX_BYTE_LO
                   when mem_bw = '1' else RX_BYTE_HI;

    -- 9.6.5.    State machine
    R6_2 : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        dbg_state <= RX_CMD;
      elsif (rising_edge(dbg_clk)) then
        dbg_state <= dbg_state_nxt;
      end if;
    end process R6_2;

    -- 9.6.6.    Utility signals
    cmd_valid   <= to_stdlogic(dbg_state = RX_CMD) and shift_rx_data_done;
    rx_lo_valid <= to_stdlogic(dbg_state = RX_BYTE_LO) and shift_rx_data_done;
    rx_hi_valid <= to_stdlogic(dbg_state = RX_BYTE_HI) and shift_rx_data_done;

    -- 9.7.              REGISTER READ/WRITE ACCESS
    -- 9.7.1.    Debug register address & bit width
    process (dbg_rst, dbg_clk)
    begin
      if (dbg_rst = '1') then
        dbg_bw   <= '0';
        dbg_addr <= (5 downto 0 => '0');
      elsif (rising_edge(dbg_clk)) then
        if (cmd_valid = '1') then
          dbg_bw   <= shift_buf(6);
          dbg_addr <= shift_buf(5 downto 0);
        elsif (mem_burst = '1') then
          dbg_bw   <= mem_bw;
          dbg_addr <= MEM_DATA;
        end if;
      end if;
    end process;

    -- 9.7.2.    Debug register data input
    R_1c : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        dbg_din_lo <= X"00";
      elsif (rising_edge(dbg_clk)) then
        if (rx_lo_valid = '1') then
          dbg_din_lo <= shift_buf(7 downto 0);
        end if;
      end if;
    end process R_1c;

    R_1c_2c : process (dbg_clk, dbg_rst)
    begin
      if (dbg_rst = '1') then
        dbg_din_hi <= X"00";
      elsif (rising_edge(dbg_clk)) then
        if (rx_lo_valid = '1') then
          dbg_din_hi <= X"00";
        elsif (rx_hi_valid = '1') then
          dbg_din_hi <= shift_buf(7 downto 0);
        end if;
      end if;
    end process R_1c_2c;

    dbg_din <= dbg_din_hi & dbg_din_lo;

    -- 9.7.3.    Debug register data write command
    process (dbg_rst, dbg_clk)
    begin
      if (dbg_rst = '1') then
        dbg_wr <= '0';
      elsif (rising_edge(dbg_clk)) then
        dbg_wr <= data_write_command;
      end if;
    end process;

    data_write_command <= rx_lo_valid
                          when (mem_burst and mem_bw) = '1'     else rx_hi_valid
                          when (mem_burst and not mem_bw) = '1' else rx_lo_valid
                          when dbg_bw = '1'                     else rx_hi_valid;

    -- 9.7.4.    Debug register data read command
    process (dbg_rst, dbg_clk)
    begin
      if (dbg_rst = '1') then
        dbg_rd <= '0';
      elsif (rising_edge(dbg_clk)) then
        dbg_rd <= data_read_command;
      end if;
    end process;

    data_read_command <= shift_tx_data_done and to_stdlogic(dbg_state = TX_BYTE_LO)
                         when (mem_burst and mem_bw) = '1'     else shift_tx_data_done and to_stdlogic(dbg_state = TX_BYTE_HI)
                         when (mem_burst and not mem_bw) = '1' else not shift_buf(7)
                         when cmd_valid = '1'                  else '0';
    -- 9.7.5.    Debug register data read value
    shift_tx_val <= dbg_dout(15 downto 8) when dbg_state = TX_BYTE_HI else dbg_dout(7 downto 0);
  end block P9_I2C_COMMUNICATION;
end omsp_dbg_i2c_ARQ;
