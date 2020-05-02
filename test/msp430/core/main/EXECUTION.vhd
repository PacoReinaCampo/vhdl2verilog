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

entity EXECUTION is
  port (
    r0  : out std_logic_vector (15 downto 0);
    r1  : out std_logic_vector (15 downto 0);
    r2  : out std_logic_vector (15 downto 0);
    r3  : out std_logic_vector (15 downto 0);
    r4  : out std_logic_vector (15 downto 0);
    r5  : out std_logic_vector (15 downto 0);
    r6  : out std_logic_vector (15 downto 0);
    r7  : out std_logic_vector (15 downto 0);
    r8  : out std_logic_vector (15 downto 0);
    r9  : out std_logic_vector (15 downto 0);
    r10 : out std_logic_vector (15 downto 0);
    r11 : out std_logic_vector (15 downto 0);
    r12 : out std_logic_vector (15 downto 0);
    r13 : out std_logic_vector (15 downto 0);
    r14 : out std_logic_vector (15 downto 0);
    r15 : out std_logic_vector (15 downto 0);

    cpuoff      : out std_logic;
    gie         : out std_logic;
    mb_en       : out std_logic;
    oscoff      : out std_logic;
    pc_sw_wr    : out std_logic;
    scg0        : out std_logic;
    scg1        : out std_logic;
    mb_wr       : out std_logic_vector (1 downto 0);
    dbg_reg_din : out std_logic_vector (15 downto 0);
    mab         : out std_logic_vector (15 downto 0);
    mdb_out     : out std_logic_vector (15 downto 0);
    pc_sw       : out std_logic_vector (15 downto 0);

    dbg_halt_st  : in std_logic;
    dbg_reg_wr   : in std_logic;
    exec_done    : in std_logic;
    inst_bw      : in std_logic;
    inst_irq_rst : in std_logic;
    inst_mov     : in std_logic;
    mclk         : in std_logic;
    puc_rst      : in std_logic;
    scan_enable  : in std_logic;
    inst_type    : in std_logic_vector (2 downto 0);
    e_state      : in std_logic_vector (3 downto 0);
    inst_ad      : in std_logic_vector (7 downto 0);
    inst_as      : in std_logic_vector (7 downto 0);
    inst_jmp     : in std_logic_vector (7 downto 0);
    inst_so      : in std_logic_vector (7 downto 0);
    inst_alu     : in std_logic_vector (11 downto 0);
    dbg_mem_dout : in std_logic_vector (15 downto 0);
    inst_dest    : in std_logic_vector (15 downto 0);
    inst_dext    : in std_logic_vector (15 downto 0);
    inst_sext    : in std_logic_vector (15 downto 0);
    inst_src     : in std_logic_vector (15 downto 0);
    mdb_in       : in std_logic_vector (15 downto 0);
    pc           : in std_logic_vector (15 downto 0);
    pc_nxt       : in std_logic_vector (15 downto 0));
end EXECUTION;

architecture EXECUTION_ARQ of EXECUTION is

  --0.INTERNAL WIRES/REGISTERS/PARAMETERS DECLARATION
  signal alu_stat    : std_logic_vector (3 downto 0);
  signal alu_stat_wr : std_logic_vector (3 downto 0);
  signal status      : std_logic_vector (3 downto 0);
  signal alu_out     : std_logic_vector (15 downto 0);
  signal alu_out_add : std_logic_vector (15 downto 0);
  signal op_dst      : std_logic_vector (15 downto 0);
  signal op_src      : std_logic_vector (15 downto 0);
  signal reg_dest    : std_logic_vector (15 downto 0);
  signal reg_src     : std_logic_vector (15 downto 0);
  signal mdb_in_bw   : std_logic_vector (15 downto 0);
  signal mdb_in_val  : std_logic_vector (15 downto 0);

  --1.REGISTER FILE
  signal reg_dest_wr : std_logic;
  signal reg_sp_wr   : std_logic;
  signal reg_sr_wr   : std_logic;
  signal reg_sr_clr  : std_logic;
  signal reg_pc_call : std_logic;
  signal reg_incr    : std_logic;

  --2.SOURCE OPERAND MUXING
  signal src_reg_src_sel    : std_logic;
  signal src_reg_dest_sel   : std_logic;
  signal src_mdb_in_val_sel : std_logic;
  signal src_inst_dext_sel  : std_logic;
  signal src_inst_sext_sel  : std_logic;

  --3.DESTINATION OPERAND MUXING
  signal dst_inst_sext_sel : std_logic;
  signal dst_mdb_in_bw_sel : std_logic;
  signal dst_fffe_sel      : std_logic;
  signal dst_reg_dest_sel  : std_logic;

  --4.ALU
  signal exec_cycle : std_logic;

  --5.MEMORY INTERFACE
  --Detect memory read/write access
  signal mb_rd_det : std_logic;
  signal mb_wr_det : std_logic;
  signal mb_wr_msk : std_logic_vector (1 downto 0);

  --Memory data bus output
  signal mdb_out_nxt_en   : std_logic;
  signal mclk_mdb_out_nxt : std_logic;
  signal mdb_out_nxt      : std_logic_vector (15 downto 0);

  --Format memory data bus input depending on BW
  signal mab_lsb : std_logic;

  --Memory data bus input buffer (buffer after a source read)
  signal mdb_in_buf_en    : std_logic;
  signal mdb_in_buf_valid : std_logic;
  signal mclk_mdb_in_buf  : std_logic;
  signal mdb_in_buf       : std_logic_vector (15 downto 0);

begin
  C1_REGISTER_FILE : block
  begin
    reg_dest_wr <= (to_stdlogic(e_state = E_EXEC) and (
      (inst_type(INST_TOC) and inst_ad(DIR) and not inst_alu(EXEC_NO_WR)) or
      (inst_type(INST_SOC) and inst_as(DIR) and not (inst_so(PUSH) or inst_so(CALL) or inst_so(RETI))) or
      inst_type(INST_JMPC))) or dbg_reg_wr;

    reg_sp_wr <= ((to_stdlogic(e_state = E_IRQ(1)) or to_stdlogic(e_state = E_IRQ(3))) and not inst_irq_rst) or
                 (to_stdlogic(e_state = E_DST_RD) and ((inst_so(PUSH) or inst_so(CALL)) and not inst_as(IDX) and not ((inst_as(INDIR) or inst_as(INDIR_I)) and inst_src(1)))) or
                 (to_stdlogic(e_state = E_SRC_AD) and ((inst_so(PUSH) or inst_so(CALL)) and inst_as(IDX))) or
                 (to_stdlogic(e_state = E_SRC_RD) and ((inst_so(PUSH) or inst_so(CALL)) and ((inst_as(INDIR) or inst_as(INDIR_I)) and inst_src(1))));
    reg_sr_wr   <= to_stdlogic(e_state = E_DST_RD) and inst_so(RETI);
    reg_sr_clr  <= to_stdlogic(e_state = E_IRQ(2));
    reg_pc_call <= (to_stdlogic(e_state = E_EXEC) and inst_so(CALL)) or
                   (to_stdlogic(e_state = E_DST_WR) and inst_so(RETI));
    reg_incr <= (exec_done and inst_as(INDIR_I)) or
                (to_stdlogic(e_state = E_SRC_RD) and inst_so(RETI)) or
                (to_stdlogic(e_state = E_EXEC) and inst_so(RETI));
    dbg_reg_din <= reg_dest;

    register_file_0 : omsp_register_file
      port map (
        r0  => r0,
        r1  => r1,
        r2  => r2,
        r3  => r3,
        r4  => r4,
        r5  => r5,
        r6  => r6,
        r7  => r7,
        r8  => r8,
        r9  => r9,
        r10 => r10,
        r11 => r11,
        r12 => r12,
        r13 => r13,
        r14 => r14,
        r15 => r15,

        cpuoff   => cpuoff,
        gie      => gie,
        oscoff   => oscoff,
        pc_sw_wr => pc_sw_wr,
        scg0     => scg0,
        scg1     => scg1,
        status   => status,
        pc_sw    => pc_sw,
        reg_dest => reg_dest,
        reg_src  => reg_src,

        inst_bw      => inst_bw,
        mclk         => mclk,
        puc_rst      => puc_rst,
        reg_dest_wr  => reg_dest_wr,
        reg_pc_call  => reg_pc_call,
        reg_sp_wr    => reg_sp_wr,
        reg_sr_wr    => reg_sr_wr,
        reg_sr_clr   => reg_sr_clr,
        reg_incr     => reg_incr,
        scan_enable  => scan_enable,
        alu_stat     => alu_stat,
        alu_stat_wr  => alu_stat_wr,
        inst_dest    => inst_dest,
        inst_src     => inst_src,
        pc           => pc,
        reg_dest_val => alu_out,
        reg_sp_val   => alu_out_add);
  end block C1_REGISTER_FILE;

  C2_SOURCE_OPERAND_MUXING : block
  begin
    src_reg_src_sel <= to_stdlogic(e_state = E_IRQ(0)) or
                       to_stdlogic(e_state = E_IRQ(2)) or
                       (to_stdlogic(e_state = E_SRC_RD) and not inst_as(ABSC)) or
                       (to_stdlogic(e_state = E_SRC_WR) and not inst_as(ABSC)) or
                       (to_stdlogic(e_state = E_EXEC) and inst_as(DIR) and not inst_type(INST_JMPC));
    src_reg_dest_sel <= to_stdlogic(e_state = E_IRQ(1)) or
                        to_stdlogic(e_state = E_IRQ(3)) or
                        (to_stdlogic(e_state = E_DST_RD) and (inst_so(PUSH) or inst_so(CALL))) or
                        (to_stdlogic(e_state = E_SRC_AD) and (inst_so(PUSH) or inst_so(CALL)) and inst_as(IDX));

    src_mdb_in_val_sel <= (to_stdlogic(e_state = E_DST_RD) and inst_so(RETI)) or
                          (to_stdlogic(e_state = E_EXEC) and (inst_as(INDIR) or inst_as(INDIR_I) or
                                                              inst_as(IDX) or inst_as(SYMB) or
                                                              inst_as(ABSC)));

    src_inst_dext_sel <= (to_stdlogic(e_state = E_DST_RD) and not (inst_so(PUSH) or inst_so(CALL))) or
                         (to_stdlogic(e_state = E_DST_WR) and not (inst_so(PUSH) or inst_so(CALL) or
                                                                   inst_so(RETI)));
    src_inst_sext_sel <= (to_stdlogic(e_state = E_EXEC) and (inst_type(INST_JMPC) or inst_as(IMM) or
                                                             inst_as(CONST) or inst_so(RETI)));
    op_src <= reg_src
              when src_reg_src_sel = '1'    else reg_dest
              when src_reg_dest_sel = '1'   else mdb_in_val
              when src_mdb_in_val_sel = '1' else inst_dext
              when src_inst_dext_sel = '1'  else inst_sext
              when src_inst_sext_sel = '1'  else (others => '0');
  end block C2_SOURCE_OPERAND_MUXING;

  C3_DESTINATION_OPERAND_MUXING : block
  begin
    dst_inst_sext_sel <= (to_stdlogic(e_state = E_SRC_RD) and (inst_as(IDX) or inst_as(SYMB) or inst_as(ABSC))) or
                         ((to_stdlogic(e_state = E_SRC_WR) and (inst_as(IDX) or inst_as(SYMB) or
                                                                inst_as(ABSC))));
    dst_mdb_in_bw_sel <= (to_stdlogic(e_state = E_DST_WR) and inst_so(RETI)) or
                         (to_stdlogic(e_state = E_EXEC) and not (inst_ad(DIR) or inst_type(INST_JMPC) or
                                                                 inst_type(INST_SOC)) and not inst_so(RETI));
    dst_fffe_sel <= to_stdlogic(e_state = E_IRQ(0)) or
                    to_stdlogic(e_state = E_IRQ(1)) or
                    to_stdlogic(e_state = E_IRQ(3)) or
                    (to_stdlogic(e_state = E_DST_RD) and (inst_so(PUSH) or inst_so(CALL)) and not inst_so(RETI)) or
                    (to_stdlogic(e_state = E_SRC_AD) and (inst_so(PUSH) or inst_so(CALL)) and inst_as(IDX)) or
                    (to_stdlogic(e_state = E_SRC_RD) and (inst_so(PUSH) or inst_so(CALL)) and (inst_as(INDIR) or inst_as(INDIR_I)) and inst_src(1));
    dst_reg_dest_sel <= (to_stdlogic(e_state = E_DST_RD) and not (inst_so(PUSH) or inst_so(CALL) or inst_ad(ABSC) or inst_so(RETI))) or
                        (to_stdlogic(e_state = E_DST_WR) and not inst_ad(ABSC)) or
                        (to_stdlogic(e_state = E_EXEC) and (inst_ad(DIR) or inst_type(INST_JMPC) or
                                                            inst_type(INST_SOC)) and not inst_so(RETI));
    op_dst <= dbg_mem_dout
              when dbg_halt_st = '1'       else inst_sext
              when dst_inst_sext_sel = '1' else mdb_in_bw
              when dst_mdb_in_bw_sel = '1' else reg_dest
              when dst_reg_dest_sel = '1'  else "1111111111111110"
              when dst_fffe_sel = '1'      else (others => '0');
  end block C3_DESTINATION_OPERAND_MUXING;

  C4_ALU : block
  begin
    exec_cycle <= to_stdlogic(e_state = E_EXEC);

    alu_0 : omsp_alu
      port map (
        alu_stat    => alu_stat,
        alu_stat_wr => alu_stat_wr,
        alu_out     => alu_out,
        alu_out_add => alu_out_add,

        dbg_halt_st => dbg_halt_st,
        exec_cycle  => exec_cycle,
        inst_bw     => inst_bw,
        status      => status,
        inst_jmp    => inst_jmp,
        inst_so     => inst_so,
        inst_alu    => inst_alu,
        op_dst      => op_dst,
        op_src      => op_src);
  end block C4_ALU;

  C5_MEMORY_INTERFACE : block
  begin
    --Detect memory read/write access
    mb_rd_det <= (to_stdlogic(e_state = E_SRC_RD) and not inst_as(IMM)) or
                 (to_stdlogic(e_state = E_EXEC) and inst_so(RETI)) or
                 (to_stdlogic(e_state = E_DST_RD) and not inst_type(INST_SOC) and not inst_mov);
    mb_wr_det <= (to_stdlogic(e_state = E_IRQ(1)) and not inst_irq_rst) or
                 (to_stdlogic(e_state = E_IRQ(3)) and not inst_irq_rst) or
                 (to_stdlogic(e_state = E_DST_WR) and not inst_so(RETI)) or to_stdlogic(e_state = E_SRC_WR);
    mb_wr_msk <= "00"
                 when inst_alu(EXEC_NO_WR) = '1' else "11"
                 when not inst_bw = '1'          else "10"
                 when alu_out_add(0) = '1'       else "01";
    mb_en <= mb_rd_det or (mb_wr_det and not inst_alu(EXEC_NO_WR));
    mb_wr <= (mb_wr_det & mb_wr_det) and mb_wr_msk;

    --Memory address bus
    mab <= alu_out_add(15 downto 0);

    clock_gating_on : if (CLOCK_GATING = '1') generate
      --Memory data bus output
      mdb_out_nxt_en <= to_stdlogic(e_state = E_DST_RD) or
                        ((to_stdlogic(e_state = E_EXEC) and not inst_so(CALL)) or
                         to_stdlogic(e_state = E_IRQ(0)) or
                         to_stdlogic(e_state = E_IRQ(2)));

      clock_gate_mdb_out_nxt : omsp_clock_gate
        port map (
          gclk        => mclk_mdb_out_nxt,
          clk         => mclk,
          enable      => mdb_out_nxt_en,
          scan_enable => scan_enable);

      mdb_out <= mdb_out_nxt(7 downto 0) & mdb_out_nxt(7 downto 0) when inst_bw = '1' else mdb_out_nxt;

      process (mclk_mdb_out_nxt, puc_rst)
      begin
        if (puc_rst = '1') then
          mdb_out_nxt <= X"0000";
        elsif (rising_edge(mclk_mdb_out_nxt)) then
          if (e_state = E_DST_RD) then
            mdb_out_nxt <= pc_nxt;
          else
            mdb_out_nxt <= alu_out;
          end if;
        end if;
      end process;

      --Format memory data bus input depending on BW
      R1_1_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          mab_lsb <= '0';
        elsif (rising_edge(mclk)) then
          mab_lsb <= alu_out_add(0);
        end if;
      end process R1_1_e;

      mdb_in_bw <= mdb_in
                   when not inst_bw = '1' else mdb_in(15 downto 8) & mdb_in(15 downto 8)
                   when mab_lsb = '1'     else mdb_in;

      --Memory data bus input buffer (buffer after a source read)
      R2_1_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          mdb_in_buf_en <= '0';
        elsif (rising_edge(mclk)) then
          mdb_in_buf_en <= to_stdlogic(e_state = E_SRC_RD);
        end if;
      end process R2_1_e;

      R_1c_2c_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          mdb_in_buf_valid <= '0';
        elsif (rising_edge(mclk)) then
          if (e_state = E_EXEC) then
            mdb_in_buf_valid <= '0';
          elsif (mdb_in_buf_en = '1') then
            mdb_in_buf_valid <= '1';
          end if;
        end if;
      end process R_1c_2c_e;

      clock_gate_mdb_in_buf : omsp_clock_gate
        port map (
          gclk        => mclk_mdb_in_buf,
          clk         => mclk,
          enable      => mdb_in_buf_en,
          scan_enable => scan_enable);

      R_1i_2ci : process (mclk_mdb_in_buf, puc_rst)
      begin
        if (puc_rst = '1') then
          mdb_in_buf <= X"0000";
        elsif (rising_edge(mclk_mdb_in_buf)) then
          mdb_in_buf <= mdb_in_bw;
        end if;
      end process R_1i_2ci;
    end generate clock_gating_on;

    clock_gating_off : if (CLOCK_GATING = '0') generate
      --Memory address bus
      mdb_out <= mdb_out_nxt(7 downto 0) & mdb_out_nxt(7 downto 0) when inst_bw = '1' else mdb_out_nxt;

      process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          mdb_out_nxt <= X"0000";
        elsif (rising_edge(mclk)) then
          if (e_state = E_DST_RD) then
            mdb_out_nxt <= pc_nxt;
          elsif (((to_stdlogic(e_state = E_EXEC) and not inst_so(CALL)) or
                  to_stdlogic(e_state = E_IRQ(0)) or
                  to_stdlogic(e_state = E_IRQ(2))) = '1') then
            mdb_out_nxt <= alu_out;
          end if;
        end if;
      end process;

      --Format memory data bus input depending on BW
      R1_1_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          mab_lsb <= '0';
        elsif (rising_edge(mclk)) then
          mab_lsb <= alu_out_add(0);
        end if;
      end process R1_1_e;

      mdb_in_bw <= mdb_in
                   when not inst_bw = '1' else mdb_in(15 downto 8) & mdb_in(15 downto 8)
                   when mab_lsb = '1'     else mdb_in;

      --Memory data bus input buffer (buffer after a source read)
      R2_1_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          mdb_in_buf_en <= '0';
        elsif (rising_edge(mclk)) then
          mdb_in_buf_en <= to_stdlogic(e_state = E_SRC_RD);
        end if;
      end process R2_1_e;

      R_1c_2c_e : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          mdb_in_buf_valid <= '0';
        elsif (rising_edge(mclk)) then
          if (e_state = E_EXEC) then
            mdb_in_buf_valid <= '0';
          elsif (mdb_in_buf_en = '1') then
            mdb_in_buf_valid <= '1';
          end if;
        end if;
      end process R_1c_2c_e;

      R_1i_2ci : process (mclk, puc_rst)
      begin
        if (puc_rst = '1') then
          mdb_in_buf <= X"0000";
        elsif (rising_edge(mclk)) then
          if (mdb_in_buf_en = '1') then
            mdb_in_buf <= mdb_in_bw;
          end if;
        end if;
      end process R_1i_2ci;
    end generate clock_gating_off;

    mdb_in_val <= mdb_in_buf when mdb_in_buf_valid = '1' else mdb_in_bw;
  end block C5_MEMORY_INTERFACE;
end EXECUTION_ARQ;
