// The MIT License (MIT)
//
// Copyright (c) 2014 Matthew Hagerty
// Copyright 2019 Google LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Simple SDRAM Controller for Winbond W9812G6JH-75
// http://codehackcreate.com/archives/444
//
// Matthew Hagerty, copyright 2014
//
// Create Date:    18:22:00 March 18, 2014
// Module Name:    sdram_simple - RTL
//
// Change Log:
//
// Jul 2019 (Phillip Pearson)
//    Added clk_en to allow running at half rate, for easy timing
//
// Jan 28, 2016
//    Changed to use positive clock edge.
//    Buffered output (read) data, sampled during RAS2.
//    Removed unused signals for features that were not implemented.
//    Changed tabs to space.
//
// March 19, 2014
//    Initial implementation.

module sdram_simple(
  // Host side
  input wire clk_100m0_i,       // Master clock
  input wire clk_en,            // Master clock enable
  input wire reset_i,           // Reset, active high
  input wire refresh_i,         // Initiate a refresh cycle, active high
  input wire rw_i,              // Initiate a read or write operation, active high
  input wire we_i,              // Write enable, active low
  input wire [23:0] addr_i,     // Address from host to SDRAM
  input wire [15:0] data_i,     // Data from host to SDRAM
  input wire ub_i,              // Data upper byte enable, active low
  input wire lb_i,              // Data lower byte enable, active low
  output wire ready_o,          // Set to '1' when the memory is ready
  output reg done_o,            // Read, write, or refresh, operation is done
  output wire [15:0] data_o,    // Data from SDRAM to host
  // SDRAM side
  output wire sdCke_o,          // Clock-enable to SDRAM
  output wire sdCe_bo,          // Chip-select to SDRAM
  output wire sdRas_bo,         // SDRAM row address strobe
  output wire sdCas_bo,         // SDRAM column address strobe
  output wire sdWe_bo,          // SDRAM write enable
  output wire [1:0] sdBs_o,     // SDRAM bank address
  output wire [12:0] sdAddr_o,  // SDRAM row/column address
  inout wire [15:0] sdData_io,  // Data to/from SDRAM
  output wire sdDqmh_o,         // Enable upper-byte of SDRAM databus if true
  output wire sdDqml_o          // Enable lower-byte of SDRAM databus if true
);

// SDRAM controller states.
parameter [3:0]
  ST_INIT_WAIT = 0,
  ST_INIT_PRECHARGE = 1,
  ST_INIT_REFRESH1 = 2,
  ST_INIT_MODE = 3,
  ST_INIT_REFRESH2 = 4,
  ST_IDLE = 5,
  ST_REFRESH = 6,
  ST_ACTIVATE = 7,
  ST_RCD = 8,
  ST_RW = 9,
  ST_RAS1 = 10,
  ST_RAS2 = 11,
  ST_PRECHARGE = 12;

reg [3:0] state_r = ST_INIT_WAIT;
reg [3:0] state_x = ST_INIT_WAIT;

// SDRAM mode register data sent on the address bus.
//
// | A12-A10 |    A9    | A8  A7 | A6 A5 A4 |    A3   | A2 A1 A0 |
// | reserved| wr burst |reserved| CAS Ltncy|addr mode| burst len|
//   0  0  0      0       0   0    0  1  0       0      0  0  0
parameter MODE_REG = {3'b000,1'b0,2'b00,3'b010,1'b0,3'b000};

// SDRAM commands combine SDRAM inputs: cs, ras, cas, we.
parameter CMD_ACTIVATE = 4'b0011;
parameter CMD_PRECHARGE = 4'b0010;
parameter CMD_WRITE = 4'b0100;
parameter CMD_READ = 4'b0101;
parameter CMD_MODE = 4'b0000;
parameter CMD_NOP = 4'b0111;
parameter CMD_REFRESH = 4'b0001;

reg [3:0] cmd_r;
reg [3:0] cmd_x;

wire [1:0] bank_s;
wire [12:0] row_s;
wire [8:0] col_s;
reg [12:0] addr_r;
reg [12:0] addr_x;  // SDRAM row/column address.
reg [15:0] sd_dout_r;
reg [15:0] sd_dout_x;
reg sd_busdir_r;
reg sd_busdir_x;

reg [31:0] timer_r = 0;  // TODO natural range 0 to 20000; probably reg [14:0]
reg [31:0] timer_x = 0;
reg [31:0] refcnt_r = 0;  // TODO natural range 0 to 8; probably reg [4:0]
reg [31:0] refcnt_x = 0;

reg [1:0] bank_r;
reg [1:0] bank_x;
reg cke_r;
reg cke_x;
reg sd_dqmu_r;
reg sd_dqmu_x;
reg sd_dqml_r;
reg sd_dqml_x;
reg ready_r;
reg ready_x;

// Data buffer for SDRAM to Host.
reg [15:0] buf_dout_r; reg [15:0] buf_dout_x;

// All signals to SDRAM buffered.

// SDRAM operation control bits
assign sdCe_bo = cmd_r[3];
assign sdRas_bo = cmd_r[2];
assign sdCas_bo = cmd_r[1];
assign sdWe_bo = cmd_r[0];

assign sdCke_o = cke_r;  // SDRAM clock enable
assign sdBs_o = bank_r;  // SDRAM bank address
assign sdAddr_o = addr_r;  // SDRAM address
assign sdData_io = sd_busdir_r == 1'b1 ? sd_dout_r : {16{1'bZ}};  // SDRAM data bus.
assign sdDqmh_o = sd_dqmu_r;  // SDRAM high data byte enable, active low
assign sdDqml_o = sd_dqml_r;  // SDRAM low date byte enable, active low

// Signals back to host.
assign ready_o = ready_r;
assign data_o = buf_dout_r;

// 23  22  | 21 20 19 18 17 16 15 14 13 12 11 10 09 | 08 07 06 05 04 03 02 01 00 |
// BS0 BS1 |        ROW (A12-A0)  8192 rows         |   COL (A8-A0)  512 cols    |
assign bank_s = addr_i[23:22];
assign row_s = addr_i[21:9];
assign col_s = addr_i[8:0];

always @(state_r, timer_r, refcnt_r, cke_r, addr_r, sd_dout_r, sd_busdir_r, sd_dqmu_r, sd_dqml_r, ready_r, bank_s, row_s, col_s, rw_i, refresh_i, addr_i, data_i, we_i, ub_i, lb_i, buf_dout_r, sdData_io) begin

  state_x <= state_r;  // Stay in the same state unless changed.
  timer_x <= timer_r;  // Hold the cycle timer by default.
  refcnt_x <= refcnt_r;  // Hold the refresh timer by default.
  cke_x <= cke_r;  // Stay in the same clock mode unless changed.
  cmd_x <= CMD_NOP;  // Default to NOP unless changed.
  bank_x <= bank_r;  // Register the SDRAM bank.
  addr_x <= addr_r;  // Register the SDRAM address.
  sd_dout_x <= sd_dout_r;  // Register the SDRAM write data.
  sd_busdir_x <= sd_busdir_r;  // Register the SDRAM bus tristate control.
  sd_dqmu_x <= sd_dqmu_r;
  sd_dqml_x <= sd_dqml_r;
  buf_dout_x <= buf_dout_r;  // SDRAM to host data buffer.

  ready_x <= ready_r;  // Always ready unless performing initialization.
  done_o <= 1'b0;  // Done tick, single cycle.

  if(timer_r != 0) begin
    timer_x <= timer_r - 1;
  end
  else begin
    cke_x <= 1'b1;
    bank_x <= bank_s;
    addr_x <= {4'b0000,col_s};  // A10 low for rd/wr commands to suppress auto-precharge.
    sd_dqmu_x <= 1'b0;
    sd_dqml_x <= 1'b0;
    case(state_r)
    ST_INIT_WAIT : begin
      // 1. Wait 200us with DQM signals high, cmd NOP.
      // 2. Precharge all banks.
      // 3. Eight refresh cycles.
      // 4. Set mode register.
      // 5. Eight refresh cycles.
      state_x <= ST_INIT_PRECHARGE;
      timer_x <= 20000;
      // Wait 200us (20,000 cycles).
      //          timer_x <= 2;              -- for simulation
      sd_dqmu_x <= 1'b1;
      sd_dqml_x <= 1'b1;
    end
    ST_INIT_PRECHARGE : begin
      state_x <= ST_INIT_REFRESH1;
      refcnt_x <= 8;
      // Do 8 refresh cycles in the next state.
      //          refcnt_x <= 2;             -- for simulation
      cmd_x <= CMD_PRECHARGE;
      timer_x <= 2;
      // Wait 2 cycles plus state overhead for 20ns Trp.
      bank_x <= 2'b00;
      addr_x[10] <= 1'b1;
      // Precharge all banks.
    end
    ST_INIT_REFRESH1 : begin
      if(refcnt_r == 0) begin
        state_x <= ST_INIT_MODE;
      end
      else begin
        refcnt_x <= refcnt_r - 1;
        cmd_x <= CMD_REFRESH;
        timer_x <= 7;
        // Wait 7 cycles plus state overhead for 70ns refresh.
      end
    end
    ST_INIT_MODE : begin
      state_x <= ST_INIT_REFRESH2;
      refcnt_x <= 8;
      // Do 8 refresh cycles in the next state.
      //          refcnt_x <= 2;             -- for simulation
      bank_x <= 2'b00;
      addr_x <= MODE_REG;
      cmd_x <= CMD_MODE;
      timer_x <= 2;
      // Trsc == 2 cycles after issuing MODE command.
    end
    ST_INIT_REFRESH2 : begin
      if(refcnt_r == 0) begin
        state_x <= ST_IDLE;
        ready_x <= 1'b1;
      end
      else begin
        refcnt_x <= refcnt_r - 1;
        cmd_x <= CMD_REFRESH;
        timer_x <= 7;
        // Wait 7 cycles plus state overhead for 70ns refresh.
      end
      //
      // Normal Operation
      //
      // Trc  - 70ns - Attive to active command.
      // Trcd - 20ns - Active to read/write command.
      // Tras - 50ns - Active to precharge command.
      // Trp  - 20ns - Precharge to active command.
      // TCas - 2clk - Read/write to data out.
      //
      //         |<-----------       Trc      ------------>|
      //         |<----------- Tras ---------->|
      //         |<- Trcd  ->|<- TCas  ->|     |<-  Trp  ->|
      //  T0__  T1__  T2__  T3__  T4__  T5__  T6__  T7__  T8__  T9__ T10__
      // __|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__
      // IDLE  ACTVT  NOP  RD/WR  NOP   NOP  PRECG IDLE  ACTVT
      //     --<Row>-------<Col>------------<Bank>-------<Row>--
      //    ---------------<A10>-------------<A10>-------------------
      // ------------------<Din>------------<Dout>--------
      //    ---------------<DQM>---------------
      //     --<Refsh>-------------
      //
      // A10 during rd/wr : 0 = disable auto-precharge, 1 = enable auto-precharge.
      // A10 during precharge: 0 = single bank, 1 = all banks.
      //  T0__  T1__  T2__  T3__  T4__  T5__  T6__  T7__  T8__  T9__ T10__
      // __|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__
      // IDLE  ACTVT  NOP  RD/WR  NOP   NOP  PRECG IDLE  ACTVT
      //  T0__  T1__  T2__  T3__  T4__  T5__  T6__  T7__  T8__  T9__ T10__
      // __|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__
      //       IDLE  ACTVT  NOP  RD/WR  NOP   NOP  PRECG IDLE  ACTVT
    end
    ST_IDLE : begin
      // 60ns since activate when coming from PRECHARGE state.
      // 10ns since PRECHARGE.  Trp == 20ns min.
      if(rw_i == 1'b1) begin
        state_x <= ST_ACTIVATE;
        cmd_x <= CMD_ACTIVATE;
        addr_x <= row_s;
        // Set bank select and row on activate command.
      end
      else if(refresh_i == 1'b1) begin
        state_x <= ST_REFRESH;
        cmd_x <= CMD_REFRESH;
        timer_x <= 7;
        // Wait 7 cycles plus state overhead for 70ns refresh.
      end
    end
    ST_REFRESH : begin
      state_x <= ST_IDLE;
      done_o <= 1'b1;
    end
    ST_ACTIVATE : begin
      // Trc (Active to Active Command Period) is 65ns min.
      // 70ns since activate when coming from PRECHARGE -> IDLE states.
      // 20ns since PRECHARGE.
      // ACTIVATE command is presented to the SDRAM.  The command out of this
      // state will be NOP for one cycle.
      state_x <= ST_RCD;
      sd_dout_x <= data_i;
      // Register any write data, even if not used.
    end
    ST_RCD : begin
      // 10ns since activate.
      // Trcd == 20ns min.  The clock is 10ns, so the requirement is satisfied by this state.
      // READ or WRITE command will be active in the next cycle.
      state_x <= ST_RW;
      if(we_i == 1'b0) begin
        cmd_x <= CMD_WRITE;
        sd_busdir_x <= 1'b1;
        // The SDRAM latches the input data with the command.
        sd_dqmu_x <= ub_i;
        sd_dqml_x <= lb_i;
      end
      else begin
        cmd_x <= CMD_READ;
      end
    end
    ST_RW : begin
      // 20ns since activate.
      // READ or WRITE command presented to SDRAM.
      state_x <= ST_RAS1;
      sd_busdir_x <= 1'b0;
    end
    ST_RAS1 : begin
      // 30ns since activate.
      state_x <= ST_RAS2;
    end
    ST_RAS2 : begin
      // 40ns since activate.
      // Tras (Active to precharge Command Period) 45ns min.
      // PRECHARGE command will be active in the next cycle.
      state_x <= ST_PRECHARGE;
      cmd_x <= CMD_PRECHARGE;
      addr_x[10] <= 1'b1;
      // Precharge all banks.
      buf_dout_x <= sdData_io;
    end
    ST_PRECHARGE : begin
      // 50ns since activate.
      // PRECHARGE presented to SDRAM.
      state_x <= ST_IDLE;
      done_o <= 1'b1;
      // Read data is ready and should be latched by the host.
      timer_x <= 1;
      // Buffer to make sure host takes down memory request before going IDLE.
    end
    endcase
  end
end

always @(posedge clk_100m0_i) begin
  if(clk_en == 1'b1) begin
    if(reset_i == 1'b1) begin
      state_r <= ST_INIT_WAIT;
      timer_r <= 0;
      cmd_r <= CMD_NOP;
      cke_r <= 1'b0;
      ready_r <= 1'b0;
    end
    else begin
      state_r <= state_x;
      timer_r <= timer_x;
      refcnt_r <= refcnt_x;
      cke_r <= cke_x;
      // CKE to SDRAM.
      cmd_r <= cmd_x;
      // Command to SDRAM.
      bank_r <= bank_x;
      // Bank to SDRAM.
      addr_r <= addr_x;
      // Address to SDRAM.
      sd_dout_r <= sd_dout_x;
      // Data to SDRAM.
      sd_busdir_r <= sd_busdir_x;
      // SDRAM bus direction.
      sd_dqmu_r <= sd_dqmu_x;
      // Upper byte enable to SDRAM.
      sd_dqml_r <= sd_dqml_x;
      // Lower byte enable to SDRAM.
      ready_r <= ready_x;
      buf_dout_r <= buf_dout_x;
    end
  end
end

endmodule
