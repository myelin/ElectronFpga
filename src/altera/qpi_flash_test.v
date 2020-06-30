// Copyright 2019 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


`timescale 1ns/100ps

`include "qpi_flash.v"

`define assert(condition, message) if(!(condition)) begin $display("ASSERTION FAILED: %s", message); $finish(1); end

// 1 to test for 96 MHz operation, 0 for 48 MHz
`define FAST_MODE 1

// Model of Max 10 DDR output
module altddio_out(
  input datain_h,
  input datain_l,
  input outclock,
  output wire dataout
);

  parameter width = 1;

  reg data_h, data_l;

  always @(posedge outclock) begin
    data_h <= datain_h;
    data_l <= datain_l;
  end

  assign dataout = outclock ? data_h : data_l;

endmodule

// Test module
module qpi_flash_test;

  // test clock
  reg clk;

  // inputs to the module under test
  wire ready;
  reg reset = 0;
  reg read = 0;
  reg [23:0] addr = 0;
  reg passthrough = 0;
  reg passthrough_nCE;
  reg passthrough_SCK;
  reg passthrough_MOSI;

  // outputs from the module under test
  wire [7:0] data_out;

  // connections to the outside world
  wire flash_SCK;
  wire flash_nCE;
  wire flash_IO0;
  wire flash_IO1;
  wire flash_IO2;
  wire flash_IO3;

  reg driving_IO = 0;
  reg [3:0] output_IO = 4'bz;

  assign flash_IO3 = output_IO[3];
  assign flash_IO2 = output_IO[2];
  assign flash_IO1 = output_IO[1];
  assign flash_IO0 = output_IO[0];

  // test spi feeder
  reg spi_ss = 1'b1;
  reg spi_sck = 1'b0;
  reg spi_mosi = 1'b0;
  wire spi_miso;

  `define SHIFT_HIGH 63
  reg [`SHIFT_HIGH:0] spi_shift;
  reg [8:0] shift_count = 0;

  reg waiting_for_reset = 0;
  reg reset_wait_finished = 0;
  reg [31:0] reset_wait_count = 0;

  // module under test
  qpi_flash dut(
    .clk(clk),
    .ready(ready),
    .reset(reset),
    .read(read),

    .addr(addr),
    .data_out(data_out),
    .passthrough(passthrough),
    .passthrough_nCE(passthrough_nCE),
    .passthrough_SCK(passthrough_SCK),
    .passthrough_MOSI(passthrough_MOSI),

    .flash_nCE(flash_nCE),
    .flash_SCK(flash_SCK),
    .flash_IO0(flash_IO0),
    .flash_IO1(flash_IO1),
    .flash_IO2(flash_IO2),
    .flash_IO3(flash_IO3)
  );
  defparam dut.FAST_MODE = `FAST_MODE;

  task check_flash_deselected;
    begin
      #1 `assert(flash_nCE == 1'b1, "FAIL: flash still selected at end of transaction");
    end
  endtask

  // Expect the controller to output a given byte on flash_IO on the next two clocks, or fail
  task expectout;
    input [7:0] expected_byte;

    reg [7:0] in_byte;

    begin
      `assert(flash_nCE == 1'b0, "FAIL: flash deselected during transaction");
      @(posedge flash_SCK);
      in_byte[7:4] = {flash_IO3, flash_IO2, flash_IO1, flash_IO0};
      @(posedge flash_SCK);
      in_byte[3:0] = {flash_IO3, flash_IO2, flash_IO1, flash_IO0};
      @(negedge flash_SCK);
      $display("Controller output %02x (expected %02x)", in_byte, expected_byte);
      if (in_byte !== expected_byte) begin
        $display("FAIL");
        $finish;
      end
    end
  endtask

  task expectdummyclocks;
    begin
      if (`FAST_MODE) begin
        $display("expect dummy clocks now");
        expectout(8'h0);
        expectout(8'h0);
      end else begin
        $display("in slow mode; dummy clocks all done now");
      end
    end
  endtask

  // Expect the controller to output a given byte on flash_IO on the next two clocks, or fail
  task expectoutspi;
    input [7:0] expected_byte;

    reg [7:0] in_byte;

    begin
      `assert(flash_nCE == 1'b0, "FAIL: flash deselected during transaction");
      repeat(8) begin
        @(posedge flash_SCK);
        $display("bit %d", flash_IO0);
        in_byte = {in_byte[6:0], flash_IO0};
      end
      @(negedge flash_SCK);
      $display("Controller SPI output %02x (expected %02x)", in_byte, expected_byte);
      if (in_byte != expected_byte) begin
        $display("FAIL");
        $finish;
      end
    end
  endtask

  // Simulate byte sent in from flash to controller
  task inbyte;
    input [7:0] byte;

    begin
      `assert(flash_nCE == 1'b0, "FAIL: flash deselected during transaction");
      output_IO = byte[7:4];
      @(negedge flash_SCK);
      output_IO = byte[3:0];
      @(negedge flash_SCK);
      output_IO = 4'bz;
    end
  endtask

  // clock driver
  initial begin
    clk = 1'b0;
    forever #(1000.0/96/2) clk = ~clk;
  end

  // CE pulse measurer
  integer ce_dropped_at = 0;
  always @(negedge flash_nCE) begin
    ce_dropped_at = $time;
  end
  always @(posedge flash_nCE) begin
    $display("/CE pulse width: %0d ns", $time - ce_dropped_at);
  end

  always @(posedge dut.ready) begin
    $display("rising edge on ready");
  end

  always @(negedge dut.reset) begin
    $display("falling edge on reset");
  end

  always @(negedge dut.read) begin
    $display("falling edge on read");
  end

  always @(negedge flash_nCE) begin
    $display("falling edge on flash_nCE");
    shift_count = 0;
  end

  always @(posedge flash_SCK) begin
    if (flash_nCE == 0) begin
      if (dut.qpi_mode == 1) begin
        spi_shift = {spi_shift[`SHIFT_HIGH-4:0], flash_IO3, flash_IO2, flash_IO1, flash_IO0};
        $display("rising QPI edge with output nybble %x", {flash_IO3, flash_IO2, flash_IO1, flash_IO0});
        shift_count = shift_count + 4;
      end else begin
        spi_shift = {spi_shift[`SHIFT_HIGH-1:0], flash_IO0};
        // $display("rising SPI edge with MOSI %x", flash_IO0);
        shift_count = shift_count + 1;
      end
      if (shift_count == 8) begin
        $display("-> output byte %x", spi_shift[7:0]);
        shift_count = 0;
      end
    end
  end

  always @(posedge clk) begin
    if (ready == 1 || reset_wait_count > 10000) begin
      reset_wait_finished = 1;
    end else if (reset_wait_finished == 0) begin
      reset_wait_count = reset_wait_count + 1;
    end
  end

  initial begin

    $display("running qpi_flash_test");

    $dumpfile("qpi_flash_test.lxt2");
    $dumpvars(0, qpi_flash_test);

    $display("start");
    reset = 1;
    repeat(10) @(posedge clk);
    reset = 0;

    // Follow the reset process
    @(negedge flash_nCE);
    $display("* SPI FF to disable continuous read");
    expectoutspi(8'hFF);
    check_flash_deselected();

    $display("* QPI FF to disable QPI");
    @(negedge flash_nCE);
    expectout(8'hFF);
    check_flash_deselected();

    $display("* SPI 38 to re-enter QPI");
    @(negedge flash_nCE);
    expectoutspi(8'h38);
    check_flash_deselected();

    if (`FAST_MODE) begin
      $display("* QPI C0 20 to set 6 dummy clocks (for 96 MHz operation)");
    end else begin
      $display("* QPI C0 00 to set 2 dummy clocks (for 48 MHz operation)");
    end;
    @(negedge flash_nCE);
    expectout(8'hC0);
    expectout(`FAST_MODE ? 8'h20 : 8'h00);
    check_flash_deselected();

    $display("* QPI: set up continuous read");
    @(negedge flash_nCE);
    expectout(8'hEB);
    expectout(8'h00);
    expectout(8'h00);
    expectout(8'h03);
    expectout(8'h20);
    expectdummyclocks();
    inbyte(8'hff);
    check_flash_deselected();

    $display("* QPI: test continuous read");
    @(negedge flash_nCE);
    expectout(8'h00);
    expectout(8'h00);
    expectout(8'h07);
    expectout(8'h20);
    expectdummyclocks();
    inbyte(8'hff);
    check_flash_deselected();

    // Expect `ready` to go high shortly after the reset transactions
    repeat(4) @(posedge clk);
    `assert(ready == 1'b1, "FAIL: device not ready");

    $display("\nReset successful; trying a read (0 alignment)");
    addr = 24'h123454;
    read = 1;
    @(posedge clk);
    #1 read = 0;
    // wait for 4 qpi bytes
    @(negedge flash_nCE);
    expectout(8'h12);
    expectout(8'h34);
    expectout(8'h54);
    expectout(8'h20);
    expectdummyclocks();
    // now push some data
    inbyte(8'hAB);
    // wait for end of txn
    if (flash_nCE == 1'b0) @(posedge flash_nCE);
    $display("Read transaction finished, by the looks of things; shifter == %x", dut.shifter);
    // `assert(dut.shifter[31:0] == 32'habcdef12, "shift value incorrect");
    $display("data_out == %x", data_out);
    `assert(data_out == 8'hab, "data_out incorrect");

    // finish off
    $display("running out the clock");
    repeat(32) @(posedge clk);

    $display("PASS");

    $finish;

  end

endmodule
