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


// QPI flash interface

// Targetted at Winbond W25Q128JV (16MB QPI)

// We use the Fast Read Quad I/O (EBh) instruction in QPI mode, and enable
// Continuous Read Mode.

// First request:
// EB AA AA AA MM -> DO
// Subsequent requests:
// AA AA AA MM -> DO

// Format of the mode bits (MM):
// M7-6: xx
// M5-4: 10 to enable Continuous Read Mode
// M3-0: xxxx
// So 00100000 (0x20) will enter Continuous Read Mode

// To exit Continuous Read Mode, it's recommended to go into SPI mode (IO1-3
// don't care) and clock in 0xFFFF (sixteen clocks).  Won't this cause a clash
// on the first query though?

// Probably want to do two transactions with spi_mode==1, clocking in 0xFF
// (eight clocks) each time.  If we're in QPI continuous read mode, this will
// register as an address of FF FF FF and a mode of FF.  If we're in QPI mode,
// it will register as an FF command, which exits QPI mode.

// READ COMMAND OPTIONS

// Fast Read (0Bh) in QPI Mode with 2 dummy clocks:
// \ 0B AA AA AA 00 DD / = 13 clocks

// Fast Read Quad I/O (EBh) in QPI Mode with 2 dummy clocks (used for the Mode byte)
// \ EB AA AA AA 20 DD / = 13 clocks for first request
// \ AA AA AA 20 DD / = 11 clocks for subsequent requests

// The datasheet suggests that in QPI mode, addresses may need to be aligned
// on a 4-byte boundary, but this doesn't seem to be the case.

module qpi_flash(

    // Fast clock for flash
    input wire clk,

    // Control interface; all signals synchronous w.r.t. clk

    // Active high to indicate that the module can accept a read command
    output reg ready = 0,

    // Active high reset; when this transitions low, the module will
    // initialize the flash chip.
    input wire reset,

    // Active high read instruction; pulse this for one clock with valid addr
    // to start a read.  When finished, ready will go high and data_out will
    // be valid.
    input wire read,

    // Bus
    input wire [23:0] addr,
    // input wire [7:0] data_in,  // one day...
    output reg [7:0] data_out = 8'hFF,

    // Passthrough
    input wire passthrough,
    input wire passthrough_nCE,
    input wire passthrough_SCK,
    input wire passthrough_MOSI,

    // Flash pins
    output reg flash_nCE = 1,
    output wire flash_SCK,
    inout wire flash_IO0,
    inout wire flash_IO1,
    inout wire flash_IO2,
    inout wire flash_IO3

);

// Set to 1 to run at full clock rate, using a DDR clock buffer for flash_SCK
parameter FAST_MODE = 1;

// Set to 4 to run at 96 MHz
`define EXTRA_DUMMY_CLOCKS (FAST_MODE ? 4 : 0)

// Setup/hold notes:
// - Don't change /CS within 3ns of a rising clock edge.  (min 3ns clk to flash_nCE)
// - IO* setup 1ns hold 2ns w.r.t. SCK.  (min 2ns clk to flash_IO*)
// - Data output by the flash is referenced to the falling edge of SCK: 6ns clock low to output, 1.5ns hold.

// Reset: state register
reg [3:0] reset_state = 0;

`define RESET_START 0
`define RESET_DISABLE_CONT_READ 1
`define RESET_DISABLE_QPI 2
`define RESET_RESET_CHIP 3
`define RESET_WAIT_CHIP 4
`define RESET_ENTER_QPI 5
`define RESET_SET_DUMMY_CLOCKS 6
`define RESET_ENTER_CONT_READ 7
`define RESET_TEST_CONT_READ 8
`define RESET_SET_READY 9
`define RESET_DONE 10


// Reset: 30us counter
reg [12:0] reset_delay_counter = 13'b0;

// Set to 0 for 48 MHz, 4 for 96 MHz operation.
`define EXTRA_DUMMY_BITS (`EXTRA_DUMMY_CLOCKS * 4)
`define DUMMY_DATA {`EXTRA_DUMMY_BITS{1'b0}}
`define SHIFTER_SIZE (40 + `EXTRA_DUMMY_BITS)
// Shifter for IO[3:0]
reg [`SHIFTER_SIZE-1:0] shifter = 0;
reg [6:0] shift_count = 0;

// Tracking previous passthrough value so we can reset after a passthrough ends
reg last_passthrough = 0;
// High when passthrough mode is actually active and we're out of QPI mode
reg passthrough_active = 0;

// IO state
reg spi_mode = 1'b0;    // IO0 output, IO1 input, IO2/3 tristate
reg qpi_mode = 1'b0;
reg qpi_output = 1'b0;  // if qpi_mode == 1, this means we're in the output phase
reg [5:0] qpi_output_count = 0;  // countdown to turnaround

`define TXN_IDLE 0
`define TXN_START 1
`define TXN_RUNNING 2
`define TXN_DONE 4

reg [2:0] txn_state = `TXN_IDLE;
reg reading = 0;

// Output register
reg [3:0] output_IO = 4'b0;

// Drive flash_IO* correctly depending on mode
assign flash_IO0 = (spi_mode == 1'b1 || qpi_output == 1'b1) ? output_IO[0] : 1'bZ;
assign flash_IO1 = (qpi_output == 1'b1) ? output_IO[1] : 1'bZ;
assign flash_IO2 = (qpi_output == 1'b1) ? output_IO[2] : 1'bZ;
assign flash_IO3 = (qpi_output == 1'b1) ? output_IO[3] : 1'bZ;

// Clock generation.
// In passthrough mode, these follow passthrough_SCK.
// In 48MHz mode, these alternate between 0 and 1.
// In 96MHz mode, these are constant at 0 and 1.
reg sck_int_h = 0;
reg sck_int_l = 0;

// When not in fast mode, we should only output data when sck_int_h == 0, i.e.
// the latched datain_h is 1 and going low.  (sck_ddr_buf latches its inputs,
// so they're a clock behind sck_int_h and sck_int_l).
`define CLOCK_ENABLED (FAST_MODE || sck_int_h == 1'b0)

altddio_out sck_ddr_buf (
    .datain_h (sck_int_h),
    .datain_l (sck_int_l),
    .outclock (clk),
    .dataout (flash_SCK));
defparam
    sck_ddr_buf.width = 1;

always @(posedge clk) begin

    // Generate clock
    if (FAST_MODE) begin
        // 96 MHz: DDR output buffers and invertes the 96 MHz clock
        sck_int_h <= 0;
        sck_int_l <= 1;
    end else begin
        // 48 MHz: half-speed clock generated here
        sck_int_h <= !sck_int_l;
        sck_int_l <= !sck_int_l;
    end

    // Lowest priority: read by MCU
    if (passthrough == 0 && read == 1) begin
        $display("Read triggered with addr %x", addr);
        qpi_output_count <= 6'd24 + 6'd8 + `EXTRA_DUMMY_BITS;  // output address and mode byte

        // 4-byte alignment
        // shifter <= {addr[23:2], 2'b00, 8'h20, 8'b0};  // addr & ~3
        // shift_count <= 7'd24 + 7'd8 + 7'd8 + (7'd8 * addr[1:0]);  // read 1-4 data bytes

        // 1-byte alignment (seems to work, despite what the datasheet says...)
        // Address:24, Mode:8, [Dummy:16 if in 96MHz mode], Response:8
        shifter <= {addr, 8'h20, `DUMMY_DATA, 8'b0};
        shift_count <= 7'd24 + 7'd8 + `EXTRA_DUMMY_BITS + 7'd8;  // read one data byte

        txn_state <= `TXN_START;
        reading <= 1;
        ready <= 0;

        // When running at 2MHz with an internal 6502, we have to be stable
        // while cpu_clken==1, but we have 7 x 62.5ns 16MHz clocks = 42 x
        // 96MHz clocks = 437.5 ns for memory access.

        // With running at 2MHz with an external 6502, we have 280 ns after
        // allowing for 170 ns address setup (30ns PHI0-PHI2 delay, 140ns
        // PHI2-A setup) and 50ns data hold.

        // In 48MHz mode (FAST_MODE == 0), a read takes (24 + 8 + 8) / 4 * 2
        // clocks (two clocks to transmit 4 bits in the data phase) = 20 x
        // 10.42 ns = 208.4 ns, which works at 2MHz with an external CPU.

        // In 96MHz mode (FAST_MODE == 1), a read takes (24 + 24 + 8) / 4
        // clocks (one clock per 4 bits, but with an extra 4 clock delay on
        // the read command) = 14 * 10.42 = 145.9 ns.

    end
    if (reading && txn_state == `TXN_DONE) begin
        reading <= 0;
        ready <= 1;
        data_out <= shifter[7:0];
    end

    // Medium priority: SPI passthrough for MCU-driven SPI
    if (passthrough == 1 && passthrough_active == 1) begin
        spi_mode <= 1'b1;
        qpi_mode <= 0;
        qpi_output <= 1'b0;
        flash_nCE <= passthrough_nCE;
        sck_int_h <= passthrough_SCK;
        sck_int_l <= passthrough_SCK;
        output_IO[0] <= passthrough_MOSI;
    end
    last_passthrough <= passthrough;
    if (last_passthrough == 0 && passthrough == 1) begin
        // Entering passthrough mode: exit continuous read and QPI
        reset_state <= `RESET_START;
    end
    if (last_passthrough == 1'b1 && passthrough == 1'b0) begin
        // Reset line after passthrough is disabled
        reset_state <= `RESET_START;
    end

    // Execute SPI requests
    if (spi_mode == 1'b1 && `CLOCK_ENABLED) begin
        case (txn_state)
            `TXN_START : begin
                flash_nCE <= 1'b0;
                output_IO[0] = shifter[`SHIFTER_SIZE-1];
                shifter <= {shifter[`SHIFTER_SIZE-2:0], 1'b0};
                txn_state <= `TXN_RUNNING;
            end
            `TXN_RUNNING : begin
                // Falling SCK edge; clock data in and out
                output_IO[0] = shifter[`SHIFTER_SIZE-1];
                shifter <= {shifter[`SHIFTER_SIZE-2:0], flash_IO1};
                shift_count <= shift_count - 7'd1;
                if (shift_count == 7'd1) begin
                    // Finished with transaction; we can raise /CE on the falling SCK edge
                    flash_nCE <= 1'b1;
                    txn_state <= `TXN_DONE;
                end
            end
        endcase
    end

    // Execute QPI requests
    if (qpi_mode == 1'b1 && (FAST_MODE || sck_int_h == 1'b0)) begin
        case (txn_state)
            `TXN_START : begin
                flash_nCE <= 1'b0;
                output_IO <= shifter[`SHIFTER_SIZE-1:`SHIFTER_SIZE-4];
                shifter <= {shifter[`SHIFTER_SIZE-5:0], 4'b0};
                qpi_output <= 1;
                txn_state <= `TXN_RUNNING;
            end
            `TXN_RUNNING : begin
                // Falling SCK edge; clock data in and out
                output_IO <= shifter[`SHIFTER_SIZE-1:`SHIFTER_SIZE-4];
                shifter <= {shifter[`SHIFTER_SIZE-5:0], flash_IO3, flash_IO2, flash_IO1, flash_IO0};
                shift_count <= shift_count - 7'd4;
                if (qpi_output_count == 4) begin
                    qpi_output <= 0;
                end else begin
                    qpi_output_count <= qpi_output_count - 6'd4;
                end
                if (qpi_output == 0 && shift_count == 7'd4) begin
                    data_out <= {shifter[3:0], flash_IO3, flash_IO2, flash_IO1, flash_IO0};
                end
                if (shift_count == 7'd4) begin
                    // Finished with transaction; we can raise /CE on the falling SCK edge
                    flash_nCE <= 1'b1;
                    qpi_output <= 1'b0;
                    txn_state <= `TXN_DONE;
                end
            end
        endcase
    end

    // Highest priority: reset
    case (reset_state)
        `RESET_START : begin
            // During flash programming, the QE (Quad Enable) bit is set, and
            // it's non-volatile, so we don't need to deal with that here.

            ready <= 1'b0;

            flash_nCE <= 1'b1;
            output_IO <= 4'b0;
            spi_mode <= 1'b1;
            qpi_mode <= 1'b0;
            qpi_output <= 1'b0;
            reading <= 0;
            passthrough_active <= 0;
            txn_state <= `TXN_IDLE;

            reset_state <= `RESET_DISABLE_CONT_READ;
        end

        `RESET_DISABLE_CONT_READ : begin
            // Start with a single-byte (FF) SPI transaction to disable
            // continuous read mode, if enabled.

            case (txn_state)
                `TXN_IDLE : begin
                    $display("qpi_flash: Disabling continuous read");
                    shifter <= {40'hFF00000000, `DUMMY_DATA};
                    shift_count <= 8;
                    txn_state <= `TXN_START;
                end
                `TXN_DONE : begin
                    txn_state <= `TXN_IDLE;
                    reset_state <= `RESET_DISABLE_QPI;
                end
            endcase
        end

        `RESET_DISABLE_QPI : begin

            // Now a second single-byte (FF) SPI transaction, to disable
            // QPI mode, if enabled.

            case (txn_state)
                `TXN_IDLE : begin
                    $display("qpi_flash: Disabling QPI mode");
                    shifter <= {40'hFF00000000, `DUMMY_DATA};
                    // Temporarily switch to QPI mode
                    spi_mode <= 0;
                    qpi_mode <= 1;
                    qpi_output_count <= 8;
                    shift_count <= 8;
                    txn_state <= `TXN_START;
                end
                `TXN_DONE : begin
                    // Back to SPI mode for the command to re-enter QPI mode (or to begin passthrough)
                    spi_mode <= 1;
                    qpi_mode <= 0;
                    txn_state <= `TXN_IDLE;
                    if (passthrough == 1) begin
                        passthrough_active <= 1;
                        reset_state <= `RESET_SET_READY;
                    end else begin
                        reset_state <= `RESET_ENTER_QPI;
                        // reset_state <= `RESET_SET_READY; // DEBUG
                    end
                end
            endcase

        end

       //`RESET_RESET_CHIP: begin
       //    // Now a two-byte (66 99) SPI transaction to reset the chip.
       //    case (txn_state)
       //        `TXN_IDLE : begin
       //            shifter <= {40'h6699000000, `DUMMY_DATA};
       //            shift_count <= 8;
       //        end
       //        `TXN_DONE : begin
       //            txn_state <= `TXN_IDLE;
       //            reset_state <= `RESET_WAIT_CHIP;
       //        end
       //    endcase
       //end

       //`RESET_WAIT_CHIP : begin
       //    // Now delay 30us (2880 96MHz clocks using reset_delay_counter) to
       //    // let the chip reset finish.
       //    //TODO
       //    reset_state <= `RESET_ENTER_QPI;
       //end

        `RESET_ENTER_QPI : begin

            // Now a one-byte (38) SPI transaction to enter QPI mode.

            case (txn_state)
                `TXN_IDLE : begin
                    $display("qpi_flash: Entering QPI mode");
                    shifter <= {40'h3800000000, `DUMMY_DATA};
                    shift_count <= 8;
                    txn_state <= `TXN_START;
                end
                `TXN_DONE : begin
                    txn_state <= `TXN_IDLE;
                    reset_state <= `RESET_SET_DUMMY_CLOCKS;
                    spi_mode <= 0;
                    qpi_mode <= 1;
                    // reset_state <= `RESET_SET_READY;  // DEBUG skip QPI transactions
                end
            endcase

        end

        `RESET_SET_DUMMY_CLOCKS : begin

            // Now a two-byte (C0 20) QPI transaction to set 6 dummy clocks
            // (which is what we need below 100 MHz), or C0 00 to set 2 dummy
            // clocks if clk < 50 MHz.

            case (txn_state)
                `TXN_IDLE : begin
                    $display("qpi_flash: Setting read params with %0d extra dummy clocks", `EXTRA_DUMMY_CLOCKS);
                    shifter <= {8'hC0, (`EXTRA_DUMMY_CLOCKS == 4 ? 8'h20 : 8'h00), 24'h0, `DUMMY_DATA};
                    shift_count <= 16;
                    qpi_output_count <= 20;  // Remain in output state after txn
                    txn_state <= `TXN_START;
                end
                `TXN_DONE : begin
                    txn_state <= `TXN_IDLE;
                    reset_state <= `RESET_ENTER_CONT_READ;
                end
            endcase

        end

        `RESET_ENTER_CONT_READ : begin

            // Now a five-byte (EB 00 00 00 20) QPI transaction to enter
            // continuous read mode.

            case (txn_state)
                `TXN_IDLE : begin
                    $display("qpi_flash: Entering continuous read mode");
                    // shifter <= {40'hEB00000020, `DUMMY_DATA};
                    // shifter <= {40'hEB00000120, `DUMMY_DATA};  // test unaligned read; expect ab
                    // shifter <= {40'hEB00000220, `DUMMY_DATA};  // test unaligned read; expect 8e
                    shifter <= {40'hEB00000320, `DUMMY_DATA};  // test unaligned read; expect 82 (or 4c with single byte read)
                    // shift_count <= 72;  // read 4 bytes at the end
                    shift_count <= 48 + `EXTRA_DUMMY_BITS;  // read one byte at the end
                    qpi_output_count <= 40 + `EXTRA_DUMMY_BITS;
                    txn_state <= `TXN_START;
                end
                `TXN_DONE : begin
                    txn_state <= `TXN_IDLE;
                    reset_state <= `RESET_TEST_CONT_READ;
                end
            endcase

        end

        `RESET_TEST_CONT_READ : begin

            // Try aa four-byte (00 00 00 20) QPI read

            case (txn_state)
                `TXN_IDLE : begin
                    $display("qpi_flash: Testing continuous read mode");
                    shifter <= {40'h0000072000, `DUMMY_DATA};  // test unaligned read; expect 1b
                    shift_count <= 40 + `EXTRA_DUMMY_BITS;  // read one byte at the end
                    qpi_output_count <= 32 + `EXTRA_DUMMY_BITS;
                    txn_state <= `TXN_START;
                end
                `TXN_DONE : begin
                    txn_state <= `TXN_IDLE;
                    reset_state <= `RESET_SET_READY;
                end
            endcase

        end

        `RESET_SET_READY : begin

            // Now set ready = 1'b1 and go to normal operation

            $display("qpi_flash: Reset done");
            ready <= 1;
            reset_state <= `RESET_DONE;

        end

        `RESET_DONE : begin

            // Normal operation!

        end

        default : begin

            // ERROR

        end

    endcase

    if (reset == 1'b1) begin
        reset_state <= `RESET_START;
    end
end

endmodule