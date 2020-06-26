### Timing constraints for the ElectronULA_max10 project

# https://fpgawiki.intel.com/wiki/Timing_Constraints is helpful to understand
# the directives in this file.


### CREATE CLOCKS ###

# External clock inputs
create_clock -period "16 MHz" -name clk_in [get_ports clk_in]
create_clock -period "16 MHz" -name clk_osc [get_ports clk_osc]

# Generated clocks (via PLL from 16MHz clk_in or clk_osc)
derive_pll_clocks

# Nicer names for the autogenerated clocks
set clock_16 "max10_pll1_inst|altpll_component|auto_generated|pll1|clk[0]"
set clock_96_sdram "max10_pll1_inst|altpll_component|auto_generated|pll1|clk[1]"
set clock_33 "max10_pll1_inst|altpll_component|auto_generated|pll1|clk[4]"
set clock_40 "max10_pll1_inst|altpll_component|auto_generated|pll1|clk[3]"
set clock_96 "max10_pll1_inst|altpll_component|auto_generated|pll1|clk[2]"

# Generated with logic
create_generated_clock -name clock_32 -source $clock_96 -divide_by 3 [get_ports clock_32]
set clock_32 "clock_32"

# PLL output to SDRAM
# This is advanced w.r.t. clock_96 to give the SDRAM more time to return data to us.
create_generated_clock -name ram_output_clock -source [get_pins $clock_96_sdram] [get_ports sdram_CLK]

# Include this if building with IncludeICEDebugger
# create_clock -period "16 MHz" -name clock_avr {electron_core:bbc_micro|clock_avr}

# Allow for clock jitter etc
derive_clock_uncertainty


### I/O TIMING ###

# set_input_delay is easy: after a clock, the previous data value on the pin
# is stable for -min, and the next data value is stable after -max.  So
# -min is the hold time of the remote device, and -max is the clock-to-output
# time.  (Plus board delays.)

# set_output_delay specifies data required time at the specified port relative
# to the clock.  I always have a hard time understanding exactly what's going
# on.

# http://billauer.co.il/blog/2017/04/io-timing-constraints-meaning/ says that
# set_output_delay specifies the range where the clock can change after the
# data changes.  This makes more sense: -min 4 -max -3 means the data has to
# be stable between clock-4ns and clock-(-3ns), which will work for a chip with
# 4ns setup and 3ns hold time.

# With a 10.42ns clock, this constrains so 3ns < clock-to-output < 6.42ns,
# and the data will be stable for at least 7ns.

# Everything seems to assume that remote devices are clocked with the FPGA
# clock, which isn't true in our case: We also generate flash_SCK and
# sdram_CLK, and have to constrain them as well.


# *** QPI flash at 96MHz ***

# We're running the flash clock at half the system clock, so we're quite
# tolerant of delays.  If we can constrain the forwarded clock and all the
# data signals to at least be close enough to each other, we'll be OK.
# I wonder if we can just constrain the forwarded clock to transition in a
# particular window -- for example -min 3 -max -5, for 8ns stable time and
# a clock-to-output window of 3ns-5.52ns.

# W25Q128JV timing parameters:

# Output from FPGA: Flash samples inputs on rising flash_SCK, and needs 3ns
# setup 3ns hold for flash_nCE, 1ns setup 2ns hold for flash_IO*.

# New data is available 6ns (tCLQV) after falling flash_SCK, and old data is
# stable for 1.5ns (tCLQX) after falling flash_SCK.  i.e. for a 96MHz
# (10.42ns) clock, the FPGA has 4.42ns + 1.5ns = 5.92ns to sample data.

# For now we just run the flash at 48 MHz; eventually we'll generate flash_SCK
# using a DDR output and run at 96 MHz, but for now things are a lot easier.

# For now we just want to make sure there's a relatively consistent
# clock-to-output delay across flash_*.  If we say the remote chip needs 3ns
# setup and 5ns hold, we can have a 5.42 ns clock to output time, which
# Quartus should be able to manage.

# TODO these were -max 3 / -min -5, but that failed timing (although worked in practice)

# flash_SCK (10.42ns) is toggled from $clock_96
set_output_delay -clock $clock_96 -max 0 [get_ports flash_SCK]
set_output_delay -clock $clock_96 -min 0 [get_ports flash_SCK]

# flash_nCE and flash_IO* are updated in sync with the falling edge of flash_SCK
set_output_delay -clock $clock_96 -max 0 [get_ports flash_nCE]
set_output_delay -clock $clock_96 -min 0 [get_ports flash_nCE]
set_output_delay -clock $clock_96 -max 0 [get_ports flash_IO*]
set_output_delay -clock $clock_96 -min 0 [get_ports flash_IO*]

# flash_SCK will go low max 5.42ns after clock_96, and the flash will update
# IO* max 6ns after that, so the clock-to-output time for us relative to the
# next clock cycle is 5.42+6-10.42=1ns + board delays.  The flash will hold
# IO* for 1.25ns after its next clock.

#--- t=0: clock_96 edge, set SCK low ---
# t=3-5.42ns: flash_SCK low
#--- t=10.42ns: clock_96 edge, set SCK high ---
# t=9-11.42ns: flash_IO* driven by flash
# t=13.42-15.84ns: flash_SCK high
#--- t=20.83ns: clock_96 edge, set SCK low ---
# t=23.83-26.25ns: flash_SCK low
# t=25.08-27.5ns: flash_IO* hold time expires
#--- t=31.25ns: clock_96 edge, set SCK high ---

# So the FPGA can latch input data any time between 11.42-25.08ns, i.e.
# from its perspective the flash has a hold time of 4.25ns and a clock
# to output time of 1.02ns.  This would normally not make sense, but we're
# splitting the transaction over two clock cycles.  We're just going to
# be super conservative here and say no hold and 8ns clock-to-output,
# so if the signal is super delayed we still pick it up.

# Setting -min 2 / -max 8 results in the fitter adding 1.5-2ns of delay
# between flash_IO* and qpi_flash:flash|data_out, i.e. the fitter is delaying
# the input to give us a better hold time.

# TODO figure out why this all works with just zeros everywhere!
set_input_delay -clock $clock_96 -min 0 [get_ports flash_IO*]
set_input_delay -clock $clock_96 -max 0 [get_ports flash_IO*]


# *** SDRAM at 96MHz ***

# References:
# - http://retroramblings.net/?p=515
# - https://www.joshbassett.info/sdram-controller/
# - http://codehackcreate.com/archives/444

# MT48LC16M16A2F4-6A:GTR: 167 MHz CL3, or 100 MHz CL2.

# Access time at CL=3, f=166MHz (tCK(3) =  6ns): tAC(3) = 5.4 ns
# Access time at CL=2, f=100MHz (tCK(2) = 10ns): tAC(2) = 7.5 ns
#
# I think these are calculated using:
# tAC(3) = 18ns (=CL) - 2 * tCK(3) = 6ns (so CL here is really 17.4ns)
# tAC(2) = 18ns (=CL) - tCK(2) = 8ns (so CL here is really 17.5ns)
#
# Our clock period is 1000/96 = 10.41666 ns, so at CL=2 tAC is probably 17.5-10.41666 = 7.083ns.

# Address, control, data outputs to SDRAM
set sdram_board_delay 1
# We need to give the SDRAM ~2.5ns setup time because of board delays
set sdram_setup_time [expr 1.5 + $sdram_board_delay]
# Board delays work in our favor for hold time, so just 0.8ns is fine.
set sdram_hold_time 0.8
set_output_delay -clock ram_output_clock -max $sdram_setup_time [get_ports sdram_CKE]
set_output_delay -clock ram_output_clock -min -$sdram_hold_time [get_ports sdram_CKE] -add_delay
set_output_delay -clock ram_output_clock -max $sdram_setup_time [get_ports sdram_nCS]
set_output_delay -clock ram_output_clock -min -$sdram_hold_time [get_ports sdram_nCS] -add_delay
set_output_delay -clock ram_output_clock -max $sdram_setup_time [get_ports sdram_nRAS]
set_output_delay -clock ram_output_clock -min -$sdram_hold_time [get_ports sdram_nRAS] -add_delay
set_output_delay -clock ram_output_clock -max $sdram_setup_time [get_ports sdram_nCAS]
set_output_delay -clock ram_output_clock -min -$sdram_hold_time [get_ports sdram_nCAS] -add_delay
set_output_delay -clock ram_output_clock -max $sdram_setup_time [get_ports sdram_nWE]
set_output_delay -clock ram_output_clock -min -$sdram_hold_time [get_ports sdram_nWE] -add_delay
set_output_delay -clock ram_output_clock -max $sdram_setup_time [get_ports sdram_BA[*]]
set_output_delay -clock ram_output_clock -min -$sdram_hold_time [get_ports sdram_BA[*]] -add_delay
set_output_delay -clock ram_output_clock -max $sdram_setup_time [get_ports sdram_A[*]]
set_output_delay -clock ram_output_clock -min -$sdram_hold_time [get_ports sdram_A[*]] -add_delay
set_output_delay -clock ram_output_clock -max $sdram_setup_time [get_ports sdram_DQ[*]]
set_output_delay -clock ram_output_clock -min -$sdram_hold_time [get_ports sdram_DQ[*]] -add_delay
set_output_delay -clock ram_output_clock -max $sdram_setup_time [get_ports sdram_UDQM]
set_output_delay -clock ram_output_clock -min -$sdram_hold_time [get_ports sdram_UDQM] -add_delay
set_output_delay -clock ram_output_clock -max $sdram_setup_time [get_ports sdram_LDQM]
set_output_delay -clock ram_output_clock -min -$sdram_hold_time [get_ports sdram_LDQM] -add_delay

# Clock-to-output and hold time for SDRAM data outputs.
# From MT48LC16M16A2 datasheet - CL=18, and at 96MHz one clock is 10.42ns, so tAC = 18 - 10.42 = 7.58,
# or more precisely 17.5 - 10.417 = 7.083
set sdram_access_time 7.083
# tOH = 3ns
set sdram_data_hold_time 3
set_input_delay -clock ram_output_clock -max [expr $sdram_access_time + $sdram_board_delay] [get_ports sdram_DQ[*]]
set_input_delay -clock ram_output_clock -min $sdram_data_hold_time [get_ports sdram_DQ[*]] -add_delay

# Multicycle path for 96 MHz operation when ram_output_clock is slightly earlier than clock_96
set_multicycle_path -from [get_clocks {ram_output_clock}] -to [get_clocks $clock_96] -setup -end 2


# *** Audio DAC at 16MHz (62.5 ns) ***

# Clock outputs to DAC should be quick enough; let's say they have to be within 10ns of clock_16.

# In this case I think we want to do the opposite to a normal constraint; -max
# 10 -min 10 would specify that we want to set up our outputs 10 ns before
# clock_16 and hold them for 10 ns.  We actually want to set up our outputs
# sometime around clock_16, so we specify -10ns setup time (-max -10) and
# 52.5ns hold (-min -52.5).  This results in the fitter adding ~98.5ns of
# delay on dac_lrclk, dacdat, and bclk though, so something is wrong with my reasoning!

set_output_delay -clock $clock_16 -max 0 [get_ports dac_mclk]
set_output_delay -clock $clock_16 -min 0 [get_ports dac_mclk] -add_delay
set_output_delay -clock $clock_16 -max 0 [get_ports dac_bclk]
set_output_delay -clock $clock_16 -min 0 [get_ports dac_bclk] -add_delay

# Signal outputs can be sloppier; nmute, lrclk, and dacdat have lots of wiggle room, so let's say
# they can change anywhere within 20ns of the clock.
set_output_delay -clock $clock_16 -max 0 [get_ports dac_nmute]
set_output_delay -clock $clock_16 -min 0 [get_ports dac_nmute] -add_delay
set_output_delay -clock $clock_16 -max 0 [get_ports dac_lrclk]
set_output_delay -clock $clock_16 -min 0 [get_ports dac_lrclk] -add_delay
set_output_delay -clock $clock_16 -max 0 [get_ports dac_dacdat]
set_output_delay -clock $clock_16 -min 0 [get_ports dac_dacdat] -add_delay


# *** SPI with MCU is half rate ***
set_output_delay -clock $clock_96 -min 0 [get_ports mcu_MISO]
set_output_delay -clock $clock_96 -max 0 [get_ports mcu_MISO]
set_input_delay  -clock $clock_96 -min 0 [get_ports mcu_MOSI]
set_input_delay  -clock $clock_96 -max 0 [get_ports mcu_MOSI]
set_input_delay  -clock $clock_96 -min 0 [get_ports mcu_SCK]
set_input_delay  -clock $clock_96 -max 0 [get_ports mcu_SCK]
set_input_delay  -clock $clock_96 -min 0 [get_ports mcu_SS]
set_input_delay  -clock $clock_96 -max 0 [get_ports mcu_SS]


# *** MCU serial port
set_output_delay -clock $clock_96 -min 0 [get_ports mcu_debug_RXD]
set_output_delay -clock $clock_96 -max 0 [get_ports mcu_debug_RXD]
set_input_delay  -clock $clock_96 -min 0 [get_ports mcu_debug_TXD]
set_input_delay  -clock $clock_96 -max 0 [get_ports mcu_debug_TXD]


# *** External serial port
set_output_delay -clock $clock_96 -min 0 [get_ports serial_TXD]
set_output_delay -clock $clock_96 -max 0 [get_ports serial_TXD]
# TODO swap the following around when we stop using this as a debug output
set_output_delay -clock $clock_96 -min 0 [get_ports serial_RXD]
set_output_delay -clock $clock_96 -max 0 [get_ports serial_RXD]
#set_input_delay  -clock $clock_96 -min 0 [get_ports serial_RXD]
#set_input_delay  -clock $clock_96 -max 0 [get_ports serial_RXD]
# TODO remove these when we're using this as a serial port
set_false_path -from * -to [get_ports serial_*XD]


# *** Everything else is slow ***
set_output_delay -clock $clock_16 -min 0 [get_ports clk_out]
set_output_delay -clock $clock_16 -max 0 [get_ports clk_out]

set_output_delay -clock $clock_16 -min 0 [get_ports red]
set_output_delay -clock $clock_16 -max 0 [get_ports red]
set_output_delay -clock $clock_16 -min 0 [get_ports green]
set_output_delay -clock $clock_16 -max 0 [get_ports green]
set_output_delay -clock $clock_16 -min 0 [get_ports blue]
set_output_delay -clock $clock_16 -max 0 [get_ports blue]
set_output_delay -clock $clock_16 -min 0 [get_ports csync]
set_output_delay -clock $clock_16 -max 0 [get_ports csync]

set_input_delay  -clock $clock_16 -min 0 [get_ports sd_DAT0_MISO]
set_input_delay  -clock $clock_16 -max 0 [get_ports sd_DAT0_MISO]
set_output_delay -clock $clock_16 -min 0 [get_ports sd_CLK_SCK]
set_output_delay -clock $clock_16 -max 0 [get_ports sd_CLK_SCK]
set_output_delay -clock $clock_16 -min 0 [get_ports sd_CMD_MOSI]
set_output_delay -clock $clock_16 -max 0 [get_ports sd_CMD_MOSI]

set_input_delay  -clock $clock_16 -min 0 [get_ports IRQ_n_in]
set_input_delay  -clock $clock_16 -max 0 [get_ports IRQ_n_in]
set_output_delay -clock $clock_16 -min 0 [get_ports IRQ_n_out]
set_output_delay -clock $clock_16 -max 0 [get_ports IRQ_n_out]

set_output_delay -clock $clock_16 -min 0 [get_ports ROM_n]
set_output_delay -clock $clock_16 -max 0 [get_ports ROM_n]

set_input_delay  -clock $clock_16 -min 0 [get_ports NMI_n_in]
set_input_delay  -clock $clock_16 -max 0 [get_ports NMI_n_in]

set_input_delay  -clock $clock_16 -min 0 [get_ports RST_n_in]
set_input_delay  -clock $clock_16 -max 0 [get_ports RST_n_in]
set_output_delay -clock $clock_16 -min 0 [get_ports RST_n_out]
set_output_delay -clock $clock_16 -max 0 [get_ports RST_n_out]

set_output_delay -clock $clock_16 -min 0 [get_ports RnW_out]
set_output_delay -clock $clock_16 -max 0 [get_ports RnW_out]

set_output_delay -clock $clock_16 -min 0 [get_ports addr[*]]
set_output_delay -clock $clock_16 -max 0 [get_ports addr[*]]

set_input_delay  -clock $clock_16 -min 0 [get_ports casIn]
set_input_delay  -clock $clock_16 -max 0 [get_ports casIn]
set_output_delay -clock $clock_16 -min 0 [get_ports casOut]
set_output_delay -clock $clock_16 -max 0 [get_ports casOut]
set_output_delay -clock $clock_16 -min 0 [get_ports casMO]
set_output_delay -clock $clock_16 -max 0 [get_ports casMO]

set_input_delay  -clock $clock_16 -min 0 [get_ports data[*]]
set_input_delay  -clock $clock_16 -max 0 [get_ports data[*]]
set_output_delay -clock $clock_16 -min 0 [get_ports data[*]]
set_output_delay -clock $clock_16 -max 0 [get_ports data[*]]
set_output_delay -clock $clock_16 -min 0 [get_ports D_buf_DIR]
set_output_delay -clock $clock_16 -max 0 [get_ports D_buf_DIR]

set_output_delay -clock $clock_16 -min 0 [get_ports caps]
set_output_delay -clock $clock_16 -max 0 [get_ports caps]
set_input_delay  -clock $clock_16 -min 0 [get_ports kbd[*]]
set_input_delay  -clock $clock_16 -max 0 [get_ports kbd[*]]

# data[*] is a slow external bus, so output from clock_96 can take two cycles
#set_multicycle_path -from [get_clocks $clock_96] -to [get_ports data[*]] -setup -end 2

# General multicycle to say that anything coming from clock_96 to clock_16
# will be ready one clock_96 cycle into the clock_16 period and will be valid
# until the end of the clock_16 period.

# See http://www.ee.bgu.ac.il/~digivlsi/slides/Multicycles_6_2.pdf Figure 0-12
# for an explanation of what's happening here.  We could possibly also use 6
# -setup / 5 -hold.

set_multicycle_path 5 -setup -start -from [get_clocks $clock_96] -to [get_clocks $clock_16]
set_multicycle_path 4 -hold -start -from [get_clocks $clock_96] -to [get_clocks $clock_16]


### ASYNCHRONOUS CLOCKS ###

set_clock_groups -asynchronous -group $clock_16 -group $clock_32
set_clock_groups -asynchronous -group $clock_32 -group $clock_16
set_clock_groups -asynchronous -group $clock_16 -group $clock_33
set_clock_groups -asynchronous -group $clock_33 -group $clock_16
set_clock_groups -asynchronous -group $clock_16 -group $clock_40
set_clock_groups -asynchronous -group $clock_40 -group $clock_16
