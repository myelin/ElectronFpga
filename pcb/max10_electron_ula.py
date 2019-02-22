#!/usr/bin/python

# Copyright 2019 Google LLC
#
# This source file is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This source file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# ------------------
# max10_electron_ula
# ------------------

# by Phillip Pearson

# A PCB for the Electron ULA replacement project, using an Intel Max 10 FPGA
# chip (10M08).

import sys, os
here = os.path.dirname(sys.argv[0])
sys.path.insert(0, os.path.join(here, "myelin-kicad.pretty"))
import myelin_kicad_pcb
Pin = myelin_kicad_pcb.Pin


# Electron ULA header -- this will plug into the PGA socket once one has been
# installed, as shown in Dave Hitchins' post:
# https://stardot.org.uk/forums/viewtopic.php?f=3&t=9223&start=60#p118234

# The ULA came in various different forms over the years, but in all cases the
# PCB can take a standard 68-pin PGA socket once the ULA socket (Issue 4) or
# ULA carrier board (Issue 6) is desoldered.  Details by Hoglet here:
# https://stardot.org.uk/forums/viewtopic.php?f=3&t=9223&start=90#p149175

# The following list of signal details was collated by jmw2:
# https://stardot.org.uk/forums/viewtopic.php?f=3&t=9223&start=30#p105883
#
# 16 address lines (inputs only)
# 8 processor data lines (bi-directional)
# 4 keyboard lines, CAPS LK, RST (inputs only)
# IRQ, NMI, Phi0, RnW (outputs only)
# Clock and Divide-by-13 (inputs)
# R, G, B & sync (outputs)
# 4 cassette lines (CASOUT, CASRC and CASMO outputs, CASIN input)
# Sound (output)
# nPOR (power-on reset)

ula = myelin_kicad_pcb.Component(
    footprint="Package_LCC:PLCC-68_THT-Socket",
    identifier="ULA",
    value="ULA header",
    desc="Set of pin headers to plug into an Acorn Electron ULA socket",
    pins=[
        Pin( 1, "", ""),
        Pin( 2, "", ""),
        Pin( 3, "", ""),
        Pin( 4, "", ""),
        Pin( 5, "", ""),
        Pin( 6, "", ""),
        Pin( 7, "", ""),
        Pin( 8, "", ""),
        Pin( 9, "", ""),
        Pin(10, "", ""),
        Pin(11, "", ""),
        Pin(12, "", ""),
        Pin(13, "", ""),
        Pin(14, "", ""),
        Pin(15, "", ""),
        Pin(16, "", ""),
        Pin(17, "", ""),
        Pin(18, "", ""),
        Pin(19, "", ""),
        Pin(20, "", ""),
        Pin(21, "", ""),
        Pin(22, "", ""),
        Pin(23, "", ""),
        Pin(24, "", ""),
        Pin(25, "", ""),
        Pin(26, "", ""),
        Pin(27, "", ""),
        Pin(28, "", ""),
        Pin(29, "", ""),
        Pin(30, "", ""),
        Pin(31, "", ""),
        Pin(32, "", ""),
        Pin(33, "", ""),
        Pin(34, "", ""),
        Pin(35, "", ""),
        Pin(36, "", ""),
        Pin(37, "", ""),
        Pin(38, "", ""),
        Pin(39, "", ""),
        Pin(40, "", ""),
        Pin(41, "", ""),
        Pin(42, "", ""),
        Pin(43, "", ""),
        Pin(44, "", ""),
        Pin(45, "", ""),
        Pin(46, "", ""),
        Pin(47, "", ""),
        Pin(48, "", ""),
        Pin(49, "", ""),
        Pin(50, "", ""),
        Pin(51, "", ""),
        Pin(52, "", ""),
        Pin(53, "", ""),
        Pin(54, "", ""),
        Pin(55, "", ""),
        Pin(56, "", ""),
        Pin(57, "", ""),
        Pin(58, "", ""),
        Pin(59, "", ""),
        Pin(60, "", ""),
        Pin(61, "", ""),
        Pin(62, "", ""),
        Pin(63, "", ""),
        Pin(64, "", ""),
        Pin(65, "", ""),
        Pin(66, "", ""),
        Pin(67, "", ""),
        Pin(68, "", ""),
    ],
)


fpga = myelin_kicad_pcb.Component(
    footprint="myelin-kicad:intel_ubga169",
    identifier="FPGA",
    value="10M08SCU169",
    pins=[
        # IOs

        # Outer ring -- 45 IOs (4 x 13 - 1 for TMS on G1)
        Pin("A2",  "", "C1_11"),
        Pin("A3",  "", "C1_9"),
        Pin("A4",  "", "C1_14"),
        Pin("A5",  "", "C1_19"),
        Pin("A6",  "", "C1_17"),
        Pin("A7",  "", "C1_23"),
        Pin("A8",  "", "C1_16"),
        Pin("A9",  "", "C1_20"),
        Pin("A10", "", "C1_26"),
        Pin("A11", "", "C1_25"),
        Pin("A12", "", "C1_30"),
        Pin("N4",  "", "C4_17"),
        Pin("N5",  "", "C4_19"),
        Pin("N6",  "", "C4_24"),
        Pin("N7",  "", "C4_25"),
        Pin("N8",  "", "C4_22"),
        Pin("N9",  "", "C4_28"),
        Pin("N10", "", "C4_26"),
        Pin("N11", "", "C4_31"),
        Pin("N12", "", "C4_36"),
        Pin("B1",  "", "C1_2"),
        Pin("C1",  "", "C1_1"),
        Pin("D1",  "", "C2_1"),
        Pin("E1",  "", "C2_5"),
        Pin("F1",  "", "C2_8"),
        Pin("H1",  "", "C1_3"),
        Pin("J1",  "", "C4_4"),
        Pin("K1",  "", "C2_12"),
        Pin("M1",  "", "C4_12"),
        Pin("B13", "", "C1_36"),
        Pin("C13", "", "C1_33"),
        Pin("D13", "", "C1_37"),
        Pin("G13", "", "C3_4"),
        Pin("H13", "", "C4_34"),
        Pin("J13", "", "C3_10"),
        Pin("K13", "", "C4_35"),
        Pin("L13", "", "C4_37"),
        Pin("M13", "", "C4_39"),

        # Next ring in -
        Pin("B2",  "", "C1_6"),
        Pin("B3",  "", "C1_5"),
        Pin("B4",  "", "C1_10"),
        Pin("B5",  "", "C1_13"),
        Pin("B6",  "", "C1_18"),
        Pin("B7",  "", "C1_12"),
        Pin("B10", "", "C1_24"),
        Pin("B11", "", "C1_31"),
        Pin("M2",  "", "C4_11"),
        Pin("M3",  "", "C4_13"),
        Pin("M4",  "", "C4_14"),
        Pin("M5",  "", "C4_20"),
        Pin("M7",  "", "C4_21"),
        Pin("M8",  "", "C4_23"),
        Pin("M9",  "", "C4_33"),
        Pin("M10", "", "C4_27"),
        Pin("M11", "", "C4_30"),
        Pin("M12", "", "C4_38"),

        Pin("C2",  "", "C2_4"),
        Pin("H2",  "", "C1_7"),
        Pin("J2",  "", "C4_7"),
        Pin("K2",  "", "C4_8"),
        Pin("L2",  "", "C4_6"),
        Pin("L3",  "", "C4_10"),
        Pin("C9",  "", "C1_22"),
        Pin("C10", "", "C1_27"),
        Pin("B12", "", "C1_29"),
        Pin("C12", "", "C1_34"),
        Pin("D11", "", "C1_38"),
        Pin("D12", "", "C3_3"),
        Pin("E12", "", "C3_1"),
        Pin("F12", "", "C3_6"),
        Pin("G12", "", "C3_8"),
        Pin("J12", "", "C3_7"),
        Pin("K11", "", "C3_12"),
        Pin("K12", "", "C3_9"),
        Pin("L11", "", "C3_11"),
        Pin("L12", "", "C4_40"),
        Pin("L10", "", "C4_32"),
        Pin("C11", "", "C1_28"),
        Pin("E3",  "", "C2_3"),
        Pin("L5",  "", "C4_18"),

        # Special pins
        Pin("G5", "CLK0n", "CLK0n"),
        Pin("H6", "CLK0p", "CLK0p"),
        Pin("H5", "CLK1n", "CLK1n"),
        Pin("H4", "CLK1p", "CLK1p"),
        Pin("G10", "CLK2n", "CLK2n"),
        Pin("G9", "CLK2p", "CLK2p"),
        Pin("E13", "CLK3n", "C3_2_CLK3n"),
        Pin("F13", "CLK3p", "C3_5_CLK3p"),
        Pin("N2", "DPCLK0", "C4_16_DPCLK0"),
        Pin("N3", "DPCLK1", "C4_15_DPCLK1"),
        Pin("F10", "DPCLK2", "DPCLK2"),
        Pin("F9", "DPCLK3", "DPCLK3"),
        Pin("L1", "VREFB2N0", "C4_3_VREFB2N0"),

        # JTAG and other config pins
        Pin("E5",  "JTAGEN",     "fpga_JTAGEN"),
        Pin("G1",  "TMS",        "fpga_TMS"),
        Pin("G2",  "TCK",        "fpga_TCK"),
        Pin("F5",  "TDI",        "fpga_TDI"),
        Pin("F6",  "TDO",        "fpga_TDO"),
        Pin("B9",  "DEV_CLRn",   "fpga_DEV_CLRn"),  # measures high on first soldered board
        Pin("D8",  "DEV_OE",     "fpga_DEV_OE"),
        Pin("D7",  "CONFIG_SEL", "GND"),  # unused, so connected to GND
        Pin("E7",  "nCONFIG",    "3V3"),  # can be connected straight to VCCIO
        Pin("D6",  "CRC_ERROR",  "GND"),  # WARNING: disable Error Detection CRC option
        Pin("C4",  "nSTATUS",    "fpga_nSTATUS"),  # measures high on first soldered board
        Pin("C5",  "CONF_DONE",  "fpga_CONF_DONE"),  # measures high on first soldered board

        # Signals used as power/ground to enable vias in the 2-layer version
        # TODO none of these are listed as IOs; add them
        Pin("E4",  "",     ""),  # TODO
        Pin("J5",  "",     ""),  # TODO
        Pin("J6",  "",     ""),  # TODO
        Pin("K10", "",     ""),  # TODO
        Pin("E10", "",     ""),  # TODO
        Pin("J10", "",     ""),  # TODO
        Pin("E8",  "",     ""),  # TODO
        Pin("H8",  "",     ""),  # TODO
        Pin("D9",  "",     ""),  # TODO
        Pin("K7",  "",     ""),  # TODO
        Pin("K8",  "",     ""),  # TODO
        Pin("E6",  "",     ""),  # TODO
        Pin("F8",  "",     ""),  # TODO
        Pin("G4",  "",     ""),  # TODO
        Pin("L4",  "",     ""),  # TODO

        # Power and ground
        Pin("D2",  "GND",     "GND"),
        Pin("E2",  "GND",     "GND"),
        Pin("N13", "GND",     "GND"),
        Pin("N1",  "GND",     "GND"),
        Pin("M6",  "GND",     "GND"),
        Pin("L9",  "GND",     "GND"),
        Pin("J4",  "GND",     "GND"),
        Pin("H12", "GND",     "GND"),
        Pin("G7",  "GND",     "GND"),
        Pin("F3",  "GND",     "GND"),
        Pin("E11", "GND",     "GND"),
        Pin("D5",  "GND",     "GND"),
        Pin("C3",  "GND",     "GND"),
        Pin("B8",  "GND",     "GND"),
        Pin("A13", "GND",     "GND"),
        Pin("A1",  "GND",     "GND"),
        Pin("F2",  "VCCIO1A", "3V3"),
        Pin("G3",  "VCCIO1B", "3V3"),
        Pin("K3",  "VCCIO2",  "3V3"),
        Pin("J3",  "VCCIO2",  "3V3"),
        Pin("L8",  "VCCIO3",  "3V3"),
        Pin("L7",  "VCCIO3",  "3V3"),
        Pin("L6",  "VCCIO3",  "3V3"),
        Pin("J11", "VCCIO5",  "3V3"),
        Pin("H11", "VCCIO5",  "3V3"),
        Pin("G11", "VCCIO6",  "3V3"),
        Pin("F11", "VCCIO6",  "3V3"),
        Pin("C8",  "VCCIO8",  "3V3"),
        Pin("C7",  "VCCIO8",  "3V3"),
        Pin("C6",  "VCCIO8",  "3V3"),
        Pin("K4",  "VCCA1",   "3V3"),
        Pin("D10", "VCCA2",   "3V3"),
        Pin("D3",  "VCCA3",   "3V3"),
        Pin("D4",  "VCCA3",   "3V3"),
        Pin("K9",  "VCCA4",   "3V3"),
        Pin("H7",  "VCC_ONE", "3V3"),
        Pin("G8",  "VCC_ONE", "3V3"),
        Pin("G6",  "VCC_ONE", "3V3"),
        Pin("F7",  "VCC_ONE", "3V3"),
    ],
)

# chip won't init unless this is pulled high
conf_done_pullup = myelin_kicad_pcb.R0805("10k", "fpga_CONF_DONE", "3V3", ref="R1", handsoldering=False)

# chip goes into error state if this is pulled low
nstatus_pullup = myelin_kicad_pcb.R0805("10k", "fpga_nSTATUS", "3V3", ref="R2", handsoldering=False)

# prevent spurious jtag clocks
tck_pulldown = myelin_kicad_pcb.R0805("1-10k", "fpga_TCK", "GND", ref="R3", handsoldering=False)

# fpga_nCONFIG doesn't need a pullup, just connect straight to 3V3

# fpga_CONFIG_SEL is connected to GND because we don't use this

decoupling = [
    myelin_kicad_pcb.C0402("100n", "3V3", "GND", ref="C%d" % r)
    for r in range(10, 24)
]
bulk = [
    myelin_kicad_pcb.C0805("1u", "3V3", "GND", ref="C%d" % r, handsoldering=False)
    for r in range(8, 10)
]

myelin_kicad_pcb.dump_netlist("max10_electron_ula.net")
