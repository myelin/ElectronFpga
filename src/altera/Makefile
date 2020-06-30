test_qpi_flash:
	iverilog -o qpi_flash_test.vvp qpi_flash_test.v
	vvp qpi_flash_test.vvp -lxt2
	open -a gtkwave qpi_flash_test.lxt2
