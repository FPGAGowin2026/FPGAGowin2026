//Copyright (C)2014-2026 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.12.03 (64-bit)
//IP Version: 1.0
//Part Number: GW5A-LV25MG121NC1/I0
//Device: GW5A-25
//Device Version: B
//Created Time: Thu Aug 27 03:30:13 2026

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

	Gowin_GoRV32_Top your_instance_name(
		.HCLK(HCLK), //input HCLK
		.PCLK(PCLK), //input PCLK
		.POR_N(POR_N), //input POR_N
		.UART1_CTSN(UART1_CTSN), //input UART1_CTSN
		.UART1_DCDN(UART1_DCDN), //input UART1_DCDN
		.UART1_DSRN(UART1_DSRN), //input UART1_DSRN
		.UART1_RIN(UART1_RIN), //input UART1_RIN
		.UART1_RXD(UART1_RXD), //input UART1_RXD
		.UART1_DTRN(UART1_DTRN), //output UART1_DTRN
		.UART1_OUT1N(UART1_OUT1N), //output UART1_OUT1N
		.UART1_OUT2N(UART1_OUT2N), //output UART1_OUT2N
		.UART1_RTSN(UART1_RTSN), //output UART1_RTSN
		.UART1_TXD(UART1_TXD), //output UART1_TXD
		.FLASH_QSPI_CSN(FLASH_QSPI_CSN), //inout FLASH_QSPI_CSN
		.FLASH_QSPI_MISO(FLASH_QSPI_MISO), //inout FLASH_QSPI_MISO
		.FLASH_QSPI_MOSI(FLASH_QSPI_MOSI), //inout FLASH_QSPI_MOSI
		.FLASH_QSPI_CLK(FLASH_QSPI_CLK), //inout FLASH_QSPI_CLK
		.FLASH_QSPI_HOLDN(FLASH_QSPI_HOLDN), //inout FLASH_QSPI_HOLDN
		.FLASH_QSPI_WPN(FLASH_QSPI_WPN), //inout FLASH_QSPI_WPN
		.JTAG_TMS(JTAG_TMS), //input JTAG_TMS
		.JTAG_TDI(JTAG_TDI), //input JTAG_TDI
		.JTAG_TDO(JTAG_TDO), //output JTAG_TDO
		.JTAG_TCK(JTAG_TCK) //input JTAG_TCK
	);

//--------Copy end-------------------
