UART API in ROM (USART API ROM) Interrupt Example
=================================================

Example Description
-------------------
The UART_ROM example demonstrates using the USART API ROM functions
in interrupt mode for terminated data input and output.

An example ringbuffer capability is also provided using the
uart_get_line() and uart_put_line() functions in interrupt mode.

Special connection requirements
-------------------------------
See ../../../../examples_8xx/periph_uart/readme.txt for details on configuring the UART.

Board [NXP_LPCXPRESSO_812]:

Board [NXP_812_MAX]:

Board [NXP_LPCXPRESSO_824]:

NOTE: When connecting to the base board make sure P3 Pin4 [RST] pin of the
LPCXpresso board is *NOT* connected to the base board for the debugger to work.

