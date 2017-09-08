LPC8xx ADC Example
==================

Example Description
-------------------
The LPC82x ADC example shows how to use the ADC to perform a sequence of
conversions and monitor a threshold crossing. Sampling is started by manually
initiating the ADC sequence in the system tick interrupt. When the ADC sample
sequence is complete, it generates an interrupt. During ADC sequence conversion,
an interrupt will also be generated if the sample crosses the threshold value.

Notes:
1)	Converted ADC values are displayed via the DEBUG UART.
2)	When using the LPCXpresso base-board the debug UART loses several characters
	at the start of each transmission. This is due to C20 on the base-board.
	The issue can be resolved by removing C20.
3)	The LPCXpresso base-board does not require power to run this example.

Special Connection Requirements
-------------------------------
Board [NXP_LPCXPRESSO_812]:
There are no special connection requirements for this example.
The LPCXpresso 812 board has an on-board potentiometer.  This is connected to
ACMP-I1 (pin 19). The example uses this to drive the ADC.

Board [NXP_812_MAX]:
The 812 MAX board expects analog input on pin-19 (PIO0_0/ACMP_I1/TDO). The processor
pin connects to P2-10 on the LPCXpresso connector and pin J2-8 on the Arduino terminals.
To use this demo, connect a 100k linear potentiometer to the following Arduino pins:
	Potentiometer	Arduino		Notes
		T1 (end-1)	J8-4		3.3V
		T2 (wiper)	J2-8		comparator input
		T3 (end-2)	J8-7		GND

Board [NXP_LPCXPRESSO_824]:
There are no special connection requirements for this example.
The LPCXpresso 824 board expects analog input on pin-23 (P0_6/ADC1/VDDCMP). The demo is 
designed to use the LPCXpresso base-board without any special wiring or jumpers.
The processor pin connects to P3-15 on the LPCXpresso connector and pin J5-1 on
the Arduino terminals. Note however, that base-board J-27 must be installed.
This jumper connects the potentiometer (R105) to J4-15 on the LPCXpresso connector.


