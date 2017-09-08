CLKOUT Example
==============

Example Description
-------------------
This example shows how to use the SYSCTL driver to enable a specific
clock source on the CLKOUT pin. To use this example, you'll need to
connect an oscilloscope to the CLKOUT pin on your board which is connected
to PIO0_19.

Using the standard configuration: standard clock initialization, using internal
clock only the demo will output the following:
	-- 24MHZ clock for 30-seconds
	-- 12MHz clock for 30-seconds
	-- 523kHz clock for 30-seconds
This sequence repeats indefinitely.

Special Connection Requirements
-------------------------------
Board [NXP_LPCXPRESSO_812]:
There are no special connection requirements for this example.

Board [NXP_812_MAX]:
There are no special connection requirements for this example.

Board [NXP_LPCXPRESSO_824]:
There are no special connection requirements for this example.
