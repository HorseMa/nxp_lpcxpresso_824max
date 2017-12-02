/*
 * @brief NXP LPCXpresso LPC824 Sysinit file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"

/* The System initialization code is called prior to the application and
   initializes the board for run-time operation. Board initialization
   for the NXP LPC824 board includes default pin muxing and clock setup
   configuration. */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Sets up system pin muxing */
void Board_SetupMuxing(void)
{
	/* Enable IOCON and Switch Matrix clocks */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
}

/* Set up and initialize clocking prior to call to main */
void Board_SetupClocking(void)
{
	/* Crystal is available on the board
	 * but not connected by default.
	 */
	Chip_SetupIrcClocking();
        Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
  	/* Enable power to the system osc */
	Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_SYSOSC_PD);

	/* Set the P0.8 and P0.9 pin modes to no pull-up or pull-down */
	Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO8, PIN_MODE_INACTIVE);
	Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO9, PIN_MODE_INACTIVE);

	/* Enable SYSOSC function on the pins */
	Chip_SWM_FixedPinEnable(SWM_FIXED_XTALIN, true);
	Chip_SWM_FixedPinEnable(SWM_FIXED_XTALOUT, true);
        Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
        
	//Chip_SetupXtalClocking();
	SystemCoreClockUpdate();
}

/* Set up and initialize hardware prior to call to main */
void Board_SystemInit(void)
{
	/* Setup system clocking and muxing */
	Board_SetupMuxing();
	Board_SetupClocking();

	/* IOCON clock left on, but may be turned off if no other IOCON
	   changes are needed */
}
