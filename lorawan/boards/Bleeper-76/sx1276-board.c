/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "radio.h"
#include "sx1276/sx1276.h"
#include "sx1276-board.h"

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby, 
    SX1276SetRx,
    SX1276StartCad,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength
};

/*!
 * Antenna switch GPIO pins objects
 */
uint8_t AntSwitchLf;
uint8_t AntSwitchHf;

void SX1276IoInit( void )
{
    SPI_DELAY_CONFIG_T DelayConfigStruct;

    /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
    Chip_SYSCTL_SetPinInterrupt(0, 6);

    /* Configure channel 0 as wake up interrupt in SysCon block */
    Chip_SYSCTL_EnablePINTWakeup(0);

    /* Configure GPIO pin as input pin */
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 6);

    /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
    Chip_SYSCTL_SetPinInterrupt(1, 14);

    /* Configure channel 0 as wake up interrupt in SysCon block */
    Chip_SYSCTL_EnablePINTWakeup(1);

    /* Configure GPIO pin as input pin */
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 1, 14);

    /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
    Chip_SYSCTL_SetPinInterrupt(2, 0);

    /* Configure channel 0 as wake up interrupt in SysCon block */
    Chip_SYSCTL_EnablePINTWakeup(2);

    /* Configure GPIO pin as input pin */
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 2, 0);

    /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
    Chip_SYSCTL_SetPinInterrupt(3, 15);

    /* Configure channel 0 as wake up interrupt in SysCon block */
    Chip_SYSCTL_EnablePINTWakeup(3);

    /* Configure GPIO pin as input pin */
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 3, 15);

    /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
    Chip_SYSCTL_SetPinInterrupt(4, 7);

    /* Configure channel 0 as wake up interrupt in SysCon block */
    Chip_SYSCTL_EnablePINTWakeup(4);

    /* Configure GPIO pin as input pin */
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 4, 7);

    /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
    Chip_SYSCTL_SetPinInterrupt(5, 24);

    /* Configure channel 0 as wake up interrupt in SysCon block */
    Chip_SYSCTL_EnablePINTWakeup(5);

    /* Configure GPIO pin as input pin */
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 5, 24);
    //GpioInit( &SX1276.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );

    //GpioInit( &SX1276.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    //GpioInit( &SX1276.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    //GpioInit( &SX1276.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    //GpioInit( &SX1276.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    //GpioInit( &SX1276.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    //GpioInit( &SX1276.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );

    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
    Chip_SWM_MovablePinAssign(SWM_SPI1_SSEL0_IO, 26);
    Chip_SWM_MovablePinAssign(SWM_SPI1_SCK_IO, 27);
    Chip_SWM_MovablePinAssign(SWM_SPI1_MISO_IO, 11);
    Chip_SWM_MovablePinAssign(SWM_SPI1_MOSI_IO, 16);
    //Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
    /*
       ConfigStruct.Mode = SPI_MODE_MASTER;
       ConfigStruct.ClkDiv = Chip_SPI_CalClkRateDivider(LPC_SPI, 100000);
       ConfigStruct.ClockMode = SPI_CLOCK_CPHA0_CPOL0;
       ConfigStruct.DataOrder = SPI_DATA_MSB_FIRST;
       ConfigStruct.SSELPol = SPI_SSEL_ACTIVE_LO;
     */
    Chip_SPI_Init(LPC_SPI1);
    Chip_SPI_ConfigureSPI(LPC_SPI1, SPI_MODE_MASTER |  /* Enable master/Slave mode */
                          SPI_CLOCK_CPHA0_CPOL0 |   /* Set Clock polarity to 0 */
                          SPI_CFG_MSB_FIRST_EN |/* Enable MSB first option */
                          SPI_CFG_SPOL_LO); /* Chipselect is active low */

    DelayConfigStruct.FrameDelay = 0;
    DelayConfigStruct.PostDelay = 0;
    DelayConfigStruct.PreDelay = 0;
    DelayConfigStruct.TransferDelay = 0;
    Chip_SPI_DelayConfig(LPC_SPI1, &DelayConfigStruct);

    Chip_SPI_Enable(LPC_SPI1);
}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
    /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH0);
    Chip_PININT_EnableIntLow(LPC_PININT, PININTCH0);

    /* Enable interrupt in the NVIC */
    NVIC_EnableIRQ(PININT0_IRQn);

    /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH1);
    Chip_PININT_EnableIntLow(LPC_PININT, PININTCH1);

    /* Enable interrupt in the NVIC */
    NVIC_EnableIRQ(PININT1_IRQn);

    /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH2);
    Chip_PININT_EnableIntLow(LPC_PININT, PININTCH2);

    /* Enable interrupt in the NVIC */
    NVIC_EnableIRQ(PININT2_IRQn);

    /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH3);
    Chip_PININT_EnableIntLow(LPC_PININT, PININTCH3);

    /* Enable interrupt in the NVIC */
    NVIC_EnableIRQ(PININT3_IRQn);

    /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH4);
    Chip_PININT_EnableIntLow(LPC_PININT, PININTCH4);

    /* Enable interrupt in the NVIC */
    NVIC_EnableIRQ(PININT4_IRQn);

    /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH5);
    Chip_PININT_EnableIntLow(LPC_PININT, PININTCH5);

    /* Enable interrupt in the NVIC */
    NVIC_EnableIRQ(PININT5_IRQn);
    //GpioSetInterrupt( &SX1276.DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[0] );
    //GpioSetInterrupt( &SX1276.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[1] );
    //GpioSetInterrupt( &SX1276.DIO2, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[2] );
    //GpioSetInterrupt( &SX1276.DIO3, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[3] );
    //GpioSetInterrupt( &SX1276.DIO4, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[4] );
    //GpioSetInterrupt( &SX1276.DIO5, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[5] );
}

void SX1276IoDeInit( void )
{
    //GpioInit( &SX1276.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    //GpioInit( &SX1276.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    //GpioInit( &SX1276.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    //GpioInit( &SX1276.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    //GpioInit( &SX1276.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    //GpioInit( &SX1276.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    //GpioInit( &SX1276.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

uint8_t SX1276GetPaSelect( uint32_t channel )
{
    if( channel > RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    if( RadioIsActive != status )
    {
        RadioIsActive = status;
    
        if( status == false )
        {
            SX1276AntSwInit( );
        }
        else
        {
            SX1276AntSwDeInit( );
        }
    }
}

void SX1276AntSwInit( void )
{
    //GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    //GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
}

void SX1276AntSwDeInit( void )
{
    //GpioInit( &AntSwitchLf, RADIO_ANT_SWITCH_LF, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
    //GpioInit( &AntSwitchHf, RADIO_ANT_SWITCH_HF, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
}

void SX1276SetAntSw( uint8_t rxTx )
{
    if( SX1276.RxTx == rxTx )
    {
        return;
    }

    SX1276.RxTx = rxTx;

    if( rxTx != 0 ) // 1: TX, 0: RX
    {
        //GpioWrite( &AntSwitchLf, 0 );
        //GpioWrite( &AntSwitchHf, 1 );
    }
    else
    {
        //GpioWrite( &AntSwitchLf, 1 );
        //GpioWrite( &AntSwitchHf, 0 );
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
