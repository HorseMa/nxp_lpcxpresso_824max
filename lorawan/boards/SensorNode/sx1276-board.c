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
#include "sx1276.h"
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
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork
};

/*!
 * Antenna switch GPIO pins objects
 */
//Gpio_t AntSwitchLf;
//Gpio_t AntSwitchHf;

void SX1276IoInit( void )
{
  SPI_DELAY_CONFIG_T DelayConfigStruct;
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
  /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
  //Chip_SWM_MovablePinAssign(SWM_SCT_IN0_I, 19);
  Chip_SYSCTL_SetPinInterrupt(0, 19);   // DIO0

  /* Configure channel 0 as wake up interrupt in SysCon block */
  Chip_SYSCTL_EnablePINTWakeup(0);

  /* Configure GPIO pin as input pin */
  Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 19);

  /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
  //Chip_SWM_MovablePinAssign(SWM_SCT_IN1_I, 18);
  Chip_SYSCTL_SetPinInterrupt(1, 18);   // DIO1

  /* Configure channel 0 as wake up interrupt in SysCon block */
  Chip_SYSCTL_EnablePINTWakeup(1);

  /* Configure GPIO pin as input pin */
  Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 18);

  /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
  //Chip_SWM_MovablePinAssign(SWM_SCT_IN2_I, 17);
  Chip_SYSCTL_SetPinInterrupt(2, 17);   // DIO2

  /* Configure channel 0 as wake up interrupt in SysCon block */
  Chip_SYSCTL_EnablePINTWakeup(2);

  /* Configure GPIO pin as input pin */
  Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 17);

  /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
  //Chip_SWM_MovablePinAssign(SWM_SCT_IN3_I, 21);
  Chip_SYSCTL_SetPinInterrupt(3, 21);   // DIO3

  /* Configure channel 0 as wake up interrupt in SysCon block */
  Chip_SYSCTL_EnablePINTWakeup(3);

  /* Configure GPIO pin as input pin */
  Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 21);

  /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
  //Chip_SWM_MovablePinAssign(SWM_SCT_IN4_I, 20);
  Chip_SYSCTL_SetPinInterrupt(4, 20);   // DIO4

  /* Configure channel 0 as wake up interrupt in SysCon block */
  Chip_SYSCTL_EnablePINTWakeup(4);

  /* Configure GPIO pin as input pin */
  Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 20);
  
  #if 0   // dio5 is not used, interrupt too often
  /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
  //Chip_SWM_MovablePinAssign(SWM_SCT_IN5_I, 22);
  Chip_SYSCTL_SetPinInterrupt(5, 22);   // DIO5

  /* Configure channel 0 as wake up interrupt in SysCon block */
  Chip_SYSCTL_EnablePINTWakeup(5);

  /* Configure GPIO pin as input pin */
  Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 22);
#endif
  
  Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,23,TRUE); // NRESET
  Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO23,PIN_MODE_PULLUP);
  
  Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,14,TRUE); // SSEL
  Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO14,PIN_MODE_PULLUP);
  Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,25,TRUE); // SCK
  Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO25,PIN_MODE_PULLUP);
  Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,7,FALSE); // MISO
  Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO7,PIN_MODE_INACTIVE);
  Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,6,TRUE); // MOSI
  Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO6,PIN_MODE_PULLUP);
  
  
  Chip_SWM_MovablePinAssign(SWM_SPI1_SSEL0_IO, 14);
  Chip_SWM_MovablePinAssign(SWM_SPI1_SCK_IO, 25);
  Chip_SWM_MovablePinAssign(SWM_SPI1_MISO_IO, 7);
  Chip_SWM_MovablePinAssign(SWM_SPI1_MOSI_IO, 6);
  Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
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
  Chip_SPIM_SetClockRate(LPC_SPI1,1000000);
  DelayConfigStruct.FrameDelay = 0;
  DelayConfigStruct.PostDelay = 0;
  DelayConfigStruct.PreDelay = 0;
  DelayConfigStruct.TransferDelay = 0;
  Chip_SPI_DelayConfig(LPC_SPI1, &DelayConfigStruct);
  Chip_SPI_Enable(LPC_SPI1);
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SPI1);
}

void SX1276IoIrqInit( void )
{
  /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
  Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH0);
  Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH0);

  /* Enable interrupt in the NVIC */
  NVIC_EnableIRQ(PININT0_IRQn);

  /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
  Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH1);
  Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH1);

  /* Enable interrupt in the NVIC */
  NVIC_EnableIRQ(PININT1_IRQn);

  /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
  Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH2);
  Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH2);

  /* Enable interrupt in the NVIC */
  NVIC_EnableIRQ(PININT2_IRQn);

  /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
  Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH3);
  Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH3);

  /* Enable interrupt in the NVIC */
  NVIC_EnableIRQ(PININT3_IRQn);

  /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
  Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH4);
  Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH4);

  /* Enable interrupt in the NVIC */
  NVIC_EnableIRQ(PININT4_IRQn);
#if 0
  /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
  Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH5);
  Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH5);

  /* Enable interrupt in the NVIC */
  NVIC_EnableIRQ(PININT5_IRQn);
#endif
}

void SX1276IoDeInit( void )
{
#if 0
    GpioInit( &SX1276.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    GpioInit( &SX1276.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1276.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1276.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1276.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1276.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX1276.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( SX1276.Settings.Channel );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

uint8_t SX1276GetPaSelect( uint32_t channel )
{
    if( channel < RF_MID_BAND_THRESH )
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

void SX1276SetAntSw( uint8_t opMode )
{
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        //GpioWrite( &AntSwitchLf, 0 );
        //GpioWrite( &AntSwitchHf, 1 );
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        //GpioWrite( &AntSwitchLf, 1 );
        //GpioWrite( &AntSwitchHf, 0 );
        break;
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
