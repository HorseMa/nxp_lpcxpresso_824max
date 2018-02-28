/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

/*! \file classA/SensorNode/main.c */

#include <string.h>
#include <math.h>
#include "board.h"
#include "utilities.h"
#include "LoRaMac.h"
#include "Region.h"
#include "Commissioning.h"
#include "modem.h"
#include "radio.h"
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )

#define LC4                { 867100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, 0, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, 0, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

#endif

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size
 */
#if defined( REGION_CN470 ) || defined( REGION_CN779 ) || defined( REGION_EU433 ) || defined( REGION_EU868 ) || defined( REGION_IN865 ) || defined( REGION_KR920 )

#define LORAWAN_APP_DATA_SIZE                       16

#elif defined( REGION_AS923 ) || defined( REGION_AU915 ) || defined( REGION_US915 ) || defined( REGION_US915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#else

#error "Please define a region in the compiler options."

#endif

uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           242

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

extern RINGBUFF_T txring, rxring;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

/*!
 * \brief   Prepares the payload of the frame
 */
void PrepareTxFrame( uint8_t port ,uint8_t *data ,uint8_t len)
{
    if((port > 0) && (port < 224))
    {
        memcpy(AppData,data,len);
        AppPort = port;
        AppDataSize = len;
    }
#if 0
    switch( port )
    {
    case 2:
        {
#if defined( REGION_CN470 ) || defined( REGION_CN779 ) || defined( REGION_EU433 ) || defined( REGION_EU868 ) || defined( REGION_IN865 ) || defined( REGION_KR920 )
            uint16_t pressure = 0;
            int16_t altitudeBar = 0;
            int16_t temperature = 0;
            int32_t latitude, longitude = 0;
            int16_t altitudeGps = 0xFFFF;
            uint8_t batteryLevel = 0;

            //pressure = ( uint16_t )( MPL3115ReadPressure( ) / 10 );             // in hPa / 10
            //temperature = ( int16_t )( MPL3115ReadTemperature( ) * 100 );       // in °C * 100
            //altitudeBar = ( int16_t )( MPL3115ReadAltitude( ) * 10 );           // in m * 10
            //batteryLevel = BoardGetBatteryLevel( );                             // 1 (very low) to 254 (fully charged)
            //GpsGetLatestGpsPositionBinary( &latitude, &longitude );
            //altitudeGps = GpsGetLatestGpsAltitude( );                           // in m

            AppData[0] = AppLedStateOn;
            AppData[1] = ( pressure >> 8 ) & 0xFF;
            AppData[2] = pressure & 0xFF;
            AppData[3] = ( temperature >> 8 ) & 0xFF;
            AppData[4] = temperature & 0xFF;
            AppData[5] = ( altitudeBar >> 8 ) & 0xFF;
            AppData[6] = altitudeBar & 0xFF;
            AppData[7] = batteryLevel;
            AppData[8] = ( latitude >> 16 ) & 0xFF;
            AppData[9] = ( latitude >> 8 ) & 0xFF;
            AppData[10] = latitude & 0xFF;
            AppData[11] = ( longitude >> 16 ) & 0xFF;
            AppData[12] = ( longitude >> 8 ) & 0xFF;
            AppData[13] = longitude & 0xFF;
            AppData[14] = ( altitudeGps >> 8 ) & 0xFF;
            AppData[15] = altitudeGps & 0xFF;
#elif defined( REGION_AS923 ) || defined( REGION_AU915 ) || defined( REGION_US915 ) || defined( REGION_US915_HYBRID )
            int16_t temperature = 0;
            int32_t latitude, longitude = 0;
            uint16_t altitudeGps = 0xFFFF;
            uint8_t batteryLevel = 0;

            temperature = ( int16_t )( MPL3115ReadTemperature( ) * 100 );       // in °C * 100

            batteryLevel = BoardGetBatteryLevel( );                             // 1 (very low) to 254 (fully charged)
            GpsGetLatestGpsPositionBinary( &latitude, &longitude );
            altitudeGps = GpsGetLatestGpsAltitude( );                           // in m

            AppData[0] = AppLedStateOn;
            AppData[1] = temperature;                                           // Signed degrees celsius in half degree units. So,  +/-63 C
            AppData[2] = batteryLevel;                                          // Per LoRaWAN spec; 0=Charging; 1...254 = level, 255 = N/A
            AppData[3] = ( latitude >> 16 ) & 0xFF;
            AppData[4] = ( latitude >> 8 ) & 0xFF;
            AppData[5] = latitude & 0xFF;
            AppData[6] = ( longitude >> 16 ) & 0xFF;
            AppData[7] = ( longitude >> 8 ) & 0xFF;
            AppData[8] = longitude & 0xFF;
            AppData[9] = ( altitudeGps >> 8 ) & 0xFF;
            AppData[10] = altitudeGps & 0xFF;
#endif
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
#endif
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
extern uint8_t JoinRequestTrials;
extern LoRaMacParams_t LoRaMacParams;
bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    AlternateDrParams_t altDr;

    altDr.NbTrials = persist.sesspar.JoinRequestTrials;//JoinRequestTrials;
    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = RegionAlternateDr( LORAMAC_REGION_CN470, &altDr );
        LoRaMacParams.ChannelsDatarate = mcpsReq.Req.Unconfirmed.Datarate;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = RegionAlternateDr( LORAMAC_REGION_CN470, &altDr );
            LoRaMacParams.ChannelsDatarate = mcpsReq.Req.Unconfirmed.Datarate;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 3;
            mcpsReq.Req.Confirmed.Datarate = RegionAlternateDr( LORAMAC_REGION_CN470, &altDr );
            LoRaMacParams.ChannelsDatarate = mcpsReq.Req.Confirmed.Datarate;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void funMcpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
    //NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
extern McpsConfirm_t McpsConfirm;
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }
    LMIC.txrxFlags = 0;
    if(McpsConfirm.McpsRequest == MCPS_CONFIRMED)
    {
        LMIC.txrxFlags |= (mcpsIndication->AckReceived ? TXRX_ACK : TXRX_NACK);
    }
    mcpsIndication->RxSlot?(LMIC.txrxFlags |= TXRX_DNW2):(LMIC.txrxFlags |= TXRX_DNW1);
    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
#if 0
    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }
#endif
    if( mcpsIndication->RxData == true )
    {
        LMIC.txrxFlags |= TXRX_PORT;
        LMIC.dataBeg = 1;
        LMIC.frame[LMIC.dataBeg - 1] = mcpsIndication->Port;
        LMIC.dataLen = mcpsIndication->BufferSize;
        memcpy(LMIC.frame + LMIC.dataBeg,mcpsIndication->Buffer,mcpsIndication->BufferSize);
#if 0
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
                //GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                    //GpsStop( );
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    //GpsStart( );
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < MIN( AppDataSize, LORAWAN_APP_DATA_MAX_SIZE ); i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = LORAWAN_APP_DATA_SIZE;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                        LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                        //GpsStart( );

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.NbTrials = 3;

                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        else if( mcpsIndication->BufferSize == 7 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW_1;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            mlmeReq.Req.TxCw.Frequency = ( uint32_t )( ( mcpsIndication->Buffer[3] << 16 ) | ( mcpsIndication->Buffer[4] << 8 ) | mcpsIndication->Buffer[5] ) * 100;
                            mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
#endif
    }
    else
    {
        LMIC.dataLen = 0;
        LMIC.txrxFlags |= TXRX_NOPORT;
    }
    if(isAlarmDuty() == true)
    {
        onEvent(EV_RXCOMPLETE);
    }
    else
    {
        onEvent(EV_TXCOMPLETE);
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SLEEP;
                onEvent(EV_JOINED);
                Board_LED_Set(0,0);
            }
            else
            {
                // Join was not successful. Try to join again
                DeviceState = DEVICE_STATE_JOIN;
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

uint8_t BoardGetBatteryLevel(void)
{
    return 0;
}

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();
}

static uint32_t uartflashtimer = 0;
void enableUart(void)
{
    usart_init();
    //Chip_UART_SendRB(LPC_USART0, &txring, "PIN4 WAKEUP\r\n", 13);
    uartflashtimer = TimerGetCurrentTime();// this is need , if not the mcu will go to powerdown again
}
/**
 * Main application entry point.
 */
int main( void )
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;
    int bytes;
    uint8_t byte;
    
    //BoardInitMcu( );
    //BoardInitPeriph( );
    prvSetupHardware();
    modem_init();
    /* Enable SysTick Timer */
    SystemCoreClockUpdate();
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SYS);
    SysTick_Config(SystemCoreClock / 1000);
    //Chip_UART_SendRB(LPC_USART0, &txring, "PIN4 WAKEUP\r\n", 13);
    DeviceState = DEVICE_STATE_INIT;
    modem_wkt_init();
    uartflashtimer = TimerGetCurrentTime();
    //LoRaMacState = 0;
    //modem_wwdt_init();
    while( 1 )
    {
        //Chip_WWDT_Feed(LPC_WWDT);
        bytes = Chip_UART_ReadRB(LPC_USART0, &rxring, &byte, 1);
        if(bytes > 0)
        {
            uartflashtimer = TimerGetCurrentTime();
            //Chip_UART_SendRB(LPC_USART0, &txring, &byte, 1);
            frame_rx(byte);
        }

        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                LoRaMacPrimitives.MacMcpsConfirm = funMcpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
#if defined( REGION_AS923 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AS923 );
#elif defined( REGION_AU915 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AU915 );
#elif defined( REGION_CN470 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470 );
#elif defined( REGION_CN779 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN779 );
#elif defined( REGION_EU433 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU433 );
#elif defined( REGION_EU868 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868 );
#elif defined( REGION_IN865 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_IN865 );
#elif defined( REGION_KR920 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KR920 );
#elif defined( REGION_US915 )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915 );
#elif defined( REGION_US915_HYBRID )
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915_HYBRID );
#else
    #error "Please define a region in the compiler options."
#endif
                //TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 )
                LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )
                LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
                LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
                LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
                LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
                LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
                LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
                LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );

                mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
                mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_RX2_CHANNEL;
                mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
                LoRaMacMibSetRequestConfirm( &mibReq );
#endif

#endif
                DeviceState = DEVICE_STATE_SLEEP;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
                if( persist.flags & FLAGS_JOINPAR )
                {
                    MlmeReq_t mlmeReq;

                    // Initialize LoRaMac device unique ID
                    //BoardGetUniqueId( DevEui );

                    mlmeReq.Type = MLME_JOIN;

                    mlmeReq.Req.Join.DevEui = DevEui;
                    mlmeReq.Req.Join.AppEui = AppEui;
                    mlmeReq.Req.Join.AppKey = AppKey;
                    mlmeReq.Req.Join.NbTrials = 3;

                    if( NextTx == true )
                    {
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    // Choose a random device address if not already defined in Commissioning.h
                    /*if( DevAddr == 0 )
                    {
                        // Random seed initialization
                        srand1(0x55aa);//( BoardGetRandomSeed( ) );

                        // Choose a random device address
                        DevAddr = randr( 0, 0x01FFFFFF );
                    }*/

                    mibReq.Type = MIB_NET_ID;
                    mibReq.Param.NetID = persist.sesspar.netid;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_DEV_ADDR;
                    mibReq.Param.DevAddr = persist.sesspar.devaddr;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_NWK_SKEY;
                    mibReq.Param.NwkSKey = persist.sesspar.nwkkey;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_APP_SKEY;
                    mibReq.Param.AppSKey = persist.sesspar.artkey;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_NETWORK_JOINED;
                    mibReq.Param.IsNetworkJoined = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    DeviceState = DEVICE_STATE_SLEEP;
                }
                break;
            }
#if 0
            case DEVICE_STATE_SEND:
            {
                if( NextTx == true )
                {
                    uint8_t txcycledata[] = {0x00 ,0x01 ,0x02 ,0x0A ,0x0B};
                    PrepareTxFrame( AppPort ,txcycledata,5);

                    NextTx = SendFrame( );
                }
#if 0
                if( ComplianceTest.Running == true )
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = 5000; // 5000 ms
                }
                else
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                }
#endif
                // Schedule next packet transmission
                TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                //TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                //TimerStart( &TxNextPacketTimer );
                break;
            }
#endif
            case DEVICE_STATE_SLEEP:
            {
                //funWkt[wktType]();
                if(LoRaMacState == 0)
                {
                    if((TimerGetElapsedTime(uartflashtimer) > 10) && (RingBuffer_IsEmpty(&txring)))
                    {
#if 1
                        if(( persist.flags & FLAGS_JOINPAR ) && (atcmdtoactivaty == true))
                        {
                            DeviceState = DEVICE_STATE_JOIN;
                            continue;
                        }
                        extern uint32_t UpLinkCounter;
                        if((UpLinkCounter == 0) && ( persist.flags & FLAGS_SESSPAR ))
                        {
                            funWktAlarm();
                            break;
                        }
                        NVIC_DisableIRQ(PININT0_IRQn);
                        NVIC_DisableIRQ(PININT1_IRQn);
                        Radio.Sleep( );
                        if( persist.flags & FLAGS_JOINPAR )
                        {
                            Chip_PMU_ClearPowerDownControl(LPC_PMU, PMU_DPDCTRL_LPOSCDPDEN);
                        }
                        else
                        {
                            Chip_PMU_SetPowerDownControl(LPC_PMU, PMU_DPDCTRL_LPOSCDPDEN);
                        }
                        //enablePio4IntToWakeup();
                        /*Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO23,PIN_MODE_INACTIVE);
                        Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO14,PIN_MODE_INACTIVE);
                        Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO25,PIN_MODE_INACTIVE);
                        Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO6,PIN_MODE_INACTIVE);
                        Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,23,FALSE); // NRESET
                        Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,14,FALSE); // SSEL
                        Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,25,FALSE); // SCK
                        Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,6,FALSE); // MOSI*/
                        WakeupTest(WKT_CLKSRC_10KHZ,persist.sesspar.alarm,PMU_MCU_DEEP_PWRDOWN);
                        //WakeupTest(WKT_CLKSRC_10KHZ,persist.sesspar.alarm,PMU_MCU_SLEEP);
                        /*Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,23,TRUE); // NRESET
                        Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,14,TRUE); // SSEL
                        Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,25,TRUE); // SCK
                        Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,6,TRUE); // MOSI*/
                        NVIC_EnableIRQ(PININT0_IRQn);
                        NVIC_EnableIRQ(PININT1_IRQn);
                        //enableUart();
                        //Chip_UART_SendRB(LPC_USART0, &txring, "PIN4 WAKEUP\r\n", 13);
#endif
                    }
                    else
                    {
                        WakeupTest(WKT_CLKSRC_10KHZ,0xfffffffe,PMU_MCU_SLEEP);
                    }
                }
                else
                {
                    WakeupTest(WKT_CLKSRC_10KHZ,0xfffffffe,PMU_MCU_SLEEP);
                }
                // Wake up through events
                //TimerLowPowerHandler( );
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
        /*
        if( GpsGetPpsDetectedState( ) == true )
        {
            // Switch LED 4 ON
            //GpioWrite( &Led4, 0 );
            TimerStart( &Led4Timer );
        }*/
    }
}
