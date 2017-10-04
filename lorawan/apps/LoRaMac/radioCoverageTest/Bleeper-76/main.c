/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Radio coverage tester implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <string.h>
#include <math.h>
#include "board.h"

#include "LoRaMac.h"

/*!
 * Number of packets sent by channel
 */
#define NB_PACKETS                                  15

/*!
 * Enables/Disables the downlink test
 */
#define DOWNLINK_TEST_ON                            0

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     0

/*!
 * Indicates if the end-device is to be connected to a private or public network
 */
#define LORAWAN_PUBLIC_NETWORK                      true

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000000  // 10 [s] value in us

/*!
 * Mote device IEEE EUI
 *
 * \remark must be written as a little endian value (reverse order of normal reading)
 *
 * \remark In this application the value is automatically generated by calling
 *         BoardGetUniqueId function
 */
#define LORAWAN_DEVICE_EUI                          { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

/*!
 * Application IEEE EUI
 *
 * \remark must be written as a little endian value (reverse order of normal reading)
 */
#define LORAWAN_APPLICATION_EUI                     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

/*!
 * AES encryption/decryption cipher application key
 */
#define LORAWAN_APPLICATION_KEY                     { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00 }

#else

/*!
 * Current network ID
 */
#define LORAWAN_NETWORK_ID                          ( uint32_t )0

/*!
 * Device address on the network
 *
 * \remark must be written as a big endian value (normal reading order)
 *
 * \remark In this application the value is automatically generated using
 *         a pseudo random generator seeded with a value derived from
 *         BoardUniqueId value
 */
#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0x00000000

/*!
 * AES encryption/decryption cipher network session key
 */
#define LORAWAN_NWKSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }

/*!
 * AES encryption/decryption cipher application session key
 */
#define LORAWAN_APPSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }

#endif

/*!
 * Defines the application data transmission duty cycle
 */
#define APP_TX_DUTYCYCLE                            5000000  // 5 [s] value in us
#define APP_TX_DUTYCYCLE_RND                        1000000  // 1 [s] value in us

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptative Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_SIZE                       6

#if( OVER_THE_AIR_ACTIVATION != 0 )

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#else

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr;

#endif

/*!
 * Indicates if the MAC layer has already joined a network.
 */
static bool IsNetworkJoined = false;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

static TimerHandle_t TxNextPacketTimer;

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * Defines the join request timer
 */
static TimerHandle_t JoinReqTimer;

#endif

/*!
 * Indicates if a new packet can be sent
 */
static bool TxNextPacket = true;
static bool ScheduleNextTx = false;
static bool DownlinkStatusUpdate = false;

static bool AppLedStateOn = false;

static LoRaMacCallbacks_t LoRaMacCallbacks;
static LoRaMacEventInfo_t LoRaMacLastRxEvent;

static TimerHandle_t Led1Timer;
volatile bool Led1StateChanged = false;
static TimerHandle_t Led2Timer;
volatile bool Led2StateChanged = false;

volatile bool Led3StateChanged = false;

static TimerHandle_t StopTimer;

static uint8_t ChannelNb;
static uint16_t DownLinkCounter = 0;

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * \brief Function executed on JoinReq Timeout event
 */
static void OnJoinReqTimerEvent( void )
{
    xTimerStop( &JoinReqTimer );
    TxNextPacket = true;
}

#endif

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    xTimerStop( &TxNextPacketTimer );
    TxNextPacket = true;
}

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    xTimerStop( &Led1Timer );
    Led1StateChanged = true;
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    xTimerStop( &Led2Timer );
    Led2StateChanged = true;
}

/*!
 * \brief Function executed on Stop Timeout event
 */
static void OnStopTimerEvent( void )
{
    xTimerStart( &StopTimer );
}

/*!
 * \brief Function to be executed on MAC layer event
 */
static void OnMacEvent( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    int32_t snr = 0;

    LoRaMacLastRxEvent = *info;
    
    if( flags->Bits.JoinAccept == 1 )
    {
#if( OVER_THE_AIR_ACTIVATION != 0 )
    #if ( DOWNLINK_TEST_ON == 0 )
        // Once joined disable reception windows opening
        LoRaMacTestRxWindowsOn( false );
    #endif
        xTimerStop( &JoinReqTimer );
#endif
        IsNetworkJoined = true;
    }
    else
    {
        if( flags->Bits.Tx == 1 )
        {
        }

        if( flags->Bits.Rx == 1 )
        {
            if( (flags->Bits.RxData == true ) && ( ( info->RxPort == 1 ) || ( info->RxPort == 2 ) ) )
            {
                AppLedStateOn = info->RxBuffer[0];
                Led3StateChanged = true;
            }

            if( info->RxSnr & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                snr = ( ( ~info->RxSnr + 1 ) & 0xFF ) >> 2;
                snr = -snr;
            }
            else
            {
                // Divide by 4
                snr = ( info->RxSnr & 0xFF ) >> 2;
            }
            DownLinkCounter++;

            DownlinkStatusUpdate = true;
            xTimerStart( &Led2Timer );
        }
    }
    // Schedule a new transmission
    ScheduleNextTx = true;
}

/**
 * Main application entry point.
 */
int main( void )
{
#if defined( USE_BAND_433 )
    const ChannelParams_t channels[] =
    { 
        { 433175000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 433375000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 433575000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 433775000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 433975000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 434175000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 434300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 434500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
     };
    const uint8_t  channelsDatarate[] = { DR_5, DR_2, DR_0 };
#elif defined( USE_BAND_780 )
    const ChannelParams_t channels[] =
    { 
        { 779500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 779700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 779900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 780100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 780300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 780500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 780700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 780900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
     };
    const uint8_t  channelsDatarate[] = { DR_5, DR_2, DR_0 };
#elif defined( USE_BAND_868 )
    const ChannelParams_t channels[] = 
    { 
        { 868100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 868300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 868500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 867100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 867300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 867500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 867700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
        { 867900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 },
     };
    const uint8_t  channelsDatarate[] = { DR_5, DR_2, DR_0 };
#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    const ChannelParams_t channels[] =
    { 
        { 902300000, { ( ( DR_3 << 4 ) | DR_0 ) }, 0 },
        { 902500000, { ( ( DR_3 << 4 ) | DR_0 ) }, 0 },
        { 902700000, { ( ( DR_3 << 4 ) | DR_0 ) }, 0 },
        { 902900000, { ( ( DR_3 << 4 ) | DR_0 ) }, 0 },
        { 903100000, { ( ( DR_3 << 4 ) | DR_0 ) }, 0 },
        { 903500000, { ( ( DR_3 << 4 ) | DR_0 ) }, 0 },
        { 903700000, { ( ( DR_3 << 4 ) | DR_0 ) }, 0 },
        { 903900000, { ( ( DR_3 << 4 ) | DR_0 ) }, 0 },
     };
    const uint8_t  channelsDatarate[] = { DR_3, DR_2, DR_0 };
#else
    #error "Please define a frequency band in the compiler options."
#endif

    ChannelNb = ( sizeof( channels ) / sizeof( ChannelParams_t ) );

#if( OVER_THE_AIR_ACTIVATION != 0 )
    uint8_t sendFrameStatus = 0;
#endif
    uint8_t tstState = 0;
    int16_t pktCnt = NB_PACKETS;
    LoRaMacHeader_t macHdr;
    LoRaMacFrameCtrl_t fCtrl;
    uint8_t channelsIndex = 0;
    uint8_t datarateIndex = 0;
    
    BoardInitMcu( );
    BoardInitPeriph( );

    LoRaMacCallbacks.MacEvent = OnMacEvent;
    LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
    LoRaMacInit( &LoRaMacCallbacks );

    IsNetworkJoined = false;

#if( OVER_THE_AIR_ACTIVATION == 0 )
    // Random seed initialization
    srand1( BoardGetRandomSeed( ) );
    // Choose a random device address based on Board unique ID
    // NwkAddr rand [0, 33554431]
    DevAddr = randr( 0, 0x01FFFFFF );

    LoRaMacInitNwkIds( LORAWAN_NETWORK_ID, DevAddr, NwkSKey, AppSKey );
    IsNetworkJoined = true;
#else
    // Initialize LoRaMac device unique ID
    BoardGetUniqueId( DevEui );

    // Sends a JoinReq Command every OVER_THE_AIR_ACTIVATION_DUTYCYCLE
    // seconds until the network is joined
    xTimerCreate( &JoinReqTimer, OnJoinReqTimerEvent ); 
    xTimerChangePeriod( &JoinReqTimer, OVER_THE_AIR_ACTIVATION_DUTYCYCLE );
#endif

    TxNextPacket = true;
    xTimerCreate( &TxNextPacketTimer, OnTxNextPacketTimerEvent );
    
    xTimerCreate( &Led1Timer, OnLed1TimerEvent );
    xTimerChangePeriod( &Led1Timer, 25000 );

    xTimerCreate( &Led2Timer, OnLed2TimerEvent );
    xTimerChangePeriod( &Led2Timer, 25000 );
    
    // Low power timer to be run when tests are finished.
    xTimerCreate( &StopTimer, OnStopTimerEvent );
    xTimerChangePeriod( &StopTimer, 3.6e9 ); // wakes up the microcontroller every hour

    DownLinkCounter = 0;

    // Initialize MAC frame
    macHdr.Value = 0;
#if ( DOWNLINK_TEST_ON == 1 )
    macHdr.Bits.MType = FRAME_TYPE_DATA_CONFIRMED_UP;
#else
    // Disable reception windows opening
    LoRaMacTestRxWindowsOn( false );
    macHdr.Bits.MType = FRAME_TYPE_DATA_UNCONFIRMED_UP;
#endif
    fCtrl.Value = 0;
    fCtrl.Bits.FOptsLen      = 0;
    fCtrl.Bits.FPending      = 0;
    fCtrl.Bits.Ack           = false;
    fCtrl.Bits.AdrAckReq     = false;
    fCtrl.Bits.Adr           = false;

    LoRaMacSetChannelsTxPower( TX_POWER_14_DBM );
    
    LoRaMacTestSetDutyCycleOn( false );
    
    while( 1 )
    {
        while( IsNetworkJoined == false )
        {
#if( OVER_THE_AIR_ACTIVATION != 0 )
            if( TxNextPacket == true )
            {
                TxNextPacket = false;
                
                sendFrameStatus = LoRaMacJoinReq( DevEui, AppEui, AppKey );
                switch( sendFrameStatus )
                {
                case 1: // BUSY
                    break;
                case 0: // OK
                case 2: // NO_NETWORK_JOINED
                case 3: // LENGTH_PORT_ERROR
                case 4: // MAC_CMD_ERROR
                case 6: // DEVICE_OFF
                default:
                    // Relaunch timer for next trial
                    xTimerStart( &JoinReqTimer );
                    break;
                }
            }
            TimerLowPowerHandler( );
#endif
        }
        for( datarateIndex = 0; datarateIndex < 3; datarateIndex++ )
        {
            pktCnt = NB_PACKETS * ChannelNb;
            while( pktCnt > 0 )
            {
                if( Led1StateChanged == true )
                {
                    Led1StateChanged = false;
                    // Switch LED 1 OFF
                    GpioWrite( &Led1, 1 );
                }
                if( Led2StateChanged == true )
                {
                    Led2StateChanged = false;
                    // Switch LED 2 OFF
                    GpioWrite( &Led2, 1 );
                }
                if( Led3StateChanged == true )
                {
                    Led3StateChanged = false;
                    GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
                }
                if( DownlinkStatusUpdate == true )
                {
                    DownlinkStatusUpdate = false;
                    // Switch LED 2 ON for each received downlink
                    GpioWrite( &Led2, 0 );
                }
                switch( tstState )
                {
                    case 0: // Init
                        AppData[0] = SelectorGetValue( );
                        AppData[1] = LoRaMacLastRxEvent.RxRssi >> 8;
                        AppData[2] = LoRaMacLastRxEvent.RxRssi & 0xFF;
                        AppData[3] = LoRaMacLastRxEvent.RxSnr;
                        AppData[4] = ( ( DownLinkCounter >> 8 ) & 0xFF );
                        AppData[5] = ( ( DownLinkCounter & 0xFF ) );
                        
                        LoRaMacSetChannelsDatarate( channelsDatarate[datarateIndex] );
                        LoRaMacSendOnChannel( channels[channelsIndex], &macHdr, &fCtrl, NULL, 15, AppData, AppDataSize );
                        
                        // Switch LED 1 ON
                        GpioWrite( &Led1, 0 );
                        xTimerStart( &Led1Timer );

                        channelsIndex = ( channelsIndex + 1 ) % ChannelNb;

                        tstState = 1;
                        break;
                    case 1: // Wait for end of transmission
                        if( ScheduleNextTx == true )
                        {
                            ScheduleNextTx = false;
                            pktCnt--;
                            // Schedule next packet transmission after 100 ms
                            xTimerChangePeriod( &TxNextPacketTimer, 100000 );
                            xTimerStart( &TxNextPacketTimer );
                            tstState = 2;
                        }
                        break;
                    case 2: // Wait for next packet timer to expire
                        if( TxNextPacket == true )
                        {
                            TxNextPacket = false;
                            tstState = 0;
                        }
                        break;
                }
                
                TimerLowPowerHandler( );
            }
        }

        // Switch LED OFF
        GpioWrite( &Led1, 1 );
        GpioWrite( &Led2, 1 );
        GpioWrite( &Led3, 1 );
    
        xTimerStart( &StopTimer );
        while( 1 ) // Reset device to restart
        {
            TimerLowPowerHandler( );
        }
    }
}
