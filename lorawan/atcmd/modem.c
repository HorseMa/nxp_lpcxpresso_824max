/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "modem.h"
#include "board.h"
#include "LoRaMac.h"
#include "utilities.h"


//////////////////////////////////////////////////
// CONFIGURATION (WILL BE PATCHED)
//////////////////////////////////////////////////
#if 0
static const union {
    joinparam_t param;
    uint8_t pattern[sizeof(joinparam_t)];
} joincfg = {
    .pattern = { PATTERN_JOINCFG_STR }
};

static const union {
    sessparam_t param;
    uint8_t pattern[sizeof(sessparam_t)];
} sesscfg = {
    .pattern = { PATTERN_SESSCFG_STR }
};
#endif
// COMMAND TO SET DEVEUI=FF-FF-FF-FF-FF-FF-FF-00, APPEUI=DE-DE-AA-AA-00-00-00-1A (PORT=25501):
//   ATJ=FFFFFFFFFFFFFF00,DEDEAAAA0000001A,AA5555555555555555AAAAAAAAAAAAAA
//   OK

// Event types for event callback
enum _ev_t { EV_SCAN_TIMEOUT=1, EV_BEACON_FOUND,
             EV_BEACON_MISSED, EV_BEACON_TRACKED, EV_JOINING,
             EV_JOINED, EV_RFU1, EV_JOIN_FAILED, EV_REJOIN_FAILED,
             EV_TXCOMPLETE, EV_LOST_TSYNC, EV_RESET,
             EV_RXCOMPLETE, EV_LINK_DEAD, EV_LINK_ALIVE, EV_SCAN_FOUND,
             EV_TXSTART };
typedef enum _ev_t ev_t;

static const char* evnames[] = {
    [EV_SCAN_TIMEOUT]   = "SCAN_TIMEOUT",
    [EV_BEACON_FOUND]   = "BEACON_FOUND",
    [EV_BEACON_MISSED]  = "BEACON_MISSED",
    [EV_BEACON_TRACKED] = "BEACON_TRACKED",
    [EV_JOINING]        = "JOINING",
    [EV_JOINED]         = "JOINED",
    [EV_JOIN_FAILED]    = "JOIN_FAILED",
    [EV_REJOIN_FAILED]  = "REJOIN_FAILED",
    [EV_TXCOMPLETE]     = "TXCOMPLETE",
    [EV_LOST_TSYNC]     = "LOST_TSYNC",
    [EV_RESET]          = "RESET",
    [EV_RXCOMPLETE]     = "RXCOMPLETE",
    [EV_LINK_DEAD]      = "LINK_DEAD",
    [EV_LINK_ALIVE]     = "LINK_ALIVE",
};

static const char* evnameLoRaMacEventInfoStatus[] = {
    /*!
     * Service performed successfully
     */
    [LORAMAC_EVENT_INFO_STATUS_OK]                              = "LORAMAC_EVENT_INFO_STATUS_OK",
    /*!
     * An error occurred during the execution of the service
     */
    [LORAMAC_EVENT_INFO_STATUS_ERROR]                           = "LORAMAC_EVENT_INFO_STATUS_ERROR",
    /*!
     * A Tx timeout occurred
     */
    [LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT]                      = "LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT",
    /*!
     * An Rx timeout occurred on receive window 1
     */
    [LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT]                     = "LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT",
    /*!
     * An Rx timeout occurred on receive window 2
     */
    [LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT]                     = "LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT",
    /*!
     * An Rx error occurred on receive window 1
     */
    [LORAMAC_EVENT_INFO_STATUS_RX1_ERROR]                       = "LORAMAC_EVENT_INFO_STATUS_RX1_ERROR",
    /*!
     * An Rx error occurred on receive window 2
     */
    [LORAMAC_EVENT_INFO_STATUS_RX2_ERROR]                       = "LORAMAC_EVENT_INFO_STATUS_RX2_ERROR",
    /*!
     * An error occurred in the join procedure
     */
    [LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL]                       = "LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL",
    /*!
     * A frame with an invalid downlink counter was received. The
     * downlink counter of the frame was equal to the local copy
     * of the downlink counter of the node.
     */
    [LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED]               = "LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED",
    /*!
     * The MAC could not retransmit a frame since the MAC decreased the datarate. The
     * payload size is not applicable for the datarate.
     */
    [LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR]        = "LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR",
    /*!
     * The node has lost MAX_FCNT_GAP or more frames.
     */
    [LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS]   = "LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS",
    /*!
     * An address error occurred
     */
    [LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL]                    = "LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL",
    /*!
     * message integrity check failure
     */
    [LORAMAC_EVENT_INFO_STATUS_MIC_FAIL]                        = "LORAMAC_EVENT_INFO_STATUS_MIC_FAIL",
};

static const char* nameLoRaMacStatus[] = {
    /*!
     * Service started successfully
     */
    [LORAMAC_STATUS_OK]                     = "LORAMAC_STATUS_OK",
    /*!
     * Service not started - LoRaMAC is busy
     */
    [LORAMAC_STATUS_BUSY]                   = "LORAMAC_STATUS_BUSY",
    /*!
     * Service unknown
     */
    [LORAMAC_STATUS_SERVICE_UNKNOWN]        = "LORAMAC_STATUS_SERVICE_UNKNOWN",
    /*!
     * Service not started - invalid parameter
     */
    [LORAMAC_STATUS_PARAMETER_INVALID]      = "LORAMAC_STATUS_PARAMETER_INVALID",
    /*!
     * Service not started - invalid frequency
     */
    [LORAMAC_STATUS_FREQUENCY_INVALID]      = "LORAMAC_STATUS_FREQUENCY_INVALID",
    /*!
     * Service not started - invalid datarate
     */
    [LORAMAC_STATUS_DATARATE_INVALID]       = "LORAMAC_STATUS_DATARATE_INVALID",
    /*!
     * Service not started - invalid frequency and datarate
     */
    [LORAMAC_STATUS_FREQ_AND_DR_INVALID]    = "LORAMAC_STATUS_FREQ_AND_DR_INVALID",
    /*!
     * Service not started - the device is not in a LoRaWAN
     */
    [LORAMAC_STATUS_NO_NETWORK_JOINED]      = "LORAMAC_STATUS_NO_NETWORK_JOINED",
    /*!
     * Service not started - payload length error
     */
    [LORAMAC_STATUS_LENGTH_ERROR]           = "LORAMAC_STATUS_LENGTH_ERROR",
    /*!
     * Service not started - the device is switched off
     */
    [LORAMAC_STATUS_DEVICE_OFF]             = "LORAMAC_STATUS_DEVICE_OFF",
    /*!
     * Service not started - the specified region is not supported
     * or not activated with preprocessor definitions.
     */
    [LORAMAC_STATUS_REGION_NOT_SUPPORTED]   = "LORAMAC_STATUS_REGION_NOT_SUPPORTED",
};

static const char* nameMib[] = {
    /*!
     * LoRaWAN device class
     *
     * LoRaWAN Specification V1.0.2
     */
    [MIB_DEVICE_CLASS] = "MIB_DEVICE_CLASS",
    /*!
     * LoRaWAN Network joined attribute
     *
     * LoRaWAN Specification V1.0.2
     */
    [MIB_NETWORK_JOINED] = "MIB_NETWORK_JOINED",
    /*!
     * Adaptive data rate
     *
     * LoRaWAN Specification V1.0.2, chapter 4.3.1.1
     *
     * [true: ADR enabled, false: ADR disabled]
     */
    [MIB_ADR] = "MIB_ADR",
    /*!
     * Network identifier
     *
     * LoRaWAN Specification V1.0.2, chapter 6.1.1
     */
    [MIB_NET_ID] = "MIB_NET_ID",
    /*!
     * End-device address
     *
     * LoRaWAN Specification V1.0.2, chapter 6.1.1
     */
    [MIB_DEV_ADDR] = "MIB_DEV_ADDR",
    /*!
     * Network session key
     *
     * LoRaWAN Specification V1.0.2, chapter 6.1.3
     */
    [MIB_NWK_SKEY] = "MIB_NWK_SKEY",
    /*!
     * Application session key
     *
     * LoRaWAN Specification V1.0.2, chapter 6.1.4
     */
    [MIB_APP_SKEY] = "MIB_APP_SKEY",
    /*!
     * Set the network type to public or private
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     *
     * [true: public network, false: private network]
     */
    [MIB_PUBLIC_NETWORK] = "MIB_PUBLIC_NETWORK",
    /*!
     * Support the operation with repeaters
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     *
     * [true: repeater support enabled, false: repeater support disabled]
     */
    [MIB_REPEATER_SUPPORT] = "MIB_REPEATER_SUPPORT",
    /*!
     * Communication channels. A get request will return a
     * pointer which references the first entry of the channel list. The
     * list is of size LORA_MAX_NB_CHANNELS
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     */
    [MIB_CHANNELS] = "MIB_CHANNELS",
    /*!
     * Set receive window 2 channel
     *
     * LoRaWAN Specification V1.0.2, chapter 3.3.1
     */
    [MIB_RX2_CHANNEL] = "MIB_RX2_CHANNEL",
    /*!
     * Set receive window 2 channel
     *
     * LoRaWAN Specification V1.0.2, chapter 3.3.2
     */
    [MIB_RX2_DEFAULT_CHANNEL] = "MIB_RX2_DEFAULT_CHANNEL",
    /*!
     * LoRaWAN channels mask
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     */
    [MIB_CHANNELS_MASK] = "MIB_CHANNELS_MASK",
    /*!
     * LoRaWAN default channels mask
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     */
    [MIB_CHANNELS_DEFAULT_MASK] = "MIB_CHANNELS_DEFAULT_MASK",
    /*!
     * Set the number of repetitions on a channel
     *
     * LoRaWAN Specification V1.0.2, chapter 5.2
     */
    [MIB_CHANNELS_NB_REP] = "MIB_CHANNELS_NB_REP",
    /*!
     * Maximum receive window duration in [ms]
     *
     * LoRaWAN Specification V1.0.2, chapter 3.3.3
     */
    [MIB_MAX_RX_WINDOW_DURATION] = "MIB_MAX_RX_WINDOW_DURATION",
    /*!
     * Receive delay 1 in [ms]
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     */
    [MIB_RECEIVE_DELAY_1] = "MIB_RECEIVE_DELAY_1",
    /*!
     * Receive delay 2 in [ms]
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     */
    [MIB_RECEIVE_DELAY_2] = "MIB_RECEIVE_DELAY_2",
    /*!
     * Join accept delay 1 in [ms]
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     */
    [MIB_JOIN_ACCEPT_DELAY_1] = "MIB_JOIN_ACCEPT_DELAY_1",
    /*!
     * Join accept delay 2 in [ms]
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     */
    [MIB_JOIN_ACCEPT_DELAY_2] = "MIB_JOIN_ACCEPT_DELAY_2",
    /*!
     * Default Data rate of a channel
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     *
     * The allowed ranges are region specific. Please refer to \ref DR_0 to \ref DR_15 for details.
     */
    [MIB_CHANNELS_DEFAULT_DATARATE] = "MIB_CHANNELS_DEFAULT_DATARATE",
    /*!
     * Data rate of a channel
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     *
     * The allowed ranges are region specific. Please refer to \ref DR_0 to \ref DR_15 for details.
     */
    [MIB_CHANNELS_DATARATE] = "MIB_CHANNELS_DATARATE",
    /*!
     * Transmission power of a channel
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     *
     * The allowed ranges are region specific. Please refer to \ref TX_POWER_0 to \ref TX_POWER_15 for details.
     */
    [MIB_CHANNELS_TX_POWER] = "MIB_CHANNELS_TX_POWER",
    /*!
     * Transmission power of a channel
     *
     * LoRaWAN Regional Parameters V1.0.2rB
     *
     * The allowed ranges are region specific. Please refer to \ref TX_POWER_0 to \ref TX_POWER_15 for details.
     */
    [MIB_CHANNELS_DEFAULT_TX_POWER] = "MIB_CHANNELS_DEFAULT_TX_POWER",
    /*!
     * LoRaWAN Up-link counter
     *
     * LoRaWAN Specification V1.0.2, chapter 4.3.1.5
     */
    [MIB_UPLINK_COUNTER] = "MIB_UPLINK_COUNTER",
    /*!
     * LoRaWAN Down-link counter
     *
     * LoRaWAN Specification V1.0.2, chapter 4.3.1.5
     */
    [MIB_DOWNLINK_COUNTER] = "MIB_DOWNLINK_COUNTER",
    /*!
     * Multicast channels. A get request will return a pointer to the first
     * entry of the multicast channel linked list. If the pointer is equal to
     * NULL, the list is empty.
     */
    [MIB_MULTICAST_CHANNEL] = "MIB_MULTICAST_CHANNEL",
    /*!
     * System overall timing error in milliseconds.
     * [-SystemMaxRxError : +SystemMaxRxError]
     * Default: +/-10 ms
     */
    [MIB_SYSTEM_MAX_RX_ERROR] = "MIB_SYSTEM_MAX_RX_ERROR",
    /*!
     * Minimum required number of symbols to detect an Rx frame
     * Default: 6 symbols
     */
    [MIB_MIN_RX_SYMBOLS] = "MIB_MIN_RX_SYMBOLS",
    /*!
     * Antenna gain of the node. Default value is region specific.
     * The antenna gain is used to calculate the TX power of the node.
     * The formula is:
     * radioTxPower = ( int8_t )floor( maxEirp - antennaGain )
     */
    [MIB_ANTENNA_GAIN] = "MIB_ANTENNA_GAIN",
};

static const char* nameMlme[] = {
    /*!
     * Initiates the Over-the-Air activation
     *
     * LoRaWAN Specification V1.0.2, chapter 6.2
     */
    [MLME_JOIN] = "MLME_JOIN",
    /*!
     * LinkCheckReq - Connectivity validation
     *
     * LoRaWAN Specification V1.0.2, chapter 5, table 4
     */
    [MLME_LINK_CHECK] = "MLME_LINK_CHECK",
    /*!
     * Sets Tx continuous wave mode
     *
     * LoRaWAN end-device certification
     */
    [MLME_TXCW] = "MLME_TXCW",
    /*!
     * Sets Tx continuous wave mode (new LoRa-Alliance CC definition)
     *
     * LoRaWAN end-device certification
     */
    [MLME_TXCW_1] = "MLME_TXCW_1",
};

extern uint8_t DevEui[];
extern uint8_t AppEui[];
extern uint8_t AppKey[];
extern RINGBUFF_T txring, rxring;

#if 0
// return mask of all events matching given string prefix
static uint32_t evmatch (uint8_t* name, uint8_t len) {
    uint32_t mask = 0;
    for(uint8_t e=0; e < sizeof(evnames)/sizeof(evnames[0]); e++) {
    if(evnames[e]) {
        uint8_t l;
        for(l=0; l<len && toupper(name[l]) == evnames[e][l]; l++); // compare
        if(l == len) mask |= (1 << e);
    }
    }
    return mask;
}


// transient state
static struct {
    uint8_t cmdbuf[255];
    uint16_t rsplen;
    uint8_t txpending;
    //osjob_t alarmjob;
    //osjob_t blinkjob;
    //osjob_t ledjob;
} MODEM;

// provide device EUI (LSBF)
void os_getDevEui (uint8_t* buf) {
    memcpy(buf, DevEui, 8);
}

// provide device key
void os_getDevKey (uint8_t* buf) {
    memcpy(buf, AppKey, 16);
}

// provide application EUI (LSBF)
void os_getArtEui (uint8_t* buf) {
    memcpy(buf, AppEui, 8);
}
#endif
extern FRAME rxframe;
extern FRAME txframe;

// return mask of all events matching given string prefix
static uint32_t evmatch (uint8_t* name, uint8_t len) {
    uint32_t mask = 0;
    for(uint8_t e=0; e < sizeof(evnames)/sizeof(evnames[0]); e++) {
    if(evnames[e]) {
        uint8_t l;
        for(l=0; l<len && toupper(name[l]) == evnames[e][l]; l++); // compare
        if(l == len) mask |= (1 << e);
    }
    }
    return mask;
}


// transient state
static struct {
    uint8_t cmdbuf[300];
    uint16_t rsplen;
    uint8_t txpending;
    //osjob_t alarmjob;
    //osjob_t blinkjob;
    //osjob_t ledjob;
} MODEM;

// provide device EUI (LSBF)
void os_getDevEui (uint8_t* buf) {
    memcpy(buf, PERSIST->joinpar.deveui, 8);
}

// provide device key
void os_getDevKey (uint8_t* buf) {
    memcpy(buf, PERSIST->joinpar.devkey, 16);
}

// provide application EUI (LSBF)
void os_getArtEui (uint8_t* buf) {
    memcpy(buf, PERSIST->joinpar.appeui, 8);
}

extern FRAME rxframe;
extern FRAME txframe;


// blink session led
static void blinkfunc () {
    static uint8_t ledstate;
    // toggle LED
    ledstate = !ledstate;
    //leds_set(LED_SESSION, ledstate);
    // reschedule blink job
    //os_setTimedCallback(j, os_getTime()+ms2osticks(100), blinkfunc);
}

// switch on power led again
static void ledfunc () {
    leds_set(LED_POWER, 1);
}

// start transmission of prepared response or queued event message
static void modem_starttx () {
    hal_disableIRQs();
    if( !MODEM.txpending ) {
    // check if we have something to send
    if(queue_shift(&txframe)) { // send next queued event
        usart_starttx();
        MODEM.txpending = 1;
    } else if(MODEM.rsplen) { // send response
        frame_init(&txframe, MODEM.cmdbuf, MODEM.rsplen);
        MODEM.rsplen = 0;
        usart_starttx();
        MODEM.txpending = 1;
    }
    }
    hal_enableIRQs();
}

// LRSC MAC event handler
// encode and queue event for output
void onEvent (ev_t ev) {
#if 0
    // turn LED off for a short moment
    leds_set(LED_POWER, 0);
    os_setTimedCallback(&MODEM.ledjob, os_getTime()+ms2osticks(200), ledfunc);

    // update sequence counters for session
    if(PERSIST->flags & FLAGS_SESSPAR) {
    eeprom_write(&PERSIST->seqnoUp, LMIC.seqnoUp);
    eeprom_write(&PERSIST->seqnoDn, LMIC.seqnoDn);
    }

    // take action on specific events
    switch(ev) {
    case EV_JOINING:
    // start blinking
    blinkfunc(&MODEM.blinkjob);
    break;
    case EV_JOINED: {
    // cancel blink job
    os_clearCallback(&MODEM.blinkjob);
    // switch on LED
    leds_set(LED_SESSION, 1);
    // save newly established session
    sessparam_t newsession;
    newsession.netid = LMIC.netid;
    newsession.devaddr = LMIC.devaddr;
    memcpy(newsession.nwkkey, LMIC.nwkKey, 16);
    memcpy(newsession.artkey, LMIC.artKey, 16);
    eeprom_copy(&PERSIST->sesspar, &newsession, sizeof(newsession));
    eeprom_write(&PERSIST->seqnoDn, LMIC.seqnoDn);
    eeprom_write(&PERSIST->seqnoUp, LMIC.seqnoUp);
    eeprom_write(&PERSIST->flags, PERSIST->flags | FLAGS_SESSPAR);
      }
    }

    // report events (selected by eventmask)
    if(PERSIST->eventmask & (1 << ev)) {
    u1_t len = strlen(evnames[ev]);
    u1_t* buf;
    switch(ev) {
    case EV_TXCOMPLETE:
    case EV_RXCOMPLETE: { // report EV_XXCOMPLETE,<flags>[,<port>[,<downstream data>]]
        u1_t f = LMIC.txrxFlags; // EV_XXCOMPLETE,FF[,PP[,DDDDDDDD...]]
        buf = buffer_alloc(3 + len + 1 + 2 + ((f & TXRX_PORT) ? 3 : 0) + (LMIC.dataLen ? 1+2*LMIC.dataLen:0) + 2);
        memcpy(buf, "EV_", 3);
        memcpy(buf+3, evnames[ev], len);
        len += 3;
        buf[len++] = ',';
        // flags
        buf[len++] = (f & TXRX_ACK) ? 'A' : (f & TXRX_NACK) ? 'N' : '0';
        buf[len++] = (f & TXRX_DNW1) ? '1' : (f & TXRX_DNW2) ? '2' : (f & TXRX_PING) ? 'P' : '0';
        if(f & TXRX_PORT) { // port
        buf[len++] = ',';
        len += puthex(buf+len, &LMIC.frame[LMIC.dataBeg-1], 1);
        }
        if(LMIC.dataLen) { // downstream data
        buf[len++] = ',';
        len += puthex(buf+len, LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
        }
        break;
    }
    default: // just report event name
        buf = buffer_alloc(3+len+2);
        memcpy(buf, "EV_", 3);
        memcpy(buf+3, evnames[ev], len);
        len += 3;
        break;
    }
    buf[len++] = '\r';
    buf[len++] = '\n';
    queue_add(buf, len);
    modem_starttx();
    }
#endif
}

static void onAlarm (void) {
    queue_add("EV_ALARM\r\n", 10);
    modem_starttx();
}

static void modem_reset () {
    //os_clearCallback(&MODEM.alarmjob); // cancel alarm job
    //os_clearCallback(&MODEM.blinkjob); // cancel blink job
    //os_clearCallback(&MODEM.ledjob);   // cancel LED job


    //leds_set(LED_POWER, 1); // LED on
    //leds_set(LED_SESSION, 0); // LED off

    //LMIC_reset();

    // check for existing session
    if(PERSIST->flags & FLAGS_SESSPAR) {
    // configure session
    //LMIC_setSession(PERSIST->sesspar.netid, PERSIST->sesspar.devaddr, PERSIST->sesspar.nwkkey, PERSIST->sesspar.artkey);
    //LMIC.seqnoDn = PERSIST->seqnoDn;
    //LMIC.seqnoUp = PERSIST->seqnoUp + 2; // avoid reuse of seq numbers
    //leds_set(LED_SESSION, 1); // LED on
    }
}

// initialize persistent state (factory reset)
static void persist_init (uint8_t factory) {
#if 0
    // check if stored state matches firmware config
    //uint16_t joincfgcrc = os_crc16((uint8_t*)&joincfg, sizeof(joincfg));
    //uint16_t sesscfgcrc = os_crc16((uint8_t*)&sesscfg, sizeof(sesscfg));
    //uint32_t cfghash = (joincfgcrc << 16) | sesscfgcrc;
    if(PERSIST->cfghash != cfghash || factory) {
    uint32_t flags = 0;
    if(joincfgcrc != PATTERN_JOINCFG_CRC) { // patched
        //eeprom_copy(&PERSIST->joinpar, &joincfg, sizeof(joincfg));
        flags |= FLAGS_JOINPAR;
    }
    if(sesscfgcrc != PATTERN_SESSCFG_CRC) { // patched
        //eeprom_copy(&PERSIST->sesspar, &sesscfg, sizeof(sesscfg));
        //eeprom_write(&PERSIST->seqnoDn, 0);
        //eeprom_write(&PERSIST->seqnoUp, 0);
        //flags |= FLAGS_SESSPAR;
    }
    eeprom_write(&PERSIST->flags, flags);
    eeprom_write(&PERSIST->eventmask, ~0); // report ALL events
    eeprom_write(&PERSIST->cfghash, cfghash);
    }
#endif
}

// called by initial job
void modem_init () {
    // clear modem state
    memset(&MODEM, 0, sizeof(MODEM));

    persist_init(0);

    //leds_init();

    modem_reset();

    buffer_init();
    queue_init();

    // initialize USART
    usart_init();

    // start reception of command
    frame_init(&rxframe, MODEM.cmdbuf, sizeof(MODEM.cmdbuf));
    //usart_startrx();
}

// called by frame job
// process command and prepare response in MODEM.cmdbuf[]
void modem_rxdone () {
    uint8_t ok = 0;
    uint8_t cmd = tolower(MODEM.cmdbuf[0]);
    uint8_t len = rxframe.len;
    uint8_t* rspbuf = MODEM.cmdbuf;
    if(len == 0) { // AT
    ok = 1;
    } else if(cmd == 'v' && len == 2 && MODEM.cmdbuf[1] == '?') { // ATV? query version
    rspbuf += cpystr(rspbuf, "OK,");
    rspbuf += cpystr(rspbuf, VERSION_STR);
    ok = 1;
    } else if(cmd == 'z' && len == 1) { // ATZ reset
    modem_reset();
    ok = 1;
    } else if(cmd == '&' && len == 2 && tolower(MODEM.cmdbuf[1]) == 'f') { // AT&F factory reset
    persist_init(1);
    modem_reset();
    ok = 1;
    }/* else if(cmd == 'e' && len >= 2) { // event mask (query/set/add/remove)
    uint8_t mode = MODEM.cmdbuf[1];
    if(mode == '?' && len == 2) { // ATE? query
        rspbuf += cpystr(rspbuf, "OK,");
        if(PERSIST->eventmask == 0)       rspbuf += cpystr(rspbuf, "NONE");
        else if(PERSIST->eventmask == ~0) rspbuf += cpystr(rspbuf, "ALL");
        else {
        for(uint8_t e=0; e<32; e++) {
            if(e < sizeof(evnames)/sizeof(evnames[0]) && evnames[e] && (PERSIST->eventmask & (1 << e))) {
            if(rspbuf - MODEM.cmdbuf != 3) *rspbuf++ = '|';
            rspbuf += cpystr(rspbuf, evnames[e]);
            }
        }
        }
        ok = 1;
    } else if(mode == '=' || mode == '+' || mode == '-') { // ATE= ATE+ ATE- set/add/remove
        uint8_t i, j, clear = 0;
        uint32_t mask = 0;
        for(i=2; i < len; i = j+1) {
        for(j=i; j < len && MODEM.cmdbuf[j]!='|'; j++);
        mask |= evmatch(MODEM.cmdbuf+i, j-i);
        }
        if(mask == 0 && mode == '=') {
        if(cmpstr(MODEM.cmdbuf+2, len-2, "ALL")) mask = ~0;
        else if(cmpstr(MODEM.cmdbuf+2, len-2, "NONE")) clear = 1;
        }
        if(mask || clear) {
        if(mode == '+')      mask = PERSIST->eventmask | mask;
        else if(mode == '-') mask = PERSIST->eventmask & ~mask;
        eeprom_write(&PERSIST->eventmask, mask);
        ok = 1;
        }
    }
    }*/ else if(cmd == 's' && len >= 2) { // SESSION parameters
    if(MODEM.cmdbuf[1] == '?' && len == 2) { // ATS? query (netid,devaddr,seqnoup,seqnodn)
        if(PERSIST->flags & FLAGS_SESSPAR) {
        rspbuf += cpystr(rspbuf, "OK,");
        rspbuf += int2hex(rspbuf, PERSIST->sesspar.netid);
        *rspbuf++ = ',';
        rspbuf += int2hex(rspbuf, PERSIST->sesspar.devaddr);
        *rspbuf++ = ',';
        rspbuf += int2hex(rspbuf, PERSIST->seqnoUp);
        *rspbuf++ = ',';
        rspbuf += int2hex(rspbuf, PERSIST->seqnoDn);
        ok = 1;
        }
    } else if(MODEM.cmdbuf[1] == '=' && len == 2+8+1+8+1+32+1+32) { // ATS= set (netid,devaddr,nwkkey,artkey)
        sessparam_t par;
        if( hex2int(&par.netid, MODEM.cmdbuf+2, 8) &&
        MODEM.cmdbuf[2+8] == ',' &&
        hex2int(&par.devaddr, MODEM.cmdbuf+2+8+1, 8) &&
        MODEM.cmdbuf[2+8+1+8] == ',' &&
        gethex(par.nwkkey, MODEM.cmdbuf+2+8+1+8+1, 32) == 16 &&
        MODEM.cmdbuf[2+8+1+8+1+32] == ',' &&
        gethex(par.artkey, MODEM.cmdbuf+2+8+1+8+1+32+1, 32) == 16 ) {
        // use new parameters
        //LMIC_reset();
        //LMIC_setSession(par.netid, par.devaddr, par.nwkkey, par.artkey);
        // switch on LED
        //leds_set(LED_SESSION, 1);
        // save parameters
        //eeprom_copy(&PERSIST->sesspar, &par, sizeof(par));
        //eeprom_write(&PERSIST->seqnoUp, LMIC.seqnoUp);
        //eeprom_write(&PERSIST->seqnoDn, LMIC.seqnoDn);
        //eeprom_write(&PERSIST->flags, PERSIST->flags | FLAGS_SESSPAR);
        ok = 1;
        }
    }
    } else if(cmd == 'j' && len >= 2) { // JOIN parameters
    if(MODEM.cmdbuf[1] == '?' && len == 2) { // ATJ? query (deveui,appeui)
        if(PERSIST->flags & FLAGS_JOINPAR) {
        uint8_t tmp[8];
        rspbuf += cpystr(rspbuf, "OK,");
        reverse(tmp, PERSIST->joinpar.deveui, 8);
        rspbuf += puthex(rspbuf, tmp, 8);
        *rspbuf++ = ',';
        reverse(tmp, PERSIST->joinpar.appeui, 8);
        rspbuf += puthex(rspbuf, tmp, 8);
        ok = 1;
        }
    } else if(MODEM.cmdbuf[1] == '=' && len == 2+16+1+16+1+32) { // ATJ= set (deveui,appeui,devkey)
        joinparam_t par;
        if( gethex(par.deveui, MODEM.cmdbuf+2,           16) == 8 &&
        MODEM.cmdbuf[2+16] == ',' &&
        gethex(par.appeui, MODEM.cmdbuf+2+16+1,      16) == 8 &&
        MODEM.cmdbuf[2+16+1+16] == ',' &&
        gethex(par.devkey, MODEM.cmdbuf+2+16+1+16+1, 32) == 16 ) {
        reverse(par.deveui, par.deveui, 8);
        reverse(par.appeui, par.appeui, 8);
        /*eeprom_copy(&PERSIST->joinpar, &par, sizeof(par));
        eeprom_write(&PERSIST->flags, PERSIST->flags | FLAGS_JOINPAR);*/
        ok = 1;
        }
    }
    } else if(cmd == 'j' && len == 1) { // ATJ join network
    if(PERSIST->flags & FLAGS_JOINPAR) {
        //LMIC_reset(); // force join
        //LMIC_startJoining();
        ok = 1;
    }
    } /*else if(cmd == 't' && len >= 1) { // ATT transmit
    if(len == 1) { // no conf, no port, no data
        if(LMIC.devaddr || (PERSIST->flags & FLAGS_JOINPAR)) { // implicitely join!
        //LMIC_sendAlive(); // send empty frame
        ok = 1;
        }
    } else if(len >= 5) { // confirm,port[,data]  (0,FF[,112233...])
        if((MODEM.cmdbuf[1]=='0' || MODEM.cmdbuf[1]=='1') && MODEM.cmdbuf[2]==',' && // conf
           gethex(&LMIC.pendTxPort, MODEM.cmdbuf+1+1+1, 2) == 1 && LMIC.pendTxPort) { // port
        LMIC.pendTxConf = MODEM.cmdbuf[1] - '0';
        LMIC.pendTxLen = 0;
        if(len > 5 && MODEM.cmdbuf[5]==',') { // data
            LMIC.pendTxLen = gethex(LMIC.pendTxData, MODEM.cmdbuf+6, len-6);
        }
        if(len == 5 || LMIC.pendTxLen) {
            if(LMIC.devaddr || (PERSIST->flags & FLAGS_JOINPAR)) { // implicitely join!
            //LMIC_setTxData();
            ok = 1;
            }
        }
        }
    }
}*/ else if(cmd == 'p' && len == 2) { // ATP set ping mode
    /*if(LMIC.devaddr) { // requires a session
        uint8_t n = MODEM.cmdbuf[1];
        if(n>='0' && n<='7') {
        //LMIC_setPingable(n-'0');
        ok = 1;
        }
    }*/
    } else if(cmd == 'a' && len >= 2) { // ATA set alarm timer
    uint32_t secs;
    if(hex2int(&secs, MODEM.cmdbuf+1, len-1)) {
        //os_setTimedCallback(&MODEM.alarmjob, os_getTime()+sec2osticks(secs), onAlarm);
        ok = 1;
    }
    }

    // send response
    if(ok) {
    if(rspbuf == MODEM.cmdbuf) {
        rspbuf += cpystr(rspbuf, "OK");
    }
    } else {
    rspbuf += cpystr(rspbuf, "ERROR");
    }
    *rspbuf++ = '\r';
    *rspbuf++ = '\n';
    MODEM.rsplen = rspbuf - MODEM.cmdbuf;
    Chip_UART_SendRB(LPC_USART0, &txring, MODEM.cmdbuf, MODEM.rsplen);
    frame_init(&rxframe, MODEM.cmdbuf, sizeof(MODEM.cmdbuf));
    //modem_starttx();
}

// called by frame job
void modem_txdone () {
    MODEM.txpending = 0;
    // free events (static buffers ignored)
    buffer_free(txframe.buf, txframe.len);
    if(txframe.buf == MODEM.cmdbuf) { // response transmitted
    // restart reception for new command
    frame_init(&rxframe, MODEM.cmdbuf, sizeof(MODEM.cmdbuf));
    usart_startrx();
    }
    // start transmission of next output message
    modem_starttx();
}
