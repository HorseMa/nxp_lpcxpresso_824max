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
#include "timer.h"
#include "radio.h"

//////////////////////////////////////////////////
// CONFIGURATION (WILL BE PATCHED)
//////////////////////////////////////////////////

static union {
    joinparam_t param;
    uint8_t pattern[sizeof(joinparam_t)];
} joincfg;// = {
    //.pattern = { PATTERN_JOINCFG_STR }
//};

static const union {
    sessparam_t param;
    uint8_t pattern[sizeof(sessparam_t)];
} sesscfg;// = {
    //.pattern = { PATTERN_SESSCFG_STR }
//};
//joinparam_t joincfg;
//sessparam_t sesscfg;
uint16_t PATTERN_JOINCFG_CRC;
uint16_t PATTERN_SESSCFG_CRC;
persist_t persist;
/*!
 * Timer to handle the state of LED1
 */
TimerEvent_t Led1Timer_Tx;
TimerEvent_t Led1Timer_Rx;
TimerEvent_t Led1Timer_OnLine;
TimerEvent_t Led1Timer_OffLine;
struct lmic_t LMIC;
// COMMAND TO SET DEVEUI=FF-FF-FF-FF-FF-FF-FF-00, APPEUI=DE-DE-AA-AA-00-00-00-1A (PORT=25501):
//   ATJ=FFFFFFFFFFFFFF00,DEDEAAAA0000001A,AA5555555555555555AAAAAAAAAAAAAA
//   OK

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

extern uint8_t DevEui[];
extern uint8_t AppEui[];
extern uint8_t AppKey[];
extern RINGBUFF_T txring, rxring;

void WDT_IRQHandler(void)
{
}

void modem_wwdt_init(void)
{
    uint32_t wdtFreq;
    
    /* Freq = 0.6Mhz, divided by 64. WDT_OSC should be 9.375khz */
    Chip_Clock_SetWDTOSC(WDTLFO_OSC_0_60, 64);

    /* Enable the power to the WDT */
    Chip_SYSCTL_PowerUp(SYSCTL_SLPWAKE_WDTOSC_PD);

    /* The WDT divides the input frequency into it by 4 */
    wdtFreq = Chip_Clock_GetWDTOSCRate() / 4;

    /* Initialize WWDT (also enables WWDT clock) */
    Chip_WWDT_Init(LPC_WWDT);

    /* Set watchdog feed time constant to approximately 2s
    Set watchdog warning time to 512 ticks after feed time constant
    Set watchdog window time to 3s */
    Chip_WWDT_SetTimeOut(LPC_WWDT, wdtFreq / 2);//* 2);
    Chip_WWDT_SetWarning(LPC_WWDT, 512);
    Chip_WWDT_SetWindow(LPC_WWDT, wdtFreq / 2);//* 3);

    /* Configure WWDT to reset on timeout */
    Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);

    /* Clear watchdog warning and timeout interrupts */
    Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT);

    /* Clear and enable watchdog interrupt */
    NVIC_ClearPendingIRQ(WDT_IRQn);
    NVIC_EnableIRQ(WDT_IRQn);

    /* Start watchdog */
    Chip_WWDT_Start(LPC_WWDT);
}
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

static void OnLed1TimerEventRx( void )
{
    TimerStop( &Led1Timer_Rx );
    // Switch LED 1 OFF
    Board_LED_Set(0,1);
    LedIndication(EN_LED_SENSSION_NET);
}

static void OnLed1TimerEventTx( void )
{
    TimerStop( &Led1Timer_Tx );
    // Switch LED 1 OFF
    Board_LED_Set(0,1);
    LedIndication(EN_LED_SENSSION_NET);
}

static void OnLed1TimerEventNetOffline( void )
{
    static uint8_t state = 0;
    switch(state)
    {
        case 0:
        TimerStop( &Led1Timer_OffLine );
        // Switch LED 1 ON
        Board_LED_Set(0,1);
        TimerSetValue( &Led1Timer_OffLine, 100 );
        TimerStart( &Led1Timer_OffLine );
        state = 1;
        break;
        case 1:
        TimerStop( &Led1Timer_OffLine );
        // Switch LED 1 OFF
        Board_LED_Set(0,0);
        TimerSetValue( &Led1Timer_OffLine, 200 );
        TimerStart( &Led1Timer_OffLine );
        state = 2;
        break;
        case 2:
        TimerStop( &Led1Timer_OffLine );
        // Switch LED 1 ON
        Board_LED_Set(0,1);
        TimerSetValue( &Led1Timer_OffLine, 100 );
        TimerStart( &Led1Timer_OffLine );
        state = 3;
        break;
        case 3:
        TimerStop( &Led1Timer_OffLine );
        // Switch LED 1 OFF
        Board_LED_Set(0,0);
        TimerSetValue( &Led1Timer_OffLine, 1000 );
        TimerStart( &Led1Timer_OffLine );
        state = 0;
        break;
        default:
        state = 0;
        break;
    }
}

static void OnLed1TimerEventNetOnline( void )
{
    static uint8_t state = 0;
    switch(state)
    {
        case 0:
        TimerStop( &Led1Timer_OnLine );
        // Switch LED 1 ON
        Board_LED_Set(0,1);
        TimerSetValue( &Led1Timer_OnLine, 100 );
        TimerStart( &Led1Timer_OnLine );
        state = 1;
        break;
        case 1:
        TimerStop( &Led1Timer_OnLine );
        // Switch LED 1 OFF
        Board_LED_Set(0,0);
        TimerSetValue( &Led1Timer_OnLine, 1000 );
        TimerStart( &Led1Timer_OnLine );
        state = 0;
        break;
        default:
        state = 0;
        break;
    }
}

void LedIndication(LedSension_t senssion)
{
    TimerStop( &Led1Timer_Tx );
    TimerStop( &Led1Timer_Rx );
    TimerStop( &Led1Timer_OnLine );
    TimerStop( &Led1Timer_OffLine );
    // Switch LED 1 OFF
    Board_LED_Set(0,0);
    // Switch LED 1 ON
    //Board_LED_Set(0,0);
    switch(senssion)
    {
        case EN_LED_SENSSION_TX:
        // Switch LED 1 ON
        Board_LED_Set(0,1);
        TimerSetValue( &Led1Timer_Tx, 25 );
        TimerStart( &Led1Timer_Tx );
        break;
        case EN_LED_SENSSION_RX:
        // Switch LED 1 ON
        Board_LED_Set(0,1);
        TimerSetValue( &Led1Timer_Rx, 50 );
        TimerStart( &Led1Timer_Rx );
        break;
        case EN_LED_SENSSION_NET:
        if(IsLoRaMacNetworkJoined)
        {
            TimerSetValue( &Led1Timer_OnLine, 1000 );
            TimerStart( &Led1Timer_OnLine );
        }
        else
        {
            TimerSetValue( &Led1Timer_OffLine, 1000 );
            TimerStart( &Led1Timer_OffLine );
        }
        break;
        default:
        break;
    }
}
/*
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
}*/

// LRSC MAC event handler
// encode and queue event for output
#if 0
void onEvent (ev_t ev) {
    static uint8_t buf[64];
#if 0
  /*
    // update sequence counters for session
    if(PERSIST->flags & FLAGS_SESSPAR) {
    eeprom_write(&PERSIST->seqnoUp, LMIC.seqnoUp);
    eeprom_write(&PERSIST->seqnoDn, LMIC.seqnoDn);
    }
*/
    // take action on specific events
    switch(ev) {
    case EV_JOINING:
    break;
    case EV_JOINED: {
    LedIndication(EN_LED_SENSSION_NET);
    // save newly established session
    /*sessparam_t newsession;
    newsession.netid = LMIC.netid;
    newsession.devaddr = LMIC.devaddr;
    memcpy(newsession.nwkkey, LMIC.nwkKey, 16);
    memcpy(newsession.artkey, LMIC.artKey, 16);
    eeprom_copy(&PERSIST->sesspar, &newsession, sizeof(newsession));
    eeprom_write(&PERSIST->seqnoDn, LMIC.seqnoDn);
    eeprom_write(&PERSIST->seqnoUp, LMIC.seqnoUp);
    eeprom_write(&PERSIST->flags, PERSIST->flags | FLAGS_SESSPAR);*/
      }
    }

    // report events (selected by eventmask)
    if(PERSIST->eventmask & (1 << ev)) {
    uint8_t len = strlen(evnames[ev]);
    uint8_t* buf;
    switch(ev) {
    case EV_TXCOMPLETE:
    case EV_RXCOMPLETE: { // report EV_XXCOMPLETE,<flags>[,<port>[,<downstream data>]]
        uint8_t f = LMIC.txrxFlags; // EV_XXCOMPLETE,FF[,PP[,DDDDDDDD...]]
        //buf = buffer_alloc(3 + len + 1 + 2 + ((f & TXRX_PORT) ? 3 : 0) + (LMIC.dataLen ? 1+2*LMIC.dataLen:0) + 2);
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
        //buf = buffer_alloc(3+len+2);
        memcpy(buf, "EV_", 3);
        memcpy(buf+3, evnames[ev], len);
        len += 3;
        break;
    }
    buf[len++] = '\r';
    buf[len++] = '\n';
    Chip_UART_SendRB(LPC_USART0, &txring, buf, len);
    //buffer_free(buf,len);
    //queue_add(buf, len);
    //modem_starttx();
    }
#endif
}
#endif
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

typedef              uint8_t* xref2u1_t;
typedef unsigned int       uint;
uint16_t os_crc16 (xref2u1_t data, uint len) {
    uint16_t remainder = 0;
    uint16_t polynomial = 0x1021;
    for( uint i = 0; i < len; i++ ) {
        remainder ^= data[i] << 8;
        for( uint8_t bit = 8; bit > 0; bit--) {
            if( (remainder & 0x8000) )
                remainder = (remainder << 1) ^ polynomial;
            else 
                remainder <<= 1;
        }
    }
    return remainder;
}

// initialize persistent state (factory reset)
static void persist_init (uint8_t factory) {
    // check if stored state matches firmware config
    uint16_t joincfgcrc = os_crc16((uint8_t*)&joincfg, sizeof(joincfg));
    uint16_t sesscfgcrc = os_crc16((uint8_t*)&sesscfg, sizeof(sesscfg));
    
    uint32_t cfghash = (joincfgcrc << 16) | sesscfgcrc;
    if(PERSIST->cfghash != cfghash || factory) {
	uint32_t flags = 0;
        //eeprom_erase();
	//if(joincfgcrc != PATTERN_JOINCFG_CRC) { // patched
            memcpy(&persist.joinpar,&joincfg,sizeof(joinparam_t));
	    //eeprom_copy(&PERSIST->joinpar, &joincfg, sizeof(joincfg));
	    flags |= FLAGS_JOINPAR;
	//}
	//if(sesscfgcrc != PATTERN_SESSCFG_CRC) { // patched
            memcpy(&persist.joinpar,&joincfg,sizeof(joinparam_t));
            persist.seqnoDn = 0;
            persist.seqnoUp = 0;
	    //eeprom_copy(&PERSIST->sesspar, &sesscfg, sizeof(sesscfg));
	    //eeprom_write(&PERSIST->seqnoDn, 0);
	    //eeprom_write(&PERSIST->seqnoUp, 0);
	    //flags |= FLAGS_SESSPAR;
	//}
        persist.flags = flags;
        persist.eventmask = ~0;
        persist.cfghash = cfghash;
	//eeprom_write(&PERSIST->flags, flags);
	//eeprom_write(&PERSIST->eventmask, ~0); // report ALL events
	//eeprom_write(&PERSIST->cfghash, cfghash);
        //memset(src_iap_array_data,0,IAP_NUM_BYTES_TO_WRITE);
        //memcpy(src_iap_array_data,&persist,sizeof(persist));
        eeprom_write();
    }
}

// called by initial job
void modem_init () {
    uint32_t unique_id[4];
    Chip_IAP_ReadUID(unique_id);
    memcpy(&DevEui,&unique_id[2],8);
    memcpy(joincfg.param.deveui,DevEui,8);
    memcpy(joincfg.param.appeui,AppEui,8);
    memcpy(joincfg.param.devkey,AppKey,16);
    uint16_t joincfgcrc = os_crc16((uint8_t*)&joincfg, sizeof(joincfg));
    uint16_t sesscfgcrc = os_crc16((uint8_t*)&sesscfg, sizeof(sesscfg));
    PATTERN_JOINCFG_CRC = joincfgcrc;
    PATTERN_SESSCFG_CRC = sesscfgcrc;
    persist.cfghash = (joincfgcrc << 16) | sesscfgcrc;
    
    // clear modem state
    memset(&MODEM, 0, sizeof(MODEM));

    persist_init(0);
    memcpy(DevEui,PERSIST->joinpar.deveui,8);
    memcpy(AppEui,PERSIST->joinpar.appeui,8);
    memcpy(AppKey,PERSIST->joinpar.devkey,16);
    //leds_init();

    modem_reset();

    //buffer_init();
    //queue_init();

    // initialize USART
    usart_init();

    // start reception of command
    frame_init(&rxframe, MODEM.cmdbuf, sizeof(MODEM.cmdbuf));
    TimerInit( &Led1Timer_Tx,OnLed1TimerEventTx);
    TimerInit( &Led1Timer_Rx,OnLed1TimerEventRx);
    TimerInit( &Led1Timer_OnLine, OnLed1TimerEventNetOnline );
    TimerInit( &Led1Timer_OffLine, OnLed1TimerEventNetOffline );
    //TimerSetValue(&Led1Timer,1000);
    //TimerStart( &Led1Timer );
    //usart_startrx();
}

// called by frame job
// process command and prepare response in MODEM.cmdbuf[]
void modem_rxdone () {
    uint8_t ok = 0;
    uint8_t cmd = tolower(MODEM.cmdbuf[0]);
    uint8_t len = rxframe.len;
    uint8_t* rspbuf = MODEM.cmdbuf;
    uint8_t rst = false;
    if(len == 0) { // AT
    ok = 1;
    } else if(cmd == 'v' && len == 2 && MODEM.cmdbuf[1] == '?') { // ATV? query version
    rspbuf += cpystr(rspbuf, "OK,");
    rspbuf += cpystr(rspbuf, VERSION_STR);
    ok = 1;
    } else if(cmd == 'z' && len == 1) { // ATZ reset
    Radio.Sleep( );
    rst = true;
    ok = 1;
    } else if(cmd == '&' && len == 2 && tolower(MODEM.cmdbuf[1]) == 'f') { // AT&F factory reset
    persist_init(1);
    rst = true;
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
        memcpy(&persist.sesspar,&par,sizeof(par));
        persist.seqnoUp = LMIC.seqnoUp;
        persist.seqnoDn = LMIC.seqnoDn;
        persist.flags = PERSIST->flags | FLAGS_SESSPAR;
        
        //eeprom_erase();
        eeprom_write();
        Radio.Sleep( );
        rst = true;
        //eeprom_copy(&PERSIST->sesspar, &par, sizeof(par));
        //eeprom_write(&PERSIST->seqnoUp, LMIC.seqnoUp);
        //eeprom_write(&PERSIST->seqnoDn, LMIC.seqnoDn);
        //eeprom_write(&PERSIST->flags, PERSIST->flags | FLAGS_SESSPAR);
        ok = 1;
        }
    }
    }else if(cmd == 'j' && len >= 2) { // JOIN parameters
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
        
        memcpy(&persist.joinpar,&par,sizeof(par));
        persist.flags = PERSIST->flags | FLAGS_JOINPAR;
        
        //eeprom_erase();
        eeprom_write();
        Radio.Sleep( );
        rst = true;
        //eeprom_copy(&PERSIST->joinpar, &par, sizeof(par));
        //eeprom_write(&PERSIST->flags, PERSIST->flags | FLAGS_JOINPAR);
        ok = 1;
        }
    }
    }/* else if(cmd == 'j' && len == 1) { // ATJ join network
    if(PERSIST->flags & FLAGS_JOINPAR) {
        //LMIC_reset(); // force join
        //LMIC_startJoining();
        ok = 1;
    }
    } *//*else if(cmd == 't' && len >= 1) { // ATT transmit
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
}*/ /*else if(cmd == 'p' && len == 2) { // ATP set ping mode
    if(LMIC.devaddr) { // requires a session
        uint8_t n = MODEM.cmdbuf[1];
        if(n>='0' && n<='7') {
        //LMIC_setPingable(n-'0');
        ok = 1;
        }
    }
    }*//* else if(cmd == 'a' && len >= 2) { // ATA set alarm timer
    uint32_t secs;
    if(hex2int(&secs, MODEM.cmdbuf+1, len-1)) {
        //os_setTimedCallback(&MODEM.alarmjob, os_getTime()+sec2osticks(secs), onAlarm);
        ok = 1;
    }
    }*/

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
    if(rst == true)
    {
        while(1);//Delay(100);
        //Reset_Handler();
    }
    frame_init(&rxframe, MODEM.cmdbuf, sizeof(MODEM.cmdbuf));
    //modem_starttx();
}

// called by frame job
/*void modem_txdone () {
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
}*/
