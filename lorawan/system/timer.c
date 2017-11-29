/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Timer objects and scheduling management

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"
#include "timer.h"
//#include "rtc-board.h"
#include "utilities.h"
uint32_t systicks = 0;
/*!
 * This flag is used to loop through the main several times in order to be sure
 * that all pending events have been processed.
 */
volatile uint8_t HasLoopedThroughMain = 0;

/*!
 * Timers list head pointer
 */
static TimerEvent_t *TimerListHead = NULL;

/*!
 * \brief Adds or replace the head timer of the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be become the new head
 * \param [IN]  remainingTime Remaining time of the previous head to be replaced
 */
static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime );

/*!
 * \brief Adds a timer to the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be added to the list
 * \param [IN]  remainingTime Remaining time of the running head after which the object may be added
 */
static void TimerInsertTimer( TimerEvent_t *obj, uint32_t remainingTime );

/*!
 * \brief Sets a timeout with the duration "timestamp"
 *
 * \param [IN] timestamp Delay duration
 */
static void TimerSetTimeout( TimerEvent_t *obj );

/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */
static bool TimerExists( TimerEvent_t *obj );

/*!
 * \brief Read the timer value of the currently running timer
 *
 * \retval value current timer value
 */
TimerTime_t TimerGetValue( void );

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->IsRunning = false;
    obj->Callback = callback;
    obj->Next = NULL;
}

void TimerStart( TimerEvent_t *obj )
{
    TimerEvent_t *cur = TimerListHead;
    TimerEvent_t *prev = TimerListHead;
    BoardDisableIrq( );

    if( ( obj == NULL ) || obj->ReloadValue == 0)
    {
        BoardEnableIrq( );
        return;
    }
    obj->Timestamp = obj->ReloadValue;
    obj->Timestamp += systicks;
    obj->IsRunning = true;
    if( TimerExists( obj ) )
    {
        BoardEnableIrq( );
        return;
    }
    obj->Next = NULL;
    if(TimerListHead == NULL)
    {
        TimerListHead = obj;
        BoardEnableIrq( );
        return;
    }
    prev = cur;
    cur = cur->Next;
    while(cur != NULL)
    {
        prev = cur;
        cur = cur->Next;
    }
    prev->Next = obj;
    BoardEnableIrq( );
    return;
}

void SysTick_Handler( void )
{
    TimerEvent_t *cur = TimerListHead;
    BoardDisableIrq( );
    systicks ++;
    while(cur != NULL)
    {
        if((cur->IsRunning) && (cur->Timestamp == systicks))
        {
            cur->IsRunning = false;
            if( cur->Callback != NULL )
            {
                cur->Callback( );
            }
        }
        cur = cur->Next;
    }
    BoardEnableIrq( );
}

void TimerStop( TimerEvent_t *obj )
{
    BoardDisableIrq( );
    obj->IsRunning = false;
    BoardEnableIrq( );
}

static bool TimerExists( TimerEvent_t *obj )
{
    TimerEvent_t* cur = TimerListHead;

    while( cur != NULL )
    {
        if( cur == obj )
        {
            return true;
        }
        cur = cur->Next;
    }
    return false;
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    TimerStop( obj );
    obj->Timestamp = value;
    obj->ReloadValue = value;
}
#if 0
TimerTime_t TimerGetValue( void )
{
    return 0;//RtcGetElapsedAlarmTime( );
}
#endif
TimerTime_t TimerGetCurrentTime( void )
{
    return systicks;//RtcGetTimerValue( );
}

TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    if(savedTime > systicks)
    {
        return (0xffffffff - savedTime + systicks);
    }
    else
    {
        return (systicks - savedTime);
    }
    return 0;//RtcComputeElapsedTime( savedTime );
}
#if 0
TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture )
{
    return 0;// RtcComputeFutureEventTime( eventInFuture );
}

static void TimerSetTimeout( TimerEvent_t *obj )
{
    HasLoopedThroughMain = 0;
    obj->Timestamp = 0;//RtcGetAdjustedTimeoutValue( obj->Timestamp );
    //RtcSetTimeout( obj->Timestamp );
}

void TimerLowPowerHandler( void )
{
    if( ( TimerListHead != NULL ) && ( TimerListHead->IsRunning == true ) )
    {
        if( HasLoopedThroughMain < 5 )
        {
            HasLoopedThroughMain++;
        }
        else
        {
            HasLoopedThroughMain = 0;
            /*
            if( GetBoardPowerSource( ) == BATTERY_POWER )
            {
                RtcEnterLowPowerStopMode( );
            }*/
        }
    }
}
#endif