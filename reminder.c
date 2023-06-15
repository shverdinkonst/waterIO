#ifdef __cplusplus
 extern "C" {
#endif

/**
  ******************************************************************************
  * @file           : reminder.c
  * @brief          : 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 .
  * All rights reserved.</center></h2>
  *
  *  ADD GENERAL BRIEF
  *
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "reminder.h"
#include "RTC_Module.h"
#include "debugPort_Module.h"
#include "flash_Module.h"
#include "HYD_Detection.h"
#include "../Peripheral/LEDSBasicPlans.h"

/* Private typedef -----------------------------------------------------------*/

// these are the events to handle in this module
typedef enum
{
    TIME_CHANGED                            =  1,   
    REMINDER_INTERVAL_ALARM                 =  2,
    REMINDER_ROUND_ALARM                    =  4,
    REMINDER_CYCLE_ALARM                    =  8,
    REMINDER_NEXT_DAY_ALARM                 = 16,
    REMINDER_END_DAY_ALARM                  = 32,
    REMINDER_START_MEASURE_ALARM            = 64,
    REMINDER_SYSTEM_PARAMETERS_CHANGE_EVENT = 128,
}reminderFlags_t;

// when we prepare  to next round , interval or cycle we should ignore these existing events
#define REMINDER_MASK (~(REMINDER_INTERVAL_ALARM | REMINDER_ROUND_ALARM | REMINDER_CYCLE_ALARM))

typedef struct
{
    uint16_t            dailyGoal;              // millilitre
    uint8_t             startTime;              // hour, time in day to start the reminder
    uint8_t             endTime;                // hour, time in day to end the reminder
    uint16_t            reminderInterval;       // minutes, time between 2 reminders
    uint8_t             reminderRounds;         // how many rounds in one reminder
    uint8_t             reminderRoundWait;      // time in minutes, wait time between two round
    uint8_t             reminderCycles;         // how many cycles in one round
    uint8_t             reminderCycleWait;      // time in seconds, wait time between two cycles
    reminderOptions_t   reminderOptions;        // bitmap, indicating what is active. leds, sound and vibration. 
    reminderOptions_t   reminderOptionsMask;    // mask to enable \ disable these options dynamically
    uint8_t             reminderLedsPattern;    // indicate which plan to run during reminder (pulse, bounce,...)
    uint8_t             autoOff;
}reminderConfig_t;

typedef struct
{
    uint64_t            reminderTimeStamp;                          // interval time stamp
    uint64_t            reminderRoundTimeStamp;                     // round time stamp
    uint64_t            remindersTimeoutTillNextDayOrOpenClose;     // its the timeout for two hours timout of reminders
    uint64_t            remindersTimeoutTillOpenClose;              // its the timeout for two days of not using the bottle
    uint32_t            systemParametersChange;                     // bitmap represnt which system parameter was updated via CLI\BLE
    uint8_t             flags;                                      // these flag represents events 
    uint8_t             reminderRoundsCounter;                      // round counter
    uint8_t             reminderCyclesCounter;                      // cycle counter
    endOfDayCallback_t  endOfDayCallback;                           // any none who needs to be aware of end of day can register this event
    uint8_t             enalbed;                                    // reminder state     
}reminderInfo_t;


/* Private define ------------------------------------------------------------*/
#define END_OF_DAY_NUM_OF_CALLBACKS         (2)     // end of of day callback array size
#define START_OF_DAY_NUM_OF_CALLBACKS       (2)     // start of of day callback array size
#define START_HYD_MEASURE_NUM_OF_CALLBACKS  (2)     // start hyration measure callback array size
#define REMINDER_INTERVAL_NUM_OF_CALLBACKS  (2)     // reminder interval event callback array sze


#define REMINDER_OPTIONS_ALL                (0xff)  // by default, all enalbed
#define MEASURE_START_TIME                  (3)     // means 3AM is the time we start calculate day hydration
/* Private macro -------------------------------------------------------------*/


/* Private variables --------------------------------------------------------,-*/
static reminderConfig_t reminderConfig = 
{   
    .startTime          = START_TIME,
    .endTime            = END_TIME,
    .dailyGoal          = DAILY_GOAL,             
    .reminderInterval   = REMINDER_INTERVAL,
    .reminderRounds     = REMINDER_ROUNDS,
    .reminderRoundWait  = REMINDER_ROUNDS_WAIT,
    .reminderCycles     = REMINDER_CYCLES,
    .reminderCycleWait  = REMINDER_CYCLES_WAIT,
    .reminderOptions    = REMINDER_OPTIONS,
    .reminderOptionsMask= REMINDER_OPTIONS_ALL,
    .autoOff            = REMINDER_AUTO_OFF,
    .reminderLedsPattern= 0,
};

static reminderInfo_t reminderInfo = 
{   
    .reminderTimeStamp                      = 0,
    .reminderRoundTimeStamp                 = 0,
    .remindersTimeoutTillNextDayOrOpenClose = 0,
    .remindersTimeoutTillOpenClose          = 0,
    .systemParametersChange                 = 0,
    .flags                                  = 0,
    .reminderRoundsCounter                  = 1,
    .reminderCyclesCounter                  = 1,
    .enalbed                                = 0         
};

static endOfDayCallback_t               endOfDayCallbackArray[END_OF_DAY_NUM_OF_CALLBACKS];
static startOfDayCallback_t             startOfDayCallbackArray[START_OF_DAY_NUM_OF_CALLBACKS];
static startHydrationMeasureCallback_t  startHydrationMeasureCallbackArray[START_HYD_MEASURE_NUM_OF_CALLBACKS];
static reminderIntervalCallback_t       reminderIntervalCallbackArray[REMINDER_INTERVAL_NUM_OF_CALLBACKS];



// this is a list of system parameters that this module should be aware if they were changed 
// CLI or BLE Commands.
static const flashParam_t systemParametersList[] = 
{
    PARAM_START_TIME,
    PARAM_END_TIME,
    PARAM_REMINDER_UI_ELEMENTS,
    PARAM_REMINDER_LEDS_PATTERN,
    PARAM_SOUND_VOLUME,
    PARAM_SOUND_TYPE,
    PARAM_VIBRATION_LENGTH,
    PARAM_REMINDER_LEDS_COLOR,
    PARAM_REMINDER_INTERVAL,
    PARAM_REMINDER_ROUNDS,
    PARAM_REMINDER_ROUNDS_WAIT,
    PARAM_REMINDER_CYCLES,
    PARAM_REMINDER_CYCLES_WAIT,    
    PARAM_REMINDER_STOP_AFTER_CLOSE
};

#define NUM_OF_REMINDER_PLANS (3)
static const planId_t ledsPattern[NUM_OF_REMINDER_PLANS] =
{
    LED_PLAN_ID_HYDRATION_LEVEL,
    LED_PLAN_ID_REMINDER_BOUNCE,
    LED_PLAN_ID_REMINDER_PULSE,
};

/* Private function prototypes -----------------------------------------------*/
static void     REMINDER_timeChanged(void);
static void     REMINDER_ScheduleNextAlarm(reminderFlags_t reminderFlags);
static void     REMINDER_PlayReminder(reminderFlags_t reminderFlags);
static uint8_t  REMINDER_CheckValidity(void);
static uint8_t  REMINDER_EmitEndOfDay(void);
static uint8_t  REMINDER_EmitStartOfDay(void);
static uint8_t  REMINDER_EmitStartHydrationMeasure(void);
static uint8_t  REMINDER_EmitReminderInterval(void);
static uint8_t  REMINDER_isInDayRange(void);
static void     REMINDER_readSystemParameters(void);
static void     REMINDER_ClearInfo(void);
static uint64_t REMINDER_Time2Seconds(rtcTime_t rtcTime);
static void     REMINDER_StopPlan(void);
static void     REMINDER_PlayPlan(void);

static void     REMINDER_systemParametersChanged(flashParam_t flashParam);
/* functions ---------------------------------------------------------*/

/**
* @brief  : init the open close detection module.
* @param  : 0 - OK, other - will be an error code.
* @retval 
*/
uint8_t REMINDER_Init(void)
{
    uint8_t res = REMINDER_CheckValidity();
    
    if (OK != res)
    {
        return res;
    }
    
    for (int i = 0 ; i < END_OF_DAY_NUM_OF_CALLBACKS; i++)
    {
        endOfDayCallbackArray[i] = NULL;
    }
    
    for (int i = 0 ; i < START_OF_DAY_NUM_OF_CALLBACKS; i++)
    {
        startOfDayCallbackArray[i] = NULL;
    }
    
    for (int i = 0 ; i < START_HYD_MEASURE_NUM_OF_CALLBACKS; i++)
    {
        startHydrationMeasureCallbackArray[i] = NULL;
    }

    REMINDER_readSystemParameters();
        
    // register for time changed event
    RTC_RegisterTimeChanged(REMINDER_timeChanged);
    
    if (OK != UserFlash_RegisterSystemParametersChanged(REMINDER_systemParametersChanged, (flashParam_t*)systemParametersList, sizeof(systemParametersList)))
    {
        uprint("REMINDER - failed to register system parameters change event\n\r");
    }
    
    //reminderInfo.remindersTimeoutTillOpenClose = RTC_GetRawTime()+1;
    reminderInfo.remindersTimeoutTillNextDayOrOpenClose = RTC_GetRawTime();    
    
    REMINDER_RestartReminder(true);
    
	return OK;
}

void REMINDER_RestartReminder(bool isNeedLEDsOFF)
{
    uprint("REMINDER_RestartReminder\n\r");
    
    //reminderInfo.remindersTimeoutTillOpenClose = RTC_GetRawTime();//RTC_GetUnixTimeAndDate();
    reminderInfo.remindersTimeoutTillNextDayOrOpenClose = RTC_GetRawTime();
    // so every time this function is called and an interval alarm is really happened, we start counting again
    if (isNeedLEDsOFF == true)
    {
        REMINDER_StopPlan();
    }

    if (REMINDER_isInDayRange())
    {
        //uprint("REMINDER_INTERVAL_ALARM\n\r");
        REMINDER_ScheduleNextAlarm(REMINDER_INTERVAL_ALARM);    
    }
    else
    {
        uprint("REMINDER_NEXT_DAY_ALARM\n\r");
        REMINDER_ScheduleNextAlarm(REMINDER_NEXT_DAY_ALARM); 
    }
}

static void REMINDER_StopPlan(void)
{
    if (reminderConfig.reminderOptions & OPT_LEDS & reminderConfig.reminderOptionsMask)
    {
        uprint("LEDs OFF by REMINDER_Stop function\n\r");
        LEDS_Stop();
    }
}

static void REMINDER_PlayPlan(void)
{
    if (reminderConfig.reminderOptions & OPT_LEDS & reminderConfig.reminderOptionsMask)
    {
        switch(reminderConfig.reminderLedsPattern)
        {
            case 1:
            case 2:
                LEDS_Play(ledsPattern[reminderConfig.reminderLedsPattern]);
            break;
            case 0:
                {
                    HYD_INFO_t hydInfo;
                    hydInfo = HYD_getHydrationLevel();
                    LEDS_ShowHydrationLevel(hydInfo.hydrationLevel, hydInfo.needToDrinkNow, false);
                }
            break;
            default:
                LEDS_Play(ledsPattern[0]);
        }
    }
}

bool REMINDER_isReminderIndicationsAreActiveNow(void)
{
    return LEDS_isActiveNow();
}

bool REMINDER_isReminderPlanIsRunning(void)
{
    planId_t planId = LEDS_returnActivePlanId();

    if (planId == ledsPattern[reminderConfig.reminderLedsPattern])
    {
        //uprint("reminder busy..........\n\r");
        return true;
    }
    return false;
}

void REMINDER_DisableReminder(void)
{
    uprint("REMINDER_DisableReminder\n\r");
    
    REMINDER_StopPlan();
    
    RTC_StopAlarm();   
}

reminderEvent_t REMINDER_Handler(time_t time)
{
    reminderEvent_t status = reminderidle;
    //uint64_t currTime = RTC_GetUnixTimeAndDate();     // NOT USED
    
    if (reminderInfo.flags & TIME_CHANGED)
    {
        reminderInfo.flags &= (~TIME_CHANGED);
        REMINDER_ScheduleNextAlarm(TIME_CHANGED);        
        return status; // if time changed then the time we get in this function is not updated. 
    }
    
    if (reminderInfo.flags & REMINDER_INTERVAL_ALARM)
    {
        uprint("reminder Alarm\n\r");
        reminderInfo.flags &= (~REMINDER_INTERVAL_ALARM);
        REMINDER_ScheduleNextAlarm(REMINDER_INTERVAL_ALARM); 
        REMINDER_EmitReminderInterval();
        status = intervalEvent;
    }
    
    if (reminderInfo.flags & REMINDER_ROUND_ALARM)
    {
        uprintf("reminder round Alarm - %d\\%d\\(%d)\n\r", reminderInfo.reminderRoundsCounter, reminderInfo.reminderCyclesCounter, reminderConfig.reminderRounds);
        reminderInfo.flags &= (~REMINDER_ROUND_ALARM);  
        // get time of first round 
        reminderInfo.reminderRoundTimeStamp = RTC_GetUnixTimeAndDate();
        // play is generating the next cycle, so we dont need to call schedule next alarm directly
        REMINDER_PlayReminder(REMINDER_ROUND_ALARM);
    }
    
    if (reminderInfo.flags & REMINDER_CYCLE_ALARM)
    {
        uprintf("reminder cycle Alarm - %d\\%d\\(%d)\n\r", reminderInfo.reminderRoundsCounter, reminderInfo.reminderCyclesCounter, reminderConfig.reminderCycles);
        reminderInfo.flags &= (~REMINDER_CYCLE_ALARM);
        // play is generating the next cycle, so we dont need to call schedule next alarm directly
        REMINDER_PlayReminder(REMINDER_CYCLE_ALARM);        
    }
    
    if (reminderInfo.flags & REMINDER_NEXT_DAY_ALARM)
    {
        uprint("rem. day started\n\r");
        REMINDER_ClearInfo();
        reminderInfo.enalbed = true;
        reminderInfo.flags &= (~REMINDER_NEXT_DAY_ALARM);
        REMINDER_ScheduleNextAlarm(REMINDER_INTERVAL_ALARM);
        REMINDER_EmitStartOfDay();
        status = dayStartedEvent;
    }
    
    if (reminderInfo.flags & REMINDER_END_DAY_ALARM)
    {
        uprint("rem. day ended\n\r");
        REMINDER_ClearInfo();
        reminderInfo.enalbed = false;
        reminderInfo.flags &= (~REMINDER_END_DAY_ALARM);
        REMINDER_ScheduleNextAlarm(REMINDER_NEXT_DAY_ALARM);        
        REMINDER_EmitEndOfDay();
        status = dayEndedEvent;
    }
    
    if (reminderInfo.flags & REMINDER_START_MEASURE_ALARM)
    {
        uprint("rem. day measure started\n\r");
        reminderInfo.flags &= (~REMINDER_START_MEASURE_ALARM);
        REMINDER_ScheduleNextAlarm(REMINDER_NEXT_DAY_ALARM);  
        REMINDER_EmitStartHydrationMeasure();        
    }
    
    if ((reminderInfo.remindersTimeoutTillNextDayOrOpenClose) && 
        (uint64_t)(RTC_GetRawTime() - reminderInfo.remindersTimeoutTillNextDayOrOpenClose > (uint64_t)REMINDER_WINDOW))
    {
        uprint("two hours passed, stop timer alarm until next open or close\n\r");
        reminderInfo.remindersTimeoutTillNextDayOrOpenClose = 0;
        reminderInfo.enalbed = false;
        
        if (REMINDER_isInDayRange())
        {
            REMINDER_ScheduleNextAlarm(REMINDER_END_DAY_ALARM); 
        }
        else
        {
            REMINDER_ScheduleNextAlarm(REMINDER_NEXT_DAY_ALARM); 
        }
    }
    
    if (reminderInfo.flags & REMINDER_SYSTEM_PARAMETERS_CHANGE_EVENT)
    {
        //uprint("REMINDER_SYSTEM_PARAMETERS_CHANGE_EVENT\n\r");
        reminderInfo.flags &= (~REMINDER_SYSTEM_PARAMETERS_CHANGE_EVENT);
        reminderInfo.systemParametersChange = 0;
        REMINDER_readSystemParameters();
        REMINDER_RestartReminder(false);
    }
    
    // 25/4/21 Amir asked me to cancel the 48 hours */ 
    /*if ((reminderInfo.remindersTimeoutTillOpenClose) && (uint64_t)((RTC_GetRawTime() - reminderInfo.remindersTimeoutTillOpenClose)/60/60 > reminderConfig.autoOff))
    {
        uprint("48 hours passed, stop timer alarm until next open or close\n\r");
        reminderInfo.remindersTimeoutTillOpenClose = 0;
        REMINDER_DisableReminder();
    }*/
    
    return status;
}

void REMINDER_Play(void)
{
    if (!reminderInfo.enalbed)
    {
        return;
    }
    
    if (reminderInfo.remindersTimeoutTillNextDayOrOpenClose == 0)
    {
        // reminderStartTime counts the two hours reminders timeout and 48 hours auto off
        reminderInfo.remindersTimeoutTillNextDayOrOpenClose = RTC_GetRawTime();
    }
    
    REMINDER_PlayReminder(REMINDER_CYCLE_ALARM);
}

void REMINDER_EnableAllReminderIndications(void)
{
    reminderConfig.reminderOptionsMask = 0xff;
}
void REMINDER_DisableAllReminderIndications(void)
{
    reminderConfig.reminderOptionsMask = 0x00;
}
/**
* @brief  : this function clears two hours timeout. which means, 
*           after two hours the reminder will stop till open \ close or next day 
* @param  : none
* @param  : none.
* @retval 
*/
void REMINDER_clearReminderShortTimeout(void)
{
    // reminderStartTime counts the two hours reminders timeout and 48 hours auto off
    reminderInfo.remindersTimeoutTillNextDayOrOpenClose = 0;
}

static void REMINDER_PlayReminder(reminderFlags_t reminderFlags)
{
    rtcTimeAndDate_t    timeAndDate;
    RTC_GetTimeAndDate(&timeAndDate); 
    //uint64_t time = RTC_GetUnixTimeAndDate();     // UNUSED
    
    // the application layer might call us every time the person opened the bottle. 
    // so, if we are out of range then do not remind
    if ((timeAndDate.hour >= reminderConfig.endTime) ||
        (timeAndDate.hour < reminderConfig.startTime))
    {
        REMINDER_ScheduleNextAlarm(REMINDER_NEXT_DAY_ALARM);
        return;        
    }
    
    // in case two hours passed, make an alarm at the end of the day.
    if ((reminderInfo.remindersTimeoutTillNextDayOrOpenClose) && (uint64_t)(RTC_GetRawTime() - reminderInfo.remindersTimeoutTillNextDayOrOpenClose >= (uint64_t)REMINDER_WINDOW))
    {
        uprint("REMINDER_PlayReminder - two hours timeout\n\r");
        reminderInfo.remindersTimeoutTillNextDayOrOpenClose = 0;
        REMINDER_ScheduleNextAlarm(REMINDER_END_DAY_ALARM);
        return;
    }
        
    // schedule next alarm.
    REMINDER_ScheduleNextAlarm(reminderFlags);
    
    REMINDER_PlayPlan();    
}

/**
* @brief  : called when time was updated.
* @param  : none
* @param  : none.
* @retval 
*/
static void REMINDER_timeChanged(void)
{
    reminderInfo.flags |= TIME_CHANGED;
}

static void REMINDER_reminderAlarm(void)
{
    reminderInfo.flags |= REMINDER_INTERVAL_ALARM;
}

static void REMINDER_reminderRoundAlarm(void)
{
    reminderInfo.flags |= REMINDER_ROUND_ALARM;
}

static void REMINDER_reminderCycleAlarm(void)
{
    reminderInfo.flags |= REMINDER_CYCLE_ALARM;
}

static void REMINDER_nextDay(void)
{
    reminderInfo.flags |= REMINDER_NEXT_DAY_ALARM;
}

static void REMINDER_endDay(void)
{
    reminderInfo.flags |= REMINDER_END_DAY_ALARM;
}

static void REMINDER_startMeasure(void)
{
    uprint("REMINDER_start Measure\n\r");
    reminderInfo.flags |= REMINDER_START_MEASURE_ALARM;
}

static void REMINDER_systemParametersChanged(flashParam_t flashParam)
{
    reminderInfo.flags |= REMINDER_SYSTEM_PARAMETERS_CHANGE_EVENT;
    reminderInfo.systemParametersChange |= 1<<flashParam;
    //uprint("REMINDER system parameter change event\n\r");
}

static void REMINDER_ClearForNewRound(void)
{
    //uprint("REMINDER_ClearForNewRound\n\r");
    reminderInfo.flags &= REMINDER_MASK;
    reminderInfo.reminderCyclesCounter = 1;    
}

static void REMINDER_ClearForNewInterval(void)
{
    //uprint("REMINDER_ClearForNewInterval\n\r");
    REMINDER_ClearForNewRound();
    reminderInfo.flags &= REMINDER_MASK;
    reminderInfo.reminderRoundsCounter = 1;
}

static void REMINDER_ClearInfo(void)
{
    //uprint("REMINDER_ClearInfo\n\r");
    REMINDER_ClearForNewInterval();
    reminderInfo.reminderTimeStamp = 0;
}

static uint8_t REMINDER_CheckValidity(void)
{
    if (reminderConfig.reminderCycles * reminderConfig.reminderCycleWait > (reminderConfig.reminderRoundWait*60))
    {
        return ERR_PARAMETER_IS_INVALID;
    }
    
    if (reminderConfig.reminderRounds * reminderConfig.reminderRoundWait > (reminderConfig.reminderInterval))
    {
        return ERR_PARAMETER_IS_INVALID;
    }
    
    return OK;
}

static void REMINDER_ScheduleNextAlarm(reminderFlags_t reminderFlags)
{
    uint8_t             res = -1;    
    rtcTimeAndDate_t    timeAndDate;
    rtcTime_t           rtcTime;
    alarmCallback_t     alarmCallback = REMINDER_nextDay;
    
    rtcTime_t rtcStartTime      = {reminderConfig.startTime, 0,0};
    rtcTime_t rtcEndTime        = {reminderConfig.endTime, 0,0};
    rtcTime_t rtcMeasureTime    = {MEASURE_START_TIME,0,0};
        
    RTC_GetTimeAndDate(&timeAndDate);    
    uint64_t time = RTC_GetUnixTimeAndDate();
    
    switch (reminderFlags)
    {
        // NOTE - TIME_CHANGED do not have "break" because i still want to generate interval alarm
        case TIME_CHANGED:
            uprint("TIME_CHANGED\n\r");
            reminderInfo.remindersTimeoutTillOpenClose = reminderInfo.remindersTimeoutTillNextDayOrOpenClose = RTC_GetRawTime();//RTC_GetUnixTimeAndDate();
        case REMINDER_INTERVAL_ALARM:
            REMINDER_ClearInfo();            
            reminderInfo.reminderRoundTimeStamp = reminderInfo.reminderTimeStamp = RTC_GetUnixTimeAndDate();
            time += reminderConfig.reminderInterval*60;
            alarmCallback = REMINDER_reminderAlarm;
            // convert unix time to time and date struct and then check
            RTC_ConvertUnixToTime(time, &rtcTime);   
        break;
        case REMINDER_ROUND_ALARM:
        case REMINDER_CYCLE_ALARM:
            if (reminderInfo.reminderCyclesCounter >= reminderConfig.reminderCycles)
            {
                if (reminderInfo.reminderRoundsCounter >= reminderConfig.reminderRounds)
                {
                    uprint("config timer to next interval\n\r"); 
                    REMINDER_ClearForNewInterval();
                    time = (uint64_t)(reminderInfo.reminderTimeStamp + (uint64_t)(reminderConfig.reminderInterval*60));
                    alarmCallback = REMINDER_reminderAlarm;
                }
                else
                {
                    uprint("config timer to next round\n\r");
                    REMINDER_ClearForNewRound();
                    reminderInfo.reminderRoundsCounter++;
                    time = (uint64_t)(reminderInfo.reminderRoundTimeStamp + (uint64_t)(reminderConfig.reminderRoundWait*60));
                    alarmCallback = REMINDER_reminderRoundAlarm;
                }
            }
            else
            {
                uprint("config timer to next cycle\n\r");
                reminderInfo.reminderCyclesCounter++;
                alarmCallback = REMINDER_reminderCycleAlarm;
                time += reminderConfig.reminderCycleWait;
            }
            // convert unix time to time and date struct and then check
            RTC_ConvertUnixToTime(time, &rtcTime);   
        break;
        case REMINDER_END_DAY_ALARM:
            uprint("set a timer to end day\n\r");
            rtcTime = (rtcTime_t){reminderConfig.endTime, 0,0};
            alarmCallback = REMINDER_endDay;
        break; 
        case REMINDER_NEXT_DAY_ALARM:
            {
                rtcTime_t startDay = (rtcTime_t){reminderConfig.endTime, 0,0};
                rtcTime_t currTime = (rtcTime_t){timeAndDate.hour,timeAndDate.minute,timeAndDate.second};
                if (REMINDER_Time2Seconds(currTime) >= REMINDER_Time2Seconds(rtcMeasureTime) && 
                    REMINDER_Time2Seconds(currTime) < REMINDER_Time2Seconds(startDay))
                {
                    uprint("set a timer to next day\n\r"); 
                    rtcTime = (rtcTime_t){reminderConfig.startTime, 0,0};
                    alarmCallback = REMINDER_nextDay;
                }
                else
                {
                    uprintf("set a timer to %2d:00:00AM\n\r", rtcMeasureTime.hour); 
                    rtcTime = rtcMeasureTime;
                    alarmCallback = REMINDER_startMeasure;
                }
            }
        break; 
        default:
        break;
    }    
    rtcTime_t currTime = (rtcTime_t){timeAndDate.hour,timeAndDate.minute,timeAndDate.second};
    /* 03:00:00AM  < currTime < start time */    
    if ((REMINDER_Time2Seconds(currTime) < REMINDER_Time2Seconds(rtcStartTime)) && 
        (REMINDER_Time2Seconds(currTime) > REMINDER_Time2Seconds(rtcMeasureTime)))  
       
    {/* in case current time and the reqired time to set is between measure time to start day*/
        uprint("out of active day or day ended, set to next day\n\r");
        rtcTime = (rtcTime_t){reminderConfig.startTime, 0,0};
        res = RTC_SetTimeAlarm(rtcTime, REMINDER_nextDay);
    }
    else
    if (REMINDER_Time2Seconds(rtcTime) > REMINDER_Time2Seconds(rtcEndTime) && 
       (REMINDER_Time2Seconds(currTime) < REMINDER_Time2Seconds(rtcEndTime)))
    {/* in case current time is early then end-time but the required time to set is after end-time*/
        reminderInfo.remindersTimeoutTillNextDayOrOpenClose = 0;
        uprint("out of active day or reminders timeout , set to end of day\n\r");
        rtcTime = (rtcTime_t){reminderConfig.endTime, 0,0};
        res = RTC_SetTimeAlarm(rtcTime, REMINDER_endDay);
    }
    else
    if (((REMINDER_Time2Seconds(currTime) > REMINDER_Time2Seconds(rtcEndTime)) ||
        (REMINDER_Time2Seconds(currTime) < REMINDER_Time2Seconds(rtcMeasureTime))))
    { /* in case current time is between end-time to measure-time*/
        
        uprintf("out of active day or day ended, set timer to start measure %2d:00:00AM\n\r", rtcMeasureTime.hour);
        rtcTime = rtcMeasureTime;
        res = RTC_SetTimeAlarm(rtcTime, REMINDER_startMeasure);
    }
    else
    {
        reminderInfo.enalbed = true;
        res = RTC_SetTimeAlarm(rtcTime, alarmCallback);
    }
    
#if 0    
    if ((REMINDER_Time2Seconds(rtcTime) <= REMINDER_Time2Seconds(rtcMeasureTime)) //|| 
       //(REMINDER_Time2Seconds(rtcTime) > REMINDER_Time2Seconds(rtcEndTime)))
    {
        uprintf("out of active day or day ended, set timer to start measure %2d:00:00AM\n\r", rtcMeasureTime.hour);
        rtcTime = rtcMeasureTime;
        res = RTC_SetTimeAlarm(rtcTime, REMINDER_startMeasure);
    }
    else
    if ((REMINDER_Time2Seconds(rtcTime) <= REMINDER_Time2Seconds(rtcStartTime)) ||   
        (timeAndDate.hour < reminderConfig.startTime)) 
    {
        //uprintf("rtcAlarm %d, timeAndDate %d\n\r", rtcTime.hour, timeAndDate.hour); 
        uprint("out of active day or day ended, set to next day\n\r");
        rtcTime = (rtcTime_t){reminderConfig.startTime, 0,0};
        res = RTC_SetTimeAlarm(rtcTime, REMINDER_nextDay);
    }
    else
    /* if current time or the timer we required to set is after end of the day 
       or we got two hours reminders timeout then just set a timer to the end of the day*/
    if (REMINDER_Time2Seconds(rtcTime) > REMINDER_Time2Seconds(rtcEndTime)) //||         
    {
        rtcTime_t currTime = (rtcTime_t){timeAndDate.hour,timeAndDate.minute,timeAndDate.second};
        if (REMINDER_Time2Seconds(currTime) > REMINDER_Time2Seconds(rtcEndTime))
        {
            uprintf("out of active day or day ended, set timer to start measure %2d:00:00AM\n\r", rtcMeasureTime.hour);
            rtcTime = rtcMeasureTime;
            res = RTC_SetTimeAlarm(rtcTime, REMINDER_startMeasure);
        }
        else
        {
            reminderInfo.remindersTimeoutTillNextDayOrOpenClose = 0;
            uprint("out of active day or reminders timeout , set to end of day\n\r");
            rtcTime = (rtcTime_t){reminderConfig.endTime, 0,0};
            res = RTC_SetTimeAlarm(rtcTime, REMINDER_endDay);
        }
    }
    else
    {
        reminderInfo.enalbed = true;
        res = RTC_SetTimeAlarm(rtcTime, alarmCallback);    
    }   
#endif     
    if (res != OK)
    {
        uprint("alarm was not set due to an RTC error\n\r");
    }
}

static uint8_t REMINDER_isInDayRange(void)
{
    rtcTimeAndDate_t timeAndDate;
    
    RTC_GetTimeAndDate(&timeAndDate);
    
    if ((timeAndDate.hour >= reminderConfig.startTime) && 
        (timeAndDate.hour < reminderConfig.endTime))
    {
        return true;
    }
        
    return false;
}

static uint64_t REMINDER_Time2Seconds(rtcTime_t rtcTime)
{
    return ((rtcTime.hour*60*60)+(rtcTime.minute*60)+rtcTime.second);
}

int32_t REMINDER_getTimePassedSinceStartDay(void)
{
    rtcTime_t           startDayTime;
    rtcTime_t           rtcCurrentTime;
    rtcTimeAndDate_t    timeAndDate;
    int32_t             dayOffset = 0;

    RTC_GetTimeAndDate(&timeAndDate);

    rtcCurrentTime  = (rtcTime_t){timeAndDate.hour, timeAndDate.minute, timeAndDate.second};
    startDayTime    = (rtcTime_t){reminderConfig.startTime, 0,0};
    if (timeAndDate.hour < reminderConfig.startTime)
    {
        dayOffset = 24*60*60; // total seconds in one day
    }

    return (int32_t)(dayOffset + (int32_t)REMINDER_Time2Seconds(rtcCurrentTime) - (int32_t)REMINDER_Time2Seconds(startDayTime));
}

static void REMINDER_readSystemParameters(void)
{
    /*PARAM_START_TIME,
    PARAM_END_TIME,
    PARAM_REMINDER_UI_ELEMENTS,
    PARAM_REMINDER_LEDS_PATTERN,
    PARAM_SOUND_VOLUME,
    PARAM_SOUND_TYPE,
    PARAM_VIBRATION_LENGTH,
    PARAM_REMINDER_LEDS_COLOR,
    PARAM_REMINDER_INTERVAL,
    PARAM_REMINDER_ROUNDS,
    PARAM_REMINDER_ROUNDS_WAIT,
    PARAM_REMINDER_CYCLES,
    PARAM_REMINDER_CYCLES_WAIT,    
    */
    UserFlash_readParam(PARAM_REMINDER_INTERVAL, (void*) &reminderConfig.reminderInterval , sizeof(reminderConfig.reminderInterval));
    UserFlash_readParam(PARAM_REMINDER_ROUNDS, (void*) &reminderConfig.reminderRounds, sizeof(reminderConfig.reminderRounds));
    UserFlash_readParam(PARAM_REMINDER_ROUNDS_WAIT, (void*) &reminderConfig.reminderRoundWait, sizeof(reminderConfig.reminderRoundWait));
    UserFlash_readParam(PARAM_REMINDER_CYCLES, (void*) &reminderConfig.reminderCycles, sizeof(reminderConfig.reminderCycles));
    UserFlash_readParam(PARAM_REMINDER_CYCLES_WAIT, (void*) &reminderConfig.reminderCycleWait, sizeof(reminderConfig.reminderCycleWait));
    UserFlash_readParam(PARAM_REMINDER_INTERVAL, (void*) &reminderConfig.reminderInterval , sizeof(reminderConfig.reminderInterval));
    UserFlash_readParam(PARAM_START_TIME, (void*) &reminderConfig.startTime , sizeof(reminderConfig.startTime));
    UserFlash_readParam(PARAM_END_TIME, (void*) &reminderConfig.endTime , sizeof(reminderConfig.endTime));
    UserFlash_readParam(PARAM_REMINDER_UI_ELEMENTS, (void*) &reminderConfig.reminderOptions , sizeof(reminderConfig.reminderOptions));
    UserFlash_readParam(PARAM_REMINDER_LEDS_PATTERN, (void*) &reminderConfig.reminderLedsPattern , sizeof(reminderConfig.reminderLedsPattern));

    LEDS_readSystemParameters();
}

uint8_t REMINDER_RegisterEndOfDay(endOfDayCallback_t endOfDayCallback)
{
    for (int i = 0 ; i < END_OF_DAY_NUM_OF_CALLBACKS ; i++)
    {
        if (endOfDayCallbackArray[i] == NULL)
        {
            endOfDayCallbackArray[i] = endOfDayCallback;
            return OK;
        }
    }
    return ERR_UNAVAILABLE_RESOURCE;
}

uint8_t REMINDER_UnregisterEndOfDay(endOfDayCallback_t endOfDayCallback)
{
    for (int i = 0 ; i < END_OF_DAY_NUM_OF_CALLBACKS ; i++)
    {
        if (endOfDayCallbackArray[i] == endOfDayCallback)
        {
            endOfDayCallbackArray[i] = NULL;
            return OK;
        }
    }
    return ERR_OBJ_NOT_FOUND;
}

static uint8_t REMINDER_EmitEndOfDay(void)
{
    for (int i = 0 ; i < END_OF_DAY_NUM_OF_CALLBACKS ; i++)
    {
        if (endOfDayCallbackArray[i] != NULL)
        {
            endOfDayCallbackArray[i]();
        }
    }
    return OK;
}

uint8_t REMINDER_RegisterStartOfDay(startOfDayCallback_t startOfDayCallback)
{
    for (int i = 0 ; i < END_OF_DAY_NUM_OF_CALLBACKS ; i++)
    {
        if (startOfDayCallbackArray[i] == NULL)
        {
            startOfDayCallbackArray[i] = startOfDayCallback;
            return OK;
        }
    }
    return ERR_UNAVAILABLE_RESOURCE;
}

uint8_t REMINDER_UnregisterStartOfDay(startOfDayCallback_t startOfDayCallback)
{
    for (int i = 0 ; i < END_OF_DAY_NUM_OF_CALLBACKS ; i++)
    {
        if (startOfDayCallbackArray[i] == startOfDayCallback)
        {
            startOfDayCallbackArray[i] = NULL;
            return OK;
        }
    }
    return ERR_OBJ_NOT_FOUND;
}

static uint8_t REMINDER_EmitStartOfDay(void)
{
    for (int i = 0 ; i < END_OF_DAY_NUM_OF_CALLBACKS ; i++)
    {
        if (startOfDayCallbackArray[i] != NULL)
        {
            startOfDayCallbackArray[i]();
        }
    }
    return OK;
}

uint8_t REMINDER_RegisterStartHydrationMeasure(startHydrationMeasureCallback_t startHydrationMeasureCallback)
{
    for (int i = 0 ; i < START_HYD_MEASURE_NUM_OF_CALLBACKS ; i++)
    {
        if (startHydrationMeasureCallbackArray[i] == NULL)
        {
            startHydrationMeasureCallbackArray[i] = startHydrationMeasureCallback;           
            return OK;
        }
    }   
    return ERR_UNAVAILABLE_RESOURCE;
}

uint8_t REMINDER_UnregisterStartHydrationMeasure(startHydrationMeasureCallback_t startHydrationMeasureCallback)
{
    for (int i = 0 ; i < START_HYD_MEASURE_NUM_OF_CALLBACKS ; i++)
    {
        if (startHydrationMeasureCallbackArray[i] == startHydrationMeasureCallback)
        {
            startHydrationMeasureCallbackArray[i] = NULL;
            return OK;
        }
    }
    return ERR_OBJ_NOT_FOUND;
}

static uint8_t REMINDER_EmitStartHydrationMeasure(void)
{
    for (int i = 0 ; i < START_HYD_MEASURE_NUM_OF_CALLBACKS ; i++)
    {
        if (startHydrationMeasureCallbackArray[i] != NULL)
        {
            startHydrationMeasureCallbackArray[i]();            
        }
    }    
    return OK;
}

uint8_t REMINDER_RegisterReminderInterval(reminderIntervalCallback_t reminderIntervalCallback)
{
    for (int i = 0 ; i < REMINDER_INTERVAL_NUM_OF_CALLBACKS ; i++)
    {
        if (reminderIntervalCallbackArray[i] == NULL)
        {
            reminderIntervalCallbackArray[i] = reminderIntervalCallback;           
            return OK;
        }
    }   
    return ERR_UNAVAILABLE_RESOURCE;
}

uint8_t REMINDER_UnregisterReminderInterval(reminderIntervalCallback_t reminderIntervalCallback)
{
    for (int i = 0 ; i < REMINDER_INTERVAL_NUM_OF_CALLBACKS ; i++)
    {
        if (reminderIntervalCallbackArray[i] == reminderIntervalCallback)
        {
            reminderIntervalCallbackArray[i] = NULL;
            return OK;
        }
    }
    return ERR_OBJ_NOT_FOUND;
}

static uint8_t REMINDER_EmitReminderInterval(void)
{
    for (int i = 0 ; i < REMINDER_INTERVAL_NUM_OF_CALLBACKS ; i++)
    {
        if (reminderIntervalCallbackArray[i] != NULL)
        {
            reminderIntervalCallbackArray[i]();            
        }
    }    
    return OK;
}
#ifdef __cplusplus
}
#endif
