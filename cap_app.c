
#ifdef __cplusplus
 extern "C" {
#endif

#include <CyLib.h>
#include "cap_app.h"
#include "init_app.h"
#include "../App/main.h"
#include "../Modules/CLI_Module.h"
#include "../Modules/debugPort_Module.h"
#include "../Modules/flash_Module.h"
#include "../Modules/RTC_Module.h"
#include "../Modules/Log_Module.h"
#include "../Modules/reminder.h"
#include "../Modules/OCD_Module.h"
#include "../Modules/HYD_Detection.h"
#include "../Modules/measurement_Module.h"
#include "../Modules/Timer_Engine.h"
#include "../Peripheral/GyroSwitch_Module.h"
#include "../Peripheral/accelerometer.h"
#include "../Peripheral/ToF_Module_L3.h"
#include "../Peripheral/LEDSBasicPlans.h"
#include "../Peripheral/charger.h"
#include "../Peripheral/PushButton_Module.h"
#include "../BLE/BLE_main.h"
#include "../BLE/BLE_Process.h"
#include "../Drivers/DoF_Driver.h"

unsigned char st_version_bootloader_OTA[16]  = "BOOTL#0004";
// notification delay after connection in order to avoid the bug we have
// when getRTC response is sent because periodic notification.
#define NOTIFICATION_DELAY              (2)
#define PAIRING_PROCCESS_TIMEOUT        (30) 
// these are the events to handle in this module
typedef enum
{
    APP_IDLE_EVENT                      = (uint32_t)(0x000000),                                               
    SYS_TEMPERATURE_OVER_RANGE_EVENT    = (uint32_t)(0x000001),
    SYS_BATTERY_VERY_LOW_EVENT          = (uint32_t)(0x000002),
    SYS_BATTERY_LOW_EVENT               = (uint32_t)(0x000004),
    SYS_BATTERY_NORMAL_EVENT            = (uint32_t)(0x000008),
    SYS_CHARGER_PLUGGED                 = (uint32_t)(0x000010),
    SYS_CHARGER_UNPLUGGED               = (uint32_t)(0x000020),
    SYS_BLE_CONNECTION_EVENT            = (uint32_t)(0x000040),
    SYS_BLE_DISCONNECTION_EVENT         = (uint32_t)(0x000080),
    SYS_BLE_PAIRING_REQUEST_EVENT       = (uint32_t)(0x000100),
    SYS_BLE_PAIRING_SUCCEED_EVENT       = (uint32_t)(0x000200),
    SYS_BLE_PAIRING_RETRY_EVENT         = (uint32_t)(0x000400),
    PB_ONE_SHORT_CLICK_EVENT            = (uint32_t)(0x000800),
    PB_ONE_LONG_CLICK_EVENT             = (uint32_t)(0x001000),
    PB_ONE_VERY_LONG_CLICK              = (uint32_t)(0x002000),
    PB_DOUBLE_CLICK_EVENT               = (uint32_t)(0x004000),
    OCD_NEW_OPEN_EVENT                  = (uint32_t)(0x008000),
    OCD_NEW_CLOSE_EVENT                 = (uint32_t)(0x010000),
    //LED_TEST_PRODUCTION_EVENT           = (uint32_t)(0x020000),
    RUN_SET_DARK_ACTIVE_EVENT           = (uint32_t)(0x020000),
    RUN_HYDRATION_STATUS_EVENT          = (uint32_t)(0x040000),
    BATTERY_SERVICE_EVENT               = (uint32_t)(0x080000),
    CAP_SYSTEM_PARAMETERS_CHANGE_EVENT  = (uint32_t)(0x100000),
    SYS_BLE_PAIRING_TIMEOUT             = (uint32_t)(0x200000),
    SYS_PERIODIC_ADVERTISE              = (uint32_t)(0x400000),
    SYS_MEAS_READY                      = (uint32_t)(0x800000),
    SYS_MEAS_NOT_VALID                  = (uint32_t)(0x1000000),
    LEDS_OFF_AFTER_CLOSE_TIMEOUT_EVENT  = (uint32_t)(0x2000000),
    CLOSE_STABILITY_TIMEOUT_EVENT       = (uint32_t)(0x4000000),
    BLE_PAIRING_PLAN_COMPLETE_SUCCESS_EVENT  = (uint32_t)(0x8000000),
    SYS_REMINDER_INTERVAL_EVENT         = (uint32_t)(0x10000000),
    SYS_BOTTLE_FILLED_EVENT             = (uint32_t)(0x20000000),
    USER_ANSWER_TIMEOUT_EVENT           = (uint32_t)(0x40000000),
    START_NOTIFICATION_AFTER_CONNECTION = (uint32_t)(0x80000000),
}applicationEvents_t;

typedef enum
{
    reminderEvent,
    closeEvent
}measureReason_t;

typedef enum
{
    NORMAL,
    BATTERY_LOW,
    SAVE_MODE
}POWER_MODE_t;

typedef enum
{
    ACTIVE,
    DARK
}APP_MODE_t;

typedef enum
{
    BLE_IDLE,
    BLE_PAIRING_IN_PROCESS,
    BLE_CONNECTED,
}BLE_STATE_t;

typedef enum
{
    APP_CRITICAL_TEMPERATURE_STATE     = 0, // do nothing
    APP_BATTERY_VERY_LOW_STATE         = 1, // only battery level measure
    APP_PAIRING_PROCESS_STATE          = 2, // pairing process, if paired - open\close working but no leds plans
    APP_CHARGING_STATE                 = 3, // charging display,if paired - open\close working but no leds plans 
    APP_PAUSED_STATE                   = 4, // do nothing, for device not paired
    APP_OPERATION_STATE                = 5, // everything is working
    APP_MAX_STATES                     = 6,
}APP_MAIN_STATE_t;

uint8_t st_go_to_bootloader      = 0;
uint8_t st_make_reset            = 0;
uint8_t st_make_delay_for_wdt    = 0;

typedef struct
{
    uint16_t        pairingRetriesCount;
    timerHandler_t  timerHandler;
}pairingInfo_t;

typedef struct
{
    uint64_t            systemParametersChange; // bitmap represent which system parameter was updated via CLI\BLE
    uint32_t            flags;                  // these flag represents events
    MEAS_STATUS_t       lastMeasureStatus;      // measure status
    measureReason_t     measureReason;          // we use it just for debug (debug log message )
    APP_MODE_t          appMode;
    advertiseMessage_t  advertiseMessage;
    lowBatteryState_t   lowBatteryState;
    APP_MAIN_STATE_t    appState;
    pairingInfo_t       pairingInfo;
    timerHandler_t      advertiseTimerHandler;
    timerHandler_t      stabilityCloseTimerHandler;
    timerHandler_t      ledsOffAfterCloseTimerHandler;
    timerHandler_t      notificationDelayTimerHandler;
    OCD_State_t         bottleState;
    bool                notificationEnable;
}appInfo_t;

typedef struct
{
    uint8_t                     waitTillMeasure;
    uint16_t                    advertiseInterval;
    uint16_t                    advertisePeriod;
    uint8_t                     paired;                 // indicate if device is paired already
    bool                        liquidDetection;
    reminderLogic_t             reminderLogic;
}appConfig_t;

static appInfo_t appInfo = {
    .flags                      = 0,
    .systemParametersChange     = 0,
    .lastMeasureStatus          = MEAS_NOT_VALID,
    .measureReason              = reminderEvent,
    .appMode                    = ACTIVE,
    .advertiseMessage           =
    {
        .event                  = 0,
        .lastMeasurement        = 0,
        .lastMeasurementStd     = 0,
        .logCounter             = 0,
        .reminder               = 0,
        .pairingStatus          = PAIRING_STATUS_NOT_PAIRED,
        .advertiseVersion       = ADVERTISE_VERSION,
        .rfu                    = 0
    },
    .lowBatteryState               = lowBattery_20,
    .pairingInfo                = {0,0},
    .appState                   = APP_OPERATION_STATE,
    .advertiseTimerHandler      = 0,
    .stabilityCloseTimerHandler = 0,
    .ledsOffAfterCloseTimerHandler = 0,
    .notificationDelayTimerHandler = 0,
    .notificationEnable         = false,
    .bottleState                = Open,
};

static appConfig_t appConfig =
{
    .waitTillMeasure            = WAIT_TILL_MEASURE,
    .advertiseInterval          = ADVERTISE_INTERVAL,
    .advertisePeriod            = ADVERTISE_PERIOD,
    .paired                     = false,
    .reminderLogic              = whenBehind,
};

extern uint8_t                  g_TxIntOccured;
extern uint8_t                  st_go_sleep;
extern unsigned long            st_idle_timeout_till;
extern uint16                   mtuSize; // Packet size is (mtuSize - 3), max packet size is 247, iOS 8 is 135, iOS 9 is 158 and iOS 10 is 185; use CyBle_GattGetMtuSize()

static void Cap_readSystemParameters(void);
static void CAP_HandleHydration(uint8_t reminderOn, measureReason_t measureReason);
static int  Cap_handleDisconnectEvent(void);
/*static*/ void Cap_ButtonPress_Event_Handler(PB_1_CLICK_TYPE_t pb_1_clickType);
static void Cap_ShowHydrationLevel(bool longDuration);
static void Cap_pairingTimeout(void);
static void Cap_pairingRequest(void);
static void Cap_pairingRetry(void);
static void Cap_periodicAdvertiseEvent(void);
static void Cap_bottleCloseStabilityEvent(void);
static void Cap_ledsOffAfterCloseEvent(void);
static void CAP_handleBottleEvents(appInfo_t* appInfo, bool displayOn);
static void Cap_pairingCompleteLedsPlanTimeout(void);
static void Cap_userAnswerTimeout(void);
static void Cap_notificationAfterConnetionDelayTimeout(void);
static APP_MAIN_STATE_t appGetState(void);
static bool CAP_isTemperatureOutOfRange(void);

// this is a list of system parameters that this module should be aware if they were changed
// CLI or BLE Commands.
static const flashParam_t systemParametersList[] =
{
    PARAM_WAIT_TILL_MEASURE,
    PARAM_ADVERTISE_INTERVAL,
    PARAM_ADVERTISE_PERIOD,
    PARAM_PAIRING_STATE,
    //PARAM_REMINDER_LOGIC
};

int Cap_OffLineEvent(int id)
{
    switch (id)
    {
        case 3:
            DoF_LatchWakeUp();
            return 1;
        case 4:
            if (!GyroSwitch_WakeUpByAcc)
                DoF_LatchWakeUp();
            return 1;
        default:
            break;
    }
    return 0;
}

static errorCode_t Cap_CollectAdvertiseMessage(advertiseMessage_t* a_advertiseMessage, uint8_t pairingStatus)
{
    if (a_advertiseMessage == NULL)
    {
        return ERR_NULL_OBJ;
    }

    MEAS_RESULT_t p_meas_result;
    MEAS_GetResult(&p_meas_result);

    a_advertiseMessage->pairingStatus = pairingStatus;
    a_advertiseMessage->logCounter = GeneralEvents_GetCount();
    a_advertiseMessage->event = OCD_isClose() ? GeneralEvent_Code_Close : GeneralEvent_Code_Open;

    if (p_meas_result.isValid)
    {
        a_advertiseMessage->lastMeasurement = p_meas_result.range;
        a_advertiseMessage->lastMeasurementStd = p_meas_result.std;
    }
    return OK;
}

/* ***************************************  critical temperature handlers */
static void appCriticalTemperatureEntry(void)
{
    uprintf("enter to APP_CRITICAL_TEMPERATURE_STATE state\n\r");

    REMINDER_DisableReminder();
    LEDS_Stop();
    MEAS_Stop();
    // stop BLE
    CapDisconnect();
    CapStopPairing();
    LogGeneralEvent(GeneralEvent_Code_Critical_Temperature);
}

static void appCriticalTemperatureExit(void)
{
    uprintf("exit from APP_CRITICAL_TEMPERATURE_STATE state\n\r");
    LogGeneralEvent(GeneralEvent_Code_Normal_Temperature);
}

static APP_MAIN_STATE_t appCriticalTemperatureHandler(appInfo_t* appInfo)
{
    APP_MAIN_STATE_t nextState = APP_CRITICAL_TEMPERATURE_STATE;
    if (!CAP_isTemperatureOutOfRange())
    {
        nextState = APP_OPERATION_STATE;
    }
    //uprintf("APP EVENT: %d\n\r", appInfo->flags);
    return nextState;
}

/* *************************************  BATTERY VERY LOW handlers */
static void appBatteryVeryLowEntry(void)
{
    uprintf("enter to APP_BATTERY_VERY_LOW_STATE state\n\r");


    REMINDER_DisableReminder();
    //LEDS_ShowLowBatteryLevel(lowBattery_3);
    // stop BLE
    CapDisconnect();

    LogGeneralEvent(GeneralEvent_Code_VeryLowPowerMode);
    //appInfo.pairingInfo.endPairingTime = 0;     //in order to stop pairing timeout
    //appInfo.BLEState = BLE_IDLE;
}

static void appBatteryVeryLowExit(void)
{
    uprintf("exit from APP_BATTERY_VERY_LOW_STATE state\n\r");
}

static APP_MAIN_STATE_t appBatteryVeryLowHandler(appInfo_t* appInfo)
{
    APP_MAIN_STATE_t nextState = APP_BATTERY_VERY_LOW_STATE;

    if (appInfo->flags & PB_ONE_SHORT_CLICK_EVENT)
    {
        appInfo->flags &= (~PB_ONE_SHORT_CLICK_EVENT);
        LEDS_ShowLowBatteryLevel(lowBattery_5);
    }
    
    if(appInfo->flags & SYS_CHARGER_PLUGGED)
    {
        uprintf("SYS_CHARGER_PLUGGED\n\r");
        appInfo->flags &= (~SYS_CHARGER_PLUGGED);
        nextState = APP_CHARGING_STATE;
    }
    
    if (appInfo->flags & SYS_BATTERY_NORMAL_EVENT)
    {
        uprintf("SYS_BATTERY_NORMAL_EVENT\n\r");
        appInfo->flags &= (~SYS_BATTERY_NORMAL_EVENT);
        nextState = APP_OPERATION_STATE;
    }
    return nextState;
}
/* ************************************** CHARGING state handlers */
static void appChargingEntry(void)
{
    uprintf("enter to APP_CHARGING_STATE state\n\r");
    REMINDER_DisableReminder();
    REMINDER_DisableAllReminderIndications();
    
    /* IPP 4161 */
    //if (!BATTERY_isBatteryVeryLow())
    {
        CHARGER_displayBatteryStatus(true);
    }

    // for periodic advertise
    if (appConfig.paired)
    {
        // force immediate Advertise
        Cap_periodicAdvertiseEvent();

        if (!TIMER_isExist(appInfo.advertiseTimerHandler))
        {
            TIMER_Start(Cap_periodicAdvertiseEvent, appConfig.advertisePeriod, periodic, &appInfo.advertiseTimerHandler);
        }        
    }
    
    //LogGeneralEvent(GeneralEvent_Code_ChargeStart);
}
static void appChargingExit(void)
{
    uprintf("exit from APP_CHARGING_STATE state\n\r");
    REMINDER_EnableAllReminderIndications();
    CHARGER_displayBatteryStatus(false);
    if (appConfig.paired)
    {
        TIMER_Stop(appInfo.advertiseTimerHandler);
    }
    
    // sometimes the S< exit this state even that charger is still connected, for example pairing process state.
    /*if (CHARGER_getChargerState() == chargerUnplugged)
    {
        LogGeneralEvent(GeneralEvent_Code_ChargeStop);
    }*/
}

static APP_MAIN_STATE_t appChargingHandler(appInfo_t* appInfo)
{
    APP_MAIN_STATE_t nextState = APP_CHARGING_STATE;
 
    if (appInfo->flags & SYS_BATTERY_NORMAL_EVENT)
    {
        uprintf("SYS_BATTERY_NORMAL_EVENT\n\r");
        appInfo->flags &= (~SYS_BATTERY_NORMAL_EVENT);
        CHARGER_displayBatteryStatus(true);
    }
    
    if (appInfo->flags & BATTERY_SERVICE_EVENT)
    {
        uprintf("RUN_BATTERY_SERVICE_EVENT\n\r");
        appInfo->flags &= (~BATTERY_SERVICE_EVENT);

    }
    
    if(appInfo->flags & SYS_CHARGER_UNPLUGGED)
    {
        uprintf("SYS_CHARGER_UNPLUGGED\n\r");
        appInfo->flags &= (~SYS_CHARGER_UNPLUGGED);
        nextState = APP_OPERATION_STATE;
    }
    
    if  (!appConfig.paired)
    {    
        if ((appInfo->flags & PB_ONE_SHORT_CLICK_EVENT) ||
            (appInfo->flags & PB_ONE_LONG_CLICK_EVENT) ||
            (appInfo->flags & PB_ONE_VERY_LONG_CLICK) ||
            (appInfo->flags & PB_DOUBLE_CLICK_EVENT))
        {
            appInfo->flags &= (~(PB_ONE_SHORT_CLICK_EVENT | PB_ONE_LONG_CLICK_EVENT | PB_ONE_VERY_LONG_CLICK | PB_DOUBLE_CLICK_EVENT));
            uprintf("any press - SYS_BLE_PAIRING_REQUEST_EVENT\n\r");
            nextState = APP_PAIRING_PROCESS_STATE;
        }
    }
    else
    {
        if (appInfo->flags & PB_ONE_VERY_LONG_CLICK)
        {
            appInfo->flags &= (~PB_ONE_VERY_LONG_CLICK); 
            uprintf("long press - SYS_BLE_PAIRING_REQUEST_EVENT\n\r");
            nextState = APP_PAIRING_PROCESS_STATE;
        }
    }
    
    // in order to handle open / close events
    if (appConfig.paired)
    {
        CAP_handleBottleEvents(appInfo, false);
    }
    
    return nextState;
}

/* **************************************  PAUSED  state handlers */
static void appPausedEntry(void)
{
    uprintf("enter to APP_PAUSED_STATE state\n\r");
    REMINDER_DisableReminder();
    REMINDER_DisableAllReminderIndications();
    DoF_SetMode(DOF_MODE_SLEEP);
}

static void appPausedExit(void)
{
    uprintf("exit from APP_PAUSED_STATE state\n\r");
}

static APP_MAIN_STATE_t appPausedHandler(appInfo_t* appInfo)
{
    APP_MAIN_STATE_t nextState = APP_PAUSED_STATE;
    
    if ((appInfo->flags & PB_ONE_SHORT_CLICK_EVENT) ||
        (appInfo->flags & PB_ONE_LONG_CLICK_EVENT) ||
        (appInfo->flags & PB_ONE_VERY_LONG_CLICK) ||
        (appInfo->flags & PB_DOUBLE_CLICK_EVENT))
        {
            appInfo->flags &= (~(PB_ONE_SHORT_CLICK_EVENT | PB_ONE_LONG_CLICK_EVENT | PB_ONE_VERY_LONG_CLICK | PB_DOUBLE_CLICK_EVENT));
            uprintf("any press - SYS_BLE_PAIRING_REQUEST_EVENT\n\r");
            nextState = APP_PAIRING_PROCESS_STATE;
        }
    
    if(appInfo->flags & SYS_CHARGER_PLUGGED)
    {
        uprintf("SYS_CHARGER_PLUGGED\n\r");
        appInfo->flags &= (~SYS_CHARGER_PLUGGED);
        nextState = APP_CHARGING_STATE;
    }
    return nextState;
}

/* pairing in process  state handlers */
static void appPairingProcessEntry(void)
{
    uint64_t now = RTC_GetUnixTimeAndDate();
    uprintf("enter to APP_PAIRING_PROCESS_STATE state\n\r");
    
    // during pairing process, reminder is active but we do not want to show reminder's rounds or cycles
    REMINDER_DisableAllReminderIndications();

    // clear retries count
    appInfo.pairingInfo = (pairingInfo_t){0,0};

    // first, build the message to be sent
    if (OK == Cap_CollectAdvertiseMessage(&appInfo.advertiseMessage, PAIRING_STATUS_IN_PROCESS))
    {
        Cap_setAdvertiseMessage(&appInfo.advertiseMessage);
    }

    LEDS_Play(LED_PLAN_ID_PAIRING);
    
    TIMER_Stop(appInfo.pairingInfo.timerHandler);
    if (OK != CapStartPairing(LONG_PAIRING_DURATION))
    {
        TIMER_Start(Cap_pairingRetry, PAIRING_RETRY_TIMEOUT, oneShot, &appInfo.pairingInfo.timerHandler);
    }
    else
    {
        TIMER_Start(Cap_pairingTimeout, LONG_PAIRING_DURATION, oneShot, &appInfo.pairingInfo.timerHandler);  
        uprintf("pairing duration is %d\n\r", LONG_PAIRING_DURATION);
    }    
}

static void appPairingProcessExit(void)
{
    uprint("exit from APP_PAIRING_PROCESS_STATE state\n\r");
    CapStopPairing();

    //REMINDER_EnableAllReminderIndications();
}

static APP_MAIN_STATE_t appPairingProcessHandler(appInfo_t* appInfo)
{
    APP_MAIN_STATE_t nextState = APP_PAIRING_PROCESS_STATE;
    uint64_t now = RTC_GetUnixTimeAndDate();

    //when pairings failed -try again every 1 second.
    //todo :ble pairing retires should be part of the BLE module.
    if (appInfo->flags & SYS_BLE_PAIRING_RETRY_EVENT)
    {
        appInfo->flags &= (~SYS_BLE_PAIRING_RETRY_EVENT);
        if (++appInfo->pairingInfo.pairingRetriesCount <= MAX_PAIRING_RETRIES)
        {
            TIMER_Stop(appInfo->pairingInfo.timerHandler);
            if (OK != CapStartPairing(LONG_PAIRING_DURATION))
            {
                TIMER_Start(Cap_pairingRetry, PAIRING_RETRY_TIMEOUT, oneShot, &appInfo->pairingInfo.timerHandler);
            }
            else
            {
                TIMER_Start(Cap_pairingTimeout, LONG_PAIRING_DURATION, oneShot, &appInfo->pairingInfo.timerHandler);  
                uprintf("pairing duration is %d\n\r", LONG_PAIRING_DURATION);
            }
        }
        else
        {
            uprintf("stop pairing retires %d\n\r", appInfo->pairingInfo.pairingRetriesCount);
            LEDS_Stop();
            nextState = APP_OPERATION_STATE;
        }
    }

    if (appInfo->flags & SYS_BLE_PAIRING_SUCCEED_EVENT)
    {
        appInfo->flags &= (~SYS_BLE_PAIRING_SUCCEED_EVENT);
        uprint("SYS_BLE_PAIRING_SUCCEED_EVENT\n\r");
        TIMER_Stop(appInfo->pairingInfo.timerHandler);
        appInfo->pairingInfo.timerHandler = 0;
        uint8_t pairingState = true;
        UserFlash_writeParam(PARAM_PAIRING_STATE, (void*)&pairingState, true);
        LEDS_Play(LED_PLAN_ID_PAIRING_COMPLETE);
        LogMeasureEvent(GeneralEvent_Code_Pair, 0,0);
        TIMER_Start(Cap_pairingCompleteLedsPlanTimeout, 3, oneShot, &appInfo->pairingInfo.timerHandler);  
        // force immediate Advertise
        Cap_periodicAdvertiseEvent();
    }
            
    if (appInfo->flags & SYS_BLE_PAIRING_TIMEOUT)
    {
        appInfo->flags &= (~SYS_BLE_PAIRING_TIMEOUT);
        //uprint("SYS_BLE_PAIRING_TIMEOUT\n\r");
        LEDS_StopSpecificPlan(LED_PLAN_ID_PAIRING);
        nextState =  APP_OPERATION_STATE;        
    }
    
    if (appInfo->flags & BLE_PAIRING_PLAN_COMPLETE_SUCCESS_EVENT)
    {
        appInfo->flags &= (~BLE_PAIRING_PLAN_COMPLETE_SUCCESS_EVENT);
        uprint("BLE_PAIRING_PLAN_COMPLETE_SUCCESS_EVENT\n\r");
        nextState =  APP_OPERATION_STATE;
    }
    
    // in case of connection, we will not exit immediately because
    // we will wait for pairing process to be completed, if not paired yet
    // or we will let the user to approve that this the device he want to pair
    
    // in case of paired mobile app: 
    // 2. exit after a while since pairing success event will not arrive
    if (appInfo->flags & SYS_BLE_CONNECTION_EVENT)
    {
        uprint("SYS_BLE_CONNECTION_EVENT while in pairing process\n\r");
        appInfo->flags &= (~SYS_BLE_CONNECTION_EVENT);
        UserFlash_BLEConnectedEvent();
        LEDS_StopSpecificPlan(LED_PLAN_ID_PAIRING);
        LogGeneralEvent(GeneralEvent_Code_AppConnected);
        TIMER_Stop(appInfo->pairingInfo.timerHandler);
        TIMER_Start(Cap_userAnswerTimeout, PAIRING_PROCCESS_TIMEOUT, oneShot, &appInfo->pairingInfo.timerHandler);        
    }

    // when the user pressed NO for approval question then we will stay 15 second until exit this state
    if (appInfo->flags & SYS_BLE_DISCONNECTION_EVENT)
    {
        appInfo->flags &= (~SYS_BLE_DISCONNECTION_EVENT);
        uprint("SYS_BLE_DISCONNECTION_EVENT\n\r");
        LEDS_StopSpecificPlan(LED_PLAN_ID_PAIRING_FOUND);
        UserFlash_BLEDisconnectedEvent();
        LogGeneralEvent(GeneralEvent_Code_AppDisconnected);
    }
    
    // we handle two cases:
    // 1. the user didn't approve that this the device he want to paired
    // 2. the user long press to start pairing but, he opened the mobile app while he already was paired before.
    if (appInfo->flags & USER_ANSWER_TIMEOUT_EVENT)
    {
        appInfo->flags &= (~USER_ANSWER_TIMEOUT_EVENT);
        uprint("USER_ANSWER_TIMEOUT_EVENT\n\r");
        LEDS_StopSpecificPlan(LED_PLAN_ID_PAIRING);
        //CapDisconnect();
        if (appConfig.paired)
        {
             TIMER_Start(Cap_notificationAfterConnetionDelayTimeout, NOTIFICATION_DELAY, oneShot, &appInfo->notificationDelayTimerHandler);
        }
        nextState =  APP_OPERATION_STATE;
    }
    
    // in order to handle open / close events
    if (appConfig.paired)
    {
        CAP_handleBottleEvents(appInfo, false);
    }
    return nextState;
}

static void  appOperationEntry(void)
{
    uprint("enter to APP_OPERATION_STATE\n\r");
    // force immediate Advertise
    Cap_periodicAdvertiseEvent();
    // start the periodic timer for advertise
    if (!TIMER_isExist(appInfo.advertiseTimerHandler))
    {
        TIMER_Start(Cap_periodicAdvertiseEvent, appConfig.advertisePeriod, periodic, &appInfo.advertiseTimerHandler);
    }
    REMINDER_EnableAllReminderIndications();
    REMINDER_RestartReminder(false);
    OCD_EntryToOperationMode();
}

static void  appOperationExit(void)
{
    TIMER_Stop(appInfo.advertiseTimerHandler);
    uprint("exit from APP_OPERATION_STATE\n\r");
}

static APP_MAIN_STATE_t appOperationHandler(appInfo_t* appInfo)
{ 
    APP_MAIN_STATE_t nextState = APP_OPERATION_STATE;
    
    if (appInfo->flags & PB_ONE_VERY_LONG_CLICK)
    {
        appInfo->flags &= (~PB_ONE_VERY_LONG_CLICK);
        if (CyBle_GetState() != CYBLE_STATE_CONNECTED)
        {
            uprint("SYS_BLE_PAIRING_REQUEST_EVENT\n\r");
            nextState = APP_PAIRING_PROCESS_STATE;
        }
        else
        {
            uprint("can't enter pairing mode since device is allready connected\n\r");
        }
    }
    
    if(appInfo->flags & PB_ONE_SHORT_CLICK_EVENT)//RUN_HYDRATION_STATUS_EVENT)
    {
        appInfo->flags &= (~PB_ONE_SHORT_CLICK_EVENT);
        uprint("Get Hydration Status event\n\r");
        if (REMINDER_isReminderPlanIsRunning())
        {
            uprint("stop reminder plan and restart the reminder\n\r");
            REMINDER_RestartReminder(true);
            LEDS_Stop();
        }
        else
        {
            if (appInfo->appMode != DARK)
            {
                uint64_t now = RTC_GetUnixTimeAndDate();
                Cap_ShowHydrationLevel(false);
            }
        }        
    }
    
    if(appInfo->flags & PB_ONE_LONG_CLICK_EVENT)//RUN_SET_DARK_ACTIVE_EVENT)
    {
        appInfo->flags &= (~PB_ONE_LONG_CLICK_EVENT);
        uprint("Dark/Active Mode - LONG_PRESS_TYPE_1 - more 1.5 sec \n\r");
        
        LEDS_Play(LED_PLAN_ID_ENTER_EXIT_DARK_MODE);
        switch (appInfo->appMode)
        {
            case ACTIVE:
            {
                uprint("DARK MODE: LEDs OFF\n\r");
                LogGeneralEvent(GeneralEvent_Code_EnterDarkMode);
                REMINDER_DisableAllReminderIndications();
                appInfo->appMode = DARK;
            }
            break;

            case DARK:
            {
                uprint("ACTIVE: LEDs ON\n\r");
                LogGeneralEvent(GeneralEvent_Code_ExitDarkMode);
                REMINDER_EnableAllReminderIndications();
                appInfo->appMode = ACTIVE;
            }
            break;
        }
    }
    
    if(appInfo->flags & SYS_BOTTLE_FILLED_EVENT)
    {
        appInfo->flags &= (~SYS_BOTTLE_FILLED_EVENT);
        if (appInfo->appMode != DARK)
        {
            LEDS_Play(LED_PLAN_ID_FILLED_UP);
        }
    }
    
    if(appInfo->flags & SYS_CHARGER_PLUGGED)
    {
        appInfo->flags &= (~SYS_CHARGER_PLUGGED);
        uprint("SYS_CHARGER_PLUGGED\n\r");
        nextState = APP_CHARGING_STATE;
    }
    
    // in order to handle open / close events

    CAP_handleBottleEvents(appInfo, ((nextState == APP_OPERATION_STATE) && (appInfo->appMode != DARK)));
    
    return nextState;
}

typedef struct
{
    void            (*entry)            (void);
    void			(*exit)             (void);
    APP_MAIN_STATE_t(*handler)          (appInfo_t* appInfo);
}smFunctionArray_t;

smFunctionArray_t  appSMStates[APP_MAX_STATES] =
{
    /* APP_CRITICAL_TEMPERATURE_STATE   # 0*/
    {
        appCriticalTemperatureEntry,
        appCriticalTemperatureExit,
        appCriticalTemperatureHandler,
    },
    
    /* APP_BATTERY_VERY_LOW_STATE       # 1*/
    {
        appBatteryVeryLowEntry,
        appBatteryVeryLowExit,
        appBatteryVeryLowHandler,
    },
    
    /* APP_PAIRING_PROCESS_STATE        # 2*/
    {
        appPairingProcessEntry,
        appPairingProcessExit,
        appPairingProcessHandler,
    },
    
    /* APP_CHARGING_STATE               # 3*/
    {
        appChargingEntry,
        appChargingExit,
        appChargingHandler,
    },
    
    /* APP_PAUSED_STATE                 # 4*/
    {
        appPausedEntry,
        appPausedExit,
        appPausedHandler,
    },
        
    /* APP_OPERATION_STATE              # 5*/
    {
        appOperationEntry,
        appOperationExit,
        appOperationHandler,
    }
};


/* Global  user code ---------------------------------------------------------*/
void appStateMachine(appInfo_t* appInfo)
{
    /*
    APP_CRITICAL_TEMPERATURE_STATE     = 0, // do nothing
    APP_BATTERY_VERY_LOW_STATE         = 1, // only battery level measure
    APP_PAIRING_PROCESS_STATE          = 2, // pairing process, if paired - open\close working but no leds plans
    APP_CHARGING_STATE                 = 3, // charging display,if paired - open\close working but no leds plans 
    APP_PAUSED_STATE                   = 4, // do nothing, for device not paired
    APP_OPERATION_STATE                = 5, // everything is working
    */
    APP_MAIN_STATE_t nextState = appSMStates[appInfo->appState].handler(appInfo);
    
    APP_MAIN_STATE_t state = appGetState(); 

    if (state < nextState)
    {
        uprintf("States are not equals, appGetState %d,  appSMStates %d\n\r", state, nextState);
        nextState = state; 
    }
                      
    if (appInfo->appState != nextState)
    {
        appSMStates[appInfo->appState].exit();
        appSMStates[nextState].entry();
        appInfo->appState = nextState;
    }
}

/*Private Function Definition*/
static bool CAP_isTemperatureOutOfRange(void)
{
    //return false;
    static int16 temperature = 25;    
    static unsigned long time = 0;
    
    if (RTC_GetRawTime() - time >= TEMPERATURE_TEST_PERIOD)
    {
        time = RTC_GetRawTime();
        temperature = OCD_GetTemperature();
        uprintf("temperature is %d\n\r", temperature);
    }
    
    if ((temperature < TEMPERATURE_LOW_RANGE) || (temperature > TEMPERATURE_HIGH_RANGE))
    {
        return true;
    }
    
    return false;
}

static APP_MAIN_STATE_t appGetState(void)
{
    if (CAP_isTemperatureOutOfRange())
    {
        return APP_CRITICAL_TEMPERATURE_STATE;
    }

    if (chargerPlugged == CHARGER_getChargerState())
    {
        return APP_CHARGING_STATE;
    }

    if (BATTERY_isBatteryVeryLow())
    {
        return APP_BATTERY_VERY_LOW_STATE;
    }

    if (!appConfig.paired)
    {
       return APP_PAUSED_STATE;
    }

    return APP_OPERATION_STATE;
}

// for BLE API command
bool CAP_isInPairingMode(void)
{
    return (APP_PAIRING_PROCESS_STATE == appInfo.appState);
}

uint8_t Cap_mainLoop(void)
{
    MEAS_RESULT_t measure;
    uint64_t now = RTC_GetUnixTimeAndDate();
    uint8_t periodicAdv = 0;

    // close UART port on timeout
    if (isUserEnabledDebugMode() != uartLogEnabled)
    {
        CAP_checkDebugPortTimeout();
    }

    // if CLI message arrived, handle it
    if (g_TxIntOccured)
    {
        g_TxIntOccured = 0;
        handleTxInt();
    }

    // run handlers
    UserFlash_Handler();
    LEDS_Handler();
    PB_1_Handler();
    reminderEvent_t reminderStatus  = REMINDER_Handler(now);
    OCD_State_t     openCloseState  = OCD_Handler(now);
    MEAS_STATUS_t   isMeasureValid  = measure_Handler(now);
    HYD_Handler(now);
    CHARGER_Handler(now);
    TIMER_handler();

    // system parameters has changed (CLI \ BLE API)
    if (appInfo.flags & CAP_SYSTEM_PARAMETERS_CHANGE_EVENT)
    {
        uprint("CAP_SYSTEM_PARAMETERS_CHANGE_EVENT\n\r");
        appInfo.flags &= (~CAP_SYSTEM_PARAMETERS_CHANGE_EVENT);
        appInfo.systemParametersChange = 0;
        Cap_readSystemParameters();
        LogGeneralEvent(GeneralEvent_Code_ConfigUpdate);
        BLE_SetInterval(appConfig.advertiseInterval);
    }

    // run the state machine
    appStateMachine(&appInfo);

    /* events that should be handled no matter what's state machine's state is*/

    if (appInfo.flags & SYS_BLE_CONNECTION_EVENT)
    {
        uprint("SYS_BLE_CONNECTION_EVENT\n\r");
        appInfo.flags &= (~SYS_BLE_CONNECTION_EVENT);
        UserFlash_BLEConnectedEvent();
        LEDS_StopSpecificPlan(LED_PLAN_ID_PAIRING);
        LogGeneralEvent(GeneralEvent_Code_AppConnected);
        TIMER_Start(Cap_notificationAfterConnetionDelayTimeout, NOTIFICATION_DELAY, oneShot, &appInfo.notificationDelayTimerHandler);
    }

    if (appInfo.flags & SYS_BLE_DISCONNECTION_EVENT)
    {
        uprint("SYS_BLE_DISCONNECTION_EVENT\n\r");
        appInfo.flags &= (~SYS_BLE_DISCONNECTION_EVENT);
        UserFlash_BLEDisconnectedEvent();
        LogGeneralEvent(GeneralEvent_Code_AppDisconnected);
        appInfo.notificationEnable = false; // stop notifications.
    }

    if (appInfo.flags & SYS_BATTERY_LOW_EVENT)
    {
        uprint("SYS_BATTERY_LOW_EVENT\n\r");
        appInfo.flags &= (~SYS_BATTERY_LOW_EVENT);
        uprintf("lowBatteryState = %d\n\r", appInfo.lowBatteryState);
        LEDS_ShowLowBatteryLevel(appInfo.lowBatteryState);
    }

    if (appInfo.flags & BATTERY_SERVICE_EVENT)
    {
        uprint("BATTERY SERVICE EVENT\n\r");
        appInfo.flags &= (~BATTERY_SERVICE_EVENT);
        BLE_BatteryService(CHARGER_GetBatterLevel());
    }

    if (appInfo.flags & START_NOTIFICATION_AFTER_CONNECTION)
    {
        appInfo.flags &= (~START_NOTIFICATION_AFTER_CONNECTION);
        uprint("START_NOTIFICATION_AFTER_CONNECTION\n\r");
        appInfo.notificationEnable = true;
        appInfo.notificationDelayTimerHandler = 0;
    }
    if (appInfo.flags & SYS_PERIODIC_ADVERTISE)
    {
        appInfo.flags &= (~SYS_PERIODIC_ADVERTISE);
        // first, build the message to be sent
        // start periodic advertise
        if (OK == Cap_CollectAdvertiseMessage(&appInfo.advertiseMessage, appConfig.paired ? PAIRING_STATUS_PAIRED : PAIRING_STATUS_NOT_PAIRED))
        {
            Cap_setAdvertiseMessage(&appInfo.advertiseMessage);
        }
        periodicAdv = 1;
    }
    if (appInfo.flags)
    {
        uprintf("appState %d\t appInfo.flags 0x%8x\n\r", appInfo.appState, appInfo.flags);
        appInfo.flags = 0;
    }
    return periodicAdv;
}

uint8_t CAP_getSMState(void)
{
    return appInfo.appState;
}

static void CAP_handleBottleEvents(appInfo_t* appInfo, bool displayOn)
{
    //uprint("CAP_handleBottleEvents\n\r");
    switch (appInfo->bottleState)
    {
        case Open:
        {
            if (OCD_NEW_CLOSE_EVENT & appInfo->flags)
            {
                appInfo->flags &= (~OCD_NEW_CLOSE_EVENT);
                appInfo->bottleState = Close;
                
                // restart the reminder.
                REMINDER_RestartReminder(false);
                uprint("NewState_Close\n\r");
                LogGeneralEvent(GeneralEvent_Code_Close);
                              
                // stop measure if measure was in process
                MEAS_Stop();
                //
                TIMER_Start(Cap_bottleCloseStabilityEvent, appConfig.waitTillMeasure, oneShot, &appInfo->stabilityCloseTimerHandler);
                TIMER_Start(Cap_ledsOffAfterCloseEvent, LEDS_OFF_TIMER, oneShot, &appInfo->ledsOffAfterCloseTimerHandler);
            }   
            
            if (SYS_REMINDER_INTERVAL_EVENT & appInfo->flags)
            {
                appInfo->flags &= (~SYS_REMINDER_INTERVAL_EVENT);
                uprint("SYS_REMINDER_INTERVAL_EVENT\n\r");
                appInfo->measureReason = reminderEvent;
                CAP_HandleHydration(displayOn, appInfo->measureReason);
            }   
        }
        break;
        
        case Close:
            if (OCD_NEW_OPEN_EVENT & appInfo->flags)
            {
                appInfo->flags &= (~OCD_NEW_OPEN_EVENT);
                appInfo->bottleState = Open;
                
                // restart the reminder.
                REMINDER_RestartReminder(false);

                uprint("NewState_Open\n\r");
                LogGeneralEvent(GeneralEvent_Code_Open);
                if (displayOn)
                {
                    Cap_ShowHydrationLevel(true);
                }

                // stop measure if measure was in process
                //MEAS_Stop();
                /* if Liquid Detection is Not Active do MEAS_STOP */
                #ifdef GATO_RADE_PROJECT
                    uprint("ChooseLiquidType\n\r");
                    MEAS_ChooseLiquidType();
                #else
                    MEAS_Stop();
                #endif
                // keep the time in order to wait the stability time before measure
                if (TIMER_isExist(appInfo->stabilityCloseTimerHandler))
                {
                    TIMER_Stop(appInfo->stabilityCloseTimerHandler);    
                }
            }

            if (SYS_REMINDER_INTERVAL_EVENT & appInfo->flags)
            {
                appInfo->flags &= (~SYS_REMINDER_INTERVAL_EVENT);
                uprint("SYS_REMINDER_INTERVAL_EVENT\n\r");
                appInfo->measureReason = reminderEvent;
                if (appInfo->lastMeasureStatus == MEAS_NOT_VALID)
                {
                    uprint("MEAS_Start since prev was not valid\n\r");
                    MEAS_Start();
                }
                else
                {
                    CAP_HandleHydration(displayOn, appInfo->measureReason);
                }
            }    
            
            if (SYS_MEAS_READY & appInfo->flags)
            {
                appInfo->flags &= (~SYS_MEAS_READY);
                uprint("MEAS VALID\n\r");
                appInfo->lastMeasureStatus = MEAS_COMPLETE;
                MEAS_RESULT_t measure;
                MEAS_GetResult(&measure);
                LogMeasureEvent(MeasureEvent_Code_Length, measure.range, measure.std);
			    LogMeasureEvent(MeasureEvent_Code_Volume, measure.volume, 0xFF);
                if (measure.volume >= 150)
                {
                    #ifdef GATO_RADE_PROJECT
                         uprint("ACC_setZThreshold\n\r");
                        ACC_setZThreshold(measure.volume);
                    #endif
                }
                CAP_HandleHydration(displayOn, appInfo->measureReason);
            }
            
            if (SYS_MEAS_NOT_VALID & appInfo->flags)
            {
                appInfo->flags &= (~SYS_MEAS_NOT_VALID);
                MEAS_RESULT_t measure;
                MEAS_GetResult(&measure);
                LogMeasureEvent(GeneralEvent_Code_MeasNotValid, MEAS_NOT_VALID_ERROR, measure.std);
                uprint("MEAS NOT VALID\n\r");
                appInfo->lastMeasureStatus = MEAS_NOT_VALID;
                
                // in this case, when last measure was not valid, hydration level will be calculated according
                // to previous measure.
                CAP_HandleHydration(displayOn, appInfo->measureReason);
            }
            
            if (CLOSE_STABILITY_TIMEOUT_EVENT & appInfo->flags)
            {
                appInfo->flags &= (~CLOSE_STABILITY_TIMEOUT_EVENT);
                appInfo->measureReason = closeEvent;
                MEAS_Start();
            }
            
            if (LEDS_OFF_AFTER_CLOSE_TIMEOUT_EVENT & appInfo->flags)
            {
                appInfo->flags &= (~LEDS_OFF_AFTER_CLOSE_TIMEOUT_EVENT);
                if (LED_PLAN_ID_HYDRATION_LEVEL == LEDS_returnActivePlanId())
                {
                    LEDS_Stop();
                }
            }
        break;
        default:
        break;
    }
}

static void CAP_HandleHydration(uint8_t reminderOn, measureReason_t measureReason)
{
    HYD_STATUS_t HYD_status = HYD_checkLowHydrated();

    if (HYD_status == LOW_HYDRATION)// || (everyTime == appConfig.reminderLogic))
    {
		//uprintf("measure reason = %d\n\r", measureReason);
        if ((reminderOn == true) && (measureReason == reminderEvent))
        {
            LogMeasureEvent(GeneralEvent_Code_Remind, 0, 0);
            // lights, sound, vibration
            REMINDER_Play();
        }
    }
    else
    {
        REMINDER_clearReminderShortTimeout();
        uprint("REMINDER_clearReminderShortTimeout\n\r");
        // Noa asked to write this log message just in case it was a reminder event
        if (measureReason == reminderEvent)
        {
            uprint("Reminder Cancelled due to OVER HYDRATION\n\r");
        }
    }
}

static void Cap_ShowHydrationLevel(bool longDuration)
{
    HYD_INFO_t hydInfo = HYD_getHydrationLevel();

    if (hydInfo.hydrationLevel >= 100)
    {
        uprint("FULL DAILY GOAL !!!\n\r");
        LEDS_Play(LED_PLAN_ID_GOAL_REACHED);
    }
    else
    {
        uprintf("HYD Level = %d%%\n\r", hydInfo.hydrationLevel);
        uprintf("Need To Drink Now = %d%%\n\r", hydInfo.needToDrinkNow);
        
        LEDS_ShowHydrationLevel(hydInfo.hydrationLevel, hydInfo.needToDrinkNow, longDuration);
    }
}

int Cap_OffLineProcess(void)
{
    if (st_go_to_bootloader)
    {
        LEDS_Stop();
        st_go_to_bootloader = 0;
        CustomBootloaderSwitch();
    }
    if (st_make_reset)
    {
        st_make_reset = 0;
        CySoftwareReset();
    }
    if (st_make_delay_for_wdt)
    {
        st_make_delay_for_wdt = 0;
        CyDelay(25000);
    }
    return Cap_mainLoop();    
}

static int Cap_handleDisconnectEvent(void)
{
    ToF_Sleep();
    if (st_idle_timeout_is_reached())
        st_go_sleep = 1;
    st_idle_timeout_till = 0;
    LogGeneralEvent(GeneralEvent_Code_AppDisconnected);
    return 0;
}

static void Cap_readSystemParameters(void)
{
    /*PARAM_WAIT_TILL_MEASURE,
    PARAM_ADVERTISE_INTERVAL,
    PARAM_ADVERTISE_PERIOD,
    PARAM_FIRST_CONNECTION_STATE
    PARAM_REMINDER_LOGIC
    */
    uint16_t advertisePeriod = appConfig.advertisePeriod;
    
    UserFlash_readParam(PARAM_WAIT_TILL_MEASURE,        (void*) &appConfig.waitTillMeasure,     sizeof(appConfig.waitTillMeasure));
    UserFlash_readParam(PARAM_ADVERTISE_INTERVAL,       (void*) &appConfig.advertiseInterval,   sizeof(appConfig.advertiseInterval));
    UserFlash_readParam(PARAM_ADVERTISE_PERIOD,         (void*) &appConfig.advertisePeriod,     sizeof(appConfig.advertisePeriod));
    UserFlash_readParam(PARAM_PAIRING_STATE,            (void*) &appConfig.paired,              sizeof(appConfig.paired));
    UserFlash_readParam(PARAM_REMINDER_LOGIC,           (void*) &appConfig.reminderLogic,       sizeof(appConfig.reminderLogic));


    if (advertisePeriod != appConfig.advertisePeriod)
    {
        appInfo.advertiseTimerHandler = 0;
        if (TIMER_isExist(appInfo.advertiseTimerHandler))
        {
            TIMER_Stop(appInfo.advertiseTimerHandler);
        }
        TIMER_Start(Cap_periodicAdvertiseEvent, appConfig.advertisePeriod, periodic, &appInfo.advertiseTimerHandler);        
    }
}

static void Cap_systemParametersChanged (flashParam_t flashParam)
{
    appInfo.flags |= CAP_SYSTEM_PARAMETERS_CHANGE_EVENT;
    appInfo.systemParametersChange |= 1<<flashParam;
}

static void Cap_batteryVeryLowEvent(void)
{
    appInfo.flags |= SYS_BATTERY_VERY_LOW_EVENT;
}

static void Cap_batteryLowEvent(lowBatteryState_t lowBattState)
{    
    uprintf("low batt. state is %d\n\r", lowBattState);
    appInfo.lowBatteryState = lowBattState;
    appInfo.flags |= SYS_BATTERY_LOW_EVENT;
}

static void Cap_batteryOKEvent(void)
{
    uprint("battery OK event\n\r");
    appInfo.flags |= SYS_BATTERY_NORMAL_EVENT;
}

void Cap_connectionEvent(void)
{
    appInfo.flags |= SYS_BLE_CONNECTION_EVENT;
}

void Cap_disconnectionEvent (void)
{
    appInfo.flags |= SYS_BLE_DISCONNECTION_EVENT;
}

static void Cap_BatteryService(uint16_t a_batteryLevelRaw)
{
    appInfo.flags |= BATTERY_SERVICE_EVENT;
}

static void Cap_pairingRequest(void)
{
    appInfo.flags |= SYS_BLE_PAIRING_REQUEST_EVENT;
}

static void Cap_pairingSucceeded(void)
{
    appInfo.flags |= SYS_BLE_PAIRING_SUCCEED_EVENT;
}

static void Cap_pairingTimeout(void)
{
    uprint("SYS_BLE_PAIRING_TIMEOUT\n\r");
    appInfo.flags |= SYS_BLE_PAIRING_TIMEOUT;
}

static void Cap_periodicAdvertiseEvent(void)
{
    //uprint("SYS_PERIODIC_ADVERTISE\n\r");
    appInfo.flags |= SYS_PERIODIC_ADVERTISE;    
}

static void Cap_pairingRetry(void)
{
    uprint("Cap_pairingRetry\n\r");
    appInfo.flags |= SYS_BLE_PAIRING_RETRY_EVENT;
}

static void Cap_MeasReady(void)
{
    //uprint("Cap_MeasReady\n\r");
    appInfo.flags |= SYS_MEAS_READY;  
}

static void Cap_MeasNotValid(void)
{
    //uprint("Cap_MeasNotValid\n\r");
    appInfo.flags |= SYS_MEAS_NOT_VALID;    
}

static void Cap_bottleCloseStabilityEvent(void)
{
    //uprint("Cap_bottleCloseStabilityEvent\n\r");
    appInfo.flags |= CLOSE_STABILITY_TIMEOUT_EVENT; 
}

static void Cap_ledsOffAfterCloseEvent(void)
{
    //uprint("Cap_ledsOffAfterCloseEvent\n\r");
    appInfo.flags |= LEDS_OFF_AFTER_CLOSE_TIMEOUT_EVENT; 
}

static void Cap_pairingCompleteLedsPlanTimeout(void)
{
    appInfo.flags |= BLE_PAIRING_PLAN_COMPLETE_SUCCESS_EVENT;
}

static void Cap_userAnswerTimeout(void)
{
    appInfo.flags |= USER_ANSWER_TIMEOUT_EVENT;
}

static void Cap_reminderIntervalEventHandler(void)
{
    //uprint("Cap_ReminderIntervalEventHandler\n\r");
    appInfo.flags |= SYS_REMINDER_INTERVAL_EVENT;
}

static void Cap_bottleFilledEventHandler(void)
{
    uprint("Cap_bottleFilledEventHandler\n\r");
    appInfo.flags |= SYS_BOTTLE_FILLED_EVENT;
}
static void Cap_notificationAfterConnetionDelayTimeout(void)
{
    uprint("Cap_notificationAfterConnetionDelayTimeout\n\r");
    appInfo.flags |= START_NOTIFICATION_AFTER_CONNECTION;
}

static void Cap_OpenCloseEventHandler(OCD_State_t OCD_State)
{
    if (newState_Close == OCD_State)
    {
        appInfo.flags |= OCD_NEW_CLOSE_EVENT;
    }
	else if (newState_Open == OCD_State)
    {
	    appInfo.flags |= OCD_NEW_OPEN_EVENT;
    }
}

static void Cap_chargerState(chargerState_t chargerState)
{
    if (newState_chargerPlugged == chargerState)
    {
        appInfo.flags |= SYS_CHARGER_PLUGGED;
    }
    else
    if (newState_chargerUnplugged == chargerState)
    {
        appInfo.flags |= SYS_CHARGER_UNPLUGGED;
    }
}

/*static */void Cap_PBEventHandler(PB_1_CLICK_TYPE_t pb_1_clickType)
{
    switch(pb_1_clickType)
    {
        case  ONE_SHORT_CLICK:
            uprintf("PB_ONE_SHORT_CLICK_EVENT\n\r");
            LogMeasureEvent(GeneralEvent_Code_ButtonPressed, ONE_SHORT_CLICK,0);
            appInfo.flags |= PB_ONE_SHORT_CLICK_EVENT;
            break;
        case ONE_LONG_CLICK_TYPE_1:
            uprintf("PB_ONE_LONG_CLICK_EVENT\n\r");
            LogMeasureEvent(GeneralEvent_Code_ButtonPressed, ONE_LONG_CLICK_TYPE_1,0);
            appInfo.flags |= PB_ONE_LONG_CLICK_EVENT;
            break;
        case ONE_LONG_CLICK_TYPE_2:
            uprintf("PB_ONE_VERY_LONG_CLICK\n\r");
            LogMeasureEvent(GeneralEvent_Code_ButtonPressed, ONE_LONG_CLICK_TYPE_2,0);
            appInfo.flags |= PB_ONE_VERY_LONG_CLICK;
            break;
        case DOUBLE_CLICK:
            uprintf("PB_DOUBLE_CLICK_EVENT\n\r");
            LogMeasureEvent(GeneralEvent_Code_ButtonPressed, DOUBLE_CLICK,0);
            appInfo.flags |= PB_DOUBLE_CLICK_EVENT;
            break;
        case NO_CLICK:
        default:
            break;
    }
}

void CAP_Init(void)
{

    if (OK != UserFlash_RegisterSystemParametersChanged(Cap_systemParametersChanged, (flashParam_t*)systemParametersList, sizeof(systemParametersList)))
    {
        uprint("CAP - failed to register System Parameters Change Event\n\r");
    }

    if (OK != BATTERY_RegisterBatteryVeryLow(Cap_batteryVeryLowEvent))            
    {
        uprint("CAP - failed to register Battery Very Low Event\n\r");
    }
    
    if (OK != BATTERY_RegisterBatteryLow(Cap_batteryLowEvent))
    {
        uprint("CAP - failed to register Battery Low Event\n\r");
    }

    if (OK != BATTERY_RegisterBatteryOK(Cap_batteryOKEvent))
    {
        uprint("CAP - failed to register Battery OK Event\n\r");
    }

    if (OK != BLE_Register_ConnectionEvent(Cap_connectionEvent))
    {
        uprint("CAP - failed to register Connection Event\n\r");
    }
    
    if (OK != BATTERY_RegisterBatteryService(Cap_BatteryService))
    {
        uprint("CAP - failed to register Battery Service Event\n\r");
    }
    if (OK != BATTERY_RegisterChargerState(Cap_chargerState))
    {
        uprint("CAP - failed to register BATTERY Register Charger State\n\r");
    }
    if (OK != BLE_RegisterPairingSucceededEvent(Cap_pairingSucceeded))
    {
        uprint("CAP - failed to register BLE_Register Pairing Succeeded Event\n\r");
    }
    
    if (OK != MEAS_RegisterMeasReady(Cap_MeasReady))
    {
        uprint("CAP - FAILED to Register MEASURE READY Event event\n\r");
    }
    
    if (OK != MEAS_RegisterMeasNotValid(Cap_MeasNotValid))
    {
        uprint("CAP - FAILED to Register MEASURE READY Event event\n\r");
    }
    
    /* Button Press Events */
    if (OK != PB_1_Register_ClickEvent(Cap_PBEventHandler))
    {
        uprint("CAP - failed to register ONE Short Event\n\r");
    }
    
    if (OK != OCD_RegisterOpenCloseEvent(Cap_OpenCloseEventHandler))
    {
        uprint("CAP - failed to register open \\ close Event\n\r");
    }
    
    if (OK != REMINDER_RegisterReminderInterval(Cap_reminderIntervalEventHandler))
    {
        uprint("CAP - failed to register open \\ close Event\n\r");
    }

    if (OK != HYD_RegisterHydFilled(Cap_bottleFilledEventHandler))
    {
        uprint("CAP - Cap_bottleFilledEventHandler\n\r");
    }

    Cap_readSystemParameters();

    TIMER_Start(Cap_periodicAdvertiseEvent, appConfig.advertisePeriod, periodic, &appInfo.advertiseTimerHandler);
    
    if (appConfig.paired == true)
    {
        uprint("********** DEVICE  IS  PAIRED *********\n\r");
        CapStartAdvertisingCustomInternal();
    }
    else
    {
        //appInfo.appState = NOT_PAIRED;
        uprint("********** DEVICE  IS NOT PAIRED *********\n\r");
    }
    
    appInfo.bottleState = (true == OCD_isClose() ? Close : Open);
       
}

bool getAppPowerState(void)
{
    bool  appPowerState = false;
    appPowerState   |= PB_ActiveNow();
    //uprintf("PB_ActiveNow %d\n\r", appPowerState);
    appPowerState   |= MEAS_ActiveNow();
    //uprintf("MEAS_ActiveNow %d\n\r", appPowerState);
    appPowerState   |= LEDS_isActiveNow();
    //uprintf("LEDS_isActiveNow %d\n\r", appPowerState);
    appPowerState   |= UART_ActiveNow();
    //uprintf("UART_ActiveNow %d\n\r", appPowerState);
    return appPowerState;
}

bool CAP_isActiveMode(void)
{
    return (ACTIVE == appInfo.appMode);
}

bool CAP_IsNotificationEnabled(void)
{
    return appInfo.notificationEnable;
}

void AllowNotificationsForFT2(void)
{
    appInfo.notificationEnable = true;
}

#ifdef __cplusplus
}
#endif
