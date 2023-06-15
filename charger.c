#ifdef __cplusplus
 extern "C" {
#endif

/**
  ******************************************************************************
  * @file           : charger.c
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
#include "../Peripheral/charger.h"
#include "../Peripheral/LEDSBasicPlans.h"
#include "../Modules/RTC_Module.h"
#include "../Modules/Log_Module.h"
#include "../Modules/debugPort_Module.h"
#include "ADC_SAR_Seq.h"


/* Private typedef -----------------------------------------------------------*/
// these are the events to handle in this module
typedef enum
{
    CHARGER_CHRG_INT            =  1,    
}chargerFlags_t;

typedef enum  
{
    batteryUnknown,
    batteryFull,
    batteryNormal,
    batteryOK_level5,     
    batteryLowLevel20,
    batteryLowLevel10,
    batteryLowLevel3,
    batteryVeryLowLevel,
}batteryLevelState_t; 

typedef struct
{
    time_t              lastTimeUpdate;     // last time we updated the LEDS regarding the battery level;
    time_t              lastTimeMeasure;    // last time we measure the battery.
    time_t              lastEmitBatteryLow; // time stamp of last time this module sent the event to app.
    uint8_t             flags;              // these flag represents events
    chargerState_t      chargerState;       // plugged, unplugged, battery is full
    uint16_t            period;             // measuring period (seconds)
    bool                displayBatteryStatusPermanently;
    batteryLevelState_t batteryLevelState;  // logic state
    uint16_t            a2dMeasure;         // ad2 measure
    isNotQuiet_t        isNotQuiet;         // function to call in order to check if we can measure battery level(accurate measure will be when leds are off)
    uint64_t            chargerPluggedTimeStamp;
}chargerInfo_t;

/* Private define ------------------------------------------------------------*/

#define BATTERY_VERY_LOW_NUM_OF_CALLBACKS       (2)     // battery very low callback array size
#define BATTERY_LOW_NUM_OF_CALLBACKS            (2)     // battery low callback array size
#define BATTERY_OK_NUM_OF_CALLBACKS             (2)     // battery OK callback array size
#define BATTERY_SERVICE_NUM_OF_CALLBACKS        (2)     // battery service callback array size
#define CHARGER_STATE_CHANGED_NUM_OF_CALLBACKS  (2)     // charger states callback array size
#define BATTERY_MEASURE_NUM_OF_CALLBACKS        (2)     //

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static chargerInfo_t chargerInfo =
{
    .lastTimeUpdate                  = 0,
    .lastTimeMeasure                 = 0,
    .lastEmitBatteryLow              = 0,
    .flags                           = 0,
    .chargerState                    = chargerUnknown,
    .period                          = HIGH_BATTERY_LEVEL_MEASURE_PERIOD,
    .displayBatteryStatusPermanently = false,
    .batteryLevelState               = batteryUnknown,
    .a2dMeasure                      = 0,
    .isNotQuiet                      = NULL,
    .chargerPluggedTimeStamp         = 0,
};

/* Private CallBack function prototypes -----------------------------------------------*/

static batteryVeryLowCallback_t batteryVeryLowCallbackArray[BATTERY_VERY_LOW_NUM_OF_CALLBACKS];
static batteryLowCallback_t     batteryLowCallbackArray[BATTERY_LOW_NUM_OF_CALLBACKS];
static batteryOKCallback_t      batteryOKCallbackArray[BATTERY_OK_NUM_OF_CALLBACKS];
static batteryServiceCallback_t batteryServiceCallBackArray[BATTERY_SERVICE_NUM_OF_CALLBACKS];
static chargerStateCallback_t   chargerStateCallBackArray[CHARGER_STATE_CHANGED_NUM_OF_CALLBACKS];


/* Private function prototypes -----------------------------------------------*/
static short                ADC_Read(unsigned int channel);
static chargerState_t       CHARGER_CheckChargerStatus(void);
static void                 CHARGER_UpdatePeriod(batteryLevelState_t batteryLevelState);
static batteryLevelState_t  CHARGER_getBatteryLevelState(uint16_t level);
static uint8_t              BATTERY_EmitBatteryVeryLow(void);
static uint8_t              BATTERY_EmitBatteryLow(batteryLevelState_t batteryLevelState);
static uint8_t              BATTERY_EmitBatteryOK(void);
static uint8_t              BATTERY_EmitBatteryService(uint16_t batteryLevelRaw);
static uint8_t              BATTERY_EmitChargerState(chargerState_t chargerState);
static void                 BATTERY_handleBatteryLow(time_t a_Time, batteryLevelState_t batteryLevelState, bool forceStart);

/* functions -----------------------------------------------------------------*/

/**
* @brief  : init the charger module.
* @param  : none
* @retval : 0 - OK, other - will be an error code.
*/
uint8_t CHARGER_Init(isNotQuiet_t a_isNotQuiet)
{
    chargerInfo.lastTimeMeasure = RTC_GetUnixTimeAndDate();

    // its a trick to update charger status during power up
    chargerInfo.flags = CHARGER_CHRG_INT;

    for (int i = 0 ; i < BATTERY_VERY_LOW_NUM_OF_CALLBACKS; i++)
    {
        batteryVeryLowCallbackArray[i] = NULL;
    }

    for (int i = 0 ; i < BATTERY_LOW_NUM_OF_CALLBACKS; i++)
    {
        batteryLowCallbackArray[i] = NULL;
    }

    for (int i = 0 ; i < BATTERY_OK_NUM_OF_CALLBACKS; i++)
    {
        batteryOKCallbackArray[i] = NULL;
    }

    for (int i = 0 ; i < BATTERY_SERVICE_NUM_OF_CALLBACKS; i++)
    {
        batteryServiceCallBackArray[i] = NULL;       
    }

    for (int i = 0 ; i < CHARGER_STATE_CHANGED_NUM_OF_CALLBACKS; i++)
    {
        chargerStateCallBackArray[i] = NULL;
    }    

    if (a_isNotQuiet != NULL)
    {
        chargerInfo.isNotQuiet = a_isNotQuiet;
    }

    return OK;
}

/**
* @brief  : this function run periodically and checks charger states.
* @param  : a_time - current time
* @retval : chargerState_t - return current state
*/
uint8_t batteryLevel;
chargerState_t CHARGER_Handler(const time_t a_Time)
{
    batteryLevelState_t currBatteryLevelState = batteryUnknown;
	chargerState_t retVal = chargerInfo.chargerState;
    bool forceEventEmitAfterUnpluggingTheCharger = false;
    bool isNotQuietState = true;

    if (chargerInfo.isNotQuiet != NULL)
    {
        isNotQuietState = chargerInfo.isNotQuiet();
    }
    else
    {
        isNotQuietState = false;
    }

    // this section check if charger is plugged or not.
    if (chargerInfo.flags & CHARGER_CHRG_INT)
    {
        chargerInfo.flags &= (~CHARGER_CHRG_INT);
        chargerState_t status = CHARGER_CheckChargerStatus();

        if (status != chargerInfo.chargerState)
        {
            // in order to perform immediate a2d sampling
            chargerInfo.lastTimeMeasure = 0;

            chargerInfo.chargerState = status;
            if (status == chargerUnplugged)
            {
                LogGeneralEvent(GeneralEvent_Code_ChargeStop);
                forceEventEmitAfterUnpluggingTheCharger = true;
                retVal = newState_chargerUnplugged;
                BATTERY_EmitChargerState(newState_chargerUnplugged);
            }
            else
            if (status == chargerPlugged)
            {
                chargerInfo.chargerPluggedTimeStamp = RTC_GetRawTime();
                LogGeneralEvent(GeneralEvent_Code_ChargeStart);
                CHARGER_UpdatePeriod(chargerInfo.batteryLevelState);
                // when unplugging the charger, i would like to force newest even if battery state was not changed.
                retVal = newState_chargerPlugged;
                BATTERY_EmitChargerState(newState_chargerPlugged);
            }
        }
    }

    // check if its time to measure
    if (chargerInfo.lastTimeMeasure + chargerInfo.period <= a_Time)
    {
        //uprintf("its time to measure battery level\n\r");
        if ((chargerInfo.chargerState == chargerPlugged) ||
        ((chargerInfo.chargerState == chargerUnplugged) && (!isNotQuietState)))
        {
            // measure battery level (read adc channel)
            chargerInfo.lastTimeMeasure = a_Time;
            unsigned short a2dRawData = 0;

            chargerInfo.a2dMeasure = CHARGER_ReadBattery(&a2dRawData);

            uprintf("Battery Voltage = %d [V]\n\r", chargerInfo.a2dMeasure);
            BATTERY_EmitBatteryService(chargerInfo.a2dMeasure);
        }
        else
        if ((chargerInfo.chargerState == chargerUnplugged) && (isNotQuietState))
        {
            // only for debug
            //uprintf("time to measure but since its not quiet then delay it\n\r");
        }
    }

    // get logic level
    // calculate battery level %
    batteryLevel = CHARGER_CalculateBatterLevel(chargerInfo.a2dMeasure);
    //uprintf("Bat level = %d\n\r", batteryLevel);
    currBatteryLevelState = CHARGER_getBatteryLevelState(batteryLevel);

    // handle chargerPlugged state
    if (chargerInfo.chargerState == chargerPlugged)
    {
        if (chargerInfo.batteryLevelState != currBatteryLevelState)
        {
            // update sample period according to battery level
            CHARGER_UpdatePeriod(currBatteryLevelState);

            if (batteryFull == currBatteryLevelState)
            {
                LogGeneralEvent(GeneralEvent_Code_BatteryFull);
            }

            /*if ( (batteryOK_level5 == currBatteryLevelState) &&
                    ((batteryLowLevel20 == chargerInfo.batteryLevelState) ||
                    (batteryLowLevel10 == chargerInfo.batteryLevelState) ||
                    (batteryLowLevel3 == chargerInfo.batteryLevelState))
                 )*/
            if ((batteryVeryLowLevel == chargerInfo.batteryLevelState) && 
                ((batteryNormal ==      currBatteryLevelState) ||
                 (batteryLowLevel20 ==  currBatteryLevelState) ||
                 (batteryLowLevel10 ==  currBatteryLevelState) ||
                 (batteryOK_level5 ==   currBatteryLevelState) ||
                 (batteryLowLevel3 ==   currBatteryLevelState)))
            {
                uprintf("BATTERY is OK\n\r");
                BATTERY_EmitBatteryOK();
            }
            chargerInfo.batteryLevelState = currBatteryLevelState;
            
        }

        if (chargerInfo.displayBatteryStatusPermanently)
        {
            // display refresh time
            if (a_Time - chargerInfo.lastTimeUpdate >= 60)
            {
                chargerInfo.lastTimeUpdate = a_Time;

                // calculate battery level %
               // uint8_t batteryLevel = CHARGER_CalculateBatterLevel(chargerInfo.a2dMeasure);

                LEDS_ShowBatteryLevel(batteryLevel);
            }
        }
        
        if ((chargerInfo.chargerPluggedTimeStamp ) && 
           (RTC_GetRawTime() - chargerInfo.chargerPluggedTimeStamp > CHARGER_TIMEOUT))
        {
            uprintf("charger timeout - LEDS are shutting down\n\r");
            chargerInfo.chargerPluggedTimeStamp = 0;
            chargerInfo.displayBatteryStatusPermanently = false;
            LEDS_Stop();
        }
    }
    // handle chargerUnplugged state
    else if (chargerInfo.chargerState == chargerUnplugged)
    {
        if ((chargerInfo.batteryLevelState != currBatteryLevelState) || (forceEventEmitAfterUnpluggingTheCharger))
        {
            uprintf("chargerUnplugged %d %d\n\r", chargerInfo.batteryLevelState, currBatteryLevelState);
            /* IPP 4143
               its not a logical case but it happens, since our lookup table is not accurate.
               what heppens is that:
               when disconnecting the charger during very low battery the the next measure is "jump" in 20%  
               but we dont send Battery OK event to the application so it stay in very low battery and stay disable. 
            */
            if ((batteryVeryLowLevel == chargerInfo.batteryLevelState) && 
                ((batteryNormal ==      currBatteryLevelState) ||
                 (batteryLowLevel20 ==  currBatteryLevelState) ||
                 (batteryLowLevel10 ==  currBatteryLevelState) ||
                 (batteryOK_level5 ==   currBatteryLevelState) ||
                 (batteryLowLevel3 ==   currBatteryLevelState)))
            {
                uprint("BATTERY is OK\n\r");
                BATTERY_EmitBatteryOK();
            }
            
            chargerInfo.batteryLevelState = currBatteryLevelState;

            // update sample period according to battery level
            //uprintf("batteryLevelState %d\n\r", chargerInfo.batteryLevelState);
            CHARGER_UpdatePeriod(chargerInfo.batteryLevelState);

            if ((batteryLowLevel20 == chargerInfo.batteryLevelState) || 
               (batteryLowLevel10 == chargerInfo.batteryLevelState) ||
                (batteryOK_level5 == chargerInfo.batteryLevelState) ||
               (batteryLowLevel3 == chargerInfo.batteryLevelState))
            {
                uprintf("battery is low\n\r");
                LogGeneralEvent(GeneralEvent_Code_LowPowerMode);
                BATTERY_handleBatteryLow(a_Time, chargerInfo.batteryLevelState, true);
            }
            else if (batteryVeryLowLevel == chargerInfo.batteryLevelState)
            {
                uprintf("battery is very low\n\r");
                LogGeneralEvent(GeneralEvent_Code_VeryLowPowerMode);
                BATTERY_EmitBatteryVeryLow();
            }
        }
        // check if its time to send battery low event to app
        BATTERY_handleBatteryLow(a_Time, chargerInfo.batteryLevelState, false);
    }
    return retVal;
}

void CHARGER_displayBatteryStatus(bool onOff)
{
    if (chargerInfo.displayBatteryStatusPermanently != onOff)
    {
        chargerInfo.displayBatteryStatusPermanently = onOff;

        if (false == chargerInfo.displayBatteryStatusPermanently)
        {
            LEDS_Stop();
        }
        else
        {
            chargerInfo.lastTimeUpdate = 0;
        }
    }
}

static void CHARGER_UpdatePeriod(batteryLevelState_t batteryLevelState)
{
    if ( chargerPlugged == chargerInfo.chargerState )
    {
        //uprintf("CHARGE_MODE_MEASURE_PERIOD \n\r");
        chargerInfo.period = CHARGE_MODE_MEASURE_PERIOD;
        return;
    }

    switch(batteryLevelState)
    {
        case batteryFull:
        case batteryNormal:
        default:
            //uprintf("HIGH_BATTERY_LEVEL_MEASURE_PERIOD\n\r");
            chargerInfo.period = HIGH_BATTERY_LEVEL_MEASURE_PERIOD;
        break;
        case batteryLowLevel20:
        case batteryLowLevel10:
        case batteryLowLevel3:
            //uprintf("LOW_BATTERY_LEVEL_MEASURE_PERIOD\n\r");
            chargerInfo.period = LOW_BATTERY_LEVEL_MEASURE_PERIOD;
        break;
        case batteryVeryLowLevel:
            //uprintf("VERY_LOW_BATTERY_LEVEL_MEASURE_PERIOD\n\r");
            chargerInfo.period = VERY_LOW_BATTERY_LEVEL_MEASURE_PERIOD;
        break;
    }
}

uint32_t CHARGER_CalculateBatterLevel(uint16_t measure)
{
    uint8_t chargingState = 0;
    //measure = 3615;
    //uprintf("Measure = %d\n\r", measure);
    
    if (chargerPlugged != chargerInfo.chargerState)
    {
        chargingState = 1;
    }
        
    if (measure <= volt2PrcTable[VOLT2PERCENTAGE_ARRAY_SIZE-1][chargingState])
    {
        //uprintf("less then minimum, %d, %d\n\r", measure, volt2Prc[VOLT2PERCENTAGE_ARRRAY_SIZE-1][chargingState]);
        return 0;
    }
    
    if (measure >= volt2PrcTable[0][chargingState])
    {
        //uprintf("bigger then minimum, %d, %d\n\r", measure, volt2Prc[0][chargingState]);
        return 100;
    }
        
    uint8_t vindex = VOLT2PERCENTAGE_ARRAY_SIZE-1;
    for(vindex = vindex ; vindex >= 0 ; vindex --) 
    {
        if (measure <= volt2PrcTable[vindex][chargingState])
        {
            break;
        }       
    }    
    
    uint8_t prc = ((VOLT2PERCENTAGE_ARRAY_SIZE-1)-vindex)*5;
    //uprintf("range = [%d:%d]\n\r", vindex, vindex+1);
        
    float res = (float)(measure - volt2PrcTable[vindex+1][chargingState]) / ((float)volt2PrcTable[vindex][chargingState] - volt2PrcTable[vindex+1][chargingState]);
        
    prc -= (5*(1-res));
    //uprintf("prc is = %d\n\r", prc);  
    return prc;
}

uint32_t CHARGER_GetBatterLevel(void)
{
    return CHARGER_CalculateBatterLevel(chargerInfo.a2dMeasure);
}

static short ADC_Read(unsigned int channel)
{
    short result = 0;
    ADC_SAR_Seq_Start();                /* Initialize ADC */
    ADC_SAR_Seq_IRQ_Disable();          /* Disable ADC interrupts */
    ADC_SAR_Seq_IRQ_ClearPending();     /* Clear pending interrupt */
    ADC_SAR_Seq_StartConvert();         /* Start ADC conversions */
    ADC_SAR_Seq_IsEndConversion(ADC_SAR_Seq_WAIT_FOR_RESULT); /* Wait for ADC conversion complete */
    result = ADC_SAR_Seq_GetResult16(channel); /* Get the data */
    ADC_SAR_Seq_StopConvert();          /* Stop ADC conversions */
    ADC_SAR_Seq_Stop();                 /* Initialize ADC */
    return result;
}

unsigned short CHARGER_ReadBattery(unsigned short* a_rawData)
{
    unsigned short rawData;
    unsigned short result;

    #define VBAT_THRESHOLD          (4100)
    #define RBAT                    (0.3)
    #define ICHARGE_HIGH            (50)
    #define ICHARGE_LOW             (600)

    rawData = ADC_Read(ADC_Battery_Channel);

    if (a_rawData != NULL)
    {
        *a_rawData = rawData;
    }

    result = (((rawData * ADC_Battery_Coef*ADC_VREF)*1000)/2048);
    
    /*static uint16 msr = 3454;
    result = msr;
    msr -=5;
    
    if (msr <= 3290)
    {
        msr = 4040;
    }
    */
    return result;
}

void CHARGER_intCallback(void)
{
    chargerInfo.flags |= CHARGER_CHRG_INT;    
}

chargerState_t CHARGER_getChargerState(void)
{
    return chargerInfo.chargerState;
}

static chargerState_t CHARGER_CheckChargerStatus(void)
{    
    if (CHRG_DTC_Read())
    {
        if (BAT_START_Read())
        {
            //uprint("battery is full\n\r");
            return chargerPlugged;
        }
        else
        {
            uprint("charging....\n\r");
            return chargerPlugged;
        }
    }
    else
    {
        uprint("charger is unplugged\n\r");
        return chargerUnplugged;
    }
        
}

static batteryLevelState_t CHARGER_getBatteryLevelState(uint16_t level)
{
    if (level >= 100)//HIGH_BATTERY_THRESHOLD)
    {
        //uprint("batteryFull \n\r");
        return batteryFull;
    }

    if (level > 20)//LOW_BATTERY_THRESHOLD)
    {
        //uprint("batteryNormal \n\r");
        return batteryNormal;
    }
 
    if (level > 10)//LOW_BATTERY20_THRESHOLD)
    {
        //uprint("batteryLowLevel20 \n\r");
        return batteryLowLevel20;
    }

    if (level > 5)
    {
        return batteryLowLevel10;//;
    }
    
    if (level > 3)//LOW_BATTERY10_THRESHOLD)
    {
        //uprint("batteryLowLevel10 \n\r");
        return batteryOK_level5;
    }
    else
    {
        return batteryVeryLowLevel;
    }
}

uint8_t BATTERY_RegisterBatteryOK(batteryOKCallback_t batteryOKCallback)
{
    for (int i = 0 ; i < BATTERY_OK_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryOKCallbackArray[i] == NULL)
        {
            batteryOKCallbackArray[i] = batteryOKCallback;
            return OK;
        }
    }
    return ERR_UNAVAILABLE_RESOURCE;
}

uint8_t BATTERY_UnregisterBatteryOK(batteryOKCallback_t batteryOKCallback)
{
    for (int i = 0 ; i < BATTERY_OK_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryOKCallbackArray[i] == batteryOKCallback)
        {
            batteryOKCallbackArray[i] = NULL;
            return OK;
        }
    }
    return ERR_OBJ_NOT_FOUND;
}

static uint8_t BATTERY_EmitBatteryOK(void)
{
    for (int i = 0 ; i < BATTERY_OK_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryOKCallbackArray[i] != NULL)
        {
            batteryOKCallbackArray[i]();
        }
    }
    return OK;
}

uint8_t BATTERY_RegisterBatteryVeryLow(batteryVeryLowCallback_t batteryVeryLowCallback)
{
    for (int i = 0 ; i < BATTERY_VERY_LOW_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryVeryLowCallbackArray[i] == NULL)
        {
            batteryVeryLowCallbackArray[i] = batteryVeryLowCallback;
            return OK;
        }
    }
    return ERR_UNAVAILABLE_RESOURCE;
}

uint8_t BATTERY_UnregisterBatteryVeryLow(batteryVeryLowCallback_t batteryVeryLowCallback)
{
    for (int i = 0 ; i < BATTERY_VERY_LOW_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryVeryLowCallbackArray[i] == batteryVeryLowCallback)
        {
            batteryVeryLowCallbackArray[i] = NULL;
            return OK;
        }
    }
    return ERR_OBJ_NOT_FOUND;
}

static uint8_t BATTERY_EmitBatteryVeryLow(void)
{
    for (int i = 0 ; i < BATTERY_VERY_LOW_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryVeryLowCallbackArray[i] != NULL)
        {
            batteryVeryLowCallbackArray[i]();
        }
    }
    return OK;
}

uint8_t BATTERY_RegisterBatteryLow(batteryLowCallback_t batteryLowCallback)
{
    for (int i = 0 ; i < BATTERY_LOW_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryLowCallbackArray[i] == NULL)
        {
            batteryLowCallbackArray[i] = batteryLowCallback;
            return OK;
        }
    }
    return ERR_UNAVAILABLE_RESOURCE;
}

uint8_t BATTERY_UnregisterBatteryLow(batteryLowCallback_t batteryLowCallback)
{
    for (int i = 0 ; i < BATTERY_LOW_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryLowCallbackArray[i] == batteryLowCallback)
        {
            batteryLowCallbackArray[i] = NULL;
            return OK;
        }
    }
    return ERR_OBJ_NOT_FOUND;
}

static uint8_t BATTERY_EmitBatteryLow(batteryLevelState_t batteryLevelState)
{
    // convert to low battery state - exist in global Definitions.h
    lowBatteryState_t lowBatteryState = lowBattery_20;

    if (batteryLowLevel10 ==  batteryLevelState)
    {
        lowBatteryState = lowBattery_10;
    }

    if (batteryOK_level5 ==  batteryLevelState)
    {
        lowBatteryState = lowBattery_5;
    }
    uprintf("battery Low Level = %d\n\r", batteryLevelState);
    for (int i = 0 ; i < BATTERY_LOW_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryLowCallbackArray[i] != NULL)
        {
            batteryLowCallbackArray[i](lowBatteryState);
        }
    }
    return OK;
}

uint8_t BATTERY_RegisterBatteryService(batteryServiceCallback_t batteryServiceCallback)
{
    for (int i = 0 ; i < BATTERY_SERVICE_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryServiceCallBackArray[i] == NULL)
        {
            batteryServiceCallBackArray[i] = batteryServiceCallback;            
            return OK;
        }
    }
    return ERR_UNAVAILABLE_RESOURCE;
}

uint8_t BATTERY_UnregisterBatteryService(batteryServiceCallback_t batteryServiceCallback)
{
    for (int i = 0 ; i < BATTERY_SERVICE_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryServiceCallBackArray[i] == batteryServiceCallback)
        {
            batteryServiceCallBackArray[i] = NULL;
            return OK;
        }
    }
    return ERR_OBJ_NOT_FOUND;
}

static uint8_t BATTERY_EmitBatteryService(uint16_t batteryLevelRaw)
{
       
    for (int i = 0 ; i < BATTERY_SERVICE_NUM_OF_CALLBACKS ; i++)
    {
        if (batteryServiceCallBackArray[i] != NULL)
        {
            batteryServiceCallBackArray[i](batteryLevelRaw);
        }
    }
    return OK;
}

uint8_t BATTERY_RegisterChargerState(chargerStateCallback_t chargerStateCallback)
{
    for (int i = 0 ; i < CHARGER_STATE_CHANGED_NUM_OF_CALLBACKS; i++)
    {
        if (chargerStateCallBackArray[i] == NULL)
        {
            chargerStateCallBackArray[i] = chargerStateCallback;            
            return OK;
        }
    }
    return ERR_UNAVAILABLE_RESOURCE;
}
uint8_t BATTERY_UnregisterChargerState(chargerStateCallback_t chargerStateCallback)
{
    for (int i = 0 ; i < CHARGER_STATE_CHANGED_NUM_OF_CALLBACKS ; i++)
    {
        if (chargerStateCallBackArray[i] == chargerStateCallback)
        {
            chargerStateCallBackArray[i] = NULL;
            return OK;
        }
    }
    return ERR_OBJ_NOT_FOUND;
}
static uint8_t BATTERY_EmitChargerState(chargerState_t chargerState)
{
    for (int i = 0 ; i < CHARGER_STATE_CHANGED_NUM_OF_CALLBACKS ; i++)
    {
        if (chargerStateCallBackArray[i] != NULL)
        {
            chargerStateCallBackArray[i](chargerState);           
        }
    }
    return OK;
}

static void BATTERY_handleBatteryLow(time_t a_Time, batteryLevelState_t batteryLevelState, bool forceStart)
{
    // stop in case battery low is no longer happen.
    if ((batteryLowLevel20 != batteryLevelState) &&
        (batteryLowLevel10 != batteryLevelState) &&
         (batteryOK_level5 != batteryLevelState) &&
        (batteryLowLevel3 != batteryLevelState))
    {
        chargerInfo.lastEmitBatteryLow = 0;
        return;
    }
    
    if ((forceStart) ||
        ((chargerInfo.lastEmitBatteryLow) && (chargerInfo.lastEmitBatteryLow + BATTERY_LOW_EMIT_PERIOD < a_Time)))
    {
        uprint("BATTERY_EmitBatteryLow\n\r");
        chargerInfo.lastEmitBatteryLow = a_Time;
        BATTERY_EmitBatteryLow(batteryLevelState);
    }
}

bool BATTERY_isBatteryVeryLow(void)
{
    if (chargerInfo.batteryLevelState == batteryVeryLowLevel)
    {
        //uprintf("batteryVeryLowLevel - %d\n\r", chargerInfo.batteryLevelState);
        return true;
    }
    return false;
}

uint8_t BATTERY_GetBatteryLevel(void)
{
    return chargerInfo.batteryLevelState;
}

#ifdef __cplusplus
}
#endif
