#ifdef __cplusplus
 extern "C" {
#endif

/**
  ******************************************************************************
  * @file           : LEDSBasicPlans.c
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
#include "math.h"
//#include <CyLib.h>
#include "LEDSBasicPlans.h"

#include "../App/interrapts.h"
#include "../Modules/debugPort_Module.h"
#include "../Modules/flash_Module.h"

#define MAX_LEDS_PLANS_SIZE (50)
/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint8_t                 flags;              // events
	plan_t*					currentPlan;        // pointer to a plan structure
    uint8_t                 planLoopCounter;    // loop counter
    uint32_t                startTime;          // start timestamp
    uint32_t                planDuration;       // max. plan duration
    uint32_t                _15mSecCounter;     // its module timer (15 milliSec resolution)
    planId_t                planId;             // plan id
    plan_t*                 ledPlans[MAX_LEDS_PLANS_SIZE];           //
}LedsPlanInfo_t;

typedef struct
{	
    color_t                 customPlanColor;    // the color for custom plan
    uint8_t                 numOfLoops;         // how many times we should play the reminder
}LedsPlanConfig_t;

plan_t ledsPlan;

/* Private define ------------------------------------------------------------*/
#define TICK                (1)
/* Private macro -------------------------------------------------------------*/
#define GET_TIME_MS(x)      (x*15)

/* Private variables ---------------------------------------------------------*/
static LedsPlanInfo_t ledsPlanInfo;

static LedsPlanConfig_t LedsPlanConfig = 
{
    .customPlanColor        = {255,255, 255}, //{100,100,100},
    .numOfLoops             = 1
};
//static char array[100];

/* const variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static uint8_t LEDS_Draw(color_t* a_LedsArray, uint8_t a_Size)
{

    // i am init it like that in order to force clear after reset.
    static color_t mirroLedsArray[MAX_OF_LEDS] = {{1,0,0}}; 
    uint32_t bufferToTransmit[MAX_OF_LEDS];
    
    uint8_t changed = 0;
    for (int i = 0 ; i < a_Size ; i++)
    {
        if ((mirroLedsArray[i].r != a_LedsArray[i].r) || 
            (mirroLedsArray[i].g != a_LedsArray[i].g) ||
            (mirroLedsArray[i].b != a_LedsArray[i].b)) 
        {
            changed = 1;
        }
    }
    
    if (!changed)
    {
        //uprint("!changed\n\r");
        // nothing has changed - so do nothing 
        return OK;
    }    
    
    int loc = 0;
    char array[100];
    
    for (int i = 0 ; i < a_Size ; i++)
    {
        mirroLedsArray[i] = a_LedsArray[i];
        
        bufferToTransmit[i] = a_LedsArray[i].g;
        bufferToTransmit[i]<<=8;
        
        bufferToTransmit[i] |= a_LedsArray[i].r;
        bufferToTransmit[i]<<=8;
        
        bufferToTransmit[i] |= a_LedsArray[i].b;
        bufferToTransmit[i]<<=8;        
        
        //sprintf(&array[loc], "(%d %d %d) \n\r", a_LedsArray[i].g, a_LedsArray[i].r, a_LedsArray[i].b);
        //uprint(array);
    }
    //uprint(array);
    
    // Todo - call 1-wire driver for TX.  
    LED_D_Write(0);                         
    CyDelayUs(200);  
    
    // it is bad - but, in order to match the hard requirement (each bit must be 1.2micro +/- 300nano
    // we must do it.
    CyGlobalIntDisable;
    
    // this function receive buffer of words (32bits) but transmit only the 24MSBs ,
    // the size to be given is in bytes.
    extern void OW_writeBuffer(uint32_t* buffer, unsigned int size);
    OW_writeBuffer(bufferToTransmit, a_Size*4);
    
    // enable global interrupt right after....
    CyGlobalIntEnable;       
    //sprintf (array, "%u ", GET_TIME_MS(ledsPlanInfo._15mSecCounter));
    
    //uprint(array);
    /*loc = 0;
    for (int i = 0; i < a_Size; i++)
    {
        loc += sprintf(&array[loc], "(%x %x %x) ", (bufferToTransmit[i]>>24)&0xff , (bufferToTransmit[i]>>16)&0xff, (bufferToTransmit[i]>>8)&0xff);               
    }*/
    //loc += sprintf(&array[loc], "\n\r");
    
    //uprint(array);
    
    return OK;
}

/* Private user code ---------------------------------------------------------*/

/**
* @brief  : init the LEDS module.
* @param  : none
* @param  : 0 - OK, other - will be an error code.
* @retval 
*/

errorCode_t LEDS_Init(plan_t *a_plans[], uint8_t ledPlanArraySize)
{
    uprint("LED INIT\n\r");
    ledsPlanInfo.currentPlan = NULL;
    ledsPlanInfo.startTime = 0;
    ledsPlanInfo.planDuration = 0;
    ledsPlanInfo._15mSecCounter = 0;
    ledsPlanInfo.flags = 0;
    ledsPlanInfo.planLoopCounter = 0;

    if (ledPlanArraySize > MAX_LEDS_PLANS_SIZE)
    {
        uprintf("LED INIT: ERR_OUT_OF_RANGE\n\r");
        return ERR_OUT_OF_RANGE;
    }
    for (int i = 0; i < ledPlanArraySize; i++)
    {
        if (a_plans[i] == NULL)
        {
            //uprint("a_plans[i] == NULL\n\r");
        }
        else
        {
            ledsPlanInfo.ledPlans[i] = a_plans[i];
            //cli_uprintf("numOfLoops %d\n\r", ledsPlanInfo.ledPlans[i]->numOfLoops);
            //cli_uprintf("ledsPlanInfo.size %d\n\r", ledsPlanInfo.ledPlans[i]->size);
        }
    }
    LEDS_Stop();
    LEDS_readSystemParameters();
    return OK;
}	

/**
* @brief : start playing any pre-programed plan.
* @param : a_planId - the plan to run
* @retval: 0 - OK, other - will be an error code.
*/

uint8_t LEDS_Play(const uint8_t a_planId)
{
	if (a_planId >= LED_PLAN_ID_MAX_PLANS)
	{
		uprint("ERR_OUT_OF_RANGE") ;
	    return ERR_OUT_OF_RANGE;
	}
    if (ledsPlanInfo.ledPlans[a_planId] == NULL)
    {
        return ERR_OBJ_NOT_FOUND;
    }
    // 0. stop prev plan
    LEDS_Stop();
    EnableLEDS_Write(1);
    // 1. set plan info
    ledsPlanInfo.startTime = 0;
    //ledsPlanInfo.currentPlan = (plan_t*)plans[a_planId];
    ledsPlanInfo.currentPlan = (plan_t*)ledsPlanInfo.ledPlans[a_planId];
    ledsPlanInfo._15mSecCounter = 0;
    ledsPlanInfo.planLoopCounter = 0;
    ledsPlanInfo.planId = a_planId;
    uprintf("currentPlan ID = %d\n\r", ledsPlanInfo.planId);
    // calculate plan duration
    
    for (int i = 0 ; i < ledsPlanInfo.currentPlan->size ; i++)
    {
        uint32_t ledDuration = ledsPlanInfo.currentPlan->ptr[i].duration + ledsPlanInfo.currentPlan->ptr[i].offset;
        if (ledsPlanInfo.planDuration < ledDuration)
        {
            ledsPlanInfo.planDuration = ledDuration;
        }
    }
    //uprintf("start a LEDs plan duration = %d\n\r", ledsPlanInfo.planDuration);
    // in order to force immediate start , i simulate TICK event
    // 2. start timer 15 milli Sec timer
    WDT2_Start(_15ms_);
	return OK;
}

/**
* @brief  : stop the current plan.
* @param  : none 
* @retval : 0 - OK, other - will be an error code.
*/
uint8_t LEDS_Stop(void)
{
    //uprint("LEDS_Stop\n\r");
    // todo - stop the current plan
    // array to hold the current state of LEDs.
    color_t ledsArray[MAX_OF_LEDS];  
    
    // 1. stop the timer
    WDT2_Stop();
    EnableLEDS_Write(0);
    memset(ledsArray,0, MAX_OF_LEDS*sizeof(ledsArray[0]));
    LEDS_Draw(ledsArray, MAX_OF_LEDS);
    
    if (ledsPlanInfo.currentPlan)
    {
        //uprintf("The current LEDs plan was stopped\\Completed after %u mSec\n\r", GET_TIME_MS(ledsPlanInfo._15mSecCounter));
    }
    
    //2. clear plan info
    ledsPlanInfo.startTime = 0;
    ledsPlanInfo.currentPlan = NULL; 
    ledsPlanInfo.planDuration = 0;
    ledsPlanInfo._15mSecCounter = 0;
    ledsPlanInfo.planId = LED_PLAN_ID_MAX_PLANS;
    return OK;
}

uint8_t LEDS_StopSpecificPlan(uint8_t planId)
{
    if (ledsPlanInfo.planId == planId)
    {
        return LEDS_Stop();
    }
    
    return ERR_OBJ_NOT_FOUND;
}

/**
* @brief  : return active plan id.
* @param  : none
* @retval : 
*/
uint8_t LEDS_returnActivePlanId(void)
{
    return ledsPlanInfo.planId;
}

/**
* @brief  : check if leds are on right now.
* @retval : true if on.
*/
bool LEDS_isActiveNow(void)
{
    if (ledsPlanInfo.currentPlan != NULL)
    {
        return true;
    }
    return false;
}

/**
* @brief  : turn led on or off.
* @param  : led and color. if led == -1 then turn all leds off
* @retval : 0 - OK, other - will be an error code.
*/
uint8_t LEDS_TurnOnOff(const int a_led, const color_t color)
{
    // array to hold the current state of LEDs - used for debug port only.
    static color_t ledsArray[MAX_OF_LEDS] = {    {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},
                                                 {0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
    
    if ((a_led < -1) || (a_led >= MAX_OF_LEDS))
    {
        return ERR_OUT_OF_RANGE;
    }
    
    if (a_led == -1)
    {
        // clear all.
        for (int i = 0 ; i < MAX_OF_LEDS ; i++)
        {
            ledsArray[i].r = 0;
            ledsArray[i].g = 0;
            ledsArray[i].b = 0;
        }
    }
    else
    {
		EnableLEDS_Write(1);
        ledsArray[a_led] = color;
    }
    
    uprintf("LEDS_TurnOnOff = %d\n\r",a_led);
    // its ready, draw it. 
    LEDS_Draw(ledsArray, MAX_OF_LEDS);
	
	// in case we need to clear all then we should also power off
	if (a_led == -1)
	{
		EnableLEDS_Write(0);
	}
    
    return OK;
}

/**
* @brief  : this function tun periodically, it play the current plan.
* @param  : none 
* @retval : 0 - OK, other - will be an error code.
*/
void LEDS_Handler(void)
{
    if (ledsPlanInfo.flags & TICK)
    {
        ledsPlanInfo.flags &= (~TICK);
        // array to hold the current state of LEDs.
        color_t ledsArray[MAX_OF_LEDS];      
    	
        // sanity check - we should not get in.
        if ((ledsPlanInfo.currentPlan == NULL) ||
            (ledsPlanInfo.currentPlan->size == 0) || 
            (ledsPlanInfo.currentPlan->ptr == NULL))
        {
            return;
        }
        
        // check if plan is completed, if so then kill it.
        if ((GET_TIME_MS(ledsPlanInfo._15mSecCounter)) - ledsPlanInfo.startTime > (ledsPlanInfo.planDuration))
        {
            if (++ledsPlanInfo.planLoopCounter == ledsPlanInfo.currentPlan->numOfLoops)
            {
                LEDS_Stop();
            }
            else
            {
                ledsPlanInfo._15mSecCounter = 0;
            }
            return;
        }
        
        // clear before using it.
        for (int i = 0 ; i < MAX_OF_LEDS ; i++)
        {
            ledsArray[i].r = 0;
            ledsArray[i].g = 0;
            ledsArray[i].b = 0;
        }
        
        uint32_t currentTime = GET_TIME_MS(ledsPlanInfo._15mSecCounter);
        
        // just play the plan
        for (int i = 0 ; i < ledsPlanInfo.currentPlan->size ; i++)
        {
            switch (ledsPlanInfo.currentPlan->ptr[i].mode)
            {
                case Stable: 
                    if ((currentTime >= (ledsPlanInfo.currentPlan->ptr[i].offset)) &&
                       (currentTime  <= (ledsPlanInfo.currentPlan->ptr[i].offset + ledsPlanInfo.currentPlan->ptr[i].duration)))
                    {
                        ledsArray[ledsPlanInfo.currentPlan->ptr[i].ledId] = ledsPlanInfo.currentPlan->ptr[i].color;
                    }
                break;
                case Blink:
                    if ((currentTime >= (ledsPlanInfo.currentPlan->ptr[i].offset)) &&
                        (currentTime <= (ledsPlanInfo.currentPlan->ptr[i].offset + ledsPlanInfo.currentPlan->ptr[i].duration)))
                    {
                        if ((((currentTime - (ledsPlanInfo.currentPlan->ptr[i].offset)) / ledsPlanInfo.currentPlan->ptr[i].period) % 2) == 1)
                		{
                			ledsArray[ledsPlanInfo.currentPlan->ptr[i].ledId].r = 0;
                			ledsArray[ledsPlanInfo.currentPlan->ptr[i].ledId].g = 0;
                			ledsArray[ledsPlanInfo.currentPlan->ptr[i].ledId].b = 0;
                		}
                        else
                        {
                            ledsArray[ledsPlanInfo.currentPlan->ptr[i].ledId] = ledsPlanInfo.currentPlan->ptr[i].color;
                        }
                    }
                break;
                
                case Breathing2:    
                case Breathing:
                {
                    int cycleDuration = 3000;
                    
                    if (Breathing2 == (ledsPlanInfo.currentPlan->ptr[i].mode))
                    {
                        cycleDuration = 1000;
                    }
                    
                    const int halfCycle = cycleDuration / 2;
                                        

                    if ((currentTime >= (ledsPlanInfo.currentPlan->ptr[i].offset)) &&
                       (currentTime  <= (ledsPlanInfo.currentPlan->ptr[i].offset + ledsPlanInfo.currentPlan->ptr[i].duration)))
                    {
                        uint32_t elapsed = (currentTime - ledsPlanInfo.currentPlan->ptr[i].offset) % (cycleDuration);
            			float persent = (float)elapsed / cycleDuration;
            			if( elapsed > halfCycle)
            			{
            				persent = 1-persent;
            			}
            			persent*=2;

                        ledsArray[ledsPlanInfo.currentPlan->ptr[i].ledId].r = ledsPlanInfo.currentPlan->ptr[i].color.r * persent;
                        ledsArray[ledsPlanInfo.currentPlan->ptr[i].ledId].g = ledsPlanInfo.currentPlan->ptr[i].color.g * persent;
                        ledsArray[ledsPlanInfo.currentPlan->ptr[i].ledId].b = ledsPlanInfo.currentPlan->ptr[i].color.b * persent;
                    }
                }
                break;
                default:
                break;
            }
        }
        // its ready, draw it.
        LEDS_Draw(ledsArray, MAX_OF_LEDS);
    }
}

/**
* @brief  : write custom leds plan to ext. flash.
* @param  : none
* @param  : none.
* @retval 
*/
void LEDS_readSystemParameters(void)
{
    if (ledsPlanInfo.currentPlan == NULL)
    {
        LEDS_Stop();
    }
    
    UserFlash_readParam(PARAM_REMINDER_LEDS_COLOR, &LedsPlanConfig.customPlanColor, sizeof(LedsPlanConfig.customPlanColor));
    //uprintf("custom plan color is updating...\n\r");
    //uprintf("reminderPlan.ptr = %d, ledsReminderPlan = %d \n\r", reminderPlan.ptr, ledsReminderPlan);
        
    // Change the color according to the user's wishes
    //uprintf("reminderPlan.size = %d\n\r", reminderPlan.size);
    for (int i = 0 ; i < ledsPlanInfo.ledPlans[LED_PLAN_ID_REMINDER]->size ; i++)
    {
        ledsPlanInfo.ledPlans[LED_PLAN_ID_REMINDER]->ptr[i].color = LedsPlanConfig.customPlanColor;
    }
    //uprintf("reminderBouncePlan.size = %d\n\r", reminderBouncePlan.size);
    for (int i = 0 ; i <  ledsPlanInfo.ledPlans[LED_PLAN_ID_REMINDER_BOUNCE]->size  ; i++)
    {
        ledsPlanInfo.ledPlans[LED_PLAN_ID_REMINDER_BOUNCE]->ptr[i].color = LedsPlanConfig.customPlanColor;
    }
    //uprintf("reminderPulsePlan.size = %d\n\r", reminderPulsePlan.size);
    for (int i = 0 ; i < ledsPlanInfo.ledPlans[LED_PLAN_ID_REMINDER_PULSE]->size ; i++)
    {
        ledsPlanInfo.ledPlans[LED_PLAN_ID_REMINDER_PULSE]->ptr[i].color = LedsPlanConfig.customPlanColor;
    }

    //uprintf("reminder num of cycles is updating...\n\r");
    /*UserFlash_readParam(PARAM_REMINDER_CYCLES, &LedsPlanConfig.numOfLoops, sizeof(LedsPlanConfig.numOfLoops));
    reminderPlan.numOfLoops = LedsPlanConfig.numOfLoops;    
    reminderPulsePlan.numOfLoops = LedsPlanConfig.numOfLoops;
    reminderBouncePlan.numOfLoops = LedsPlanConfig.numOfLoops;*/
}
void LEDS_ShowLowBatteryLevel(uint8_t leds)
{
    // some of the plans uses the same leds array (located in RAM).
    // so, before we prepare these kind of plans we must verify no one else is using it (how - by stopping the active plan).
    // so, we check first if the plan to be run is enabled (to be sure we don't stop a plan if not required)
    // if so, lets stop immediate the active (current) plan.
    if (ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL] == NULL)
    {
        return;
    }
    else
    {
        LEDS_Stop();
    }
    if (0 == leds)
    {
        return;
    }

    LedSettings_t led =
        {
            .offset     = 0,
            .duration   = 500,
            .color      = {8, 0, 0},
            .mode       = Blink,
            .period     = 255,
        };
    
    ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->ptr->color = led.color;
    ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->ptr->duration = led.duration;
    ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->ptr->ledId = led.ledId;
    ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->ptr->mode = led.mode;
    ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->ptr->period = led.period;
    ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->numOfLoops = 5;
    uprintf("plans[LED_PLAN_ID_BATTERY_LEVEL].color.r %d\n\r", ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->ptr->color.r);

    int i = 0;
    
    for (i = 0 ; i < leds ; i++)
    {
        led.ledId = i;
        ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->ptr[i] = led;
    }
    
    ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->size = i;
    uprintf("plans[LED_PLAN_ID_LOW_BATTERY_LEVEL].size = %d\n\r", ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->size);
    uprintf("batt level = %d\t , i = %d\n\r", leds, i);
    LEDS_Play(LED_PLAN_ID_BATTERY_LEVEL);
}

/**
* @brief  : when bottle is in charging state - use this function show battery level.
* @param  : level (%)
* @param  : none.
* @retval 
*/
void LEDS_ShowBatteryLevel(uint8_t level)
{
    // some of the plans uses the same leds array (located in RAM).
    // so, before we prepare these kind of plans we must verify no one else is using it (how - by stopping the active plan).
    // so, we check first if the plan to be run is enabled (to be sure we don't stop a plan if not required)
    // if so, lets stop immediate the active (current) plan.
    if (ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL] == NULL)
    {
        uprint("LED_PLAN_ID_BATTERY_LEVEL - Retur\n\r");
        return;
    }
    else
    {
        LEDS_Stop();
    }

    LedSettings_t led =
    {
        .offset     = 0,
        .duration   = MAX_INT32,
        .color      = {0, 8, 0},
        .mode       = Stable,
        .period     = 0
    };
    
    float resolution = (float)100 / (float)MAX_OF_LEDS;
    
    //uprintf("resolution = %d, level = %d\n\r", resolution, level);
    
    uint8_t steps = level / resolution;
    
    //uprintf("steps = %d\n\r", steps);
    
    *ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->ptr = led;
        
    int i = 0; 
    for (i = 0 ; i < steps ; i++)
    {
        led.ledId = i;
        ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->ptr[i] = led;
    }

    // the last led should be blinking
    led.ledId = i;
    led.mode = Blink;
    led.period = 250;
    
    ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->ptr[i++] = led;

    ledsPlanInfo.ledPlans[LED_PLAN_ID_BATTERY_LEVEL]->size = i;
    //uprintf("batteryLevel = %d , i = %d, steps = %d\n\r", level, i, steps);
    
    LEDS_Play(LED_PLAN_ID_BATTERY_LEVEL);
}

/**
* @brief  : when pressing a button on top, hydration level will be presented.
* @param  : level (%)
* @param  : none.
* @retval 
*/

void LEDS_ShowHydrationLevel(uint8_t level, uint8_t supposedToDrink, bool longDuration)
{
    // some of the plans uses the same leds array (located in RAM).
    // so, before we prepare these kind of plans we must verify no one else is using it (how - by stopping the active plan).
    // so, we check first if the plan to be run is enabled (to be sure we don't stop a plan if not required)
    // if so, lets stop immediate the active (current) plan.
    if (ledsPlanInfo.ledPlans[LED_PLAN_ID_HYDRATION_LEVEL] == NULL)
    {
        uprint("LED_PLAN_ID_HYDRATION_LEVEL - Return\n\r");
        return;
    }
    else
    {
        LEDS_Stop();
    }
    uint32_t duration = 3000;
    if (longDuration)
    {
        duration = 15000;
    }
    LedSettings_t led = {   .offset    = 0,
                            .duration  = duration,
                            .color     = LedsPlanConfig.customPlanColor,   //{16,8,0},
                            .mode      = Stable,
                            .period    = 0};
    uint8_t steps = (uint8_t)((level * MAX_OF_LEDS)/100 + 0.5);
    uint8_t supposedToDrinkLevel = (uint8_t)((supposedToDrink*MAX_OF_LEDS)/100 +0.5);
    uprintf("steps = %d #leds\n\r", steps);
    uprintf("supposedToDrink = %d #leds\n\r", supposedToDrinkLevel);
    ledsPlanInfo.ledPlans[LED_PLAN_ID_HYDRATION_LEVEL]->size = steps;
    *ledsPlanInfo.ledPlans[LED_PLAN_ID_HYDRATION_LEVEL]->ptr = led;
    
    #define DELAY (30)
    int i = 0; 
    for (i = 0 ; i <= steps ; i++)
    {
        led.ledId = i;
        led.offset = i*DELAY;
        led.duration = duration + ((steps+1-i)*DELAY*2);
        ledsPlanInfo.ledPlans[LED_PLAN_ID_HYDRATION_LEVEL] ->ptr[i] = led;
    }
    if (supposedToDrinkLevel >= MAX_OF_LEDS)
    {
        supposedToDrinkLevel = MAX_OF_LEDS-1;
    }
    led.ledId = supposedToDrinkLevel;
    led.mode = Blink;
    led.period = 100;
    led.duration = duration;// + ((steps+1)*DELAY*2)
    ledsPlanInfo.ledPlans[LED_PLAN_ID_HYDRATION_LEVEL] ->ptr[i++] = led;
    ledsPlanInfo.ledPlans[LED_PLAN_ID_HYDRATION_LEVEL] ->size = i;
    ledsPlanInfo.ledPlans[LED_PLAN_ID_HYDRATION_LEVEL] -> numOfLoops = 1;
    uprintf("LED PLAY - LED_PLAN_ID_HYDRATION_LEVEL\n\r");
    LEDS_Play(LED_PLAN_ID_HYDRATION_LEVEL);
}

/**
* @brief  : when pressing a button on top, hydration level will be presented.
* @param  : level (%)
* @param  : none.
* @retval 
*/

errorCode_t LEDS_ShowHydrationLevelCoachMode(hydrationLevel_t hydrationLevel, uint8_t stage, uint8_t numOfStages, bool goalReached, bool longDuration)
{
    if ((numOfStages == 0) || (stage > numOfStages))
    {
        return ERR_PARAMETER_IS_INVALID;
    }

    // some of the plans uses the same leds array (located in RAM).
    // so, before we prepare these kind of plans we must verify no one else is using it (how - by stopping the active plan).
    // so, we check first if the plan to be run is enabled (to be sure we don't stop a plan if not required)
    // if so, lets stop immediate the active (current) plan.
    if ( ledsPlanInfo.ledPlans[LED_PLAN_ID_HYDRATION_LEVEL] == NULL)
    {
        return ERR_OBJ_NOT_FOUND;
    }
    
    LEDS_Stop();

     /*
    runTimePlan.ptr = ledsRunTime;
    
    // set duration
    uint32_t duration = 3000;
    if (longDuration)
    {
        duration = 15000;
    }
         
    // in order to support coach mode, we divide it to ranges
    uint8_t sizeOfRange = MAX_OF_LEDS / numOfStages;

     */

    /* Initializes random number generator */

    /*
    #include "RTC_Module.h"
    
    srand((unsigned) RTC_GetUnixTimeAndDate());
    
    #define DELAY (30)
    
    int ledInndex = 0; 
    for (ledInndex = ledInndex ; ledInndex < (stage - 1)*sizeOfRange ; ledInndex++)
    {
        // create led instance
        LedSettings_t led = {   .ledId      = ledInndex,
                                .offset     = ledInndex*DELAY,
                                .duration   = duration + (MAX_OF_LEDS-ledInndex)*DELAY*2,
                                .color      = {0,16,0},
                                .mode       = Stable, 
                                .period     = 0};
                
        if (goalReached)
        {
            led.color = (color_t) {0, 16 + rand() % 240, 0};
            led.mode = Breathing;
        }
        ledsRunTime[ledInndex] = led;
    }
    
    uint8_t steps = (uint8_t)((hydrationLevel.level * sizeOfRange)/100 + 0.5);
    
    uint8_t supposedToDrinkLevel = (uint8_t)((hydrationLevel.supposedToDrink*sizeOfRange)/100 + 0.5);
    
    if (supposedToDrinkLevel > sizeOfRange)
    {
        supposedToDrinkLevel = sizeOfRange-1;
    }
    
    uprintf("level = %d%%\t steps = %d\n\r",hydrationLevel.level, steps);
    uprintf("ledInndex = %d\t steps+offset = %d, supposedToDrinkLevel  = %d\n\r",ledInndex, (stage - 1)*sizeOfRange + steps, supposedToDrinkLevel);
      
    
    for (ledInndex = ledInndex ; ledInndex  < (stage - 1)*sizeOfRange + steps ; ledInndex++)
    {
        LedSettings_t led = {   .ledId      = ledInndex,
                                .offset     = ledInndex*DELAY,
                                .duration   = duration + (MAX_OF_LEDS-ledInndex)*DELAY*2,
                                .color      = LedsPlanConfig.customPlanColor,
                                .mode       = Stable, 
                                .period     = 0};
        
        if (goalReached)
        {
            led.color = (color_t) {0, 16 + (rand() % 240), 0};
            led.mode = Breathing;
        }
        
        ledsRunTime[ledInndex] = led;
    }
    
    // 
    if (hydrationLevel.level < 100)
    {
        LedSettings_t led;
        led.ledId = supposedToDrinkLevel+((stage-1)*sizeOfRange)-1;
        led.mode = Blink;
        led.offset = ledInndex*DELAY,
        led.period = 100;
        led.color = LedsPlanConfig.customPlanColor;
        led.duration = duration + (MAX_OF_LEDS-ledInndex+1)*DELAY*2;// duration;
        ledsRunTime[ledInndex++] = led;
    }
    
    runTimePlan.size = ledInndex;
    
    LEDS_Play(LED_PLAN_ID_HYDRATION_LEVEL);
     */

    return OK;
}

/**
* @brief  : called when time was updated.
* @param  : none
* @param  : none.
* @retval 
*/
void LEDS_tick(void)
{
    //uprintf("tick\n\r");
    // increment 15miliSec counter
    ledsPlanInfo._15mSecCounter++;
    ledsPlanInfo.flags |= TICK;
}

#ifdef __cplusplus
}
#endif
