#include "cap.h"
#include "i2cDriver.h"
#include "ble_stack.h"
#include "flash.h"
#include "log.h"
#include "rtc.h"
#include "main.h"

const unsigned char base_version[16] = "SA02.0028";
const unsigned char st_version[16] =   "SA03.01.017";

#define PACKET_OVERHEAD (10)
    
//#define EASY_TEST
#define ADVANCED_MEASUREMENT
#define ADDITIONAL_MEASUREMENTS_AFTER_VALIDATION
#define MEASUREMENT_FORCED_CORRECTION
#ifndef USE_UART_FOR_LOG
//#define USE_UART_FOR_LOG
#endif

// Humidity and Temperature Sensor IC
#include "peripherals/aht10Driver.h"
#include "peripherals/microPressureDriver.h"

typedef enum
{
    TEST_BUTTON_EVENT,
    DEBUG_PORT_EVENT,
    SCHEDULER_EVENT,    
    MAGNET_ATTACH_EVENT,
    MAGNET_DETACH_EVENT,
    SHOCK_EVENT    
}TEventDriven;

typedef enum
{
 DutyCycle_HighVoltage = 750,      // 25% from 3000
 DutyCycle_LowVoltage = 150,       // 5%  from 3000
 DutyCycle_Voltage_OFFSET = 250    // mV  ** value for test only: 333 mV from powerSuply and 270 mV from Battery
}DC_LEVEL;

// test buton pressed interrupt
static uint8_t          g_testBoxIntOccured = 0;
//flag to indicate from interrupt that magnet is attached \ detached
static uint8_t          g_magneticSwitchIntOccured = 0;
// uart RX flag 
static uint8_t          g_rxIntOccured = 0;
// scheduling period
static uint16_t         g_samplingPeriod = 0;
// pressure offset - read from flash
static long             g_pressureOffset = 0;
// count time to sleep from magent attached event - right now it is 20 seocnds
static unsigned long    g_magnetClosed_TimeToSleep =0;
// indicate if we enter sleep mode 
static uint8_t          g_sleepHappened = 0;
// last pressure result
static short            g_lastPressure = 0;
// last hummidity result
static int16_t          g_lastHummidity = 0;
// last temperature result
static int16_t          g_lastTemperature = 0;
// hold next scheduling time 
static unsigned long    g_ATPNextTime = 0;
// count time to sleep from last debug port activity
static unsigned long    g_debugPortTimeout = 0;
// warehouse period - for advertising after scheduler event
static uint16_t         g_warehosePeriod = 0;
// last warehouse time
static unsigned long    g_lastWarehouseTime = 0;

static void CAP_getTemperature(uint16_t* measure,int16_t* cel);
static int handleTestBoxInt(TEventDriven a_eventDriven, uint16_t a_shockG);
static uint16_t getSamplingPeriod(void);
static void startSamplingPeriod(void);
void CAP_checkTimeToEnableShockInt(void);
//debug port
int handleRxInt(void);
int8_t CAP_checkDebugPortTimeout(void);
void IOSleep(void);
void IOWakeup(void);
void peripheralsPowerOn(void);
void peripheralsPowerOff(void);
void CAP_Sleep(void);
void CAP_stopBuzzer(void);
void CAP_startBuzzer(uint16_t time, uint16_t DutyCycle);


// for Yossef's calibration 
#define PRESSURE_FACTOR_MILLI (0)
#define PRESSURE_FACTOR       (PRESSURE_FACTOR_MILLI*10)

#ifdef USE_UART_FOR_LOG
#include "uartDriver.h"
#endif

#define BUZZER_IND_TIMEOUT (100)

void CAP_SetWarehousePeriod(uint16_t a_warehosePeriod)
{
    g_warehosePeriod = a_warehosePeriod;
    
    // update lastWarehouseTime when a_warehosePeriod is changed.
    g_lastWarehouseTime = st_raw_time();
    
}

//// ADC
#include "ADC_SAR_Seq.h"
static short ADC_Read(unsigned int channel)
{
    short result = 0;
    ADC_SAR_Seq_Start(); /* Initialize ADC */
    ADC_SAR_Seq_IRQ_Disable(); /* Disable ADC interrupts */
    ADC_SAR_Seq_IRQ_ClearPending(); /* Clear pending interrupt */
    ADC_SAR_Seq_StartConvert(); /* Start ADC conversions */
    ADC_SAR_Seq_IsEndConversion(ADC_SAR_Seq_WAIT_FOR_RESULT); /* Wait for ADC conversion complete */
    result = ADC_SAR_Seq_GetResult16(channel); /* Get the data */
    ADC_SAR_Seq_StopConvert(); /* Stop ADC conversions */
    ADC_SAR_Seq_Stop(); /* Unitialize ADC */
    return result;
}
#define ADC_Battery_Channel 0
#define ADC_Battery_Coef    (3.322) // 11
#define ADC_VREF            (1.024)
#define ADC_RES             (2048)
static unsigned short ADC_ReadBattery(unsigned short* a_rawData)
{
    unsigned short rawData;
    
    rawData = ADC_Read(ADC_Battery_Channel);
    
    if (a_rawData != NULL)
    {
        *a_rawData = rawData;
    }
    
    return (unsigned short)(((rawData * ADC_Battery_Coef*ADC_VREF)*100)/2048);
}

//// Delay
#include "CyLib.h"
/*static void DelayMs(unsigned long milliseconds)
{
    CyDelay(milliseconds);
}*/
////

//// Helper
static void st_serialize_uint16(unsigned char *s, unsigned short n)
{
    s[0] = (unsigned char)(n & 0x00FF);
    s[1] = (unsigned char)((n >> 8) & 0x00FF);
}
static unsigned short st_deserialize_uint16(const unsigned char *s)
{
    unsigned short n;
    n = s[1];
    n <<= 8;
    n += s[0];
    return n;
}
static void st_serialize_uint32(unsigned char *s, unsigned long n)
{
    s[0] = (unsigned char)(n & 0x000000FF);
    s[1] = (unsigned char)((n >> 8) & 0x000000FF);
    s[2] = (unsigned char)((n >> 16) & 0x000000FF);
    s[3] = (unsigned char)((n >> 24) & 0x000000FF);
}
static unsigned long st_deserialize_uint32(const unsigned char *s)
{
    unsigned long n;
    n = s[3];
    n <<= 8;
    n += s[2];
    n <<= 8;
    n += s[1];
    n <<= 8;
    n += s[0];
    return n;
}
////

//// Idle Timeout
static unsigned long st_idle_timeout = 30;
static unsigned long st_idle_timeout_till = 0;
static void st_idle_timeout_apply(void)
{
    if (st_idle_timeout)
        st_idle_timeout_till = RTC_Value + st_idle_timeout;
    else
        st_idle_timeout_till = 0;
}
static int st_idle_timeout_is_reached(void)
{
    if ((!st_idle_timeout) || (!st_idle_timeout_till))
        return 0;
    if (st_idle_timeout_till > RTC_Value)
        return 0;
    return 1;
}
////

//// Go Sleep
static int st_go_sleep = 0;
////

enum
{
    // Requests and events
    Cap_Cmd_RunSingleMeasurement = 0x01,
    Cap_Cmd_RunContinuousMeasurment = 0x02,
    // Requests
    Cap_Cmd_StopContinuousMeasurement = 0x03,
    Cap_Cmd_GetLastOpen = 0x04,
    Cap_Cmd_GetLastClose = 0x05,
    Cap_Cmd_GetLastTouchStart = 0x06,
    Cap_Cmd_GetLastTouchEnd = 0x07,
    Cap_Cmd_GetLastReminder = 0x08,
    Cap_Cmd_GetLastMeasurement = 0x09,
    Cap_Cmd_GetMeasureLogLength = 0x0a,
    Cap_Cmd_GetMeasureLog = 0x0b,
    Cap_Cmd_ClearMeasureLog = 0x0c,
    Cap_Cmd_GetLogLength = 0x0d,
    Cap_Cmd_GetLog = 0x0e,
    Cap_Cmd_ClearLog = 0x0f,
    Cap_Cmd_GetRtc = 0x10,
    Cap_Cmd_SetRtc = 0x11,
    Cap_Cmd_SetSleep = 0x12,
    Cap_Cmd_SetMesurementCount = 0x15,
    Cap_Cmd_SetBlinkingTime = 0x16,
    Cap_Cmd_RunBlinking = 0x1a,
    Cap_Cmd_GetBatteryVoltage = 0x1b,
    Cap_Cmd_GetTemperature = 0x1c,
    Cap_Cmd_GetVersion = 0x1d,
    Cap_Cmd_GetIdleTimeout = 0x1e,
    Cap_Cmd_SetIdleTimeout = 0x1f,
    Cap_Cmd_SetHourHydrationGoal = 0x20,
    Cap_Cmd_SetTotalDailyHydrationGoal = 0x21,
    Cap_Cmd_SetDailyUsageTime = 0x22,
    Cap_Cmd_SetHydrationSamplingTime = 0x23,
    Cap_Cmd_ImHere = 0x24,
    Cap_Cmd_SetMeasureToVolume = 0x25,
    Cap_Cmd_StartReminder = 0x26,
    Cap_Cmd_StopReminder = 0x27,
    Cap_Cmd_SetMaxStd = 0x28,
    Cap_Cmd_SetMinVolumeChange = 0x29,
    Cap_Cmd_GetHydrationValue = 0x2a,
    Cap_Cmd_SetHydrationValue = 0x2b,
    Cap_Cmd_GetAdvertisementID = 0x2c,
    Cap_Cmd_SetAdvertisementID = 0x2d,
    //
    Cap_Cmd_MakeSweep = 0x35,
    Cap_Cmd_GetSweepReply = 0x36,
    Cap_Cmd_GetSweepReplyFFT = 0x37,
    //
    Cap_Cmd_SetGyroDetectParams = 0x3a,
    Cap_Cmd_GetGyroDetectParams = 0x3b,
    Cap_Cmd_GetTempGyroAccelDump = 0x3d,
    Cap_Cmd_GetGyroFIFOState = 0x3e,
    Cap_Cmd_GetGyroFIFOChunk = 0x3f,
    
    Cap_Cmd_SetAcclSampleRate = 0x40,
    Cap_Cmd_SetAcclFullScaleAndThreshold = 0x41,
    Cap_Cmd_GetNextLogs = 0x42,
    
    Cap_Cmd_SetSamplingPeriod = 0x43,
    Cap_Cmd_GetSamplingPeriod = 0x44,
    Cap_Cmd_GetExecutiveSummary = 0x45,
    Cap_Cmd_Generate1000FakeEvents = 0x46,
    Cap_Cmd_SetAdvPeriodInarehouseMode = 0x47,
	Cap_Cmd_SetDutyCyclePeriodForCheckBuzzer = 0x48,
    
    // Events
    Cap_Cmd_BlinkingWasInitiated = 0x51,
    Cap_Cmd_Opened = 0x52,
    Cap_Cmd_Closed = 0x53,
    Cap_Cmd_TouchStarted = 0x54,
    Cap_Cmd_TouchEnded = 0x55,
    Cap_Cmd_Ack = 0x5a
};

#define Cap_CmdPos 0
#define Cap_CmdPayloadLenPos 3
#define Cap_CmdPrefixLen 4

static int Cap_IsActive = 0;

int Cap_Start()
{
    int ret;
    ret = 0;
    char txt[40];
    //char hWVersion[16];
    #ifdef USE_UART
    UARTCopy_Reset();
    #endif
        
    LogsInit();
    uint32 reason = CySysGetResetReason(CY_SYS_RESET_WDT | CY_SYS_RESET_PROTFAULT | CY_SYS_RESET_SW);
    LogsWriteResetReason(reason);
    I2CMaster_Init();
    
    UART_UartPutString("CapStart\n\r");
    switch (reason)
    {
        case CY_SYS_RESET_WDT:
        {
            UARTCopy_SendStr("last reset reason is WDT\n\r");
        }
        break;
        case CY_SYS_RESET_PROTFAULT:
        {
            UARTCopy_SendStr("last reset reason is protection violation\n\r");
        }
        break;
        case CY_SYS_RESET_SW:
        {
            UARTCopy_SendStr("last reset reason is SW\r");
        }
        break;
    }
    
    sprintf(txt, "base version - %s\n\r", base_version);
    UARTCopy_SendStr(txt);
    sprintf(txt, "SealedAir version - %s\n\r", st_version);
    UARTCopy_SendStr(txt);   
    //UserFlash_readHWVersion(hWVersion);
    //sprintf(txt, "HW version - %s\n\r", hWVersion);
    //UARTCopy_SendStr(txt); 
    
    Cap_IsActive = ret;
    return Cap_IsActive;
}

int Cap_IsStarted()
{
    return Cap_IsActive;
}

void Cap_Stop()
{
    Cap_IsActive = 0;
}

void Cap_OnConnect()
{
    Cap_Start();
    LogGeneralEvent(GeneralEvent_Code_AppConnected, 0);
    UARTCopy_SendStr("connect\n\r"); 
    st_idle_timeout_apply();
    st_go_sleep = 0;
    LEDs_Off();
}

#define Cap_MaxDataLen 250
static int st_data_filled = 0;
static unsigned char st_data[Cap_MaxDataLen];
static unsigned short st_data_len = 0;
#ifdef USE_UART
static unsigned char st_data_from_uart[Cap_MaxDataLen];
#endif

int Cap_OnReceive(const unsigned char *data, unsigned short len)
{
    int i;
    st_idle_timeout_apply();
    if ((!data) || (len > sizeof(st_data)))
        return 0;
    if (!Cap_IsStarted())
    {
        //st_data_filled = 0;
        //return 0;
    }
    if (!len)
        return 1;
    if (st_data_filled)
        st_data_filled = 0;
    st_data_filled = 0;
    for (i = 0; i < len; i++)
        st_data[i] = data[i];
    st_data_len = len;
    st_data_filled = 1;
    return 1;
}

// int Cap_Send(const unsigned char *data, unsigned short len); // Should be implemented by caller

static int st_Cap_Send(const unsigned char *data)
{
    unsigned short len = Cap_CmdPrefixLen + data[Cap_CmdPayloadLenPos];
    st_idle_timeout_apply();
    if (len > Cap_MaxDataLen)
        return 0;
    #ifdef USE_UART
    UARTCopy_Send(data, len);
    #endif
    return Cap_Send(data, len);
}

//static int st_switch_pressed = 0;

static unsigned char st_measurements_std_max = 4;
#include <math.h>

static int st_measurements_to_do = 0;
//static int st_measurements_count = 5;
static int st_sec_ticks_defined = 0;
static int st_sec_ticks = 100;
static int st_sec_ticks_pre = 0;
static int st_sec_ticks_definition_step = 0; // 0 - none, 1 - pre, 2 - set
static unsigned long st_sec_ticks_last_rtc = 0;
static void st_calc_ticks(void)
{
    unsigned long rtc;
    if (st_sec_ticks_defined)
        return;
    switch (st_sec_ticks_definition_step)
    {
        case 0:
            st_sec_ticks_last_rtc = st_raw_time();
            st_sec_ticks_definition_step = 1;
            break;
        case 1:
            rtc = st_raw_time();
            if (rtc != st_sec_ticks_last_rtc)
            {
                if (rtc == (st_sec_ticks_last_rtc + 1))
                {
                    st_sec_ticks_pre = 0;
                    st_sec_ticks_definition_step = 2;
                }
                st_sec_ticks_last_rtc = rtc;
            }
            break;
        case 2:
            rtc = st_raw_time();
            ++st_sec_ticks_pre;
            if (rtc != st_sec_ticks_last_rtc)
            {
                if (rtc == (st_sec_ticks_last_rtc + 1))
                {
                    st_sec_ticks = st_sec_ticks_pre;
                    st_sec_ticks_defined = 1;
                }
                else
                {
                    st_sec_ticks_last_rtc = rtc;
                    st_sec_ticks_definition_step = 1;
                }
            }
            break;
        default:
            break;
    }
}

#define ALT_BLINKING
#ifndef ALT_BLINKING
static int st_blinks_count = 3;
static int st_blink_on_time = 1;
static int st_blink_off_time = 1;
static int st_blinks_to_do = 0;
static int st_blink_state = 0;
static int st_blink_side = 0;
static int st_blink_counter = 0;
static void st_blink_setup(int on_time, int off_time, int count)
{
    st_blinks_to_do = 0;
    st_blink_state = 0;
    st_blink_counter = 0;
    st_blink_on_time = (on_time * st_sec_ticks) >> 1;
    st_blink_off_time = (off_time * st_sec_ticks) >> 1;
    st_blinks_count = count;
}
static void st_blink_start(void)
{
    st_blinks_to_do = st_blinks_count;
    st_blink_state = 0;
    st_blink_counter = 0;
}
static void st_blink(void)
{
    if (st_blinks_to_do)
    {
        ++st_blink_counter;
        if (st_blink_state)
        {
            if (st_blink_counter >= st_blink_on_time)
            {
                LEDs_Off();
                st_blink_state = 0;
                st_blink_counter = 0;
                --st_blinks_to_do;
            }
        }
        else
        {
            if (st_blink_counter >= st_blink_off_time)
            {
                st_blink_side = !st_blink_side;
                if (st_blink_side)
                    LEDs_A_On();
                else
                    LEDs_C_On();
                st_blink_state = 1;
                st_blink_counter = 0;
            }
        }
    }
    else
    {
        LEDs_Off();
        st_blink_state = 0;
        st_blink_counter = 0;
    }
}
#else
static int st_altblinks_count = 3;
static void st_blink_setup(int on_time, int off_time, int count)
{
    if (on_time == off_time)
        ;
    st_altblinks_count = count;
}
static void st_blink(void)
{
}
#endif

#ifdef EASY_TEST
#define REMINDER_TEST
#endif
static int st_hour_hydration_goal = 125;
static int st_total_daily_hydration_goal = 2200;
#define DAY_IN_SECONDS 86400
#define HOUR_IN_SECONDS 3600
#define MINUTE_IN_SECONDS 60
#ifdef REMINDER_TEST
static int st_daily_usage_time_from = 0;
//static int st_daily_usage_time_to = 0;
static int st_daily_usage_time_to = 8 * HOUR_IN_SECONDS;;
static unsigned long st_hydration_sampling_time = 3 * MINUTE_IN_SECONDS;
#else
static int st_daily_usage_time_from = 8 * HOUR_IN_SECONDS;
static int st_daily_usage_time_to = 20 * HOUR_IN_SECONDS;
static unsigned long st_hydration_sampling_time = 30 * MINUTE_IN_SECONDS;
#endif
//#define MEASURE_TO_VOLUME_COUNT 50
//static int st_measure_to_volume[MEASURE_TO_VOLUME_COUNT];
//static int st_measure_to_volume_filled = 0;
static int st_reminder_active = 1;
static int st_was_remind = 0;
static unsigned long st_last_hydration_check_time = 0;
//static int st_reminder_is_blinking = 0;
#define DEFAULT_FULL_VOLUME 920
#define DEFAULT_CM_VOLUME 70
#define REMINDER_BLINKING_COUNT 4
#ifdef REMINDER_TEST
static unsigned long st_reminder_blink_time[REMINDER_BLINKING_COUNT] =
    { 0, 3 * MINUTE_IN_SECONDS, 6 * MINUTE_IN_SECONDS, 9 * MINUTE_IN_SECONDS };
#else
//static unsigned long st_reminder_blink_time[REMINDER_BLINKING_COUNT] =
//    { 0, 30 * MINUTE_IN_SECONDS, 60 * MINUTE_IN_SECONDS, 90 * MINUTE_IN_SECONDS };
#endif
#define REMINDER_BLINKING_MULT 3
#define REMINDER_BLINKING_PAUSE (5 * MINUTE_IN_SECONDS)
#define REMINDER_BLINKING_SUBCOUNT 3
#define REMINDER_BLINKING_TIME_ON 3
#define REMINDER_BLINKING_TIME_OFF 7
#define REMINDER_BLINKING_SUBTIME (REMINDER_BLINKING_TIME_ON + REMINDER_BLINKING_TIME_OFF)
#define REMINDER_BLINKING_TIME (REMINDER_BLINKING_SUBCOUNT * REMINDER_BLINKING_SUBTIME)
#define REMINDER_BLINKING_TIMEOUT (REMINDER_BLINKING_TIME + REMINDER_BLINKING_PAUSE)
#define REMINDER_BLINKING_TIMEOUT_SEQ (REMINDER_BLINKING_TIMEOUT * REMINDER_BLINKING_MULT)
static unsigned long st_hydration_last_measured_time = 0;
//static int st_hydration_last_measured_length = 0;
//static int st_hydration_last_measured_volume = 0;
#define MEASURE_LENGTH_ERROR 5
static unsigned char st_hydration_volume_change_min = 50;
static int st_hydration_behavior_set = 0;
static double st_hydration_daily_sum = 0.0;
//static double st_ml_per_sec;
//static int st_hydration_repeat_measurement = 0;


//static int st_acc_moved = 0;
//static int st_gyro_sign = 0;
//static int st_gyro_lp_count = 0;
//static int st_gyro_lp_result = 0;
//static int st_gyro_threshold = 5000;
//static int st_gyro_threshold_for_open = 3000;
//static int st_gyro_lp_count_enough = 5;
#define OC_GYRO_STATE_WAIT_FOR_INTERRUPT 1
#define OC_GYRO_STATE_COLLECT 2
//static int st_gyro_state = 0;
//static unsigned long st_acc_moved_first_time = 0;
//static unsigned long st_acc_moved_last_time = 0;
//static unsigned long st_gyrostop_time_to_wait_after_acc_moved = 3;
//#define GYROSTOP_MIDDLE_CHECK_TIME 10
//static int st_gyrostop_was_middle_check = 0;
//static short st_gyrostop_polarity = 1;
static unsigned char st_gyrostop_turn_detected = 0;
//static short st_gyrostop_fifo_copy[DOF_FIFO_MAX_COUNT / 3];
static int st_gyrostop_fifo_copy_count = 0;
#if 0
static unsigned char st_gyrostop_fifo_copy_get_chunk(const unsigned char *target, int starting_from)
{
    int count, i;
    unsigned char *p;
    if ((!target) || (starting_from < 0))
        return 0;
    if (starting_from >= st_gyrostop_fifo_copy_count)
        return 0;
    if ((starting_from + 8) > st_gyrostop_fifo_copy_count)
        count = st_gyrostop_fifo_copy_count - starting_from;
    else
        count = 8;
    p = (unsigned char *)target;
    for (i = 0; i < count; i++)
    {
        st_serialize_uint16(p, (unsigned short)st_gyrostop_fifo_copy[starting_from + i]);
        p += 2;
    }
    return (unsigned char)(count + count);
}
#endif

void CAP_Generate1000FakeEvent(void)
{
    int i = 0;
    for (i = 0; i < 1000; i++)
    {
         LogMeasureEvent(GeneralEvent_Code_Fake, i, 2, 3, 4, 5);
    }
}


static unsigned char st_advertisement_id = 1;

static int st_Process_Received(const unsigned char *data, unsigned short len)
{
    unsigned char out_data[Cap_MaxDataLen];
    uint16 msr;
    unsigned short a2dRawData = 0;
    int i, tmp, n_start, n_stop;
    GeneralEvent e;
    MeasureEvent m;
    //uint16_t measure;
    int16_t cel = 1;
    
    
   
    if (len < Cap_CmdPrefixLen)
        return 0;
    if (len != (Cap_CmdPrefixLen + data[Cap_CmdPayloadLenPos]))
        return 0;
    for (i = 0; i < Cap_CmdPrefixLen; i++)
        out_data[i] = data[i];
    out_data[Cap_CmdPayloadLenPos] = 0;
    switch (data[Cap_CmdPos])
    {
/*        case Cap_Cmd_RunSingleMeasurement:
            st_measurements_reset(data[Cap_CmdPos], 1);
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_RunContinuousMeasurment:
            st_measurements_reset(data[Cap_CmdPos], data[Cap_CmdPrefixLen]);
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
    
        case Cap_Cmd_StopContinuousMeasurement:
            st_measurements_to_do = 0;
            ToF_Sleep();
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);*/
        /*case Cap_Cmd_GetLastOpen:
            st_serialize_uint32(out_data + Cap_CmdPrefixLen, st_fixed_time(st_last_open_time));
            out_data[Cap_CmdPayloadLenPos] = 4;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetLastClose:
            st_serialize_uint32(out_data + Cap_CmdPrefixLen, st_fixed_time(st_last_close_time));
            out_data[Cap_CmdPayloadLenPos] = 4;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetLastTouchStart:
            st_serialize_uint32(out_data + Cap_CmdPrefixLen, st_fixed_time(st_last_touch_start_time));
            out_data[Cap_CmdPayloadLenPos] = 4;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetLastTouchEnd:
            st_serialize_uint32(out_data + Cap_CmdPrefixLen, st_fixed_time(st_last_touch_end_time));
            out_data[Cap_CmdPayloadLenPos] = 4;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetLastReminder:
            st_serialize_uint32(out_data + Cap_CmdPrefixLen, st_fixed_time(st_last_reminder_time));
            out_data[Cap_CmdPayloadLenPos] = 4;
            return st_Cap_Send(out_data);*/
        /*case Cap_Cmd_GetLastMeasurement:
            tmp = ToF_RangingMeasurementDataFilled;
            ToF_GetRangingMeasurement(0);
            if (!tmp)
            LogMeasureEvent(MeasureEvent_Code_Length, ToF_RangingMeasurementData.RangeMilliMeter, st_measurements_std_compact);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen, ToF_RangingMeasurementData.RangeMilliMeter);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen + 2, st_measurements_std);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen + 4, ToF_RangingMeasurementData.SignalRateRtnMegaCps);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen + 6, ToF_RangingMeasurementData.AmbientRateRtnMegaCps);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen + 8, ToF_RangingMeasurementData.RangeStatus);
            out_data[Cap_CmdPayloadLenPos] = 10;
            return st_Cap_Send(out_data);*/
        case Cap_Cmd_GetMeasureLogLength:
            st_serialize_uint16(out_data + Cap_CmdPrefixLen, (unsigned short)MeasureEvents_GetCount());
            out_data[Cap_CmdPayloadLenPos] = 2;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetMeasureLog:
            n_start = st_deserialize_uint16(data + Cap_CmdPrefixLen);
            n_stop = st_deserialize_uint16(data + Cap_CmdPrefixLen + 2);
            tmp = MeasureEvents_GetCount();
            if ((n_stop + 1) > tmp)
                n_stop = tmp - 1;
            if ((n_start <= n_stop) && tmp)
            {
                if (n_stop > (n_start + 1))
                    n_stop = n_start + 1;
                i = 0;
                while (n_start <= n_stop)
                {
                    MeasureEvents_GetAt(n_start, &m);
                    st_serialize_uint32(out_data + Cap_CmdPrefixLen + i, st_fixed_time(m.ts));
                    i += 4;
                    out_data[Cap_CmdPrefixLen + i] = m.code;
                    ++i;
                    st_serialize_uint16(out_data + Cap_CmdPrefixLen + i, m.value);
                    i += 2;
                    ++n_start;
                }
                out_data[Cap_CmdPayloadLenPos] = i;
            }
            return st_Cap_Send(out_data);
        case Cap_Cmd_ClearMeasureLog:
            MeasureEvents_Clear();
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetLogLength:
            st_serialize_uint16(out_data + Cap_CmdPrefixLen, (unsigned short)GeneralEvents_GetCount());
            out_data[Cap_CmdPayloadLenPos] = 2;
            return st_Cap_Send(out_data);
        
        case Cap_Cmd_GetNextLogs:
            
            n_start = 0;
            n_stop = st_deserialize_uint16(data + Cap_CmdPrefixLen + 2); // reading length
                        
            
            // if we got here again then we can delete prevous events log
            if (Log_getPrevLogLengthRequest())
            {
                int lengthToDelete = Log_getPrevLogLengthRequest();//g_prevLogLengthRequest;
                
                while(lengthToDelete)
                {
                    GeneralEvents_GetHead(&e);
                    lengthToDelete--;
                }
            }
            
            tmp = GeneralEvents_GetCount();
            {
                char txt[20];
                sprintf(txt, "log size is %d\n\r", tmp);
                UARTCopy_SendStr(txt);
            }
            
            if ((n_stop) > tmp)
            {
                n_stop = tmp;
                {
                    char txt[20];
                    sprintf(txt, "almost done %d\n\r", n_stop);
                }
            }
            
            
            //g_prevLogLengthRequest = n_stop;
            Log_setPrevLogLengthRequest(n_stop);
            
            // if length is 0 then the mobile app took all the log we have. 
            // so we don;t have to "rememeber anythig in case of disconnection while log is being uploaded.
            if (0 == tmp )
            {
                //g_prevLogLengthRequest = 0;
                Log_setPrevLogLengthRequest(0);    
            }
            
            {
                char txt[20];
                sprintf(txt, "start %d, len %d \n\r", n_start, n_stop);
                UARTCopy_SendStr(txt);
            }
            
            
            i = 0;
            unsigned short value1 = 0;
            while (n_start < n_stop)
            {
                GeneralEvents_GetAt(n_start, &e);
                st_serialize_uint32(out_data + Cap_CmdPrefixLen + i, st_fixed_time(e.ts));
                i += 4;
                out_data[Cap_CmdPrefixLen + i] = e.code;
                ++i;
                st_serialize_uint16(out_data + Cap_CmdPrefixLen + i,value1);  // temporary use.
                i += 2;
                st_serialize_uint16(out_data + Cap_CmdPrefixLen + i, e.value2);
                i += 2;
                st_serialize_uint16(out_data + Cap_CmdPrefixLen + i, e.value3);
                i += 2;
                st_serialize_uint16(out_data + Cap_CmdPrefixLen + i, e.value4);
                i += 2;
                st_serialize_uint16(out_data + Cap_CmdPrefixLen + i, e.value5);
                i += 2;               
                ++n_start;
            }
            out_data[Cap_CmdPayloadLenPos] = i;
            
            return st_Cap_Send(out_data);
        break;    
        case Cap_Cmd_GetLog:           
            n_start = st_deserialize_uint16(data + Cap_CmdPrefixLen);
            n_stop = st_deserialize_uint16(data + Cap_CmdPrefixLen + 2);           
                        
            tmp = GeneralEvents_GetCount();
            if ((n_stop + 1) > tmp)
                n_stop = tmp - 1;
            if ((n_start <= n_stop) && tmp)
            {
                if (n_stop > (n_start + 1)) // for limited the number of logs to 2.
                    n_stop = n_start + 1;
                i = 0;
                while (n_start <= n_stop)
                {
                    GeneralEvents_GetAt(n_start, &e);
                    st_serialize_uint32(out_data + Cap_CmdPrefixLen + i, st_fixed_time(e.ts));
                    i += 4;
                    out_data[Cap_CmdPrefixLen + i] = e.code;
                    ++i;
                    //st_serialize_uint16(out_data + Cap_CmdPrefixLen + i, e.value1);
                    //i += 2;
                    st_serialize_uint16(out_data + Cap_CmdPrefixLen + i, e.value2);
                    i += 2;
                    st_serialize_uint16(out_data + Cap_CmdPrefixLen + i, e.value3);
                    i += 2;
                    st_serialize_uint16(out_data + Cap_CmdPrefixLen + i, e.value4);
                    i += 2;
                    st_serialize_uint16(out_data + Cap_CmdPrefixLen + i, e.value5);
                    i += 2;               
                    ++n_start;                    
                }
                out_data[Cap_CmdPayloadLenPos] = i;
            }
            return st_Cap_Send(out_data);
        case Cap_Cmd_ClearLog:
            GeneralEvents_Clear();
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetRtc:
            st_serialize_uint32(out_data + Cap_CmdPrefixLen, st_fixed_time(st_raw_time()));
            out_data[Cap_CmdPayloadLenPos] = 4;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetRtc:
            st_set_time_fix(st_deserialize_uint32(data + Cap_CmdPrefixLen));
            st_hydration_behavior_set = 0;
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetSleep:
            st_go_sleep = 1;
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        /*case Cap_Cmd_SetMesurementCount:
            st_measurements_set_count(data[Cap_CmdPrefixLen]);
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);*/
        case Cap_Cmd_SetBlinkingTime:
            st_blink_setup(data[Cap_CmdPrefixLen], data[Cap_CmdPrefixLen + 1], data[Cap_CmdPrefixLen + 2]);
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_RunBlinking:
            LEDS_powerUpBlink(GREEN);
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetBatteryVoltage:
            {
                // unsigned short a2dRawData = 0;   Need to check!!!
                msr = ADC_ReadBattery(&a2dRawData);
                //LogMeasureEvent(MeasureEvent_Code_Voltage, msr, 0);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen, msr);
                out_data[Cap_CmdPayloadLenPos] = 2;
                return st_Cap_Send(out_data);
            }
        case Cap_Cmd_GetTemperature:
            //CAP_getTemperature(&measure, &cel);
            //LogMeasureEvent(MeasureEvent_Code_Temperature, cel, 0);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen, cel);
            out_data[Cap_CmdPayloadLenPos] = 2;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetVersion:
            i = strlen((char *)st_version);
            memcpy(out_data + Cap_CmdPrefixLen, st_version, i);
            out_data[Cap_CmdPayloadLenPos] = (unsigned char)i;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetIdleTimeout:
            st_serialize_uint16(out_data + Cap_CmdPrefixLen, (unsigned short)st_idle_timeout);
            out_data[Cap_CmdPayloadLenPos] = 2;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetIdleTimeout:
            st_idle_timeout = st_deserialize_uint16(data + Cap_CmdPrefixLen);
            st_idle_timeout_apply();
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetHourHydrationGoal:
            st_hour_hydration_goal = st_deserialize_uint16(data + Cap_CmdPrefixLen);
            st_hydration_behavior_set = 0;
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetTotalDailyHydrationGoal:
            st_total_daily_hydration_goal = st_deserialize_uint16(data + Cap_CmdPrefixLen);
            st_hydration_behavior_set = 0;
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetDailyUsageTime:
            st_daily_usage_time_from = st_deserialize_uint16(data + Cap_CmdPrefixLen) * HOUR_IN_SECONDS;
            st_daily_usage_time_to = st_deserialize_uint16(data + Cap_CmdPrefixLen + 2) * HOUR_IN_SECONDS;
            st_hydration_behavior_set = 0;
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetHydrationSamplingTime:
            st_hydration_sampling_time = st_deserialize_uint16(data + Cap_CmdPrefixLen) * MINUTE_IN_SECONDS;
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_ImHere:
            st_was_remind = 0;
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        /*case Cap_Cmd_SetMeasureToVolume:
            i = st_deserialize_uint16(data + Cap_CmdPrefixLen);
            tmp = st_deserialize_uint16(data + Cap_CmdPrefixLen + 2);
            if ((i >= 0) && (i < (MEASURE_TO_VOLUME_COUNT * 10)) && (tmp >= 0))
            {
                i /= 10;
                st_measure_to_volume[i] = tmp;
                st_measure_to_volume_filled = 1;
            }
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);*/
        case Cap_Cmd_StartReminder:
            st_reminder_active = 1;
            st_was_remind = 0;
            st_last_hydration_check_time = st_raw_time();
            st_hydration_last_measured_time = 0;
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_StopReminder:
            st_reminder_active = 0;
            st_was_remind = 0;
            st_hydration_last_measured_time = 0;
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetMaxStd:
            st_measurements_std_max = data[Cap_CmdPrefixLen];
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetMinVolumeChange:
            st_hydration_volume_change_min = data[Cap_CmdPrefixLen];
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetHydrationValue:
            st_serialize_uint32(out_data + Cap_CmdPrefixLen, (unsigned long)st_hydration_daily_sum);
            out_data[Cap_CmdPayloadLenPos] = 4;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetHydrationValue:
            st_hydration_daily_sum = st_deserialize_uint32(data + Cap_CmdPrefixLen);
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetAdvertisementID:
            out_data[Cap_CmdPrefixLen] = st_advertisement_id;
            out_data[Cap_CmdPayloadLenPos] = 1;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetAdvertisementID:
            st_advertisement_id = data[Cap_CmdPrefixLen];
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        //
        case Cap_Cmd_MakeSweep:
            //st_sweep_proceed();
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetSweepReply:
            //st_sweep_fill_measurement_block(data[Cap_CmdPrefixLen], out_data + Cap_CmdPrefixLen);
            out_data[Cap_CmdPayloadLenPos] = 16;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetSweepReplyFFT:
            //st_sweep_fill_calc_block(data[Cap_CmdPrefixLen], out_data + Cap_CmdPrefixLen);
            out_data[Cap_CmdPayloadLenPos] = 16;
            return st_Cap_Send(out_data);
        //
        /*case Cap_Cmd_SetGyroDetectParams:
            st_gyro_threshold = st_deserialize_uint16(data + Cap_CmdPrefixLen);
            st_gyro_lp_count_enough = st_deserialize_uint16(data + Cap_CmdPrefixLen + 2);
            st_gyrostop_time_to_wait_after_acc_moved = st_deserialize_uint16(data + Cap_CmdPrefixLen + 4);
            DoF_ChangeThreshold(st_deserialize_uint16(data + Cap_CmdPrefixLen + 6));
            if (data[Cap_CmdPayloadLenPos] >= 10)
                st_gyro_threshold_for_open = st_deserialize_uint16(data + Cap_CmdPrefixLen + 8);
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetGyroDetectParams:
            st_serialize_uint16(out_data + Cap_CmdPrefixLen, (unsigned short)st_gyro_threshold);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen + 2, (unsigned short)st_gyro_lp_count_enough);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen + 4, (unsigned short)st_gyrostop_time_to_wait_after_acc_moved);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen + 6, (unsigned short)DoF_WakeUpThreshold);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen + 8, (unsigned short)st_gyro_threshold_for_open);
            out_data[Cap_CmdPayloadLenPos] = 10;
            return st_Cap_Send(out_data);
        case Cap_Cmd_GetTempGyroAccelDump:
            st_gyro_state = 0;
            DoF_DumpGyroAndAccel(out_data + Cap_CmdPrefixLen);
            out_data[Cap_CmdPayloadLenPos] = 14;
            return st_Cap_Send(out_data);*/
        case Cap_Cmd_GetGyroFIFOState:
            st_serialize_uint16(out_data + Cap_CmdPrefixLen, (unsigned short)st_gyrostop_fifo_copy_count);
            st_serialize_uint16(out_data + Cap_CmdPrefixLen + 2, st_gyrostop_turn_detected);
            out_data[Cap_CmdPayloadLenPos] = 4;
            return st_Cap_Send(out_data);
        /*case Cap_Cmd_GetGyroFIFOChunk:
            out_data[Cap_CmdPayloadLenPos] = st_gyrostop_fifo_copy_get_chunk(out_data + Cap_CmdPrefixLen, st_deserialize_uint16(data + Cap_CmdPrefixLen));
            return st_Cap_Send(out_data);*/
            
        case Cap_Cmd_SetAcclSampleRate:
            //sampleRate = data[Cap_CmdPrefixLen];
            //H3LIS331DL_setSampleRate(sampleRate);  
            //UserFlash_writeShockSampleRate(&sampleRate);
            
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        case Cap_Cmd_SetAcclFullScaleAndThreshold:
            //fullScale = data[Cap_CmdPrefixLen];
            //threshold = data[Cap_CmdPrefixLen+1];            
            //H3LIS331DL_setFullScaleAndThreshold(threshold, fullScale, 1); 
            //UserFlash_writeShockThreshold(&threshold);
            //UserFlash_writeShockFullScale(&fullScale);
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);            
        //
        case Cap_Cmd_SetSamplingPeriod:
            {
                uint16_t samplingPeriod = 0;
                
                samplingPeriod = st_deserialize_uint16(data + Cap_CmdPrefixLen);
                
                if (CAP_SamplingPeriodIsValid(samplingPeriod))
                {
                    UserFlash_writeSystemSamplingPeriod(samplingPeriod);
                    setSamplingPeriod(samplingPeriod);
                    CAP_calculateAdvPerodInWarehouseMode();
                    startSamplingPeriod();
                }
                out_data[Cap_CmdPos] = Cap_Cmd_Ack;
                return st_Cap_Send(out_data);
            }
        case Cap_Cmd_GetSamplingPeriod:
                st_serialize_uint16(out_data + Cap_CmdPrefixLen, getSamplingPeriod());
                out_data[Cap_CmdPayloadLenPos] = 2;
                return st_Cap_Send(out_data);
        
        case Cap_Cmd_GetExecutiveSummary:
            {
                TStatistics statistics;
                uint16_t mtuSize = 0; 
                
                //UARTDebugPort_SendStr("E\n\r");
                
                CyBle_GattGetMtuSize(&mtuSize);
                
                if (mtuSize < sizeof(TStatistics) + PACKET_OVERHEAD)
                {
                    //char txt[35];                    
                    //sprintf(txt, "mtu packet size- %d\n\r", mtuSize);
                    //UARTDebugPort_SendStr(txt);
                    out_data[Cap_CmdPos] = Cap_Cmd_Ack;
                    return st_Cap_Send(out_data);
                }
                
                //UARTDebugPort_SendStr("EE\n\r");
                
                LOG_calculateStatistics(&statistics);
                
                //UARTDebugPort_SendStr("EEE\n\r");
                
                st_serialize_uint16(out_data + Cap_CmdPrefixLen, statistics.numOfLines);
                st_serialize_uint32(out_data + Cap_CmdPrefixLen+2, statistics.startDate);
                st_serialize_uint32(out_data + Cap_CmdPrefixLen+6, statistics.endDate);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+10, statistics.numOfShocks);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+12, statistics.maxG);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+14, statistics.minG);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+16, statistics.maxPressure);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+18, statistics.minPressure);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+20, statistics.maxHummidity);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+22, statistics.minHummidity);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+24, statistics.maxTemperature);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+26, statistics.minTemperature);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+28, statistics.maxVoltage);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+30, statistics.minVoltage);
                st_serialize_uint16(out_data + Cap_CmdPrefixLen+32, statistics.numOfMeasures);
                
                out_data[Cap_CmdPayloadLenPos] = sizeof(TStatistics);
                
                //UARTDebugPort_SendStr("EEEE\n\r");
                
                return st_Cap_Send(out_data);
            }
        case Cap_Cmd_Generate1000FakeEvents:
            CAP_Generate1000FakeEvent();
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
            break;
        
        case Cap_Cmd_SetAdvPeriodInarehouseMode:
        {
            uint16_t advPeriodInWarehouseMode = 0;
            
            advPeriodInWarehouseMode = st_deserialize_uint16(data + Cap_CmdPrefixLen);
            
            if (CAP_AdvPeriodInWarehouseModeIsValid(advPeriodInWarehouseMode))
            {
                UserFlash_writeAdvPeriodInWareHouseMode(advPeriodInWarehouseMode);
                CAP_calculateAdvPerodInWarehouseMode();
            }
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
        }    
        case Cap_Cmd_SetDutyCyclePeriodForCheckBuzzer:
            {
            uint16_t DutyCycle   = 0;  // %  (Volume of the Buzzer: (DytuCycle in time: max 1500 )     
            DutyCycle = st_deserialize_uint16(data + Cap_CmdPrefixLen );
            CAP_startBuzzer(BUZZER_IND_TIMEOUT, DutyCycle);    
//            msr = ADC_ReadBattery(&a2dRawData); 
//            DC_LEVEL High = DutyCycle_HighVoltage;
//            DC_LEVEL Low  = DutyCycle_LowVoltage;
//            DC_LEVEL V_OFFSET = DutyCycle_Voltage_OFFSET;
            
            //CAP_startBuzzer(BUZZER_IND_TIMEOUT,  (msr > V_OFFSET)?High:Low); 
            out_data[Cap_CmdPos] = Cap_Cmd_Ack;
            return st_Cap_Send(out_data);
            
            }
        default:
            return 0;
    }
    return 0;
}

static int st_Process_State(void)
{
    st_calc_ticks();
    st_blink();
    return 0;
}

int Cap_Process(void)
{
    int i, ret = 0;
    unsigned char data[Cap_MaxDataLen];
    unsigned short len = 0;
    
    if (1 == g_rxIntOccured)
    {
        g_rxIntOccured = 0;
        handleRxInt();
    } 
        
    CAP_checkDebugPortTimeout();
    
    if (!Cap_IsStarted())
    {
        //st_data_filled = 0;
        //return 0;
    }
    if (st_data_filled)
    {
        len = st_data_len;
        for (i = 0; i < len; i++)
            data[i] = st_data[i];
        st_data_filled = 0;
        return st_Process_Received(data, len);
    }
    else
    {
        ret = st_Process_State();
        if (ret)
            return ret;
        if (st_idle_timeout_is_reached())
            return -1;
        if (st_go_sleep)
            return -1;
    }
    return 0;
}



//#define SEC_MEASURE_TEST
#ifdef SEC_MEASURE_TEST
#define SEC_MEASURE_TEST_SAMPLES_COUNT 30
#define SEC_MEASURE_TEST_CALC_COUNT 5
static float st_sec_measure_test_samples[SEC_MEASURE_TEST_SAMPLES_COUNT];
static int st_sec_measure_test_sample = 0;
static int st_sec_measure_test_initiated = 0;
static unsigned long st_sec_measure_test_last_timer_value = 0;
static unsigned short st_sec_measure_test_best_measure = 0;
static unsigned char st_sec_measure_test_best_std = 255;
#endif



void closeDebugPort(void)
{
    uint32_t TempVal;
    g_debugPortTimeout = 0;//st_raw_time()+1; // timeout of 30 seconds
    
    /*disconnect P0[5] and P0[4] from SCB1 I2C  and make it a GPIO */
    TempVal = CY_GET_REG32(CYREG_HSIOM_PORT_SEL3);
    TempVal &= ~0x00FF0000;  // 
    CY_SET_REG32(CYREG_HSIOM_PORT_SEL3, TempVal);
    
    ///Set pull down to UART port
    CY_SYS_PINS_SET_DRIVE_MODE(CYREG_PRT3_PC,4,CY_SYS_PINS_DM_OD_LO);
    CY_SYS_PINS_SET_DRIVE_MODE(CYREG_PRT3_PC,5,CY_SYS_PINS_DM_OD_LO);   
    
}


void clearDebugPortTimeout(void)
{
    g_debugPortTimeout = st_raw_time()+30; // timeout of 30 seconds
}

void enableDebupPort(void)
{
    clearDebugPortTimeout();
}

unsigned char isDebugPortDisabled()
{
    return (g_debugPortTimeout > 0 ? 0 : 1);
}


int8_t CAP_checkDebugPortTimeout(void)
{
    unsigned long check_time;
    check_time = st_raw_time();
    
    if (!g_debugPortTimeout)
        return 0;
    
    if (check_time < g_debugPortTimeout)
    {
        return 0;
    }
    
    g_debugPortTimeout = 0;
    //UARTCopy_SendStr("Debug Port is disabled\n\r");
    //CyDelay(15);
    //UART_Sleep();
    //enableLogMessages();
    
    return 1;
}

static uint8_t escCount = 0;
#define ESC (0x1b)
#define CR (0x0d)
#define LF (0x0a)
#define UART_RX_SIZE (32)
static uint8_t uartRxArray[UART_RX_SIZE];
static uint8_t uartRxArrayWriteIndex = 0;
int Cap_RxEvent(unsigned char byte)
{
    switch (byte)
    {
        case ESC:    
            if (++escCount >= 3)
            {
                // do not print log messages
                disableLogMessages();
                // start 30 seconds timeout.
                clearDebugPortTimeout();
            }
            byte = 0;
            
        break;
        case CR:
        break;
        case LF:
            uartRxArrayWriteIndex = 0;
            g_rxIntOccured = 1;
        break;   
        default:            
            uartRxArray[(uartRxArrayWriteIndex++) % UART_RX_SIZE] = byte;
            escCount = 0;
        break;
    }
    return 0;
}

#define NUM_OF_COMMANDS (21)
const char commandsListArray[NUM_OF_COMMANDS][48] = 
{
    {" 0 - print commans list \n\r"},
    {" A - Set pressure sensor offset\n\r"},
    {" B - Get pressure sensor offset\n\r"},
    {" C - Get BLE SSID \n\r"},
    {" D - Set sampling period (10 min - 24 hours)\n\r"},
    {" E - Get sampling period \n\r"},
    {" F - Measure\n\r"},
    {" G - Set HW Version (limited to 8 Chars\n\r"},
    {" H - Get HW Version\n\r"},
    {" I - test WDT1\n\r"},
    {" J - Set warehouse period\n\r"},
    {" K - Get warehouse period\n\r"},
    {" L - test Sleep\n\r"},
    {" M - Tx Carrier [Freq 1-80]\n\r"},
    {" N - Stop Tx \n\r"},
    {" O - Start Buzzer \n\r"}, 
    {" P - Stop Buzzer \n\r"},
    {" Q - Set Flash To Defaults\n\r"}, 
    {" R - Fake 1000 event logs\n\r"}, 
    {" S - Set Batch Number\n\r"}, 
    {" T - Get Batch Number\n\r"}    
};

void printCommandsList(void)
{
    int i = 0;
    for (i = 0 ; i < NUM_OF_COMMANDS ; i++)
    {
        UARTDebugPort_SendStr(&commandsListArray[i][0]);
    }
}

void CAP_printSamplingPeriod(void)
{
    #define PRINT_TXT_SIZE (32)
    char txt[PRINT_TXT_SIZE];
    uint16_t samplingPeriod = 0;
    UserFlash_readSystemSamplingPeriod(&samplingPeriod);
    sprintf(txt, "Sampling Period is - %d \n\r", samplingPeriod);
    UARTDebugPort_SendStr(txt);
}


void CAP_printHwVerssion(void)
{
    #define PRINT_TXT_SIZE (32)
    char txt[PRINT_TXT_SIZE];
    
    char hwVerssion[10];
                
    UserFlash_readHWVersion(hwVerssion);
    
    sprintf(txt, "HW Version - %s \n\r", hwVerssion);
    UARTDebugPort_SendStr(txt);
}

void CAP_printBatchNumber(void)
{
    #define PRINT_TXT_SIZE (32)
    char txt[PRINT_TXT_SIZE];
    
    char batchNumber[6];
    
    UserFlash_readBatchNumber(batchNumber);
    
    sprintf(txt, "Batch Number is - %s \n\r", batchNumber);
    UARTDebugPort_SendStr(txt);
}
void  CAP_printBDAddress(void)
{
    CYBLE_GAP_BD_ADDR_T bdAddr;
    #define PRINT_TXT_SIZE (32)
    char txt[PRINT_TXT_SIZE];
    bdAddr.type = 0;
    CyBle_GetDeviceAddress(&bdAddr);
    
    UARTDebugPort_SendStr("BD device Address\n\r");
    
    sprintf(txt, "%02X:%02X:%02X:%02X:%02X:%02X\n\r",
                  bdAddr.bdAddr[5], bdAddr.bdAddr[4], bdAddr.bdAddr[3],
                  bdAddr.bdAddr[2], bdAddr.bdAddr[1], bdAddr.bdAddr[0]  );
    UARTDebugPort_SendStr(txt);    
}

void CAP_printNumOfWarehouseAdv(void)
{
    #define PRINT_TXT_SIZE (32)
    char txt[PRINT_TXT_SIZE];
    uint16_t warehousePeriod = 0;
    UserFlash_readAdvPeriodInWareHouseMode(&warehousePeriod);
    sprintf(txt, "warehouse Period is - %d \n\r", warehousePeriod);
    UARTDebugPort_SendStr(txt);
}

int8_t CAP_TxCarrier(uint8_t freq)
{
    uint32 cfg2,cfgctrl,sy;
    uint32 freqReg = 0xE960;
    
    if (freq > 80)
    {
        return -1;
    }

    //// set CW mode, To prevent first time TX  frequency offset before CW mode enabled in case.
    CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_MODEM), 0x96EC);       


    // Configure DSM as first order PLL
    CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_CFGCTRL),0x0008);


    // Enable Transmit at the required TX frequency
    CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_CFG1), 0xBB48);    
    
    
    //CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_DBUS), 0xE962);   // 0x962=2402 is channel frequency in MHz
    //CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_DBUS), 0xE992);   // 0x992=2450 is channel frequency in MHz
    CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_DBUS), freqReg+freq);   // 0x9B0=2480 is channel frequency in MHz


    // Synchronisation delay for PA to ramp
    CyDelayUs(120);

    // Disable modulation port
    cfg2= CY_GET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_CFG2));
    cfg2 |=0x1000;   
    cfg2 &=~0x03FF;                    
    cfg2 |=0x0200;   

    // Enable test mode
    cfgctrl= CY_GET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_CFGCTRL));
    cfgctrl |=0x8000;

    // Close the loop
    sy= CY_GET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_SY));
    sy |=0x4000;     
    CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_CFG2), cfg2);  
    CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_CFGCTRL), cfgctrl);
    CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_SY),sy); 
    
    return 0;
}

void TxStop(void)
{
    CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLERD_DBUS), 0xC992);
    CY_SET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLELL_COMMAND_REGISTER),0x48); //setting it to idle
    CY_GET_XTND_REG32((void CYFAR *)(CYREG_BLE_BLELL_DTM_RX_PKT_COUNT));
}

uint8_t CAP_SamplingPeriodIsValid(uint16_t a_samplingPeriod)
{
    #define DAY_IN_MINUTES (24*60)
    
    if ((a_samplingPeriod > DAY_IN_MINUTES) ||
        (a_samplingPeriod % 10 != 0) ||
        (a_samplingPeriod == 0))
    {
        return 0;
    }
    return 1;
}

#define DAY_IN_MINUTES (60)
#define MAX_WAREHOUSE_ADV (2)

uint8_t CAP_AdvPeriodInWarehouseModeIsValid(uint8_t a_AdvPeriodInWarehouseMode)
{
    if (a_AdvPeriodInWarehouseMode > MAX_WAREHOUSE_ADV)
    {
        return 0;
    }
    return 1;
}

void CAP_calculateAdvPerodInWarehouseMode(void)
{
    char txt[32];
    
    uint16_t samplingPeriod = 0;
    UserFlash_readSystemSamplingPeriod(&samplingPeriod);
    uint16_t warehouseAdv = 0;
    UserFlash_readAdvPeriodInWareHouseMode(&warehouseAdv); // its only a number between 0 to 2
    
    uint16_t warehosePeriod = 0;
    
    if (0 != warehouseAdv)
    {
        uint16_t maxNumOfwarehouseAdv = DAY_IN_MINUTES / samplingPeriod;
        
        if (maxNumOfwarehouseAdv > MAX_WAREHOUSE_ADV)
        {
            maxNumOfwarehouseAdv = MAX_WAREHOUSE_ADV;
        }        
        
        if (warehouseAdv > maxNumOfwarehouseAdv)
        {
            warehouseAdv = maxNumOfwarehouseAdv;
        }
        
        warehosePeriod = (DAY_IN_MINUTES / (warehouseAdv))- 1;        
    }
    
    CAP_SetWarehousePeriod((warehosePeriod-1)*MINUTE_IN_SECONDS);
    sprintf(txt, "Wh P(M): %d\n\r", (int)warehosePeriod+1);
    UART_UartPutString(txt);
    sprintf(txt, "Wh e     %d\n\r", (int)(warehosePeriod+1) / samplingPeriod);
    UART_UartPutString(txt);
    
}


int handleRxInt(void)
{
    #define PRINT_TXT_SIZE (32)
    char txt[PRINT_TXT_SIZE];
    char str[10];
    uint16_t msr=0;
    unsigned short a2dRawData=0;
    clearDebugPortTimeout(); 
        
    switch (uartRxArray[0])
    {
        case '0':
            printCommandsList();
        break;
        case 'a':
        case 'A':
            {
                long wrOffset = 0;
                long rdOffset = 0;
                const char s = '#';
                char *token = 0; 
                
                token = strtok ((char*)&uartRxArray[2], &s);
                
                if (token == 0)
                {
                    UARTDebugPort_SendStr("Invalid Parameter\n\r");
                    return 0;
                } 
                
                wrOffset = atoi (token); 
                
                UserFlash_writePressureOffset(wrOffset);
                
                UserFlash_readPressureOffset(&rdOffset);
                
                if (wrOffset == rdOffset)
                {
                    setPressureOffset(rdOffset);
                    UARTDebugPort_SendStr("Pressure_Set\n\r");
                }
                else
                {
                    UARTDebugPort_SendStr("FAILED\n\r");                                        
                }
                itoa (rdOffset,str, 10);
                sprintf(txt, "the offset is - %s \n\r", str);
                UARTDebugPort_SendStr(txt);
            }
        break;
        case 'b':
        case 'B':
            {
                long offset = 0;
                UserFlash_readPressureOffset(&offset);
                itoa (offset,str, 10);
                sprintf(txt, "the offset is - %s \n\r", str);
                UARTDebugPort_SendStr(txt);
            }
        break;
        case 'c':
        case 'C':
            {
                CAP_printBDAddress();                
            }
        break; 
        case 'D':
        case 'd':
            {
                uint16_t wrSamplingPeriod = 0;
                uint16_t rdSamplingPeriod = 0;
                
                const char s = '#';
                char *token = 0; 
                
                token = strtok ((char*)&uartRxArray[2], &s);
                
                if (token == 0)
                {
                    UARTDebugPort_SendStr("Invalid Parameter\n\r");
                    return 0;
                } 
                
                wrSamplingPeriod = atoi (token); 
                
                if (!CAP_SamplingPeriodIsValid(wrSamplingPeriod))
                {
                    UARTDebugPort_SendStr("Invalid Parameter\n\r");
                    return 0;
                }               
                                
                UserFlash_writeSystemSamplingPeriod(wrSamplingPeriod);
                
                UserFlash_readSystemSamplingPeriod(&rdSamplingPeriod);                             
                
                if (wrSamplingPeriod == rdSamplingPeriod)
                {
                    setSamplingPeriod(rdSamplingPeriod);
                    UARTDebugPort_SendStr("OK\n\r");
                }
                else
                {
                    UARTDebugPort_SendStr("FAILED\n\r");                                        
                }
                CAP_printSamplingPeriod();
                
                CAP_calculateAdvPerodInWarehouseMode();
            }
        break;
        case 'E':
        case 'e':
            CAP_printSamplingPeriod();
        break;
        case 'F':
        case 'f':
            handleTestBoxInt(DEBUG_PORT_EVENT, 0);
        break;
        case 'G':
        case 'g':
            {
                const char s = '#';
                char *token = 0; 
                                
                token = strtok ((char*)&uartRxArray[2], &s);
                
                if ((token == 0) || (strlen(token) > HW_VERSION_MAX_LENGTH))
                {
                    UARTDebugPort_SendStr("Invalid Parameter\n\r");
                } 
                else
                {
                    UserFlash_writeHWVersion(token);
                
                    UARTDebugPort_SendStr("Hw_Set\n\r");  
                }
            }
        break;
        case 'H':
        case 'h':
            {
                CAP_printHwVerssion();                
            }
        break;
        case 'I':
        case 'i':  
            while(1);
        break;
        case 'J':
        case 'j':
            // set warehouse period
            {
                uint16_t wrWarehousePeriod = 0;
                uint16_t rdWarehousePeriod = 0;
                
                const char s = '#';
                char *token = 0; 
                
                token = strtok ((char*)&uartRxArray[2], &s);
                
                if (token == 0)
                {
                    UARTDebugPort_SendStr("Invalid Parameter\n\r");
                    return 0;
                } 
                
                wrWarehousePeriod = atoi (token);  
                
                if (!CAP_AdvPeriodInWarehouseModeIsValid(wrWarehousePeriod))
                {
                    sprintf(txt, "Invalid,must be less:%d\n\r", MAX_WAREHOUSE_ADV+1);
                    UARTDebugPort_SendStr(txt);
                    return 0;
                }
                
                UserFlash_writeAdvPeriodInWareHouseMode(wrWarehousePeriod);
                
                UserFlash_readAdvPeriodInWareHouseMode(&rdWarehousePeriod);                             
                
                if (wrWarehousePeriod == rdWarehousePeriod)
                {
                    UARTDebugPort_SendStr("OK\n\r");
                }
                else
                {
                    UARTDebugPort_SendStr("FAILED\n\r");                                        
                }
                CAP_printNumOfWarehouseAdv();
                
                CAP_calculateAdvPerodInWarehouseMode();
            }
        break;    
        case 'K':
        case 'k':
            // get warehouse period
            CAP_printNumOfWarehouseAdv();
        break;
        case 'L':
        case 'l':
            WDT_Stop();
            CAP_Sleep();
            break;
        case 'M':
        case 'm':
            {
                const char s = '#';
                char *token = 0; 
                #define FREQ_MAX_LENGTH (2)
                
                token = strtok ((char*)&uartRxArray[2], &s);
                
                if ((token == 0) || (strlen(token) > FREQ_MAX_LENGTH))
                {
                    UARTDebugPort_SendStr("Invalid Parameter\n\r");
                    return 0;
                } 
                
                uint8_t freq = atoi(token); 
                
                if (0 == CAP_TxCarrier(freq))
                {
                    UARTDebugPort_SendStr("TX...\n\r");
                    
                }
                else
                {
                    UARTDebugPort_SendStr("TX Failed\n\r");
                }
                
                UARTDebugPort_SendStr("OK\n\r");
            }
            break;
            case 'N':
            case 'n':
                TxStop();
            break;
            case 'O':
            case 'o':
                msr = ADC_ReadBattery(&a2dRawData); 
                DC_LEVEL High = DutyCycle_HighVoltage;
                DC_LEVEL Low  = DutyCycle_LowVoltage;
                DC_LEVEL V_OFFSET = DutyCycle_Voltage_OFFSET;
                CAP_startBuzzer(5000,  (msr > V_OFFSET)?High:Low);  
                
            break;
            case 'P':
            case 'p':
                CAP_stopBuzzer();
            break;
            case 'Q':
            case 'q':
                UserFlash_ReturnToDefault();
            break;                
            case 'R':
            case 'r':
                CAP_Generate1000FakeEvent();
            break;    
            case 'S':
            case 's':
            {
                const char s = '#';
                char *token = 0; 
                                
                token = strtok ((char*)&uartRxArray[2], &s);
                
                if ((token == 0) || (strlen(token) > BATCH_NUMBER_MAX_LENGTH))
                {
                    UARTDebugPort_SendStr("Invalid Parameter\n\r");
                } 
                else
                {
                    UserFlash_writeBatchNumber(token);
                
                    UARTDebugPort_SendStr("Batch_Set\n\r");  
                }
            }
            break;
            case 'T':
            case 't':
            {
                CAP_printBatchNumber();
            }            
            break;
        default:
            sprintf(txt, "Not a valid Command\n\r");
            UARTDebugPort_SendStr(txt);
    }
    
    return 0;
}

int Cap_TestBoxEvent(void)
{
    g_testBoxIntOccured = 1;
    return 1;
}

int Cap_MagneticSwitchEvent(void)
{
    g_magneticSwitchIntOccured = 1;
    return 1;
}

/*int Cap_Shock1Event(void)
{
    g_shockInt1Occured = 1;
    return 1;
}*/

void CAP_getTemperature(uint16_t* measure,int16_t* cel)
{
    *measure = 0;
    *cel = g_lastTemperature = aht10_read_temperature();
}

static int16_t CAP_getlastTemeratureMeasure()
{
    return g_lastTemperature;
}


void CAP_getHummidity(uint16_t* measure, int16_t* hum)
{
    *measure = 0;
    g_lastHummidity = *hum = aht10_read_humidity();
}

static int16_t CAP_getlastHummidityMeasure(void)
{
    return g_lastHummidity;
}


void CAP_getTemperatureAndHummidity(int16_t* a_temperature, int16_t* a_hummidity)
{
    
    float cel = 0;
    float hummidity = 0;
    *a_temperature = 0;
    *a_hummidity = 0;
    if (0 == read_temperature_and_hummidity(&cel, &hummidity))
    {      
        *a_hummidity = g_lastHummidity = hummidity;
        *a_temperature = g_lastTemperature = cel; 

    }  
    else
    {
        #define PRINT_TXT_SIZE1 (38)
        char txt[PRINT_TXT_SIZE1];
        sprintf(txt, "Failed to measure AHT10 device\n\r");
        UARTDebugPort_SendStr(txt);
    }
}
void setPressureOffset(long a_pressureOffset)
{
    g_pressureOffset = a_pressureOffset;
}


uint8_t  CAP_getPressure(int32_t* pressure, short* fmBar)
{
    uint8_t status = 0;
    
    int i = 0;
    int faults = 0;
    int32_t AccPressure = 0;
    int32_t result;
    
    // measure pressure
	#define NUM_OF_MEASURES (32)
    for(i = 0 ; i < NUM_OF_MEASURES ; i++)
    {
        status = MICRO_PRESSURE_ReadPressure(&result);
               
        if (status != 0)
        {
            faults++;
        }
        else
        {
            AccPressure += result;
        }        
    }
    
    if (faults == 0)
    {
        AccPressure = AccPressure  / NUM_OF_MEASURES; //>>=5;
    }
    else
    {
       AccPressure = AccPressure / (i-faults); 
    }
    
    *pressure = AccPressure;
    *fmBar = MICRO_PRESSURE_convertToBar10000(*pressure)+ PRESSURE_FACTOR;
    //UserFlash_readPressureOffset(&offset);
    *fmBar += g_pressureOffset;
    g_lastPressure = *fmBar;
    return status;
}   

static short CAP_getlastPressureMeasure()
{
    return g_lastPressure;
}
// it is not just test button handle but its being called in the following events:
// TEST_BUTTON_EVENT,
// DEBUG_PORT_EVENT,
// SCHEDULER_EVENT,
// MAGNET_EVENT,
// SHOCK_EVENT in this case - shock measure is receved as input otherwise its 0,
int handleTestBoxInt(TEventDriven a_eventDriven, uint16_t a_shockG)
{
    int res = 1;
    #define TEMP_BUFFER_SIZE (10)
    #define TXT_SIZE (50)
    uint16_t measure;
    uint16_t msr;
    char tempBuff [TEMP_BUFFER_SIZE];
    char txt[TXT_SIZE];
    char pressureStr[10];
    char mBarStr[10];
    char batteryMeasure[10];
    uint8_t logEventCode = 0;
    int16_t cel = 1;
    int16_t hum = 0;
    int32_t pressure = 0;
    short fmBar = 0;
    unsigned short a2dRawData = 0;
    uint8_t printOn = 0;
    uint8_t logOn = 1;
    
    extern void handleIntWDTClear(void);
    
    //Todo: Enable VCC and pullup resistors
    peripheralsPowerOn();  

    #if (!AHT10_TEST_ONLY) 
    
    MICRO_PRESSURE_Setup();
    #endif               
    switch(a_eventDriven)
    {
        case TEST_BUTTON_EVENT:
        case DEBUG_PORT_EVENT:
            printOn = 1;
            logOn = 0;
        break;
        case SCHEDULER_EVENT:
        {
            logEventCode = GeneralEvent_Code_Scheduler;
            
            // do not advertise
            res = 0;
            
            // 0 == g_warehosePeriod - means warehouse is disabled.
            if ((g_warehosePeriod > 0) &&
                (g_lastWarehouseTime + g_warehosePeriod < st_raw_time()))
            {
                UART_UartPutString("warehouse event\n\r");
                // do slow advertising
                res = 2;
                // calculate next adv time.
                g_lastWarehouseTime = st_raw_time();
                logEventCode = GeneralEvent_Code_Warehouse;
            }
            else
            {
                UART_UartPutString("scheduler event\n\r");
            }
        }
        break;
        case MAGNET_DETACH_EVENT:
            logEventCode = GeneralEvent_Code_Open;
        break;
        case MAGNET_ATTACH_EVENT:
            logEventCode = GeneralEvent_Code_Close;
        break;
        case SHOCK_EVENT:
            logEventCode = GeneralEvent_Code_Shock;
        break;
    }
    
    if (printOn)
    {
        UARTCopy_SendStr("Measures:\n\r");
    }
    handleIntWDTClear(); 
    #if (!AHT10_TEST_ONLY) 
           
    I2CMaster_SwitchIOs(MPRI2cBus);
    CAP_getPressure(&pressure, &fmBar);
    if (printOn)
    {
        itoa (fmBar,mBarStr, 10);
        itoa (pressure,pressureStr, 10);
        sprintf(txt, "pressure raw data = %s, mBar =%s\n\r", pressureStr, mBarStr);
        UARTCopy_SendStr(txt);
    }    
    handleIntWDTClear(); 
    
    msr = ADC_ReadBattery(&a2dRawData); 
    if (printOn)
    {
        itoa(msr, batteryMeasure, 10);
        sprintf(txt, "batteryMeasure - %smV\n\r", batteryMeasure);
        UARTCopy_SendStr(txt);
        
        itoa(a2dRawData, batteryMeasure, 10);
        sprintf(txt, "batteryMeasure raw data - %smV\n\r", batteryMeasure);
        UARTCopy_SendStr(txt);
    } 
    #else
    
    I2CMaster_SwitchIOs(AHTI2cBus);
    CyDelay(50);
    
    #endif 
    V_Pull_AHT_Write(1);
    I2CMaster_SwitchIOs(AHTI2cBus);
    AHT10_Setup();
    debugPin2_Write(1);    
    //CAP_getTemperature(&measure, &cel);
    //CyDelay(200);
    //CAP_getHummidity(&measure, &hum);
    CAP_getTemperatureAndHummidity(&cel, &hum);
    debugPin2_Write(0);
    handleIntWDTClear(); 

    if (printOn)
    {
        itoa (cel,tempBuff, 10);
        sprintf(txt, "Temperature - %sC [*10]\n\r", tempBuff);
        UARTCopy_SendStr(txt);    
        
        itoa (hum,tempBuff, 10);
        sprintf(txt, "Hummidity - %s%% [*10]\n\r", tempBuff);
        UARTCopy_SendStr(txt);
    }
    
    if (logOn)
    {
        LogMeasureEvent(logEventCode, a_shockG, cel, hum, fmBar, msr);   
    }
    
    //Todo: Disable VCC and pullup resistors    
    //MICRO_PRESSURE_deinit();
    AHT10_deinit();   
    //CyDelay(1000);
    peripheralsPowerOff();
    //CyDelay(1000);
    
    return res;
}

typedef enum
{
    pwmIdleState,
    pwmActiveState  
}TPwmState;

static TPwmState pwmState = pwmIdleState;
void CAP_stopBuzzer(void)
{
    PWM_Stop();
    pwmState = pwmIdleState;
}
#define BUZZER_ON_CYCLE  (100)
#define BUZZER_OFF_CYCLE (400)

void CAP_buzzer(uint16_t time)
{
    long ind = 0; 
    int onOff = 0;
    long length = time * 1000;
    
    while (ind < length)
    {
        if (onOff)
        {
            onOff = 0;
            buzzer_Write(1);
            CyDelayUs(BUZZER_ON_CYCLE);
            ind += BUZZER_ON_CYCLE;
        }
        else
        {
            onOff = 1;
            buzzer_Write(0);
            CyDelayUs(BUZZER_OFF_CYCLE);
            ind += BUZZER_OFF_CYCLE;
        }
    }
    buzzer_Write(0);
}

void CAP_startBuzzer(uint16_t time, uint16_t DC)
{
    if (pwmActiveState == pwmState)
    {
        return;
    }
    UARTCopy_SendStr("starting Buzzer\n\r");
    pwmState = pwmActiveState;
    
    PWM_Start();
    //PWM_WriteCompare(1500); //50% duty cycle 
    
    
    PWM_WriteCompare(DC);    // 
    
    PWM_WritePeriod(3000u);   // 1/12MHz * 1/4KHz
    
    if (time > 300)
    {
        // in order to avoid WDT reset
        UARTCopy_SendStr("Buzzer must be stopped manually\n\r");
        return;
    }
    CyDelay(time);
    CAP_stopBuzzer();  
}


static int g_magneticSwitch = 0;
static unsigned long g_magneticSwitchLastTime = 0;

//#define MAGNET_IND_TIMEOUT (100)

// this function is called after reset, i am checking the magnet status, in order to decide 
void CAP_InitMagnetStatus(void)
{
    g_magneticSwitch = MagneticSwitch_Read(); 
    
    if (1 == g_magneticSwitch)
    {
        UARTCopy_SendStr("Start sampling...\n\r");
        startSamplingPeriod();        
    }
    else
    {
        UART_Sleep();
        CyDelay(10);
        I2C_Sleep();
    }
}

int8_t handleMagneticSwitchInt(void)
{
     //uint8_t res = 0; 
    uint16_t msr=0;
    uint16_t a2dRawData=0;
    // to avoid debounce
    if ((MagneticSwitch_Read() != g_magneticSwitch))// && (g_magneticSwitchLastTime != st_raw_time()))
    {
        g_magneticSwitchLastTime = st_raw_time();
        if (0 == g_magneticSwitch)
        {
            if (g_sleepHappened)
            {
                LEDS_Wakeup();
                IOWakeup();
            }
            I2C_Wakeup();
            CyDelay(10);                                        //10mSec
            UART_Wakeup();            
			LogGeneralEvent(GeneralEvent_Code_Open, 0);    
            startSamplingPeriod();
            UARTCopy_SendStr("On\n\r");                         //2mSec
			g_magneticSwitch = 1;            
            return 0;
        }
        else
        {
            g_magneticSwitch = 0;
            g_sleepHappened  = 0;
            //CAP_buzzer(MAGNET_IND_TIMEOUT);
            msr = ADC_ReadBattery(&a2dRawData); 
            DC_LEVEL High = DutyCycle_HighVoltage;
            DC_LEVEL Low  = DutyCycle_LowVoltage;
            DC_LEVEL V_OFFSET = DutyCycle_Voltage_OFFSET;
            CAP_startBuzzer(BUZZER_IND_TIMEOUT, (msr > V_OFFSET)?High:Low); 
            handleTestBoxInt(MAGNET_ATTACH_EVENT, 0);                 //450 mSEc
            g_magnetClosed_TimeToSleep = st_raw_time()+(20);
            UARTCopy_SendStr("going to sleep in 20 sec\n\r");  
            return 1;           
        }
    }    
    return 0;
}

#define LOOP_PRESSURE (1) // in atp version it should be 1, in normal mode it should be 3

typedef struct
{
    int8 count;
    uint32 nextTime;
    int8 enabled;
    short data[LOOP_PRESSURE];
}TPressureCtrlData;

static int st_xshut_initialized = 0;

void setSamplingPeriod(uint16_t a_SamplingPeriod)
{
    g_samplingPeriod = a_SamplingPeriod;
}

uint16_t getSamplingPeriod(void)
{
    return g_samplingPeriod;
}

void startSamplingPeriod(void)
{
    g_ATPNextTime = st_raw_time()+ g_samplingPeriod*MINUTE_IN_SECONDS;
}

int8_t CAP_isATPTimeout(void)
{
    unsigned long check_time;
    //UARTCopy_SendStr("S");
    check_time = st_raw_time();
    if (check_time < g_ATPNextTime)
    {
        return 0;
    }
    
    //UserFlash_readSystemSamplingPeriod(&g_samplingPeriod);
    
    g_ATPNextTime = check_time + g_samplingPeriod*MINUTE_IN_SECONDS;
    
    //UARTCopy_SendStr("Sampling Period Event\n\r");
    return handleTestBoxInt(SCHEDULER_EVENT, 0);    
}
#include "sw2.h"
void IOSleep(void)
{ 
    buzzer_Sleep();
}

void IOWakeup(void)
{
    buzzer_Wakeup();
}

void peripheralsPowerOn(void)
{
    V_Pull_AHT_Write(1);
    V_RH_Write(1);
    V_Presure_Write(1);
    V_E2_Write(1);
    V_Pull_Write(1);
    presure_reset_Write(1);
    CyDelay(100);
    
}
void peripheralsPowerOff(void)
{
    V_Pull_AHT_Write(0);
    presure_reset_Write(0);
    V_Pull_Write(0);
   // V_RH_Write(0);
    V_Presure_Write(0);
    V_E2_Write(0);
    
}

uint8_t CAP_checkTimeToSleep(void)
{
    unsigned long check_time = st_raw_time();
    //UARTCopy_SendStr("0");
    if ((0 != g_debugPortTimeout) || (check_time < g_magnetClosed_TimeToSleep))
    {
        return 0;         
    }
    //UARTCopy_SendStr("1");
    g_magnetClosed_TimeToSleep = 0;
    
    if (0 == g_magneticSwitch)
    {
        return 0;
    }
    //UARTCopy_SendStr("2");
    CAP_Sleep();
    return 1;
}

void CAP_Sleep(void)
{
    //UARTCopy_SendStr("Sleep\n\r");  
    CyBle_GappStopAdvertisement();
    
    // g_sleepHappened is checked in order to avoid sleep event more then once.
    // this event should be written when the magent was nearby and then detahced.
    if (0 == g_sleepHappened)
    {
        LogMeasureEvent(GeneralEvent_Code_Sleep, 0, 0, 0, 0, 0);
    }
    peripheralsPowerOff();
    //g_shockInt1Occured = 0;
    g_testBoxIntOccured = 0;    
    g_sleepHappened = 1;
    UART_Sleep();
    //CyDelay(10);
    I2C_Sleep();   
    LEDS_Sleep();
    IOSleep();
    closeDebugPort();
}

int Cap_OffLineProcess(void)
{
    int ret = 0;
        
    if (!st_xshut_initialized)
    {
        st_xshut_initialized = 1;
        Cap_Start();
    }
    //now = st_raw_time();
    if (!Cap_IsStarted())
    {
        //st_data_filled = 0;
        //return 0;
    }
    
    // if the magnet is attached then the device is almost sleep. do not try to work with sensors
    if (1 == g_magneticSwitchIntOccured)
    {
        g_magneticSwitchIntOccured = 0;
        
        if (handleMagneticSwitchInt())       
        {
            // transmit anyway no metter the magnet status
            return 1;
        }
        else
        {
            ret = 0;
        }
    }
    
    // go to sleep 20 seconds after the magent is attached
    CAP_checkDebugPortTimeout();
    /*if (1 == CAP_checkTimeToSleep())
    {
        return 0;
    }*/
    
    //from now on - only event that are enabled if the magnet is not attached.
    if (0 == g_magneticSwitch)
    {
        return 0;
    }
    
    //UARTCopy_SendStr("end1\n\r");
    if (1 == g_rxIntOccured)
    {
        g_rxIntOccured = 0;
        handleRxInt();
    }
    //UARTCopy_SendStr("end2\n\r");
    if (1 == g_testBoxIntOccured)
    {
        g_testBoxIntOccured = 0;
        
        // do not Advetising when test button is pressed, just measure and add to log
        ret = handleTestBoxInt(TEST_BUTTON_EVENT, 0);               
    }
    //UARTCopy_SendStr("end3\n\r");
    // only if the magnet is not attached . 
    if (1 == g_magneticSwitch)
    {
        ret = CAP_isATPTimeout();
    }
    
    //UARTCopy_SendStr("end4\n\r");    
    if (!ret)
        if (st_idle_timeout_is_reached())
            ret = -1;
    return ret;
}

int Cap_OnDisconnect()
{
    UARTCopy_SendStr("Disconnect\n\r");
       
    if (st_idle_timeout_is_reached())
        st_go_sleep = 1;
    st_idle_timeout_till = 0;
    LogGeneralEvent(GeneralEvent_Code_AppDisconnected, 0);
    LEDs_Off();
    if (st_measurements_to_do)
    {
        st_measurements_to_do = 0;
    }
    if (st_go_sleep) {
        st_go_sleep = 0;
        return 0;
    }
    
    return 1;
}

static unsigned short st_adv_general_events = 0;
static unsigned char st_adv_id = 0;

int Cap_FillAdv(unsigned char *advData)
{  
    unsigned short general_events = Log_GetGeneralEventsSignature();
    
    uint16_t currTemerature = CAP_getlastTemeratureMeasure();
    static uint16_t prevTemerature = 0;
    
    uint16_t currHummidity = CAP_getlastHummidityMeasure();
    static uint16_t prevHummidity = 0;
    
    uint16_t currPressure = CAP_getlastPressureMeasure();
    static uint16_t prevPressure = 0;
    
    unsigned char id = st_advertisement_id;
        // PREV                   CURRENT 
    if (((st_adv_general_events == general_events)) &&
        (currTemerature == prevTemerature) &&
        (currHummidity == prevHummidity) &&
        (currPressure == prevPressure) &&        
        (st_adv_id == id))
        return 0;
    
    prevTemerature = currTemerature;
    prevHummidity = currHummidity;
    prevPressure = currPressure;
    
    st_adv_general_events = general_events;
    st_adv_id = id;
    
    st_serialize_uint16(advData, st_adv_general_events);    
    st_serialize_uint16(advData + 2, currTemerature);    
    st_serialize_uint16(advData + 4, currHummidity);    
    st_serialize_uint16(advData + 6, currPressure);
    return 1;
}



