/*******************************************************************************
* File Name: app_LED.c
*
* Description:
*  Common BLE application code for client devices.
*
*******************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "app_LED.h"

////

//// LEDs

void LEDs_Red_On()
{
    red_Write(0);
}

void LEDs_Red_Off()
{
    red_Write(1);
}


void LEDs_A_On(void)
{
    LEDs_Blue_On();
}
void LEDs_A_Off(void)
{
    LEDs_Blue_Off();
}
void LEDs_C_On(void)
{
    //Conn_LED_Write(0);
}
void LEDs_C_Off(void)
{
    //Conn_LED_Write(1);
}
void LEDs_On(void)
{
    LEDs_Red_On();
}
void LEDs_Off(void)
{
    LEDs_Red_Off();
}



void LEDS_powerUpBlink(TLedType a_led)
{
    int i = 0;
        
    for (i = 0 ; i < 5 ; i++)
    {
        if (RED == a_led)
            LEDs_Red_On();
               
        CyDelay(150);
        
        if (RED == a_led)
            LEDs_Red_Off();
        
        CyDelay(150);        
    }
}



void LEDS_Sleep(void)
{
    //red_Sleep();
}

void LEDS_Wakeup(void)
{
    //red_Wakeup();
}