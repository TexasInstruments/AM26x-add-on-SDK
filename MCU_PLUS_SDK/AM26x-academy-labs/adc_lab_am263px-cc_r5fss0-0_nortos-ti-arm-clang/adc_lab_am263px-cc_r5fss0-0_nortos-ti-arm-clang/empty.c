/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <math.h>
#include <drivers/adc.h>
#include <drivers/dac.h>
#include <drivers/gpio.h>
#include <drivers/epwm.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "drivers/dac/v0/dac.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*
 * This is an empty project provided for all cores present in the device.
 * User can use this project to start their application by adding more SysConfig modules.
 *
 * This application does driver and board init and just prints the pass string on the console.
 * In case of the main core, the print is redirected to the UART console.
 * For all other cores, CCS prints are used.
 */

/* Number of ADC conversions required */
#define ADC_CONVERSION_COUNT    2048U 

#define EPWM_FREQ       (200000000U)
#define EPWM_PRD        (2000U)
#define OUTPUT_FREQ     (50U)

#define PI              (2*(asin(1.0)))

/* Global variables and objects */
volatile uint32_t gAdc1Result0[ADC_CONVERSION_COUNT];
volatile uint32_t gIndex = 0;

uint32_t gAdc1baseAddr = CONFIG_ADC1_BASE_ADDR;
uint32_t gAdc1resultBaseAddr = CONFIG_ADC1_RESULT_BASE_ADDR;

static HwiP_Object  gAdcHwiObject;

float gAlpha = 0;
float gStepValue = 0;

uint32_t gDacBaseAddr = CONFIG_DAC0_BASE_ADDR;
uint16_t gDacOutput = 0;

static void App_adcISR(void *args);

void empty_main(void *args)
{
    gStepValue = (OUTPUT_FREQ * 2 * PI) / (EPWM_FREQ/EPWM_PRD);
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    int32_t  status;
    /* Initialising a Interrupt parameter */
    HwiP_Params  hwiPrms;
    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.priority    = 0;                        /* setting high priority. optional */
    hwiPrms.callback    = &App_adcISR;
    status              = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Starting the EPWM-TB Counter*/
    EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP);

    /* Wait until all the conversions are done.*/
    while(1)
    {
        GPIO_pinWriteHigh(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);
        ClockP_sleep(1);
        GPIO_pinWriteLow(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);
        ClockP_sleep(1);
    }

    Board_driversClose();
    Drivers_close();
}

static void App_adcISR(void *args)
{
    gAdc1Result0[gIndex] = ADC_readResult(gAdc1resultBaseAddr, ADC_SOC_NUMBER0);

    if(gIndex == ADC_CONVERSION_COUNT - 1) 
    {
        gIndex = 0;
    }
    else
    {
        gIndex++;
    }
    
    gDacOutput = ((uint16_t)((0x8000 *sin(gAlpha)) + 0x8000)) >> 4;

    gAlpha += gStepValue;

    if(gAlpha >= 2*PI)
    {
        gAlpha = 0;
    }
    
    // Update DAC output
    DAC_setShadowValue(gDacBaseAddr, gDacOutput);

    // Clear interrupt flags
    ADC_clearInterruptStatus(CONFIG_ADC1_BASE_ADDR, ADC_INT_NUMBER1);
    if(true == ADC_getInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
        ADC_clearInterruptOverflowStatus(gAdc1baseAddr, ADC_INT_NUMBER1);
    }
}