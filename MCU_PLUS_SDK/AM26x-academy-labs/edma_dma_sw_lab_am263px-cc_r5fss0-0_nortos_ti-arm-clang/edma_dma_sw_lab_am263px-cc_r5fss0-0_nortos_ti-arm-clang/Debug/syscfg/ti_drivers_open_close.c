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

/*
 * Auto generated file
 */

#include "ti_drivers_open_close.h"
#include <kernel/dpl/DebugP.h>

void Drivers_open(void)
{
    Drivers_edmaOpen();
    Drivers_adcOpen();
    Drivers_epwmOpen();
    Drivers_dmaTrigXbarOpen();
    Drivers_dmaXbarOpen();
    Drivers_uartOpen();
}

void Drivers_close(void)
{
    Drivers_uartClose();
    Drivers_edmaClose();
}

void Drivers_adcOpen()
{
	/* myADC0 initialization */

	/* Configures the analog-to-digital converter module prescaler. */
	ADC_setPrescaler(myADC0_BASE_ADDR, ADC_CLK_DIV_4_0);
	/* Configures the analog-to-digital converter resolution and signal mode. */
	ADC_setMode(myADC0_BASE_ADDR, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	/* Sets the priority mode of the SOCs. */
	ADC_setSOCPriority(myADC0_BASE_ADDR, ADC_PRI_ALL_ROUND_ROBIN);

	/* Start of Conversion 0 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 0, ADC_TRIGGER_EPWM2_SOCA, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 0, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 1 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 1, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 2 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 2, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 3 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 3, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 4 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 4, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 4, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 5 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 5, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 5, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 6 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 6, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 6, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 7 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 7, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 7, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 8 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 8, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 8, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 9 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 9, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 9, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 10 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 10, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 10, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 11 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 11, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 11, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 12 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 12, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 12, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 13 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 13, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 13, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 14 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 14, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 14, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 15 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(myADC0_BASE_ADDR, 15, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 16);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(myADC0_BASE_ADDR, 15, ADC_INT_SOC_TRIGGER_NONE);

	/* Powers up the analog-to-digital converter core. */
	ADC_enableConverter(myADC0_BASE_ADDR);
    /* Delay for ADC to power up. */
    ClockP_usleep(500);
	/* Sets the timing of the end-of-conversion pulse */
	ADC_setInterruptPulseMode(myADC0_BASE_ADDR, ADC_PULSE_END_OF_CONV);
    //
    // Enable alternate timings for DMA trigger
    //
	ADC_enableAltDMATiming(myADC0_BASE_ADDR);


	/* ADC Interrupt 1 Configuration */
	/* Enables an ADC interrupt source. */
	ADC_enableInterrupt(myADC0_BASE_ADDR, 0);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(myADC0_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Enables continuous mode for an ADC interrupt. */
	ADC_enableContinuousMode(myADC0_BASE_ADDR, 0);

	/* ADC Interrupt 2 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(myADC0_BASE_ADDR, 1);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(myADC0_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(myADC0_BASE_ADDR, 1);

	/* ADC Interrupt 3 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(myADC0_BASE_ADDR, 2);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(myADC0_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(myADC0_BASE_ADDR, 2);

	/* ADC Interrupt 4 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(myADC0_BASE_ADDR, 3);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(myADC0_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(myADC0_BASE_ADDR, 3);



	/* Post Processing Block 1 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(myADC0_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(myADC0_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(myADC0_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(myADC0_BASE_ADDR, 0, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(myADC0_BASE_ADDR, 0, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(myADC0_BASE_ADDR, 0);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(myADC0_BASE_ADDR, 0, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(myADC0_BASE_ADDR, 0);
	ADC_setPPBCountLimit(myADC0_BASE_ADDR, 0,0);
	ADC_selectPPBSyncInput(myADC0_BASE_ADDR, 0,ADC_SYNCIN_DISABLE);
	ADC_selectPPBCompareSource(myADC0_BASE_ADDR, 0,ADC_PPB_COMPSOURCE_RESULT);
	ADC_setPPBShiftValue(myADC0_BASE_ADDR, 0,0);
	ADC_disablePPBAbsoluteValue(myADC0_BASE_ADDR, 0);

	/* Post Processing Block 2 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(myADC0_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(myADC0_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(myADC0_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(myADC0_BASE_ADDR, 1, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(myADC0_BASE_ADDR, 1, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(myADC0_BASE_ADDR, 1);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(myADC0_BASE_ADDR, 1, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(myADC0_BASE_ADDR, 1);
	ADC_setPPBCountLimit(myADC0_BASE_ADDR, 1,0);
	ADC_selectPPBSyncInput(myADC0_BASE_ADDR, 1,ADC_SYNCIN_DISABLE);
	ADC_selectPPBCompareSource(myADC0_BASE_ADDR, 1,ADC_PPB_COMPSOURCE_RESULT);
	ADC_setPPBShiftValue(myADC0_BASE_ADDR, 1,0);
	ADC_disablePPBAbsoluteValue(myADC0_BASE_ADDR, 1);

	/* Post Processing Block 3 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(myADC0_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(myADC0_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(myADC0_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(myADC0_BASE_ADDR, 2, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(myADC0_BASE_ADDR, 2, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(myADC0_BASE_ADDR, 2);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(myADC0_BASE_ADDR, 2, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(myADC0_BASE_ADDR, 2);
	ADC_setPPBCountLimit(myADC0_BASE_ADDR, 2,0);
	ADC_selectPPBSyncInput(myADC0_BASE_ADDR, 2,ADC_SYNCIN_DISABLE);
	ADC_selectPPBCompareSource(myADC0_BASE_ADDR, 2,ADC_PPB_COMPSOURCE_RESULT);
	ADC_setPPBShiftValue(myADC0_BASE_ADDR, 2,0);
	ADC_disablePPBAbsoluteValue(myADC0_BASE_ADDR, 2);

	/* Post Processing Block 4 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(myADC0_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(myADC0_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(myADC0_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(myADC0_BASE_ADDR, 3, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(myADC0_BASE_ADDR, 3, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(myADC0_BASE_ADDR, 3);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(myADC0_BASE_ADDR, 3, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(myADC0_BASE_ADDR, 3);
	ADC_setPPBCountLimit(myADC0_BASE_ADDR, 3,0);
	ADC_selectPPBSyncInput(myADC0_BASE_ADDR, 3,ADC_SYNCIN_DISABLE);
	ADC_selectPPBCompareSource(myADC0_BASE_ADDR, 3,ADC_PPB_COMPSOURCE_RESULT);
	ADC_setPPBShiftValue(myADC0_BASE_ADDR, 3,0);
	ADC_disablePPBAbsoluteValue(myADC0_BASE_ADDR, 3);

	/* Set SOC burst mode. */
	ADC_setBurstModeConfig(myADC0_BASE_ADDR, ADC_TRIGGER_SW_ONLY, 1);
	/* Disables SOC burst mode. */
	ADC_disableBurstMode(myADC0_BASE_ADDR);
}

/*
 * EDMA
 */
/* EDMA Driver handles */
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

/* EDMA Driver Open Parameters */
EDMA_Params gEdmaParams[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        .intrEnable = TRUE,
    },
};

void Drivers_edmaOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        gEdmaHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        gEdmaHandle[instCnt] = EDMA_open(instCnt, &gEdmaParams[instCnt]);
        if(NULL == gEdmaHandle[instCnt])
        {
            DebugP_logError("EDMA open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_edmaClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_edmaClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        if(gEdmaHandle[instCnt] != NULL)
        {
            EDMA_close(gEdmaHandle[instCnt]);
            gEdmaHandle[instCnt] = NULL;
        }
    }

    return;
}

void Drivers_epwmOpen(void)
{
	/* myEPWM0 initialization */

	/* Time Base */
	EPWM_setEmulationMode(myEPWM0_BASE_ADDR, EPWM_EMULATION_STOP_AFTER_NEXT_TB);
	EPWM_setClockPrescaler(myEPWM0_BASE_ADDR, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(myEPWM0_BASE_ADDR, 25000);
	EPWM_disableGlobalLoadRegisters(myEPWM0_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(myEPWM0_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_selectPeriodLoadEvent(myEPWM0_BASE_ADDR, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);
	EPWM_setTimeBaseCounter(myEPWM0_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(myEPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP_DOWN);
	EPWM_setCountModeAfterSync(myEPWM0_BASE_ADDR, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
	EPWM_disablePhaseShiftLoad(myEPWM0_BASE_ADDR);
	EPWM_setPhaseShift(myEPWM0_BASE_ADDR, 0);
	EPWM_enableSyncOutPulseSource(myEPWM0_BASE_ADDR, 0);
	EPWM_setSyncInPulseSource(myEPWM0_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_DISABLE);
	EPWM_setOneShotSyncOutTrigger(myEPWM0_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	
    HRPWM_setSyncPulseSource(myEPWM0_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);

	/* Counter Compare */
	EPWM_setCounterCompareValue(myEPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 18750);
	EPWM_disableGlobalLoadRegisters(myEPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(myEPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 0);
	EPWM_disableGlobalLoadRegisters(myEPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(myEPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(myEPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(myEPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(myEPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(myEPWM0_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(myEPWM0_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(myEPWM0_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(myEPWM0_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(myEPWM0_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(myEPWM0_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierSWAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
    EPWM_setTripZoneAction(myEPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(myEPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(myEPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(myEPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(myEPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(myEPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(myEPWM0_BASE_ADDR);
	EPWM_enableTripZoneSignals(myEPWM0_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(myEPWM0_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(myEPWM0_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZone2Signals(myEPWM0_BASE_ADDR, 0);
	EPWM_enableTripZone2Signals(myEPWM0_BASE_ADDR, 0);
	EPWM_enableTripZoneInterrupt(myEPWM0_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(myEPWM0_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(myEPWM0_BASE_ADDR);
    EPWM_setDigitalCompareBlankingEvent(myEPWM0_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, 0);
	EPWM_setDigitalCompareWindowOffset(myEPWM0_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(myEPWM0_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(myEPWM0_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(myEPWM0_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(myEPWM0_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(myEPWM0_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(myEPWM0_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(myEPWM0_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(myEPWM0_BASE_ADDR);
	EPWM_setValleyTriggerSource(myEPWM0_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(myEPWM0_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(myEPWM0_BASE_ADDR);
	EPWM_setValleySWDelayValue(myEPWM0_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(myEPWM0_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(myEPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(myEPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(myEPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(myEPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(myEPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(myEPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(myEPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(myEPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(myEPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(myEPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(myEPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(myEPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(myEPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

    /* DCCAP Edge Detection */
	EPWM_disableCaptureInEvent(myEPWM0_BASE_ADDR);
	EPWM_selectCaptureTripInput(myEPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_CAPTURE_GATE);
	EPWM_configCaptureGateInputPolarity(myEPWM0_BASE_ADDR, EPWM_CAPGATE_INPUT_ALWAYS_ON);
	EPWM_selectCaptureTripInput(myEPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_CAPTURE_INPUT);
	EPWM_invertCaptureInputPolarity(myEPWM0_BASE_ADDR, EPWM_CAPTURE_INPUT_CAPIN_SYNC);
	EPWM_disableIndependentPulseLogic(myEPWM0_BASE_ADDR);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(myEPWM0_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(myEPWM0_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(myEPWM0_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(myEPWM0_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(myEPWM0_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayMode(myEPWM0_BASE_ADDR, EPWM_DB_RED, false);
	EPWM_setDeadBandDelayMode(myEPWM0_BASE_ADDR, EPWM_DB_FED, false);
	EPWM_setDeadBandOutputSwapMode(myEPWM0_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(myEPWM0_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(myEPWM0_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(myEPWM0_BASE_ADDR, 0);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(myEPWM0_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(myEPWM0_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(myEPWM0_BASE_ADDR, 0);
	EPWM_setDeadBandCounterClock(myEPWM0_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(myEPWM0_BASE_ADDR);
	EPWM_setChopperDutyCycle(myEPWM0_BASE_ADDR, 0);
	EPWM_setChopperFreq(myEPWM0_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(myEPWM0_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(myEPWM0_BASE_ADDR);
	EPWM_setInterruptSource(myEPWM0_BASE_ADDR, EPWM_INT_TBCTR_ZERO, 0);
	EPWM_setInterruptEventCount(myEPWM0_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(myEPWM0_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(myEPWM0_BASE_ADDR, 0);
	
	EPWM_disableADCTrigger(myEPWM0_BASE_ADDR, EPWM_SOC_A);
    EPWM_setADCTriggerSource(myEPWM0_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_DCxEVT1, 0);
	EPWM_setADCTriggerEventPrescale(myEPWM0_BASE_ADDR, EPWM_SOC_A, 0);
	EPWM_disableADCTriggerEventCountInit(myEPWM0_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(myEPWM0_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(myEPWM0_BASE_ADDR, EPWM_SOC_B);
    EPWM_setADCTriggerSource(myEPWM0_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, 0);
	EPWM_setADCTriggerEventPrescale(myEPWM0_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(myEPWM0_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(myEPWM0_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(myEPWM0_BASE_ADDR);
    EPWM_disableSplitXCMP(myEPWM0_BASE_ADDR);
	EPWM_allocAXCMP(myEPWM0_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(myEPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(myEPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(myEPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM0_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(myEPWM0_BASE_ADDR, EPWM_XMIN_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(myEPWM0_BASE_ADDR, EPWM_XMAX_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(myEPWM0_BASE_ADDR, EPWM_XMIN_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(myEPWM0_BASE_ADDR, EPWM_XMAX_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(myEPWM0_BASE_ADDR, EPWM_XMIN_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(myEPWM0_BASE_ADDR, EPWM_XMAX_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(myEPWM0_BASE_ADDR, EPWM_XMIN_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(myEPWM0_BASE_ADDR, EPWM_XMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(myEPWM0_BASE_ADDR);
    EPWM_setDiodeEmulationMode(myEPWM0_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(myEPWM0_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(myEPWM0_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPL);
    EPWM_configureDiodeEmulationTripSources(myEPWM0_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPH);
    EPWM_selectDiodeEmulationPWMsignal(myEPWM0_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(myEPWM0_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(myEPWM0_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(myEPWM0_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(myEPWM0_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(myEPWM0_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(myEPWM0_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(myEPWM0_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(myEPWM0_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(myEPWM0_BASE_ADDR);
    HRPWM_setMEPControlMode(myEPWM0_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(myEPWM0_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(myEPWM0_BASE_ADDR, 0);
    HRPWM_disablePhaseShiftLoad(myEPWM0_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(myEPWM0_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(myEPWM0_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(myEPWM0_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(myEPWM0_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(myEPWM0_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(myEPWM0_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(myEPWM0_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(myEPWM0_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(myEPWM0_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(myEPWM0_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(myEPWM0_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(myEPWM0_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(myEPWM0_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(myEPWM0_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(myEPWM0_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

    /* ICL */
    EPWM_disableIllegalComboLogic(myEPWM0_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableIllegalComboLogic(myEPWM0_BASE_ADDR, EPWM_MINDB_BLOCK_B);

    /* MDL */
    EPWM_disableMinimumDeadBand(myEPWM0_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableMinimumDeadBand(myEPWM0_BASE_ADDR, EPWM_MINDB_BLOCK_B);
	/* Global Load */
	EPWM_disableGlobalLoad(myEPWM0_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(myEPWM0_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(myEPWM0_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(myEPWM0_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(myEPWM0_BASE_ADDR, 0);
	/* myEPWM1 initialization */

	/* Time Base */
	EPWM_setEmulationMode(myEPWM1_BASE_ADDR, EPWM_EMULATION_STOP_AFTER_NEXT_TB);
	EPWM_setClockPrescaler(myEPWM1_BASE_ADDR, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(myEPWM1_BASE_ADDR, 1999);
	EPWM_disableGlobalLoadRegisters(myEPWM1_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(myEPWM1_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_selectPeriodLoadEvent(myEPWM1_BASE_ADDR, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);
	EPWM_setTimeBaseCounter(myEPWM1_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(myEPWM1_BASE_ADDR, EPWM_COUNTER_MODE_UP);
	EPWM_setCountModeAfterSync(myEPWM1_BASE_ADDR, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
	EPWM_disablePhaseShiftLoad(myEPWM1_BASE_ADDR);
	EPWM_setPhaseShift(myEPWM1_BASE_ADDR, 0);
	EPWM_enableSyncOutPulseSource(myEPWM1_BASE_ADDR, 0);
	EPWM_setSyncInPulseSource(myEPWM1_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_DISABLE);
	EPWM_setOneShotSyncOutTrigger(myEPWM1_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	
    HRPWM_setSyncPulseSource(myEPWM1_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);

	/* Counter Compare */
	EPWM_setCounterCompareValue(myEPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 0);
	EPWM_disableGlobalLoadRegisters(myEPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(myEPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 0);
	EPWM_disableGlobalLoadRegisters(myEPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(myEPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(myEPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(myEPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(myEPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(myEPWM1_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(myEPWM1_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(myEPWM1_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(myEPWM1_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(myEPWM1_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(myEPWM1_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierSWAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
    EPWM_setTripZoneAction(myEPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(myEPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(myEPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(myEPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(myEPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_setTripZoneAction(myEPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(myEPWM1_BASE_ADDR);
	EPWM_enableTripZoneSignals(myEPWM1_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(myEPWM1_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(myEPWM1_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZone2Signals(myEPWM1_BASE_ADDR, 0);
	EPWM_enableTripZone2Signals(myEPWM1_BASE_ADDR, 0);
	EPWM_enableTripZoneInterrupt(myEPWM1_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(myEPWM1_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(myEPWM1_BASE_ADDR);
    EPWM_setDigitalCompareBlankingEvent(myEPWM1_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, 0);
	EPWM_setDigitalCompareWindowOffset(myEPWM1_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(myEPWM1_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(myEPWM1_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(myEPWM1_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(myEPWM1_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(myEPWM1_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(myEPWM1_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(myEPWM1_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(myEPWM1_BASE_ADDR);
	EPWM_setValleyTriggerSource(myEPWM1_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(myEPWM1_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(myEPWM1_BASE_ADDR);
	EPWM_setValleySWDelayValue(myEPWM1_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(myEPWM1_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(myEPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(myEPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(myEPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(myEPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(myEPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(myEPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(myEPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(myEPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(myEPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(myEPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(myEPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(myEPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(myEPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

    /* DCCAP Edge Detection */
	EPWM_disableCaptureInEvent(myEPWM1_BASE_ADDR);
	EPWM_selectCaptureTripInput(myEPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_CAPTURE_GATE);
	EPWM_configCaptureGateInputPolarity(myEPWM1_BASE_ADDR, EPWM_CAPGATE_INPUT_ALWAYS_ON);
	EPWM_selectCaptureTripInput(myEPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_CAPTURE_INPUT);
	EPWM_invertCaptureInputPolarity(myEPWM1_BASE_ADDR, EPWM_CAPTURE_INPUT_CAPIN_SYNC);
	EPWM_disableIndependentPulseLogic(myEPWM1_BASE_ADDR);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(myEPWM1_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(myEPWM1_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(myEPWM1_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(myEPWM1_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(myEPWM1_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayMode(myEPWM1_BASE_ADDR, EPWM_DB_RED, false);
	EPWM_setDeadBandDelayMode(myEPWM1_BASE_ADDR, EPWM_DB_FED, false);
	EPWM_setDeadBandOutputSwapMode(myEPWM1_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(myEPWM1_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(myEPWM1_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(myEPWM1_BASE_ADDR, 0);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(myEPWM1_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(myEPWM1_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(myEPWM1_BASE_ADDR, 0);
	EPWM_setDeadBandCounterClock(myEPWM1_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(myEPWM1_BASE_ADDR);
	EPWM_setChopperDutyCycle(myEPWM1_BASE_ADDR, 0);
	EPWM_setChopperFreq(myEPWM1_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(myEPWM1_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(myEPWM1_BASE_ADDR);
	EPWM_setInterruptSource(myEPWM1_BASE_ADDR, EPWM_INT_TBCTR_ZERO, 0);
	EPWM_setInterruptEventCount(myEPWM1_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(myEPWM1_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(myEPWM1_BASE_ADDR, 0);
	
	EPWM_enableADCTrigger(myEPWM1_BASE_ADDR, EPWM_SOC_A);
    EPWM_setADCTriggerSource(myEPWM1_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO, 0);
	EPWM_setADCTriggerEventPrescale(myEPWM1_BASE_ADDR, EPWM_SOC_A, 1);
	EPWM_enableADCTriggerEventCountInit(myEPWM1_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(myEPWM1_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(myEPWM1_BASE_ADDR, EPWM_SOC_B);
    EPWM_setADCTriggerSource(myEPWM1_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, 0);
	EPWM_setADCTriggerEventPrescale(myEPWM1_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(myEPWM1_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(myEPWM1_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(myEPWM1_BASE_ADDR);
    EPWM_disableSplitXCMP(myEPWM1_BASE_ADDR);
	EPWM_allocAXCMP(myEPWM1_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(myEPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(myEPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(myEPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(myEPWM1_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(myEPWM1_BASE_ADDR, EPWM_XMIN_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(myEPWM1_BASE_ADDR, EPWM_XMAX_ACTIVE, 0);
    EPWM_setXMINMAXRegValue(myEPWM1_BASE_ADDR, EPWM_XMIN_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(myEPWM1_BASE_ADDR, EPWM_XMAX_SHADOW1, 0);
    EPWM_setXMINMAXRegValue(myEPWM1_BASE_ADDR, EPWM_XMIN_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(myEPWM1_BASE_ADDR, EPWM_XMAX_SHADOW2, 0);
    EPWM_setXMINMAXRegValue(myEPWM1_BASE_ADDR, EPWM_XMIN_SHADOW3, 0);
    EPWM_setXMINMAXRegValue(myEPWM1_BASE_ADDR, EPWM_XMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(myEPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(myEPWM1_BASE_ADDR);
    EPWM_setDiodeEmulationMode(myEPWM1_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(myEPWM1_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(myEPWM1_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPL);
    EPWM_configureDiodeEmulationTripSources(myEPWM1_BASE_ADDR, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0, EPWM_DE_TRIPH);
    EPWM_selectDiodeEmulationPWMsignal(myEPWM1_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(myEPWM1_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(myEPWM1_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(myEPWM1_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(myEPWM1_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(myEPWM1_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(myEPWM1_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(myEPWM1_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(myEPWM1_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(myEPWM1_BASE_ADDR);
    HRPWM_setMEPControlMode(myEPWM1_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(myEPWM1_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(myEPWM1_BASE_ADDR, 0);
    HRPWM_disablePhaseShiftLoad(myEPWM1_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(myEPWM1_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(myEPWM1_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(myEPWM1_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(myEPWM1_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(myEPWM1_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(myEPWM1_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(myEPWM1_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(myEPWM1_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(myEPWM1_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(myEPWM1_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(myEPWM1_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(myEPWM1_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(myEPWM1_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(myEPWM1_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(myEPWM1_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

    /* ICL */
    EPWM_disableIllegalComboLogic(myEPWM1_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableIllegalComboLogic(myEPWM1_BASE_ADDR, EPWM_MINDB_BLOCK_B);

    /* MDL */
    EPWM_disableMinimumDeadBand(myEPWM1_BASE_ADDR, EPWM_MINDB_BLOCK_A);
    EPWM_disableMinimumDeadBand(myEPWM1_BASE_ADDR, EPWM_MINDB_BLOCK_B);
	/* Global Load */
	EPWM_disableGlobalLoad(myEPWM1_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(myEPWM1_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(myEPWM1_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(myEPWM1_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(myEPWM1_BASE_ADDR, 0);
}

void Drivers_dmaTrigXbarOpen()
{
    /*
    * DMA TRIGGER XBAR
    */
    SOC_xbarSelectEdmaTrigXbarInputSource(CSL_EDMA_TRIG_XBAR_U_BASE, DMA_TRIG_XBAR_EDMA_MODULE_0, DMA_TRIG_XBAR_EDMA_MODULE_0_INPUT);
}
void Drivers_dmaXbarOpen()
{
    /* DMA XBAR */
    SOC_xbarSelectDMAXBarInputSource_ext(CSL_CONTROLSS_DMAXBAR_U_BASE, 0, 2, 0, 0, DMA_XBAR_ADC1_INT1, 0, 0, 0, 0);
}

/*
 * UART
 */

/* UART Driver handles */
UART_Handle gUartHandle[CONFIG_UART_NUM_INSTANCES];

#include <drivers/uart/v0/lld/dma/uart_dma.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>

UART_DmaChConfig gUartDmaChConfig[CONFIG_UART_NUM_INSTANCES] =
{
                NULL,
};

/* UART Driver Parameters */
UART_Params gUartParams[CONFIG_UART_NUM_INSTANCES] =
{
        {
            .baudRate           = 115200,
            .dataLength         = UART_LEN_8,
            .stopBits           = UART_STOPBITS_1,
            .parityType         = UART_PARITY_NONE,
            .readMode           = UART_TRANSFER_MODE_BLOCKING,
            .readReturnMode     = UART_READ_RETURN_MODE_FULL,
            .writeMode          = UART_TRANSFER_MODE_BLOCKING,
            .readCallbackFxn    = NULL,
            .writeCallbackFxn   = NULL,
            .hwFlowControl      = FALSE,
            .hwFlowControlThr   = UART_RXTRIGLVL_16,
            .transferMode       = UART_CONFIG_MODE_INTERRUPT,
            .skipIntrReg         = FALSE,
            .uartDmaIndex = -1,
            .intrNum            = 38U,
            .intrPriority       = 4U,
            .operMode           = UART_OPER_MODE_16X,
            .rxTrigLvl          = UART_RXTRIGLVL_8,
            .txTrigLvl          = UART_TXTRIGLVL_32,
            .rxEvtNum           = 0U,
            .txEvtNum           = 0U,
        },
};

void Drivers_uartOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = UART_open(instCnt, &gUartParams[instCnt]);
        if(NULL == gUartHandle[instCnt])
        {
            DebugP_logError("UART open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_uartClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_uartClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        if(gUartHandle[instCnt] != NULL)
        {
            UART_close(gUartHandle[instCnt]);
            gUartHandle[instCnt] = NULL;
        }
    }

    return;
}
