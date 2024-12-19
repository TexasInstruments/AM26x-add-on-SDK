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
#include <drivers/epwm.h>
#include <drivers/adc.h>
#include <drivers/soc/am263px/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SystemP.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#if defined(__ARM_ARCH_7R__)
#ifdef __ARM_ACLE
#include <arm_acle.h>
#endif /* __ARM_ACLE */
#endif

uint16_t gAdcPingBuffer __attribute__((__section__(".adcData"))) __attribute__ ((aligned (32)));                    // Ping Buffer
uint16_t gAdcPongBuffer __attribute__((__section__(".adcData"))) __attribute__ ((aligned (32)));                    // Pong Buffer
uint16_t gAdcResultBuffer __attribute__((__section__(".adcData"))) __attribute__ ((aligned (32)));                  // ADC Result Buffer
uint64_t TimDiff, CurrTim, PrevTim = 0;                                              // To measure the DMA ISR time.
uint16_t LedCtr = 0;                                                                 // Counter to slow LED toggling.

static SemaphoreP_Object gEdmaResultAvailableSem; // Semaphore to make sure EDMA result is available or not
Edma_IntrObject     intrObj;        // Setting up EDMA object

void myEDMAISR(Edma_IntrHandle intrHandle, void *args);

int32_t ConfigureEdma(void)
{
    uint32_t            baseAddr, regionId;
    int32_t             labStatus = SystemP_SUCCESS;

    /* First Param Set to be set, this will be reloaded throughout due to linking */
    uint32_t            dmaCh0, tcc0;

    uint32_t chainOptions = (EDMA_OPT_TCCHEN_MASK | EDMA_OPT_ITCCHEN_MASK);

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    /* Start configuring the actual ADC transfer via DMA */
    dmaCh0 = EDMA_RESOURCE_ALLOC_ANY;
    labStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh0);
    DebugP_assert(labStatus == SystemP_SUCCESS);

    tcc0 = EDMA_RESOURCE_ALLOC_ANY;
    labStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc0);
    DebugP_assert(labStatus == SystemP_SUCCESS);

    /* This is the DMA transfer from ADC to Ping Buffer */
    EDMACCPaRAMEntry    edmaparam0;
    uint32_t            param0;

    /* This is the first transfer to Ping Buffer via paramADCPing */
    param0 = EDMA_RESOURCE_ALLOC_ANY;
    labStatus = EDMA_allocParam(gEdmaHandle[0], &param0);
    DebugP_assert(labStatus == SystemP_SUCCESS);

    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            dmaCh0, tcc0, param0, 0U);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaparam0);
    edmaparam0.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(myADC0_RESULT_BASE_ADDR));
    edmaparam0.destAddr      = (uint32_t) SOC_virtToPhy((void *)(&gAdcPingBuffer));
    edmaparam0.aCnt          = (uint16_t) 2;
    edmaparam0.bCnt          = (uint16_t) 1;
    edmaparam0.cCnt          = (uint16_t) 1;
    edmaparam0.bCntReload    = 0;
    edmaparam0.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
    edmaparam0.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaparam0.srcCIdx       = (int16_t) 0;
    edmaparam0.destCIdx      = (int16_t) 0;
    edmaparam0.linkAddr      = 0xFFFFU;
    edmaparam0.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaparam0.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaparam0.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
                              ((((uint32_t)tcc0) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(baseAddr, param0, &edmaparam0);

    /* This is the DMA transfer from ADC to Pong Buffer */
    EDMACCPaRAMEntry    edmaADCPongParam;
    uint32_t            paramADCPong;

    /* This is the subsequent transfer to Pong Buffer via paramADCPong */
    paramADCPong = EDMA_RESOURCE_ALLOC_ANY;
    labStatus = EDMA_allocParam(gEdmaHandle[0], &paramADCPong);
    DebugP_assert(labStatus == SystemP_SUCCESS);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaADCPongParam);
    edmaADCPongParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(myADC0_RESULT_BASE_ADDR));
    edmaADCPongParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)(&gAdcPongBuffer));
    edmaADCPongParam.aCnt          = (uint16_t) 2;
    edmaADCPongParam.bCnt          = (uint16_t) 1;
    edmaADCPongParam.cCnt          = (uint16_t) 1;
    edmaADCPongParam.bCntReload    = 0;
    edmaADCPongParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
    edmaADCPongParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaADCPongParam.srcCIdx       = (int16_t) 0;
    edmaADCPongParam.destCIdx      = (int16_t) 0;
    edmaADCPongParam.linkAddr      = 0xFFFFU;
    edmaADCPongParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaADCPongParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaADCPongParam.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK);

    EDMA_setPaRAM(baseAddr, paramADCPong, &edmaADCPongParam);

    EDMA_linkChannel(baseAddr, param0, paramADCPong);

    /* This is the DMA transfer from ADC to Ping Buffer */
    EDMACCPaRAMEntry    edmaADCPingParam;
    uint32_t            paramADCPing;

    /* This is the first transfer to Ping Buffer via paramADCPing */
    paramADCPing = EDMA_RESOURCE_ALLOC_ANY;
    labStatus = EDMA_allocParam(gEdmaHandle[0], &paramADCPing);
    DebugP_assert(labStatus == SystemP_SUCCESS);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaADCPingParam);
    edmaADCPingParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(myADC0_RESULT_BASE_ADDR));
    edmaADCPingParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)(&gAdcPingBuffer));
    edmaADCPingParam.aCnt          = (uint16_t) 2;
    edmaADCPingParam.bCnt          = (uint16_t) 1;
    edmaADCPingParam.cCnt          = (uint16_t) 1;
    edmaADCPingParam.bCntReload    = 0;
    edmaADCPingParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
    edmaADCPingParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaADCPingParam.srcCIdx       = (int16_t) 0;
    edmaADCPingParam.destCIdx      = (int16_t) 0;
    edmaADCPingParam.linkAddr      = 0xFFFFU;
    edmaADCPingParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaADCPingParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaADCPingParam.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK);

    EDMA_setPaRAM(baseAddr, paramADCPing, &edmaADCPingParam);

    EDMA_linkChannel(baseAddr, paramADCPong, paramADCPing);
    EDMA_linkChannel(baseAddr, paramADCPing, paramADCPong);

    /* Second Param Set to be set, this will be reloaded throughout due to linking */
    EDMACCPaRAMEntry    edmaparam1;
    uint32_t            dmaCh1, tcc1, param1;

    /* Start configuring the actual ADC transfer via DMA */
    dmaCh1 = EDMA_RESOURCE_ALLOC_ANY;
    labStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh1);
    DebugP_assert(labStatus == SystemP_SUCCESS);

    tcc1 = EDMA_RESOURCE_ALLOC_ANY;
    labStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc1);
    DebugP_assert(labStatus == SystemP_SUCCESS);

    /* This is the first transfer to Ping Buffer via paramADCPing */
    param1 = EDMA_RESOURCE_ALLOC_ANY;
    labStatus = EDMA_allocParam(gEdmaHandle[0], &param1);
    DebugP_assert(labStatus == SystemP_SUCCESS);

    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            dmaCh1, tcc1, param1, 0U);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaparam1);
    edmaparam1.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(&gAdcPingBuffer));
    edmaparam1.destAddr      = (uint32_t) SOC_virtToPhy((void *)(0x70100000));
    edmaparam1.aCnt          = (uint16_t) 2;
    edmaparam1.bCnt          = (uint16_t) 1;
    edmaparam1.cCnt          = (uint16_t) 1;
    edmaparam1.bCntReload    = 0;
    edmaparam1.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
    edmaparam1.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaparam1.srcCIdx       = (int16_t) 0;
    edmaparam1.destCIdx      = (int16_t) 0;
    edmaparam1.linkAddr      = 0xFFFFU;
    edmaparam1.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaparam1.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaparam1.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
                              ((((uint32_t)tcc1) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(baseAddr, param1, &edmaparam1);

    /* This is the DMA transfer from Pong to Result Buffer */
    EDMACCPaRAMEntry    edmaPongResultParam;
    uint32_t            paramPongResult;

    /* This is the subsequent transfer to Pong Buffer via paramADCPong */
    paramPongResult = EDMA_RESOURCE_ALLOC_ANY;
    labStatus = EDMA_allocParam(gEdmaHandle[0], &paramPongResult);
    DebugP_assert(labStatus == SystemP_SUCCESS);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaPongResultParam);
    edmaPongResultParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(&gAdcPongBuffer));
    edmaPongResultParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)(0x70100000));
    edmaPongResultParam.aCnt          = (uint16_t) 2;
    edmaPongResultParam.bCnt          = (uint16_t) 1;
    edmaPongResultParam.cCnt          = (uint16_t) 1;
    edmaPongResultParam.bCntReload    = 0;
    edmaPongResultParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
    edmaPongResultParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaPongResultParam.srcCIdx       = (int16_t) 0;
    edmaPongResultParam.destCIdx      = (int16_t) 0;
    edmaPongResultParam.linkAddr      = 0xFFFFU;
    edmaPongResultParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaPongResultParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaPongResultParam.opt           = (EDMA_OPT_TCINTEN_MASK);

    EDMA_setPaRAM(baseAddr, paramPongResult, &edmaPongResultParam);

    EDMA_linkChannel(baseAddr, param1, paramPongResult);

    /* This is the DMA transfer from Ping to Result Buffer */
    EDMACCPaRAMEntry    edmaPingResultParam;
    uint32_t            paramPingResult;

    /* This is the third transfer to Ping Buffer via paramADCPing */
    paramPingResult = EDMA_RESOURCE_ALLOC_ANY;
    labStatus = EDMA_allocParam(gEdmaHandle[0], &paramPingResult);
    DebugP_assert(labStatus == SystemP_SUCCESS);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaPingResultParam);
    edmaPingResultParam.srcAddr       = (uint32_t) SOC_virtToPhy((void *)(&gAdcPingBuffer));
    edmaPingResultParam.destAddr      = (uint32_t) SOC_virtToPhy((void *)(0x70100000));
    edmaPingResultParam.aCnt          = (uint16_t) 2;
    edmaPingResultParam.bCnt          = (uint16_t) 1;
    edmaPingResultParam.cCnt          = (uint16_t) 1;
    edmaPingResultParam.bCntReload    = 0;
    edmaPingResultParam.srcBIdx       = (int16_t) EDMA_PARAM_BIDX(0);
    edmaPingResultParam.destBIdx      = (int16_t) EDMA_PARAM_BIDX(0);
    edmaPingResultParam.srcCIdx       = (int16_t) 0;
    edmaPingResultParam.destCIdx      = (int16_t) 0;
    edmaPingResultParam.linkAddr      = 0xFFFFU;
    edmaPingResultParam.srcBIdxExt    = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaPingResultParam.destBIdxExt   = (int8_t) EDMA_PARAM_BIDX_EXT(0);
    edmaPingResultParam.opt           = (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK);

    EDMA_setPaRAM(baseAddr, paramPingResult, &edmaPingResultParam);

    EDMA_linkChannel(baseAddr, paramPongResult, paramPingResult);
    EDMA_linkChannel(baseAddr, paramPingResult, paramPongResult);

    /*Set params for chain transfer. Chain two channels*/
    EDMA_chainChannel(baseAddr, param0, dmaCh1, chainOptions);

    /*Set params for chain transfer. Chain two channels*/
    EDMA_chainChannel(baseAddr, paramADCPing, dmaCh1, chainOptions);

    /*Set params for chain transfer. Chain two channels*/
    EDMA_chainChannel(baseAddr, paramADCPong, dmaCh1, chainOptions);

    /* Create a semaphore to signal EDMA transfer completion */
    labStatus = SemaphoreP_constructBinary(&gEdmaResultAvailableSem, 0);
    DebugP_assert(SystemP_SUCCESS == labStatus);

    /* Register interrupt for dma channel transfer completion*/
    intrObj.tccNum = tcc1;
    intrObj.cbFxn  = &myEDMAISR;
    intrObj.appData = (void *) &gEdmaResultAvailableSem;
    labStatus = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
    DebugP_assert(labStatus == SystemP_SUCCESS);


    EDMA_enableTransferRegion(baseAddr, regionId, dmaCh0,
                              EDMA_TRIG_MODE_EVENT);

    return labStatus;
}

void myEDMAISR(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);

    /* Getting time for ISR execution */
    LedCtr++;
    CurrTim = TimerP_getCount(myTIMER0_BASE_ADDR);

    /* Post the semaphore to signal end of DMA transfer */
    SemaphoreP_post(semObjPtr);
}

/*
 * This is a software lab project.
 * User can use this project to start their application by adding more SysConfig modules.
 */
void edma_sw_lab_main(void *args)
{
    int32_t labStatus = SystemP_SUCCESS;
    uint32_t cpuFreq = (200U);
    uint32_t    gpioBaseAddr, pinNum;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Start the timer */
    TimerP_start(myTIMER0_BASE_ADDR);

    /* Get address after translation translate */
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(MYBOARDLED0_GPIO_BASE_ADDR);
    pinNum       = MYBOARDLED0_GPIO_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, MYBOARDLED0_GPIO_DIR);

    /* Configure the EDMA with ADC */
    labStatus |= ConfigureEdma();

    /* Start LED here. */
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    while(labStatus == 0)
    {
        SemaphoreP_pend(&gEdmaResultAvailableSem, SystemP_WAIT_FOREVER);

        TimDiff = CurrTim - PrevTim;
        DebugP_log("Time Taken for ISR to execute : %f us\r\n", ((double)(TimDiff)/(double)cpuFreq));
        PrevTim = CurrTim;

        /* After every 1000th ISR, we will blink the LED */
        if(LedCtr%10U == 0U)
        {
            /* This is to ensure, LedCtr never lapses */
            LedCtr = 1U;

            if(GPIO_pinRead(gpioBaseAddr, pinNum) == 1U)
            {
                GPIO_pinWriteLow(gpioBaseAddr, pinNum);
            }
            else
            {
                GPIO_pinWriteHigh(gpioBaseAddr, pinNum);
            }
        }

        /* execute wfi, and reset core0 and core 1 */
        #if defined(__ARM_ARCH_7R__)
            __wfi();
        #endif
    }

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
