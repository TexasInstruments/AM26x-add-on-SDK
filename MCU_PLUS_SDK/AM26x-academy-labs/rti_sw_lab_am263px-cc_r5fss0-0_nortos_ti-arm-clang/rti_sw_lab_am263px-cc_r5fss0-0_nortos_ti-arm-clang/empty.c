/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define INCREMENT_TIME_X_TIMES        (100U)
#define APP_UART_BUFSIZE              (100U)
#define APP_UART_RECEIVE_BUFSIZE      (10U) /* format is "hh:mm:ss\r\n" */

#define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
        { \
            DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
        } \
    } while(0) \

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t validate_time();
void extract_time();

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t gUartBuffer[APP_UART_BUFSIZE];
uint8_t gUartReceiveBuffer[APP_UART_RECEIVE_BUFSIZE];
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;
static uint8_t intr_counter = 0;
static uint8_t hh = 0, mm = 0, ss = 0;
int32_t          transferOK;
UART_Transaction trans;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t validate_time()
{
    uint8_t status = SystemP_SUCCESS;

    if(hh > 24 || hh < 0)
    {
        status = SystemP_FAILURE;
    }
    if(mm > 60 || mm < 0)
    {
        status = SystemP_FAILURE;
    }
    if(ss > 60 || ss < 0)
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void extract_time()
{
    hh = (gUartReceiveBuffer[0]- '0')*10 + (gUartReceiveBuffer[1] - '0');
    mm = (gUartReceiveBuffer[3]- '0')*10 + (gUartReceiveBuffer[4] - '0');
    ss = (gUartReceiveBuffer[6]- '0')*10 + (gUartReceiveBuffer[7] - '0');
}

void empty_main(void *args)
{
    int32_t status = SystemP_FAILURE;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    UART_Transaction_init(&trans);

    /* Read time as input from User over UART */
    while(status != SystemP_SUCCESS) /* Keep reading until valid time is entered */
    {
        trans.buf = &gUartBuffer[0U];
        strncpy(trans.buf,"Enter time in hh:mm:ss format..\r\n", APP_UART_BUFSIZE);
        trans.count = strlen(trans.buf);
        transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
        APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

        /* Read 8 chars */
        trans.buf = &gUartReceiveBuffer[0U];
        trans.count = APP_UART_RECEIVE_BUFSIZE - 2; /* Subtracting "2" to adjust for \r\n characters */
        transferOK = UART_read(gUartHandle[CONFIG_UART_CONSOLE], &trans);
        gUartReceiveBuffer[8] = '\r';
        gUartReceiveBuffer[9] = '\n';
        APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

        extract_time();
        status = validate_time();
    }

    /* Echo back the time entered */
    trans.buf  = &gUartBuffer[0U];
    strncpy(trans.buf,"Starting clock, Time entered is: ", APP_UART_BUFSIZE);
    trans.count = strlen(trans.buf);
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    trans.buf  = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);

    /* Start the RTI counter */
    (void)RTI_counterEnable(CONFIG_RTI0_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0);

    while(intr_counter < INCREMENT_TIME_X_TIMES);

    /* Stop the RTI counter */
    (void)RTI_counterDisable(CONFIG_RTI0_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

void increment_time(void)
{
    ss++;
    if(ss == 60)
    {
        mm++;
        ss = 0;
    }
    if(mm == 60)
    {
        hh++;
        mm = 0;
    }
    if(hh ==24)
    {
        ss = 0;
        hh = 0;
        mm = 0;
    }

    gUartReceiveBuffer[0] = hh/10 + '0';
    gUartReceiveBuffer[1] = hh%10 + '0';
    gUartReceiveBuffer[2] = ':';
    gUartReceiveBuffer[3] = mm/10 + '0';
    gUartReceiveBuffer[4] = mm%10 + '0';
    gUartReceiveBuffer[5] = ':';
    gUartReceiveBuffer[6] = ss/10 + '0';
    gUartReceiveBuffer[7] = ss%10 + '0';

    trans.buf   = &gUartReceiveBuffer[0U];
    trans.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_write(gUartHandle[CONFIG_UART_CONSOLE], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    intr_counter++;
}
