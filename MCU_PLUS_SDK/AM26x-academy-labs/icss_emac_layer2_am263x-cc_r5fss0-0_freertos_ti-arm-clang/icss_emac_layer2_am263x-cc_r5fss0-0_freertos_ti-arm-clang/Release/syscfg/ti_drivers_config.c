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

#include "ti_drivers_config.h"

/*
 * I2C
 */
/* I2C atrributes */
static I2C_HwAttrs gI2cHwAttrs[CONFIG_I2C_NUM_INSTANCES] =
{
    {
        .baseAddr       = CSL_I2C0_U_BASE,
        .intNum         = 44,
        .eventId        = 0,
        .funcClk        = 96000000U,
        .enableIntr     = 1,
        .ownTargetAddr   = 0x1C,
    },
};
/* I2C objects - initialized by the driver */
static I2C_Object gI2cObjects[CONFIG_I2C_NUM_INSTANCES];
/* I2C driver configuration */
I2C_Config gI2cConfig[CONFIG_I2C_NUM_INSTANCES] =
{
    {
        .object = &gI2cObjects[CONFIG_I2C0],
        .hwAttrs = &gI2cHwAttrs[CONFIG_I2C0]
    },
};

uint32_t gI2cConfigNum = CONFIG_I2C_NUM_INSTANCES;

/*
 * PRUICSS
 */
/* PRUICSS HW attributes - provided by the driver */
extern PRUICSS_HwAttrs gPruIcssHwAttrs_ICSSM0;

/* PRUICSS objects - initialized by the driver */
static PRUICSS_Object gPruIcssObjects[CONFIG_PRUICSS_NUM_INSTANCES];
/* PRUICSS driver configuration */
PRUICSS_Config gPruIcssConfig[CONFIG_PRUICSS_NUM_INSTANCES] =
{
    {
        .object = &gPruIcssObjects[CONFIG_PRU_ICSS1],
        .hwAttrs = &gPruIcssHwAttrs_ICSSM0
    },
};

uint32_t gPruIcssConfigNum = CONFIG_PRUICSS_NUM_INSTANCES;


/*
 * ICSS EMAC
 */

/* ICSS EMAC Packet Buffers */
#define ICSS_EMAC_PKT_BUF_0_MEM_SIZE (65536)
uint8_t gIcssEmacPktBufMem0[ICSS_EMAC_PKT_BUF_0_MEM_SIZE] __attribute__((aligned(128), section(".bss.icss_emac_pktbuf_mem")));

/* ICSS EMAC atrributes */
static ICSS_EMAC_Attrs gIcssEmacAttrs[CONFIG_ICSS_EMAC_INSTANCES] =
{
    {
        .emacMode = (ICSS_EMAC_MODE_SWITCH | 0),
        .phyAddr = {3, 1},
        .phyToMacInterfaceMode = ICSS_EMAC_MII_MODE,
        .halfDuplexEnable = 0,
        .enableIntrPacing = ICSS_EMAC_DISABLE_PACING,
        .intrPacingMode = ICSS_EMAC_INTR_PACING_MODE1,
        .pacingThreshold = 100,
        .ethPrioQueue = ICSS_EMAC_QUEUE4,
        .learningEnable = ICSS_EMAC_LEARNING_ENABLE,
        .portMask = ICSS_EMAC_MODE_SWITCH,
        .txInterruptEnable = 0,
        .linkIntNum = 6,
        .rxIntNum = 0,
        .txIntNum = 2,
        .l3OcmcBaseAddr = (uint32_t)&gIcssEmacPktBufMem0[0],
        .l3OcmcSize = ICSS_EMAC_PKT_BUF_0_MEM_SIZE,
        .linkTaskPriority = 12,
        .rxTaskPriority = 10,
        .txTaskPriority = 10,
        .splitQueue = 0,
    },
};
/* ICSS EMAC objects - initialized by the driver */
static ICSS_EMAC_InternalObject gIcssEmacObjects[CONFIG_ICSS_EMAC_INSTANCES];
/* ICSS EMAC driver configuration */
ICSS_EMAC_Config gIcssEmacConfig[CONFIG_ICSS_EMAC_INSTANCES] =
{
    {
        &gIcssEmacObjects[CONFIG_ICSS_EMAC0],
        &gIcssEmacAttrs[CONFIG_ICSS_EMAC0],
    },
};
uint32_t gIcssEmacConfigNum = CONFIG_ICSS_EMAC_INSTANCES;

/*
 * UART
 */

/* UART atrributes */
static UART_Attrs gUartAttrs[CONFIG_UART_NUM_INSTANCES] =
{
        {
            .baseAddr           = CSL_UART0_U_BASE,
            .inputClkFreq       = 48000000U,
        },
};
/* UART objects - initialized by the driver */
static UART_Object gUartObjects[CONFIG_UART_NUM_INSTANCES];
/* UART driver configuration */
UART_Config gUartConfig[CONFIG_UART_NUM_INSTANCES] =
{
        {
            &gUartAttrs[CONFIG_UART0],
            &gUartObjects[CONFIG_UART0],
        },
};

uint32_t gUartConfigNum = CONFIG_UART_NUM_INSTANCES;

#include <drivers/uart/v0/lld/dma/uart_dma.h>
UART_DmaHandle gUartDmaHandle[] =
{
};

uint32_t gUartDmaConfigNum = CONFIG_UART_NUM_DMA_INSTANCES;

void Drivers_uartInit(void)
{
    UART_init();
}

void Pinmux_init(void);
void PowerClock_init(void);
void PowerClock_deinit(void);

/*
 * Common Functions
 */
void System_init(void)
{
    /* DPL init sets up address transalation unit, on some CPUs this is needed
     * to access SCICLIENT services, hence this needs to happen first
     */
    Dpl_init();

    
    /* initialize PMU */
    CycleCounterP_init(SOC_getSelfCpuClk());


    PowerClock_init();
    /* Now we can do pinmux */
    Pinmux_init();
    /* finally we initialize all peripheral drivers */
    I2C_init();
    PRUICSS_init();
    CSL_REG32_WR(CSL_ICSSM0_INTERNAL_U_BASE + CSL_ICSS_M_PR1_CFG_SLV_REGS_BASE + CSL_ICSS_M_PR1_CFG_SLV_MII_RT_REG , 1);

    ICSS_EMAC_init();
    Drivers_uartInit();
}

void System_deinit(void)
{
    I2C_deinit();
    PRUICSS_deinit();
    ICSS_EMAC_deinit();
    UART_deinit();
    PowerClock_deinit();
    Dpl_deinit();
}
