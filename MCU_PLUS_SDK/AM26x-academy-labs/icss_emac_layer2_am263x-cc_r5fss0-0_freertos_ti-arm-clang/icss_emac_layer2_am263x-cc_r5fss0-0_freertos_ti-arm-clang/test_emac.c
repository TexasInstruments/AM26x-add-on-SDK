/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/* lwIP core includes */
#include "ti_drivers_open_close.h"
#include "ti_drivers_config.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"

#include "lwip/opt.h"

#include <board/ethphy.h>
#include <board/eeprom.h>
#include <board/ethphy/ethphy_dp83826e.h>
#include <board/ethphy/ethphy_dp83869.h>
#ifdef SOC_AM263PX
#include <board/ioexp/ioexp_tca6424.h>
#endif

#include "test_icss_lwip.h"
#include "icss_emac.h"

#include <icss_emac_mmap.h>
#if ICSS_EMAC_MODE == ICSS_EMAC_MODE_SWITCH
#include <tiswitch_pruss_intc_mapping.h>
#else
#include <tiemac_pruicss_intc_mapping.h>
#endif

#ifdef AM263X_CC
#include <am263x-cc/pruicss_pinmux.h>
#elif AM263X_LP
#include <am263x-lp/pruicss_pinmux.h>
#elif AM263PX_CC
#include <am263px-cc/pruicss_pinmux.h>
#elif AM263PX_LP
#include <am263px-lp/pruicss_pinmux.h>
#endif


#include <drivers/hw_include/cslr_soc.h>
#ifdef SOC_AM263X
#include <drivers/hw_include/am263x/cslr_mss_ctrl.h>
#elif SOC_AM263PX
#include <drivers/hw_include/am263px/cslr_mss_ctrl.h>
#endif
#include <drivers/pinmux.h>
#include<drivers/pruicss/m_v0/pruicss.h>
#include <drivers/hw_include/hw_types.h>

#if ICSS_EMAC_MODE == ICSS_EMAC_MODE_SWITCH
#include <icss_switch/mii/PRU0_bin.h>
#include <icss_switch/mii/PRU1_bin.h>
#else
#include <icss_dual_emac/mii/PRU0_bin.h>
#include <icss_dual_emac/mii/PRU1_bin.h>
#endif

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SemaphoreP.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MSS_CTRL_ICSSM_PRU_GPIO_OUT_CTRL_VALUE  (0x0001077F)

#define UARTMENU_TASK_PRIORITY            (2)
#define UARTMENU_TASK_STACK_SIZE          (0x4000)

/*I2C Instance and Index for IO Expander programming*/
#define MDIO_MDC_MUX_SEL1                       (0x12)
#define IO_EXP_I2C_INSTANCE                     (0x01)

/*ICSS_EMAC Tx API Call Task*/
#define ICSS_EMAC_Tx_TASK_PRIORITY            (10)
#define ICSS_EMAC_Tx_TASK_STACK_SIZE          (0x4000)

uint32_t gtaskIcssEmacTxStack[ICSS_EMAC_Tx_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));

TaskP_Object taskUartMenuObject;
uint32_t gtaskUartMenuStack[UARTMENU_TASK_STACK_SIZE/sizeof(uint32_t)] __attribute__((aligned(32)));

TaskP_Object taskIcssEmacTxObject;
void recieve_packet(void *icssEmacHandle, void *queue_number, void *userArg);
void taskUartMenu(void * args);

#if defined AM263X_LP || defined AM263PX_LP
void icssmMuxSelection(void);
#endif

#ifdef AM263PX_CC
void setIOExpMuxSelection(void *args);
static TCA6424_Config  gTCA6424_Config;
#endif

#define ICSS_EMAC_MAXMTU  (1518U)
#define ICSS_EMAC_TEST_PKT_TX_COUNT 100

#define OCTETS_PER_ROW                  (16U)

/** \brief PRU-ICSS Handle */
PRUICSS_Handle pruicssHandle;

/** \brief ICSS EMAC Handle */
ICSS_EMAC_Handle icssemacHandle2;

#if defined SOC_AM263X || defined SOC_AM263PX
#define I2C_EEPROM_MAC0_DATA_OFFSET      (0x43)
#else
#define I2C_EEPROM_MAC0_DATA_OFFSET      (0x3D)
#endif

#define I2C_EEPROM_MAC1_DATA_OFFSET      (0x49)
uint8_t ICSS_EMAC_testLclMac0[6];

#define TEST_FRAME_SIZE 100
/**An HSR/PRP example frame with the header*/
uint8_t TestFrame[TEST_FRAME_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
                                      0x45, 0x00, 0x00, 0x2E, 0x00, 0x00, 0x40, 0x00, 0x40, 0x00, 0x3A, 0xD1 };
uint32_t gRxFrameCount;

/**Temporary placeholder to copy packets*/
uint8_t  tempRxFrame[ICSS_EMAC_MAXMTU]__attribute__((aligned(32)));


void print_cpu_load()
{
    static uint32_t start_time = 0;
    uint32_t print_interval_in_secs = 5;
    uint32_t cur_time = ClockP_getTimeUsec()/1000;

    if(start_time==0)
    {
        start_time = cur_time;
    }
    else
    if( (cur_time-start_time) >= (print_interval_in_secs*1000) )
    {
        uint32_t cpu_load = TaskP_loadGetTotalCpuLoad();

        DebugP_log(" %6d.%3ds : CPU load = %3d.%02d %%\r\n",
            cur_time/1000, cur_time%1000,
            cpu_load/100, cpu_load%100 );

        start_time = cur_time;

        TaskP_loadResetAll();
    }
}
void ICSS_EMAC_testBoardInit(void)
{
    ETHPHY_DP83869_LedSourceConfig ledConfig0;
    ETHPHY_DP83869_LedBlinkRateConfig ledBlinkConfig0;
    ETHPHY_DP83826E_LedSourceConfig ledConfig1;
    ETHPHY_DP83826E_LedBlinkRateConfig ledBlinkConfig1;

    Pinmux_config(gPruicssPinMuxCfg, PINMUX_DOMAIN_ID_MAIN);

    // Set bits for input pins in ICSSM_PRU0_GPIO_OUT_CTRL and ICSSM_PRU1_GPIO_OUT_CTRL registers
    HW_WR_REG32(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM_PRU0_GPIO_OUT_CTRL, MSS_CTRL_ICSSM_PRU_GPIO_OUT_CTRL_VALUE);
    HW_WR_REG32(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ICSSM_PRU1_GPIO_OUT_CTRL, MSS_CTRL_ICSSM_PRU_GPIO_OUT_CTRL_VALUE);

    DebugP_log("MII mode\r\n");

    /* PHY pin LED_0 as link */
    ledConfig0.ledNum = ETHPHY_DP83869_LED0;
    ledConfig0.mode = ETHPHY_DP83869_LED_MODE_100BTX_LINK_UP;
    ledConfig1.ledNum = ETHPHY_DP83826E_LED0;
    ledConfig1.mode = ETHPHY_DP83826E_LED_MODE_MII_LINK_100BT_FD;
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig0, sizeof(ledConfig0));
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig1, sizeof(ledConfig1));

    /* PHY pin LED_1 indication is on if 1G link established for PHY0, and if 10M speed id configured for PHY1 */
    ledConfig0.ledNum = ETHPHY_DP83869_LED1;
    ledConfig0.mode = ETHPHY_DP83869_LED_MODE_1000BT_LINK_UP;
    ledConfig1.ledNum = ETHPHY_DP83826E_LED1;
    ledConfig1.mode = ETHPHY_DP83826E_LED_MODE_SPEED_10BT;
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig0, sizeof(ledConfig0));
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig1, sizeof(ledConfig1));

    /* PHY pin LED_2 as Rx/Tx Activity */
    ledConfig0.ledNum = ETHPHY_DP83869_LED2;
    ledConfig0.mode = ETHPHY_DP83869_LED_MODE_LINK_OK_AND_BLINK_ON_RX_TX;
    ledConfig1.ledNum = ETHPHY_DP83826E_LED2;
    ledConfig1.mode = ETHPHY_DP83826E_LED_MODE_LINK_OK_AND_BLINK_ON_RX_TX;
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig0, sizeof(ledConfig0));
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_CONFIGURE_LED_SOURCE, (void *)&ledConfig1, sizeof(ledConfig1));

    ledBlinkConfig0.rate = ETHPHY_DP83869_LED_BLINK_RATE_200_MS;
    ledBlinkConfig1.rate = ETHPHY_DP83826E_LED_BLINK_RATE_200_MS;
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_CONFIGURE_LED_BLINK_RATE, (void *)&ledBlinkConfig0, sizeof(ledBlinkConfig0));
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY1], ETHPHY_CMD_CONFIGURE_LED_BLINK_RATE, (void *)&ledBlinkConfig1, sizeof(ledBlinkConfig1));

    /* Enable MII mode for DP83869 PHY */
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_ENABLE_MII, NULL, 0);

    /* Disable 1G advertisement and sof-reset to restart auto-negotiation in case 1G link was establised */
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_DISABLE_1000M_ADVERTISEMENT, NULL, 0);
    ETHPHY_command(gEthPhyHandle[CONFIG_ETHPHY0], ETHPHY_CMD_SOFT_RESTART, NULL, 0);

    /*Wait for PHY to come out of reset*/
    ClockP_sleep(1);
}

int32_t ICSS_EMAC_ConfigInit(void)
{
    PRUICSS_IntcInitData    pruss_intc_initdata = PRUSS_INTC_INITDATA;
    ICSS_EMAC_Params        icssEmacParams;

    /*PRU2 ETH0 initializations*/
    ICSS_EMAC_Params_init(&icssEmacParams);
    icssEmacParams.pruicssIntcInitData = &pruss_intc_initdata;
    icssEmacParams.fwStaticMMap = &(icss_emacFwStaticCfg[1]);
    icssEmacParams.fwDynamicMMap = &icss_emacFwDynamicCfg;
    icssEmacParams.fwVlanFilterParams = &icss_emacFwVlanFilterCfg;
    icssEmacParams.fwMulticastFilterParams = &icss_emacFwMulticastFilterCfg;
    icssEmacParams.pruicssHandle = pruicssHandle;
    icssEmacParams.callBackObject.rxNRTCallBack.callBack = (ICSS_EMAC_CallBack)recieve_packet;
    icssEmacParams.ethphyHandle[0] = gEthPhyHandle[CONFIG_ETHPHY0];
#if ICSS_EMAC_MODE == ICSS_EMAC_MODE_SWITCH
    icssEmacParams.ethphyHandle[1] = gEthPhyHandle[CONFIG_ETHPHY1];
#endif
    memcpy(&(icssEmacParams.macId[0]), &(ICSS_EMAC_testLclMac0[0]), 6);

    icssemacHandle2 = ICSS_EMAC_open(CONFIG_ICSS_EMAC0, &icssEmacParams);
    DebugP_assert(icssemacHandle2 != NULL);


    return 0;
}

void ICSS_EMAC_PRUFirmwareLoad(PRUICSS_Handle pruHandle)
{
        uint32_t icssBaseAddr;
        int32_t status;
        int32_t retVal;

    /* Setup RAT configuration for buffer region*/
        /* Setting up RAT config to map emacBaseAddr->l3OcmcBaseAddr to C30 constant of PRUICSS */
        /* Mapping 0xE0000000 (C30 constant of PRUICSS) to l3OcmcBaseAddr */
        icssBaseAddr = (uint32_t)((PRUICSS_HwAttrs *)(pruHandle->hwAttrs)->baseAddr);

        HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x24, (0xE0000000));         /*rat0 base0 */
        HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x28, (0x70000000));         /*rat0 trans_low0 */
        HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x2C, (0x00000000));         /*rat0 trans_low0 */
        HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_0_BASE + 0x20, (1u << 31) | (22));    /*rat0 ctrl0 */

        HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x24, (0xE0000000));         /*rat0 base0 */
        HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x28, (0x70000000));         /*rat0 trans_low0 */
        HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x2C, (0x00000000));         /*rat0 trans_low0 */
        HW_WR_REG32(icssBaseAddr + CSL_ICSS_RAT_REGS_1_BASE + 0x20, (1u << 31) | (22));    /*rat0 ctrl0 */

        PRUICSS_disableCore(pruHandle, PRUICSS_PRU0);
        PRUICSS_disableCore(pruHandle, PRUICSS_PRU1);

        status = PRUICSS_writeMemory(pruHandle, PRUICSS_IRAM_PRU(0), 0, (uint32_t *)PRU0_b00, sizeof(PRU0_b00));
        if(status)
        {
            DebugP_log("load to PRU0 passed\r\n");
            retVal = true;
        }
        else
        {
            DebugP_log("load to PRU0 failed\r\n");
        }
        status = PRUICSS_writeMemory(pruHandle, PRUICSS_IRAM_PRU(1), 0, (uint32_t *)PRU1_b00, sizeof(PRU1_b00));
        if(status)
        {
            DebugP_log("load to PRU1 passed\r\n");
            retVal = true;
        }
        else
        {
            DebugP_log("load to PRU0 failed\r\n");
        }

        if( retVal)
        {
            PRUICSS_enableCore(pruHandle, PRUICSS_PRU0);
            PRUICSS_enableCore(pruHandle, PRUICSS_PRU1);
        }
}

void Icss_socgetMACAddress()
{
    uint32_t status = SystemP_FAILURE;
    status = EEPROM_read(gEepromHandle[CONFIG_EEPROM0], I2C_EEPROM_MAC0_DATA_OFFSET, ICSS_EMAC_testLclMac0, 6U);
    DebugP_assert(SystemP_SUCCESS == status);
}

int icss_example(void *args)
{
    uint32_t                status = SystemP_FAILURE;

    Drivers_open();
    status = Board_driversOpen();
    DebugP_assert(status==SystemP_SUCCESS);

#if defined AM263X_LP || defined AM263PX_LP
    icssmMuxSelection();
#else
    ICSS_EMAC_testBoardInit();
#endif

#ifdef AM263PX_CC
    setIOExpMuxSelection(NULL);
#endif

    pruicssHandle = PRUICSS_open(CONFIG_PRU_ICSS1);
    DebugP_assert(pruicssHandle != NULL);

    /*Setup the local MAC Addresses of Port from EEPROM*/
    Icss_socgetMACAddress();

    ICSS_EMAC_ConfigInit();

    ICSS_EMAC_PRUFirmwareLoad(pruicssHandle);
#if defined AM263X_LP || defined AM263PX_LP
    ICSS_EMAC_testBoardInit();
#endif
       TaskP_Params taskParams;
       TaskP_Params_init(&taskParams);
       taskParams.priority = UARTMENU_TASK_PRIORITY;
       taskParams.name = "UARTMenuTask";
       taskParams.stackSize = UARTMENU_TASK_STACK_SIZE;
       taskParams.stack = (uint8_t *) gtaskUartMenuStack;
       taskParams.args = icssemacHandle2;
       taskParams.taskMain = (TaskP_FxnMain)taskUartMenu;
       status = TaskP_construct(&taskUartMenuObject, &taskParams);

       if(status != SystemP_SUCCESS)
       {
           DebugP_log("UARTMenuTask Creation failed\r\n");
       }
    // main_loop(NULL);

    return 0;
}

#if defined AM263X_LP || defined AM263PX_LP
void icssmMuxSelection(void)
{
    uint32_t pinNum[CONFIG_GPIO_NUM_INSTANCES] = {CONFIG_GPIO0_PIN, CONFIG_GPIO1_PIN, CONFIG_GPIO2_PIN};
    uint32_t pinDir[CONFIG_GPIO_NUM_INSTANCES] = {CONFIG_GPIO0_DIR, CONFIG_GPIO1_DIR, CONFIG_GPIO2_DIR};

    for(uint32_t index = 0; index < CONFIG_GPIO_NUM_INSTANCES-1; index++)
    {
        /* Address translate */
        uint32_t gGpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO0_BASE_ADDR);

        /* Setup GPIO for ICSSM MDIO Mux selection */
        GPIO_setDirMode(gGpioBaseAddr, pinNum[index], pinDir[index]);
        GPIO_pinWriteHigh(gGpioBaseAddr, pinNum[index]);
    }
}
#endif

#ifdef AM263PX_CC
/* Set MDIO/MDC_MUX_SEL1 to low using IO Expander to configure:
 *  On-Board PHY               ->  PRU0 MII0
 *  ETHERNET ADD-ON CONNECTOR  ->  PRU1 MII1
 *
 * Refer to Ethernet Routing in AM263Px User Guide for more details
 */
void setIOExpMuxSelection(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            ioIndex = MDIO_MDC_MUX_SEL1;
    TCA6424_Params      tca6424Params;

    TCA6424_Params_init(&tca6424Params);

    tca6424Params.i2cInstance = IO_EXP_I2C_INSTANCE;

    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);

    if(status == SystemP_SUCCESS)
    {
        /* Configure as output  */
        status = TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);

        /* set P22 low which controls MDIO/MDC_MUX_SEL1 -> enable PRU0_MII0 and PRU1_MII1 */
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_LOW);

    }
    TCA6424_close(&gTCA6424_Config);
}
#endif

void recieve_packet(void *icssEmacHandle, void *queue_number, void *userArg)
{

    ICSS_EMAC_RxArgument rxArg;
    int32_t size;

    ICSS_EMAC_Handle IcssEmacHandle = icssemacHandle2;

    rxArg.icssEmacHandle = IcssEmacHandle;
    rxArg.destAddress = (uint32_t)(tempRxFrame);
    rxArg.more = 0;
    rxArg.queueNumber = (uint32_t)queue_number;
    rxArg.port = 0;

    size = (uint16_t)ICSS_EMAC_rxPktGet(&rxArg, &userArg);

    if(size!=SystemP_FAILURE)
    {
        gRxFrameCount++;
    }
}
uint32_t transmit_packet(ICSS_EMAC_Handle emachandle)
{
    uint32_t status = 0;
    ICSS_EMAC_TxArgument txArg;

    txArg.icssEmacHandle = emachandle;
    txArg.srcAddress = TestFrame;
    txArg.lengthOfPacket = (int32_t)TEST_FRAME_SIZE;
    txArg.queuePriority = ICSS_EMAC_QUEUE1;
    /*Port number will decide on which port packet will be transmitted*/
    txArg.portNumber = 0;

    status = ICSS_EMAC_txPacket(&txArg, NULL);

    return status;

}

void transmit_nPacket(int n)
{
   uint16_t tx_count = 0;
   uint32_t status;


    while(1)
    {
        if(tx_count < n)
        {
            status = transmit_packet(icssemacHandle2);
            if(status != 0)
            {
                DebugP_log("\n\rFailed to transmit for tx_count = %u", tx_count);
            }
            else
            {
                DebugP_log("\n\rDevice transmitted TestFrame packet for tx_count = %u", tx_count);
            }
            tx_count++;

           ClockP_usleep(ClockP_ticksToUsec(500));
        }

        if(tx_count >= n)
        {
           DebugP_log("\n\rDevice transmitted TestFrame packets...completed");
           break;
        }
    }
}

void printFrame(uint8_t *frame, uint32_t size)
{
    uint32_t i;
    DebugP_log("\r\n           ");
    for (i = 0; i < size; i++)
    {
        DebugP_log("0x%02x ", frame[i]);
        if (i && (((i + 1) % OCTETS_PER_ROW) == 0))
        {
            DebugP_log("\r\n           ");
        }
    }

    if (size && ((size % OCTETS_PER_ROW) != 0))
    {
        DebugP_log("\r\n");
    }

    DebugP_log("\r\n");
}

void printHelpMenu()
{
    DebugP_log("\n\rICSS EMAC Application Menu Options. Press key (Upper/Lower)\n\r");
    DebugP_log("*******************************************\n\r");
    DebugP_log("T : Transmit Packet\n\r");
    DebugP_log("C : Print CPU Load\n\r");
    DebugP_log("R : Print Statistics\n\r");
    DebugP_log("X : Clear Statistics\n\r");
    DebugP_log("Z : Print last received frame \n\r");
    DebugP_log("\n\r********************************************\n\r");
}
/*
 *  ---UART Menu task---
 */
void clearStatistics()
{
    int32_t status;
    ICSS_EMAC_IoctlCmd ioctlParams;
    ioctlParams.command = ICSS_EMAC_IOCTL_STATS_CTRL;
    status = ICSS_EMAC_ioctl(icssemacHandle2, ICSS_EMAC_IOCTL_STAT_CTRL_CLEAR,
                   ICSS_EMAC_PORT_1, (void *)&ioctlParams);
    DebugP_log("\n\rStatistics cleared for port 1: %d\n\r", status);
    status = ICSS_EMAC_ioctl(icssemacHandle2, ICSS_EMAC_IOCTL_STAT_CTRL_CLEAR,
                   ICSS_EMAC_PORT_2, (void *)&ioctlParams);
    DebugP_log("\n\rStatistics cleared for port 2: %d\n\r", status);
}
void printStatistics(uint8_t port)
{
   // int32_t status;
    ICSS_EMAC_IoctlCmd ioctlParams;
    ICSS_EMAC_PruStatistics stats;
    ioctlParams.command = ICSS_EMAC_IOCTL_STATS_CTRL;
    if(port == 1)
    {
        ICSS_EMAC_ioctl(icssemacHandle2, ICSS_EMAC_IOCTL_STAT_CTRL_GET,
                       ICSS_EMAC_PORT_1, (void *)&stats);
        DebugP_log("\n\rStatistics for Port 1:\n\r");
    }
    else
    {
        ICSS_EMAC_ioctl(icssemacHandle2, ICSS_EMAC_IOCTL_STAT_CTRL_GET,
                       ICSS_EMAC_PORT_2, (void *)&stats);
        DebugP_log("\n\rStatistics for Port 2 :\n\r");
    }


        DebugP_log("*******************************************\n\r");
        DebugP_log("txBcast: %d                  \n\r", stats.txBcast);/**Number of broadcast packets sent*/
        DebugP_log("txMcast: %d                  \n\r", stats.txMcast);/**Number of multicast packets sent*/
        DebugP_log("txUcast: %d                  \n\r", stats.txUcast);/**Number of unicast packets sent*/
        DebugP_log("txOctets: %d                 \n\r", stats.txOctets);/**Number of Tx packets*/

        DebugP_log("rxBcast: %d                  \n\r", stats.rxBcast);/**Number of broadcast packets rcvd*/
        DebugP_log("rxMcast: %d                  \n\r", stats.rxMcast);/**Number of multicast packets rcvd*/
        DebugP_log("rxUcast: %d                  \n\r", stats.rxUcast);/**Number of unicast packets rcvd*/
        DebugP_log("rxOctets: %d                 \n\r", stats.rxOctets);/**Number of Rx packets*/

        DebugP_log("tx64byte: %d                 \n\r", stats.tx64byte);/**Number of 64 byte packets sent*/
        DebugP_log("tx65_127byte: %d             \n\r", stats.tx65_127byte);/**Number of 65-127 byte packets sent*/
        DebugP_log("tx128_255byte: %d            \n\r", stats.tx128_255byte);/**Number of 128-255 byte packets sent*/
        DebugP_log("tx256_511byte: %d            \n\r", stats.tx256_511byte);/**Number of 256-511 byte packets sent*/
        DebugP_log("tx512_1023byte: %d           \n\r", stats.tx512_1023byte);/**Number of 512-1023 byte packets sent*/
        DebugP_log("tx1024byte: %d               \n\r", stats.tx1024byte);/**Number of 1024 and larger size packets sent*/

        DebugP_log("rx64byte: %d                 \n\r", stats.rx64byte);/**Number of 64 byte packets rcvd*/
        DebugP_log("rx65_127byte: %d             \n\r", stats.rx65_127byte);/**Number of 65-127 byte packets rcvd*/
        DebugP_log("rx128_255byte: %d            \n\r", stats.rx128_255byte);/**Number of 128-255 byte packets rcvd*/
        DebugP_log("rx256_511byte: %d            \n\r", stats.rx256_511byte);/**Number of 256-511 byte packets rcvd*/
        DebugP_log("rx512_1023byte: %d           \n\r", stats.rx512_1023byte);/**Number of 512-1023 byte packets rcvd*/
        DebugP_log("rx1024byte: %d               \n\r", stats.rx1024byte);/**Number of 1024 and larger size packets rcvd*/

        DebugP_log("lateColl: %d                 \n\r", stats.lateColl);/**Number of late collisions(Half Duplexstats.)*/
        DebugP_log("singleColl: %d               \n\r", stats.singleColl);/**Number of single collisions (Half Duplexstats.)*/
        DebugP_log("multiColl: %d                \n\r", stats.multiColl);/**Number of multiple collisions (Half Duplexstats.)*/
        DebugP_log("excessColl: %d               \n\r", stats.excessColl);/**Number of excess collisions(Half Duplexstats.)*/

        DebugP_log("rxMisAlignmentFrames: %d     \n\r", stats.rxMisAlignmentFrames);/**Number of non multiple of 8 byte frames rcvd*/
        DebugP_log("stormPrevCounter: %d         \n\r", stats.stormPrevCounter);/**Number of packets dropped because of Storm Prevention (broadcaststats.)*/
        DebugP_log("stormPrevCounterMC: %d       \n\r", stats.stormPrevCounterMC);  /**Number of packets dropped because of Storm Prevention (multicaststats.)*/
        DebugP_log("stormPrevCounterUC: %d       \n\r", stats.stormPrevCounterUC);  /**Number of packets dropped because of Storm Prevention (unicaststats.)*/
        DebugP_log("macRxError: %d               \n\r", stats.macRxError);/**Number of MAC receive errors*/

        DebugP_log("SFDError: %d                 \n\r", stats.SFDError); /**Number of invalid SFD*/
        DebugP_log("defTx: %d                    \n\r", stats.defTx);/**Number of transmissions deferred*/
        DebugP_log("macTxError: %d               \n\r", stats.macTxError);/**Number of MAC transmit errors*/
        DebugP_log("rxOverSizedFrames: %d        \n\r", stats.rxOverSizedFrames);/**Number of oversized frames rcvd*/
        DebugP_log("rxUnderSizedFrames: %d       \n\r", stats.rxUnderSizedFrames);/**Number of undersized frames rcvd*/
        DebugP_log("rxCRCFrames: %d              \n\r", stats.rxCRCFrames);/**Number of CRC error frames rcvd*/

        DebugP_log("droppedPackets: %d \n\r", stats.droppedPackets);
}

void taskUartMenu(void * args)
{
    char rxByte = 0;
    uint32_t num_pkts = 0;
    uint8_t port;

    /* Wait for PRU load complete */
    ClockP_usleep(ClockP_ticksToUsec(3000));

    while(1)
    {
        DebugP_log("\n\r\n\r");
        printHelpMenu();

        DebugP_scanf("%c", &rxByte);

        if((rxByte == 'T') || (rxByte == 't'))
        {
            DebugP_log("\n\rEnter number of packets:\n\r");
            DebugP_scanf("%d", &num_pkts);
            transmit_nPacket(num_pkts);
        }
        else if((rxByte == 'C') || (rxByte == 'c'))
        {
            print_cpu_load();
        }
        else if((rxByte == 'R') || (rxByte == 'r'))
        {
            DebugP_log("\n\rSelect Port(1 or 2):\n\r");
            DebugP_scanf("%d", &port);
            if(port == 1 || port == 2)
            {
                printStatistics(port);
            }
            else
            {
                DebugP_log("\n\rInvalid Input!! Try again\n\r");
            }
        }
        else if((rxByte == 'X') || (rxByte == 'x'))
        {
           clearStatistics();
        }
        else if((rxByte == 'Z') || (rxByte == 'z'))
        {
            if(gRxFrameCount)
            {
                   printFrame((uint8_t*)tempRxFrame, TEST_FRAME_SIZE);
            }
            else
            {
                   DebugP_log("\n\rNo frames received!!!\n\r");
            }
        }
        else
        {
            DebugP_log("\n\rInvalid Input!! Try again\n\r");
        }
    }
}
