/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cpsw_common.h"
#include "enet_cpsw_cfg.h"
#include "enet_cpsw_dataflow.h"

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_mainTask(void *args)
{
    char option;
    int32_t status = ENET_SOK;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("==========================\r\n");
    DebugP_log(" Layer 2 CPSW SWITCH DEMO \r\n");
    DebugP_log("==========================\r\n");

    /* Initialize test config */
    memset(&gEnetApp, 0, sizeof(gEnetApp));

    /* Initialize test config */
    gEnetApp.run = true;
    gEnetApp.numPerCtxts = 1U;
    gEnetApp.perCtxt[0].enetType = ENET_CPSW_3G;
    gEnetApp.perCtxt[0].instId   = 0U;
    gEnetApp.perCtxt[0].name     = "CPSW-3G"; /* shallow copy */
    gEnetApp.perCtxt[0].macPortNum = 2U;
    gEnetApp.perCtxt[0].macPort[0]  = ENET_MAC_PORT_1;
    gEnetApp.perCtxt[0].macPort[1]  = ENET_MAC_PORT_2;
    gEnetApp.coreId = EnetSoc_getCoreId();

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetApp.txFreePktInfoQ);

    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to initialize l2 cpsw test: %d\r\n", status);
    }

    /* Open all peripherals */
    if (status == ENET_SOK)
    {
        status = EnetApp_open(gEnetApp.perCtxt, gEnetApp.numPerCtxts);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open peripherals: %d\r\n", status);
        }
    }

    EnetAppUtils_print("\nAllocated MAC addresses\r\n");
    EnetAppUtils_print("%s: \t", gEnetApp.perCtxt->name);
    EnetAppUtils_printMacAddr(&gEnetApp.perCtxt->macAddr[0U]);

    if (status == ENET_SOK)
    {

        EnetAppUtils_print("Enter x to stop the program: \r\n");
        while (true)
        {
            option = ' ';
            status = DebugP_scanf("%c", &option);
            if (option == 'x')
            {
                EnetAppUtils_print("Stopping...\r\n");
                gEnetApp.run = false;
                break;
            }
            else
            {
                EnetAppUtils_print("Invalid option, try again...\r\n");
            }
            TaskP_yield();
        }

        /* Print statistics */
        EnetApp_printStats(gEnetApp.perCtxt, gEnetApp.numPerCtxts);

        /* Wait until RX tasks have exited */
        EnetAppUtils_print("Waiting for RX task to exit\r\n");
        SemaphoreP_post(&gEnetApp.perCtxt[0].rxSemObj);
        SemaphoreP_pend(&gEnetApp.perCtxt[0].rxDoneSemObj, SystemP_WAIT_FOREVER);
    }

    /* Close all peripherals */
    EnetApp_close(gEnetApp.perCtxt, gEnetApp.numPerCtxts);

    Board_driversClose();
    Drivers_close();
}
