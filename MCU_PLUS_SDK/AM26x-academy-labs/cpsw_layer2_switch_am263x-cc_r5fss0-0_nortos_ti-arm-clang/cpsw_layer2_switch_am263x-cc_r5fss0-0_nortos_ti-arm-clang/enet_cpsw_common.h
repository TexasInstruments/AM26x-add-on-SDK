#ifndef ENET_CPSW_COMMON_H_
#define ENET_CPSW_COMMON_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>

#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>

#include <networking/enet/utils/include/enet_apputils.h>
#include <networking/enet/utils/include/enet_appmemutils.h>
#include <networking/enet/utils/include/enet_appmemutils_cfg.h>

#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Task stack size */
#define ENETAPP_TASK_STACK_SZ                     (10U * 1024U)

/* 100-ms periodic tick */
#define ENETAPP_PERIODIC_TICK_MS                  (100U)

/*Counting Semaphore count*/
#define COUNTING_SEM_COUNT                       (10U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* Context of a peripheral/port */
typedef struct EnetApp_PerCtxt_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* Peripheral's MAC ports to use */
    Enet_MacPort  macPort[2];

    /* Number of MAC ports in macPorts array */
    uint32_t macPortNum;

    /* Name of this port to be used for logging */
    char *name;

    /* Enet driver handle for this peripheral type/instance */
    Enet_Handle hEnet;

    /* CPSW configuration */
    Cpsw_Cfg cpswCfg;

    /* MAC address. It's port's MAC address in Dual-MAC or
     * host port's MAC addres in Switch */
    uint8_t macAddr[ENET_MAC_ADDR_LEN];

    /* TX channel number */
    uint32_t txChNum;

    /* TX channel handle */
    EnetDma_TxChHandle hTxCh;

    /* Regular traffic RX channel number */
    uint32_t rxChNum;

    /* RX channel handle for regular traffic */
    EnetDma_RxChHandle hRxCh;

    /* RX task handle - receives Regular packets, changes source/dest MAC addresses
     * and transmits the packets back */
    TaskP_Object rxTaskObj;

    /* Semaphore posted from RX callback when Regular packets have arrived */
    SemaphoreP_Object rxSemObj;

    /* Semaphore used to synchronize all REgular RX tasks exits */
    SemaphoreP_Object rxDoneSemObj;

    /* Core key returned by Enet RM after attaching this core */
    uint32_t coreKey;
} EnetApp_PerCtxt;

typedef struct EnetApp_Obj_s
{
    /* Flag which indicates if test shall run */
    volatile bool run;

    /* This core's id */
    uint32_t coreId;

    /* Queue of free TX packets */
    EnetDma_PktQ txFreePktInfoQ;

    /* Periodic tick timer - used only to post a semaphore */
    ClockP_Object tickTimerObj;

    /* Periodic tick task - Enet period tick needs to be called from task context
     * hence it's called from this task (i.e. as opposed to in timer callback) */
    TaskP_Object tickTaskObj;

    /* Semaphore posted by tick timer to run tick task */
    SemaphoreP_Object timerSemObj;

    /* Array of all peripheral/port contexts used in the test */
    EnetApp_PerCtxt perCtxt[1];

    /* Number of active contexts being used */
    uint32_t numPerCtxts;
} EnetApp_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetApp_triggerReset(EnetApp_PerCtxt *perCtxt);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Enet l2 cpsw test object */
EnetApp_Obj gEnetApp;

/* Statistics */
CpswStats_PortStats gEnetApp_cpswStats;

/* Test application stack */
static uint8_t gEnetAppTaskStackTick[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));
static uint8_t gEnetAppTaskStackRx[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));

#endif /* ENET_CPSW_COMMON_H_ */
