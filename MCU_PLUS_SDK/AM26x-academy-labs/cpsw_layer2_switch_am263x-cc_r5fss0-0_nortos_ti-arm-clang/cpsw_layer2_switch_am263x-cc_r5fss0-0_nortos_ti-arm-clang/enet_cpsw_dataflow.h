#ifndef ENET_CPSW_DATAFLOW_H_
#define ENET_CPSW_DATAFLOW_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cpsw_common.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetApp_createClock(void);

void EnetApp_deleteClock(void);

void EnetApp_timerCallback(ClockP_Object *clkInst, void * arg);

void EnetApp_tickTask(void *args);

int32_t EnetApp_openDma(EnetApp_PerCtxt *perCtxt, uint32_t perCtxtIndex);

void EnetApp_closeDma(EnetApp_PerCtxt *perCtxt, uint32_t perCtxtIndex);

void EnetApp_initTxFreePktQ(void);

void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);

uint32_t EnetApp_retrieveFreeTxPkts(EnetApp_PerCtxt *perCtxt);

void EnetApp_createRxTask(EnetApp_PerCtxt *perCtxt);

void EnetApp_destroyRxTask(EnetApp_PerCtxt *perCtxt);

void EnetApp_rxTask(void *args);

#endif /* ENET_CPSW_DATAFLOW_H_ */
