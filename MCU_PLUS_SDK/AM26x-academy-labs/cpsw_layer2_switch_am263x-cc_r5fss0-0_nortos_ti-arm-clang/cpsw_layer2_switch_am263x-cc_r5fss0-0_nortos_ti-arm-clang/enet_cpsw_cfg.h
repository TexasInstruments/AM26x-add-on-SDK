#ifndef ENET_CPSW_CFG_H_
#define ENET_CPSW_CFG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cpsw_common.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t EnetApp_open(EnetApp_PerCtxt *perCtxts,
                           uint32_t numPerCtxts);

void EnetApp_close(EnetApp_PerCtxt *perCtxts,
                         uint32_t numPerCtxts);

int32_t EnetApp_openPort(EnetApp_PerCtxt *perCtxt);

void EnetApp_closePort(EnetApp_PerCtxt *perCtxt);

void EnetApp_printStats(EnetApp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts);

int32_t EnetApp_waitForLinkUp(EnetApp_PerCtxt *perCtxt);

#endif /* ENET_CPSW_CFG_H_ */
