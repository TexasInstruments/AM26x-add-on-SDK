/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cpsw_cfg.h"
#include "enet_cpsw_dataflow.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static EnetApp_PerCtxt * EnetApp_getPerCtxt(Enet_Type enetType,
                                            uint32_t instId);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                          bool isLinkUp,
                                          void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                             void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                info->phyAddr,
                info->isLinked? "up" : "down");
    }
}

void EnetApp_initEnetLinkCbPrms(Cpsw_Cfg *cpswCfg)
{
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb     = NULL;
    cpswCfg->mdioLinkStateChangeCbArg  = NULL;
#else
    cpswCfg->mdioLinkStateChangeCb     = EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg  = &gEnetApp;
#endif

    cpswCfg->portLinkStatusChangeCb    = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = &gEnetApp;
}

void EnetApp_initAleConfig(CpswAle_Cfg *aleCfg)
{
    aleCfg->modeFlags = CPSW_ALE_CFG_MODULE_EN;
    aleCfg->agingCfg.autoAgingEn = true;
    aleCfg->agingCfg.agingPeriodInMs = 1000;

    aleCfg->nwSecCfg.vid0ModeEn                = true;
    aleCfg->vlanCfg.aleVlanAwareMode           = FALSE;
    aleCfg->vlanCfg.cpswVlanAwareMode          = FALSE;
    aleCfg->vlanCfg.unknownUnregMcastFloodMask = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownRegMcastFloodMask   = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->vlanCfg.unknownVlanMemberListMask  = CPSW_ALE_ALL_PORTS_MASK;
    aleCfg->policerGlobalCfg.policingEn        = true;
    aleCfg->policerGlobalCfg.yellowDropEn      = false;
    /* Enables the ALE to drop the red colored packets. */
    aleCfg->policerGlobalCfg.redDropEn         = true;
    /* Policing match mode */
    aleCfg->policerGlobalCfg.policerNoMatchMode = CPSW_ALE_POLICER_NOMATCH_MODE_GREEN;
}

void EnetApp_updateCpswInitCfg(Enet_Type enetType,  uint32_t instId,   Cpsw_Cfg *cpswCfg)
{
    EnetApp_PerCtxt *perCtxt = EnetApp_getPerCtxt(enetType, instId);

    EnetAppUtils_assert(perCtxt != NULL);
    /* Prepare init configuration for all peripherals */
    EnetAppUtils_print("\nInit all configs\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    EnetAppUtils_print("%s: init config\r\n", perCtxt->name);

    cpswCfg->vlanCfg.vlanAware          = false;
    cpswCfg->hostPortCfg.removeCrc      = true;
    cpswCfg->hostPortCfg.padShortPacket = true;
    cpswCfg->hostPortCfg.passCrcErrors  = true;
    EnetApp_initEnetLinkCbPrms(cpswCfg);
    EnetApp_initAleConfig(&cpswCfg->aleCfg);

}

int32_t EnetApp_open(EnetApp_PerCtxt *perCtxts,
                           uint32_t numPerCtxts)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nInit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_enableClocks(perCtxt->enetType, perCtxt->instId);
    }
        /* Create RX tasks for each peripheral */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\nCreate RX tasks\r\n");
        EnetAppUtils_print("----------------------------------------------\r\n");
        for (i = 0U; i < numPerCtxts; i++)
        {
            EnetApp_PerCtxt *perCtxt = &perCtxts[i];

            EnetAppUtils_print("%s: Create RX task\r\n", perCtxt->name);

            EnetApp_createRxTask(perCtxt);
        }
    }

    /* Open Enet driver for all peripherals */
    EnetAppUtils_print("\nOpen all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");

    EnetApp_driverInit();

    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_PerCtxt *perCtxt = &perCtxts[i];
        EnetApp_HandleInfo handleInfo;

        EnetAppUtils_print("%s: Open enet\r\n", perCtxt->name);
        status = EnetApp_driverOpen(perCtxt->enetType, perCtxt->instId);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: failed to open enet\r\n", perCtxt->name);
            break;
        }
        EnetApp_acquireHandleInfo(perCtxt->enetType, perCtxt->instId, &handleInfo);
        perCtxt->hEnet = handleInfo.hEnet;
    }

    /* Start PHY tick timer */
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\nAttach core id %u on all peripherals\r\n", gEnetApp.coreId);
        EnetAppUtils_print("----------------------------------------------\r\n");
        for (i = 0U; i < numPerCtxts; i++)
        {
            EnetApp_PerCtxt *perCtxt = &perCtxts[i];
            EnetPer_AttachCoreOutArgs attachCoreOutArgs;

            EnetAppUtils_print("%s: Attach core\r\n", perCtxt->name);

            EnetApp_coreAttach(perCtxt->enetType, perCtxt->instId, gEnetApp.coreId, &attachCoreOutArgs);
            perCtxt->coreKey = attachCoreOutArgs.coreKey;
        }
    }


    /* Open DMA for peripheral/port */
    if (status == ENET_SOK)
    {
        for (i = 0U; i < numPerCtxts; i++)
        {
            EnetApp_PerCtxt *perCtxt = &perCtxts[i];

            EnetAppUtils_print("%s: Open DMA\r\n", perCtxt->name);
            status = EnetApp_openDma(perCtxt, i);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: failed to open DMA: %d\r\n", perCtxt->name, status);
            }
        }
    }

    if (status == ENET_SOK)
    {
        status = EnetApp_waitForLinkUp(perCtxts);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to wait for link up: %d\r\n", perCtxts->name, status);
        }
    }

    EnetAppUtils_print("%s: MAC port addr: ", perCtxts->name);

    EnetAppUtils_printMacAddr(&perCtxts->macAddr[0U]);

    return status;
}

static int32_t EnetApp_getPerIdx(Enet_Type enetType, uint32_t instId, uint32_t *perIdx)
{
    uint32_t i;
    int32_t status = ENET_SOK;

    /* Initialize async IOCTL and TX timestamp semaphores */
    for (i = 0U; i < gEnetApp.numPerCtxts; i++)
    {
        EnetApp_PerCtxt *perCtxt = &(gEnetApp.perCtxt[i]);
        if ((perCtxt->enetType == enetType) && (perCtxt->instId == instId))
        {
            break;
        }
    }
    if (i < gEnetApp.numPerCtxts)
    {
        *perIdx = i;
        status = ENET_SOK;
    }
    else
    {
        status = ENET_ENOTFOUND;
    }
    return status;
}

static EnetApp_PerCtxt * EnetApp_getPerCtxt(Enet_Type enetType,
                                            uint32_t instId)
{
    uint32_t perIdx;
    int32_t status;

    status = EnetApp_getPerIdx(enetType, instId, &perIdx);
    EnetAppUtils_assert(status == ENET_SOK);
    EnetAppUtils_assert(perIdx < ENET_ARRAYSIZE(gEnetApp.perCtxt));
    return (&gEnetApp.perCtxt[perIdx]);
}

void EnetApp_close(EnetApp_PerCtxt *perCtxts,
                         uint32_t numPerCtxts)
{
    uint32_t i;

    EnetAppUtils_print("\nClose Ports for all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_PerCtxt *perCtxt = &perCtxts[i];

        EnetAppUtils_print("%s: Close Port\r\n", perCtxt->name);

        EnetApp_closePort(perCtxt);
    }

    EnetAppUtils_print("\nClose DMA for all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_PerCtxt *perCtxt = &perCtxts[i];

        EnetAppUtils_print("%s: Close DMA\r\n", perCtxt->name);

        EnetApp_closeDma(perCtxt, i);
    }

    /* Delete RX tasks created for all peripherals */
    EnetAppUtils_print("\nDelete RX tasks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_destroyRxTask(&perCtxts[i]);
    }

    /* Detach core */
    EnetAppUtils_print("\nDetach core from all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_PerCtxt *perCtxt = &perCtxts[i];

        EnetAppUtils_print("%s: Detach core\r\n", perCtxt->name);

        EnetApp_coreDetach(perCtxt->enetType, perCtxt->instId,
                            gEnetApp.coreId,
                            perCtxt->coreKey);
    }
    /* Close opened Enet drivers if any peripheral failed */
    EnetAppUtils_print("\nClose all peripherals\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_print("%s: Close enet\r\n", perCtxt->name);
        EnetApp_releaseHandleInfo(perCtxt->enetType, perCtxt->instId);
        perCtxt->hEnet = NULL;
    }

    /* Do peripheral dependent initalization */
    EnetAppUtils_print("\nDeinit all peripheral clocks\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_PerCtxt *perCtxt = &perCtxts[i];
        EnetAppUtils_disableClocks(perCtxt->enetType, perCtxt->instId);
    }
}

void EnetApp_printStats(EnetApp_PerCtxt *perCtxts,
                              uint32_t numPerCtxts)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i,j;
    int32_t status;

    EnetAppUtils_print("\nPrint statistics\r\n");
    EnetAppUtils_print("----------------------------------------------\r\n");
    for (i = 0U; i < numPerCtxts; i++)
    {
        EnetApp_PerCtxt *perCtxt = &gEnetApp.perCtxt[i];

        for (j = 0U; j < perCtxt->macPortNum; j++)
        {
            macPort = perCtxt->macPort[j];
            ENET_IOCTL_SET_OUT_ARGS(&prms, &gEnetApp_cpswStats);

            ENET_IOCTL(perCtxt->hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to get port stats\r\n", perCtxt->name);
                continue;
            }
            EnetAppUtils_printHostPortStats9G((CpswStats_HostPort_Ng *)&gEnetApp_cpswStats);

            EnetAppUtils_print("\n %s - Port %u statistics\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
            EnetAppUtils_print("--------------------------------\r\n");

            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &gEnetApp_cpswStats);

            ENET_IOCTL(perCtxt->hEnet, gEnetApp.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to get port %u stats\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
                continue;
            }

            EnetAppUtils_printMacPortStats9G((CpswStats_MacPort_Ng *)&gEnetApp_cpswStats);

            EnetAppUtils_print("\n");
        }

    }
}

void EnetApp_initLinkArgs(Enet_Type enetType,
                          uint32_t instId,
                          EnetPer_PortLinkCfg *linkArgs,
                          Enet_MacPort macPort)
{
    EnetBoard_EthPort ethPort;
    const EnetBoard_PhyCfg *boardPhyCfg;
    EnetMacPort_LinkCfg *linkCfg = &linkArgs->linkCfg;
    EnetMacPort_Interface *mii = &linkArgs->mii;
    EnetPhy_Cfg *phyCfg = &linkArgs->phyCfg;
    int32_t status = ENET_SOK;
    EnetApp_PerCtxt *perCtxt;

    perCtxt = EnetApp_getPerCtxt(enetType, instId);

    EnetAppUtils_print("%s: Open port %u\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));

    /* Setup board for requested Ethernet port */
    ethPort.enetType = perCtxt->enetType;
    ethPort.instId   = perCtxt->instId;
    ethPort.macPort  = macPort;
    ethPort.boardId  = EnetBoard_getId();
    ethPort.mii.layerType    = ENET_MAC_LAYER_GMII;
    ethPort.mii.sublayerType = ENET_MAC_SUBLAYER_REDUCED;
    ethPort.mii.variantType  = ENET_MAC_VARIANT_FORCED;

    status = EnetBoard_setupPorts(&ethPort, 1U);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("%s: Failed to setup MAC port %u\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
        EnetAppUtils_assert(false);
    }

    /* Set port link params */
    linkArgs->macPort = macPort;

    mii->layerType     = ethPort.mii.layerType;
    mii->sublayerType  = ethPort.mii.sublayerType;
    mii->variantType   = ENET_MAC_VARIANT_FORCED;

    linkCfg->speed     = ENET_SPEED_AUTO;
    linkCfg->duplexity = ENET_DUPLEX_AUTO;

    boardPhyCfg = EnetBoard_getPhyCfg(&ethPort);
    if (boardPhyCfg != NULL)
    {
        EnetPhy_initCfg(phyCfg);
        phyCfg->phyAddr     = boardPhyCfg->phyAddr;
        phyCfg->isStrapped  = boardPhyCfg->isStrapped;
        phyCfg->loopbackEn  = false;
        phyCfg->skipExtendedCfg = boardPhyCfg->skipExtendedCfg;
        phyCfg->extendedCfgSize = boardPhyCfg->extendedCfgSize;
        memcpy(phyCfg->extendedCfg, boardPhyCfg->extendedCfg, phyCfg->extendedCfgSize);

    }
    else
    {
        EnetAppUtils_print("%s: No PHY configuration found\r\n", perCtxt->name);
        EnetAppUtils_assert(false);
    }
}

void EnetApp_closePort(EnetApp_PerCtxt *perCtxt)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    uint32_t i;
    int32_t status;

    for (i = 0U; i < perCtxt->macPortNum; i++)
    {
        macPort = perCtxt->macPort[i];

        EnetAppUtils_print("%s: Close port %u\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));

        /* Close port link */
        ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);

        EnetAppUtils_print("%s: Close port %u link\r\n", perCtxt->name, ENET_MACPORT_ID(macPort));
        ENET_IOCTL(perCtxt->hEnet, gEnetApp.coreId, ENET_PER_IOCTL_CLOSE_PORT_LINK, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to close port link: %d\r\n", perCtxt->name, status);
        }
    }
}

int32_t EnetApp_waitForLinkUp(EnetApp_PerCtxt *perCtxt)
{
    Enet_IoctlPrms prms;
    Enet_MacPort macPort;
    bool linked;
    uint32_t i;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("%s: Waiting for link up...\r\n", perCtxt->name);

    for (i = 0U; i < perCtxt->macPortNum; i++)
    {
        macPort = perCtxt->macPort[i];
        linked = false;

        while (gEnetApp.run && !linked)
        {
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &macPort, &linked);

            ENET_IOCTL(perCtxt->hEnet, gEnetApp.coreId, ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
            if (status != ENET_SOK)
            {
                EnetAppUtils_print("%s: Failed to get port %u link status: %d\r\n",
                                    perCtxt->name, ENET_MACPORT_ID(macPort), status);
                linked = false;
                break;
            }

            if (!linked)
            {
                ClockP_sleep(1);
            }
        }
    }
    return status;
}

void EnetApp_triggerReset(EnetApp_PerCtxt *perCtxt)
{
    Enet_Type enetType;
    uint32_t instId;
    int32_t status = ENET_SOK;

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0,
                            &enetType,
                            &instId);

    Enet_notify_t hardResetCpsw =
    {
        .cbFxn = &EnetSoC_toggleCPSWResetBit,
        .cbArg = NULL,
    };
    /* Saving Enet handle context and closing it*/
    status = Enet_hardResetCpsw(perCtxt->hEnet, enetType, instId, &hardResetCpsw);
    EnetAppUtils_assert(status == ENET_SOK);
}
