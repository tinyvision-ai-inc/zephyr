/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2023 tinyVision.ai
 *
 * Proprietary part of the USB23 driver, expected to become open-source some day,
 * but released as a binary blob for now.
 */

#include "udc_common.h"
#include "udc_usb23.h"

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>
#include <app_version.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb23blob, CONFIG_UDC_DRIVER_LOG_LEVEL);

/*
 * USB23 silicon core present on Lattice CrossLinkU-NX FPGAs.
 */

#define USB23_TRB_STATUS_BUFSIZ_MASK           GENMASK(23, 0)
#define USB23_TRB_STATUS_BUFSIZ_SHIFT          0
#define USB23_TRB_STATUS_PCM1_MASK             GENMASK(25, 24)
#define USB23_TRB_STATUS_PCM1_1PKT             (0x0 << 24)
#define USB23_TRB_STATUS_PCM1_2PKT             (0x1 << 24)
#define USB23_TRB_STATUS_PCM1_3PKT             (0x2 << 24)
#define USB23_TRB_STATUS_PCM1_4PKT             (0x3 << 24)
#define USB23_TRB_STATUS_TRBSTS_MASK           GENMASK(31, 28)
#define USB23_TRB_STATUS_TRBSTS_SHIFT          28
#define USB23_TRB_STATUS_TRBSTS_OK             (0x0 << 28)
#define USB23_TRB_STATUS_TRBSTS_MISSEDISOC     (0x1 << 28)
#define USB23_TRB_STATUS_TRBSTS_SETUPPENDING   (0x2 << 28)
#define USB23_TRB_STATUS_TRBSTS_XFERINPROGRESS (0x4 << 28)
#define USB23_TRB_STATUS_TRBSTS_ZLPPENDING     (0xf << 28)
#define USB23_TRB_CTRL_HWO                     BIT(0)
#define USB23_TRB_CTRL_LST                     BIT(1)
#define USB23_TRB_CTRL_CHN                     BIT(2)
#define USB23_TRB_CTRL_CSP                     BIT(3)
#define USB23_TRB_CTRL_TRBCTL_MASK             GENMASK(9, 4)
#define USB23_TRB_CTRL_TRBCTL_SHIFT            4
#define USB23_TRB_CTRL_TRBCTL_NORMAL           (0x1 << 4)
#define USB23_TRB_CTRL_TRBCTL_CONTROL_SETUP    (0x2 << 4)
#define USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 (0x3 << 4)
#define USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_3 (0x4 << 4)
#define USB23_TRB_CTRL_TRBCTL_CONTROL_DATA     (0x5 << 4)
#define USB23_TRB_CTRL_TRBCTL_ISOCHRONOUS_1    (0x6 << 4)
#define USB23_TRB_CTRL_TRBCTL_ISOCHRONOUS_N    (0x7 << 4)
#define USB23_TRB_CTRL_TRBCTL_LINK_TRB         (0x8 << 4)
#define USB23_TRB_CTRL_TRBCTL_NORMAL_ZLP       (0x9 << 4)
#define USB23_TRB_CTRL_ISP_IMI                 BIT(10)
#define USB23_TRB_CTRL_IOC                     BIT(11)
#define USB23_TRB_CTRL_PCM1_MASK               GENMASK(25, 24)
#define USB23_TRB_CTRL_PCM1_SHIFT              24
#define USB23_TRB_CTRL_SPR                     26
#define USB23_TRB_CTRL_SIDSOFN_MASK            GENMASK(29, 14)
#define USB23_TRB_CTRL_SIDSOFN_SHIFT           14

/* Incomplete coverage of all fields, but suited for what this driver supports */
#define USB23_EVT_MASK                        GENMASK(11, 0)
#define USB23_DEPEVT_EPN_MASK                 GENMASK(5, 1)
#define USB23_DEPEVT_EPN_SHIFT                1
#define USB23_DEPEVT_XFERCOMPLETE(epn)        (((epn) << 1) | (0x01 << 6))
#define USB23_DEPEVT_XFERINPROGRESS(epn)      (((epn) << 1) | (0x02 << 6))
#define USB23_DEPEVT_XFERNOTREADY(epn)        (((epn) << 1) | (0x03 << 6))
#define USB23_DEPEVT_RXTXFIFOEVT(epn)         (((epn) << 1) | (0x04 << 6))
#define USB23_DEPEVT_STREAMEVT(epn)           (((epn) << 1) | (0x06 << 6))
#define USB23_DEPEVT_EPCMDCMPLT(epn)          (((epn) << 1) | (0x07 << 6))
/* For XferNotReady */
#define USB23_DEPEVT_STATUS_B3_MASK           GENMASK(2, 0)
#define USB23_DEPEVT_STATUS_B3_CONTROL_SETUP  (0x0 << 0)
#define USB23_DEPEVT_STATUS_B3_CONTROL_DATA   (0x1 << 0)
#define USB23_DEPEVT_STATUS_B3_CONTROL_STATUS (0x2 << 0)
/* For XferComplete or XferInProgress */
#define USB23_DEPEVT_STATUS_BUSERR            BIT(0)
#define USB23_DEPEVT_STATUS_SHORT             BIT(1)
#define USB23_DEPEVT_STATUS_IOC               BIT(2)
/* For XferComplete */
#define USB23_DEPEVT_STATUS_LST               BIT(3)
/* For XferInProgress */
#define USB23_DEPEVT_STATUS_MISSED_ISOC       BIT(3)
/* For StreamEvt */
#define USB23_DEPEVT_STATUS_STREAMFOUND       0x1
#define USB23_DEPEVT_STATUS_STREAMNOTFOUND    0x2
#define USB23_DEVT_DISCONNEVT                 (BIT(0) | (0x0 << 8))
#define USB23_DEVT_USBRST                     (BIT(0) | (0x1 << 8))
#define USB23_DEVT_CONNECTDONE                (BIT(0) | (0x2 << 8))
#define USB23_DEVT_ULSTCHNG                   (BIT(0) | (0x3 << 8))
#define USB23_DEVT_WKUPEVT                    (BIT(0) | (0x4 << 8))
#define USB23_DEVT_SUSPEND                    (BIT(0) | (0x6 << 8))
#define USB23_DEVT_SOF                        (BIT(0) | (0x7 << 8))
#define USB23_DEVT_ERRTICERR                  (BIT(0) | (0x9 << 8))
#define USB23_DEVT_CMDCMPLT                   (BIT(0) | (0xa << 8))
#define USB23_DEVT_EVNTOVERFLOW               (BIT(0) | (0xb << 8))
#define USB23_DEVT_VNDRDEVTSTRCVED            (BIT(0) | (0xc << 8))

/* Device Endpoint Commands and Parameters */
#define USB23_DEPCMDPAR2(n)                          (0xc800 + 16 * (n))
#define USB23_DEPCMDPAR1(n)                          (0xc804 + 16 * (n))
#define USB23_DEPCMDPAR0(n)                          (0xc808 + 16 * (n))
#define USB23_DEPCMD(n)                              (0xc80c + 16 * (n))
/* Common fields to DEPCMD */
#define USB23_DEPCMD_HIPRI_FORCERM                   (1 << 11)
#define USB23_DEPCMD_STATUS_MASK                     GENMASK(15, 12)
#define USB23_DEPCMD_STATUS_OK                       (0 << 12)
#define USB23_DEPCMD_STATUS_CMDERR                   (1 << 12)
#define USB23_DEPCMD_XFERRSCIDX_MASK                 GENMASK(22, 16)
#define USB23_DEPCMD_XFERRSCIDX_SHIFT                16
/* DEPCFG Command and Parameters */
#define USB23_DEPCMD_DEPCFG                          (1 << 0)
#define USB23_DEPCMDPAR0_DEPCFG_EPTYPE_MASK          GENMASK(2, 1)
#define USB23_DEPCMDPAR0_DEPCFG_EPTYPE_CTRL          (0x0 << 1)
#define USB23_DEPCMDPAR0_DEPCFG_EPTYPE_ISOC          (0x1 << 1)
#define USB23_DEPCMDPAR0_DEPCFG_EPTYPE_BULK          (0x2 << 1)
#define USB23_DEPCMDPAR0_DEPCFG_EPTYPE_INT           (0x3 << 1)
#define USB23_DEPCMDPAR0_DEPCFG_MPS_MASK             GENMASK(13, 3)
#define USB23_DEPCMDPAR0_DEPCFG_MPS_SHIFT            3
#define USB23_DEPCMDPAR0_DEPCFG_FIFONUM_MASK         GENMASK(21, 17)
#define USB23_DEPCMDPAR0_DEPCFG_FIFONUM_SHIFT        17
#define USB23_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK         GENMASK(25, 22)
#define USB23_DEPCMDPAR0_DEPCFG_BRSTSIZ_SHIFT        22
#define USB23_DEPCMDPAR0_DEPCFG_ACTION_MASK          GENMASK(31, 30)
#define USB23_DEPCMDPAR0_DEPCFG_ACTION_INIT          (0x0 << 30)
#define USB23_DEPCMDPAR0_DEPCFG_ACTION_RESTORE       (0x1 << 30)
#define USB23_DEPCMDPAR0_DEPCFG_ACTION_MODIFY        (0x2 << 30)
#define USB23_DEPCMDPAR1_DEPCFG_INTRNUM_MASK         GENMASK(4, 0)
#define USB23_DEPCMDPAR1_DEPCFG_INTRNUM_SHIFT        0
#define USB23_DEPCMDPAR1_DEPCFG_XFERCMPLEN           BIT(8)
#define USB23_DEPCMDPAR1_DEPCFG_XFERINPROGEN         BIT(9)
#define USB23_DEPCMDPAR1_DEPCFG_XFERNRDYEN           BIT(10)
#define USB23_DEPCMDPAR1_DEPCFG_RXTXFIFOEVTEN        BIT(11)
#define USB23_DEPCMDPAR1_DEPCFG_STREAMEVTEN          BIT(13)
#define USB23_DEPCMDPAR1_DEPCFG_LIMITTXDMA           BIT(15)
#define USB23_DEPCMDPAR1_DEPCFG_BINTERVAL_MASK       GENMASK(23, 16)
#define USB23_DEPCMDPAR1_DEPCFG_BINTERVAL_SHIFT      16
#define USB23_DEPCMDPAR1_DEPCFG_STRMCAP              BIT(24)
#define USB23_DEPCMDPAR1_DEPCFG_EPNUMBER_MASK        GENMASK(29, 25)
#define USB23_DEPCMDPAR1_DEPCFG_EPNUMBER_SHIFT       25
#define USB23_DEPCMDPAR1_DEPCFG_BULKBASED            BIT(30)
#define USB23_DEPCMDPAR1_DEPCFG_FIFOBASED            BIT(31)
#define USB23_DEPCMDPAR2_DEPCFG_EPSTATE_MASK         GENMASK(31, 0)
#define USB23_DEPCMDPAR2_DEPCFG_EPSTATE_SHIFT        0
/* DEPXFERCFG Command and Parameters */
#define USB23_DEPCMD_DEPXFERCFG                      (0x2 << 0)
#define USB23_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK  GENMASK(15, 0)
#define USB23_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_SHIFT 0
/* Other Commands */
#define USB23_DEPCMD_DEPGETSTATE                     (0x3 << 0)
#define USB23_DEPCMD_DEPSETSTALL                     (0x4 << 0)
#define USB23_DEPCMD_DEPCSTALL                       (0x5 << 0)
#define USB23_DEPCMD_DEPSTRTXFER                     (0x6 << 0)
#define USB23_DEPCMD_DEPUPDXFER                      (0x7 << 0)
#define USB23_DEPCMD_DEPENDXFER                      (0x8 << 0)
#define USB23_DEPCMD_DEPSTARTCFG                     (0x9 << 0)
#define USB23_DEPCMD_CMDACT                          BIT(10)

/* Global USB2 (UTMI/ULPI) PHY configuration */
#define USB23_GUSB2PHYCFG                      0xC200
#define USB23_GUSB2PHYCFG_PHYSOFTRST           BIT(31)
#define USB23_GUSB2PHYCFG_ULPIEXTVBUSINDICATOR BIT(18)
#define USB23_GUSB2PHYCFG_ULPIEXTVBUSDRV       BIT(17)
#define USB23_GUSB2PHYCFG_ULPICLKSUSM          BIT(16)
#define USB23_GUSB2PHYCFG_ULPIAUTORES          BIT(15)
#define USB23_GUSB2PHYCFG_USBTRDTIM_MASK       GENMASK(13, 10)
#define USB23_GUSB2PHYCFG_USBTRDTIM_16BIT      (5 << 10)
#define USB23_GUSB2PHYCFG_USBTRDTIM_8BIT       (9 << 10)
#define USB23_GUSB2PHYCFG_ENBLSLPM             BIT(8)
#define USB23_GUSB2PHYCFG_PHYSEL               BIT(7)
#define USB23_GUSB2PHYCFG_SUSPHY               BIT(6)
#define USB23_GUSB2PHYCFG_FSINTF               BIT(5)
#define USB23_GUSB2PHYCFG_ULPI_UTMI_SEL        BIT(4)
#define USB23_GUSB2PHYCFG_PHYIF                BIT(3)
#define USB23_GUSB2PHYCFG_TOUTCAL_MASK         GENMASK(2, 0)

/* Global USB 3.0 PIPE Control Register */
#define USB23_GUSB3PIPECTL                       0xc2c0
#define USB23_GUSB3PIPECTL_PHYSOFTRST            BIT(31)
#define USB23_GUSB3PIPECTL_UX_EXIT_IN_PX         BIT(27)
#define USB23_GUSB3PIPECTL_PING_ENHANCEMENT_EN   BIT(26)
#define USB23_GUSB3PIPECTL_U1U2EXITFAIL_TO_RECOV BIT(25)
#define USB23_GUSB3PIPECTL_REQUEST_P1P2P3        BIT(24)
#define USB23_GUSB3PIPECTL_STARTXDETU3RXDET      BIT(23)
#define USB23_GUSB3PIPECTL_DISRXDETU3RXDET       BIT(22)
#define USB23_GUSB3PIPECTL_P1P2P3DELAY_MASK      GENMASK(21, 19)
#define USB23_GUSB3PIPECTL_P1P2P3DELAY_SHIFT     19
#define USB23_GUSB3PIPECTL_DELAYP0TOP1P2P3       BIT(18)
#define USB23_GUSB3PIPECTL_SUSPENDENABLE         BIT(17)
#define USB23_GUSB3PIPECTL_DATWIDTH_MASK         GENMASK(16, 15)
#define USB23_GUSB3PIPECTL_DATWIDTH_SHIFT        15
#define USB23_GUSB3PIPECTL_ABORTRXDETINU2        BIT(14)
#define USB23_GUSB3PIPECTL_SKIPRXDET             BIT(13)
#define USB23_GUSB3PIPECTL_LFPSP0ALGN            BIT(12)
#define USB23_GUSB3PIPECTL_P3P2TRANOK            BIT(11)
#define USB23_GUSB3PIPECTL_P3EXSIGP2             BIT(10)
#define USB23_GUSB3PIPECTL_LFPSFILT              BIT(9)
#define USB23_GUSB3PIPECTL_TXSWING               BIT(6)
#define USB23_GUSB3PIPECTL_TXMARGIN_MASK         GENMASK(5, 3)
#define USB23_GUSB3PIPECTL_TXMARGIN_SHIFT        3
#define USB23_GUSB3PIPECTL_TXDEEMPHASIS_MASK     GENMASK(2, 1)
#define USB23_GUSB3PIPECTL_TXDEEMPHASIS_SHIFT    1
#define USB23_GUSB3PIPECTL_ELASTICBUFFERMODE     BIT(0)

/* USB Device Configuration Register */
#define USB23_DCFG                    0xc700
#define USB23_DCFG_IGNORESTREAMPP     BIT(23)
#define USB23_DCFG_LPMCAP             BIT(22)
#define USB23_DCFG_NUMP_MASK          GENMASK(21, 17)
#define USB23_DCFG_NUMP_SHIFT         17
#define USB23_DCFG_INTRNUM_MASK       GENMASK(16, 12)
#define USB23_DCFG_PERFRINT_MASK      GENMASK(11, 10)
#define USB23_DCFG_PERFRINT_80        (0x0 << 10)
#define USB23_DCFG_PERFRINT_85        (0x1 << 10)
#define USB23_DCFG_PERFRINT_90        (0x2 << 10)
#define USB23_DCFG_PERFRINT_95        (0x3 << 10)
#define USB23_DCFG_DEVADDR_MASK       GENMASK(9, 3)
#define USB23_DCFG_DEVADDR_SHIFT      3
#define USB23_DCFG_DEVSPD_MASK        GENMASK(2, 0)
#define USB23_DCFG_DEVSPD_HIGH_SPEED  (0x0 << 0)
#define USB23_DCFG_DEVSPD_FULL_SPEED  (0x1 << 0)
#define USB23_DCFG_DEVSPD_SUPER_SPEED (0x4 << 0)

/* Global SoC Bus Configuration Register */
#define USB23_GSBUSCFG0                0xc100
#define USB23_GSBUSCFG0_DATRDREQINFO   GENMASK(31, 28)
#define USB23_GSBUSCFG0_DESRDREQINFO   GENMASK(27, 24)
#define USB23_GSBUSCFG0_DATWRREQINFO   GENMASK(23, 20)
#define USB23_GSBUSCFG0_DESWRREQINFO   GENMASK(19, 16)
#define USB23_GSBUSCFG0_DATBIGEND      BIT(11)
#define USB23_GSBUSCFG0_DESBIGEND      BIT(10)
#define USB23_GSBUSCFG0_INCR256BRSTENA BIT(7)
#define USB23_GSBUSCFG0_INCR128BRSTENA BIT(6)
#define USB23_GSBUSCFG0_INCR64BRSTENA  BIT(5)
#define USB23_GSBUSCFG0_INCR32BRSTENA  BIT(4)
#define USB23_GSBUSCFG0_INCR16BRSTENA  BIT(3)
#define USB23_GSBUSCFG0_INCR8BRSTENA   BIT(2)
#define USB23_GSBUSCFG0_INCR4BRSTENA   BIT(1)
#define USB23_GSBUSCFG0_INCRBRSTENA    BIT(0)

/* Global Tx Threshold Control Register */
#define USB23_GTXTHRCFG                         0xc108
#define USB23_GTXTHRCFG_USBTXPKTCNTSEL          BIT(29)
#define USB23_GTXTHRCFG_USBTXPKTCNT_MASK        GENMASK(27, 24)
#define USB23_GTXTHRCFG_USBTXPKTCNT_SHIFT       24
#define USB23_GTXTHRCFG_USBMAXTXBURSTSIZE_MASK  GENMASK(23, 16)
#define USB23_GTXTHRCFG_USBMAXTXBURSTSIZE_SHIFT 16

/* Global control register */
#define USB23_GCTL                  0xc110
#define USB23_GCTL_PWRDNSCALE_MASK  GENMASK(31, 19)
#define USB23_GCTL_MASTERFILTBYPASS BIT(18)
#define USB23_GCTL_BYPSSETADDR      BIT(17)
#define USB23_GCTL_U2RSTECN         BIT(16)
#define USB23_GCTL_FRMSCLDWN_MASK   GENMASK(15, 14)
#define USB23_GCTL_PRTCAPDIR_MASK   GENMASK(13, 12)
#define USB23_GCTL_CORESOFTRESET    BIT(11)
#define USB23_GCTL_DEBUGATTACH      BIT(8)
#define USB23_GCTL_RAMCLKSEL_MASK   GENMASK(7, 6)
#define USB23_GCTL_SCALEDOWN_MASK   GENMASK(5, 4)
#define USB23_GCTL_DISSCRAMBLE      BIT(3)
#define USB23_GCTL_DSBLCLKGTNG      BIT(0)

/* Global User Control Register */
#define USB23_GUCTL                     0xc12c
#define USB23_GUCTL_NOEXTRDL            BIT(21)
#define USB23_GUCTL_PSQEXTRRESSP_MASK   GENMASK(20, 18)
#define USB23_GUCTL_PSQEXTRRESSP_EN     (0x1 << 18)
#define USB23_GUCTL_SPRSCTRLTRANSEN     BIT(17)
#define USB23_GUCTL_RESBWHSEPS          BIT(16)
#define USB23_GUCTL_CMDEVADDR           BIT(15)
#define USB23_GUCTL_USBHSTINAUTORETRYEN BIT(14)
#define USB23_GUCTL_DTCT_MASK           GENMASK(10, 9)
#define USB23_GUCTL_DTFT_MASK           GENMASK(8, 0)

/* USB Device Control register */
#define USB23_DCTL                          0xc704
#define USB23_DCTL_RUNSTOP                  BIT(31)
#define USB23_DCTL_CSFTRST                  BIT(30)
#define USB23_DCTL_HIRDTHRES_4              BIT(28)
#define USB23_DCTL_HIRDTHRES_TIME_MASK      GENMASK(27, 24)
#define USB23_DCTL_APPL1RES                 BIT(23)
#define USB23_DCTL_LPM_NYET_THRES_MASK      GENMASK(23, 20)
#define USB23_DCTL_LPM_NYET_THRES_SHIFT     20
#define USB23_DCTL_KEEPCONNECT              BIT(19)
#define USB23_DCTL_L1HIBERNATIONEN          BIT(18)
#define USB23_DCTL_CRS                      BIT(17)
#define USB23_DCTL_CSS                      BIT(16)
#define USB23_DCTL_INITU2ENA                BIT(12)
#define USB23_DCTL_ACCEPTU2ENA              BIT(11)
#define USB23_DCTL_INITU1ENA                BIT(10)
#define USB23_DCTL_ACCEPTU1ENA              BIT(9)
#define USB23_DCTL_ULSTCHNGREQ_MASK         GENMASK(8, 5)
#define USB23_DCTL_ULSTCHNGREQ_REMOTEWAKEUP (0x8 << 5)
#define USB23_DCTL_TSTCTL_MASK              GENMASK(4, 1)

/* USB Device Event Enable Register */
#define USB23_DEVTEN                     0xc708
#define USB23_DEVTEN_INACTTIMEOUTRCVEDEN BIT(13)
#define USB23_DEVTEN_VNDRDEVTSTRCVEDEN   BIT(12)
#define USB23_DEVTEN_EVNTOVERFLOWEN      BIT(11)
#define USB23_DEVTEN_CMDCMPLTEN          BIT(10)
#define USB23_DEVTEN_ERRTICERREN         BIT(9)
#define USB23_DEVTEN_SOFEN               BIT(7)
#define USB23_DEVTEN_EOPFEN              BIT(6)
#define USB23_DEVTEN_HIBERNATIONREQEVTEN BIT(5)
#define USB23_DEVTEN_WKUPEVTEN           BIT(4)
#define USB23_DEVTEN_ULSTCNGEN           BIT(3)
#define USB23_DEVTEN_CONNECTDONEEN       BIT(2)
#define USB23_DEVTEN_USBRSTEN            BIT(1)
#define USB23_DEVTEN_DISCONNEVTEN        BIT(0)

/* USB Device Event Register */

/* Endpoint Global Event Buffer Address (64-bit) */
#define USB23_GEVNTADR_LO(n) (0xc400 + 16 * (n))
#define USB23_GEVNTADR_HI(n) (0xc404 + 16 * (n))

/* Endpoint Global Event Buffer Size */
#define USB23_GEVNTSIZ(n)             (0xc408 + 16 * (n))
#define USB23_GEVNTSIZ_EVNTINTRPTMASK BIT(31)

/* Endpoint Global Event Buffer Count (of valid event) */
#define USB23_GEVNTCOUNT(n) (0xc40c + 16 * (n))

/* USB Device Active USB Endpoint Enable */
#define USB23_DALEPENA             0xC720
#define USB23_DALEPENA_USBACTEP(n) (1 << (n))

/* USB Device Core Identification and Release Number Register */
#define USB23_GCOREID            0xC120
#define USB23_GCOREID_CORE_MASK  GENMASK(31, 16)
#define USB23_GCOREID_CORE_SHIFT 16
#define USB23_GCOREID_REL_MASK   GENMASK(15, 0)
#define USB23_GCOREID_REL_SHIFT  0

/* USB Globa Status register */
#define USB23_GSTS               0xc118
#define USB23_GSTS_CBELT_MASK    GENMASK(31, 20)
#define USB23_GSTS_SSIC_IP       BIT(11)
#define USB23_GSTS_OTG_IP        BIT(10)
#define USB23_GSTS_BC_IP         BIT(9)
#define USB23_GSTS_ADP_IP        BIT(8)
#define USB23_GSTS_HOST_IP       BIT(7)
#define USB23_GSTS_DEVICE_IP     BIT(6)
#define USB23_GSTS_CSRTIMEOUT    BIT(5)
#define USB23_GSTS_BUSERRADDRVLD BIT(4)
#define USB23_GSTS_CURMOD_MASK   GENMASK(1, 0)

/* USB Global TX FIFO Size register */
#define USB23_GTXFIFOSIZ(n)              (0xc300 + 4 * (n))
#define USB23_GTXFIFOSIZ_TXFSTADDR_MASK  GENMASK(31, 16)
#define USB23_GTXFIFOSIZ_TXFSTADDR_SHIFT 16
#define USB23_GTXFIFOSIZ_TXFDEP_MASK     GENMASK(15, 0)
#define USB23_GTXFIFOSIZ_TXFDEP_SHIFT    0

/* USB Global RX FIFO Size register */
#define USB23_GRXFIFOSIZ(n)              (0xc380 + 4 * (n))
#define USB23_GRXFIFOSIZ_TXFSTADDR_MASK  GENMASK(31, 16)
#define USB23_GRXFIFOSIZ_TXFSTADDR_SHIFT 16
#define USB23_GRXFIFOSIZ_TXFDEP_MASK     GENMASK(15, 0)
#define USB23_GRXFIFOSIZ_TXFDEP_SHIFT    0

/* USB Bus Error Address registers */
#define USB23_GBUSERRADDR_LO 0xc130
#define USB23_GBUSERRADDR_HI 0xc134

/* USB Controller Debug register */
#define USB23_CTLDEBUG_LO 0xe000
#define USB23_CTLDEBUG_HI 0xe004

/* USB Analyzer Trace register */
#define USB23_ANALYZERTRACE 0xe008

/* USB Global Debug Queue/FIFO Space Available register */
#define USB23_GDBGFIFOSPACE                     0xc160
#define USB23_GDBGFIFOSPACE_AVAILABLE_MASK      GENMASK(31, 16)
#define USB23_GDBGFIFOSPACE_AVAILABLE_SHIFT     16
#define USB23_GDBGFIFOSPACE_QUEUETYPE_MASK      GENMASK(8, 5)
#define USB23_GDBGFIFOSPACE_QUEUETYPE_TX        (0x0 << 5)
#define USB23_GDBGFIFOSPACE_QUEUETYPE_RX        (0x1 << 5)
#define USB23_GDBGFIFOSPACE_QUEUETYPE_TXREQ     (0x2 << 5)
#define USB23_GDBGFIFOSPACE_QUEUETYPE_RXREQ     (0x3 << 5)
#define USB23_GDBGFIFOSPACE_QUEUETYPE_RXINFO    (0x4 << 5)
#define USB23_GDBGFIFOSPACE_QUEUETYPE_PROTOCOL  (0x5 << 5)
#define USB23_GDBGFIFOSPACE_QUEUETYPE_DESCFETCH (0x6 << 5)
#define USB23_GDBGFIFOSPACE_QUEUETYPE_WREVENT   (0x7 << 5)
#define USB23_GDBGFIFOSPACE_QUEUETYPE_AUXEVENT  (0x8 << 5)
#define USB23_GDBGFIFOSPACE_QUEUENUM_MASK       GENMASK(4, 0)
#define USB23_GDBGFIFOSPACE_QUEUENUM_SHIFT      0

/* USB Global Debug LTSSM register */
#define USB23_GDBGLTSSM 0xc164

/* Global Debug LNMCC Register */
#define USB23_GDBGLNMCC 0xc168

/* Global Debug BMU Register */
#define USB23_GDBGBMU 0xc16c

/* Global Debug LSP MUX Register - Device*/
#define USB23_GDBGLSPMUX_DEV 0xc170

/* Global Debug LSP MUX Register - Host */
#define USB23_GDBGLSPMUX_HST 0xc170

/* Global Debug LSP Register */
#define USB23_GDBGLSP 0xc174

/* Global Debug Endpoint Information Register 0 */
#define USB23_GDBGEPINFO0 0xc178

/* Global Debug Endpoint Information Register 1 */
#define USB23_GDBGEPINFO1 0xc17c

/* U3 Root Hub Debug Register */
#define USB23_BU3RHBDBG0 0xd800

/* USB Device Status register */
#define USB23_DSTS                             0xC70C
#define USB23_DSTS_DCNRD                       BIT(29)
#define USB23_DSTS_SRE                         BIT(28)
#define USB23_DSTS_RSS                         BIT(25)
#define USB23_DSTS_SSS                         BIT(24)
#define USB23_DSTS_COREIDLE                    BIT(23)
#define USB23_DSTS_DEVCTRLHLT                  BIT(22)
#define USB23_DSTS_USBLNKST_MASK               GENMASK(21, 18)
#define USB23_DSTS_USBLNKST_USB3_U0            (0x0 << 18)
#define USB23_DSTS_USBLNKST_USB3_U1            (0x1 << 18)
#define USB23_DSTS_USBLNKST_USB3_U2            (0x2 << 18)
#define USB23_DSTS_USBLNKST_USB3_U3            (0x3 << 18)
#define USB23_DSTS_USBLNKST_USB3_SS_DIS        (0x4 << 18)
#define USB23_DSTS_USBLNKST_USB3_RX_DET        (0x5 << 18)
#define USB23_DSTS_USBLNKST_USB3_SS_INACT      (0x6 << 18)
#define USB23_DSTS_USBLNKST_USB3_POLL          (0x7 << 18)
#define USB23_DSTS_USBLNKST_USB3_RECOV         (0x8 << 18)
#define USB23_DSTS_USBLNKST_USB3_HRESET        (0x9 << 18)
#define USB23_DSTS_USBLNKST_USB3_CMPLY         (0xa << 18)
#define USB23_DSTS_USBLNKST_USB3_LPBK          (0xb << 18)
#define USB23_DSTS_USBLNKST_USB3_RESET_RESUME  (0xf << 18)
#define USB23_DSTS_USBLNKST_USB2_ON_STATE      (0x0 << 18)
#define USB23_DSTS_USBLNKST_USB2_SLEEP_STATE   (0x2 << 18)
#define USB23_DSTS_USBLNKST_USB2_SUSPEND_STATE (0x3 << 18)
#define USB23_DSTS_USBLNKST_USB2_DISCONNECTED  (0x4 << 18)
#define USB23_DSTS_USBLNKST_USB2_EARLY_SUSPEND (0x5 << 18)
#define USB23_DSTS_USBLNKST_USB2_RESET         (0xe << 18)
#define USB23_DSTS_USBLNKST_USB2_RESUME        (0xf << 18)
#define USB23_DSTS_RXFIFOEMPTY                 BIT(17)
#define USB23_DSTS_SOFFN_MASK                  GENMASK(16, 3)
#define USB23_DSTS_CONNECTSPD_MASK             GENMASK(2, 0)
#define USB23_DSTS_CONNECTSPD_HS               (0x0 << 0)
#define USB23_DSTS_CONNECTSPD_FS               (0x1 << 0)
#define USB23_DSTS_CONNECTSPD_SS               (0x4 << 0)

/* Device Generic Command and Parameter */
#define USB23_DGCMDPAR                         0xc710
#define USB23_DGCMD                            0xc714
#define USB23_DGCMD_STATUS_MASK                GENMASK(15, 12)
#define USB23_DGCMD_STATUS_ERR                 (1 << 12)
#define USB23_DGCMD_STATUS_OK                  (0 << 12)
#define USB23_DGCMD_ACT                        BIT(10)
#define USB23_DGCMD_IOC                        BIT(8)
#define USB23_DGCMD_MASK                       GENMASK(7, 0)
/* EXITLATENCY command and parameters */
#define USB23_DGCMD_EXITLATENCY                (2 << 0)
#define USB23_DGCMDPAR_EXITLATENCY_U1SEL_SHIFT 0
#define USB23_DGCMDPAR_EXITLATENCY_U1PEL_SHIFT 8
#define USB23_DGCMDPAR_EXITLATENCY_U2SEL_SHIFT 16
#define USB23_DGCMDPAR_EXITLATENCY_U2PEL_SHIFT 24
/* Other Commands and Parameters */
#define USB23_DGCMD_LINKFUNCTION               (1 << 0)
#define USB23_DGCMD_WAKENOTIFNUM               (3 << 0)
#define USB23_DGCMD_FIFOFLUSHONE               (9 << 0)
#define USB23_DGCMD_FIFOFLUSHALL               (10 << 0)
#define USB23_DGCMD_ENDPOINTNRDY               (12 << 0)
#define USB23_DGCMD_LOOPBACKTEST               (16 << 0)
#define USB23_DGCMD_ROLEREQUEST                (6 << 0)

/* USB 2.0 PHY External CSR Configuration register 0 */
#define USB23_U2PHYCTRL0                     0x00018000
#define USB23_U2PHYCTRL0_LDO_1P0_ADJ_MASK    GENMASK(1, 0)
#define USB23_U2PHYCTRL0_LDO_1P0_ADJ_SHIFT   0
#define USB23_U2PHYCTRL0_LS_CROSS_ADJ_MASK   GENMASK(4, 2)
#define USB23_U2PHYCTRL0_LS_CROSS_ADJ_SHIFT  2
#define USB23_U2PHYCTRL0_HS_RISE_TUNE_MASK   GENMASK(6, 5)
#define USB23_U2PHYCTRL0_HS_RISE_TUNE_SHIFT  5
#define USB23_U2PHYCTRL0_CDR_RST_SEL         BIT(7)
#define USB23_U2PHYCTRL0_GLOBAL_CONFIG_MASK  GENMASK(15, 8)
#define USB23_U2PHYCTRL0_GLOBAL_CONFIG_SHIFT 8
#define USB23_U2PHYCTRL0_FS_CROSS_ADJ_MASK   GENMASK(18, 16)
#define USB23_U2PHYCTRL0_FS_CROSS_ADJ_SHIFT  16
#define USB23_U2PHYCTRL0_REG14_ADJ_MASK      GENMASK(21, 19)
#define USB23_U2PHYCTRL0_REG14_ADJ_SHIFT     19
#define USB23_U2PHYCTRL0_HS_REG0P8_ADJ_MASK  GENMASK(23, 22)
#define USB23_U2PHYCTRL0_HS_REG0P8_ADJ_SHIFT 22
#define USB23_U2PHYCTRL0_SQL_SP_ADJ_MASK     GENMASK(27, 24)
#define USB23_U2PHYCTRL0_SQL_SP_ADJ_SHIFT    24
#define USB23_U2PHYCTRL0_VBUS_VLD_ADJ_MASK   GENMASK(31, 28)
#define USB23_U2PHYCTRL0_VBUS_VLD_ADJ_SHIFT  28

/* USB 2.0 PHY External CSR Configuration register 1 */
#define USB23_U2PHYCTRL1                      0x00018004
#define USB23_U2PHYCTRL1_CALIB_ONCE_EN        BIT(0)
#define USB23_U2PHYCTRL1_HS_EMP_ADJ_MASK      GENMASK(2, 1)
#define USB23_U2PHYCTRL1_HS_EMP_ADJ_SHIFT     1
#define USB23_U2PHYCTRL1_SQL_CUR_ADJ_MASK     GENMASK(4, 3)
#define USB23_U2PHYCTRL1_SQL_CUR_ADJ_SHIFT    3
#define USB23_U2PHYCTRL1_PLLBW_SEL_MASK       GENMASK(7, 5)
#define USB23_U2PHYCTRL1_PLLBW_SEL_SHIFT      5
#define USB23_U2PHYCTRL1_BIST_EN_N            BIT(8)
#define USB23_U2PHYCTRL1_SCP_EN               BIT(9)
#define USB23_U2PHYCTRL1_SEL_12_24M           BIT(10)
#define USB23_U2PHYCTRL1_HS_LP_MODE_EN        BIT(11)
#define USB23_U2PHYCTRL1_SQL_VTH_ADJ_MASK     GENMASK(15, 12)
#define USB23_U2PHYCTRL1_SQL_VTH_ADJ_SHIFT    12
#define USB23_U2PHYCTRL1_HS_EMP_EN            BIT(16)
#define USB23_U2PHYCTRL1_RSTN_BYPASS          BIT(17)
#define USB23_U2PHYCTRL1_CDR_BW_SEL_MASK      GENMASK(19, 18)
#define USB23_U2PHYCTRL1_CDR_BW_SEL_SHIFT     18
#define USB23_U2PHYCTRL1_CDR_TIMING_SEL_MASK  GENMASK(23, 20)
#define USB23_U2PHYCTRL1_CDR_TIMING_SEL_SHIFT 20
#define USB23_U2PHYCTRL1_SEL_INTERNALCLK      BIT(24)
#define USB23_U2PHYCTRL1_XOSC_CUR_ADJ_MASK    GENMASK(27, 25)
#define USB23_U2PHYCTRL1_XOSC_CUR_ADJ_SHIFT   25
#define USB23_U2PHYCTRL1_DISC_ADJ_MASK        GENMASK(31, 28)
#define USB23_U2PHYCTRL1_DISC_ADJ_SHIFT       28

/* USB 2.0 PHY External CSR Configuration register 2 */
#define USB23_U2PHYCTRL2                 0x00018008
#define USB23_U2PHYCTRL2_XCLK12MOUTEN    BIT(0)
#define USB23_U2PHYCTRL2_TEST_LOOP_MASK  GENMASK(3, 1)
#define USB23_U2PHYCTRL2_TEST_LOOP_SHIFT 1
#define USB23_U2PHYCTRL2_RX_LP_EN        BIT(4)
#define USB23_U2PHYCTRL2_REG20_ADJ_MASK  GENMASK(7, 5)
#define USB23_U2PHYCTRL2_REG20_ADJ_SHIFT 5
#define USB23_U2PHYCTRL2_CLK_SEL         BIT(8)
#define USB23_U2PHYCTRL2_INTERNAL_RST    BIT(9)
#define USB23_U2PHYCTRL2_REFCLK_SEL      BIT(10)
#define USB23_U2PHYCTRL2_BIST_DONE       BIT(11)
#define USB23_U2PHYCTRL2_BIST_ERR        BIT(12)

/* USB 3.0 PHY External Configuration registers (undocumented) */
#define USB23_U3PHYCTRL0           0x000100C8
#define USB23_U3PHYCTRL1           0x0001008C
#define USB23_U3PHYCTRL2           0x00010090
#define USB23_U3PHYCTRL3           0x00010094
#define USB23_U3PHYCTRL4           0x00010040
#define USB23_U3PHYCTRL4_INT_CLOCK BIT(14)

/* USB 3.0 PHY Internal Configuration register (undocumented) */
#define USB23_U3PHYCTRL5 0x00014010

/* Hardware parameters */
#define USB23_GHWPARAMS0                      0xc140
#define USB23_GHWPARAMS1                      0xc144
#define USB23_GHWPARAMS2                      0xc148
#define USB23_GHWPARAMS3                      0xc14c
#define USB23_GHWPARAMS4                      0xc150
#define USB23_GHWPARAMS5                      0xc154
#define USB23_GHWPARAMS6                      0xc158
#define USB23_GHWPARAMS6_USB3_HSPHY_INTERFACE GENMASK(5, 4)
#define USB23_GHWPARAMS7                      0xc15c
#define USB23_GHWPARAMS8                      0xc600

/* Helper macros */
#define LO32(n)                 ((uint32_t)((uint64_t)(n) & 0xffffffff))
#define HI32(n)                 ((uint32_t)((uint64_t)(n) >> 32))
#define U64(a, b)               (((uint64_t)(a)) << 32 | (((uint64_t)(b)) & 0xffffffff))
#define MEM32(addr)             (*(volatile uint32_t *)(addr))
#define GETFIELD(reg, prefix)   ((reg & prefix##_MASK) >> prefix##_SHIFT)
#define LOG_EVENT(name)         LOG_DBG("--- %s ---", #name)
#define USB_EP_IS_CONTROL(addr) ((addr) == USB_CONTROL_EP_IN || (addr) == USB_CONTROL_EP_OUT)
#define FOREACH_NORMAL_IN_EP(f)                                                                    \
	f(2) f(4) f(6) f(8) f(10) f(12) f(14) f(16) f(18) f(20) f(22) f(24) f(26) f(28) f(30)
#define FOREACH_NORMAL_OUT_EP(f)                                                                   \
	f(3) f(5) f(7) f(9) f(11) f(13) f(15) f(17) f(19) f(21) f(23) f(25) f(27) f(29) f(31)
#define CASE_DEPEVT_XFERCOMPLETE(n) case USB23_DEPEVT_XFERCOMPLETE(n):
#define CASE_DEPEVT_XFERINPROGRESS(n) case USB23_DEPEVT_XFERINPROGRESS(n):

/*
 * uvcmanager
 *
 * Extra core that automates the endpoint enqueueing to increase throughput with smaller buffers.
 * This reduces memory usage and improves latency.
 */

#define USB23_MANAGER_IRQRAW                              0x0000
#define USB23_MANAGER_IRQRAW_USB                          BIT(0)
#define USB23_MANAGER_IRQRAW_IRQ                          BIT(1)
#define USB23_MANAGER_IRQRAW_EOF                          BIT(2)
#define USB23_MANAGER_IRQFORCE                            0x0004
#define USB23_MANAGER_IRQFORCE_USB                        BIT(0)
#define USB23_MANAGER_IRQFORCE_IRQ                        BIT(1)
#define USB23_MANAGER_IRQFORCE_EOF                        BIT(2)
#define USB23_MANAGER_IRQMASK                             0x0008
#define USB23_MANAGER_IRQMASK_MASK                        GENMASK(2, 0)
#define USB23_MANAGER_IRQMASK_USB                         BIT(0)
#define USB23_MANAGER_IRQMASK_IRQ                         BIT(1)
#define USB23_MANAGER_IRQMASK_EOF                         BIT(2)
#define USB23_MANAGER_IRQSTATUS                           0x000c
#define USB23_MANAGER_IRQSTATUS_USB                       BIT(0)
#define USB23_MANAGER_IRQSTATUS_IRQ                       BIT(1)
#define USB23_MANAGER_IRQSTATUS_EOF                       BIT(2)
#define USB23_MANAGER_CONTROLSTATUS                       0x0010
#define USB23_MANAGER_CONTROLSTATUS_ENABLE                BIT(0)
#define USB23_MANAGER_CONTROLSTATUS_BUSY                  BIT(1)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_SHIFT        2
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_MASK         GENMASK(5, 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_IDLE         (0x0 << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_FIFOCHECK    (0x1 << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WAIT         (0x2 << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_RTRBCTRL     (0x3 << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WTRBCTRL     (0x4 << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_RTRBSIZE     (0x5 << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WTRBSIZE     (0x6 << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WTRBADDR     (0x7 << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_RINGDOORBELL (0x8 << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_DONE         (0x9 << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_FIFOWAIT     (0xa << 2)
#define USB23_MANAGER_CONTROLSTATUS_FSMSTATE_ERROR        (0xf << 2)
#define USB23_MANAGER_CONTROLSTATUS_TRBID_SHIFT           6
#define USB23_MANAGER_CONTROLSTATUS_TRBID_MASK            GENMASK(13, 6)
#define USB23_MANAGER_CONTROLSTATUS_CONTINUE              BIT(14)
#define USB23_MANAGER_TRBADDR                             0x0014
#define USB23_MANAGER_NUMBYTES                            0x0018
#define USB23_MANAGER_DONEBYTES                           0x001c
#define USB23_MANAGER_DOORBELLADDR                        0x0020
#define USB23_MANAGER_DOORBELLDATA                        0x0024
#define USB23_MANAGER_FIFOTHRESHOLD                       0x0028
#define USB23_MANAGER_STREAMADDRESS                       0x002c
#define USB23_MANAGER_TRBCTRL                             0x0030
#define USB23_MANAGER_FIFOOCCUPANCY                       0x0034
#define USB23_MANAGER_TOTALBYTESOUT                       0x0038
#define USB23_MANAGER_HEADER0                             0x003c
#define USB23_MANAGER_HEADER1                             0x0040
#define USB23_MANAGER_FRAMECOUNTER                        0x0044

/*
 * Format for an endpoint-specific event.
 */
struct usb23_depevt {
	uint32_t category: 1;
	uint32_t ep_num: 5;
	uint32_t type: 4;
	uint32_t reserved_11_10: 2;
	uint32_t status: 4;
	uint32_t parameters: 16;
} __packed;

/*
 * Format for a device event unrelated to any endpoint.
 */
struct usb23_devt {
	uint32_t category: 1;
	uint32_t event: 7;
	uint32_t type: 4;
	uint32_t reserved_15_12: 4;
	uint32_t evtinfo: 9;
	uint32_t reserved_31_25: 7;
} __packed;

/*
 * Format of each entry of the event buffer.
 */
union usb23_evt {
	struct usb23_depevt depevt;
	struct usb23_devt devt;
	uint32_t raw;
};

void usb23_dump_all(const struct device *dev, struct usb23_ep_data *ep_data, const struct shell *sh);

uint32_t usb23_manager_read(const struct device *dev, struct usb23_ep_data *ep_data, uint32_t reg)
{
	return MEM32(ep_data->manager_base + reg);
}

void usb23_manager_write(const struct device *dev, struct usb23_ep_data *ep_data, uint32_t reg,
			 uint32_t val)
{
	LOG_DBG("manager: write ep=%02x usb23_manager=0x%08lx offset=0x%08x",
		ep_data->addr, ep_data->manager_base, reg);
	MEM32(ep_data->manager_base + reg) = val;
}

/*
 * Configure the uvcmanager: a custom FPGA core that polls the HWO flag of a TRB to send a new
 * TRB whenever the previous one is completed.
 */
void usb23_manager_enable(const struct device *dev, struct usb23_ep_data *ep_data)
{
	const struct usb23_config *conf = dev->config;
	uint32_t reg;

	/* Disable the uvcmanager while it is being configured */
	usb23_manager_write(dev, ep_data, USB23_MANAGER_CONTROLSTATUS, 0);

	/* Let uvcmanager know about the address of the TRB it uses */
	usb23_manager_write(dev, ep_data, USB23_MANAGER_TRBADDR, (uint32_t)ep_data->trb_buf);

	/* Configure the TRB trigger address of the USB23 core so it can enqueue TRBs */
	reg = conf->base + USB23_DEPCMD(ep_data->epn);
	usb23_manager_write(dev, ep_data, USB23_MANAGER_DOORBELLADDR, reg);

	/* Write the address of the FIFO stream that will be pulled by the uvcmanager */
	usb23_manager_write(dev, ep_data, USB23_MANAGER_STREAMADDRESS, ep_data->manager_fifo);

	/* Enable the IRQs */
	reg = USB23_MANAGER_IRQMASK_MASK;
	reg &= ~USB23_MANAGER_IRQMASK_USB;
	reg &= ~USB23_MANAGER_IRQMASK_IRQ;
	reg &= ~USB23_MANAGER_IRQMASK_EOF;
	usb23_manager_write(dev, ep_data, USB23_MANAGER_IRQMASK, 0);
}

/*
 * Input/Output
 *
 * Communication with the USB23 register interface through memory addresses
 * The USB23 core has a register interface and a DMA interface memory.
 * These helpers are the only places where I/O is done with it.
 */

static uint32_t usb23_io_read(const struct device *dev, uint32_t addr)
{
	const struct usb23_config *conf = dev->config;

	return MEM32(conf->base + addr);
}

static void usb23_io_write(const struct device *dev, uint32_t addr, uint32_t data)
{
	const struct usb23_config *conf = dev->config;

	MEM32(conf->base + addr) = data;
}

static void usb23_io_wait_go_low(const struct device *dev, uint32_t addr, uint32_t mask)
{
	while ((usb23_io_read(dev, addr) & mask) != 0)
		;
}

static void usb23_io_set(const struct device *dev, uint32_t addr, uint32_t mask)
{
	usb23_io_write(dev, addr, usb23_io_read(dev, addr) | mask);
}

static void usb23_io_clear(const struct device *dev, uint32_t addr, uint32_t mask)
{
	usb23_io_write(dev, addr, usb23_io_read(dev, addr) & ~mask);
}

static void usb23_io_field(const struct device *dev, uint32_t addr, uint32_t mask, uint32_t val)
{
	usb23_io_write(dev, addr, (usb23_io_read(dev, addr) & ~mask) | val);
}

/*
 * Ring buffer
 *
 * Helpers to operate the TRB and event ring buffers, shared with the hardware.
 */

static void usb23_push_trb(const struct device *dev, struct usb23_ep_data *ep_data,
			   struct net_buf *buf, uint32_t ctrl)
{
	volatile struct usb23_trb *trb = &ep_data->trb_buf[ep_data->head];

	/* If the next TRB in the chain is still owned by the hardware, need
	 * to retry later when more resources become available. */
	__ASSERT_NO_MSG(!ep_data->full);

	/* Associate an active buffer and a TRB together */
	ep_data->net_buf[ep_data->head] = buf;

	/* TRB# with one more chunk of data */
	trb->ctrl = ctrl;
	trb->addr_lo = (uintptr_t)buf->data;
	trb->status = USB_EP_DIR_IS_IN(ep_data->addr) ? buf->len : buf->size;
	LOG_DBG("PUSH %u buf=%p data=%p size=%u", ep_data->head, buf, buf->data, buf->size);

	/* Shift the head */
	ep_data->head = (ep_data->head + 1) % (ep_data->num_of_trbs - 1);

	/* If the head touches the tail after we add something, we are full */
	ep_data->full = (ep_data->head == ep_data->tail);
}

static struct net_buf *usb23_pop_trb(const struct device *dev, struct usb23_ep_data *ep_data)
{
	struct net_buf *buf = NULL;

	/* Skip all the buffer left emtpy by the uvcmanager */
	for (int i = 0;i < ep_data->num_of_trbs && buf == NULL; i++) {
		/* Collect the buffer and clear it from the list */
		buf = ep_data->net_buf[ep_data->tail];
		ep_data->net_buf[ep_data->tail] = NULL;

		LOG_DBG("POP %u ep=0x%02x buf=%p", ep_data->addr, ep_data->tail, buf);

		/* Move to the next position in the ring buffer */
		ep_data->tail = (ep_data->tail + 1) % (ep_data->num_of_trbs - 1);
	}
	if (buf == NULL) {
		LOG_ERR("pop: trying to pop empty stack");
		usb23_dump_all(dev, ep_data, NULL);
		return NULL;
	}

	/* If we just pulled a TRB, we know we made one hole and we are not full anymore */
	ep_data->full = false;

	return buf;
}

/*
 * Commands
 *
 * The DEPCMD register acts as a command interface, where a command number
 * is written along with parameters, an action is performed and a CMDACT bit
 * is reset whenever the command completes.
 */

static uint32_t usb23_depcmd(const struct device *dev, uint32_t addr, uint32_t cmd)
{
	uint32_t reg;

	usb23_io_write(dev, addr, cmd | USB23_DEPCMD_CMDACT);
	do {
		reg = usb23_io_read(dev, addr);
	} while ((reg & USB23_DEPCMD_CMDACT) != 0);

	switch (reg & USB23_DEPCMD_STATUS_MASK) {
	case USB23_DEPCMD_STATUS_OK:
		break;
	case USB23_DEPCMD_STATUS_CMDERR:
		LOG_ERR("epcmd: endpoint command failed");
		break;
	default:
		LOG_ERR("epcmd: command failed with unknown status: 0x%08x", reg);
	}

	return GETFIELD(reg, USB23_DEPCMD_XFERRSCIDX);
}

static void usb23_depcmd_ep_config(const struct device *dev, struct usb23_ep_data *ep_data)
{
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep_data->addr);
	uint32_t param0 = 0, param1 = 0;

	LOG_INF("epcmd: configuring endpoint 0x%02x with wMaxPacketSize=%u", ep_data->addr, ep_cfg->mps);

	if (ep_cfg->stat.enabled) {
		param0 |= USB23_DEPCMDPAR0_DEPCFG_ACTION_MODIFY;
	} else {
		param0 |= USB23_DEPCMDPAR0_DEPCFG_ACTION_INIT;
	}

	switch (ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_CONTROL:
		param0 |= USB23_DEPCMDPAR0_DEPCFG_EPTYPE_CTRL;
		break;
	case USB_EP_TYPE_BULK:
		param0 |= USB23_DEPCMDPAR0_DEPCFG_EPTYPE_BULK;
		break;
	case USB_EP_TYPE_INTERRUPT:
		param0 |= USB23_DEPCMDPAR0_DEPCFG_EPTYPE_INT;
		break;
	case USB_EP_TYPE_ISO:
		param0 |= USB23_DEPCMDPAR0_DEPCFG_EPTYPE_ISOC;
		break;
	default:
		__ASSERT_NO_MSG(false);
	}

	/* Max Packet Size according to the USB descriptor configuration */
	param0 |= ep_cfg->mps << USB23_DEPCMDPAR0_DEPCFG_MPS_SHIFT;

	/* Burst Size of a single packet per burst (encoded as '0'): no
	 * burst */
	param0 |= 15 << USB23_DEPCMDPAR0_DEPCFG_BRSTSIZ_SHIFT;

	/* Set the FIFO number, must be 0 for all OUT EPs */
	if (USB_EP_DIR_IS_IN(ep_data->addr)) {
		param0 |= (ep_data->addr & 0b01111111) << USB23_DEPCMDPAR0_DEPCFG_FIFONUM_SHIFT;
	}

	/* Per-endpoint events */
	param1 |= USB23_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
	param1 |= USB23_DEPCMDPAR1_DEPCFG_XFERCMPLEN;
	// param1 |= USB23_DEPCMDPAR1_DEPCFG_XFERNRDYEN;

	/* This is the usb protocol endpoint number, but the data encoding
	 * we chose for physical endpoint number is the same as this
	 * register */
	param1 |= ep_data->epn << USB23_DEPCMDPAR1_DEPCFG_EPNUMBER_SHIFT;

	usb23_io_write(dev, USB23_DEPCMDPAR0(ep_data->epn), param0);
	usb23_io_write(dev, USB23_DEPCMDPAR1(ep_data->epn), param1);
	usb23_depcmd(dev, USB23_DEPCMD(ep_data->epn), USB23_DEPCMD_DEPCFG);
}

static void usb23_depcmd_ep_xfer_config(const struct device *dev, struct usb23_ep_data *ep_data)
{
	LOG_DBG("epcmd: DepXferConfig: ep=0x%02x", ep_data->addr);
	usb23_io_write(dev, USB23_DEPCMDPAR0(ep_data->epn),
		       1 << USB23_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_SHIFT);
	usb23_depcmd(dev, USB23_DEPCMD(ep_data->epn), USB23_DEPCMD_DEPXFERCFG);
}

#if 0 /* useful for hibernating */
static uint32_t usb23_depcmd_ep_get_state(const struct device *dev, struct usb23_ep_data *ep_data)
{
	LOG_DBG("epcmd: DepGetState: ep=0x%02x", ep_data->addr);

	usb23_depcmd(dev, USB23_DEPCMD(ep_data->epn), USB23_DEPCMD_DEPGETSTATE);
	return usb23_io_read(dev, USB23_DEPCMDPAR2(ep_data->epn));
}
#endif

static void usb23_depcmd_set_stall(const struct device *dev, struct usb23_ep_data *ep_data)
{
	LOG_WRN("epcmd: DepSetStall: ep=0x%02x", ep_data->addr);

	usb23_depcmd(dev, USB23_DEPCMD(ep_data->epn), USB23_DEPCMD_DEPSETSTALL);
}

static void usb23_depcmd_clear_stall(const struct device *dev, struct usb23_ep_data *ep_data)
{
	LOG_INF("epcmd: DepClearStall ep=0x%02x", ep_data->addr);

	usb23_depcmd(dev, USB23_DEPCMD(ep_data->epn), USB23_DEPCMD_DEPCSTALL);
}

static void usb23_depcmd_start_xfer(const struct device *dev, struct usb23_ep_data *ep_data)
{
	/* Make sure the device is in U0 state, assuming TX FIFO is empty */
	usb23_io_field(dev, USB23_DCTL, USB23_DCTL_ULSTCHNGREQ_MASK,
		       USB23_DCTL_ULSTCHNGREQ_REMOTEWAKEUP);

	usb23_io_write(dev, USB23_DEPCMDPAR0(ep_data->epn), HI32((uintptr_t)ep_data->trb_buf));
	usb23_io_write(dev, USB23_DEPCMDPAR1(ep_data->epn), LO32((uintptr_t)ep_data->trb_buf));

	ep_data->xferrscidx =
		usb23_depcmd(dev, USB23_DEPCMD(ep_data->epn), USB23_DEPCMD_DEPSTRTXFER);
	LOG_DBG("epcmd: DepStartXfer done ep=0x%02x xferrscidx=0x%x", ep_data->addr, ep_data->xferrscidx);
}

static void usb23_depcmd_update_xfer(const struct device *dev, struct usb23_ep_data *ep_data)
{
	uint32_t flags;

	flags = ep_data->xferrscidx << USB23_DEPCMD_XFERRSCIDX_SHIFT;
	usb23_depcmd(dev, USB23_DEPCMD(ep_data->epn), USB23_DEPCMD_DEPUPDXFER | flags);
	LOG_DBG("epcmd: DepUpdateXfer done ep=0x%02x addr=0x%08x data=0x%08x", ep_data->addr,
		USB23_DEPCMD(ep_data->epn), USB23_DEPCMD_DEPUPDXFER | flags);
}

static void usb23_depcmd_end_xfer(const struct device *dev, struct usb23_ep_data *ep_data,
				  uint32_t flags)
{
	flags |= ep_data->xferrscidx << USB23_DEPCMD_XFERRSCIDX_SHIFT;
	usb23_depcmd(dev, USB23_DEPCMD(ep_data->epn), USB23_DEPCMD_DEPENDXFER | flags);
	LOG_DBG("epcmd: DepEndXfer done ep=0x%02x", ep_data->addr);

	ep_data->head = ep_data->tail = 0;
	ep_data->head = ep_data->tail = 0;
}

static void usb23_depcmd_start_config(const struct device *dev, struct usb23_ep_data *ep_data)
{
	uint32_t flags;

	flags = (USB_EP_IS_CONTROL(ep_data->addr) ? 0 : 2) << USB23_DEPCMD_XFERRSCIDX_SHIFT;
	usb23_depcmd(dev, USB23_DEPCMD(ep_data->epn), USB23_DEPCMD_DEPSTARTCFG | flags);
	LOG_DBG("epcmd: DepStartConfig done ep=0x%02x", ep_data->addr);
}

static void usb23_dgcmd(const struct device *dev, uint32_t cmd)
{
	uint32_t reg;

	usb23_io_write(dev, USB23_DGCMD, cmd);
	do {
		reg = usb23_io_read(dev, USB23_DGCMD);
	} while ((reg & USB23_DEPCMD_CMDACT) != 0);
	LOG_DBG("dgcmd: done: status=0x%08x", reg);

	if ((reg & USB23_DGCMD_STATUS_MASK) != USB23_DGCMD_STATUS_OK) {
		LOG_ERR("dgcmd: failed: status returned is not 'ok'");
	}
}

static void usb23_dgcmd_exit_latency(const struct device *dev, const struct udc_exit_latency *el)
{
	uint32_t reg;
	uint32_t param;

	reg = usb23_io_read(dev, USB23_DCTL);
	param = (reg & USB23_DCTL_INITU2ENA) ? el->u2pel : el->u1pel;
	usb23_io_write(dev, USB23_DGCMDPAR, (param > 125) ? 0 : param);
	usb23_dgcmd(dev, USB23_DGCMD_EXITLATENCY);
}

static int usb23_set_address(const struct device *dev, const uint8_t addr)
{
	LOG_INF("addr: setting to %u", addr);

	/* Configure the new address */
	usb23_io_field(dev, USB23_DCFG, USB23_DCFG_DEVADDR_MASK, addr << USB23_DCFG_DEVADDR_SHIFT);

	/* Re-apply the same endpoint configuration */
	usb23_depcmd_ep_config(dev, usb23_get_ep_data(dev, USB_CONTROL_EP_OUT));
	usb23_depcmd_ep_config(dev, usb23_get_ep_data(dev, USB_CONTROL_EP_IN));

	return 0;
}

/*
 * Transfer Requests (TRB)
 *
 * USB23 receives transfer requests from this driver through a shared memory
 * buffer, resubmitted upon every new transfer (through either Start or
 * Update command).
 */

static void usb23_trb_norm_init(const struct device *dev, struct usb23_ep_data *ep_data)
{
	volatile struct usb23_trb *trb = ep_data->trb_buf;
	size_t i = ep_data->num_of_trbs - 1;

	LOG_DBG("trb: normal: init");

	/* TRB0 that blocks the transfer from going further */
	trb[0].ctrl = 0;

	/* TRB LINK that loops the ring buffer back to the beginning */
	trb[i].ctrl = USB23_TRB_CTRL_TRBCTL_LINK_TRB | USB23_TRB_CTRL_HWO;
	trb[i].addr_lo = (uintptr_t)ep_data->trb_buf;

	/* Start the transfer now, update it later */
	usb23_depcmd_start_xfer(dev, ep_data);
}

static void usb23_trb_ctrl_in(const struct device *dev, uint32_t ctrl)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, USB_CONTROL_EP_IN);
	struct net_buf *buf = ep_data->net_buf[0];
	volatile struct usb23_trb *trb = ep_data->trb_buf;

	/* TRB0 sending the data */
	trb[0].addr_lo = (uintptr_t)buf->data;
	trb[0].status = buf->len;
	trb[0].ctrl = ctrl | USB23_TRB_CTRL_LST | USB23_TRB_CTRL_HWO;

	/* Start a new transfer every time: no ring buffer */
	usb23_depcmd_start_xfer(dev, ep_data);
}

static void usb23_trb_ctrl_out(const struct device *dev, struct net_buf *buf, uint32_t ctrl)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, USB_CONTROL_EP_OUT);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	const struct usb23_config *conf = dev->config;
	volatile struct usb23_trb *trb = ep_data->trb_buf;

	__ASSERT_NO_MSG(buf != NULL);

	/* Associate the buffer with the TRB for picking it up later */
	__ASSERT_NO_MSG(ep_data->net_buf[0] == NULL);
	ep_data->net_buf[0] = buf;

	/* TRB0 for recieinving the data */
	trb[0].addr_lo = (uintptr_t)ep_data->net_buf[0]->data;
	trb[0].status = buf->size;
	trb[0].ctrl = ctrl | USB23_TRB_CTRL_CHN | USB23_TRB_CTRL_HWO;

	/* TRB1 discarding any overflow data */
	trb[1].addr_lo = conf->discard;
	trb[1].status = ep_cfg->mps - buf->size;
	trb[1].ctrl = ctrl | USB23_TRB_CTRL_LST | USB23_TRB_CTRL_HWO;

	/* Start a new transfer every time: no ring buffer */
	usb23_depcmd_start_xfer(dev, ep_data);
}

static void usb23_trb_ctrl_setup_out(const struct device *dev)
{
	struct net_buf *buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8);

	LOG_DBG("trb: CONTROL_SETUP ep=0x%02x size=%u", USB_CONTROL_EP_OUT, buf->size);
	usb23_trb_ctrl_out(dev, buf, USB23_TRB_CTRL_TRBCTL_CONTROL_SETUP);
}

static void usb23_trb_ctrl_data_out(const struct device *dev)
{
	struct usb23_data *priv = udc_get_private(dev);
	struct net_buf *buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, priv->data_stage_length);

	LOG_DBG("trb: CONTROL_DATA_OUT ep=0x%02x", USB_CONTROL_EP_OUT);
	usb23_trb_ctrl_out(dev, buf, USB23_TRB_CTRL_TRBCTL_CONTROL_DATA);
}

static void usb23_trb_ctrl_data_in(const struct device *dev)
{
	LOG_DBG("trb: CONTROL_DATA_IN ep=0x%02x", USB_CONTROL_EP_IN);
	usb23_trb_ctrl_in(dev, USB23_TRB_CTRL_TRBCTL_CONTROL_DATA);
}

static void usb23_trb_ctrl_status_2_in(const struct device *dev)
{
	LOG_DBG("trb: CONTROL_STATUS_2_IN ep=0x%02x", USB_CONTROL_EP_IN);
	usb23_trb_ctrl_in(dev, USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
}

static void usb23_trb_ctrl_status_3_in(const struct device *dev)
{
	LOG_DBG("trb: CONTROL_STATUS_3_IN ep=0x%02x", USB_CONTROL_EP_IN);
	usb23_trb_ctrl_in(dev, USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
}

static void usb23_trb_ctrl_status_3_out(const struct device *dev)
{
	struct net_buf *buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 0);

	LOG_DBG("trb: CONTROL_STATUS_3_OUT ep=0x%02x", USB_CONTROL_EP_OUT);
	usb23_trb_ctrl_out(dev, buf, USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
}

static int usb23_trb_bulk_native(const struct device *dev, struct usb23_ep_data *ep_data,
				 struct net_buf *buf)
{
	uint32_t ctrl = USB23_TRB_CTRL_IOC | USB23_TRB_CTRL_HWO;

	LOG_DBG("trb: BULK_NATIVE ep=0x%02x buf=%p data=%p size=%u len=%u", ep_data->addr, buf,
		buf->data, buf->size, buf->len);

	if (udc_ep_buf_has_zlp(buf)) {
		/* Mark the TRB as end of a chain: CHN=0 */
		ctrl |= USB23_TRB_CTRL_TRBCTL_NORMAL_ZLP;
	} else {
		/* Mark the next TRB as being part of the same USB transfer */
		ctrl |= USB23_TRB_CTRL_TRBCTL_NORMAL;
		ctrl |= USB_EP_DIR_IS_IN(ep_data->addr) ? USB23_TRB_CTRL_CHN : USB23_TRB_CTRL_CSP;
	}

	/* Let the software driver do the work: enable reception of completion events */
	if (ep_data->full) {
		return -EBUSY;
	}
	usb23_push_trb(dev, ep_data, buf, ctrl);

	/* Update the transfer ourselves with the new transfer descriptor */
	usb23_depcmd_update_xfer(dev, ep_data);

	return 0;
}

static int usb23_trb_bulk_manager(const struct device *dev, struct usb23_ep_data *ep_data,
				  struct net_buf *buf)
{
	uint32_t reg;

	if (!USB_EP_DIR_IS_IN(ep_data->addr)) {
		LOG_ERR("trb: uvcmanager only supports input direction");
		return -EINVAL;
	}
	if (buf->len % 8 != 0) {
		LOG_ERR("trb: uvcmanager assumes transfer sizes to be multiple of 8");
		return -EINVAL;
	}

	LOG_DBG("trb: BULK_MANAGER ep=0x%02x data=%p len=%u", ep_data->addr, buf->data, buf->len);

	/* Disable the uvcmanager when configuring it */
	usb23_manager_write(dev, ep_data, USB23_MANAGER_CONTROLSTATUS, 0);

	/* Number of bytes that must be ready before the uvcmanager enqueues a new TRB */
	usb23_manager_write(dev, ep_data, USB23_MANAGER_FIFOTHRESHOLD, 2048);

	/* Configure as a NORMAL TRB, letting the USB Manager decide when set the CHN field */
	usb23_manager_write(dev, ep_data, USB23_MANAGER_TRBCTRL, USB23_TRB_CTRL_TRBCTL_NORMAL);

	/* Configure the TRB trigger data from the XFERRSCIDX result of the command */
	reg = USB23_DEPCMD_DEPUPDXFER | (ep_data->xferrscidx << USB23_DEPCMD_XFERRSCIDX_SHIFT);
	usb23_manager_write(dev, ep_data, USB23_MANAGER_DOORBELLDATA, reg);

	/* Configure the length according to the incoming net buf */
	usb23_manager_write(dev, ep_data, USB23_MANAGER_NUMBYTES, buf->len);

	/* Let the uvcmanager enqueue until the FIFO is empty */
	__ASSERT_NO_MSG((uintptr_t)buf->data == ep_data->manager_fifo);
	usb23_manager_write(dev, ep_data, USB23_MANAGER_STREAMADDRESS, ep_data->manager_fifo);

	/* Fill the payload header */
	usb23_manager_write(dev, ep_data, USB23_MANAGER_HEADER0, ep_data->manager_header[0]);
	usb23_manager_write(dev, ep_data, USB23_MANAGER_HEADER1, ep_data->manager_header[1]);

	/* Synchronize the current position between the driver and uvcmanager then let the
	 * uvcmanager enqueue until the FIFO is empty */
	reg = USB23_MANAGER_CONTROLSTATUS_ENABLE;
	reg |= USB23_MANAGER_CONTROLSTATUS_CONTINUE;
	reg |= (ep_data->head << USB23_MANAGER_CONTROLSTATUS_TRBID_SHIFT);
	usb23_manager_write(dev, ep_data, USB23_MANAGER_CONTROLSTATUS, reg);

	/* Keep the buffer of the uvcmanager for later */
	ep_data->manager_buf = buf;
	return 0;
}

static int usb23_trb_bulk_manager_header(const struct device *dev, struct usb23_ep_data *ep_data,
				  struct net_buf *buf)
{
	int ret;

	LOG_DBG("trb: BULK_MANAGER_HEADER ep=0x%02x buf=%p data=%p size=%u len=%u", ep_data->addr, buf,
		buf->data, buf->size, buf->len);

	if (!USB_EP_DIR_IS_IN(ep_data->addr)) {
		LOG_ERR("trb: uvcmanager header only supported for output direction");
		return -EINVAL;
	}

	if (buf->len != sizeof(uint64_t)) {
		LOG_ERR("trb: uvcmanager header only supports uint64_t sized buffer");
		return -EINVAL;
	}

	ep_data->manager_header[0] = *(uint32_t *)(buf->data + sizeof(uint32_t));
	ep_data->manager_header[1] = *(uint32_t *)(buf->data + 0);

	/* Submit the buffer completed by the uvcmanager back to Zephyr */
	ret = udc_submit_ep_event(dev, buf, 0);
	__ASSERT_NO_MSG(ret == 0);

	return 0;
}

static int usb23_trb_bulk(const struct device *dev, struct usb23_ep_data *ep_data,
			  struct net_buf *buf)
{
	/* If the uvcmanager is configured for this endpoint */
	if (ep_data->manager_base != 0) {
		if (ep_data->manager_buf != NULL) {
			LOG_DBG("trb: abort: uvcmanager is busy");
			return -EAGAIN;
		}

		/* If we submit a buffer that goes for uvcmanager endpoint address. */
		if ((uintptr_t)buf->data == ep_data->manager_fifo) {
			/* Use the uvcmanager as an accelerator */
			return usb23_trb_bulk_manager(dev, ep_data, buf);
		} else {
			/* Consider it as header and pass it first */
			return usb23_trb_bulk_manager_header(dev, ep_data, buf);
		}
	} else {
		/* Submit a native TRB without using RTL in the wayto manage the transfer */
		return usb23_trb_bulk_native(dev, ep_data, buf);
	}
}

static void usb23_enqueue_buf(const struct device *dev, struct usb23_ep_data *ep_data,
			      struct net_buf *buf)
{
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep_data->addr);
	int ret;

	/* Do not interfer with the uvcmanager operation */
	if (ep_cfg->stat.halted || ep_data->manager_buf) {
		udc_buf_put(ep_cfg, buf);
		return;
	}

	/* Try to enqueue, and if full, send to FIFO instead */
	ret = usb23_trb_bulk(dev, ep_data, buf);
	if (ret < 0) {
		udc_buf_put(ep_cfg, buf);
		return;
	}
}

void usb23_process_queue(const struct device *dev, struct usb23_ep_data *ep_data)
{
	struct net_buf *buf;
	int ret;

	LOG_DBG("queue: checking for pending transfers");

	/* Do not interfer with the uvcmanager operation */
	if (ep_data->manager_buf) {
		LOG_DBG("queue: abort: not interfering with uvcmanager");
		return;
	}

	while ((buf = udc_buf_peek(dev, ep_data->addr)) != NULL) {
		LOG_DBG("processing buffer %p from queue", buf);

		ret = usb23_trb_bulk(dev, ep_data, buf);
		if (ret != 0) {
			LOG_DBG("queue: abort: no more room for buffer");
			break;
		}

		LOG_DBG("queue: success: buffer enqueued");
		udc_buf_get(dev, ep_data->addr);
	} 
	LOG_DBG("queue: done");
}

/*
 * Events
 *
 * Process the events from the event ring buffer. Interrupts gives us a
 * hint that an event is available, which we fetch from a ring buffer shared
 * with the hardware.
 */

/**
 * After did run and enqueued a few TRBs, we need to pick-up the ring buffer where the USB23
 * Manager did stop so that we can pursue at the right position.
 */
static void usb23_on_manager_done(const struct device *dev, struct usb23_ep_data *ep_data)
{
	struct net_buf *buf = ep_data->manager_buf;
	uint32_t reg;
	uint32_t next;
	int ret;

	/* Check that uvcmanager just ran and that the driver did catch-up */
	if (buf == NULL) {
		return;
	}

	LOG_EVENT(MANAGER_DONE);

	/* Sync the position in the ring buffer with the uvcmanager */
	reg = usb23_manager_read(dev, ep_data, USB23_MANAGER_CONTROLSTATUS);
	next = (GETFIELD(reg, USB23_MANAGER_CONTROLSTATUS_TRBID) + 1) % (ep_data->num_of_trbs - 1);

	/* Skip all the TRBs submitted by the uvcmanager */
	ep_data->head = next;

	/* The uvcmanager buffer was submitted as a normal TRB and will be picked-up later */
	ep_data->manager_buf = NULL;

	/* Submit the buffer completed by the uvcmanager back to Zephyr */
	ret = udc_submit_ep_event(dev, buf, 0);
	__ASSERT_NO_MSG(ret == 0);

	/* Walk through the list of buffer to enqueue we might have blocked */
	k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);

	LOG_EVENT(*);
}

static void usb23_on_soft_reset(const struct device *dev)
{
	const struct usb23_config *conf = dev->config;
	uint32_t reg;

	/* Configure and reset the Device Controller */
	/* TODO confirm that DWC_USB3_EN_LPM_ERRATA == 1 */
	usb23_io_write(dev, USB23_DCTL,
		       USB23_DCTL_CSFTRST | (15 << USB23_DCTL_LPM_NYET_THRES_SHIFT));
	usb23_io_wait_go_low(dev, USB23_DCTL, USB23_DCTL_CSFTRST);

	/* Enable AXI64 bursts for various sizes expected */
	usb23_io_set(dev, USB23_GSBUSCFG0,
		     USB23_GSBUSCFG0_INCR256BRSTENA | USB23_GSBUSCFG0_INCR128BRSTENA |
			     USB23_GSBUSCFG0_INCR64BRSTENA | USB23_GSBUSCFG0_INCR32BRSTENA |
			     USB23_GSBUSCFG0_INCR16BRSTENA | USB23_GSBUSCFG0_INCR8BRSTENA |
			     USB23_GSBUSCFG0_INCR4BRSTENA);

	/* Letting GTXTHRCFG and GRXTHRCFG unchanged */
	usb23_io_write(dev, USB23_GTXTHRCFG,
		       USB23_GTXTHRCFG_USBTXPKTCNTSEL | (1 << USB23_GTXTHRCFG_USBTXPKTCNT_SHIFT) |
			       (2 << USB23_GTXTHRCFG_USBMAXTXBURSTSIZE_SHIFT));

	/* Read the chip identification */
	reg = usb23_io_read(dev, USB23_GCOREID);
	LOG_INF("event: coreid=0x%04lx rel=0x%04lx", GETFIELD(reg, USB23_GCOREID_CORE),
		GETFIELD(reg, USB23_GCOREID_REL));
	__ASSERT_NO_MSG(GETFIELD(reg, USB23_GCOREID_CORE) == 0x5533);

	/* Letting GUID unchanged */
	/* Letting GUSB2PHYCFG and GUSB3PIPECTL unchanged */

	/* Setting fifo size for both TX and RX, experimental values
	 * GRXFIFOSIZ too far below or above  512 * 3 leads to errors */
	usb23_io_write(dev, USB23_GTXFIFOSIZ(0), 512 * 3);
	usb23_io_write(dev, USB23_GRXFIFOSIZ(0), 512 * 3);

	/* Setup the event buffer address, size and start event reception */
	memset((void *)conf->evt_buf, 0, CONFIG_USB23_EVT_NUM * sizeof(uint32_t));
	usb23_io_write(dev, USB23_GEVNTADR_LO(0), LO32((uintptr_t)conf->evt_buf));
	usb23_io_write(dev, USB23_GEVNTADR_HI(0), HI32((uintptr_t)conf->evt_buf));
	usb23_io_write(dev, USB23_GEVNTSIZ(0), CONFIG_USB23_EVT_NUM * sizeof(*conf->evt_buf));
	usb23_io_write(dev, USB23_GEVNTCOUNT(0), 0);

	/* Letting GCTL unchanged */

	/* Set the USB device configuration, including max supported speed */
	usb23_io_write(dev, USB23_DCFG, USB23_DCFG_PERFRINT_90);
	switch (conf->speed_idx) {
	case USB23_SPEED_IDX_SUPER_SPEED:
		usb23_io_set(dev, USB23_DCFG, USB23_DCFG_DEVSPD_SUPER_SPEED);
		break;
	case USB23_SPEED_IDX_HIGH_SPEED:
		usb23_io_set(dev, USB23_DCFG, USB23_DCFG_DEVSPD_HIGH_SPEED);
		break;
	case USB23_SPEED_IDX_FULL_SPEED:
		usb23_io_set(dev, USB23_DCFG, USB23_DCFG_DEVSPD_FULL_SPEED);
		break;
	default:
		__ASSERT_NO_MSG(false);
	}
	usb23_io_field(dev, USB23_DCFG, USB23_DCFG_NUMP_MASK, 15 << USB23_DCFG_NUMP_SHIFT);

	/* Enable reception of all USB events except USB23_DEVTEN_ULSTCNGEN */
	usb23_io_write(dev, USB23_DEVTEN,
		       USB23_DEVTEN_INACTTIMEOUTRCVEDEN | USB23_DEVTEN_VNDRDEVTSTRCVEDEN |
			       USB23_DEVTEN_EVNTOVERFLOWEN | USB23_DEVTEN_CMDCMPLTEN |
			       USB23_DEVTEN_ERRTICERREN | USB23_DEVTEN_HIBERNATIONREQEVTEN |
			       USB23_DEVTEN_WKUPEVTEN | USB23_DEVTEN_CONNECTDONEEN |
			       USB23_DEVTEN_USBRSTEN | USB23_DEVTEN_DISCONNEVTEN);

	/* Configure endpoint 0 and 1 only for now */
	usb23_depcmd_start_config(dev, usb23_get_ep_data(dev, USB_CONTROL_EP_OUT));
	usb23_depcmd_start_config(dev, usb23_get_ep_data(dev, USB_CONTROL_EP_IN));
}

static void usb23_on_usb_reset(const struct device *dev)
{
	const struct usb23_config *conf = dev->config;

	/* Reset all ongoing transfers on non-0 endpoints */
	for (uint8_t addr = 1; addr < conf->num_bidir_eps; addr++) {
		struct usb23_ep_data *ep_data;

		continue; /* TODO */

		ep_data = usb23_get_ep_data(dev, 0x00 | addr);
		usb23_depcmd_end_xfer(dev, ep_data, 0);
		usb23_depcmd_clear_stall(dev, ep_data);

		ep_data = usb23_get_ep_data(dev, 0x80 | addr);
		usb23_depcmd_end_xfer(dev, ep_data, 0);
		usb23_depcmd_clear_stall(dev, ep_data);
	}

	/* Perform the USB reset operations manually to improve latency */
	usb23_set_address(dev, 0);

	/* Let Zephyr set the device address 0 */
	udc_submit_event(dev, UDC_EVT_RESET, 0);
}

static void usb23_on_connect_done(const struct device *dev)
{
	int mps = 0;

	/* Adjust parameters against the connection speed */
	switch (usb23_io_read(dev, USB23_DSTS) & USB23_DSTS_CONNECTSPD_MASK) {
	case USB23_DSTS_CONNECTSPD_FS:
	case USB23_DSTS_CONNECTSPD_HS:
		mps = 64;
		/* TODO this is not suspending USB3, it enable suspend feature */
		// usb23_io_set(dev, USB23_GUSB3PIPECTL, USB23_GUSB3PIPECTL_SUSPENDENABLE);
		break;
	case USB23_DSTS_CONNECTSPD_SS:
		mps = 512;
		// usb23_io_set(dev, USB23_GUSB2PHYCFG, USB23_GUSB2PHYCFG_SUSPHY);
		break;
	}
	__ASSERT_NO_MSG(mps != 0);

	/* Reconfigure control endpoints connection speed */
	udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT)->mps = mps;
	udc_get_ep_cfg(dev, USB_CONTROL_EP_IN)->mps = mps;
	usb23_depcmd_ep_config(dev, usb23_get_ep_data(dev, USB_CONTROL_EP_OUT));
	usb23_depcmd_ep_config(dev, usb23_get_ep_data(dev, USB_CONTROL_EP_IN));

	/* Letting GTXFIFOSIZn unchanged */
}

static void usb23_on_link_state_event(const struct device *dev)
{
	uint32_t reg;

	reg = usb23_io_read(dev, USB23_DSTS);

	switch (reg & USB23_DSTS_CONNECTSPD_MASK) {
	case USB23_DSTS_CONNECTSPD_SS:
		switch (reg & USB23_DSTS_USBLNKST_MASK) {
		case USB23_DSTS_USBLNKST_USB3_U0:
			LOG_EVENT(DSTS_USBLNKST_USB3_U0);
			break;
		case USB23_DSTS_USBLNKST_USB3_U1:
			LOG_EVENT(DSTS_USBLNKST_USB3_U1);
			break;
		case USB23_DSTS_USBLNKST_USB3_U2:
			LOG_EVENT(DSTS_USBLNKST_USB3_U2);
			break;
		case USB23_DSTS_USBLNKST_USB3_U3:
			LOG_EVENT(DSTS_USBLNKST_USB3_U3);
			break;
		case USB23_DSTS_USBLNKST_USB3_SS_DIS:
			LOG_EVENT(DSTS_USBLNKST_USB3_SS_DIS);
			break;
		case USB23_DSTS_USBLNKST_USB3_RX_DET:
			LOG_EVENT(DSTS_USBLNKST_USB3_RX_DET);
			break;
		case USB23_DSTS_USBLNKST_USB3_SS_INACT:
			LOG_EVENT(DSTS_USBLNKST_USB3_SS_INACT);
			break;
		case USB23_DSTS_USBLNKST_USB3_POLL:
			LOG_EVENT(DSTS_USBLNKST_USB3_POLL);
			break;
		case USB23_DSTS_USBLNKST_USB3_RECOV:
			LOG_EVENT(DSTS_USBLNKST_USB3_RECOV);
			break;
		case USB23_DSTS_USBLNKST_USB3_HRESET:
			LOG_EVENT(DSTS_USBLNKST_USB3_HRESET);
			break;
		case USB23_DSTS_USBLNKST_USB3_CMPLY:
			LOG_EVENT(DSTS_USBLNKST_USB3_CMPLY);
			break;
		case USB23_DSTS_USBLNKST_USB3_LPBK:
			LOG_EVENT(DSTS_USBLNKST_USB3_LPBK);
			break;
		case USB23_DSTS_USBLNKST_USB3_RESET_RESUME:
			LOG_EVENT(DSTS_USBLNKST_USB3_RESET_RESUME);
			break;
		default:
			LOG_ERR("event: unknown USB3 link state");
		}
		break;
	case USB23_DSTS_CONNECTSPD_HS:
	case USB23_DSTS_CONNECTSPD_FS:
		switch (reg & USB23_DSTS_USBLNKST_MASK) {
		case USB23_DSTS_USBLNKST_USB2_ON_STATE:
			LOG_EVENT(DSTS_USBLNKST_USB2_ON_STATE);
			break;
		case USB23_DSTS_USBLNKST_USB2_SLEEP_STATE:
			LOG_EVENT(DSTS_USBLNKST_USB2_SLEEP_STATE);
			break;
		case USB23_DSTS_USBLNKST_USB2_SUSPEND_STATE:
			LOG_EVENT(DSTS_USBLNKST_USB2_SUSPEND_STATE);
			break;
		case USB23_DSTS_USBLNKST_USB2_DISCONNECTED:
			LOG_EVENT(DSTS_USBLNKST_USB2_DISCONNECTED);
			break;
		case USB23_DSTS_USBLNKST_USB2_EARLY_SUSPEND:
			LOG_EVENT(DSTS_USBLNKST_USB2_EARLY_SUSPEND);
			break;
		case USB23_DSTS_USBLNKST_USB2_RESET:
			LOG_EVENT(DSTS_USBLNKST_USB2_RESET);
			break;
		case USB23_DSTS_USBLNKST_USB2_RESUME:
			LOG_EVENT(DSTS_USBLNKST_USB2_RESUME);
			break;
		default:
			LOG_ERR("event: unknown USB2 link state");
		}
		break;
	default:
		LOG_ERR("event: unknown connection speed");
	}
}

/* Control Write */

/* OUT */
static void usb23_on_ctrl_write_setup(const struct device *dev, struct net_buf *buf)
{
	LOG_DBG("event: CTRL_WRITE_SETUP (out) buf=%p", buf);
	usb23_trb_ctrl_data_out(dev);
}

/* OUT */
static void usb23_on_ctrl_write_data(const struct device *dev, struct net_buf *buf)
{
	int ret;

	LOG_DBG("event: CTRL_WRITE_DATA (out) buf=%p", buf);
	udc_ctrl_update_stage(dev, buf);
	ret = udc_ctrl_submit_s_out_status(dev, buf);
	__ASSERT_NO_MSG(ret == 0);
	k_sleep(K_MSEC(1));
}

/* IN */
static void usb23_on_ctrl_write_status(const struct device *dev, struct net_buf *buf)
{
	int ret;

	LOG_DBG("event: CTRL_WRITE_STATUS (in) buf=%p", buf);
	ret = udc_ctrl_submit_status(dev, buf);
	__ASSERT_NO_MSG(ret == 0);
	udc_ctrl_update_stage(dev, buf);
}

/* Control Read */

/* OUT */
static void usb23_on_ctrl_read_setup(const struct device *dev, struct net_buf *buf)
{
	int ret;

	LOG_DBG("event: CTRL_READ_SETUP (out) buf=%p", buf);
	ret = udc_ctrl_submit_s_in_status(dev);
	__ASSERT_NO_MSG(ret == 0);
}

/* IN */
static void usb23_on_ctrl_read_data(const struct device *dev, struct net_buf *buf)
{
	LOG_DBG("event: CTRL_READ_DATA (in) buf=%p", buf);
	usb23_trb_ctrl_status_3_out(dev);
	udc_ctrl_update_stage(dev, buf);
	net_buf_unref(buf);
}

/* OUT */
static void usb23_on_ctrl_read_status(const struct device *dev, struct net_buf *buf)
{
	int ret;

	LOG_DBG("event: CTRL_READ_STATUS (out) buf=%p", buf);
	ret = udc_ctrl_submit_status(dev, buf);
	__ASSERT_NO_MSG(ret == 0);
	udc_ctrl_update_stage(dev, buf);
}

/* No-Data Control */

/* OUT */
static void usb23_on_ctrl_nodata_setup(const struct device *dev, struct net_buf *buf)
{
	int ret;

	LOG_DBG("event: CTRL_NODATA_SETUP (out) buf=%p", buf);
	ret = udc_ctrl_submit_s_status(dev);
	__ASSERT_NO_MSG(ret == 0);
}

/* IN */
static void usb23_on_ctrl_nodata_status(const struct device *dev, struct net_buf *buf)
{
	int ret;

	LOG_DBG("event: CTRL_NODATA_STATUS (in) buf=%p", buf);
	ret = udc_ctrl_submit_status(dev, buf);
	__ASSERT_NO_MSG(ret == 0);
	udc_ctrl_update_stage(dev, buf);
}

static void usb23_on_ctrl_setup_out(const struct device *dev, struct net_buf *buf)
{
	struct usb23_data *priv = udc_get_private(dev);

	/* Only moment where this information is accessible */
	priv->data_stage_length = udc_data_stage_length(buf);

	/* To be able to differentiate the next stage*/
	udc_ep_buf_set_setup(buf);
	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_data_out(dev)) {
		usb23_on_ctrl_write_setup(dev, buf);
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		usb23_on_ctrl_read_setup(dev, buf);
	} else if (udc_ctrl_stage_is_no_data(dev)) {
		usb23_on_ctrl_nodata_setup(dev, buf);
	} else {
		LOG_ERR("event: unknown setup stage");
	}
}

static void usb23_on_ctrl_in(const struct device *dev)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, USB_CONTROL_EP_IN);
	struct usb23_trb *trb = &ep_data->trb_buf[0];
	struct net_buf *buf = ep_data->net_buf[0];

	/* We are not expected to touch that buffer anymore */
	ep_data->net_buf[0] = NULL;
	__ASSERT_NO_MSG(buf != NULL);

	/* Continue to the next step */
	switch (trb->ctrl & USB23_TRB_CTRL_TRBCTL_MASK) {
	case USB23_TRB_CTRL_TRBCTL_CONTROL_DATA:
		usb23_on_ctrl_read_data(dev, buf);
		break;
	case USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_2:
		usb23_on_ctrl_nodata_status(dev, buf);
		usb23_trb_ctrl_setup_out(dev);
		break;
	case USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_3:
		usb23_on_ctrl_write_status(dev, buf);
		usb23_trb_ctrl_setup_out(dev);
		break;
	default:
		__ASSERT_NO_MSG(false);
		break;
	}
}

static void usb23_on_ctrl_out(const struct device *dev)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, USB_CONTROL_EP_OUT);
	struct usb23_trb *trb = &ep_data->trb_buf[0];
	struct net_buf *buf = ep_data->net_buf[0];

	/* For buffers coming from the host, update the size actually received */
	buf->len = buf->size - GETFIELD(trb->status, USB23_TRB_STATUS_BUFSIZ);

	/* Latency optimization: set the address immediately to be
	 * able to be able to ACK/NAK the first packets from the
	 * host with the new address, otherwise the host issue a
	 * reset */
	if (buf->len > 2 && buf->data[0] == 0x00 && buf->data[1] == 0x05) {
		usb23_set_address(dev, buf->data[2]);
	}

	/* We are not expected to touch that buffer anymore */
	ep_data->net_buf[0] = NULL;
	__ASSERT_NO_MSG(buf != NULL);

	/* Continue to the next step */
	switch (trb->ctrl & USB23_TRB_CTRL_TRBCTL_MASK) {
	case USB23_TRB_CTRL_TRBCTL_CONTROL_SETUP:
		usb23_on_ctrl_setup_out(dev, buf);
		break;
	case USB23_TRB_CTRL_TRBCTL_CONTROL_DATA:
		usb23_on_ctrl_write_data(dev, buf);
		break;
	case USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_3:
		usb23_on_ctrl_read_status(dev, buf);
		usb23_trb_ctrl_setup_out(dev);
		break;
	default:
		__ASSERT_NO_MSG(false);
		break;
	}
}

static void usb23_on_xfer_not_ready(const struct device *dev, uint32_t evt)
{
	switch (evt & USB23_DEPEVT_STATUS_B3_MASK) {
	case USB23_DEPEVT_STATUS_B3_CONTROL_SETUP:
		LOG_EVENT(USB23_DEPEVT_XFERNOTREADY_CONTROL_SETUP);
		break;
	case USB23_DEPEVT_STATUS_B3_CONTROL_DATA:
		LOG_EVENT(USB23_DEPEVT_XFERNOTREADY_CONTROL_DATA);
		break;
	case USB23_DEPEVT_STATUS_B3_CONTROL_STATUS:
		LOG_EVENT(USB23_DEPEVT_XFERNOTREADY_CONTROL_STATUS);
		break;
	}
	return;
}

static void usb23_on_xfer_done(const struct device *dev, struct usb23_ep_data *ep_data)
{
	struct usb23_trb *trb = &ep_data->trb_buf[ep_data->tail];

	switch (trb->status & USB23_TRB_STATUS_TRBSTS_MASK) {
	case USB23_TRB_STATUS_TRBSTS_OK:
		break;
	case USB23_TRB_STATUS_TRBSTS_MISSEDISOC:
		LOG_ERR("USB23_TRB_STATUS_TRBSTS_MISSEDISOC");
		break;
	case USB23_TRB_STATUS_TRBSTS_SETUPPENDING:
		LOG_ERR("USB23_TRB_STATUS_TRBSTS_SETUPPENDING");
		break;
	case USB23_TRB_STATUS_TRBSTS_XFERINPROGRESS:
		LOG_ERR("USB23_TRB_STATUS_TRBSTS_XFERINPROGRESS");
		break;
	case USB23_TRB_STATUS_TRBSTS_ZLPPENDING:
		LOG_ERR("USB23_TRB_STATUS_TRBSTS_ZLPPENDING");
		break;
	default:
		__ASSERT_NO_MSG(false);
	}
}

static void usb23_on_xfer_done_norm(const struct device *dev, uint32_t evt)
{
	uint8_t ep_addr = usb23_get_addr(GETFIELD(evt, USB23_DEPEVT_EPN));
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_addr);
	struct usb23_trb *trb = &ep_data->trb_buf[ep_data->tail];
	struct net_buf *buf;
	int ret;

	/* Clear the TRB that triggered the event */
	buf = usb23_pop_trb(dev, ep_data);
	LOG_DBG("event: XFER_DONE_NORM: ep=0x%02x buf=%p", ep_data->addr, buf);
	__ASSERT_NO_MSG(buf != NULL);
	usb23_on_xfer_done(dev, ep_data);

	/* For buffers coming from the host, update the size actually received */
	if (USB_EP_DIR_IS_OUT(ep_data->addr)) {
		buf->len = buf->size - GETFIELD(trb->status, USB23_TRB_STATUS_BUFSIZ);
	}

	ret = udc_submit_ep_event(dev, buf, 0);
	__ASSERT_NO_MSG(ret == 0);

	/* We just made some room for a new buffer, check if something more to enqueue */
	k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
}

void usb23_on_event(const struct device *dev)
{
	const struct usb23_config *conf = dev->config;
	struct usb23_data *priv = udc_get_private(dev);

	/* Process the USB23 events */
	while (usb23_io_read(dev, USB23_GEVNTCOUNT(0)) > 0) {
		/* Cache the current event */
		uint32_t evt = conf->evt_buf[priv->evt_next];

		switch (evt & USB23_EVT_MASK) {
		case USB23_DEPEVT_XFERCOMPLETE(0):
			LOG_EVENT(DEPEVT_XFERCOMPLETE(0));
			usb23_on_ctrl_out(dev);
			break;
		case USB23_DEPEVT_XFERCOMPLETE(1):
			LOG_EVENT(DEPEVT_XFERCOMPLETE(1));
			usb23_on_ctrl_in(dev);
			break;
		FOREACH_NORMAL_IN_EP(CASE_DEPEVT_XFERCOMPLETE)
		FOREACH_NORMAL_OUT_EP(CASE_DEPEVT_XFERCOMPLETE)
		FOREACH_NORMAL_IN_EP(CASE_DEPEVT_XFERINPROGRESS)
		FOREACH_NORMAL_OUT_EP(CASE_DEPEVT_XFERINPROGRESS)
			LOG_EVENT(DEPEVT_XFERINPROGRESS);
			usb23_on_xfer_done_norm(dev, evt);
			break;
		case USB23_DEPEVT_XFERNOTREADY(0):
		case USB23_DEPEVT_XFERNOTREADY(1):
			usb23_on_xfer_not_ready(dev, evt);
			break;
		case USB23_DEVT_DISCONNEVT:
			LOG_EVENT(DEVT_DISCONNEVT);
			break;
		case USB23_DEVT_USBRST:
			LOG_EVENT(DEVT_USBRST);
			usb23_on_usb_reset(dev);
			break;
		case USB23_DEVT_CONNECTDONE:
			LOG_EVENT(DEVT_CONNECTDONE);
			usb23_on_connect_done(dev);
			break;
		case USB23_DEVT_ULSTCHNG:
			LOG_EVENT(DEVT_ULSTCHNG);
			usb23_on_link_state_event(dev);
			break;
		case USB23_DEVT_WKUPEVT:
			LOG_EVENT(DEVT_WKUPEVT);
			break;
		case USB23_DEVT_SUSPEND:
			LOG_EVENT(DEVT_SUSPEND);
			break;
		case USB23_DEVT_SOF:
			LOG_EVENT(DEVT_SOF);
			break;
		case USB23_DEVT_CMDCMPLT:
			LOG_EVENT(DEVT_CMDCMPLT);
			break;
		case USB23_DEVT_VNDRDEVTSTRCVED:
			LOG_EVENT(DEVT_VNDRDEVTSTRCVED);
			break;
		case USB23_DEVT_ERRTICERR:
			__ASSERT(false, "DEVT_ERRTICERR");
			break;
		case USB23_DEVT_EVNTOVERFLOW:
			__ASSERT(false, "DEVT_EVNTOVERFLOW");
			break;
		default:
			LOG_ERR("unnhandled event: 0x%x", evt);
		}

		if (evt == 0x00000000) {
			LOG_ERR("event: empty event received");
		}

		/* We can already release the resource now that we copied it */
		usb23_io_write(dev, USB23_GEVNTCOUNT(0), sizeof(evt));

		LOG_EVENT(*);

		/* This is a ring buffer, wrap around */
		priv->evt_next++;
		priv->evt_next %= CONFIG_USB23_EVT_NUM;
	}

	/* Process the uvcmanager events */
	for (struct usb23_ep_data **ep_data = conf->manager_list; *ep_data != NULL; ep_data++) {
		uint32_t reg;

		/* Fetch ongoing IRQ flags and acknowledge those we are about to process */
		reg = usb23_manager_read(dev, *ep_data, USB23_MANAGER_CONTROLSTATUS);
		switch (reg & USB23_MANAGER_CONTROLSTATUS_FSMSTATE_MASK) {
		case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_IDLE:
			usb23_on_manager_done(dev, *ep_data);
		}
	}
}

void usb23_irq_handler(void *ptr)
{
	const struct device *dev = ptr;
	const struct usb23_config *conf = dev->config;
	struct usb23_data *priv = udc_get_private(dev);

	conf->irq_clear_func();
	k_work_submit_to_queue(udc_get_work_q(), &priv->work);
}

/*
 * UDC API
 *
 * Interface called by Zehpyr from the upper levels of abstractions.
 */

int usb23_api_ep_enqueue(const struct device *dev, struct udc_ep_config *ep_cfg,
			 struct net_buf *buf)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg->addr);
	struct udc_buf_info *bi = udc_get_buf_info(buf);

	switch (ep_data->addr) {
	case USB_CONTROL_EP_IN:
		/* Save the buffer to fetch it back later */
		__ASSERT_NO_MSG(ep_data->net_buf[0] == NULL);
		ep_data->net_buf[0] = buf;

		if (bi->data) {
			usb23_trb_ctrl_data_in(dev);
		} else if (bi->status && udc_ctrl_stage_is_no_data(dev)) {
			usb23_trb_ctrl_status_2_in(dev);
		} else if (bi->status) {
			usb23_trb_ctrl_status_3_in(dev);
		} else {
			__ASSERT_NO_MSG(false);
		}
		break;
	case USB_CONTROL_EP_OUT:
		__ASSERT(false, "expected to be handled by the driver directly");
		break;
	default:
		usb23_enqueue_buf(dev, ep_data, buf);
	}

	return 0;
}

int usb23_api_ep_dequeue(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;

	lock_key = irq_lock();
	buf = udc_buf_get_all(dev, ep_cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}
	irq_unlock(lock_key);
	return 0;
}

int usb23_api_ep_disable(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg->addr);

	usb23_io_clear(dev, USB23_DALEPENA, BIT(ep_data->epn));
	return 0;
}

/*
 * Halt endpoint. Halted endpoint should respond with a STALL handshake.
 */
int usb23_api_ep_set_halt(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg->addr);

	LOG_WRN("api: setting stall state on endpoint ep=0x%02x", ep_data->addr);

	switch (ep_data->addr) {
	case USB_CONTROL_EP_IN:
		/* Remove the TRBs transfer for the cancelled sequence */
		usb23_depcmd_end_xfer(dev, ep_data, USB23_DEPCMD_HIPRI_FORCERM);

		/* The datasheet says to only set stall for the OUT
		 * direction */
		ep_data = usb23_get_ep_data(dev, USB_CONTROL_EP_OUT);
		/* fallthrough */
	case USB_CONTROL_EP_OUT:
		usb23_depcmd_end_xfer(dev, ep_data, 0);
		usb23_depcmd_set_stall(dev, ep_data);

		/* The hardware will automatically clear the halt state upon
		 * the next setup packet received. */
		usb23_trb_ctrl_setup_out(dev);
		break;
	default:
		usb23_depcmd_set_stall(dev, ep_data);
		ep_cfg->stat.halted = true;
	}

	return 0;
}

int usb23_api_ep_clear_halt(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg->addr);

	LOG_DBG("api: clearing stall state on endpoint ep=0x%02x", ep_data->addr);
	__ASSERT_NO_MSG(ep_data->addr != USB_CONTROL_EP_OUT);
	__ASSERT_NO_MSG(ep_data->addr != USB_CONTROL_EP_IN);

	usb23_depcmd_clear_stall(dev, ep_data);
	ep_cfg->stat.halted = false;
	return 0;
}

int usb23_api_set_address(const struct device *dev, const uint8_t addr)
{
	/* The address is set in the code earlier to improve latency, only
	 * checking that it is still the value done for consistency. */
	if (GETFIELD(usb23_io_read(dev, USB23_DCFG), USB23_DCFG_DEVADDR) != addr) {
		return -EPROTO;
	}
	return 0;
}

int usb23_api_set_exit_latency(const struct device *dev, const struct udc_exit_latency *el)
{
	LOG_DBG("api: u1sel=%u u1pel=%u u2sel=%u u2pel=%u",
		el->u1sel, el->u1pel, el->u2sel, el->u2pel);
	usb23_dgcmd_exit_latency(dev, el);
	return 0;
}

enum udc_bus_speed usb23_api_device_speed(const struct device *dev)
{
	switch (usb23_io_read(dev, USB23_DSTS) & USB23_DSTS_CONNECTSPD_MASK) {
	case USB23_DSTS_CONNECTSPD_HS:
		return UDC_BUS_SPEED_HS;
	case USB23_DSTS_CONNECTSPD_FS:
		return UDC_BUS_SPEED_FS;
	case USB23_DSTS_CONNECTSPD_SS:
		return UDC_BUS_SPEED_SS;
	}
	__ASSERT(0, "unknown device speed");
	return 0;
}

void usb23_enable(const struct device *dev)
{
	const struct usb23_config *conf = dev->config;

	/* Enable the uvcmanager events */
	for (struct usb23_ep_data **ep_data = conf->manager_list; *ep_data != NULL; ep_data++) {
		/* For enabled endpoint, use the uvcmanager to automatically reload the TRBs */
		if ((*ep_data)->manager_base) {
			LOG_DBG("enable: uvcmanager for ep=0x%02x", (*ep_data)->addr);
			usb23_manager_enable(dev, *ep_data);
		}
	}

	/* Bootstrap: prepare reception of the initial Setup packet */
	usb23_trb_ctrl_setup_out(dev);

	/* Enable the USB23 events */
	usb23_io_set(dev, USB23_DCTL, USB23_DCTL_RUNSTOP);
	conf->irq_enable_func();
}

/*
 * Hardware Init
 *
 * Prepare the driver and the hardware to being used.
 * This goes through register configuration and register commands.
 */

int usb23_api_ep_enable(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg->addr);

	memset(ep_data->trb_buf, 0, ep_data->num_of_trbs * sizeof(*ep_data->trb_buf));
	usb23_depcmd_ep_config(dev, ep_data);
	usb23_depcmd_ep_xfer_config(dev, ep_data);

	if (!USB_EP_IS_CONTROL(ep_data->addr)) {
		usb23_trb_norm_init(dev, ep_data);
	}

	/* Starting from here, the endpoint can be used */
	usb23_io_set(dev, USB23_DALEPENA, USB23_DALEPENA_USBACTEP(ep_data->epn));

	return 0;
}

/*
 * Prepare and configure most of the parts, if the controller has a way
 * of detecting VBUS activity it should be enabled here.
 * Only usb23_enable() makes device visible to the host.
 */
int usb23_api_init(const struct device *dev)
{
	struct udc_data *data = dev->data;
	int ret;

	/* Issue a soft reset to the core and USB2 and USB3 PHY */
	usb23_io_set(dev, USB23_GCTL, USB23_GCTL_CORESOFTRESET);
	usb23_io_set(dev, USB23_GUSB3PIPECTL, USB23_GUSB3PIPECTL_PHYSOFTRST);
	usb23_io_set(dev, USB23_GUSB2PHYCFG, USB23_GUSB2PHYCFG_PHYSOFTRST);
	k_sleep(K_USEC(1000)); /* TODO: reduce amount of wait time */

	/* Teriminate the reset of the USB2 and USB3 PHY first */
	usb23_io_clear(dev, USB23_GUSB3PIPECTL, USB23_GUSB3PIPECTL_PHYSOFTRST);
	usb23_io_clear(dev, USB23_GUSB2PHYCFG, USB23_GUSB2PHYCFG_PHYSOFTRST);

	/* Teriminate the reset of the USB23 core after it */
	usb23_io_clear(dev, USB23_GCTL, USB23_GCTL_CORESOFTRESET);

	/* Initialize USB2 PHY Lattice wrappers */
	usb23_io_set(dev, USB23_U2PHYCTRL1, USB23_U2PHYCTRL1_SEL_INTERNALCLK);
	usb23_io_set(dev, USB23_U2PHYCTRL2, USB23_U2PHYCTRL2_REFCLK_SEL);

	/* Initialize USB3 PHY Lattice wrappers */
	usb23_io_set(dev, USB23_U3PHYCTRL1, BIT(22));
	usb23_io_clear(dev, USB23_U3PHYCTRL4, USB23_U3PHYCTRL4_INT_CLOCK);

	/* The USB core was reset, configure it as documented */
	usb23_on_soft_reset(dev);

	/* Configure the control OUT endpoint */
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, data->caps.mps0,
				     0);
	__ASSERT_NO_MSG(ret == 0);

	/* Configure the control IN endpoint */
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, data->caps.mps0,
				     0);
	__ASSERT_NO_MSG(ret == 0);

	return ret;
}

/*
 * Shell commands
 *
 * Debug commands that can be accessed through the UART shell to
 * query the internal state of the driver or hardware.
 */

struct usb23_reg {
	uint32_t addr;
	char *name;
	uint32_t last;
} usb23_regs[] = {
#define R(reg)                                                                                     \
	{                                                                                          \
		.addr = USB23_##reg, .name = #reg, .last = 0                                       \
	}

	/* main registers */
	R(GCTL),
	R(DCTL),
	R(DCFG),
	R(DEVTEN),
	R(DALEPENA),
	R(GCOREID),
	R(GSTS),
	R(DSTS),
	R(GEVNTADR_LO(0)),
	R(GEVNTADR_HI(0)),
	R(GEVNTSIZ(0)),
	R(GEVNTCOUNT(0)),
	R(GUSB2PHYCFG),
	R(GUSB3PIPECTL),

	/* debug */
	R(GBUSERRADDR_LO),
	R(GBUSERRADDR_HI),
	R(CTLDEBUG_LO),
	R(CTLDEBUG_HI),
	R(ANALYZERTRACE),
	R(GDBGFIFOSPACE),
	R(GDBGLTSSM),
	R(GDBGLNMCC),
	R(GDBGBMU),
	R(GDBGLSPMUX_DEV),
	R(GDBGLSPMUX_HST),
	R(GDBGLSP),
	R(GDBGEPINFO0),
	R(GDBGEPINFO1),
	R(BU3RHBDBG0),

	/* undocumented registers from Lattice */
	R(U2PHYCTRL0),
	R(U2PHYCTRL1),
	R(U2PHYCTRL2),
	R(U3PHYCTRL0),
	R(U3PHYCTRL1),
	R(U3PHYCTRL2),
	R(U3PHYCTRL3),
	R(U3PHYCTRL4),
	R(U3PHYCTRL5),

	/* physical endpoint numbers */
	R(DEPCMDPAR2(0)),
	R(DEPCMDPAR1(0)),
	R(DEPCMDPAR0(0)),
	R(DEPCMD(0)),
	R(DEPCMDPAR2(1)),
	R(DEPCMDPAR1(1)),
	R(DEPCMDPAR0(1)),
	R(DEPCMD(1)),
	R(DEPCMDPAR2(2)),
	R(DEPCMDPAR1(2)),
	R(DEPCMDPAR0(2)),
	R(DEPCMD(2)),
	R(DEPCMDPAR2(3)),
	R(DEPCMDPAR1(3)),
	R(DEPCMDPAR0(3)),
	R(DEPCMD(3)),
	R(DEPCMDPAR2(4)),
	R(DEPCMDPAR1(4)),
	R(DEPCMDPAR0(4)),
	R(DEPCMD(4)),
	R(DEPCMDPAR2(5)),
	R(DEPCMDPAR1(5)),
	R(DEPCMDPAR0(5)),
	R(DEPCMD(5)),
	R(DEPCMDPAR2(6)),
	R(DEPCMDPAR1(6)),
	R(DEPCMDPAR0(6)),
	R(DEPCMD(6)),
	R(DEPCMDPAR2(7)),
	R(DEPCMDPAR1(7)),
	R(DEPCMDPAR0(7)),
	R(DEPCMD(7)),

	/* Hardware parameters */
	R(GHWPARAMS0),
	R(GHWPARAMS1),
	R(GHWPARAMS2),
	R(GHWPARAMS3),
	R(GHWPARAMS4),
	R(GHWPARAMS5),
	R(GHWPARAMS6),
	R(GHWPARAMS7),
	R(GHWPARAMS8),

#undef R
};

#define SHELL_OR_LOG(sh, ...)                                                                      \
	if (sh) {                                                                                  \
		shell_print(sh, __VA_ARGS__);                                                      \
	} else {                                                                                   \
		LOG_ERR(__VA_ARGS__);                                                              \
	}

void usb23_dump_registers(const struct device *dev, const struct shell *sh)
{
	for (int i = 0; i < ARRAY_SIZE(usb23_regs); i++) {
		struct usb23_reg *ureg = &usb23_regs[i];
		uint32_t data;

		data = usb23_io_read(dev, ureg->addr);
		if (data != ureg->last) {
			SHELL_OR_LOG(sh, "reg 0x%08x == 0x%08x  (was 0x%08x)  // %s", ureg->addr,
				    data, ureg->last, ureg->name);
			ureg->last = data;
		}
	}
}

void usb23_dump_bus_error(const struct device *dev, const struct shell *sh)
{
	if (usb23_io_read(dev, USB23_GSTS) & USB23_GSTS_BUSERRADDRVLD) {
		SHELL_OR_LOG(sh, "BUS_ERROR addr=0x%08x%08x",
			    usb23_io_read(dev, USB23_GBUSERRADDR_HI),
			    usb23_io_read(dev, USB23_GBUSERRADDR_LO));
	} else {
		SHELL_OR_LOG(sh, "no bus error");
	}
}

void usb23_dump_link_state(const struct device *dev, const struct shell *sh)
{
	uint32_t reg;

	reg = usb23_io_read(dev, USB23_DSTS);

	switch (reg & USB23_DSTS_CONNECTSPD_MASK) {
	case USB23_DSTS_CONNECTSPD_HS:
		SHELL_OR_LOG(sh, "USB23_DSTS_CONNECTSPD_HS");
		goto usb2;
	case USB23_DSTS_CONNECTSPD_FS:
		SHELL_OR_LOG(sh, "USB23_DSTS_CONNECTSPD_FS");
		goto usb2;
	case USB23_DSTS_CONNECTSPD_SS:
		SHELL_OR_LOG(sh, "USB23_DSTS_CONNECTSPD_SS");
		goto usb3;
	default:
		shell_error(sh, "unknown speed");
	}
	return;
usb2:
	switch (reg & USB23_DSTS_USBLNKST_MASK) {
	case USB23_DSTS_USBLNKST_USB2_ON_STATE:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB2_ON_STATE");
		break;
	case USB23_DSTS_USBLNKST_USB2_SLEEP_STATE:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB2_SLEEP_STATE");
		break;
	case USB23_DSTS_USBLNKST_USB2_SUSPEND_STATE:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB2_SUSPEND_STATE");
		break;
	case USB23_DSTS_USBLNKST_USB2_DISCONNECTED:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB2_DISCONNECTED");
		break;
	case USB23_DSTS_USBLNKST_USB2_EARLY_SUSPEND:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB2_EARLY_SUSPEND");
		break;
	case USB23_DSTS_USBLNKST_USB2_RESET:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB2_RESET");
		break;
	case USB23_DSTS_USBLNKST_USB2_RESUME:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB2_RESUME");
		break;
	}
	return;
usb3:
	switch (reg & USB23_DSTS_USBLNKST_MASK) {
	case USB23_DSTS_USBLNKST_USB3_U0:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_U0");
		break;
	case USB23_DSTS_USBLNKST_USB3_U1:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_U1");
		break;
	case USB23_DSTS_USBLNKST_USB3_U2:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_U2");
		break;
	case USB23_DSTS_USBLNKST_USB3_U3:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_U3");
		break;
	case USB23_DSTS_USBLNKST_USB3_SS_DIS:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_SS_DIS");
		break;
	case USB23_DSTS_USBLNKST_USB3_RX_DET:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_RX_DET");
		break;
	case USB23_DSTS_USBLNKST_USB3_SS_INACT:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_SS_INACT");
		break;
	case USB23_DSTS_USBLNKST_USB3_POLL:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_POLL");
		break;
	case USB23_DSTS_USBLNKST_USB3_RECOV:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_RECOV");
		break;
	case USB23_DSTS_USBLNKST_USB3_HRESET:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_HRESET");
		break;
	case USB23_DSTS_USBLNKST_USB3_CMPLY:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_CMPLY");
		break;
	case USB23_DSTS_USBLNKST_USB3_LPBK:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_LPBK");
		break;
	case USB23_DSTS_USBLNKST_USB3_RESET_RESUME:
		SHELL_OR_LOG(sh, "USB23_DSTS_USBLNKST_USB3_RESET_RESUME");
		break;
	}
}

void usb23_dump_events(const struct device *dev, const struct shell *sh)
{
	const struct usb23_config *conf = dev->config;
	struct usb23_data *priv = udc_get_private(dev);

	for (int i = 0; i < CONFIG_USB23_EVT_NUM; i++) {
		uint32_t evt = conf->evt_buf[i];
		char *s = (i == priv->evt_next) ? " <-" : "";

		SHELL_OR_LOG(sh, "evt 0x%02x: 0x%08x%s", i, evt, s);
	}
}

void usb23_dump_trbs(const struct device *dev, struct usb23_ep_data *ep_data,
		     const struct shell *sh)
{
	uint32_t reg, trbid;

	reg = usb23_manager_read(dev, ep_data, USB23_MANAGER_CONTROLSTATUS);
	trbid = GETFIELD(reg, USB23_MANAGER_CONTROLSTATUS_TRBID);

	for (int i = 0; i < ep_data->num_of_trbs; i++) {
		struct usb23_trb trb = ep_data->trb_buf[i];

		SHELL_OR_LOG(
			sh,
			"%02d ep=0x%02x addr=0x%08x%08x ctl=%ld sts=%ld hwo=%u lst=%u chn=%u"
			" csp=%u isp=%u ioc=%u spr=%u pcm1=%ld sof=%ld bufsiz=%ld%s%s%s%s",
			i, ep_data->addr, trb.addr_hi, trb.addr_lo,
			GETFIELD(trb.ctrl, USB23_TRB_CTRL_TRBCTL),
			GETFIELD(trb.status, USB23_TRB_STATUS_TRBSTS),
			!!(trb.ctrl & USB23_TRB_CTRL_HWO), !!(trb.ctrl & USB23_TRB_CTRL_LST),
			!!(trb.ctrl & USB23_TRB_CTRL_CHN), !!(trb.ctrl & USB23_TRB_CTRL_CSP),
			!!(trb.ctrl & USB23_TRB_CTRL_ISP_IMI), !!(trb.ctrl & USB23_TRB_CTRL_IOC),
			!!(trb.ctrl & USB23_TRB_CTRL_SPR), GETFIELD(trb.ctrl, USB23_TRB_CTRL_PCM1),
			GETFIELD(trb.ctrl, USB23_TRB_CTRL_SIDSOFN),
			GETFIELD(trb.status, USB23_TRB_STATUS_BUFSIZ),
			(i == ep_data->head) ? " <HEAD" : "", (i == ep_data->tail) ? " <TAIL" : "",
			(i == ep_data->head && ep_data->full) ? " <FULL" : "",
			(i == trbid) ? " <TRBID" : "");
	}
}

void usb23_dump_manager(const struct device *dev, struct usb23_ep_data *ep_data,
			const struct shell *sh)
{
	uint32_t reg;

	if (ep_data->manager_base == 0) {
		shell_error(sh, "no uvcmanager for this endpoint");
		return;
	}

#define DUMP(reg)                                                                                  \
	SHELL_OR_LOG(sh, "%-30s [0x%04x] 0x%08x", #reg, reg, usb23_manager_read(dev, ep_data, reg))
	DUMP(USB23_MANAGER_IRQRAW);
	DUMP(USB23_MANAGER_IRQFORCE);
	DUMP(USB23_MANAGER_IRQMASK);
	DUMP(USB23_MANAGER_IRQSTATUS);

	DUMP(USB23_MANAGER_CONTROLSTATUS);
	reg = usb23_manager_read(dev, ep_data, USB23_MANAGER_CONTROLSTATUS);
	switch (reg & USB23_MANAGER_CONTROLSTATUS_FSMSTATE_MASK) {
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_IDLE:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_IDLE");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WAIT:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WAIT");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_FIFOCHECK:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_FIFOCHECK");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_RTRBCTRL:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_RTRBCTRL");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WTRBCTRL:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WTRBCTRL");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_RTRBSIZE:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_RTRBSIZE");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WTRBSIZE:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WTRBSIZE");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WTRBADDR:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_WTRBADDR");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_RINGDOORBELL:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_RINGDOORBELL");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_DONE:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_DONE");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_ERROR:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_ERROR");
		break;
	case USB23_MANAGER_CONTROLSTATUS_FSMSTATE_FIFOWAIT:
		SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_FSMSTATE_FIFOWAIT");
		break;
	default:
		SHELL_OR_LOG(sh, "unknown state: 0x%02lx", GETFIELD(reg, USB23_MANAGER_CONTROLSTATUS_FSMSTATE));
		break;
	}

	SHELL_OR_LOG(sh, "USB23_MANAGER_CONTROLSTATUS_TRBID=%ld",
		    GETFIELD(reg, USB23_MANAGER_CONTROLSTATUS_TRBID));

	DUMP(USB23_MANAGER_TRBADDR);
	DUMP(USB23_MANAGER_NUMBYTES);
	DUMP(USB23_MANAGER_DONEBYTES);
	DUMP(USB23_MANAGER_DOORBELLADDR);
	DUMP(USB23_MANAGER_DOORBELLDATA);
	DUMP(USB23_MANAGER_FIFOTHRESHOLD);
	DUMP(USB23_MANAGER_STREAMADDRESS);
	DUMP(USB23_MANAGER_TRBCTRL);
	DUMP(USB23_MANAGER_FIFOOCCUPANCY);
	DUMP(USB23_MANAGER_TOTALBYTESOUT);
	DUMP(USB23_MANAGER_HEADER0);
	DUMP(USB23_MANAGER_HEADER1);
	DUMP(USB23_MANAGER_FRAMECOUNTER);
#undef DUMP
}

void usb23_dump_fifo_space(const struct device *dev, const struct shell *sh)
{
	struct { char *name; uint32_t addr, num; } fifo[] = {
#define R(r, n) {.name = #r, .addr = USB23_GDBGFIFOSPACE_QUEUETYPE_##r, .num = n}
		R(TX, 0),        R(RX, 0),        R(TXREQ, 0),    R(RXREQ, 0), R(RXINFO, 0),
		R(DESCFETCH, 0), R(TX, 1),        R(RX, 1),       R(TXREQ, 1), R(RXREQ, 1),
		R(RXINFO, 1),    R(DESCFETCH, 1), R(PROTOCOL, 2),
#undef R
	};

	for (size_t i = 0; i < sizeof(fifo) / sizeof(*fifo); i++) {
		uint32_t reg;

		reg = fifo[i].addr | (fifo[i].num << USB23_GDBGFIFOSPACE_QUEUENUM_SHIFT);
		usb23_io_write(dev, USB23_GDBGFIFOSPACE, reg);

		reg = usb23_io_read(dev, USB23_GDBGFIFOSPACE);
		reg = GETFIELD(reg, USB23_GDBGFIFOSPACE_AVAILABLE);

		SHELL_OR_LOG(sh, "%-15s [%u] = %u", fifo[i].name, fifo[i].num, reg);
	}
}

void usb23_dump_all(const struct device *dev, struct usb23_ep_data *ep_data, const struct shell *sh)
{
	usb23_dump_registers(dev, sh);
	usb23_dump_bus_error(dev, sh);
	usb23_dump_link_state(dev, sh);
	usb23_dump_events(dev, sh);
	usb23_dump_fifo_space(dev, sh);
	if (ep_data) {
		usb23_dump_trbs(dev, ep_data, sh);
		usb23_dump_manager(dev, ep_data, sh);
	}
}

static int dump_cmd2_handler(const struct shell *sh, size_t argc, char **argv,
			     void (*fn)(const struct device *, const struct shell *sh))
{
	const struct device *dev;

	__ASSERT_NO_MSG(argc == 2);

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(sh, "Device %s not found", argv[1]);
		return -ENODEV;
	}

	(*fn)(dev, sh);
	return 0;
}

static int dump_cmd3_handler(const struct shell *sh, size_t argc, char **argv,
			     void (*fn)(const struct device *, struct usb23_ep_data *,
					const struct shell *sh))
{
	const struct device *dev;
	struct usb23_ep_data *ep_data;
	long ep_addr;

	__ASSERT_NO_MSG(argc == 3);

	dev = device_get_binding(argv[1]);
	if (!dev) {
		shell_error(sh, "Device %s not found", argv[1]);
		return -ENODEV;
	}

	errno = 0;
	ep_addr = strtol(argv[2], NULL, 16);
	if (errno || ep_addr < 0 || ep_addr > UINT8_MAX) {
		shell_error(sh, "Invalid endpoint address: %s", argv[2]);
		return -EINVAL;
	}

	ep_data = usb23_get_ep_data(dev, ep_addr);
	if (!ep_data) {
		shell_error(sh, "Endpoint 0x%02x not found", (uint8_t)ep_addr);
		return -EINVAL;
	}

	(*fn)(dev, ep_data, sh);
	return 0;
}

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev == NULL) ? NULL : dev->name;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}
SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

static int cmd_usb23_manager(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd3_handler(sh, argc, argv, usb23_dump_manager);
}

static int cmd_usb23_trb(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd3_handler(sh, argc, argv, usb23_dump_trbs);
}

static int cmd_usb23_evt(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, usb23_dump_events);
}

static int cmd_usb23_reg(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, usb23_dump_registers);
}

static int cmd_usb23_buserr(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, usb23_dump_bus_error);
}

static int cmd_usb23_link(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, usb23_dump_link_state);
}

static int cmd_usb23_fifo(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, usb23_dump_fifo_space);
}

static int cmd_usb23_all (const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd3_handler(sh, argc, argv, usb23_dump_all);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_usb23,
			       SHELL_CMD_ARG(manager, &dsub_device_name,
					     "Dump the uvcmanager registers\n"
					     "Usage: manager <device> <ep>",
					     cmd_usb23_manager, 3, 0),
			       SHELL_CMD_ARG(trb, &dsub_device_name,
					     "Dump an endpoint's TRB buffer\n"
					     "Usage: trb <device> <ep>",
					     cmd_usb23_trb, 3, 0),
			       SHELL_CMD_ARG(evt, &dsub_device_name,
					     "Dump the device event buffer\n"
					     "Usage: evt <device>",
					     cmd_usb23_evt, 2, 0),
			       SHELL_CMD_ARG(reg, &dsub_device_name,
					     "Dump the device status registers\n"
					     "Usage: reg <device>",
					     cmd_usb23_reg, 2, 0),
			       SHELL_CMD_ARG(buserr, &dsub_device_name,
					     "Dump the AXI64 bus I/O errors\n"
					     "Usage: buserr <device>",
					     cmd_usb23_buserr, 2, 0),
			       SHELL_CMD_ARG(link, &dsub_device_name,
					     "Dump the USB link state\n"
					     "Usage: link <device>",
					     cmd_usb23_link, 2, 0),
			       SHELL_CMD_ARG(fifo, &dsub_device_name,
					     "Dump the FIFO available space\n"
					     "Usage: fifo <device>",
					     cmd_usb23_fifo, 2, 0),
			       SHELL_CMD_ARG(all, &dsub_device_name,
					     "Dump everything\n"
					     "Usage: all <device> <ep>",
					     cmd_usb23_all, 3, 0),
			       SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(usb23, &sub_usb23, "Lattice USB2/3 controller commands", NULL);
