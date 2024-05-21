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
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/util.h>
#include <app_version.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb23blob, CONFIG_UDC_DRIVER_LOG_LEVEL);

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

#define USB23_DEPEVT_TYPE_XFERCOMPLETE        0x01
#define USB23_DEPEVT_TYPE_XFERINPROGRESS      0x02
#define USB23_DEPEVT_TYPE_XFERNOTREADY        0x03
#define USB23_DEPEVT_TYPE_RXTXFIFOEVT         0x04
#define USB23_DEPEVT_TYPE_STREAMEVT           0x06
#define USB23_DEPEVT_TYPE_EPCMDCMPLT          0x07
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

#define USB23_DEVT_TYPE_DISCONNEVT      0x0
#define USB23_DEVT_TYPE_USBRST          0x1
#define USB23_DEVT_TYPE_CONNECTDONE     0x2
#define USB23_DEVT_TYPE_ULSTCHNG        0x3
#define USB23_DEVT_TYPE_WKUPEVT         0x4
#define USB23_DEVT_TYPE_SUSPEND         0x6
#define USB23_DEVT_TYPE_SOF             0x7
#define USB23_DEVT_TYPE_ERRTICERR       0x9
#define USB23_DEVT_TYPE_CMDCMPLT        0xa
#define USB23_DEVT_TYPE_EVNTOVERFLOW    0xb
#define USB23_DEVT_TYPE_VNDRDEVTSTRCVED 0xc

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

#define LO32(n)               ((uint32_t)((uint64_t)(n) & 0xffffffff))
#define HI32(n)               ((uint32_t)((uint64_t)(n) >> 32))
#define U64(a, b)             (((uint64_t)(a)) << 32 | (((uint64_t)(b)) & 0xffffffff))
#define MEM32(addr)           (*(volatile uint32_t *)(addr))
#define GETFIELD(reg, prefix) ((reg & prefix##_MASK) >> prefix##_SHIFT)
#define LOG_EVENT(name)       LOG_DBG("--- USB23 %s ---", #name)

static int usb23_set_address(const struct device *dev, const uint8_t addr);
static void usb23_ep_enqueue_pending(const struct device *dev, struct udc_ep_config *const ep_cfg);
void usb23_dump_trbs(const struct device *dev, struct udc_ep_config *const ep_cfg);

/*
 * Input/Output
 *
 * Communication with the USB23 register interface through memory addresses
 * The USB23 core has a register interface and a DMA interface memory.
 * These helpers are the only places where I/O is done with it.
 */

static uint32_t usb23_io_read(const struct device *dev, uint32_t addr)
{
	const struct usb23_config *config = dev->config;

	return MEM32(config->base + addr);
}

static void usb23_io_write(const struct device *dev, uint32_t addr, uint32_t data)
{
	const struct usb23_config *config = dev->config;

	MEM32(config->base + addr) = data;
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
 * Setters/Getters
 *
 * Helpers to convert between various numbers formats or accessing one struct
 * from another. No action on the hardware.
 */

static struct usb23_trb usb23_get_trb(const struct device *dev, struct udc_ep_config *const ep_cfg,
				      int n)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);

	__ASSERT_NO_MSG(n < ep_data->num_of_trbs);
	return ep_data->trb_buf[n];
}

static void usb23_set_trb(const struct device *dev, struct udc_ep_config *const ep_cfg, int n,
			  struct usb23_trb *trb)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);

	__ASSERT_NO_MSG(n < ep_data->num_of_trbs);
	ep_data->trb_buf[n] = *trb;
}

static void usb23_set_link_trb(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	struct usb23_trb link_trb = {
		.addr_lo = LO32((uintptr_t)ep_data->trb_buf),
		.addr_hi = HI32((uintptr_t)ep_data->trb_buf),
		.status = 0, /* Special rule for Link TRBs */
		.ctrl = USB23_TRB_CTRL_TRBCTL_LINK_TRB | USB23_TRB_CTRL_HWO,
	};

	__ASSERT_NO_MSG(ep_data->num_of_trbs > 0);
	usb23_set_trb(dev, ep_cfg, ep_data->num_of_trbs - 1, &link_trb);
}

static void usb23_clear_tail_trb(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);

	__ASSERT_NO_MSG((ep_data->trb_buf[ep_data->tail].ctrl & USB23_TRB_CTRL_HWO) == 0);
	usb23_set_trb(dev, ep_cfg, ep_data->tail, &(struct usb23_trb){0});
	ep_data->net_buf[ep_data->tail] = NULL; // TODO free net_buf here
	ep_data->tail = (ep_data->tail + 1) % (ep_data->num_of_trbs - 1);
}

static union usb23_evt usb23_get_next_evt(const struct device *dev)
{
	const struct usb23_config *config = dev->config;
	struct usb23_data *priv = udc_get_private(dev);
	volatile union usb23_evt evt;

	/* Cache the current event */
	evt = config->evt_buf[priv->evt_next];

	/* Clear it on the event buffer */
	config->evt_buf[priv->evt_next].raw = 0x00000000;

	/* This is a ring buffer, wrap around */
	priv->evt_next++;
	priv->evt_next %= CONFIG_USB23_EVT_NUM;

	return evt;
}

/*
 * Debug
 *
 * Temporary functions used for debugging, meant to be removed once the
 * implementation is complete.
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

void usb23_dump_registers(const struct device *dev)
{
	for (int i = 0; i < ARRAY_SIZE(usb23_regs); i++) {
		struct usb23_reg *ureg = &usb23_regs[i];
		uint32_t data;

		data = usb23_io_read(dev, ureg->addr);
		if (data != ureg->last) {
			LOG_DBG("reg 0x%08x == 0x%08x  (was 0x%08x)  // %s", ureg->addr, data,
				ureg->last, ureg->name);
			ureg->last = data;
		}
	}
}

void usb23_dump_bus_error(const struct device *dev)
{
	if (usb23_io_read(dev, USB23_GSTS) & USB23_GSTS_BUSERRADDRVLD) {
		LOG_ERR("BUS_ERROR addr=0x%08x%08x", usb23_io_read(dev, USB23_GBUSERRADDR_HI),
			usb23_io_read(dev, USB23_GBUSERRADDR_LO));
	}
}

void usb23_dump_link_state(const struct device *dev)
{
	uint32_t reg;

	reg = usb23_io_read(dev, USB23_DSTS);

	switch (reg & USB23_DSTS_CONNECTSPD_MASK) {
	case USB23_DSTS_CONNECTSPD_HS:
	case USB23_DSTS_CONNECTSPD_FS:
		switch (reg & USB23_DSTS_USBLNKST_MASK) {
		case USB23_DSTS_USBLNKST_USB2_ON_STATE:
			break;
		case USB23_DSTS_USBLNKST_USB2_SLEEP_STATE:
			break;
		case USB23_DSTS_USBLNKST_USB2_SUSPEND_STATE:
			break;
		case USB23_DSTS_USBLNKST_USB2_DISCONNECTED:
			break;
		case USB23_DSTS_USBLNKST_USB2_EARLY_SUSPEND:
			break;
		case USB23_DSTS_USBLNKST_USB2_RESET:
			break;
		case USB23_DSTS_USBLNKST_USB2_RESUME:
			break;
		}
		break;
	case USB23_DSTS_CONNECTSPD_SS:
		switch (reg & USB23_DSTS_USBLNKST_MASK) {
		case USB23_DSTS_USBLNKST_USB3_U0:
			break;
		case USB23_DSTS_USBLNKST_USB3_U1:
			break;
		case USB23_DSTS_USBLNKST_USB3_U2:
			break;
		case USB23_DSTS_USBLNKST_USB3_U3:
			break;
		case USB23_DSTS_USBLNKST_USB3_SS_DIS:
			break;
		case USB23_DSTS_USBLNKST_USB3_RX_DET:
			break;
		case USB23_DSTS_USBLNKST_USB3_SS_INACT:
			break;
		case USB23_DSTS_USBLNKST_USB3_POLL:
			break;
		case USB23_DSTS_USBLNKST_USB3_RECOV:
			break;
		case USB23_DSTS_USBLNKST_USB3_HRESET:
			break;
		case USB23_DSTS_USBLNKST_USB3_CMPLY:
			break;
		case USB23_DSTS_USBLNKST_USB3_LPBK:
			break;
		case USB23_DSTS_USBLNKST_USB3_RESET_RESUME:
			break;
		}
		break;
	default:
		LOG_ERR("unknown speed");
		return;
	}
}

void usb23_dump_events(const struct device *dev)
{
	const struct usb23_config *config = dev->config;
	struct usb23_data *priv = udc_get_private(dev);

	for (int i = 0; i < CONFIG_USB23_EVT_NUM; i++) {
		union usb23_evt evt = config->evt_buf[i];
		char *s = (i == priv->evt_next) ? " <-" : "";

		LOG_DBG("evt 0x%02x: 0x%08x%s", i, evt.raw, s);
	}
}

void usb23_dump_trbs(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);

	for (int i = 0; i < ep_data->num_of_trbs; i++) {
		struct usb23_trb trb = usb23_get_trb(dev, ep_cfg, i);

		LOG_DBG("%s:"
			" ep=0x%02x n=%02d addr=0x%08x%08x ctl=%ld sts=%ld"
			" hwo=%d lst=%d chn=%d csp=%d isp=%d ioc=%d spr=%d"
			" pcm1=%ld sof=%ld bufsiz=%ld"
			"%s%s",
			__func__, ep_cfg->addr, i, trb.addr_hi, trb.addr_lo,
			GETFIELD(trb.ctrl, USB23_TRB_CTRL_TRBCTL),
			GETFIELD(trb.status, USB23_TRB_STATUS_TRBSTS),
			!!(trb.ctrl & USB23_TRB_CTRL_HWO), !!(trb.ctrl & USB23_TRB_CTRL_LST),
			!!(trb.ctrl & USB23_TRB_CTRL_CHN), !!(trb.ctrl & USB23_TRB_CTRL_CSP),
			!!(trb.ctrl & USB23_TRB_CTRL_ISP_IMI), !!(trb.ctrl & USB23_TRB_CTRL_IOC),
			!!(trb.ctrl & USB23_TRB_CTRL_SPR), GETFIELD(trb.ctrl, USB23_TRB_CTRL_PCM1),
			GETFIELD(trb.status, USB23_TRB_CTRL_SIDSOFN),
			GETFIELD(trb.status, USB23_TRB_STATUS_BUFSIZ),
			(i == ep_data->head) ? " <HEAD" : "", (i == ep_data->tail) ? " <TAIL" : "");
	}
}

void usb23_dump_fifo_space(const struct device *dev)
{
	struct {
		char *name;
		uint32_t reg;
		int num;
	} fifo[] = {
#define R(r, n) {.name = #r, .reg = USB23_GDBGFIFOSPACE_QUEUETYPE_##r, .num = n}
		R(TX, 0),        R(RX, 0),        R(TXREQ, 0),    R(RXREQ, 0), R(RXINFO, 0),
		R(DESCFETCH, 0), R(TX, 1),        R(RX, 1),       R(TXREQ, 1), R(RXREQ, 1),
		R(RXINFO, 1),    R(DESCFETCH, 1), R(PROTOCOL, 2),
#undef R
	};

	for (size_t i = 0; i < sizeof(fifo) / sizeof(*fifo); i++) {
		usb23_io_write(dev, USB23_GDBGFIFOSPACE,
			       fifo[i].reg | (fifo[i].num << USB23_GDBGFIFOSPACE_QUEUENUM_SHIFT));
		fifo[i].reg = GETFIELD(usb23_io_read(dev, USB23_GDBGFIFOSPACE),
				       USB23_GDBGFIFOSPACE_AVAILABLE);
	}
	LOG_DBG("fifo %s=%d %s=%d %s=%d %s=%d %s=%d %s=%d %s=%d %s=%d "
		"%s=%d %s=%d %s=%d %s=%d "
		"%s=%d",
		fifo[0].name, fifo[0].reg, fifo[1].name, fifo[1].reg, fifo[2].name, fifo[2].reg,
		fifo[3].name, fifo[3].reg, fifo[4].name, fifo[4].reg, fifo[5].name, fifo[5].reg,
		fifo[6].name, fifo[6].reg, fifo[7].name, fifo[7].reg, fifo[8].name, fifo[8].reg,
		fifo[9].name, fifo[9].reg, fifo[10].name, fifo[10].reg, fifo[11].name, fifo[11].reg,
		fifo[12].name, fifo[12].reg);
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
		LOG_ERR("Endpoint command failed");
		break;
	default:
		LOG_ERR("Command failed with unknown status: 0x%08x", reg);
	}

	return GETFIELD(reg, USB23_DEPCMD_XFERRSCIDX);
}

static void usb23_depcmd_ep_config(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_epn(ep_cfg->addr);
	uint32_t param0 = 0, param1 = 0;

	LOG_INF("Configuring endpoint 0x%02x with wMaxPacketSize=%d", ep_cfg->addr, ep_cfg->mps);

	if (ep_cfg->stat.enabled) {
		param0 |= USB23_DEPCMDPAR0_DEPCFG_ACTION_MODIFY;
	} else {
		param0 |= USB23_DEPCMDPAR0_DEPCFG_ACTION_INIT;
	}

	switch (ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_CONTROL:
		param0 |= USB23_DEPCMDPAR0_DEPCFG_EPTYPE_CTRL;
		// param1 |= USB23_DEPCMDPAR1_DEPCFG_XFERNRDYEN; // for
		// debug
		break;
	case USB_EP_TYPE_BULK:
		param0 |= USB23_DEPCMDPAR0_DEPCFG_EPTYPE_BULK;
		// param1 |= USB23_DEPCMDPAR1_DEPCFG_XFERNRDYEN; // for
		// debug
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
	param0 |= (ep_cfg->caps.out ? 0 : (ep_cfg->addr & 0b01111111))
		  << USB23_DEPCMDPAR0_DEPCFG_FIFONUM_SHIFT;

	/* Per-endpoint events */
	param1 |= USB23_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
	param1 |= USB23_DEPCMDPAR1_DEPCFG_XFERCMPLEN;

	/* This is the usb protocol endpoint number, but the data encoding
	 * we chose for physical endpoint number is the same as this
	 * register */
	param1 |= epn << USB23_DEPCMDPAR1_DEPCFG_EPNUMBER_SHIFT;

	usb23_io_write(dev, USB23_DEPCMDPAR0(epn), param0);
	usb23_io_write(dev, USB23_DEPCMDPAR1(epn), param1);
	usb23_depcmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPCFG);
}

static void usb23_depcmd_ep_xfer_config(const struct device *dev,
					struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_epn(ep_cfg->addr);

	LOG_DBG("DepXferConfig: ep=0x%02x", ep_cfg->addr);
	usb23_io_write(dev, USB23_DEPCMDPAR0(epn),
		       1 << USB23_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_SHIFT);
	usb23_depcmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPXFERCFG);
}

#if 0 /* useful for hibernating */
static uint32_t usb23_depcmd_ep_get_state(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_epn(ep_cfg->addr);

	LOG_DBG("DepGetState: ep=0x%02x", ep_cfg->addr);

	usb23_depcmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPGETSTATE);
	return usb23_io_read(dev, USB23_DEPCMDPAR2(epn));
}
#endif

static void usb23_depcmd_set_stall(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_epn(ep_cfg->addr);

	LOG_WRN("DepSetStall: ep=0x%02x", ep_cfg->addr);

	usb23_depcmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPSETSTALL);
}

static void usb23_depcmd_clear_stall(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_epn(ep_cfg->addr);

	LOG_INF("DepClearStall ep=0x%02x", ep_cfg->addr);

	usb23_depcmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPCSTALL);
}

static void usb23_depcmd_start_xfer(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_epn(ep_cfg->addr);
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);

	LOG_DBG("DepStartXfer ep=0x%02x", ep_cfg->addr);

	/* Make sure the device is in U0 state, assuming TX FIFO is empty */
	usb23_io_field(dev, USB23_DCTL, USB23_DCTL_ULSTCHNGREQ_MASK,
		       USB23_DCTL_ULSTCHNGREQ_REMOTEWAKEUP);

	usb23_io_write(dev, USB23_DEPCMDPAR0(epn), HI32((uintptr_t)ep_data->trb_buf));
	usb23_io_write(dev, USB23_DEPCMDPAR1(epn), LO32((uintptr_t)ep_data->trb_buf));

	ep_data->xferrscidx = usb23_depcmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPSTRTXFER);
}

static void usb23_depcmd_update_xfer(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_epn(ep_cfg->addr);
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	uint32_t flags;

	LOG_DBG("DepUpdateXfer ep=0x%02x", ep_cfg->addr);

	flags = ep_data->xferrscidx << USB23_DEPCMD_XFERRSCIDX_SHIFT;
	usb23_depcmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPUPDXFER | flags);
}

static void usb23_depcmd_end_xfer(const struct device *dev, struct udc_ep_config *const ep_cfg,
				  uint32_t flags)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	int epn = usb23_get_epn(ep_cfg->addr);

	LOG_DBG("DepEndXfer: ep=0x%02x", ep_cfg->addr);

	flags |= ep_data->xferrscidx << USB23_DEPCMD_XFERRSCIDX_SHIFT;
	usb23_depcmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPENDXFER | flags);

	for (int i = 0; i < ep_data->num_of_trbs; i++) {
		usb23_set_trb(dev, ep_cfg, i, &(struct usb23_trb){0});
	}
}

static void usb23_depcmd_start_config(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_epn(ep_cfg->addr);
	uint32_t flags;

	LOG_DBG("DepStartConfig: ep=0x%02x", ep_cfg->addr);

	flags = (ep_cfg->caps.control ? 0 : 2) << USB23_DEPCMD_XFERRSCIDX_SHIFT;
	usb23_depcmd(dev, USB23_DEPCMD(epn), USB23_DEPCMD_DEPSTARTCFG | flags);
}

static void usb23_dgcmd(const struct device *dev, uint32_t cmd)
{
	uint32_t reg;

	usb23_io_write(dev, USB23_DGCMD, cmd);
	do {
		reg = usb23_io_read(dev, USB23_DGCMD);
		LOG_DBG("%s: reg=0x%08x", __func__, reg);
	} while ((reg & USB23_DEPCMD_CMDACT) != 0);

	if ((reg & USB23_DGCMD_STATUS_MASK) != USB23_DGCMD_STATUS_OK) {
		LOG_ERR("Device general command failed");
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

/*
 * Transfer Requests (TRB)
 *
 * USB23 receives transfer requests from this driver through a shared memory
 * buffer, resubmitted upon every new transfer (through either Start or
 * Update command).
 */

static void usb23_trb_ctrl_in(const struct device *dev, uint32_t ctrl)
{
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	struct net_buf *buf = ep_data->net_buf[0];
	struct usb23_trb trb0 = {
		.addr_lo = LO32((uintptr_t)buf->data),
		.addr_hi = HI32((uintptr_t)buf->data),
		.status = buf->len,
		.ctrl = ctrl | USB23_TRB_CTRL_LST | USB23_TRB_CTRL_HWO,
	};

	__ASSERT_NO_MSG(usb23_get_trb(dev, ep_cfg, 0).ctrl == 0x00000000);
	usb23_set_trb(dev, ep_cfg, 0, &trb0);
	usb23_depcmd_start_xfer(dev, ep_cfg);
}

static void usb23_trb_ctrl_out(const struct device *dev, struct net_buf *buf, uint32_t ctrl)
{
	const struct usb23_config *config = dev->config;
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	struct usb23_trb trb0 = {
		.status = buf->size,
		.ctrl = ctrl | USB23_TRB_CTRL_CHN | USB23_TRB_CTRL_HWO,
	};
	struct usb23_trb trb1 = {
		.addr_lo = LO32(config->discard),
		.addr_hi = HI32(config->discard),
		.status = ep_cfg->mps - buf->size,
		.ctrl = ctrl | USB23_TRB_CTRL_LST | USB23_TRB_CTRL_HWO,
	};

	ep_data->net_buf[0] = buf;
	__ASSERT_NO_MSG(ep_data->net_buf[0] != NULL);

	trb0.addr_lo = LO32((uintptr_t)ep_data->net_buf[0]->data);
	trb0.addr_hi = HI32((uintptr_t)ep_data->net_buf[0]->data);
	LOG_DBG("%s: buf=%p data=%p", __func__, ep_data->net_buf[0], ep_data->net_buf[0]->data);

	__ASSERT_NO_MSG(usb23_get_trb(dev, ep_cfg, 0).ctrl == 0x00000000);
	__ASSERT_NO_MSG(usb23_get_trb(dev, ep_cfg, 1).ctrl == 0x00000000);
	usb23_set_trb(dev, ep_cfg, 0, &trb0);
	usb23_set_trb(dev, ep_cfg, 1, &trb1);
	usb23_depcmd_start_xfer(dev, ep_cfg);
}

static void usb23_trb_ctrl_setup_out(const struct device *dev)
{
	struct net_buf *buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8);

	memset(buf->data, 0xff, buf->size);
	LOG_DBG("TRB_CONTROL_SETUP ep=0x%02x", USB_CONTROL_EP_OUT);
	usb23_trb_ctrl_out(dev, buf, USB23_TRB_CTRL_TRBCTL_CONTROL_SETUP);
}

static void usb23_trb_ctrl_data_out(const struct device *dev)
{
	struct usb23_data *priv = udc_get_private(dev);
	struct net_buf *buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, priv->data_stage_length);

	LOG_DBG("TRB_CONTROL_DATA_OUT ep=0x%02x", USB_CONTROL_EP_OUT);
	usb23_trb_ctrl_out(dev, buf, USB23_TRB_CTRL_TRBCTL_CONTROL_DATA);
}

static void usb23_trb_ctrl_data_in(const struct device *dev)
{
	LOG_DBG("TRB_CONTROL_DATA_IN ep=0x%02x", USB_CONTROL_EP_IN);
	usb23_trb_ctrl_in(dev, USB23_TRB_CTRL_TRBCTL_CONTROL_DATA);
}

static void usb23_trb_ctrl_status_2_in(const struct device *dev)
{
	LOG_DBG("TRB_CONTROL_STATUS_2_IN ep=0x%02x", USB_CONTROL_EP_IN);
	usb23_trb_ctrl_in(dev, USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
}

static void usb23_trb_ctrl_status_3_in(const struct device *dev)
{
	LOG_DBG("TRB_CONTROL_STATUS_3_IN ep=0x%02x", USB_CONTROL_EP_IN);
	usb23_trb_ctrl_in(dev, USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
}

static void usb23_trb_ctrl_status_3_out(const struct device *dev)
{
	struct net_buf *buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 0);

	LOG_DBG("TRB_CONTROL_STATUS_3_OUT ep=0x%02x", USB_CONTROL_EP_OUT);
	usb23_trb_ctrl_out(dev, buf, USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
}

/*
 * Push the TRB at the end and move one step, skipping the Link TRB
 */
static int usb23_send_trb(const struct device *dev, struct udc_ep_config *const ep_cfg,
			  struct net_buf *buf, uint32_t ctrl)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	struct usb23_trb trb = {
		.addr_lo = LO32((uintptr_t)buf->data),
		.addr_hi = HI32((uintptr_t)buf->data),
		.ctrl = ctrl,
		.status = ep_cfg->caps.in ? buf->len : buf->size,
	};
	size_t next_head = (ep_data->head + 1) % (ep_data->num_of_trbs - 1);

	/* If the next TRB in the chain is still owned by the hardware, need
	 * to retry later when more resources become available. */
	if (next_head == ep_data->tail) {
		LOG_DBG("%s: busy head=%d next_head=%d tail=%d "
			"num_of_trbs=%d",
			__func__, ep_data->head, next_head, ep_data->tail, ep_data->num_of_trbs);
		return -EBUSY;
	}

	/* Associate an active buffer and a TRB together */
	usb23_set_trb(dev, ep_cfg, ep_data->head, &trb);
	ep_data->net_buf[ep_data->head] = buf;
	ep_data->head = next_head;

	/* Update the transfer with the new transfer descriptor */
	usb23_depcmd_update_xfer(dev, ep_cfg);

	return 0;
}

static int usb23_trb_bulk(const struct device *dev, struct udc_ep_config *const ep_cfg,
			  struct net_buf *buf)
{
	uint32_t ctrl;

	LOG_DBG("TRB_BULK ep=0x%02x buf=%p data=%p size=%d len=%d", ep_cfg->addr, buf, buf->data,
		buf->size, buf->len);

	/* Make sure the transfer is terminated */
	if (udc_ep_buf_has_zlp(buf)) {
		/* Mark the TRB as end of a chain: CHN=0 */
		ctrl = USB23_TRB_CTRL_TRBCTL_NORMAL_ZLP;
		ctrl |= USB23_TRB_CTRL_IOC;
		ctrl |= USB23_TRB_CTRL_HWO;
	} else {
		/* Mark the next TRB as being part of the same USB transfer */
		ctrl = USB23_TRB_CTRL_TRBCTL_NORMAL;
		if (ep_cfg->caps.in) {
			ctrl |= USB23_TRB_CTRL_CHN;
		} else {
			ctrl |= USB23_TRB_CTRL_CSP;
		}
		ctrl |= USB23_TRB_CTRL_IOC;
		ctrl |= USB23_TRB_CTRL_HWO;
	}

	/* Try to submit the TRB if some is available */
	return usb23_send_trb(dev, ep_cfg, buf, ctrl);
}

/*
 * Events
 *
 * Process the events from the event ring buffer. Interrupts gives us a
 * hint that an event is available, which we fetch from a ring buffer shared
 * with the hardware.
 */

static void usb23_on_soft_reset(const struct device *dev)
{
	const struct usb23_config *config = dev->config;
	uint32_t reg;

	/* Configure and reset the Device Controller */
	usb23_io_write(dev, USB23_DCTL,
		       USB23_DCTL_CSFTRST |
			       // TODO confirm that DWC_USB3_EN_LPM_ERRATA == 1
			       (15 << USB23_DCTL_LPM_NYET_THRES_SHIFT));
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
	LOG_INF("coreid=0x%04lx rel=0x%04lx", GETFIELD(reg, USB23_GCOREID_CORE),
		GETFIELD(reg, USB23_GCOREID_REL));
	__ASSERT_NO_MSG(GETFIELD(reg, USB23_GCOREID_CORE) == 0x5533);

	/* Letting GUID unchanged */
	/* Letting GUSB2PHYCFG and GUSB3PIPECTL unchanged */

	/* Setting fifo size for both TX and RX, experimental values
	 * GRXFIFOSIZ too far below or above  512 * 3 leads to errors */
	usb23_io_write(dev, USB23_GTXFIFOSIZ(0), 512 * 3);
	usb23_io_write(dev, USB23_GRXFIFOSIZ(0), 512 * 3);

	/* Setup the event buffer address, size and start event reception */
	memset((void *)config->evt_buf, 0, CONFIG_USB23_EVT_NUM * sizeof(union usb23_evt));
	usb23_io_write(dev, USB23_GEVNTADR_LO(0), LO32((uintptr_t)config->evt_buf));
	usb23_io_write(dev, USB23_GEVNTADR_HI(0), HI32((uintptr_t)config->evt_buf));
	usb23_io_write(dev, USB23_GEVNTSIZ(0), CONFIG_USB23_EVT_NUM * sizeof(*config->evt_buf));
	usb23_io_write(dev, USB23_GEVNTCOUNT(0), 0);

	/* Letting GCTL unchanged */

	/* Set the USB device configuration, including max supported speed */
	usb23_io_write(dev, USB23_DCFG, USB23_DCFG_PERFRINT_90);
	switch (config->speed_idx) {
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
	usb23_depcmd_start_config(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
	usb23_depcmd_start_config(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));
}

static void usb23_on_usb_reset(const struct device *dev)
{
	const struct usb23_config *config = dev->config;

	/* Reset all ongoing transfers on non-0 endpoints */
	for (int epn = 1; epn < config->num_bidir_eps; epn++) {
		continue; // TODO
		usb23_depcmd_end_xfer(dev, usb23_get_ep_cfg(dev, epn), 0);
		usb23_depcmd_clear_stall(dev, usb23_get_ep_cfg(dev, epn));
	}

	/* Perform the USB reset operations manually to improve latency */
	usb23_set_address(dev, 0);

	/* Let Zephyr set the device address 0 */
	udc_submit_event(dev, UDC_EVT_RESET, 0);
}

static void usb23_on_connect_done(const struct device *dev)
{
	struct udc_ep_config *ep_cfg;
	int mps = 0;

	/* Adjust parameters against the connection speed */
	switch (usb23_io_read(dev, USB23_DSTS) & USB23_DSTS_CONNECTSPD_MASK) {
	case USB23_DSTS_CONNECTSPD_FS:
	case USB23_DSTS_CONNECTSPD_HS:
		mps = 64;
		// TODO this is not suspending USB3, it enable suspend feature
		// usb23_io_set(dev, USB23_GUSB3PIPECTL, USB23_GUSB3PIPECTL_SUSPENDENABLE);
		break;
	case USB23_DSTS_CONNECTSPD_SS:
		mps = 512;
		// usb23_io_set(dev, USB23_GUSB2PHYCFG, USB23_GUSB2PHYCFG_SUSPHY);
		break;
	}
	__ASSERT_NO_MSG(mps != 0);

	/* Reconfigure ep=0x00 connection speed */
	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	ep_cfg->mps = mps;
	usb23_depcmd_ep_config(dev, ep_cfg);

	/* Reconfigure ep=0x80 connection speed */
	ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
	ep_cfg->mps = mps;
	usb23_depcmd_ep_config(dev, ep_cfg);

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
			LOG_ERR("unknown USB3 link state");
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
			LOG_ERR("unknown USB2 link state");
		}
		break;
	default:
		LOG_ERR("unknown connection speed");
	}
}

static void usb23_on_device_event(const struct device *dev, struct usb23_devt devt)
{
	switch (devt.type) {
	case USB23_DEVT_TYPE_DISCONNEVT:
		LOG_EVENT(DEVT_TYPE_DISCONNEVT);
		break;
	case USB23_DEVT_TYPE_USBRST:
		LOG_EVENT(DEVT_TYPE_USBRST);
		usb23_on_usb_reset(dev);
		break;
	case USB23_DEVT_TYPE_CONNECTDONE:
		LOG_EVENT(DEVT_TYPE_CONNECTDONE);
		usb23_on_connect_done(dev);
		break;
	case USB23_DEVT_TYPE_ULSTCHNG:
		LOG_EVENT(DEVT_TYPE_ULSTCHNG);
		usb23_on_link_state_event(dev);
		break;
	case USB23_DEVT_TYPE_WKUPEVT:
		LOG_EVENT(DEVT_TYPE_WKUPEVT);
		break;
	case USB23_DEVT_TYPE_SUSPEND:
		LOG_EVENT(DEVT_TYPE_SUSPEND);
		break;
	case USB23_DEVT_TYPE_SOF:
		LOG_EVENT(DEVT_TYPE_SOF);
		break;
	case USB23_DEVT_TYPE_CMDCMPLT:
		LOG_EVENT(DEVT_TYPE_CMDCMPLT);
		break;
	case USB23_DEVT_TYPE_VNDRDEVTSTRCVED:
		LOG_EVENT(DEVT_TYPE_VNDRDEVTSTRCVED);
		break;
	case USB23_DEVT_TYPE_ERRTICERR:
		__ASSERT(false, "DEVT_TYPE_ERRTICERR");
		break;
	case USB23_DEVT_TYPE_EVNTOVERFLOW:
		__ASSERT(false, "DEVT_TYPE_EVNTOVERFLOW");
		break;
	default:
		LOG_ERR("unknown device event: 0x%08d", *(uint32_t *)&devt);
	}
}

/* Control Write */

/* OUT */
static void usb23_on_ctrl_write_setup(const struct device *dev, struct udc_ep_config *const ep_cfg,
				      struct net_buf *buf)
{
	LOG_DBG("%s: buf=%p", __func__, buf);
	usb23_trb_ctrl_data_out(dev);
}

/* OUT */
static void usb23_on_ctrl_write_data(const struct device *dev, struct udc_ep_config *const ep_cfg,
				     struct net_buf *buf)
{
	int ret;

	LOG_DBG("%s: buf=%p", __func__, buf);
	udc_ctrl_update_stage(dev, buf);
	ret = udc_ctrl_submit_s_out_status(dev, buf);
	__ASSERT_NO_MSG(ret == 0);
	k_sleep(K_MSEC(1));
}

/* IN */
static void usb23_on_ctrl_write_status(const struct device *dev, struct udc_ep_config *const ep_cfg,
				       struct net_buf *buf)
{
	int ret;

	LOG_DBG("%s: buf=%p", __func__, buf);
	ret = udc_ctrl_submit_status(dev, buf);
	__ASSERT_NO_MSG(ret == 0);
	udc_ctrl_update_stage(dev, buf);
}

/* Control Read */

/* OUT */
static void usb23_on_ctrl_read_setup(const struct device *dev, struct udc_ep_config *const ep_cfg,
				     struct net_buf *buf)
{
	int ret;

	LOG_DBG("%s: buf=%p", __func__, buf);
	ret = udc_ctrl_submit_s_in_status(dev);
	__ASSERT_NO_MSG(ret == 0);
}

/* IN */
static void usb23_on_ctrl_read_data(const struct device *dev, struct udc_ep_config *const ep_cfg,
				    struct net_buf *buf)
{
	LOG_DBG("%s: buf=%p", __func__, buf);
	usb23_trb_ctrl_status_3_out(dev);
	udc_ctrl_update_stage(dev, buf);
}

/* OUT */
static void usb23_on_ctrl_read_status(const struct device *dev, struct udc_ep_config *const ep_cfg,
				      struct net_buf *buf)
{
	int ret;

	LOG_DBG("%s: buf=%p", __func__, buf);
	ret = udc_ctrl_submit_status(dev, buf);
	__ASSERT_NO_MSG(ret == 0);
	udc_ctrl_update_stage(dev, buf);
}

/* No-Data Control */

/* OUT */
static void usb23_on_ctrl_nodata_setup(const struct device *dev, struct udc_ep_config *const ep_cfg,
				       struct net_buf *buf)
{
	int ret;

	LOG_DBG("%s: buf=%p", __func__, buf);
	ret = udc_ctrl_submit_s_status(dev);
	__ASSERT_NO_MSG(ret == 0);
}

/* IN */
static void usb23_on_ctrl_nodata_status(const struct device *dev,
					struct udc_ep_config *const ep_cfg, struct net_buf *buf)
{
	int ret;

	LOG_DBG("%s: buf=%p", __func__, buf);
	ret = udc_ctrl_submit_status(dev, buf);
	__ASSERT_NO_MSG(ret == 0);
	udc_ctrl_update_stage(dev, buf);
}

static void usb23_on_ctrl_setup(const struct device *dev, struct udc_ep_config *const ep_cfg,
				struct net_buf *buf)
{
	struct usb23_data *priv = udc_get_private(dev);

	/* Only moment where this information is accessible */
	priv->data_stage_length = udc_data_stage_length(buf);

	LOG_DBG("%s: buf=%p data=%p", __func__, buf, buf->data);

	/* To be able to differentiate the next stage*/
	udc_ep_buf_set_setup(buf);
	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_data_out(dev)) {
		usb23_on_ctrl_write_setup(dev, ep_cfg, buf);
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		usb23_on_ctrl_read_setup(dev, ep_cfg, buf);
	} else if (udc_ctrl_stage_is_no_data(dev)) {
		usb23_on_ctrl_nodata_setup(dev, ep_cfg, buf);
	} else {
		LOG_ERR("unknown setup stage");
	}
}

static void usb23_on_ctrl_data(const struct device *dev, struct udc_ep_config *const ep_cfg,
			       struct net_buf *buf)
{
	if (udc_ctrl_stage_is_data_out(dev)) {
		__ASSERT_NO_MSG(ep_cfg->addr == USB_CONTROL_EP_OUT);
		usb23_on_ctrl_write_data(dev, ep_cfg, buf);
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		__ASSERT_NO_MSG(ep_cfg->addr == USB_CONTROL_EP_IN);
		usb23_on_ctrl_read_data(dev, ep_cfg, buf);
		net_buf_unref(buf);
	} else {
		LOG_ERR("unknown data stage");
	}

	// net_buf_unref(buf);
}

static void usb23_on_ctrl_status(const struct device *dev, struct udc_ep_config *const ep_cfg,
				 struct net_buf *buf)
{
	if (udc_ctrl_stage_is_status_in(dev)) {
		usb23_on_ctrl_write_status(dev, ep_cfg, buf);
	} else if (udc_ctrl_stage_is_status_out(dev)) {
		usb23_on_ctrl_read_status(dev, ep_cfg, buf);
	} else if (udc_ctrl_stage_is_no_data(dev)) {
		usb23_on_ctrl_nodata_status(dev, ep_cfg, buf);
	} else {
		LOG_ERR("unknown status stage");
		return;
	}

	usb23_trb_ctrl_setup_out(dev);
}

/*
 * Only used for logging purpose
 */
static void usb23_on_xfer_not_ready(const struct device *dev, struct udc_ep_config *const ep_cfg,
				    uint32_t status)
{
	switch (status & USB23_DEPEVT_STATUS_B3_MASK) {
	case USB23_DEPEVT_STATUS_B3_CONTROL_SETUP:
		break;
	case USB23_DEPEVT_STATUS_B3_CONTROL_DATA:
		break;
	case USB23_DEPEVT_STATUS_B3_CONTROL_STATUS:
		break;
	}
	return;
}

static void usb23_on_xfer_done_norm(const struct device *dev, struct udc_ep_config *const ep_cfg,
				    struct net_buf *buf, struct usb23_trb *trb)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	int ret;

	LOG_DBG("%s: head=%d tail=%d", __func__, ep_data->head, ep_data->tail);

	__ASSERT_NO_MSG(!ep_cfg->caps.control);
	__ASSERT_NO_MSG(buf != NULL);

	/* Clear the TRB that triggered the event */
	usb23_clear_tail_trb(dev, ep_cfg);

	ret = udc_submit_ep_event(dev, buf, 0);
	__ASSERT_NO_MSG(ret == 0);

	/* Now that there is a free slot, enqueue the next buffer if any */
	usb23_ep_enqueue_pending(dev, ep_cfg);
}

static void usb23_on_xfer_done_ctrl(const struct device *dev, struct udc_ep_config *const ep_cfg,
				    struct net_buf *buf, struct usb23_trb *trb)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);

	/* Clear all TRBs now that the transfer is complete */
	for (int i = 0; i < ep_data->num_of_trbs; i++) {
		usb23_set_trb(dev, ep_cfg, i, &(struct usb23_trb){0});
	}

	__ASSERT_NO_MSG(buf != NULL);

	/* Continue to the next step */
	switch (trb->ctrl & USB23_TRB_CTRL_TRBCTL_MASK) {
	case USB23_TRB_CTRL_TRBCTL_CONTROL_SETUP:
		usb23_on_ctrl_setup(dev, ep_cfg, buf);
		break;
	case USB23_TRB_CTRL_TRBCTL_CONTROL_DATA:
		usb23_on_ctrl_data(dev, ep_cfg, buf);
		break;
	case USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_2:
		usb23_on_ctrl_status(dev, ep_cfg, buf);
		break;
	case USB23_TRB_CTRL_TRBCTL_CONTROL_STATUS_3:
		usb23_on_ctrl_status(dev, ep_cfg, buf);
		break;
	default:
		__ASSERT_NO_MSG(false);
		break;
	}
}

static void usb23_on_xfer_done(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	struct usb23_trb trb = usb23_get_trb(dev, ep_cfg, ep_data->tail);
	struct net_buf *buf = ep_data->net_buf[ep_data->tail];

	__ASSERT_NO_MSG(trb.ctrl != 0x00000000);
	__ASSERT_NO_MSG((trb.ctrl & USB23_TRB_CTRL_HWO) == 0);

	/* For buffers coming from the host, update the size actually
	 * received */
	if (ep_cfg->caps.out) {
		buf->len = buf->size - GETFIELD(trb.status, USB23_TRB_STATUS_BUFSIZ);
	}

	if (ep_cfg->addr == USB_CONTROL_EP_OUT) {
		/* Latency optimization: set the address immediately to be
		 * able to be able to ACK/NAK the first packets from the
		 * host with the new address, otherwise the host issue a
		 * reset */
		if (buf->len > 2 && buf->data[0] == 0 && buf->data[1] == 5) {
			usb23_set_address(dev, buf->data[2]);
		}
	}

	switch (trb.status & USB23_TRB_STATUS_TRBSTS_MASK) {
	case USB23_TRB_STATUS_TRBSTS_OK:
		break;
	case USB23_TRB_STATUS_TRBSTS_MISSEDISOC:
	case USB23_TRB_STATUS_TRBSTS_SETUPPENDING:
	case USB23_TRB_STATUS_TRBSTS_XFERINPROGRESS:
	case USB23_TRB_STATUS_TRBSTS_ZLPPENDING:
	default:
		__ASSERT_NO_MSG(false);
	}

	if (ep_cfg->caps.control) {
		usb23_on_xfer_done_ctrl(dev, ep_cfg, buf, &trb);
	} else {
		usb23_on_xfer_done_norm(dev, ep_cfg, buf, &trb);
	}
}

static void usb23_on_endpoint_event(const struct device *dev, struct usb23_depevt depevt)
{
	int epn = depevt.ep_num;
	struct udc_ep_config *const ep_cfg = usb23_get_ep_cfg(dev, epn);

	switch (depevt.type) {
	case USB23_DEPEVT_TYPE_XFERCOMPLETE:
		LOG_EVENT(DEPEVT_TYPE_XFERCOMPLETE);
		__ASSERT_NO_MSG((depevt.status & USB23_DEPEVT_STATUS_BUSERR) == 0);
		usb23_on_xfer_done(dev, ep_cfg);
		break;
	case USB23_DEPEVT_TYPE_XFERINPROGRESS:
		LOG_EVENT(DEPEVT_TYPE_XFERINPROGRESS);
		__ASSERT_NO_MSG((depevt.status & USB23_DEPEVT_STATUS_BUSERR) == 0);
		usb23_on_xfer_done(dev, ep_cfg);
		break;
	case USB23_DEPEVT_TYPE_XFERNOTREADY:
		LOG_EVENT(DEPEVT_TYPE_XFERNOTREADY);
		usb23_on_xfer_not_ready(dev, ep_cfg, depevt.status);
		break;
	case USB23_DEPEVT_TYPE_RXTXFIFOEVT:
		LOG_EVENT(DEPEVT_TYPE_RXTXFIFOEVT);
		break;
	case USB23_DEPEVT_TYPE_STREAMEVT:
		LOG_EVENT(DEPEVT_TYPE_STREAMEVT);
		break;
	case USB23_DEPEVT_TYPE_EPCMDCMPLT:
		LOG_EVENT(DEPEVT_TYPE_EPCMDCMPLT);
		break;
	default:
		LOG_ERR("unknown endpoint event: 0x%x", depevt.type);
	}
}

void usb23_on_event(const struct device *dev)
{
	/* Process each pending event from the list */
	while (usb23_io_read(dev, USB23_GEVNTCOUNT(0)) > 0) {
		union usb23_evt evt = usb23_get_next_evt(dev);

		/* We can already release the resource now that we copied it */
		usb23_io_write(dev, USB23_GEVNTCOUNT(0), sizeof(evt));

		if (evt.raw == 0x00000000) {
			LOG_ERR("empty event received");
			usb23_dump_bus_error(dev);
		} else if (evt.devt.category) {
			usb23_on_device_event(dev, evt.devt);
		} else {
			usb23_on_endpoint_event(dev, evt.depevt);
		}
	}
}

void usb23_irq_handler(void *ptr)
{
	const struct device *dev = ptr;
	const struct usb23_config *config = dev->config;
	struct usb23_data *priv = udc_get_private(dev);

	config->irq_clear_func();
	k_work_submit(&priv->work);
}

/*
 * UDC API
 *
 * Interface called by Zehpyr from the upper levels of abstractions.
 */

static void usb23_ep_enqueue_pending(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct net_buf *buf;
	int ret;

	/* See if there is another buffer to be processed */
	buf = net_buf_get(&ep_cfg->fifo, K_NO_WAIT);
	if (buf) {
		/* There is more room for one buffer to be submitted */
		ret = usb23_trb_bulk(dev, ep_cfg, buf);
		__ASSERT_NO_MSG(ret == 0);
	}
}

int usb23_api_ep_enqueue(const struct device *dev, struct udc_ep_config *const ep_cfg,
			 struct net_buf *buf)
{
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);
	struct udc_buf_info *bi = udc_get_buf_info(buf);

	LOG_DBG("%s: ep=0x%02x buf=%p", __func__, ep_cfg->addr, buf);

	switch (ep_cfg->addr) {
	case USB_CONTROL_EP_IN:
		/* Save the buffer to fetch it back later */
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
		/* Try to submit the buffer directly to the hardware */
		if (usb23_trb_bulk(dev, ep_cfg, buf) != 0) {
			/* Fallback to submit it to a queue for later */
			udc_buf_put(ep_cfg, buf);
		}
	}

	return 0;
}

int usb23_api_ep_dequeue(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;

	LOG_DBG("%s: ep=0x%02x", __func__, ep_cfg->addr);
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
	int epn = usb23_get_epn(ep_cfg->addr);

	LOG_DBG("%s: ep=0x%02x", __func__, ep_cfg->addr);
	usb23_io_clear(dev, USB23_DALEPENA, BIT(epn));
	return 0;
}

/*
 * Halt endpoint. Halted endpoint should respond with a STALL handshake.
 */
int usb23_api_ep_set_halt(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	LOG_WRN("Setting stall state on endpoint ep=0x%02x", ep_cfg->addr);

	switch (ep_cfg->addr) {
	case USB_CONTROL_EP_IN:
		/* Remove the TRBs transfer for the cancelled sequence */
		usb23_depcmd_end_xfer(dev, ep_cfg, USB23_DEPCMD_HIPRI_FORCERM);
		usb23_set_trb(dev, ep_cfg, 0, &(struct usb23_trb){0});

		/* The datasheet says to only set stall for the OUT
		 * direction */
		ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
		/* fallthrough */
	case USB_CONTROL_EP_OUT:
		usb23_depcmd_end_xfer(dev, ep_cfg, 0);
		usb23_depcmd_set_stall(dev, ep_cfg);

		/* The hardware will automatically clear the halt state upon
		 * the next setup packet received. */
		usb23_trb_ctrl_setup_out(dev);
		break;
	default:
		usb23_depcmd_set_stall(dev, ep_cfg);
		ep_cfg->stat.halted = true;
	}

	return 0;
}

int usb23_api_ep_clear_halt(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	LOG_WRN("Clearing stall state on endpoint ep=0x%02x", ep_cfg->addr);
	__ASSERT_NO_MSG(ep_cfg->addr != USB_CONTROL_EP_OUT);
	__ASSERT_NO_MSG(ep_cfg->addr != USB_CONTROL_EP_IN);

	usb23_depcmd_clear_stall(dev, ep_cfg);
	ep_cfg->stat.halted = false;
	return 0;
}

int usb23_api_set_address(const struct device *dev, const uint8_t addr)
{
	/* The address is set in the code earlier to improve latency, only
	 * checking that it is still the value done for consistency. */
	__ASSERT_NO_MSG(GETFIELD(usb23_io_read(dev, USB23_DCFG), USB23_DCFG_DEVADDR) == addr);
	return 0;
}

static int usb23_set_address(const struct device *dev, const uint8_t addr)
{
	LOG_INF("ADDR %d", addr);

	/* Configure the new address */
	usb23_io_field(dev, USB23_DCFG, USB23_DCFG_DEVADDR_MASK, addr << USB23_DCFG_DEVADDR_SHIFT);

	/* Re-apply the same endpoint configuration */
	usb23_depcmd_ep_config(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT));
	usb23_depcmd_ep_config(dev, udc_get_ep_cfg(dev, USB_CONTROL_EP_IN));

	return 0;
}

int usb23_api_set_exit_latency(const struct device *dev, const struct udc_exit_latency *el)
{
	LOG_DBG("%s u1sel=%d u1pel=%d u2sel=%d u2pel=%d", __func__, el->u1sel, el->u1pel, el->u2sel,
		el->u2pel);
	usb23_dgcmd_exit_latency(dev, el);
	return 0;
}

int usb23_api_host_wakeup(const struct device *dev)
{
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

int usb23_api_enable(const struct device *dev)
{
	const struct usb23_config *config = dev->config;

	LOG_INF("%s", __func__);

	/* Bootstrap: prepare reception of the initial Setup packet */
	usb23_trb_ctrl_setup_out(dev);

	usb23_io_set(dev, USB23_DCTL, USB23_DCTL_RUNSTOP);
	config->irq_enable_func();
	return 0;
}

int usb23_api_disable(const struct device *dev)
{
	LOG_INF("%s", __func__);
	return 0;
}

/*
 * Shut down the controller completely
 */
int usb23_api_shutdown(const struct device *dev)
{
	LOG_INF("%s", __func__);
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}
	return 0;
}

int usb23_api_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

int usb23_api_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

/*
 * Hardware Init
 *
 * Prepare the driver and the hardware to being used.
 * This goes through register configuration and register commands.
 */

int usb23_api_ep_enable(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	int epn = usb23_get_epn(ep_cfg->addr);
	struct usb23_ep_data *ep_data = usb23_get_ep_data(dev, ep_cfg);

	LOG_DBG("%s: ep=0x%02x", __func__, ep_cfg->addr);

	/* Initialize the TRB buffer */
	memset((void *)ep_data->trb_buf, 0, ep_data->num_of_trbs * sizeof(struct usb23_trb));

	usb23_depcmd_ep_config(dev, ep_cfg);
	usb23_depcmd_ep_xfer_config(dev, ep_cfg);

	/* No active TRB for now, DepUpdateXfer will add some later */
	if (!ep_cfg->caps.control) {
		usb23_set_link_trb(dev, ep_cfg);
		usb23_depcmd_start_xfer(dev, ep_cfg);
	}

	/* Starting from here, the endpoint can be used */
	usb23_io_set(dev, USB23_DALEPENA, USB23_DALEPENA_USBACTEP(epn));

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
	k_sleep(K_MSEC(1)); // TODO: reduce amount of wait time

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
	LOG_DBG("%s: ep=%p ctrl=%d", __func__, udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT),
		udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT)->caps.control);
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, data->caps.mps0,
				     0);
	__ASSERT_NO_MSG(ret == 0);

	/* Configure the control IN endpoint */
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, data->caps.mps0,
				     0);
	__ASSERT_NO_MSG(ret == 0);

	return ret;
}
