/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT snps_dwc3

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include "udc_common.h"
/*
 * Public Lattice USB23 API (endpoint ownership handoff). Note this is a
 * DIFFERENT file from the same-named private quirks header included further
 * down with quotes: that one needs this driver's structs and must stay
 * below them, while the ownership typedef is needed by udc_dwc3_data.
 */
#include <zephyr/drivers/usb/udc/udc_dwc3_lattice_usb23.h>

/* TRB memory buffer fields */
#define UDC_DWC3_TRB_STATUS_BUFSIZ_MASK				GENMASK(23, 0)
#define UDC_DWC3_TRB_STATUS_PCM1_MASK				GENMASK(25, 24)
#define UDC_DWC3_TRB_STATUS_PCM1_1PKT				(0x0 << 24)
#define UDC_DWC3_TRB_STATUS_PCM1_2PKT				(0x1 << 24)
#define UDC_DWC3_TRB_STATUS_PCM1_3PKT				(0x2 << 24)
#define UDC_DWC3_TRB_STATUS_PCM1_4PKT				(0x3 << 24)
#define UDC_DWC3_TRB_STATUS_TRBSTS_MASK				GENMASK(31, 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_OK				(0x0 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_MISSEDISOC			(0x1 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING			(0x2 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS		(0x4 << 28)
#define UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING			(0xf << 28)
#define UDC_DWC3_TRB_CTRL_HWO					BIT(0)
#define UDC_DWC3_TRB_CTRL_LST					BIT(1)
#define UDC_DWC3_TRB_CTRL_CHN					BIT(2)
#define UDC_DWC3_TRB_CTRL_CSP					BIT(3)
#define UDC_DWC3_TRB_CTRL_TRBCTL_MASK				GENMASK(9, 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL				(0x1 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP			(0x2 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2		(0x3 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3		(0x4 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA			(0x5 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_ISOCHRONOUS_1			(0x6 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_ISOCHRONOUS_N			(0x7 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB			(0x8 << 4)
#define UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL_ZLP			(0x9 << 4)
#define UDC_DWC3_TRB_CTRL_ISP_IMI				BIT(10)
#define UDC_DWC3_TRB_CTRL_IOC					BIT(11)
#define UDC_DWC3_TRB_CTRL_PCM1_MASK				GENMASK(25, 24)
#define UDC_DWC3_TRB_CTRL_SPR					26
#define UDC_DWC3_TRB_CTRL_SIDSOFN_MASK				GENMASK(29, 14)

/* Incomplete coverage of all fields, but suited for what this driver supports */
#define UDC_DWC3_EVT_MASK					GENMASK(11, 0)
#define UDC_DWC3_DEPEVT_EPN_MASK				GENMASK(5, 1)
#define UDC_DWC3_DEPEVT_XFERCOMPLETE(epn)			(((epn) << 1) | (0x01 << 6))
#define UDC_DWC3_DEPEVT_XFERINPROGRESS(epn)			(((epn) << 1) | (0x02 << 6))
#define UDC_DWC3_DEPEVT_XFERNOTREADY(epn)			(((epn) << 1) | (0x03 << 6))
#define UDC_DWC3_DEPEVT_RXTXFIFOEVT(epn)			(((epn) << 1) | (0x04 << 6))
#define UDC_DWC3_DEPEVT_STREAMEVT(epn)				(((epn) << 1) | (0x06 << 6))
#define UDC_DWC3_DEPEVT_EPCMDCMPLT(epn)				(((epn) << 1) | (0x07 << 6))
/* For XferNotReady */
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK			GENMASK(13, 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP			(0x0 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA			(0x1 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS			(0x2 << 12)
/* For XferComplete or XferInProgress */
#define UDC_DWC3_DEPEVT_STATUS_BUSERR				BIT(0)
#define UDC_DWC3_DEPEVT_STATUS_SHORT				BIT(1)
#define UDC_DWC3_DEPEVT_STATUS_IOC				BIT(2)
/* For XferComplete */
#define UDC_DWC3_DEPEVT_STATUS_LST				BIT(3)
/* For XferInProgress */
#define UDC_DWC3_DEPEVT_STATUS_MISSED_ISOC			BIT(3)
/* For StreamEvt */
#define UDC_DWC3_DEPEVT_STATUS_STREAMFOUND			0x1
#define UDC_DWC3_DEPEVT_STATUS_STREAMNOTFOUND			0x2
#define UDC_DWC3_DEVT_DISCONNEVT				(BIT(0) | (0x0 << 8))
#define UDC_DWC3_DEVT_USBRST					(BIT(0) | (0x1 << 8))
#define UDC_DWC3_DEVT_CONNECTDONE				(BIT(0) | (0x2 << 8))
#define UDC_DWC3_DEVT_ULSTCHNG					(BIT(0) | (0x3 << 8))
#define UDC_DWC3_DEVT_WKUPEVT					(BIT(0) | (0x4 << 8))
#define UDC_DWC3_DEVT_SUSPEND					(BIT(0) | (0x6 << 8))
#define UDC_DWC3_DEVT_SOF					(BIT(0) | (0x7 << 8))
#define UDC_DWC3_DEVT_ERRTICERR					(BIT(0) | (0x9 << 8))
#define UDC_DWC3_DEVT_CMDCMPLT					(BIT(0) | (0xa << 8))
#define UDC_DWC3_DEVT_EVNTOVERFLOW				(BIT(0) | (0xb << 8))
#define UDC_DWC3_DEVT_VNDRDEVTSTRCVED				(BIT(0) | (0xc << 8))

/* Device Endpoint Commands and Parameters */
#define UDC_DWC3_DEPCMDPAR2(n)					(0xc800 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR1(n)					(0xc804 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR0(n)					(0xc808 + 16 * (n))
#define UDC_DWC3_DEPCMD(n)					(0xc80c + 16 * (n))
/* Common fields to DEPCMD */
#define UDC_DWC3_DEPCMD_HIPRI_FORCERM				(1 << 11)
#define UDC_DWC3_DEPCMD_STATUS_MASK				GENMASK(15, 12)
#define UDC_DWC3_DEPCMD_STATUS_OK				(0 << 12)
#define UDC_DWC3_DEPCMD_STATUS_CMDERR				(1 << 12)
#define UDC_DWC3_DEPCMD_XFERRSCIDX_MASK				GENMASK(22, 16)
/* DEPCFG Command and Parameters */
#define UDC_DWC3_DEPCMD_DEPCFG					(1 << 0)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_MASK			GENMASK(2, 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_CTRL			(0x0 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_ISOC			(0x1 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_BULK			(0x2 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_INT			(0x3 << 1)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_MPS_MASK			GENMASK(13, 3)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_FIFONUM_MASK			GENMASK(21, 17)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK			GENMASK(25, 22)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MASK			GENMASK(31, 30)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_INIT			(0x0 << 30)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_RESTORE		(0x1 << 30)
#define UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MODIFY		(0x2 << 30)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_INTRNUM_MASK			GENMASK(4, 0)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN			BIT(8)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN			BIT(9)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN			BIT(10)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_RXTXFIFOEVTEN		BIT(11)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_STREAMEVTEN			BIT(13)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_LIMITTXDMA			BIT(15)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_BINTERVAL_MASK		GENMASK(23, 16)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_STRMCAP			BIT(24)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_EPNUMBER_MASK		GENMASK(29, 25)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_BULKBASED			BIT(30)
#define UDC_DWC3_DEPCMDPAR1_DEPCFG_FIFOBASED			BIT(31)
#define UDC_DWC3_DEPCMDPAR2_DEPCFG_EPSTATE_MASK			GENMASK(31, 0)
/* DEPXFERCFG Command and Parameters */
#define UDC_DWC3_DEPCMD_DEPXFERCFG				(0x2 << 0)
#define UDC_DWC3_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK		GENMASK(15, 0)
/* Other Commands */
#define UDC_DWC3_DEPCMD_DEPGETSTATE				(0x3 << 0)
#define UDC_DWC3_DEPCMD_DEPSETSTALL				(0x4 << 0)
#define UDC_DWC3_DEPCMD_DEPCSTALL				(0x5 << 0)
#define UDC_DWC3_DEPCMD_DEPSTRTXFER				(0x6 << 0)
#define UDC_DWC3_DEPCMD_DEPUPDXFER				(0x7 << 0)
#define UDC_DWC3_DEPCMD_DEPENDXFER				(0x8 << 0)
#define UDC_DWC3_DEPCMD_DEPSTARTCFG				(0x9 << 0)
#define UDC_DWC3_DEPCMD_CMDACT					BIT(10)

/* Global USB2 (UTMI/ULPI) PHY configuration */
#define UDC_DWC3_GUSB2PHYCFG					0xC200
#define UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST				BIT(31)
#define UDC_DWC3_GUSB2PHYCFG_ULPIEXTVBUSINDICATOR		BIT(18)
#define UDC_DWC3_GUSB2PHYCFG_ULPIEXTVBUSDRV			BIT(17)
#define UDC_DWC3_GUSB2PHYCFG_ULPICLKSUSM			BIT(16)
#define UDC_DWC3_GUSB2PHYCFG_ULPIAUTORES			BIT(15)
#define UDC_DWC3_GUSB2PHYCFG_USBTRDTIM_MASK			GENMASK(13, 10)
#define UDC_DWC3_GUSB2PHYCFG_USBTRDTIM_16BIT			(5 << 10)
#define UDC_DWC3_GUSB2PHYCFG_USBTRDTIM_8BIT			(9 << 10)
#define UDC_DWC3_GUSB2PHYCFG_ENBLSLPM				BIT(8)
#define UDC_DWC3_GUSB2PHYCFG_PHYSEL				BIT(7)
#define UDC_DWC3_GUSB2PHYCFG_SUSPHY				BIT(6)
#define UDC_DWC3_GUSB2PHYCFG_FSINTF				BIT(5)
#define UDC_DWC3_GUSB2PHYCFG_ULPI_UTMI_SEL			BIT(4)
#define UDC_DWC3_GUSB2PHYCFG_PHYIF				BIT(3)
#define UDC_DWC3_GUSB2PHYCFG_TOUTCAL_MASK			GENMASK(2, 0)

/* Global USB 3.0 PIPE Control Register */
#define UDC_DWC3_GUSB3PIPECTL					0xc2c0
#define UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST			BIT(31)
#define UDC_DWC3_GUSB3PIPECTL_UX_EXIT_IN_PX			BIT(27)
#define UDC_DWC3_GUSB3PIPECTL_PING_ENHANCEMENT_EN		BIT(26)
#define UDC_DWC3_GUSB3PIPECTL_U1U2EXITFAIL_TO_RECOV		BIT(25)
#define UDC_DWC3_GUSB3PIPECTL_REQUEST_P1P2P3			BIT(24)
#define UDC_DWC3_GUSB3PIPECTL_STARTXDETU3RXDET			BIT(23)
#define UDC_DWC3_GUSB3PIPECTL_DISRXDETU3RXDET			BIT(22)
#define UDC_DWC3_GUSB3PIPECTL_P1P2P3DELAY_MASK			GENMASK(21, 19)
#define UDC_DWC3_GUSB3PIPECTL_DELAYP0TOP1P2P3			BIT(18)
#define UDC_DWC3_GUSB3PIPECTL_SUSPENDENABLE			BIT(17)
#define UDC_DWC3_GUSB3PIPECTL_DATWIDTH_MASK			GENMASK(16, 15)
#define UDC_DWC3_GUSB3PIPECTL_ABORTRXDETINU2			BIT(14)
#define UDC_DWC3_GUSB3PIPECTL_SKIPRXDET				BIT(13)
#define UDC_DWC3_GUSB3PIPECTL_LFPSP0ALGN			BIT(12)
#define UDC_DWC3_GUSB3PIPECTL_P3P2TRANOK			BIT(11)
#define UDC_DWC3_GUSB3PIPECTL_P3EXSIGP2				BIT(10)
#define UDC_DWC3_GUSB3PIPECTL_LFPSFILT				BIT(9)
#define UDC_DWC3_GUSB3PIPECTL_TXSWING				BIT(6)
#define UDC_DWC3_GUSB3PIPECTL_TXMARGIN_MASK			GENMASK(5, 3)
#define UDC_DWC3_GUSB3PIPECTL_TXDEEMPHASIS_MASK			GENMASK(2, 1)
#define UDC_DWC3_GUSB3PIPECTL_ELASTICBUFFERMODE			BIT(0)

/* USB Device Configuration Register */
#define UDC_DWC3_DCFG						0xc700
#define UDC_DWC3_DCFG_IGNORESTREAMPP				BIT(23)
#define UDC_DWC3_DCFG_LPMCAP					BIT(22)
#define UDC_DWC3_DCFG_NUMP_MASK					GENMASK(21, 17)
#define UDC_DWC3_DCFG_INTRNUM_MASK				GENMASK(16, 12)
#define UDC_DWC3_DCFG_PERFRINT_MASK				GENMASK(11, 10)
#define UDC_DWC3_DCFG_PERFRINT_80				(0x0 << 10)
#define UDC_DWC3_DCFG_PERFRINT_85				(0x1 << 10)
#define UDC_DWC3_DCFG_PERFRINT_90				(0x2 << 10)
#define UDC_DWC3_DCFG_PERFRINT_95				(0x3 << 10)
#define UDC_DWC3_DCFG_DEVADDR_MASK				GENMASK(9, 3)
#define UDC_DWC3_DCFG_DEVSPD_MASK				GENMASK(2, 0)
#define UDC_DWC3_DCFG_DEVSPD_SUPER_SPEED			(0x4 << 0)
#define UDC_DWC3_DCFG_DEVSPD_HIGH_SPEED				(0x0 << 0)
#define UDC_DWC3_DCFG_DEVSPD_FULL_SPEED				(0x1 << 0)

/* Global SoC Bus Configuration Register */
#define UDC_DWC3_GSBUSCFG0					0xc100
#define UDC_DWC3_GSBUSCFG0_DATRDREQINFO				GENMASK(31, 28)
#define UDC_DWC3_GSBUSCFG0_DESRDREQINFO				GENMASK(27, 24)
#define UDC_DWC3_GSBUSCFG0_DATWRREQINFO				GENMASK(23, 20)
#define UDC_DWC3_GSBUSCFG0_DESWRREQINFO				GENMASK(19, 16)
#define UDC_DWC3_GSBUSCFG0_DATBIGEND				BIT(11)
#define UDC_DWC3_GSBUSCFG0_DESBIGEND				BIT(10)
#define UDC_DWC3_GSBUSCFG0_INCR256BRSTENA			BIT(7)
#define UDC_DWC3_GSBUSCFG0_INCR128BRSTENA			BIT(6)
#define UDC_DWC3_GSBUSCFG0_INCR64BRSTENA			BIT(5)
#define UDC_DWC3_GSBUSCFG0_INCR32BRSTENA			BIT(4)
#define UDC_DWC3_GSBUSCFG0_INCR16BRSTENA			BIT(3)
#define UDC_DWC3_GSBUSCFG0_INCR8BRSTENA				BIT(2)
#define UDC_DWC3_GSBUSCFG0_INCR4BRSTENA				BIT(1)
#define UDC_DWC3_GSBUSCFG0_INCRBRSTENA				BIT(0)

/* Global Tx Threshold Control Register */
#define UDC_DWC3_GTXTHRCFG					0xc108
#define UDC_DWC3_GTXTHRCFG_USBTXPKTCNTSEL			BIT(29)
#define UDC_DWC3_GTXTHRCFG_USBTXPKTCNT_MASK			GENMASK(27, 24)
#define UDC_DWC3_GTXTHRCFG_USBMAXTXBURSTSIZE_MASK		GENMASK(23, 16)

/* Global control register */
#define UDC_DWC3_GCTL						0xc110
#define UDC_DWC3_GCTL_PWRDNSCALE_MASK				GENMASK(31, 19)
#define UDC_DWC3_GCTL_MASTERFILTBYPASS				BIT(18)
#define UDC_DWC3_GCTL_BYPSSETADDR				BIT(17)
#define UDC_DWC3_GCTL_U2RSTECN					BIT(16)
#define UDC_DWC3_GCTL_FRMSCLDWN_MASK				GENMASK(15, 14)
#define UDC_DWC3_GCTL_PRTCAPDIR_MASK				GENMASK(13, 12)
#define UDC_DWC3_GCTL_CORESOFTRESET				BIT(11)
#define UDC_DWC3_GCTL_DEBUGATTACH				BIT(8)
#define UDC_DWC3_GCTL_RAMCLKSEL_MASK				GENMASK(7, 6)
#define UDC_DWC3_GCTL_SCALEDOWN_MASK				GENMASK(5, 4)
#define UDC_DWC3_GCTL_DISSCRAMBLE				BIT(3)
#define UDC_DWC3_GCTL_DSBLCLKGTNG				BIT(0)

/* Global User Control Register */
#define UDC_DWC3_GUCTL						0xc12c
#define UDC_DWC3_GUCTL_NOEXTRDL					BIT(21)
#define UDC_DWC3_GUCTL_PSQEXTRRESSP_MASK			GENMASK(20, 18)
#define UDC_DWC3_GUCTL_PSQEXTRRESSP_EN				BIT(18)
#define UDC_DWC3_GUCTL_SPRSCTRLTRANSEN				BIT(17)
#define UDC_DWC3_GUCTL_RESBWHSEPS				BIT(16)
#define UDC_DWC3_GUCTL_CMDEVADDR				BIT(15)
#define UDC_DWC3_GUCTL_USBHSTINAUTORETRYEN			BIT(14)
#define UDC_DWC3_GUCTL_DTCT_MASK				GENMASK(10, 9)
#define UDC_DWC3_GUCTL_DTFT_MASK				GENMASK(8, 0)

/* Global User Control Register 2 */
#define UDC_DWC3_GUCTL2						0xc19c
#define UDC_DWC3_GUCTL2_EN_HP_PM_TIMER				GENMASK(25, 19)
#define UDC_DWC3_GUCTL2_NOLOWPWRDUR				GENMASK(18, 15)
#define UDC_DWC3_GUCTL2_RST_ACTBITLATER				BIT(14)
#define UDC_DWC3_GUCTL2_ENABLEEPCACHEEVICT			BIT(12)
#define UDC_DWC3_GUCTL2_DISABLECFC				BIT(11)
#define UDC_DWC3_GUCTL2_RXPINGDURATION				GENMASK(10, 5)
#define UDC_DWC3_GUCTL2_TXPINGDURATION				GENMASK(4, 0)

/* USB Device Control register */
#define UDC_DWC3_DCTL						0xc704
#define UDC_DWC3_DCTL_RUNSTOP					BIT(31)
#define UDC_DWC3_DCTL_CSFTRST					BIT(30)
#define UDC_DWC3_DCTL_HIRDTHRES_4				BIT(28)
#define UDC_DWC3_DCTL_HIRDTHRES_TIME_MASK			GENMASK(27, 24)
#define UDC_DWC3_DCTL_APPL1RES					BIT(23)
#define UDC_DWC3_DCTL_LPM_NYET_THRES_MASK			GENMASK(23, 20)
#define UDC_DWC3_DCTL_KEEPCONNECT				BIT(19)
#define UDC_DWC3_DCTL_L1HIBERNATIONEN				BIT(18)
#define UDC_DWC3_DCTL_CRS					BIT(17)
#define UDC_DWC3_DCTL_CSS					BIT(16)
#define UDC_DWC3_DCTL_INITU2ENA					BIT(12)
#define UDC_DWC3_DCTL_ACCEPTU2ENA				BIT(11)
#define UDC_DWC3_DCTL_INITU1ENA					BIT(10)
#define UDC_DWC3_DCTL_ACCEPTU1ENA				BIT(9)
#define UDC_DWC3_DCTL_ULSTCHNGREQ_MASK				GENMASK(8, 5)
#define UDC_DWC3_DCTL_ULSTCHNGREQ_REMOTEWAKEUP			(0x8 << 5)
#define UDC_DWC3_DCTL_TSTCTL_MASK				GENMASK(4, 1)

/* USB Device Event Enable Register */
#define UDC_DWC3_DEVTEN						0xc708
#define UDC_DWC3_DEVTEN_INACTTIMEOUTRCVEDEN			BIT(13)
#define UDC_DWC3_DEVTEN_VNDRDEVTSTRCVEDEN			BIT(12)
#define UDC_DWC3_DEVTEN_EVNTOVERFLOWEN				BIT(11)
#define UDC_DWC3_DEVTEN_CMDCMPLTEN				BIT(10)
#define UDC_DWC3_DEVTEN_ERRTICERREN				BIT(9)
#define UDC_DWC3_DEVTEN_SOFEN					BIT(7)
#define UDC_DWC3_DEVTEN_EOPFEN					BIT(6)
#define UDC_DWC3_DEVTEN_HIBERNATIONREQEVTEN			BIT(5)
#define UDC_DWC3_DEVTEN_WKUPEVTEN				BIT(4)
#define UDC_DWC3_DEVTEN_ULSTCNGEN				BIT(3)
#define UDC_DWC3_DEVTEN_CONNECTDONEEN				BIT(2)
#define UDC_DWC3_DEVTEN_USBRSTEN				BIT(1)
#define UDC_DWC3_DEVTEN_DISCONNEVTEN				BIT(0)

/* USB Device Event Register */

/* Endpoint Global Event Buffer Address (64-bit) */
#define UDC_DWC3_GEVNTADR(n)					(0xc400 + 16 * (n))
#define UDC_DWC3_GEVNTADR_LO(n)					(0xc400 + 16 * (n))
#define UDC_DWC3_GEVNTADR_HI(n)					(0xc404 + 16 * (n))

/* Endpoint Global Event Buffer Size */
#define UDC_DWC3_GEVNTSIZ(n)					(0xc408 + 16 * (n))
#define UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK			BIT(31)

/* Endpoint Global Event Buffer Count (of valid event) */
#define UDC_DWC3_GEVNTCOUNT(n)					(0xc40c + 16 * (n))

/* USB Device Active USB Endpoint Enable */
#define UDC_DWC3_DALEPENA					0xC720
#define UDC_DWC3_DALEPENA_USBACTEP(n)				(1 << (n))

/* USB Device Core Identification and Release Number Register */
#define UDC_DWC3_GCOREID					0xC120
#define UDC_DWC3_GCOREID_CORE_MASK				GENMASK(31, 16)
#define UDC_DWC3_GCOREID_REL_MASK				GENMASK(15, 0)

/* USB Globa Status register */
#define UDC_DWC3_GSTS						0xc118
#define UDC_DWC3_GSTS_CBELT_MASK				GENMASK(31, 20)
#define UDC_DWC3_GSTS_SSIC_IP					BIT(11)
#define UDC_DWC3_GSTS_OTG_IP					BIT(10)
#define UDC_DWC3_GSTS_BC_IP					BIT(9)
#define UDC_DWC3_GSTS_ADP_IP					BIT(8)
#define UDC_DWC3_GSTS_HOST_IP					BIT(7)
#define UDC_DWC3_GSTS_DEVICE_IP					BIT(6)
#define UDC_DWC3_GSTS_CSRTIMEOUT				BIT(5)
#define UDC_DWC3_GSTS_BUSERRADDRVLD				BIT(4)
#define UDC_DWC3_GSTS_CURMOD_MASK				GENMASK(1, 0)

/* USB Global TX FIFO Size register */
#define UDC_DWC3_GTXFIFOSIZ(n)					(0xc300 + 4 * (n))
#define UDC_DWC3_GTXFIFOSIZ_TXFSTADDR_MASK			GENMASK(31, 16)
#define UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK				GENMASK(15, 0)

/* USB Global RX FIFO Size register */
#define UDC_DWC3_GRXFIFOSIZ(n)					(0xc380 + 4 * (n))
#define UDC_DWC3_GRXFIFOSIZ_RXFSTADDR_MASK			GENMASK(31, 16)
#define UDC_DWC3_GRXFIFOSIZ_RXFDEP_MASK				GENMASK(15, 0)

/* USB Bus Error Address registers */
#define UDC_DWC3_GBUSERRADDR					0xc130
#define UDC_DWC3_GBUSERRADDR_LO					0xc130
#define UDC_DWC3_GBUSERRADDR_HI					0xc134

/* USB Controller Debug register */
#define UDC_DWC3_CTLDEBUG					0xe000
#define UDC_DWC3_CTLDEBUG_LO					0xe000
#define UDC_DWC3_CTLDEBUG_HI					0xe004

/* USB Analyzer Trace register */
#define UDC_DWC3_ANALYZERTRACE					0xe008

/* USB Global Debug Queue/FIFO Space Available register */
#define UDC_DWC3_GDBGFIFOSPACE					0xc160
#define UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK			GENMASK(31, 16)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_MASK			GENMASK(8, 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXQ			(0x0 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXQ			(0x1 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXREQQ			(0x2 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXREQQ			(0x3 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXINFOQ		(0x4 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_PROTOCOLSTATUSQ	(0x5 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCHQ		(0x6 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_WREVENTQ		(0x7 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_AUXEVENTQ		(0x8 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK			GENMASK(4, 0)

/* USB Global Debug LTSSM register */
#define UDC_DWC3_GDBGLTSSM					0xc164

/* Global Debug LNMCC Register */
#define UDC_DWC3_GDBGLNMCC					0xc168

/* Global Debug BMU Register */
#define UDC_DWC3_GDBGBMU					0xc16c

/* Global Debug LSP MUX Register - Device*/
#define UDC_DWC3_GDBGLSPMUX_DEV					0xc170

/* Global Debug LSP MUX Register - Host */
#define UDC_DWC3_GDBGLSPMUX_HST					0xc170

/* Global Debug LSP Register */
#define UDC_DWC3_GDBGLSP					0xc174

/* Global Debug Endpoint Information Register 0 */
#define UDC_DWC3_GDBGEPINFO0					0xc178

/* Global Debug Endpoint Information Register 1 */
#define UDC_DWC3_GDBGEPINFO1					0xc17c

/* U3 Root Hub Debug Register */
#define UDC_DWC3_BU3RHBDBG0					0xd800

/* USB Device Status register */
#define UDC_DWC3_DSTS						0xC70C
#define UDC_DWC3_DSTS_DCNRD					BIT(29)
#define UDC_DWC3_DSTS_SRE					BIT(28)
#define UDC_DWC3_DSTS_RSS					BIT(25)
#define UDC_DWC3_DSTS_SSS					BIT(24)
#define UDC_DWC3_DSTS_COREIDLE					BIT(23)
#define UDC_DWC3_DSTS_DEVCTRLHLT				BIT(22)
#define UDC_DWC3_DSTS_USBLNKST_MASK				GENMASK(21, 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U0				(0x0 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U1				(0x1 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U2				(0x2 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_U3				(0x3 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_SS_DIS			(0x4 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_RX_DET			(0x5 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_SS_INACT			(0x6 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_POLL			(0x7 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_RECOV			(0x8 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_HRESET			(0x9 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_CMPLY			(0xa << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_LPBK			(0xb << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB3_RESET_RESUME		(0xf << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_ON_STATE			(0x0 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE			(0x2 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE		(0x3 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_DISCONNECTED		(0x4 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND		(0x5 << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_RESET			(0xe << 18)
#define UDC_DWC3_DSTS_USBLNKST_USB2_RESUME			(0xf << 18)
#define UDC_DWC3_DSTS_RXFIFOEMPTY				BIT(17)
#define UDC_DWC3_DSTS_SOFFN_MASK				GENMASK(16, 3)
#define UDC_DWC3_DSTS_CONNECTSPD_MASK				GENMASK(2, 0)
#define UDC_DWC3_DSTS_CONNECTSPD_HS				(0x0 << 0)
#define UDC_DWC3_DSTS_CONNECTSPD_FS				(0x1 << 0)
#define UDC_DWC3_DSTS_CONNECTSPD_SS				(0x4 << 0)

/* Device Generic Command and Parameter */
#define UDC_DWC3_DGCMDPAR					0xc710
#define UDC_DWC3_DGCMD						0xc714
#define UDC_DWC3_DGCMD_STATUS_MASK				GENMASK(15, 12)
#define UDC_DWC3_DGCMD_STATUS_ERR				(1 << 12)
#define UDC_DWC3_DGCMD_STATUS_OK				(0 << 12)
#define UDC_DWC3_DGCMD_ACT					BIT(10)
#define UDC_DWC3_DGCMD_IOC					BIT(8)
#define UDC_DWC3_DGCMD_MASK					GENMASK(7, 0)
/* EXITLATENCY command and parameters */
#define UDC_DWC3_DGCMD_EXITLATENCY				(2 << 0)
/* Other Commands and Parameters */
#define UDC_DWC3_DGCMD_LINKFUNCTION				(1 << 0)
#define UDC_DWC3_DGCMD_WAKENOTIFNUM				(3 << 0)
#define UDC_DWC3_DGCMD_FIFOFLUSHONE				(9 << 0)
/*
 * DGCMDPAR for FIFOFLUSHONE: bits[4:0] are the FIFO number and bit 5 selects
 * TX. Without bit 5 the command is still legal and still reports success --
 * it flushes RxFIFO n instead. The reference tree lost three soak attempts to
 * exactly that, with the bytes left sitting in the TX FIFO it meant to clear.
 */
#define UDC_DWC3_DGCMDPAR_FIFO_NUM(n)				((n) << 0)
#define UDC_DWC3_DGCMDPAR_RX_FIFO				(0 << 5)
#define UDC_DWC3_DGCMDPAR_TX_FIFO				(1 << 5)
#define UDC_DWC3_DGCMD_FIFOFLUSHALL				(10 << 0)
#define UDC_DWC3_DGCMD_ENDPOINTNRDY				(12 << 0)
#define UDC_DWC3_DGCMD_LOOPBACKTEST				(16 << 0)
#define UDC_DWC3_DGCMD_ROLEREQUEST				(6 << 0)

/* Hardware parameters */
#define UDC_DWC3_GHWPARAMS0					0xc140
#define UDC_DWC3_GHWPARAMS1					0xc144
#define UDC_DWC3_GHWPARAMS2					0xc148
#define UDC_DWC3_GHWPARAMS3					0xc14c
#define UDC_DWC3_GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES_MASK	GENMASK(30, 23)
#define UDC_DWC3_GHWPARAMS3_NUM_IN_EPS_MASK			GENMASK(22, 18)
#define UDC_DWC3_GHWPARAMS3_NUM_EPS_MASK			GENMASK(17, 12)
#define UDC_DWC3_GHWPARAMS4					0xc150
#define UDC_DWC3_GHWPARAMS4_BMU_LSP_DEPTH_MASK			GENMASK(31, 28)
#define UDC_DWC3_GHWPARAMS4_BMU_PTL_DEPTH_M1_MASK		GENMASK(27, 24)
#define UDC_DWC3_GHWPARAMS4_CACHE_TRBS_PER_TRANSFER_MASK	GENMASK(5, 0)
#define UDC_DWC3_GHWPARAMS5					0xc154
#define UDC_DWC3_GHWPARAMS5_DFQ_FIFO_DEPTH_MASK			GENMASK(27, 22)
#define UDC_DWC3_GHWPARAMS5_DWQ_FIFO_DEPTH_MASK			GENMASK(21, 16)
#define UDC_DWC3_GHWPARAMS5_TXQ_FIFO_DEPTH_MASK			GENMASK(15, 10)
#define UDC_DWC3_GHWPARAMS5_RXQ_FIFO_DEPTH_MASK			GENMASK(9, 4)
#define UDC_DWC3_GHWPARAMS5_BMU_BUSGM_DEPTH_MASK		GENMASK(3, 0)
#define UDC_DWC3_GHWPARAMS6					0xc158
#define UDC_DWC3_GHWPARAMS6_RAM0_DEPTH_MASK			GENMASK(31, 16)
#define UDC_DWC3_GHWPARAMS6_PSQ_FIFO_DEPTH_MASK			GENMASK(5, 0)
#define UDC_DWC3_GHWPARAMS7					0xc15c
#define UDC_DWC3_GHWPARAMS7_RAM2_DEPTH_MASK			GENMASK(31, 16)
#define UDC_DWC3_GHWPARAMS7_RAM1_DEPTH_MASK			GENMASK(15, 0)
#define UDC_DWC3_GHWPARAMS8					0xc600

/* Helper macros */
#define LO32(n)			((uint32_t)((uint64_t)(n) & 0xffffffff))
#define HI32(n)			((uint32_t)((uint64_t)(n) >> 32))
#define _EP_DATA_FROM_EPN(cfg, epn) \
	(((epn) & 1) ? &(cfg)->ep_data_in[(epn) >> 1] : &(cfg)->ep_data_out[(epn) >> 1])
#define _NUM_FIFO_SPACE 16
#define _NUM_AUX_EVENT 8
#define _NUM_FIFO_REGS 7

/*
 * One DMA transaction request passed from the CPU to the DWC3 core.
 *
 * This structure is described by the datasheet of DWC3 and shared between the hardware and
 * software driver. If the architecture involves cache, it must be flushed before accessing
 * this memory region.
 */
struct udc_dwc3_trb {
	uint32_t addr_lo;
	uint32_t addr_hi;
	uint32_t status;
	uint32_t ctrl;
} __packed __aligned(16);

/*
 * Controller configuration items that can remain in non-volatile memory
 */
struct udc_dwc3_config {
	DEVICE_MMIO_NAMED_ROM(base);
	/* USB endpoints data */
	struct udc_dwc3_ep_data *ep_data_in;
	struct udc_dwc3_ep_data *ep_data_out;
	/* Pointer to the DMA-accessible buffer of TRBs and its size */
	struct udc_dwc3_trb (*trb_buf_in)[CONFIG_UDC_DWC3_TRB_NUM];
	struct udc_dwc3_trb (*trb_buf_out)[CONFIG_UDC_DWC3_TRB_NUM];
	/* USB device configuration */
	int maximum_speed_idx;
	/* Pointers to event buffer fetched by DWC3 with DMA */
	volatile uint32_t *evt_buf;
	/* Data used by vendor-specific functions ("quirks") */
	const void *quirk_config;
	void *quirk_data;
	/* IRQ management functions */
	void (*irq_enable_func)(void);
	void (*irq_disable_func)(void);
	/* Number of hardware endpoint set for input or output */
	uint8_t num_in_eps;
	uint8_t num_out_eps;
	/* Communicate state between IRQs and API functions */
	atomic_t flags;
};

/*
 * All data specific to one endpoint for use by the driver.
 */
struct udc_dwc3_ep_data {
	/* Allow to cast a pointer between ep_data and ep_cfg */
	struct udc_ep_config cfg;
	/* Endpoint number (physical address): the logical address is on ep_cfg */
	int epn;
	/* A work queue entry to process the buffers to submit on that endpoint */
	struct k_work work;
	/* To re-queue cancelled buffers after an endpoint is disabled */
	struct k_fifo requeue_fifo;
	/* Point back to the device for work queues */
	const struct device *dev;
	/* Buffer of pointers to net_buf, with index matching the position in the TRB buffers */
	struct net_buf *net_buf[CONFIG_UDC_DWC3_TRB_NUM];
	/* Buffer of TRB structures, with index matching the position in the net_buf buffers */
	struct udc_dwc3_trb *trb_buf;
	/* Index of the next TRB to receive data in the TRB ring, Link TRB excluded */
	uint32_t head;
	uint32_t tail;
	/* A flag to tell when the ring buffer is full */
	bool full;
	/* Given by the hardware for use in endpoint commands */
	uint32_t xferrscidx;
	/*
	 * True while the video RTL (UsbMgr / TRBRamSink) owns this endpoint:
	 * it writes the TRBs and rings DEPCMD(UpdateXfer) itself, roughly
	 * 56,000 times a second per streaming lane.
	 *
	 * DWC3 has ONE endpoint-command engine. A CPU-side EndXfer, ClearStall
	 * or StartXfer issued while the RTL is driving that same endpoint
	 * collides on it, the CPU command never retires against the right
	 * transfer, and the endpoint never reaches Stopped. The host's xHCI
	 * Stop Endpoint then times out after 5 s and Linux declares the
	 * controller dead -- six times on this bench on 2026-11-09, always
	 * logging "xHCI host not responding to stop endpoint command" with
	 * nothing device-side before it.
	 *
	 * The reference stack avoids this by refusing every CPU-side doorbell,
	 * stall and disable while the RTL owns the endpoint (its frozen rule
	 * 3) and by halting the manager before any command it must issue. The
	 * ownership API in udc_dwc3_lattice_usb23.h was already declared in
	 * this tree; only the enforcement was missing, so the video driver
	 * took the TRB address and the doorbell word via the getters and
	 * started the manager without the UDC ever knowing.
	 */
	bool hw_owned;
	/* CSR base of the UsbMgr instance driving this endpoint, or 0. */
	uintptr_t hw_mgr_base;
#if defined(CONFIG_UDC_DWC3_PARK_NUDGE)
	/* Park detection: tail and remaining count last seen, and for how long */
	uint32_t park_tail;
	uint32_t park_remain;
	uint16_t park_ticks;
	uint8_t park_nudges;
	/* Set once the escalation ladder has been run and did not recover */
	uint8_t park_exhausted;
#endif
	/*
	 * Whether DEPXFERCFG has allocated a transfer resource for this
	 * endpoint out of the CURRENT pool generation.
	 *
	 * DWC3 hands out transfer resources from a pool opened by
	 * DEPSTARTCFG(XferRscIdx=2), sized once for every non-control
	 * endpoint. DEPXFERCFG draws one. Drawing again without reopening the
	 * pool runs past it: the reference tree saw "endpoint command failed"
	 * four times over and one endpoint fall back to a STALE INDEX once the
	 * endpoint count reached 9.
	 *
	 * udc_dwc3_ep_resume() issues DEPXFERCFG on every enable, and
	 * udc_dwc3_on_set_config_or_interface() reopens the pool on every
	 * SetConfiguration AND every SetInterface -- which UVC does on each
	 * alt-setting switch, i.e. every stream start and stop. So the two run
	 * against each other continuously once video and CDC-ACM share the
	 * device, and not at all when ACM runs alone.
	 *
	 * This flag makes DEPXFERCFG once-per-endpoint-per-pool: set when the
	 * resource is drawn, cleared for every non-control endpoint when the
	 * pool is reopened.
	 */
	bool xfer_rsc_allocated;
};

/*
 * Data of each instance of the driver, that can be read and written to.
 *
 * Accessed via "udc_get_private(dev)".
 */
struct udc_dwc3_data {
	DEVICE_MMIO_NAMED_RAM(base);
	/* Index within trb where to queue new TRBs */
	uint32_t evt_next;
	/* Back-reference to parent */
	const struct device *dev;
	/* Dispatch from IRQ events to workqueue jobs */
	struct k_work event_work;
	/* First endpoint to be configured */
	uint8_t first_ep;
	/* CPU-side endpoint commands refused because the RTL owned the EP. */
	uint32_t hw_owned_refused;
	/* Managers that did not report idle within the halt timeout. */
	uint32_t hw_mgr_halt_timeouts;
	/* Notified after ownership is revoked, so video can stop its pipeline */
	lattice_usb23_hw_owner_cb_t hw_owner_cb;
	void *hw_owner_user;
#if defined(CONFIG_UDC_DWC3_COMPLETION_POLL)
	/* Periodic recovery of completions the controller never reported */
	struct k_work_delayable poll_work;
	uint8_t poll_grace_out[8];
	uint8_t poll_grace_in[8];
	uint32_t poll_retired_out;
	uint32_t poll_retired_in;
#if defined(CONFIG_UDC_DWC3_PARK_NUDGE)
	uint32_t park_nudged_out;
	uint32_t park_nudged_in;
#endif
#endif
#if CONFIG_UDC_DWC3_SHELL
	/* FIFO space initial values */
	uint16_t max_bytes_avail[_NUM_FIFO_SPACE][_NUM_FIFO_REGS];
#endif
};

/*
 * Indexes matching the "device-speed" devicetree property values.
 */
enum {
	UDC_DWC3_SPEED_IDX_FULL_SPEED = 1,
	UDC_DWC3_SPEED_IDX_HIGH_SPEED = 2,
	UDC_DWC3_SPEED_IDX_SUPER_SPEED = 3,
};

/*
 * Vendor quirks
 *
 * Definition of vendor-specific functions that can be overwritten on a per-SoC basis.
 */

struct udc_dwc3_vendor_quirks {
	int (*preinit)(const struct device *const dev);
	int (*init)(const struct device *const dev);
	int (*enable)(const struct device *const dev);
	int (*disable)(const struct device *const dev);
	int (*shutdown)(const struct device *const dev);
};

/* Helper for accessing vendor quirks */
#define UDC_DWC3_QUIRK_CFG(dev)  (((const struct udc_dwc3_config *)(dev->config))->quirk_config)
#define UDC_DWC3_QUIRK_DATA(dev) (((const struct udc_dwc3_config *)(dev->config))->quirk_data)

#if DT_HAS_COMPAT_STATUS_OKAY(snps_dwc3 /* <- replace with your more specific compatible */)
#include "udc_dwc3_lattice_usb23.h"
#endif

/* Wrapper functions that fallback to returning 0 if no quirk is needed */
#define UDC_DWC3_QUIRK_FUNC_DEFINE(fn)						\
	static inline int udc_dwc3_quirk_##fn(const struct device *const dev)	\
	{									\
		if (udc_dwc3_vendor_quirks.fn != NULL) {			\
			return udc_dwc3_vendor_quirks.fn(dev);			\
		}								\
										\
		return 0;							\
	}

UDC_DWC3_QUIRK_FUNC_DEFINE(preinit);
UDC_DWC3_QUIRK_FUNC_DEFINE(init);
UDC_DWC3_QUIRK_FUNC_DEFINE(enable);
UDC_DWC3_QUIRK_FUNC_DEFINE(disable);
UDC_DWC3_QUIRK_FUNC_DEFINE(shutdown);

#define DEV_CFG(dev) ((const struct udc_dwc3_config *)(dev->config))
#define DEV_DATA(dev) ((struct udc_dwc3_data *)udc_get_private(dev))

static int udc_dwc3_set_address(const struct device *const dev, const uint8_t addr);
static int udc_dwc3_ep_disable(const struct device *const dev, struct udc_ep_config *const ep_cfg);
static int udc_dwc3_ep_resume(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data);
#ifdef CONFIG_UDC_DWC3_SHELL
static void udc_dwc3_init_fifo_space(const struct device *dev);
#endif

/* Shut down the controller completely  */
static int udc_dwc3_shutdown(const struct device *const dev)
{
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

static void udc_dwc3_lock(const struct device *const dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_dwc3_unlock(const struct device *const dev)
{
	udc_unlock_internal(dev);
}

/*
 * Commands
 *
 * The DEPCMD register acts as a command interface, where a command number
 * is written along with parameters, an action is performed and a CMDACT bit
 * is reset whenever the command completes.
 */

/*
 * DEPCMD arbiter mailbox.
 *
 * DWC3 has one endpoint-command engine. On the tinyCLUNX quad build the FPGA
 * also drives it: each of the four TRBRamSink lanes used to run its own
 * "poll CMDACT clear, write with CMDACT set, poll until clear" sequence on
 * DEPCMD, and so did this function. Five independent agents doing a
 * read-modify-poll on one register, over a single-outstanding LMMI port, is a
 * race: any of them can observe CMDACT clear, lose it to another writer, and
 * have its command dropped or applied to the wrong endpoint.
 *
 * DepCmdArbiter is now the sole writer. The RTL lanes post on internal
 * ports and the CPU posts here, lowest port first. The arbiter sets CMDACT
 * itself, so the word posted below must not carry it.
 *
 * MBX_TRB arms the arbiter's packet hold, and NOT arming it was a mistake.
 *
 * The hold keeps the next video doorbell off the pipe while a bulk IN short
 * packet is still in its TX FIFO (FIFO 2 for 0x82). Without it, a video
 * UpdateXfer can run before the packet has left, DWC3 DROPS that transfer, the
 * TRB stays HWO with its remaining count at the full length, and the endpoint
 * parks. That is exactly the wedge measured on 2026-11-09: 0x82 frozen at
 * remain=11, our 11-byte echo frame, with video unaffected.
 *
 * This file previously skipped the write, reasoning that the hold's release
 * needs the completion-event snoop and that the snoop needs UsbEngine, "which
 * this design does not build". The premise is wrong: the snoop is wired
 * unconditionally in TinyClunx.scala (evtPend/xferIp/takeXferIp), and
 * UsbEngine is instantiated unconditionally. So the release path exists.
 *
 * The arm term is xferTrb = (startXfer || updateXfer) && !isEp0 && mbxTrb != 0,
 * and acmIn = xferTrb && (physEp === 5) covers 0x82. This tree's arbiter also
 * carries the hold02 split, so the packet hold arms WITHOUT QUIET_OTHER, and
 * holdMin is 0 for earlyIn -- the hold ends on the endpoint's own completion
 * event rather than a timer, so it imposes no flat minimum and costs no
 * frames. QUIET_OTHER (reset 0 in this tree, 60000 in the reference) gates
 * only the flat everyOther/flatRest holds, which are a separate mechanism.
 *
 * One-shot, written between MBX_ADDR and the posting MBX_DATA write, and
 * re-armed per command by the caller.
 *
 * Designs whose device tree has no mailbox node keep the direct write.
 */
#define UDC_DWC3_DEPCMD_MBX_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(tinyvision_depcmd_mailbox)

#if DT_NODE_EXISTS(UDC_DWC3_DEPCMD_MBX_NODE)
#define UDC_DWC3_DEPCMD_MBX_BASE DT_REG_ADDR(UDC_DWC3_DEPCMD_MBX_NODE)
#define UDC_DWC3_MBX_ADDR	 (UDC_DWC3_DEPCMD_MBX_BASE + 0x00)
#define UDC_DWC3_MBX_DATA	 (UDC_DWC3_DEPCMD_MBX_BASE + 0x04)
#define UDC_DWC3_MBX_STATUS	 (UDC_DWC3_DEPCMD_MBX_BASE + 0x08)
#define UDC_DWC3_MBX_RESULT	 (UDC_DWC3_DEPCMD_MBX_BASE + 0x0c)
#define UDC_DWC3_MBX_TRB	 (UDC_DWC3_DEPCMD_MBX_BASE + 0x14)
/*
 * udc_dwc3_depcmd() returns the command's XFERRSCIDX, so it cannot use 0 to
 * mean failure: 0 is a legitimate index, and StartXfer assigns the return
 * straight into ep_data->xferrscidx. A failed post that returned 0 therefore
 * POISONED the index, and every later UpdateXfer was posted as 0x...7 with
 * XFERRSCIDX=0, which matches no transfer resource, so DWC3 never retired it
 * and CMDACT hung. Measured 2026-11-09: MBX_DATA 0x30007 (idx 3) before the
 * wedge, 0x7 (idx 0) at it, MBX_RESULT 0, MBX_STATUS 0x6, seven CMDACT
 * timeouts per 300 s run.
 */
#define UDC_DWC3_DEPCMD_FAILED 0xFFFFFFFFU

/* Armed by udc_dwc3_mbx_arm_trb() for the next post only. */
static uint32_t udc_dwc3_mbx_trb;

#if defined(CONFIG_UDC_DWC3_COMPLETION_POLL)
/*
 * Software recovery must never be aimed at an endpoint the video RTL drives:
 * a nudge or a retire there would act on the video ring's own transfer.
 */
BUILD_ASSERT((CONFIG_UDC_DWC3_IN_RECOVER_EP_MASK &
	      CONFIG_UDC_DWC3_HW_IN_EP_MASK) == 0,
	     "UDC_DWC3_IN_RECOVER_EP_MASK must not include RTL (HW_IN) endpoints");
#endif
#define UDC_DWC3_MBX_STATUS_BUSY BIT(0)
#define UDC_DWC3_MBX_STATUS_DONE BIT(1)
#define UDC_DWC3_MBX_STATUS_ERR	 BIT(2)
#endif

/*
 * Arm the arbiter's packet hold for the next DEPCMD on this endpoint.
 *
 * Limited to the endpoints IN_RECOVER_EP_MASK names, which by construction
 * excludes the RTL-owned video endpoints: arming on those would hold a video
 * doorbell against the video ring's own transfer.
 */
static void udc_dwc3_mbx_arm_trb(const struct udc_dwc3_ep_data *const ep_data,
				 const void *const trb)
{
#if DT_NODE_EXISTS(UDC_DWC3_DEPCMD_MBX_NODE) && \
	defined(CONFIG_UDC_DWC3_ARM_PACKET_HOLD)
	if (!USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		return;
	}
	if ((CONFIG_UDC_DWC3_IN_RECOVER_EP_MASK &
	     BIT(USB_EP_GET_IDX(ep_data->cfg.addr))) == 0U) {
		return;
	}

	udc_dwc3_mbx_trb = LO32((uintptr_t)trb);
#else
	ARG_UNUSED(ep_data);
	ARG_UNUSED(trb);
#endif
}

static uint32_t udc_dwc3_depcmd(const struct device *const dev,
				const uint32_t addr, const uint32_t cmd)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(1000));
	uint32_t reg;

#if DT_NODE_EXISTS(UDC_DWC3_DEPCMD_MBX_NODE)
	/*
	 * There is exactly one poster, and a DATA write while the mailbox is
	 * BUSY is DROPPED by the RTL:
	 *
	 *   when(io.mbxPost && !mbxBusy) { ...; mbxDone := False }
	 *
	 * The post is ignored and mbxDone keeps the PREVIOUS command's value.
	 * The first cut of this function wrote ADDR and DATA with no BUSY
	 * check and no mutual exclusion, then polled for DONE -- so a post
	 * made while busy was silently discarded and the stale DONE was read
	 * back as this command's own completion, returning the previous
	 * command's result. Every caller then believed a command had executed
	 * when it had never reached DWC3.
	 *
	 * That is consistent with what the bench showed on 2026-11-09: a
	 * four-rung park-recovery ladder (TX FIFO flush, EndXfer+StartXfer,
	 * DEPCFG MODIFY, SetStall/ClearStall) in which every rung reported
	 * success while the endpoint stayed frozen at a constant BUFSIZ, and
	 * MBX_STATUS latching ERR.
	 *
	 * So: wait for !BUSY, post ADDR and DATA with interrupts locked so the
	 * pair cannot interleave with a post from another context, then
	 * confirm the post was ACCEPTED (BUSY or DONE observed) before waiting
	 * for completion. An immediate !BUSY must not be read as completion.
	 */
	unsigned int key;

	for (;;) {
		key = irq_lock();
		if ((sys_read32(UDC_DWC3_MBX_STATUS) & UDC_DWC3_MBX_STATUS_BUSY) == 0) {
			break;
		}
		irq_unlock(key);

		if (sys_timepoint_expired(end)) {
			LOG_ERR("depcmd mailbox busy, cmd 0x%x dropped", cmd);
			return UDC_DWC3_DEPCMD_FAILED;
		}
	}

	/* MBX_ADDR takes the full byte address; the MBX_DATA write posts.
	 * MBX_TRB must land between the two, and is one-shot.
	 */
	sys_write32(base + addr, UDC_DWC3_MBX_ADDR);
	if (udc_dwc3_mbx_trb != 0U) {
		sys_write32(udc_dwc3_mbx_trb, UDC_DWC3_MBX_TRB);
		udc_dwc3_mbx_trb = 0U;
	}
	sys_write32(cmd, UDC_DWC3_MBX_DATA);
	irq_unlock(key);

	/* BUSY is registered on the DATA write, so confirm acceptance first. */
	do {
		reg = sys_read32(UDC_DWC3_MBX_STATUS);

		if (sys_timepoint_expired(end)) {
			LOG_ERR("depcmd mailbox post not accepted, cmd 0x%x sts 0x%x",
				cmd, reg);
			return UDC_DWC3_DEPCMD_FAILED;
		}
	} while ((reg & (UDC_DWC3_MBX_STATUS_BUSY | UDC_DWC3_MBX_STATUS_DONE |
			 UDC_DWC3_MBX_STATUS_ERR)) == 0);

	while ((reg & UDC_DWC3_MBX_STATUS_BUSY) != 0) {
		reg = sys_read32(UDC_DWC3_MBX_STATUS);

		if (sys_timepoint_expired(end)) {
			LOG_ERR("depcmd mailbox never completed, cmd 0x%x sts 0x%x",
				cmd, reg);
			return UDC_DWC3_DEPCMD_FAILED;
		}
	}

	if ((reg & UDC_DWC3_MBX_STATUS_ERR) != 0) {
		/* The arbiter's CMDACT poll timed out: the command reached DWC3
		 * and DWC3 never retired it. Distinct from a dropped post.
		 */
		LOG_ERR("depcmd CMDACT timeout in arbiter, cmd 0x%x addr 0x%x", cmd, addr);
	}

	reg = sys_read32(UDC_DWC3_MBX_RESULT);
#else
	sys_write32(cmd | UDC_DWC3_DEPCMD_CMDACT, base + addr);

	do {
		reg = sys_read32(base + addr);

		if (sys_timepoint_expired(end)) {
			LOG_ERR("command expired");
			break;
		}
	} while ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0);
#endif

	switch (reg & UDC_DWC3_DEPCMD_STATUS_MASK) {
	case UDC_DWC3_DEPCMD_STATUS_OK:
		break;
	case UDC_DWC3_DEPCMD_STATUS_CMDERR:
		LOG_ERR("endpoint command 0x%x, addr 0x%x failed (0x%08x)", cmd, addr, reg);
		break;
	default:
		LOG_ERR("command failed with unknown status: 0x%08x", reg);
	}

	return FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);
}

static void udc_dwc3_depcmd_ep_config(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t param0 = 0;
	uint32_t param1 = 0;

	LOG_INF("Configuring endpoint 0x%02x with wMaxPacketSize=%u",
		ep_data->cfg.addr, ep_data->cfg.mps);

	if (ep_data->cfg.stat.enabled) {
		LOG_DBG("UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MODIFY");
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MODIFY;
	} else {
		LOG_DBG("UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_INIT");
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_INIT;
	}

	switch (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) {
	case USB_EP_TYPE_CONTROL:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_CTRL;
		break;
	case USB_EP_TYPE_BULK:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_BULK;
		break;
	case USB_EP_TYPE_INTERRUPT:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_INT;
		break;
	case USB_EP_TYPE_ISO:
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_EPTYPE_ISOC;
		break;
	default:
		CODE_UNREACHABLE;
	}

	/* Max Packet Size according to the USB descriptor configuration */
	param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_MPS_MASK, ep_data->cfg.mps);

	/* Burst Size of a single packet per burst (encoded as '0'): no burst */
	param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 15);

	/* Set the FIFO number, must be 0 for all OUT EPs */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_FIFONUM_MASK,
				     ep_data->cfg.addr & 0x7f);
	}

	/* Per-endpoint events */
	/*
	 * Soft-IP UVC: skip XferComplete/XferInProgress on endpoints the
	 * video RTL drives, or the event ring floods.
	 *
	 * The RTL writes those TRBs and retires them itself, so each event is
	 * one the CPU has no use for -- and two 1640x1232 lanes post them by
	 * the tens of thousands per second. The ring then has no room for the
	 * ACM completions that do matter, the class never learns a transfer
	 * finished, and the endpoint parks. Raising the ring from 16 to 128
	 * slots only buys headroom against that flood; this removes it.
	 */
	if ((USB_EP_DIR_IS_IN(ep_data->cfg.addr) &&
	     (CONFIG_UDC_DWC3_HW_IN_EP_MASK &
	      BIT(USB_EP_GET_IDX(ep_data->cfg.addr))) != 0U)) {
		LOG_DBG("ep 0x%02x is RTL-owned: transfer events masked",
			ep_data->cfg.addr);
	} else {
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN;
	}
	/* To log CTRL DATA/STATUS messages: UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN */

	/* This is the usb protocol endpoint number, but the data encoding
	 * we chose for physical endpoint number is the same as this register
	 */
	param1 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR1_DEPCFG_EPNUMBER_MASK, ep_data->epn);

	sys_write32(param0, base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(param1, base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPCFG);
}

static void udc_dwc3_depcmd_ep_xfer_config(const struct device *const dev,
					   struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	/*
	 * One transfer resource per endpoint per pool generation. Control
	 * endpoints are not drawn from the DEPSTARTCFG(2) pool, so they are
	 * exempt and keep the previous unconditional behaviour.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) != 0 && ep_data->xfer_rsc_allocated) {
		LOG_DBG("DepXferConfig: ep=0x%02x already holds a transfer resource, skipping",
			ep_data->cfg.addr);
		return;
	}

	LOG_DBG("DepXferConfig: ep=0x%02x", ep_data->cfg.addr);

	reg = FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK, 1);
	sys_write32(reg, base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPXFERCFG);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) != 0) {
		ep_data->xfer_rsc_allocated = true;
	}
}

/*
 * UsbMgr control register, mirroring tvai_uvcmanager.c. Kept here rather than
 * shared through a header because the UDC only ever needs to stop a manager
 * and see it go idle; the video driver owns everything else about it.
 */
#define UDC_DWC3_UVCMGR_CONTROL_STATUS		0x0010U
#define UDC_DWC3_UVCMGR_CONTROL_ENABLE		BIT(0)
#define UDC_DWC3_UVCMGR_CONTROL_BUSY		BIT(1)
#define UDC_DWC3_UVCMGR_CONTROL_CONTINUE	BIT(14)

/* How long to wait for a halted manager to drop BUSY. One video TRB is about
 * a kilobyte at SuperSpeed, so this is thousands of times the expected wait;
 * it is a backstop against a manager that is already wedged, not a tuning
 * knob. Timing out is counted and reported, never ignored.
 */
#define UDC_DWC3_UVCMGR_HALT_TIMEOUT_US		2000U

/*
 * Stop the manager driving @p ep_data and wait until it reports idle, so the
 * caller can issue an endpoint command without racing it.
 *
 * Our RTL has no HALT_DOORBELL/HALT_ACK handshake (the reference RTL does:
 * CONTROL bits 15 and 16, which defer the doorbell while video data keeps
 * flowing into the FIFO). TRBRamSink has no halt at all, so the coarse
 * ENABLE=0 + poll BUSY is what this platform can offer today. It costs the
 * frames in flight, which is why it is used only on the rare quiesce paths
 * where the host is already tearing the endpoint down -- never per packet.
 */
static int udc_dwc3_hw_mgr_halt(const struct device *const dev,
				struct udc_dwc3_ep_data *const ep_data)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uintptr_t mgr = ep_data->hw_mgr_base;

	if (mgr == 0U) {
		return 0;
	}

	sys_write32(0U, mgr + UDC_DWC3_UVCMGR_CONTROL_STATUS);

	for (uint32_t us = 0; us < UDC_DWC3_UVCMGR_HALT_TIMEOUT_US; us += 10U) {
		if ((sys_read32(mgr + UDC_DWC3_UVCMGR_CONTROL_STATUS) &
		     UDC_DWC3_UVCMGR_CONTROL_BUSY) == 0U) {
			return 0;
		}
		k_busy_wait(10);
	}

	priv->hw_mgr_halt_timeouts++;
	LOG_ERR("UsbMgr at 0x%lx still busy %u us after halt; ep 0x%02x command "
		"will race it", (unsigned long)mgr, UDC_DWC3_UVCMGR_HALT_TIMEOUT_US,
		ep_data->cfg.addr);
	return -ETIMEDOUT;
}

/*
 * Refuse a CPU-side endpoint command on an RTL-owned endpoint.
 *
 * Returns true when the caller must do nothing. Callers that legitimately
 * need the command first revoke ownership (udc_dwc3_hw_owned_revoke), which
 * halts the manager; anything else reaching here is a driver bug and is
 * counted so a soak can assert on it.
 */
static bool udc_dwc3_ep_refuse_if_hw_owned(const struct device *const dev,
					   struct udc_dwc3_ep_data *const ep_data,
					   const char *const what)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	if (!ep_data->hw_owned) {
		return false;
	}

	priv->hw_owned_refused++;
	LOG_ERR("ep 0x%02x RTL-owned: refusing %s (refused=%u)",
		ep_data->cfg.addr, what, priv->hw_owned_refused);
	return true;
}

/*
 * Device generic command: write DGCMDPAR, then DGCMD with CMDACT, poll until
 * it clears. Same shape and the same 500-iteration bound as Linux's
 * dwc3_send_gadget_generic_command(). Nothing in this driver issued a DGCMD
 * before the TX-FIFO flush recovery below needed one.
 */
__maybe_unused static int udc_dwc3_dgcmd(const mm_reg_t base, const uint32_t cmd,
					 const uint32_t param)
{
	sys_write32(param, base + UDC_DWC3_DGCMDPAR);
	sys_write32(cmd | UDC_DWC3_DGCMD_ACT, base + UDC_DWC3_DGCMD);

	for (uint32_t i = 0U; i < 500U; i++) {
		const uint32_t reg = sys_read32(base + UDC_DWC3_DGCMD);

		if ((reg & UDC_DWC3_DGCMD_ACT) == 0U) {
			return ((reg & UDC_DWC3_DGCMD_STATUS_MASK) ==
				UDC_DWC3_DGCMD_STATUS_OK) ? 0 : -EIO;
		}
	}

	return -ETIMEDOUT;
}

static void udc_dwc3_depcmd_set_stall(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	if (udc_dwc3_ep_refuse_if_hw_owned(dev, ep_data, "SetStall")) {
		return;
	}

	LOG_DBG("DepSetStall: ep=0x%02x", ep_data->cfg.addr);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPSETSTALL);
}

static void udc_dwc3_depcmd_clear_stall(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data,
					uint32_t flags)
{
	if (udc_dwc3_ep_refuse_if_hw_owned(dev, ep_data, "ClearStall")) {
		return;
	}

	LOG_DBG("DepClearStall ep=0x%02x", ep_data->cfg.addr);

	/*
	 * Callers must not pass HIPRI_FORCERM here.
	 *
	 * Bit 11 is ForceRM, which DWC3 defines only for DEPENDXFER; Linux
	 * issues CLEARSTALL bare, and so does the reference tree. Both call
	 * sites in this file used to set it, and a ClearStall that never
	 * retires leaves a halted endpoint that cannot be un-halted -- a dead
	 * ACM by definition.
	 *
	 * Measured 2026-11-09: after the XFERRSCIDX poisoning was fixed, the
	 * arbiter's CMDACT timeout moved from UpdateXfer (MBX_DATA 0x7) to
	 * this command (MBX_DATA 0x5), with MBX_RESULT 0 and MBX_STATUS 0x6.
	 * The reference's own note on the matching EndXfer case is that
	 * "ForceRM is what is fatal with words loaded", which is exactly the
	 * state a stalled bulk IN is in.
	 */
	__ASSERT((flags & UDC_DWC3_DEPCMD_HIPRI_FORCERM) == 0,
		 "ForceRM is not valid on ClearStall");

	flags |= UDC_DWC3_DEPCMD_DEPCSTALL;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);
}

static void udc_dwc3_depcmd_start_xfer(const struct device *const dev,
				       struct udc_dwc3_ep_data *const ep_data)
{
	if (udc_dwc3_ep_refuse_if_hw_owned(dev, ep_data, "StartXfer")) {
		return;
	}

	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;
	uint32_t rsc;

	/* Make sure the device is in U0 state, assuming TX FIFO is empty */
	reg = sys_read32(base + UDC_DWC3_DCTL);
	reg &= ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;
	reg |= UDC_DWC3_DCTL_ULSTCHNGREQ_REMOTEWAKEUP;
	sys_write32(reg, base + UDC_DWC3_DCTL);

	sys_write32(HI32((uintptr_t)ep_data->trb_buf), base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(LO32((uintptr_t)ep_data->trb_buf), base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	udc_dwc3_mbx_arm_trb(ep_data, ep_data->trb_buf);

	rsc = udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
			      UDC_DWC3_DEPCMD_DEPSTRTXFER);

	if (rsc == UDC_DWC3_DEPCMD_FAILED) {
		/*
		 * Keep whatever index the endpoint already had. Overwriting it
		 * with a failure value is what left UpdateXfer posting
		 * XFERRSCIDX=0 into a controller that then hung on it.
		 */
		LOG_ERR("DepStartXfer failed ep=0x%02x, keeping xferrscidx=0x%x",
			ep_data->cfg.addr, ep_data->xferrscidx);
	} else {
		ep_data->xferrscidx = rsc;
	}

	LOG_DBG("DepStartXfer done ep=0x%02x xferrscidx=0x%x",
		ep_data->cfg.addr, ep_data->xferrscidx);
}

static void udc_dwc3_depcmd_update_xfer(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	if (udc_dwc3_ep_refuse_if_hw_owned(dev, ep_data, "UpdateXfer")) {
		return;
	}

	/*
	 * An UpdateXfer with XFERRSCIDX=0 matches no transfer resource, so the
	 * controller accepts it and never retires it: CMDACT stays set and the
	 * endpoint is dead. Refuse to post one -- an endpoint with no live
	 * resource needs a StartXfer, not a doorbell.
	 */
	if (ep_data->xferrscidx == 0U) {
		LOG_ERR("ep 0x%02x has no transfer resource; refusing UpdateXfer",
			ep_data->cfg.addr);
		return;
	}

	uint32_t flags = 0;

	flags |= UDC_DWC3_DEPCMD_DEPUPDXFER;
	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);

	udc_dwc3_mbx_arm_trb(ep_data, &ep_data->trb_buf[ep_data->tail]);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	LOG_INF("DepUpdateXfer done ep 0x%02x, addr 0x%08x, data 0x%08x, xferrscidx 0x%x",
		ep_data->cfg.addr, UDC_DWC3_DEPCMD(ep_data->epn), flags, ep_data->xferrscidx);
}

static void udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags)
{
	if (udc_dwc3_ep_refuse_if_hw_owned(dev, ep_data, "EndXfer")) {
		return;
	}

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);
	flags |= UDC_DWC3_DEPCMD_DEPENDXFER;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	LOG_DBG("DepEndXfer done ep=0x%02x", ep_data->cfg.addr);
}


static void udc_dwc3_depcmd_start_config(const struct device *const dev,
					 bool is_control)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	uint32_t flags = 0;

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, is_control ? 0 : 2);
	flags |= UDC_DWC3_DEPCMD_DEPSTARTCFG;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(0), flags);

	/*
	 * XferRscIdx=2 reopens the non-control transfer-resource pool, which
	 * invalidates every resource previously drawn from it. Anything still
	 * marked allocated would skip its DEPXFERCFG and keep using an index
	 * the controller no longer recognises, so drop the marks here. The
	 * control pool (XferRscIdx=0) is separate and does not affect them.
	 */
	if (!is_control) {
		for (int i = 1; i < cfg->num_in_eps; i++) {
			cfg->ep_data_in[i].xfer_rsc_allocated = false;
		}
		for (int i = 1; i < cfg->num_out_eps; i++) {
			cfg->ep_data_out[i].xfer_rsc_allocated = false;
		}
	}

	LOG_DBG("DepStartConfig done ep=%s", is_control ? "control" : "non-control");
}

/*
 * Transfer Requests (TRB)
 *
 * DWC3 receives transfer requests from this driver through a shared memory
 * buffer, resubmitted upon every new transfer (through either Start or
 * Update command).
 */

static void udc_dwc3_push_trb(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data,
			      struct net_buf *const buf, const uint32_t ctrl)
{
	volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->head];

	/* If the next TRB in the chain is still owned by the hardware, need
	 * to retry later when more resources become available.
	 */
	__ASSERT_NO_MSG(!ep_data->full);

	/* Associate an active buffer and a TRB together */
	ep_data->net_buf[ep_data->head] = buf;

	/* TRB# with one more chunk of data */
	trb->addr_lo = LO32((uintptr_t)buf->data);
	trb->addr_hi = HI32((uintptr_t)buf->data);
	trb->status = USB_EP_DIR_IS_IN(ep_data->cfg.addr) ? buf->len : buf->size;
	trb->ctrl = ctrl;

	LOG_DBG("PUSH %u, buf %p, data %p, size %u",
		ep_data->head, (void *)buf, (void *)buf->data, buf->size);

	/* -1 for link trb */
	ep_data->head = (ep_data->head + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

	/* If the head touches the tail after we add something, we are full */
	ep_data->full = (ep_data->head == ep_data->tail);
}

static int udc_dwc3_pop_trb(const struct device *const dev, struct udc_dwc3_ep_data *const ep_data,
			    struct net_buf **buf, struct udc_dwc3_trb *trb)
{
	*buf = ep_data->net_buf[ep_data->tail];
	*trb = ep_data->trb_buf[ep_data->tail];

	if ((trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0) {
		return -EBUSY;
	}
	if (*buf == NULL) {
		return -ENOBUFS;
	}

	/* Clear the last TRB */
	ep_data->net_buf[ep_data->tail] = NULL;

	/* -1 for link trb */
	ep_data->tail = (ep_data->tail + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

	LOG_DBG("POP %u EP 0x%02x, buf %p, data %p",
		ep_data->tail, ep_data->cfg.addr, (void *)*buf, (void *)(*buf)->data);

	/* If we just pulled a TRB, we know we made one hole and we are not full anymore */
	ep_data->full = false;

	/* For buffers coming from the host, update the size actually received */
	if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
		(*buf)->len =
			(*buf)->size - FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb->status);
	}

	return 0;
}

static void udc_dwc3_trb_nonctrl_init(const struct device *const dev,
				   struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *trb = ep_data->trb_buf;
	const uint32_t i = CONFIG_UDC_DWC3_TRB_NUM - 1;

	LOG_DBG("Initializing normal TRB");

	/* HWO=0 on the first TRB will prevent the transfers to start until configured */
	memset((void *)trb, 0x00, sizeof(*trb) * CONFIG_UDC_DWC3_TRB_NUM);

	/* TRB LINK that loops the ring buffer back to the beginning */
	trb[i].ctrl = UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB | UDC_DWC3_TRB_CTRL_HWO;
	trb[i].addr_lo = LO32((uintptr_t)ep_data->trb_buf);
	trb[i].addr_hi = HI32((uintptr_t)ep_data->trb_buf);

	/* Start the transfer now, update it later */
	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

void lattice_usb23_restart_xfer(const struct device *dev, uint8_t ep_addr)
{
	struct udc_dwc3_ep_data *ep_data = (void *)udc_get_ep_cfg(dev, ep_addr);

	/* Stream-off already ended the transfer via ClearStall. Ending it
	 * again with the stale resource index sticks CmdAct and the next
	 * control transfer times out. Rebuild the ring and start once.
	 */
	udc_dwc3_trb_nonctrl_init(dev, ep_data);
}

static void udc_dwc3_trb_ctrl_out(const struct device *const dev, struct net_buf *const buf,
				  const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

	trb[0].addr_lo = LO32((uintptr_t)buf->data);
	trb[0].addr_hi = HI32((uintptr_t)buf->data);
	trb[0].status = buf->size;
	trb[0].ctrl = ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO;

	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

static void udc_dwc3_trb_ctrl_in(const struct device *const dev,
				 struct net_buf *const buf,
				 const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

	if (udc_ep_buf_has_zlp(buf)) {
		trb[0].addr_lo = LO32((uintptr_t)buf->data);
		trb[0].addr_hi = HI32((uintptr_t)buf->data);
		trb[0].status = buf->len;
		trb[0].ctrl = ctrl | UDC_DWC3_TRB_CTRL_CHN | UDC_DWC3_TRB_CTRL_HWO;

		trb[1].addr_lo = 0;
		trb[1].addr_hi = 0;
		trb[1].status = 0;
		trb[1].ctrl = ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO;
	} else {
		trb[0].addr_lo = LO32((uintptr_t)buf->data);
		trb[0].addr_hi = HI32((uintptr_t)buf->data);
		trb[0].status = buf->len;
		trb[0].ctrl = ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO;
	}

	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

static int udc_dwc3_trb_bulk(const struct device *const dev,
			     struct udc_dwc3_ep_data *const ep_data,
			     struct net_buf *const buf)
{
	uint32_t ctrl = UDC_DWC3_TRB_CTRL_IOC | UDC_DWC3_TRB_CTRL_HWO | UDC_DWC3_TRB_CTRL_CSP;

	LOG_INF("TRB_BULK_EP_0x%02x, buf %p, data %p, size %u, len %u",
		ep_data->cfg.addr, (void *)buf, (void *)buf->data, buf->size, buf->len);

	if (ep_data->full) {
		return -EBUSY;
	}

	if (udc_ep_buf_has_zlp(buf)) {
		LOG_INF("Buffer has a ZLP flag");
		ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL_ZLP;
	} else {
		ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL;
	}

	udc_dwc3_push_trb(dev, ep_data, buf, ctrl);
	udc_ep_set_busy(&ep_data->cfg, true);
	udc_dwc3_depcmd_update_xfer(dev, ep_data);

	return 0;
}

/*
 * Control buffers
 *
 * There is no worker for control buffers, and instead buffers are udc_dwc3_next_ctrl() is called
 * whenever there is an opportunity to send more, and only when all conditions are met.
 */

static void udc_dwc3_next_ctrl_in(const struct device *const dev,
				  struct net_buf *const buf)
{
	struct udc_data *const data = dev->data;
	const struct usb_setup_packet *const setup = (void *)data->setup;
	struct udc_buf_info *const bi = udc_get_buf_info(buf);

	if (bi->data) {
		LOG_DBG("TRB_CONTROL_IN_DATA len %d, data %p", buf->len, (void *)buf->data);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi->status && setup->wLength == 0) {
		buf->size = 0;
		LOG_DBG("TRB_CONTROL_IN_STATUS_2 len %d, data %p", buf->len, (void *)buf->data);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
	} else if (bi->status) {
		buf->size = 0;
		LOG_DBG("TRB_CONTROL_IN_STATUS_3 len %d, data %p", buf->len, (void *)buf->data);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
	} else {
		LOG_ERR("Unknown buffer IN type");
		udc_submit_ep_event(dev, buf, -EINVAL);
	}
}

static void udc_dwc3_next_ctrl_out(const struct device *const dev,
				   struct net_buf *const buf)
{
	struct udc_buf_info *const bi = udc_get_buf_info(buf);

	if (bi->setup) {
		LOG_DBG("TRB_CONTROL_OUT_SETUP size %d, data %p", buf->size, (void *)buf->data);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);
	} else if (bi->data) {
		LOG_DBG("TRB_CONTROL_OUT_DATA size %d, data %p", buf->size, (void *)buf->data);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi->status) {
		buf->size = 0;
		LOG_DBG("TRB_CONTROL_OUT_STATUS_3 size %d, data %p", buf->size, (void *)buf->data);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
	} else {
		LOG_ERR("Unknown buffer OUT, size %d, data %p", buf->size, (void *)buf->data);
		udc_submit_ep_event(dev, buf, -EINVAL);
	}
}

static void udc_dwc3_next_ctrl(const struct device *const dev,
			       struct udc_dwc3_ep_data *const ep_data)
{
	struct net_buf *buf;

	if (udc_ep_is_busy(&ep_data->cfg)) {
		LOG_INF("Endpoint still busy, not loading next buffer");
		return;
	}

	buf = udc_buf_peek(&ep_data->cfg);
	if (buf == NULL) {
		LOG_INF("No extra buffer for this endpoint yet");
		return;
	}

	udc_ep_set_busy(&ep_data->cfg, true);

	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		udc_dwc3_next_ctrl_in(dev, buf);
	} else {
		udc_dwc3_next_ctrl_out(dev, buf);
	}
}

/*
 * Events
 *
 * Process the events from the event ring buffer. Interrupts gives us a
 * hint that an event is available, which we fetch from a ring buffer shared
 * with the hardware.
 */

static void udc_dwc3_on_soft_reset(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	/* Configure and reset the Device Controller */
	/* TODO confirm that DWC_USB3_EN_LPM_ERRATA == 1 */
	reg = UDC_DWC3_DCTL_CSFTRST;
	reg |= FIELD_PREP(UDC_DWC3_DCTL_LPM_NYET_THRES_MASK, 15);
	sys_write32(reg, base + UDC_DWC3_DCTL);
	while (sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_CSFTRST) {
		continue;
	}

	/* Enable AXI64 bursts for various sizes expected */
	reg = UDC_DWC3_GSBUSCFG0_INCR256BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR128BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR64BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR32BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR16BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR8BRSTENA;
	reg |= UDC_DWC3_GSBUSCFG0_INCR4BRSTENA;
	sys_set_bits(base + UDC_DWC3_GSBUSCFG0, reg);

	/* Letting GTXTHRCFG and GRXTHRCFG unchanged */

	/* Read the chip identification */
	reg = sys_read32(base + UDC_DWC3_GCOREID);
	LOG_INF("event: coreid=0x%04lx rel=0x%04lx",
		FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg),
		FIELD_GET(UDC_DWC3_GCOREID_REL_MASK, reg));
	__ASSERT_NO_MSG(FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg) == 0x5533);

	/* Letting GUID unchanged */
	/* Letting GUSB2PHYCFG and GUSB3PIPECTL unchanged */
	/* Letting GRXFIFOSIZ unchanged */

	/* Setup the event buffer address, size and start event reception */
	memset((void *)cfg->evt_buf, 0, CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t));
	sys_write32(HI32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_HI(0));
	sys_write32(LO32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_LO(0));
	sys_write32(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t), base + UDC_DWC3_GEVNTSIZ(0));
	LOG_INF("Event buffer size is %u bytes", sys_read32(base + UDC_DWC3_GEVNTSIZ(0)));
	sys_write32(0, base + UDC_DWC3_GEVNTCOUNT(0));

	/* Letting GCTL unchanged */

	reg = sys_read32(base + UDC_DWC3_GUCTL2);
	reg |= UDC_DWC3_GUCTL2_RST_ACTBITLATER;
	sys_write32(reg, base + UDC_DWC3_GUCTL2);

	/* Set the USB device configuration, including max supported speed */
	sys_write32(UDC_DWC3_DCFG_PERFRINT_90, base + UDC_DWC3_DCFG);
	switch (cfg->maximum_speed_idx) {
	case UDC_DWC3_SPEED_IDX_SUPER_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_SUPER_SPEED");
		sys_set_bits(base + UDC_DWC3_DCFG, UDC_DWC3_DCFG_DEVSPD_SUPER_SPEED);
		break;
	case UDC_DWC3_SPEED_IDX_HIGH_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_HIGH_SPEED");
		sys_set_bits(base + UDC_DWC3_DCFG, UDC_DWC3_DCFG_DEVSPD_HIGH_SPEED);
		break;
	case UDC_DWC3_SPEED_IDX_FULL_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_FULL_SPEED");
		sys_set_bits(base + UDC_DWC3_DCFG, UDC_DWC3_DCFG_DEVSPD_FULL_SPEED);
		break;
	default:
		CODE_UNREACHABLE;
	}

	/* Set the number of USB3 packets the device can receive at once */
	reg = sys_read32(base + UDC_DWC3_DCFG);
	reg &= ~UDC_DWC3_DCFG_NUMP_MASK;
	reg |= FIELD_PREP(UDC_DWC3_DCFG_NUMP_MASK, 15);
	sys_write32(reg, base + UDC_DWC3_DCFG);

	/* Enable reception of all USB events except UDC_DWC3_DEVTEN_ULSTCNGEN */
	reg = UDC_DWC3_DEVTEN_INACTTIMEOUTRCVEDEN;
	reg |= UDC_DWC3_DEVTEN_VNDRDEVTSTRCVEDEN;
	reg |= UDC_DWC3_DEVTEN_EVNTOVERFLOWEN;
	reg |= UDC_DWC3_DEVTEN_CMDCMPLTEN;
	reg |= UDC_DWC3_DEVTEN_ERRTICERREN;
	reg |= UDC_DWC3_DEVTEN_HIBERNATIONREQEVTEN;
	reg |= UDC_DWC3_DEVTEN_WKUPEVTEN;
	reg |= UDC_DWC3_DEVTEN_CONNECTDONEEN;
	reg |= UDC_DWC3_DEVTEN_USBRSTEN;
	reg |= UDC_DWC3_DEVTEN_DISCONNEVTEN;
	sys_write32(reg, base + UDC_DWC3_DEVTEN);

	/* Configure control endpoints */
	udc_dwc3_depcmd_start_config(dev, true);
}

static void udc_dwc3_on_usb_reset(const struct device *const dev)
{
	LOG_DBG("Going through DWC3 reset logic");

	/* TODO: wait that all transfers did complete (if needed) */

	/* Perform the USB reset operations manually to improve latency */
	/* TODO: do after endpoints are configured? */
	udc_dwc3_set_address(dev, 0);
}

static void udc_dwc3_on_connect_done(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	int mps = 0;

	/* Adjust parameters against the connection speed */
	switch (sys_read32(base + UDC_DWC3_DSTS) & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
		mps = 64;
		break;
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		mps = 512;
		break;
	}
	__ASSERT_NO_MSG(mps != 0);

	/* Reconfigure control endpoints connection speed */
	udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT)->mps = mps;
	udc_get_ep_cfg(dev, USB_CONTROL_EP_IN)->mps = mps;
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0]);
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_out[0]);

	/* Letting GTXFIFOSIZn unchanged */

	/* After successful speed negotiation, DWC3 sends a CONNECT_DONE event.
	 * Then only the speed-related registers are populated, and we can
	 * report the "reset" event (instead of during USB_RESET).
	 */
	udc_submit_event(dev, UDC_EVT_RESET, 0);
}

static void udc_dwc3_on_set_config_or_interface(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	LOG_DBG("SetConfiguration or SetInterface extra init");

	for (int i = 1; i < cfg->num_in_eps; i++) {
		if (udc_ep_is_busy(&cfg->ep_data_in[i].cfg)) {
			udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_in[i], 0);
		}
	}
	for (int i = 1; i < cfg->num_out_eps; i++) {
		if (udc_ep_is_busy(&cfg->ep_data_out[i].cfg)) {
			udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_out[i], 0);
		}
	}

	/* To trigger a reconfiguration of the TX FIFO */
	udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_in[0], UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0]);

	/* Re-initialize resources IDs for non-control endpoints */
	udc_dwc3_depcmd_start_config(dev, false);
}

/*
 * Handle completion of a CONTROL IN packet (device -> host).
 *
 * Further characterize which type of CONTROL IN packet that is.
 * Handle actions common to all CONTROL IN packets.
 */
static void udc_dwc3_on_ctrl_in(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[0];
	const uint32_t trb_trbctl = ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK;
	struct net_buf *buf;

	buf = udc_buf_get(&ep_data->cfg);
	if (buf == NULL) {
		LOG_ERR("Failed to get a buffer for ep 0x%02x", ep_data->cfg.addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return;
	}

	if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
	    trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
		buf->len = 0;
		LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL STATUS packet sent");
	} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
		LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL DATA packet sent");
	} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		LOG_ERR("Unexpected SETUP IN packet");
	} else {
		LOG_ERR("Unexpected IN packet type: 0x%x", trb_trbctl);
	}

	udc_submit_ep_event(dev, buf, 0);

	/* Used when receiving a completed buffer from the hardware: mark as free */
	udc_ep_set_busy(&ep_data->cfg, false);

	udc_dwc3_next_ctrl(dev, ep_data);
}

/*
 * Handle completion of a CONTROL OUT packet (host -> device).
 *
 * Further characterize which type of CONTROL OUT packet that is.
 * Handle actions common to all CONTROL OUT packets.
 */
static void udc_dwc3_on_ctrl_out(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[0];
	const uint32_t trb_trbctl = ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK;
	const uint32_t trb_status = ep_data->trb_buf[0].status;
	struct net_buf *buf;

	if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		struct usb_setup_packet *setup_packet;

		buf = udc_buf_peek(&ep_data->cfg);
		if (buf == NULL) {
			LOG_ERR("Failed to get the buffer for ep 0x%02x", ep_data->cfg.addr);
			udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
			return;
		}

		setup_packet = (void *)buf->data;

		/* Update the size to the setup packet size */
		if (buf->size < sizeof(*setup_packet)) {
			LOG_ERR("Invalid size for setup packet buffer: %u", buf->size);
			udc_submit_ep_event(dev, buf, -ENOBUFS);
			return;
		}

		buf->len = sizeof(*setup_packet);

		/* Latency optimization: set the address immediately to be able to be able
		 * to ACK/NAK the first packets from the host with the new address,
		 * otherwise the host issue a reset.
		 */
		if (setup_packet->bmRequestType == USB_REQTYPE_TYPE_STANDARD &&
		    setup_packet->bRequest == USB_SREQ_SET_ADDRESS) {
			udc_dwc3_set_address(dev, sys_le16_to_cpu(setup_packet->wValue));
		}

		LOG_HEXDUMP_DBG(setup_packet, sizeof(*setup_packet), "Submitting SETUP");
		udc_setup_received(dev, NULL);
	} else {
		buf = udc_buf_get(&ep_data->cfg);
		if (buf == NULL) {
			LOG_ERR("Failed to get the buffer for ep 0x%02x", ep_data->cfg.addr);
			udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
			return;
		}

		/* Update the size to what the hardware reports */
		buf->len = buf->size - FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb_status);

		if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
			LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL DATA received");
		} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
			buf->len = 0;
			LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL STATUS received");
		} else {
			LOG_ERR("Unexpected OUT packet type: 0x%x", trb_trbctl);
		}

		udc_submit_ep_event(dev, buf, 0);
	}

	/* Used when receiving a completed buffer from the hardware: mark as free */
	udc_ep_set_busy(&ep_data->cfg, false);

	udc_dwc3_next_ctrl(dev, ep_data);
}

static void udc_dwc3_on_xfer_not_ready(const struct device *const dev, const uint32_t evt,
				       struct udc_dwc3_ep_data *const ep_data)
{
	switch (evt & UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK) {
	case UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP:
		LOG_DBG("UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP");
		break;
	case UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA:
		LOG_DBG("UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA");
		break;
	case UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS:
		LOG_DBG("UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS");
		break;
	}
}

static void udc_dwc3_on_xfer_done(const struct device *const dev,
				  struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->tail];

	switch (trb->status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK) {
	case UDC_DWC3_TRB_STATUS_TRBSTS_OK:
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_MISSEDISOC:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_MISSEDISOC");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING");
		break;
	default:
		CODE_UNREACHABLE;
	}
}

static void udc_dwc3_on_xfer_error_nonctrl(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *const ep_data = _EP_DATA_FROM_EPN(cfg, epn);
	struct udc_dwc3_trb trb;
	struct net_buf *buf;
	int ret;

	ret = udc_dwc3_pop_trb(dev, ep_data, &buf, &trb);
	if (ret != 0) {
		udc_submit_event(dev, UDC_EVT_ERROR, ret);
		return;
	}

	ret = udc_submit_ep_event(dev, buf, -ECANCELED);
	if (ret != 0) {
		LOG_ERR("Failed to report error event for buf %p", buf);
		return;
	}

	ret = udc_dwc3_ep_disable(dev, &ep_data->cfg);
	if (ret != 0) {
		LOG_ERR("Failed to resume endpoint 0x%02x", ep_data->cfg.addr);
		return;
	}

	ret = udc_dwc3_ep_resume(dev, ep_data);
	if (ret != 0) {
		LOG_ERR("Failed to resume endpoint 0x%02x", ep_data->cfg.addr);
		return;
	}
}

static void udc_dwc3_on_xfer_done_nonctrl(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *const ep_data = _EP_DATA_FROM_EPN(cfg, epn);
	struct net_buf *buf;
	int ret;

	while (true) {
		struct udc_dwc3_trb trb;

		ret = udc_dwc3_pop_trb(dev, ep_data, &buf, &trb);
		if (ret == -ENOBUFS || ret == -EBUSY) {
			break;
		}
		if (ret != 0) {
			udc_submit_event(dev, UDC_EVT_ERROR, ret);
			break;
		}

		LOG_DBG("XFER_DONE_NORM: EP 0x%02x, data %p",
			ep_data->cfg.addr, (void *)buf->data);

		udc_dwc3_on_xfer_done(dev, ep_data);

		udc_ep_set_busy(&ep_data->cfg, false);

		ret = udc_submit_ep_event(dev, buf, 0);
		if (ret != 0) {
			LOG_ERR("Failed to submit buffer %p: %d", buf, ret);
		}

		/* We just made some room for a new buffer, check if something more to enqueue */
		k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
	}
}

#if defined(CONFIG_UDC_DWC3_COMPLETION_POLL)
/*
 * Retire bulk slots whose completion the controller never reported.
 *
 * This Soft-IP clears HWO on a CPU-managed bulk TRB without posting a DEPEVT
 * when UVC and CDC run concurrently. Nothing then retires the buffer: the OUT
 * packet is never handed to the class, so the host's write stalls forever,
 * and the IN side stops enqueueing. Measured on 2026-11-09: CDC-ACM OUT
 * parked after 91 writes with both video lanes streaming and stayed parked
 * for the remaining 15 s of the run.
 *
 * The recovery is to notice HWO clear with a buffer still waiting and
 * synthesise the XferInProgress that was dropped. HWO must stay clear for
 * CONFIG_UDC_DWC3_COMPLETION_POLL_TICKS ticks first: a short grace races a
 * DEPEVT that is merely late and double-pops the ring, which is worse than
 * the stall. The reference tree measured that directly -- 2 ticks raced live
 * events, 8 did not.
 */
static void udc_dwc3_completion_poll_dir(const struct device *const dev,
					 struct udc_dwc3_ep_data *const eps,
					 const int num_eps, uint8_t *const grace,
					 const int grace_len, uint32_t *const retired,
					 const bool is_in, const char *const dir)
{
	for (int idx = 1; idx < num_eps && idx < grace_len; idx++) {
		struct udc_dwc3_ep_data *const ep_data = &eps[idx];

		/*
		 * IN recovery is limited to the endpoints the mask names.
		 * An RTL-owned IN must never be retired in software, and the
		 * mask settles that at build time rather than relying on a
		 * runtime flag that a handoff could be in the middle of
		 * changing.
		 */
		if (is_in && (CONFIG_UDC_DWC3_IN_RECOVER_EP_MASK & BIT(idx)) == 0U) {
			grace[idx] = 0;
			continue;
		}

		/*
		 * Skip anything that cannot be a dropped completion: no ring,
		 * not bulk, nothing waiting, the transfer still owned by
		 * hardware, or an endpoint the video RTL retires itself.
		 */
		if (ep_data->trb_buf == NULL || ep_data->hw_owned ||
		    (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) !=
			    USB_EP_TYPE_BULK ||
		    ep_data->net_buf[ep_data->tail] == NULL ||
		    (ep_data->trb_buf[ep_data->tail].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
			grace[idx] = 0;
			continue;
		}

		if (grace[idx] < (uint8_t)CONFIG_UDC_DWC3_COMPLETION_POLL_TICKS) {
			grace[idx]++;
			continue;
		}

		grace[idx] = 0;

		/* Re-check under lock so the event thread cannot double-pop. */
		{
			const unsigned int key = irq_lock();

			if (ep_data->net_buf[ep_data->tail] == NULL ||
			    (ep_data->trb_buf[ep_data->tail].ctrl &
			     UDC_DWC3_TRB_CTRL_HWO) != 0U) {
				irq_unlock(key);
				continue;
			}

			(*retired)++;
			udc_dwc3_on_xfer_done_nonctrl(
				dev, UDC_DWC3_DEPEVT_XFERINPROGRESS(ep_data->epn));
			irq_unlock(key);
		}

		LOG_WRN("%s-POLL: retired lost completion ep=0x%02x tail=%u (total=%u)",
			dir, ep_data->cfg.addr, ep_data->tail, *retired);
	}
}

#if defined(CONFIG_UDC_DWC3_PARK_NUDGE)
/* Ticks of the poll interval that a frozen TRB must persist before a nudge. */
#define UDC_DWC3_PARK_TICKS                                                    \
	((CONFIG_UDC_DWC3_PARK_NUDGE_MS + CONFIG_UDC_DWC3_COMPLETION_POLL_MS - 1) /    \
	 CONFIG_UDC_DWC3_COMPLETION_POLL_MS)
/* Ticks between nudges, and how long to back off once the cap is reached. */
#define UDC_DWC3_PARK_COOLDOWN_TICKS 4
#define UDC_DWC3_PARK_BACKOFF_TICKS  50

/*
 * Re-ring the doorbell on a bulk endpoint whose tail TRB is still owned by
 * hardware with a frozen remaining count: the UpdateXfer that should have
 * started it was dropped. The completion poll cannot see this case, because
 * HWO never clears.
 *
 * Only UpdateXfer is issued, never a ring rebuild. An armed TRB with a frozen
 * remaining count is also exactly what an endpoint looks like when the host
 * has not polled it yet, and that is indistinguishable from here; an extra
 * doorbell costs nothing in that case, while tearing down a live ring does.
 */
/* Still owned by hardware with a buffer waiting: the park has not cleared. */
static bool udc_dwc3_park_stuck(const struct udc_dwc3_ep_data *const ep_data)
{
	return ep_data->net_buf[ep_data->tail] != NULL &&
	       (ep_data->trb_buf[ep_data->tail].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U;
}

/*
 * Walk progressively harder resets of a parked endpoint's own state, stopping
 * at the first that clears HWO. Returns true if one did.
 *
 * This runs as a SINGLE pass with a re-check after each rung, which is not a
 * style choice. The first cut advanced one rung per 100 ms poll tick, and s2's
 * StartXfer makes the core re-fetch and re-load the TRB -- which changes the
 * remaining count, which the park detector then read as forward progress and
 * reset the ladder to the bottom. s3 and s4 were never reached once in a
 * 300 s soak (both counters zero while s1 and s2 logged twice each). Checking
 * the endpoint directly after each step, inside one call, is what makes the
 * ladder able to finish.
 */
static bool udc_dwc3_park_escalate(const struct device *const dev,
				   struct udc_dwc3_ep_data *const ep_data,
				   const char *const dir)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t fifo = USB_EP_GET_IDX(ep_data->cfg.addr);
	int rc;

	/*
	 * s1: drop the orphaned bytes out of this endpoint's TX FIFO, then
	 * re-ring once. Does not disturb the transfer. TX_FIFO is the bit that
	 * makes this the TX FIFO and not RX n.
	 */
	rc = udc_dwc3_dgcmd(base, UDC_DWC3_DGCMD_FIFOFLUSHONE,
			    UDC_DWC3_DGCMDPAR_TX_FIFO | UDC_DWC3_DGCMDPAR_FIFO_NUM(fifo));
	LOG_WRN("%s-PARK: s1 flush TX fifo %u ep=0x%02x rc=%d",
		dir, fifo, ep_data->cfg.addr, rc);
	udc_dwc3_depcmd_update_xfer(dev, ep_data);
	k_busy_wait(2000);
	if (!udc_dwc3_park_stuck(ep_data)) {
		LOG_WRN("%s-PARK: recovered at s1 ep=0x%02x", dir, ep_data->cfg.addr);
		return true;
	}

	/*
	 * s2: EndXfer WITHOUT ForceRM, then StartXfer the same slot. ForceRM
	 * is the form that is fatal with words still loaded; plain EndXfer is
	 * not, and the ring is left intact so the class completion path stays
	 * in step.
	 */
	LOG_WRN("%s-PARK: s2 EndXfer(no ForceRM)+StartXfer ep=0x%02x",
		dir, ep_data->cfg.addr);
	udc_dwc3_depcmd_end_xfer(dev, ep_data, 0);
	udc_dwc3_depcmd_start_xfer(dev, ep_data);
	k_busy_wait(2000);
	if (!udc_dwc3_park_stuck(ep_data)) {
		LOG_WRN("%s-PARK: recovered at s2 ep=0x%02x", dir, ep_data->cfg.addr);
		return true;
	}

	/*
	 * s3: re-apply DEPCFG with ACTION_MODIFY, which
	 * udc_dwc3_depcmd_ep_config picks once the endpoint is enabled, then
	 * re-ring. MODIFY preserves endpoint state, so it comes before
	 * anything that resets it.
	 */
	LOG_WRN("%s-PARK: s3 DEPCFG(MODIFY)+UpdateXfer ep=0x%02x", dir, ep_data->cfg.addr);
	udc_dwc3_depcmd_ep_config(dev, ep_data);
	udc_dwc3_depcmd_update_xfer(dev, ep_data);
	k_busy_wait(2000);
	if (!udc_dwc3_park_stuck(ep_data)) {
		LOG_WRN("%s-PARK: recovered at s3 ep=0x%02x", dir, ep_data->cfg.addr);
		return true;
	}

	/*
	 * s4: SetStall then ClearStall, then re-ring.
	 *
	 * By here s1 has emptied the TX FIFO and s2's StartXfer has made the
	 * core re-fetch and re-load the TRB, so TRB fetch, the AXI path and
	 * FIFO fill all demonstrably work and the TRB still sits HWO. The only
	 * stage left unaccounted for is transmit, and DEPCSTALL is the one
	 * command that resets an endpoint's transmit state and its data
	 * sequence number. Both halves are issued here, so no host
	 * CLEAR_FEATURE(HALT) is needed.
	 *
	 * Raw DEPCMDs rather than the stall helpers: the clear-side helper
	 * logs at INFO, which is compiled out at this log level, and a log
	 * showing SetStall with no matching clear would leave it unclear
	 * whether the endpoint had been left stalled.
	 */
	LOG_WRN("%s-PARK: s4 SetStall+ClearStall+UpdateXfer ep=0x%02x",
		dir, ep_data->cfg.addr);
	(void)udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
			      UDC_DWC3_DEPCMD_DEPSETSTALL);
	(void)udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
			      UDC_DWC3_DEPCMD_DEPCSTALL);
	ep_data->cfg.stat.halted = false;
	udc_dwc3_depcmd_update_xfer(dev, ep_data);
	k_busy_wait(2000);
	if (!udc_dwc3_park_stuck(ep_data)) {
		LOG_WRN("%s-PARK: recovered at s4 ep=0x%02x", dir, ep_data->cfg.addr);
		return true;
	}

	LOG_ERR("%s-PARK: ladder exhausted, ep=0x%02x still parked (remain=%u)",
		dir, ep_data->cfg.addr,
		(unsigned int)(ep_data->trb_buf[ep_data->tail].status &
			       UDC_DWC3_TRB_STATUS_BUFSIZ_MASK));
	return false;
}

static void udc_dwc3_park_nudge_dir(const struct device *const dev,
				    struct udc_dwc3_ep_data *const eps,
				    const int num_eps, uint32_t *const nudged,
				    const bool is_in, const char *const dir)
{
	for (int idx = 1; idx < num_eps; idx++) {
		struct udc_dwc3_ep_data *const ep_data = &eps[idx];
		const struct udc_dwc3_trb *trb;
		uint32_t remain;

		/* Same rule as the completion poll: masked IN endpoints only. */
		if (is_in && (CONFIG_UDC_DWC3_IN_RECOVER_EP_MASK & BIT(idx)) == 0U) {
			ep_data->park_ticks = 0;
			ep_data->park_nudges = 0;
			continue;
		}

		if (ep_data->trb_buf == NULL || ep_data->hw_owned ||
		    (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) !=
			    USB_EP_TYPE_BULK ||
		    ep_data->net_buf[ep_data->tail] == NULL ||
		    ep_data->xferrscidx == 0U) {
			ep_data->park_ticks = 0;
			ep_data->park_nudges = 0;
			ep_data->park_exhausted = 0;
			continue;
		}

		trb = &ep_data->trb_buf[ep_data->tail];
		if ((trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) == 0U) {
			/* Finished but unreported: the completion poll owns this. */
			ep_data->park_ticks = 0;
			ep_data->park_nudges = 0;
			continue;
		}

		remain = trb->status & UDC_DWC3_TRB_STATUS_BUFSIZ_MASK;

		if (ep_data->tail != ep_data->park_tail ||
		    remain != ep_data->park_remain) {
			/* Forward progress; not parked. */
			ep_data->park_tail = ep_data->tail;
			ep_data->park_remain = remain;
			ep_data->park_ticks = 0;
			ep_data->park_nudges = 0;
			ep_data->park_exhausted = 0;
			continue;
		}

		ep_data->park_ticks++;

		if (ep_data->park_nudges >= (uint8_t)CONFIG_UDC_DWC3_PARK_NUDGE_CAP) {
			bool recovered;

			if (ep_data->park_exhausted != 0U) {
				/* Out of moves; wait for the host or a revoke. */
				if (ep_data->park_ticks >= UDC_DWC3_PARK_BACKOFF_TICKS) {
					ep_data->park_ticks = 0;
					ep_data->park_nudges = 0;
					ep_data->park_exhausted = 0;
				}
				continue;
			}

			if (ep_data->park_ticks < UDC_DWC3_PARK_TICKS +
				    UDC_DWC3_PARK_COOLDOWN_TICKS *
					    (uint16_t)CONFIG_UDC_DWC3_PARK_NUDGE_CAP) {
				continue;
			}

			recovered = udc_dwc3_park_escalate(dev, ep_data, dir);

			ep_data->park_ticks = 0;
			ep_data->park_nudges = 0;
			ep_data->park_remain =
				ep_data->trb_buf[ep_data->tail].status &
				UDC_DWC3_TRB_STATUS_BUFSIZ_MASK;
			if (!recovered) {
				ep_data->park_exhausted = 1;
			}
			continue;
		}

		if (ep_data->park_ticks < UDC_DWC3_PARK_TICKS +
			    (uint16_t)ep_data->park_nudges * UDC_DWC3_PARK_COOLDOWN_TICKS) {
			continue;
		}

		ep_data->park_nudges++;
		(*nudged)++;
		udc_dwc3_depcmd_update_xfer(dev, ep_data);

		LOG_WRN("%s-PARK: nudged ep=0x%02x tail=%u remain=%u (nudge %u, total=%u)",
			dir, ep_data->cfg.addr, ep_data->tail, remain,
			ep_data->park_nudges, *nudged);
	}
}
#endif /* CONFIG_UDC_DWC3_PARK_NUDGE */

static void udc_dwc3_completion_poll_worker(struct k_work *work)
{
	struct k_work_delayable *const dwork = k_work_delayable_from_work(work);
	struct udc_dwc3_data *const priv =
		CONTAINER_OF(dwork, struct udc_dwc3_data, poll_work);
	const struct device *const dev = priv->dev;
	const struct udc_dwc3_config *const cfg = dev->config;

	/*
	 * The completion poll runs on OUT as well, and that is safe where park
	 * recovery is not: it acts only when HWO is CLEAR with a buffer still
	 * waiting, i.e. a transfer the controller finished and never reported.
	 * An armed-and-waiting OUT has HWO SET and is skipped.
	 */
	udc_dwc3_completion_poll_dir(dev, cfg->ep_data_out, cfg->num_out_eps,
				     priv->poll_grace_out,
				     (int)ARRAY_SIZE(priv->poll_grace_out),
				     &priv->poll_retired_out, false, "OUT");
	udc_dwc3_completion_poll_dir(dev, cfg->ep_data_in, cfg->num_in_eps,
				     priv->poll_grace_in,
				     (int)ARRAY_SIZE(priv->poll_grace_in),
				     &priv->poll_retired_in, true, "IN");

#if defined(CONFIG_UDC_DWC3_PARK_NUDGE)
	/*
	 * Park recovery is IN-only, deliberately.
	 *
	 * An idle bulk OUT SHOULD sit with HWO set and its remaining count at
	 * the full buffer length: that is an armed endpoint waiting for the
	 * host to write, not a stuck one, and from inside the driver it is
	 * indistinguishable from a park. The reference tree established this
	 * the hard way -- treating armed-and-waiting OUT as stuck produced
	 * "false STUCK storms", and eager EndXfer recycling on it BROKE ACM
	 * enumeration.
	 *
	 * This also retires an inference of mine: I read 0x01 freezing at
	 * remain=1024 alongside 0x82 at remain=11 as two endpoints parking
	 * together, and used it to argue the fault could not be transmit-side.
	 * It is far more likely one symptom: once 0x82 wedges, the host stops
	 * writing because its echo never returns, and 0x01 is then simply
	 * idle-armed and innocent.
	 *
	 * A genuine OUT failure (host actively writing, TRB not retiring) needs
	 * evidence the host was pushing bytes at the time, which this tick does
	 * not have. Until it does, the OUT side is left alone.
	 */
	udc_dwc3_park_nudge_dir(dev, cfg->ep_data_in, cfg->num_in_eps,
				&priv->park_nudged_in, true, "IN");
#endif

	k_work_schedule_for_queue(udc_get_work_q(), &priv->poll_work,
				  K_MSEC(CONFIG_UDC_DWC3_COMPLETION_POLL_MS));
}
#endif /* CONFIG_UDC_DWC3_COMPLETION_POLL */

static const char *udc_dwc3_get_devt_ulstchng_name(const uint32_t dsts)
{
	switch (dsts & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		switch (dsts & UDC_DWC3_DSTS_USBLNKST_MASK) {
		case UDC_DWC3_DSTS_USBLNKST_USB3_U0:
			return "DSTS_USBLNKST_USB3_U0";
		case UDC_DWC3_DSTS_USBLNKST_USB3_U1:
			return "DSTS_USBLNKST_USB3_U1";
		case UDC_DWC3_DSTS_USBLNKST_USB3_U2:
			return "DSTS_USBLNKST_USB3_U2";
		case UDC_DWC3_DSTS_USBLNKST_USB3_U3:
			return "DSTS_USBLNKST_USB3_U3";
		case UDC_DWC3_DSTS_USBLNKST_USB3_SS_DIS:
			return "DSTS_USBLNKST_USB3_SS_DIS";
		case UDC_DWC3_DSTS_USBLNKST_USB3_RX_DET:
			return "DSTS_USBLNKST_USB3_RX_DET";
		case UDC_DWC3_DSTS_USBLNKST_USB3_SS_INACT:
			return "DSTS_USBLNKST_USB3_SS_INACT";
		case UDC_DWC3_DSTS_USBLNKST_USB3_POLL:
			return "DSTS_USBLNKST_USB3_POLL";
		case UDC_DWC3_DSTS_USBLNKST_USB3_RECOV:
			return "DSTS_USBLNKST_USB3_RECOV";
		case UDC_DWC3_DSTS_USBLNKST_USB3_HRESET:
			return "DSTS_USBLNKST_USB3_HRESET";
		case UDC_DWC3_DSTS_USBLNKST_USB3_CMPLY:
			return "DSTS_USBLNKST_USB3_CMPLY";
		case UDC_DWC3_DSTS_USBLNKST_USB3_LPBK:
			return "DSTS_USBLNKST_USB3_LPBK";
		case UDC_DWC3_DSTS_USBLNKST_USB3_RESET_RESUME:
			return "DSTS_USBLNKST_USB3_RESET_RESUME";
		default:
			return "unknown USB3 link state event";
		}
		break;
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
		switch (dsts & UDC_DWC3_DSTS_USBLNKST_MASK) {
		case UDC_DWC3_DSTS_USBLNKST_USB2_ON_STATE:
			return "DSTS_USBLNKST_USB2_ON_STATE";
		case UDC_DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE:
			return "DSTS_USBLNKST_USB2_SLEEP_STATE";
		case UDC_DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE:
			return "DSTS_USBLNKST_USB2_SUSPEND_STATE";
		case UDC_DWC3_DSTS_USBLNKST_USB2_DISCONNECTED:
			return "DSTS_USBLNKST_USB2_DISCONNECTED";
		case UDC_DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND:
			return "DSTS_USBLNKST_USB2_EARLY_SUSPEND";
		case UDC_DWC3_DSTS_USBLNKST_USB2_RESET:
			return "DSTS_USBLNKST_USB2_RESET";
		case UDC_DWC3_DSTS_USBLNKST_USB2_RESUME:
			return "DSTS_USBLNKST_USB2_RESUME";
		default:
			return "unknown USB2 link state event";
		}
		break;
	default:
		return "DSTS_USBLNKST (unknown)";
	}
}

#define _NORMAL_EP(n, fn) fn(n + 2)

static const char *udc_dwc3_get_event_name(const uint32_t evt, const uint32_t dsts)
{
	switch (evt & UDC_DWC3_EVT_MASK) {
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(0):
		return "DEPEVT_XFERCOMPLETE(0)";
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(1):
		return "DEPEVT_XFERCOMPLETE(1)";
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERCOMPLETE):
		return "DEPEVT_XFERCOMPLETE(n)";
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERINPROGRESS):
		return "DEPEVT_XFERINPROGRESS(n)";
	case UDC_DWC3_DEPEVT_XFERNOTREADY(0):
		return "DEPEVT_XFERNOTREADY(0)";
	case UDC_DWC3_DEPEVT_XFERNOTREADY(1):
		return "DEPEVT_XFERNOTREADY(1)";
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERNOTREADY):
		return "DEPEVT_XFERNOTREADY(n)";
	case UDC_DWC3_DEVT_DISCONNEVT:
		return "DEVT_DISCONNEVT";
	case UDC_DWC3_DEVT_USBRST:
		return "DEVT_USBRST";
	case UDC_DWC3_DEVT_CONNECTDONE:
		return "DEVT_CONNECTDONE";
	case UDC_DWC3_DEVT_ULSTCHNG:
		return udc_dwc3_get_devt_ulstchng_name(dsts);
	case UDC_DWC3_DEVT_WKUPEVT:
		return "DEVT_WKUPEVT";
	case UDC_DWC3_DEVT_SUSPEND:
		return "DEVT_SUSPEND";
	case UDC_DWC3_DEVT_SOF:
		return "DEVT_SOF";
	case UDC_DWC3_DEVT_CMDCMPLT:
		return "DEVT_CMDCMPLT";
	case UDC_DWC3_DEVT_VNDRDEVTSTRCVED:
		return "DEVT_VNDRDEVTSTRCVED";
	case UDC_DWC3_DEVT_ERRTICERR:
		return "DEVT_ERRTICERR";
	case UDC_DWC3_DEVT_EVNTOVERFLOW:
		return "DEVT_EVNTOVERFLOW";
	case 0:
		return "empty";
	default:
		return "unknown event";
	}
}

static void udc_dwc3_handle_event(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t dsts = sys_read32(base + UDC_DWC3_DSTS);

	LOG_INF("Event %s", udc_dwc3_get_event_name(evt, dsts));

	switch (evt & UDC_DWC3_EVT_MASK) {
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(0):
		udc_dwc3_on_ctrl_out(dev);
		break;
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(1):
		udc_dwc3_on_ctrl_in(dev);
		break;
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERCOMPLETE):
		udc_dwc3_on_xfer_error_nonctrl(dev, evt);
		break;
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERINPROGRESS):
		udc_dwc3_on_xfer_done_nonctrl(dev, evt);
		break;
	case UDC_DWC3_DEPEVT_XFERNOTREADY(0):
		udc_dwc3_on_xfer_not_ready(dev, evt, &cfg->ep_data_out[0]);
		break;
	case UDC_DWC3_DEPEVT_XFERNOTREADY(1):
		udc_dwc3_on_xfer_not_ready(dev, evt, &cfg->ep_data_in[0]);
		break;
	case UDC_DWC3_DEVT_USBRST:
		udc_dwc3_on_usb_reset(dev);
		break;
	case UDC_DWC3_DEVT_CONNECTDONE:
		udc_dwc3_on_connect_done(dev);
		break;
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERNOTREADY):
	case UDC_DWC3_DEVT_ULSTCHNG:
	case UDC_DWC3_DEVT_DISCONNEVT:
	case UDC_DWC3_DEVT_WKUPEVT:
	case UDC_DWC3_DEVT_SUSPEND:
	case UDC_DWC3_DEVT_SOF:
	case UDC_DWC3_DEVT_CMDCMPLT:
	case UDC_DWC3_DEVT_VNDRDEVTSTRCVED:
		break;
	case UDC_DWC3_DEVT_EVNTOVERFLOW:
		LOG_ERR("Event overflow");
		break;
	default:
		/*
		 * Counted, not fatal. CODE_UNREACHABLE here is undefined
		 * behaviour on an event the controller is entitled to post,
		 * and this Soft-IP is known to post events this driver does
		 * not model. Dropping one is survivable; faulting is not.
		 */
		LOG_ERR("unknown event: 0x%x (ignored)", evt);
		break;
	}
}

static void udc_dwc3_event_worker(struct k_work *work)
{
	struct udc_dwc3_data *const priv = CONTAINER_OF(work, struct udc_dwc3_data, event_work);
	const struct device *const dev = priv->dev;
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	while (sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) > 0) {
		const uint32_t evt = cfg->evt_buf[priv->evt_next];

		/* Dispatch the even directly from IRQ */
		udc_dwc3_handle_event(dev, evt);

		/* Move to next event entry for both hardware and software */
		sys_write32(sizeof(uint32_t), base + UDC_DWC3_GEVNTCOUNT(0));
		priv->evt_next = (priv->evt_next + 1) % CONFIG_UDC_DWC3_EVENTS_NUM;
	}

	/* Allow further interrupts */
	sys_clear_bits(base + UDC_DWC3_GEVNTSIZ(0), UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
	cfg->irq_enable_func();
}

static void udc_dwc3_irq_handler(void *const ptr)
{
	const struct device *const dev = ptr;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	k_work_submit_to_queue(udc_get_work_q(), &priv->event_work);

	/* Disable further interrupts until all events are processed */
	sys_set_bits(base + UDC_DWC3_GEVNTSIZ(0), UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
	cfg->irq_disable_func();
}

/*
 * UDC API
 *
 * Interface called by Zehpyr from the upper levels of abstractions.
 */

/*
 * Drop the RTL's claim on @p ep_data and leave the endpoint quiesced.
 *
 * This is the sequence the reference stack runs on a forced revoke
 * (clear-halt, reset, disconnect, ep-disable): stop that manager -- and only
 * that manager, so the other camera keeps streaming -- wait for it to go
 * idle, then end the transfer and scrub the ring. Only once the manager is
 * silent may a CPU endpoint command be issued, which is the whole point:
 * the two must never drive DWC3's single endpoint-command engine at once.
 *
 * Returns true when ownership was actually revoked, so the caller knows it
 * has to re-arm the endpoint afterwards.
 */
static bool udc_dwc3_hw_owned_revoke(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     const char *const reason)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	if (!ep_data->hw_owned) {
		return false;
	}

	/* Halt first. Everything below is a DEPCMD. */
	(void)udc_dwc3_hw_mgr_halt(dev, ep_data);

	ep_data->hw_owned = false;

	LOG_INF("ep 0x%02x <- RTL forced revoke (%s)", ep_data->cfg.addr, reason);

	/* Now safe: the guard above no longer refuses these. */
	udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);

	/* Scrub the ring the RTL was writing into, keeping the Link TRB. */
	memset(ep_data->trb_buf, 0,
	       sizeof(*ep_data->trb_buf) * (CONFIG_UDC_DWC3_TRB_NUM - 1));
	ep_data->head = ep_data->tail = 0;
	ep_data->full = false;

	if (priv->hw_owner_cb != NULL) {
		priv->hw_owner_cb(dev, ep_data->cfg.addr, reason, priv->hw_owner_user);
	}

	return true;
}

int lattice_usb23_ep_give_to_rtl(const struct device *const dev, const uint8_t ep_addr,
				 const uintptr_t mgr_base, uint32_t *const depupdxfer)
{
	struct udc_ep_config *const ep_cfg = udc_get_ep_cfg(dev, ep_addr);
	struct udc_dwc3_ep_data *ep_data;

	if (ep_cfg == NULL || mgr_base == 0U) {
		return -EINVAL;
	}

	ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	/*
	 * The endpoint must already have a live transfer resource: the RTL
	 * reuses the xferrscidx that the CPU's StartXfer obtained at
	 * ep_enable. Handing over without one leaves the manager ringing
	 * UpdateXfer against an index DWC3 does not recognise.
	 */
	if (ep_data->xferrscidx == 0U) {
		LOG_ERR("ep 0x%02x has no xferrscidx; refusing handoff", ep_addr);
		return -EAGAIN;
	}

	ep_data->hw_mgr_base = mgr_base;
	ep_data->hw_owned = true;

	if (depupdxfer != NULL) {
		*depupdxfer = UDC_DWC3_DEPCMD_DEPUPDXFER |
			      FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK,
					 ep_data->xferrscidx);
	}

	LOG_INF("ep 0x%02x -> RTL (xferrscidx=%u, mgr=0x%lx)", ep_addr,
		ep_data->xferrscidx, (unsigned long)mgr_base);
	return 0;
}

int lattice_usb23_ep_take_from_rtl(const struct device *const dev, const uint8_t ep_addr)
{
	struct udc_ep_config *const ep_cfg = udc_get_ep_cfg(dev, ep_addr);
	struct udc_dwc3_ep_data *ep_data;

	if (ep_cfg == NULL) {
		return -EINVAL;
	}

	ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	if (!ep_data->hw_owned) {
		return -EALREADY;
	}

	(void)udc_dwc3_hw_owned_revoke(dev, ep_data, "take_from_rtl");
	return 0;
}

bool lattice_usb23_ep_is_rtl_owned(const struct device *const dev, const uint8_t ep_addr)
{
	struct udc_ep_config *const ep_cfg = udc_get_ep_cfg(dev, ep_addr);

	if (ep_cfg == NULL) {
		return false;
	}

	return CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg)->hw_owned;
}

int lattice_usb23_set_hw_owner_cb(const struct device *const dev,
				  const lattice_usb23_hw_owner_cb_t cb, void *const user)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	priv->hw_owner_cb = cb;
	priv->hw_owner_user = user;
	return 0;
}

static int udc_dwc3_ep_enqueue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg,
			       struct net_buf *const buf)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	udc_buf_put(ep_cfg, buf);

	switch (ep_cfg->addr) {
	case USB_CONTROL_EP_IN:
	case USB_CONTROL_EP_OUT:
		udc_dwc3_next_ctrl(dev, ep_data);
		break;
	default:
		/* Process this buffer along with other waiting */
		if (sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP &&
		    ep_cfg->stat.enabled) {
			LOG_DBG("submitting to EP 0x%02x", ep_cfg->addr);
			k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
		}
	}

	return 0;
}

static int udc_dwc3_ep_dequeue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg)
{
	udc_ep_cancel_queued(dev, ep_cfg);
	udc_ep_set_busy(ep_cfg, false);

	return 0;
}

static int udc_dwc3_ep_resume(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	struct net_buf *buf;
	int ret;

	/* Reset all ongoing transfers on non-control OUT endpoints */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		/* No ForceRM: see udc_dwc3_depcmd_clear_stall(). */
		udc_dwc3_depcmd_clear_stall(dev, ep_data, 0);
	}

	udc_dwc3_depcmd_ep_config(dev, ep_data);
	udc_dwc3_depcmd_ep_xfer_config(dev, ep_data);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		udc_dwc3_trb_nonctrl_init(dev, ep_data);
	}

	/* Starting from here, the endpoint can be used */
	sys_set_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

	/* Re-enqueue all the previously dequeued buffers */
	while (true) {
		buf = k_fifo_get(&ep_data->requeue_fifo, K_NO_WAIT);
		if (buf == NULL) {
			break;
		}

		ret = udc_dwc3_trb_bulk(dev, ep_data, buf);
		if (ret != 0) {
			return ret;
		}
	}

	/* We might have blocked transfers earlier */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
	}

	return 0;
}

static int udc_dwc3_ep_enable(const struct device *const dev, struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = (struct udc_dwc3_ep_data *)ep_cfg;
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	LOG_DBG("ep 0x%02x, first ep 0x%02x", ep_data->cfg.addr, priv->first_ep);

	if (USB_EP_GET_IDX(ep_cfg->addr) > 0) {
		if (priv->first_ep == 0) {
			priv->first_ep = ep_cfg->addr;
		}
		if (ep_cfg->addr == priv->first_ep) {
			udc_dwc3_on_set_config_or_interface(dev);
		}
	}

	return udc_dwc3_ep_resume(dev, ep_data);
}

static int udc_dwc3_ep_disable(const struct device *const dev, struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	int last_num = CONFIG_UDC_DWC3_TRB_NUM - 1 - 1;
	struct net_buf *buf;

	LOG_DBG("Disabling EP 0x%02x", ep_cfg->addr);

	/*
	 * If the RTL is driving this endpoint, stop it before anything else.
	 * The EndXfer below used to be issued with the manager still ringing
	 * UpdateXfer on the same endpoint; that is the collision that leaves
	 * the endpoint unstoppable and hangs the host's Stop Endpoint.
	 */
	(void)udc_dwc3_hw_owned_revoke(dev, ep_data, "ep_disable");

	/* Disable the endpoint */
	sys_clear_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

	/* Reset ongoing transfers */
	udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);

	udc_ep_set_busy(ep_cfg, false);

	/* Walk in reverse order to enqueue them in correct order */
	for (int n = 0; n <= last_num; n++) {
		buf = ep_data->net_buf[ep_data->head];
		if (buf != NULL) {
			k_fifo_put(&ep_data->requeue_fifo, buf);
		}

		if (ep_data->head > 0) {
			ep_data->head--;
		} else {
			ep_data->head = last_num;
		}
	}

	/* Reset the buffers */
	memset(ep_data->trb_buf, 0, sizeof(*ep_data->trb_buf) * (CONFIG_UDC_DWC3_TRB_NUM - 1));
	memset(ep_data->net_buf, 0, sizeof(*ep_data->net_buf) * (CONFIG_UDC_DWC3_TRB_NUM - 1));
	ep_data->head = ep_data->tail = 0;
	ep_data->full = false;

	return 0;
}

static int udc_dwc3_ep_set_halt(const struct device *const dev,
				struct udc_ep_config *const ep_cfg)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	switch (ep_data->cfg.addr) {
	case USB_CONTROL_EP_IN:
		/* The datasheet says to only set stall the OUT direction */
		ep_data = &cfg->ep_data_out[0];
		__fallthrough;
	case USB_CONTROL_EP_OUT:
		udc_dwc3_depcmd_set_stall(dev, ep_data);
		break;
	default:
		udc_dwc3_depcmd_set_stall(dev, ep_data);
		ep_data->cfg.stat.halted = true;
	}

	return 0;
}

static int udc_dwc3_ep_clear_halt(const struct device *const dev,
				  struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	LOG_INF("Clearing stall for ep 0x%02x", ep_cfg->addr);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		return 0;
	}

	/*
	 * A bulk UVC stream has a single alt setting, so uvcvideo signals
	 * STREAMOFF with CLEAR_FEATURE(ENDPOINT_HALT) on the video endpoint --
	 * which lands here while the RTL is still streaming it. Revoking
	 * first halts that manager and ends its transfer; only then is the
	 * ClearStall below a command against an endpoint nothing else drives.
	 */
	const bool was_rtl = udc_dwc3_hw_owned_revoke(dev, ep_data, "clear-halt");

	/* No ForceRM: see udc_dwc3_depcmd_clear_stall(). */
	udc_dwc3_depcmd_clear_stall(dev, ep_data, 0);
	ep_data->cfg.stat.halted = false;

	/*
	 * ClearStall resets the SuperSpeed sequence number, so an endpoint
	 * that was RTL-owned needs a fresh StartXfer before it can carry
	 * anything again; the video driver's next STREAMON hands back an
	 * endpoint that is already armed.
	 */
	if (was_rtl) {
		udc_dwc3_depcmd_start_xfer(dev, ep_data);
	}

	/* Resume halted previously transfers */
	k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);

	return 0;
}

static int udc_dwc3_set_address_no_op(const struct device *const dev, const uint8_t addr)
{
	return 0;
}

static int udc_dwc3_set_address(const struct device *const dev, const uint8_t addr)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	LOG_INF("Setting address to %u", addr);

	/* Configure the new address */
	reg = sys_read32(base + UDC_DWC3_DCFG);
	reg &= ~UDC_DWC3_DCFG_DEVADDR_MASK;
	reg |= FIELD_PREP(UDC_DWC3_DCFG_DEVADDR_MASK, addr);
	sys_write32(reg, base + UDC_DWC3_DCFG);

	return 0;
}

static enum udc_bus_speed udc_dwc3_device_speed(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	switch (sys_read32(base + UDC_DWC3_DSTS) & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
		return UDC_BUS_SPEED_FS;
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
		return UDC_BUS_SPEED_HS;
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		return UDC_BUS_SPEED_SS;
	}

	LOG_ERR("Unknown device speed");

	return 0;
}

static int udc_dwc3_enable(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	int ret;

	LOG_INF("Enabling DWC3 driver");

	ret = udc_dwc3_quirk_enable(dev);
	if (ret != 0) {
		return ret;
	}

	/* Enable the DWC3 events */
	sys_set_bits(base + UDC_DWC3_DCTL, UDC_DWC3_DCTL_RUNSTOP);

	/* Enable the IRQ (for now, just schedule a first work queue job) */
	cfg->irq_enable_func();

	return 0;
}

static int udc_dwc3_disable(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	LOG_DBG("Disabling DWC3 driver");

	sys_clear_bits(base + UDC_DWC3_DCTL, UDC_DWC3_DCTL_RUNSTOP);

	return 0;
}

static int udc_dwc3_init(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;
	int ret;

	LOG_DBG("Initializing the DWC3 core");

	ret = udc_dwc3_quirk_init(dev);
	if (ret != 0) {
		return ret;
	}

	/* Issue a soft reset to the core and USB2 and USB3 PHY */
	sys_set_bits(base + UDC_DWC3_GCTL, UDC_DWC3_GCTL_CORESOFTRESET);
	sys_set_bits(base + UDC_DWC3_GUSB3PIPECTL, UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST);
	sys_set_bits(base + UDC_DWC3_GUSB2PHYCFG, UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST);
	k_sleep(K_USEC(100));

	/* Teriminate the reset of the USB2 and USB3 PHY first */
	sys_clear_bits(base + UDC_DWC3_GUSB3PIPECTL, UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST);
	sys_clear_bits(base + UDC_DWC3_GUSB2PHYCFG, UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST);

	/* Teriminate the reset of the DWC3 core after it */
	sys_clear_bits(base + UDC_DWC3_GCTL, UDC_DWC3_GCTL_CORESOFTRESET);

	/* The USB core was reset, configure it as documented */
	udc_dwc3_on_soft_reset(dev);

	/* Configure the control OUT endpoint */
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 512, 0);
	if (ret != 0) {
		LOG_ERR("could not enable control OUT ep");
		return ret;
	}

	/* Configure the control IN endpoint */
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 512, 0);
	if (ret != 0) {
		LOG_ERR("could not enable control IN ep");
		return ret;
	}

#if CONFIG_UDC_DWC3_SHELL
	/* Initialize default queue sizes */
	udc_dwc3_init_fifo_space(dev);
#endif

	LOG_DBG("GHWPARAMS3");
	reg = sys_read32(base + UDC_DWC3_GHWPARAMS3);

	LOG_DBG("- GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES_MASK, reg));
	LOG_DBG("- GHWPARAMS3_NUM_IN_EPS %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS3_NUM_IN_EPS_MASK, reg));
	LOG_DBG("- GHWPARAMS3_NUM_EPS %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS3_NUM_EPS_MASK, reg));

	LOG_DBG("GHWPARAMS4");
	reg = sys_read32(base + UDC_DWC3_GHWPARAMS4);

	LOG_DBG("- BMU_LSP_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS4_BMU_LSP_DEPTH_MASK, reg));
	LOG_DBG("- BMU_PTL_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS4_BMU_PTL_DEPTH_M1_MASK, reg) + 1);
	LOG_DBG("- CACHE_TRBS_PER_TRANSFER: %lu",
		FIELD_GET(UDC_DWC3_GHWPARAMS4_CACHE_TRBS_PER_TRANSFER_MASK, reg));

	LOG_DBG("GHWPARAMS5");
	reg = sys_read32(base + UDC_DWC3_GHWPARAMS5);

	LOG_DBG("- GHWPARAMS5_DFQ_FIFO_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS5_DFQ_FIFO_DEPTH_MASK, reg));
	LOG_DBG("- GHWPARAMS5_DWQ_FIFO_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS5_DWQ_FIFO_DEPTH_MASK, reg));
	LOG_DBG("- GHWPARAMS5_TXQ_FIFO_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS5_TXQ_FIFO_DEPTH_MASK, reg));
	LOG_DBG("- GHWPARAMS5_RXQ_FIFO_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS5_RXQ_FIFO_DEPTH_MASK, reg));
	LOG_DBG("- GHWPARAMS5_BMU_BUSGM_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS5_BMU_BUSGM_DEPTH_MASK, reg));

	LOG_DBG("Event buffer is %u bytes", sys_read32(base + UDC_DWC3_GEVNTSIZ(0)));

	return 0;
}

static const struct udc_api udc_dwc3_api = {
	.lock = udc_dwc3_lock,
	.unlock = udc_dwc3_unlock,
	.device_speed = udc_dwc3_device_speed,
	.init = udc_dwc3_init,
	.enable = udc_dwc3_enable,
	.disable = udc_dwc3_disable,
	.shutdown = udc_dwc3_shutdown,
	.set_address = udc_dwc3_set_address_no_op,
	.ep_enable = udc_dwc3_ep_enable,
	.ep_disable = udc_dwc3_ep_disable,
	.ep_set_halt = udc_dwc3_ep_set_halt,
	.ep_clear_halt = udc_dwc3_ep_clear_halt,
	.ep_enqueue = udc_dwc3_ep_enqueue,
	.ep_dequeue = udc_dwc3_ep_dequeue,
};

static void udc_dwc3_ep_worker(struct k_work *const work)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(work, struct udc_dwc3_ep_data, work);
	const struct device *const dev = ep_data->dev;
	struct net_buf *buf;
	int ret;

	LOG_DBG("checking for pending transfers for EP 0x%02x", ep_data->cfg.addr);

	if (ep_data->cfg.stat.halted) {
		LOG_DBG("endpoint is halted, not processing buffers");
		return;
	}

	while ((buf = udc_buf_peek(&ep_data->cfg)) != NULL) {
		LOG_INF("Processing buffer %p from queue", (void *)buf);

		ret = udc_dwc3_trb_bulk(dev, ep_data, buf);
		if (ret != 0) {
			LOG_DBG("abort: No more room for buffer");
			break;
		}

		LOG_DBG("success: Buffer enqueued");

		udc_buf_get(&ep_data->cfg);
	}
}

/*
 * Initialize the controller and endpoints capabilities,
 * register endpoint structures, no hardware I/O yet.
 */
static int udc_dwc3_driver_preinit(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_data *const data = dev->data;
	struct udc_dwc3_ep_data *ep_data;
	uint16_t mps = 0;
	int ret;

	ret = udc_dwc3_quirk_preinit(dev);
	if (ret != 0) {
		return ret;
	}

	DEVICE_MMIO_NAMED_MAP(dev, base, K_MEM_CACHE_NONE);

	k_mutex_init(&data->mutex);
	k_work_init(&priv->event_work, udc_dwc3_event_worker);
#if defined(CONFIG_UDC_DWC3_COMPLETION_POLL)
	/*
	 * Runs on the UDC work queue, the same one as the event worker and the
	 * per-endpoint work, so a software retire is serialised against the
	 * real event path rather than racing it from another thread.
	 */
	k_work_init_delayable(&priv->poll_work, udc_dwc3_completion_poll_worker);
	k_work_schedule_for_queue(udc_get_work_q(), &priv->poll_work,
				  K_MSEC(CONFIG_UDC_DWC3_COMPLETION_POLL_MS));
#endif

	data->caps.rwup = false;
	data->caps.addr_before_status = true;

	switch (cfg->maximum_speed_idx) {
	case UDC_DWC3_SPEED_IDX_SUPER_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_SUPER_SPEED");
		data->caps.mps0 = UDC_MPS0_512;
		data->caps.hs = true;
		data->caps.ss = true;
		mps = 1024;
		break;
	case UDC_DWC3_SPEED_IDX_HIGH_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_HIGH_SPEED");
		data->caps.mps0 = UDC_MPS0_64;
		data->caps.hs = true;
		mps = 1024;
		break;
	case UDC_DWC3_SPEED_IDX_FULL_SPEED:
		LOG_DBG("UDC_DWC3_SPEED_IDX_FULL_SPEED");
		data->caps.mps0 = UDC_MPS0_64;
		mps = 64;
		break;
	default:
		LOG_ERR("Speed %d not supported", cfg->maximum_speed_idx);
		return -ENOTSUP;
	}

	/* Control IN endpoint */

	ep_data = &cfg->ep_data_in[0];
	ep_data->dev = dev;
	ep_data->cfg.addr = USB_CONTROL_EP_IN;
	ep_data->cfg.caps.in = 1;
	ep_data->cfg.caps.control = 1;
	ep_data->cfg.caps.mps = mps;
	ep_data->trb_buf = cfg->trb_buf_in[0];
	ep_data->epn = 1;

	ret = udc_register_ep(dev, &ep_data->cfg);
	if (ret != 0) {
		LOG_ERR("Failed to register endpoint");
		return ret;
	}

	/* Control OUT endpoint */

	ep_data = &cfg->ep_data_out[0];
	ep_data->dev = dev;
	ep_data->cfg.addr = USB_CONTROL_EP_OUT;
	ep_data->cfg.caps.out = 1;
	ep_data->cfg.caps.control = 1;
	ep_data->cfg.caps.mps = mps;
	ep_data->trb_buf = cfg->trb_buf_out[0];
	ep_data->epn = 0;

	ret = udc_register_ep(dev, &ep_data->cfg);
	if (ret != 0) {
		LOG_ERR("Failed to register endpoint");
		return ret;
	}

	/* Normal IN endpoints */
	for (int i = 1; i < cfg->num_in_eps; i++) {
		LOG_DBG("Preinit endpoint 0x%02x", USB_EP_DIR_IN | i);

		ep_data = &cfg->ep_data_in[i];
		k_work_init(&ep_data->work, udc_dwc3_ep_worker);
		k_fifo_init(&ep_data->requeue_fifo);

		ep_data->dev = dev;
		ep_data->cfg.addr = USB_EP_DIR_IN | i;
		ep_data->cfg.caps.in = true;
		ep_data->cfg.caps.bulk = true;
		ep_data->cfg.caps.interrupt = true;
		ep_data->cfg.caps.iso = true;
		ep_data->cfg.caps.mps = mps;
		ep_data->trb_buf = cfg->trb_buf_in[i];
		ep_data->epn = (i << 1) | 1;

		ret = udc_register_ep(dev, &ep_data->cfg);
		if (ret != 0) {
			LOG_ERR("Failed to register endpoint");
			return ret;
		}
	}

	/* Normal OUT endpoints */
	for (int i = 1; i < cfg->num_out_eps; i++) {
		LOG_DBG("Preinit endpoint 0x%02x", USB_EP_DIR_OUT | i);

		ep_data = &cfg->ep_data_out[i];
		k_work_init(&ep_data->work, udc_dwc3_ep_worker);

		ep_data->dev = dev;
		ep_data->cfg.addr = USB_EP_DIR_OUT | i;
		ep_data->cfg.caps.out = true;
		ep_data->cfg.caps.bulk = true;
		ep_data->cfg.caps.interrupt = true;
		ep_data->cfg.caps.iso = true;
		ep_data->cfg.caps.mps = mps;
		ep_data->trb_buf = cfg->trb_buf_out[i];
		ep_data->epn = (i << 1) | 0;

		ret = udc_register_ep(dev, &ep_data->cfg);
		if (ret != 0) {
			LOG_ERR("Failed to register endpoint");
			return ret;
		}
	}

	return 0;
}

#define UDC_DWC3_DEVICE_DEFINE(n)						\
	UDC_DWC3_QUIRK_DEFINE(n);						\
										\
	static void udc_dwc3_irq_enable_func_##n(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
			    udc_dwc3_irq_handler, DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQN(n));					\
	}									\
										\
	static void udc_dwc3_irq_disable_func_##n(void)				\
	{									\
		irq_disable(DT_INST_IRQN(n));					\
	}									\
										\
	static __nocache uint32_t udc_dwc3_dma_evt_buf_##n			\
		[CONFIG_UDC_DWC3_EVENTS_NUM]					\
		__aligned(16);							\
										\
	static __nocache struct udc_dwc3_trb udc_dwc3_dma_trb_i##n		\
		[DT_INST_PROP(n, num_in_endpoints)][CONFIG_UDC_DWC3_TRB_NUM]	\
		__aligned(16);							\
										\
	static __nocache struct udc_dwc3_trb udc_dwc3_dma_trb_o##n		\
		[DT_INST_PROP(n, num_out_endpoints)][CONFIG_UDC_DWC3_TRB_NUM]	\
		__aligned(16);							\
										\
	static struct udc_dwc3_ep_data udc_dwc3_ep_data_i##n			\
		[DT_INST_PROP(n, num_in_endpoints)];				\
										\
	static struct udc_dwc3_ep_data udc_dwc3_ep_data_o##n			\
		[DT_INST_PROP(n, num_out_endpoints)];				\
										\
	static const struct udc_dwc3_config udc_dwc3_config_##n = {		\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(base, DT_DRV_INST(n)),	\
		.quirk_data = &udc_dwc3_quirk_data_##n,				\
		.quirk_config = &udc_dwc3_quirk_config_##n,			\
		.num_in_eps = DT_INST_PROP(n, num_in_endpoints),		\
		.num_out_eps = DT_INST_PROP(n, num_out_endpoints),		\
		.ep_data_in  = udc_dwc3_ep_data_i##n,				\
		.ep_data_out = udc_dwc3_ep_data_o##n,				\
		.trb_buf_in = udc_dwc3_dma_trb_i##n,				\
		.trb_buf_out = udc_dwc3_dma_trb_o##n,				\
		.evt_buf = udc_dwc3_dma_evt_buf_##n,				\
		.maximum_speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),\
		.irq_enable_func = udc_dwc3_irq_enable_func_##n,		\
		.irq_disable_func = udc_dwc3_irq_disable_func_##n,		\
	};									\
										\
	static struct udc_dwc3_data udc_dwc3_priv_##n = {			\
		.dev = DEVICE_DT_INST_GET(n),					\
	};									\
										\
	static struct udc_data udc_data_##n = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),		\
		.priv = &udc_dwc3_priv_##n,					\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, udc_dwc3_driver_preinit, NULL, &udc_data_##n,	\
			      &udc_dwc3_config_##n, POST_KERNEL,		\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			      &udc_dwc3_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_DWC3_DEVICE_DEFINE)

/*
 * Shell
 *
 * Commands to debug DWC3 hardware, DWC3 driver, USB hosts, device-side applications,
 * and cable problems.
 */

#ifdef CONFIG_UDC_DWC3_SHELL

const struct udc_dwc3_reg {
	uint32_t addr;
	char *name;
} udc_dwc3_regs[] = {
	/* main registers */
	{.addr = UDC_DWC3_GCTL, .name = "GCTL"},
	{.addr = UDC_DWC3_DCTL, .name = "DCTL"},
	{.addr = UDC_DWC3_DCFG, .name = "DCFG"},
	{.addr = UDC_DWC3_DEVTEN, .name = "DEVTEN"},
	{.addr = UDC_DWC3_DALEPENA, .name = "DALEPENA"},
	{.addr = UDC_DWC3_GCOREID, .name = "GCOREID"},
	{.addr = UDC_DWC3_GSTS, .name = "GSTS"},
	{.addr = UDC_DWC3_DSTS, .name = "DSTS"},
	{.addr = UDC_DWC3_GEVNTADR_LO(0), .name = "GEVNTADR_LO(0)"},
	{.addr = UDC_DWC3_GEVNTADR_HI(0), .name = "GEVNTADR_HI(0)"},
	{.addr = UDC_DWC3_GEVNTSIZ(0), .name = "GEVNTSIZ(0)"},
	{.addr = UDC_DWC3_GEVNTCOUNT(0), .name = "GEVNTCOUNT(0)"},
	{.addr = UDC_DWC3_GUSB2PHYCFG, .name = "GUSB2PHYCFG"},
	{.addr = UDC_DWC3_GUSB3PIPECTL, .name = "GUSB3PIPECTL"},
	/* debug */
	{.addr = UDC_DWC3_GBUSERRADDR_LO, .name = "GBUSERRADDR_LO"},
	{.addr = UDC_DWC3_GBUSERRADDR_HI, .name = "GBUSERRADDR_HI"},
	{.addr = UDC_DWC3_CTLDEBUG_LO, .name = "CTLDEBUG_LO"},
	{.addr = UDC_DWC3_CTLDEBUG_HI, .name = "CTLDEBUG_HI"},
	{.addr = UDC_DWC3_ANALYZERTRACE, .name = "ANALYZERTRACE"},
	{.addr = UDC_DWC3_GDBGFIFOSPACE, .name = "GDBGFIFOSPACE"},
	{.addr = UDC_DWC3_GDBGLTSSM, .name = "GDBGLTSSM"},
	{.addr = UDC_DWC3_GDBGLNMCC, .name = "GDBGLNMCC"},
	{.addr = UDC_DWC3_GDBGBMU, .name = "GDBGBMU"},
	{.addr = UDC_DWC3_GDBGLSPMUX_DEV, .name = "GDBGLSPMUX_DEV"},
	{.addr = UDC_DWC3_GDBGLSPMUX_HST, .name = "GDBGLSPMUX_HST"},
	{.addr = UDC_DWC3_GDBGLSP, .name = "GDBGLSP"},
	{.addr = UDC_DWC3_GDBGEPINFO0, .name = "GDBGEPINFO0"},
	{.addr = UDC_DWC3_GDBGEPINFO1, .name = "GDBGEPINFO1"},
	{.addr = UDC_DWC3_BU3RHBDBG0, .name = "BU3RHBDBG0"},
	/* physical endpoint numbers */
	{.addr = UDC_DWC3_DEPCMDPAR2(0), .name = "DEPCMDPAR2(0)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(0), .name = "DEPCMDPAR1(0)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(0), .name = "DEPCMDPAR0(0)"},
	{.addr = UDC_DWC3_DEPCMD(0), .name = "DEPCMD(0)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(1), .name = "DEPCMDPAR2(1)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(1), .name = "DEPCMDPAR1(1)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(1), .name = "DEPCMDPAR0(1)"},
	{.addr = UDC_DWC3_DEPCMD(1), .name = "DEPCMD(1)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(2), .name = "DEPCMDPAR2(2)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(2), .name = "DEPCMDPAR1(2)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(2), .name = "DEPCMDPAR0(2)"},
	{.addr = UDC_DWC3_DEPCMD(2), .name = "DEPCMD(2)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(3), .name = "DEPCMDPAR2(3)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(3), .name = "DEPCMDPAR1(3)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(3), .name = "DEPCMDPAR0(3)"},
	{.addr = UDC_DWC3_DEPCMD(3), .name = "DEPCMD(3)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(4), .name = "DEPCMDPAR2(4)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(4), .name = "DEPCMDPAR1(4)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(4), .name = "DEPCMDPAR0(4)"},
	{.addr = UDC_DWC3_DEPCMD(4), .name = "DEPCMD(4)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(5), .name = "DEPCMDPAR2(5)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(5), .name = "DEPCMDPAR1(5)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(5), .name = "DEPCMDPAR0(5)"},
	{.addr = UDC_DWC3_DEPCMD(5), .name = "DEPCMD(5)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(6), .name = "DEPCMDPAR2(6)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(6), .name = "DEPCMDPAR1(6)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(6), .name = "DEPCMDPAR0(6)"},
	{.addr = UDC_DWC3_DEPCMD(6), .name = "DEPCMD(6)"},
	{.addr = UDC_DWC3_DEPCMDPAR2(7), .name = "DEPCMDPAR2(7)"},
	{.addr = UDC_DWC3_DEPCMDPAR1(7), .name = "DEPCMDPAR1(7)"},
	{.addr = UDC_DWC3_DEPCMDPAR0(7), .name = "DEPCMDPAR0(7)"},
	{.addr = UDC_DWC3_DEPCMD(7), .name = "DEPCMD(7)"},
	/* Hardware parameters */
	{.addr = UDC_DWC3_GHWPARAMS0, .name = "GHWPARAMS0"},
	{.addr = UDC_DWC3_GHWPARAMS1, .name = "GHWPARAMS1"},
	{.addr = UDC_DWC3_GHWPARAMS2, .name = "GHWPARAMS2"},
	{.addr = UDC_DWC3_GHWPARAMS3, .name = "GHWPARAMS3"},
	{.addr = UDC_DWC3_GHWPARAMS4, .name = "GHWPARAMS4"},
	{.addr = UDC_DWC3_GHWPARAMS5, .name = "GHWPARAMS5"},
	{.addr = UDC_DWC3_GHWPARAMS6, .name = "GHWPARAMS6"},
	{.addr = UDC_DWC3_GHWPARAMS7, .name = "GHWPARAMS7"},
	{.addr = UDC_DWC3_GHWPARAMS8, .name = "GHWPARAMS8"},
};

static const struct {
	char *name;
	uint32_t type;
} udc_dwc3_fifo_regs[_NUM_FIFO_REGS] = {
	{
		.name = "TxQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXQ
	}, {
		.name = "RxQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXQ
	}, {
		.name = "TxReqQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXREQQ
	}, {
		.name = "RxReqQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXREQQ
	}, {
		.name = "RxInfoQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXINFOQ
	}, {
		.name = "DescFetchQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCHQ
	}, {
		.name = "WriteBack/EventQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_WREVENTQ
	},
};

static uint32_t udc_dwc3_read_fifo_space(const struct device *dev, uint32_t type, uint32_t num)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t mdwidth = (sys_read32(base + UDC_DWC3_GHWPARAMS0) >> 8) & 0xFF;
	uint32_t reg;

	reg = type;
	reg |= FIELD_PREP(UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK, num);
	sys_write32(reg, base + UDC_DWC3_GDBGFIFOSPACE);

	reg = sys_read32(base + UDC_DWC3_GDBGFIFOSPACE);
	reg = FIELD_GET(UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK, reg);
	return reg * mdwidth / BITS_PER_BYTE;
}

static void udc_dwc3_init_fifo_space(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);

	for (int n = 0; n < _NUM_FIFO_SPACE; n++) {
		for (int i = 0; i < _NUM_FIFO_REGS; i++) {
			priv->max_bytes_avail[n][i] = udc_dwc3_read_fifo_space(
				dev, udc_dwc3_fifo_regs[i].type, n);
		}
	}
}

static void udc_dwc3_dump_registers(const struct device *dev, const struct shell *sh)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	for (int i = 0; i < ARRAY_SIZE(udc_dwc3_regs); i++) {
		const struct udc_dwc3_reg *ureg = &udc_dwc3_regs[i];

		reg = sys_read32(base + ureg->addr);
		shell_print(sh, "reg 0x%08x == 0x%08x %s", ureg->addr, reg, ureg->name);
	}
}

static void udc_dwc3_dump_bus_error(const struct device *dev, const struct shell *sh)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	if (sys_read32(base + UDC_DWC3_GSTS) & UDC_DWC3_GSTS_BUSERRADDRVLD) {
		shell_print(sh, "BUS_ERROR addr=0x%08x%08x",
			     sys_read32(base + UDC_DWC3_GBUSERRADDR_HI),
			     sys_read32(base + UDC_DWC3_GBUSERRADDR_LO));
	} else {
		shell_print(sh, "no bus error");
	}
}

static void udc_dwc3_dump_link_state(const struct device *dev, const struct shell *sh)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	reg = sys_read32(base + UDC_DWC3_DSTS);
	switch (reg & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
		shell_print(sh, "DWC3_DSTS_CONNECTSPD_HS");
		goto usb2;
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
		shell_print(sh, "DWC3_DSTS_CONNECTSPD_FS");
		goto usb2;
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		shell_print(sh, "DWC3_DSTS_CONNECTSPD_SS");
		goto usb3;
	default:
		shell_print(sh, "unknown speed");
	}
	return;
usb2:
	switch (reg & UDC_DWC3_DSTS_USBLNKST_MASK) {
	case UDC_DWC3_DSTS_USBLNKST_USB2_ON_STATE:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_ON_STATE");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_DISCONNECTED:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_DISCONNECTED");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_RESET:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_RESET");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB2_RESUME:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB2_RESUME");
		break;
	}
	return;
usb3:
	switch (reg & UDC_DWC3_DSTS_USBLNKST_MASK) {
	case UDC_DWC3_DSTS_USBLNKST_USB3_U0:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_U0");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_U1:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_U1");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_U2:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_U2");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_U3:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_U3");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_SS_DIS:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_SS_DIS");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_RX_DET:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_RX_DET");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_SS_INACT:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_SS_INACT");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_POLL:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_POLL");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_RECOV:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_RECOV");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_HRESET:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_HRESET");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_CMPLY:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_CMPLY");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_LPBK:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_LPBK");
		break;
	case UDC_DWC3_DSTS_USBLNKST_USB3_RESET_RESUME:
		shell_print(sh, "DWC3_DSTS_USBLNKST_USB3_RESET_RESUME");
		break;
	}
}

static void udc_dwc3_dump_events(const struct device *dev, const struct shell *sh)
{
	const struct udc_dwc3_config *cfg = dev->config;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	for (int i = 0; i < CONFIG_UDC_DWC3_EVENTS_NUM; i++) {
		uint32_t evt = cfg->evt_buf[i];
		char *s = (i == priv->evt_next) ? "<-" : "  ";

		shell_print(sh, "evt 0x%02x: 0x%08x %s %s",
			i, evt, s, udc_dwc3_get_event_name(evt, 0));
	}
}

static void udc_dwc3_dump_trb(const struct device *dev, struct udc_dwc3_ep_data *ep_data,
		     const struct shell *sh)
{
	for (int i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
		struct udc_dwc3_trb trb = ep_data->trb_buf[i];
		bool hwo = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_HWO);
		bool lst = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_LST);
		bool chn = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_CHN);
		bool csp = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_CSP);
		bool isp = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_ISP_IMI);
		bool ioc = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_IOC);
		bool spr = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_SPR);
		uint32_t trbctl = FIELD_GET(UDC_DWC3_TRB_CTRL_TRBCTL_MASK, trb.ctrl);
		uint32_t trbsts = FIELD_GET(UDC_DWC3_TRB_STATUS_TRBSTS_MASK, trb.status);
		uint32_t pcm1 = FIELD_GET(UDC_DWC3_TRB_CTRL_PCM1_MASK, trb.ctrl);
		uint32_t sidsofn = FIELD_GET(UDC_DWC3_TRB_CTRL_SIDSOFN_MASK, trb.ctrl);
		uint32_t bufsiz = FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb.status);
		char *head = (i == ep_data->head) ? " <HEAD" : "";
		char *tail = (i == ep_data->tail) ? " <TAIL" : "";
		char *full = (i == ep_data->head && ep_data->full) ? " <FULL" : "";

		shell_print(sh, "%p ep=0x%02x addr=0x%08x%08x ctl=%u sts=%u hwo=%u lst=%u chn=%u"
			" csp=%u isp=%u ioc=%u spr=%u pcm1=%u sof=%u bufsiz=%u%s%s%s",
			&ep_data->trb_buf[i], ep_data->cfg.addr, trb.addr_hi, trb.addr_lo, trbctl,
			trbsts, hwo, lst, chn, csp, isp, ioc, spr, pcm1, sidsofn, bufsiz, head,
			tail, full);
	}
}

static void udc_dwc3_dump_each(const struct device *dev,
			     void (*fn)(const struct device *, struct udc_dwc3_ep_data *,
					const struct shell *),
			     char *label, const struct shell *sh)
{
	const struct udc_dwc3_config *cfg = dev->config;

	for (int i = 0; i < cfg->num_in_eps; i++) {
		struct udc_dwc3_ep_data *ep_data = &cfg->ep_data_in[i];
		uint8_t addr = ep_data->cfg.addr;

		if (ep_data->trb_buf == NULL) {
			continue;
		}

		shell_print(sh, "%s for IN endpoint 0x%02x (%u %s)",
			  label, addr, addr & 0x7f, (addr & 0x80) ? "IN" : "OUT");
		(*fn)(dev, ep_data, sh);
	}

	for (int i = 0; i < cfg->num_out_eps; i++) {
		struct udc_dwc3_ep_data *ep_data = &cfg->ep_data_out[i];
		uint8_t addr = ep_data->cfg.addr;

		if (ep_data->trb_buf == NULL) {
			continue;
		}

		shell_print(sh, "%s for OUT endpoint 0x%02x (%u %s)",
			  label, addr, addr & 0x7f, (addr & 0x80) ? "IN" : "OUT");
		(*fn)(dev, ep_data, sh);
	}
}

static void udc_dwc3_dump_each_trb(const struct device *dev, const struct shell *sh)
{
	udc_dwc3_dump_each(dev, udc_dwc3_dump_trb, "trb", sh);
}

static void udc_dwc3_dump_fifo_space(const struct device *dev, const struct shell *sh)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t num_in_eps;
	uint32_t num_eps;
	uint32_t total_xfer_resources;
	uint32_t avail;
	uint32_t reg;

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS3);
	num_in_eps = FIELD_GET(UDC_DWC3_GHWPARAMS3_NUM_IN_EPS_MASK, reg);
	num_eps = FIELD_GET(UDC_DWC3_GHWPARAMS3_NUM_EPS_MASK, reg);
	total_xfer_resources = FIELD_GET(UDC_DWC3_GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES_MASK, reg);

	shell_print(sh, "num_in_eps: %u", num_in_eps);
	shell_print(sh, "num_eps: %u", num_eps);
	shell_print(sh, "total_xfer_resources: %u", total_xfer_resources);

	for (size_t n = 0; n < _NUM_FIFO_SPACE; n++) {
		shell_print(sh, "");
		shell_print(sh, "FIFO %u", n);

		for (size_t i = 0; i < ARRAY_SIZE(udc_dwc3_fifo_regs); i++) {
			avail = udc_dwc3_read_fifo_space(dev, udc_dwc3_fifo_regs[i].type, n);
			shell_print(sh, "- %-15s = %u / %u bytes available",
				udc_dwc3_fifo_regs[i].name, avail,
				(uint32_t)priv->max_bytes_avail[n][i]);
		}
	}

	shell_print(sh, "");
	shell_print(sh, "Common");

	avail = udc_dwc3_read_fifo_space(
		dev, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_PROTOCOLSTATUSQ, 0);
	shell_print(sh, "- %-15s = %u bytes available", "PROTOCOLSTATUS", avail);
}

static void udc_dwc3_dump_all(const struct device *dev, const struct shell *sh)
{
	shell_print(sh, "");
	shell_print(sh, "Registers:");
	udc_dwc3_dump_registers(dev, sh);
	shell_print(sh, "");
	shell_print(sh, "Bus Errors:");
	udc_dwc3_dump_bus_error(dev, sh);
	shell_print(sh, "");
	shell_print(sh, "Link State:");
	udc_dwc3_dump_link_state(dev, sh);
	shell_print(sh, "");
	shell_print(sh, "Events:");
	udc_dwc3_dump_events(dev, sh);
	shell_print(sh, "");
	shell_print(sh, "FIFO Space:");
	udc_dwc3_dump_fifo_space(dev, sh);
	shell_print(sh, "");
	shell_print(sh, "TRBs:");
	udc_dwc3_dump_each_trb(dev, sh);
	shell_print(sh, "");
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

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev == NULL) ? NULL : dev->name;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}
SHELL_DYNAMIC_CMD_CREATE(dsub_device_name, device_name_get);

static int cmd_dwc3_trb(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_each_trb);
}

static int cmd_dwc3_evt(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_events);
}

static int cmd_dwc3_reg(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_registers);
}

static int cmd_dwc3_buserr(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_bus_error);
}

static int cmd_dwc3_link(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_link_state);
}

static int cmd_dwc3_fifo(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_fifo_space);
}

static int cmd_dwc3_all(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_dump_all);
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_dwc3,
	SHELL_CMD_ARG(trb, &dsub_device_name,
		      "Dump an endpoint's TRB buffer\nUsage: trb <device>",
		      cmd_dwc3_trb, 2, 0),
	SHELL_CMD_ARG(evt, &dsub_device_name,
		      "Dump the device event buffer\nUsage: evt <device>",
		      cmd_dwc3_evt, 2, 0),
	SHELL_CMD_ARG(reg, &dsub_device_name,
		      "Dump the device status registers\nUsage: reg <device>",
		      cmd_dwc3_reg, 2, 0),
	SHELL_CMD_ARG(buserr, &dsub_device_name,
		      "Dump the AXI64 bus I/O errors\nUsage: buserr <device>",
		      cmd_dwc3_buserr, 2, 0),
	SHELL_CMD_ARG(link, &dsub_device_name,
		      "Dump the USB link state\nUsage: link <device>",
		      cmd_dwc3_link, 2, 0),
	SHELL_CMD_ARG(fifo, &dsub_device_name,
		      "Dump the FIFO available space\nUsage: fifo <device>",
		      cmd_dwc3_fifo, 2, 0),
	SHELL_CMD_ARG(all, &dsub_device_name,
		      "Dump everything\nUsage: all <device>",
		      cmd_dwc3_all, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dwc3, &sub_dwc3, "Synopsys DWC3 controller commands", NULL);

#endif /* CONFIG_UDC_DWC3_SHELL */
