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
#include <zephyr/sys/util.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include "udc_common.h"

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
#define UDC_DWC3_DEPEVT_STATUS_B3_MASK				GENMASK(2, 0)
#define UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_SETUP			(0x0 << 0)
#define UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_DATA			(0x1 << 0)
#define UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_STATUS		(0x2 << 0)
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
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TX			(0x0 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RX			(0x1 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXREQ			(0x2 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXREQ			(0x3 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXINFO			(0x4 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_PROTOCOL		(0x5 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCH		(0x6 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_WREVENT		(0x7 << 5)
#define UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_AUXEVENT		(0x8 << 5)
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
#define UDC_DWC3_DGCMD_FIFOFLUSHALL				(10 << 0)
#define UDC_DWC3_DGCMD_ENDPOINTNRDY				(12 << 0)
#define UDC_DWC3_DGCMD_LOOPBACKTEST				(16 << 0)
#define UDC_DWC3_DGCMD_ROLEREQUEST				(6 << 0)

/* Hardware parameters */
#define UDC_DWC3_GHWPARAMS0					0xc140
#define UDC_DWC3_GHWPARAMS1					0xc144
#define UDC_DWC3_GHWPARAMS2					0xc148
#define UDC_DWC3_GHWPARAMS3					0xc14c
#define UDC_DWC3_GHWPARAMS4					0xc150
#define UDC_DWC3_GHWPARAMS5					0xc154
#define UDC_DWC3_GHWPARAMS6					0xc158
#define UDC_DWC3_GHWPARAMS6_USB3_HSPHY_INTERFACE		GENMASK(5, 4)
#define UDC_DWC3_GHWPARAMS7					0xc15c
#define UDC_DWC3_GHWPARAMS7_RAM1_DEPTH_MASK			GENMASK(15, 0)
#define UDC_DWC3_GHWPARAMS8					0xc600

/* Helper macros */
#define LO32(n)			((uint32_t)((uint64_t)(n) & 0xffffffff))
#define HI32(n)			((uint32_t)((uint64_t)(n) >> 32))

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
	/* Point back to the device for work queues */
	const struct device *dev;
	/* Buffer of pointers to net_buf, with index matching the position in the TRB buffers */
	struct net_buf *net_buf[CONFIG_UDC_DWC3_TRB_NUM];
	/* Buffer of TRB structures, with index matching the position in the net_buf buffers */
	struct udc_dwc3_trb *trb_buf;
	/* Index of the next TRB to receive data in the TRB ring, Link TRB excluded */
	uint32_t head;
	uint32_t tail;
	/* Total size sent from the device to the host for the ongoing transfer */
	uint32_t total;
	/* A flag to tell when the ring buffer is full */
	bool full;
	/* Given by the hardware for use in endpoint commands */
	uint32_t xferrscidx;
	/* Diagnostics: number of DEPXFERCFG (transfer-resource allocations)
	 * issued for this EP since boot. Should be 1; >1 means a resource leak.
	 */
	uint32_t xfercfg_count;
	/* Linux-style DWC3_EP_RESOURCE_ALLOCATED: a transfer resource is
	 * allocated for this EP only once per DEPSTARTCFG(0) pool reset.
	 * Re-running DEPXFERCFG on EP re-enable would leak a resource and push
	 * the bulk-IN transfer resource index out of range for platform handoff.
	 */
	bool xfer_res_allocated;
	/* Edge detector for bulk-IN stall snapshot: capture controller state
	 * once at the first XferNotReady after (re)arm. Cleared on DepStartXfer.
	 */
	bool nrdy_reported;
#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
	/* Bulk-IN TRB ring is driven by the FPGA uvcmanager after handoff. */
	bool fpga_owned;
	/* True from HANDOFF until stream stop; gates idle watchdog + heartbeat. */
	bool fpga_streaming;
	/* Consecutive TRBSTS_INPROGRESS events at the same tail (underrun wedge). */
	uint8_t inprog_tail;
	uint8_t inprog_cnt;
#endif
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
	/* CONNECTDONE received since last bus reset */
	bool link_ready;
	/* EP0 needs DEPCFG INIT (not MODIFY) after USBRST */
	bool ep_reinit_after_reset;
	/* Stack bus-reset handler still running; block early SET_ADDRESS */
	bool bus_reset_recovering;
	/* Back-reference to parent */
	const struct device *dev;
	/* Per-physical-endpoint DEPEVT counters (session total, survives the
	 * 128-entry flight recorder ring). Index = physical EP number (epn<<1|dir).
	 * Confirms whether FPGA-owned (accelerated) endpoints raise any events.
	 */
	uint32_t depevt_count[32];
#ifdef CONFIG_UDC_DWC3_NRDY_PROBE_OUT
	/* Per-physical-EP XferNotReady counter (only meaningful for EPs that
	 * have XFERNRDYEN set, i.e. the ACM OUT EP under this probe). A non-zero
	 * value at a wedge means the host is knocking (it wants to transfer) and
	 * the device went NRDY without re-issuing ERDY: a SuperSpeed bulk
	 * flow-control deadlock. Also records the last NRDY event word.
	 */
	uint32_t depevt_nrdy_count[32];
	uint32_t depevt_nrdy_last[32];
#endif
#if CONFIG_UDC_DWC3_FLIGHT_RECORDER && CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS > 0
	/* Last CPU-handled bulk-IN XferComplete (stream progress heartbeat). */
	uint32_t last_bulk_in_complete_cyc;
	/* Last FPGA-managed bulk-IN completion (ignored by pop_trb). */
	uint32_t last_fpga_in_complete_cyc;
	/* When FPGA streaming last started (grace before STREAM-IDLE). */
	uint32_t last_fpga_handoff_cyc;
	struct k_work_delayable fr_watchdog_work;
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
	void (*post_soft_reset)(const struct device *const dev);
};

/* Helper for accessing vendor quirks */
#define UDC_DWC3_QUIRK_CFG(dev)  (((const struct udc_dwc3_config *)(dev->config))->quirk_config)
#define UDC_DWC3_QUIRK_DATA(dev) (((const struct udc_dwc3_config *)(dev->config))->quirk_data)

#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
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

static inline void udc_dwc3_quirk_post_soft_reset(const struct device *const dev)
{
	if (udc_dwc3_vendor_quirks.post_soft_reset != NULL) {
		udc_dwc3_vendor_quirks.post_soft_reset(dev);
	}
}

#define DEV_CFG(dev) ((const struct udc_dwc3_config *)(dev->config))
#define DEV_DATA(dev) ((struct udc_dwc3_data *)udc_get_private(dev))

static void udc_dwc3_next_ctrl(const struct device *const dev,
			       struct udc_dwc3_ep_data *const ep_data);

/* Optional hooks for platform-specific USB video offload. */
void udc_dwc3_vendor_bus_reset(const struct device *const dev)
	__attribute__((weak));

void udc_dwc3_vendor_ep_clear_halt(const struct device *const dev, const uint8_t ep_addr)
	__attribute__((weak));

/* Called once at the first bulk-IN XferNotReady after (re)arm so platform
 * code can snapshot accelerator state at the stall instant.
 */
void udc_dwc3_vendor_bulk_stall(const struct device *const dev, const uint8_t ep_addr)
	__attribute__((weak));

void udc_dwc3_vendor_failure_capture(const struct device *const dev,
				     const char *const reason)
	__attribute__((weak));

bool udc_dwc3_vendor_fpga_frames_active(void)
	__attribute__((weak));

void udc_dwc3_vendor_fpga_handoff(const struct device *const dev, uint8_t ep_addr)
	__attribute__((weak));

void udc_dwc3_vendor_ep_recovery(const struct device *const dev, uint8_t ep_addr)
	__attribute__((weak));

void udc_bus_reset_recovery_done(const struct device *const dev);

static int udc_dwc3_apply_address(const struct device *const dev, const uint8_t addr,
				  bool ep_reconfigure);
static int udc_dwc3_set_address(const struct device *const dev, const uint8_t addr);

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
 * Ring buffer
 *
 * Helpers to operate the TRB and event ring buffers, shared with the hardware.
 */

/* Increment the counter position "nump" of the TRBs/event ring buffer) */
void udc_dwc3_ring_inc(uint32_t *const nump, const uint32_t size)
{
	uint32_t num = *nump + 1;

	*nump = (num >= size) ? 0 : num;
}

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

	LOG_DBG("PUSH %u buf %p, data %p, size %u",
		ep_data->head, (void *)buf, (void *)buf->data, buf->size);

	/* Shift the head */
	udc_dwc3_ring_inc(&ep_data->head, CONFIG_UDC_DWC3_TRB_NUM - 1);

	/* If the head touches the tail after we add something, we are full */
	ep_data->full = (ep_data->head == ep_data->tail);
}

static void udc_dwc3_trb_ring_reset(struct udc_dwc3_ep_data *const ep_data)
{
	for (int i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
		ep_data->net_buf[i] = NULL;
	}

	ep_data->head = 0;
	ep_data->tail = 0;
	ep_data->total = 0;
	ep_data->full = false;
}

static struct net_buf *udc_dwc3_pop_trb(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	struct net_buf *const buf = ep_data->net_buf[ep_data->tail];

	if (buf == NULL) {
		if (ep_data->head == ep_data->tail) {
			LOG_DBG("pop: stray completion ep=0x%02x tail=%u",
				ep_data->cfg.addr, ep_data->tail);
		} else {
			LOG_ERR("pop: the next TRB is emtpy ep=0x%02x head=%u tail=%u",
				ep_data->cfg.addr, ep_data->head, ep_data->tail);
		}
		return NULL;
	}

	/* Clear the last TRB */
	ep_data->net_buf[ep_data->tail] = NULL;

	/* Move to the next position in the ring buffer */
	udc_dwc3_ring_inc(&ep_data->tail, CONFIG_UDC_DWC3_TRB_NUM - 1);

	LOG_DBG("POP %u EP 0x%02x, buf %p, data %p",
		ep_data->tail, ep_data->cfg.addr, (void *)buf, (void *)buf->data);

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

static uint32_t udc_dwc3_depcmd(const struct device *const dev,
				const uint32_t addr, const uint32_t cmd)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	sys_write32(cmd | UDC_DWC3_DEPCMD_CMDACT, base + addr);
	{
		/* Bound the CMDACT poll. The DWC3 normally clears CMDACT within
		 * microseconds, but a command issued while the SuperSpeed link
		 * is unstable (Recovery/SS.Inactive) can leave CMDACT set far
		 * longer. This runs in the USB IRQ, so an unbounded spin wedges
		 * all event processing and can freeze streaming.
		 */
		const uint32_t start = k_cycle_get_32();
		const uint32_t hz = sys_clock_hw_cycles_per_sec();
		const uint32_t limit = hz / 100U; /* ~10 ms */
		uint32_t spun;

		do {
			reg = sys_read32(base + addr);
			spun = k_cycle_get_32() - start;
		} while ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0 && spun < limit);

		if (reg & UDC_DWC3_DEPCMD_CMDACT) {
			LOG_ERR("DEPCMD CMDACT stuck: ep_reg_off=0x%lx cmdtype=0x%x "
				"cmd=0x%08x reg=0x%08x spun=%uus",
				(unsigned long)addr, (unsigned int)(cmd & 0xfU), cmd, reg,
				(unsigned int)(((uint64_t)spun * 1000000U) / hz));
			return 0;
		}
	}

	switch (reg & UDC_DWC3_DEPCMD_STATUS_MASK) {
	case UDC_DWC3_DEPCMD_STATUS_OK:
		break;
	case UDC_DWC3_DEPCMD_STATUS_CMDERR:
		/* Log endpoint and command on failure for post-mortem debug. */
		LOG_ERR("DEPCMD failed: ep_reg_off=0x%lx cmdtype=0x%x cmd=0x%08x reg=0x%08x",
			(unsigned long)addr, (unsigned int)(cmd & 0xfU), cmd, reg);
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

	struct udc_dwc3_data *const priv = udc_get_private(dev);
	bool force_init = priv->ep_reinit_after_reset &&
		(ep_data->cfg.addr == USB_CONTROL_EP_IN ||
		 ep_data->cfg.addr == USB_CONTROL_EP_OUT);

	LOG_INF("configuring endpoint 0x%02x with wMaxPacketSize=%u",
		ep_data->cfg.addr, ep_data->cfg.mps);

	if (ep_data->cfg.stat.enabled && !force_init) {
		param0 |= UDC_DWC3_DEPCMDPAR0_DEPCFG_ACTION_MODIFY;
	} else {
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

	/* Burst size (encoded as N-1). The high-bandwidth video IN endpoints
	 * (the top CONFIG_UDC_DWC3_NUM_VIDEO_IN_EPS) and the low-bandwidth ones
	 * (CDC ACM bulk, CDC interrupt) get separately tunable bursts. A
	 * perpetually-ready 16-packet video burst can monopolise the controller
	 * TX datapath and starve a concurrent CPU bulk-IN; lowering the video
	 * burst and/or the ACM burst lets ACM interleave. Defaults (15/15)
	 * preserve the original behaviour.
	 */
	{
		/* Default low (single packet). Only the high-bandwidth video IN
		 * endpoints get the deep burst. This must match the bMaxBurst the
		 * class advertises in its SS endpoint companion descriptor: UVC
		 * uses 15, CDC ACM bulk/interrupt and all OUT endpoints use 0. A
		 * BRSTSIZ that exceeds the host-negotiated bMaxBurst makes the
		 * endpoint depend on SS burst resync, which wedges under concurrent
		 * FPGA video streaming.
		 */
		uint32_t brstsiz = CONFIG_UDC_DWC3_LOWBW_EP_BURST;

		if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
			const struct udc_dwc3_config *const ccfg = dev->config;
			uint32_t fifonum = ep_data->cfg.addr & 0x7f;
			uint32_t first_video = (uint32_t)ccfg->num_in_eps -
					       CONFIG_UDC_DWC3_NUM_VIDEO_IN_EPS;

			if (fifonum >= first_video) {
				brstsiz = CONFIG_UDC_DWC3_VIDEO_EP_BURST;
			}
		}
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, brstsiz);
	}

	/* Set the FIFO number, must be 0 for all OUT EPs */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_FIFONUM_MASK,
				     ep_data->cfg.addr & 0x7f);
	}

	/* Per-endpoint events. An FPGA-owned (hardware-accelerated) endpoint is
	 * driven entirely by the uvcmanager via the doorbell after handoff; the
	 * CPU never services its TRB ring, so the controller must not raise
	 * XferComplete/XferInProgress for it. Leaving these enabled makes the
	 * controller generate (and internally track) completion events for the
	 * FPGA transfers that nothing consumes, which interferes with servicing
	 * the concurrent CPU bulk endpoints (CDC ACM).
	 */
#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23) && defined(CONFIG_UDC_DWC3_MASK_FPGA_EP_EVENTS)
	if (!ep_data->fpga_owned) {
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN;
	}
#else
	param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
	param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN;
#endif
	/* Do NOT enable XFERNRDYEN on bulk-IN EPs. The known-good usb23 driver
	 * left this commented out; turning it on for the flight recorder
	 * floods the 16-entry hardware event ring during a bulk-IN NRDY storm
	 * (762k+ EVNTOVERFLOW in one run), wedging the IRQ and drowning the
	 * console even with rate-limited logging.
	 *
	 * Probe: enable XferNotReady on bulk-OUT EPs only (the ACM OUT 0x01).
	 * OUT NRDY does not storm under the modest host write rate of the ACM
	 * stress test, and it lets us prove whether, at the wedge, the host is
	 * still asking to send (NRDY fires) while the device fails to re-ERDY.
	 */
#ifdef CONFIG_UDC_DWC3_NRDY_PROBE_OUT
	if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr) &&
	    USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN;
	}
#endif

	/* This is the usb protocol endpoint number, but the data encoding
	 * we chose for physical endpoint number is the same as this
	 * register
	 */
	param1 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR1_DEPCFG_EPNUMBER_MASK, ep_data->epn);

	sys_write32(param0, base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(param1, base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPCFG);

	/* Surface the TX FIFO RAM region this IN EP was assigned. Overlapping
	 * start/depth across IN EPs lets a high-bandwidth EP (FPGA video on
	 * 0x84/FIFO4) corrupt another EP's bulk FIFO (ACM on 0x82/FIFO2).
	 */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		uint32_t fifonum = ep_data->cfg.addr & 0x7f;
		uint32_t txf = sys_read32(base + UDC_DWC3_GTXFIFOSIZ(fifonum));

		LOG_INF("ep 0x%02x -> TXFIFO%u start=%u depth=%u (raw=0x%08x)",
			ep_data->cfg.addr, fifonum,
			(unsigned int)FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFSTADDR_MASK, txf),
			(unsigned int)FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK, txf), txf);
	}
}

static void udc_dwc3_depcmd_ep_xfer_config(const struct device *const dev,
					   struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	/* Allocate a transfer resource only once per EP (until the next
	 * DEPSTARTCFG(0) pool reset), matching the Linux dwc3 driver's
	 * DWC3_EP_RESOURCE_ALLOCATED gate. Each DEPXFERCFG (NumXferRes=1)
	 * consumes the next slot from the pool; re-running it on EP re-enable
	 * (every UVC stream restart re-enables the bulk-IN EP) leaks a resource
	 * and advances the pool, pushing the bulk-IN rscidx out of the range
	 * expected by platform handoff.
	 */
	if (ep_data->xfer_res_allocated) {
		LOG_INF("DepXferConfig: ep=0x%02x epn=%u SKIP (already allocated)",
			ep_data->cfg.addr, ep_data->epn);
		return;
	}

	ep_data->xfercfg_count++;
	LOG_INF("DepXferConfig: ep=0x%02x epn=%u alloc#%u",
		ep_data->cfg.addr, ep_data->epn, ep_data->xfercfg_count);

	reg = FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK, 1);
	sys_write32(reg, base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPXFERCFG);

	ep_data->xfer_res_allocated = true;
}

static void udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags);

static void __maybe_unused udc_dwc3_ep0_release_xfer(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	/* Release a dangling EP0 transfer resource left over from a transfer
	 * that was aborted by a USB bus reset. A DEPCFG INIT issued while the
	 * endpoint still owns a transfer resource returns CMDERR ("endpoint
	 * command failed"); on rapid (Windows-style) reset storms this
	 * eventually wedges re-enumeration. This mirrors the proven
	 * lattice_usb23_bulk_restart_xfer() teardown and is safe here because
	 * ep0_reconfigure() runs at set_address(0), after the controller is out
	 * of reset (DEPENDXFER must not be issued during USBRST).
	 */
	if (ep_data->xferrscidx == 0) {
		return;
	}

	udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	ep_data->xferrscidx = 0;
	udc_ep_set_busy(&ep_data->cfg, false);
}

static void udc_dwc3_ep0_reconfigure(const struct device *const dev, const bool final)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	if (!priv->ep_reinit_after_reset) {
		return;
	}

#if CONFIG_UDC_DWC3_EP0_RELEASE_XFER
	udc_dwc3_ep0_release_xfer(dev, &cfg->ep_data_in[0]);
	udc_dwc3_ep0_release_xfer(dev, &cfg->ep_data_out[0]);
#endif

	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0]);
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_out[0]);

	/* DEPXFERCFG (transfer-resource allocation) is intentionally NOT redone
	 * here. The EP0 IN/OUT transfer resources are allocated once in
	 * udc_dwc3_ep_enable() at power-on and are preserved across USB bus
	 * resets. Re-running DEPXFERCFG on every reset allocates a fresh
	 * transfer resource each time without freeing the previous one, leaking
	 * one resource per reset. On hosts that issue many resets during
	 * enumeration (e.g. some Windows laptops) the pool is exhausted and the
	 * controller starts returning CMDERR ("endpoint command failed").
	 * This mirrors the Linux dwc3 driver, which gates set_xfer_resource()
	 * behind a one-time DWC3_EP_RESOURCE_ALLOCATED flag.
	 */

	if (final) {
		priv->ep_reinit_after_reset = false;
	}
}

static void udc_dwc3_depcmd_set_stall(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	LOG_WRN("DepSetStall: ep=0x%02x", ep_data->cfg.addr);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPSETSTALL);
}

static void udc_dwc3_depcmd_clear_stall(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	LOG_INF("DepClearStall ep=0x%02x", ep_data->cfg.addr);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPCSTALL);
}

static void udc_dwc3_depcmd_start_xfer(const struct device *const dev,
				       struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

#if CONFIG_UDC_DWC3_REMOTEWAKEUP_GUARD
	/* If the SuperSpeed link is in a low-power state (U1/U2/U3), request a
	 * transition back to U0 (remote wakeup) before starting the transfer.
	 *
	 * This DCTL.ULSTCHNGREQ write MUST be gated on the link actually being
	 * in a low-power state. Issuing the request while the link is already in
	 * U0 (the normal case during active streaming) drives the LTSSM into
	 * Recovery and, on this PHY, into SS.Inactive, killing the link the
	 * instant a stream's first transfer is started. This matches the Linux
	 * dwc3 driver, which only issues remote wakeup from U1/U2/U3.
	 */
	reg = sys_read32(base + UDC_DWC3_DSTS);
	if ((reg & UDC_DWC3_DSTS_CONNECTSPD_MASK) == UDC_DWC3_DSTS_CONNECTSPD_SS) {
		uint32_t lnkst = reg & UDC_DWC3_DSTS_USBLNKST_MASK;

		if (lnkst == UDC_DWC3_DSTS_USBLNKST_USB3_U1 ||
		    lnkst == UDC_DWC3_DSTS_USBLNKST_USB3_U2 ||
		    lnkst == UDC_DWC3_DSTS_USBLNKST_USB3_U3) {
			reg = sys_read32(base + UDC_DWC3_DCTL);
			reg &= ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;
			reg |= UDC_DWC3_DCTL_ULSTCHNGREQ_REMOTEWAKEUP;
			sys_write32(reg, base + UDC_DWC3_DCTL);

			/* Wait for the link to actually reach U0 before issuing
			 * DEPSTRTXFER. Issuing a transfer command while the link
			 * is still transitioning out of a low-power state is a
			 * suspected SS.Inactive trigger on this PHY.
			 */
			for (int i = 0; i < 1000; i++) {
				lnkst = sys_read32(base + UDC_DWC3_DSTS) &
					UDC_DWC3_DSTS_USBLNKST_MASK;
				if (lnkst == UDC_DWC3_DSTS_USBLNKST_USB3_U0) {
					break;
				}
				k_busy_wait(1);
			}
		}
	}
#else
	/* Known-good custom-driver behaviour: unconditionally request remote
	 * wakeup on every DepStartXfer, assuming the TX FIFO is empty.
	 */
	reg = sys_read32(base + UDC_DWC3_DCTL);
	reg &= ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;
	reg |= UDC_DWC3_DCTL_ULSTCHNGREQ_REMOTEWAKEUP;
	sys_write32(reg, base + UDC_DWC3_DCTL);
#endif /* CONFIG_UDC_DWC3_REMOTEWAKEUP_GUARD */

	sys_write32(HI32((uintptr_t)ep_data->trb_buf), base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(LO32((uintptr_t)ep_data->trb_buf), base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	ep_data->xferrscidx =
		udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPSTRTXFER);

	reg = sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn));
	if ((reg & UDC_DWC3_DEPCMD_STATUS_MASK) != UDC_DWC3_DEPCMD_STATUS_OK) {
		udc_ep_set_busy(&ep_data->cfg, false);
	}

	/* Log bulk stream-arm for handoff debug. */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) != 0) {
		/* Re-arm: re-prime the stall edge detector so the next
		 * XferNotReady captures a fresh snapshot.
		 */
		ep_data->nrdy_reported = false;
		LOG_DBG("ARM bulk ep=0x%02x epn=%u xferrscidx=%u depcmd_status=0x%x trb=%p trbctl=0x%08x",
			ep_data->cfg.addr, ep_data->epn, ep_data->xferrscidx,
			(unsigned int)FIELD_GET(UDC_DWC3_DEPCMD_STATUS_MASK, reg),
			(void *)ep_data->trb_buf, ep_data->trb_buf[0].ctrl);
	}

	LOG_DBG("DepStartXfer done ep=0x%02x xferrscidx=0x%x",
		ep_data->cfg.addr, ep_data->xferrscidx);
}

static void udc_dwc3_depcmd_update_xfer(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	uint32_t flags = 0;

	flags |= UDC_DWC3_DEPCMD_DEPUPDXFER;
	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	LOG_DBG("DepUpdateXfer done ep=0x%02x addr=0x%08x data=0x%08x",
		ep_data->cfg.addr, UDC_DWC3_DEPCMD(ep_data->epn), flags);
}

static void udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags)
{
	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);
	flags |= UDC_DWC3_DEPCMD_DEPENDXFER;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	LOG_DBG("DepEndXfer done ep=0x%02x", ep_data->cfg.addr);

	udc_dwc3_trb_ring_reset(ep_data);
}

static void udc_dwc3_depcmd_start_config(const struct device *const dev,
					 struct udc_dwc3_ep_data *const ep_data)
{
	const bool is_control = USB_EP_GET_IDX(ep_data->cfg.addr) == 0;
	uint32_t flags = 0;

	/* DEPSTARTCFG XferRscIdx: 0 for the control endpoint (this also resets
	 * the whole transfer-resource pool after a USB reset, as required by the
	 * DWC3 databook), 2 for non-control endpoints (after the two EP0
	 * resources). The previous (idx > 0) test inverted this, leaving the
	 * resource pool uninitialised and corrupting transfers under heavy
	 * streaming load.
	 */
	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, is_control ? 0 : 2);
	flags |= UDC_DWC3_DEPCMD_DEPSTARTCFG;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	/* DEPSTARTCFG with XferRscIdx=0 frees the entire transfer-resource
	 * pool, so every EP must re-allocate its resource on the next
	 * DEPXFERCFG. Drop the "already allocated" gate for all EPs.
	 */
	if (is_control) {
		const struct udc_dwc3_config *const cfg = dev->config;

		for (int i = 0; i < cfg->num_in_eps; i++) {
			cfg->ep_data_in[i].xfer_res_allocated = false;
		}
		for (int i = 0; i < cfg->num_out_eps; i++) {
			cfg->ep_data_out[i].xfer_res_allocated = false;
		}
	}

	LOG_DBG("DepStartConfig done ep=0x%02x", ep_data->cfg.addr);
}

/*
 * Transfer Requests (TRB)
 *
 * DWC3 receives transfer requests from this driver through a shared memory
 * buffer, resubmitted upon every new transfer (through either Start or
 * Update command).
 */

static void udc_dwc3_trb_norm_init(const struct device *const dev,
				   struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;
	const uint32_t i = CONFIG_UDC_DWC3_TRB_NUM - 1;

	LOG_DBG("Initializing normal TRB");

	/* TRB0 that prevents the transfer to be started (until it is overwritten) */
	trb[0].ctrl = 0;

	/* TRB LINK that loops the ring buffer back to the beginning */
	trb[i].ctrl = UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB | UDC_DWC3_TRB_CTRL_HWO;
	trb[i].addr_lo = LO32((uintptr_t)ep_data->trb_buf);
	trb[i].addr_hi = HI32((uintptr_t)ep_data->trb_buf);

	/* Start the transfer now, update it later */
	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

static void udc_dwc3_trb_ctrl_out(const struct device *const dev,
				  struct net_buf *const buf,
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

	LOG_DBG("TRB_BULK_EP_0x%02x, buf %p, data %p, size %u, len %u",
		ep_data->cfg.addr, (void *)buf, (void *)buf->data, buf->size, buf->len);

	if (ep_data->full) {
		return -EBUSY;
	}

	if (udc_ep_buf_has_zlp(buf)) {
		LOG_DBG("Buffer has a ZLP flag, terminating the transfer");
		ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL_ZLP;
		ep_data->total = 0;
	} else {
		ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL;
		ep_data->total += buf->len;

		if (USB_EP_DIR_IS_IN(ep_data->cfg.addr) &&
		    ep_data->total % ep_data->cfg.mps == 0) {
			LOG_DBG("Buffer is a multiple of %d, continuing this transfer of %u bytes",
				ep_data->cfg.mps, ep_data->total);
			ctrl |= UDC_DWC3_TRB_CTRL_CHN;
		} else {
			LOG_DBG("End of USB transfer, %u bytes transferred", ep_data->total);
			ep_data->total = 0;
		}
	}

	udc_dwc3_push_trb(dev, ep_data, buf, ctrl);
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
		LOG_DBG("TRB_CONTROL_IN_DATA len=%d buf=%p", buf->len, buf);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi->status && setup->wLength == 0) {
		buf->size = 0;
		LOG_DBG("TRB_CONTROL_IN_STATUS_2 len=%d buf=%p", buf->len, buf);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
	} else if (bi->status) {
		buf->size = 0;
		LOG_DBG("TRB_CONTROL_IN_STATUS_3 len=%d buf=%p", buf->len, buf);
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
		buf->size = MIN(buf->size, sizeof(struct usb_setup_packet));
		LOG_DBG("TRB_CONTROL_OUT_SETUP size=%d buf=%p", buf->size, buf);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);
	} else if (bi->data) {
		LOG_DBG("TRB_CONTROL_OUT_DATA size=%d buf=%p", buf->size, buf);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi->status) {
		buf->size = 0;
		LOG_DBG("TRB_CONTROL_OUT_STATUS_3 size=%d buf=%p", buf->size, buf);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
	} else {
		LOG_ERR("Unknown buffer OUT, size=%d buf=%p", buf->size, buf);
		udc_submit_ep_event(dev, buf, -EINVAL);
	}
}

static void udc_dwc3_next_ctrl(const struct device *const dev,
			       struct udc_dwc3_ep_data *const ep_data)
{
	struct net_buf *buf;

	if (udc_ep_is_busy(&ep_data->cfg)) {
		return;
	}

	buf = udc_buf_peek(&ep_data->cfg);
	if (buf == NULL) {
		return;
	}

	udc_ep_set_busy(&ep_data->cfg, true);

	/* In DWC3 */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		udc_dwc3_next_ctrl_in(dev, buf);
	} else {
		udc_dwc3_next_ctrl_out(dev, buf);
	}
}

static void udc_dwc3_ctrl_setup_rearm(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep0_out = &cfg->ep_data_out[0];
	struct net_buf *buf;

	if (udc_ep_is_busy(&ep0_out->cfg)) {
		return;
	}

	buf = udc_buf_peek(&ep0_out->cfg);
	if (buf != NULL && udc_get_buf_info(buf)->setup) {
		net_buf_reset(buf);
		udc_dwc3_next_ctrl(dev, ep0_out);
	}
}

static void udc_dwc3_finish_address_zero(const struct device *const dev)
{
	udc_dwc3_ep0_reconfigure(dev, true);
	udc_dwc3_ctrl_setup_rearm(dev);
}

static void udc_dwc3_ep_reset_state(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	if (!ep_data->cfg.stat.enabled && !udc_ep_is_busy(&ep_data->cfg)) {
		return;
	}

	LOG_INF("bus reset: ep 0x%02x busy=%u en=%u halted=%u",
		ep_data->cfg.addr, udc_ep_is_busy(&ep_data->cfg),
		ep_data->cfg.stat.enabled, ep_data->cfg.stat.halted);

	k_work_cancel(&ep_data->work);
	udc_ep_cancel_queued(dev, &ep_data->cfg);
	udc_ep_set_busy(&ep_data->cfg, false);
	ep_data->cfg.stat.halted = false;
	/* Leave stat.enabled for the stack: usbd_config_set(0) disables
	 * endpoints and clears ep_active. Clearing enabled here desyncs
	 * the driver from the stack and makes ep_disable return -EALREADY.
	 */
	udc_dwc3_trb_ring_reset(ep_data);
	ep_data->xferrscidx = 0;
#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
	ep_data->fpga_owned = false;
	ep_data->fpga_streaming = false;
#endif

	sys_clear_bit(base + UDC_DWC3_DALEPENA, ep_data->epn);
}

static void udc_dwc3_reset_noncontrol_eps(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	for (int epn = 1; epn < cfg->num_in_eps; epn++) {
		udc_dwc3_ep_reset_state(dev, &cfg->ep_data_in[epn]);
	}

	for (int epn = 1; epn < cfg->num_out_eps; epn++) {
		udc_dwc3_ep_reset_state(dev, &cfg->ep_data_out[epn]);
	}
}

/*
 * Events
 *
 * Process the events from the event ring buffer. Interrupts gives us a
 * hint that an event is available, which we fetch from a ring buffer shared
 * with the hardware.
 */

#if CONFIG_UDC_DWC3_RESIZE_TX_FIFOS
/*
 * A high-bandwidth (video) IN endpoint is one of the top
 * CONFIG_UDC_DWC3_NUM_VIDEO_IN_EPS bulk-IN FIFOs. These carry the FPGA video
 * stream and get the large burst + large FIFO. All lower-numbered IN
 * endpoints (CDC ACM bulk, CDC interrupt) are low-bandwidth: a 1-packet burst
 * and a small FIFO are plenty, and starving them of RAM is what frees enough
 * for the video EP to own a non-overlapping region.
 */
static bool udc_dwc3_ep_in_is_video(const struct device *const dev, uint32_t fifonum)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const uint32_t first_video = (uint32_t)cfg->num_in_eps - CONFIG_UDC_DWC3_NUM_VIDEO_IN_EPS;

	return fifonum >= first_video;
}

/*
 * Give each IN endpoint a non-overlapping TX FIFO RAM region.
 *
 * FIFO0 (EP0) is left as the IP configured it; the remaining RAM1 words are
 * handed out in FIFO-number order: low-bandwidth EPs take a fixed small slice,
 * the high-bandwidth video EPs split whatever remains.
 */
static void udc_dwc3_resize_tx_fifos(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t ram1 = FIELD_GET(UDC_DWC3_GHWPARAMS7_RAM1_DEPTH_MASK,
					sys_read32(base + UDC_DWC3_GHWPARAMS7));
	const uint32_t f0 = sys_read32(base + UDC_DWC3_GTXFIFOSIZ(0));
	const uint32_t f0_start = FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFSTADDR_MASK, f0);
	const uint32_t f0_depth = FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK, f0);
	const int n_in = cfg->num_in_eps - 1; /* data IN FIFOs: 1..num_in_eps-1 */
	const uint32_t lowbw_depth = CONFIG_UDC_DWC3_LOWBW_FIFO_DEPTH;
	const int n_video = CONFIG_UDC_DWC3_NUM_VIDEO_IN_EPS;
	uint32_t start = f0_start + f0_depth;
	int n_lowbw;
	uint32_t video_total;
	uint32_t video_unit;

	if (ram1 == 0U || n_in <= 0 || start >= ram1) {
		LOG_WRN("TX FIFO resize skipped: ram1=%u f0_start=%u f0_depth=%u n_in=%d",
			ram1, f0_start, f0_depth, n_in);
		return;
	}

	n_lowbw = n_in - n_video;
	if (n_lowbw < 0) {
		n_lowbw = 0;
	}

	if (start + (uint32_t)n_lowbw * lowbw_depth >= ram1) {
		LOG_WRN("TX FIFO resize skipped: RAM too small (ram1=%u, lowbw needs %u)",
			ram1, (uint32_t)n_lowbw * lowbw_depth);
		return;
	}

	video_total = ram1 - start - (uint32_t)n_lowbw * lowbw_depth;
	video_unit = (n_video > 0) ? (video_total / (uint32_t)n_video) : 0U;

	LOG_INF("TX FIFO resize: ram1=%u f0=[%u,%u] lowbw=%u x%d video=%u x%d",
		ram1, f0_start, f0_depth, lowbw_depth, n_lowbw, video_unit, n_video);

	for (int fifonum = 1; fifonum <= n_in; fifonum++) {
		bool is_video = udc_dwc3_ep_in_is_video(dev, fifonum);
		uint32_t depth = is_video ? video_unit : lowbw_depth;
		uint32_t val;

		/* Last endpoint absorbs any rounding remainder. */
		if (fifonum == n_in) {
			depth = ram1 - start;
		}
		if (start + depth > ram1) {
			depth = ram1 - start;
		}

		val = FIELD_PREP(UDC_DWC3_GTXFIFOSIZ_TXFSTADDR_MASK, start) |
		      FIELD_PREP(UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK, depth);
		sys_write32(val, base + UDC_DWC3_GTXFIFOSIZ(fifonum));

		LOG_INF("  GTXFIFOSIZ(%d) start=%u depth=%u %s (raw=0x%08x)",
			fifonum, start, depth, is_video ? "video" : "lowbw", val);

		start += depth;
	}
}
#endif /* CONFIG_UDC_DWC3_RESIZE_TX_FIFOS */

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

	/* TX packet-count threshold to reduce bulk-IN transmit underrun on
	 * SuperSpeed. With threshold disabled, DWC3 may start a burst before
	 * enough data is buffered, yielding truncated packets (-71 / EPROTO).
	 * Values restored from the pre-2026-05-12 custom tinyvision driver.
	 */
	{
		const uint32_t tx_thr_num = 1;
		const uint32_t tx_max_burst = 2;

		reg = UDC_DWC3_GTXTHRCFG_USBTXPKTCNTSEL;
		reg |= FIELD_PREP(UDC_DWC3_GTXTHRCFG_USBTXPKTCNT_MASK, tx_thr_num);
		reg |= FIELD_PREP(UDC_DWC3_GTXTHRCFG_USBMAXTXBURSTSIZE_MASK, tx_max_burst);
		sys_write32(reg, base + UDC_DWC3_GTXTHRCFG);
		LOG_INF("GTXTHRCFG=0x%08x (tx_thr_num=%u tx_max_burst=%u)",
			sys_read32(base + UDC_DWC3_GTXTHRCFG), tx_thr_num, tx_max_burst);
	}

	/* Read the chip identification */
	reg = sys_read32(base + UDC_DWC3_GCOREID);
	LOG_INF("event: coreid=0x%04lx rel=0x%04lx",
		FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg),
		FIELD_GET(UDC_DWC3_GCOREID_REL_MASK, reg));
	__ASSERT_NO_MSG(FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg) == 0x5533);

	/* Letting GUID unchanged */
	/* Letting GUSB2PHYCFG and GUSB3PIPECTL unchanged */
	/* Letting GRXFIFOSIZ unchanged */

#if CONFIG_UDC_DWC3_RESIZE_TX_FIFOS
	udc_dwc3_resize_tx_fifos(dev);
#endif

	/* Setup the event buffer address, size and start event reception */
	memset((void *)cfg->evt_buf, 0, CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t));
	sys_write32(HI32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_HI(0));
	sys_write32(LO32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_LO(0));
	sys_write32(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t), base + UDC_DWC3_GEVNTSIZ(0));
	LOG_INF("Event buffer size is %u bytes", sys_read32(base + UDC_DWC3_GEVNTSIZ(0)));
	sys_write32(0, base + UDC_DWC3_GEVNTCOUNT(0));

	/* Letting GCTL unchanged */

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

#if CONFIG_UDC_DWC3_DISABLE_LPM
	/* Pin the SuperSpeed link in U0 (disable U1/U2/U3). Matches the original
	 * custom driver; video offload on this platform assumes a live transfer.
	 */
	reg = sys_read32(base + UDC_DWC3_DCTL);
	reg &= ~(UDC_DWC3_DCTL_INITU1ENA | UDC_DWC3_DCTL_ACCEPTU1ENA |
		 UDC_DWC3_DCTL_INITU2ENA | UDC_DWC3_DCTL_ACCEPTU2ENA);
	sys_write32(reg, base + UDC_DWC3_DCTL);

	sys_clear_bits(base + UDC_DWC3_DCFG, UDC_DWC3_DCFG_LPMCAP);
	sys_clear_bits(base + UDC_DWC3_GUSB3PIPECTL, UDC_DWC3_GUSB3PIPECTL_SUSPENDENABLE);
	sys_clear_bits(base + UDC_DWC3_GUSB2PHYCFG, UDC_DWC3_GUSB2PHYCFG_ENBLSLPM);
	LOG_INF("LPM disabled: DCTL=0x%08x DCFG=0x%08x PIPECTL=0x%08x PHYCFG=0x%08x",
		sys_read32(base + UDC_DWC3_DCTL), sys_read32(base + UDC_DWC3_DCFG),
		sys_read32(base + UDC_DWC3_GUSB3PIPECTL), sys_read32(base + UDC_DWC3_GUSB2PHYCFG));
#endif /* CONFIG_UDC_DWC3_DISABLE_LPM */

	/* Enable reception of USB events. ULSTCNGEN is off by default to
	 * match the known-good usb23 driver, which explicitly excluded it;
	 * every ULSTCHNG -> U0 during streaming was flooding the event ring.
	 */
	reg = UDC_DWC3_DEVTEN_INACTTIMEOUTRCVEDEN;
	reg |= UDC_DWC3_DEVTEN_VNDRDEVTSTRCVEDEN;
	reg |= UDC_DWC3_DEVTEN_EVNTOVERFLOWEN;
	reg |= UDC_DWC3_DEVTEN_CMDCMPLTEN;
	reg |= UDC_DWC3_DEVTEN_ERRTICERREN;
	reg |= UDC_DWC3_DEVTEN_HIBERNATIONREQEVTEN;
	reg |= UDC_DWC3_DEVTEN_WKUPEVTEN;
#if CONFIG_UDC_DWC3_ULSTCNGEN
	reg |= UDC_DWC3_DEVTEN_ULSTCNGEN;
#endif
	reg |= UDC_DWC3_DEVTEN_CONNECTDONEEN;
	reg |= UDC_DWC3_DEVTEN_USBRSTEN;
	reg |= UDC_DWC3_DEVTEN_DISCONNEVTEN;
	sys_write32(reg, base + UDC_DWC3_DEVTEN);

	/* Configure endpoint 0x00 and 0x80 only for now */
	udc_dwc3_depcmd_start_config(dev, &cfg->ep_data_in[0]);
	udc_dwc3_depcmd_start_config(dev, &cfg->ep_data_out[0]);

	udc_dwc3_quirk_post_soft_reset(dev);
}

static bool udc_dwc3_collapse_captured;

#if CONFIG_UDC_DWC3_FLIGHT_RECORDER && CONFIG_UDC_DWC3_FR_CLEAR_HALT_STORM
struct udc_dwc3_clear_halt_storm {
	uint32_t window_start_cyc;
	uint8_t ep_addr;
	uint16_t count;
};

static struct udc_dwc3_clear_halt_storm udc_dwc3_clr_halt_storm;
#endif

#if CONFIG_UDC_DWC3_FLIGHT_RECORDER
/*
 * Flight recorder.
 *
 * Every controller event (raw 32-bit DEPEVT/DEVT word) is timestamped and
 * stored in a RAM ring on the IRQ hot path with NO logging, so it does not
 * generate extra events or flood the console during streaming. On a mid-stream
 * USB reset the ring is dumped, giving the exact sequence of events leading up
 * to the failure (e.g. an XferNotReady storm = TX FIFO underrun, or a DEPEVT
 * status with BUSERR set = controller/DMA fault) which is impossible to see
 * from a single post-mortem snapshot.
 */
struct udc_dwc3_fr_entry {
	uint32_t first_cyc; /* k_cycle_get_32() of first occurrence in this run */
	uint32_t last_cyc;  /* k_cycle_get_32() of most recent occurrence */
	uint32_t evt;       /* full, unmasked event word */
	uint32_t count;     /* number of consecutive identical events coalesced */
};

static struct udc_dwc3_fr_entry udc_dwc3_fr[CONFIG_UDC_DWC3_FLIGHT_RECORDER_NUM];
static uint32_t udc_dwc3_fr_head;  /* index of the most recently written entry */
static uint32_t udc_dwc3_fr_count; /* number of valid entries */
/* Once set, recording stops so the ring preserves the run-up to a failure
 * instead of being overwritten by the (potentially many seconds of) identical
 * storm that follows it. Frozen at the first bulk-IN stall edge.
 */
static bool udc_dwc3_fr_frozen;

/*
 * Coalesce runs of identical consecutive events. A streaming endpoint that
 * stalls produces thousands of identical XferNotReady events; without
 * coalescing they would evict the interesting transition (the last good
 * XferInProgress and whatever immediately preceded the stall) from the ring.
 * Collapsing a run into a single entry with a repeat count keeps every slot
 * meaningful.
 */
static inline void udc_dwc3_fr_freeze(void)
{
	udc_dwc3_fr_frozen = true;
}

static inline void udc_dwc3_fr_record(const uint32_t evt)
{
	const uint32_t now = k_cycle_get_32();
	struct udc_dwc3_fr_entry *cur = &udc_dwc3_fr[udc_dwc3_fr_head];
	const uint32_t devt_type = (evt >> 8) & 0xfU;

	if (udc_dwc3_fr_frozen) {
		return;
	}

	/* Never store overflow/errtic storms: they coalesce to one useless
	 * entry and burn IRQ cycles while the ring is already full.
	 */
	if (devt_type == 0xbU || devt_type == 0x9U) {
		return;
	}

	if (udc_dwc3_fr_count != 0U && cur->evt == evt) {
		cur->last_cyc = now;
		cur->count++;
		return;
	}

	udc_dwc3_fr_head = (udc_dwc3_fr_head + 1U) % ARRAY_SIZE(udc_dwc3_fr);
	cur = &udc_dwc3_fr[udc_dwc3_fr_head];
	cur->first_cyc = now;
	cur->last_cyc = now;
	cur->evt = evt;
	cur->count = 1U;
	if (udc_dwc3_fr_count < ARRAY_SIZE(udc_dwc3_fr)) {
		udc_dwc3_fr_count++;
	}
}

static const char *udc_dwc3_fr_depevt_name(const uint32_t type)
{
	switch (type) {
	case 0x01: return "XFERCOMPLETE";
	case 0x02: return "XFERINPROGRESS";
	case 0x03: return "XFERNOTREADY";
	case 0x04: return "RXTXFIFOEVT";
	case 0x06: return "STREAMEVT";
	case 0x07: return "EPCMDCMPLT";
	default:   return "DEPEVT?";
	}
}

static const char *udc_dwc3_fr_devt_name(const uint32_t type)
{
	switch (type) {
	case 0x0: return "DISCONNECT";
	case 0x1: return "USBRST";
	case 0x2: return "CONNECTDONE";
	case 0x3: return "ULSTCHNG";
	case 0x4: return "WKUP";
	case 0x6: return "SUSPEND";
	case 0x7: return "SOF";
	case 0x9: return "ERRTICERR";
	case 0xa: return "CMDCMPLT";
	case 0xb: return "EVNTOVERFLOW";
	case 0xc: return "VNDRDEVTSTRCVED";
	default:  return "DEVT?";
	}
}

/* SuperSpeed link state names, indexed by DSTS.USBLNKST / the ULSTCHNG event
 * information field (raw_event >> 16) & 0xf. U1/U2 = LPM low power; RECOV/
 * SS_INACT = link error path (SS_INACT is a hard failure needing re-enum).
 */
static const char *udc_dwc3_fr_linkstate_name(const uint32_t st)
{
	switch (st) {
	case 0x0: return "U0";
	case 0x1: return "U1";
	case 0x2: return "U2";
	case 0x3: return "U3";
	case 0x4: return "SS.Dis";
	case 0x5: return "RX.Det";
	case 0x6: return "SS.Inact";
	case 0x7: return "Poll";
	case 0x8: return "Recov";
	case 0x9: return "HReset";
	case 0xa: return "Cmply";
	case 0xb: return "Lpbk";
	case 0xf: return "ResetResume";
	default:  return "?";
	}
}

static void udc_dwc3_fr_dump(const char *const reason)
{
	const uint32_t n = udc_dwc3_fr_count;
	const uint32_t hz = sys_clock_hw_cycles_per_sec();
	/* Walk oldest -> newest. Newest is at fr_head; oldest is n-1 behind it. */
	uint32_t idx = (udc_dwc3_fr_head + ARRAY_SIZE(udc_dwc3_fr) + 1U - n) %
		       ARRAY_SIZE(udc_dwc3_fr);
	uint32_t prev_cyc = 0;

	LOG_ERR("FR[%s]: %u distinct event runs (oldest first), hz=%u", reason, n, hz);

	for (uint32_t i = 0; i < n; i++) {
		const struct udc_dwc3_fr_entry *const e = &udc_dwc3_fr[idx];
		const uint32_t evt = e->evt;
		/* gap (us) from the previous run's last event to this run's first */
		const uint32_t gap_us = (i == 0) ? 0U :
			(uint32_t)(((uint64_t)(e->first_cyc - prev_cyc) * 1000000U) / hz);
		/* span (us) covered by this run (first -> last occurrence) */
		const uint32_t span_us =
			(uint32_t)(((uint64_t)(e->last_cyc - e->first_cyc) * 1000000U) / hz);

		if (evt & BIT(0)) {
			/* Device event (DEVT): type in bits[11:8]. For ULSTCHNG
			 * the new link state is in the event-info field [20:16].
			 */
			const uint32_t devt = (evt >> 8) & 0xf;

			if (devt == 0x3) {
				LOG_ERR("FR %3u +%7uus x%-5u DEVT   ULSTCHNG -> %-11s raw=0x%08x",
					i, gap_us, e->count,
					udc_dwc3_fr_linkstate_name((evt >> 16) & 0xf), evt);
			} else {
				LOG_ERR("FR %3u +%7uus x%-5u DEVT   %-14s raw=0x%08x",
					i, gap_us, e->count,
					udc_dwc3_fr_devt_name(devt), evt);
			}
		} else {
			/* DEPEVT: epn[5:1] type[9:6] status[15:12] param[31:16] */
			const uint32_t epn = (evt >> 1) & 0x1f;
			const uint32_t type = (evt >> 6) & 0xf;
			const uint32_t status = (evt >> 12) & 0xf;
			/* BUSERR (status bit0) is only meaningful for transfer
			 * completion events; for XferNotReady bit3 = "transfer
			 * resource active", lower bits are the request reason.
			 */
			const bool is_xfer_done = (type == 0x01 || type == 0x02);
			const char *flag =
				(is_xfer_done && (status & UDC_DWC3_DEPEVT_STATUS_BUSERR))
					? " BUSERR"
				: (type == 0x03 && !(status & BIT(3))) ? " NOT_ACTIVE"
									: "";

			LOG_ERR("FR %3u +%7uus x%-5u DEPEVT phys_ep=%u %-14s status=0x%x%s "
				"span=%uus param=0x%04x",
				i, gap_us, e->count, epn,
				udc_dwc3_fr_depevt_name(type), status, flag,
				span_us, (evt >> 16) & 0xffff);
		}

		prev_cyc = e->last_cyc;
		idx = (idx + 1U) % ARRAY_SIZE(udc_dwc3_fr);
	}
}

#else
static inline void udc_dwc3_fr_record(const uint32_t evt) { ARG_UNUSED(evt); }
static inline void udc_dwc3_fr_dump(const char *const reason) { ARG_UNUSED(reason); }
static inline void udc_dwc3_fr_freeze(void) { }
#endif /* CONFIG_UDC_DWC3_FLIGHT_RECORDER */

/* Read one GDBGFIFOSPACE {queue type, queue num} AVAILABLE counter. */
static uint32_t udc_dwc3_dbg_queue_space(const mm_reg_t base, uint32_t qtype, uint32_t qnum)
{
	sys_write32(qtype | FIELD_PREP(UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK, qnum),
		    base + UDC_DWC3_GDBGFIFOSPACE);
	return FIELD_GET(UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK,
			sys_read32(base + UDC_DWC3_GDBGFIFOSPACE));
}

/* Dump the full software TRB ring for one endpoint (HWO = hardware still owns
 * it, i.e. the controller has not completed that descriptor).
 */
static void udc_dwc3_diag_dump_trb_ring(const char *const reason,
					struct udc_dwc3_ep_data *const ep_data)
{
	for (int i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
		volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[i];

		LOG_ERR("DIAG[%s]:   ep 0x%02x trb[%d] addr=0x%08x sts=0x%08x ctrl=0x%08x "
			"HWO=%u LST=%u IOC=%u",
			reason, ep_data->cfg.addr, i, trb->addr_lo, trb->status, trb->ctrl,
			(trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) ? 1 : 0,
			(trb->ctrl & UDC_DWC3_TRB_CTRL_LST) ? 1 : 0,
			(trb->ctrl & UDC_DWC3_TRB_CTRL_IOC) ? 1 : 0);
	}
}

/*
 * Failure-state capture for the bulk-IN streaming path.
 * Dumps controller link/FIFO/TRB state at bus reset during streaming.
 */
static void udc_dwc3_diag_snapshot(const struct device *const dev, const char *const reason)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t dsts = sys_read32(base + UDC_DWC3_DSTS);
	const uint32_t gsts = sys_read32(base + UDC_DWC3_GSTS);

	LOG_ERR("DIAG[%s]: DSTS=0x%08x linkst=0x%x spd=%u LTSSM=0x%08x GSTS=0x%08x",
		reason, dsts,
		(unsigned int)FIELD_GET(UDC_DWC3_DSTS_USBLNKST_MASK, dsts),
		(unsigned int)FIELD_GET(UDC_DWC3_DSTS_CONNECTSPD_MASK, dsts),
		sys_read32(base + UDC_DWC3_GDBGLTSSM), gsts);

	if (gsts & UDC_DWC3_GSTS_BUSERRADDRVLD) {
		LOG_ERR("DIAG[%s]: BUS_ERROR addr=0x%08x%08x", reason,
			sys_read32(base + UDC_DWC3_GBUSERRADDR_HI),
			sys_read32(base + UDC_DWC3_GBUSERRADDR_LO));
	}

	/* Controller transfer-engine queue depths (GDBGFIFOSPACE AVAILABLE).
	 * DESCFETCH per phys-ep shows whether the controller is still able to
	 * fetch that endpoint's TRB from memory; a stuck ACM EP with an empty
	 * TX FIFO but pending TRBs points at the descriptor-fetch / DMA engine
	 * being monopolised by the FPGA video transfer rather than a FIFO
	 * collision (the FIFO partition was confirmed non-overlapping).
	 */
	LOG_ERR("DIAG[%s]: QSPACE txreq=%u rxreq=%u protocol=%u wrevent=%u auxevent=%u", reason,
		udc_dwc3_dbg_queue_space(base, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TXREQ, 0),
		udc_dwc3_dbg_queue_space(base, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_RXREQ, 0),
		udc_dwc3_dbg_queue_space(base, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_PROTOCOL, 0),
		udc_dwc3_dbg_queue_space(base, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_WREVENT, 0),
		udc_dwc3_dbg_queue_space(base, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_AUXEVENT, 0));
	/* DESCFETCH for ACM OUT (phys2), ACM IN (phys5), video IN (phys9). */
	LOG_ERR("DIAG[%s]: DESCFETCH phys2(0x01)=%u phys5(0x82)=%u phys9(0x84)=%u", reason,
		udc_dwc3_dbg_queue_space(base, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCH, 2),
		udc_dwc3_dbg_queue_space(base, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCH, 5),
		udc_dwc3_dbg_queue_space(base, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_DESCFETCH, 9));
	/* Per-EP DEPEVT counters (session total). FPGA-owned EPs must read 0. */
	{
		struct udc_dwc3_data *const priv = udc_get_private(dev);

		for (int epn = 1; epn < cfg->num_in_eps; epn++) {
			struct udc_dwc3_ep_data *const ed = &cfg->ep_data_in[epn];
			uint32_t phys = (epn << 1) | 1;

			if (!ed->cfg.stat.enabled) {
				continue;
			}
			LOG_ERR("DIAG[%s]: EVTCNT ep 0x%02x phys%u depevt=%u fpga_owned=%u",
				reason, ed->cfg.addr, phys, priv->depevt_count[phys & 0x1f],
				ed->fpga_owned);
		}
#ifdef CONFIG_UDC_DWC3_NRDY_PROBE_OUT
		/* OUT EP XferNotReady probe. nrdy>0 at the wedge == host is still
		 * asking to send while the device never re-ERDYs (SS bulk hang).
		 */
		for (int epn = 1; epn < cfg->num_out_eps; epn++) {
			struct udc_dwc3_ep_data *const ed = &cfg->ep_data_out[epn];
			uint32_t phys = (epn << 1);

			if (!ed->cfg.stat.enabled) {
				continue;
			}
			LOG_ERR("DIAG[%s]: NRDY ep 0x%02x phys%u nrdy=%u last=0x%08x depevt=%u",
				reason, ed->cfg.addr, phys, priv->depevt_nrdy_count[phys & 0x1f],
				priv->depevt_nrdy_last[phys & 0x1f], priv->depevt_count[phys & 0x1f]);
		}
#endif
	}

	for (int epn = 1; epn < cfg->num_in_eps; epn++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[epn];
		volatile struct udc_dwc3_trb *trb;
		uint32_t txfifo;

		if (!ep_data->cfg.stat.enabled) {
			continue;
		}

		/* TX FIFO occupancy for this IN EP. GDBGFIFOSPACE must be primed
		 * with a {queue type, queue num} select before reading the
		 * AVAILABLE field; the bare read returns FIFO 0 (EP0), which is
		 * useless here. FIFONUM for an IN EP is (addr & 0x7f), matching
		 * what DEPCFG programs. AVAILABLE counts free MDWIDTH words; a
		 * value at/near the FIFO depth means the FIFO is empty.
		 */
		sys_write32(UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TX |
			    FIELD_PREP(UDC_DWC3_GDBGFIFOSPACE_QUEUENUM_MASK,
				       ep_data->cfg.addr & 0x7f),
			    base + UDC_DWC3_GDBGFIFOSPACE);
		txfifo = FIELD_GET(UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK,
				   sys_read32(base + UDC_DWC3_GDBGFIFOSPACE));

		trb = &ep_data->trb_buf[ep_data->tail];
		LOG_ERR("DIAG[%s]: ep 0x%02x IN busy=%u halted=%u rscidx=%u head=%u tail=%u "
			"trbsts=%u trbctl=0x%08x txfifo_avail=%u queued=%u"
#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
			" fpga_owned=%u fpga_streaming=%u"
#endif
			,
			reason, ep_data->cfg.addr, udc_ep_is_busy(&ep_data->cfg),
			ep_data->cfg.stat.halted, ep_data->xferrscidx,
			ep_data->head, ep_data->tail,
			(unsigned int)FIELD_GET(UDC_DWC3_TRB_STATUS_TRBSTS_MASK, trb->status),
			trb->ctrl, txfifo, udc_buf_peek(&ep_data->cfg) != NULL
#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
			, ep_data->fpga_owned, ep_data->fpga_streaming
#endif
			);
		if (!ep_data->fpga_owned) {
			udc_dwc3_diag_dump_trb_ring(reason, ep_data);
		}
	}

	for (int epn = 1; epn < cfg->num_out_eps; epn++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[epn];
		volatile struct udc_dwc3_trb *trb;

		if (!ep_data->cfg.stat.enabled) {
			continue;
		}

		trb = &ep_data->trb_buf[ep_data->tail];
		LOG_ERR("DIAG[%s]: ep 0x%02x OUT busy=%u halted=%u rscidx=%u head=%u tail=%u "
			"trbsts=%u trbctl=0x%08x queued=%u",
			reason, ep_data->cfg.addr, udc_ep_is_busy(&ep_data->cfg),
			ep_data->cfg.stat.halted, ep_data->xferrscidx,
			ep_data->head, ep_data->tail,
			(unsigned int)FIELD_GET(UDC_DWC3_TRB_STATUS_TRBSTS_MASK, trb->status),
			trb->ctrl, udc_buf_peek(&ep_data->cfg) != NULL);
		udc_dwc3_diag_dump_trb_ring(reason, ep_data);
	}
}

/* True while at least one bulk-IN endpoint has FPGA streaming active. */
#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
static bool udc_dwc3_any_fpga_streaming(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	for (int epn = 1; epn < cfg->num_in_eps; epn++) {
		if (cfg->ep_data_in[epn].fpga_streaming) {
			return true;
		}
	}

	return false;
}

static bool udc_dwc3_fpga_bringup_pending(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	for (int epn = 1; epn < cfg->num_in_eps; epn++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[epn];

		if (ep_data->fpga_owned && !ep_data->fpga_streaming) {
			return true;
		}
	}

	return false;
}
#endif

/* True while at least one bulk-IN (video) endpoint is streaming. */
static bool udc_dwc3_is_streaming(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	for (int epn = 1; epn < cfg->num_in_eps; epn++) {
		if (cfg->ep_data_in[epn].cfg.stat.enabled) {
			return true;
		}
	}

	return false;
}

#if CONFIG_UDC_DWC3_FLIGHT_RECORDER
static void udc_dwc3_failure_capture(const struct device *const dev,
				     const char *const reason)
{
	if (udc_dwc3_collapse_captured) {
		return;
	}

	udc_dwc3_collapse_captured = true;
	udc_dwc3_fr_freeze();
	LOG_ERR("USB failure capture: %s", reason);
	udc_dwc3_vendor_failure_capture(dev, reason);
	udc_dwc3_diag_snapshot(dev, reason);
	udc_dwc3_fr_dump(reason);
}

#if CONFIG_UDC_DWC3_FR_CLEAR_HALT_STORM
static void udc_dwc3_clear_halt_storm_reset(void)
{
	udc_dwc3_clr_halt_storm.window_start_cyc = 0;
	udc_dwc3_clr_halt_storm.ep_addr = 0;
	udc_dwc3_clr_halt_storm.count = 0;
}

static void udc_dwc3_clear_halt_storm_check(const struct device *const dev,
					    const uint8_t ep_addr)
{
	const uint32_t now = k_cycle_get_32();
	const uint32_t hz = sys_clock_hw_cycles_per_sec();
	const uint32_t window_cyc =
		((uint64_t)CONFIG_UDC_DWC3_FR_CLEAR_HALT_WINDOW_MS * hz) / 1000U;
	char reason[24];

	if (!udc_dwc3_is_streaming(dev)) {
		return;
	}

	if (udc_dwc3_clr_halt_storm.count != 0U &&
	    (now - udc_dwc3_clr_halt_storm.window_start_cyc) > window_cyc) {
		udc_dwc3_clear_halt_storm_reset();
	}

	if (udc_dwc3_clr_halt_storm.count == 0U ||
	    udc_dwc3_clr_halt_storm.ep_addr != ep_addr) {
		udc_dwc3_clr_halt_storm.window_start_cyc = now;
		udc_dwc3_clr_halt_storm.ep_addr = ep_addr;
		udc_dwc3_clr_halt_storm.count = 1U;
		return;
	}

	udc_dwc3_clr_halt_storm.count++;

	if (udc_dwc3_clr_halt_storm.count < CONFIG_UDC_DWC3_FR_CLEAR_HALT_THRESHOLD) {
		return;
	}

	snprintk(reason, sizeof(reason), "CLR-HALT-0x%02x", ep_addr);
	udc_dwc3_failure_capture(dev, reason);
}

#if CONFIG_UDC_DWC3_FR_CLEAR_HALT_BULK_IN
static void udc_dwc3_clear_halt_bulk_in_check(const struct device *const dev,
					      const uint8_t ep_addr)
{
	char reason[28];

	if (!udc_dwc3_is_streaming(dev)) {
		return;
	}

	if (!USB_EP_DIR_IS_IN(ep_addr) || USB_EP_GET_IDX(ep_addr) == 0) {
		return;
	}

	snprintk(reason, sizeof(reason), "CLR-HALT-IN-0x%02x", ep_addr);
	udc_dwc3_failure_capture(dev, reason);
}
#endif /* CONFIG_UDC_DWC3_FR_CLEAR_HALT_BULK_IN */
#endif /* CONFIG_UDC_DWC3_FR_CLEAR_HALT_STORM */

#if CONFIG_UDC_DWC3_FR_TRB_DESYNC
static void udc_dwc3_trb_desync_check(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	char reason[24];

	if (ep_data->head == ep_data->tail) {
		return;
	}

	snprintk(reason, sizeof(reason), "TRB-DESYNC-0x%02x", ep_data->cfg.addr);
	udc_dwc3_failure_capture(dev, reason);
}
#endif

#if CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS > 0
static void udc_dwc3_fr_watchdog_kick(const struct device *const dev);

static void udc_dwc3_fr_stream_progress_reset(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t now = k_cycle_get_32();

	(void)k_work_cancel_delayable(&priv->fr_watchdog_work);
	priv->last_bulk_in_complete_cyc = now;
	priv->last_fpga_in_complete_cyc = now;
	udc_dwc3_fr_watchdog_kick(dev);
}

static uint32_t udc_dwc3_stream_progress_cyc(const struct udc_dwc3_data *const priv)
{
	uint32_t progress = priv->last_bulk_in_complete_cyc;

#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
	if (priv->last_fpga_in_complete_cyc > progress) {
		progress = priv->last_fpga_in_complete_cyc;
	}
#endif

	return progress;
}

static void udc_dwc3_fr_watchdog_fn(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct udc_dwc3_data *priv = CONTAINER_OF(dwork, struct udc_dwc3_data,
						  fr_watchdog_work);
	const struct device *const dev = priv->dev;
	const uint32_t hz = sys_clock_hw_cycles_per_sec();
	const uint32_t idle_cyc =
		((uint64_t)CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS * hz) / 1000U;
	const uint32_t now = k_cycle_get_32();
	uint32_t progress;

	if (udc_dwc3_collapse_captured) {
		return;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
	/* COMMIT set fpga_owned before sensor bring-up; do not idle-capture yet. */
	if (udc_dwc3_fpga_bringup_pending(dev)) {
		k_work_schedule(dwork, K_MSEC(CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS / 2));
		return;
	}

	if (!udc_dwc3_any_fpga_streaming(dev) && !udc_dwc3_is_streaming(dev)) {
		return;
	}

#if CONFIG_UDC_DWC3_FR_STREAM_HANDOFF_GRACE_MS > 0
	if (priv->last_fpga_handoff_cyc != 0U) {
		const uint32_t grace_cyc =
			((uint64_t)CONFIG_UDC_DWC3_FR_STREAM_HANDOFF_GRACE_MS * hz) /
			1000U;

		if ((now - priv->last_fpga_handoff_cyc) < grace_cyc) {
			k_work_schedule(dwork, K_MSEC(CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS / 2));
			return;
		}
	}
#endif

	/* RTL accelerator may keep producing frames after ACM dies; poll frame
	 * counters when FPGA owns the bulk-IN path (DepEvt heartbeats can lag).
	 */
	if (udc_dwc3_any_fpga_streaming(dev) && udc_dwc3_vendor_fpga_frames_active()) {
		priv->last_fpga_in_complete_cyc = now;
		k_work_schedule(dwork, K_MSEC(CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS / 2));
		return;
	}

	progress = udc_dwc3_any_fpga_streaming(dev) ? priv->last_fpga_in_complete_cyc :
						      udc_dwc3_stream_progress_cyc(priv);
#else
	if (!udc_dwc3_is_streaming(dev)) {
		return;
	}

	progress = udc_dwc3_stream_progress_cyc(priv);
#endif

	if (progress != 0U && (now - progress) > idle_cyc) {
		udc_dwc3_failure_capture(dev, "STREAM-IDLE");
		return;
	}

	k_work_schedule(dwork, K_MSEC(CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS / 2));
}

static void udc_dwc3_fr_watchdog_kick(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	k_work_schedule(&priv->fr_watchdog_work,
			K_MSEC(CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS / 2));
}
#endif /* CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS > 0 */
#else
static inline void udc_dwc3_failure_capture(const struct device *const dev,
					    const char *const reason)
{
	if (udc_dwc3_collapse_captured) {
		return;
	}

	udc_dwc3_collapse_captured = true;
	udc_dwc3_diag_snapshot(dev, reason);
}
#endif /* CONFIG_UDC_DWC3_FLIGHT_RECORDER */

void udc_dwc3_flight_recorder_dump(const struct device *const dev, const char *reason)
{
#if CONFIG_UDC_DWC3_FLIGHT_RECORDER
	ARG_UNUSED(dev);
	udc_dwc3_fr_dump(reason != NULL ? reason : "manual");
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(reason);
#endif
}

void udc_dwc3_debug_snapshot(const struct device *const dev, const char *reason)
{
#if CONFIG_UDC_DWC3_FLIGHT_RECORDER
	udc_dwc3_diag_snapshot(dev, reason != NULL ? reason : "debug");
#else
	ARG_UNUSED(dev);
	ARG_UNUSED(reason);
#endif
}

static void udc_dwc3_on_usb_reset(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *const ep0_out = &cfg->ep_data_out[0];
	struct udc_dwc3_ep_data *const ep0_in = &cfg->ep_data_in[0];
	struct net_buf *buf;

	priv->link_ready = false;
	priv->ep_reinit_after_reset = true;
	priv->bus_reset_recovering = true;
	udc_dwc3_collapse_captured = false;
#if CONFIG_UDC_DWC3_FLIGHT_RECORDER && CONFIG_UDC_DWC3_FR_CLEAR_HALT_STORM
	udc_dwc3_clear_halt_storm_reset();
#endif
#if CONFIG_UDC_DWC3_FLIGHT_RECORDER && CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS > 0
	{
		struct udc_dwc3_data *const priv = udc_get_private(dev);

		priv->last_bulk_in_complete_cyc = 0;
		priv->last_fpga_in_complete_cyc = 0;
		k_work_cancel_delayable(&priv->fr_watchdog_work);
	}
#endif

	/* Instrumentation: capture the EP0 control-pipe state on reset entry.
	 * A "stuck busy" EP0 OUT or a missing SETUP buffer here is the
	 * signature of the Windows re-enumeration reset loop.
	 */
	buf = udc_buf_peek(&ep0_out->cfg);
	LOG_INF("USB bus reset (hw addr -> 0): ep0 in_busy=%u out_busy=%u out_buf=%p setup=%u",
		udc_ep_is_busy(&ep0_in->cfg), udc_ep_is_busy(&ep0_out->cfg),
		(void *)buf, (buf != NULL) ? udc_get_buf_info(buf)->setup : 0);

	/* If the host reset us mid-stream it is reacting to a bad bulk-IN packet
	 * (-71). Snapshot the controller's failure state before we tear the
	 * non-control endpoints down below.
	 */
	if (udc_dwc3_is_streaming(dev)) {
		udc_dwc3_failure_capture(dev, "USBRST-streaming");
	}

	/* Reset all ongoing transfers on non-control endpoints (e.g. UVC bulk
	 * IN while streaming). Software-only: do not DepEndXfer during USBRST.
	 */
	udc_dwc3_reset_noncontrol_eps(dev);
	udc_dwc3_vendor_bus_reset(dev);

	/* A USB reset aborts in-flight EP0 transfers in hardware; completion
	 * events never arrive, so the busy flags go stale. Clear them without
	 * DepEndXfer — endpoint commands issued during bus reset can hang the
	 * controller (infinite CMDACT poll) and prevent boot.
	 */
	udc_ep_set_busy(&ep0_in->cfg, false);
	udc_ep_set_busy(&ep0_out->cfg, false);

	/* Drop stale DATA/STATUS OUT buffers from an aborted control transfer.
	 * Keep a SETUP buffer at queue head — it is the idle control-pipe
	 * buffer and is re-armed in finish_address_zero(). When the queue is
	 * empty the stack enqueues a fresh SETUP immediately after
	 * set_address(0).
	 */
	while ((buf = udc_buf_peek(&ep0_out->cfg)) != NULL &&
	       !udc_get_buf_info(buf)->setup) {
		buf = udc_buf_get(&ep0_out->cfg);
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	/* Clear DCFG address only; CONNECTDONE reconfigures EP0 at link speed. */
	udc_dwc3_apply_address(dev, 0, false);

	/* Do not re-arm SETUP here: DepStartXfer fails while the controller
	 * is still in reset. CONNECTDONE (and set_address(0) from the stack)
	 * reconfigures EP0 and re-arms SETUP once the link is ready.
	 */

	/* Let Zephyr set the device address 0 */
	udc_submit_event(dev, UDC_EVT_RESET, 0);
}

static void udc_dwc3_on_connect_done(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	int mps = 0;

	priv->link_ready = true;

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
	/* CONNECTDONE updates MPS only; EP0 DEPCFG INIT runs in
	 * finish_address_zero() when the stack calls set_address(0).
	 * Reconfiguring here clears ep_reinit_after_reset before the
	 * stack reset handler runs and leaves EP0 unconfigured (log-17).
	 */

	/* Letting GTXFIFOSIZn unchanged */
}

static void udc_dwc3_on_link_state_event(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	reg = sys_read32(base + UDC_DWC3_DSTS);

	switch (reg & UDC_DWC3_DSTS_CONNECTSPD_MASK) {
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		switch (reg & UDC_DWC3_DSTS_USBLNKST_MASK) {
		case UDC_DWC3_DSTS_USBLNKST_USB3_U0:
			LOG_DBG("DSTS_USBLNKST_USB3_U0");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_U1:
			LOG_DBG("DSTS_USBLNKST_USB3_U1");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_U2:
			LOG_DBG("DSTS_USBLNKST_USB3_U2");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_U3:
			LOG_DBG("DSTS_USBLNKST_USB3_U3");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_SS_DIS:
			LOG_DBG("DSTS_USBLNKST_USB3_SS_DIS");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_RX_DET:
			LOG_DBG("DSTS_USBLNKST_USB3_RX_DET");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_SS_INACT:
			LOG_DBG("DSTS_USBLNKST_USB3_SS_INACT");
			/* Irreversible SuperSpeed link failure. If a bulk-IN stall
			 * preceded it, this is the real death (not a clean
			 * disconnect): freeze the flight recorder and snapshot both
			 * sides exactly once, so the ring holds the true run-up
			 * (the XferNotReady storm coalesces into one FR entry).
			 */
			if (udc_dwc3_is_streaming(dev) && !udc_dwc3_collapse_captured) {
				LOG_ERR("LINK-COLLAPSE SS.Inactive after bulk stall");
				udc_dwc3_failure_capture(dev, "LINK-COLLAPSE");
				if (udc_dwc3_vendor_bulk_stall != NULL) {
					udc_dwc3_vendor_bulk_stall(dev, 0x81);
				}
			}
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_POLL:
			LOG_DBG("DSTS_USBLNKST_USB3_POLL");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_RECOV:
			LOG_DBG("DSTS_USBLNKST_USB3_RECOV");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_HRESET:
			LOG_DBG("DSTS_USBLNKST_USB3_HRESET");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_CMPLY:
			LOG_DBG("DSTS_USBLNKST_USB3_CMPLY");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_LPBK:
			LOG_DBG("DSTS_USBLNKST_USB3_LPBK");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB3_RESET_RESUME:
			LOG_DBG("DSTS_USBLNKST_USB3_RESET_RESUME");
			break;
		default:
			LOG_ERR("unknown USB3 link state");
		}
		break;
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
		switch (reg & UDC_DWC3_DSTS_USBLNKST_MASK) {
		case UDC_DWC3_DSTS_USBLNKST_USB2_ON_STATE:
			LOG_DBG("DSTS_USBLNKST_USB2_ON_STATE");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_SLEEP_STATE:
			LOG_DBG("DSTS_USBLNKST_USB2_SLEEP_STATE");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_SUSPEND_STATE:
			LOG_DBG("DSTS_USBLNKST_USB2_SUSPEND_STATE");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_DISCONNECTED:
			LOG_DBG("DSTS_USBLNKST_USB2_DISCONNECTED");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_EARLY_SUSPEND:
			LOG_DBG("DSTS_USBLNKST_USB2_EARLY_SUSPEND");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_RESET:
			LOG_DBG("DSTS_USBLNKST_USB2_RESET");
			break;
		case UDC_DWC3_DSTS_USBLNKST_USB2_RESUME:
			LOG_DBG("DSTS_USBLNKST_USB2_RESUME");
			break;
		default:
			LOG_ERR("unknown USB2 link state");
		}
		break;
	default:
		LOG_ERR("unknown connection speed");
	}
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
		struct usb_setup_packet *setup;

		buf = udc_buf_peek(&ep_data->cfg);
		if (buf == NULL) {
			LOG_ERR("missing buffer for SETUP packet");
			udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
			return;
		}

		setup = (struct usb_setup_packet *)buf->data;

		/* Apply SET_ADDRESS before the status stage so the device can
		 * ACK at the new address (Windows SS). Skip while the stack
		 * reset handler is still running — that races with set_address(0).
		 */
		if (setup->bmRequestType == USB_REQTYPE_TYPE_STANDARD &&
		    setup->bRequest == USB_SREQ_SET_ADDRESS) {
			struct udc_dwc3_data *const priv = udc_get_private(dev);

			if (!priv->ep_reinit_after_reset && !priv->bus_reset_recovering &&
			    setup->wValue != 0) {
				udc_dwc3_apply_address(dev, setup->wValue, false);
			}
		}

		/* Update the size to what the hardware reports */
		buf->len = buf->size - FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb_status);

		LOG_HEXDUMP_DBG(buf->data, buf->len, "SETUP received");

		/* The buffer will directly be taken from the UDC queue */
		udc_setup_received(dev, NULL);
	} else {
		buf = udc_buf_get(&ep_data->cfg);
		if (buf == NULL) {
			LOG_ERR("Failed to get a buffer for ep 0x%02x", ep_data->cfg.addr);
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

static void udc_dwc3_on_xfer_not_ready(const struct device *const dev,
				       const uint32_t evt)
{
	switch (evt & UDC_DWC3_DEPEVT_STATUS_B3_MASK) {
	case UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_SETUP:
		LOG_DBG("UDC_DWC3_DEPEVT_XFERNOTREADY_CONTROL_SETUP");
		break;
	case UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_DATA:
		LOG_DBG("UDC_DWC3_DEPEVT_XFERNOTREADY_CONTROL_DATA");
		break;
	case UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_STATUS:
		LOG_DBG("UDC_DWC3_DEPEVT_XFERNOTREADY_CONTROL_STATUS");
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
		udc_dwc3_diag_snapshot(dev, "TRB-MISSEDISOC");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS");
		udc_dwc3_diag_snapshot(dev, "TRB-XFERINPROGRESS");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING:
		LOG_ERR("UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING");
		break;
	default:
		CODE_UNREACHABLE;
	}
}

static void udc_dwc3_on_xfer_done_norm(const struct device *const dev,
				       const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *const ep_data =
		(epn & 1) ? &cfg->ep_data_in[epn >> 1] : &cfg->ep_data_out[epn >> 1];
	volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->tail];
	struct net_buf *buf;
	int ret;

#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
	/* FPGA uvcmanager owns the TRB ring after handoff; completions have no
	 * net_buf association and must not advance the CPU-side ring.
	 */
	if (ep_data->fpga_owned) {
		if (ep_data->fpga_streaming && USB_EP_DIR_IS_IN(ep_data->cfg.addr) &&
		    USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
#if CONFIG_UDC_DWC3_FLIGHT_RECORDER && CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS > 0
			struct udc_dwc3_data *const priv = udc_get_private(dev);

			priv->last_fpga_in_complete_cyc = k_cycle_get_32();
			udc_dwc3_fr_watchdog_kick(dev);
#endif
		}
		LOG_DBG("ignore FPGA-managed completion ep=0x%02x", ep_data->cfg.addr);
		return;
	}
#endif

	/* Lattice USB23 often raises DEPEVT XFERINPROGRESS for bulk completion while
	 * the TRB still shows XFERINPROGRESS (underrun / not yet done). Only pop
	 * when the TRB status indicates the transfer finished (OK). Popping on
	 * every INPROGRESS event breaks ACM; never popping breaks bulk entirely.
	 */
	if ((trb->status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK) ==
	    UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS) {
#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
		if (ep_data->tail == ep_data->inprog_tail) {
			if (ep_data->inprog_cnt < 255) {
				ep_data->inprog_cnt++;
			}
		} else {
			ep_data->inprog_tail = ep_data->tail;
			ep_data->inprog_cnt = 1;
		}

		if (ep_data->inprog_cnt >= 6U && ep_data->xferrscidx != 0U) {
			LOG_ERR("Force DepEndXfer ep=0x%02x tail=%u head=%u (stuck INPROGRESS)",
				ep_data->cfg.addr, ep_data->tail, ep_data->head);
			udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
			udc_ep_cancel_queued(dev, &ep_data->cfg);
			udc_dwc3_trb_ring_reset(ep_data);
			ep_data->inprog_cnt = 0;
			udc_dwc3_vendor_ep_recovery(dev, ep_data->cfg.addr);
			k_work_submit(&ep_data->work);
			return;
		}
#endif
		LOG_DBG("TRB in-progress ep=0x%02x, re-arm only", ep_data->cfg.addr);
		k_work_submit(&ep_data->work);
		return;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
	ep_data->inprog_cnt = 0;
#endif

	/* Clear the TRB that triggered the event */
	buf = udc_dwc3_pop_trb(dev, ep_data);
	if (buf == NULL) {
		/* Stray completion after DepEndXfer with an empty ring. */
		if (ep_data->head == ep_data->tail) {
			LOG_DBG("discard stray completion ep=0x%02x", ep_data->cfg.addr);
			return;
		}

#if CONFIG_UDC_DWC3_FR_TRB_DESYNC
		udc_dwc3_trb_desync_check(dev, ep_data);
#endif
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return;
	}

	LOG_DBG("XFER_DONE_NORM: EP 0x%02x, data %p", ep_data->cfg.addr, (void *)buf->data);
	udc_dwc3_on_xfer_done(dev, ep_data);

	/* For buffers coming from the host, update the size actually received */
	if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
		buf->len = buf->size - FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb->status);
	}

	ret = udc_submit_ep_event(dev, buf, 0);
	if (ret != 0) {
		LOG_ERR("Failed to submit buffer %p: %d", buf, ret);
	}

#if CONFIG_UDC_DWC3_FLIGHT_RECORDER && CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS > 0
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr) && USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		struct udc_dwc3_data *const priv = udc_get_private(dev);

		priv->last_bulk_in_complete_cyc = k_cycle_get_32();
		udc_dwc3_fr_watchdog_kick(dev);
	}
#endif

	/* We just made some room for a new buffer, check if something more to enqueue */
	k_work_submit(&ep_data->work);
}

#define NORMAL_EP(n, fn) fn(n + 2)

static void udc_dwc3_handle_event(const struct device *const dev, const uint32_t evt)
{
	switch (evt) {
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(0):
		LOG_DBG("DEPEVT_XFERCOMPLETE(0)");
		udc_dwc3_on_ctrl_out(dev);
		break;
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(1):
		LOG_DBG("DEPEVT_XFERCOMPLETE(1)");
		udc_dwc3_on_ctrl_in(dev);
		break;
	case LISTIFY(30, NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERCOMPLETE):
	case LISTIFY(30, NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERINPROGRESS):
		udc_dwc3_on_xfer_done_norm(dev, evt);
		break;
	case UDC_DWC3_DEPEVT_XFERNOTREADY(0):
	case UDC_DWC3_DEPEVT_XFERNOTREADY(1):
		udc_dwc3_on_xfer_not_ready(dev, evt);
		break;
	case LISTIFY(30, NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERNOTREADY):
#ifdef CONFIG_UDC_DWC3_NRDY_PROBE_OUT
		{
			struct udc_dwc3_data *const priv = udc_get_private(dev);
			uint32_t phys = (evt >> 1) & 0x1f;

			priv->depevt_nrdy_count[phys]++;
			priv->depevt_nrdy_last[phys] = evt;
		}
#endif
		break;
	case UDC_DWC3_DEVT_DISCONNEVT:
		LOG_INF("USB disconnect event");
		break;
	case UDC_DWC3_DEVT_USBRST:
		LOG_DBG("DEVT_USBRST");
		udc_dwc3_on_usb_reset(dev);
		break;
	case UDC_DWC3_DEVT_CONNECTDONE:
		LOG_DBG("DEVT_CONNECTDONE");
		udc_dwc3_on_connect_done(dev);
		break;
	case UDC_DWC3_DEVT_ULSTCHNG:
		LOG_DBG("DEVT_ULSTCHNG");
		udc_dwc3_on_link_state_event(dev);
		break;
	case UDC_DWC3_DEVT_WKUPEVT:
		LOG_DBG("DEVT_WKUPEVT");
		break;
	case UDC_DWC3_DEVT_SUSPEND:
		LOG_DBG("DEVT_SUSPEND");
		break;
	case UDC_DWC3_DEVT_SOF:
		LOG_DBG("DEVT_SOF");
		break;
	case UDC_DWC3_DEVT_CMDCMPLT:
		LOG_DBG("DEVT_CMDCMPLT");
		break;
	case UDC_DWC3_DEVT_VNDRDEVTSTRCVED:
		LOG_DBG("DEVT_VNDRDEVTSTRCVED");
		break;
	case UDC_DWC3_DEVT_ERRTICERR:
	case UDC_DWC3_DEVT_EVNTOVERFLOW: {
		const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
		static bool overflow_logged;

		if (!overflow_logged) {
			overflow_logged = true;
			/* Stop the overflow->event->overflow feedback loop. */
			sys_clear_bits(base + UDC_DWC3_DEVTEN, UDC_DWC3_DEVTEN_EVNTOVERFLOWEN);
			LOG_ERR("DEVT overflow/errtic evt=0x%x (further logs suppressed)",
				evt);
		}
		break;
	}
	default:
		LOG_ERR("unhandled event: 0x%x", evt);
		CODE_UNREACHABLE;
	}
}

static void udc_dwc3_irq_handler(void *const ptr)
{
	const struct device *const dev = ptr;
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	/* disable further interrupts until all events are processed */
	sys_set_bits(base + UDC_DWC3_GEVNTSIZ(0), UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);

	while (sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) > 0) {
		const uint32_t evt = cfg->evt_buf[priv->evt_next];

		/* Flight recorder: store the full, unmasked word (carries the
		 * DEPEVT status nibble, e.g. BUSERR) before the dispatch mask
		 * strips it. RAM-only, no logging on the hot path.
		 */
		udc_dwc3_fr_record(evt);

		/* Count DEPEVT events per physical endpoint (bit0=0 => DEPEVT).
		 * An FPGA-owned EP should stay at 0 here; any increase means the
		 * controller is still raising events for the accelerated transfer.
		 */
		if ((evt & 0x1) == 0U) {
			priv->depevt_count[(evt >> 1) & 0x1f]++;
		}

		/* Dispatch the even directly from IRQ */
		udc_dwc3_handle_event(dev, evt & UDC_DWC3_EVT_MASK);

		/* Move to next event entry for both hardware and software */
		sys_write32(sizeof(uint32_t), base + UDC_DWC3_GEVNTCOUNT(0));
		udc_dwc3_ring_inc(&priv->evt_next, CONFIG_UDC_DWC3_EVENTS_NUM);
	}

	/* Allow further interrupts */
	sys_clear_bits(base + UDC_DWC3_GEVNTSIZ(0), UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
}

/*
 * UDC API
 *
 * Interface called by Zehpyr from the upper levels of abstractions.
 */

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
		if (sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) {
			LOG_DBG("submitting to EP 0x%02x", ep_cfg->addr);
			k_work_submit(&ep_data->work);
		}
	}

	return 0;
}

static int udc_dwc3_ep_dequeue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);

	udc_ep_cancel_queued(dev, ep_cfg);
	udc_ep_set_busy(ep_cfg, false);

	return 0;
}

static int udc_dwc3_ep_disable(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	/* End any active transfer before disabling. ep_enable() always issues
	 * DEPSTRTXFER; if the previous transfer was never ended the command
	 * returns CMDERR (status 0x1) and platform handoff gets a dead resource.
	 * Skip during bus-reset recovery: DEPENDXFER must not run in USBRST.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 && ep_data->xferrscidx != 0 &&
	    !priv->bus_reset_recovering) {
		LOG_INF("DepEndXfer on disable ep=0x%02x rscidx=%u",
			ep_data->cfg.addr, ep_data->xferrscidx);
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		ep_data->xferrscidx = 0;
	}

	k_work_cancel(&ep_data->work);
	udc_ep_set_busy(&ep_data->cfg, false);

	sys_clear_bit(base + UDC_DWC3_DALEPENA, ep_data->epn);

	return 0;
}

static int udc_dwc3_ep_set_halt(const struct device *const dev,
				struct udc_ep_config *const ep_cfg)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	/* TODO: empty the buffers from the queue */

	switch (ep_data->cfg.addr) {
	case USB_CONTROL_EP_IN:
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		/* The datasheet says to only set stall the OUT direction */
		ep_data = &cfg->ep_data_out[0];
		__fallthrough;
	case USB_CONTROL_EP_OUT:
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		udc_dwc3_depcmd_set_stall(dev, ep_data);
		break;
	default:
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		udc_dwc3_depcmd_set_stall(dev, ep_data);
		ep_data->cfg.stat.halted = true;
	}

	return 0;
}

static int udc_dwc3_ep_clear_halt(const struct device *const dev,
				  struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	__ASSERT_NO_MSG(ep_data->cfg.addr != USB_CONTROL_EP_OUT);
	__ASSERT_NO_MSG(ep_data->cfg.addr != USB_CONTROL_EP_IN);

	/* Host CLEAR_FEATURE must terminate any active transfer before
	 * DepClearStall, even when stat.halted was cleared in software.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 && ep_data->xferrscidx != 0) {
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	}

	udc_dwc3_depcmd_clear_stall(dev, ep_data);
	ep_data->cfg.stat.halted = false;

	udc_dwc3_vendor_ep_clear_halt(dev, ep_data->cfg.addr);

#if CONFIG_UDC_DWC3_FLIGHT_RECORDER
#if CONFIG_UDC_DWC3_FR_CLEAR_HALT_BULK_IN
	udc_dwc3_clear_halt_bulk_in_check(dev, ep_data->cfg.addr);
#endif
#if CONFIG_UDC_DWC3_FR_CLEAR_HALT_STORM
	if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
		udc_dwc3_clear_halt_storm_check(dev, ep_data->cfg.addr);
	}
#endif
#if CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS > 0
	udc_dwc3_fr_watchdog_kick(dev);
#endif
#endif /* CONFIG_UDC_DWC3_FLIGHT_RECORDER */

	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		k_work_submit(&ep_data->work);
	}

	return 0;
}

static void udc_dwc3_dgcmd(const struct device *const dev, const uint32_t cmd)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	sys_write32(cmd, base + UDC_DWC3_DGCMD);
	do {
		reg = sys_read32(base + UDC_DWC3_DGCMD);
	} while ((reg & UDC_DWC3_DGCMD_ACT) != 0);

	if ((reg & UDC_DWC3_DGCMD_STATUS_MASK) != UDC_DWC3_DGCMD_STATUS_OK) {
		LOG_ERR("DGCMD 0x%x failed, status 0x%08x", cmd, reg);
	}
}

static int udc_dwc3_set_system_exit_latency(const struct device *const dev,
					    const struct usb_system_exit_latency *sel)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;
	uint32_t pel;

	reg = sys_read32(base + UDC_DWC3_DCTL);
	pel = (reg & UDC_DWC3_DCTL_INITU2ENA) ? sel->u2pel : sel->u1pel;
	pel = (pel > 125) ? 0 : pel;

	LOG_INF("SET_SEL pel=%u", pel);

	sys_write32(pel, base + UDC_DWC3_DGCMDPAR);
	udc_dwc3_dgcmd(dev, UDC_DWC3_DGCMD_EXITLATENCY);

	return 0;
}

static int udc_dwc3_apply_address(const struct device *const dev, const uint8_t addr,
				  bool ep_reconfigure)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	LOG_INF("Setting address to %u", addr);

	reg = sys_read32(base + UDC_DWC3_DCFG);
	reg &= ~UDC_DWC3_DCFG_DEVADDR_MASK;
	reg |= FIELD_PREP(UDC_DWC3_DCFG_DEVADDR_MASK, addr);
	sys_write32(reg, base + UDC_DWC3_DCFG);

	if (ep_reconfigure) {
		udc_dwc3_finish_address_zero(dev);
	}

	return 0;
}

static int udc_dwc3_set_address(const struct device *const dev, const uint8_t addr)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint8_t current;

	current = FIELD_GET(UDC_DWC3_DCFG_DEVADDR_MASK, sys_read32(base + UDC_DWC3_DCFG));

	if (current == addr) {
		if (addr == 0) {
			udc_dwc3_finish_address_zero(dev);
		}

		return 0;
	}

	return udc_dwc3_apply_address(dev, addr, addr == 0);
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

	LOG_DBG("Enabling DWC3 driver");

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

/*
 * Hardware Init
 *
 * Prepare the driver and the hardware to being used.
 * This goes through register configuration and register commands.
 */

static int udc_dwc3_ep_enable(const struct device *const dev,
			      struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = (struct udc_dwc3_ep_data *)ep_cfg;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	LOG_DBG("%s 0x%02x", __func__, ep_data->cfg.addr);

	memset(ep_data->trb_buf, 0, sizeof(*ep_data->trb_buf) * CONFIG_UDC_DWC3_TRB_NUM);
	udc_dwc3_depcmd_ep_config(dev, ep_data);
	udc_dwc3_depcmd_ep_xfer_config(dev, ep_data);

	/* After a mid-stream crash the host re-enables bulk EPs during re-
	 * enumeration while the controller still owns the previous transfer
	 * (bus reset clears software state but does not issue DEPENDXFER).
	 * Run after DEPCFG so the endpoint is configured; skip during bus-
	 * reset recovery when DEPENDXFER can hang the controller.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 && ep_data->xferrscidx != 0 &&
	    !priv->bus_reset_recovering) {
		LOG_INF("DepEndXfer on enable ep=0x%02x rscidx=%u",
			ep_data->cfg.addr, ep_data->xferrscidx);
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		ep_data->xferrscidx = 0;
	}

	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		udc_dwc3_trb_norm_init(dev, ep_data);
	}

	/* Starting from here, the endpoint can be used */
	sys_set_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		udc_dwc3_depcmd_clear_stall(dev, ep_data);
		ep_data->cfg.stat.halted = false;
	}

	/* Walk through the list of buffer to enqueue we might have blocked */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		k_work_submit(&ep_data->work);
	}

	return 0;
}

/*
 * Prepare and configure most of the parts, if the controller has a way
 * of detecting VBUS activity it should be enabled here.
 * Only udc_dwc3_enable() makes device visible to the host.
 */
static int udc_dwc3_init(const struct device *const dev)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
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

	LOG_INF("Event buffer is %u bytes", sys_read32(base + UDC_DWC3_GEVNTSIZ(0)));

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
	.set_address = udc_dwc3_set_address,
	.set_system_exit_latency = udc_dwc3_set_system_exit_latency,
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

#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
	if (ep_data->fpga_owned) {
		LOG_DBG("endpoint is FPGA-owned, not processing buffers");
		return;
	}
#endif

	while ((buf = udc_buf_peek(&ep_data->cfg)) != NULL) {
		LOG_DBG("Processing buffer %p from queue", (void *)buf);

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

#if CONFIG_UDC_DWC3_FLIGHT_RECORDER
#if CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS > 0
	{
		struct udc_dwc3_data *const priv = udc_get_private(dev);

		k_work_init_delayable(&priv->fr_watchdog_work, udc_dwc3_fr_watchdog_fn);
	}
#endif
	LOG_INF("UDC flight recorder ON: %u entries, bulk_in_halt=%d, out_storm_thr=%d, "
		"idle_ms=%d",
		CONFIG_UDC_DWC3_FLIGHT_RECORDER_NUM,
		IS_ENABLED(CONFIG_UDC_DWC3_FR_CLEAR_HALT_BULK_IN),
		CONFIG_UDC_DWC3_FR_CLEAR_HALT_THRESHOLD,
		CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS);
#endif

	return 0;
}

void udc_bus_reset_recovery_done(const struct device *const dev)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	priv->bus_reset_recovering = false;
}

#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
void lattice_usb23_bulk_restart_xfer(const struct device *dev, uint8_t ep_addr)
{
	struct udc_dwc3_ep_data *ep_data = (void *)udc_get_ep_cfg(dev, ep_addr);

	if (ep_data == NULL || USB_EP_GET_IDX(ep_addr) == 0 ||
	    !ep_data->cfg.stat.enabled) {
		return;
	}

	k_work_cancel(&ep_data->work);
	udc_ep_cancel_queued(dev, &ep_data->cfg);
	udc_ep_set_busy(&ep_data->cfg, false);
	ep_data->cfg.stat.halted = false;

	if (ep_data->xferrscidx != 0) {
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	}

	udc_dwc3_trb_ring_reset(ep_data);
	/* Keep xferrscidx: DEPXFERCFG allocated it once; platform handoff reads
	 * this index. Zeroing it here made handoff use rscidx=0 after restart.
	 */

	memset((void *)ep_data->trb_buf, 0,
	       sizeof(*ep_data->trb_buf) * CONFIG_UDC_DWC3_TRB_NUM);
	udc_dwc3_trb_norm_init(dev, ep_data);
}

void lattice_usb23_ep_set_fpga_owned(const struct device *dev, uint8_t ep_addr, bool owned)
{
	struct udc_dwc3_ep_data *ep_data = (void *)udc_get_ep_cfg(dev, ep_addr);

	if (ep_data == NULL || USB_EP_GET_IDX(ep_addr) == 0 ||
	    !USB_EP_DIR_IS_IN(ep_addr)) {
		return;
	}

	if (ep_data->fpga_owned == owned) {
		return;
	}

	ep_data->fpga_owned = owned;
	if (!owned) {
		ep_data->fpga_streaming = false;
	}

	/* Re-issue DEPCFG (MODIFY) so the controller stops/starts raising
	 * XferComplete/XferInProgress events for this endpoint: an FPGA-owned EP
	 * must be event-free (the CPU never services it), and on release it must
	 * generate events again for normal CPU-driven transfers. Only meaningful
	 * when the masking is compiled in; otherwise events are always enabled
	 * and re-issuing DEPCFG mid-stream would be a pointless disruption.
	 */
#ifdef CONFIG_UDC_DWC3_MASK_FPGA_EP_EVENTS
	if (ep_data->cfg.stat.enabled) {
		udc_dwc3_depcmd_ep_config(dev, ep_data);
		LOG_INF("ep 0x%02x fpga_owned=%d, events %s", ep_addr, owned,
			owned ? "disabled" : "enabled");
	} else {
		LOG_DBG("ep 0x%02x fpga_owned=%d", ep_addr, owned);
	}
#else
	LOG_INF("ep 0x%02x fpga_owned=%d (events left enabled)", ep_addr, owned);
#endif
}

void lattice_usb23_ep_set_fpga_streaming(const struct device *dev, uint8_t ep_addr,
					 bool streaming)
{
	struct udc_dwc3_ep_data *ep_data = (void *)udc_get_ep_cfg(dev, ep_addr);

	if (ep_data == NULL || USB_EP_GET_IDX(ep_addr) == 0 ||
	    !USB_EP_DIR_IS_IN(ep_addr)) {
		return;
	}

	if (streaming) {
#if CONFIG_UDC_DWC3_FLIGHT_RECORDER && CONFIG_UDC_DWC3_FR_STREAM_IDLE_MS > 0
		struct udc_dwc3_data *const priv = udc_get_private(dev);

		udc_dwc3_fr_stream_progress_reset(dev);
		priv->last_fpga_handoff_cyc = k_cycle_get_32();
#endif
	}

	ep_data->fpga_streaming = streaming;
	LOG_INF("ep 0x%02x fpga_streaming=%d", ep_addr, streaming);

	if (streaming) {
		udc_dwc3_vendor_fpga_handoff(dev, ep_addr);
	}
}

void lattice_usb23_log_endpoint_map(const struct device *dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	LOG_INF("USB endpoint map (%s):", dev->name);
	for (int epn = 1; epn < cfg->num_in_eps; epn++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[epn];

		if (!ep_data->cfg.stat.enabled) {
			continue;
		}

		LOG_INF("  IN 0x%02x epn=%u rscidx=%u mps=%u"
#if DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23)
			" fpga_capable=1"
#endif
			, ep_data->cfg.addr, ep_data->epn, ep_data->xferrscidx,
			ep_data->cfg.mps);
	}

	for (int epn = 1; epn < cfg->num_out_eps; epn++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[epn];

		if (!ep_data->cfg.stat.enabled) {
			continue;
		}

		LOG_INF("  OUT 0x%02x epn=%u rscidx=%u mps=%u",
			ep_data->cfg.addr, ep_data->epn, ep_data->xferrscidx,
			ep_data->cfg.mps);
	}
}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(lattice_usb23) */

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
