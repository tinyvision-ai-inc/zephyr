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
#include <zephyr/devicetree.h>
#include <zephyr/drivers/usb/udc.h>
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
/* XferNotReady isoc: IsocMicroFrameNum (Synopsys DEPEVT EventParam). */
#define UDC_DWC3_DEPEVT_PARAM_MASK				GENMASK(31, 16)
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
/* StartXfer on isoc: StartMicroFramNum already in the past. */
#define UDC_DWC3_DEPCMD_STATUS_BUSEXPIRY			(2 << 12)
#define UDC_DWC3_DEPCMD_XFERRSCIDX_MASK				GENMASK(22, 16)
/* StartXfer: StreamID / isoc StartMicroFramNum in CommandParam. */
#define UDC_DWC3_DEPCMD_CMDPARAM_MASK				GENMASK(31, 16)
/*
 * Linux dwc3 (__dwc3_gadget_start_isoc): never prestart isoc — wait for
 * XferNotReady, then StartXfer at cur_uf + N*interval. Soft-IP needs a
 * longer lead than Linux's N=4 to fill the first HWO TRB.
 */
/* Soft-IP must post HWO TRBs (with per-SI UVC headers) before the UF. */
#define UDC_DWC3_ISOC_START_UF_AHEAD				128U
#define UDC_DWC3_ISOC_START_RETRIES				8U
#define UDC_DWC3_ISOC_PRIME_SPINS				50000U

/* Soft-IP UsbManager64 doorbell_data / control (uvcmanager@b4000000). */
#define UDC_DWC3_SOFTIP_CTRL_OFF				0x10U
#define UDC_DWC3_SOFTIP_DOORBELL_DATA_OFF			0x24U
#define UDC_DWC3_SOFTIP_CTRL_ENABLE				BIT(0)
#define UDC_DWC3_SOFTIP_CTRL_SHOULD_CONT			BIT(14)
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
#define UDC_DWC3_GDBGLSPMUX_EPSELECT(n)				(((n) & 0xfU) << 4)

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

/* Owner-callback slots: one per RTL UsbMgr (bit set in HW_IN_EP_MASK) */
#define UDC_DWC3_HW_OWNER_CB_MAX 4

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
	/*
	 * True while a DepStartXfer resource is live. Soft-IP starts once at
	 * ep_enable; when the ring runs dry under UVC the next UpdateXfer is
	 * silently dropped (LiteX OUT-RUNDRY), so re-arm must verify.
	 */
	bool xfer_active;
	/* Given by the hardware for use in endpoint commands */
	uint32_t xferrscidx;
	/*
	 * DEPXFERCFG ran for this endpoint since the last DEPSTARTCFG(2). The
	 * pool of transfer resources is finite (it fits every non-control
	 * endpoint exactly once); re-issuing DEPXFERCFG on a re-enable without
	 * a bus reset (SET_CONFIGURATION(0) then (1), sysfs deauthorize/
	 * authorize) drains it and later DEPCMDs fail. Linux allocates once
	 * per endpoint too (DWC3_EP_RESOURCE_ALLOCATED).
	 */
	bool xfer_rsc_allocated;
	/*
	 * Runtime handoff: the RTL (uvcmanager) owns the TRB ring and the
	 * DEPCMD doorbell of this endpoint. Set by lattice_usb23_ep_give_to_rtl,
	 * cleared by take_from_rtl or a UDC-side revoke (reset, disconnect,
	 * disable, host CLEAR_FEATURE(HALT)). While set, every CPU StartXfer,
	 * UpdateXfer, EndXfer, stall and buffer queue on this EP is refused.
	 */
	bool hw_owned;
	/*
	 * CSR base of the UsbMgr instance driving this endpoint (given by the
	 * video driver at give_to_rtl). Used to stop that manager's doorbell on
	 * a forced revoke without touching the other camera's manager.
	 */
	mm_reg_t hw_mgr_base;
	/* DEPEVTs seen while RTL-owned: counted only, never acted on */
	uint32_t hw_owned_evts;
	/* Refused CPU-side operations while RTL-owned (should stay 0) */
	uint32_t hw_owned_refused;
};

/*
 * Recovery paths may only touch CPU-managed endpoints. An RTL-owned endpoint in
 * the recover/poll masks would get UpdateXfer nudges on the video ring.
 * HW_IN_EP_MASK is Kconfig-gated on RTL_DOORBELL; IEBM builds have neither.
 */
#ifndef CONFIG_UDC_DWC3_HW_IN_EP_MASK
#define CONFIG_UDC_DWC3_HW_IN_EP_MASK 0
#endif
BUILD_ASSERT((CONFIG_UDC_DWC3_IN_RECOVER_EP_MASK & CONFIG_UDC_DWC3_HW_IN_EP_MASK) == 0,
	     "UDC_DWC3_IN_RECOVER_EP_MASK must not include RTL (HW_IN) endpoints");

/* Controller-level error events: counted, never fatal (see udc_dwc3_handle_event). */
static struct {
	uint32_t erratic;
	uint32_t overflow;
	uint32_t unknown;
} udc_dwc3_evt_errors;

/*
 * Last raw events popped from the ring (ISR fast lane tags bit 31). Dumped
 * once on the first EVNTOVERFLOW to see what was flooding the ring.
 */
#define UDC_DWC3_EVT_TRACE_N 32U
static uint32_t udc_dwc3_evt_trace[UDC_DWC3_EVT_TRACE_N];
static uint32_t udc_dwc3_evt_trace_idx;
static uint32_t udc_dwc3_evt_trace_total;

static inline void udc_dwc3_evt_trace_add(uint32_t evt_raw, bool isr)
{
	udc_dwc3_evt_trace[udc_dwc3_evt_trace_idx] = evt_raw | (isr ? BIT(31) : 0U);
	udc_dwc3_evt_trace_idx = (udc_dwc3_evt_trace_idx + 1U) % UDC_DWC3_EVT_TRACE_N;
	udc_dwc3_evt_trace_total++;
}

/*
 * Linux-style isoc: Soft-IP is programmed but held disabled until the first
 * XferNotReady supplies StartMicroFramNum (see __dwc3_gadget_start_isoc).
 */
static struct {
	bool pending;
	uintptr_t softip_base;
	uint32_t trb_addr;
	uint32_t depcmd_addr;
	uint8_t ep_addr;
} udc_dwc3_isoc_arm;

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
	/* Bottom-half: drain event ring in thread context (UDC mutex) */
	struct k_sem evt_sem;
	struct k_thread evt_thread;
	k_thread_stack_t *evt_stack;
	/*
	 * Non-control transfer resources allocated for the current config.
	 * DEPSTARTCFG(rsc_idx=2) must run once when the first bulk/interrupt
	 * endpoint is enabled; without it StartXfer fails with no resource
	 * and the RTL UVC manager never gets a usable doorbell index.
	 */
	bool startcfg_nonctrl_done;
	/*
	 * Notified after the UDC revokes RTL ownership of an endpoint. One
	 * slot per UsbMgr instance (dual UVC = two video drivers); every
	 * registered callback is called and filters on the endpoint address.
	 */
	struct {
		void (*cb)(const struct device *dev, uint8_t ep_addr, const char *reason,
			   void *user);
		void *user;
	} hw_owner[UDC_DWC3_HW_OWNER_CB_MAX];
	/* Lattice-ref EP0: SETUP landed while DATA/STATUS was armed. */
	bool ep0_setup_pending;
	/* STATUS TRB is held until XferNotReady(STATUS). */
	bool ep0_status_nrd;
	bool ep0_data_inflight;
	uint8_t ep0_data_dir; /* 0 = EP0 OUT, 1 = EP0 IN */
	void (*iebm_complete_cb)(const struct device *dev, uint8_t ep_addr, void *user);
	void *iebm_complete_user;
	uint8_t iebm_complete_ep;
};

#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
/*
 * IEBM fast lane.
 *
 * Lattice-ref lsc_iebm_buf_int_handler() arms one IOC TRB per 16 KiB page
 * and hands the page back from the USB ISR. The Zephyr path was
 * udc_ep_enqueue -> sysworkq -> trb_bulk, then a 1 ms HWO poll to retire:
 * ~700 pages/s, 4 fps Y16. This lane keeps the TRB ring and the net_buf
 * bookkeeping but runs retire / push / UpdateXfer with interrupts locked,
 * straight from the DWC3 ISR when the head of the event ring is a video
 * DEPEVT. Non-video events still go to the event thread.
 *
 * DEPCMD is one shared engine. A CmdAct=1 command from thread context
 * (depcmd_ex / depcmd_reg) marks depcmd_active; the lane then defers its
 * UpdateXfer and the thread issues it when its own command completes.
 */
static struct {
	struct udc_dwc3_ep_data *ep;
	int (*refill)(const struct device *dev, void *user);
	void *user;
	volatile bool depcmd_active;
	volatile bool kick_pending;
	uint32_t isr_evts;
	uint32_t isr_calls;
	uint32_t retired;
	uint32_t pushed;
	uint32_t kicks;
	uint32_t kicks_deferred;
	uint32_t kick_spin_max;
	uint32_t kick_err;
	uint32_t starts;
	uint32_t last_start_slot;
	uint32_t last_start_idx;
	bool started;
	volatile bool work_pending;
	/* Cycle accounting (k_cycle_get_32, 75 MHz): where the per-page budget goes. */
	uint32_t n_service;
	uint32_t cyc_retire;
	uint32_t cyc_refill;
	uint32_t cyc_kick;
	uint32_t n_evt_thread;
	uint32_t cyc_evt_thread;
	/* Non-video DEPEVTs stolen off the HW ring in the ISR so a 256-slot
	 * video flood cannot overwrite ACM/EP0 completes.
	 */
	uint32_t swq[32];
	uint8_t swq_w;
	uint8_t swq_r;
	uint32_t swq_push;
	uint32_t swq_full;
} iebm_fast;

static int udc_dwc3_iebm_fast_service(const struct device *dev);

static ALWAYS_INLINE bool udc_dwc3_iebm_is_video_evt(uint32_t evt)
{
	const uint32_t epn = (uint32_t)iebm_fast.ep->epn;

	evt &= UDC_DWC3_EVT_MASK;
	return evt == UDC_DWC3_DEPEVT_XFERINPROGRESS(epn) ||
	       evt == UDC_DWC3_DEPEVT_XFERCOMPLETE(epn) ||
	       evt == UDC_DWC3_DEPEVT_XFERNOTREADY(epn);
}

static ALWAYS_INLINE bool iebm_swq_put(uint32_t evt)
{
	const uint8_t n = (uint8_t)((iebm_fast.swq_w + 1U) % 32U);

	if (n == iebm_fast.swq_r) {
		iebm_fast.swq_full++;
		return false;
	}
	iebm_fast.swq[iebm_fast.swq_w] = evt;
	iebm_fast.swq_w = n;
	iebm_fast.swq_push++;
	return true;
}

static ALWAYS_INLINE bool iebm_swq_get(uint32_t *evt)
{
	if (iebm_fast.swq_r == iebm_fast.swq_w) {
		return false;
	}
	*evt = iebm_fast.swq[iebm_fast.swq_r];
	iebm_fast.swq_r = (uint8_t)((iebm_fast.swq_r + 1U) % 32U);
	return true;
}
static uint32_t udc_dwc3_ring_data_hwo_mask(const struct udc_dwc3_ep_data *ep_data);
static void udc_dwc3_iebm_depcmd_enter(void);
static void udc_dwc3_iebm_depcmd_done(const struct device *dev);
#else
static inline void udc_dwc3_iebm_depcmd_enter(void)
{
}
static inline void udc_dwc3_iebm_depcmd_done(const struct device *dev)
{
	ARG_UNUSED(dev);
}
#endif

/*
 * Phase 0 / A2.6 health counters. Printed on a 5 s tick from the event thread.
 * Live soak of Spinex is not required to confirm page size: generated RTL
 * already uses 16 KiB pages (usbPageSize = 0x4000).
 */
static uint32_t udc_dwc3_health_cmd_n;
static uint32_t udc_dwc3_health_spin_sum;
static uint32_t udc_dwc3_health_spin_max;
static uint32_t udc_dwc3_health_halt_to;
static uint32_t udc_dwc3_health_setup_pending;
static uint32_t udc_dwc3_health_ep0_rst;
static int64_t udc_dwc3_health_last_ms;
static const struct device *udc_dwc3_health_dev;
/* 2 ms backstop after connect only. 2 ms from thread start collides with
 * EP0 DEPCMD and the host sees SETUP-address -71.
 */
static bool udc_dwc3_evt_fast;

/* ACM 0x01 / 0x82: last retired DEPCMD words + event vs retire counts. */
#define UDC_DWC3_ACM_TRACE_N 8U
static struct {
	uint8_t ep;
	uint8_t kind;
	uint8_t sts;
	uint32_t result;
} udc_dwc3_acm_cmd[UDC_DWC3_ACM_TRACE_N];
static uint8_t udc_dwc3_acm_cmd_w;
static uint32_t udc_dwc3_acm_cmd_n;
static uint32_t udc_dwc3_acm_cmd_err;
static uint32_t udc_dwc3_acm_xfer_evt[2];
static uint32_t udc_dwc3_acm_xfer_ret[2];

static uint32_t udc_dwc3_ring_data_hwo_mask(const struct udc_dwc3_ep_data *ep_data);

static int udc_dwc3_acm_slot(uint8_t addr)
{
	if (addr == 0x01U) {
		return 0;
	}
	if (addr == 0x82U) {
		return 1;
	}
	return -1;
}

static void udc_dwc3_acm_note_cmd(uint8_t addr, uint32_t cmd, uint32_t result)
{
	const int slot = udc_dwc3_acm_slot(addr);
	const uint8_t kind = (uint8_t)(cmd & 0xfU);
	const uint8_t sts =
		(uint8_t)((result & UDC_DWC3_DEPCMD_STATUS_MASK) >> 12);
	uint8_t i;

	if (slot < 0) {
		return;
	}
	i = udc_dwc3_acm_cmd_w;
	udc_dwc3_acm_cmd[i].ep = addr;
	udc_dwc3_acm_cmd[i].kind = kind;
	udc_dwc3_acm_cmd[i].sts = sts;
	udc_dwc3_acm_cmd[i].result = result;
	udc_dwc3_acm_cmd_w = (uint8_t)((i + 1U) % UDC_DWC3_ACM_TRACE_N);
	udc_dwc3_acm_cmd_n++;
	if (sts != 0U) {
		udc_dwc3_acm_cmd_err++;
		printk("dwc3: ACM CMDSTATUS ep=0x%02x kind=%u sts=%u result=0x%08x\n",
		       addr, kind, sts, result);
	}
}

static void udc_dwc3_acm_health_dump(void)
{
	const struct device *dev = udc_dwc3_health_dev;
	static const uint8_t eps[] = { 0x01U, 0x82U };
	int i;

	printk("ACM cmd_n=%u cmd_err=%u last:",
	       udc_dwc3_acm_cmd_n, udc_dwc3_acm_cmd_err);
	for (i = 0; i < (int)UDC_DWC3_ACM_TRACE_N; i++) {
		const uint8_t idx =
			(uint8_t)((udc_dwc3_acm_cmd_w + (uint8_t)i) % UDC_DWC3_ACM_TRACE_N);
		const uint8_t ep = udc_dwc3_acm_cmd[idx].ep;

		if (ep != 0U) {
			printk(" 0x%02x/%u/%u/0x%08x", ep,
			       udc_dwc3_acm_cmd[idx].kind,
			       udc_dwc3_acm_cmd[idx].sts,
			       udc_dwc3_acm_cmd[idx].result);
		}
	}
	printk(" evt01=%u ret01=%u evt82=%u ret82=%u\n",
	       udc_dwc3_acm_xfer_evt[0], udc_dwc3_acm_xfer_ret[0],
	       udc_dwc3_acm_xfer_evt[1], udc_dwc3_acm_xfer_ret[1]);
	if (dev == NULL) {
		return;
	}
	printk("ACM ring:");
	for (i = 0; i < 2; i++) {
		struct udc_dwc3_ep_data *const ep =
			(struct udc_dwc3_ep_data *)udc_get_ep_cfg(dev, eps[i]);
		uint32_t nb = 0U;
		uint32_t t;

		if (ep == NULL) {
			continue;
		}
		for (t = 0U; t < CONFIG_UDC_DWC3_TRB_NUM - 1U; t++) {
			if (ep->net_buf[t] != NULL) {
				nb++;
			}
		}
		printk(" 0x%02x:xa=%u idx=%u hd=%u tl=%u f=%u hwo=0x%x nb=%u",
		       ep->cfg.addr, ep->xfer_active, ep->xferrscidx,
		       ep->head, ep->tail, ep->full,
		       udc_dwc3_ring_data_hwo_mask(ep), nb);
	}
	printk("\n");
}

void udc_dwc3_health_print(void)
{
	const uint32_t n = udc_dwc3_health_cmd_n;
	const uint32_t mean = n ? (udc_dwc3_health_spin_sum / n) : 0U;

	printk("DWC3HEALTH cmd=%u spin_mean=%u spin_max=%u halt_to=%u "
	       "setup_pend=%u ep0_rst=%u ovf=%u unk=%u\n",
	       n, mean, udc_dwc3_health_spin_max, udc_dwc3_health_halt_to,
	       udc_dwc3_health_setup_pending, udc_dwc3_health_ep0_rst,
	       udc_dwc3_evt_errors.overflow, udc_dwc3_evt_errors.unknown);
	udc_dwc3_acm_health_dump();
}

/*
 * Phase 0 park dump: two snapshots 100 ms apart decide whether TX DMA
 * (AR) stopped, AXI reads stalled, or the TRB ring is inconsistent.
 * usbPiped_* counters live only when Soft-IP debug_regs is wired; CMD_COUNT
 * and GDBG* are always on the hard IP / mailbox.
 */
#define UDC_DWC3_GDBGLSPMUX_EPSELECT_PHYS(epn)	UDC_DWC3_GDBGLSPMUX_EPSELECT(epn)

static uint32_t udc_dwc3_dbg_fifo_avail(const mm_reg_t base, uint32_t fifo_num,
					uint32_t qtype)
{
	uint32_t sel = (fifo_num & 0x1fU) | qtype;

	sys_write32(sel, base + UDC_DWC3_GDBGFIFOSPACE);
	return FIELD_GET(UDC_DWC3_GDBGFIFOSPACE_AVAILABLE_MASK,
			 sys_read32(base + UDC_DWC3_GDBGFIFOSPACE));
}

static void udc_dwc3_dbg_epinfo(const mm_reg_t base, uint32_t epn,
				uint32_t *info0, uint32_t *info1)
{
	sys_write32(UDC_DWC3_GDBGLSPMUX_EPSELECT_PHYS(epn),
		    base + UDC_DWC3_GDBGLSPMUX_DEV);
	*info0 = sys_read32(base + UDC_DWC3_GDBGEPINFO0);
	*info1 = sys_read32(base + UDC_DWC3_GDBGEPINFO1);
}

static void udc_dwc3_park_snapshot(const struct device *const dev, const char *tag);
static void udc_dwc3_rateprobe_print(void);

/* Force the TRB write onto the CPU→SRAM path before a mailbox DEPCMD. */
static void udc_dwc3_trb_fence(volatile struct udc_dwc3_trb *const trb)
{
	volatile uint32_t sink;

	if (trb == NULL) {
		return;
	}
	sink = trb->addr_lo;
	sink += trb->status;
	sink += trb->ctrl;
	ARG_UNUSED(sink);
}

/*
 * After hard-IP DMA into an OUT net_buf, pull the payload onto the CPU
 * path before the class stack (ACM / srp_push_byte) reads it. Same class
 * of same-master race as trb_fence, but on the data buffer.
 */
static void udc_dwc3_out_data_fence(const struct net_buf *const buf)
{
	volatile uint32_t sink = 0;
	const volatile uint8_t *p;
	size_t n;
	size_t i;

	if (buf == NULL || buf->data == NULL || buf->len == 0U) {
		return;
	}
	p = buf->data;
	n = buf->len;
	if (n <= 256U) {
		for (i = 0; i < n; i++) {
			sink += p[i];
		}
	} else {
		sink += p[0];
		sink += p[n / 2U];
		sink += p[n - 1U];
		for (i = 0; i < n; i += 16U) {
			sink += p[i];
		}
	}
	ARG_UNUSED(sink);
}

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

#define DEV_CFG(dev) ((const struct udc_dwc3_config *)(dev->config))
#define DEV_DATA(dev) ((struct udc_dwc3_data *)udc_get_private(dev))

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

__ramfunc static void udc_dwc3_push_trb(const struct device *const dev,
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
	udc_dwc3_trb_fence(trb);

	LOG_DBG("PUSH %u buf %p, data %p, size %u",
		ep_data->head, (void *)buf, (void *)buf->data, buf->size);

	/* Shift the head */
	udc_dwc3_ring_inc(&ep_data->head, CONFIG_UDC_DWC3_TRB_NUM - 1);

	/* If the head touches the tail after we add something, we are full */
	ep_data->full = (ep_data->head == ep_data->tail);
}

__ramfunc static struct net_buf *udc_dwc3_pop_trb(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	struct net_buf *const buf = ep_data->net_buf[ep_data->tail];

	/*
	 * Spurious DEPEVTs under Soft-IP UVC must not advance the ring.
	 * Advancing on an empty slot permanently desyncs ACM IN (host sees
	 * prompt timeout → Write timeout → xHCI death).
	 */
	if (buf == NULL) {
		LOG_WRN("pop: empty TRB ep=0x%02x tail=%u (ignored)",
			ep_data->cfg.addr, ep_data->tail);
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

#if CONFIG_UDC_DWC3_HW_IN_EP_MASK != 0
#define UDC_DWC3_UVCMGR_CONTROL_STATUS			0x0010U
#endif

#if CONFIG_UDC_DWC3_HW_IN_EP_MASK != 0 && !defined(CONFIG_UDC_DWC3_DEPCMD_MAILBOX)
/*
 * The DEPCMD engine is shared across all endpoints, and on Soft-IP it has two
 * masters: this driver, and the uvcmanager RTL which rings DepUpdateXfer
 * directly into DEPCMD(hw_ep) for every video TRB (~40k/s at 60 fps). There
 * is no arbiter; overlapping commands are dropped or jam CMDACT (observed:
 * CDC-RAW IN park + frozen video + "DEPCMD never completed" storms).
 *
 * Waiting for CMDACT idle alone is not enough — the RTL can still ring while
 * our command is executing (soak: 3/10 pass then bulk+video die together).
 * Gating CONTINUE only shrank the window (7/10) since it races the FSM.
 *
 * The RTL now provides a real arbitration handshake: setting HALT_DOORBELL
 * makes the TRB FSM finish any in-flight doorbell, then park; it asserts
 * HALT_ACK, which hardware-guarantees no DEPCMD writes from the uvcmanager
 * until HALT_DOORBELL is cleared. Video data keeps flowing into the FIFO;
 * only the doorbell is deferred, so this is not a stream stop / UVC quiet.
 */
/*
 * Every UsbMgr instance in the Soft-IP (one per camera / video IN endpoint).
 * They all ring the same DEPCMD engine, so the halt handshake must park every
 * manager that is currently enabled, not only the first one.
 */
#define UDC_DWC3_UVCMGR_BASE_ENTRY(node) ((mm_reg_t)DT_REG_ADDR_BY_NAME(node, base)),
static const mm_reg_t udc_dwc3_uvcmgr_bases[] = {
#if DT_HAS_COMPAT_STATUS_OKAY(tinyvision_uvcmanager)
	DT_FOREACH_STATUS_OKAY(tinyvision_uvcmanager, UDC_DWC3_UVCMGR_BASE_ENTRY)
#else
	(mm_reg_t)0xb4000000U,
#endif
};
#define UDC_DWC3_UVCMGR_NUM ARRAY_SIZE(udc_dwc3_uvcmgr_bases)
BUILD_ASSERT(ARRAY_SIZE(udc_dwc3_uvcmgr_bases) <= 32, "uvcmanager pause mask is 32 bits");
#define UDC_DWC3_UVCMGR_CONTROL_ENABLE			BIT(0)
#define UDC_DWC3_UVCMGR_CONTROL_HALT_DOORBELL		BIT(15)
#define UDC_DWC3_UVCMGR_CONTROL_HALT_ACK		BIT(16)

/* Nesting depth of the halt request. Tracked CPU-side (not via the register
 * bit): CONTROL_STATUS is RMW'd by other code without our lock, so a stale
 * write-back could re-assert bit 15; if we treated a set bit as "outer
 * holder", one stale write would make every later pause a no-op and park the
 * uvcmanager doorbell forever (observed: sts=0x1c005, video dead at start).
 * Only touched with interrupts locked.
 */
static uint32_t udc_dwc3_uvcmgr_halt_depth;
/* Bit i set: udc_dwc3_uvcmgr_bases[i] was halted by the outermost pause. */
static uint32_t udc_dwc3_uvcmgr_paused_mask;

/*
 * Halt-ack wait under irq_lock. With narrow Soft-IP ack (IDLE/WAIT_US only),
 * 50 µs almost never saw HALT_ACK at 1080p60. Widened Soft-IP ack (ack except
 * active DEPCMD) should succeed quickly; keep 200 µs as a bound while TRB
 * RAM states finish before DEPCMD.
 */
#define UDC_DWC3_UVCMGR_HALT_TIMEOUT_US 500U

/*
 * Pause Soft-IP DEPCMD doorbell. Return codes (irq locked):
 *  1 — haltAck observed (Soft-IP DEPCMD silent; safe to issue CPU DepCmd)
 *  0 — nested pause or UsbMgr disabled (safe to issue; outer hold / idle)
 * -1 — haltAck timeout (do NOT issue UpdateXfer; Soft-IP may own DEPCMD)
 */
static void udc_dwc3_uvcmgr_clear_halt(const uint32_t mask)
{
	for (uint32_t i = 0; i < UDC_DWC3_UVCMGR_NUM; i++) {
		if ((mask & BIT(i)) != 0U) {
			const mm_reg_t ubase = udc_dwc3_uvcmgr_bases[i];
			const uint32_t sts = sys_read32(ubase + UDC_DWC3_UVCMGR_CONTROL_STATUS);

			sys_write32(sts & ~UDC_DWC3_UVCMGR_CONTROL_HALT_DOORBELL,
				    ubase + UDC_DWC3_UVCMGR_CONTROL_STATUS);
		}
	}
}

static int udc_dwc3_uvcmgr_pause_doorbell(void)
{
	static uint32_t timeouts;
	uint32_t mask = 0U;
	uint32_t acked = 0U;

	if (udc_dwc3_uvcmgr_halt_depth++ != 0U) {
		return 0;
	}

	/* Halt every enabled manager; a disabled one never rings DEPCMD. */
	for (uint32_t i = 0; i < UDC_DWC3_UVCMGR_NUM; i++) {
		const mm_reg_t ubase = udc_dwc3_uvcmgr_bases[i];
		const uint32_t sts = sys_read32(ubase + UDC_DWC3_UVCMGR_CONTROL_STATUS);

		if ((sts & UDC_DWC3_UVCMGR_CONTROL_ENABLE) != 0U) {
			sys_write32(sts | UDC_DWC3_UVCMGR_CONTROL_HALT_DOORBELL,
				    ubase + UDC_DWC3_UVCMGR_CONTROL_STATUS);
			mask |= BIT(i);
		}
	}
	if (mask == 0U) {
		return 0;
	}

	const uint32_t deadline = k_cycle_get_32() +
		(uint32_t)((uint64_t)sys_clock_hw_cycles_per_sec() *
			   UDC_DWC3_UVCMGR_HALT_TIMEOUT_US / 1000000U);

	do {
		for (uint32_t i = 0; i < UDC_DWC3_UVCMGR_NUM; i++) {
			if ((mask & ~acked & BIT(i)) != 0U &&
			    (sys_read32(udc_dwc3_uvcmgr_bases[i] +
					UDC_DWC3_UVCMGR_CONTROL_STATUS) &
			     UDC_DWC3_UVCMGR_CONTROL_HALT_ACK) != 0U) {
				acked |= BIT(i);
			}
		}
		if (acked == mask) {
			udc_dwc3_uvcmgr_paused_mask = mask;
			return 1;
		}
	} while ((int32_t)(k_cycle_get_32() - deadline) < 0);

	udc_dwc3_uvcmgr_clear_halt(mask);
	udc_dwc3_health_halt_to++;
	if ((++timeouts % 64U) == 1U) {
		printk("DEPCMD guard: halt ack timeout (n=%u, acked=0x%x of 0x%x)\n",
		       timeouts, acked, mask);
	}
	return -1;
}

/* Must be called with interrupts locked. */
static void udc_dwc3_uvcmgr_resume_doorbell(const bool paused)
{
	__ASSERT_NO_MSG(udc_dwc3_uvcmgr_halt_depth > 0U);
	udc_dwc3_uvcmgr_halt_depth--;

	if (!paused) {
		return;
	}

	udc_dwc3_uvcmgr_clear_halt(udc_dwc3_uvcmgr_paused_mask);
	udc_dwc3_uvcmgr_paused_mask = 0U;
}

static bool udc_dwc3_uvcmgr_any_enabled(void)
{
	for (uint32_t i = 0; i < UDC_DWC3_UVCMGR_NUM; i++) {
		if ((sys_read32(udc_dwc3_uvcmgr_bases[i] + UDC_DWC3_UVCMGR_CONTROL_STATUS) &
		     UDC_DWC3_UVCMGR_CONTROL_ENABLE) != 0U) {
			return true;
		}
	}
	return false;
}

static void udc_dwc3_depcmd_hw_doorbell_sync(const mm_reg_t base)
{
	static uint32_t caught;
	uint32_t hw_mask = CONFIG_UDC_DWC3_HW_IN_EP_MASK;

	/*
	 * Only glance at Soft-IP doorbells while a UsbMgr is enabled.  At boot /
	 * after a wedged stream, DEPCMD(hw) can sit CMDACT=1 forever.
	 *
	 * CRITICAL: this runs under irq_lock().  Under Soft-IP 1080p60 the
	 * HW_IN DEPCMD is busy nearly continuously — a long spin here starves
	 * the DWC3 event thread and kills ACM in ~20–40s.  Bound to a few
	 * polls; prefer a rare collision over IRQ lockup.
	 */
	if (!udc_dwc3_uvcmgr_any_enabled()) {
		return;
	}

	while (hw_mask != 0U) {
		const uint8_t idx = (uint8_t)__builtin_ctz(hw_mask);
		const uint32_t hw_addr = UDC_DWC3_DEPCMD(idx * 2U + 1U);
		uint32_t spins = 0U;
		bool was_busy = false;

		hw_mask &= ~BIT(idx);

		while ((sys_read32(base + hw_addr) & UDC_DWC3_DEPCMD_CMDACT) != 0U) {
			was_busy = true;
			if (++spins > 64U) {
				break;
			}
		}

		if (was_busy && (++caught % 256U) == 1U) {
			printk("DEPCMD guard: short-wait HW_IN busy (n=%u)\n", caught);
		}
	}
}
#else
static inline int udc_dwc3_uvcmgr_pause_doorbell(void)
{
	return 0;
}

static inline void udc_dwc3_uvcmgr_resume_doorbell(const bool paused)
{
	ARG_UNUSED(paused);
}

static inline void udc_dwc3_depcmd_hw_doorbell_sync(const mm_reg_t base)
{
	ARG_UNUSED(base);
}
#endif

#if defined(CONFIG_UDC_DWC3_DEPCMD_MAILBOX)
#if DT_HAS_COMPAT_STATUS_OKAY(tinyvision_depcmd_mailbox)
#define UDC_DWC3_MBX_BASE \
	((mm_reg_t)DT_REG_ADDR(DT_COMPAT_GET_ANY_STATUS_OKAY(tinyvision_depcmd_mailbox)))
#else
#define UDC_DWC3_MBX_BASE ((mm_reg_t)0xb4008000U)
#endif
#define UDC_DWC3_MBX_ADDR	0x00U
#define UDC_DWC3_MBX_DATA	0x04U
#define UDC_DWC3_MBX_STATUS	0x08U
#define UDC_DWC3_MBX_RESULT	0x0cU
#define UDC_DWC3_MBX_CMD_COUNT	0x10U
#define UDC_DWC3_MBX_TRB	0x14U
#define UDC_DWC3_MBX_QUIET_ARM	0x18U
#define UDC_DWC3_MBX_QUIET_HIT	0x1cU
#define UDC_DWC3_MBX_QUIET_TO	0x20U
#define UDC_DWC3_MBX_STATUS_BUSY	BIT(0)
#define UDC_DWC3_MBX_STATUS_DONE	BIT(1)
#define UDC_DWC3_MBX_STATUS_ERR		BIT(2)

static struct k_spinlock udc_dwc3_mbx_lock;
static uint32_t udc_dwc3_mbx_trb;

/* Post {addr, cmd} to the RTL mailbox; return the retired DEPCMD word. */
static uint32_t udc_dwc3_depcmd_mailbox(const struct device *const dev,
					const uint32_t addr, const uint32_t cmd)
{
	const mm_reg_t dwc = DEVICE_MMIO_NAMED_GET(dev, base);
	const mm_reg_t mbx = UDC_DWC3_MBX_BASE;
	k_spinlock_key_t key;
	uint32_t spins = 0U;
	uint32_t sts;
	uint32_t reg;

	/* One poster. A DATA write while BUSY is dropped by RTL. */
	key = k_spin_lock(&udc_dwc3_mbx_lock);
	while ((sys_read32(mbx + UDC_DWC3_MBX_STATUS) & UDC_DWC3_MBX_STATUS_BUSY) != 0U) {
		k_spin_unlock(&udc_dwc3_mbx_lock, key);
		if (++spins > 1000000U) {
			printk("DEPCMD mailbox busy: cmd 0x%08x\n", cmd);
			return 0U;
		}
		key = k_spin_lock(&udc_dwc3_mbx_lock);
	}
	sys_write32((uint32_t)(dwc + addr), mbx + UDC_DWC3_MBX_ADDR);
	/* Latch expected TRB only for ACM/bulk fetch. A 0 write on every
	 * EP0 post was extra LMMI traffic; skip when unused. */
	if (udc_dwc3_mbx_trb != 0U) {
		sys_write32(udc_dwc3_mbx_trb, mbx + UDC_DWC3_MBX_TRB);
		udc_dwc3_mbx_trb = 0U;
	}
	sys_write32(cmd, mbx + UDC_DWC3_MBX_DATA);
	k_spin_unlock(&udc_dwc3_mbx_lock, key);

	/* BUSY is registered on the DATA write. Do not treat an immediate
	 * !BUSY as completion (that was RESULT=0 / silent EP0 death). */
	do {
		sts = sys_read32(mbx + UDC_DWC3_MBX_STATUS);
		if (++spins > 10000U) {
			printk("DEPCMD mailbox post not accepted: cmd 0x%08x sts 0x%08x\n",
			       cmd, sts);
			return 0U;
		}
	} while ((sts & (UDC_DWC3_MBX_STATUS_BUSY | UDC_DWC3_MBX_STATUS_DONE)) == 0U);

	while ((sts & UDC_DWC3_MBX_STATUS_BUSY) != 0U) {
		sts = sys_read32(mbx + UDC_DWC3_MBX_STATUS);
		if (++spins > 1000000U) {
			printk("DEPCMD mailbox never completed: cmd 0x%08x sts 0x%08x\n",
			       cmd, sts);
			break;
		}
	}

	reg = sys_read32(mbx + UDC_DWC3_MBX_RESULT);
	if ((sts & UDC_DWC3_MBX_STATUS_ERR) != 0U) {
		printk("DEPCMD mailbox ERR: cmd 0x%08x result 0x%08x\n", cmd, reg);
	}

	udc_dwc3_health_cmd_n++;
	udc_dwc3_health_spin_sum += spins;
	if (spins > udc_dwc3_health_spin_max) {
		udc_dwc3_health_spin_max = spins;
	}

	return reg;
}
#endif

static void udc_dwc3_park_snapshot(const struct device *const dev, const char *tag)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t i0_82, i1_82, i0_85, i1_85;
	uint32_t tx2, tx5, evc, dsts, buserr;
	uint32_t cmd5, cmd11, mbx_n = 0U;
	struct udc_dwc3_ep_data *ep82;
	struct udc_dwc3_ep_data *ep85;

	udc_dwc3_dbg_epinfo(base, 5U, &i0_82, &i1_82);
	udc_dwc3_dbg_epinfo(base, 11U, &i0_85, &i1_85);
	tx2 = udc_dwc3_dbg_fifo_avail(base, 2U, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TX);
	tx5 = udc_dwc3_dbg_fifo_avail(base, 5U, UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_TX);
	evc = sys_read32(base + UDC_DWC3_GEVNTCOUNT(0));
	dsts = sys_read32(base + UDC_DWC3_DSTS);
	buserr = sys_read32(base + UDC_DWC3_GBUSERRADDR_LO);
	cmd5 = sys_read32(base + UDC_DWC3_DEPCMD(5));
	cmd11 = sys_read32(base + UDC_DWC3_DEPCMD(11));
#if defined(CONFIG_UDC_DWC3_DEPCMD_MAILBOX)
	mbx_n = sys_read32(UDC_DWC3_MBX_BASE + UDC_DWC3_MBX_CMD_COUNT);
#endif
	printk("PARK0 %s t=%u cmd5=0x%08x cmd11=0x%08x mbx=%u evc=0x%08x "
	       "dsts=0x%08x buserr=0x%08x tx2=%u tx5=%u "
	       "ep82=0x%08x/0x%08x ep85=0x%08x/0x%08x\n",
	       tag, (uint32_t)k_uptime_get(), cmd5, cmd11, mbx_n, evc, dsts,
	       buserr, tx2, tx5, i0_82, i1_82, i0_85, i1_85);

	ep82 = (struct udc_dwc3_ep_data *)udc_get_ep_cfg(dev, 0x82);
	ep85 = (struct udc_dwc3_ep_data *)udc_get_ep_cfg(dev, 0x85);
	if (ep82 != NULL && ep82->trb_buf != NULL) {
		volatile struct udc_dwc3_trb *trb = &ep82->trb_buf[ep82->tail];

		printk("PARK0 %s 82 xa=%u idx=%u hd=%u tl=%u hwo=0x%x "
		       "trb ctrl=0x%08x st=0x%08x lo=0x%08x\n",
		       tag, ep82->xfer_active, ep82->xferrscidx,
		       ep82->head, ep82->tail, udc_dwc3_ring_data_hwo_mask(ep82),
		       trb->ctrl, trb->status, trb->addr_lo);
	}
	if (ep85 != NULL && ep85->trb_buf != NULL) {
		printk("PARK0 %s 85 xa=%u idx=%u hd=%u tl=%u hwo=0x%x\n",
		       tag, ep85->xfer_active, ep85->xferrscidx,
		       ep85->head, ep85->tail, udc_dwc3_ring_data_hwo_mask(ep85));
	}
	udc_dwc3_health_print();
}

/*
 * Phase 2 rate probe: print the RTL-side DepCmdArbiter CMD_COUNT (CPU +
 * video DEPCMDs completed by the shared engine) alongside the existing
 * 5 s DWC3HEALTH line. Two consecutive RATEPROBE lines give a live,
 * measured DEPCMD/s under whatever traffic is running (Y16 only, or
 * Y16 + CDC-RAW/ACM), instead of the theoretical/never-measured
 * estimates in docs/usb_ep_ownership.md and docs/IEBM_PORT_FINDINGS.md.
 */
static void udc_dwc3_rateprobe_print(void)
{
#if defined(CONFIG_UDC_DWC3_DEPCMD_MAILBOX)
	uint32_t mbx_n = sys_read32(UDC_DWC3_MBX_BASE + UDC_DWC3_MBX_CMD_COUNT);

	printk("RATEPROBE t=%u mbx=%u qarm=%u qhit=%u qto=%u\n",
	       (uint32_t)k_uptime_get(), mbx_n,
	       sys_read32(UDC_DWC3_MBX_BASE + UDC_DWC3_MBX_QUIET_ARM),
	       sys_read32(UDC_DWC3_MBX_BASE + UDC_DWC3_MBX_QUIET_HIT),
	       sys_read32(UDC_DWC3_MBX_BASE + UDC_DWC3_MBX_QUIET_TO));
#endif
}

/*
 * Issue a DepCmd. irq_lock covers halt + doorbell glance + CMDACT write.
 * Waiting for CMDACT clear runs with IRQs enabled, but HALT_DOORBELL stays
 * asserted until this command finishes so the UsbMgr cannot ring a
 * No-Response UpdateXfer into a live CPU command (shared DEPCMD engine).
 */
static uint32_t udc_dwc3_depcmd_ex(const struct device *const dev,
				   const uint32_t addr, const uint32_t cmd,
				   const bool halt)
{
#if defined(CONFIG_UDC_DWC3_DEPCMD_MAILBOX)
	uint32_t reg = udc_dwc3_depcmd_mailbox(dev, addr, cmd);

	ARG_UNUSED(halt);
	switch (reg & UDC_DWC3_DEPCMD_STATUS_MASK) {
	case UDC_DWC3_DEPCMD_STATUS_OK:
		break;
	case UDC_DWC3_DEPCMD_STATUS_CMDERR:
		LOG_ERR("endpoint command failed");
		break;
	default:
		LOG_ERR("command failed with unknown status: 0x%08x", reg);
	}
	return FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);
#else
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;
	uint32_t spins = 0U;
	unsigned int key = irq_lock();
	const int pause_rc = halt ? udc_dwc3_uvcmgr_pause_doorbell() : 0;

	udc_dwc3_iebm_depcmd_enter();
	udc_dwc3_depcmd_hw_doorbell_sync(base);
	sys_write32(cmd | UDC_DWC3_DEPCMD_CMDACT, base + addr);
	irq_unlock(key);

	do {
		reg = sys_read32(base + addr);
		if (++spins > 1000000U) {
#if CONFIG_UDC_DWC3_HW_IN_EP_MASK != 0
			const uint8_t hw_idx =
				(uint8_t)__builtin_ctz(CONFIG_UDC_DWC3_HW_IN_EP_MASK);
			const uint32_t hw_addr = UDC_DWC3_DEPCMD(hw_idx * 2U + 1U);

			printk("DEPCMD 0x%03x never completed: cmd 0x%08x reg 0x%08x "
			       "doorbell[0x%03x]=0x%08x\n",
			       addr, cmd, reg, hw_addr, sys_read32(base + hw_addr));
#else
			printk("DEPCMD 0x%03x never completed: cmd 0x%08x reg 0x%08x\n",
			       addr, cmd, reg);
#endif
			break;
		}
	} while ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0);

	if (halt) {
		key = irq_lock();
		udc_dwc3_uvcmgr_resume_doorbell(pause_rc == 1);
		irq_unlock(key);
	}

	udc_dwc3_iebm_depcmd_done(dev);
	udc_dwc3_health_cmd_n++;
	udc_dwc3_health_spin_sum += spins;
	if (spins > udc_dwc3_health_spin_max) {
		udc_dwc3_health_spin_max = spins;
	}

	switch (reg & UDC_DWC3_DEPCMD_STATUS_MASK) {
	case UDC_DWC3_DEPCMD_STATUS_OK:
		break;
	case UDC_DWC3_DEPCMD_STATUS_CMDERR:
		LOG_ERR("endpoint command failed");
		break;
	default:
		LOG_ERR("command failed with unknown status: 0x%08x", reg);
	}

	return FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);
#endif
}

static uint32_t udc_dwc3_depcmd(const struct device *const dev,
				const uint32_t addr, const uint32_t cmd)
{
	return udc_dwc3_depcmd_ex(dev, addr, cmd, true);
}

/*
 * Hardware-owned endpoint guard.
 *
 * Returns true (and refuses the operation) when the RTL currently owns the
 * endpoint. Every CPU-side DEPCMD / queue path on a non-control endpoint goes
 * through this so a stray Zephyr doorbell on the video ring is impossible
 * rather than merely unlikely. Refusals are counted and asserted in debug.
 */
static bool udc_dwc3_ep_refuse_if_hw_owned(struct udc_dwc3_ep_data *const ep_data,
					   const char *const what)
{
#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
	ARG_UNUSED(ep_data);
	ARG_UNUSED(what);
	return false;
#else
	if (!ep_data->hw_owned) {
		return false;
	}

	ep_data->hw_owned_refused++;
	printk("dwc3: ep 0x%02x RTL-owned: refusing %s (n=%u)\n", ep_data->cfg.addr, what,
	       ep_data->hw_owned_refused);
	__ASSERT(false, "Zephyr %s on RTL-owned endpoint 0x%02x", what, ep_data->cfg.addr);

	return true;
#endif
}

static bool udc_dwc3_ep_is_hw_in(const struct udc_dwc3_ep_data *ep_data);

static void udc_dwc3_depcmd_ep_config(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t param0 = 0;
	uint32_t param1 = 0;

	LOG_INF("configuring endpoint 0x%02x with wMaxPacketSize=%u",
		ep_data->cfg.addr, ep_data->cfg.mps);

	if (ep_data->cfg.stat.enabled) {
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

	/*
	 * Burst: bulk/int keep 0. SuperSpeed isoc: DEPCFG.BRSTSIZ must fit the
	 * ~4KiB TX FIFO (dep≈517×8B ≈ 3×1024B packets). Companion still
	 * advertises bMaxBurst=15 / Mult=1 (32KiB/SI) so the host budgets
	 * bandwidth; the core sends multiple smaller bursts within the SI.
	 * BRSTSIZ=15 previously stalled TX (never enough FIFO for one burst)
	 * → host C Zi completions with 0-byte slots.
	 */
	if ((ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_ISO) {
		/*
		 * Linux dwc3_gadget_set_ep_config: DEPCFG.bInterval_m1 =
		 * min(bInterval - 1, 13). Programming raw bInterval (1) makes
		 * the core expect every 2^(1) UF while the host schedules
		 * every UF → MissedIsoc / ZLPs.
		 */
		const uint8_t binterval = ep_data->cfg.interval ? ep_data->cfg.interval : 1U;
		const uint8_t binterval_m1 = (uint8_t)MIN(binterval - 1U, 13U);

		/* Match FIFO (~16KiB) / companion burst without oversizing. */
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 7);
		param1 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR1_DEPCFG_BINTERVAL_MASK,
				     binterval_m1);
	} else {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 0);
	}

	/* Set the FIFO number, must be 0 for all OUT EPs */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_FIFONUM_MASK,
				     ep_data->cfg.addr & 0x7f);
	}

	/*
	 * Soft-IP UVC (HW_IN): skip XferComplete/InProgress (event-ring flood).
	 * Isochronous HW_IN must enable XferNotReady — Linux never prestarts
	 * isoc; StartXfer uses the UF from that event (gadget.c).
	 */
	const bool hw_in =
		USB_EP_DIR_IS_IN(ep_data->cfg.addr) &&
		(CONFIG_UDC_DWC3_HW_IN_EP_MASK &
		 BIT(USB_EP_GET_IDX(ep_data->cfg.addr))) != 0U;
	const bool hw_in_isoc =
		hw_in &&
		(ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) ==
			USB_EP_TYPE_ISO;

	if (hw_in_isoc) {
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN;
		printk("dwc3: HW_IN isoc ep=0x%02x: XferNotReady enabled (Linux isoc)\n",
		       ep_data->cfg.addr);
	} else if (!hw_in) {
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN;
		/* Linux/lattice-ref: EP0 STATUS is armed only on XferNotReady.
		 * next_ctrl gates bi->status on ep0_status_nrd; without this
		 * bit the core never emits that event and SET_ADDRESS hangs.
		 */
		if ((ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) ==
		    USB_EP_TYPE_CONTROL) {
			param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN;
		}
#if defined(CONFIG_UDC_DWC3_OUT_NOTREADY_RETAKE)
		if (!USB_EP_DIR_IS_IN(ep_data->cfg.addr) &&
		    (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) ==
			    USB_EP_TYPE_BULK) {
			param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN;
		}
#endif
	} else {
		printk("dwc3: HW_IN ep=0x%02x: DEPEVT disabled (event-ring guard)\n",
		       ep_data->cfg.addr);
	}

	/* This is the usb protocol endpoint number, but the data encoding
	 * we chose for physical endpoint number is the same as this
	 * register
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

	LOG_DBG("DepXferConfig: ep=0x%02x", ep_data->cfg.addr);

	reg = FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK, 1);
	sys_write32(reg, base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPXFERCFG);
}

static void udc_dwc3_depcmd_set_stall(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	if (udc_dwc3_ep_refuse_if_hw_owned(ep_data, "SetStall")) {
		return;
	}

	LOG_WRN("DepSetStall: ep=0x%02x", ep_data->cfg.addr);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPSETSTALL);
}

static void udc_dwc3_depcmd_clear_stall(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	if (udc_dwc3_ep_refuse_if_hw_owned(ep_data, "ClearStall")) {
		return;
	}

	LOG_INF("DepClearStall ep=0x%02x", ep_data->cfg.addr);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPCSTALL);
}

static uint32_t udc_dwc3_depcmd_reg(const struct device *const dev,
				    const uint32_t addr, const uint32_t cmd,
				    const bool halt)
{
#if defined(CONFIG_UDC_DWC3_DEPCMD_MAILBOX)
	ARG_UNUSED(halt);
	return udc_dwc3_depcmd_mailbox(dev, addr, cmd);
#else
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;
	uint32_t spins = 0U;
	unsigned int key = irq_lock();
	const int pause_rc = halt ? udc_dwc3_uvcmgr_pause_doorbell() : 0;

	udc_dwc3_iebm_depcmd_enter();
	udc_dwc3_depcmd_hw_doorbell_sync(base);
	sys_write32(cmd | UDC_DWC3_DEPCMD_CMDACT, base + addr);
	irq_unlock(key);

	do {
		reg = sys_read32(base + addr);
		if (++spins > 1000000U) {
			printk("DEPCMD 0x%03x never completed: cmd 0x%08x reg 0x%08x\n",
			       addr, cmd, reg);
			break;
		}
	} while ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0);

	if (halt) {
		key = irq_lock();
		udc_dwc3_uvcmgr_resume_doorbell(pause_rc == 1);
		irq_unlock(key);
	}

	udc_dwc3_iebm_depcmd_done(dev);

	return reg;
#endif
}

static void udc_dwc3_depcmd_start_xfer_trb(const struct device *const dev,
					   struct udc_dwc3_ep_data *const ep_data,
					   volatile struct udc_dwc3_trb *const start)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t addr = UDC_DWC3_DEPCMD(ep_data->epn);
	const bool isoc = (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) ==
			  USB_EP_TYPE_ISO;
	uint32_t reg;

	if (udc_dwc3_ep_refuse_if_hw_owned(ep_data, "StartXfer")) {
		return;
	}
	if (!isoc && USB_EP_GET_IDX(ep_data->cfg.addr) != 0U &&
	    ep_data->xfer_active && ep_data->xferrscidx != 0U) {
		/* Already started on a non-control EP. A second StartXfer
		 * re-arms cpuQuiet and does not make Soft-IP re-fetch a
		 * parked HWO TRB. EP0 must still re-StartXfer for setup
		 * (81414ea6 skipped that → setup_pend=0 / GET_DESCRIPTOR
		 * -110 on an otherwise good snoop bit). */
		return;
	}

	/*
	 * REMOTEWAKEUP from U0 is illegal and can drop SS to Inactive.  Only
	 * request it when the link is already in a low-power state.
	 */
	reg = sys_read32(base + UDC_DWC3_DSTS);
	if ((reg & UDC_DWC3_DSTS_CONNECTSPD_MASK) == UDC_DWC3_DSTS_CONNECTSPD_SS) {
		const uint32_t lnkst = reg & UDC_DWC3_DSTS_USBLNKST_MASK;

		if (lnkst == UDC_DWC3_DSTS_USBLNKST_USB3_U1 ||
		    lnkst == UDC_DWC3_DSTS_USBLNKST_USB3_U2 ||
		    lnkst == UDC_DWC3_DSTS_USBLNKST_USB3_U3) {
			reg = sys_read32(base + UDC_DWC3_DCTL);
			reg &= ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;
			reg |= UDC_DWC3_DCTL_ULSTCHNGREQ_REMOTEWAKEUP;
			sys_write32(reg, base + UDC_DWC3_DCTL);
		}
	}

	sys_write32(HI32((uintptr_t)start), base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(LO32((uintptr_t)start), base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));
#if defined(CONFIG_UDC_DWC3_DEPCMD_MAILBOX)
	if (USB_EP_GET_IDX(ep_data->cfg.addr) != 0U) {
		udc_dwc3_mbx_trb = LO32((uintptr_t)start);
	}
#endif
	/* Same-master fence: TRB + PAR must land before mailbox CMDACT. */
	udc_dwc3_trb_fence(start);
	{
		const uint32_t par0 = sys_read32(base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
		const uint32_t par1 = sys_read32(base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

		if (par1 != LO32((uintptr_t)start) || par0 != HI32((uintptr_t)start)) {
			printk("dwc3: PAR fence mismatch ep=0x%02x par=0x%08x/0x%08x trb=%p\n",
			       ep_data->cfg.addr, par0, par1, (void *)start);
		}
	}

	if (!isoc) {
		/*
		 * Check the command status: a StartXfer that fails (no
		 * transfer resource, CMDERR) must not leave xfer_active set,
		 * otherwise the RTL would be handed a dead xferrscidx.
		 */
		reg = udc_dwc3_depcmd_reg(dev, addr, UDC_DWC3_DEPCMD_DEPSTRTXFER, true);
		udc_dwc3_acm_note_cmd(ep_data->cfg.addr, UDC_DWC3_DEPCMD_DEPSTRTXFER, reg);
		if ((reg & UDC_DWC3_DEPCMD_STATUS_MASK) != UDC_DWC3_DEPCMD_STATUS_OK) {
			printk("dwc3: StartXfer FAILED ep=0x%02x reg=0x%08x (no resource?)\n",
			       ep_data->cfg.addr, reg);
			ep_data->xferrscidx = 0;
			ep_data->xfer_active = false;
			return;
		}
		ep_data->xferrscidx = FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);
		ep_data->xfer_active = true;
		LOG_DBG("DepStartXfer done ep=0x%02x xferrscidx=0x%x",
			ep_data->cfg.addr, ep_data->xferrscidx);
		return;
	}

	/*
	 * Isochronous: do not StartXfer from ep_enable. Linux gadget.c:
	 * "Isochronous endpoints should NEVER be prestarted. We must wait
	 * for a XferNotReady event". lattice_usb23_isoc_arm() + NRDY handler
	 * perform StartXfer with EventParam UF.
	 */
	ep_data->xfer_active = false;
	ep_data->xferrscidx = 0;
	ARG_UNUSED(reg);
}

static void udc_dwc3_depcmd_start_xfer(const struct device *const dev,
				       struct udc_dwc3_ep_data *const ep_data)
{
	udc_dwc3_depcmd_start_xfer_trb(dev, ep_data, ep_data->trb_buf);
}

/*
 * Linux __dwc3_gadget_start_isoc: EventParam UF can be stale by the time we
 * run; refresh low 14 bits from DSTS.SOFFN and keep EventParam[15:14].
 */
static uint32_t udc_dwc3_isoc_refresh_uf(const struct device *const dev,
					 const uint32_t event_uf)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t soffn =
		FIELD_GET(UDC_DWC3_DSTS_SOFFN_MASK, sys_read32(base + UDC_DWC3_DSTS));
	const bool rollover = soffn < (event_uf & 0x3fffU);
	uint32_t uf = (event_uf & ~0x3fffU) | soffn;

	if (rollover) {
		uf = (uf + BIT(14)) & 0xffffU;
	}
	return uf;
}

static int udc_dwc3_isoc_start_with_uf(const struct device *const dev,
				       struct udc_dwc3_ep_data *const ep_data,
				       const uint32_t event_uf)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t addr = UDC_DWC3_DEPCMD(ep_data->epn);
	/* Linux dep->interval = 1 << (bInterval - 1) — period in microframes. */
	const uint8_t binterval = ep_data->cfg.interval ? ep_data->cfg.interval : 1U;
	const uint32_t interval = 1U << (MIN(binterval, 14U) - 1U);
	const uint32_t cur_uf = udc_dwc3_isoc_refresh_uf(dev, event_uf);
	uint32_t reg = 0;

	sys_write32(HI32((uintptr_t)ep_data->trb_buf),
		    base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(LO32((uintptr_t)ep_data->trb_buf),
		    base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));
#if defined(CONFIG_UDC_DWC3_DEPCMD_MAILBOX)
	udc_dwc3_mbx_trb = LO32((uintptr_t)ep_data->trb_buf);
#endif

	/*
	 * Linux: uf = ALIGN(cur + interval*(i+1)). Soft-IP needs lead after
	 * NRDY enables it — keep UF_AHEAD ≥ a few SI.
	 */
	for (uint32_t i = 0U; i < UDC_DWC3_ISOC_START_RETRIES; i++) {
		const uint32_t uf =
			(cur_uf + interval * (UDC_DWC3_ISOC_START_UF_AHEAD + i * 8U)) &
			0xffffU;
		const uint32_t cmd = UDC_DWC3_DEPCMD_DEPSTRTXFER |
				     FIELD_PREP(UDC_DWC3_DEPCMD_CMDPARAM_MASK, uf);

		reg = udc_dwc3_depcmd_reg(dev, addr, cmd, true);
		if ((reg & UDC_DWC3_DEPCMD_STATUS_MASK) == UDC_DWC3_DEPCMD_STATUS_OK) {
			ep_data->xferrscidx =
				FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);
			ep_data->xfer_active = true;
			printk("dwc3: isoc StartXfer (NRDY) ep=0x%02x evt=%u cur=%u uf=%u idx=0x%x\n",
			       ep_data->cfg.addr, event_uf, cur_uf, uf,
			       ep_data->xferrscidx);
			return 0;
		}
		if ((reg & UDC_DWC3_DEPCMD_STATUS_MASK) !=
		    UDC_DWC3_DEPCMD_STATUS_BUSEXPIRY) {
			printk("dwc3: isoc StartXfer hard-fail ep=0x%02x uf=%u reg=0x%08x\n",
			       ep_data->cfg.addr, uf, reg);
			break;
		}
	}

	printk("dwc3: isoc StartXfer FAILED ep=0x%02x evt=%u cur=%u last=0x%08x\n",
	       ep_data->cfg.addr, event_uf, cur_uf, reg);
	ep_data->xferrscidx = 0;
	ep_data->xfer_active = false;
	return -EIO;
}

/*
 * Wipe Soft-IP data TRBs but keep the Link TRB that closes the ring. Used on
 * STREAMON/OFF so a restarted host session never consumes stale HWO pages.
 */
static void udc_dwc3_isoc_scrub_ring(struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;
	const uint32_t link = CONFIG_UDC_DWC3_TRB_NUM - 1U;

	if (trb == NULL) {
		return;
	}

	for (uint32_t i = 0U; i < link; i++) {
		trb[i].ctrl = 0;
		trb[i].status = 0;
		trb[i].addr_lo = 0;
		trb[i].addr_hi = 0;
	}

	trb[link].ctrl = UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB | UDC_DWC3_TRB_CTRL_HWO;
	trb[link].status = 0;
	trb[link].addr_lo = LO32((uintptr_t)ep_data->trb_buf);
	trb[link].addr_hi = HI32((uintptr_t)ep_data->trb_buf);

	ep_data->head = 0;
	ep_data->tail = 0;
	ep_data->full = false;
}

int lattice_usb23_isoc_arm(const struct device *dev, uint8_t ep_addr,
			   uintptr_t softip_base, uint32_t trb_addr,
			   uint32_t depcmd_addr)
{
	struct udc_dwc3_ep_data *const ep_data =
		(struct udc_dwc3_ep_data *)udc_get_ep_cfg(dev, ep_addr);

	if (ep_data == NULL || softip_base == 0U) {
		return -EINVAL;
	}
	if ((ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) != USB_EP_TYPE_ISO) {
		return -EINVAL;
	}

	/*
	 * Do NOT scrub the ring here (leaks Soft-IP pages). Enable Soft-IP now
	 * so frame-sync + HWO fill finish before the host's first XferNotReady.
	 * StartXfer stays deferred (Linux isoc model); only Soft-IP runs early.
	 * First-ffplay used to race sync (~33–50 ms) against the StartXfer UF
	 * window and show vq=0 until the second open.
	 */
	sys_write32(0U, softip_base + UDC_DWC3_SOFTIP_CTRL_OFF);

	udc_dwc3_isoc_arm.pending = true;
	udc_dwc3_isoc_arm.softip_base = softip_base;
	udc_dwc3_isoc_arm.trb_addr = trb_addr;
	udc_dwc3_isoc_arm.depcmd_addr = depcmd_addr;
	udc_dwc3_isoc_arm.ep_addr = ep_addr;
	ep_data->xfer_active = false;
	ep_data->xferrscidx = 0;

	sys_write32(UDC_DWC3_DEPCMD_DEPUPDXFER,
		    softip_base + UDC_DWC3_SOFTIP_DOORBELL_DATA_OFF);
	sys_write32(UDC_DWC3_SOFTIP_CTRL_ENABLE | UDC_DWC3_SOFTIP_CTRL_SHOULD_CONT,
		    softip_base + UDC_DWC3_SOFTIP_CTRL_OFF);

	/* Frame sync ≤50 ms + first SI fill. */
	for (uint32_t ms = 0U; ms < 80U; ms++) {
		if ((ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
			break;
		}
		k_msleep(1);
	}
	/* Prefer a second HWO so StartXfer is not one-TRB-starved. */
	for (uint32_t ms = 0U; ms < 30U; ms++) {
		if ((ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U &&
		    (ep_data->trb_buf[1].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
			break;
		}
		k_msleep(1);
	}

	printk("dwc3: isoc armed+primed ep=0x%02x hwo0=%u hwo1=%u — wait NRDY\n",
	       ep_addr,
	       (ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U,
	       (ep_data->trb_buf[1].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U);
	return 0;
}

void lattice_usb23_isoc_disarm(const struct device *dev, uint8_t ep_addr)
{
	struct udc_dwc3_ep_data *const ep_data =
		(struct udc_dwc3_ep_data *)udc_get_ep_cfg(dev, ep_addr);
	const uintptr_t softip = udc_dwc3_isoc_arm.softip_base;
	const uint32_t xferrscidx = (ep_data != NULL) ? ep_data->xferrscidx : 0U;
	const bool xfer_active = (ep_data != NULL) && ep_data->xfer_active;

	udc_dwc3_isoc_arm.pending = false;

	/*
	 * Halt Soft-IP before EndXfer so it cannot race DEPCMD. Soft-IP
	 * enable-fall resets FID/EOF and walks the TRB ring to free pages
	 * still referenced by uncleared HWO slots (see TRBRamSink flush).
	 */
	if (softip != 0U) {
		sys_write32(0U, softip + UDC_DWC3_SOFTIP_CTRL_OFF);
		/* ~100 µs @ 75 MHz Soft-IP clock is plenty for ring reclaim. */
		k_busy_wait(100);
	}

	if (ep_data != NULL && xfer_active && xferrscidx != 0U) {
		(void)udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
				      UDC_DWC3_DEPCMD_DEPENDXFER |
					      UDC_DWC3_DEPCMD_HIPRI_FORCERM |
					      FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK,
							 xferrscidx));
		printk("dwc3: isoc EndXfer(ForceRM) ep=0x%02x idx=0x%x\n",
		       ep_addr, xferrscidx);
	}

	if (ep_data != NULL) {
		/* Soft-IP flush already cleared slots; keep Link TRB healthy. */
		udc_dwc3_isoc_scrub_ring(ep_data);
		ep_data->xfer_active = false;
		ep_data->xferrscidx = 0;
	}

	if (ep_addr == udc_dwc3_isoc_arm.ep_addr) {
		udc_dwc3_isoc_arm.ep_addr = 0;
	}
}

static void udc_dwc3_on_isoc_xfer_not_ready(const struct device *const dev,
					      const uint32_t evt_raw)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt_raw);
	struct udc_dwc3_ep_data *const ep_data =
		(epn & 1) ? &cfg->ep_data_in[epn >> 1] : &cfg->ep_data_out[epn >> 1];
	const uint32_t cur_uf = FIELD_GET(UDC_DWC3_DEPEVT_PARAM_MASK, evt_raw);
	uint32_t depupdxfer;

	if (ep_data == NULL ||
	    (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) != USB_EP_TYPE_ISO) {
		return;
	}
	if (!USB_EP_DIR_IS_IN(ep_data->cfg.addr) ||
	    (CONFIG_UDC_DWC3_HW_IN_EP_MASK &
	     BIT(USB_EP_GET_IDX(ep_data->cfg.addr))) == 0U) {
		return;
	}
	if (!udc_dwc3_isoc_arm.pending ||
	    udc_dwc3_isoc_arm.ep_addr != ep_data->cfg.addr) {
		return;
	}
	if (ep_data->xfer_active) {
		return;
	}

	/*
	 * Soft-IP was already enabled and primed in isoc_arm (frame sync + HWO).
	 * Re-assert ENABLE without a disable pulse (that would re-enter sync and
	 * race the StartXfer UF again). Wait briefly if prime was slow.
	 */
	sys_write32(UDC_DWC3_SOFTIP_CTRL_ENABLE | UDC_DWC3_SOFTIP_CTRL_SHOULD_CONT,
		    udc_dwc3_isoc_arm.softip_base + UDC_DWC3_SOFTIP_CTRL_OFF);
	for (uint32_t i = 0U; i < UDC_DWC3_ISOC_PRIME_SPINS; i++) {
		if ((ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
			break;
		}
	}

	if (udc_dwc3_isoc_start_with_uf(dev, ep_data, cur_uf) != 0) {
		sys_write32(0U, udc_dwc3_isoc_arm.softip_base + UDC_DWC3_SOFTIP_CTRL_OFF);
		return;
	}

	depupdxfer = UDC_DWC3_DEPCMD_DEPUPDXFER |
		     FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);
	sys_write32(depupdxfer,
		    udc_dwc3_isoc_arm.softip_base + UDC_DWC3_SOFTIP_DOORBELL_DATA_OFF);
	sys_write32(UDC_DWC3_SOFTIP_CTRL_ENABLE | UDC_DWC3_SOFTIP_CTRL_SHOULD_CONT,
		    udc_dwc3_isoc_arm.softip_base + UDC_DWC3_SOFTIP_CTRL_OFF);
	udc_dwc3_isoc_arm.pending = false;

	printk("dwc3: isoc Soft-IP primed+StartXfer ep=0x%02x depupdxfer=0x%08x hwo0=%u\n",
	       ep_data->cfg.addr, depupdxfer,
	       (ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U);
}

static void udc_dwc3_depcmd_update_xfer(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t addr = UDC_DWC3_DEPCMD(ep_data->epn);
	uint32_t flags = 0;

	if (udc_dwc3_ep_refuse_if_hw_owned(ep_data, "UpdateXfer")) {
		return;
	}

	flags |= UDC_DWC3_DEPCMD_DEPUPDXFER;
	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);

	if (ep_data->trb_buf != NULL) {
		udc_dwc3_trb_fence(&ep_data->trb_buf[ep_data->tail]);
#if defined(CONFIG_UDC_DWC3_DEPCMD_MAILBOX)
		if (USB_EP_GET_IDX(ep_data->cfg.addr) != 0U) {
			udc_dwc3_mbx_trb = LO32((uintptr_t)&ep_data->trb_buf[ep_data->tail]);
		}
#endif
	}

	/*
	 * Waited UpdateXfer (CMDACT=1) so the CPU has a completion the UsbMgr
	 * is excluded across. A No-Response write (CMDACT=0) left the shared
	 * command engine's busy window unobservable, and resume-before-wait
	 * let the RTL ring into that window. Use depcmd_reg so ACM CMDSTATUS
	 * is visible (depcmd_ex only returns XFERRSCIDX).
	 */
	{
		const uint32_t reg = udc_dwc3_depcmd_reg(dev, addr, flags, true);

		udc_dwc3_acm_note_cmd(ep_data->cfg.addr, flags, reg);
	}

	LOG_DBG("DepUpdateXfer done ep=0x%02x addr=0x%08x data=0x%08x",
		ep_data->cfg.addr, addr, flags);
}

static void udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags)
{
	if (udc_dwc3_ep_refuse_if_hw_owned(ep_data, "EndXfer")) {
		return;
	}

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);
	flags |= UDC_DWC3_DEPCMD_DEPENDXFER;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	LOG_DBG("DepEndXfer done ep=0x%02x", ep_data->cfg.addr);

	ep_data->head = ep_data->tail = 0;
	ep_data->xfer_active = false;
}

/*
 * DEPSTARTCFG allocates the pool of transfer resources the endpoints draw on.
 * Issue rsc_idx=0 after reset (control), and rsc_idx=2 once when the first
 * non-control endpoint is enabled for the selected configuration.
 */
static void udc_dwc3_depcmd_start_config(const struct device *const dev,
					 struct udc_dwc3_ep_data *const ep_data,
					 const uint32_t rsc_idx)
{
	uint32_t flags = 0;

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, rsc_idx);
	flags |= UDC_DWC3_DEPCMD_DEPSTARTCFG;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	LOG_INF("DepStartConfig done ep=0x%02x rsc_idx=%u", ep_data->cfg.addr, rsc_idx);
}

/*
 * Return every buffer still on a CPU-managed ring to its class and reset the
 * ring bookkeeping. Runs after EndXfer on dequeue / disable.
 *
 * Before this, EndXfer zeroed head/tail but left net_buf[] slots, `full` and
 * `total` behind. After a bus reset the class re-armed on a ring that still
 * held its pre-reset buffers: the OUT completion poll "retired" a stale slot,
 * tail ran ahead of the real TRBs, and ACM came back with one byte per
 * transfer and a parked IN (0x82 nudge/backoff) while video was fine because
 * the handoff path scrubs its own ring. The class-side accounting (ACM
 * rx_outstanding, TX busy) is corrected by the -ECONNABORTED completions,
 * same as udc_ep_cancel_queued() does for buffers that never reached the ring.
 */
static void udc_dwc3_ring_flush(const struct device *const dev,
				struct udc_dwc3_ep_data *const ep_data)
{
	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0U || ep_data->trb_buf == NULL) {
		return;
	}

	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
		struct net_buf *const buf = ep_data->net_buf[i];

		if (buf == NULL) {
			continue;
		}
		ep_data->net_buf[i] = NULL;
#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
		if (ep_data->cfg.addr == 0x85) {
			struct udc_dwc3_data *const priv = udc_get_private(dev);

			/*
			 * Fast-lane pages are static (pool_id=0xff). unref
			 * walks net_buf_pool_get(0xff) and CPU-exceptions
			 * right after leftover EndXfer.
			 */
			if (priv->iebm_complete_cb != NULL &&
			    priv->iebm_complete_ep == ep_data->cfg.addr) {
				priv->iebm_complete_cb(dev, ep_data->cfg.addr,
						       priv->iebm_complete_user);
			}
			continue;
		}
#endif
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	udc_dwc3_isoc_scrub_ring(ep_data);
	ep_data->total = 0;
}

/*
 * Hardware-owned endpoint handoff
 *
 * The UsbMgr RTL is the TRB engine for video: it fills the ring and rings
 * DEPCMD(UpdateXfer) itself. Zephyr configures the endpoint, starts one
 * transfer resource, and then hands the endpoint over. From that point until
 * STREAMOFF the UDC must not touch the ring or the doorbell of that endpoint
 * (see udc_dwc3_ep_refuse_if_hw_owned). Ownership comes back either through
 * lattice_usb23_ep_take_from_rtl() (orderly STREAMOFF) or a UDC-side revoke
 * (USB reset, disconnect, ep_disable, host CLEAR_FEATURE(HALT) on the bulk
 * video endpoint, which is how Linux/Windows uvcvideo signal STREAMOFF).
 */

static void udc_dwc3_hw_owned_notify(const struct device *const dev,
				     const uint8_t ep_addr, const char *const reason)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	for (size_t i = 0; i < ARRAY_SIZE(priv->hw_owner); i++) {
		if (priv->hw_owner[i].cb != NULL) {
			priv->hw_owner[i].cb(dev, ep_addr, reason, priv->hw_owner[i].user);
		}
	}
}

/*
 * Leave the ring empty and (for bulk) with a live transfer resource, so the
 * next give_to_rtl() hands the RTL a usable xferrscidx. Isochronous stays
 * deferred to XferNotReady (lattice_usb23_isoc_arm path).
 */
static void udc_dwc3_hw_owned_rearm(const struct device *const dev,
				    struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const bool isoc = (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) ==
			  USB_EP_TYPE_ISO;

	udc_dwc3_isoc_scrub_ring(ep_data);

	if (isoc || !ep_data->cfg.stat.enabled ||
	    (sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) == 0U) {
		return;
	}

	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

/* Clear ownership, end the RTL's transfer and leave the ring clean. */
static void udc_dwc3_hw_owned_release(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data,
				      const bool end_xfer)
{
	unsigned int key = irq_lock();

	ep_data->hw_owned = false;
	irq_unlock(key);

	if (end_xfer && ep_data->xfer_active && ep_data->xferrscidx != 0U) {
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	}

	ep_data->xfer_active = false;
	ep_data->xferrscidx = 0;
	udc_dwc3_isoc_scrub_ring(ep_data);
}

/*
 * UDC-initiated revoke. The RTL may still be ringing the doorbell, so stop
 * UsbMgr first (same register isoc_disarm uses), then release. The owner is
 * told afterwards so it can stop the sensor pipeline from its own context.
 */
static void udc_dwc3_hw_owned_revoke(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     const char *const reason, const bool end_xfer)
{
	if (!ep_data->hw_owned) {
		return;
	}

	printk("dwc3: ep 0x%02x <- RTL forced revoke (%s) evts=%u refused=%u\n",
	       ep_data->cfg.addr, reason, ep_data->hw_owned_evts, ep_data->hw_owned_refused);

#if CONFIG_UDC_DWC3_HW_IN_EP_MASK != 0
	/* Stop only the UsbMgr that drives this endpoint; the other camera keeps streaming */
	if (ep_data->hw_mgr_base != 0U) {
		sys_write32(0U, ep_data->hw_mgr_base + UDC_DWC3_UVCMGR_CONTROL_STATUS);
		/* ~100 us at the Soft-IP clock is enough for an in-flight doorbell */
		k_busy_wait(100);
	}
#endif

	udc_dwc3_hw_owned_release(dev, ep_data, end_xfer);
	udc_dwc3_hw_owned_notify(dev, ep_data->cfg.addr, reason);
}

static void udc_dwc3_hw_owned_revoke_all(const struct device *const dev,
					 const char *const reason)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	for (int i = 1; i < cfg->num_in_eps; i++) {
		/* Reset/disconnect kill the resource pool: no EndXfer needed */
		udc_dwc3_hw_owned_revoke(dev, &cfg->ep_data_in[i], reason, false);
	}
}

int lattice_usb23_ep_give_to_rtl(const struct device *dev, uint8_t ep_addr,
				 uintptr_t mgr_base, uint32_t *depupdxfer)
{
	struct udc_dwc3_ep_data *const ep_data =
		(struct udc_dwc3_ep_data *)udc_get_ep_cfg(dev, ep_addr);
	const bool isoc = ep_data != NULL &&
			  (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) ==
				  USB_EP_TYPE_ISO;
	unsigned int key;

	if (ep_data == NULL || !USB_EP_DIR_IS_IN(ep_addr) || USB_EP_GET_IDX(ep_addr) == 0U) {
		return -EINVAL;
	}

	/*
	 * Only endpoints whose DEPEVTs were masked at DEPCFG time may be
	 * handed off: on any other endpoint the CPU would pop the RTL's TRBs.
	 */
	if (!udc_dwc3_ep_is_hw_in(ep_data)) {
		printk("dwc3: ep 0x%02x not in UDC_DWC3_HW_IN_EP_MASK, cannot give to RTL\n",
		       ep_addr);
		return -EACCES;
	}

	if (!ep_data->cfg.stat.enabled) {
		return -ENODEV;
	}

	if (ep_data->hw_owned) {
		return -EALREADY;
	}

	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
		if (ep_data->net_buf[i] != NULL) {
			printk("dwc3: ep 0x%02x has CPU buffers queued, cannot give to RTL\n",
			       ep_addr);
			return -EBUSY;
		}
	}

	if (!isoc && !ep_data->xfer_active) {
		udc_dwc3_hw_owned_rearm(dev, ep_data);
		if (!ep_data->xfer_active) {
			printk("dwc3: ep 0x%02x has no live transfer resource\n", ep_addr);
			return -EIO;
		}
	}

	key = irq_lock();
	ep_data->hw_owned = true;
	ep_data->hw_mgr_base = (mm_reg_t)mgr_base;
	ep_data->hw_owned_evts = 0;
	irq_unlock(key);

	if (depupdxfer != NULL) {
		*depupdxfer = UDC_DWC3_DEPCMD_DEPUPDXFER |
			      FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);
	}

	printk("dwc3: ep 0x%02x -> RTL (xferrscidx=0x%x)\n", ep_addr, ep_data->xferrscidx);

	return 0;
}

int lattice_usb23_ep_take_from_rtl(const struct device *dev, uint8_t ep_addr)
{
	struct udc_dwc3_ep_data *const ep_data =
		(struct udc_dwc3_ep_data *)udc_get_ep_cfg(dev, ep_addr);

	if (ep_data == NULL) {
		return -EINVAL;
	}

	if (!ep_data->hw_owned) {
		return -EALREADY;
	}

	printk("dwc3: ep 0x%02x <- RTL (stream off) evts=%u refused=%u\n", ep_addr,
	       ep_data->hw_owned_evts, ep_data->hw_owned_refused);

	udc_dwc3_hw_owned_release(dev, ep_data, true);
	udc_dwc3_hw_owned_rearm(dev, ep_data);

	return 0;
}

bool lattice_usb23_ep_is_rtl_owned(const struct device *dev, uint8_t ep_addr)
{
	const struct udc_dwc3_ep_data *const ep_data =
		(const struct udc_dwc3_ep_data *)udc_get_ep_cfg(dev, ep_addr);

	return ep_data != NULL && ep_data->hw_owned;
}

int lattice_usb23_set_hw_owner_cb(const struct device *dev,
				  void (*cb)(const struct device *dev, uint8_t ep_addr,
					     const char *reason, void *user),
				  void *user)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	for (size_t i = 0; i < ARRAY_SIZE(priv->hw_owner); i++) {
		if (priv->hw_owner[i].cb == NULL || priv->hw_owner[i].user == user) {
			priv->hw_owner[i].cb = cb;
			priv->hw_owner[i].user = user;
			return 0;
		}
	}

	return -ENOMEM;
}

/* One EndXfer to free the enable-time empty StartXfer. Later 0x85
 * pages keep the transfer resource and use UpdateXfer.
 */
static bool iebm_uvc_unparked;

int lattice_usb23_set_iebm_complete_cb(const struct device *dev, uint8_t ep_addr,
				       void (*cb)(const struct device *dev, uint8_t ep_addr,
						  void *user),
				       void *user)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	priv->iebm_complete_cb = cb;
	priv->iebm_complete_user = user;
	priv->iebm_complete_ep = ep_addr;
	if (cb == NULL) {
		iebm_uvc_unparked = false;
#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
		{
			unsigned int key = irq_lock();

			iebm_fast.ep = NULL;
			iebm_fast.refill = NULL;
			iebm_fast.user = NULL;
			iebm_fast.kick_pending = false;
			iebm_fast.swq_w = 0;
			iebm_fast.swq_r = 0;
			irq_unlock(key);
		}
#endif
	} else {
#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
		unsigned int key = irq_lock();

		iebm_fast.ep = (struct udc_dwc3_ep_data *)udc_get_ep_cfg(dev, ep_addr);
		iebm_fast.kick_pending = false;
		iebm_fast.swq_w = 0;
		iebm_fast.swq_r = 0;
		irq_unlock(key);
#endif
	}
	return 0;
}

#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
int lattice_usb23_set_iebm_refill_cb(const struct device *dev,
				     int (*refill)(const struct device *dev, void *user),
				     void *user)
{
	unsigned int key = irq_lock();

	ARG_UNUSED(dev);
	iebm_fast.refill = refill;
	iebm_fast.user = user;
	irq_unlock(key);
	return 0;
}

static void udc_dwc3_iebm_depcmd_enter(void)
{
	iebm_fast.depcmd_active = true;
}

/* Interrupts locked by the caller. */
static ALWAYS_INLINE bool udc_dwc3_iebm_fast_ready_locked(void)
{
	struct udc_dwc3_ep_data *const ep_data = iebm_fast.ep;

	if (ep_data == NULL || iebm_fast.refill == NULL || ep_data->trb_buf == NULL) {
		return false;
	}
	if (ep_data->cfg.stat.halted || !ep_data->cfg.stat.enabled || ep_data->hw_owned) {
		return false;
	}
	/* Never interleave with a sysworkq buffer: keep page order. Inline
	 * list peek: caller holds irq_lock, k_fifo_peek_head is cold XIP code.
	 */
	return sys_sflist_peek_head(&ep_data->cfg.fifo._queue.data_q) == NULL;
}

bool lattice_usb23_iebm_fast_ready(const struct device *dev)
{
	unsigned int key = irq_lock();
	bool ready;

	ARG_UNUSED(dev);
	ready = udc_dwc3_iebm_fast_ready_locked();
	irq_unlock(key);
	return ready;
}

/* Interrupts locked by the caller. One CmdAct=1 UpdateXfer on the video EP. */
__ramfunc static void udc_dwc3_iebm_fast_kick_locked(const struct device *const dev)
{
	struct udc_dwc3_ep_data *const ep_data = iebm_fast.ep;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t addr;
	uint32_t cmd;
	uint32_t reg;
	uint32_t spins = 0U;

	if (ep_data == NULL || !ep_data->xfer_active) {
		return;
	}
	if (iebm_fast.depcmd_active) {
		iebm_fast.kick_pending = true;
		iebm_fast.kicks_deferred++;
		return;
	}

	addr = UDC_DWC3_DEPCMD(ep_data->epn);
	cmd = UDC_DWC3_DEPCMD_DEPUPDXFER |
	      FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx) |
	      UDC_DWC3_DEPCMD_CMDACT;

	iebm_fast.depcmd_active = true;
	sys_write32(cmd, base + addr);
	do {
		reg = sys_read32(base + addr);
		if (++spins > 200000U) {
			iebm_fast.kick_err++;
			break;
		}
	} while ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0U);
	iebm_fast.depcmd_active = false;
	iebm_fast.kicks++;
	if (spins > iebm_fast.kick_spin_max) {
		iebm_fast.kick_spin_max = spins;
	}
	if ((reg & UDC_DWC3_DEPCMD_STATUS_MASK) != UDC_DWC3_DEPCMD_STATUS_OK) {
		iebm_fast.kick_err++;
	}
}

static void udc_dwc3_iebm_depcmd_done(const struct device *dev)
{
	unsigned int key = irq_lock();

	iebm_fast.depcmd_active = false;
	if (iebm_fast.kick_pending) {
		iebm_fast.kick_pending = false;
		udc_dwc3_iebm_fast_kick_locked(dev);
	}
	irq_unlock(key);
}

void lattice_usb23_iebm_kick(const struct device *dev)
{
	unsigned int key = irq_lock();

	udc_dwc3_iebm_fast_kick_locked(dev);
	irq_unlock(key);
}

/*
 * Push one IEBM page TRB. Interrupts locked by the caller (refill hook).
 * IOC on every page: the ISR consumes the DEPEVT and refills, which is the
 * Lattice-ref cadence.
 *
 * @p eof breaks the TRB chain, which is what puts a short packet on the wire
 * and ends the UVC payload. Everything else chains, so a page that is not a
 * multiple of MPS (IEBM flushes an 8 KiB+16 page mid-frame) has its remainder
 * packetized together with the next page instead of closing the frame early.
 */
__ramfunc int lattice_usb23_iebm_fast_push(const struct device *dev, struct net_buf *buf,
					   bool eof)
{
	struct udc_dwc3_ep_data *const ep_data = iebm_fast.ep;
	uint32_t ctrl = UDC_DWC3_TRB_CTRL_HWO | UDC_DWC3_TRB_CTRL_CSP |
			UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL | UDC_DWC3_TRB_CTRL_IOC;
	uint32_t slot;

	if (ep_data == NULL || ep_data->full || buf == NULL || buf->len == 0U) {
		return -EBUSY;
	}

	/*
	 * Enable-time StartXfer parks an empty video ring. One EndXfer frees
	 * that stale resource, then the first page gets its own StartXfer.
	 * Same sequence as trb_bulk(); here it runs with interrupts locked.
	 */
	if (ep_data->xfer_active && !iebm_uvc_unparked &&
	    udc_dwc3_ring_data_hwo_mask(ep_data) == 0U) {
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		iebm_uvc_unparked = true;
	}

	ep_data->total += buf->len;
	if (!eof) {
		ctrl |= UDC_DWC3_TRB_CTRL_CHN;
	} else {
		ep_data->total = 0;
	}

	slot = ep_data->head;
	udc_dwc3_push_trb(dev, ep_data, buf, ctrl);
	iebm_fast.pushed++;

	if (!ep_data->xfer_active) {
		/* After EndXfer the resource is gone: StartXfer at this page. */
		udc_dwc3_depcmd_start_xfer_trb(dev, ep_data, &ep_data->trb_buf[slot]);
		iebm_fast.started = true;
		iebm_fast.starts++;
		/* Interrupts locked: no printk (see fast_health). */
		iebm_fast.last_start_slot = slot;
		iebm_fast.last_start_idx = ep_data->xferrscidx;
	}
	return 0;
}

/*
 * Retire every HWO=0 page at the tail (HAND_OVER via the complete cb), then
 * let the video driver push whatever IEBM has ready and ring UpdateXfer once.
 * Interrupts locked by the caller. Returns the number of pages retired.
 */
__ramfunc static int udc_dwc3_iebm_fast_service(const struct device *dev)
{
	struct udc_dwc3_ep_data *const ep_data = iebm_fast.ep;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	int retired = 0;
	uint32_t t0;
	uint32_t t1;

	if (ep_data == NULL || ep_data->trb_buf == NULL) {
		return 0;
	}

	iebm_fast.n_service++;
	t0 = k_cycle_get_32();
	while (ep_data->net_buf[ep_data->tail] != NULL) {
		volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->tail];
		struct net_buf *buf;

		if ((trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
			break;
		}
		buf = udc_dwc3_pop_trb(dev, ep_data);
		if (buf == NULL) {
			break;
		}
		if (priv->iebm_complete_cb != NULL &&
		    priv->iebm_complete_ep == ep_data->cfg.addr) {
			priv->iebm_complete_cb(dev, ep_data->cfg.addr, priv->iebm_complete_user);
		}
		/*
		 * Fast-lane net_bufs are static, owned by the video driver
		 * (tvai_uvcmgr_iebm.c) and recycled after this pop: no unref.
		 */
		retired++;
	}
	iebm_fast.retired += retired;
	t1 = k_cycle_get_32();
	iebm_fast.cyc_retire += t1 - t0;

	if (udc_dwc3_iebm_fast_ready_locked()) {
		int pushed;

		iebm_fast.started = false;
		pushed = iebm_fast.refill(dev, iebm_fast.user);
		t0 = k_cycle_get_32();
		iebm_fast.cyc_refill += t0 - t1;
		/*
		 * UpdateXfer after every batch, also right after a StartXfer:
		 * pages pushed behind the start slot may have been fetched as
		 * HWO=0 already and parked the endpoint.
		 */
		if (pushed > 0) {
			udc_dwc3_iebm_fast_kick_locked(dev);
			iebm_fast.cyc_kick += k_cycle_get_32() - t0;
		}
	}

	return retired;
}

/* Non-video ring diagnostics: why an ACM completion did not retire. */
static uint32_t udc_dwc3_nv_drop_nb_null;
static uint32_t udc_dwc3_nv_drop_hwo;
static uint32_t udc_dwc3_nv_full;
static uint32_t udc_dwc3_nv_retake;
static uint32_t udc_dwc3_nv_evt[32];

static void udc_dwc3_nv_ep_dump(const struct udc_dwc3_ep_data *ep_data)
{
	uint32_t nb = 0U;

	if (ep_data->trb_buf == NULL || !ep_data->cfg.stat.enabled) {
		return;
	}
	for (uint32_t i = 0U; i < CONFIG_UDC_DWC3_TRB_NUM - 1U; i++) {
		nb += ep_data->net_buf[i] != NULL;
	}
	printk(" %02x:xa=%u hd=%u tl=%u f=%u hwo=0x%x nb=%u", ep_data->cfg.addr,
	       ep_data->xfer_active, ep_data->head, ep_data->tail, ep_data->full,
	       udc_dwc3_ring_data_hwo_mask(ep_data), nb);
}

void lattice_usb23_iebm_fast_health(const struct device *dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	printk("nv: drop_nb=%u drop_hwo=%u full=%u retake=%u ev:",
	       udc_dwc3_nv_drop_nb_null, udc_dwc3_nv_drop_hwo, udc_dwc3_nv_full,
	       udc_dwc3_nv_retake);
	for (uint32_t i = 0U; i < 32U; i++) {
		if (udc_dwc3_nv_evt[i] != 0U) {
			printk(" %u=%u", i, udc_dwc3_nv_evt[i]);
		}
	}
	printk("\nnv-ep:");
	for (int i = 1; i < cfg->num_in_eps; i++) {
		if (cfg->ep_data_in[i].cfg.addr != 0x85) {
			udc_dwc3_nv_ep_dump(&cfg->ep_data_in[i]);
		}
	}
	for (int i = 1; i < cfg->num_out_eps; i++) {
		udc_dwc3_nv_ep_dump(&cfg->ep_data_out[i]);
	}
	printk("\n");
	printk("iebm-fast: isr=%u evts=%u ret=%u push=%u starts=%u(slot=%u idx=0x%x) kicks=%u def=%u spin_max=%u err=%u ovf=%u swq=%u full=%u\n",
	       iebm_fast.isr_calls, iebm_fast.isr_evts, iebm_fast.retired, iebm_fast.pushed,
	       iebm_fast.starts, iebm_fast.last_start_slot, iebm_fast.last_start_idx,
	       iebm_fast.kicks, iebm_fast.kicks_deferred,
	       iebm_fast.kick_spin_max, iebm_fast.kick_err, udc_dwc3_evt_errors.overflow,
	       iebm_fast.swq_push, iebm_fast.swq_full);
	printk("iebm-cyc: svc=%u retire/svc=%u refill/svc=%u kick/kick=%u evt_thr=%u cyc/evt=%u\n",
	       iebm_fast.n_service,
	       iebm_fast.n_service ? iebm_fast.cyc_retire / iebm_fast.n_service : 0U,
	       iebm_fast.n_service ? iebm_fast.cyc_refill / iebm_fast.n_service : 0U,
	       iebm_fast.kicks ? iebm_fast.cyc_kick / iebm_fast.kicks : 0U,
	       iebm_fast.n_evt_thread,
	       iebm_fast.n_evt_thread ? iebm_fast.cyc_evt_thread / iebm_fast.n_evt_thread : 0U);
}
#else /* CONFIG_UDC_DWC3_RTL_DOORBELL — ACM/RAW ring counters only */
static uint32_t udc_dwc3_nv_drop_nb_null;
static uint32_t udc_dwc3_nv_drop_hwo;
static uint32_t udc_dwc3_nv_full;
static uint32_t udc_dwc3_nv_retake;
static uint32_t udc_dwc3_nv_evt[32];
#endif /* !CONFIG_UDC_DWC3_RTL_DOORBELL */

int lattice_usb23_iebm_abort(const struct device *dev, uint8_t ep_addr)
{
#if defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
	ARG_UNUSED(dev);
	ARG_UNUSED(ep_addr);
	return 0;
#else
	struct udc_dwc3_ep_data *const ep_data =
		(struct udc_dwc3_ep_data *)udc_get_ep_cfg(dev, ep_addr);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	unsigned int key;
	uint32_t leftover = 0U;
	uint32_t spins = 0U;
	bool did_end = false;
	bool did_rearm = false;

	if (ep_data == NULL || ep_addr == 0U) {
		return -EINVAL;
	}

	key = irq_lock();
	/* Detach the ISR fast lane before touching DEPCMD on 0x85. */
	iebm_fast.ep = NULL;
	iebm_fast.refill = NULL;
	iebm_fast.user = NULL;
	iebm_fast.kick_pending = false;
	iebm_fast.swq_w = 0;
	iebm_fast.swq_r = 0;
	iebm_uvc_unparked = false;
	while ((sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn)) &
		UDC_DWC3_DEPCMD_CMDACT) != 0U) {
		if (++spins > 200000U) {
			break;
		}
	}
	iebm_fast.depcmd_active = false;

	/* One EndXfer at STREAMOFF only. Do not EndXfer ACM. */
	if (ep_data->xfer_active && ep_data->xferrscidx != 0U) {
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		did_end = true;
	}
	ep_data->xfer_active = false;
	ep_data->xferrscidx = 0;
	for (uint32_t i = 0U; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
		leftover += ep_data->net_buf[i] != NULL;
	}
	udc_dwc3_ring_flush(dev, ep_data);
	/* Match SET_CONFIG: empty StartXfer so the next STREAMON can unpark.
	 * Skip a halted EP — host STREAMOFF often CLEAR_FEATURE(HALT) first.
	 */
	if (ep_data->cfg.stat.enabled && !ep_data->cfg.stat.halted) {
		udc_dwc3_depcmd_start_xfer(dev, ep_data);
		did_rearm = true;
	}
	irq_unlock(key);
	printk("dwc3: 0x%02x leftover EndXfer=%d drop=%u rearm=%d xa=%d idx=0x%x\n",
	       ep_addr, did_end, leftover, did_rearm, ep_data->xfer_active,
	       ep_data->xferrscidx);
	return 0;
#endif
}

__ramfunc int lattice_usb23_iebm_retire(const struct device *dev, uint8_t ep_addr)
{
#if defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
	ARG_UNUSED(dev);
	ARG_UNUSED(ep_addr);
	return 0;
#else
	unsigned int key;
	int n;

	if (iebm_fast.ep == NULL || iebm_fast.ep->cfg.addr != ep_addr) {
		return 0;
	}

	key = irq_lock();
	n = udc_dwc3_iebm_fast_service(dev);
	irq_unlock(key);
	return n;
#endif
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
	const bool isoc = (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) ==
			  USB_EP_TYPE_ISO;

	LOG_DBG("Initializing normal TRB");

	/* TRB0 that prevents the transfer to be started (until it is overwritten) */
	trb[0].ctrl = 0;

	/* TRB LINK that loops the ring buffer back to the beginning */
	trb[i].ctrl = UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB | UDC_DWC3_TRB_CTRL_HWO;
	trb[i].addr_lo = LO32((uintptr_t)ep_data->trb_buf);
	trb[i].addr_hi = HI32((uintptr_t)ep_data->trb_buf);

	/*
	 * Isochronous Soft-IP: defer StartXfer until uvcmanager arms TPG +
	 * Soft-IP (lattice_usb23_isoc_start_xfer). Early StartXfer expires
	 * before the first HWO TRB exists → permanent MissedIsoc / 0 fps.
	 */
	if (isoc) {
		ep_data->xfer_active = false;
		ep_data->xferrscidx = 0;
		printk("dwc3: isoc ep=0x%02x TRB ring ready (StartXfer deferred)\n",
		       ep_data->cfg.addr);
		return;
	}

	/* Bulk/int: start the transfer now, update it later */
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

/*
 * Bounce large EP0 IN through uncached SRAM. Must be volatile: otherwise the
 * compiler can DCE memcpy/patch stores because only the DMA address is
 * consumed in C (hardware reads the payload).
 */
/*
 * 768 covers the SuperSpeed configuration descriptor of ACM + UVC + ACM
 * (~420 bytes) with the 64-byte DMA offset. Larger EP0 IN payloads fall back
 * to direct DMA from the class buffer (see the size check below). The rest of
 * the 8 KiB USB RAM goes to the UDC buffer pool.
 */
static __nocache volatile uint8_t udc_dwc3_ep0_in_bounce[768] __aligned(64);

/*
 * Lattice USB23 EP0 has been observed to deliver config blobs where a few
 * UVC VS fields are OR-corrupted on the wire (e.g. 640x480 -> 642x484,
 * wTotalLength 0x0072 -> 0x0272) even when the source buffer was correct.
 * Re-assert BA81 frame geometry and VS input-header fields in the bounce
 * buffer immediately before DMA.
 */
static void udc_dwc3_ep0_patch_uvc_config(volatile uint8_t *buf, size_t len)
{
	for (size_t i = 0; i + 13 <= len; i++) {
		uint8_t bl = buf[i];
		uint8_t ep;

		/* VS Input Header: subtype 1 with bulk IN (CDC may claim 0x81/0x82) */
		if (bl < 13 || (size_t)bl + i > len || buf[i + 1] != 0x24 ||
		    buf[i + 2] != 0x01) {
			continue;
		}
		ep = buf[i + 6];
		if (ep != 0x81 && ep != 0x82 && ep != 0x83 && ep != 0x84) {
			continue;
		}

		buf[i + 5] = 0x00; /* wTotalLength high (clear OR corruption) */
		buf[i + 7] = 0x00; /* bmInfo */
		buf[i + 9] = 0x00; /* bStillCaptureMethod */
		/* Both UVC functions use OT id 5; keep VS link consistent. */
		buf[i + 8] = 0x05;
	}

	/* Bulk EP: Soft-IP OR-corrupts bDescriptorType 0x05 -> 0x07 at some offs */
	for (size_t i = 0; i + 7 <= len; i++) {
		if (buf[i] == 0x07 && (buf[i + 1] == 0x05 || buf[i + 1] == 0x07) &&
		    (buf[i + 2] == 0x81 || buf[i + 2] == 0x82 ||
		     buf[i + 2] == 0x83 || buf[i + 2] == 0x84) &&
		    (buf[i + 3] == 0x02 || buf[i + 3] == 0x06)) {
			buf[i + 1] = 0x05; /* USB_DESC_ENDPOINT */
			buf[i + 3] = 0x02; /* bulk */
		}
	}

	for (size_t i = 0; i + 27 <= len; i++) {
		/* Uncompressed format with BA81 GUID */
		if (!(buf[i] == 0x1b && buf[i + 1] == 0x24 && buf[i + 2] == 0x04 &&
		      buf[i + 5] == 'B' && buf[i + 6] == 'A' && buf[i + 7] == '8' &&
		      buf[i + 8] == '1')) {
			continue;
		}

		uint8_t nframes = buf[i + 4];
		size_t p = i + buf[i];
		uint8_t fi = 1;

		while (nframes-- > 0 && p + 9 <= len && buf[p] != 0) {
			/* Accept type 0x24 or corrupted 0x26 */
			if ((buf[p + 1] == 0x24 || buf[p + 1] == 0x26) &&
			    buf[p + 2] == 0x05) {
				uint16_t w = buf[p + 5] | ((uint16_t)buf[p + 6] << 8);
				uint16_t h = buf[p + 7] | ((uint16_t)buf[p + 8] << 8);

				buf[p + 1] = 0x24;
				buf[p + 3] = fi;
				if (h == 1080 || h == 1084 || w == 1920 || w == 1924 ||
				    (w & ~0x4U) == 1920) {
					buf[p + 5] = (uint8_t)(1920);
					buf[p + 6] = (uint8_t)(1920 >> 8);
					buf[p + 7] = (uint8_t)(1080);
					buf[p + 8] = (uint8_t)(1080 >> 8);
				} else if (h == 480 || h == 484 || w == 640 || w == 642 ||
					   w == 644 || (w & ~0x6U) == 640) {
					buf[p + 5] = (uint8_t)(640);
					buf[p + 6] = (uint8_t)(640 >> 8);
					buf[p + 7] = (uint8_t)(480);
					buf[p + 8] = (uint8_t)(480 >> 8);
				} else if (h == 720 || w == 1280 || w == 1284 ||
					   (w & ~0x4U) == 1280) {
					buf[p + 5] = (uint8_t)(1280);
					buf[p + 6] = (uint8_t)(1280 >> 8);
					buf[p + 7] = (uint8_t)(720);
					buf[p + 8] = (uint8_t)(720 >> 8);
				}
				fi++;
			}
			p += buf[p];
		}
	}
}

static void udc_dwc3_ep0_copy_to_bounce(volatile uint8_t *dst, const uint8_t *src,
					size_t len)
{
	for (size_t i = 0; i < len; i++) {
		dst[i] = src[i];
	}
}

static void udc_dwc3_trb_ctrl_in(const struct device *const dev,
				 struct net_buf *const buf,
				 const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;
	const uint8_t *dma_data = buf->data;
	uint32_t dma_len = buf->len;

	memset((void *)trb, 0, sizeof(*trb) * CONFIG_UDC_DWC3_TRB_NUM);

	/*
	 * Keep EP0 DMA 64-byte aligned (DWC3 requirement). Offset 64 relocates
	 * the payload in nocache SRAM; UVC geometry fields still sit on odd
	 * byte lanes by USB descriptor layout.
	 */
	enum { EP0_BOUNCE_OFF = 64 };

	if (buf->len > 0 &&
	    buf->len + EP0_BOUNCE_OFF <= sizeof(udc_dwc3_ep0_in_bounce) &&
	    (ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK) ==
		    UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
		volatile uint8_t *payload = &udc_dwc3_ep0_in_bounce[EP0_BOUNCE_OFF];

		udc_dwc3_ep0_copy_to_bounce(payload, buf->data, buf->len);
		udc_dwc3_ep0_patch_uvc_config(payload, buf->len);
		/* Volatile read-back keeps patch stores from being DCE'd. */
		if (buf->len > 200) {
			uint16_t w = 0;
			uint16_t h = 0;

			for (size_t i = 0; i + 27 <= buf->len; i++) {
				if (payload[i] == 0x1b && payload[i + 1] == 0x24 &&
				    payload[i + 2] == 0x04 && payload[i + 5] == 'B') {
					size_t p = i + payload[i];

					if (p + 9 <= buf->len) {
						w = payload[p + 5] |
						    ((uint16_t)payload[p + 6] << 8);
						h = payload[p + 7] |
						    ((uint16_t)payload[p + 8] << 8);
					}
					break;
				}
			}
			LOG_INF("EP0 bounce UVC frm1 %ux%u len=%u off=%u", w, h, buf->len,
				EP0_BOUNCE_OFF);
		}
		compiler_barrier();
		dma_data = (const uint8_t *)payload;
		dma_len = buf->len;
	}

	if (udc_ep_buf_has_zlp(buf)) {
		trb[0].addr_lo = LO32((uintptr_t)dma_data);
		trb[0].addr_hi = HI32((uintptr_t)dma_data);
		trb[0].status = dma_len;
		trb[0].ctrl = ctrl | UDC_DWC3_TRB_CTRL_CHN | UDC_DWC3_TRB_CTRL_HWO;

		trb[1].addr_lo = 0;
		trb[1].addr_hi = 0;
		trb[1].status = 0;
		trb[1].ctrl = ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO;
	} else {
		trb[0].addr_lo = LO32((uintptr_t)dma_data);
		trb[0].addr_hi = HI32((uintptr_t)dma_data);
		trb[0].status = dma_len;
		trb[0].ctrl = ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO;
	}

	if (dma_len > 200) {
		LOG_INF("EP0 TRB addr=0x%08x len=%u", trb[0].addr_lo, dma_len);
	}

	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

/*
 * OUT run-dry park resume.
 *
 * When the ring empties between OUT packets, DWC3 parks the endpoint. On this
 * IP a DepUpdateXfer in that window is often silently dropped under UVC.
 * Retry UpdateXfer with a short settle; if the tail TRB stays HWO, escalate to
 * EndXfer(ForceRM)+StartXfer on the same slot (pipe is already dead).
 */
static uint32_t udc_dwc3_ring_data_hwo_mask(const struct udc_dwc3_ep_data *ep_data)
{
	uint32_t mask = 0U;
	const uint32_t n = CONFIG_UDC_DWC3_TRB_NUM - 1U;

	for (uint32_t i = 0U; i < n; i++) {
		if ((ep_data->trb_buf[i].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
			mask |= BIT(i);
		}
	}

	return mask;
}

/*
 * OUT run-dry: ring was empty (parked) and a new buffer was just pushed.
 * One UpdateXfer is enough to unpark.  Do NOT wait for HWO clear — an armed
 * OUT keeps HWO until the host writes; treating that as failure caused
 * EndXfer recycle at SET_CONFIG, EP0 net_buf exhaustion, and host -32.
 * EndXfer retake belongs only on XferNotReady (armed TRB ignored).
 */
static void udc_dwc3_out_rundry_restart(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	/* Soft-IP drops the first OUT UpdateXfer into a parked ring
	 * (same defect as ACM IN). Second nudge after a short settle.
	 */
	udc_dwc3_depcmd_update_xfer(dev, ep_data);
	k_busy_wait(20U);
	udc_dwc3_depcmd_update_xfer(dev, ep_data);
}

/*
 * IN park resume: Soft-IP often drops the first UpdateXfer into a parked
 * ACM IN. Second No-Response nudge after a short settle (unlocked — do not
 * hold irq_lock across the wait). Do not wait for HWO clear.
 * Do not EndXfer here: poll_out bursts make every ACM IN look parked and
 * a retake-per-enqueue wedged 0x82 at 6771/27775 under Y16. Periodic
 * IN_PARK_RECOVER is the backstop.
 */
static void udc_dwc3_in_park_resume(const struct device *const dev,
				    struct udc_dwc3_ep_data *const ep_data)
{
	udc_dwc3_depcmd_update_xfer(dev, ep_data);
	k_busy_wait(20U);
	udc_dwc3_depcmd_update_xfer(dev, ep_data);
}

static int udc_dwc3_trb_bulk(const struct device *const dev,
			     struct udc_dwc3_ep_data *const ep_data,
			     struct net_buf *const buf)
{
	uint32_t ctrl = UDC_DWC3_TRB_CTRL_HWO | UDC_DWC3_TRB_CTRL_CSP;

	LOG_DBG("TRB_BULK_EP_0x%02x, buf %p, data %p, size %u, len %u",
		ep_data->cfg.addr, (void *)buf, (void *)buf->data, buf->size, buf->len);

	if (ep_data->full) {
		if (ep_data->cfg.addr == 0x85) {
			printk("dwc3: 0x85 trb_bulk full head=%u tail=%u\n",
			       ep_data->head, ep_data->tail);
		} else {
			udc_dwc3_nv_full++;
			if (udc_dwc3_nv_full <= 4U) {
				printk("dwc3: nv full ep=0x%02x hd=%u tl=%u hwo=0x%x\n",
				       ep_data->cfg.addr, ep_data->head, ep_data->tail,
				       udc_dwc3_ring_data_hwo_mask(ep_data));
			}
		}
		return -EBUSY;
	}

	if (udc_ep_buf_has_zlp(buf)) {
		LOG_DBG("Buffer has a ZLP flag, terminating the transfer");
		ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL_ZLP;
		ep_data->total = 0;
	} else {
		ctrl |= UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL;
		{
			ep_data->total += buf->len;

			if (USB_EP_DIR_IS_IN(ep_data->cfg.addr) &&
			    ep_data->total % ep_data->cfg.mps == 0) {
				LOG_DBG("Buffer is a multiple of %d, continuing this transfer of %u bytes",
					ep_data->cfg.mps, ep_data->total);
				ctrl |= UDC_DWC3_TRB_CTRL_CHN;
				/* 0x85: IOC on every 16 KiB page flooded the
				 * event ring next to ACM and dropped completes.
				 * HWO-poll retires CHN pages; IOC only on the
				 * short last TRB.
				 */
				if (ep_data->cfg.addr != 0x85) {
					ctrl |= UDC_DWC3_TRB_CTRL_IOC;
				}
			} else {
				LOG_DBG("End of USB transfer, %u bytes transferred", ep_data->total);
				ep_data->total = 0;
				ctrl |= UDC_DWC3_TRB_CTRL_IOC;
			}
		}
	}

	/*
	 * Sample park before push. IN and OUT both park when the ring drains;
	 * Soft-IP UVC makes fire-and-forget UpdateXfer on ACM IN drop (shell
	 * timeout → host Write timeout → xHCI death).
	 */
	const bool resume_from_park =
		ep_data->xfer_active && udc_dwc3_ring_data_hwo_mask(ep_data) == 0U;
	const bool out_resume_from_park =
		resume_from_park && USB_EP_DIR_IS_OUT(ep_data->cfg.addr);
	const uint32_t push_slot = ep_data->head;
	const bool in_resume_from_park =
		resume_from_park && USB_EP_DIR_IS_IN(ep_data->cfg.addr);

#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
	/*
	 * Enable-time StartXfer parks an empty 0x85. One EndXfer frees that
	 * stale resource; later pages must wait for XferComplete (inflight=1)
	 * so a second StartXfer does not hit CMDERR/no-resource.
	 */
	if (ep_data->cfg.addr == 0x85 && resume_from_park && !iebm_uvc_unparked) {
		printk("dwc3: 0x85 unpark EndXfer (empty ring)\n");
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		iebm_uvc_unparked = true;
	}
#endif

	udc_dwc3_push_trb(dev, ep_data, buf, ctrl);

	if (ep_data->cfg.addr == 0x85) {
		static uint32_t n85;

		if (n85 < 6U) {
			printk("dwc3: 0x85 push slot=%u data=%p len=%u ctrl=0x%x xa=%d\n",
			       push_slot, (void *)buf->data, buf->len, ctrl,
			       ep_data->xfer_active);
			n85++;
		}
	}

	if (!ep_data->xfer_active) {
		if (ep_data->cfg.addr == 0x85) {
			static uint32_t n85s;

			/* After LST the resource is freed. StartXfer at
			 * trb_buf[0] would see a retired HWO=0 descriptor
			 * and never walk to this page.
			 */
			udc_dwc3_depcmd_start_xfer_trb(dev, ep_data,
						       &ep_data->trb_buf[push_slot]);
			if (n85s < 3U) {
				printk("dwc3: 0x85 StartXfer slot=%u xa=%d idx=0x%x trb=0x%x\n",
				       push_slot, ep_data->xfer_active,
				       ep_data->xferrscidx,
				       ep_data->trb_buf[push_slot].ctrl);
				n85s++;
			}
		} else {
			udc_dwc3_depcmd_start_xfer(dev, ep_data);
		}
	} else if (out_resume_from_park) {
		udc_dwc3_out_rundry_restart(dev, ep_data);
	} else if (in_resume_from_park && ep_data->tail == push_slot) {
		udc_dwc3_in_park_resume(dev, ep_data);
	} else {
#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
		/*
		 * IEBM video kicks CmdAct=1 thousands of times/s. A single
		 * ACM IN UpdateXfer is dropped (1 Hz SRP dies by tick 4;
		 * 0.1 Hz by tick 10). Second nudge after 20 us, same as
		 * park-resume: Soft-IP also needed two doorbells on 0x82.
		 */
		if (iebm_fast.ep != NULL &&
		    USB_EP_DIR_IS_IN(ep_data->cfg.addr) &&
		    ep_data->cfg.addr != 0x85) {
			udc_dwc3_in_park_resume(dev, ep_data);
		} else {
			udc_dwc3_depcmd_update_xfer(dev, ep_data);
		}
#else
		udc_dwc3_depcmd_update_xfer(dev, ep_data);
#endif
	}

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

static void udc_dwc3_ep0_flush_pending(const struct device *const dev, const int err)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct net_buf *buf;

	while ((buf = udc_buf_get(&cfg->ep_data_out[0].cfg)) != NULL) {
		udc_submit_ep_event(dev, buf, err);
	}
	while ((buf = udc_buf_get(&cfg->ep_data_in[0].cfg)) != NULL) {
		udc_submit_ep_event(dev, buf, err);
	}
	udc_ep_set_busy(&cfg->ep_data_out[0].cfg, false);
	udc_ep_set_busy(&cfg->ep_data_in[0].cfg, false);
}

static void udc_dwc3_ep0_stall_and_restart(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	udc_dwc3_health_ep0_rst++;
	priv->ep0_setup_pending = false;
	priv->ep0_status_nrd = false;
	priv->ep0_data_inflight = false;
	priv->ep0_data_dir = 0U;

	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_out[0]);
	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_in[0]);
	udc_dwc3_ep0_flush_pending(dev, -ECONNRESET);
	LOG_WRN("EP0 stall-and-restart");
}

static bool udc_dwc3_ep0_trb_setup_pending(struct udc_dwc3_ep_data *const ep_data)
{
	return (ep_data->trb_buf[0].status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK) ==
	       UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING;
}

static void udc_dwc3_next_ctrl(const struct device *const dev,
			       struct udc_dwc3_ep_data *const ep_data)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct net_buf *buf;
	struct udc_buf_info *bi;

	if (udc_ep_is_busy(&ep_data->cfg)) {
		return;
	}

	buf = udc_buf_peek(&ep_data->cfg);
	if (buf == NULL) {
		return;
	}

	bi = udc_get_buf_info(buf);
	/* Lattice-ref: never pre-arm STATUS while DATA is in flight. */
	if (bi->status && !priv->ep0_status_nrd) {
		return;
	}

	udc_ep_set_busy(&ep_data->cfg, true);

	if (bi->data) {
		priv->ep0_data_inflight = true;
		priv->ep0_data_dir = USB_EP_DIR_IS_IN(ep_data->cfg.addr) ? 1U : 0U;
	}

	/* In DWC3 */
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
	for (uint32_t spins = 0U; sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_CSFTRST;) {
		if (++spins > 1000000U) {
			LOG_ERR("DCTL.CSFTRST never cleared: 0x%08x",
				sys_read32(base + UDC_DWC3_DCTL));
			break;
		}
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

	/*
	 * After TX FIFO rebalance, isoc FIFONUM3 is ~16KiB — allow up to 8
	 * packet bursts (still below full Mult=16 so underruns stay rare).
	 */
	reg = UDC_DWC3_GTXTHRCFG_USBTXPKTCNTSEL;
	reg |= FIELD_PREP(UDC_DWC3_GTXTHRCFG_USBTXPKTCNT_MASK, 4);
	reg |= FIELD_PREP(UDC_DWC3_GTXTHRCFG_USBMAXTXBURSTSIZE_MASK, 8);
	sys_write32(reg, base + UDC_DWC3_GTXTHRCFG);

	/* Read the chip identification */
	reg = sys_read32(base + UDC_DWC3_GCOREID);
	LOG_INF("event: coreid=0x%04lx rel=0x%04lx",
		FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg),
		FIELD_GET(UDC_DWC3_GCOREID_REL_MASK, reg));
	__ASSERT_NO_MSG(FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg) == 0x5533);
	/* Always-on boot marker: a dead register block reads 0 here */
	printk("dwc3: GCOREID=0x%08x GHWPARAMS7=0x%08x%s\n", reg,
	       sys_read32(base + UDC_DWC3_GHWPARAMS7),
	       FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg) == 0x5533 ? "" :
	       " (CORE NOT RESPONDING)");

	/* Letting GUID unchanged */
	/* Letting GUSB2PHYCFG and GUSB3PIPECTL unchanged */

	/*
	 * Rebalance TX FIFOs (FIFONUM n = IN ep 0x8n, see DEPCFG). Default
	 * dep≈517 MDWIDTH words (~4KiB @ 64-bit) is tight for the video
	 * endpoints; give each video FIFONUM 8 KiB and size the rest by role.
	 * A bulk IN endpoint needs at least one SS max packet (1024 B = 128
	 * words) plus margin or the core never starts the transfer — an ACM IN
	 * on a 64-word FIFO sits at "remain=N" forever while the host polls.
	 * RAM1 depth from GHWPARAMS7 (0x0ac1 = 2753 words on USB23) must cover
	 * the sum of depths.
	 *
	 * Dual-UVC product: ACM0, UVC0, UVC1, ACM1 → video on FIFO3/4.
	 * FLIR IEBM (app_flir): ACM0, RAW, UVC last → video on FIFO5 0x85.
	 * A 64-word FIFO5 cannot start SS bulk (MPS 1024); that matches
	 * StartXfer xa=1 + HWO stuck + host C Bi 0 bytes.
	 */
	{
		const uint32_t ram1 =
			(uint32_t)FIELD_GET(GENMASK(15, 0),
					    sys_read32(base + UDC_DWC3_GHWPARAMS7));
		/* depths in MDWIDTH units (64-bit → 8 B/unit on this core) */
#if defined(CONFIG_UDC_DWC3_TXFIFO_VIDEO_EP5)
		/*
		 * FIFO5 = UVC 0x85, 8 KiB. FIFO3/4 = RAW INT/bulk.
		 * Phase 1c: FIFONUM2 (ACM0 IN 0x82) 192->384 words. park82 A/B
		 * dumps under live Y16 + CDC-RAW bulk show tx2 (FIFO2 avail)
		 * frozen at 190/192 with ep82 HWO stuck and IN-RECOVER
		 * remain=12 — FIFO2 was full and not draining. 384 words
		 * (3 KiB) gives the ACM0 IN path more margin against the
		 * TxFIFO2/5 refill DMA being starved on the shared AXI slave
		 * during a live video + bulk overlap. sum stays well under
		 * ram1 (2753 words on USB23).
		 */
		const uint16_t dep[] = { 66, 64, 384, 64, 192, 1024, 32, 32 };
#else
		const uint16_t dep[] = { 66, 64, 192, 1024, 1024, 64, 192, 32 };
#endif
		uint32_t addr = 0;
		uint32_t sum = 0;

		for (uint32_t i = 0; i < ARRAY_SIZE(dep); i++) {
			sum += dep[i];
		}
		if (sum <= ram1) {
			for (uint32_t i = 0; i < ARRAY_SIZE(dep); i++) {
				sys_write32(FIELD_PREP(UDC_DWC3_GTXFIFOSIZ_TXFSTADDR_MASK,
						       addr) |
						    FIELD_PREP(UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK,
							       dep[i]),
					    base + UDC_DWC3_GTXFIFOSIZ(i));
				addr += dep[i];
			}
			printk("dwc3: TXFIFO dep %u/%u/%u/%u/%u/%u/%u/%u sum=%u ram1=%u\n",
			       dep[0], dep[1], dep[2], dep[3], dep[4], dep[5],
			       dep[6], dep[7], sum, ram1);
			LOG_INF("TX FIFO rebalance: video FIFONUM3/4/5 dep=%u/%u/%u (sum=%u ram1=%u)",
				dep[3], dep[4], dep[5], sum, ram1);
		} else {
			LOG_WRN("TX FIFO rebalance skipped: need %u > ram1 %u", sum, ram1);
		}
	}

	LOG_INF("GTXTHRCFG=0x%08x GTXFIFOSIZ[0]=0x%08x[dep=%u] [1]=0x%08x[dep=%u] "
		"[2]=0x%08x[dep=%u] [3]=0x%08x[dep=%u] GHWPARAMS7=0x%08x",
		sys_read32(base + UDC_DWC3_GTXTHRCFG),
		sys_read32(base + UDC_DWC3_GTXFIFOSIZ(0)),
		(uint32_t)FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK,
				    sys_read32(base + UDC_DWC3_GTXFIFOSIZ(0))),
		sys_read32(base + UDC_DWC3_GTXFIFOSIZ(1)),
		(uint32_t)FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK,
				    sys_read32(base + UDC_DWC3_GTXFIFOSIZ(1))),
		sys_read32(base + UDC_DWC3_GTXFIFOSIZ(2)),
		(uint32_t)FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK,
				    sys_read32(base + UDC_DWC3_GTXFIFOSIZ(2))),
		sys_read32(base + UDC_DWC3_GTXFIFOSIZ(3)),
		(uint32_t)FIELD_GET(UDC_DWC3_GTXFIFOSIZ_TXFDEP_MASK,
				    sys_read32(base + UDC_DWC3_GTXFIFOSIZ(3))),
		sys_read32(base + UDC_DWC3_GHWPARAMS7));

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

	/*
	 * Control endpoint pool only. Non-control resources are allocated when
	 * the first of those endpoints is enabled after SET_CONFIGURATION.
	 */
	udc_dwc3_depcmd_start_config(dev, &cfg->ep_data_out[0], 0U);
	DEV_DATA(dev)->startcfg_nonctrl_done = false;
}

static void udc_dwc3_on_usb_reset(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	LOG_DBG("Going through DWC3 reset logic");

	/* The RTL must not keep ringing a video endpoint the host just reset */
	udc_dwc3_hw_owned_revoke_all(dev, "usb-reset");

	/* Host will re-enumerate; non-control pool must be reallocated. */
	DEV_DATA(dev)->startcfg_nonctrl_done = false;

	/* Reset all ongoing transfers on non-control IN endpoints */
	for (int epn = 1; epn < cfg->num_in_eps; epn++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[epn];

		continue; /* TODO */
		udc_dwc3_depcmd_end_xfer(dev, ep_data, 0);
		udc_dwc3_depcmd_clear_stall(dev, ep_data);
	}

	/* Reset all ongoing transfers on non-control OUT endpoints */
	for (int epn = 1; epn < cfg->num_out_eps; epn++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[epn];

		continue; /* TODO */
		udc_dwc3_depcmd_end_xfer(dev, ep_data, 0);
		udc_dwc3_depcmd_clear_stall(dev, ep_data);
	}

	/* Perform the USB reset operations manually to improve latency */
	udc_dwc3_set_address(dev, 0);

	/* Let Zephyr set the device address 0 */
	udc_submit_event(dev, UDC_EVT_RESET, 0);
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
	udc_dwc3_evt_fast = true;
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
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[0];
	const uint32_t trb_trbctl = ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK;
	struct net_buf *buf;

	if (udc_dwc3_ep0_trb_setup_pending(ep_data)) {
		udc_dwc3_health_setup_pending++;
		priv->ep0_setup_pending = true;
		priv->ep0_data_inflight = false;
		buf = udc_buf_get(&ep_data->cfg);
		if (buf != NULL) {
			udc_submit_ep_event(dev, buf, -ECONNRESET);
		}
		udc_ep_set_busy(&ep_data->cfg, false);
		udc_dwc3_ep0_stall_and_restart(dev);
		return;
	}

	buf = udc_buf_get(&ep_data->cfg);
	if (buf == NULL) {
		/* Stale EP0 completion under UVC — do not raise UDC_EVT_ERROR. */
		LOG_WRN("CTRL IN completion with no buffer");
		return;
	}

	if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
	    trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
		buf->len = 0;
		priv->ep0_status_nrd = false;
		LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL STATUS packet sent");
	} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
		const uint32_t residual =
			FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, ep_data->trb_buf[0].status);

		priv->ep0_data_inflight = false;
		if (residual != 0) {
			LOG_WRN("CTRL DATA IN short: requested %u residual %u (sent %u)",
				buf->len, residual, buf->len - residual);
		}
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
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[0];
	const uint32_t trb_trbctl = ep_data->trb_buf[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK;
	const uint32_t trb_status = ep_data->trb_buf[0].status;
	struct net_buf *buf;

	if (trb_trbctl != UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP &&
	    udc_dwc3_ep0_trb_setup_pending(ep_data)) {
		udc_dwc3_health_setup_pending++;
		priv->ep0_setup_pending = true;
		priv->ep0_data_inflight = false;
		buf = udc_buf_get(&ep_data->cfg);
		if (buf != NULL) {
			udc_submit_ep_event(dev, buf, -ECONNRESET);
		}
		udc_ep_set_busy(&ep_data->cfg, false);
		udc_dwc3_ep0_stall_and_restart(dev);
		return;
	}

	if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		struct usb_setup_packet *setup;

		buf = udc_buf_peek(&ep_data->cfg);
		if (buf == NULL) {
			LOG_WRN("missing buffer for SETUP packet");
			return;
		}

		setup = (struct usb_setup_packet *)buf->data;

		/* Latency optimization: set the address immediately to be able to be able
		 * to ACK/NAK the first packets from the host with the new address,
		 * otherwise the host issue a reset.
		 */
		if (setup->bmRequestType == USB_REQTYPE_TYPE_STANDARD &&
		    setup->bRequest == USB_SREQ_SET_ADDRESS) {
			udc_dwc3_set_address(dev, setup->wValue);
		}

		/* Update the size to what the hardware reports */
		buf->len = buf->size - FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb_status);

		LOG_HEXDUMP_DBG(buf->data, buf->len, "SETUP received");

		/* The buffer will directly be taken from the UDC queue */
		udc_setup_received(dev, NULL);
	} else {
		buf = udc_buf_get(&ep_data->cfg);
		if (buf == NULL) {
			LOG_WRN("CTRL OUT completion with no buffer");
			return;
		}

		/* Update the size to what the hardware reports */
		buf->len = buf->size - FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb_status);

		if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
			priv->ep0_data_inflight = false;
			LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL DATA received");
		} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
			buf->len = 0;
			priv->ep0_status_nrd = false;
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
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);

	switch (evt & UDC_DWC3_DEPEVT_STATUS_B3_MASK) {
	case UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_SETUP:
		LOG_DBG("UDC_DWC3_DEPEVT_XFERNOTREADY_CONTROL_SETUP");
		break;
	case UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_DATA:
		LOG_DBG("UDC_DWC3_DEPEVT_XFERNOTREADY_CONTROL_DATA");
		/* Wrong-direction DATA NRDY: Lattice-ref EndXfer + stall-restart. */
		if (priv->ep0_data_inflight && (uint8_t)epn != priv->ep0_data_dir) {
			struct udc_dwc3_ep_data *const ep_data =
				(priv->ep0_data_dir != 0U) ? &cfg->ep_data_in[0]
							   : &cfg->ep_data_out[0];

			udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
			udc_dwc3_ep0_stall_and_restart(dev);
		}
		break;
	case UDC_DWC3_DEPEVT_STATUS_B3_CONTROL_STATUS:
		LOG_DBG("UDC_DWC3_DEPEVT_XFERNOTREADY_CONTROL_STATUS");
		if (priv->ep0_setup_pending) {
			udc_dwc3_ep0_stall_and_restart(dev);
			break;
		}
		priv->ep0_status_nrd = true;
		udc_dwc3_next_ctrl(dev, &cfg->ep_data_in[0]);
		udc_dwc3_next_ctrl(dev, &cfg->ep_data_out[0]);
		break;
	}
}

#if defined(CONFIG_UDC_DWC3_OUT_NOTREADY_RETAKE)
/*
 * Drop and retake a bulk OUT transfer resource without clearing the ring
 * (ordinary DepEndXfer zeroes head/tail and desyncs the class).
 */
static void udc_dwc3_out_notready_recycle(const struct device *const dev,
					  struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t tail = ep_data->tail;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
			UDC_DWC3_DEPCMD_DEPENDXFER | UDC_DWC3_DEPCMD_HIPRI_FORCERM |
			FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx));

	udc_dwc3_depcmd_start_xfer_trb(dev, ep_data, &ep_data->trb_buf[tail]);
}

static void udc_dwc3_on_xfer_not_ready_norm(const struct device *const dev,
					    const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *const ep_data =
		(epn & 1) ? &cfg->ep_data_in[epn >> 1] : &cfg->ep_data_out[epn >> 1];
	static int64_t last_retake_ms[16];
	const int64_t now = k_uptime_get();
	const uint8_t oidx = USB_EP_GET_IDX(ep_data->cfg.addr);

	if (ep_data->trb_buf == NULL || USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		return;
	}

	if (ep_data->net_buf[ep_data->tail] == NULL ||
	    (ep_data->trb_buf[ep_data->tail].ctrl & UDC_DWC3_TRB_CTRL_HWO) == 0U) {
		return;
	}

	/* Cooldown: EndXfer spam under UVC wedges EP0 (Malformed setup). */
	if (oidx < ARRAY_SIZE(last_retake_ms) &&
	    (now - last_retake_ms[oidx]) < 200) {
		return;
	}

	/*
	 * Prefer UpdateXfer first.  EndXfer(ForceRM) under Soft-IP UVC has
	 * wedged EP0; only recycle if the armed TRB is still stuck after a
	 * nudge (next NOTREADY after cooldown).
	 */
	static uint8_t notready_escalated[16];

	if (oidx < ARRAY_SIZE(notready_escalated) && notready_escalated[oidx] == 0U) {
		printk("OUT-NOTREADY ep=0x%02x armed, UpdateXfer (tail=%u)\n",
		       ep_data->cfg.addr, ep_data->tail);
		udc_dwc3_depcmd_update_xfer(dev, ep_data);
		notready_escalated[oidx] = 1U;
	} else {
		/* Prefer UpdateXfer-only under Soft-IP; EndXfer kept as last resort. */
		printk("OUT-NOTREADY ep=0x%02x armed, UpdateXfer-retry (tail=%u)\n",
		       ep_data->cfg.addr, ep_data->tail);
		udc_dwc3_depcmd_update_xfer(dev, ep_data);
		if (oidx < ARRAY_SIZE(notready_escalated)) {
			notready_escalated[oidx] = 0U;
		}
	}
	if (oidx < ARRAY_SIZE(last_retake_ms)) {
		last_retake_ms[oidx] = now;
	}
}
#endif /* CONFIG_UDC_DWC3_OUT_NOTREADY_RETAKE */

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
		udc_dwc3_health_setup_pending++;
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

static bool udc_dwc3_ep_is_hw_in(const struct udc_dwc3_ep_data *ep_data)
{
	const uint8_t addr = ep_data->cfg.addr;

	if (!USB_EP_DIR_IS_IN(addr)) {
		return false;
	}

	return (CONFIG_UDC_DWC3_HW_IN_EP_MASK & BIT(USB_EP_GET_IDX(addr))) != 0U;
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
	const int acm_slot = udc_dwc3_acm_slot(ep_data->cfg.addr);

	if (acm_slot >= 0) {
		udc_dwc3_acm_xfer_evt[acm_slot]++;
	}

	/*
	 * RTL-owned UVC IN: uvcmanager writes TRBs and rings UpdateXfer.  There
	 * is no software net_buf on the ring; retiring here races the hardware
	 * and produces host-side EPROTO (-71).
	 */
	if (ep_data->cfg.addr == 0x85) {
		static uint32_t n85d;

		if (n85d < 0U) { /* off: see ev85 */
			printk("dwc3: 0x85 xfer_done hw_owned=%d hwin=%d tail=%u hwo=%d nb=%p\n",
			       ep_data->hw_owned, udc_dwc3_ep_is_hw_in(ep_data),
			       ep_data->tail,
			       (trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U,
			       ep_data->net_buf[ep_data->tail]);
			n85d++;
		}
	}

	if (ep_data->hw_owned) {
		ep_data->hw_owned_evts++;
		return;
	}
	if (udc_dwc3_ep_is_hw_in(ep_data)) {
		return;
	}

#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
	if (ep_data->cfg.addr == 0x85) {
		(void)lattice_usb23_iebm_retire(dev, 0x85);
		return;
	}
#endif

	/* No CPU buffer at tail: residual HWO=0 on an idle slot — ignore. */
	if (ep_data->net_buf[ep_data->tail] == NULL) {
		udc_dwc3_nv_drop_nb_null++;
		return;
	}

	/*
	 * Soft-IP posts XferInProgress while the descriptor can still be HWO.
	 * Retiring early desyncs ACM IN (shell output stalls, then OUT wedges).
	 */
	if ((trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
		udc_dwc3_nv_drop_hwo++;
		if (udc_dwc3_nv_drop_hwo <= 8U) {
			printk("dwc3: nv hwo-set ep=0x%02x tl=%u hd=%u ctrl=0x%08x sts=0x%08x evt=0x%08x\n",
			       ep_data->cfg.addr, ep_data->tail, ep_data->head,
			       trb->ctrl, trb->status, evt);
		}
		return;
	}

	/*
	 * Latch TRB status before retiring the ring slot. Soft-IP pop_trb()
	 * does not zero the TRB today, but LiteX soak proved reading residual
	 * after clear yields MPS-sized garbage into CDC ACM (shell fed its
	 * own prompt). Keep the latch + underflow guard for both trees.
	 */
	const uint32_t trb_status_done = trb->status;

	buf = udc_dwc3_pop_trb(dev, ep_data);
	if (buf == NULL) {
		/*
		 * Under UVC the controller posts occasional DEPEVTs with no
		 * matching CPU net_buf (stale / HW-IN bleed). Raising
		 * UDC_EVT_ERROR here used to take ACM down mid-SRP.
		 */
		return;
	}

	LOG_DBG("XFER_DONE_NORM: EP 0x%02x, data %p", ep_data->cfg.addr, (void *)buf->data);
	if (acm_slot >= 0) {
		udc_dwc3_acm_xfer_ret[acm_slot]++;
	}
	udc_dwc3_on_xfer_done(dev, ep_data);

	/* For buffers coming from the host, update the size actually received */
	if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
		const uint32_t residual =
			FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb_status_done);

		buf->len = (residual <= buf->size) ? (buf->size - residual) : 0U;
		udc_dwc3_out_data_fence(buf);
	}

#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
	{
		struct udc_dwc3_data *const priv = udc_get_private(dev);

		if (ep_data->cfg.addr == 0x85 &&
		    (priv->iebm_complete_cb != NULL ||
		     priv->iebm_complete_ep == 0x85)) {
			/* IEBM pages are not class-owned; submitting them
			 * returns -ENOTSUP and the stack takes the EP down.
			 * Keep xfer_active so the next page is UpdateXfer.
			 * After STREAMOFF the cb is cleared; still unref.
			 */
			if (priv->iebm_complete_cb != NULL &&
			    priv->iebm_complete_ep == ep_data->cfg.addr) {
				priv->iebm_complete_cb(dev, ep_data->cfg.addr,
						       priv->iebm_complete_user);
			}
			net_buf_unref(buf);
			k_work_submit(&ep_data->work);
			return;
		}
	}
#endif

	ret = udc_submit_ep_event(dev, buf, 0);
	if (ret != 0) {
		LOG_ERR("Failed to submit buffer %p: %d", buf, ret);
	}

	/* We just made some room for a new buffer, check if something more to enqueue */
	k_work_submit(&ep_data->work);
}

#if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL)
/*
 * Soft-IP sometimes clears HWO on a CPU-managed IN TRB without posting a
 * DEPEVT. Require HWO clear across two 100 ms wakes before retiring to avoid
 * the desync seen with immediate software-retire.
 */
static uint8_t udc_dwc3_in_poll_grace[32];

static void udc_dwc3_in_completion_poll_tick(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	uint32_t mask = CONFIG_UDC_DWC3_IN_RECOVER_EP_MASK &
			~CONFIG_UDC_DWC3_HW_IN_EP_MASK;

	while (mask != 0U) {
		const uint8_t idx = (uint8_t)__builtin_ctz(mask);
		struct udc_dwc3_ep_data *ep_data;
		volatile struct udc_dwc3_trb *trb;

		mask &= ~BIT(idx);
		if (idx >= cfg->num_in_eps || idx >= ARRAY_SIZE(udc_dwc3_in_poll_grace)) {
			continue;
		}

		ep_data = &cfg->ep_data_in[idx];
		if (ep_data->trb_buf == NULL || udc_dwc3_ep_is_hw_in(ep_data)) {
			udc_dwc3_in_poll_grace[idx] = 0;
			continue;
		}
		if (ep_data->net_buf[ep_data->tail] == NULL) {
			udc_dwc3_in_poll_grace[idx] = 0;
			continue;
		}

		trb = &ep_data->trb_buf[ep_data->tail];
		if ((trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
			udc_dwc3_in_poll_grace[idx] = 0;
			continue;
		}

		/*
		 * Grace ticks of the 100 ms event-thread timeout.  Prior
		 * grace=2 (~200 ms) raced live DEPEVT and cliffed at ~6 ACM
		 * OK; use ~800 ms so only true dropped completions retire.
		 */
		if (udc_dwc3_in_poll_grace[idx] < 8U) {
			udc_dwc3_in_poll_grace[idx]++;
			continue;
		}

		udc_dwc3_in_poll_grace[idx] = 0;
		/* Re-check under lock so event-thread retire cannot double-pop. */
		{
			unsigned int key = irq_lock();

			if (ep_data->net_buf[ep_data->tail] == NULL ||
			    (ep_data->trb_buf[ep_data->tail].ctrl &
			     UDC_DWC3_TRB_CTRL_HWO) != 0U) {
				irq_unlock(key);
				continue;
			}
			printk("IN-POLL: retire lost-cmpl ep=0x%02x tail=%u\n",
			       ep_data->cfg.addr, ep_data->tail);
			udc_dwc3_on_xfer_done_norm(dev,
				UDC_DWC3_DEPEVT_XFERINPROGRESS(ep_data->epn));
			irq_unlock(key);
		}
	}
}
#endif /* CONFIG_UDC_DWC3_IN_COMPLETION_POLL */

#if defined(CONFIG_UDC_DWC3_OUT_COMPLETION_POLL)
/*
 * Same dropped-DEPEVT defect as the IN poll above, on CPU-managed bulk OUT.
 * Observed under isoc UVC: CDC OUT 0x01 sat at nb=1 hwo=0 rem=1016 (an 8-byte
 * packet already in the TRB) for seconds, so the class never saw the data and
 * every reply arrived one transfer late.
 */
static uint8_t udc_dwc3_out_poll_grace[8];

static void udc_dwc3_out_completion_poll_tick(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	for (int idx = 1; idx < cfg->num_out_eps &&
			  idx < (int)ARRAY_SIZE(udc_dwc3_out_poll_grace); idx++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[idx];

		if (ep_data->trb_buf == NULL ||
		    (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) !=
			    USB_EP_TYPE_BULK ||
		    ep_data->net_buf[ep_data->tail] == NULL ||
		    (ep_data->trb_buf[ep_data->tail].ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U) {
			udc_dwc3_out_poll_grace[idx] = 0;
			continue;
		}

		if (udc_dwc3_out_poll_grace[idx] <
		    (uint8_t)CONFIG_UDC_DWC3_OUT_COMPLETION_POLL_TICKS) {
			udc_dwc3_out_poll_grace[idx]++;
			continue;
		}

		udc_dwc3_out_poll_grace[idx] = 0;
		/* Re-check under lock so event-thread retire cannot double-pop. */
		{
			unsigned int key = irq_lock();

			if (ep_data->net_buf[ep_data->tail] == NULL ||
			    (ep_data->trb_buf[ep_data->tail].ctrl &
			     UDC_DWC3_TRB_CTRL_HWO) != 0U) {
				irq_unlock(key);
				continue;
			}
			printk("OUT-POLL: retire lost-cmpl ep=0x%02x tail=%u\n",
			       ep_data->cfg.addr, ep_data->tail);
			udc_dwc3_on_xfer_done_norm(dev,
				UDC_DWC3_DEPEVT_XFERINPROGRESS(ep_data->epn));
			irq_unlock(key);
		}
	}
}
#endif /* CONFIG_UDC_DWC3_OUT_COMPLETION_POLL */

#if defined(CONFIG_UDC_DWC3_OUT_STALL_REFRESH)
/*
 * UpdateXfer-only backstop (no EndXfer). Soft-IP drops OUT UpdateXfer under
 * UVC; a periodic nudge is a no-op on a healthy idle OUT and can unwedge a
 * parked ring without the EP0 storms from EndXfer spam.
 */
static void udc_dwc3_out_stall_refresh_tick(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	static int64_t last_nudge_ms;
	const int64_t now = k_uptime_get();

	if ((now - last_nudge_ms) < CONFIG_UDC_DWC3_OUT_STALL_REFRESH_MS) {
		return;
	}
	last_nudge_ms = now;

	for (int i = 1; i < cfg->num_out_eps; i++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[i];

		if (ep_data->trb_buf == NULL || !ep_data->xfer_active ||
		    (ep_data->cfg.attributes & USB_EP_TRANSFER_TYPE_MASK) !=
			    USB_EP_TYPE_BULK) {
			continue;
		}
		if (ep_data->net_buf[ep_data->tail] == NULL) {
			continue;
		}
		if ((ep_data->trb_buf[ep_data->tail].ctrl & UDC_DWC3_TRB_CTRL_HWO) == 0U) {
			continue;
		}

		udc_dwc3_depcmd_update_xfer(dev, ep_data);
	}
}
#endif /* CONFIG_UDC_DWC3_OUT_STALL_REFRESH */

#if defined(CONFIG_UDC_DWC3_IN_PARK_RECOVER)
/*
 * Soft-IP IN recover: UpdateXfer first. Under IEBM the 0x82 doorbell is
 * accepted but the TRB is never fetched (HWO stays set through STREAMOFF).
 * IN_START_ENDXFER_ESCALATE then EndXfer(ForceRM)+StartXfer on the same
 * slot without zeroing the ring (ordinary DepEndXfer resets head/tail).
 */
#define UDC_DWC3_IN_PARK_COOLDOWN_MS 400
#define UDC_DWC3_IN_PARK_NUDGE_CAP 6
#define UDC_DWC3_IN_PARK_BACKOFF_MS 5000
#define UDC_DWC3_IN_NUDGE_EP_MASK CONFIG_UDC_DWC3_IN_RECOVER_EP_MASK

static void udc_dwc3_in_park_retake(const struct device *const dev,
				    struct udc_dwc3_ep_data *const ep_data)
{
	const uint32_t n = CONFIG_UDC_DWC3_TRB_NUM - 1U;
	const uint32_t tail = ep_data->tail;

	if (ep_data->xfer_active && ep_data->xferrscidx != 0U) {
		udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn),
				UDC_DWC3_DEPCMD_DEPENDXFER |
					UDC_DWC3_DEPCMD_HIPRI_FORCERM |
					FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK,
						   ep_data->xferrscidx));
		ep_data->xfer_active = false;
		ep_data->xferrscidx = 0;
	}

	/*
	 * ForceRM returns the descriptors with BUFSIZ=0. OR-ing HWO on a
	 * zero-length TRB StartXfers nothing (1 Hz probe: remain=0, 17
	 * retakes, ACM still deaf). Rebuild each live slot from net_buf.
	 */
	for (uint32_t i = 0U; i < n; i++) {
		struct net_buf *const buf = ep_data->net_buf[i];
		volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[i];

		if (buf == NULL) {
			continue;
		}
		trb->addr_lo = LO32((uintptr_t)buf->data);
		trb->addr_hi = HI32((uintptr_t)buf->data);
		trb->status = USB_EP_DIR_IS_IN(ep_data->cfg.addr) ? buf->len : buf->size;
		trb->ctrl = UDC_DWC3_TRB_CTRL_HWO | UDC_DWC3_TRB_CTRL_CSP |
			    UDC_DWC3_TRB_CTRL_TRBCTL_NORMAL | UDC_DWC3_TRB_CTRL_IOC;
	}

	udc_dwc3_depcmd_start_xfer_trb(dev, ep_data, &ep_data->trb_buf[tail]);
#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
	udc_dwc3_nv_retake++;
#endif
}

struct udc_dwc3_in_park_state {
	int64_t since;
	int64_t cooldown_until;
	uint32_t tail;
	uint32_t remain;
	uint8_t nudges;
};

/* Indexed by IN endpoint number; the DWC3 has at most 16 per direction. */
static struct udc_dwc3_in_park_state udc_dwc3_park[16];
static uint8_t udc_dwc3_park0_snap;
static int64_t udc_dwc3_park0_t0;

static bool udc_dwc3_depcmd_is_idle(const struct device *const dev,
				    const uint8_t epn)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t reg = sys_read32(base + UDC_DWC3_DEPCMD(epn));

	return (reg & UDC_DWC3_DEPCMD_CMDACT) == 0U;
}

static void udc_dwc3_in_recover_one(const struct device *const dev,
				    const uint8_t idx)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *ep_data;
	struct udc_dwc3_in_park_state *park;
	volatile struct udc_dwc3_trb *trb;
	struct net_buf *buf;
	uint32_t remain;
	int64_t now;
	uint32_t tail;
	const bool allow_nudge = (UDC_DWC3_IN_NUDGE_EP_MASK & BIT(idx)) != 0U;

	if (idx >= cfg->num_in_eps || idx >= ARRAY_SIZE(udc_dwc3_park)) {
		return;
	}

	ep_data = &cfg->ep_data_in[idx];
	if (ep_data->trb_buf == NULL || udc_dwc3_ep_is_hw_in(ep_data)) {
		return;
	}

	park = &udc_dwc3_park[idx];
	now = k_uptime_get();
	if (now < park->cooldown_until) {
		return;
	}

	tail = ep_data->tail;
	buf = ep_data->net_buf[tail];
	if (buf == NULL) {
		park->since = 0;
		park->nudges = 0;
		return;
	}

	trb = &ep_data->trb_buf[tail];
	/* Do not software-retire HWO=0 — wait for real DEPEVT only. */
	if ((trb->ctrl & UDC_DWC3_TRB_CTRL_HWO) == 0U) {
		park->since = 0;
		park->nudges = 0;
		return;
	}

	if (!allow_nudge) {
		park->since = 0;
		park->nudges = 0;
		return;
	}

	remain = FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb->status);
	if (park->since == 0 || park->tail != tail || park->remain != remain) {
		park->since = now;
		park->tail = tail;
		park->remain = remain;
		park->nudges = 0;
		return;
	}

	if ((now - park->since) < CONFIG_UDC_DWC3_IN_PARK_RECOVER_MS) {
		return;
	}

	if (ep_data->cfg.addr == 0x82) {
		if (udc_dwc3_park0_snap == 0U) {
			udc_dwc3_park_snapshot(dev, "A");
			udc_dwc3_park0_snap = 1U;
			udc_dwc3_park0_t0 = now;
		} else if (udc_dwc3_park0_snap == 1U &&
			   (now - udc_dwc3_park0_t0) >= 100) {
			udc_dwc3_park_snapshot(dev, "B");
			udc_dwc3_park0_snap = 2U;
		}
	}

#if defined(CONFIG_UDC_DWC3_IN_START_ENDXFER_ESCALATE)
	/*
	 * Enqueue already double-UpdateXfer'd. IEBM 1 Hz probe: 0x82 stays
	 * HWO through STREAMOFF, so another nudge cannot unstick it.
	 */
	if (park->nudges >= 1U || ep_data->cfg.addr == 0x82) {
		/*
		 * 0x82 under Y16: UpdateXfer never fetches. Immediate
		 * EndXfer+StartXfer every 400 ms (3682 times in 8 min)
		 * starved video DEPCMD and froze Y16 at the ACM-death
		 * tick. Three retakes, then 30 s quiet.
		 */
		if (ep_data->cfg.addr == 0x82 && park->nudges >= 3U) {
			printk("IN-RECOVER: quiet ep=0x82 remain=%u\n", remain);
			park->nudges = 0;
			park->since = now;
			/* 30 s left the host 9-byte read dead. 2 s is enough
			 * for video to run after three failed retakes. */
			park->cooldown_until = now + 2000;
			return;
		}
#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
		if (udc_dwc3_nv_retake < 8U)
#endif
		{
			printk("IN-RECOVER: retake ep=0x%02x tail=%u remain=%u n=%u\n",
			       ep_data->cfg.addr, tail, remain, park->nudges);
		}
		udc_dwc3_in_park_retake(dev, ep_data);
		park->nudges++;
		park->since = now;
		park->cooldown_until = now + UDC_DWC3_IN_PARK_BACKOFF_MS;
		return;
	}
#endif

	if (park->nudges >= UDC_DWC3_IN_PARK_NUDGE_CAP) {
		printk("IN-RECOVER: backoff ep=0x%02x remain=%u nudges=%u\n",
		       ep_data->cfg.addr, remain, park->nudges);
		park->nudges = 0;
		park->since = now;
		park->cooldown_until = now + UDC_DWC3_IN_PARK_BACKOFF_MS;
		return;
	}

	if (!udc_dwc3_depcmd_is_idle(dev, ep_data->epn)) {
		const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

		printk("IN-RECOVER: busy ep=0x%02x cmd=0x%08x — NoResp nudge\n",
		       ep_data->cfg.addr,
		       sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn)));
	}

	printk("IN-RECOVER: park ep=0x%02x nudge remain=%u n=%u\n",
	       ep_data->cfg.addr, remain, park->nudges);
	udc_dwc3_depcmd_update_xfer(dev, ep_data);
	park->nudges++;
	park->since = now;
	park->cooldown_until = now + UDC_DWC3_IN_PARK_COOLDOWN_MS;
}

static void udc_dwc3_in_recover_tick(const struct device *const dev)
{
	uint32_t mask = CONFIG_UDC_DWC3_IN_RECOVER_EP_MASK;

	while (mask != 0U) {
		const uint8_t idx = (uint8_t)__builtin_ctz(mask);

		mask &= ~BIT(idx);
		udc_dwc3_in_recover_one(dev, idx);
	}
}

#endif /* CONFIG_UDC_DWC3_IN_PARK_RECOVER */

#if defined(CONFIG_UDC_DWC3_CDC_TRACE)
/*
 * CDC endpoint stall trace on the console.
 *
 * With the shell on UART this is the only view of CDC state once the ACM data
 * pipe goes silent under Soft-IP isoc. Read the printed nb/hwo pair as:
 *   nb=0        nothing queued — the stall is upstream of this driver
 *   nb=1 hwo=1  TRB handed to hardware and never completed (lost doorbell)
 *   nb=1 hwo=0  completion arrived but the ring was not retired
 */
#define UDC_DWC3_CDC_TRACE_STALL_MS 2000
#define UDC_DWC3_CDC_TRACE_PERIOD_MS 5000

struct udc_dwc3_cdc_trace {
	int64_t since;
	int64_t last_print;
	uint32_t sig;
};

static void udc_dwc3_cdc_trace_ep(const struct device *const dev,
				  struct udc_dwc3_ep_data *const ep_data,
				  struct udc_dwc3_cdc_trace *const tr,
				  const int64_t now)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t tail = ep_data->tail;
	volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[tail];
	const uint32_t ctrl = trb->ctrl;
	const uint32_t status = trb->status;
	const bool armed = ep_data->net_buf[tail] != NULL;
	const bool hwo = (ctrl & UDC_DWC3_TRB_CTRL_HWO) != 0U;
	const uint32_t sig = (tail << 24) | (ep_data->head << 16) |
			     ((uint32_t)armed << 9) | ((uint32_t)hwo << 8) |
			     (status & 0xFFU);

	if (tr->sig != sig || tr->since == 0) {
		tr->sig = sig;
		tr->since = now;
		return;
	}

	if ((now - tr->since) < UDC_DWC3_CDC_TRACE_STALL_MS ||
	    (now - tr->last_print) < UDC_DWC3_CDC_TRACE_PERIOD_MS) {
		return;
	}

	tr->last_print = now;
	printk("CDC-TRACE ep=0x%02x act=%u hd=%u tl=%u nb=%u hwo=%u rem=%u "
	       "sts=0x%08x depcmd=0x%08x dsts=0x%08x quiet_ms=%lld\n",
	       ep_data->cfg.addr, ep_data->xfer_active, ep_data->head, tail,
	       armed, hwo,
	       (uint32_t)FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, status),
	       status, sys_read32(base + UDC_DWC3_DEPCMD(ep_data->epn)),
	       sys_read32(base + UDC_DWC3_DSTS), now - tr->since);
}

static void udc_dwc3_cdc_trace_tick(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	static struct udc_dwc3_cdc_trace trace_in[8];
	static struct udc_dwc3_cdc_trace trace_out[8];
	const int64_t now = k_uptime_get();

	for (int i = 1; i < cfg->num_in_eps && i < (int)ARRAY_SIZE(trace_in); i++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[i];

		if (ep_data->trb_buf == NULL || !ep_data->cfg.stat.enabled ||
		    udc_dwc3_ep_is_hw_in(ep_data)) {
			continue;
		}

		udc_dwc3_cdc_trace_ep(dev, ep_data, &trace_in[i], now);
	}

	for (int i = 1; i < cfg->num_out_eps && i < (int)ARRAY_SIZE(trace_out); i++) {
		struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[i];

		if (ep_data->trb_buf == NULL || !ep_data->cfg.stat.enabled) {
			continue;
		}

		udc_dwc3_cdc_trace_ep(dev, ep_data, &trace_out[i], now);
	}
}
#endif /* CONFIG_UDC_DWC3_CDC_TRACE */

#define NORMAL_EP(n, fn) fn(n + 2)

static void udc_dwc3_handle_event(const struct device *const dev, const uint32_t evt_raw)
{
	const uint32_t evt = evt_raw & UDC_DWC3_EVT_MASK;
	static uint32_t ev85;

	if ((evt_raw & 1U) == 0U) { /* DEPEVT: tally per physical epn */
		udc_dwc3_nv_evt[(evt_raw >> 1) & 0x1fU]++;
	} else {
		udc_dwc3_nv_evt[0]++; /* DEVT bucket */
	}

	/* Physical epn 11 = 0x85 IN. Print the first few DEPEVTs. */
	if (((evt_raw >> 1) & 0x1f) == 11U && ev85 < 0U) { /* off: evt thread printk (~6 ms) overflows the ring */
		printk("dwc3: evt 0x85 raw=0x%08x\n", evt_raw);
		ev85++;
	}

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
		LOG_DBG("DEPEVT_XFERINPROGRESS");
		udc_dwc3_on_xfer_done_norm(dev, evt);
		break;
	case UDC_DWC3_DEPEVT_EPCMDCMPLT(0):
	case UDC_DWC3_DEPEVT_EPCMDCMPLT(1):
	case LISTIFY(30, NORMAL_EP, (: case), UDC_DWC3_DEPEVT_EPCMDCMPLT):
		/*
		 * CMDACT=1 (mailbox + video UpdateXfer) posts one of these per
		 * command. Completion is the CMDACT poll; do not LOG_ERR —
		 * a printk here refills the event ring and kills ACM under Y16.
		 */
		break;
	case UDC_DWC3_DEPEVT_XFERNOTREADY(0):
	case UDC_DWC3_DEPEVT_XFERNOTREADY(1):
		udc_dwc3_on_xfer_not_ready(dev, evt);
		break;
	case LISTIFY(30, NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERNOTREADY):
		/* Linux isoc: StartXfer from XferNotReady EventParam UF. */
		udc_dwc3_on_isoc_xfer_not_ready(dev, evt_raw);
#if defined(CONFIG_UDC_DWC3_OUT_NOTREADY_RETAKE)
		udc_dwc3_on_xfer_not_ready_norm(dev, evt);
#endif
		break;
	case UDC_DWC3_DEVT_DISCONNEVT:
		LOG_DBG("DEVT_DISCONNEVT");
		udc_dwc3_hw_owned_revoke_all(dev, "disconnect");
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
		/*
		 * Erratic error / event overflow: both used to be
		 * CODE_UNREACHABLE, i.e. undefined behaviour in a release build.
		 * The event ring stays consistent (GEVNTCOUNT is acknowledged per
		 * entry), so count, log and keep draining: recovery must not
		 * depend on the next event, and a wedge here takes ACM and video
		 * down together.
		 */
		udc_dwc3_evt_errors.erratic++;
		printk("dwc3: DEVT_ERRTICERR (n=%u)\n", udc_dwc3_evt_errors.erratic);
		break;
	case UDC_DWC3_DEVT_EVNTOVERFLOW:
		udc_dwc3_evt_errors.overflow++;
		/*
		 * Rate-limited: one ~70-char line costs ~6 ms at 115200 and the
		 * IEBM fast lane posts ~9600 DEPEVTs/s, so a printk per overflow
		 * event refilled the 128-entry ring before the thread got back
		 * to it (self-sustaining storm, 14k lines on the bench).
		 */
		if ((udc_dwc3_evt_errors.overflow & (udc_dwc3_evt_errors.overflow - 1U)) == 0U ||
		    (udc_dwc3_evt_errors.overflow % 4096U) == 0U) {
			printk("dwc3: DEVT_EVNTOVERFLOW (n=%u) — raise CONFIG_UDC_DWC3_EVENTS_NUM\n",
			       udc_dwc3_evt_errors.overflow);
		}
		if (udc_dwc3_evt_errors.overflow == 1U || udc_dwc3_evt_errors.overflow == 65536U) {
			const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

			printk("dwc3: ovf trace total=%u cnt=%u:", udc_dwc3_evt_trace_total,
			       sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)));
			for (uint32_t i = 0; i < UDC_DWC3_EVT_TRACE_N; i++) {
				printk(" %08x", udc_dwc3_evt_trace[(udc_dwc3_evt_trace_idx + i) %
								   UDC_DWC3_EVT_TRACE_N]);
			}
			printk("\n");
#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
			if (iebm_fast.ep != NULL) {
				printk("dwc3: ovf 0x85 head=%u tail=%u hwo=0x%x xa=%d idx=0x%x full=%d\n",
				       iebm_fast.ep->head, iebm_fast.ep->tail,
				       udc_dwc3_ring_data_hwo_mask(iebm_fast.ep),
				       iebm_fast.ep->xfer_active, iebm_fast.ep->xferrscidx,
				       iebm_fast.ep->full);
				lattice_usb23_iebm_fast_health(dev);
			}
#endif
		}
		break;
	default:
		udc_dwc3_evt_errors.unknown++;
		if ((udc_dwc3_evt_errors.unknown & (udc_dwc3_evt_errors.unknown - 1U)) == 0U) {
			printk("dwc3: unhandled event 0x%x (n=%u)\n",
			       evt, udc_dwc3_evt_errors.unknown);
		}
	}
}

static void udc_dwc3_evt_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *const dev = arg1;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct udc_dwc3_config *const cfg = dev->config;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	udc_dwc3_health_dev = dev;
	printk("PARK0 fence+snap ready (TRB/PAR readback, A/B dump on 0x82 park)\n");

	while (true) {
		const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
		int ret;

		/*
		 * Drain until idle *before* sleeping. The lost-wakeup was:
		 * ISR masks + k_sem_give, thread consumes that give and
		 * empties the ring, a later event is posted while still
		 * masked (no second IRQ / no second give), thread then
		 * sleeps. GEVNTCOUNT > 0, mask set, sem = 0 until the
		 * backstop. Do not take the sem while the ring is live.
		 */
		for (;;) {
		if ((k_uptime_get() - udc_dwc3_health_last_ms) >= 5000) {
			udc_dwc3_health_last_ms = k_uptime_get();
			udc_dwc3_health_print();
			udc_dwc3_rateprobe_print();
		}

#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
		/* ACM/EP0 events the ISR lifted off the HW ring. */
		for (;;) {
			uint32_t stolen;
			unsigned int key = irq_lock();

			if (!iebm_swq_get(&stolen)) {
				irq_unlock(key);
				break;
			}
			irq_unlock(key);
			udc_dwc3_handle_event(dev, stolen);
		}
#endif

		do {
			bool yield_video = false;

			for (;;) {
				/*
				 * Pop atomically: the ISR fast lane also consumes
				 * video DEPEVTs from this ring while the interrupt
				 * is unmasked (backstop wake, or after the unmask
				 * below). Ack before handling; the entry is copied.
				 */
				unsigned int key = irq_lock();
				uint32_t evt;

				if (sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) == 0U) {
					irq_unlock(key);
					break;
				}
#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
				/*
				 * Video DEPEVT at the head: do not ack it.
				 * Leave the ring to the ISR fast lane. Acking
				 * here without refill dropped us to ~3 fps;
				 * servicing here held irq_lock and killed ACM.
				 */
				if (iebm_fast.ep != NULL &&
				    udc_dwc3_iebm_is_video_evt(cfg->evt_buf[priv->evt_next])) {
					irq_unlock(key);
					iebm_fast.n_evt_thread++;
					yield_video = true;
					break;
				}
#endif
				evt = cfg->evt_buf[priv->evt_next];
				sys_write32(sizeof(uint32_t), base + UDC_DWC3_GEVNTCOUNT(0));
				udc_dwc3_ring_inc(&priv->evt_next, CONFIG_UDC_DWC3_EVENTS_NUM);
				udc_dwc3_evt_trace_add(evt, false);
				irq_unlock(key);

				/* Keep EventParam (bits 31:16) for isoc XferNotReady UF. */
				udc_dwc3_handle_event(dev, evt);
			}

			/*
			 * Allow further interrupts. Plain write (not RMW: the
			 * ISR's set_bits could interleave with a read-modify-
			 * write here), then re-check the count: an event that
			 * landed just before the unmask is picked up by the
			 * loop instead of being stranded behind a masked
			 * interrupt.
			 */
			sys_write32(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t),
				    base + UDC_DWC3_GEVNTSIZ(0));
			if (yield_video) {
				break;
			}
		} while (sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) > 0);

#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
		for (;;) {
			uint32_t stolen;
			unsigned int key = irq_lock();

			if (!iebm_swq_get(&stolen)) {
				irq_unlock(key);
				break;
			}
			irq_unlock(key);
			udc_dwc3_handle_event(dev, stolen);
		}
#endif

#if defined(CONFIG_UDC_DWC3_IN_COMPLETION_POLL)
		udc_dwc3_in_completion_poll_tick(dev);
#endif
#if defined(CONFIG_UDC_DWC3_OUT_COMPLETION_POLL)
		udc_dwc3_out_completion_poll_tick(dev);
#endif
#if defined(CONFIG_UDC_DWC3_IN_PARK_RECOVER)
		udc_dwc3_in_recover_tick(dev);
#endif
#if defined(CONFIG_UDC_DWC3_OUT_STALL_REFRESH)
		udc_dwc3_out_stall_refresh_tick(dev);
#endif
#if defined(CONFIG_UDC_DWC3_CDC_TRACE)
		udc_dwc3_cdc_trace_tick(dev);
#endif

			/*
			 * Last look with IRQs blocked so the ISR cannot
			 * mask+give in the gap. If the ring is empty, unmask
			 * and re-check (DWC3 does not always IRQ for events
			 * that were already pending at unmask). Only then sleep.
			 */
			{
				unsigned int key = irq_lock();
				uint32_t cnt = sys_read32(base + UDC_DWC3_GEVNTCOUNT(0));

				if (cnt == 0U) {
					sys_write32(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t),
						    base + UDC_DWC3_GEVNTSIZ(0));
					cnt = sys_read32(base + UDC_DWC3_GEVNTCOUNT(0));
				}
				irq_unlock(key);
				if (cnt == 0U) {
					break;
				}
			}
		}

		/*
		 * Backstop: if a wakeup is still lost (mask set, ring
		 * pending, sem = 0) drain shortly instead of wedging.
		 */
#if defined(CONFIG_UDC_DWC3_EVENT_BACKSTOP)
		ret = k_sem_take(&priv->evt_sem,
				 udc_dwc3_evt_fast ? K_MSEC(2) : K_MSEC(100));
		if (ret == -EAGAIN &&
		    sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) > 0 &&
		    (sys_read32(base + UDC_DWC3_GEVNTSIZ(0)) & UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK)) {
			LOG_WRN("event ring serviced by timeout backstop (lost wakeup)");
		}
#else
		ret = k_sem_take(&priv->evt_sem, K_FOREVER);
#endif
		ARG_UNUSED(ret);
	}
}

__ramfunc static void udc_dwc3_irq_handler(void *const ptr)
{
	const struct device *const dev = ptr;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

#if !defined(CONFIG_UDC_DWC3_RTL_DOORBELL)
	/*
	 * IEBM fast lane: consume video DEPEVTs at the head of the ring here,
	 * retire + refill + UpdateXfer, and only wake the thread for the rest.
	 * Stops at the first non-video event so ordering is preserved.
	 */
	if (iebm_fast.ep != NULL) {
		const struct udc_dwc3_config *const cfg = dev->config;
		const uint32_t epn = (uint32_t)iebm_fast.ep->epn;
		uint32_t n = 0U;
		uint32_t stole = 0U;

		while (sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) > 0U) {
			const uint32_t evt_raw = cfg->evt_buf[priv->evt_next];
			const uint32_t evt = evt_raw & UDC_DWC3_EVT_MASK;

			if (evt != UDC_DWC3_DEPEVT_XFERINPROGRESS(epn) &&
			    evt != UDC_DWC3_DEPEVT_XFERCOMPLETE(epn) &&
			    evt != UDC_DWC3_DEPEVT_XFERNOTREADY(epn)) {
				/* Lift ACM/EP0/DEVT off the HW ring so a
				 * video flood cannot overwrite it.
				 */
				if (!iebm_swq_put(evt_raw)) {
					break;
				}
				stole++;
			} else {
				n++;
			}
			sys_write32(sizeof(uint32_t), base + UDC_DWC3_GEVNTCOUNT(0));
			udc_dwc3_ring_inc(&priv->evt_next, CONFIG_UDC_DWC3_EVENTS_NUM);
			udc_dwc3_evt_trace_add(evt_raw, true);
		}
		if (n != 0U) {
			iebm_fast.isr_evts += n;
			iebm_fast.isr_calls++;
			(void)udc_dwc3_iebm_fast_service(dev);
		}
		if (stole != 0U) {
			k_sem_give(&priv->evt_sem);
		}
		if (sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) == 0U) {
			return;
		}
	}
#endif

	/*
	 * Mask event interrupts and wake the dedicated bottom-half thread.
	 * udc_setup_received() (and other UDC helpers) take the UDC mutex and
	 * must not run in ISR context. A dedicated high-priority thread is
	 * used instead of the system workqueue so large control transfers are
	 * not delayed.
	 */
	sys_set_bits(base + UDC_DWC3_GEVNTSIZ(0), UDC_DWC3_GEVNTSIZ_EVNTINTRPTMASK);
	k_sem_give(&priv->evt_sem);
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

	if (udc_dwc3_ep_refuse_if_hw_owned(ep_data, "enqueue")) {
		return -EBUSY;
	}

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

	if (ep_data->hw_owned) {
		/* Class-level teardown of a live video EP: take it back cleanly */
		udc_dwc3_hw_owned_revoke(dev, ep_data, "dequeue", true);
		udc_dwc3_hw_owned_rearm(dev, ep_data);
		udc_ep_cancel_queued(dev, ep_cfg);
		udc_ep_set_busy(ep_cfg, false);
		return 0;
	}

	/* ep_disable may already have ended the transfer (usbd_ep_disable order) */
	if (ep_data->xfer_active) {
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	}
	udc_dwc3_ring_flush(dev, ep_data);

	udc_ep_cancel_queued(dev, ep_cfg);
	udc_ep_set_busy(ep_cfg, false);

	return 0;
}

static int udc_dwc3_ep_disable(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	/* SET_CONFIGURATION(0) / reset path: the RTL loses the endpoint too */
	udc_dwc3_hw_owned_revoke(dev, ep_data, "ep-disable", true);

	/*
	 * Release the transfer resource here (Linux __dwc3_gadget_ep_disable →
	 * dwc3_stop_active_transfer) rather than relying on the caller to
	 * follow up with dequeue. uvcvideo's SET_INTERFACE(alt 0) at probe
	 * disables and re-enables 0x83, so an enable must never stack a second
	 * StartXfer on a still-live resource.
	 */
	if (ep_data->xfer_active && ep_data->xferrscidx != 0U &&
	    (sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0U) {
		udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
		printk("dwc3: ep_disable 0x%02x EndXfer\n", ep_data->cfg.addr);
	}
	ep_data->xfer_active = false;
	ep_data->xferrscidx = 0;
	udc_dwc3_ring_flush(dev, ep_data);

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
		if (udc_dwc3_ep_refuse_if_hw_owned(ep_data, "SetHalt")) {
			return -EBUSY;
		}
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

	if (ep_data->hw_owned) {
		/*
		 * Bulk UVC has no alternate setting, so uvcvideo (Linux and
		 * Windows) signals STREAMOFF with CLEAR_FEATURE(ENDPOINT_HALT)
		 * on the video endpoint. Take the endpoint back from the RTL
		 * first (UsbMgr halt + EndXfer), then ClearStall so the
		 * SuperSpeed sequence number restarts at 0 like the host's,
		 * then re-arm an empty ring for the next STREAMON.
		 */
		udc_dwc3_hw_owned_revoke(dev, ep_data, "clear-halt", true);
		udc_dwc3_depcmd_clear_stall(dev, ep_data);
		udc_dwc3_hw_owned_rearm(dev, ep_data);
		ep_data->cfg.stat.halted = false;
		return 0;
	}

	udc_dwc3_depcmd_clear_stall(dev, ep_data);
	ep_data->cfg.stat.halted = false;

	return 0;
}

static int udc_dwc3_set_address(const struct device *const dev, const uint8_t addr)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	LOG_INF("Setting address to %u", addr);

	/* Configure the new address */
	reg = sys_read32(base + UDC_DWC3_DCFG);
	reg &= ~UDC_DWC3_DCFG_DEVADDR_MASK;
	reg |= FIELD_PREP(UDC_DWC3_DCFG_DEVADDR_MASK, addr);
	sys_write32(reg, base + UDC_DWC3_DCFG);

	/* Re-apply the same endpoint configuration */
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0]);
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_out[0]);

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

	udc_dwc3_evt_fast = false;
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
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	LOG_DBG("%s 0x%02x", __func__, ep_data->cfg.addr);

	/* Never re-enable over a live handoff (missed STREAMOFF / reset) */
	udc_dwc3_hw_owned_revoke(dev, ep_data, "ep-enable", false);

	memset(ep_data->trb_buf, 0, sizeof(*ep_data->trb_buf) * CONFIG_UDC_DWC3_TRB_NUM);

	/*
	 * Allocate the non-control transfer resource pool before configuring
	 * the first endpoint that needs it. DEPSTARTCFG reassigns the whole
	 * pool, so this runs once per configuration.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 &&
	    !DEV_DATA(dev)->startcfg_nonctrl_done) {
		const struct udc_dwc3_config *const cfg = dev->config;

		DEV_DATA(dev)->startcfg_nonctrl_done = true;
		udc_dwc3_depcmd_start_config(dev, &cfg->ep_data_out[0], 2U);
		/* Fresh pool: every non-control endpoint must allocate again */
		for (int epn = 1; epn < cfg->num_in_eps; epn++) {
			cfg->ep_data_in[epn].xfer_rsc_allocated = false;
		}
		for (int epn = 1; epn < cfg->num_out_eps; epn++) {
			cfg->ep_data_out[epn].xfer_rsc_allocated = false;
		}
		printk("dwc3: DEPSTARTCFG non-control pool (first ep=0x%02x)\n",
		       ep_data->cfg.addr);
	}

	udc_dwc3_depcmd_ep_config(dev, ep_data);

	/*
	 * One transfer resource per endpoint per DEPSTARTCFG round. A re-enable
	 * without a bus reset keeps the resource (and xferrscidx) it already has.
	 */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0 || !ep_data->xfer_rsc_allocated) {
		udc_dwc3_depcmd_ep_xfer_config(dev, ep_data);
		ep_data->xfer_rsc_allocated = true;
	}

	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		udc_dwc3_trb_norm_init(dev, ep_data);
		if (ep_data->xfer_active) {
			printk("dwc3: ep_enable 0x%02x StartXfer xferrscidx=0x%x\n",
			       ep_data->cfg.addr, ep_data->xferrscidx);
		}
	}

	/* Starting from here, the endpoint can be used */
	sys_set_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

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

	printk("dwc3: pre-reset GCTL=0x%08x GUSB3PIPECTL=0x%08x GUSB2PHYCFG=0x%08x GHWPARAMS7=0x%08x\n",
	       sys_read32(base + UDC_DWC3_GCTL), sys_read32(base + UDC_DWC3_GUSB3PIPECTL),
	       sys_read32(base + UDC_DWC3_GUSB2PHYCFG), sys_read32(base + UDC_DWC3_GHWPARAMS7));

#if CONFIG_UDC_DWC3_SKIP_GCTL_SOFTRESET
	/*
	 * TinyCLUNX Lattice USB23: GCTL.CORESOFTRESET / PHYSOFTRST make
	 * GHWPARAMS7 read 0 and SuperSpeed enum dies. Skip both;
	 * udc_dwc3_on_soft_reset() still does DCTL.CSFTRST.
	 */
	printk("dwc3: skip GCTL/PHY softrst (GHWPARAMS7=0x%08x GCTL=0x%08x)\n",
	       sys_read32(base + UDC_DWC3_GHWPARAMS7),
	       sys_read32(base + UDC_DWC3_GCTL));
#else
	/* Documented Synopsys sequence — required on FLIR UAB hybrid_cmdact. */
	sys_set_bits(base + UDC_DWC3_GCTL, UDC_DWC3_GCTL_CORESOFTRESET);
	sys_set_bits(base + UDC_DWC3_GUSB3PIPECTL, UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST);
	sys_set_bits(base + UDC_DWC3_GUSB2PHYCFG, UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST);
	k_sleep(K_USEC(100));
	sys_clear_bits(base + UDC_DWC3_GUSB3PIPECTL, UDC_DWC3_GUSB3PIPECTL_PHYSOFTRST);
	sys_clear_bits(base + UDC_DWC3_GUSB2PHYCFG, UDC_DWC3_GUSB2PHYCFG_PHYSOFTRST);
	sys_clear_bits(base + UDC_DWC3_GCTL, UDC_DWC3_GCTL_CORESOFTRESET);
	printk("dwc3: GCTL/PHY softrst done GCTL=0x%08x GHWPARAMS7=0x%08x\n",
	       sys_read32(base + UDC_DWC3_GCTL),
	       sys_read32(base + UDC_DWC3_GHWPARAMS7));
#endif

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
		if (ep_data->cfg.addr == 0x85) {
			printk("dwc3: 0x85 worker skip halted\n");
		}
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
	k_sem_init(&DEV_DATA(dev)->evt_sem, 0, K_SEM_MAX_LIMIT);
	k_thread_create(&DEV_DATA(dev)->evt_thread, DEV_DATA(dev)->evt_stack,
			CONFIG_UDC_DWC3_THREAD_STACK_SIZE,
			udc_dwc3_evt_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_UDC_DWC3_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&DEV_DATA(dev)->evt_thread, "udc_dwc3");

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
	K_THREAD_STACK_DEFINE(udc_dwc3_evt_stack_##n,				\
			      CONFIG_UDC_DWC3_THREAD_STACK_SIZE);		\
										\
	static struct udc_dwc3_data udc_dwc3_priv_##n = {			\
		.dev = DEVICE_DT_INST_GET(n),					\
		.evt_stack = udc_dwc3_evt_stack_##n,				\
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
