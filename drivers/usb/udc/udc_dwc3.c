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

#include <zephyr/drivers/usb/udc.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include "udc_common.h"

/* TRB memory buffer fields */
#define UDC_DWC3_TRB_STATUS_BUFSIZ_MASK				GENMASK(23, 0)
#define UDC_DWC3_TRB_STATUS_PCM1_MASK				GENMASK(25, 24)
#define UDC_DWC3_TRB_STATUS_SPR					BIT(26)
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
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_MASK			GENMASK(13, 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_SETUP			(0x0 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_DATA			(0x1 << 12)
#define UDC_DWC3_DEPEVT_STATUS_CONTROL_STATUS			(0x2 << 12)
#define UDC_DWC3_DEPEVT_STATUS_SHORT				BIT(13)
#define UDC_DWC3_DEPEVT_STATUS_IOC				BIT(14)
#define UDC_DWC3_DEPEVT_STATUS_LST				BIT(15)
#define UDC_DWC3_DEPEVT_STATUS_MISSED_ISOC			BIT(15)
#define UDC_DWC3_DEPEVT_STATUS_STREAMFOUND			(0x1 << 12)
#define UDC_DWC3_DEPEVT_STATUS_STREAMNOTFOUND			(0x2 << 12)
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
#define UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK			GENMASK(19, 16)
#define UDC_DWC3_DEVT_EVTINFO_LINKSTATE_			GENMASK(19, 16)
#define UDC_DWC3_DEVT_EVTINFO_SS				BIT(20)

/*
 * The point at which the ring has no room left to give.
 *
 * SPEC, Programming Guide 3.30b: "The controller always leaves one entry free
 * in each Event Buffer." So the fullest GEVNTCOUNT can legitimately report is
 * one entry short of the ring, and that - not the whole ring - is the
 * threshold worth reacting to.
 */
#define UDC_DWC3_EVT_RING_FULL_BYTES				\
	(((CONFIG_UDC_DWC3_EVENTS_NUM) - 1u) * sizeof(uint32_t))

#define UDC_DWC3_SETUP_STUCK_RESET
#define UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE			0xFFFFFFFFu
#define UDC_DWC3_DEVT_VNDRDEVTSTRCVED				(BIT(0) | (0xc << 8))

/* Device Endpoint Commands and Parameters */
#define UDC_DWC3_DEPCMDPAR2(n)					(0xc800 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR1(n)					(0xc804 + 16 * (n))
#define UDC_DWC3_DEPCMDPAR0(n)					(0xc808 + 16 * (n))
#define UDC_DWC3_DEPCMD(n)					(0xc80c + 16 * (n))
/* Common fields to DEPCMD */
#define UDC_DWC3_DEPCMD_HIPRI_FORCERM				(1 << 11)
/*
 * Command Interrupt On Completion. Asks the controller to raise an Endpoint
 * Command Complete event (DEPEVT_EPCMDCMPLT) once the command has finished.
 *
 * This matters on End Transfer: CmdAct clearing means only that the command was
 * ACCEPTED, whereas the databook states the controller "will wait until it can
 * complete operations for the endpoint before returning the Command Complete
 * event". Without it there is no way to know that DMA for the ended transfer
 * has actually stopped before a fresh Start Transfer is issued.
 *
 * The field must not be set while DCTL.RunStop is 0.
 */
#define UDC_DWC3_DEPCMD_CMDIOC					BIT(8)
#define UDC_DWC3_DEPCMD_STATUS_MASK				GENMASK(15, 12)
#define UDC_DWC3_DEPCMD_STATUS_OK				(0 << 12)
#define UDC_DWC3_DEPCMD_STATUS_CMDERR				(1 << 12)
#define UDC_DWC3_DEPCMD_XFERRSCIDX_MASK				GENMASK(22, 16)
#define UDC_DWC3_XFERRSCIDX_INVALID				0xffU
/* DEPCFG Command and Parameters */
/* Command type occupies bits 3:0 - DEPCFG(1) through DEPSTARTCFG(9). */
#define UDC_DWC3_DEPCMD_CMDTYP_MASK				GENMASK(3, 0)
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
/* Global Rx Threshold Control Register (databook 1.2.4) */
#define UDC_DWC3_GRXTHRCFG					0xc10c
#define UDC_DWC3_GRXTHRCFG_USBRXPKTCNTSEL			BIT(29)
#define UDC_DWC3_GRXTHRCFG_USBRXPKTCNT_MASK			GENMASK(27, 24)
/*
 * Apply the databook 1.2.4 erratum workaround - clear GRXTHRCFG.UsbRxPktCntSel
 * so a fixed NUMP is transmitted rather than one derived from the RX threshold.
 * See udc_dwc3_on_soft_reset() for the citations.
 *
 * Set to 0 to leave the register untouched for an A/B comparison. The value the
 * core powered up with is logged either way.
 */
#define UDC_DWC3_RX_THRESHOLD_WORKAROUND 1

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
#define UDC_DWC3_GCTL_RAMCLKSEL_BUS_CLK				(0x00 << 6)
#define UDC_DWC3_GCTL_RAMCLKSEL_PIPE_CLK			(0x01 << 6)
#define UDC_DWC3_GCTL_RAMCLKSEL_PIPE_DIV2_CLK			(0x02 << 6)
#define UDC_DWC3_GCTL_RAMCLKSEL_MAC2_CLK			(0x03 << 6)
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
/*
 * DWC_usb3 Programming Guide 3.30b, section 1.2.56 GEVNTCOUNT(#n), Table 1-68
 * "Fields for Register: GEVNTCOUNT(#n)" (p.197): bits 15:0 EVNTCOUNT, bits 30:16
 * reserved, bit 31 EVNT_HANDLER_BUSY.
 *
 * The same section states the rule the drain is built around, verbatim:
 *
 *   "Clock crossing delays may result in the continuous assertion of the
 *    interrupt after software acknowledges the last event. Therefore, when the
 *    interrupt line is asserted, software must read the GEVNTCOUNT register and
 *    only process events if the GEVNTCOUNT is greater than 0."
 *
 * Note what that permits and what it does not. It licenses ONE read per
 * assertion, taken before processing. It does not license re-reading the
 * register after acknowledging and treating the result as a fresh count: by the
 * databook's own statement that value may still reflect events already handed
 * back. See udc_dwc3_evt_drain(), which reads it exactly once.
 *
 * Only bits 15:0 are the count. Bit 31 is EVNT_HANDLER_BUSY
 * and 30:16 are reserved, so the raw register must never be compared against zero -
 * any of those bits set makes an empty buffer look non-empty, and the drain then
 * reads a slot that legitimately holds nothing and waits out the whole poll budget
 * on it. That is indistinguishable from a lost write in the logs.
 *
 * Read the count through this; keep the raw register only where the point is to
 * SEE those upper bits.
 */
#define UDC_DWC3_GEVNTCOUNT_MASK				GENMASK(15, 0)
#define UDC_DWC3_GEVNTCOUNT_EVNT_HANDLER_BUSY			BIT(31)

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
#define UDC_DWC3_DSTS_RXFIFOEMPTY				BIT(17)
#define UDC_DWC3_DSTS_SOFFN_MASK				GENMASK(16, 3)
#define UDC_DWC3_DSTS_CONNECTSPD_MASK				GENMASK(2, 0)
#define UDC_DWC3_DSTS_CONNECTSPD_HS				(0x0 << 0)
#define UDC_DWC3_DSTS_CONNECTSPD_FS				(0x1 << 0)
#define UDC_DWC3_DSTS_CONNECTSPD_SS				(0x4 << 0)

/* Fields common to both UDC_DWC3_DSTS_USBLNKST and UDC_DWC3_DEVT_EVTINFO_LINKSTATE,
 * apply respective offset for reading/writing these registers.
 */
#define UDC_DWC3_LINK_STATE_USB3_U0				0x0
#define UDC_DWC3_LINK_STATE_USB3_U1				0x1
#define UDC_DWC3_LINK_STATE_USB3_U2				0x2
#define UDC_DWC3_LINK_STATE_USB3_U3				0x3
#define UDC_DWC3_LINK_STATE_USB3_SS_DIS				0x4
#define UDC_DWC3_LINK_STATE_USB3_RX_DET				0x5
#define UDC_DWC3_LINK_STATE_USB3_SS_INACT			0x6
#define UDC_DWC3_LINK_STATE_USB3_POLL				0x7
#define UDC_DWC3_LINK_STATE_USB3_RECOV				0x8
#define UDC_DWC3_LINK_STATE_USB3_HRESET				0x9
#define UDC_DWC3_LINK_STATE_USB3_CMPLY				0xa
#define UDC_DWC3_LINK_STATE_USB3_LPBK				0xb
#define UDC_DWC3_LINK_STATE_USB3_RESET_RESUME			0xf
#define UDC_DWC3_LINK_STATE_USB2_ON_STATE			0x0
#define UDC_DWC3_LINK_STATE_USB2_SLEEP_STATE			0x2
#define UDC_DWC3_LINK_STATE_USB2_SUSPEND_STATE			0x3
#define UDC_DWC3_LINK_STATE_USB2_DISCONNECTED			0x4
#define UDC_DWC3_LINK_STATE_USB2_EARLY_SUSPEND			0x5
#define UDC_DWC3_LINK_STATE_USB2_RESET				0xe
#define UDC_DWC3_LINK_STATE_USB2_RESUME				0xf

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
/* DGCMDPAR for 09h: [4:0] FIFO number, [5] 1 = TX FIFO, 0 = RX FIFO. */
#define UDC_DWC3_DGCMD_FIFOFLUSH_NUM_MASK			GENMASK(4, 0)
#define UDC_DWC3_DGCMD_FIFOFLUSH_TX				BIT(5)
/* Generic commands retire in microseconds; this is a wedge guard, not a wait. */
#define UDC_DWC3_DGCMD_POLL_MAX					1000u
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
#define _EPN_IS_VALID(cfg, epn) \
	(((epn) & 1) ? ((uint32_t)((epn) >> 1) < (cfg)->num_in_eps) \
		     : ((uint32_t)((epn) >> 1) < (cfg)->num_out_eps))
#define _NUM_FIFO_SPACE 16
#define _NUM_AUX_EVENT 8
#define _NUM_FIFO_REGS 8

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
	/*
	 * DMA-accessible scratch the control OUT data stage points its trailing
	 * alignment TRB at.  Never read: it exists only so the controller has
	 * somewhere to put the bytes it insists on being able to receive.
	 */
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
	/* To re-queue cancelled buffers after an endpoint is disabled */
	struct k_fifo requeue_fifo;
	/* Point back to the device for work queues */
	const struct device *dev;
	struct udc_dwc3_trb trb_cache[2];
	/* Buffer of pointers to net_buf, with index matching the position in the TRB buffers */
	struct net_buf *net_buf[CONFIG_UDC_DWC3_TRB_NUM];
	/* Buffer of TRB structures, with index matching the position in the net_buf buffers */
	struct udc_dwc3_trb *trb_buf;
	/* Index of the next TRB to receive data in the TRB ring, Link TRB excluded */
	uint32_t head;
	uint32_t tail;
	/* When the TRB ring buffer is full */
	bool full;
	/* Given by the hardware for use in endpoint commands */
	uint32_t xferrscidx;
	/*
	 * Set when an End Transfer was issued with CmdIOC, cleared by the resulting
	 * Endpoint Command Complete event. While set, the controller may still be
	 * concluding bus traffic for the ended transfer even though CmdAct has
	 * cleared, so a new Start Transfer on this endpoint is premature.
	 *
	 * A Start Transfer is not issued while this is set: udc_dwc3_ep_resume()
	 * defers instead, and udc_dwc3_on_ep_cmd_cmplt() runs it when the event
	 * arrives. udc_dwc3_depcmd_start_xfer() warns if anything gets past that.
	 */
	bool end_xfer_pending;
	/*
	 * Set when udc_dwc3_ep_resume() was called while end_xfer_pending was still
	 * set on this endpoint, and the resume was therefore postponed. Cleared by
	 * udc_dwc3_on_ep_cmd_cmplt(), which then performs the resume.
	 */
	bool resume_pending;
	/*
	 * Whether the postponed resume meant Init or Modify. Captured when the
	 * resume defers, because by the time it runs the stack has already set
	 * cfg.stat.enabled and the answer can no longer be read off it.
	 */
	bool resume_modify;
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
#if CONFIG_UDC_DWC3_SHELL
	/* FIFO space initial values */
	uint16_t max_bytes_avail[_NUM_FIFO_SPACE][_NUM_FIFO_REGS];
#endif
	/* Updated whenever a packet is submitted */
	uint32_t last_xfer_type;
	uint8_t last_xfer_dir;
	/* Cache that is always up to date (before stack could get time to react) */
	struct usb_setup_packet setup_packet;
	uint32_t evt_copy[CONFIG_UDC_DWC3_EVENTS_NUM];
	/*
	 * The two ends of a control transfer, counted independently.
	 *
	 * A request that runs to completion retires exactly one SETUP and exactly
	 * one status stage, so on a healthy link these two track each other with a
	 * difference of at most one (the request currently in flight). A gap that
	 * GROWS is the signature of the host starting requests it never finishes -
	 * a new SETUP arriving while the previous transfer is still outstanding.
	 * That is the question the -110 (ETIMEDOUT) windows in the host log raise
	 * and that no existing counter answers: ctrl_desync says a stage looked
	 * out of order, but not whether requests are being abandoned wholesale.
	 */
	uint32_t ctrl_status_done;	/* status stages retired (IN and OUT) */
	uint32_t ctrl_trbsts_other;
	/*
	 * One bit per physical endpoint, set the first time this driver writes that
	 * endpoint's DEPCMD register.
	 *
	 * Until that write, the register must not be read. Databook section 1.3.12,
	 * DEPCMD[0:7]: "Several fields (including Command Type) are write-only, so
	 * their read values are undefined. After power-on, prior to issuing the
	 * first endpoint command, the read value of this register is undefined. In
	 * particular, the CmdAct bit may be set after power-on. In this case, it is
	 * safe to issue an endpoint command."
	 *
	 * Cleared on core soft reset, where the same reasoning applies again and
	 * where no command issued earlier can still be running.
	 */
	uint32_t depcmd_issued;
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
 * Runtime flags
 */
 enum {
	UDC_DWC3_CTRL_SETUP = 1,
	UDC_DWC3_CTRL_IN,
	UDC_DWC3_CTRL_OUT,
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
			      struct udc_dwc3_ep_data *const ep_data,
			      const bool modify);
static void udc_dwc3_fifo_flush_tx(const struct device *const dev, const uint8_t fifo);

#ifdef CONFIG_UDC_DWC3_SHELL
static void udc_dwc3_init_fifo_space(const struct device *dev);
static void udc_dwc3_dump_trb(const struct device *dev, struct udc_dwc3_ep_data *ep_data,
			      const struct shell *sh);
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

/* Ceiling on the CSftRst completion wait - see udc_dwc3_on_soft_reset(). */
#define UDC_DWC3_CSFTRST_POLL_US 10u
#define UDC_DWC3_CSFTRST_MAX_POLLS 10000u
#define UDC_DWC3_CMD_FAST_POLLS 32u
#define UDC_DWC3_CMD_FAST_POLL_US 1u
#define UDC_DWC3_CMD_SLOW_POLL_US 1000u
#define UDC_DWC3_CMD_TIMEOUT_MS 100u

static const char *udc_dwc3_get_link_state_name(const uint32_t state);

/*
 * Physical endpoint number behind a DEPCMD register address, for the
 * priv->depcmd_issued bookkeeping. The addresses all come from
 * UDC_DWC3_DEPCMD(n), so this simply undoes that macro.
 */
static inline uint32_t udc_dwc3_depcmd_epn(const uint32_t addr)
{
	return (addr - UDC_DWC3_DEPCMD(0)) / 16u;
}

static bool udc_dwc3_wait_cmdact_zero(const struct device *const dev,
				      const uint32_t addr, uint32_t *const reg_out)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(UDC_DWC3_CMD_TIMEOUT_MS));
	uint32_t reg = 0;

	for (uint32_t i = 0; i < UDC_DWC3_CMD_FAST_POLLS; i++) {
		reg = sys_read32(base + addr);
		if ((reg & UDC_DWC3_DEPCMD_CMDACT) == 0) {
			*reg_out = reg;
			return true;
		}
		k_busy_wait(UDC_DWC3_CMD_FAST_POLL_US);
	}

	LOG_WRN_RATELIMIT("cmdact still set on 0x%x after the fast poll; sleeping "
			  "with the UDC mutex held", addr);

	while (!sys_timepoint_expired(end)) {
		k_sleep(K_USEC(UDC_DWC3_CMD_SLOW_POLL_US));

		reg = sys_read32(base + addr);
		if ((reg & UDC_DWC3_DEPCMD_CMDACT) == 0) {
			*reg_out = reg;
			return true;
		}
	}

	*reg_out = reg;
	return false;
}

static uint32_t udc_dwc3_depcmd(const struct device *const dev,
				const uint32_t addr, const uint32_t cmd)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t epn = udc_dwc3_depcmd_epn(addr);
	const bool needs_result =
		(cmd & UDC_DWC3_DEPCMD_CMDTYP_MASK) == UDC_DWC3_DEPCMD_DEPSTRTXFER;
	uint32_t phycfg_saved;
	uint32_t reg = 0;

	/* TODO: enable again? was skipped for first endpoint somehow */
	//__ASSERT_NO_MSG(udc_dwc3_wait_cmdact_zero(dev, addr, &reg));

	/*
	 * At USB 2.0 speeds the USB2 PHY must not be suspended and L1 sleep must not
	 * be enabled while an endpoint command executes: "if GUSB2PHYCFG[6] or
	 * GUSB2PHYCFG[8] is set to '1', it must be set to '0' prior to issuing this
	 * command". The databook repeats this on every endpoint command. Leaving
	 * them set makes commands take far longer or fail outright.
	 *
	 * Save and restore rather than clear once at init, because these bits belong
	 * to power management and may be turned on from outside this driver. When
	 * both are already clear - which they appear to be on this core - this costs
	 * one register read and no writes.
	 */
	reg = sys_read32(base + UDC_DWC3_GUSB2PHYCFG);
	phycfg_saved = reg & (UDC_DWC3_GUSB2PHYCFG_SUSPHY |
			      UDC_DWC3_GUSB2PHYCFG_ENBLSLPM);
	if (phycfg_saved != 0) {
		sys_write32(reg & ~phycfg_saved, base + UDC_DWC3_GUSB2PHYCFG);
	}

	sys_write32(cmd | UDC_DWC3_DEPCMD_CMDACT, base + addr);

	/*
	 * From here the register has been written, so its read value is defined and
	 * the pre-poll above may use it on the next command for this endpoint.
	 */
	if (epn < 32u) {
		priv->depcmd_issued |= BIT(epn);
	}

	if (!needs_result && phycfg_saved == 0) {
		/*
		 * Nothing to collect and nothing to put back, so the command is left
		 * running and this returns. Whoever issues the next command on this
		 * endpoint waits for it in the pre-poll above.
		 */
		return 0;
	}

	/*
	 * Two reasons to wait here. Start Transfer, because its transfer resource
	 * index is the one result this driver consumes. And any command at all when
	 * the PHY bits had to be cleared: those must stay clear while the command
	 * executes, not merely at the instant it is issued, so putting them back
	 * early could stall the very command that required clearing them. On this
	 * core they are already clear, so that second case never arises.
	 */

	if (!udc_dwc3_wait_cmdact_zero(dev, addr, &reg)) {
		LOG_ERR("command expired");
		reg = UDC_DWC3_DEPCMD_CMDACT;
	}

	if (phycfg_saved != 0) {
		/* Re-read: the command may have changed other fields of this register. */
		sys_set_bits(base + UDC_DWC3_GUSB2PHYCFG, phycfg_saved);
	}

	if (!needs_result) {
		return 0;
	}

	/*
	 * Only report a transfer resource index when the command actually
	 * succeeded. On timeout the status field still reads as OK - the command is
	 * simply still active - so it is caught by the CmdAct bit left set above.
	 *
	 * Returning the field regardless would let one failed command overwrite a
	 * good index with a value the controller never assigned, and every later
	 * Update Transfer and End Transfer on that endpoint would address the wrong
	 * transfer resource, silently and permanently.
	 */
	if ((reg & UDC_DWC3_DEPCMD_CMDACT) != 0) {
		return UDC_DWC3_XFERRSCIDX_INVALID;
	}

	switch (reg & UDC_DWC3_DEPCMD_STATUS_MASK) {
	case UDC_DWC3_DEPCMD_STATUS_OK:
		break;
	case UDC_DWC3_DEPCMD_STATUS_CMDERR:
		LOG_ERR("endpoint command 0x%x, addr 0x%x failed (0x%08x)", cmd, addr, reg);
		return UDC_DWC3_XFERRSCIDX_INVALID;
	default:
		LOG_ERR("command failed with unknown status: 0x%08x", reg);
		return UDC_DWC3_XFERRSCIDX_INVALID;
	}

	return FIELD_GET(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, reg);
}

static void udc_dwc3_depcmd_ep_config(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data,
				      const bool modify)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);
	uint32_t param0 = 0;
	uint32_t param1 = 0;

	LOG_INF("Configuring endpoint 0x%02x with wMaxPacketSize=%u",
		ep_data->cfg.addr, ep_data->cfg.mps);

	if (modify) {
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
	param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_MPS_MASK, mps);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 0);
	} else {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_BRSTSIZ_MASK, 15);
	}

	/* Set the FIFO number, must be 0 for all OUT EPs */
	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		param0 |= FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPCFG_FIFONUM_MASK,
				     ep_data->cfg.addr & 0x7f);
	}

	/* Per-endpoint events */
	param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERINPROGEN;
	param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERCMPLEN;

	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		param1 |= UDC_DWC3_DEPCMDPAR1_DEPCFG_XFERNRDYEN;
	}

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

	LOG_DBG("DepXferConfig: EP%02x", ep_data->cfg.addr);

	reg = FIELD_PREP(UDC_DWC3_DEPCMDPAR0_DEPXFERCFG_NUMXFERRES_MASK, 1);
	sys_write32(reg, base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPXFERCFG);
}

static void udc_dwc3_depcmd_set_stall(const struct device *const dev,
				      struct udc_dwc3_ep_data *const ep_data)
{
	LOG_DBG("DepSetStall: EP%02x", ep_data->cfg.addr);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPSETSTALL);
}

static void udc_dwc3_depcmd_clear_stall(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data,
					uint32_t flags)
{
	LOG_DBG("DepClearStall EP%02x", ep_data->cfg.addr);

	flags |= UDC_DWC3_DEPCMD_DEPCSTALL;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);
}

/* Defined below; needed here to release a transfer resource a rejected Start
 * Transfer could not obtain.
 */
static bool udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags);

static int udc_dwc3_depcmd_start_xfer(const struct device *const dev,
				       struct udc_dwc3_ep_data *const ep_data)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t idx;
	uint32_t reg;

	/* Make sure the device is in U0 state, assuming TX FIFO is empty */
	reg = sys_read32(base + UDC_DWC3_DSTS);
	if ((reg & UDC_DWC3_DSTS_CONNECTSPD_MASK) == UDC_DWC3_DSTS_CONNECTSPD_SS &&
	    (reg & UDC_DWC3_DSTS_USBLNKST_MASK) != UDC_DWC3_LINK_STATE_USB3_U0) {
		reg = sys_read32(base + UDC_DWC3_DCTL);
		reg &= ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;
		reg |= UDC_DWC3_DCTL_ULSTCHNGREQ_REMOTEWAKEUP;
		sys_write32(reg, base + UDC_DWC3_DCTL);

		/* Return the field to 0 so the next request is seen as a change */
		reg &= ~UDC_DWC3_DCTL_ULSTCHNGREQ_MASK;
		sys_write32(reg, base + UDC_DWC3_DCTL);
	}

	sys_write32(HI32((uintptr_t)ep_data->trb_buf), base + UDC_DWC3_DEPCMDPAR0(ep_data->epn));
	sys_write32(LO32((uintptr_t)ep_data->trb_buf), base + UDC_DWC3_DEPCMDPAR1(ep_data->epn));

	idx = udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), UDC_DWC3_DEPCMD_DEPSTRTXFER);
	if (idx == UDC_DWC3_XFERRSCIDX_INVALID) {
		LOG_ERR("Start Transfer failed on EP%02x", ep_data->cfg.addr);
		return -EIO;
	}

	ep_data->xferrscidx = idx;

	LOG_DBG("start EP%02x idx=0x%x", ep_data->cfg.addr, ep_data->xferrscidx);

	return 0;
}

static void udc_dwc3_depcmd_update_xfer(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	uint32_t flags = 0;

	flags |= UDC_DWC3_DEPCMD_DEPUPDXFER;
	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags);

	/* DBG: this fires once per buffer from udc_dwc3_trb_bulk(). */
	LOG_DBG("DepUpdateXfer done EP%02x, addr 0x%08x, data 0x%08x, xferrscidx 0x%x",
		ep_data->cfg.addr, UDC_DWC3_DEPCMD(ep_data->epn), flags, ep_data->xferrscidx);
}

/*
 * Issue End Transfer. Returns true when the command was issued AND will report
 * an Endpoint Command Complete event, which is what a caller needs to know
 * before deciding to wait for one.
 */
static bool udc_dwc3_depcmd_end_xfer(const struct device *const dev,
				     struct udc_dwc3_ep_data *const ep_data,
				     uint32_t flags)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, ep_data->xferrscidx);
	flags |= UDC_DWC3_DEPCMD_DEPENDXFER;

	/*
	 * Ask for a completion event so the conclusion of bus traffic for this
	 * transfer is observable - see UDC_DWC3_DEPCMD_CMDIOC.
	 *
	 * Only while the controller is running: the field must not be set when
	 * DCTL.RunStop is 0, which is reachable because teardown can end a transfer
	 * after the controller has been stopped. With the controller stopped no
	 * completion event would arrive anyway, so the request is dropped and the
	 * endpoint left un-armed. The command itself still runs either way.
	 *
	 * Armed before the command is posted, never after: the event can only be
	 * raised once the command is in flight, but the work queue that consumes it
	 * may run the moment this function returns.
	 */
	if ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP) != 0) {
		flags |= UDC_DWC3_DEPCMD_CMDIOC;
		ep_data->end_xfer_pending = true;
	}

	/*
	 * A failed return means the command was never issued - the previous one on
	 * this endpoint was still active when the pre-poll gave up. Clearing the
	 * flag matters more than the log line: a caller waiting for a completion
	 * event that no command will ever generate waits for ever.
	 */
	if (udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(ep_data->epn), flags) ==
	    UDC_DWC3_XFERRSCIDX_INVALID) {
		ep_data->end_xfer_pending = false;
		LOG_ERR("End Transfer not issued on EP%02x", ep_data->cfg.addr);
		return false;
	}

	LOG_DBG("DepEndXfer done EP%02x", ep_data->cfg.addr);

	/*
	 * True only when a completion event is genuinely expected. Without CmdIOC -
	 * which is refused while the controller is stopped - the command still runs
	 * but reports nothing, so a caller must not wait for it.
	 */
	return (flags & UDC_DWC3_DEPCMD_CMDIOC) != 0;
}

static void udc_dwc3_depcmd_start_config(const struct device *const dev,
					 bool is_control)
{
	uint32_t flags = 0;

	flags |= FIELD_PREP(UDC_DWC3_DEPCMD_XFERRSCIDX_MASK, is_control ? 0 : 2);
	flags |= UDC_DWC3_DEPCMD_DEPSTARTCFG;

	udc_dwc3_depcmd(dev, UDC_DWC3_DEPCMD(0), flags);

	LOG_DBG("DepStartConfig done ep=%s", is_control ? "control" : "non-control");
}

/*
 * Transfer Requests (TRB)
 *
 * DWC3 receives transfer requests from this driver through a shared memory
 * buffer, resubmitted upon every new transfer (through either Start or
 * Update command).
 */

/* Read the data back to force async bus like AXI to complete the write */
static inline void udc_dwc3_mem_commit(volatile uint32_t *const last_word)
{
	sys_read32((uintptr_t)last_word);
}

static void udc_dwc3_push_trb(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data,
			      struct net_buf *const buf, const uint32_t ctrl)
{
	volatile struct udc_dwc3_trb *const trb = &ep_data->trb_buf[ep_data->head];
	const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);
	const uint32_t out_size = !USB_EP_DIR_IS_OUT(ep_data->cfg.addr) ? buf->len
				  : (mps != 0U ? ROUND_UP(buf->size, mps) : buf->size);

	/*
	 * head, tail, full and net_buf[] are shared with udc_dwc3_pop_trb(), and
	 * neither side takes a lock.
	 *
	 * Against pop_trb specifically that is safe, because both run on the same
	 * work queue: this function is reached from udc_dwc3_ep_worker(), pop_trb
	 * from udc_dwc3_event_worker(), and a work queue runs its items one at a
	 * time. Moving either side off that queue - back into the ISR for latency,
	 * or onto a second queue - would need explicit protection here.
	 *
	 * The work queue alone is NOT enough, because other threads reach this ring
	 * too, off that queue and on the caller's thread:
	 *
	 *   udc_dwc3_ep_enable() -> udc_dwc3_ep_resume(), whose requeue loop calls
	 *     udc_dwc3_trb_bulk() and so reaches this function directly,
	 *   udc_dwc3_ep_disable(), which walks net_buf[] in reverse to drain it,
	 *   udc_dwc3_ep_dequeue().
	 *
	 * Those all hold the UDC mutex. udc_dwc3_ep_worker() now takes the same
	 * mutex for its whole body, so every toucher of head/tail/full/net_buf[]
	 * holds one common lock and the cases above are excluded. Before that it did
	 * not, and holding the mutex elsewhere excluded nothing on this path.
	 *
	 * Both properties are load-bearing: moving either side off the work queue,
	 * or dropping the mutex from the endpoint worker, reopens this.
	 */

	/* If the next TRB in the chain is still owned by the hardware, need
	 * to retry later when more resources become available.
	 */
	__ASSERT_NO_MSG(!ep_data->full);

	/* Associate an active buffer and a TRB together */
	ep_data->net_buf[ep_data->head] = buf;

	trb->addr_lo = LO32((uintptr_t)buf->data);
	trb->addr_hi = HI32((uintptr_t)buf->data);
	trb->status = out_size;
	trb->ctrl = ctrl;

	udc_dwc3_mem_commit(&trb->ctrl);

	LOG_DBG("PUSH %u, buf %p, data %p, size %u -> %u",
		ep_data->head, (void *)buf, (void *)buf->data, buf->size, out_size);

	ep_data->head = (ep_data->head + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

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

	LOG_DBG("POP %u EP%02x, buf %p, data %p",
		ep_data->tail, ep_data->cfg.addr, (void *)*buf, (void *)(*buf)->data);

	/* -1 for link trb */
	ep_data->tail = (ep_data->tail + 1) % (CONFIG_UDC_DWC3_TRB_NUM - 1);

	/* If we just pulled a TRB, we know we made one hole and we are not full anymore */
	ep_data->full = false;

	if (USB_EP_DIR_IS_OUT(ep_data->cfg.addr)) {
		/* TODO: why mps here? */
		const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);
		const uint32_t programmed =
			(mps != 0U) ? ROUND_UP((*buf)->size, mps) : (*buf)->size;
		const uint32_t residual =
			FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb->status);
		const uint32_t received =
			(programmed > residual) ? (programmed - residual) : 0U;

		(*buf)->len = MIN(received, (*buf)->size);
	}

	return 0;
}

static int udc_dwc3_trb_nonctrl_init(const struct device *const dev,
				   struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *trb = ep_data->trb_buf;
	const uint32_t i = CONFIG_UDC_DWC3_TRB_NUM - 1;
	int ret;

	LOG_DBG("Initializing normal TRB");

	/* HWO=0 on the first TRB will prevent the transfers to start until configured */
	memset((void *)trb, 0x00, sizeof(*trb) * CONFIG_UDC_DWC3_TRB_NUM);

	/* TRB LINK that loops the ring buffer back to the beginning */
	trb[i].ctrl = UDC_DWC3_TRB_CTRL_TRBCTL_LINK_TRB | UDC_DWC3_TRB_CTRL_HWO;
	trb[i].addr_lo = LO32((uintptr_t)ep_data->trb_buf);
	trb[i].addr_hi = HI32((uintptr_t)ep_data->trb_buf);

	udc_dwc3_mem_commit(&trb[i].addr_hi);

	ret = udc_dwc3_depcmd_start_xfer(dev, ep_data);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static void udc_dwc3_trb_ctrl_out(const struct device *const dev, struct net_buf *const buf,
				  const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_out[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	uint32_t size;

	priv->last_xfer_type = ctrl;
	priv->last_xfer_dir = USB_EP_DIR_OUT;

	if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		size = sizeof(struct usb_setup_packet);
	} else if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2 ||
		   ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3) {
		size = USB_MPS_EP_SIZE(ep_data->cfg.mps);
	} else {
		size = buf->size;
	}

	if (ctrl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
		const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);
		__ASSERT_NO_MSG(size == ROUND_UP(size, mps));
	}

	trb[0].addr_lo = LO32((uintptr_t)buf->data);
	trb[0].addr_hi = HI32((uintptr_t)buf->data);
	trb[0].status = size;
	trb[0].ctrl = ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO;
	udc_dwc3_mem_commit(&trb[0].ctrl);

	memcpy(&ep_data->trb_cache[0], (void *)&trb[0], sizeof(ep_data->trb_cache[0]));

	udc_dwc3_depcmd_start_xfer(dev, ep_data);
}

static void udc_dwc3_trb_ctrl_in(const struct device *const dev,
				 struct net_buf *const buf,
				 const uint32_t ctrl)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	struct udc_dwc3_ep_data *const ep_data = &cfg->ep_data_in[0];
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;

	priv->last_xfer_type = ctrl;
	priv->last_xfer_dir = USB_EP_DIR_IN;

	trb[0].addr_lo = LO32((uintptr_t)buf->data);
	trb[0].addr_hi = HI32((uintptr_t)buf->data);
	trb[0].status = buf->len;
	trb[0].ctrl = ctrl | UDC_DWC3_TRB_CTRL_LST | UDC_DWC3_TRB_CTRL_HWO;
	udc_dwc3_mem_commit(&trb[1].ctrl);

	memcpy(&ep_data->trb_cache[0], (void *)&trb[0], sizeof(ep_data->trb_cache[0]));

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
		LOG_DBG("Buffer has a ZLP flag");
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
 * There is no worker for control buffers, and instead udc_dwc3_ctrl_next()/udc_dwc3_ctrl_try()
 * is called whenever there is an opportunity to send more, and only when all conditions are met.
 *
 * DWC3 internal control buffer state machine does not support them being submitted out of order.
 * This means the driver has to wait the XferNotReady event from the host to make sure the order
 * is respected. This trusts the host for sending the requests in correct order.
 *
 * The USB stack will submit the control buffers out of order, which is supported by most USB
 * controllers (i.e. IN and OUT submitted at the same time rather than one after another).
 *
 *  The TRBs are effectively submitted when the following conditions are met:
 *
 * - There is a buffer ready for ths endopint.
 * - There is an XferNotReady event submitted.
 * - The other endpoint is not busy anymore
 */

static bool udc_dwc3_ctrl_next_in(const struct device *const dev,
				  struct net_buf *const buf)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct usb_setup_packet *const setup = &priv->setup_packet;
	const struct udc_buf_info bi = *udc_get_buf_info(buf);

	if (bi.data) {
		LOG_DBG("trb IN_DATA ln=%d d=%p", buf->len, (void *)buf->data);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi.status && setup->wLength == 0) {
		buf->size = 0;
		buf->len = 0;
		LOG_DBG("trb IN_STATUS_2 ln=%d d=%p", buf->len, (void *)buf->data);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2);
	} else if (bi.status) {
		/* Same as the two-stage case above: buf->len is what reaches the TRB. */
		buf->size = 0;
		buf->len = 0;
		LOG_DBG("trb IN_STATUS_3 ln=%d d=%p", buf->len, (void *)buf->data);
		udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
	} else {
		LOG_ERR("Unknown buffer IN type");
		udc_submit_ep_event(dev, buf, -EINVAL);
		return false;
	}

	return true;
}

static bool udc_dwc3_ctrl_next_out(const struct device *const dev,
				   struct net_buf *const buf)
{
	const struct udc_buf_info bi = *udc_get_buf_info(buf);

	if (bi.setup) {
		LOG_DBG("trb OUT_SETUP sz=%d d=%p", buf->size, (void *)buf->data);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);
	} else if (bi.data) {
		LOG_DBG("trb OUT_DATA sz=%d d=%p", buf->size, (void *)buf->data);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
	} else if (bi.status) {
		LOG_DBG("trb OUT_STATUS_3 sz=%d d=%p", buf->size, (void *)buf->data);
		udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
	} else {
		LOG_ERR("Unknown buffer OUT, size %d, data %p", buf->size, (void *)buf->data);
		udc_submit_ep_event(dev, buf, -EINVAL);
		return false;
	}

	return true;
}

/* Defined below; used by the abandon path before their definitions. */
static void udc_dwc3_ctrl_next(const struct device *const dev);
static void udc_dwc3_ctrl_try(const struct device *const dev,
			      struct udc_dwc3_ep_data *ep_data);

static bool udc_dwc3_ctrl_setup_pending(const struct device *const dev,
					struct udc_dwc3_ep_data *const ep_data)
{
	volatile struct udc_dwc3_trb *const trb = ep_data->trb_buf;
	uint32_t sts = trb[0].status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK;

	/* TODO: check if CHN is the proper way to handle ZLP control transfers. */
	if (sts == UDC_DWC3_TRB_STATUS_TRBSTS_OK &&
	    (trb[0].ctrl & UDC_DWC3_TRB_CTRL_CHN) != 0) {
		sts = trb[1].status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK;
	}

	return (sts == UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING);
}

/*
 * Return every buffer queued on this control endpoint that belongs to the
 * transfer being abandoned, stopping at a SETUP.
 *
 * A SETUP marks the start of the next transfer, so anything queued ahead of it
 * belongs to the one the host walked away from. Leaving those behind would mean
 * arming a stage of a dead transfer: a device-to-host request aborted during
 * its IN data stage still has its status buffer sitting on the OUT endpoint,
 * and that is what the next arm would pick up.
 */
static void udc_dwc3_ctrl_drain_abandoned(const struct device *const dev,
					  struct udc_dwc3_ep_data *const ep_data)
{
	struct net_buf *buf;

	while ((buf = udc_buf_peek(&ep_data->cfg)) != NULL) {
		if (udc_get_buf_info(buf)->setup) {
			break;
		}

		buf = udc_buf_get(&ep_data->cfg);
		if (buf == NULL) {
			break;
		}

		udc_submit_ep_event(dev, buf, -ECONNRESET);
	}
}

static void udc_dwc3_ctrl_abandon(const struct device *const dev,
				  struct udc_dwc3_ep_data *const ep_data)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_ep_data *const peer = (ep_data == &cfg->ep_data_in[0]) ?
					      &cfg->ep_data_out[0] : &cfg->ep_data_in[0];

	LOG_WRN_RATELIMIT("new SETUP on EP%02x, abandonging previous transaction",
			  ep_data->cfg.addr);

	udc_dwc3_ctrl_drain_abandoned(dev, ep_data);
	udc_ep_set_busy(&ep_data->cfg, false);

	memset((void *)&ep_data->trb_buf[0], 0x00, sizeof(ep_data->trb_buf[0]));
	memset((void *)&ep_data->trb_buf[1], 0x00, sizeof(ep_data->trb_buf[1]));
	memset((void *)&ep_data->trb_cache[0], 0x00, sizeof(ep_data->trb_cache));

	udc_dwc3_fifo_flush_tx(dev, cfg->ep_data_in[0].cfg.addr & 0x7fU);

	if (udc_ep_is_busy(&peer->cfg)) {
		LOG_DBG("EP%02x is busy with the replacement transfer, "
			"leaving it to its own completion", peer->cfg.addr);
		return;
	}

	udc_dwc3_ctrl_drain_abandoned(dev, peer);
	udc_ep_set_busy(&peer->cfg, false);

	udc_dwc3_ctrl_try(dev, &cfg->ep_data_out[0]);
}

static void udc_dwc3_ctrl_try(const struct device *const dev,
			      struct udc_dwc3_ep_data *ep_data)
{
	struct net_buf *buf;
	bool armed;

	/*
	 * The busy test and the claim below are not atomic on their own, and this
	 * function is reached from two places: udc_dwc3_ep_enqueue(), on the caller's
	 * thread, and udc_dwc3_handle_event(), on the event work queue.
	 *
	 * No lock is needed here regardless, because both entries already hold the
	 * UDC mutex: the stack takes it through the driver's .lock op around every
	 * API call including ep_enqueue, and handle_event takes the same mutex with
	 * udc_lock_internal() for the whole dispatch. The two are therefore mutually
	 * exclusive.
	 *
	 * Recorded because it is not obvious from this function alone: anything that
	 * calls it from a path NOT holding the UDC mutex would reintroduce a race in
	 * which both callers pass the test before either sets the flag, and both go
	 * on to program TRB 0 and issue Start Transfer.
	 */
	buf = udc_buf_peek(&ep_data->cfg);
	if (buf == NULL) {
		/*
		 * DBG, not INF: in steady state "nothing queued right now" is the normal
		 * answer, and at INF it was 36% of uart_v5_3.log - 2.4 MB, about 258 s of
		 * console time in a 638 s run. It matters when chasing buffer starvation,
		 * nowhere else.
		 */
		LOG_DBG("EP%02X: no buf", ep_data->cfg.addr);
		return;
	}

	if (ep_data->end_xfer_pending) {
		LOG_DBG("EP%02X still concluding an End Transfer, not arming yet",
			ep_data->cfg.addr);
		return;
	}

	if (udc_get_buf_info(buf)->setup) {
		if (udc_ep_is_busy(&ep_data->cfg)) {
			LOG_DBG("EP%02X: busy (SETUP)", ep_data->cfg.addr);
			return;
		}
	} else if (udc_ep_is_busy(&ep_data->cfg)) {
		LOG_DBG("EP%02X: busy", ep_data->cfg.addr);
		return;
	}

	udc_ep_set_busy(&ep_data->cfg, true);

	if (USB_EP_DIR_IS_IN(ep_data->cfg.addr)) {
		armed = udc_dwc3_ctrl_next_in(dev, buf);
	} else {
		armed = udc_dwc3_ctrl_next_out(dev, buf);
	}

	if (!armed) {
		udc_ep_set_busy(&ep_data->cfg, false);
	}
}

static void udc_dwc3_ctrl_next(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	/* TODO: control transfers might/will arrive out of order from the usb stack */
	udc_dwc3_ctrl_try(dev, &cfg->ep_data_in[0]);
	udc_dwc3_ctrl_try(dev, &cfg->ep_data_out[0]);
}

#include "../../subsys/usb/device_next/usbd_ch9.h"

static int udc_dwc3_ep_dequeue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg);
static int udc_dwc3_disable(const struct device *const dev);
static int udc_dwc3_enable(const struct device *const dev);
static int udc_dwc3_init(const struct device *const dev);
static int udc_dwc3_shutdown(const struct device *const dev);
static int udc_dwc3_ep_enable(const struct device *const dev, struct udc_ep_config *const ep_cfg);
static int udc_dwc3_ep_disable(const struct device *const dev, struct udc_ep_config *const ep_cfg);

/*
 * Events
 *
 * Process the events from the event ring buffer. Interrupts gives us a
 * hint that an event is available, which we fetch from a ring buffer shared
 * with the hardware.
 */

static void udc_dwc3_drop_xfer_state(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	for (int i = 0; i < cfg->num_in_eps; i++) {
		cfg->ep_data_in[i].end_xfer_pending = false;
		cfg->ep_data_in[i].resume_pending = false;
	}

	for (int i = 0; i < cfg->num_out_eps; i++) {
		cfg->ep_data_out[i].end_xfer_pending = false;
		cfg->ep_data_out[i].resume_pending = false;
	}
}

static void udc_dwc3_on_soft_reset(const struct device *const dev)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	k_timepoint_t end = sys_timepoint_calc(K_SECONDS(1));
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	uint32_t reg;

	/* Configure and reset the Device Controller */
	/* TODO confirm that DWC_USB3_EN_LPM_ERRATA == 1 */
	reg = UDC_DWC3_DCTL_CSFTRST;
	reg |= FIELD_PREP(UDC_DWC3_DCTL_LPM_NYET_THRES_MASK, 15);
	sys_write32(reg, base + UDC_DWC3_DCTL);

	while ((sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_CSFTRST) != 0) {
		if (sys_timepoint_expired(end)) {
			LOG_WRN("soft reset failed");
			break;
		}

		k_busy_wait(UDC_DWC3_CSFTRST_POLL_US);
	}

	/*
	 * The endpoint command registers go back to being undefined on read, and no
	 * command issued before the reset can still be active, so the next command
	 * on each endpoint must skip the pre-poll again. See priv->depcmd_issued.
	 */
	DEV_DATA(dev)->depcmd_issued = 0;

	/* Enable AXI64 bursts for various sizes expected */
	//reg = UDC_DWC3_GSBUSCFG0_INCR256BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR128BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR64BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR32BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR16BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR8BRSTENA;
	//reg |= UDC_DWC3_GSBUSCFG0_INCR4BRSTENA;
	reg = 0;
	sys_set_bits(base + UDC_DWC3_GSBUSCFG0, reg);

	/*
	 * Global Rx Threshold: disable multi-packet RX thresholding.
	 *
	 * Databook 1.2.4 erratum: an "ACK TP with NumP=0 followed by ACK TP with NumP=1
	 * without ERDY TP ... during a burst bulk OUT transfer" can leave third-party
	 * USB 3.0 hosts waiting for an ERDY.  The documented workaround is
	 * GRXTHRCFG.UsbRxPktCntSel=0 plus fixed DCFG.NUMP - NUMP is programmed below, so
	 * clearing this bit is what selects that mode.
	 *
	 * Databook 4.2.4 reaches the same condition another way: with RX thresholding on,
	 * "do not use the 'on-demand' mode of transfer for SS OUT endpoints", or a final
	 * ACK TP (NumP=0) deadlocks the host waiting for an ERDY that on-demand software
	 * never sends.  This driver arms OUT TRBs only when the stack enqueues a buffer -
	 * exactly that on-demand mode - so either this bit goes or SS OUT endpoints must
	 * keep a TRB permanently armed.  Clearing it is much the smaller change.
	 *
	 * The power-on value is logged before anything is modified.
	 */
	reg = sys_read32(base + UDC_DWC3_GRXTHRCFG);
	LOG_INF("GRXTHRCFG=0x%08x at reset (UsbRxPktCntSel=%u, UsbRxPktCnt=%u)", reg,
		(reg & UDC_DWC3_GRXTHRCFG_USBRXPKTCNTSEL) ? 1U : 0U,
		(uint32_t)FIELD_GET(UDC_DWC3_GRXTHRCFG_USBRXPKTCNT_MASK, reg));

#if UDC_DWC3_RX_THRESHOLD_WORKAROUND
	if ((reg & UDC_DWC3_GRXTHRCFG_USBRXPKTCNTSEL) != 0) {
		LOG_WRN("clearing GRXTHRCFG.UsbRxPktCntSel (databook 1.2.4 erratum)");
		sys_clear_bits(base + UDC_DWC3_GRXTHRCFG,
			       UDC_DWC3_GRXTHRCFG_USBRXPKTCNTSEL);
	}
#endif

	/* Letting GTXTHRCFG unchanged - the erratum above is RX-side only */

	/* Read the chip identification */
	reg = sys_read32(base + UDC_DWC3_GCOREID);
	LOG_INF("event: coreid=0x%04lx rel=0x%04lx",
		FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg),
		FIELD_GET(UDC_DWC3_GCOREID_REL_MASK, reg));
	__ASSERT_NO_MSG(FIELD_GET(UDC_DWC3_GCOREID_CORE_MASK, reg) == 0x5533);

	/* Letting GUID unchanged */
	/*
	 * "The PHY must not be enabled for auto-resume in device mode. Therefore,
	 * the field GUSB2PHYCFG[15] (ULPIAutoRes) must be written with '0' during
	 * the power-on initialization in case the reset value is '1'" (databook
	 * Table 4-1, Power-On or Soft Reset Register Initialization).
	 *
	 * One bit, and inert if the core already powers up with it clear - but the
	 * spec explicitly says not to rely on that, and the reset value can differ
	 * between configurations. With it set the ULPI PHY auto-resumes without the
	 * driver's involvement, so the link can leave suspend in a state the driver
	 * never learns about.
	 *
	 * Everything else in GUSB2PHYCFG (USBTrdTim, FSIntf, PHYIf, TOUTCal) and all
	 * of GUSB3PIPECTL stay at their coreConsultant power-on values, which the
	 * same table permits.
	 */
	sys_clear_bits(base + UDC_DWC3_GUSB2PHYCFG,
		       UDC_DWC3_GUSB2PHYCFG_ULPIAUTORES);
	/* Letting GRXFIFOSIZ unchanged */

	/* Setup the event buffer address, size and start event reception */
	memset((void *)cfg->evt_buf, 0, CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t));

	/*
	 * The read pointer belongs with that memset and must never be separated from it.
	 *
	 * CSftRst clears all CSRs except GSTS, GSNPSID, GGPIO, GUID, GUSB2PHYCFGn,
	 * GUSB3PIPECTLn, DCFG, DCTL, DEVTEN and DSTS.  GEVNTADR/SIZ/COUNT are absent from
	 * that list, which is why the lines below reprogram them - and it means the
	 * controller's write pointer restarts at slot 0, so anything the driver believed
	 * about its read position is now wrong.
	 *
	 * Leaving it stale does not lose an event, it HANGS the ring: the drain parks on
	 * evt_next, which the controller will not reach again until it has written every
	 * slot ahead of it, while GEVNTCOUNT reports events piling up at slot 0 that
	 * nothing collects - the empty-slot give-up, arrived at by way of a bug.
	 *
	 * udc_dwc3_init() is a UDC API entry point, not boot-only, so this is reachable on
	 * any re-init.  The give-up bookkeeping goes with it: every field names a slot index
	 * or an age that no longer refers to anything.
	 */
	priv->evt_next = 0;

	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_EVENTS_NUM; i++) {
		cfg->evt_buf[i] = UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
	}
	udc_dwc3_mem_commit(&cfg->evt_buf[CONFIG_UDC_DWC3_EVENTS_NUM - 1]);

	sys_write32(HI32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_HI(0));
	sys_write32(LO32((uintptr_t)cfg->evt_buf), base + UDC_DWC3_GEVNTADR_LO(0));
	sys_write32(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t), base + UDC_DWC3_GEVNTSIZ(0));
	LOG_INF("Event buffer size is %u bytes", sys_read32(base + UDC_DWC3_GEVNTSIZ(0)));

	/* Last step: writing 0 here is what enables the event buffer. */
	sys_write32(0, base + UDC_DWC3_GEVNTCOUNT(0));

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
	reg |= FIELD_PREP(UDC_DWC3_DCFG_NUMP_MASK, 1);
	sys_write32(reg, base + UDC_DWC3_DCFG);

	/*
	 * Enable reception of all USB events except ULSTCNGEN, which is a
	 * diagnostic-only class - see below.
	 */
	reg = UDC_DWC3_DEVTEN_INACTTIMEOUTRCVEDEN;
	reg |= UDC_DWC3_DEVTEN_EVNTOVERFLOWEN;
	reg |= UDC_DWC3_DEVTEN_CMDCMPLTEN;
	reg |= UDC_DWC3_DEVTEN_ERRTICERREN;
	reg |= UDC_DWC3_DEVTEN_HIBERNATIONREQEVTEN;
	reg |= UDC_DWC3_DEVTEN_WKUPEVTEN;
	reg |= UDC_DWC3_DEVTEN_ULSTCNGEN;
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

	udc_dwc3_drop_xfer_state(dev);

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
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0], true);
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_out[0], true);

	/*
	 * GTXFIFOSIZn is deliberately left alone, and the databook is explicit that
	 * this is the normal case: the defaults are "assigned in coreConsultant
	 * based on the maximum packet size, number of packets to be buffered, speed
	 * of host bus instance, bus latency, and mode of operation", and "upon reset
	 * and mode transitions, hardware automatically programs these registers to
	 * the default values. Consequently, there is typically no need for the
	 * software to modify the pre-defined default values."
	 *
	 * Table 4-1 lists GTXFIFOSIZn conditionally - "unless the packet sizes" of
	 * this configuration differ from what the defaults assume - not as an
	 * unconditional power-on write. Recomputing an allocation here would need
	 * the RAM depth the RTL was built with, which is not something this driver
	 * can discover, and getting it wrong breaks transmit outright.
	 *
	 * GTXFIFOPRIDEV is left alone for the same kind of reason. Its bits select
	 * high (1) or low (0) DMA priority per IN-endpoint TxFIFO, and low-priority
	 * FIFOs are served round-robin only once the high-priority ones have
	 * nothing left to do. The reset value is all zero, which is every endpoint
	 * at equal priority in round-robin - the fair arrangement. Writing it would
	 * create starvation between the video and ACM endpoints rather than remove
	 * it, so it stays unwritten unless a capture argues otherwise.
	 */

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
	udc_dwc3_depcmd_ep_config(dev, &cfg->ep_data_in[0], true);

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
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t trb_trbctl = ep_data->trb_cache[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK;
	struct net_buf *buf;

	if (udc_dwc3_ctrl_setup_pending(dev, ep_data)) {
		udc_dwc3_ctrl_abandon(dev, ep_data);
		return;
	}

	buf = udc_buf_get(&ep_data->cfg);
	if (buf == NULL) {
		LOG_ERR("Missing buffer submitted for EP%02X", ep_data->cfg.addr);
		udc_ep_set_busy(&ep_data->cfg, false);
		udc_dwc3_ctrl_next(dev);
		return;
	}

	LOG_DBG("%u:%u:%u",
		udc_get_buf_info(buf)->setup,
		udc_get_buf_info(buf)->data,
		udc_get_buf_info(buf)->status);

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

	memset(&ep_data->trb_buf[0], 0x00, sizeof(ep_data->trb_buf[0]));
	memset(&ep_data->trb_cache[0], 0x00, sizeof(ep_data->trb_cache[0]));

	udc_submit_ep_event(dev, buf, 0);

	/* Used when receiving a completed buffer from the hardware: mark as free */
	udc_ep_set_busy(&ep_data->cfg, false);

	udc_dwc3_ctrl_next(dev);
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
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const uint32_t trb_trbctl = ep_data->trb_cache[0].ctrl & UDC_DWC3_TRB_CTRL_TRBCTL_MASK;
	struct net_buf *buf;

	if (udc_dwc3_ctrl_setup_pending(dev, ep_data)) {
		udc_dwc3_ctrl_abandon(dev, ep_data);
		return;
	}

	if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP) {
		buf = udc_buf_peek(&ep_data->cfg);
		if (buf == NULL) {
			LOG_ERR("Missing buffer for EP%02X", ep_data->cfg.addr);
			udc_ep_set_busy(&ep_data->cfg, false);
			udc_dwc3_ctrl_next(dev);
			return;
		}

		/* Update the size to the setup packet size */
		if (buf->size < sizeof(priv->setup_packet)) {
			LOG_ERR("Invalid size for setup packet buffer: %u", buf->size);
			udc_submit_ep_event(dev, buf, -ENOBUFS);
			udc_ep_set_busy(&ep_data->cfg, false);
			udc_dwc3_ctrl_next(dev);
			return;
		}

		memcpy(&priv->setup_packet, buf->data, sizeof(priv->setup_packet));

		buf->len = 0;

		/* Latency optimization: set the address immediately to be able to be able
		 * to ACK/NAK the first packets from the host with the new address,
		 * otherwise the host issue a reset.
		 */
		if (priv->setup_packet.bmRequestType == USB_REQTYPE_TYPE_STANDARD &&
		    priv->setup_packet.bRequest == USB_SREQ_SET_ADDRESS) {
			udc_dwc3_set_address(dev, sys_le16_to_cpu(priv->setup_packet.wValue));
		}

		/*
		 * The one line a healthy control transfer prints, and the only context
		 * an error line needs: the eight setup bytes identify the request that
		 * was in flight. Everything the stages used to narrate is derivable
		 * from it, at a tenth of the console cost.
		 */
		udc_setup_received(dev, &priv->setup_packet);

		/*
		 * udc_setup_received() has just invalidated whatever the IN endpoint
		 * was doing - it drains that queue and releases the endpoint - but the
		 * controller has already retired that TRB and its completion is still
		 * queued behind this event. Mark it so the handler discards it, and
		 * clear the writeback so nothing else reads the abort out of a ring
		 * that is about to be re-armed for the replacement transfer.
		 *
		 * Without this the late completion is matched against the newly armed
		 * TRB and the replacement transfer's buffer is reported complete before
		 * the host has seen it.
		 */
		if (udc_dwc3_ctrl_setup_pending(dev, &cfg->ep_data_in[0])) {
			memset((void *)&cfg->ep_data_in[0].trb_buf[0], 0x00,
			       sizeof(cfg->ep_data_in[0].trb_buf[0]));
			memset((void *)&cfg->ep_data_in[0].trb_buf[1], 0x00,
			       sizeof(cfg->ep_data_in[0].trb_buf[1]));
			/*
			 * ep_data_in[0], not ep_data.  This runs in
			 * udc_dwc3_on_ctrl_out(), so ep_data is the OUT endpoint;
			 * clearing its cache here left the IN cache stale - the one
			 * whose TRBs the three memsets above just discarded - and
			 * wrongly discarded the OUT endpoint's.
			 */
			memset(&cfg->ep_data_in[0].trb_cache[0], 0x00,
			       sizeof(cfg->ep_data_in[0].trb_cache[0]));
		}
	} else {
		buf = udc_buf_get(&ep_data->cfg);
		if (buf == NULL) {
			LOG_ERR("Missing buffer for EP%02X", ep_data->cfg.addr);
			udc_ep_set_busy(&ep_data->cfg, false);
			udc_dwc3_ctrl_next(dev);
			udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
			return;
		}

		/*
		 * What the hardware actually received, not what the host declared.
		 *
		 * The old code assigned setup_packet.wLength - the DECLARED length -
		 * under a comment claiming it was what the hardware reported.  A host is
		 * free to send a short data stage, and every such transfer was handed
		 * upstream as if the full wLength had arrived, with stale bytes beyond
		 * the real data counted as valid.
		 *
		 * The controller decrements BUFSIZ in the live TRB as it fills the
		 * buffer, so received = programmed - residual, using the same rounding
		 * udc_dwc3_trb_ctrl_out() applied when it armed the descriptor.  The
		 * live TRB is still intact here - it is not cleared until the end of
		 * this function.  Clamped to both the buffer and the declared length, so
		 * a host that overruns either cannot inflate buf->len.
		 */
		if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
			const uint32_t mps = USB_MPS_EP_SIZE(ep_data->cfg.mps);
			const uint32_t programmed =
				(mps != 0U) ? ROUND_UP(buf->size, mps) : buf->size;
			const uint32_t residual = FIELD_GET(
				UDC_DWC3_TRB_STATUS_BUFSIZ_MASK,
				ep_data->trb_buf[0].status);
			const uint32_t received =
				(programmed > residual) ? (programmed - residual) : 0U;

			buf->len = MIN(received,
				       MIN((uint32_t)buf->size,
					   (uint32_t)priv->setup_packet.wLength));
		} else {
			buf->len = priv->setup_packet.wLength;
		}

		if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA) {
			LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL DATA received");
		} else if (trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3 ||
			   trb_trbctl == UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2) {
			buf->len = 0;
			LOG_HEXDUMP_DBG(buf->data, buf->len, "CTRL STATUS received");
		} else {
			LOG_ERR("Unexpected OUT packet type: 0x%x", trb_trbctl);
		}

		udc_submit_ep_event(dev, buf, 0);
	}

	memset(&ep_data->trb_buf[0], 0x00, sizeof(ep_data->trb_buf[0]));
	memset(&ep_data->trb_buf[1], 0x00, sizeof(ep_data->trb_buf[1]));
	memset(&ep_data->trb_cache[0], 0x00, sizeof(ep_data->trb_cache[0]));

	/* Used when receiving a completed buffer from the hardware: mark as free */
	udc_ep_set_busy(&ep_data->cfg, false);

	udc_dwc3_ctrl_next(dev);
}

static bool udc_dwc3_dgcmd_wait_idle(const mm_reg_t base)
{
	uint32_t polls = 0;

	while ((sys_read32(base + UDC_DWC3_DGCMD) & UDC_DWC3_DGCMD_ACT) != 0U) {
		if (++polls >= UDC_DWC3_DGCMD_POLL_MAX) {
			return false;
		}
		k_busy_wait(1);
	}

	return true;
}

static void udc_dwc3_fifo_flush_tx(const struct device *const dev, const uint8_t fifo)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	if (!udc_dwc3_dgcmd_wait_idle(base)) {
		LOG_ERR_RATELIMIT("a generic command stayed active for %u us; TxFIFO "
				  "%u not flushed", UDC_DWC3_DGCMD_POLL_MAX, fifo);
		return;
	}

	sys_write32(UDC_DWC3_DGCMD_FIFOFLUSH_TX |
		    FIELD_PREP(UDC_DWC3_DGCMD_FIFOFLUSH_NUM_MASK, fifo),
		    base + UDC_DWC3_DGCMDPAR);
	sys_write32(UDC_DWC3_DGCMD_FIFOFLUSHONE | UDC_DWC3_DGCMD_ACT,
		    base + UDC_DWC3_DGCMD);

	/*
	 * Wait for it. The next control stage is armed immediately after this
	 * returns, and arming it while the flush is still in flight would race the
	 * very bytes being discarded.
	 */
	if (!udc_dwc3_dgcmd_wait_idle(base)) {
		LOG_ERR_RATELIMIT("TxFIFO %u flush did not complete in %u us", fifo,
				  UDC_DWC3_DGCMD_POLL_MAX);
	}
}

static void udc_dwc3_on_xfer_not_ready_in(const struct device *const dev, const uint32_t evt)
{
	udc_dwc3_ctrl_next(dev);
}

static void udc_dwc3_on_xfer_not_ready_out(const struct device *const dev, const uint32_t evt)
{
	udc_dwc3_ctrl_next(dev);
}

/*
 * Decode the completion status of a retired TRB.
 *
 * Takes the snapshot that udc_dwc3_pop_trb() already produced, rather than
 * re-reading the ring. Re-reading would be wrong twice over: pop_trb has
 * advanced "tail" by the time this runs, so trb_buf[tail] is the NEXT slot and
 * not the one that just completed; and that slot may still be owned by the
 * controller, so its contents are whatever was last written there.
 */
static void udc_dwc3_on_xfer_done(const struct device *const dev,
				  const struct udc_dwc3_trb *const trb)
{
	switch (trb->status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK) {
	case UDC_DWC3_TRB_STATUS_TRBSTS_OK:
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_MISSEDISOC:
		LOG_ERR_RATELIMIT("TRBSTS MISSEDISOC on a non-control endpoint");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_SETUPPENDING:
		LOG_ERR_RATELIMIT("TRBSTS SETUPPENDING on a non-control endpoint");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_XFERINPROGRESS:
		LOG_ERR_RATELIMIT("TRBSTS XFERINPROGRESS on a non-control endpoint");
		break;
	case UDC_DWC3_TRB_STATUS_TRBSTS_ZLPPENDING:
		LOG_ERR_RATELIMIT("TRBSTS ZLPPENDING on a non-control endpoint");
		break;
	default:
		LOG_ERR_RATELIMIT("Invalid TRB type: 0x%08lx",
				  (trb->status & UDC_DWC3_TRB_STATUS_TRBSTS_MASK));
		break;
	}
}

/*
 * Retained for comparison against the unmodified driver, and deliberately not
 * compiled.
 *
 * This used to handle XferComplete on a non-control endpoint. That routing was
 * wrong: the event means TRBs were retired, not that anything failed, so it is
 * now handled with XferInProgress by udc_dwc3_on_xfer_done_nonctrl() - see the
 * note on Table 4-8 at that dispatch. Reporting -ECANCELED and tearing the
 * endpoint down on one unexpected event is destructive where draining is not.
 *
 * Kept rather than removed so the difference against the original file stays
 * easy to read. Nothing calls it; if a genuine per-endpoint error path is ever
 * needed, this is the shape it had.
 */
#if 0
static void udc_dwc3_on_xfer_error_nonctrl(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *const ep_data = _EP_DATA_FROM_EPN(cfg, epn);
	struct udc_dwc3_trb trb;
	struct net_buf *buf;
	int ret;

	LOG_ERR("Transfer error on endpoint 0x%02x", ep_data->cfg.addr);

	ret = udc_dwc3_pop_trb(dev, ep_data, &buf, &trb);
	if (ret != 0) {
		LOG_ERR("Failed to pop a TRB");
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

	ret = udc_dwc3_ep_resume(dev, ep_data, ep_data->cfg.stat.enabled);
	if (ret != 0) {
		LOG_ERR("Failed to resume endpoint 0x%02x", ep_data->cfg.addr);
		return;
	}
}
#endif

static void udc_dwc3_on_xfer_done_nonctrl(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *ep_data;
	struct net_buf *buf;
	int ret;

	if (!_EPN_IS_VALID(cfg, epn)) {
		LOG_ERR_RATELIMIT("event 0x%08x names physical endpoint %d, which "
				  "this controller does not have (%u IN, %u OUT) - "
				  "discarded",
				  evt, epn, cfg->num_in_eps, cfg->num_out_eps);
		return;
	}

	ep_data = _EP_DATA_FROM_EPN(cfg, epn);

	while (true) {
		struct udc_dwc3_trb trb;

		ret = udc_dwc3_pop_trb(dev, ep_data, &buf, &trb);
		if (ret == -ENOBUFS || ret == -EBUSY) {
			break;
		}
		if (ret != 0) {
			LOG_ERR("Failed to pop TRB from non-control endpoint");
			udc_submit_event(dev, UDC_EVT_ERROR, ret);
			break;
		}

		LOG_DBG("XFER_DONE_NORM: EP%02x, data %p",
			ep_data->cfg.addr, (void *)buf->data);

		udc_dwc3_on_xfer_done(dev, &trb);

		udc_ep_set_busy(&ep_data->cfg, false);

		ret = udc_submit_ep_event(dev, buf, 0);
		if (ret != 0) {
			LOG_ERR("Failed to submit buffer %p: %d", buf, ret);
		}

		/* We just made some room for a new buffer, check if something more to enqueue */
		k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
	}
}

static const char *udc_dwc3_get_link_state_name(const uint32_t field)
{
	/* TODO support USB2 */
	switch (UDC_DWC3_DSTS_CONNECTSPD_SS) {
	case UDC_DWC3_DSTS_CONNECTSPD_SS:
		switch (field) {
		case UDC_DWC3_LINK_STATE_USB3_U0:
			return "LINK_STATE_USB3_U0";
		case UDC_DWC3_LINK_STATE_USB3_U1:
			return "LINK_STATE_USB3_U1";
		case UDC_DWC3_LINK_STATE_USB3_U2:
			return "LINK_STATE_USB3_U2";
		case UDC_DWC3_LINK_STATE_USB3_U3:
			return "LINK_STATE_USB3_U3";
		case UDC_DWC3_LINK_STATE_USB3_SS_DIS:
			return "LINK_STATE_USB3_SS_DIS";
		case UDC_DWC3_LINK_STATE_USB3_RX_DET:
			return "LINK_STATE_USB3_RX_DET";
		case UDC_DWC3_LINK_STATE_USB3_SS_INACT:
			return "LINK_STATE_USB3_SS_INACT";
		case UDC_DWC3_LINK_STATE_USB3_POLL:
			return "LINK_STATE_USB3_POLL";
		case UDC_DWC3_LINK_STATE_USB3_RECOV:
			return "LINK_STATE_USB3_RECOV";
		case UDC_DWC3_LINK_STATE_USB3_HRESET:
			return "LINK_STATE_USB3_HRESET";
		case UDC_DWC3_LINK_STATE_USB3_CMPLY:
			return "LINK_STATE_USB3_CMPLY";
		case UDC_DWC3_LINK_STATE_USB3_LPBK:
			return "LINK_STATE_USB3_LPBK";
		case UDC_DWC3_LINK_STATE_USB3_RESET_RESUME:
			return "LINK_STATE_USB3_RESET_RESUME";
		default:
			return "unknown USB3 link state event";
		}
		break;
	case UDC_DWC3_DSTS_CONNECTSPD_HS:
	case UDC_DWC3_DSTS_CONNECTSPD_FS:
		switch (field) {
		case UDC_DWC3_LINK_STATE_USB2_ON_STATE:
			return "LINK_STATE_USB2_ON_STATE";
		case UDC_DWC3_LINK_STATE_USB2_SLEEP_STATE:
			return "LINK_STATE_USB2_SLEEP_STATE";
		case UDC_DWC3_LINK_STATE_USB2_SUSPEND_STATE:
			return "LINK_STATE_USB2_SUSPEND_STATE";
		case UDC_DWC3_LINK_STATE_USB2_DISCONNECTED:
			return "LINK_STATE_USB2_DISCONNECTED";
		case UDC_DWC3_LINK_STATE_USB2_EARLY_SUSPEND:
			return "LINK_STATE_USB2_EARLY_SUSPEND";
		case UDC_DWC3_LINK_STATE_USB2_RESET:
			return "LINK_STATE_USB2_RESET";
		case UDC_DWC3_LINK_STATE_USB2_RESUME:
			return "LINK_STATE_USB2_RESUME";
		default:
			return "unknown USB2 link state event";
		}
		break;
	default:
		return "DSTS_USBLNKST (unknown)";
	}
}

#define _NORMAL_EP(n, fn) fn(n + 2)

static const char *udc_dwc3_get_event_name(const uint32_t evt)
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
	case UDC_DWC3_DEPEVT_EPCMDCMPLT(0):
		return "DEPEVT_EPCMDCMPLT(0)";
	case UDC_DWC3_DEPEVT_EPCMDCMPLT(1):
		return "DEPEVT_EPCMDCMPLT(1)";
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_EPCMDCMPLT):
		return "DEPEVT_EPCMDCMPLT(n)";
	case UDC_DWC3_DEVT_DISCONNEVT:
		return "DEVT_DISCONNEVT";
	case UDC_DWC3_DEVT_USBRST:
		return "DEVT_USBRST";
	case UDC_DWC3_DEVT_CONNECTDONE:
		return "DEVT_CONNECTDONE";
	case UDC_DWC3_DEVT_ULSTCHNG:
		return udc_dwc3_get_link_state_name(
			FIELD_GET(UDC_DWC3_DEVT_EVTINFO_LINKSTATE_MASK, evt));
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
	case UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE:
		return "free";
	default:
		return "unknown event";
	}
}

static void udc_dwc3_on_ep_cmd_cmplt(const struct device *const dev, const uint32_t evt)
{
	const struct udc_dwc3_config *const cfg = dev->config;
	const int epn = FIELD_GET(UDC_DWC3_DEPEVT_EPN_MASK, evt);
	struct udc_dwc3_ep_data *ep_data;
	bool rearmed = false;

	if (!_EPN_IS_VALID(cfg, epn)) {
		LOG_ERR_RATELIMIT("event 0x%08x names physical endpoint %d, which "
				  "this controller does not have (%u IN, %u OUT) - "
				  "discarded",
				  evt, epn, cfg->num_in_eps, cfg->num_out_eps);
		return;
	}

	ep_data = _EP_DATA_FROM_EPN(cfg, epn);

	if (!ep_data->end_xfer_pending) {
		LOG_WRN("EpCmdCmplt on EP%02x with no End Transfer outstanding",
			ep_data->cfg.addr);
	}

	ep_data->end_xfer_pending = false;

	LOG_DBG("EpCmdCmplt: DMA stopped for EP%02x", ep_data->cfg.addr);

	/*
	 * Second half of a non-control resume that udc_dwc3_ep_resume() postponed
	 * because this End Transfer was still concluding. Now that the controller
	 * has reported it complete, the Start Transfer inside is legal.
	 *
	 * Cleared before the call, not after: the resume issues commands of its own
	 * and must be able to defer again on a fresh End Transfer rather than find
	 * its own stale flag still standing.
	 */
	if (ep_data->resume_pending) {
		int ret;

		ep_data->resume_pending = false;

		LOG_DBG("running deferred resume for EP%02x", ep_data->cfg.addr);

		ret = udc_dwc3_ep_resume(dev, ep_data, ep_data->resume_modify);
		if (ret != 0) {
			LOG_ERR("deferred resume failed on EP%02x: %d",
				ep_data->cfg.addr, ret);
			udc_submit_event(dev, UDC_EVT_ERROR, ret);
		}
	} else if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 && ep_data->cfg.stat.enabled) {
		/*
		 * No resume was postponed, but udc_dwc3_ep_worker() may have stopped
		 * against end_xfer_pending while this End Transfer was concluding, and
		 * nothing else would wake it - udc_dwc3_ep_enqueue() only submits the
		 * work when a new buffer arrives. This is reachable whenever an
		 * endpoint is ended without being resumed, which is what
		 * udc_dwc3_on_set_config_or_interface() does to every busy non-control
		 * endpoint other than the one being enabled.
		 *
		 * The resume path needs no equivalent: it ends with the same submit.
		 */
		k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
	} else if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0 && !rearmed) {
		udc_dwc3_ctrl_next(dev);
	}
}

static void udc_dwc3_handle_event(const struct device *const dev, const uint32_t evt)
{
	udc_lock_internal(dev, K_FOREVER);

	switch (evt & UDC_DWC3_EVT_MASK) {
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(0):
		udc_dwc3_on_ctrl_out(dev);
		break;
	case UDC_DWC3_DEPEVT_XFERCOMPLETE(1):
		udc_dwc3_on_ctrl_in(dev);
		break;
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERCOMPLETE):
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERINPROGRESS):
		udc_dwc3_on_xfer_done_nonctrl(dev, evt);
		break;
	case UDC_DWC3_DEPEVT_XFERNOTREADY(0):
		udc_dwc3_on_xfer_not_ready_out(dev, evt);
		break;
	case UDC_DWC3_DEPEVT_XFERNOTREADY(1):
		udc_dwc3_on_xfer_not_ready_in(dev, evt);
		break;
	case UDC_DWC3_DEPEVT_EPCMDCMPLT(0):
	case UDC_DWC3_DEPEVT_EPCMDCMPLT(1):
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_EPCMDCMPLT):
		udc_dwc3_on_ep_cmd_cmplt(dev, evt);
		break;
	case UDC_DWC3_DEVT_USBRST:
		udc_dwc3_on_usb_reset(dev);
		break;
	case UDC_DWC3_DEVT_CONNECTDONE:
		udc_dwc3_on_connect_done(dev);
		break;
	case UDC_DWC3_DEVT_DISCONNEVT:
		udc_dwc3_drop_xfer_state(dev);
		break;
	case LISTIFY(30, _NORMAL_EP, (: case), UDC_DWC3_DEPEVT_XFERNOTREADY):
	case UDC_DWC3_DEVT_ULSTCHNG:
	case UDC_DWC3_DEVT_WKUPEVT:
	case UDC_DWC3_DEVT_SUSPEND:
	case UDC_DWC3_DEVT_SOF:
	case UDC_DWC3_DEVT_CMDCMPLT:
	case UDC_DWC3_DEVT_VNDRDEVTSTRCVED:
		break;
	case UDC_DWC3_DEVT_ERRTICERR:
		LOG_ERR_RATELIMIT("PHY erratic error");
		udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
		break;
	case UDC_DWC3_DEVT_EVNTOVERFLOW:
		LOG_ERR_RATELIMIT("Event ring overflow");
		udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
		break;
	default:
		LOG_ERR_RATELIMIT("unknown event: 0x%x", evt);
		break;
	}

	udc_unlock_internal(dev);
}

/*
 * Upper bound on the wait for a posted event write to land in the buffer.
 *
 * The core's AXI write is posted: GEVNTCOUNT can become visible while the slot
 * it refers to is still empty.  Reading unconditionally would consume whatever
 * the previous occupant left - a well-formed event from one ring-wrap ago,
 * indistinguishable downstream from a real one.  Hence the sentinel every slot
 * is re-armed with, and the buffer initialised to, at setup.
 *
 * Three properties matter, all learned the hard way:
 *
 *   - It is a TIME budget, not a read count.  A fixed 64-read spin expired on
 *     most events (53558 expiries against 33319 events handled), abandoning the
 *     drain each time until the ring overflowed.  A time also keeps its meaning
 *     across core clocks.
 *   - Reads are PACED.  Unthrottled polling issues back-to-back Wishbone reads
 *     against the same RAM the controller is writing over AXI, holding off the
 *     very event being waited for.
 *   - Expiry is COUNTED, not logged per occurrence.  At 115200 baud tens of
 *     thousands of error lines would themselves starve the ring; "dwc3 evt"
 *     reports the total.
 */
/*
 * Constraints the event buffer has to satisfy, checked here rather than
 * discovered on a rig: the size must be a multiple of four and at least 32
 * bytes, and it has to be a power of two for the size-alignment above to be
 * expressible.
 */
BUILD_ASSERT(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) >= 32,
	     "DWC3 event buffer must be at least 32 bytes");

BUILD_ASSERT(CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) <= 64,
	     "DWC3 event buffer is capped at 64 bytes on this core, and the "
	     "buffer is aligned to 64 so the size-alignment rule holds");

static void udc_dwc3_event_worker(struct k_work *work)
{
	struct udc_dwc3_data *const priv = CONTAINER_OF(work, struct udc_dwc3_data, event_work);
	const struct device *const dev = priv->dev;
	const struct udc_dwc3_config *const cfg = dev->config;
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	while ((sys_read32(base + UDC_DWC3_GEVNTCOUNT(0)) & UDC_DWC3_GEVNTCOUNT_MASK) > 0) {
		uint32_t evt = cfg->evt_buf[priv->evt_next];

		cfg->evt_buf[priv->evt_next] = UDC_DWC3_EVT_CONSUMED_ENTRY_VALUE;
		priv->evt_next = (priv->evt_next + 1) % CONFIG_UDC_DWC3_EVENTS_NUM;
		sys_write32(sizeof(uint32_t), base + UDC_DWC3_GEVNTCOUNT(0));

		udc_dwc3_handle_event(dev, evt);
	}

	cfg->irq_enable_func();
}

static void udc_dwc3_irq_handler(void *const ptr)
{
	const struct device *const dev = ptr;
	struct udc_dwc3_data *const priv = udc_get_private(dev);
	const struct udc_dwc3_config *const cfg = dev->config;

	k_work_submit_to_queue(udc_get_work_q(), &priv->event_work);

	/* Disable further interrupts until all events are processed */
	cfg->irq_disable_func();
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
	const struct udc_buf_info bi = *udc_get_buf_info(buf);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);

	LOG_DBG("enq %p d=%p sz=%u ln=%u EP%02x %u:%u:%u",
		buf, buf->data, buf->size, buf->len, ep_cfg->addr, bi.setup, bi.data, bi.status);

	if (ep_data->cfg.addr == USB_CONTROL_EP_OUT) {
		memset(buf->data, 0x00, buf->size);
	}

	udc_buf_put(ep_cfg, buf);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		udc_dwc3_ctrl_next(dev);
	} else {
		/* Process this buffer along with other waiting */
		if (sys_read32(base + UDC_DWC3_DCTL) & UDC_DWC3_DCTL_RUNSTOP &&
		    ep_cfg->stat.enabled) {
			LOG_DBG("submitting to EP%02x", ep_cfg->addr);
			k_work_submit_to_queue(udc_get_work_q(), &ep_data->work);
		}
	}

	return 0;
}

static int udc_dwc3_ep_dequeue(const struct device *const dev,
			       struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data =
		CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	if (USB_EP_GET_IDX(ep_cfg->addr) != 0U && udc_ep_is_busy(ep_cfg) &&
	    udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
		LOG_INF("EP%02x dequeued with a transfer still active; recovering "
			"through End Transfer before the buffers are released",
			ep_cfg->addr);
	}

	udc_ep_cancel_queued(dev, ep_cfg);
	udc_ep_set_busy(ep_cfg, false);

	return 0;
}

static int udc_dwc3_ep_resume(const struct device *const dev,
			      struct udc_dwc3_ep_data *const ep_data,
			      const bool modify)
{
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	struct net_buf *buf;
	int ret;

	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0 && ep_data->end_xfer_pending) {
		LOG_DBG("End Transfer still concluding on EP%02x, deferring resume",
			ep_data->cfg.addr);
		ep_data->resume_modify = modify;
		ep_data->resume_pending = true;
		return 0;
	}

	/* Reset all ongoing transfers on non-control OUT endpoints */
	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		udc_dwc3_depcmd_clear_stall(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	}

	udc_dwc3_depcmd_ep_config(dev, ep_data, modify);
	udc_dwc3_depcmd_ep_xfer_config(dev, ep_data);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) > 0) {
		ret = udc_dwc3_trb_nonctrl_init(dev, ep_data);
		if (ret != 0) {
			return ret;
		}
	}

	/* Starting from here, the endpoint can be used */
	sys_set_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

	/* Re-enqueue all the previously dequeued buffers */
	while (true) {
		buf = k_fifo_get(&ep_data->requeue_fifo, K_NO_WAIT);
		if (buf == NULL) {
			break;
		}

		LOG_DBG("Requeueing buffer %p %d:%d:%d",
			buf,
			udc_get_buf_info(buf)->setup,
			udc_get_buf_info(buf)->data,
			udc_get_buf_info(buf)->status);

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

	LOG_DBG("EP%02x, first EP%02x", ep_data->cfg.addr, priv->first_ep);

	if (USB_EP_GET_IDX(ep_cfg->addr) > 0) {
		if (priv->first_ep == 0) {
			priv->first_ep = ep_cfg->addr;
		}
		if (ep_cfg->addr == priv->first_ep) {
			udc_dwc3_on_set_config_or_interface(dev);
		}
	}

	return udc_dwc3_ep_resume(dev, ep_data, ep_data->cfg.stat.enabled);
}

static int udc_dwc3_ep_disable(const struct device *const dev, struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);
	const mm_reg_t base = DEVICE_MMIO_NAMED_GET(dev, base);
	const int slots = CONFIG_UDC_DWC3_TRB_NUM - 1;
	struct net_buf *buf;

	LOG_DBG("Disabling EP%02x", ep_cfg->addr);

	/*
	 * Drop any postponed resume as well. The End Transfer below sets
	 * end_xfer_pending again, and its completion would otherwise run a resume
	 * left over from before this teardown - re-arming an endpoint that has just
	 * been disabled. Callers that disable and then resume set the flag again on
	 * their own way through udc_dwc3_ep_resume().
	 */
	ep_data->resume_pending = false;

	/* Disable the endpoint */
	sys_clear_bits(base + UDC_DWC3_DALEPENA, UDC_DWC3_DALEPENA_USBACTEP(ep_data->epn));

	udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);

	udc_ep_set_busy(ep_cfg, false);

	/*
	 * Oldest first.  The ring holds the oldest queued buffer at tail, and
	 * requeue_fifo is a FIFO, so walking tail forward preserves submission
	 * order.  The previous walk went backwards from head - newest first - which
	 * combined with FIFO semantics handed the buffers back REVERSED whenever two
	 * or more were outstanding.  (Backwards would have been right for a LIFO,
	 * which is what the old comment assumed.)
	 */
	for (int n = 0; n < slots; n++) {
		const int idx = (ep_data->tail + n) % slots;

		buf = ep_data->net_buf[idx];
		if (buf != NULL) {
			LOG_DBG("Popping buffer %p %d:%d:%d",
				buf,
				udc_get_buf_info(buf)->setup,
				udc_get_buf_info(buf)->data,
				udc_get_buf_info(buf)->status);

			k_fifo_put(&ep_data->requeue_fifo, buf);
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
	struct udc_dwc3_data *const priv = udc_get_private(dev);
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

	/* So that the next type is SETUP */
	priv->last_xfer_type = UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_2;

	return 0;
}

static int udc_dwc3_ep_clear_halt(const struct device *const dev,
				  struct udc_ep_config *const ep_cfg)
{
	struct udc_dwc3_ep_data *const ep_data = CONTAINER_OF(ep_cfg, struct udc_dwc3_ep_data, cfg);

	LOG_INF("Clearing stall for EP%02x", ep_cfg->addr);

	if (USB_EP_GET_IDX(ep_data->cfg.addr) == 0) {
		return 0;
	}

	/*
	 * SPEC, Programming Guide 3.30b section 4.2.7 "Handling ENDPOINT_HALT":
	 *
	 *   "On ClearFeature (ENDPOINT_HALT), software must first remove all pending
	 *    transfers for the endpoint through the End Transfer command. It may then
	 *    issue a Clear Stall command on the endpoint followed by Start Transfer to
	 *    start transfers again."
	 *
	 */
	if (udc_ep_is_busy(ep_cfg) &&
	    udc_dwc3_depcmd_end_xfer(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM)) {
		LOG_INF("EP%02x halted with a transfer still pending; recovering "
			"through End Transfer before Clear Stall", ep_cfg->addr);
	}

	udc_dwc3_depcmd_clear_stall(dev, ep_data, UDC_DWC3_DEPCMD_HIPRI_FORCERM);
	ep_data->cfg.stat.halted = false;

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
	const struct udc_dwc3_config *const cfg = dev->config;

	LOG_DBG("Disabling DWC3 driver");

	sys_clear_bits(base + UDC_DWC3_DCTL, UDC_DWC3_DCTL_RUNSTOP);

	/*
	 * With RunStop cleared the controller raises no further Endpoint Command
	 * Complete events, so anything outstanding is stranded. Dropping it here
	 * means a disable/enable cycle starts from a clean state rather than from
	 * flags describing transfers that no longer exist.
	 */
	udc_dwc3_drop_xfer_state(dev);

	cfg->irq_disable_func();

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

	//reg = sys_read32(base + UDC_DWC3_GCTL);
	//reg &= ~UDC_DWC3_GCTL_RAMCLKSEL_MASK;
	//reg |= UDC_DWC3_GCTL_RAMCLKSEL_BUS_CLK;
	//reg |= UDC_DWC3_GCTL_RAMCLKSEL_PIPE_CLK;
	//reg |= UDC_DWC3_GCTL_RAMCLKSEL_PIPE_DIV2_CLK;
	//reg |= UDC_DWC3_GCTL_RAMCLKSEL_MAC2_CLK;
	//sys_write32(reg, base + UDC_DWC3_GCTL);

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

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS0);
	LOG_DBG("GHWPARAMS0 = 0x%08x", reg);

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS1);
	LOG_DBG("GHWPARAMS1 = 0x%08x", reg);

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS2);
	LOG_DBG("GHWPARAMS2 = 0x%08x", reg);

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS3);
	LOG_DBG("GHWPARAMS3 = 0x%08x", reg);

	LOG_DBG("- GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS3_CACHE_TOTAL_XFER_RESOURCES_MASK, reg));
	LOG_DBG("- GHWPARAMS3_NUM_IN_EPS %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS3_NUM_IN_EPS_MASK, reg));
	LOG_DBG("- GHWPARAMS3_NUM_EPS %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS3_NUM_EPS_MASK, reg));

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS4);
	LOG_DBG("GHWPARAMS4 = 0x%08x", reg);

	LOG_DBG("- BMU_LSP_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS4_BMU_LSP_DEPTH_MASK, reg));
	LOG_DBG("- BMU_PTL_DEPTH: %lu locations",
		FIELD_GET(UDC_DWC3_GHWPARAMS4_BMU_PTL_DEPTH_M1_MASK, reg) + 1);
	LOG_DBG("- CACHE_TRBS_PER_TRANSFER: %lu",
		FIELD_GET(UDC_DWC3_GHWPARAMS4_CACHE_TRBS_PER_TRANSFER_MASK, reg));

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS5);
	LOG_DBG("GHWPARAMS5 = 0x%08x", reg);

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

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS6);
	LOG_DBG("GHWPARAMS6 = 0x%08x", reg);

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS7);
	LOG_DBG("GHWPARAMS7 = 0x%08x", reg);

	reg = sys_read32(base + UDC_DWC3_GHWPARAMS8);
	LOG_DBG("GHWPARAMS8 = 0x%08x", reg);

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

	LOG_DBG("checking for pending transfers for EP%02x", ep_data->cfg.addr);

	udc_lock_internal(dev, K_FOREVER);

	if (ep_data->cfg.stat.halted) {
		LOG_DBG("endpoint is halted, not processing buffers");
		goto unlock;
	}

	if (ep_data->end_xfer_pending || ep_data->resume_pending) {
		LOG_DBG("EP%02x still concluding an End Transfer, deferring", ep_data->cfg.addr);
		goto unlock;
	}

	while (true) {
		buf = udc_buf_peek(&ep_data->cfg);
		if (buf == NULL) {
			break;
		}

		LOG_DBG("Processing buffer %p from queue", (void *)buf);

		ret = udc_dwc3_trb_bulk(dev, ep_data, buf);
		if (ret != 0) {
			LOG_DBG("abort: No more room for buffer");
			break;
		}

		LOG_DBG("success: Buffer enqueued");

		udc_buf_get(&ep_data->cfg);
	}

unlock:
	udc_unlock_internal(dev);
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

	k_fifo_init(&ep_data->requeue_fifo);

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
		/* The IN loop above does this; the OUT loop did not, leaving every
		 * non-control OUT endpoint with an uninitialised requeue_fifo that
		 * udc_dwc3_ep_resume() reads on every resume. */
		k_fifo_init(&ep_data->requeue_fifo);

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

#define _EVT_BUF_ALIGNMENT (CONFIG_UDC_DWC3_EVENTS_NUM * sizeof(uint32_t) * BITS_PER_BYTE)

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
		__aligned(_EVT_BUF_ALIGNMENT);					\
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
	}, {
		.name = "AuxEventQ",
		.type = UDC_DWC3_GDBGFIFOSPACE_QUEUETYPE_AUXEVENTQ
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

	for (size_t i = 0; i < ARRAY_SIZE(udc_dwc3_regs); i++) {
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
	case UDC_DWC3_LINK_STATE_USB2_ON_STATE:
		shell_print(sh, "DWC3_LINK_STATE_USB2_ON_STATE");
		break;
	case UDC_DWC3_LINK_STATE_USB2_SLEEP_STATE:
		shell_print(sh, "DWC3_LINK_STATE_USB2_SLEEP_STATE");
		break;
	case UDC_DWC3_LINK_STATE_USB2_SUSPEND_STATE:
		shell_print(sh, "DWC3_LINK_STATE_USB2_SUSPEND_STATE");
		break;
	case UDC_DWC3_LINK_STATE_USB2_DISCONNECTED:
		shell_print(sh, "DWC3_LINK_STATE_USB2_DISCONNECTED");
		break;
	case UDC_DWC3_LINK_STATE_USB2_EARLY_SUSPEND:
		shell_print(sh, "DWC3_LINK_STATE_USB2_EARLY_SUSPEND");
		break;
	case UDC_DWC3_LINK_STATE_USB2_RESET:
		shell_print(sh, "DWC3_LINK_STATE_USB2_RESET");
		break;
	case UDC_DWC3_LINK_STATE_USB2_RESUME:
		shell_print(sh, "DWC3_LINK_STATE_USB2_RESUME");
		break;
	}
	return;
usb3:
	switch (reg & UDC_DWC3_DSTS_USBLNKST_MASK) {
	case UDC_DWC3_LINK_STATE_USB3_U0:
		shell_print(sh, "LINK_STATE_USB3_U0");
		break;
	case UDC_DWC3_LINK_STATE_USB3_U1:
		shell_print(sh, "LINK_STATE_USB3_U1");
		break;
	case UDC_DWC3_LINK_STATE_USB3_U2:
		shell_print(sh, "LINK_STATE_USB3_U2");
		break;
	case UDC_DWC3_LINK_STATE_USB3_U3:
		shell_print(sh, "LINK_STATE_USB3_U3");
		break;
	case UDC_DWC3_LINK_STATE_USB3_SS_DIS:
		shell_print(sh, "LINK_STATE_USB3_SS_DIS");
		break;
	case UDC_DWC3_LINK_STATE_USB3_RX_DET:
		shell_print(sh, "LINK_STATE_USB3_RX_DET");
		break;
	case UDC_DWC3_LINK_STATE_USB3_SS_INACT:
		shell_print(sh, "LINK_STATE_USB3_SS_INACT");
		break;
	case UDC_DWC3_LINK_STATE_USB3_POLL:
		shell_print(sh, "LINK_STATE_USB3_POLL");
		break;
	case UDC_DWC3_LINK_STATE_USB3_RECOV:
		shell_print(sh, "LINK_STATE_USB3_RECOV");
		break;
	case UDC_DWC3_LINK_STATE_USB3_HRESET:
		shell_print(sh, "LINK_STATE_USB3_HRESET");
		break;
	case UDC_DWC3_LINK_STATE_USB3_CMPLY:
		shell_print(sh, "LINK_STATE_USB3_CMPLY");
		break;
	case UDC_DWC3_LINK_STATE_USB3_LPBK:
		shell_print(sh, "LINK_STATE_USB3_LPBK");
		break;
	case UDC_DWC3_LINK_STATE_USB3_RESET_RESUME:
		shell_print(sh, "LINK_STATE_USB3_RESET_RESUME");
		break;
	}
}

static void udc_dwc3_dump_events(const struct device *dev, const struct shell *sh)
{
	const struct udc_dwc3_config *cfg = dev->config;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_EVENTS_NUM; i++) {
		uint32_t evt = cfg->evt_buf[i];
		char *s = (i == priv->evt_next) ? "<-" : "  ";

		shell_print(sh, "evt 0x%02x: 0x%08x %s %s",
			i, evt, s, udc_dwc3_get_event_name(evt));
	}
}

static void udc_dwc3_dump_trb(const struct device *dev, struct udc_dwc3_ep_data *ep_data,
			      const struct shell *sh)
{
	for (uint32_t i = 0; i < CONFIG_UDC_DWC3_TRB_NUM; i++) {
		struct udc_dwc3_trb trb = ep_data->trb_buf[i];
		bool hwo = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_HWO);
		bool lst = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_LST);
		bool chn = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_CHN);
		bool csp = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_CSP);
		bool isp = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_ISP_IMI);
		bool ioc = !!(trb.ctrl & UDC_DWC3_TRB_CTRL_IOC);
		bool spr = !!(trb.status & UDC_DWC3_TRB_STATUS_SPR);
		uint32_t trbctl = FIELD_GET(UDC_DWC3_TRB_CTRL_TRBCTL_MASK, trb.ctrl);
		uint32_t trbsts = FIELD_GET(UDC_DWC3_TRB_STATUS_TRBSTS_MASK, trb.status);
		uint32_t pcm1 = FIELD_GET(UDC_DWC3_TRB_STATUS_PCM1_MASK, trb.status);
		uint32_t sidsofn = FIELD_GET(UDC_DWC3_TRB_CTRL_SIDSOFN_MASK, trb.ctrl);
		uint32_t bufsiz = FIELD_GET(UDC_DWC3_TRB_STATUS_BUFSIZ_MASK, trb.status);
		char *head = (i == ep_data->head) ? " <HEAD" : "";
		char *tail = (i == ep_data->tail) ? " <TAIL" : "";
		char *full = (i == ep_data->head && ep_data->full) ? " <FULL" : "";

#define FMT	"%p EP%02x addr=0x%08x%08x ctl=%u sts=%u hwo=%u lst=%u chn=%u" \
		" csp=%u isp=%u ioc=%u spr=%u pcm1=%u sof=%u bufsiz=%u%s%s%s"

#define ARGS	&ep_data->trb_buf[i], ep_data->cfg.addr, trb.addr_hi, trb.addr_lo, trbctl, trbsts, \
		hwo, lst, chn, csp, isp, ioc, spr, pcm1, sidsofn, bufsiz, head, tail, full

		if (sh == NULL) {
			LOG_WRN(FMT, ARGS);
		} else {
			shell_print(sh, FMT, ARGS);
		}
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

		/* xferrscidx is included so a stuck endpoint can be compared against the
		 * value reported by its last "DepStartXfer done ... xferrscidx=" line.
		 * A mismatch means the index was overwritten by a command that did not
		 * succeed, which every later Update and End Transfer would then be using.
		 * Shell output only - no effect on the data path.
		 */
		shell_print(sh, "%s for IN endpoint 0x%02x (%u %s) xferrscidx=0x%x",
			  label, addr, addr & 0x7f, (addr & 0x80) ? "IN" : "OUT",
			  ep_data->xferrscidx);
		(*fn)(dev, ep_data, sh);
	}

	for (int i = 0; i < cfg->num_out_eps; i++) {
		struct udc_dwc3_ep_data *ep_data = &cfg->ep_data_out[i];
		uint8_t addr = ep_data->cfg.addr;

		if (ep_data->trb_buf == NULL) {
			continue;
		}

		shell_print(sh, "%s for OUT endpoint 0x%02x (%u %s) xferrscidx=0x%x",
			  label, addr, addr & 0x7f, (addr & 0x80) ? "IN" : "OUT",
			  ep_data->xferrscidx);
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
		shell_print(sh, "FIFO %zu", n);

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

	udc_lock_internal(dev, K_FOREVER);
	(*fn)(dev, sh);
	udc_unlock_internal(dev);

	return 0;
}

static void udc_dwc3_cmd_trb_ctrl_status_in(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3 IN");

	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_IN, 128);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
}
static int cmd_dwc3_trb_ctrl_status_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_status_in);
}

static void udc_dwc3_cmd_trb_ctrl_status_out(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3 OUT");

	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_OUT, 128);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_STATUS_3);
}
static int cmd_dwc3_trb_ctrl_status_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_status_out);
}

static void udc_dwc3_cmd_trb_ctrl_data_out(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA OUT");

	/* 512 is wMaxPacketSize0 for USB3 */
	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_OUT, 512);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
}
static int cmd_dwc3_trb_ctrl_data_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_data_out);
}

static void udc_dwc3_cmd_trb_ctrl_data_in(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA IN");

	/* 512 is wMaxPacketSize0 for USB3 */
	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_IN, 512);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	udc_dwc3_trb_ctrl_in(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_DATA);
}
static int cmd_dwc3_trb_ctrl_data_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_data_in);
}

static void udc_dwc3_cmd_trb_ctrl_setup(const struct device *dev, const struct shell *sh)
{
	struct net_buf *buf;

	shell_print(sh, "New UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP (OUT)");

	/* 512 is wMaxPacketSize0 for USB3 */
	buf = udc_ep_buf_alloc(dev, USB_CONTROL_EP_OUT, 512);
	if (buf == NULL) {
		shell_error(sh, "Failed to allocate a buffer");
		return;
	}

	udc_dwc3_trb_ctrl_out(dev, buf, UDC_DWC3_TRB_CTRL_TRBCTL_CONTROL_SETUP);
}
static int cmd_dwc3_trb_ctrl_setup(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_trb_ctrl_setup);
}

static void udc_dwc3_cmd_end_ctrl_in(const struct device *dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_in[0], 0);
}
static int cmd_dwc3_end_ctrl_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_end_ctrl_in);
}

static void udc_dwc3_cmd_end_ctrl_out(const struct device *dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	udc_dwc3_depcmd_end_xfer(dev, &cfg->ep_data_out[0], UDC_DWC3_DEPCMD_HIPRI_FORCERM);
}
static int cmd_dwc3_end_ctrl_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_end_ctrl_out);
}

static void udc_dwc3_cmd_fake_xfercomplete(const struct device *const dev, const struct shell *sh)
{
	struct udc_dwc3_data *const priv = udc_get_private(dev);

	if (priv->last_xfer_dir == USB_EP_DIR_IN) {
		udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(1));
	} else {
		udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(0));
	}
}
static int cmd_fake_xfercomplete(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_fake_xfercomplete);
}

static void udc_dwc3_cmd_fake_xfercomplete0(const struct device *const dev, const struct shell *sh)
{
	udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(0));
}
static int cmd_fake_xfercomplete0(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_fake_xfercomplete0);
}

static void udc_dwc3_cmd_fake_xfercomplete1(const struct device *const dev, const struct shell *sh)
{
	udc_dwc3_handle_event(dev, UDC_DWC3_DEPEVT_XFERCOMPLETE(1));
}
static int cmd_fake_xfercomplete1(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_fake_xfercomplete1);
}

static void udc_dwc3_cmd_dwc3_stall_ctrl_out(const struct device *const dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_out[0]);
}
static int cmd_dwc3_stall_ctrl_out(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_dwc3_stall_ctrl_out);
}

static void udc_dwc3_cmd_dwc3_stall_ctrl_in(const struct device *const dev, const struct shell *sh)
{
	const struct udc_dwc3_config *const cfg = dev->config;

	udc_dwc3_depcmd_set_stall(dev, &cfg->ep_data_in[0]);
}
static int cmd_dwc3_stall_ctrl_in(const struct shell *sh, size_t argc, char **argv)
{
	return dump_cmd2_handler(sh, argc, argv, udc_dwc3_cmd_dwc3_stall_ctrl_in);
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
	SHELL_CMD_ARG(trb_ctrl_setup, &dsub_device_name,
		      "Send a SETUP TRB to the CTRL OUT endpoint\nUsage: trb_ctrl_setup <device>",
		      cmd_dwc3_trb_ctrl_setup, 2, 0),
	SHELL_CMD_ARG(trb_ctrl_data_in, &dsub_device_name,
		      "Send a DATA TRB to the CTRL IN endpoint\nUsage: trb_ctrl_data_in <device>",
		      cmd_dwc3_trb_ctrl_data_in, 2, 0),
	SHELL_CMD_ARG(trb_ctrl_data_out, &dsub_device_name,
		      "Send a DATA TRB to the CTRL OUT endpoint\nUsage: trb_ctrl_data_out <device>",
		      cmd_dwc3_trb_ctrl_data_out, 2, 0),
	SHELL_CMD_ARG(trb_ctrl_status_in, &dsub_device_name,
		      "Send a STATUS TRB to the CTRL IN endpoint\nUsage: trb_ctrl_status_in <device>",
		      cmd_dwc3_trb_ctrl_status_in, 2, 0),
	SHELL_CMD_ARG(trb_ctrl_status_out, &dsub_device_name,
		      "Send a STATUS TRB to the CTRL OUT endpoint\nUsage: trb_ctrl_status_out <device>",
		      cmd_dwc3_trb_ctrl_status_out, 2, 0),
	SHELL_CMD_ARG(end_ctrl_in, &dsub_device_name,
		      "End a transfer for the CTRL IN endpoint\nUsage: end_ctrl_in <device>",
		      cmd_dwc3_end_ctrl_in, 2, 0),
	SHELL_CMD_ARG(end_ctrl_out, &dsub_device_name,
		      "End a transfer for the CTRL OUT endpoint\nUsage: end_ctrl_out <device>",
		      cmd_dwc3_end_ctrl_out, 2, 0),
	SHELL_CMD_ARG(stall_ctrl_in, &dsub_device_name,
		      "Stall the CTRL IN endpoint\nUsage: stall_ctrl_in <device>",
		      cmd_dwc3_stall_ctrl_in, 2, 0),
	SHELL_CMD_ARG(stall_ctrl_out, &dsub_device_name,
		      "Stall the CTRL OUT endpoint\nUsage: stall_ctrl_out <device>",
		      cmd_dwc3_stall_ctrl_out, 2, 0),
	SHELL_CMD_ARG(fake_xfercomplete, &dsub_device_name,
		      "Fake an XFERCOMMPLETE(0|1) event\nUsage: cmd_fake_xfercomplete <device>",
		      cmd_fake_xfercomplete, 2, 0),
	SHELL_CMD_ARG(fake_xfercomplete0, &dsub_device_name,
		      "Fake an XFERCOMMPLETE(0) event\nUsage: cmd_fake_xfercomplete0 <device>",
		      cmd_fake_xfercomplete0, 2, 0),
	SHELL_CMD_ARG(fake_xfercomplete1, &dsub_device_name,
		      "Fake an XFERCOMMPLETE(1) event\nUsage: cmd_fake_xfercomplete1 <device>",
		      cmd_fake_xfercomplete1, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(dwc3, &sub_dwc3, "Synopsys DWC3 controller commands", NULL);

#endif /* CONFIG_UDC_DWC3_SHELL */
