#include <zephyr/linker/linker-tool.h>
#include <zephyr/devicetree.h>
#include <zephyr/linker/devicetree_regions.h>

MEMORY
{
	LINKER_DT_REGIONS()
}

SECTIONS
{
	SECTION_PROLOGUE(USB23_DMA,,)
	{
		/* All symbols with this section are going to a memory region
		 * accessible to the USB23 core performing DMA requests */

		. = ALIGN(16);
		/* Control endpoint IN and OUT */
		*(*.kheap_buf_net_buf_mem_pool_udc_ep_pool);

		/* CDC ACM class */
		*(*.kheap_buf_net_buf_mem_pool_cdc_acm_ep_pool);

		/* TRB buffers and event buffer of USB23 driver */
		*(.bss.usb23_dma_*);

		/* UVC class */
		*(.bss.uvc_payload_header_*);
	} GROUP_LINK_IN(USB23_DMA)
}

#include <zephyr/arch/riscv/common/linker.ld>
