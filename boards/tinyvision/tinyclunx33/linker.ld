#include <zephyr/devicetree.h>
#include <zephyr/linker/linker-tool.h>

SECTIONS
{
	SECTION_PROLOGUE(USB23_DMA,,)
	{
		/* All symbols with this section are going to a memory region
		 * accessible to the USB23 core performing DMA requests */

		. = ALIGN(16);
		*(*.kheap_buf_net_buf_mem_pool_udc_ep_pool);
		*(.bss.usb23_dma_*);
		*(.bss.uvc_payload_header_*);
	} GROUP_LINK_IN(USB23_DMA)
}

#include <zephyr/arch/riscv/common/linker.ld>
