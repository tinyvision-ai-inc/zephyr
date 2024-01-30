#define MAIN_RAM_BASE 0x40000000
#define CONFIG_L2_SIZE 1000

static void flush_l2_cache(void)
{
	unsigned int i;

	for(i = 0; i < 2 * CONFIG_L2_SIZE / 4; i++) {
		((volatile unsigned int *)MAIN_RAM_BASE)[i];
	}
}

__attribute__((unused)) static void flush_cpu_icache(void)
{
	__asm__ volatile(".word(0x100F)\n""nop\n""nop\n""nop\n""nop\n""nop\n");
}

__attribute__((unused)) static void flush_cpu_dcache(void)
{
	__asm__ volatile(".word(0x500F)\n");
}
