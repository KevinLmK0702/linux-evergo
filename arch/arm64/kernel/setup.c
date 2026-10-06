// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on arch/arm/kernel/setup.c
 *
 * Copyright (C) 1995-2001 Russell King
 * Copyright (C) 2012 ARM Ltd.
 */

#include <linux/acpi.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/initrd.h>
#include <linux/console.h>
#include <linux/cache.h>
#include <linux/screen_info.h>
#include <linux/init.h>
#include <linux/kexec.h>
#include <linux/root_dev.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/fs.h>
#include <linux/panic_notifier.h>
#include <linux/proc_fs.h>
#include <linux/memblock.h>
#include <linux/of_fdt.h>
#include <linux/firmware.h>
#include <linux/efi.h>
#include <linux/psci.h>
#include <linux/sched/task.h>
#include <linux/scs.h>
#include <linux/mm.h>
#include <linux/io.h>		/* memremap() for the pstore text log */
#include <linux/slab.h>		/* slab_is_available() gate for it */

#include <asm/acpi.h>
#include <asm/fixmap.h>
#include <asm/cpu.h>
#include <asm/cputype.h>
#include <asm/daifflags.h>
#include <asm/elf.h>
#include <asm/cpufeature.h>
#include <asm/cpu_ops.h>
#include <asm/evergo.h>
#include <asm/kasan.h>
#include <asm/numa.h>
#include <asm/rsi.h>
#include <asm/scs.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/traps.h>
#include <asm/efi.h>
#include <asm/xen/hypervisor.h>
#include <asm/mmu_context.h>

static int num_standard_resources;
static struct resource *standard_resources;

phys_addr_t __fdt_pointer __initdata;
u64 mmu_enabled_at_boot __initdata;

/*
 * Standard memory resources
 */
static struct resource mem_res[] = {
	{
		.name = "Kernel code",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_SYSTEM_RAM
	},
	{
		.name = "Kernel data",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_SYSTEM_RAM
	}
};

#define kernel_code mem_res[0]
#define kernel_data mem_res[1]

/*
 * The recorded values of x0 .. x3 upon kernel entry.
 */
u64 __cacheline_aligned boot_args[4];

void __init smp_setup_processor_id(void)
{
	u64 mpidr = read_cpuid_mpidr() & MPIDR_HWID_BITMASK;
	set_cpu_logical_map(0, mpidr);

	pr_info("Booting Linux on physical CPU 0x%010lx [0x%08x]\n",
		(unsigned long)mpidr, read_cpuid_id());
}

bool arch_match_cpu_phys_id(int cpu, u64 phys_id)
{
	return phys_id == cpu_logical_map(cpu);
}

struct mpidr_hash mpidr_hash;
/**
 * smp_build_mpidr_hash - Pre-compute shifts required at each affinity
 *			  level in order to build a linear index from an
 *			  MPIDR value. Resulting algorithm is a collision
 *			  free hash carried out through shifting and ORing
 */
static void __init smp_build_mpidr_hash(void)
{
	u32 i, affinity, fs[4], bits[4], ls;
	u64 mask = 0;
	/*
	 * Pre-scan the list of MPIDRS and filter out bits that do
	 * not contribute to affinity levels, ie they never toggle.
	 */
	for_each_possible_cpu(i)
		mask |= (cpu_logical_map(i) ^ cpu_logical_map(0));
	pr_debug("mask of set bits %#llx\n", mask);
	/*
	 * Find and stash the last and first bit set at all affinity levels to
	 * check how many bits are required to represent them.
	 */
	for (i = 0; i < 4; i++) {
		affinity = MPIDR_AFFINITY_LEVEL(mask, i);
		/*
		 * Find the MSB bit and LSB bits position
		 * to determine how many bits are required
		 * to express the affinity level.
		 */
		ls = fls(affinity);
		fs[i] = affinity ? ffs(affinity) - 1 : 0;
		bits[i] = ls - fs[i];
	}
	/*
	 * An index can be created from the MPIDR_EL1 by isolating the
	 * significant bits at each affinity level and by shifting
	 * them in order to compress the 32 bits values space to a
	 * compressed set of values. This is equivalent to hashing
	 * the MPIDR_EL1 through shifting and ORing. It is a collision free
	 * hash though not minimal since some levels might contain a number
	 * of CPUs that is not an exact power of 2 and their bit
	 * representation might contain holes, eg MPIDR_EL1[7:0] = {0x2, 0x80}.
	 */
	mpidr_hash.shift_aff[0] = MPIDR_LEVEL_SHIFT(0) + fs[0];
	mpidr_hash.shift_aff[1] = MPIDR_LEVEL_SHIFT(1) + fs[1] - bits[0];
	mpidr_hash.shift_aff[2] = MPIDR_LEVEL_SHIFT(2) + fs[2] -
						(bits[1] + bits[0]);
	mpidr_hash.shift_aff[3] = MPIDR_LEVEL_SHIFT(3) +
				  fs[3] - (bits[2] + bits[1] + bits[0]);
	mpidr_hash.mask = mask;
	mpidr_hash.bits = bits[3] + bits[2] + bits[1] + bits[0];
	pr_debug("MPIDR hash: aff0[%u] aff1[%u] aff2[%u] aff3[%u] mask[%#llx] bits[%u]\n",
		mpidr_hash.shift_aff[0],
		mpidr_hash.shift_aff[1],
		mpidr_hash.shift_aff[2],
		mpidr_hash.shift_aff[3],
		mpidr_hash.mask,
		mpidr_hash.bits);
	/*
	 * 4x is an arbitrary value used to warn on a hash table much bigger
	 * than expected on most systems.
	 */
	if (mpidr_hash_size() > 4 * num_possible_cpus())
		pr_warn("Large number of MPIDR hash buckets detected\n");
}

static void __init setup_machine_fdt(phys_addr_t dt_phys)
{
	int size = 0;
	void *dt_virt = fixmap_remap_fdt(dt_phys, &size, PAGE_KERNEL);
	const char *name;

	if (dt_virt)
		memblock_reserve(dt_phys, size);

	/*
	 * dt_virt is a fixmap address, hence __pa(dt_virt) can't be used.
	 * Pass dt_phys directly.
	 */
	if (!early_init_dt_scan(dt_virt, dt_phys)) {
		pr_crit("\n"
			"Error: invalid device tree blob: PA=%pa, VA=%px, size=%d bytes\n"
			"The dtb must be 8-byte aligned and must not exceed 2 MB in size.\n"
			"\nPlease check your bootloader.\n",
			&dt_phys, dt_virt, size);

		/*
		 * Note that in this _really_ early stage we cannot even BUG()
		 * or oops, so the least terrible thing to do is cpu_relax(),
		 * or else we could end-up printing non-initialized data, etc.
		 */
		while (true)
			cpu_relax();
	}

	/* Early fixups are done, map the FDT as read-only now */
	fixmap_remap_fdt(dt_phys, &size, PAGE_KERNEL_RO);

	name = of_flat_dt_get_machine_name();
	if (!name)
		return;

	pr_info("Machine model: %s\n", name);
	dump_stack_set_arch_desc("%s (DT)", name);
}

static void __init request_standard_resources(void)
{
	struct memblock_region *region;
	struct resource *res;
	unsigned long i = 0;
	size_t res_size;

	kernel_code.start   = __pa_symbol(_text);
	kernel_code.end     = __pa_symbol(__init_begin - 1);
	kernel_data.start   = __pa_symbol(_sdata);
	kernel_data.end     = __pa_symbol(_end - 1);
	insert_resource(&iomem_resource, &kernel_code);
	insert_resource(&iomem_resource, &kernel_data);

	num_standard_resources = memblock.memory.cnt;
	res_size = num_standard_resources * sizeof(*standard_resources);
	standard_resources = memblock_alloc_or_panic(res_size, SMP_CACHE_BYTES);

	for_each_mem_region(region) {
		res = &standard_resources[i++];
		if (memblock_is_nomap(region)) {
			res->name  = "reserved";
			res->flags = IORESOURCE_MEM;
			res->start = __pfn_to_phys(memblock_region_reserved_base_pfn(region));
			res->end = __pfn_to_phys(memblock_region_reserved_end_pfn(region)) - 1;
		} else {
			res->name  = "System RAM";
			res->flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;
			res->start = __pfn_to_phys(memblock_region_memory_base_pfn(region));
			res->end = __pfn_to_phys(memblock_region_memory_end_pfn(region)) - 1;
		}

		insert_resource(&iomem_resource, res);
	}
}

static int __init reserve_memblock_reserved_regions(void)
{
	u64 i, j;

	for (i = 0; i < num_standard_resources; ++i) {
		struct resource *mem = &standard_resources[i];
		phys_addr_t r_start, r_end, mem_size = resource_size(mem);

		if (!memblock_is_region_reserved(mem->start, mem_size))
			continue;

		for_each_reserved_mem_range(j, &r_start, &r_end) {
			resource_size_t start, end;

			start = max(PFN_PHYS(PFN_DOWN(r_start)), mem->start);
			end = min(PFN_PHYS(PFN_UP(r_end)) - 1, mem->end);

			if (start > mem->end || end < mem->start)
				continue;

			reserve_region_with_split(mem, start, end, "reserved");
		}
	}

	return 0;
}
arch_initcall(reserve_memblock_reserved_regions);

u64 __cpu_logical_map[NR_CPUS] = { [0 ... NR_CPUS-1] = INVALID_HWID };

u64 cpu_logical_map(unsigned int cpu)
{
	return __cpu_logical_map[cpu];
}

#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
/*
 * Early boot visibility on a board without UART.
 *
 * A reserved DRAM area (0x48090000 on this board, see the device tree) is used
 * as a log buffer that the MTK LK copies into the expdb partition on the next
 * boot, so whatever ends up there is readable afterwards - even when the
 * kernel died before any console or the ramoops driver existed.
 *
 * The buffer is filled by a console registered right after bootmem_init(),
 * i.e. as soon as the linear map covers DRAM: before paging_init() a direct
 * DRAM write is not possible (DRAM is not mapped yet and early_ioremap()
 * refuses RAM). That console is the only writer of the area, so no
 * persistent_ram/pstore bookkeeping is involved: it just keeps the layout LK
 * expects (signature, start, size) and appends text.
 *
 * evergo_mark() sends a breadcrumb through printk; before the console exists
 * it only lands in the kernel log ring, but CON_PRINTBUFFER replays the ring
 * into the area as soon as the console registers.
 */
#define EVERGO_ZONE_BASE	0x48090000UL	/* pstore console area */
#define EVERGO_ZONE_SIG		0x43474244U	/* "DBGC" */
#define EVERGO_MARK_LIMIT	0x800U		/* breadcrumbs only */

/*
 * Last resort channel: the RAM_CONSOLE buffer in SRAM (LK's boot argument
 * area, 0x11d000 + 0xec0). Its "exp_type" field is printed by the LK on every
 * boot as "RAM_CONSOLE. wdt_status ..., exp_type 0x..", with the magic below
 * decoded back to a small value, so a 4 bit progress code here survives even
 * if nothing else can be read back.
 */
#define EVERGO_RAMCONSOLE_BASE	0x11dec0UL
#define EVERGO_RAMCONSOLE_SIG	0x43474244U	/* same sig the vendor sets */
#define EVERGO_EXP_TYPE_MAGIC	0xaeedead0U
#define EVERGO_OFF_LINUX_OFF	44		/* off_linux in ram_console_buffer */
#define EVERGO_EXP_TYPE_OFF	4		/* exp_type in last_reboot_reason */

/*
 * The SRAM ram console (0x11d000) is the channel that survives a death at
 * any stage: LK prints its "fiq_step" field on every single boot
 * ("RAM_CONSOLE. wdt_status 0x?, fiq_step 0x?, exp_type 0x?") and that field
 * is always 0x0 in factory logs, so any non-zero value proves this code ran.
 * Values grow with every checkpoint, so the highest one seen in a later
 * read-back tells us exactly how far the kernel got.  Unlike the DRAM
 * breadcrumb zone this needs no MMU-resident memory at all.
 *
 * head.S uses 0xa1..0xa5, this file (setup_arch and init/main.c) 0xb1..0xbf.
 */
#define EVERGO_SRAM_BASE	0x11d000UL
#define EVERGO_FIQ_STEP_OFF	0x3c

/* cleared once paging_init() has built the linear map */
static bool evergo_sram_early = true;

/*
 * evergo_ring - the only console this board really has.
 *
 * LK keeps its own log in a 256 KiB ring at 0x7ffbf000 and flushes the whole
 * ring into the expdb partition at the end of every session:
 *
 *	LK_LOG_STORE: dram pl/lk log buff mapping start addr = 0x7ffbf000, size = 0x40000
 *	LK_LOG_STORE: start save pllk log
 *	LK_LOG_STORE: part_size 41943040.
 *
 * The ring is ordinary DRAM (the DT node is reserved but not no-map, so it is
 * inside the linear map), it survives watchdog resets, and a read-back proved
 * that byte-for-byte copies of what the kernel leaves there reach expdb (a test
 * stamp written by head.S survived 81 times).  That matters because this board
 * has no UART and ramoops only reaches expdb once the kernel got far enough to
 * register it - i.e. never, when the kernel dies early.
 *
 * Layout inside each 4 KiB slot (LK keeps the other 93% of its log):
 *	+0x000	16 B	tick:  "EVTICK:" + u32 value
 * The log is a stream of 2 KiB frames, 64 of them (128 KiB of text), written
 * round-robin, so LK still keeps half of its own log and we keep roughly a
 * thousand lines of kernel log - which the next read-back returns verbatim.
 *
 * Frame: "EVFRAME" + u32 seq + u32 len + text
 */
#define EVERGO_RING_BASE	0x7ffbf000UL
#define EVERGO_RING_SIZE	0x40000UL
#define EVERGO_RING_SLOT	0x1000UL
#define EVERGO_RING_SLOTS	64
#define EVERGO_TICK_OFF		0x000
#define EVERGO_LOG_OFF		0x100
#define EVERGO_LOG_LEN		0x800	/* 2 KiB of text per frame */

static unsigned int evergo_frame;	/* frame sequence number */
static unsigned int evergo_slot;	/* frame currently being filled */
static unsigned int evergo_fill;	/* bytes used inside it */

/*
 * The ring lives in DRAM that is only reachable through the linear map once
 * paging_init() has run.  Writing to the raw physical address as if it were a
 * kernel pointer takes a translation fault - that is what killed the first
 * version of this code, exactly between the last head.S checkpoint and the
 * first C one - so before that point the ring is reached through the fixmap.
 */
static char *evergo_ring_enter(unsigned int i, bool *mapped)
{
	phys_addr_t pa = EVERGO_RING_BASE + i * EVERGO_RING_SLOT;

	*mapped = false;
	if (evergo_sram_early) {
		char *slot = (char *)early_memremap(pa, EVERGO_RING_SLOT);

		if (!slot)
			return NULL;
		*mapped = true;
		return slot;
	}
	return (char *)__va(pa);
}

static void evergo_ring_leave(char *slot, bool mapped)
{
	dcache_clean_poc((unsigned long)slot, (unsigned long)slot + EVERGO_RING_SLOT);
	if (mapped)
		early_iounmap((void __iomem *)slot, EVERGO_RING_SLOT);
}

/*
 * HBEAT - a heartbeat that does not go through printk.
 *
 * The frame ring and plog are both written from the console path, so one CPU
 * wedged inside a console write (holding the log lock - b189 went silent mid
 * fbdev commit at 0.65 s and never printed again) takes both channels down at
 * once, and a read-back cannot tell "the machine died" from "the log died".
 * This timer writes a plain-text stamp into every ring slot at offset +0x20
 * (free: EVTICK sits at +0x00, the build stamp at +0x40, the log at +0x100):
 *
 *	HBEAT:<uptime seconds>:<seq>:<cpu>
 *
 * It runs from a timer softirq, touches no printk lock, and is shipped to
 * expdb by LK like every other ring stamp, so a read-back always shows the
 * last moment the scheduler was still running.
 */
#define EVERGO_HBEAT_OFF	0x20

static void evergo_hbeat(struct timer_list *t);
static DEFINE_TIMER(evergo_hbeat_timer, evergo_hbeat);
static u32 evergo_hbeat_seq;

static void evergo_hbeat(struct timer_list *t)
{
	char buf[40];
	int n, i;

	evergo_hbeat_seq++;
	n = snprintf(buf, sizeof(buf), "HBEAT:%lu:%u:%u",
		     (unsigned long)(jiffies_64 / HZ), evergo_hbeat_seq,
		     raw_smp_processor_id());
	for (i = 0; i < EVERGO_RING_SLOTS; i++) {
		char *slot = (char *)__va(EVERGO_RING_BASE + i * EVERGO_RING_SLOT);

		memcpy(slot + EVERGO_HBEAT_OFF, buf, n + 1);
		dcache_clean_poc((unsigned long)(slot + EVERGO_HBEAT_OFF),
				 (unsigned long)(slot + EVERGO_HBEAT_OFF) + 48);
	}
	mod_timer(&evergo_hbeat_timer, jiffies + 2 * HZ);
}

static int __init evergo_hbeat_init(void)
{
	mod_timer(&evergo_hbeat_timer, jiffies + 2 * HZ);
	pr_info("evergo: hbeat timer armed\n");
	return 0;
}
late_initcall(evergo_hbeat_init);

/*
 * Progress tick. Stamped into every slot, so whatever LK overwrites later, the
 * highest surviving value in a read-back tells us how far the kernel got.
 */
static void evergo_tick(u32 val)
{
	unsigned int i;

	for (i = 0; i < EVERGO_RING_SLOTS; i++) {
		bool mapped;
		char *slot = evergo_ring_enter(i, &mapped);

		if (!slot)
			return;
		memcpy(slot + EVERGO_TICK_OFF, "EVTICK:", 7);
		*(u32 *)(slot + EVERGO_TICK_OFF + 8) = val;
		evergo_ring_leave(slot, mapped);
	}
}

/*
 * Mirror a frame into the slot half a ring away.  LK rewrites its own log
 * sequentially when the next session starts (about 85 KiB per session, i.e. a
 * contiguous window of ~21 slots), so a second copy 32 slots away guarantees
 * that every frame survives in at least one of its two copies.
 */
static void evergo_log_mirror(unsigned int i)
{
	unsigned int j = (i + EVERGO_RING_SLOTS / 2) % EVERGO_RING_SLOTS;
	bool ma, mb;
	char *a = evergo_ring_enter(i, &ma);
	char *b = evergo_ring_enter(j, &mb);

	if (a && b)
		memcpy(b + EVERGO_LOG_OFF, a + EVERGO_LOG_OFF, EVERGO_LOG_LEN);
	if (a)
		evergo_ring_leave(a, ma);
	if (b)
		evergo_ring_leave(b, mb);
}

/* Append text to the frame stream, starting a new frame when one is full. */
/*
 * evergo_plog - a plain text log in the unused tail of the pstore region.
 *
 * The ring above is LK's own buffer, and every console-based channel shares
 * one weakness: printk writes to the consoles in order, so a console that
 * blocks takes the whole log with it.  That is not hypothetical on this board
 * - uart0 has no working driver by default, but the moment something makes it
 * probe, "console=ttyS0" in the bootargs puts the 8250 console first and its
 * first printk busy-waits on a UART_THR that never drains.  A channel that
 * lives in memory the kernel writes directly cannot be caught by that.
 *
 * ramoops lays its zones out from 0x48090000: the console zone takes the
 * first 0x40000 and pmsg the next 0x10000.  With no record-size the "dmesg"
 * zone is skipped *without* advancing the cursor - ramoops_init_przs()
 * returns early for record_size == 0 - so the 0x90000 bytes from +0x50000 to
 * the end of the 0xe0000 window are never touched by anything (the next
 * reserved region, minirdump, starts exactly at 0x48170000).
 *
 * LK copies that whole window into expdb on its next boot, and mtkclient can
 * read it straight out of DRAM, so this is also a channel that survives a
 * board_reset() that never gets as far as LK.
 *
 * The region is no-map in the device tree, so it is *not* part of the linear
 * map and __va() must never be used on it; it is remapped with memremap()
 * once the vmalloc area exists (slab_is_available() is the gate).
 *
 *	+0x00	char[8]	"EVPLOG\0\0"
 *	+0x08	u32	capacity, bytes of text (EVERGO_PLOG_CAP)
 *	+0x0c	u32	written, monotonic, wraps at 2^32
 *	+0x10	u32	wpos, index of the next byte to write
 *	+0x14	u32	reserved
 *	+0x18	text ring; the most recent min(written, capacity) bytes end here
 */
#define EVERGO_PLOG_BASE	0x48090000UL
#define EVERGO_PLOG_OFF		0x50000UL	/* past ramoops console + pmsg */
#define EVERGO_PLOG_SIZE	0x90000UL
#define EVERGO_PLOG_HDR		0x18
#define EVERGO_PLOG_CAP		(EVERGO_PLOG_SIZE - EVERGO_PLOG_HDR)

static char *evergo_plog;
static u32 evergo_plog_written;
static u32 evergo_plog_wpos;

static void evergo_plog_open(void)
{
	u32 *h;

	if (evergo_plog || !slab_is_available())
		return;

	/*
	 * The window is no-map, so this is an ioremap under the hood, not a
	 * linear-map lookup.  WB is what we want (it is ordinary DRAM that LK
	 * copies out verbatim), but a no-map region is not always granted a
	 * cacheable mapping, so fall back to WC and say which one won: if both
	 * fail the next read-back must still tell us that, instead of looking
	 * exactly like a kernel that died before printk came up.
	 */
	evergo_plog = memremap(EVERGO_PLOG_BASE + EVERGO_PLOG_OFF,
			       EVERGO_PLOG_SIZE, MEMREMAP_WB);
	if (!evergo_plog)
		evergo_plog = memremap(EVERGO_PLOG_BASE + EVERGO_PLOG_OFF,
				       EVERGO_PLOG_SIZE, MEMREMAP_WC);
	if (!evergo_plog) {
		pr_warn("evergo: plog %#lx not mappable, ring only\n",
			EVERGO_PLOG_BASE + EVERGO_PLOG_OFF);
		return;
	}

	h = (u32 *)evergo_plog;
	if (memcmp(evergo_plog, "EVPLOG", 6)) {
		/* Cold boot: start a fresh one. */
		memset(evergo_plog, 0, EVERGO_PLOG_HDR);
		memcpy(evergo_plog, "EVPLOG", 6);
		h[2] = EVERGO_PLOG_CAP;
		evergo_plog_written = 0;
		evergo_plog_wpos = 0;
	} else {
		/* Warm reset: carry on where the previous life stopped. */
		evergo_plog_written = h[3];
		evergo_plog_wpos = h[4] % EVERGO_PLOG_CAP;
	}
	dcache_clean_poc((unsigned long)h, (unsigned long)(h + 6));
	pr_info("evergo: plog ready at %#lx, %lu bytes, carried %u\n",
		(unsigned long)(EVERGO_PLOG_BASE + EVERGO_PLOG_OFF),
		EVERGO_PLOG_SIZE, evergo_plog_written);
}

/*
 * The arming above happens inside setup_arch(), long before any of our log
 * channels exist, so that pr_info() is dropped.  Say it again once the ring
 * console is up (and reload once more, so the full window starts here): a
 * read-back then shows both that the arming took and what the hardware really
 * has, instead of us inferring it from where the log happens to stop.
 */
static void evergo_wdt_report(void)
{
	void __iomem *wdt = ioremap(0x10007000, 0x1000);
	u32 mode, len;

	if (!wdt) {
		pr_warn("evergo: wdt window not mappable\n");
		return;
	}
	writel(0x1971, wdt + 0x08);	/* WDT_RST: full window from here */
	mode = readl(wdt);
	len = readl(wdt + 0x04);
	iounmap(wdt);
	pr_info("evergo: wdt mode %#x len %#x -> %u s, pretimeout %u/64 s\n",
		mode, len, (len >> 11) & 0x1f, (len >> 6) & 0x1f);
}

/*
 * Opened from initcalls, never lazily from the write path: printk runs with
 * interrupts disabled often enough, and memremap() allocates, so mapping
 * there would sleep in atomic context.  Registered twice on purpose - the
 * second call is a no-op if the first one managed to map, and a retry if the
 * vmalloc area was not ready that early.
 */
static int __init evergo_plog_initcall(void)
{
	evergo_plog_open();
	if (evergo_plog)
		evergo_wdt_report();
	return 0;
}
early_initcall(evergo_plog_initcall);
subsys_initcall(evergo_plog_initcall);

static void evergo_plog_put(const char *s, size_t n)
{
	if (!evergo_plog)
		return;

	while (n) {
		size_t room = EVERGO_PLOG_CAP - evergo_plog_wpos;
		char *dst;

		if (room > n)
			room = n;
		dst = evergo_plog + EVERGO_PLOG_HDR + evergo_plog_wpos;
		memcpy(dst, s, room);
		dcache_clean_poc((unsigned long)dst, (unsigned long)dst + room);
		evergo_plog_wpos = (evergo_plog_wpos + room) % EVERGO_PLOG_CAP;
		evergo_plog_written += room;
		s += room;
		n -= room;
	}

	((u32 *)evergo_plog)[3] = evergo_plog_written;
	((u32 *)evergo_plog)[4] = evergo_plog_wpos;
	dcache_clean_poc((unsigned long)evergo_plog,
			 (unsigned long)evergo_plog + EVERGO_PLOG_HDR);
}

/* Append text to the frame stream, starting a new frame when one is full. */
static void evergo_log_put(const char *s, size_t n)
{
	/*
	 * Mirror everything into the pstore text log as well.  Same text, same
	 * order, but a channel nothing else can take away.
	 */
	evergo_plog_put(s, n);

	while (n) {
		bool mapped;
		char *f;
		size_t room, take;
		char *slot = evergo_ring_enter(evergo_slot, &mapped);

		if (!slot)
			return;
		f = slot + EVERGO_LOG_OFF;
		if (!evergo_fill) {
			memcpy(f, "EVFRAME", 7);
			*(u32 *)(f + 8) = evergo_frame++;
			*(u32 *)(f + 12) = 0;
			evergo_fill = 16;
		}
		room = EVERGO_LOG_LEN - evergo_fill;
		take = n < room ? n : room;
		memcpy(f + evergo_fill, s, take);
		evergo_fill += take;
		*(u32 *)(f + 12) = evergo_fill - 16;
		evergo_ring_leave(slot, mapped);
		evergo_log_mirror(evergo_slot);
		s += take;
		n -= take;
		if (evergo_fill == EVERGO_LOG_LEN) {
			evergo_fill = 0;
			evergo_slot = (evergo_slot + 1) % EVERGO_RING_SLOTS;
		}
	}
}

static void evergo_log(const char *s)
{
	evergo_log_put(s, strlen(s));
}

/*
 * The console.  It is registered with CON_PRINTBUFFER, which makes printk start
 * feeding it from the beginning of the ring buffer, so the messages printed
 * before it existed - "Booting Linux on physical CPU", "Linux version", the
 * reserved memory map - are replayed into our log as well.
 *
 * CON_ENABLED is set explicitly: register_console() only auto-enables a console
 * when no preferred console was named, and the bootargs name ttyS0 (whose
 * driver never registers on this board), so without the flag printk would
 * simply never call us.
 */
static void evergo_con_write(struct console *co, const char *s, unsigned int count)
{
	evergo_log_put(s, count);
}

static struct console evergo_console = {
	.name	= "evenring",
	.write	= evergo_con_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED,
	.index	= -1,
};

/*
 * Clock controls that decide whether the PMIC wrapper is reachable.
 *
 * The values are captured as early as possible - before any clk driver has
 * run - but they are deliberately NOT printed here.  The LK log ring is a
 * plain circular buffer, and the replay of the printk buffer that happens
 * when the ring console registers overwrites its front; a pr_info from
 * setup_arch is gone by the time userspace can look.  evergo_console_init()
 * re-emits the captured words once the ring exists, which does survive.
 */
static u32 evergo_clk_snap[12];

static void evergo_clk_capture(void)
{
	void __iomem *p;

	/* topckgen: pwrap_ulposc_sel mux+gate live in 0x090 */
	p = early_ioremap(0x10000000, 0x1000);
	if (p) {
		evergo_clk_snap[0] = readl(p + 0x90);
		evergo_clk_snap[1] = readl(p + 0x94);
		evergo_clk_snap[2] = readl(p + 0x98);
		evergo_clk_snap[3] = readl(p + 0x08);
		early_iounmap(p, 0x1000);
	}

	/* infracfg_ao bank 2: bit 0 ifrao_pmic_tmr, bit 1 ifrao_pmic_ap */
	p = early_ioremap(0x10001000, 0x1000);
	if (p) {
		evergo_clk_snap[4] = readl(p + 0x80);
		evergo_clk_snap[5] = readl(p + 0x84);
		evergo_clk_snap[6] = readl(p + 0x90);
		early_iounmap(p, 0x1000);
	}

	/* the wrapper itself, plus its second window */
	p = early_ioremap(0x10026000, 0x1000);
	if (p) {
		evergo_clk_snap[7] = readl(p);
		evergo_clk_snap[8] = readl(p + 0xc00);
		evergo_clk_snap[9] = readl(p + 0xc24);
		early_iounmap(p, 0x1000);
	}

	/* SPM ULPOSC_CON: the actual source of the wrapper's clock */
	p = early_ioremap(0x10006000, 0x1000);
	if (p) {
		evergo_clk_snap[10] = readl(p + 0x420);
		early_iounmap(p, 0x1000);
	}
}

static void evergo_clk_report(void)
{
	pr_info("EVERGOSNAP top[90]=%08x[94]=%08x[98]=%08x[08]=%08x\n",
		evergo_clk_snap[0], evergo_clk_snap[1],
		evergo_clk_snap[2], evergo_clk_snap[3]);
	pr_info("EVERGOSNAP inf[80]=%08x[84]=%08x[90]=%08x\n",
		evergo_clk_snap[4], evergo_clk_snap[5], evergo_clk_snap[6]);
	pr_info("EVERGOSNAP pw[00]=%08x[c00]=%08x[c24]=%08x\n",
		evergo_clk_snap[7], evergo_clk_snap[8], evergo_clk_snap[9]);
	pr_info("EVERGOSNAP spm[420]=%08x EN=%d RST=%d CG=%d SEL=%d\n",
		evergo_clk_snap[10], !!(evergo_clk_snap[10] & 1),
		!!(evergo_clk_snap[10] & 2), !!(evergo_clk_snap[10] & 4),
		!!(evergo_clk_snap[10] & 8));
}

void __init evergo_console_init(void)
{
	register_console(&evergo_console);
	evergo_clk_report();
}

static void __iomem *evergo_sram_io;

static void evergo_sram_step(u32 step)
{
	void __iomem *rc;

	if (evergo_sram_early) {
		rc = early_memremap(EVERGO_SRAM_BASE, 0x1000);
		if (!rc)
			return;
		writel(step, rc + EVERGO_FIQ_STEP_OFF);
		early_iounmap(rc, 0x1000);
		return;
	}

	/*
	 * Between the end of the early fixmap (paging_init() tears the fixmap
	 * down) and the point where the slab allocator exists there is no way
	 * to reach SRAM at all: early_memremap() is gone and
	 * generic_ioremap_prot() only warns ("WARNING: mm/ioremap.c:23") and
	 * returns NULL before the slab is up.  Drop the mark instead of
	 * triggering that warning.
	 */
	if (!slab_is_available())
		return;

	if (!evergo_sram_io)
		evergo_sram_io = ioremap(EVERGO_SRAM_BASE, 0x1000);
	if (!evergo_sram_io)
		return;
	writel(step, evergo_sram_io + EVERGO_FIQ_STEP_OFF);
}

static unsigned int evergo_code;

static void evergo_exp_type(unsigned int code)
{
	void __iomem *rc;
	u32 off_linux;

	/*
	 * early_memremap() lives in .init.text and its fixmap window is gone
	 * after paging_init(); never call it from the late marks (M1..M9 in
	 * init/main.c) or we jump into freed memory.
	 */
	if (!evergo_sram_early)
		return;

	rc = early_memremap(EVERGO_RAMCONSOLE_BASE, 0x100);
	if (!rc)
		return;
	if (readl(rc) == EVERGO_RAMCONSOLE_SIG) {
		off_linux = readl(rc + EVERGO_OFF_LINUX_OFF);
		if (off_linux && off_linux < 0x800)
			writel(EVERGO_EXP_TYPE_MAGIC | (code & 0xf),
			       rc + off_linux + EVERGO_EXP_TYPE_OFF);
	}
	early_iounmap(rc, 0x100);
}

static void __iomem *evergo_zone;
static u32 evergo_off = 12;	/* past the persistent_ram_buffer header */

void __init evergo_zone_map(void)
{
#ifdef CONFIG_PSTORE_RAM
	/*
	 * pstore/ramoops is enabled: the first 0x50000 bytes of the window are
	 * the ramoops zones and the persistent_ram header at 0x48090000 belongs
	 * to the driver.  Writing breadcrumbs into it would clobber the previous
	 * boot's bookkeeping and turn every replay into the breadcrumb text.
	 * The other early channels (SRAM fiq_step, the EVTICK stamps in the LK
	 * ring and, later, plog) still carry the boot progress; only the fake
	 * DBGC record here is skipped.
	 */
	return;
#else
	evergo_zone = early_memremap(EVERGO_ZONE_BASE, 0x1000);
#endif
}

void evergo_zone_close(void)
{
	/*
	 * NOTE: the early fixmap slot taken by evergo_zone_map() is deliberately
	 * *not* released here.  early_iounmap() is only valid until paging_init()
	 * tears the early fixmap down, and by this point (after bootmem_init())
	 * that window is long gone.  Releasing it earlier is not an option either:
	 * evergo_mark() keeps writing into the zone throughout setup_arch(), and
	 * b75 showed that closing it before paging_init() cuts the mark stream off
	 * early (the kernel then appeared to die right after EVB62:M0-dt-ok with
	 * only 4 marks recorded).  The fixmap slot therefore leaks, which is what
	 * check_early_ioremap_leak() reports; that warning is cosmetic.  Doing it
	 * properly means re-mapping the zone with a real ioremap() after
	 * paging_init(), the way evergo_sram already does.
	 */
	evergo_zone = NULL;	/* pstore owns the area from here on */
}

void evergo_mark(const char *text)
{
	u32 *hdr;
	u8 *zone;
	size_t n = strlen(text);

	pr_info("%s", text);
	evergo_code++;
	evergo_tick(0xb0 + (evergo_code > 0xf ? 0xf : evergo_code));
	evergo_log(text);
	evergo_sram_step(0xb0 + (evergo_code > 0xf ? 0xf : evergo_code));
	evergo_exp_type(evergo_code);
	if (!evergo_zone || evergo_off + n > EVERGO_MARK_LIMIT)
		return;
	zone = (u8 *)evergo_zone;
	hdr = (u32 *)zone;
	hdr[0] = EVERGO_ZONE_SIG;
	hdr[1] = 0;
	hdr[2] = evergo_off - 12 + n;
	memcpy(zone + evergo_off, text, n);
	evergo_off += n;
	/* a watchdog reset does not flush the caches, so push it out now */
	dcache_clean_poc((unsigned long)zone, (unsigned long)zone + evergo_off);
}
#endif

void __init __no_sanitize_address setup_arch(char **cmdline_p)
{
	setup_initial_init_mm(_text, _etext, _edata, _end);

	*cmdline_p = boot_command_line;

	kaslr_init();

	early_fixmap_init();
	early_ioremap_init();

#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	evergo_zone_map();
	evergo_mark("EVB62:SA1-enter\n");
#endif /* CONFIG_MT6833_EVERGO_FORCE_DT */

/*
 * Deliberately independent of CONFIG_MT6833_EVERGO_FORCE_DT: this is a
 * functional fix for this board (recovering from a hang that happens before
 * userspace is up), not a bring-up experiment, and it has to stay in place even
 * if the embedded device tree ends up being used some other way.
 */
#ifdef CONFIG_MT6833_EVERGO_WDT
	/*
	 * The preloader/LK hand us an *armed* top reset generator watchdog with a
	 * short timeout, so it fires in the middle of the boot.  This block used
	 * to disarm it (the way mtk_wdt_stop() does).  That removed the spurious
	 * resets, but it also left the board with no way out of a hang that
	 * happens before userspace is up: b73 died exactly like that -- the
	 * machine just sat there and had to be force-powered-off, and a force
	 * power-off risks dropping DRAM and with it the log ring before the next
	 * boot can flush it into expdb.
	 *
	 * So keep it armed instead, reloaded to the longest timeout the hardware
	 * offers (31 s, WDT_MAX_TIMEOUT in drivers/watchdog/mtk_wdt.c) with the
	 * system reset action enabled.  mtk_wdt_probe() -> mtk_wdt_init() sees
	 * WDT_MODE_EN set, marks the device WDOG_HW_RUNNING, and because
	 * CONFIG_WATCHDOG_HANDLE_BOOT_ENABLED=y the watchdog core then keeps
	 * pinging it by itself until init opens /dev/watchdog right after the
	 * banner.  A normal boot is therefore unaffected, but a hang now resets
	 * the SoC on its own -- and a watchdog reset does not cut DRAM power the
	 * way a long-press power-off can, so the log survives.
	 *
	 * Register layout mirrors the driver:
	 *   WDT_MODE   0x00  bit0 EN, bit2 EXRST_EN, bit3 IRQ_EN, bit6 DUAL_EN,
	 *                    key 0x22 in bits[31:24]
	 *   WDT_LENGTH 0x04  key 0x8 in bits[3:0], seconds in bits[7:5]
	 *   WDT_RST    0x08  write 0x1971 to reload
	 */
	{
		void __iomem *wdt = early_ioremap(0x10007000, 0x1000);

		if (wdt) {
			u32 mode = readl(wdt);		/* WDT_MODE */
			u32 len = readl(wdt + 0x04);	/* WDT_LENGTH */

			/*
			 * Field layout (drivers/watchdog/mtk_wdt.c): seconds in
			 * bits[15:11], pretimeout in bits[10:6], key 0x8 in
			 * bits[3:0].  The driver writes
			 *
			 *   WDT_LENGTH_TIMEOUT((timeout - pretimeout) << 6)
			 *                          -- and WDT_LENGTH_TIMEOUT(n) is
			 *                          n << 5 --
			 *
			 * i.e. the seconds field really does live at bit 11.
			 * "31 << 5" would only poke the pretimeout field and leave
			 * the preloader's short timeout in place, which is exactly
			 * what kept resetting the board at ~0.31 s: the reset landed
			 * inside a different core_initcall on every build, but always
			 * at the same wall-clock time, and LK reported
			 * "wdt_status 0x1 / detect abnormal boot" afterwards.
			 */
			/*
			 * The encoding is ambiguous in this tree: the driver computes
			 * ((timeout - pretimeout) << 6) << 5, i.e. seconds in
			 * bits[15:11], but the preloader's own value reads back as
			 * 0x3e0 = 31 << 5, as if the seconds field were bits[9:5].
			 * Set both fields so the timeout is 31 s under either
			 * reading; bits[10:6] then hold a pretimeout, which the mode
			 * write below disables anyway (no IRQ_EN, no DUAL_EN).
			 */
			writel((31 << 11) | (31 << 5) | 0x8, wdt + 0x04);
			writel((mode & ~((1 << 3) | (1 << 6))) | (1 << 0) | (1 << 2) |
			       0x22000000, wdt);	/* EN|EXRST_EN|key */
			writel(0x1971, wdt + 0x08);	/* WDT_RST reload */
			pr_info("evergo: wdt mode %#x -> %#x len %#x -> %#x (%u s)\n",
				mode, readl(wdt), len, readl(wdt + 0x04),
				(readl(wdt + 0x04) >> 11) & 0x1f);
			early_iounmap(wdt, 0x1000);
		} else {
			pr_warn("evergo: could not map the reset generator\n");
		}
	}
#endif /* CONFIG_MT6833_EVERGO_WDT */

#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	evergo_mark("EVB62:SA2-wdt-armed\n");

	/*
	 * Capture the clock controls that decide whether the PMIC wrapper is
	 * reachable, before any clk driver has run: LK talks to the same window
	 * happily and the kernel then reads all zeroes out of it, so the
	 * question is who leaves it dead.  evergo_clk_capture() only records;
	 * evergo_clk_report() prints it once the ring console is up.
	 */
	evergo_clk_capture();
	{
		char b[160];

		snprintf(b, sizeof(b),
			 "EVB65:SNAP top[90]=%08x inf[90]=%08x pw=%08x spm=%08x\n",
			 evergo_clk_snap[0], evergo_clk_snap[6],
			 evergo_clk_snap[7], evergo_clk_snap[10]);
		evergo_mark(b);
	}

	/*
	 * Bring-up aid: the LK of the evergo composes the device tree itself
	 * (its display/lcm code needs the vendor panel data), so the one it
	 * hands over is not the one we want. Use the DT embedded through
	 * CONFIG_EXTRA_FIRMWARE instead; it is found without any allocation,
	 * which is what firmware_request_builtin() is meant for.
	 */
	{
		struct firmware fw;

		if (firmware_request_builtin(&fw, "mt6833-xiaomi-evergo.dtb")) {
			__fdt_pointer = __pa(fw.data);
			pr_info("evergo: using embedded device tree (%zu bytes)\n",
				fw.size);
		} else {
			pr_warn("evergo: embedded device tree missing, keeping the bootloader one\n");
		}
	}
#endif

	setup_machine_fdt(__fdt_pointer);
	evergo_mark("EVB62:M0-dt-ok\n");

	/*
	 * Initialise the static keys early as they may be enabled by the
	 * cpufeature code and early parameters.
	 */
	jump_label_init();
	evergo_mark("EVM:SA2-jumplabel\n");
	parse_early_param();
	evergo_mark("EVM:SA2-earlyparam\n");

	dynamic_scs_init();
	evergo_mark("EVM:SA2-scs\n");

	/*
	 * The primary CPU enters the kernel with all DAIF exceptions masked.
	 *
	 * We must unmask Debug and SError before preemption or scheduling is
	 * possible to ensure that these are consistently unmasked across
	 * threads, and we want to unmask SError as soon as possible after
	 * initializing earlycon so that we can report any SErrors immediately.
	 *
	 * IRQ and FIQ will be unmasked after the root irqchip has been
	 * detected and initialized.
	 */
	local_daif_restore(DAIF_PROCCTX_NOIRQ);
	evergo_mark("EVM:SA2-daif\n");

	/*
	 * TTBR0 is only used by the identity mapping at this stage. Make it
	 * point to zero page to avoid speculatively fetching new entries.
	 */
	cpu_uninstall_idmap();
	evergo_mark("EVM:SA2-idmap\n");

	xen_early_init();
	efi_init();

	if (!efi_enabled(EFI_BOOT)) {
		if ((u64)_text % MIN_KIMG_ALIGN)
			pr_warn(FW_BUG "Kernel image misaligned at boot, please fix your bootloader!");
		WARN_TAINT(mmu_enabled_at_boot, TAINT_FIRMWARE_WORKAROUND,
			   FW_BUG "Booted with MMU enabled!");
	}

	arm64_memblock_init();
	evergo_mark("EVM:SA2-memblock\n");

	/*
	 * paging_init() builds the real page tables.  It was accidentally dropped
	 * here in b74 while the evergo_zone_close() call was being moved, and that
	 * is why b74 and b75 died right after EVB62:M0-dt-ok with only four marks
	 * recorded.  Do not move this.
	 */
	paging_init();
	evergo_mark("EVM:SA2-paging\n");

#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	/* early_memremap()'s fixmap window is gone, use a real mapping now */
	evergo_sram_early = false;
#endif

	acpi_table_upgrade();

	/* Parse the ACPI tables for possible boot-time configuration */
	acpi_boot_table_init();

	if (acpi_disabled)
		unflatten_device_tree();
	evergo_mark("EVM:SA-unflat\n");

	bootmem_init();
	/* from here on pstore/ramoops captures the log in that same area */
	evergo_zone_close();
	evergo_mark("EVM:SA-bootmem\n");

	kasan_init();

	request_standard_resources();

	if (acpi_disabled)
		psci_dt_init();
	else
		psci_acpi_init();
	evergo_mark("EVM:SA-psci\n");

	arm64_rsi_init();

	init_bootcpu_ops();
	smp_init_cpus();
	evergo_mark("EVM:SA-smp-init\n");
	smp_build_mpidr_hash();

#ifdef CONFIG_ARM64_SW_TTBR0_PAN
	/*
	 * Make sure init_thread_info.ttbr0 always generates translation
	 * faults in case uaccess_enable() is inadvertently called by the init
	 * thread.
	 */
	init_task.thread_info.ttbr0 = phys_to_ttbr(__pa_symbol(reserved_pg_dir));
#endif

	if (boot_args[1] || boot_args[2] || boot_args[3]) {
		pr_err("WARNING: x1-x3 nonzero in violation of boot protocol:\n"
			"\tx1: %016llx\n\tx2: %016llx\n\tx3: %016llx\n"
			"This indicates a broken bootloader or old kernel\n",
			boot_args[1], boot_args[2], boot_args[3]);
	}
	evergo_mark("EVM:SA-end\n");
}

static inline bool cpu_can_disable(unsigned int cpu)
{
#ifdef CONFIG_HOTPLUG_CPU
	const struct cpu_operations *ops = get_cpu_ops(cpu);

	if (ops && ops->cpu_can_disable)
		return ops->cpu_can_disable(cpu);
#endif
	return false;
}

bool arch_cpu_is_hotpluggable(int num)
{
	return cpu_can_disable(num);
}

static void dump_kernel_offset(void)
{
	const unsigned long offset = kaslr_offset();

	if (IS_ENABLED(CONFIG_RANDOMIZE_BASE) && offset > 0) {
		pr_emerg("Kernel Offset: 0x%lx from 0x%lx\n",
			 offset, KIMAGE_VADDR);
		pr_emerg("PHYS_OFFSET: 0x%llx\n", PHYS_OFFSET);
	} else {
		pr_emerg("Kernel Offset: disabled\n");
	}
}

static int arm64_panic_block_dump(struct notifier_block *self,
				  unsigned long v, void *p)
{
	dump_kernel_offset();
	dump_cpu_features();
	dump_mem_limit();
	return 0;
}

static struct notifier_block arm64_panic_block = {
	.notifier_call = arm64_panic_block_dump
};

static int __init register_arm64_panic_block(void)
{
	atomic_notifier_chain_register(&panic_notifier_list,
				       &arm64_panic_block);
	return 0;
}
device_initcall(register_arm64_panic_block);

static int __init check_mmu_enabled_at_boot(void)
{
	if (!efi_enabled(EFI_BOOT) && mmu_enabled_at_boot)
		panic("Non-EFI boot detected with MMU and caches enabled");
	return 0;
}
device_initcall_sync(check_mmu_enabled_at_boot);
