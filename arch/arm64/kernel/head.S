/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Low-level CPU initialisation
 * Based on arch/arm/kernel/head.S
 *
 * Copyright (C) 1994-2002 Russell King
 * Copyright (C) 2003-2012 ARM Ltd.
 * Authors:	Catalin Marinas <catalin.marinas@arm.com>
 *		Will Deacon <will.deacon@arm.com>
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <linux/pgtable.h>

#include <asm/asm_pointer_auth.h>
#include <asm/assembler.h>
#include <asm/boot.h>
#include <asm/bug.h>
#include <asm/ptrace.h>
#include <asm/asm-offsets.h>
#include <asm/cache.h>
#include <asm/cputype.h>
#include <asm/el2_setup.h>
#include <asm/elf.h>
#include <asm/image.h>
#include <asm/kernel-pgtable.h>
#include <asm/kvm_arm.h>
#include <asm/memory.h>
#include <asm/pgtable-hwdef.h>
#include <asm/page.h>
#include <asm/scs.h>
#include <asm/smp.h>
#include <asm/sysreg.h>
#include <asm/stacktrace/frame.h>
#include <asm/thread_info.h>
#include <asm/virt.h>

#include "efi-header.S"

#if (PAGE_OFFSET & 0x1fffff) != 0
#error PAGE_OFFSET must be at least 2MB aligned
#endif

/*
 * Kernel startup entry point.
 * ---------------------------
 *
 * The requirements are:
 *   MMU = off, D-cache = off, I-cache = on or off,
 *   x0 = physical address to the FDT blob.
 *
 * Note that the callee-saved registers are used for storing variables
 * that are useful before the MMU is enabled. The allocations are described
 * in the entry routines.
 */
	__HEAD
	/*
	 * DO NOT MODIFY. Image header expected by Linux boot-loaders.
	 */
	efi_signature_nop			// special NOP to identity as PE/COFF executable
	b	primary_entry			// branch to kernel start, magic
	.quad	0				// Image load offset from start of RAM, little-endian
	le64sym	_kernel_size_le			// Effective size of kernel image, little-endian
	le64sym	_kernel_flags_le		// Informative flags, little-endian
	.quad	0				// reserved
	.quad	0				// reserved
	.quad	0				// reserved
	.ascii	ARM64_IMAGE_MAGIC		// Magic number
	.long	.Lpe_header_offset		// Offset to the PE header.

	__EFI_PE_HEADER

	.section ".idmap.text","a"

	/*
	 * The following callee saved general purpose registers are used on the
	 * primary lowlevel boot path:
	 *
	 *  Register   Scope                      Purpose
	 *  x19        primary_entry() .. start_kernel()        whether we entered with the MMU on
	 *  x20        primary_entry() .. __primary_switch()    CPU boot mode
	 *  x21        primary_entry() .. start_kernel()        FDT pointer passed at boot in x0
	 */
#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
/*
 * evergo bring-up probes.
 *
 * The board has no UART and printk only reaches pstore once ramoops has
 * registered, so a kernel that dies early leaves no trace at all - which is
 * indistinguishable from a kernel that never ran.
 *
 * These probes build a "DBGC" record in the DRAM window at 0x48090000, which
 * is the format the MTK LK looks for when it copies that area into the expdb
 * partition (same layout the vendor ram console and evergo_mark() use):
 *
 *      +0x00  magic "DBGC"
 *      +0x04  0
 *      +0x08  length of the payload
 *      +0x0c  payload (ASCII)
 *
 * Each checkpoint stores its own 4-byte tag ("EVM0".."EVM4") at its slot and
 * updates the length, so the highest tag present - and the length - show how
 * far the kernel got.  The MMU is still off, so these are plain physical
 * stores.  Only x4/x5/x6 are clobbered.
 *
 * NOTE: when CONFIG_PSTORE_RAM is enabled the window at 0x48090000 belongs
 * to the pstore/ramoops driver (its persistent_ram header sits at +0), and
 * these stores would overwrite the previous boot's bookkeeping - every
 * replay would then show nothing but the EVM tag bytes.  The probe is
 * compiled out in that configuration; early progress is still readable via
 * evergo_sram() (SRAM fiq_step) and the EVTICK stamps in the LK ring.
 *
 * evergo_sram() is the more reliable half of the pair: it stamps the ram
 * console "fiq_step" field in SRAM (0x11d000 + 0x3c), which the MTK LK prints
 * on every single boot ("RAM_CONSOLE. wdt_status 0x?, fiq_step 0x?, ...").
 * fiq_step is always 0x0 in factory logs, so any non-zero value is proof that
 * this code ran - and because each checkpoint writes a bigger number, the
 * highest value seen tells us exactly where the kernel died.  It is written
 * FIRST, before any DRAM access: a DRAM store can be blocked by the EMI MPU,
 * an SRAM store cannot.
 *
 * Values: 0xa1..0xaf = arch/arm64/kernel/head.S, 0xb1.. = setup.c.
 */
	.macro	evergo_sram, val
	movz	x4, #0xd000
	movk	x4, #0x0011, lsl #16		// x4 = 0x11d000 (ram console, SRAM)
	movz	w5, #\val
	str	w5, [x4, #0x3c]			// fiq_step = progress marker
	mov	w5, #6
	str	w5, [x4, #0x40]			// exp_type = 6 (what LK expects)
	.endm

	.macro	evergo_sram_only, val
	movz	x4, #0xd000
	movk	x4, #0x0011, lsl #16
	movz	w5, #\val
	str	w5, [x4, #0x3c]
	.endm

/*
 * evergo_tick_h - stamp a progress tick into the MTK LK/preloader log ring.
 *
 * LK keeps its own log in a 256 KiB ring at 0x7ffbf000 (its "log_store"
 * mblock, map:1, so the normal world may touch it) and copies that whole
 * buffer into the expdb partition at the end of every session:
 *
 *	LK_LOG_STORE: dram pl/lk log buff mapping start addr = 0x7ffbf000, size = 0x40000
 *	LK_LOG_STORE: start save pllk log
 *
 * The ring is plain DRAM, it accumulates across watchdog resets and LK flushes
 * it on its own - so a stamp left by the kernel is readable afterwards even
 * when nothing else works.  This is the only channel that survives a death in
 * the first instructions, which is exactly where this port was dying.  Stamp
 * every 4 KiB so it lands wherever LK's write pointer happens to be; with the
 * MMU off these are raw physical stores.  The value grows with every
 * checkpoint, so the highest one seen in a read-back is the death point.
 */
	.macro	evergo_tick_h, val
	movz	x4, #0xf000
	movk	x4, #0x7ffb, lsl #16		// x4 = 0x7ffbf000 (LK log ring)
	add	x7, x4, #0x40000		// end of the ring
	movz	x6, #0x5645			// "EV"
	movk	x6, #0x4954, lsl #16		// "TI"
	movk	x6, #0x4b43, lsl #32		// "CK"
	movk	x6, #0x003a, lsl #48		// ":"  -> "EVTICK:"
	movz	w5, #\val
1:	str	x6, [x4]
	str	w5, [x4, #8]
	add	x4, x4, #0x1000
	cmp	x4, x7
	b.lo	1b
	.endm

	.macro	evergo_dbgc, half, idx
#ifndef CONFIG_PSTORE_RAM
	movz	x4, #0x0000
	movk	x4, #0x4809, lsl #16		// x4 = 0x48090000 (pstore window)
	movz	w6, #0x4244
	movk	w6, #0x4347, lsl #16		// "DBGC"
	str	w6, [x4]			// magic
	str	wzr, [x4, #4]
	mov	w6, #(4 * (\idx + 1))
	str	w6, [x4, #8]			// payload length
	movz	w5, #0x5645			// 'E','V'
	movk	w5, #\half, lsl #16		// -> "EVM<n>"
	str	w5, [x4, #(12 + 4 * \idx)]
#endif
	.endm

/*
 * evergo_stamp_h - build-specific stamp in the LK log ring.
 *
 * The "EVTICK:" and "EVM<n>" breadcrumbs are byte-identical in every build,
 * so a read-back of the ring cannot tell *which* image left them - and the
 * ring also still holds the previous session's text, because DRAM is not
 * cleared by a warm reset.  This stamp is unique per build, so its presence
 * in a read-back proves that THIS build's first instruction executed; if it
 * is absent, the image never ran at all (the bootloader did not jump to it).
 *
 * Written at +0x40 inside each 4 KiB ring slot so evergo_tick_h() (offset 0,
 * the same slots) cannot overwrite it.
 */
	.macro	evergo_stamp_h, a, b, c, d
	movz	x4, #0xf000
	movk	x4, #0x7ffb, lsl #16		// x4 = 0x7ffbf000 (LK log ring)
	add	x7, x4, #0x40000		// end of the ring
	movz	x6, #\a
	movk	x6, #\b, lsl #16
	movk	x6, #\c, lsl #32
	movk	x6, #\d, lsl #48
2:	str	x6, [x4, #0x40]
	add	x4, x4, #0x1000
	cmp	x4, x7
	b.lo	2b
	/* same tag in the DBGC/pstore window, a different physical area */
	movz	x4, #0x0000
	movk	x4, #0x4809, lsl #16
	str	x6, [x4, #0x100]
	.endm
#endif

SYM_CODE_START(primary_entry)
#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	evergo_stamp_h 0x5645, 0x5245, 0x4f47, 0x3737	// "EVERGO77"
	evergo_sram 0xa1			// SRAM first, it cannot fault on a mapping
	evergo_tick_h 0xa1			// and the LK log ring, the channel that works
	evergo_dbgc 0x304d, 0			// EVM0
#endif
	bl	record_mmu_state
	bl	preserve_boot_args

	adrp	x1, early_init_stack
	mov	sp, x1
	mov	x29, xzr
	adrp	x0, __pi_init_idmap_pg_dir
	mov	x1, xzr
	bl	__pi_create_init_idmap
#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	evergo_tick_h 0xa2
	evergo_dbgc 0x314d, 1			// EVM1: idmap page tables built
#endif

	/*
	 * If the page tables have been populated with non-cacheable
	 * accesses (MMU disabled), invalidate those tables again to
	 * remove any speculatively loaded cache lines.
	 */
	cbnz	x19, 0f
	dmb     sy
	mov	x1, x0				// end of used region
	adrp    x0, __pi_init_idmap_pg_dir
	adr_l	x2, dcache_inval_poc
	blr	x2
	b	1f

	/*
	 * If we entered with the MMU and caches on, clean the ID mapped part
	 * of the primary boot code to the PoC so we can safely execute it with
	 * the MMU off.
	 */
0:	adrp	x0, __idmap_text_start
	adr_l	x1, __idmap_text_end
	adr_l	x2, dcache_clean_poc
	blr	x2

1:	mov	x0, x19
#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	evergo_tick_h 0xa3
	evergo_dbgc 0x324d, 2			// EVM2: entering init_kernel_el
#endif
	bl	init_kernel_el			// w0=cpu_boot_mode
	mov	x20, x0
#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	evergo_tick_h 0xa4
	evergo_dbgc 0x334d, 3			// EVM3: init_kernel_el returned
#endif

	/*
	 * The following calls CPU setup code, see arch/arm64/mm/proc.S for
	 * details.
	 * On return, the CPU will be ready for the MMU to be turned on and
	 * the TCR will have been set.
	 */
	bl	__cpu_setup			// initialise processor
#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
	evergo_tick_h 0xa5
	evergo_dbgc 0x344d, 4			// EVM4: __cpu_setup done, about to enable MMU
#endif
	b	__primary_switch
SYM_CODE_END(primary_entry)

	__INIT
SYM_CODE_START_LOCAL(record_mmu_state)
	mrs	x19, CurrentEL
	cmp	x19, #CurrentEL_EL2
	mrs	x19, sctlr_el1
	b.ne	0f
	mrs	x19, sctlr_el2
0:
CPU_LE( tbnz	x19, #SCTLR_ELx_EE_SHIFT, 1f	)
CPU_BE( tbz	x19, #SCTLR_ELx_EE_SHIFT, 1f	)
	tst	x19, #SCTLR_ELx_C		// Z := (C == 0)
	and	x19, x19, #SCTLR_ELx_M		// isolate M bit
	csel	x19, xzr, x19, eq		// clear x19 if Z
	ret

	/*
	 * Set the correct endianness early so all memory accesses issued
	 * before init_kernel_el() occur in the correct byte order. Note that
	 * this means the MMU must be disabled, or the active ID map will end
	 * up getting interpreted with the wrong byte order.
	 */
1:	eor	x19, x19, #SCTLR_ELx_EE
	bic	x19, x19, #SCTLR_ELx_M
	b.ne	2f
	pre_disable_mmu_workaround
	msr	sctlr_el2, x19
	b	3f
2:	pre_disable_mmu_workaround
	msr	sctlr_el1, x19
3:	isb
	mov	x19, xzr
	ret
SYM_CODE_END(record_mmu_state)

/*
 * Preserve the arguments passed by the bootloader in x0 .. x3
 */
SYM_CODE_START_LOCAL(preserve_boot_args)
	mov	x21, x0				// x21=FDT

	adr_l	x0, boot_args			// record the contents of
	stp	x21, x1, [x0]			// x0 .. x3 at kernel entry
	stp	x2, x3, [x0, #16]

	cbnz	x19, 0f				// skip cache invalidation if MMU is on
	dmb	sy				// needed before dc ivac with
						// MMU off

	add	x1, x0, #0x20			// 4 x 8 bytes
	b	dcache_inval_poc		// tail call
0:	str_l   x19, mmu_enabled_at_boot, x0
	ret
SYM_CODE_END(preserve_boot_args)

	/*
	 * Initialize CPU registers with task-specific and cpu-specific context.
	 *
	 * Create a final frame record at task_pt_regs(current)->stackframe, so
	 * that the unwinder can identify the final frame record of any task by
	 * its location in the task stack. We reserve the entire pt_regs space
	 * for consistency with user tasks and kthreads.
	 */
	.macro	init_cpu_task tsk, tmp1, tmp2
	msr	sp_el0, \tsk

	ldr	\tmp1, [\tsk, #TSK_STACK]
	add	sp, \tmp1, #THREAD_SIZE
	sub	sp, sp, #PT_REGS_SIZE

	stp	xzr, xzr, [sp, #S_STACKFRAME]
	mov	\tmp1, #FRAME_META_TYPE_FINAL
	str	\tmp1, [sp, #S_STACKFRAME_TYPE]
	add	x29, sp, #S_STACKFRAME

	scs_load_current

	adr_l	\tmp1, __per_cpu_offset
	ldr	w\tmp2, [\tsk, #TSK_TI_CPU]
	ldr	\tmp1, [\tmp1, \tmp2, lsl #3]
	set_this_cpu_offset \tmp1
	.endm

/*
 * The following fragment of code is executed with the MMU enabled.
 *
 *   x0 = __pa(KERNEL_START)
 */
SYM_FUNC_START_LOCAL(__primary_switched)
	adr_l	x4, init_task
	init_cpu_task x4, x5, x6

	adr_l	x8, vectors			// load VBAR_EL1 with virtual
	msr	vbar_el1, x8			// vector table address
	isb

	stp	x29, x30, [sp, #-16]!
	mov	x29, sp

	str_l	x21, __fdt_pointer, x5		// Save FDT pointer

	adrp	x4, _text			// Save the offset between
	sub	x4, x4, x0			// the kernel virtual and
	str_l	x4, kimage_voffset, x5		// physical mappings

	mov	x0, x20
	bl	set_cpu_boot_mode_flag

#if defined(CONFIG_KASAN_GENERIC) || defined(CONFIG_KASAN_SW_TAGS)
	bl	kasan_early_init
#endif
	mov	x0, x20
	bl	finalise_el2			// Prefer VHE if possible
	ldp	x29, x30, [sp], #16
	bl	start_kernel
	ASM_BUG()
SYM_FUNC_END(__primary_switched)

/*
 * end early head section, begin head code that is also used for
 * hotplug and needs to have the same protections as the text region
 */
	.section ".idmap.text","a"

/*
 * Starting from EL2 or EL1, configure the CPU to execute at the highest
 * reachable EL supported by the kernel in a chosen default state. If dropping
 * from EL2 to EL1, configure EL2 before configuring EL1.
 *
 * Since we cannot always rely on ERET synchronizing writes to sysregs (e.g. if
 * SCTLR_ELx.EOS is clear), we place an ISB prior to ERET.
 *
 * Returns either BOOT_CPU_MODE_EL1 or BOOT_CPU_MODE_EL2 in x0 if
 * booted in EL1 or EL2 respectively, with the top 32 bits containing
 * potential context flags. These flags are *not* stored in __boot_cpu_mode.
 *
 * x0: whether we are being called from the primary boot path with the MMU on
 */
SYM_FUNC_START(init_kernel_el)
	mrs	x1, CurrentEL
	cmp	x1, #CurrentEL_EL2
	b.eq	init_el2

SYM_INNER_LABEL(init_el1, SYM_L_LOCAL)
	mov_q	x0, INIT_SCTLR_EL1_MMU_OFF
	pre_disable_mmu_workaround
	msr	sctlr_el1, x0
	isb
	mov_q	x0, INIT_PSTATE_EL1
	msr	spsr_el1, x0
	msr	elr_el1, lr
	mov	w0, #BOOT_CPU_MODE_EL1
	eret

SYM_INNER_LABEL(init_el2, SYM_L_LOCAL)
	msr	elr_el2, lr

	// clean all HYP code to the PoC if we booted at EL2 with the MMU on
	cbz	x0, 0f
	adrp	x0, __hyp_idmap_text_start
	adr_l	x1, __hyp_text_end
	adr_l	x2, dcache_clean_poc
	blr	x2

	mov_q	x0, INIT_SCTLR_EL2_MMU_OFF
	pre_disable_mmu_workaround
	msr	sctlr_el2, x0
	isb
0:

	init_el2_hcr	HCR_HOST_NVHE_FLAGS | HCR_ATA
	init_el2_state

	/* Hypervisor stub */
	adr_l	x0, __hyp_stub_vectors
	msr	vbar_el2, x0
	isb

	mov_q	x1, INIT_SCTLR_EL1_MMU_OFF

	mrs	x0, hcr_el2
	and	x0, x0, #HCR_E2H
	cbz	x0, 2f

	/* Set a sane SCTLR_EL1, the VHE way */
	msr_s	SYS_SCTLR_EL12, x1
	mov	x2, #BOOT_CPU_FLAG_E2H
	b	3f

2:
	msr	sctlr_el1, x1
	mov	x2, xzr
3:
	mov	x0, #INIT_PSTATE_EL1
	msr	spsr_el2, x0

	mov	w0, #BOOT_CPU_MODE_EL2
	orr	x0, x0, x2
	eret
SYM_FUNC_END(init_kernel_el)

	/*
	 * This provides a "holding pen" for platforms to hold all secondary
	 * cores are held until we're ready for them to initialise.
	 */
SYM_FUNC_START(secondary_holding_pen)
	mov	x0, xzr
	bl	init_kernel_el			// w0=cpu_boot_mode
	mrs	x2, mpidr_el1
	mov_q	x1, MPIDR_HWID_BITMASK
	and	x2, x2, x1
	adr_l	x3, secondary_holding_pen_release
pen:	ldr	x4, [x3]
	cmp	x4, x2
	b.eq	secondary_startup
	wfe
	b	pen
SYM_FUNC_END(secondary_holding_pen)

	/*
	 * Secondary entry point that jumps straight into the kernel. Only to
	 * be used where CPUs are brought online dynamically by the kernel.
	 */
SYM_FUNC_START(secondary_entry)
	mov	x0, xzr
	bl	init_kernel_el			// w0=cpu_boot_mode
	b	secondary_startup
SYM_FUNC_END(secondary_entry)

SYM_FUNC_START_LOCAL(secondary_startup)
	/*
	 * Common entry point for secondary CPUs.
	 */
	mov	x20, x0				// preserve boot mode

#ifdef CONFIG_ARM64_VA_BITS_52
alternative_if ARM64_HAS_VA52
	bl	__cpu_secondary_check52bitva
alternative_else_nop_endif
#endif

	bl	__cpu_setup			// initialise processor
	adrp	x1, swapper_pg_dir
	adrp	x2, idmap_pg_dir
	bl	__enable_mmu
	ldr	x8, =__secondary_switched
	br	x8
SYM_FUNC_END(secondary_startup)

	.text
SYM_FUNC_START_LOCAL(__secondary_switched)
	mov	x0, x20
	bl	set_cpu_boot_mode_flag

	mov	x0, x20
	bl	finalise_el2

	str_l	xzr, __early_cpu_boot_status, x3
	adr_l	x5, vectors
	msr	vbar_el1, x5
	isb

	adr_l	x0, secondary_data
	ldr	x2, [x0, #CPU_BOOT_TASK]
	cbz	x2, __secondary_too_slow

	init_cpu_task x2, x1, x3

#ifdef CONFIG_ARM64_PTR_AUTH
	ptrauth_keys_init_cpu x2, x3, x4, x5
#endif

	bl	secondary_start_kernel
	ASM_BUG()
SYM_FUNC_END(__secondary_switched)

SYM_FUNC_START_LOCAL(__secondary_too_slow)
	wfe
	wfi
	b	__secondary_too_slow
SYM_FUNC_END(__secondary_too_slow)

/*
 * Sets the __boot_cpu_mode flag depending on the CPU boot mode passed
 * in w0. See arch/arm64/include/asm/virt.h for more info.
 */
SYM_FUNC_START_LOCAL(set_cpu_boot_mode_flag)
	adr_l	x1, __boot_cpu_mode
	cmp	w0, #BOOT_CPU_MODE_EL2
	b.ne	1f
	add	x1, x1, #4
1:	str	w0, [x1]			// Save CPU boot mode
	ret
SYM_FUNC_END(set_cpu_boot_mode_flag)

/*
 * The booting CPU updates the failed status @__early_cpu_boot_status,
 * with MMU turned off.
 *
 * update_early_cpu_boot_status tmp, status
 *  - Corrupts tmp1, tmp2
 *  - Writes 'status' to __early_cpu_boot_status and makes sure
 *    it is committed to memory.
 */

	.macro	update_early_cpu_boot_status status, tmp1, tmp2
	mov	\tmp2, #\status
	adr_l	\tmp1, __early_cpu_boot_status
	str	\tmp2, [\tmp1]
	dmb	sy
	dc	ivac, \tmp1			// Invalidate potentially stale cache line
	.endm

/*
 * Enable the MMU.
 *
 *  x0  = SCTLR_EL1 value for turning on the MMU.
 *  x1  = TTBR1_EL1 value
 *  x2  = ID map root table address
 *
 * Returns to the caller via x30/lr. This requires the caller to be covered
 * by the .idmap.text section.
 *
 * Checks if the selected granule size is supported by the CPU.
 * If it isn't, park the CPU
 */
	.section ".idmap.text","a"
SYM_FUNC_START(__enable_mmu)
	mrs	x3, ID_AA64MMFR0_EL1
	ubfx	x3, x3, #ID_AA64MMFR0_EL1_TGRAN_SHIFT, 4
	cmp     x3, #ID_AA64MMFR0_EL1_TGRAN_SUPPORTED_MIN
	b.lt    __no_granule_support
	cmp     x3, #ID_AA64MMFR0_EL1_TGRAN_SUPPORTED_MAX
	b.gt    __no_granule_support
	phys_to_ttbr x2, x2
	msr	ttbr0_el1, x2			// load TTBR0
	load_ttbr1 x1, x1, x3

	set_sctlr_el1	x0

	ret
SYM_FUNC_END(__enable_mmu)

#ifdef CONFIG_ARM64_VA_BITS_52
SYM_FUNC_START(__cpu_secondary_check52bitva)
#ifndef CONFIG_ARM64_LPA2
	mrs_s	x0, SYS_ID_AA64MMFR2_EL1
	and	x0, x0, ID_AA64MMFR2_EL1_VARange_MASK
	cbnz	x0, 2f
#else
	mrs	x0, id_aa64mmfr0_el1
	sbfx	x0, x0, #ID_AA64MMFR0_EL1_TGRAN_SHIFT, 4
	cmp	x0, #ID_AA64MMFR0_EL1_TGRAN_LPA2
	b.ge	2f
#endif

	update_early_cpu_boot_status \
		CPU_STUCK_IN_KERNEL | CPU_STUCK_REASON_52_BIT_VA, x0, x1
1:	wfe
	wfi
	b	1b

2:	ret
SYM_FUNC_END(__cpu_secondary_check52bitva)
#endif

SYM_FUNC_START_LOCAL(__no_granule_support)
	/* Indicate that this CPU can't boot and is stuck in the kernel */
	update_early_cpu_boot_status \
		CPU_STUCK_IN_KERNEL | CPU_STUCK_REASON_NO_GRAN, x1, x2
1:
	wfe
	wfi
	b	1b
SYM_FUNC_END(__no_granule_support)

SYM_FUNC_START_LOCAL(__primary_switch)
	adrp	x1, reserved_pg_dir
	adrp	x2, __pi_init_idmap_pg_dir
	bl	__enable_mmu

	adrp	x1, early_init_stack
	mov	sp, x1
	mov	x29, xzr
	mov	x0, x20				// pass the full boot status
	mov	x1, x21				// pass the FDT
	bl	__pi_early_map_kernel		// Map and relocate the kernel

	ldr	x8, =__primary_switched
	adrp	x0, KERNEL_START		// __pa(KERNEL_START)
	br	x8
SYM_FUNC_END(__primary_switch)
