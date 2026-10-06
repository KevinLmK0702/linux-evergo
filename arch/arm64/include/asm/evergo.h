/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Bring-up breadcrumbs for the Xiaomi Redmi Note 11 5G (MT6833 "evergo").
 *
 * evergo_mark() drops a short text marker into the pstore console area of
 * DRAM. The MTK LK copies that area into the expdb partition on the next boot,
 * so a boot that dies before the ramoops driver has registered (and therefore
 * before pstore can capture anything) is still diagnosable: read the expdb
 * partition and look for the last "EVM:" line.
 *
 * Only available when the bring-up option is enabled; otherwise this is a
 * no-op so the calls can stay in the generic code paths.
 */
#ifndef __ASM_EVERGO_H
#define __ASM_EVERGO_H

#ifdef CONFIG_MT6833_EVERGO_FORCE_DT
void evergo_mark(const char *text);
void __init evergo_zone_map(void);
void evergo_zone_close(void);
void __init evergo_console_init(void);
#else
static inline void evergo_mark(const char *text) { }
static inline void evergo_zone_map(void) { }
static inline void evergo_zone_close(void) { }
static inline void evergo_console_init(void) { }
#endif

#endif /* __ASM_EVERGO_H */
