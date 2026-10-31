# SPDX-License-Identifier: GPL-2.0
#
# Vendor "connectivity" tree: WLAN only.
#
# The gen4m driver ships a vendor-style Makefile that is driven by plain
# make variables instead of Kconfig symbols, and it picks them up from the
# *environment* of the nested scripts/Makefile.build invocation.  They are
# exported below, mirroring what the vendor's own connectivity/Makefile
# does -- with the Android-specific parts switched off.
#
ifneq ($(CONFIG_MTK_GEN4M_WIFI),)

# --- gen4m configuration ---------------------------------------------
# MT6833 integrates MT6631, which belongs to the "SOC2_1X1" chip family
# and is reachable over the internal AXI bus.
export CONFIG_MTK_COMBO_WIFI_HIF=axi
export WLAN_CHIP_ID=6833
export MTK_COMBO_CHIP=SOC2_1X1

# --- Android glue: ON, matched to the working a22x bring-up ----------
# MTK_ANDROID_WMT makes gen4m register hifAxiProbe() as a WMT callback
# instead of calling it directly from mtk_axi_probe(), so CONN power-up,
# ROM-patch download and the STP handshake are owned by common/ (wmt_drv)
# rather than by hand-written platform code.  MTK_ANDROID_EMI routes the
# firmware sections through the EMI window that mtk_wcn_consys_hw.c
# allocates (ALLOCATE_CONNSYS_EMI_FROM_KO).
#
# The WLAN service layer is an Android HAL/test-engine helper whose sources
# are not part of this tree, so it stays off.
export MTK_ANDROID_WMT=y
export MTK_ANDROID_EMI=y
export MTK_WLAN_SERVICE=n
export WIFI_IP_SET=1

# The vendor stack was written for a 4.19 vendor kernel and trips a long
# list of warnings that mainline only grew later.  Purely stylistic ones are
# silenced here.  Note that -Wno-error would NOT help: gcc's -Werror=foo is a
# per-warning escalation that the global -Wno-error does not undo.
#
# Deliberately NOT silenced: -Wimplicit-function-declaration and
# -Wincompatible-pointer-types.  Those are real defects from the 4.19 ->
# 7.3 transition (missing includes, changed prototypes); letting them slide
# would truncate pointers to int at runtime instead of failing the build.
# 4.14 -> mainline glue.  mtk_compat.h is force-included so the vendor
# sources keep seeing timeval, strncpy, del_timer, sched_setscheduler and
# the other helpers mainline removed, without patching every file.
subdir-ccflags-y += -I$(src)/compat/include
subdir-ccflags-y += -include $(src)/compat/include/mtk_compat.h
# wmt_exp.h and the STP/WMT client interface live with the WMT core; the
# /dev/wmtWifi driver and the BT HCI transport both include them directly.
subdir-ccflags-y += -I$(src)/common/common_main/include
subdir-ccflags-y += -I$(src)/common/common_main/linux/include
subdir-ccflags-y += -DMTK_WCN_WMT_STP_EXP_SYMBOL_ABSTRACT
subdir-ccflags-y += -DBTIF_DBG_SUPPORT=0

subdir-ccflags-y += -Wno-missing-prototypes
subdir-ccflags-y += -Wno-missing-declarations
subdir-ccflags-y += -Wno-implicit-fallthrough
subdir-ccflags-y += -Wno-misleading-indentation
# KAL_WAKE_LOCK/UNLOCK expand to nothing when wakeup-source support is off,
# leaving "if (cond) ;" behind.
subdir-ccflags-y += -Wno-empty-body
# ASSERT(list) on an array name: the vendor uses it as a non-NULL check.
subdir-ccflags-y += -Wno-address
# This driver is built into vmlinux, not a module.  The gen4m Makefile keys
# off this flag to set CFG_BUILT_IN_DRIVER, which several places need (e.g.
# is_world_regdom() is already provided by net/wireless/reg.c).
subdir-ccflags-y += -D MTK_WCN_REMOVE_KERNEL_MODULE

# Link order matters: BTIF provides the STP transport, the WMT core sits on
# top of it, and the /dev/wmtWifi character device plus gen4m are WMT clients
# that must find a live WMT core at their own initcall.
obj-y += btif/
obj-y += common/
obj-y += wlan/
obj-y += bt/hci/

endif
