#
# Makefile for the Novatek NT36672C touchscreen driver.
#

obj-$(CONFIG_TOUCHSCREEN_NOVATEK_NVT36672C) += novatek-nvt36672c.o

# ext_proc (Android debug) and mp_ctrlram (factory test) are disabled for
# bring-up -- the core + fw-update files are all we need (b248).
novatek-nvt36672c-y := nt36xxx.o nt36xxx_fw_update.o
