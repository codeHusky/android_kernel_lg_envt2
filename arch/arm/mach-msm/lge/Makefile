obj-$(CONFIG_I2C_GPIO) += board-$(TARGET_PRODUCT)-gpio-i2c.o
obj-$(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA) += board-$(TARGET_PRODUCT)-slcd-aat2870.o
obj-$(CONFIG_LGE_FB_MSM_MDDI_LGIT_WVGA) += board-$(TARGET_PRODUCT)-mlcd-aat2870.o
#obj-$(CONFIG_MACH_ENVT2) += board-$(TARGET_PRODUCT)-mddi-lgit.o
#obj-$(CONFIG_MACH_ENVT2) += board-$(TARGET_PRODUCT)-ami602.o
#obj-$(CONFIG_MACH_ENVT2) += board-$(TARGET_PRODUCT)-gp2ap.o

#obj-$(CONFIG_MACH_ENVT2) += board-$(TARGET_PRODUCT)-ec-ak8973.o
#obj-$(CONFIG_MACH_ENVT2) += board-$(TARGET_PRODUCT)-ac-kr3dh.o
obj-$(CONFIG_LGE_DIAG_UTS_SCREEN_SHOT) += lg_diag_screen_shot.o
#LG_FW_USE_OEM_RAPI
obj-$(CONFIG_LGE_USE_OEM_RAPI) += LG_rapi_client.o
#LG_FW_DIAG_KERNEL_SERVICE add lg_diag_testmode.o
obj-$(CONFIG_LGE_DIAG_TESTMODE) += lg_diag_testmode.o
obj-$(CONFIG_LGE_AUDIO_TESTMODE) += lg_diag_audiotest.o
#LG_FW_AUDIO_TESTMODE, SKW_TEST
obj-$(CONFIG_LGE_DIAG_KERNEL_SERVICE) += lg_diag_class.o
obj-$(CONFIG_LGE_DIAG_KERNEL_SERVICE) += lg_diag_communication.o
#LG_FW_USE_DIAG_KEYPRESS
obj-$(CONFIG_LGE_DIAG_UTS_KEYPRESS) += lg_diag_keypress.o
obj-$(CONFIG_LGE_AUDIO_HEADSET) += lg_gpio_h2w.o
#obj-$(CONFIG_MACH_ENVT2) += lg_fw_pwm_vibrator.o
obj-$(CONFIG_LGE_DIAG_UTS) += lg_diag_udm.o

obj-$(CONFIG_LGE_DIAG_WMC) += lg_diag_wmc.o
obj-$(CONFIG_LGE_ATS_ETA_MTC) += lg_diag_mtc.o
obj-$(CONFIG_LGE_MTD_DIRECT_ACCESS) += lge_mtd_direct_access.o
obj-$(CONFIG_LGE_PROC_COMM) += lge_proc_comm.o

## LG_FW : 2010.01.06 hoonylove004 -------------------------------------------
## Kernel ERS
##if defined (LG_FW_ERS) || defined (FEATURE_LGE_ERS)
#obj-$(CONFIG_ANDROID_RAM_CONSOLE) += board-aloha-ers.o
#obj-$(CONFIG_MACH_ENVT2) += lge_errorhandler.o
##endif /*LG_FW_ERS*/
## ---------------------------------------------------------------------------
obj-$(CONFIG_LGE_ERS) += lge_handle_panic.o
obj-$(CONFIG_LGE_ERS) += lge_ers.o

