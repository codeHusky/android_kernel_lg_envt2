obj-y += dal.o
obj-y += q6audio.o
# LGE_CHANGE_S [eklee.lee@lge.com] 2010-03-19, LGE_AUDIO_SUBSYSTEM
ifeq (y,$(CONFIG_LGE_AUDIO_AMP_LM49250))
obj-y += analog_audio_lm49250.o
else
obj-y += analog_audio.o
endif
obj-$(CONFIG_LGE_AUDIO_AMP_LM49250) += lm49250_audio_amp.o
# LGE_CHANGE_E [eklee.lee@lge.com] 2010-03-19, LGE_AUDIO_SUBSYSTEM
obj-y += pcm_out.o
obj-y += pcm_in.o
obj-y += aac_in.o
obj-y += qcelp_in.o
obj-y += evrc_in.o
obj-y += amrnb_in.o
obj-y += mp3.o
obj-y += dtmf.o
obj-y += routing.o
obj-y += audio_ctl.o
obj-y += msm_q6vdec.o
obj-y += msm_q6venc.o
obj-y += dsp_debug.o
obj-$(CONFIG_QSD_AUDIO) += audiov2/

# LGE_DOM_UPDATE_S jin333.kim@lge.com 2010/02/20 MVS_PKT_LOOPBACK {
obj-$(CONFIG_LGE_AUDIO_MVS_PKT_LOOPBACK) += lge_mvs_pkt_loopback_clnt.o
# LGE_DOM_UPDATE_E jin333.kim@lge.com 2010/02/20 }
