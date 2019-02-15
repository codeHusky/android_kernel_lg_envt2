/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/msm_audio.h>

#include <mach/msm_qdsp6_audio.h>
#include <mach/debug_mm.h>

#define BUFSZ (0)

static DEFINE_MUTEX(voice_lock);
static int voice_started;

static struct audio_client *voc_tx_clnt;
static struct audio_client *voc_rx_clnt;

// LGE_DOM_UPDATE_S jin333.kim@lge.com 2010/02/20 {
#define MVS_PKT_LOOPBACK
#ifdef MVS_PKT_LOOPBACK
#include "lge_mvs_pkt_loopback_clnt.h"
#include <mach/pmic.h>
#include <linux/workqueue.h>



static int ftm_audio_stop_mvs_loopback(void);
static int ftm_audio_start_mvs_loopback(void);


u8 loopback_falg = true;

struct ftm_audio_state_struct {
  struct {
	bool 			  mvs_active;
	mvs_mode_type		  mvs_mode;
	u8				  lb_pktbuf[MVS_LB_PKT_DEPTH][320];
	u32				  lb_pktsize[MVS_LB_PKT_DEPTH];
	mvs_frame_info_type   lb_pktinfo[MVS_LB_PKT_DEPTH];
	u32				  rd_index;
	u32				  wr_index;
  } mvs;
} ftm_audio_state;
#endif
// LGE_DOM_UPDATE_E jin333.kim@lge.com 2010/02/20 }
static int q6_voice_start(void)
{
	int rc = 0;

	mutex_lock(&voice_lock);

	if (voice_started) {
		pr_err("[%s:%s] busy\n", __MM_FILE__, __func__);
		rc = -EBUSY;
		goto done;
	}

	voc_tx_clnt = q6voice_open(AUDIO_FLAG_WRITE);
	if (!voc_tx_clnt) {
		pr_err("[%s:%s] open voice tx failed.\n", __MM_FILE__,
				__func__);
		rc = -ENOMEM;
		goto done;
	}

	voc_rx_clnt = q6voice_open(AUDIO_FLAG_READ);
	if (!voc_rx_clnt) {
		pr_err("[%s:%s] open voice rx failed.\n", __MM_FILE__,
				__func__);
		q6voice_close(voc_tx_clnt);
		rc = -ENOMEM;
	}

	voice_started = 1;
done:
	mutex_unlock(&voice_lock);
	return rc;
}

static int q6_voice_stop(void)
{
	mutex_lock(&voice_lock);
	pr_info("** q6_voice_stop **\n");
	if (voice_started) {
		q6voice_close(voc_tx_clnt);
		q6voice_close(voc_rx_clnt);
		voice_started = 0;
	}
	mutex_unlock(&voice_lock);
	return 0;
}

static int q6_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int q6_ioctl(struct inode *inode, struct file *file,
		    unsigned int cmd, unsigned long arg)
{
	int rc;
	uint32_t n;
	uint32_t id[2];

	switch (cmd) {
	case AUDIO_SWITCH_DEVICE:
		rc = copy_from_user(&id, (void *)arg, sizeof(id));
		if (!rc) {
			pr_info("AUDIO_SWITCH_DEVICE %x , %x\n",id[0],id[1]);
			rc = q6audio_do_routing(id[0], id[1]);

		}
		break;
	case AUDIO_SET_VOLUME:
		rc = copy_from_user(&n, (void *)arg, sizeof(n));
		if (!rc)
			rc = q6audio_set_rx_volume(n);
		break;
	case AUDIO_SET_MUTE:
		rc = copy_from_user(&n, (void *)arg, sizeof(n));
		if (!rc)
			rc = q6audio_set_tx_mute(n);
		break;
	case AUDIO_UPDATE_ACDB:
		rc = copy_from_user(&id, (void *)arg, sizeof(id));
		if (!rc)
			rc = q6audio_update_acdb(id[0], 0);
		break;
	case AUDIO_START_VOICE:
		rc = q6_voice_start();
		break;
	case AUDIO_STOP_VOICE:
		rc = q6_voice_stop();
		break;
	case AUDIO_REINIT_ACDB:
		rc = 0;
		break;
#ifdef CONFIG_LGE_AUDIO_MVS_PKT_LOOPBACK		
// LGE_DOM_UPDATE_S jin333.kim@lge.com 2010/03/30 {
	case AUDIO_START_LOOPBACK:
		rc = ftm_audio_start_mvs_loopback();
		break;
	case AUDIO_STOP_LOOPBACK:
		rc = ftm_audio_stop_mvs_loopback();
		break;
// LGE_DOM_UPDATE_E jin333.kim@lge.com 2010/03/30 }
#endif 			
	default:
		rc = -EINVAL;
	}

	return rc;
}


static int q6_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations q6_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= q6_open,
	.ioctl		= q6_ioctl,
	.release	= q6_release,
};

struct miscdevice q6_control_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_audio_ctl",
	.fops	= &q6_dev_fops,
};


static int __init q6_audio_ctl_init(void) {
	return misc_register(&q6_control_device);
}

device_initcall(q6_audio_ctl_init);

// LGE_DOM_UPDATE_S jin333.kim@lge.com 2010/02/20  {
#ifdef MVS_PKT_LOOPBACK
/**
  @ingroup ftm_audio_misc_doxy_group
  @brief FTM audio initialization

  Initializes global state variables as well as CAD. This should be called
  before any FTM audio command is processed. Not all, but most commands depend
  on the initializations being performed here.
*/
#if 0
static void ftm_audio_one_time_init()
{
  ftm_audio_state.mvs.mvs_active = false;
  ftm_audio_state.mvs.mvs_mode = MVS_MODE_LINEAR_PCM;
  memset(ftm_audio_state.mvs.lb_pktbuf,0,320*MVS_LB_PKT_DEPTH);
  memset(ftm_audio_state.mvs.lb_pktsize,0,
    sizeof(ftm_audio_state.mvs.lb_pktsize));
  memset(&ftm_audio_state.mvs.lb_pktinfo,0,
    sizeof(ftm_audio_state.mvs.lb_pktinfo));
}
#endif 
/**   
  @brief 


  
  @param activate Whether to activate CAD or not.
  @return TRUE if successful, FALSE if not.
*/
void ftm_audio_dummy_mvs_event_cb(mvs_event_type *event)
{
}

/**
  @ingroup ftm_audio_mvs_doxy_group
  @brief MVS callback for uplink vocoder frames.

  This function is registered with MVS as the callback to handle uplink
  vocoder packets. This will copy the packet into a local buffer for use
  by the downlink callback function in order to implement packet loopback.

  @see ftm_audio_mvs_dl_cb
  @param vocoder_packet The buffer data.
  @param frame_info Signalling and frame type info. This should be stored
  along with the packet data.
  @param packet_length The length of the buffer in bytes.
  @param status No idea...? Anyway, we don't need it.
*/
void ftm_audio_mvs_ul_cb(
  u8 *vocoder_packet,
  mvs_frame_info_type *frame_info,
  u16               packet_length,
  mvs_pkt_status_type *status
) {

  if (vocoder_packet != NULL && frame_info != NULL)
  {
    pr_err("%s: UL vocoder packet buffer: %d bytes\n", __func__,packet_length);
    memcpy(
      &ftm_audio_state.mvs.lb_pktbuf[ftm_audio_state.mvs.wr_index],
      vocoder_packet, packet_length);
    ftm_audio_state.mvs.lb_pktinfo[ftm_audio_state.mvs.wr_index] = *frame_info;
    ftm_audio_state.mvs.lb_pktsize[ftm_audio_state.mvs.wr_index] = packet_length;
    ftm_audio_state.mvs.wr_index++;
    ftm_audio_state.mvs.wr_index %= MVS_LB_PKT_DEPTH;
  }
  else
  {
	pr_err("%s: UL buffer missing!\n", __func__);
  }

}

/**
  @ingroup ftm_audio_mvs_doxy_group
  @brief MVS callback for downlink vocoder frames.

  This function is registered with MVS as the callback to handle downlink
  vocoder packets. MVS is expecting that a vocoder frame and info will be
  copied into the supplied pointers by the time this function returns.

  In this implemention, we're getting the frame from a local buffer that is
  filled by the uplink callback function.

  @see ftm_audio_mvs_ul_cb
  @param vocoder_packet The buffer data.
  @param frame_info Signalling and frame type info. This should be stored
  along with the packet data.
  @param packet_length The length of the buffer in bytes.
  @param status No idea...? Anyway, we don't need it.
*/
void ftm_audio_mvs_dl_cb(
  u8 *vocoder_packet,
  mvs_frame_info_type  *frame_info,
  mvs_pkt_status_type  *status
) {
  if (vocoder_packet == NULL || frame_info == NULL)
  {
    return;
  }

  if (ftm_audio_state.mvs.lb_pktsize[ftm_audio_state.mvs.rd_index] == 0)
  {
    pr_err("%s: DL buffer missing!!!\n", __func__);
	ftm_audio_state.mvs.lb_pktsize[ftm_audio_state.mvs.rd_index] = 0;
	
    switch (ftm_audio_state.mvs.mvs_mode)
    {
    case MVS_MODE_AMR:
      frame_info->amr_rate.dl_info.frame = MVS_AMR_NO_DATA;
      frame_info->amr_rate.dl_info.mode = MVS_AMR_MODE_1220;
      break;
    case MVS_MODE_IS127:
      frame_info->voc_rate.rx_info.rate = MVS_VOC_0_RATE;            
      break;
    case MVS_MODE_IS733:
      frame_info->voc_rate.rx_info.rate = MVS_VOC_0_RATE;            
      break;
    case MVS_MODE_4GV_NB:
      frame_info->voc_rate.rx_info.rate = MVS_VOC_0_RATE;
      break;
    case MVS_MODE_EFR:
    case MVS_MODE_FR:
    case MVS_MODE_HR:
    default:
      break;
    }
  }
  else
  {
    pr_err("%s: DL vocoder packet buffer: %d bytes\n", __func__,ftm_audio_state.mvs.lb_pktsize[ftm_audio_state.mvs.rd_index]);
	
    memcpy(
      vocoder_packet,
      ftm_audio_state.mvs.lb_pktbuf[ftm_audio_state.mvs.rd_index],
      ftm_audio_state.mvs.lb_pktsize[ftm_audio_state.mvs.rd_index]);
    *frame_info = ftm_audio_state.mvs.lb_pktinfo[ftm_audio_state.mvs.rd_index];

    ftm_audio_state.mvs.lb_pktsize[ftm_audio_state.mvs.rd_index] = 0;

    ftm_audio_state.mvs.rd_index++;
    ftm_audio_state.mvs.rd_index %= MVS_LB_PKT_DEPTH;
  }
}


/**
  @ingroup ftm_audio_mvs_doxy_group
  @brief Stops MVS packet loopback, if active.

  If an MVS loopback session is active, this function releases MVS, closes
  the rx and tx active session handles in CAD, and decrements reference count
  to CAD.
  
  @see ftm_audio_cad_add_ref
*/
#if 0
static int ftm_audio_stop_mvs_loopback(void)
{
  int rc = 0;

  if( ftm_audio_state.mvs.mvs_active && !loopback_falg)
  {
    mvs_release(MVS_CLIENT_TEST);

    ftm_audio_state.mvs.mvs_active = false;

    /* MICBIAS - this really is only necessary for MIC1 & MIC2 */
    pmic_mic_en(false);
  }

  return rc;
}
#endif 

static int ftm_audio_stop_mvs_loopback(void)
{
  int rc = 0;

  if( ftm_audio_state.mvs.mvs_active)
  {
    mvs_loopback_stop();

    ftm_audio_state.mvs.mvs_active = false;

    /* MICBIAS - this really is only necessary for MIC1 & MIC2 */
    pmic_mic_en(false);
  }

  return rc;
}


/**
  @ingroup ftm_audio_mvs_doxy_group
  @brief 

*/
#if 0
static int ftm_audio_start_mvs_loopback(void)
{
  int rc = 0;
  pr_info("ftm_audio_start_mvs_loopback\n");
  if( !ftm_audio_state.mvs.mvs_active )
  {

      pr_info("ftm_audio_start_mvs_loopback 11 \n ");
  	if(loopback_falg)
	{
		loopback_falg=false;
		      pr_info("ftm_audio_start_mvs_loopback 22 \n ");
		ftm_audio_one_time_init();
	}
	
    memset(ftm_audio_state.mvs.lb_pktbuf,0,320 * MVS_LB_PKT_DEPTH);
    memset(ftm_audio_state.mvs.lb_pktsize,0,
      sizeof(ftm_audio_state.mvs.lb_pktsize));
    memset(ftm_audio_state.mvs.lb_pktinfo,0,
      sizeof(ftm_audio_state.mvs.lb_pktinfo));
    ftm_audio_state.mvs.rd_index = 0;
    ftm_audio_state.mvs.wr_index = 1;

	pr_info("ftm_audio_start_mvs_loopback 33\n ");

    mvs_acquire(
      MVS_CLIENT_TEST,
      &ftm_audio_dummy_mvs_event_cb
      );

	pr_info("ftm_audio_start_mvs_loopback 44\n ");
		
    mvs_enable(
      MVS_CLIENT_TEST,
      ftm_audio_state.mvs.mvs_mode,
      &ftm_audio_mvs_ul_cb,
      &ftm_audio_mvs_dl_cb,
      MVS_PKT_CONTEXT_NONE
      );

	pr_info("ftm_audio_start_mvs_loopback 55\n ");

    ftm_audio_state.mvs.mvs_active = true;

	pmic_mic_set_volt(MIC_VOLT_1_80V);
	pmic_mic_en(true);

  }

  return rc;
}
#endif 

static int ftm_audio_start_mvs_loopback(void)
{
  int rc = 0;

  pr_info("ftm_audio_start_mvs_loopback\n");

  if( !ftm_audio_state.mvs.mvs_active )
  {
	mvs_loopback_start();

	ftm_audio_state.mvs.mvs_active = true;
	pmic_mic_set_volt(MIC_VOLT_1_80V);
	pmic_mic_en(true);

  }

  return rc;
}

#endif
// LGE_DOM_UPDATE_E jin333.kim@lge.com 2010/02/20 }

MODULE_AUTHOR("eklee.lee@lge.com");
MODULE_DESCRIPTION("LGE qdsp6 audio_ctl driver");
MODULE_LICENSE("GPL");
