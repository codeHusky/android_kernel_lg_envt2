// LGE_DOM_UPDATE_S jin333.kim@lge.com 2010/02/20 {
/* arch/arm/mach-msm/lge_mvs_pkt_loopback_clnt.c
 *
 * Copyright (c) 2008-2010, LG Electronics. All rights reserved.
 *
 * Original code is from Qualcomm mvs_clnt.c in AMSS region.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */


#include "lge_mvs_pkt_loopback_clnt.h"
#include <mach/msm_rpcrouter.h>
#include <linux/err.h>
#include <linux/delay.h>


static struct msm_rpc_client *rpc_client;
//static uint32_t open_count;
static DEFINE_MUTEX(mvs_client_lock);


struct mvs_event_cb_recv {
    uint32_t cb_id;
	mvs_event_type *event;
};

struct mvs_ul_fn_recv {
    uint32_t cb_id;
	uint32_t length;
//	uint16_t length;
	uint8_t vocoder_packet[320];
	mvs_frame_info_type frame_info;
	uint32_t packet_length;
//	uint16_t packet_length;
	mvs_pkt_status_type output_pointer_not_null;	
};

struct mvs_dl_fn_recv {
//    uint32_t cb_id;
	uint8_t vocoder_packet[320];
	mvs_frame_info_type frame_info;
	mvs_pkt_status_type output_pointer_not_null;
};


extern void ftm_audio_dummy_mvs_event_cb(mvs_event_type *event);

extern void ftm_audio_mvs_ul_cb(
  uint8_t *vocoder_packet,
  mvs_frame_info_type *frame_info,
  uint16_t               packet_length,
  mvs_pkt_status_type *status
);

extern void ftm_audio_mvs_dl_cb(
  uint8_t *vocoder_packet,
  mvs_frame_info_type  *frame_info,
  mvs_pkt_status_type  *status
);


static int mvs_packet_event_callback(struct mvs_event_cb_recv *recv)
{
	if (!recv)
		return -ENODATA;

   ftm_audio_dummy_mvs_event_cb((recv->event));

   return 0;
}

static int mvs_packet_ul_callback(struct mvs_ul_fn_recv *recv)
{
	if (!recv)
		return -ENODATA;
		
 	ftm_audio_mvs_ul_cb(&(recv->vocoder_packet[0]),
		                &(recv->frame_info),
		                be32_to_cpu((recv->length)),
		                &(recv->output_pointer_not_null));

   return 0;
}


static int mvs_packet_dl_callback(struct msm_rpc_client *client,struct mvs_dl_fn_recv *recv, struct mvs_dl_fn_recv *resp )
{
	if (!recv)
		return -ENODATA;

	ftm_audio_mvs_dl_cb(&(resp->vocoder_packet[0]),
						&(resp->frame_info),
						&(resp->output_pointer_not_null));
		
	return sizeof(struct mvs_dl_fn_recv) ;
}



static int process_mvs_rpc_request(struct msm_rpc_client *client,uint32_t proc, void *req , void *resp)
{
    if (proc == ONCRPC_MVS_EVENT_CB_TYPE_PROC)
		mvs_packet_event_callback(req);
	else if (proc == ONCRPC_MVS_PACKET_UL_FN_TYPE_PROC)
		mvs_packet_ul_callback(req);
	else if(proc == ONCRPC_MVS_PACKET_DL_FN_TYPE_PROC)
		return mvs_packet_dl_callback(client,req,resp);
	else
		pr_err("%s: unknown rpc proc %d\n", __func__, proc);
	
	return 0 ;
}


static int mvs_cb_func(struct msm_rpc_client *client, void *buffer, int in_size)
{
	int rc = -1;

	struct rpc_request_hdr *hdr = buffer;

	void *rpc_reply_data ;
	int resp_data_size ;

	hdr->type = be32_to_cpu(hdr->type);
	hdr->xid = be32_to_cpu(hdr->xid);
	hdr->rpc_vers = be32_to_cpu(hdr->rpc_vers);
	hdr->prog = be32_to_cpu(hdr->prog);
	hdr->vers = be32_to_cpu(hdr->vers);
	hdr->procedure = be32_to_cpu(hdr->procedure);

	if (hdr->type != 0  )
		return rc;
	if (hdr->rpc_vers != 2  )
		return rc;
	if (hdr->prog != MVSCBPROG)
		return rc;
	if (!msm_rpc_is_compatible_version(MVSCBVERS,
				hdr->vers))
		return rc;

	rpc_reply_data = msm_rpc_start_accepted_reply(client, hdr->xid,
					 RPC_ACCEPTSTAT_SUCCESS);

	resp_data_size = process_mvs_rpc_request(client,hdr->procedure,
				(void *) (hdr + 1) , rpc_reply_data);

	rc = msm_rpc_send_accepted_reply(client, resp_data_size);
	
	if (rc) {
		pr_err("%s: sending reply failed: %d\n", __func__, rc);
		return rc;
	}

	return 0;
}



struct msm_rpc_client *mvs_init(void)
{
	mutex_lock(&mvs_client_lock);
//	if (open_count++ == 0) {
	rpc_client = msm_rpc_register_client("mvs",
						     MVSPROG,
						     MVSVERS, 0,
						     mvs_cb_func);

	//memset(rpc_client->reply,0,MSM_RPC_MSGSIZE_MAX) ;
	//memset(&rpc_client->xdr,0,MSM_RPC_MSGSIZE_MAX) ;
	
//	}
	mutex_unlock(&mvs_client_lock);
	return rpc_client;	
}

static void mvs_deinit(void)
{
	mutex_lock(&mvs_client_lock);
//	if (--open_count == 0) {
		msm_rpc_unregister_client(rpc_client);
		pr_info("%s: disconnected from remote mvs server\n",
			__func__);
//	}
	mutex_unlock(&mvs_client_lock);
}

struct mvs_acquire_rpc_args {
	mvs_client_type client;
	uint32_t cb_id1;
};


static int mvs_acquire_rpc_arg_cb(struct msm_rpc_client *client,
				    void *buffer, void *data)
{

	memcpy( buffer , data , sizeof(struct mvs_acquire_rpc_args)) ;

	return sizeof(struct mvs_acquire_rpc_args);

}



void mvs_acquire(mvs_client_type client,  mvs_event_cb_type cb_func)
{
	int rc = 0;
	struct mvs_acquire_rpc_args req ;

	(void)mvs_init() ;
	
	req.client	= cpu_to_be32(client);
	req.cb_id1	= (uint32_t)cb_func;


	rc = msm_rpc_client_req(rpc_client, ONCRPC_MVS_ACQUIRE_PROC,
				mvs_acquire_rpc_arg_cb, &req,
				NULL, NULL, -1);
	if (rc) {
		pr_err("%s: couldn't send rpc client request\n", __func__);
		msm_rpc_unregister_client(rpc_client);
	}

} /* mvs_acquire */




struct mvs_enable_rpc_args {
	mvs_client_type client;
	mvs_mode_type mode;
	uint32_t cb_id1;
	uint32_t cb_id2;
	mvs_pkt_context_type context;
};


static int mvs_enable_rpc_arg_cb(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	memcpy( buffer , data , sizeof(struct mvs_enable_rpc_args)) ;
	
	return sizeof(struct mvs_enable_rpc_args);
}


void mvs_enable(
  mvs_client_type client,
  mvs_mode_type mode,
  mvs_packet_ul_fn_type ul_func,
  mvs_packet_dl_fn_type dl_func,
  mvs_pkt_context_type context
)
{
	int rc = 0;
	struct mvs_enable_rpc_args req ;
	
	req.client	= cpu_to_be32(client);
	req.mode	= cpu_to_be32(mode);
	req.cb_id1	= (uint32_t)ul_func;
	req.cb_id2	= (uint32_t)dl_func;
	req.context	= cpu_to_be32(context);
	
	rc = msm_rpc_client_req(rpc_client, ONCRPC_MVS_ENABLE_PROC,
				mvs_enable_rpc_arg_cb, &req,
				NULL, NULL, -1);
	if (rc) {
		pr_err("%s: couldn't send rpc client request\n", __func__);
		msm_rpc_unregister_client(rpc_client);
	}

} /* mvs_enable */


static int mvs_release_rpc_arg_cb(struct msm_rpc_client *client,
				    void *buffer, void *data)
{
	memcpy( buffer , data , sizeof(mvs_client_type)) ;

	return sizeof(mvs_client_type);
}


void mvs_release(mvs_client_type client)
{
	int rc = 0;

	mvs_client_type req;

	req	= cpu_to_be32(client);

	rc = msm_rpc_client_req(rpc_client, ONCRPC_MVS_RELEASE_PROC,
				mvs_release_rpc_arg_cb, &req,
				NULL, NULL, -1);

	(void)mvs_deinit() ;

} /* mvs_release */
// LGE_DOM_UPDATE_E jin333.kim@lge.com 2010/02/20 }

void mvs_loopback_start(void)
{
		int rc = 0;
	
		(void)mvs_init() ;

		rc = msm_rpc_client_req(rpc_client, ONCRPC_MVS_LOOPBACK_START_PROC,
						NULL, NULL,
						NULL, NULL, -1);
		if (rc) {  
			pr_err("%s: couldn't send rpc client request\n", __func__);
			msm_rpc_unregister_client(rpc_client);
		}
	
}

void mvs_loopback_stop(void)
{
		int rc = 0;
	
		rc = msm_rpc_client_req(rpc_client, ONCRPC_MVS_LOOPBACK_STOP_PROC,
						NULL, NULL,
						NULL, NULL, -1);
		
		(void)mvs_deinit() ;
	
}

MODULE_AUTHOR("eklee.lee@lge.com");
MODULE_DESCRIPTION("LGE mvs_pkt loopback driver");
MODULE_LICENSE("GPL");
