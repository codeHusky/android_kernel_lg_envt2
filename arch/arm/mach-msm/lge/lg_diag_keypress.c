#include <linux/module.h>
#include <linux/delay.h>
#include <mach/lg_diagcmd.h>
#include <mach/lg_diag_keypress.h>
#include <linux/input.h>
#include <mach/gpio.h>
/*==========================================================================*/
#define HS_RELEASE_K 0xFFFF

enum {
	GPIO_SLIDE_CLOSE=0,
	GPIO_SLIDE_OPEN,
};

#define HALL_IC_IRQ  39

/* ==========================================================================
===========================================================================*/
extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
extern unsigned int LGF_KeycodeTrans(word input);
extern void Send_Touch( unsigned int x, unsigned int y);
/*==========================================================================*/

static unsigned saveKeycode =0 ;

#ifdef CONFIG_LGE_DIAG_KERNEL_SERVICE

void SendKey(unsigned int keycode, unsigned char bHold)
{
  extern struct input_dev *qwerty_get_input_dev(void);
  struct input_dev *idev = qwerty_get_input_dev();

  if( keycode != HS_RELEASE_K)
    input_report_key( idev,keycode , 1 ); // press event

  if(bHold)
  {
    saveKeycode = keycode;
  }
  else
  {
    if( keycode != HS_RELEASE_K)
      input_report_key( idev,keycode , 0 ); // release  event
    else
      input_report_key( idev,saveKeycode , 0 ); // release  event
  }
}


int is_slide_open(void)
{
   if(gpio_get_value(HALL_IC_IRQ) == GPIO_SLIDE_OPEN) // hall ic
      return 0;
   else
      return 1;
}


PACK (void *)LGF_KeyPress (
        PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
        uint16		pkt_len )		      /* length of request packet   */
{
  DIAG_HS_KEY_F_req_type *req_ptr = (DIAG_HS_KEY_F_req_type *) req_pkt_ptr;
  DIAG_HS_KEY_F_rsp_type *rsp_ptr;
  unsigned int keycode = 0;
  const int rsp_len = sizeof( DIAG_HS_KEY_F_rsp_type );

  rsp_ptr = (DIAG_HS_KEY_F_rsp_type *) diagpkt_alloc( DIAG_HS_KEY_F, rsp_len );

  if((req_ptr->magic1 == 0xEA2B7BC0) && (req_ptr->magic2 == 0xA5B7E0DF))
  {
    rsp_ptr->magic1 = req_ptr->magic1;
    rsp_ptr->magic2 = req_ptr->magic2;
    rsp_ptr->key = 0xff; //ignore byte key code
    rsp_ptr->ext_key = req_ptr->ext_key;

    keycode = LGF_KeycodeTrans((word) req_ptr->ext_key);
  }
  else
  {
    rsp_ptr->key = req_ptr->key;
    keycode = LGF_KeycodeTrans((word) req_ptr->key);

  }

  if( keycode == 0xff)
    keycode = HS_RELEASE_K;  // to mach the size
  

  switch (keycode){
	    /* LG_FW 2010.02.24 khlee - UTS TEST needs key to delete call log*/
  	case 0x60 :
    	if( is_slide_open() == 0) // touch call logs icon
      		Send_Touch(100,500);
    	else
      		Send_Touch(170,95);
		break;
	case 0x61:
    	if( is_slide_open() == 0) //touch select all icon
      		Send_Touch(420,190);
    	else
      		Send_Touch(360,720);
		break;
	case 0x62:
    	if( is_slide_open() == 0) //delete icon
      		Send_Touch(410,580);
    	else
      		Send_Touch(110,750);
		break;
	case 0x63:
    	if( is_slide_open() == 0) //ok  icon
      		Send_Touch(350,500);
    	else
      		Send_Touch(120,510);
		break;

	case 0x40 :
	    /* LG_FW 2010.02.24 khlee - UTS TEST needs send key to call in idle screen*/
    	if( is_slide_open() == 0) // folder open
      		Send_Touch(420,20);
    	else
      		Send_Touch(30,770);

		break;
	default:
    	SendKey(keycode , req_ptr->hold);
		break;
  	}
  	
  return (rsp_ptr);
}


EXPORT_SYMBOL(LGF_KeyPress);

#endif
