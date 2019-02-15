#include <linux/module.h>
#include <linux/delay.h>
#include <mach/lg_diagcmd.h>
#include <mach/lg_diag_wmc.h>

#include <linux/unistd.h> /*for open/close*/
#include <linux/fcntl.h> /*for O_RDWR*/
#include <linux/syscalls.h> //for sys operations
#include <linux/fs.h> // for file struct
/* ==========================================================================

                      EXTERNAL FUNCTION AND VARIABLE DEFINITIONS

===========================================================================*/
//extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
//extern unsigned int LGF_KeycodeTrans(word input);

/*==========================================================================

            LOCAL DEFINITIONS AND DECLARATIONS FOR MODULE

  This section contains local definitions for constants, macros, types,
  variables and other items needed by this module.

===========================================================================*/
#ifdef CONFIG_LGE_DIAG_WMC

#ifdef CONFIG_LGE_DIAG_TESTMODE
extern int get_first_booting_complete_status(void);
extern int get_first_booting_chg_mode_status(void);
#endif

#ifdef CONFIG_LGE_USB_GADGET_DRIVER
extern void lge_usb_composite_unregister(void);
extern void lge_usb_composite_register(void);
#endif

void* lg_diag_wmc_req_pkt_ptr;
uint16 lg_diag_wmc_req_pkt_length;
uint16 lg_diag_wmc_rsp_pkt_length;

static unsigned char wmc_retry_cnt = 0;

// if the starup retry count is 3, then reconnect modem port so that the tool retries the startup again.
void wmc_check_retry(void)
{
  if(wmc_retry_cnt == 3)
  {
    printk(KERN_INFO "%s, startup count is : %d, reconnect modem port", __func__, wmc_retry_cnt);
    lge_usb_composite_unregister();
    lge_usb_composite_register();
    wmc_retry_cnt = 0;
  }
  else
    printk(KERN_INFO "%s, startup count is : %d", __func__, wmc_retry_cnt);
}

PACK (void *)LGF_WMC (
        PACK (void	*)req_pkt_ptr,	/* pointer to request packet  */
        uint16		pkt_len )		      /* length of request packet   */
{

  int ret;
  char cmdstr[100];
  int fd;

  wmcSync_command_req_type *req_ptr;
  
  char *envp[] = {
    "HOME=/",
    "TERM=linux",
    NULL,
  };

  char *argv[] = {
    "sh",
    "-c",
    cmdstr,
    NULL,
  };	

 // static struct diagcmd_dev *diagpdev;
  
  printk(KERN_INFO "%s, pkt_len : %d\n",__func__, pkt_len);
  printk(KERN_INFO "%s, cmd_code : 0x%x, sub_cmd : 0x%x\n",__func__, *(unsigned char *)(req_pkt_ptr + 0), *(unsigned char *)(req_pkt_ptr + 1));

  lg_diag_wmc_req_pkt_ptr = req_pkt_ptr;
  lg_diag_wmc_req_pkt_length = pkt_len;

  req_ptr = (wmcSync_command_req_type *) req_pkt_ptr;
  if(req_ptr->sub_cmd_code == WMC_STARTUP)
  {
    wmc_retry_cnt++;
    printk(KERN_INFO "%s, startup is called, incread count : %d", __func__, wmc_retry_cnt); 
  }

// check first boot complete and charging mode to clarify whether wmc services are available.
// wmc won't be ready if the target is in the charging mode or it is on boot-up
#ifdef CONFIG_LGE_DIAG_TESTMODE
  if(get_first_booting_chg_mode_status() == 1)
  {
    printk(KERN_INFO "%s, the target is in the charging mode so this service is not available", __func__);
    //wmc_check_retry(); // do not re-attatch modem port in the charging mode
    return NULL;
  }
  else if (get_first_booting_complete_status() != 1)
  {
    printk(KERN_INFO "%s, the target is on boot-up so this service is not available", __func__);  
    wmc_check_retry();
    return NULL;
  }
#endif

  if ( (fd = sys_open((const char __user *) "/system/bin/lg_diag_wmc", O_RDONLY ,0) ) < 0 )
  {
    printk("\n can not open /system/bin/lg_diag - execute /system/bin/lg_diag_wmc\n");
    sprintf(cmdstr, "/system/bin/lg_diag_wmc\n");
  }
  else
  {
    //printk("\n execute /system/bin/lg_diag_wmc\n");
    sprintf(cmdstr, "/system/bin/lg_diag_wmc\n");
    sys_close(fd);
  }

  printk(KERN_INFO "execute - %s", cmdstr);

  if ((ret = call_usermodehelper("/system/bin/sh", argv, envp, UMH_WAIT_PROC)) != 0) {
  	printk(KERN_ERR "%s, failed to run %s: %i\n", __func__, cmdstr, ret);
	wmc_check_retry();
  }
  else
  {
    printk(KERN_INFO "%s, %s execute ok\n", __func__, cmdstr);
    //if userspace call is ready to handle wmc commands, initialize the retry count.
    wmc_retry_cnt = 0;
  }
  
  return NULL;

}

EXPORT_SYMBOL(LGF_WMC);

#endif
