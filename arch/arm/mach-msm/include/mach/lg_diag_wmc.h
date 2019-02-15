#ifndef LG_DIAG_WMC_H
#define LG_DIAG_WMC_H

#include "lg_comdef.h"
/*********************** BEGIN PACK() Definition ***************************/
#if defined __GNUC__
  #define PACK(x)       x __attribute__((__packed__))
  #define PACKED        __attribute__((__packed__))
#elif defined __arm
  #define PACK(x)       __packed x
  #define PACKED        __packed
#else
  #error No PACK() macro defined for this compiler
#endif
/********************** END PACK() Definition *****************************/

typedef struct _wmcSync_command_req_type
{
    byte cmd_code; 		 // DM Command = 0xF1(241) for WMC
    byte sub_cmd_code;	 	 // WMC Sub Command
//    wmc_pim_data_type pim_data;	 // PIM DATA
}PACKED wmcSync_command_req_type;

//_________________________
//Diag Response Packet

typedef struct _wmcSync_command_rsp_type
{
    byte cmd_code; 		  // DM Command = 0xF1(241) for WMC
    byte sub_cmd_code; 		  // WMC Sub command
//    wmc_pim_data_type pim_data;   // PIM DATA
}PACKED wmcSync_command_rsp_type;


typedef enum {
	WMC_STARTUP = 100,
}WmcType;

#endif /* LG_DIAG_WMC_H */
