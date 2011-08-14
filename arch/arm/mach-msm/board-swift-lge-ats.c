/* arch/arm/mach-msm/lge_ats.c
 *  This module is stud for rild
 * Copyright (C) 2008 LGE, Inc.
 * Copyright (C) 2011 wingrime
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/msm_rpcrouter.h>
//#include <linux/lge_alohag_at.h>
//#include <mach/board_lge.h>
#include "board-swift-lge-ats.h"

/* Ats server definitions. */

#define ATS_APPS_APISPROG		0x30000006
#define ATS_APPS_APISVERS		0

#define ONCRPC_LGE_ATCMD_ATS_PROC 3
#define ONCRPC_LGE_ATCMD_ATS_ETA_PROC 6
#define ONCRPC_LGE_GET_FLEX_MCC_PROC 7 //LGE_UPDATE_S [irene.park@lge.com] 2010.06.04 - Get Flex MCC/MNC value
#define ONCRPC_LGE_GET_FLEX_MNC_PROC 8 //LGE_UPDATE_S [irene.park@lge.com] 2010.06.04 - Get Flex MCC/MNC value
#define ONCRPC_LGE_GET_FLEX_OPERATOR_CODE_PROC 9 ///LGE_UPDATE_S [irene.park@lge.com] 2010.06.04 - Get Flex Operator ?value
#define ONCRPC_LGE_GET_FLEX_COUNTRY_CODE_PROC 10 //LGE_UPDATE_S [irene.park@lge.com] 2010.06.04 - Get Flex Operator ?value

//struct ats_data {
//	struct atcmd_dev *atdev;
//	int (*handle_atcmd) (struct msm_rpc_server *server,
//						 struct rpc_request_hdr *req, unsigned len,
//						 void (*update_atcmd_state)(char *cmd, int state) );
//	int (*handle_atcmd_eta) (struct msm_rpc_server *server,
//							 struct rpc_request_hdr *req, unsigned len);
//	void (*update_atcmd_state) (char *cmd, int state);
//LGE_CHAGE[irene.park@lge.com] 2010-06- 04 - to get flex value from ARM9 
//	int (*handle_flex) (struct msm_rpc_server *server,
//							 struct rpc_request_hdr *req, unsigned len);
//
//};

//struct ats_data lge_ats_data;

//static void lge_ats_update_atcmd_state(char *cmd, int state)
//{
//
//	struct ats_data *data = &lge_ats_data;
//
//	if(!data->atdev)
//		data->atdev = atcmd_get_dev();
//	if(data->atdev)
//		update_atcmd_state(data->atdev, cmd, state);
//
//}

static int handle_ats_rpc_call(struct msm_rpc_server *server,
							   struct rpc_request_hdr *req, unsigned len)
{
//	struct ats_data *data = &lge_ats_data;

	switch (req->procedure)
	{
		case ONCRPC_LGE_ATCMD_ATS_ETA_PROC:
			printk("LGE DRM %s: ONCRPC_LGE_ATCMD_ATS_ETA_PROC\n", __func__);
			//if(data->handle_atcmd_eta)
			//	return data->handle_atcmd_eta(server, req, len);
			break;
		case ONCRPC_LGE_ATCMD_ATS_PROC:
			printk("LGE DRM %s: ONCRPC_LGE_ATCMD_ATS_PROC\n", __func__);
			//if(data->handle_atcmd)
			//	return data->handle_atcmd(server, req, len, data->update_atcmd_state);
			break;
	    //LGE_CHAGE[irene.park@lge.com] 2010-06- 04 - to get flex value from ARM9 			
		case ONCRPC_LGE_GET_FLEX_MCC_PROC:
		case ONCRPC_LGE_GET_FLEX_MNC_PROC:
		case ONCRPC_LGE_GET_FLEX_OPERATOR_CODE_PROC:
		case ONCRPC_LGE_GET_FLEX_COUNTRY_CODE_PROC:
			printk("LGE DRM %s:ONCRPC_LGE_GET_FLEX_ %d\n", __func__,req->procedure);
			//if(data->handle_flex)
			//	return data->handle_flex(server, req, len);
			break;
		default:
			printk("LGE DRM UNreq call");
			return -ENODEV;
	}

	return 0;//success
}

static int __devexit ats_remove(struct platform_device *pdev)
{
	return 0;
}

static int ats_probe(struct platform_device *pdev)
{
	printk("LGE DRM ATS Stud driver probe\n");
	return 0;
}
static struct platform_driver ats_driver = {
	
	.probe = ats_probe,
	.remove = __devexit_p(ats_remove),
	.suspend = NULL,
	.resume  = NULL,
	.driver = {
		.name = "alohag_atcmd",
		.owner = THIS_MODULE,
		  },
};
//static struct platform_device ats_input_device = {
//	.name = "ats_input",
//};


static struct msm_rpc_server ats_rpc_server = {
	.prog = ATS_APPS_APISPROG,
	.vers = ATS_APPS_APISVERS,
	.rpc_call = handle_ats_rpc_call,
};

static int __init lge_ats_init(void)
{
	int err;

	if((err = msm_rpc_create_server(&ats_rpc_server)) != 0) {
		printk(KERN_ERR"LGE DRM %s: Error during creating rpc server for ats atcmd\n", __func__);
		return err;
	}

	//lge_ats_data.handle_atcmd = lge_ats_handle_atcmd;
	//lge_ats_data.handle_atcmd_eta = lge_ats_handle_atcmd_eta;
	//lge_ats_data.update_atcmd_state = lge_ats_update_atcmd_state;
	//lge_ats_data.handle_flex = lge_ats_handle_flex; //LGE_CHAGE[irene.park@lge.com] 2010-06- 04 - to get flex value from ARM9 


	platform_driver_register(&ats_driver);


//	platform_device_register(&ats_input_device);


	return err;
}

module_init(lge_ats_init);

static void __exit ats_exit(void)
{
    platform_driver_unregister(&ats_driver);
}
module_exit(ats_exit);
MODULE_DESCRIPTION("Swift LGE ATS  driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:swift-lge-ats");

