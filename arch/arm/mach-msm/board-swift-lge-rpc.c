/* arch/arm/mach-msm/lge_ats.c
 * This driver implements additional RPC functons that needed bye lge rild
 * without this driver android will hung after start rild application
 * also lge bring headset button on modem, this driver send message direct to ts driver
 * TODO: make input device that handle HS button messages right way
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
#include "board-swift-lge-ats.h"

int swift_hs_press();

 
/* Ats server definitions. */

#define ATS_APPS_APISPROG		0x30000006
#define ATS_APPS_APISVERS		0

#define ONCRPC_LGE_ATCMD_ATS_PROC 3
#define ONCRPC_LGE_ATCMD_ATS_ETA_PROC 6
#define ONCRPC_LGE_GET_FLEX_MCC_PROC 7 // Get Flex MCC/MNC value
#define ONCRPC_LGE_GET_FLEX_MNC_PROC 8 // Get Flex MCC/MNC value
#define ONCRPC_LGE_GET_FLEX_OPERATOR_CODE_PROC 9 // Get Flex Operator value
#define ONCRPC_LGE_GET_FLEX_COUNTRY_CODE_PROC 10 // Get Flex Operator ?value

static int handle_ats_rpc_call(struct msm_rpc_server *server,
							   struct rpc_request_hdr *req, unsigned len)
{
	switch (req->procedure)
	{
		case ONCRPC_LGE_ATCMD_ATS_PROC:
		  /*handle AT commands from modem*/
		  /*we only want Head set button */
		  /*without bring all structures */
		  swift_hs_press();	
			break;
	        case ONCRPC_LGE_ATCMD_ATS_ETA_PROC:		
		case ONCRPC_LGE_GET_FLEX_MCC_PROC:
		case ONCRPC_LGE_GET_FLEX_MNC_PROC:
		case ONCRPC_LGE_GET_FLEX_OPERATOR_CODE_PROC:
		case ONCRPC_LGE_GET_FLEX_COUNTRY_CODE_PROC:
			printk("LGE RPC call number %d\n",req->procedure);
			break;
		default:
			printk("LGE UNDEF call");
			return -ENODEV;
	}
	/*without return success on all FLEX command we get hung phone instant after rild start*/
	return 0;//success
}

static int __devexit ats_remove(struct platform_device *pdev)
{
	return 0;
}

static int ats_probe(struct platform_device *pdev)
{;
	return 0;
}
static struct platform_driver ats_driver = {
	
	.probe = ats_probe,
	.remove = __devexit_p(ats_remove),
	.suspend = NULL,
	.resume  = NULL,
	.driver = {
		.name = "swift_lge_rpc",
		.owner = THIS_MODULE,
		  },
};


static struct msm_rpc_server ats_rpc_server = {
	.prog = ATS_APPS_APISPROG,
	.vers = ATS_APPS_APISVERS,
	.rpc_call = handle_ats_rpc_call,
};

static int __init lge_ats_init(void)
{
	int err;

	if((err = msm_rpc_create_server(&ats_rpc_server)) != 0) {
		printk(KERN_ERR"LGE RPC Register failed:%s", __func__);
		return err;
	}


	platform_driver_register(&ats_driver);

	return err;
}

module_init(lge_ats_init);

static void __exit ats_exit(void)
{
    platform_driver_unregister(&ats_driver);
}
module_exit(ats_exit);
MODULE_DESCRIPTION("Swift LGE RPC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:swift-lge-rpc");

