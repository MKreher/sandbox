#include <zephyr.h>
#include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include <e3000h.h>

#define MODULE e3000h_barcode_module

#include <logging/log.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_E3000H_BARCODE_MODULE_LOG_LEVEL);

#define DT_DRV_COMPAT yoko_e3000h

static int barcode_init(const struct device *dev)
{
	LOG_DBG("barcode_init()");
	return 0;
}

static int barcode_start_decoding(const struct device *dev)
{
	LOG_DBG("barcode_start_decoding()");
	return 0;
}

static int barcode_stop_decoding(const struct device *dev)
{
	LOG_DBG("barcode_stop_decoding()");
	return 0;
}

static e3000h_command_return_t barcode_send_command(const struct device *dev, const e3000h_command_t command)
{
	LOG_DBG("barcode_send_command()");
	return NULL;
}

static const struct barcode_driver_api barcode_api = {
    // .set_barcode_handler = barcode_set_barcode_handler,
	.start_decoding = barcode_start_decoding,
	.stop_decoding = barcode_stop_decoding,
	.send_command = barcode_send_command,
};

DEVICE_DT_INST_DEFINE(0,            			   		//node_id	
					  &barcode_init,                    //init_fn
			  	 	  NULL,                             //pm_control_fn
		      	 	  NULL,                             //data_ptr
			  	 	  NULL,                             //cfg_ptr
		      	 	  APPLICATION,                      //level
			  	 	  CONFIG_APPLICATION_INIT_PRIORITY, //prio
		      	 	  &barcode_api                      //api_ptr
			  		 );
