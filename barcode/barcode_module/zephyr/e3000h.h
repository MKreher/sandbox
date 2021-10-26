#ifndef ZEPHYR_INCLUDE_DRIVERS_E3000H_H_
#define ZEPHYR_INCLUDE_DRIVERS_E3000H_H_

#include <device.h>
#include <stddef.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef char* e3000h_command_t;
typedef char* e3000h_command_return_t;

//typedef void (*barcode_handler)(const void *barcode_buf);

// API
//typedef int(*barcode_set_barcode_handler_api)(const struct device *dev, const *barcode_handler);
typedef int (*barcode_start_decoding_api)(const struct device *dev);
typedef int (*barcode_stop_decoding_api)(const struct device *dev);
typedef e3000h_command_return_t (*barcode_send_command_api)(const struct device *dev, const e3000h_command_t command);


struct barcode_driver_api {
//	barcode_set_barcode_handler_api set_barcode_handler;
	barcode_start_decoding_api start_decoding;
	barcode_stop_decoding_api stop_decoding;
	barcode_send_command_api send_command;
};

/*
static inline int set_barcode_handler(const struct device *dev, const *barcode_handler)
{
	struct barcode_driver_api *api = (struct barcode_driver_api *) dev->api;

	return api->set_barcode_handler(dev, barcode_handler);
}
*/

static inline int start_decoding(const struct device *dev)
{
	struct barcode_driver_api *api = (struct barcode_driver_api *) dev->api;

	return api->start_decoding(dev);
}

static inline int stop_decoding(const struct device *dev)
{
	struct barcode_driver_api *api = (struct barcode_driver_api *) dev->api;

	return api->stop_decoding(dev);
}

static inline e3000h_command_return_t send_command(const struct device *dev, const e3000h_command_t command)
{
	struct barcode_driver_api *api = (struct barcode_driver_api *) dev->api;

	return api->send_command(dev, command);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_E3000H_H_ */
