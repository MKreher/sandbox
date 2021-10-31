#ifndef ZEPHYR_INCLUDE_DRIVERS_E3000H_H_
#define ZEPHYR_INCLUDE_DRIVERS_E3000H_H_

#include <device.h>
#include <stddef.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct barcode_t
{
	int length;
	char* data;
};

typedef void (*barcode_handler_t)(const struct barcode_t *barcode);

// API
typedef int (*barcode_start_decoding_api)(const struct device *dev, barcode_handler_t barcode_handler);
typedef int (*barcode_stop_decoding_api)(const struct device *dev);
typedef int (*barcode_send_command_api)(const struct device *dev, const char* command, const int command_length, const bool check_for_ack_message);
typedef int (*barcode_send_command_with_response_api)(const struct device *dev, const char* command, const int command_length, char* response, int* response_length);


struct barcode_driver_api {
	barcode_start_decoding_api start_decoding;
	barcode_stop_decoding_api stop_decoding;
	barcode_send_command_api send_command;
	barcode_send_command_with_response_api send_command_with_response;
};

static inline int barcode_start_decoding(const struct device *dev, barcode_handler_t barcode_handler)
{
	struct barcode_driver_api *api = (struct barcode_driver_api *) dev->api;

	return api->start_decoding(dev, barcode_handler);
}

static inline int barcode_stop_decoding(const struct device *dev)
{
	struct barcode_driver_api *api = (struct barcode_driver_api *) dev->api;

	return api->stop_decoding(dev);
}

static inline int barcode_send_command(const struct device *dev, const char* command, const int command_length, const bool check_for_ack_message)
{
	struct barcode_driver_api *api = (struct barcode_driver_api *) dev->api;

	return api->send_command(dev, command, command_length, check_for_ack_message);
}

static inline int barcode_send_command_with_response(const struct device *dev, const char* command, const int command_length, char* response, int* response_length)
{
	struct barcode_driver_api *api = (struct barcode_driver_api *) dev->api;

	return api->send_command_with_response(dev, command, command_length, response, response_length);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_E3000H_H_ */
