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

enum driver_state_enum {IDLE, AWAITING_BARCODE, AWAITING_CMD_ACK, AWAITING_CMD_RESPONSE};

struct e3000h_data
{
	const struct gpio_dt_spec *trigger;
	const struct gpio_dt_spec *reset;
	const struct gpio_dt_spec *buzzer;
	const struct gpio_dt_spec *led;
	const struct device *uart;
	barcode_handler_t barcode_handler;
	enum driver_state_enum state;
};

static struct e3000h_data e3000h_driver;

const struct gpio_dt_spec e3000h_trigger_gpio = GPIO_DT_SPEC_INST_GET(0, trigger_gpios);
const struct gpio_dt_spec e3000h_reset_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios);
const struct gpio_dt_spec e3000h_buzzer_gpio = GPIO_DT_SPEC_INST_GET(0, buzzer_gpios);
const struct gpio_dt_spec e3000h_led_gpio = GPIO_DT_SPEC_INST_GET(0, led_gpios);

#define UART_RX_TIMEOUT_MS      100
#define UART_RX_BUF_SIZE		16

// UART RX primary buffers
char uart_rx_buffer[UART_RX_BUF_SIZE];

enum message_type_enum {
	UNKNOWN,
	DEVICE_ACK_MESSAGE,
	DEVICE_NACK_RESEND_MESSAGE,
	DEVICE_NACK_DENIED_MESSAGE,
	DEVICE_NACK_BAD_CONTEXT_MESSAGE};

char CMD_ACK_RESPONSE_MESSAGE[] = { 0x04, 0xD0, 0x00, 0x00, 0xFF, 0x2C }; // Response ACK-Message from scan engine
char CMD_NACK_RESEND_RESPONSE_MESSAGE[] = { 0x04, 0xD1, 0x00, 0x01, 0xFF, 0x25 }; // Response NACK-Message (RESEND) from scan engine
char CMD_NACK_BAD_CONTEXT_RESPONSE_MESSAGE[] = { 0x04, 0xD1, 0x00, 0x02, 0xFF, 0x24 };  // Response NACK-Message (BAD_CONTEXT) from scan engine
char CMD_NACK_DENIED_RESPONSE_MESSAGE[] = { 0x04, 0xD1, 0x00, 0x06, 0xFF, 0x20 }; // Response NACK-Message (DENIED) from scan engine

// commands from host to the scan engine
char CMD_ACK[] = { 0x04, 0xD0, 0x04, 0x00, 0xFF, 0x28 };
char CMD_NACK_RESEND[] = { 0x05, 0xD1, 0x04, 0x00, 0x01, 0xFF, 0x25 };
char CMD_NACK_BAD_CONTEXT[] = { 0x05, 0xD1, 0x04, 0x00, 0x02, 0xFF, 0x24 };
char CMD_NACK_DENIED[] = { 0x05, 0xD1, 0x04, 0x00, 0x06, 0xFF, 0x20 };

char CMD_LED_OFF[] =  { 0x05, 0xE8, 0x04, 0x00, 0x01, 0xFF, 0x0E };
char CMD_LED_ON[] =  { 0x05, 0xE7, 0x04, 0x00, 0x01, 0xFF, 0x0F };

char CMD_REQUEST_REVISION[] =  { 0x04, 0xA3, 0x04, 0x00, 0xFF, 0x55 };
char CMD_REPLY_REVISION[] = { 0x00 };

char CMD_SCAN_DISABLE[] =  { 0x04, 0xEA, 0x04, 0x00, 0xFF, 0x0E };
char CMD_SCAN_ENABLE[] =  { 0x04, 0xE9, 0x04, 0x00, 0xFF, 0x0F };

char CMD_START_DECODE[] =  { 0x04, 0xE4, 0x04, 0x00, 0xFF, 0x14 };
char CMD_STOP_DECODE[] =  { 0x04, 0xE5, 0x04, 0x00, 0xFF, 0x13 };

char CMD_WAKEUP[] = { 0x00 };
char CMD_SLEEP[] =  { 0x04, 0xEB, 0x04, 0x00, 0xFF, 0x0D };
char CMD_RESET[] =  { 0x04, 0xFA, 0x04, 0x00, 0xFE, 0xFE };

char CMD_SET_FACTORY_DEFAULTS_0[] =  { 0x08, 0xC6, 0x04, 0x08, 0x00, 0xF2, 0xFF, 0x00, 0xFD, 0x35 };

char CMD_PARAM_SET_DEFAULTS[] = { 0x04, 0xC8, 0x04, 0x00, 0xFF, 0x30 };

char CMD_PARAM_GET_POWER_MODE[] = { 0x05, 0xC7, 0x04, 0x00, 0x80, 0xFE, 0xB0 };
char CMD_PARAM_SET_POWER_MODE_CONTINUOUS[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x80, 0x00, 0xFE, 0xA7 };
char CMD_PARAM_SET_POWER_MODE_LOW[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x80, 0x01, 0xFE, 0xA6 };

char CMD_PARAM_GET_TRIGGER_MODE[] = { 0x05, 0xC7, 0x04, 0x00, 0x8A, 0xFE, 0xA6 };
char CMD_PARAM_SET_TRIGGER_MODE_LEVEL[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x8A, 0x00, 0xFE, 0x9D };
char CMD_PARAM_SET_TRIGGER_MODE_PULSE[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x8A, 0x02, 0xFE, 0x9B };
char CMD_PARAM_SET_TRIGGER_MODE_CONTINUOUS[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x8A, 0x04, 0xFE, 0x99 };
char CMD_PARAM_SET_TRIGGER_MODE_HOST[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x8A, 0x08, 0xFE, 0x95 };
char CMD_PARAM_SET_TRIGGER_MODE_AUTOMATIC_INDUCTION[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x8A, 0x09, 0xFE, 0x94 };
char CMD_PARAM_SET_TRIGGER_MODE_BUTTON_CONTINUOUS[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x8A, 0x0A, 0xFE, 0x93 };

char CMD_PARAM_SET_NO_READ_MESSAGE_ENABLE[] = {0x07, 0xC6, 0x04, 0x08, 0x00, 0x5E, 0x01, 0xFE, 0xC8};
char CMD_PARAM_SET_NO_READ_MESSAGE_DISABLE[] = {0x07, 0xC6, 0x04, 0x08, 0x00, 0x5E, 0x00, 0xFE, 0xC9};

char CMD_PARAM_GET_BAUD_RATE[] = { 0x05, 0xC7, 0x04, 0x00, 0x9C, 0xFE, 0x94 };
char CMD_PARAM_SET_BAUD_RATE_4800[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x9C, 0x05, 0xFE, 0x86 };
char CMD_PARAM_SET_BAUD_RATE_9600[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x9C, 0x06, 0xFE, 0x85 };
char CMD_PARAM_SET_BAUD_RATE_19200[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x9C, 0x07, 0xFE, 0x84 };

char CMD_PARAM_GET_STOP_BITS[] = { 0x05, 0xC7, 0x04, 0x00, 0x9D, 0xFE, 0x93 };
char CMD_PARAM_SET_STOP_BITS_1[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x9D, 0x01, 0xFE, 0x89 };
char CMD_PARAM_SET_STOP_BITS_2[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x9D, 0x02, 0xFE, 0x88 };

char CMD_PARAM_GET_COMMUNICATION_MODE[] = { 0x06, 0xC7, 0x04, 0x00, 0xF2, 0x01, 0xFE, 0x3C };

char CMD_PARAM_GET_SOFTWARE_HANDSHAKING[] = { 0x05, 0xC7, 0x04, 0x00, 0x9F, 0xFE, 0x91 };
char CMD_PARAM_SET_SOFTWARE_HANDSHAKING_ENABLE[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x9F, 0x01, 0xFE, 0x87 };
char CMD_PARAM_SET_SOFTWARE_HANDSHAKING_DISABLE[] = { 0x07, 0xC6, 0x04, 0x08, 0x00, 0x9F, 0x00, 0xFE, 0x88 };

char CMD_PARAM_GET_TERMINATOR[] = { 0x06, 0xC7, 0x04, 0x00, 0xF2, 0x05, 0xFE, 0x38 };
char CMD_PARAM_SET_TERMINATOR_DISABLE[] = { 0x08, 0xC6, 0x04, 0x08, 0x00, 0xF2, 0x05, 0x00, 0xFE, 0x2F };

char CMD_PARAM_SET_ALL_1D_BARCODE_TYPES_ENABLE[] = {0x08, 0xC6, 0x04, 0x08, 0x00, 0xF2, 0x11, 0x01, 0xFE, 0x22};
char CMD_PARAM_SET_ALL_1D_BARCODE_TYPES_DISABLE[] = {0x08, 0xC6, 0x04, 0x08, 0x00, 0xF2, 0x11, 0x00, 0xFE, 0x23};
char CMD_PARAM_SET_ALL_2D_BARCODE_TYPES_ENABLE[] = {0x08, 0xC6, 0x04, 0x08, 0x00, 0xF2, 0x50, 0x01, 0xFD, 0xE3};
char CMD_PARAM_SET_ALL_2D_BARCODE_TYPES_DISABLE[] = {0x08, 0xC6, 0x04, 0x08, 0x00, 0xF2, 0x50, 0x00, 0xFD, 0xE4};
char CMD_PARAM_SET_ALL_BARCODE_TYPES_ENABLE[] = {0x08, 0xC6, 0x04, 0x08, 0x00, 0xF2, 0x90, 0x01, 0xFD, 0xA3};
char CMD_PARAM_SET_ALL_BARCODE_TYPES_DISABLE[] = {0x08, 0xC6, 0x04, 0x08, 0x00, 0xF2, 0x90, 0x00, 0xFD, 0xA4};

// forward declatations
static int e3000h_uart_read(const struct device *uart, char* rx_buf, const int rx_buf_len, int *rx_len, k_timeout_t timeout_in_ms);

/**
 * Function for calculating the 2's complement checksum for the given content.
 */
static uint16_t calc_checksum(const char * str, const uint8_t len)
{
    uint16_t checksum, i, sum = 0;

    for (i = 0; i < len; i++)
    {
        sum += str[i];
    }

    checksum = ~sum + 1;

    return checksum;
}

void clear_uart_rx_buffer()
{
	LOG_DBG("clear_uart_rx_buffer()");
	memset(&uart_rx_buffer, 0, UART_RX_BUF_SIZE);
}

static void e3000h_barcode_reading_entry_point(int unused1, int unused2, int unused3)
{	
	while (1)
	{
		while (e3000h_driver.state == AWAITING_BARCODE)
		{
			LOG_DBG("do_scanning...");
			
			clear_uart_rx_buffer();

			int rx_len = 0;

			int ret = e3000h_uart_read(e3000h_driver.uart, uart_rx_buffer, UART_RX_BUF_SIZE, &rx_len, K_MSEC(UART_RX_TIMEOUT_MS));
			
			LOG_DBG("e3000h_uart_read - end: ret=%d, rx_len=%d", ret, rx_len);
			LOG_HEXDUMP_DBG(uart_rx_buffer, UART_RX_BUF_SIZE, "UART RX");

			if (ret == 0)
			{
				LOG_DBG("Message received.");

				static uint8_t string_buffer[UART_RX_BUF_SIZE + 1];
				memcpy(string_buffer, uart_rx_buffer, rx_len);
				string_buffer[rx_len] = 0;

				LOG_INF("BARCODE: %s\n", string_buffer);

				struct barcode_t barcode = {
					.data = string_buffer,
					.length = rx_len,
				};

				e3000h_driver.barcode_handler(&barcode);

				break;
			}
		}

		// go sleep, will be wakeuped on next scann
		k_thread_suspend(k_current_get());
	}
}

K_THREAD_DEFINE(e3000h_barcode_reading_thread_tid, CONFIG_E3000H_BARCODE_DRIVER_THREAD_STACK_SIZE,
                e3000h_barcode_reading_entry_point, NULL, NULL, NULL,
                CONFIG_E3000H_BARCODE_DRIVER_THREAD_PRIORITY, 0, 0);

static int e3000h_uart_read(const struct device *uart, char* rx_buf, const int rx_buf_len, int *rx_len, k_timeout_t timeout_in_ms)
{
	LOG_DBG("e3000h_uart_read()");

	//struct e3000h_data *driver = dev->data;

	int bytes_read_counter = 0; 
	int poll_result;
	bool byte_received = false;

	char byte_buf[1];

    // compute the end time from the timeout
    uint64_t end = sys_clock_timeout_end_calc(timeout_in_ms);

    while (1) {

		// check timeout
		if (end < k_uptime_ticks() && byte_received == false)
		{	
			// timeout
			LOG_DBG("e3000h_uart_read(): timeout");
			return -1;
		}

		// check buffer space left
		if (bytes_read_counter > rx_buf_len)
		{
			// buffer full
			LOG_DBG("e3000h_uart_read(): buffer full");
			memcpy(rx_len, &bytes_read_counter, 1);
			return ENOBUFS;
		}
		
		poll_result = uart_poll_in(uart, byte_buf);

		switch (poll_result)
		{
			case 0:
				// byte received
				//LOG_HEXDUMP_DBG(byte_buf, 1, "e3000h_uart_read(): byte received.");
				byte_received = true;
				bytes_read_counter++;
				rx_buf[bytes_read_counter-1] = byte_buf[0];
				k_msleep(1); // IMPORTANT: sleep 1 msec to prevent receive bytes not correctly
				break;

			case -1:
				// no character was available to read
				//LOG_DBG("e3000h_uart_read(): no byte received.");
				if (byte_received)
				{
					//LOG_HEXDUMP_DBG(rx_buf, rx_buf_len, "e3000h_uart_read(): Message received.");
					memcpy(rx_len, &bytes_read_counter, 1);
					return 0;
				}
				break;

			default:
				LOG_DBG("Unexpected poll_result: %d.", poll_result);
				return poll_result;
		}

        // Wait for notification of state change
        //k_sem_take(obj->sem, timeout_in_ms);
    }

	// should never been reached
	return 0;
}

static void e3000h_uart_send(const struct device *uart, const uint8_t * data_ptr, uint32_t data_len)
{
	LOG_HEXDUMP_DBG(data_ptr, data_len, "uart tx buffer");
	
	int i=0;

	while (i < data_len)
	{
		uart_poll_out(uart, data_ptr[i++]);
	}
}

static int e3000h_start_decoding(const struct device *dev, barcode_handler_t barcode_handler)
{
	LOG_DBG("e3000h_start_decoding()");

	struct e3000h_data *driver = dev->data;

	device_busy_set(dev);

	driver->barcode_handler = barcode_handler;

	driver->state = AWAITING_BARCODE;	
	
	k_thread_resume(e3000h_barcode_reading_thread_tid);

	gpio_pin_set(driver->trigger->port, driver->trigger->pin, 0);

	return 0;
}

static int e3000h_stop_decoding(const struct device *dev)
{
	LOG_DBG("e3000h_stop_decoding()");

	struct e3000h_data *driver = dev->data;

    gpio_pin_set(driver->trigger->port, driver->trigger->pin, 1);		

	driver->state = IDLE;

	device_busy_clear(dev);

	return 0;
}

static enum message_type_enum get_message_type(const char* msg)
{
	LOG_DBG("message_type_enum()");

	if (msg == NULL)
	{
		LOG_DBG("msg is null");
		return UNKNOWN;
	}

	// ACK-Message is {0x04, 0xD0, 0x00, 0x00, 0xFF, 0x2C}
	if (msg[0] == 0x04
		&& msg[1] == 0xD0
		&& msg[2] == 0x00
		&& msg[3] == 0x00
		&& msg[4] == 0xFF
		&& msg[5] == 0x2C)
	{
		return DEVICE_ACK_MESSAGE;
	}
	
	// NACK resend Message is { 0x04, 0xD1, 0x00, 0x01, 0xFF, 0x25 }
	if (msg[0] == 0x04
		&& msg[1] == 0xD0
		&& msg[2] == 0x00
		&& msg[3] == 0x01
		&& msg[4] == 0xFF
		&& msg[5] == 0x25)
	{
		return DEVICE_NACK_RESEND_MESSAGE;
	}

	// NACK denied Message is { 0x04, 0xD1, 0x00, 0x06, 0xFF, 0x20 }
	if (msg[0] == 0x04
		&& msg[1] == 0xD0
		&& msg[2] == 0x00
		&& msg[3] == 0x06
		&& msg[4] == 0xFF
		&& msg[5] == 0x20)
	{
		return DEVICE_NACK_DENIED_MESSAGE;
	}

	// NACK bad context Message is { 0x04, 0xD1, 0x00, 0x02, 0xFF, 0x24 }
	if (msg[0] == 0x04
		&& msg[1] == 0xD0
		&& msg[2] == 0x00
		&& msg[3] == 0x02
		&& msg[4] == 0xFF
		&& msg[5] == 0x22)
	{
		return DEVICE_NACK_BAD_CONTEXT_MESSAGE;
	}

	return UNKNOWN;
}


static int e3000h_send_command(const struct device *dev, const char* command, const int command_length, const bool awaiting_ack_message)
{
	LOG_DBG("e3000h_send_command()");

	struct e3000h_data *driver = dev->data;

	clear_uart_rx_buffer();
	
	if (awaiting_ack_message)
	{
		// send command
		int retry_counter = 0;
		int ret_ack;
		int rx_len = 0;

		while(1)
		{
			LOG_HEXDUMP_DBG(command, command_length, "Send command and wait for ACK message");
			e3000h_uart_send(driver->uart, command, command_length);

			ret_ack = e3000h_uart_read(driver->uart, uart_rx_buffer, UART_RX_BUF_SIZE, &rx_len, K_MSEC(UART_RX_TIMEOUT_MS));
			LOG_DBG("e3000h_uart_read - end: ret=%d, rx_len=%d", ret_ack, rx_len);
			LOG_HEXDUMP_DBG(uart_rx_buffer, UART_RX_BUF_SIZE, "UART RX");

			if (ret_ack == 0)
			{	
				// check for ACK message from scan engine: 
				switch (get_message_type(uart_rx_buffer))
				{
					case DEVICE_ACK_MESSAGE:
						// this is the expected case
						LOG_DBG("ACK message received.");
						return 0;
						break;
					case DEVICE_NACK_RESEND_MESSAGE:
						LOG_WRN("NACK message received (resend)");						
						// In this case a resend of the command should be done.
						// But only if max retry count is not reached.
						if (retry_counter == CONFIG_E3000H_BARCODE_DRIVER_SEND_COMMAND_DEFAULT_MAX_RETRIES)
						{							
							// give up
							LOG_WRN("No NACK message received (resend). Give up after %d retries.", retry_counter);
							return -1;
						}
						break;
					case DEVICE_NACK_DENIED_MESSAGE:
						LOG_WRN("NACK message received (denied)");
						return -1;
						break;
					case DEVICE_NACK_BAD_CONTEXT_MESSAGE:
						LOG_WRN("NACK message received (bad context)");
						return -1;
						break;
					default:
						// ACK message expected. But unknown message received.
						LOG_HEXDUMP_WRN(uart_rx_buffer, rx_len, "ACK message expected. But unknown message received.");
						return -1;
						break;
				}
			}
			else
			{
				// No ACK message received.
				// In this case a resend of the command should be done.
				// But only if max retry count is not reached.
				LOG_WRN("No ACK message received.");
				if (retry_counter == CONFIG_E3000H_BARCODE_DRIVER_SEND_COMMAND_DEFAULT_MAX_RETRIES)
				{
					// give up
					LOG_WRN("No ACK message received. Give up after %d retries.", retry_counter);
					return -1;
				}
			}

			retry_counter++;
		}
	}
	else // not awaiting_ack_message
	{
		// send command
		LOG_HEXDUMP_DBG(command, command_length, "Send command NO wait for ACK message");
		e3000h_uart_send(driver->uart, command, command_length);
	}

	return 0;
}

static int e3000h_send_command_with_response(const struct device *dev, const char* command, const int command_length, char* response, int* response_length)
{
	LOG_DBG("e3000h_send_command_with_response()");

	struct e3000h_data *driver = dev->data;

	clear_uart_rx_buffer();
	
	// send command
	int retry_counter = 0;
	int ret_response;
	int rx_len = 0;

	while(1)
	{
		LOG_HEXDUMP_DBG(command, command_length, "Send command and wait for response message");
		e3000h_uart_send(driver->uart, command, command_length);

		ret_response = e3000h_uart_read(driver->uart, uart_rx_buffer, UART_RX_BUF_SIZE, &rx_len, K_MSEC(UART_RX_TIMEOUT_MS));
		LOG_DBG("e3000h_uart_read - end: ret=%d, rx_len=%d", ret_response, rx_len);
		LOG_HEXDUMP_DBG(uart_rx_buffer, UART_RX_BUF_SIZE, "UART RX");

		if (ret_response == 0)
		{	
			// check for response message from scan engine: 
			switch (get_message_type(uart_rx_buffer))
			{
				case DEVICE_NACK_RESEND_MESSAGE:
					LOG_WRN("NACK message received (resend)");						
					// In this case a resend of the command should be done.
					// But only if max retry count is not reached.
					if (retry_counter == CONFIG_E3000H_BARCODE_DRIVER_SEND_COMMAND_DEFAULT_MAX_RETRIES)
					{							
						// give up
						LOG_WRN("No NACK message received (resend). Give up after %d retries.", retry_counter);
						return -1;
					}
					break;
				case DEVICE_NACK_DENIED_MESSAGE:
					LOG_WRN("NACK message received (denied)");
					return -1;
					break;
				case DEVICE_NACK_BAD_CONTEXT_MESSAGE:
					LOG_WRN("NACK message received (bad context)");
					return -1;
					break;
				default:
					// If response message matches non of the ACK/NACK messages,
					// interprete the response message as the response of the command.
					memcpy(response, &uart_rx_buffer, rx_len);
					memcpy(response_length, &rx_len, 1);
					e3000h_send_command(dev, CMD_ACK, sizeof(CMD_ACK), false);
					return 0;
					break;
			}
		}
		else
		{
			// No response message received.
			// In this case a resend of the command should be done.
			// But only if max retry count is not reached.
			LOG_WRN("No ACK message received.");
			if (retry_counter == CONFIG_E3000H_BARCODE_DRIVER_SEND_COMMAND_DEFAULT_MAX_RETRIES)
			{
				// give up
				LOG_WRN("No response message received. Give up after %d retries.", retry_counter);
				return -1;
			}
		}

		retry_counter++;
	}
}

static void e3000h_init_gpio(const struct device *dev)
{
	struct e3000h_data *driver = dev->data;

	gpio_pin_configure(driver->trigger->port, driver->trigger->pin, GPIO_OUTPUT);
	gpio_pin_set(driver->trigger->port, driver->trigger->pin, 1);
	gpio_pin_configure(driver->reset->port, driver->reset->pin, GPIO_OUTPUT);
	gpio_pin_configure(driver->buzzer->port, driver->buzzer->pin, GPIO_INPUT);
	gpio_pin_configure(driver->led->port, driver->led->pin, GPIO_INPUT);
}

static void e3000h_init_uart(const struct device *dev)
{
	struct e3000h_data *driver = dev->data;

	struct uart_config cfg;

	uart_config_get(driver->uart, &cfg);

	LOG_DBG("UART-Config: baudrate=%d", cfg.baudrate);
	LOG_DBG("UART-Config: data_bits=%d", cfg.data_bits);
	LOG_DBG("UART-Config: flow_ctrl=%d", cfg.flow_ctrl);
	LOG_DBG("UART-Config: parity=%d", cfg.parity);
	LOG_DBG("UART-Config: stop_bits=%d", cfg.stop_bits);
}

struct command_t
{
	char* code;
	int length;
} command;


static void e3000h_barcode_init(const struct device *dev)
{
	struct e3000h_data *driver = dev->data;

	driver->state = IDLE;

	e3000h_send_command(dev, CMD_WAKEUP, sizeof(CMD_WAKEUP), false);
	//e3000h_send_command(dev, CMD_SET_FACTORY_DEFAULTS_0, sizeof(CMD_SET_FACTORY_DEFAULTS_0), true);
	e3000h_send_command(dev, CMD_PARAM_SET_SOFTWARE_HANDSHAKING_ENABLE, sizeof(CMD_PARAM_SET_SOFTWARE_HANDSHAKING_ENABLE), true);
	e3000h_send_command(dev, CMD_PARAM_SET_BAUD_RATE_9600, sizeof(CMD_PARAM_SET_BAUD_RATE_9600), true);
	e3000h_send_command(dev, CMD_SCAN_ENABLE, sizeof(CMD_SCAN_ENABLE), true);
	e3000h_send_command(dev, CMD_PARAM_SET_TERMINATOR_DISABLE, sizeof(CMD_PARAM_SET_TERMINATOR_DISABLE), true);
	e3000h_send_command(dev, CMD_PARAM_SET_POWER_MODE_LOW, sizeof(CMD_PARAM_SET_POWER_MODE_LOW), true);
	e3000h_send_command(dev, CMD_PARAM_SET_NO_READ_MESSAGE_DISABLE, sizeof(CMD_PARAM_SET_NO_READ_MESSAGE_DISABLE), true);
}

static int e3000h_init(const struct device *dev)
{
	LOG_DBG("e3000h_init()");

	struct e3000h_data *driver = dev->data;

	driver->uart = device_get_binding(DT_LABEL(DT_INST_PHANDLE(0, uart)));
	__ASSERT(driver->uart, "Failed to get device binding for UART instance");
	
	driver->trigger = &e3000h_trigger_gpio;
	driver->reset = &e3000h_reset_gpio;
	driver->buzzer = &e3000h_buzzer_gpio;
	driver->led = &e3000h_led_gpio;	

	e3000h_init_gpio(dev);
	e3000h_init_uart(dev);
	e3000h_barcode_init(dev);

	return 0;
}

static const struct barcode_driver_api e3000h_barcode_api = {
	.start_decoding = e3000h_start_decoding,
	.stop_decoding = e3000h_stop_decoding,
	.send_command = e3000h_send_command,
	.send_command_with_response = e3000h_send_command_with_response,
};

DEVICE_DT_INST_DEFINE(0,            			   		//instance_id	
					  &e3000h_init,               		//init_fn
			  	 	  NULL,                             //pm_control_fn
		      	 	  &e3000h_driver,                   //data_ptr
			  	 	  NULL,                             //cfg_ptr
		      	 	  APPLICATION,                      //level
			  	 	  CONFIG_APPLICATION_INIT_PRIORITY, //prio
		      	 	  &e3000h_barcode_api               //api_ptr
			  		 );
