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

struct e3000h_data
{
	const struct gpio_dt_spec *trigger;
	const struct gpio_dt_spec *reset;
	const struct gpio_dt_spec *buzzer;
	const struct gpio_dt_spec *led;
	const struct device *uart;
	barcode_handler_t barcode_handler;
};

static struct e3000h_data e3000h_driver;

const struct gpio_dt_spec e3000h_trigger_gpio = GPIO_DT_SPEC_INST_GET(0, trigger_gpios);
const struct gpio_dt_spec e3000h_reset_gpio = GPIO_DT_SPEC_INST_GET(0, reset_gpios);
const struct gpio_dt_spec e3000h_buzzer_gpio = GPIO_DT_SPEC_INST_GET(0, buzzer_gpios);
const struct gpio_dt_spec e3000h_led_gpio = GPIO_DT_SPEC_INST_GET(0, led_gpios);

#define UART_RX_TIMEOUT_MS      100
#define UART_RX_BUF_SIZE		16
#define UART_RX_MSG_QUEUE_SIZE	8

struct uart_msg_queue_item
{
	uint8_t bytes[UART_RX_BUF_SIZE];
	uint32_t length;
};

// semaphores
K_SEM_DEFINE(tx_done, 1, 1);
K_SEM_DEFINE(rx_disabled, 0, 1);

// UART RX primary buffers
uint8_t uart_buffer[UART_RX_BUF_SIZE];

// UART RX message queue
K_MSGQ_DEFINE(uart_rx_msgq, sizeof(struct uart_msg_queue_item), UART_RX_MSG_QUEUE_SIZE, 4);

static bool f_do_scanning = false;

#define CMD_ACK_OPCODE  0xD0
#define CMD_NACK_OPCODE 0xD1

char CMD_ACK_RESPONSE_MESSAGE[] = { 0x04, 0xD0, 0x00, 0x00, 0xFF, 0x2C }; // Response ACK-Message from scan engine
char CMD_NACK_RESEND_RESPONSE_MESSAGE[] = { 0x04, 0xD1, 0x00, 0x01, 0xFF, 0x25 }; // Response NACK-Message (RESEND) from scan engine
char CMD_NACK_BAD_CONTEXT_RESPONSE_MESSAGE[] = { 0x04, 0xD1, 0x00, 0x02, 0xFF, 0x24 };  // Response NACK-Message (BAD_CONTEXT) from scan engine
char CMD_NACK_DENIED_RESPONSE_MESSAGE[] = { 0x04, 0xD1, 0x00, 0x06, 0xFF, 0x20 }; // Response NACK-Message (DENIED) from scan engine

// commands from host to the scan engine
char CMD_ACK[] = { 0x04, 0xD0, 0x04, 0x00, 0xFF, 0x28 };
char CMD_NACK_RESEND[] = { 0x05, 0xD1, 0x04, 0x00, 0x01, 0xFF, 0x25 };
char CMD_NACK_BAD_CONTEXT[] = { 0x05, 0xD1, 0x04, 0x00, 0x02, 0xFF, 0x24 };
char CMD_NACK_DENIED[] = { 0x05, 0xD1, 0x04, 0x00, 0x06, 0xFF, 0x20 };

char CMD_DECODE_DATE[] = { 0x00 };

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

char CMD_PARAM_GET_BAUD_RATE[] = { 0x05, 0xC7, 0x04, 0x00, 0x9C, 0xFE, 0x94 };

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

static void e3000h_barcode_module_reading_entry_point(int unused1, int unused2, int unused3)
{	
    struct uart_msg_queue_item incoming_message;

	while (1)
	{
		while (f_do_scanning)
		{
			LOG_DBG("do_scanning...");
			int ret = k_msgq_get(&uart_rx_msgq, &incoming_message, K_MSEC(50));

			if (ret == 0)
			{
				LOG_DBG("Message received.");

				static uint8_t string_buffer[UART_RX_BUF_SIZE + 1];
				memcpy(string_buffer, incoming_message.bytes, incoming_message.length);
				string_buffer[incoming_message.length] = 0;

				LOG_INF("BARCODE: %s\n", string_buffer);

				struct barcode_t barcode = {
					.data = string_buffer,
					.length = incoming_message.length,
				};

				e3000h_driver.barcode_handler(&barcode);

				break;
			}
		}

		//k_thread_suspend(k_current_get());
	}
}

K_THREAD_DEFINE(e3000h_barcode_module_reading_thread_tid, CONFIG_E3000H_BARCODE_DRIVER_THREAD_STACK_SIZE,
                e3000h_barcode_module_reading_entry_point, NULL, NULL, NULL,
                CONFIG_E3000H_BARCODE_DRIVER_THREAD_PRIORITY, 0, 0);


static void e3000h_uart_send(const struct device *dev, const uint8_t * data_ptr, uint32_t data_len)
{
	struct e3000h_data *driver = dev->data;

	if (k_sem_take(&tx_done, K_NO_WAIT) == 0)
	{
		uart_tx(driver->uart, data_ptr, data_len, SYS_FOREVER_MS);
	}
}

static void e3000h_uart_async_callback(const struct device *uart_dev, struct uart_event *evt, void *user_data)
{
	static struct uart_msg_queue_item new_message;

	switch (evt->type)
	{
		case UART_TX_DONE:
			LOG_DBG("UART_TX_DONE");
			k_sem_give(&tx_done);
			break;

		case UART_TX_ABORTED:
			LOG_DBG("UART_TX_ABORTED");
			break;

		case UART_RX_RDY:
			LOG_DBG("UART_RX_RDY");
			if (f_do_scanning)
			{
				memcpy(new_message.bytes, evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
				new_message.length = evt->data.rx.len;
				LOG_HEXDUMP_DBG(evt->data.rx.buf, UART_RX_BUF_SIZE, "uart rx buffer");
				LOG_HEXDUMP_DBG(new_message.bytes, new_message.length, "uart rx message");
				if (k_msgq_put(&uart_rx_msgq, &new_message, K_NO_WAIT) != 0)
				{
					LOG_ERR("Error: Uart RX message queue full!\n");
				}
			}
			break;
		
		case UART_RX_BUF_REQUEST:
			LOG_DBG("UART_RX_BUF_REQUEST");
			break;

		case UART_RX_BUF_RELEASED:
			LOG_DBG("UART_RX_BUF_RELEASED");
			memset(&uart_buffer, 0, UART_RX_BUF_SIZE);
			break;

		case UART_RX_DISABLED:
			LOG_DBG("UART_RX_DISABLED");
			memset(&uart_buffer, 0, UART_RX_BUF_SIZE);
			k_sem_give(&rx_disabled);
			break;

		case UART_RX_STOPPED:
			LOG_DBG("UART_RX_STOPPED");
			memset(&uart_buffer, 0, UART_RX_BUF_SIZE);
			break;			

		default:
			LOG_DBG("app_uart_async_callback(): %d", evt->type);
			break;
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

	uart_callback_set(driver->uart, e3000h_uart_async_callback, NULL);
	uart_rx_enable(driver->uart, &uart_buffer, UART_RX_BUF_SIZE, UART_RX_TIMEOUT_MS);
}

static void e3000h_barcode_init(const struct device *dev)
{
	struct e3000h_data *driver = dev->data;

	uart_rx_enable(driver->uart, &uart_buffer, UART_RX_BUF_SIZE, UART_RX_TIMEOUT_MS);

	e3000h_uart_send(dev, CMD_WAKEUP, sizeof(CMD_WAKEUP));
	k_msleep(10);
	e3000h_uart_send(dev, CMD_PARAM_SET_SOFTWARE_HANDSHAKING_ENABLE, sizeof(CMD_PARAM_SET_SOFTWARE_HANDSHAKING_ENABLE));
	e3000h_uart_send(dev, CMD_SCAN_ENABLE, sizeof(CMD_SCAN_ENABLE));	
	e3000h_uart_send(dev, CMD_PARAM_SET_TERMINATOR_DISABLE, sizeof(CMD_PARAM_SET_TERMINATOR_DISABLE));
	e3000h_uart_send(dev, CMD_PARAM_SET_POWER_MODE_LOW, sizeof(CMD_PARAM_SET_POWER_MODE_LOW));
	
	uart_rx_disable(driver->uart);
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

static int e3000h_start_decoding(const struct device *dev, barcode_handler_t barcode_handler)
{
	LOG_DBG("e3000h_start_decoding()");

	struct e3000h_data *driver = dev->data;

	device_busy_set(dev);

	driver->barcode_handler = barcode_handler;

	f_do_scanning = true;
	
	k_thread_resume(e3000h_barcode_module_reading_thread_tid);

	k_msgq_purge(&uart_rx_msgq);
	memset(&uart_buffer, 0, UART_RX_BUF_SIZE);

	uart_rx_enable(driver->uart, &uart_buffer, UART_RX_BUF_SIZE, UART_RX_TIMEOUT_MS);

	gpio_pin_set(driver->trigger->port, driver->trigger->pin, 0);

	return 0;
}

static int e3000h_stop_decoding(const struct device *dev)
{
	LOG_DBG("e3000h_stop_decoding()");

	struct e3000h_data *driver = dev->data;

    gpio_pin_set(driver->trigger->port, driver->trigger->pin, 1);
		
	uart_rx_disable(driver->uart);

	f_do_scanning = false;

	k_msgq_purge(&uart_rx_msgq);

	k_thread_suspend(e3000h_barcode_module_reading_thread_tid);

	device_busy_clear(dev);

	return 0;
}

static e3000h_command_return_t e3000h_send_command(const struct device *dev, const e3000h_command_t command)
{
	LOG_DBG("e3000h_send_command()");
	return NULL;
}

static const struct barcode_driver_api e3000h_barcode_api = {
	.start_decoding = e3000h_start_decoding,
	.stop_decoding = e3000h_stop_decoding,
	.send_command = e3000h_send_command,
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
