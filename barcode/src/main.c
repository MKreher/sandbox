#include <assert.h>
#include <zephyr.h>
#include <kernel.h>
#include <stdio.h>
#include <event_manager.h>
#include <irq.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <string.h>

#define MODULE main

#include <caf/events/module_state_event.h>
#include <caf/events/button_event.h>
#include <caf/events/led_event.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE);

#define BUTTON_1_ID 0
#define BUTTON_2_ID 1
#define BUTTON_3_ID 2
#define BUTTON_4_ID 3

#define LED_1_ID 0
#define LED_2_ID 1
#define LED_3_ID 2
#define LED_4_ID 3

#define BARCODE_SCANNER_BUZZER_PIN 4
#define BARCODE_SCANNER_LED_PIN 3
#define BARCODE_SCANNER_RESET_PIN 28
#define BARCODE_SCANNER_TRIGGER_PIN 29

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

const static struct device *gpio0;

enum system_led_effects {
	LED_ON,
	LED_OFF,
    LED_BLINKY_SLOW,
    LED_BLINKY_FAST,
    LED_BLINKY_VERYFAST,
};

static const struct led_effect led_effect_def[5] = {
	[LED_ON]          = LED_EFFECT_LED_ON(LED_COLOR(255, 255, 255)),
	[LED_OFF]         = LED_EFFECT_LED_OFF(),
    [LED_BLINKY_SLOW] = LED_EFFECT_LED_BLINK(1000, LED_COLOR(255, 255, 255)),
    [LED_BLINKY_FAST] = LED_EFFECT_LED_BLINK(250, LED_COLOR(255, 255, 255)),
    [LED_BLINKY_VERYFAST] = LED_EFFECT_LED_BLINK(75, LED_COLOR(255, 255, 255)),
};


#define UART_RX_TIMEOUT_MS      100
#define UART_RX_BUF_SIZE		16
#define UART_RX_MSG_QUEUE_SIZE	8
struct uart_msg_queue_item {
	uint8_t bytes[UART_RX_BUF_SIZE];
	uint32_t length;
};


K_SEM_DEFINE(tx_done, 1, 1);
K_SEM_DEFINE(rx_disabled, 0, 1);

// UART RX primary buffers
uint8_t uart_buffer[UART_RX_BUF_SIZE];

// UART RX message queue
K_MSGQ_DEFINE(uart_rx_msgq, sizeof(struct uart_msg_queue_item), UART_RX_MSG_QUEUE_SIZE, 4);


#define APP_BARCODE_THREAD_STACK_SIZE 2048
#define APP_BARCODE_THREAD_PRIORITY 5

static bool f_do_scanning = false;

void barcode_thread_entry_point(int unused1, int unused2, int unused3)
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

				// Process the message here.
				static uint8_t string_buffer[UART_RX_BUF_SIZE + 1];
				memcpy(string_buffer, incoming_message.bytes, incoming_message.length);
				string_buffer[incoming_message.length] = 0;
				LOG_INF("BARCODE: %s\n", string_buffer);
				break;
			}
		}

		//k_thread_suspend(k_current_get());
	}
}

K_THREAD_DEFINE(app_barcode_thread_tid, APP_BARCODE_THREAD_STACK_SIZE,
                barcode_thread_entry_point, NULL, NULL, NULL,
                APP_BARCODE_THREAD_PRIORITY, 0, 0);
				static const struct device *dev_uart;

static void app_init_gpio()
{
        LOG_DBG("app_init_gpio()");

        gpio0 = device_get_binding("GPIO_0");

        gpio_pin_configure(gpio0, BARCODE_SCANNER_BUZZER_PIN, GPIO_INPUT);
        gpio_pin_configure(gpio0, BARCODE_SCANNER_LED_PIN, GPIO_INPUT);
        gpio_pin_configure(gpio0, BARCODE_SCANNER_RESET_PIN, GPIO_OUTPUT);
        gpio_pin_configure(gpio0, BARCODE_SCANNER_TRIGGER_PIN, GPIO_OUTPUT);
		
        gpio_pin_set(gpio0, BARCODE_SCANNER_TRIGGER_PIN, 1);
}

void app_uart_async_callback(const struct device *uart_dev, struct uart_event *evt, void *user_data)
{
	static struct uart_msg_queue_item new_message;

	switch (evt->type) {
		case UART_TX_DONE:
			LOG_DBG("app_uart_async_callback(): UART_TX_DONE");
			k_sem_give(&tx_done);
			break;

		case UART_TX_ABORTED:
			LOG_DBG("app_uart_async_callback(): UART_TX_ABORTED");
			break;

		case UART_RX_RDY:
			LOG_DBG("app_uart_async_callback(): UART_RX_RDY");
			memcpy(new_message.bytes, evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
			new_message.length = evt->data.rx.len;
			LOG_HEXDUMP_DBG(evt->data.rx.buf, UART_RX_BUF_SIZE, "uart rx buffer");
			LOG_HEXDUMP_DBG(new_message.bytes, new_message.length, "uart rx message");
			if (k_msgq_put(&uart_rx_msgq, &new_message, K_NO_WAIT) != 0){
				LOG_ERR("Error: Uart RX message queue full!\n");
			}
			break;
		
		case UART_RX_BUF_REQUEST:
			LOG_DBG("app_uart_async_callback(): UART_RX_BUF_REQUEST");
			// clear buffer
			//memset(uart_buf_next, 0, sizeof(uart_buf_next));
			//uart_rx_buf_rsp(dev_uart, uart_buf_next, UART_RX_BUF_SIZE);
			break;

		case UART_RX_BUF_RELEASED:
			LOG_DBG("app_uart_async_callback(): UART_RX_BUF_RELEASED");
			// clear buffer
			memset(&uart_buffer, 0, UART_RX_BUF_SIZE);
			break;

		case UART_RX_DISABLED:
			LOG_DBG("app_uart_async_callback(): UART_RX_DISABLED");
			memset(&uart_buffer, 0, UART_RX_BUF_SIZE);
			k_sem_give(&rx_disabled);
			break;

		case UART_RX_STOPPED:
			LOG_DBG("app_uart_async_callback(): UART_RX_STOPPED");
			memset(&uart_buffer, 0, UART_RX_BUF_SIZE);
			break;			

		default:
			LOG_DBG("app_uart_async_callback(): %d", evt->type);
			break;
	}
}

static void app_uart_init(void)
{
    LOG_DBG("app_uart_init()");

	dev_uart = device_get_binding("UART_1");
	if (dev_uart == NULL) {
		LOG_WRN("Failed to get UART binding\n");
		return;
	}

	uart_callback_set(dev_uart, app_uart_async_callback, NULL);
	uart_rx_enable(dev_uart, &uart_buffer, UART_RX_BUF_SIZE, UART_RX_TIMEOUT_MS);
}

void app_uart_send(const uint8_t * data_ptr, uint32_t data_len)
{
	LOG_DBG("app_uart_send()");

	if (k_sem_take(&tx_done, K_NO_WAIT) == 0) {
		uart_tx(dev_uart, data_ptr, data_len, SYS_FOREVER_MS);
	}

	LOG_DBG("app_uart_send() - end");
}

static void button_1_pressed_handler()
{
        LOG_DBG("Button 1 pressed.");

        enum system_led_effects system_led_effect = LED_ON;
        
        struct led_event *led_evt = new_led_event();

		led_evt->led_id = LED_1_ID;
		led_evt->led_effect = &led_effect_def[system_led_effect];
		EVENT_SUBMIT(led_evt);

		f_do_scanning = true;
		//k_thread_resume(app_barcode_thread_tid);

		k_msgq_purge(&uart_rx_msgq);
		memset(&uart_buffer, 0, UART_RX_BUF_SIZE);
		uart_rx_enable(dev_uart, &uart_buffer, UART_RX_BUF_SIZE, UART_RX_TIMEOUT_MS);

		gpio_pin_set(gpio0, BARCODE_SCANNER_TRIGGER_PIN, 0);
}

static void button_1_released_handler()
{
        LOG_DBG("Button 1 released.");

        enum system_led_effects system_led_effect = LED_OFF;

        struct led_event *led_evt = new_led_event();

		led_evt->led_id = LED_1_ID;
		led_evt->led_effect = &led_effect_def[system_led_effect];
		EVENT_SUBMIT(led_evt);

        gpio_pin_set(gpio0, BARCODE_SCANNER_TRIGGER_PIN, 1);
		
		// Das sleep verhindert, dass beim schnellen Button drücken UART disabled wird
		// während gerade ein Barcode empfangen wird. Das würde in einem abgeschnittenen
		// Barcode String enden.
		k_msleep(100);

		uart_rx_disable(dev_uart);

		f_do_scanning = false;

		k_msgq_purge(&uart_rx_msgq);

		k_thread_suspend(app_barcode_thread_tid);
}

static void button_2_pressed_handler()
{
        LOG_DBG("Button 2 pressed.");
        
        //uint8_t* data = CMD_SCAN_ENABLE;
        //int data_len = sizeof(CMD_SCAN_ENABLE);

		k_msleep(100);
		app_uart_send(CMD_WAKEUP, 1);
		k_msleep(10);
		app_uart_send(CMD_START_DECODE, 6);
}

static void button_2_released_handler()
{
        LOG_DBG("Button 2 released.");

		app_uart_send(CMD_STOP_DECODE, 6);
}


static int read_barcode(uint8_t *buf)
{
	return 0;
} 

void my_work_handler(struct k_work *work);
K_WORK_DEFINE(my_work, my_work_handler);

void my_timer_handler(struct k_timer *dummy);
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

bool f_do_read = false;

void my_work_handler(struct k_work *work)
{
	if (!f_do_read)
	{
		k_timer_stop(&my_timer);
	}

	uint8_t buf[16];
	int len = read_barcode(buf);

	if (len > 0)
	{
		LOG_DBG("Barcode read!!!");
		k_timer_stop(&my_timer);
	}

}

void my_timer_handler(struct k_timer *dummy)
{
	LOG_DBG("my_timer_handler()");
    k_work_submit(&my_work);
}

static void button_3_pressed_handler()
{
        LOG_DBG("Button 3 pressed.");

		f_do_read = true;
		k_timer_start(&my_timer, K_MSEC(0), K_MSEC(100));
}

static void button_3_released_handler()
{
        LOG_DBG("Button 3 released.");
		f_do_read = false;
		k_timer_stop(&my_timer);
}

static void button_4_pressed_handler()
{
        LOG_DBG("Button 4 pressed.");
}

static void button_4_released_handler()
{
        LOG_DBG("Button 4 released.");
}

static bool modul_event_handler(const struct event_header *eh)
{       
	if (is_module_state_event(eh)) {
                LOG_DBG("module_state_event()");
                //const struct module_state_event *event = cast_module_state_event(eh);

                return false;
        }

	if (is_button_event(eh)) {
                //LOG_DBG("button_event()- isr? %d", k_is_in_isr());
                
                const struct button_event *event = cast_button_event(eh);

                switch (event->key_id) {
                case BUTTON_1_ID:
                        if (event->pressed) {
                                  button_1_pressed_handler();
                        } else {
                                  button_1_released_handler();
                        }

                        break;
                case BUTTON_2_ID:
                        if (event->pressed) {
                                  button_2_pressed_handler();
                        } else {
                                  button_2_released_handler();
                        }

                        break;
                case BUTTON_3_ID:
                        if (event->pressed) {
                                  button_3_pressed_handler();
                        } else {
                                  button_3_released_handler();
                        }

                        break;
                case BUTTON_4_ID:
                        if (event->pressed) {
                                  button_4_pressed_handler();
                        } else {
                                  button_4_released_handler();
                        }

                        break;
                default:
                        LOG_WRN("Unknown key id: %d", event->key_id);
                	break;
                }

                return false;
        }

	// if event is unhandled, assertion error
	__ASSERT_NO_MSG(false);

	return false;
}

void app_barcode_init(void)
{
	LOG_DBG("app_barcode_init()");
	
	uart_rx_enable(dev_uart, &uart_buffer, UART_RX_BUF_SIZE, UART_RX_TIMEOUT_MS);

	app_uart_send(CMD_WAKEUP, sizeof(CMD_WAKEUP));
	k_msleep(10);
	//app_uart_send(CMD_PARAM_SET_SOFTWARE_HANDSHAKING_ENABLE, sizeof(CMD_PARAM_SET_SOFTWARE_HANDSHAKING_ENABLE));
	app_uart_send(CMD_PARAM_SET_SOFTWARE_HANDSHAKING_DISABLE, sizeof(CMD_PARAM_SET_SOFTWARE_HANDSHAKING_DISABLE));	
	app_uart_send(CMD_SCAN_ENABLE, sizeof(CMD_SCAN_ENABLE));	
	app_uart_send(CMD_PARAM_SET_TERMINATOR_DISABLE, sizeof(CMD_PARAM_SET_TERMINATOR_DISABLE));
	app_uart_send(CMD_PARAM_SET_POWER_MODE_LOW, sizeof(CMD_PARAM_SET_POWER_MODE_LOW));
	app_uart_send(CMD_PARAM_GET_BAUD_RATE, sizeof(CMD_PARAM_GET_BAUD_RATE));
	app_uart_send(CMD_PARAM_GET_TRIGGER_MODE, sizeof(CMD_PARAM_GET_TRIGGER_MODE));	
	
	uart_rx_disable(dev_uart);

	LOG_DBG("app_barcode_init() - end");
}


#define APP_BARCODE_SCANNER DT_ALIAS(barcode_scanner)

const struct device *barcode_scanner_dev;

const struct gpio_dt_spec e3000h_buzzer_gpio = GPIO_DT_SPEC_GET(APP_BARCODE_SCANNER, buzzer_gpios);

void init(void)
{

	barcode_scanner_dev = device_get_binding(DT_LABEL(APP_BARCODE_SCANNER));
	LOG_DBG("E3000H state:%d", barcode_scanner_dev->state->initialized);
	LOG_DBG("E3000H name:%s", barcode_scanner_dev->name);
	LOG_DBG("E3000H label:%s", DT_LABEL(APP_BARCODE_SCANNER));
	LOG_DBG("BUZZER pin: %d", e3000h_buzzer_gpio.pin);
	LOG_DBG("UART instance:%s", DT_LABEL(DT_PHANDLE(APP_BARCODE_SCANNER, uart)));
	/*
	gpio_pin_configure_dt(&signal, GPIO_OUTPUT_INACTIVE);
	gpio_pin_set(signal.port, signal.pin, 1);
	*/



    app_init_gpio();
	app_uart_init();
	app_barcode_init();
}

void show_ready(void)
{
	enum system_led_effects system_led_effect = LED_ON;

    struct led_event *led_evt = new_led_event();

	led_evt->led_id = LED_1_ID;
	led_evt->led_effect = &led_effect_def[system_led_effect];
	EVENT_SUBMIT(led_evt);
}

void main(void)
{
	if (event_manager_init()) {
		LOG_ERR("Event manager could not be initialized, rebooting...");
		k_sleep(K_SECONDS(5));
		sys_reboot(SYS_REBOOT_COLD);
	} else {
		init();
		module_set_state(MODULE_STATE_READY);
		show_ready();
	}
}

EVENT_LISTENER(MODULE, modul_event_handler);
EVENT_SUBSCRIBE(MODULE, module_state_event);
EVENT_SUBSCRIBE(MODULE, button_event);
