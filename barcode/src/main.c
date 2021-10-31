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

#include <e3000h.h>

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


static void barcode_handler(const struct barcode_t *barcode)
{
        LOG_DBG("barcode_handler()");

        LOG_DBG("Barcode received: %s (%d)", barcode->data, barcode->length);
}

static const struct device *e3000h_dev;

static void button_1_pressed_handler()
{
        LOG_DBG("Button 1 pressed.");

        enum system_led_effects system_led_effect = LED_ON;
        
        struct led_event *led_evt = new_led_event();

        led_evt->led_id = LED_1_ID;
        led_evt->led_effect = &led_effect_def[system_led_effect];
        EVENT_SUBMIT(led_evt);

        int ret = barcode_start_decoding(e3000h_dev, &barcode_handler);
}

static void button_1_released_handler()
{
        LOG_DBG("Button 1 released.");

        enum system_led_effects system_led_effect = LED_OFF;

        struct led_event *led_evt = new_led_event();

        led_evt->led_id = LED_1_ID;
        led_evt->led_effect = &led_effect_def[system_led_effect];
        EVENT_SUBMIT(led_evt);

        int ret = barcode_stop_decoding(e3000h_dev);
}

static void button_2_pressed_handler()
{
        LOG_DBG("Button 2 pressed.");
}

static void button_2_released_handler()
{
        LOG_DBG("Button 2 released.");
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

void init(void)
{
	e3000h_dev = device_get_binding("E3000H");

	if (e3000h_dev == NULL)
	{
                LOG_ERR("Failed to get E3000H device binding\n");
                return;
	}
        else
        {
		LOG_INF("Found E3000H device binding\n");
	}

	//int ret = barcode_start_decoding(e3000h_dev, &barcode_handler);
	//k_msleep(500);
	//ret = barcode_stop_decoding(e3000h_dev);
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
	}
        else
        {
		init();
		module_set_state(MODULE_STATE_READY);
		show_ready();
	}
}

EVENT_LISTENER(MODULE, modul_event_handler);
EVENT_SUBSCRIBE(MODULE, module_state_event);
EVENT_SUBSCRIBE(MODULE, button_event);
