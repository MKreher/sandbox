#include <assert.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <stdio.h>
#include <event_manager.h>
#include <irq.h>

#define MODULE ui_module

#include <caf/events/module_state_event.h>
#include <caf/events/button_event.h>
#include <caf/events/led_event.h>

#include "modules_common.h"
#include "events/ui_module_event.h"


#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_UI_MODULE_LOG_LEVEL);

/* Forward declarations */
static int init();

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

bool led3_is_blinky = false;
bool led4_is_blinky = false;

//#define ADV_STATUS_LED DK_LED1
//#define CON_STATUS_LED DK_LED2
//#define LED_CAPS_LOCK  DK_LED3
//#define NFC_LED	 DK_LED4


static void button_1_pressed_handler()
{
        LOG_DBG("Button 1 pressed.");


        enum system_led_effects system_led_effect = LED_ON;
        
        struct led_event *led_evt = new_led_event();

	led_evt->led_id = LED_1_ID;
	led_evt->led_effect = &led_effect_def[system_led_effect];
	EVENT_SUBMIT(led_evt);


        struct ui_module_event *ui_evt = new_ui_module_event();

	ui_evt->type = UI_EVT_BARCODE_READER_TRIGGER;
	ui_evt->data.ui.button_number = 0;
        ui_evt->data.ui.pressed = true;
        ui_evt->data.ui.timestamp = k_uptime_get();
	EVENT_SUBMIT(ui_evt);
}

static void button_1_released_handler()
{
        LOG_DBG("Button 1 released.");
 

        enum system_led_effects system_led_effect = LED_OFF;

        struct led_event *led_evt = new_led_event();

	led_evt->led_id = LED_1_ID;
	led_evt->led_effect = &led_effect_def[system_led_effect];
	EVENT_SUBMIT(led_evt);


        struct ui_module_event *ui_evt = new_ui_module_event();

	ui_evt->type = UI_EVT_BARCODE_READER_TRIGGER;
	ui_evt->data.ui.button_number = 0;
        ui_evt->data.ui.pressed = false;
        ui_evt->data.ui.timestamp = k_uptime_get();
	EVENT_SUBMIT(ui_evt);
}

static void button_2_pressed_handler()
{
        LOG_DBG("Button 2 pressed.");
        
        enum system_led_effects system_led_effect = LED_ON;

        struct led_event *event = new_led_event();

	event->led_id = LED_2_ID;
	event->led_effect = &led_effect_def[system_led_effect];
	EVENT_SUBMIT(event);
}

static void button_2_released_handler()
{
        LOG_DBG("Button 2 released.");

        enum system_led_effects system_led_effect = LED_OFF;

        struct led_event *event = new_led_event();

	event->led_id = LED_2_ID;
	event->led_effect = &led_effect_def[system_led_effect];
	EVENT_SUBMIT(event);
}

static void button_3_pressed_handler()
{
        LOG_DBG("Button 3 pressed.");
}

static void button_3_released_handler()
{
        LOG_DBG("Button 3 released.");

        enum system_led_effects system_led_effect;

        if (led3_is_blinky) {
                system_led_effect = LED_OFF;
        } else {
                system_led_effect = LED_BLINKY_SLOW;
        }
        
        struct led_event *event = new_led_event();

	event->led_id = LED_3_ID;
	event->led_effect = &led_effect_def[system_led_effect];
	EVENT_SUBMIT(event);

        led3_is_blinky = !led3_is_blinky;
}

static void button_4_pressed_handler()
{
        LOG_DBG("Button 4 pressed.");
}

static void button_4_released_handler()
{
        LOG_DBG("Button 4 released.");

        struct ui_module_event *ui_evt = new_ui_module_event();

	ui_evt->type = UI_EVT_DISPLAY_SHOW;
	ui_evt->data.ui.button_number = 0;
        ui_evt->data.ui.pressed = false;
        ui_evt->data.ui.timestamp = k_uptime_get();
	EVENT_SUBMIT(ui_evt);
}

static bool modul_event_handler(const struct event_header *eh)
{       
	if (is_module_state_event(eh)) {
                LOG_DBG("module_state_event() - isr? %d", k_is_in_isr());
                
                const struct module_state_event *event = cast_module_state_event(eh);
                
                // check if the main module reported state READY
		if (check_state(event, MODULE_ID(main), MODULE_STATE_READY)) {
			static bool initialized;

			__ASSERT_NO_MSG(!initialized);

			initialized = true;

                        int err = init();

                        if (!err) {
                                module_set_state(MODULE_STATE_READY);
                        } else {
                                module_set_state(MODULE_STATE_ERROR);
                        }

                        return false;
                }

                return false;
        }
        
	if (is_button_event(eh)) {
                LOG_DBG("button_event()- isr? %d", k_is_in_isr());
                
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


static int init(void)
{
        LOG_DBG("init()");

	int err;
        
        /*
	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
                return err;
	}
        */

        return 0;
}


EVENT_LISTENER(MODULE, modul_event_handler);
EVENT_SUBSCRIBE(MODULE, module_state_event);
//EVENT_SUBSCRIBE_EARLY(MODULE, app_module_event);
//EVENT_SUBSCRIBE(MODULE, ble_module_event);
EVENT_SUBSCRIBE(MODULE, button_event);
//EVENT_SUBSCRIBE(MODULE, power_down_event);
//EVENT_SUBSCRIBE(MODULE, wake_up_event);
