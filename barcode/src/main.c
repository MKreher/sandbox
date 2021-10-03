#include <assert.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <stdio.h>
#include <event_manager.h>
#include <irq.h>

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

static void button_1_pressed_handler()
{
        LOG_DBG("Button 1 pressed.");


        enum system_led_effects system_led_effect = LED_ON;
        
        struct led_event *led_evt = new_led_event();

	led_evt->led_id = LED_1_ID;
	led_evt->led_effect = &led_effect_def[system_led_effect];
	EVENT_SUBMIT(led_evt);
}

static void button_1_released_handler()
{
        LOG_DBG("Button 1 released.");
 

        enum system_led_effects system_led_effect = LED_OFF;

        struct led_event *led_evt = new_led_event();

	led_evt->led_id = LED_1_ID;
	led_evt->led_effect = &led_effect_def[system_led_effect];
	EVENT_SUBMIT(led_evt);
}

static void button_2_pressed_handler()
{
        LOG_DBG("Button 2 pressed.");
}

static void button_2_released_handler()
{
        LOG_DBG("Button 2 released.");
}

static void button_3_pressed_handler()
{
        LOG_DBG("Button 3 pressed.");
}

static void button_3_released_handler()
{
        LOG_DBG("Button 3 released.");
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

void main(void)
{
	if (event_manager_init()) {
		LOG_ERR("Event manager could not be initialized, rebooting...");
		k_sleep(K_SECONDS(5));
		sys_reboot(SYS_REBOOT_COLD);
	} else {
                module_set_state(MODULE_STATE_READY);
	}
}

EVENT_LISTENER(MODULE, modul_event_handler);
EVENT_SUBSCRIBE(MODULE, module_state_event);
EVENT_SUBSCRIBE(MODULE, button_event);