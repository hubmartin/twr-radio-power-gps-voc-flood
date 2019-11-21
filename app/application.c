#include <application.h>

#define TEMPERATURE_UPDATE_SERVICE_INTERVAL (1 * 1000)
#define TEMPERATURE_PUB_INTERVAL (15 * 60 * 1000)
#define TEMPERATURE_PUB_DIFFERENCE 0.2f

#define VOC_TAG_UPDATE_INTERVAL (15 * 60 * 1000)

#define FLOOD_DETECTOR_NO_CHANGE_INTEVAL (15 * 60 * 1000)
#define FLOOD_DETECTOR_UPDATE_SERVICE_INTERVAL (1 * 1000)
#define FLOOD_DETECTOR_UPDATE_NORMAL_INTERVAL  (5 * 1000)

#define GPS_PUB_INTERVAL (15 * 1000)

// Time of next temperature report
bc_tick_t tick_temperature_report = 0;

// LED instance
bc_led_t led;

// Button instance
bc_button_t button;
// Time of button has press
bc_tick_t tick_start_button_press;
// Flag for button hold event
bool button_hold_event;
// Counters for button events
uint16_t button_click_count = 0;
uint16_t button_hold_count = 0;

// Thermometer instance
bc_tmp112_t tmp112;

// VOC Tag instance
bc_tag_voc_t tag_voc;

// Flood detector instance
bc_flood_detector_t flood_detector;

// GPS virtual LEDs
bc_led_t gps_led_r;
bc_led_t gps_led_g;

typedef struct
{
    uint8_t channel;
    float value;
    bc_tick_t next_pub;

} event_param_t;

event_param_t flood_detector_event_param = { .next_pub = 0 };
event_param_t gps_event_param = { .next_pub = 0 };


void flood_detector_event_handler(bc_flood_detector_t *self, bc_flood_detector_event_t event, void *event_param)
{
    bool is_alarm;
    event_param_t *param = (event_param_t *)event_param;


    if (event == BC_FLOOD_DETECTOR_EVENT_UPDATE)
    {
       is_alarm = bc_flood_detector_is_alarm(self);

       if ((is_alarm != param->value) || (param->next_pub < bc_scheduler_get_spin_tick()))
       {
           bc_radio_pub_bool("flood-detector/a/alarm", &is_alarm);

           param->value = is_alarm;
           param->next_pub = bc_scheduler_get_spin_tick() + FLOOD_DETECTOR_NO_CHANGE_INTEVAL;
       }
    }
}

void voc_tag_event_handler(bc_tag_voc_t *self, bc_tag_voc_event_t event, void *event_param)
{
    if (event == BC_TAG_VOC_EVENT_UPDATE)
    {
        uint16_t value;

        if (bc_tag_voc_get_tvoc_ppb(&tag_voc, &value))
        {
            int radio_tvoc = value;
            bc_radio_pub_int("voc-sensor/0:0/tvoc", &radio_tvoc);
        }

    }
}

// This function dispatches thermometer events
void tmp112_event_handler(bc_tmp112_t *self, bc_tmp112_event_t event, void *event_param)
{
    // Update event?
    if (event == BC_TMP112_EVENT_UPDATE)
    {
        float temperature;

        // Successfully read temperature?
        if (bc_tmp112_get_temperature_celsius(self, &temperature))
        {
            bc_log_info("APP: Temperature = %0.1f C", temperature);

            // Implicitly do not publish message on radio
            bool publish = false;

            // Is time up to report temperature?
            if (bc_tick_get() >= tick_temperature_report)
            {
                // Publish message on radio
                publish = true;
            }

            // Last temperature value used for change comparison
            static float last_published_temperature = NAN;

            // Temperature ever published?
            if (last_published_temperature != NAN)
            {
                // Is temperature difference from last published value significant?
                if (fabsf(temperature - last_published_temperature) >= TEMPERATURE_PUB_DIFFERENCE)
                {
                    bc_log_info("APP: Temperature change threshold reached");

                    // Publish message on radio
                    publish = true;
                }
            }

            // Publish message on radio?
            if (publish)
            {
                bc_log_info("APP: Publish temperature");

                // Publish temperature message on radio
                bc_radio_pub_temperature(BC_RADIO_PUB_CHANNEL_R1_I2C0_ADDRESS_ALTERNATE, &temperature);

                // Schedule next temperature report
                tick_temperature_report = bc_tick_get() + TEMPERATURE_PUB_INTERVAL;

                // Remember last published value
                last_published_temperature = temperature;
            }
        }
    }
    // Error event?
    else if (event == BC_TMP112_EVENT_ERROR)
    {
        bc_log_error("APP: Thermometer error");
    }
}


void gps_module_event_handler(bc_module_gps_event_t event, void *event_param)
{
    event_param_t *param = (event_param_t *)event_param;

    if (event == BC_MODULE_GPS_EVENT_START)
    {
        bc_log_info("APP: Event BC_MODULE_GPS_EVENT_START");

        bc_led_set_mode(&gps_led_g, BC_LED_MODE_ON);
    }
    else if (event == BC_MODULE_GPS_EVENT_STOP)
    {
        bc_log_info("APP: Event BC_MODULE_GPS_EVENT_STOP");

        bc_led_set_mode(&gps_led_g, BC_LED_MODE_OFF);
    }
    else if (event == BC_MODULE_GPS_EVENT_UPDATE)
    {
        bc_led_pulse(&gps_led_r, 50);

        /*
        bc_module_gps_time_t time;
        if (bc_module_gps_get_time(&time))
        {
        }*/

        bc_module_gps_position_t position;

        if (bc_module_gps_get_position(&position))
        {
            char buffer[64];
            snprintf(buffer, sizeof(buffer),
                "%03.5f, %03.5f", position.latitude, position.longitude);

            if(param->next_pub < bc_scheduler_get_spin_tick())
            {
                bc_radio_pub_string("gps/0:0/position", buffer);
                param->next_pub = bc_scheduler_get_spin_tick() + GPS_PUB_INTERVAL;
            }
        }

        /*
        bc_module_gps_altitude_t altitude;
        if (bc_module_gps_get_altitude(&altitude))
        {
        }
        */

        /*
        bc_module_gps_quality_t quality;
        if (bc_module_gps_get_quality(&quality))
        {
        }
        */


        bc_module_gps_invalidate();
    }
    else if (event == BC_MODULE_GPS_EVENT_ERROR)
    {
        bc_log_info("APP: Event BC_MODULE_GPS_EVENT_ERROR");
    }
}


// This function dispatches button events
void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{
    if (event == BC_BUTTON_EVENT_CLICK)
    {
        // Pulse LED for 100 milliseconds
        bc_led_pulse(&led, 100);

        // Increment press count
        button_click_count++;

        bc_log_info("APP: Publish button press count = %u", button_click_count);

        // Publish button message on radio
        bc_radio_pub_push_button(&button_click_count);
    }
    else if (event == BC_BUTTON_EVENT_HOLD)
    {
        // Pulse LED for 250 milliseconds
        bc_led_pulse(&led, 250);

        // Increment hold count
        button_hold_count++;

        bc_log_info("APP: Publish button hold count = %u", button_hold_count);

        // Publish message on radio
        bc_radio_pub_event_count(BC_RADIO_PUB_EVENT_HOLD_BUTTON, &button_hold_count);

        // Set button hold event flag
        button_hold_event = true;
    }
    else if (event == BC_BUTTON_EVENT_PRESS)
    {
        // Reset button hold event flag
        button_hold_event = false;

        tick_start_button_press = bc_tick_get();
    }
    else if (event == BC_BUTTON_EVENT_RELEASE)
    {
        if (button_hold_event)
        {
            int hold_duration = bc_tick_get() - tick_start_button_press;

            bc_log_info("APP: Publish button hold duration = %d", hold_duration);

            bc_radio_pub_value_int(BC_RADIO_PUB_VALUE_HOLD_DURATION_BUTTON, &hold_duration);
        }
    }
}

void application_init(void)
{
    // Initialize logging
    bc_log_init(BC_LOG_LEVEL_DUMP, BC_LOG_TIMESTAMP_ABS);

    // Initialize LED
    bc_led_init(&led, BC_GPIO_LED, false, false);
    bc_led_set_mode(&led, BC_LED_MODE_ON);

    // Initialize button
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize thermometer
    bc_tmp112_init(&tmp112, BC_I2C_I2C0, 0x49);
    bc_tmp112_set_event_handler(&tmp112, tmp112_event_handler, NULL);
    bc_tmp112_set_update_interval(&tmp112, TEMPERATURE_UPDATE_SERVICE_INTERVAL);

    // Initialize VOC
    bc_tag_voc_init(&tag_voc, BC_I2C_I2C0);
    bc_tag_voc_set_event_handler(&tag_voc, voc_tag_event_handler, NULL);
    bc_tag_voc_set_update_interval(&tag_voc, VOC_TAG_UPDATE_INTERVAL);

    // Initialize flood detector
    bc_flood_detector_init(&flood_detector, BC_FLOOD_DETECTOR_TYPE_LD_81_SENSOR_MODULE_CHANNEL_A);
    bc_flood_detector_set_event_handler(&flood_detector, flood_detector_event_handler, &flood_detector_event_param);
    bc_flood_detector_set_update_interval(&flood_detector, FLOOD_DETECTOR_UPDATE_SERVICE_INTERVAL);

    // Initilize GPS
    if (!bc_module_gps_init())
    {
        bc_log_error("APP: GPS Module initialization failed");
    }
    else
    {
        bc_module_gps_set_event_handler(gps_module_event_handler, &gps_event_param);
        bc_module_gps_start();
    }

    bc_led_init_virtual(&gps_led_r, BC_MODULE_GPS_LED_RED, bc_module_gps_get_led_driver(), 0);
    bc_led_init_virtual(&gps_led_g, BC_MODULE_GPS_LED_GREEN, bc_module_gps_get_led_driver(), 0);

    // Initialize radio
    bc_radio_init(BC_RADIO_MODE_NODE_SLEEPING);

    // Send radio pairing request
    bc_radio_pairing_request("power-sensor", VERSION);

    // Turn on LED for 2 seconds
    bc_led_pulse(&led, 2000);

}

void application_task(void)
{
    // Logging in action
    bc_log_debug("application_task run");

    // Plan next run this function after 1000 ms
    bc_scheduler_plan_current_from_now(1000);
}
