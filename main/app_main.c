/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_timer.h"
// #include "esp32/rom/ets_sys.h"
#include "sys/unistd.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

#include "usb/usb_host.h"
#include "errno.h"

#include "hid_host.h"
#include "hid_usage_keyboard.h"
#include "hid_usage_mouse.h"

#include "driver/ledc.h"
#include <sys/time.h>

struct timeval tv_now;
struct timeval tv_next;
int64_t time_us_start;
int64_t time_us_end;

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (38) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (2047) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

#define READY_TO_UNINSTALL          (HOST_NO_CLIENT | HOST_ALL_FREE)
#define USBHID_QUEUE_SIZE       20
#define USBHID_MAX_DELAY        512

/* Main char symbol for ENTER key */
#define KEYBOARD_ENTER_MAIN_CHAR    '\r'
/* When set to 1 pressing ENTER will be extending with LineFeed during serial debug output */
#define KEYBOARD_ENTER_LF_EXTEND    1

#define HID_DEMO_TAG "HID_DEMO"

#define APP_QUIT_PIN                GPIO_NUM_0
#define APP_QUIT_PIN_POLL_MS        500

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool send_volum_up = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "Rubbish Mouse"

int curr_dev_id = 0;
const char* ble_device_name[2] = {"Rubbish Mouse", "Rubbish Mouse-2"};
esp_bd_addr_t rand_addr = {0xc3,0x11,0x11,0x11,0x11,0xc3};//{0xab,0x91,0x28,0x26,0xf3,0xad};
esp_bd_addr_t rand_addr2 = {0xc3,0x91,0x28,0x26,0xf4,0xc3};

static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

// queue
static QueueHandle_t g_usbhid_queue;
static QueueHandle_t g_blehid_queue;

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_RANDOM,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


/**
 * @brief Application Event from USB Host driver
 *
 */
typedef enum {
    HOST_NO_CLIENT = 0x1,
    HOST_ALL_FREE = 0x2,
    DEVICE_CONNECTED = 0x4,
    DEVICE_DISCONNECTED = 0x8,
    DEVICE_ADDRESS_MASK = 0xFF0,
} app_event_t;

/**
 * @brief Key event
 *
 */
typedef struct {
    enum key_state {
        KEY_STATE_PRESSED = 0x00,
        KEY_STATE_RELEASED = 0x01
    } state;
    uint8_t modifier;
    uint8_t key_code;
} key_event_t;

#define USB_EVENTS_TO_WAIT      (DEVICE_CONNECTED | DEVICE_ADDRESS_MASK | DEVICE_DISCONNECTED)

static const char *TAG = "HID Host";
static EventGroupHandle_t usb_flags;
static bool hid_device_connected = false;

hid_host_interface_handle_t keyboard_handle = NULL;
hid_host_interface_handle_t mouse_handle = NULL;

/**
 * @brief Scancode to ascii table
 */
const uint8_t keycode2ascii [57][2] = {
    {0, 0}, /* HID_KEY_NO_PRESS        */
    {0, 0}, /* HID_KEY_ROLLOVER        */
    {0, 0}, /* HID_KEY_POST_FAIL       */
    {0, 0}, /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'}, /* HID_KEY_A               */
    {'b', 'B'}, /* HID_KEY_B               */
    {'c', 'C'}, /* HID_KEY_C               */
    {'d', 'D'}, /* HID_KEY_D               */
    {'e', 'E'}, /* HID_KEY_E               */
    {'f', 'F'}, /* HID_KEY_F               */
    {'g', 'G'}, /* HID_KEY_G               */
    {'h', 'H'}, /* HID_KEY_H               */
    {'i', 'I'}, /* HID_KEY_I               */
    {'j', 'J'}, /* HID_KEY_J               */
    {'k', 'K'}, /* HID_KEY_K               */
    {'l', 'L'}, /* HID_KEY_L               */
    {'m', 'M'}, /* HID_KEY_M               */
    {'n', 'N'}, /* HID_KEY_N               */
    {'o', 'O'}, /* HID_KEY_O               */
    {'p', 'P'}, /* HID_KEY_P               */
    {'q', 'Q'}, /* HID_KEY_Q               */
    {'r', 'R'}, /* HID_KEY_R               */
    {'s', 'S'}, /* HID_KEY_S               */
    {'t', 'T'}, /* HID_KEY_T               */
    {'u', 'U'}, /* HID_KEY_U               */
    {'v', 'V'}, /* HID_KEY_V               */
    {'w', 'W'}, /* HID_KEY_W               */
    {'x', 'X'}, /* HID_KEY_X               */
    {'y', 'Y'}, /* HID_KEY_Y               */
    {'z', 'Z'}, /* HID_KEY_Z               */
    {'1', '!'}, /* HID_KEY_1               */
    {'2', '@'}, /* HID_KEY_2               */
    {'3', '#'}, /* HID_KEY_3               */
    {'4', '$'}, /* HID_KEY_4               */
    {'5', '%'}, /* HID_KEY_5               */
    {'6', '^'}, /* HID_KEY_6               */
    {'7', '&'}, /* HID_KEY_7               */
    {'8', '*'}, /* HID_KEY_8               */
    {'9', '('}, /* HID_KEY_9               */
    {'0', ')'}, /* HID_KEY_0               */
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /* HID_KEY_ENTER           */
    {0, 0}, /* HID_KEY_ESC             */
    {'\b', 0}, /* HID_KEY_DEL             */
    {0, 0}, /* HID_KEY_TAB             */
    {' ', ' '}, /* HID_KEY_SPACE           */
    {'-', '_'}, /* HID_KEY_MINUS           */
    {'=', '+'}, /* HID_KEY_EQUAL           */
    {'[', '{'}, /* HID_KEY_OPEN_BRACKET    */
    {']', '}'}, /* HID_KEY_CLOSE_BRACKET   */
    {'\\', '|'}, /* HID_KEY_BACK_SLASH      */
    {'\\', '|'}, /* HID_KEY_SHARP           */  // HOTFIX: for NonUS Keyboards repeat HID_KEY_BACK_SLASH
    {';', ':'}, /* HID_KEY_COLON           */
    {'\'', '"'}, /* HID_KEY_QUOTE           */
    {'`', '~'}, /* HID_KEY_TILDE           */
    {',', '<'}, /* HID_KEY_LESS            */
    {'.', '>'}, /* HID_KEY_GREATER         */
    {'/', '?'} /* HID_KEY_SLASH           */
};

/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto)
{
    static hid_protocol_t prev_proto_output = HID_PROTOCOL_NONE;

    if (prev_proto_output != proto) {
        prev_proto_output = proto;
        printf("\r\n");
        if (proto == HID_PROTOCOL_MOUSE) {
            printf("Mouse\r\n");
        }
        if (proto == HID_PROTOCOL_KEYBOARD) {
            printf("Keyboard\r\n");
        }
        fflush(stdout);
    }
}

/**
 * @brief HID Keyboard modifier verification for capitalization application (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier)
{
    if ((modifier && HID_LEFT_SHIFT) ||
            (modifier && HID_RIGHT_SHIFT)) {
        return true;
    }
    return false;
}

/**
 * @brief HID Keyboard get char symbol from key code
 *
 * @param[in] modifier  Keyboard modifier data
 * @param[in] key_code  Keyboard key code
 * @param[in] key_char  Pointer to key char data
 *
 * @return true  Key scancode converted successfully
 * @return false Key scancode unknown
 */
static inline bool hid_keyboard_get_char(uint8_t modifier,
        uint8_t key_code,
        unsigned char *key_char)
{
    uint8_t mod = (hid_keyboard_is_modifier_shift(modifier)) ? 1 : 0;

    if ((key_code >= HID_KEY_A) && (key_code <= HID_KEY_SLASH)) {
        *key_char = keycode2ascii[key_code][mod];
    } else {
        // All other key pressed
        return false;
    }

    return true;
}

/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char)
{
    if (!!key_char) {
        putchar(key_char);
#if (KEYBOARD_ENTER_LF_EXTEND)
        if (KEYBOARD_ENTER_MAIN_CHAR == key_char) {
            putchar('\n');
        }
#endif // KEYBOARD_ENTER_LF_EXTEND
        fflush(stdout);
    }
}

/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event)
{
    unsigned char key_char;

    hid_print_new_device_report_header(HID_PROTOCOL_KEYBOARD);

    if (KEY_STATE_PRESSED == key_event->state) {
        if (hid_keyboard_get_char(key_event->modifier,
                                  key_event->key_code, &key_char)) {

            hid_keyboard_print_char(key_char);

        }
    }
}

/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src,
                             uint8_t key,
                             unsigned int length)
{
    for (unsigned int i = 0; i < length; i++) {
        if (src[i] == key) {
            return true;
        }
    }
    return false;
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data, const int length)
{
    hid_keyboard_input_report_boot_t *kb_report = (hid_keyboard_input_report_boot_t *)data;

    ESP_LOGI(TAG, "HID Device report length: %d; buffer len: %d", sizeof(hid_keyboard_input_report_boot_t), length);

    if (length < sizeof(hid_keyboard_input_report_boot_t)) {
        return;
    }

    static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = { 0 };
    key_event_t key_event;

    for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {

        // key has been released verification
        if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
                !key_found(kb_report->key, prev_keys[i], HID_KEYBOARD_KEY_MAX)) {
            key_event.key_code = prev_keys[i];
            key_event.modifier = 0;
            key_event.state = KEY_STATE_RELEASED;
            key_event_callback(&key_event);
        }

        // key has been pressed verification
        if (kb_report->key[i] > HID_KEY_ERROR_UNDEFINED &&
                !key_found(prev_keys, kb_report->key[i], HID_KEYBOARD_KEY_MAX)) {
            key_event.key_code = kb_report->key[i];
            key_event.modifier = kb_report->modifier.val;
            key_event.state = KEY_STATE_PRESSED;
            key_event_callback(&key_event);
        }
    }

    memcpy(prev_keys, &kb_report->key, HID_KEYBOARD_KEY_MAX);
}

static void switch_device()
{
    ESP_LOGI(HID_DEMO_TAG, "BLE Device Switch");
    curr_dev_id = 1-curr_dev_id;
    if (curr_dev_id) {esp_ble_gap_set_rand_addr(rand_addr2);}
    else {esp_ble_gap_set_rand_addr(rand_addr);}
    esp_ble_gap_set_device_name(ble_device_name[curr_dev_id]);//HIDD_DEVICE_NAME);
    esp_ble_gap_config_adv_data(&hidd_adv_data);
    if (sec_conn)
    {
        esp_ble_gatts_close(hidd_le_env.gatt_if, hid_conn_id);
        sec_conn = false;
    }
    esp_ble_gap_start_advertising(&hidd_adv_params);
}

hid_mouse_input_report_queue_t hold_hid_reports;
uint8_t tmp_state;
uint8_t need_send = 1;
uint64_t _last_time_stamp = 0;

/**
 * @brief USB HID Host Mouse Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_mouse_report_callback(const uint8_t *const data, const int length)
{
    // gettimeofday(&tv_now, NULL);
    // time_us_start = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    hid_mouse_input_report_boot_t *mouse_report = (hid_mouse_input_report_boot_t *)data;

    // ESP_LOGI(TAG, "HID Device report length: %d; buffer len: %d", sizeof(hid_mouse_input_report_boot_t), length);

    if (length < sizeof(hid_mouse_input_report_boot_t)) {
        return;
    }

    // static int x_pos = 0;
    // static int y_pos = 0;

    // Calculate absolute position from displacement
    // x_pos += mouse_report->x_displacement;
    // y_pos += mouse_report->y_displacement;
    /* hid_print_new_device_report_header(HID_PROTOCOL_MOUSE);

    printf("X: %06d\tY: %06d\tZ: %06d\t|%c|%c|%c|%c|%c|\r",
           x_pos, y_pos, mouse_report->z_displacement,
           (mouse_report->buttons.button1 ? 'o' : ' '),
           (mouse_report->buttons.button3 ? 'o' : ' '),
           (mouse_report->buttons.button2 ? 'o' : ' '),
           (mouse_report->buttons.button4 ? 'o' : ' '),
           (mouse_report->buttons.button5 ? 'o' : ' '));
    fflush(stdout); */
    if (mouse_report->buttons.button5) {switch_device(); return;}
    if (sec_conn) {
        // ESP_LOGI(HID_DEMO_TAG, "Send the movement");
        //uint8_t key_vaule = {HID_KEY_A};
        //esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
        // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
        // ESP_LOGI(TAG, "Leftclick: %d\r", mouse_report->buttons.val);
        /*
        #ifdef DELL
        int16_t *temp_pot;
        temp_pot = &mouse_report->displacement[0];
        int16_t x_disp = *temp_pot;
        // x_disp = x_disp >> 4;
        x_disp = (x_disp & 0x0800) ? (x_disp | 0xf800) : (x_disp & 0x07ff);
        temp_pot = &mouse_report->displacement[1];
        int16_t y_disp = *temp_pot;
        y_disp = y_disp >> 4;
        y_disp = (y_disp & 0x0800) ? (y_disp | 0xf800) : (y_disp & 0x07ff);
        esp_hidd_send_mouse_value(hid_conn_id, mouse_report->buttons.val, x_disp, y_disp, mouse_report->z_displacement);
        #endif
        */
        // #ifdef LOGITECH
        // uint16_t mouse_button = mouse_report->buttons.val >> 8;
        /*
        hid_mouse_input_report_boot_t evt;
        memcpy(&evt, mouse_report, sizeof(hid_mouse_input_report_boot_t));
        if (xQueueSend(g_usbhid_queue, &evt, USBHID_MAX_DELAY) != pdTRUE) {
            ESP_LOGW(TAG, "Send receive queue fail");
        }
        */
        hold_hid_reports.buttons.val = hold_hid_reports.buttons.val | mouse_report->buttons.val;
        hold_hid_reports.displacement[0] += mouse_report->displacement[0];
        hold_hid_reports.displacement[1] += mouse_report->displacement[1];
        hold_hid_reports.z_displacement += mouse_report->z_displacement;
        hold_hid_reports.steps += 1;
        // _last_time_stamp = esp_timer_get_time();
        if (xQueueReceive(g_blehid_queue, &tmp_state, 0) == pdTRUE)
        {
            // hold_hid_reports._time_stamp = esp_timer_get_time() - _last_time_stamp;
            if (xQueueSend(g_usbhid_queue, &hold_hid_reports, 0) != pdTRUE) 
            {
                // ESP_LOGW(TAG, "Send receive queue fail");
                xQueueSend(g_blehid_queue, &tmp_state, 0);
            }
            else
            {
                hold_hid_reports.buttons.val = 0;
                hold_hid_reports.displacement[0] = 0;
                hold_hid_reports.displacement[1] = 0;
                hold_hid_reports.z_displacement = 0;
                hold_hid_reports.steps = 0;
            }
        }
        // vTaskDelay(1 / portTICK_PERIOD_MS);
        // ESP_LOGW(TAG, "C");
    }
}

/*
static void ble_hid_proceed()
{
    hid_mouse_input_report_boot_t evt;
    while (xQueueReceive(g_usbhid_queue, &evt, portMAX_DELAY) == pdTRUE)
    {
        while (send_lock) 
        {
            //delay hold and wait;
        }
        temp_ble_button = temp_ble_button | evt.buttons.val;
        temp_ble_displacement_x += evt.displacement[0];
        temp_ble_displacement_y += evt.displacement[1];
        temp_ble_wheel += evt.z_displacement;
        if (!need_send)
        {
            need_send = 1;
            if (xQueueSend(g_blehid_queue, &need_send, USBHID_MAX_DELAY) != pdTRUE) {
                ESP_LOGW(TAG, "Send receive queue fail");
            }
        }
        // if (esp_hidd_send_mouse_value(hid_conn_id, (uint8_t)mouse_report->buttons.val, mouse_report->displacement[0], mouse_report->displacement[1], mouse_report->z_displacement))
        // int64_t time_us_start = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        // int64_t time_us_end = (int64_t)tv_next.tv_sec * 1000000L + (int64_t)tv_next.tv_usec; 
        // ESP_LOGW(TAG, "USB Time: %lld", time_us_end - time_us_start);
        // ESP_LOGW(TAG, "USB");
    }
}
*/

static void ble_hid_sender()
{
    uint8_t buffer[HID_MOUSE_IN_RPT_LEN];
    hid_mouse_input_report_queue_t evt;
    // need_send = 1;
    // initial
    if (xQueueSend(g_blehid_queue, &need_send, USBHID_MAX_DELAY) != pdTRUE) 
        { ESP_LOGW(TAG, "Send receive queue fail"); }
    while (xQueueReceive(g_usbhid_queue, &evt, portMAX_DELAY) == pdTRUE)
    {// wait for the packing of data from usb
        // vTaskSuspendAll ();
        buffer[0] = evt.buttons.val;                    // Buttons
        buffer[1] = evt.z_displacement;                 // Wheel
        buffer[2] = evt.displacement[0];                // X
        buffer[3] = evt.displacement[0] >> 8;           // X lower
        buffer[4] = evt.displacement[1];                // Y
        buffer[5] = evt.displacement[1] >> 8;           // Y lower
        // ESP_LOGI(HID_LE_PRF_TAG, "Leftclick: %d\r", buffer[0]);
        hid_dev_send_report(hidd_le_env.gatt_if, hid_conn_id,
                            HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT, HID_MOUSE_IN_RPT_LEN, buffer);
        // send calling queue back to usb func and ask for next commands for send
        // ESP_LOGW(TAG, "Current BLE Include %d reports, overall %lld us", evt.steps, evt._time_stamp);
        // _last_time_stamp = evt._time_stamp;
        // wait here for rough 0.5ms for balance
        usleep(500);
        if (xQueueSend(g_blehid_queue, &need_send, USBHID_MAX_DELAY) != pdTRUE)
            { ESP_LOGW(TAG, "Send receive queue fail"); }
        // xTaskResumeAll();
    }
}

/**
 * @brief USB HID Host event callback. Handle such event as device connection and removing
 *
 * @param[in] event  HID device event
 * @param[in] arg    Pointer to arguments, does not used
 */
static void hid_host_event_callback(const hid_host_event_t *event, void *arg)
{
    if (event->event == HID_DEVICE_CONNECTED) {
        // Obtained USB device address is placed after application events
        xEventGroupSetBits(usb_flags, DEVICE_CONNECTED | (event->device.address << 4));
    } else if (event->event == HID_DEVICE_DISCONNECTED) {
        xEventGroupSetBits(usb_flags, DEVICE_DISCONNECTED);
    }
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] event  HID interface event
 * @param[in] arg    Pointer to arguments, does not used
 */
static void hid_host_interface_event_callback(const hid_host_interface_event_t *event, void *arg)
{
    switch (event->event) {
    case HID_DEVICE_INTERFACE_INIT:
        ESP_LOGI(TAG, "Interface number %d, protocol %s",
                 event->interface.num,
                 (event->interface.proto == HID_PROTOCOL_KEYBOARD)
                 ? "Keyboard"
                 : "Mouse");

        if (event->interface.proto == HID_PROTOCOL_KEYBOARD) {
            const hid_host_interface_config_t hid_keyboard_config = {
                .proto = HID_PROTOCOL_KEYBOARD,
                .callback = hid_host_keyboard_report_callback,
            };

            hid_host_claim_interface(&hid_keyboard_config, &keyboard_handle);
        }

        if (event->interface.proto == HID_PROTOCOL_MOUSE) {
            const hid_host_interface_config_t hid_mouse_config = {
                .proto = HID_PROTOCOL_MOUSE,
                .callback = hid_host_mouse_report_callback,
            };

            hid_host_claim_interface(&hid_mouse_config, &mouse_handle);
        }

        break;
    case HID_DEVICE_INTERFACE_TRANSFER_ERROR:
        ESP_LOGD(TAG, "Interface number %d, transfer error",
                 event->interface.num);
        break;

    case HID_DEVICE_INTERFACE_CLAIM:
    case HID_DEVICE_INTERFACE_RELEASE:
        // ... do nothing here for now
        break;

    default:
        ESP_LOGI(TAG, "%s Unhandled event %X, Interface number %d",
                 __FUNCTION__,
                 event->event,
                 event->interface.num);
        break;
    }
}

/**
 * @brief Handle common USB host library events
 *
 * @param[in] args  Pointer to arguments, does not used
 */
static void handle_usb_events(void *args)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
            xEventGroupSetBits(usb_flags, HOST_NO_CLIENT);
        }
        // Give ready_to_uninstall_usb semaphore to indicate that USB Host library
        // can be deinitialized, and terminate this task.
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            xEventGroupSetBits(usb_flags, HOST_ALL_FREE);
        }
    }

    vTaskDelete(NULL);
}

static bool wait_for_event(EventBits_t event, TickType_t timeout)
{
    return xEventGroupWaitBits(usb_flags, event, pdTRUE, pdTRUE, timeout) & event;
}

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}


/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                if (curr_dev_id) {esp_ble_gap_set_rand_addr(rand_addr2);}
                else {esp_ble_gap_set_rand_addr(rand_addr);}
                esp_ble_gap_set_device_name(ble_device_name[curr_dev_id]);//HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->led_write.data, param->led_write.length);
            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

void hid_demo_task(void *pvParameters)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    send_volum_up = true;
    while(1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (sec_conn) {
            // ESP_LOGI(HID_DEMO_TAG, "Send the movement");
            //uint8_t key_vaule = {HID_KEY_A};
            //esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
            // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
            if (send_volum_up) { esp_hidd_send_mouse_value(hid_conn_id, 0, -10, 0, 0); send_volum_up = false;}
            else {esp_hidd_send_mouse_value(hid_conn_id, 0, +10, 0, 0); send_volum_up = true;}
        }
    }
}


void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    g_usbhid_queue = xQueueCreate(USBHID_QUEUE_SIZE, sizeof(hid_mouse_input_report_queue_t));
    if (g_usbhid_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return;
    }

    g_blehid_queue = xQueueCreate(USBHID_QUEUE_SIZE, sizeof(uint8_t));
    if (g_blehid_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return;
    }

    // Set the LEDC peripheral configuration
    // example_ledc_init();
    // Set duty to 50%
    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    // xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);

    TaskHandle_t usb_events_task_handle;
    TaskHandle_t blehid_proc_handle;
    TaskHandle_t blehid_send_handle;
    hid_host_device_handle_t hid_device;

    BaseType_t task_created;

    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK( gpio_config(&input_pin) );

    ESP_LOGI(TAG, "HID HOST example");

    usb_flags = xEventGroupCreate();
    assert(usb_flags);

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    ESP_ERROR_CHECK( usb_host_install(&host_config) );
    task_created = xTaskCreate(handle_usb_events, "usb_events", 4096, NULL, 3, &usb_events_task_handle);
    assert(task_created);

    // hid host driver config
    const hid_host_driver_config_t hid_host_config = {
        .create_background_task = true,
        .task_priority = configMAX_PRIORITIES - 2,
        .stack_size = 4096,
        .core_id = 1,
        .callback = hid_host_event_callback,
        .callback_arg = NULL
    };

    ESP_ERROR_CHECK( hid_host_install(&hid_host_config) );

    // task_created = xTaskCreatePinnedToCore(ble_hid_proceed, "blehid_proc_evt", 4096, NULL, configMAX_PRIORITIES - 1, &blehid_proc_handle, 1);
    // assert(task_created);
    task_created = xTaskCreatePinnedToCore(ble_hid_sender, "blehid_send_evt", 4096, NULL, configMAX_PRIORITIES - 4, &blehid_send_handle, 0);
    assert(task_created);

    do {
        EventBits_t event = xEventGroupWaitBits(usb_flags, USB_EVENTS_TO_WAIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(APP_QUIT_PIN_POLL_MS));

        if (event & DEVICE_CONNECTED) {
            xEventGroupClearBits(usb_flags, DEVICE_CONNECTED);
            hid_device_connected = true;
        }

        if (event & DEVICE_ADDRESS_MASK) {
            xEventGroupClearBits(usb_flags, DEVICE_ADDRESS_MASK);

            const hid_host_device_config_t hid_host_device_config = {
                .dev_addr = (event & DEVICE_ADDRESS_MASK) >> 4,
                        .iface_event_cb = hid_host_interface_event_callback,
                        .iface_event_arg = NULL,
            };

            ESP_ERROR_CHECK( hid_host_install_device(&hid_host_device_config, &hid_device) );
            // ESP_LOGD(TAG, "Report size: %d", );
            hid_host_print_descriptors(hid_device);
        }

        if (event & DEVICE_DISCONNECTED) {
            xEventGroupClearBits(usb_flags, DEVICE_DISCONNECTED);

            hid_host_release_interface(keyboard_handle);
            hid_host_release_interface(mouse_handle);

            ESP_ERROR_CHECK( hid_host_uninstall_device(hid_device) );

            hid_device_connected = false;
        }

    } while (gpio_get_level(APP_QUIT_PIN) != 0);

    if (hid_device_connected) {
        ESP_LOGI(TAG, "Uninitializing HID Device");
        hid_host_release_interface(keyboard_handle);
        hid_host_release_interface(mouse_handle);
        ESP_ERROR_CHECK( hid_host_uninstall_device(hid_device) );
        hid_device_connected = false;
    }

    ESP_LOGI(TAG, "Uninitializing USB");
    ESP_ERROR_CHECK( hid_host_uninstall() );
    wait_for_event(READY_TO_UNINSTALL, portMAX_DELAY);
    ESP_ERROR_CHECK( usb_host_uninstall() );
    vTaskDelete(usb_events_task_handle);
    vEventGroupDelete(usb_flags);
    ESP_LOGI(TAG, "Done");
}
