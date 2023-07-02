#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief HID Mouse Input Report for Boot Interfaces
 *
 * @see B.1, p.60 of Device Class Definition for Human Interface Devices (HID) Version 1.11
 */
typedef struct {
    union {
        struct {
            uint8_t button1:    1;
            uint8_t button2:    1;
            uint8_t button3:    1;
            uint8_t button4:    1;
            uint8_t button5:    1;
            uint8_t reserved:   3;
        };
        uint8_t val;
    } buttons;
    int8_t displacement[3];
    int8_t z_displacement;
} __attribute__((packed)) hid_mouse_input_report_boot_t;

#ifdef __cplusplus
}
#endif //__cplusplus
