#ifndef _TASK_CONTROLLER_H
#define _TASK_CONTROLLER_H


extern volatile uint8_t g_controller_state;

/* Simpler mapping to encode ever button state into a single byte */
enum controller_map {
    BTN_FORWARD = 0x01,
    BTN_REVERSE = 0x02,
    BTN_LEFT    = 0x04,
    BTN_RIGHT   = 0x08,
    BTN_A       = 0x10,
    BTN_BEEP 	  = BTN_A,
    BTN_B       = 0x20,
    BTN_START   = 0x40,
    BTN_SELECT  = 0x80,
    BNT_NONE    = 0x00
};


#endif /* _TASK_CONTROLLER_H */

