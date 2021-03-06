#include "MeUSBHost.h"
#include "TaskController.h"


MeUSBHost usbhost(PORT_5);
volatile uint8_t g_controller_state = 0;

/* Structure overlay on the received buffer (native format) */
struct retro_map {
    uint8_t reserved[3];
    uint8_t dpad[2]; /* First byte maps to LEFT/RIGHT, then second byte is UP/DOWN */
    uint8_t r_buttons;
    uint8_t b_buttons;
    uint8_t reserved2;
} __attribute__((packed));


/* Mapping we get from the controller directly */
enum {
    BUTTON_A =      0x20,
    BUTTON_B =      0x40,
    BUTTON_SELECT = 0x10,
    BUTTON_START =  0x20,
    BUTTON_UP =     0x00,
    BUTTON_DOWN =   0x80,
    BUTTON_LEFT =   0x00,
    BUTTON_RIGHT =  0x80
};

void wait_for_controller(void)
{
    while(0 == usbhost.device_online) {
        usbhost.probeDevice();
        delay(250);
    }
}


void controller_parse(const uint8_t *buf, int len)
{
    static struct retro_map last_map = {0};
    struct retro_map *cmap = (struct retro_map *) buf;
    uint8_t local_button_map = 0;

    /* First, check to see if anything has changed from the last run (most common state is no) */
    if(0 == memcmp(&last_map, cmap, sizeof(struct retro_map)))
        return;

    if(cmap->r_buttons & BUTTON_A)
        local_button_map |= BTN_A;

    if(cmap->r_buttons & BUTTON_B)
        local_button_map |= BTN_B;

    if(cmap->b_buttons & BUTTON_START)
        local_button_map |= BTN_START;

    if(cmap->b_buttons & BUTTON_SELECT)
        local_button_map |= BTN_SELECT;

    if(0 == cmap->dpad[0])
        local_button_map |= BTN_LEFT;

    if(cmap->dpad[0] & BUTTON_RIGHT)
        local_button_map |= BTN_RIGHT;

    if(0 == cmap->dpad[1])
        local_button_map |= BTN_FORWARD;

    if(cmap->dpad[1] & BUTTON_DOWN)
        local_button_map |= BTN_REVERSE;

    /* Set the global byte in one instruction to defeat any race conditions */
    g_controller_state = local_button_map;

    /* Copy the contents of the map to a cache for later */
    memcpy(&last_map, buf, sizeof(struct retro_map));
}


void TaskController(void *pvParameters)
{
    int len;
    usbhost.init(USB1_0);

    wait_for_controller();
    Serial.println("Controller detected .. ");

    while(1) {
        len = usbhost.host_recv();
        controller_parse((uint8_t *)usbhost.RECV_BUFFER, len);
    }
}
