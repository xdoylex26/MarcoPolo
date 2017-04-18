#include "MeUSBHost.h"


MeUSBHost usbhost(PORT_5);  
        
struct retro_controller_map {
  uint8_t reserved[3];
  uint8_t dpad[2];    /* First byte maps to LEFT/RIGHT, then second byte is UP/DOWN */
  uint8_t r_buttons;
  uint8_t b_buttons;
} __attribute__((packed));


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


void wait_for_controller(void){
  while(0 == usbhost.device_online){
    usbhost.probeDevice();
    delay(250);
  }
}


void controller_parse(const uint8_t *buf, int len){
  static struct retro_controller_map last_map = {0};
  struct retro_controller_map *cmap = (struct retro_controller_map *) buf;

  /* First, check to see if anything has changed from the last run (most common state is no) */
  if(0 == memcmp(&last_map, cmap, sizeof(struct retro_controller_map)))
    return;

  if(cmap->r_buttons & BUTTON_A)
    Serial.print("A, ");

  if(cmap->r_buttons & BUTTON_B)
    Serial.print("B, ");

  if(cmap->b_buttons & BUTTON_START)
    Serial.print("START, ");

  if(cmap->b_buttons & BUTTON_SELECT)
    Serial.print("SELECT, ");
    
  if(0 == cmap->dpad[0])
    Serial.print("LEFT, ");
        
  if(cmap->dpad[0] & BUTTON_RIGHT)
    Serial.print("RIGHT, ");    

  if(0 == cmap->dpad[1])
    Serial.print("UP, ");

  if(cmap->dpad[1] & BUTTON_DOWN)
    Serial.print("DOWN, ");

  Serial.println("");

  /* Copy the contents of the map to a cache for later */
  memcpy(&last_map, buf, sizeof(struct retro_controller_map));
}


void TaskController(void *pvParameters)
{
    int len;   
    usbhost.init(USB1_0);
    
    wait_for_controller();
    Serial.println("Controller detected .. ");

    while(1){
      len = usbhost.host_recv();
      controller_parse((uint8_t *)usbhost.RECV_BUFFER, len);
      delay(250);
    }
}
