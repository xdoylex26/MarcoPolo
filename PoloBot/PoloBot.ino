
#include <SoftwareSerial.h>

// Final Project v 0.1
// Microprocessors for Robotics 525.410
// Doyle 4.11.17
// Test change because why not
#include <Arduino_FreeRTOS.h>
#include <MeAuriga.h>
#include <Arduino.h>
#include <util/atomic.h>
#include "project.h"
#include "TaskController.h"

#define BUZZER_PORT 45

#define motion_halt(){        \
    Encoder_1.setPulsePos(0); \
    Encoder_2.setPulsePos(0); \
    Encoder_1.moveTo(0, 128); \
    Encoder_2.moveTo(0, 128); \
}


MeEncoderOnBoard Encoder_1(SLOT1); // 1 is right, needs negative value to move forward
MeEncoderOnBoard Encoder_2(SLOT2); // 2 is left, needs positive value to move forward
MeBuzzer buzzer;
double const INCHES_PER_TURN = 4.91;
int const DEGREES_NEEDED_FOR_ROTATION = 2000; // tested empirically
int buzzer_tone = 6000;

// define tasks
void TaskMotion( void *pvParameters );
void TaskController( void *pvParameters );

void isr_process_encoder1(void)
{
    if(digitalRead(Encoder_1.getPortB()) == 0) {
        Encoder_1.pulsePosMinus();
    } else {
        Encoder_1.pulsePosPlus();
    }
}


void isr_process_encoder2(void)
{
    if(digitalRead(Encoder_2.getPortB()) == 0) {
        Encoder_2.pulsePosMinus();
    } else {
        Encoder_2.pulsePosPlus();
    }
}


void setup()
{
    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    buzzer.setpin(BUZZER_PORT);
    Serial.begin(115200);
    prelude_report();

    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);

    sei(); // enable interrupts

    // Set up Encoder / pids
    Encoder_1.setPulse(9);
    Encoder_2.setPulse(9);
    Encoder_1.setRatio(39.267);
    Encoder_2.setRatio(39.267);
    Encoder_1.setPosPid(1.8,0,1.2); // 1.8 0 1.2 orig
    Encoder_2.setPosPid(1.8,0,1.2);
    Encoder_1.setSpeedPid(.18,0.0,0); // 0.18 0 0 orig
    Encoder_2.setSpeedPid(.18,0.0,0); // 0.18

    xTaskCreate(
        TaskMotion
        , (const portCHAR *)"Motion"
        , 128
        , NULL
        , 1
        , NULL );

    xTaskCreate(
        TaskController
        , (const portCHAR *)"Controller"
        , 128
        , NULL
        , 1
        , NULL );
}


void loop()
{
    // Empty. Things are done in Tasks.
}


void buzzer_beep(int state)
{
    if(1 == state) {
        //analogWrite(BUZZER_PORT, 100);
        buzzer.tone(buzzer_tone, 1000);
        //Serial.println(buzzer_tone);
        delay(1000);
        //buzzer_tone += 50;
    }
    else
        analogWrite(BUZZER_PORT, 0);
}


/*---------------------- Tasks ---------------------*/
void TaskMotion(void *pvParameters)
{
    /* Default state */
    int latched_controller_state;
    int controller_speed = 80;

    for (;;) {
        /* Check if any new information has come over from the controller task */
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            latched_controller_state = g_controller_state;
        }

        /* We can make things simple by assuming only one button will be pressed at a time */
        switch(latched_controller_state) {
        case BNT_NONE:
            buzzer_beep(0);
            motion_halt();
            break;
        case BTN_FORWARD:
            Encoder_1.moveTo(-999, controller_speed);
            Encoder_2.moveTo(999, controller_speed);
            break;
        case BTN_REVERSE:
            Encoder_1.moveTo(999, controller_speed);
            Encoder_2.moveTo(-999, controller_speed);
            break;
        case BTN_LEFT:
            Encoder_1.moveTo(-999, controller_speed);
            Encoder_2.moveTo(-999, controller_speed);
            break;
        case BTN_RIGHT:
            Encoder_1.moveTo(999, controller_speed);
            Encoder_2.moveTo(999, controller_speed);
            break;
        case BTN_BEEP:
            buzzer_beep(1);
            break;
        default:
            break;
        }

        /*
         * This ensures we never get to our "target" destintation.
         * There really is no "target" we just want to drive in the requested direction
         */
        Encoder_1.setPulsePos(0);
        Encoder_2.setPulsePos(0);

        Encoder_1.loop();
        Encoder_2.loop();
        vTaskDelay(1);
    }
}
