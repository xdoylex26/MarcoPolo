// Final Project v 0.1
// Microprocessors for Robotics 525.410
// Doyle 4.11.17
// Test change because why not
#include <Arduino_FreeRTOS.h>
#include <MeAuriga.h>
#include <Arduino.h>
#include <util/atomic.h>
#include "project.h"

#define BUZZER_PORT 45
#define SET 1
#define CLEAR 0

MeEncoderOnBoard Encoder_1(SLOT1); // 1 is right, needs negative value to move forward
MeEncoderOnBoard Encoder_2(SLOT2); // 2 is left, needs positive value to move forward
volatile int soundVal = 0;
volatile int g_soundHeading = 0;
//extern volatile uint8_t g_nes_state;
long int clapTime = 0;
double const INCHES_PER_TURN = 4.91;
int const DEGREES_NEEDED_FOR_ROTATION = 1700; // tested empirically

//Variables required for sound sensor
int count = 0;
int channel_A_sound, channel_B_sound, channel_C_sound; //these will store the values of the adc registers
int max_sound = 68;
unsigned long debounceDelay = 300000;
unsigned long settleDelay = 20000;
int loud_noise_flag_A, loud_noise_flag_B, loud_noise_flag_C;  //these are the flags when the sound is higher than the max sound
int noise_state_A, noise_state_B, noise_state_C; //these are the flags for a debounced loud noise on each channel.
unsigned long lastDebounceTime_A, lastDebounceTime_B, lastDebounceTime_C; //these are variables to use for the debounce comparison
unsigned long lastTimeHighA, lastTimeHighB, lastTimeHighC = 0;
int last_reading_A, last_reading_B, last_reading_C;   //these store the last values for the loud_noise_flags
int temp = 0;
char ADMUX_A = 0b11100011;
char ADMUX_B = 0b11100100;
char ADMUX_C = 0b11100101;
bool mux = 1;
int bearing = 0;
bool no_direction = 1;  //set to one by default just so I can start to detect bearing immediately.  This should be set to 0 in our program.
int degree;

enum MotionStates   {
    WAITING_FOR_HEADING,
    IN_PURSUIT_TURNING,
    IN_PURSUIT_CHASING
};

// define tasks
void TaskMotion( void *pvParameters );

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

ISR(ADC_vect)
{
    /* Grab the data from the latest conversion */
    /* Schedule the next channel to undergo conversion */
    /* Clear the ADC conversion-complete interrupt flag (starts next conversion?) */
    /* Check if current channel has passed from pre-determined threshold for activation */
    /* If it has, set a global volatile data element (g_soundHeading) to the heading corresponding to the triggered ADC channel's direction */
    /* Disable ADC until enabled again (some other state where we start listening for tones again) */
    if(count == 0) {
      temp = ADCH;
    //Serial.println(ADCH);
    //Check to see if the analog input is higher than the threshold
    if(temp >= max_sound) {
      loud_noise_flag_A = SET;
      lastTimeHighA = micros();
    }
    else {
      if(micros() - lastTimeHighA > settleDelay) {
        loud_noise_flag_A = CLEAR;
        //Serial.println("A LOW");
      }
    }
  
    if (loud_noise_flag_A != last_reading_A) {  //if our flag does not match the last time we had a reading, then we set the time that we got this new loud noise flag to be used to debounce
      lastDebounceTime_A = micros();
    }
    
    if ((micros() - lastDebounceTime_A ) > debounceDelay) {  //If we have had the high loud noise for a certain amount of time we check to see if it's bee high, then we act as if we have a loud noise (debounced)

      if(loud_noise_flag_A != noise_state_A) {
        noise_state_A = loud_noise_flag_A;
        if(no_direction && noise_state_A == HIGH) {//if we have a debounced loud noise (debounce time has surpassed and the noise state is high), then we want to check to see if there have been any other
                                                     //legit loud noise on microphones.  If there have, depending on which came first, we determin the direction of the source. Then we clear the no_direction
                                                     //flag to let other programs know that we have a bearing.  If we haven't had another microphone then the current mic is the first to recieve a signal.
          //Serial.println(" Debounce A ");
          if(channel_B_sound) {
            bearing = 1;
            no_direction = CLEAR;
            //Serial.println(bearing);
          }
          else if (channel_C_sound) {
            bearing = 4;
            no_direction = CLEAR;
            //Serial.println(bearing);  
          }
        }
      }
      if (noise_state_A == HIGH) {  //set the noise state of the current mic to be used elsewhere if needed
        channel_A_sound = SET;
      }
      else {
        channel_A_sound = CLEAR;
      }
    }

    last_reading_A = loud_noise_flag_A;
    if(mux == 1) {ADMUX = ADMUX_B;}   //We want to move to the next plug channel, channel 6
  }
  else if (count == 1) {
    temp = ADCH;
    
     //Check to see if the analog input is higher than the threshold
    if(temp >= max_sound) {
      loud_noise_flag_B = SET;
      lastTimeHighB = micros();
    }
    else {
      if(micros() - lastTimeHighB > settleDelay) {
        loud_noise_flag_B = CLEAR;
        //Serial.println("B LOW");
      }
    }
  
    if (loud_noise_flag_B != last_reading_B) {  //if our flag does not match the last time we had a reading, then we set the time that we got this new loud noise flag to be used to debounce
      lastDebounceTime_B = micros();
    }
    
    if ((micros() - lastDebounceTime_B ) > debounceDelay) {  //If we have had the high loud noise for a certain amount of time we check to see if it's bee high, then we act as if we have a loud noise (debounced)

      if(loud_noise_flag_B != noise_state_B) {
        noise_state_B = loud_noise_flag_B;
        if(no_direction && noise_state_B == HIGH) {//if we have a debounced loud noise (debounce time has surpassed and the noise state is high), then we want to check to see if there have been any other
                                                     //legit loud noise on microphones.  If there have, depending on which came first, we determin the direction of the source. Then we clear the no_direction
                                                     //flag to let other programs know that we have a bearing.  If we haven't had another microphone then the current mic is the first to recieve a signal.
          //Serial.println(" Debounce B ");
          if(channel_A_sound) {
            bearing = 2;
            no_direction = CLEAR;
            //Serial.println(bearing);
          }
          else if (channel_C_sound) {
            bearing = 5;
            no_direction = CLEAR;
            //Serial.println(bearing);
          }
        }
      }
      if (noise_state_B == HIGH) {//set the noise state of the current mic to be used elsewhere if needed
        channel_B_sound = SET;
      }
      else {
        channel_B_sound = CLEAR;
      }
    }

    last_reading_B = loud_noise_flag_B;
    if (mux == 1) {ADMUX = ADMUX_C;}   //We want to move to the next plug channel, channel 6
  }
  else if (count == 2) {
    temp = ADCH;
    
     //Check to see if the analog input is higher than the threshold
    if(temp >= max_sound) {
      loud_noise_flag_C = SET;
      lastTimeHighC = micros();
    }
    else {
      if(micros() - lastTimeHighC > settleDelay) {
        loud_noise_flag_C = CLEAR;
        //Serial.println("C LOW");
      }
    }
  
    if (loud_noise_flag_C != last_reading_C) {  //if our flag does not match the last time we had a reading, then we set the time that we got this new loud noise flag to be used to debounce
      lastDebounceTime_C = micros();
    }
    
    if ((micros() - lastDebounceTime_C ) > debounceDelay) {  //If we have had the high loud noise for a certain amount of time we check to see if it's high, then we act as if we have a loud noise (debounced)
      if(loud_noise_flag_C != noise_state_C) {
        noise_state_C = loud_noise_flag_C;
        if(no_direction && noise_state_C == HIGH) {  //if we have a debounced loud noise (debounce time has surpassed and the noise state is high), then we want to check to see if there have been any other
                                                     //legit loud noise on microphones.  If there have, depending on which came first, we determin the direction of the source. Then we clear the no_direction
                                                     //flag to let other programs know that we have a bearing.  If we haven't had another microphone then the current mic is the first to recieve a signal.
          //Serial.println(" Debounce C ");
          if(channel_A_sound) {
            bearing = 3;
            no_direction = CLEAR;
            //Serial.println(bearing);
          }
          else if (channel_B_sound) {
            bearing = 6;
            no_direction = CLEAR;
            //Serial.println(bearing);
          }
        }
      }
      if (noise_state_C == HIGH) {//set the noise state of the current mic to be used elsewhere if needed
        channel_C_sound = SET;
      }
      else {
        channel_C_sound = CLEAR;
      }
    }
    last_reading_C = loud_noise_flag_C;
    if (mux == 1) {ADMUX = ADMUX_A;}   //We want to move to the next plug channel, channel 6
  }  
  if (mux == 1) {count++;}
  if (count > 2) {count = 0;}
  ADCSRA |= 1<<ADSC;                                    // Re-start Conversion
}


void setup()
{
    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    Serial.begin(19200);
    prelude_report();
    
    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);

    sei(); // enable interrupts

    // ADC
    ADCSRA = 0b10001100;  //0x8F; //Verified
                      //bit 7 = ADC Enable
                      //bit 6 = ADC Start Conversion
                      //bit 5 = ADC Auto Trigger Enable
                      //bit 4 = ADC interrupt flag
                      //bit 3 = adc interrupt enable
                      //bit 2:0 = ADC Prescaler Select Bits
    ADCSRB = 0b00001000;
    ADMUX = ADMUX_A;   //0xE5;  //Verified
                    //bit 7:6 = Reference voltage set to 2.56
                      //bit 5 = ADLAR, justification of results in ADC output register, 0 right, 1 left
                      //bit 4:0 = MUX for ADC input source
    ADCSRA |= 1<<ADSC;  //Start conversion

    // Set up Encoder / pids
    Encoder_1.setPulse(9);
    Encoder_2.setPulse(9);
    Encoder_1.setRatio(39.267);
    Encoder_2.setRatio(39.267);
    Encoder_1.setPosPid(1.8,0,1.2); // 1.8 0 1.2 orig
    Encoder_2.setPosPid(1.8,0,1.2);
    Encoder_1.setSpeedPid(.5,0.0,0); // 0.18 0 0 orig
    Encoder_2.setSpeedPid(.5,0.0,0); // 0.18

    xTaskCreate(
        TaskMotion
        , (const portCHAR *)"Motion"
        , 128
        , NULL
        , 1
        , NULL );
}


void loop()
{
    // Empty. Things are done in Tasks.
}


/*---------------------- Tasks ---------------------*/
// This task need to wait until a valid bearing is determined,
// then rotate toward that bearing and move straight towards it
// for a reasonable duration and speed (Move 2 ft towards sound)
void TaskMotion(void *pvParameters)
{
    /* Default state */
    int marcoMotionState = WAITING_FOR_HEADING;
    int latched_soundHeading;

    for (;;) {
        // Need to move, valid sound sensed, probably switch-case for cleaner code
        switch(marcoMotionState) {
        case IN_PURSUIT_TURNING: {
            ADCSRA &= 0b01111111; // Disable ADC for now
            /* Take latched_soundHeading here and determine how to move */
            // But do this so we don't turn more than 180 degs :)
            if(latched_soundHeading > 180) {
              latched_soundHeading = latched_soundHeading - 360;  // So we don't go 270 CW, but instead 90 CCW
            }
            int degToRotate = DEGREES_NEEDED_FOR_ROTATION * (latched_soundHeading / 360.0);
            //Serial.print("TURNING ");
            //Serial.println(latched_soundHeading);
            // 90 deg rotation
            Encoder_1.moveTo(degToRotate, 120);
            Encoder_2.moveTo(degToRotate, 120);
            if(abs(Encoder_1.getCurPos()-degToRotate) < 15 && abs(Encoder_2.getCurPos()-degToRotate) < 15) {
                // Finished turning, now move forward
                marcoMotionState = IN_PURSUIT_CHASING;
                Encoder_1.setPulsePos(0);
                Encoder_2.setPulsePos(0);
            }
        }
        break;

        case IN_PURSUIT_CHASING: {
          //Serial.println("CHASING");
            /* Check if any new information has come over from the controller task */
            /* If yes, quickly parse the data and translate buttons into motion */
            //if(0 == g_nes_state)    /* No buttons pressed */
            //    continue;
        
            // temps for testing
            double inchesToMove = 24.0;
            double degToMove = (inchesToMove/INCHES_PER_TURN)*360;

            // Move 2 ft forward toward sound
            Encoder_1.moveTo(-degToMove, 120);
            Encoder_2.moveTo(degToMove, 120);
            if(abs(Encoder_1.getCurPos()+degToMove) < 25 || abs(Encoder_2.getCurPos()-degToMove) < 25) {
                // Finished turning, now move forward
                /* Enable ADC to begin converting data again, then transfer to WAITING state */
                marcoMotionState = WAITING_FOR_HEADING;
                Encoder_1.setPulsePos(0);
                Encoder_2.setPulsePos(0);
                Encoder_1.moveTo(0, 128);
                Encoder_2.moveTo(0, 128);

                vTaskDelay( 2000 / portTICK_PERIOD_MS ); 
                // Restart the ADC after 2 sec waiting period
                // Not sure if all of this is needed again, but just toggled ADEN didn't work - mark
                ADCSRA = 0b10001100;  //0x8F; //Verified
                                  //bit 7 = ADC Enable
                                  //bit 6 = ADC Start Conversion
                                  //bit 5 = ADC Auto Trigger Enable
                                  //bit 4 = ADC interrupt flag
                                  //bit 3 = adc interrupt enable
                                  //bit 2:0 = ADC Prescaler Select Bits
                ADCSRB = 0b00001000;
                ADMUX = ADMUX_A;   //0xE5;  //Verified
                                //bit 7:6 = Reference voltage set to 2.56
                                  //bit 5 = ADLAR, justification of results in ADC output register, 0 right, 1 left
                                  //bit 4:0 = MUX for ADC input source
                ADCSRA |= 1<<ADSC;  //Start conversion
                no_direction = SET;
            }
        }
        break;

        case WAITING_FOR_HEADING:
            Serial.println("WAITING ");
            //vTaskDelay( 3000 / portTICK_PERIOD_MS );            
            //no_direction = SET;  //This is my method to restart the "bearing determination" but this will be something we will do only when we want a new bearing (state machine maybe?
            Encoder_1.setPulsePos(0);
            Encoder_2.setPulsePos(0);
            //Encoder_1.moveTo(0, 128);
            //Encoder_2.moveTo(0, 128);
            Encoder_1.setTarPWM(0);
            Encoder_2.setTarPWM(0);
            while(no_direction){
              Encoder_1.loop();
              Encoder_2.loop();
              vTaskDelay( 1 / portTICK_PERIOD_MS );
              }
            //vTaskDelay( 1000 / portTICK_PERIOD_MS );    
                if(!no_direction) {
                    //Serial.print("Bearing is - ");
                    switch (bearing) { //convert the bearing location to an actual degree value.
                    case 0: 
                      degree = 0;
                    break;
                    case 1: 
                      degree = 150;
                    break;
                    case 2: 
                      degree = 210;
                    break;
                    case 3: 
                      degree = 270;
                    break;
                    case 4: 
                      degree = 330;
                    break;
                    case 5: 
                      degree = 30;
                    break;
                    case 6: 
                      degree = 90;
                    break;
                    default: 
                      degree = 0;
                    break;
                    }
                    //Serial.println(degree, DEC);
                    no_direction = SET;  //This is my method to restart the "bearing determination" but this will be something we will do only when we want a new bearing (state machine maybe?)
                    //vTaskDelay( 1000 / portTICK_PERIOD_MS );
                    latched_soundHeading = degree;  //This is the latch that you had below.  I just brought it into the code that I know works
                    degree = 0; 
                    marcoMotionState = IN_PURSUIT_TURNING;
                  }  

            /* Check global ADC flagbyte for new heading information; latch the current value to prevent race conditions */
//             ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//                 latched_soundHeading = g_soundHeading;
//                 g_soundHeading = 0;
//             }

            /* 
             * Non-zero means some direction has been determined, other states decode this information to 
             * determine how to move. This byte should only be set by the ADC ISR .  
             *It is possible to have a 0 degree heading, so i put the change of the state machine into the while loop if we have a no bearing.
             */
//             if(0 != latched_soundHeading){
//                 /* Clear the old sound heading after we've latched it */
//                 ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
//                     g_soundHeading = 0;
//                 }

//                 /* Disable ADC before moving as we do not need to waste cycles */
//                 marcoMotionState = IN_PURSUIT_TURNING;
//             }

            break;

        default:
            break;

        }

        Encoder_1.loop();
        Encoder_2.loop();
        vTaskDelay(1);
    }
}
