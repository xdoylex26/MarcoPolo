#define SET 1
#define CLEAR 0

int count = 0;
int channel_A_sound, channel_B_sound, channel_C_sound; //these will store the values of the adc registers
int max_sound = 110;
int debounceDelay = 20000;
int loud_noise_flag_A, loud_noise_flag_B, loud_noise_flag_C;  //these are the flags when the sound is higher than the max sound
int noise_state_A, noise_state_B, noise_state_C; //these are the flags for a debounced loud noise on each channel.
int lastDebounceTime_A, lastDebounceTime_B, lastDebounceTime_C; //these are variables to use for the debounce comparison
int last_reading_A, last_reading_B, last_reading_C;   //these store the last values for the loud_noise_flags
int temp = 0;
char ADMUX_A = 0b11100011;
char ADMUX_B = 0b11100100;
char ADMUX_C = 0b11100101;
bool mux = 1;
int bearing = 0;
bool no_direction = 1;  //set to one by default just so I can start to detect bearing immediately.  This should be set to 0 in our program.
int degree;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600); //Set serial monitor to run at 9600 baud  
 
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
  sei();
  ADCSRA |= 1<<ADSC;  //Start conversion
}

void loop()
{
  if(!no_direction) {
    Serial.print("Bearing is - ");
    switch (bearing) {
    case 0: 
      degree = 0;
    break;
    case 1: 
      degree = 270;
    break;
    case 2: 
      degree = 330;
    break;
    case 3: 
      degree = 30;
    break;
    case 4: 
      degree = 90;
    break;
    case 5: 
      degree = 150;
    break;
    case 6: 
      degree = 210;
    break;
    default: 
      degree = 0;
    break;
    }
    Serial.println(degree, DEC);
    no_direction = SET;  //This is my method to restart the "bearing determination" but this will be something we will do only when we want a new bearing (state machine maybe?)
    delay(1000);
  }  
}


ISR(ADC_vect)  //This is the function that is called whenever the ADC completes a conversion
{
  if(count == 0) {
    temp = ADCH;

    //Check to see if the analog input is higher than the threshold
    if(temp >= max_sound) {
      loud_noise_flag_A = SET;
    }
    else {
      loud_noise_flag_A = CLEAR;
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
          if(channel_B_sound) {
            bearing = 1;
            no_direction = CLEAR;
            Serial.println(bearing);  
          }
          else if (channel_C_sound) {
            bearing = 4;
            no_direction = CLEAR;
            Serial.println(bearing);  
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
    }
    else {
      loud_noise_flag_B = CLEAR;
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
          if(channel_A_sound) {
            bearing = 2;
            no_direction = CLEAR;
            Serial.println(bearing);
          }
          else if (channel_C_sound) {
            bearing = 5;
            no_direction = CLEAR;
            Serial.println(bearing);
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
    }
    else {
      loud_noise_flag_C = CLEAR;
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
          if(channel_A_sound) {
            bearing = 3;
            no_direction = CLEAR;
            Serial.println(bearing);
          }
          else if (channel_B_sound) {
            bearing = 6;
            no_direction = CLEAR;
            Serial.println(bearing);
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
