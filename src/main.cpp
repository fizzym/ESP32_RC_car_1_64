#include <Arduino.h>
#include "driver/ledc.h"

#define LED_PIN 4

#define PWM_FREQ_HZ 10000
#define PWM_BIT_DEPTH 12

#define PWM_CHNL_THROTTLE 2
#define PWM_PIN_THROTTLE 10

#define PWM_CHNL_STEERING 3
#define PWM_PIN_STEERING 9

#define INCREMENT 100

#define ZERO_PWM 1400 //This corresponds to about 1.7V
#define MAX_PWM 3200
#define MIN_PWM 0

#define MAX_BUFF_LEN 2

int throttle = ZERO_PWM;
int steering = ZERO_PWM;
int prev_throttle = ZERO_PWM;
int prev_steering = ZERO_PWM;

int state = 1;
int iteration = 0;

uint8_t cmd_buffer[MAX_BUFF_LEN];
uint8_t character;
uint8_t i = 0;
uint32_t cmd_strength = 0;

//int increment = 100;
int increment = 1200;

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  // Set pwmchannel to use,  frequency in Hz, number of bits)
  ledcSetup(PWM_CHNL_THROTTLE, PWM_FREQ_HZ, PWM_BIT_DEPTH);
  ledcSetup(PWM_CHNL_STEERING, PWM_FREQ_HZ, PWM_BIT_DEPTH);
  ledcAttachPin(PWM_PIN_THROTTLE, PWM_CHNL_THROTTLE);
  ledcAttachPin(PWM_PIN_STEERING, PWM_CHNL_STEERING);
}

void loop() {

  // Receiving a command from laptop:
  if (Serial.available() > 0){

    // Read a command from serial. Commands are formatted as 'ax' where
    // a is a character determining command type and x is a numerical
    // indicating command strength percentage in increments of 10% 
    // this is (mapped to PWM);
    i = 0;
    character = Serial.read();
    while (character != '\n'){
      cmd_buffer[i] = character;
      i++;
      character = Serial.read();

      //In case we are receiving more characters than expected stop the
      //robot.
      if (i > MAX_BUFF_LEN){
        cmd_buffer[0] = 's';
        cmd_buffer[1] = '0';
        break;
      }
    }

    // I subtract 48 because ASCII(0) = 48
    cmd_strength = (cmd_buffer[1] - 48 + 1) * INCREMENT;

    switch (cmd_buffer[0]) {
      case 'f': // forward PWM is between MIN_PWM and ZERO_PWM
        throttle = ZERO_PWM - cmd_strength;
        steering = ZERO_PWM;
        Serial.print("ESP: Driving forward. Throttle: ");
        Serial.print(throttle);
        Serial.print(" Steering: ");
        Serial.print(steering);
        break;
      case 'b': // backward PWM is between ZERO_PWM and MAX_PWM
        throttle = ZERO_PWM + cmd_strength;
        steering = ZERO_PWM;
        Serial.print("ESP: Driving backward. Throttle: ");
        Serial.print(throttle);
        Serial.print(" Steering: ");
        Serial.print(steering);
        break;
      case 'l': // left steer is between ZERO_PWM and MAX_PWM
        throttle = MIN_PWM;
        steering = ZERO_PWM + cmd_strength;
        Serial.print("ESP: Driving left. Throttle: ");
        Serial.print(throttle);
        Serial.print(" Steering: ");
        Serial.print(steering);
        break;
      case 'r': // right steer is between MIN_PWM and ZERO_PWM
        throttle = MIN_PWM;
        steering = ZERO_PWM - cmd_strength;
        Serial.print("ESP: Driving right. Throttle: ");
        Serial.print(throttle);
        Serial.print(" Steering: ");
        Serial.print(steering);
        break;
      default: //stop
        throttle = ZERO_PWM;
        steering = ZERO_PWM;
        // To investigate dead band s5 is ZERO_PWM while s0 is 
        // ZERO_PWM - 50 and S9 is ZERO_PWM + 50
        Serial.print("ESP: Stopped. Throttle: ");
        Serial.print(throttle -50 + ((cmd_strength - 100) / 10));
        Serial.print(" Steering: ");
        Serial.print(steering -50 + ((cmd_strength - 100) / 10));
        break;
    }
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    digitalWrite(LED_PIN, LOW);
  }


  // state 0 establishes connection
  if (state == 1){
    // pot read max value: 4096 (12bit)
    throttle = ZERO_PWM;
    steering = ZERO_PWM;

    // writes a duty cycle to the specified pwmchannel 
    // (which in this case was linked to pin 4)
    ledcWrite(PWM_CHNL_THROTTLE, throttle);
    ledcWrite(PWM_CHNL_STEERING, steering);

    if (iteration > 10000){
      state = 2;
    }
  }

  // stat 2 driving
  if (state == 2){
    //Sanitize controls before sending them out to motors:
    throttle = (throttle <= MIN_PWM) ? MIN_PWM : throttle;
    throttle = (throttle >= MAX_PWM) ? MAX_PWM : throttle;
    steering = (steering <= MIN_PWM) ? MIN_PWM : steering;
    steering = (steering >= MAX_PWM) ? MAX_PWM : steering;

    // Send commands only when value change:
    if (prev_throttle != throttle) {
      ledcWrite(PWM_CHNL_THROTTLE, throttle);
      prev_throttle = throttle;
    }
    if (prev_steering != steering) {
      ledcWrite(PWM_CHNL_STEERING, steering);
    }
  }

  iteration++;
}