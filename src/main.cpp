#include <Arduino.h>
#include "driver/ledc.h"

#define LED_PIN 4

#define PWM_FREQ_HZ 10000
#define PWM_BIT_DEPTH 12

#define PWM_CHNL_THROTTLE 2
#define PWM_PIN_THROTTLE 10

#define PWM_CHNL_STEERING 3
#define PWM_PIN_STEERING 9

#define ZERO_PWM 1400
#define MAX_PWM 3200
#define MIN_PWM 0

int throttle = ZERO_PWM;
int steering = ZERO_PWM;
int prev_throttle = ZERO_PWM;
int prev_steering = ZERO_PWM;

int state = 1;
int iteration = 0;

char character;

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

  digitalWrite(LED_PIN, LOW);
}

void loop() {

  // Receiving a command from laptop:
  if (Serial.available() > 0){
    character = Serial.read();

    switch (character) {
      case 'f':
        throttle = MAX_PWM;
        steering = ZERO_PWM;
        Serial.print("ESP: Driving forward: ");
        break;
      case 'b':
        throttle = MIN_PWM;
        steering = ZERO_PWM;
        Serial.print("ESP: Driving backward: ");
        break;
      case 'l':
        throttle = MAX_PWM;
        steering = MAX_PWM;
        Serial.print("ESP: Driving left: ");
        break;
      case 'r':
        throttle = MAX_PWM;
        steering = MIN_PWM;
        Serial.print("ESP: Driving right: ");
        break;
      default: //stop
        throttle = ZERO_PWM;
        steering = ZERO_PWM;
        Serial.print("ESP: Stopped: ");
        break;
    }
    //digitalWrite(LED_PIN, HIGH);
  }
  else {
    //digitalWrite(LED_PIN, LOW);
  }


  // state 1 establishes connection
  if (state == 1){
    digitalWrite(LED_PIN, LOW);
    // pot read max value: 4096 (12bit)
    throttle = ZERO_PWM;
    steering = ZERO_PWM;

    // writes a duty cycle to the specified pwmchannel 
    // (which in this case was linked to pin 4)
    ledcWrite(PWM_CHNL_THROTTLE, throttle);
    ledcWrite(PWM_CHNL_STEERING, steering);

    if (iteration > 1000){
      state = 2;
    }
  }


  // state 2 increases / decreases throttle and steering by 100 every 
  // 1 second
  if (state == 2){
    // increment = (throttle >= MAX_PWM) ? -800 : increment;
    // increment = (throttle <= MIN_PWM) ? 800 : increment;
    // throttle += increment;

    // throttle = 1200;
    // steering = iteration % 2 == 0 ? 3000 : 0;

    digitalWrite(LED_PIN, HIGH);

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