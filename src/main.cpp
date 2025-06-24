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
#define MAX_PWM 3000
#define MIN_PWM 100

int throttle = 0;
int steering = 0;

int state = 1;
int iteration = 0;

//int increment = 100;
int increment = 1200;

void setup() {
  pinMode(LED_PIN, OUTPUT);

  // Set pwmchannel to use,  frequency in Hz, number of bits)
  ledcSetup(PWM_CHNL_THROTTLE, PWM_FREQ_HZ, PWM_BIT_DEPTH);
  ledcSetup(PWM_CHNL_STEERING, PWM_FREQ_HZ, PWM_BIT_DEPTH);
  ledcAttachPin(PWM_PIN_THROTTLE, PWM_CHNL_THROTTLE);
  ledcAttachPin(PWM_PIN_STEERING, PWM_CHNL_STEERING);
}

void loop() {
  // state 0 establishes connection
  if (state == 1){
    // pot read max value: 4096 (12bit)
    throttle = ZERO_PWM;
    steering = ZERO_PWM;

    // writes a duty cycle to the specified pwmchannel 
    // (which in this case was linked to pin 4)
    ledcWrite(PWM_CHNL_THROTTLE, throttle);
    ledcWrite(PWM_CHNL_STEERING, steering);

    if (iteration > 2){
      state = 2;
    }
  }
  
  // state 2 increases / decreases throttle and steering by 100 every 
  // 1 second
  if (state == 2){
    increment = (throttle >= MAX_PWM) ? -800 : increment;
    increment = (throttle <= MIN_PWM) ? 800 : increment;
    throttle += increment;

    throttle = 1200;
    steering = iteration % 2 == 0 ? 3000 : 0;

    throttle = (throttle >= MAX_PWM) ? MAX_PWM : throttle;
    ledcWrite(PWM_CHNL_THROTTLE, throttle);
    ledcWrite(PWM_CHNL_STEERING, steering);
  }

  iteration++;

  delay(2000); // brief delay of 500ms
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW);


}