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

#define MAX_BUFF_LEN 4

#define WAIT_TIME_ms 2000 //How long to wait for the RC controller to connect

#define MIN_THROTTLE_FROM_STOP 4
#define START_THROTTLE 6
#define START_THROTTLE_TIME_ms 100

// Need to use two throttle parameters as the static friction prevents
// the car from driving slow enough to be controllable on a small track
int commanded_throttle = ZERO_PWM; // throttle received over serial
int applied_throttle = ZERO_PWM; // thrtottle sent to the motors
int prev_throttle = ZERO_PWM;
bool jump_start = false;

int steering = ZERO_PWM;
int prev_steering = ZERO_PWM;
bool stopped = false;

int state = 1;
int iteration = 0;

uint8_t cmd_buffer[MAX_BUFF_LEN];
uint8_t character;
uint8_t i = 0;
uint32_t cmd_strength = 0;

//int increment = 100;
int increment = 1200;

unsigned long start_time_ms = 0;
unsigned long current_time_ms = 0;
unsigned long prev_time_ms = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  // Set pwmchannel to use,  frequency in Hz, number of bits)
  ledcSetup(PWM_CHNL_THROTTLE, PWM_FREQ_HZ, PWM_BIT_DEPTH);
  ledcSetup(PWM_CHNL_STEERING, PWM_FREQ_HZ, PWM_BIT_DEPTH);
  ledcAttachPin(PWM_PIN_THROTTLE, PWM_CHNL_THROTTLE);
  ledcAttachPin(PWM_PIN_STEERING, PWM_CHNL_STEERING);

  start_time_ms = millis();
}

void loop() {

  // Receiving a command from laptop:
  if (Serial.available() > 0){

    // Read a command from serial. Commands are formatted as 'ax' where
    // a is a character determining command type and x is a numerical
    // indicating command strength percentage in increments of 10% 
    // this is (mapped to PWM);
    /*
    Command format is direction (forward, backward) strength steering 
    (left, right) strength

    For example:
     'f2l0' : forward 2; left 0 (i.e. straight)
     'b3r2' : backward 3; right 2
    */
    i = 0;
    character = Serial.read();
    while (character != '\n'){
      // Turn LED on while receiving a command.
      digitalWrite(LED_PIN, HIGH);

      cmd_buffer[i] = character;
      i++;
      character = Serial.read();

      //In case we are receiving more characters than expected stop the
      //robot.
      if (i > MAX_BUFF_LEN){
        cmd_buffer[0] = 's';
        cmd_buffer[1] = '0';
        cmd_buffer[2] = 's';
        cmd_buffer[3] = '0';
        break;
      }
    }

    // **** Parse the commands:

    // I subtract 48 because ASCII(0) = 48
    cmd_strength = (cmd_buffer[1] - 48 + 1) * INCREMENT;
    stopped = false;

    switch (cmd_buffer[0]) {
      case 'f': // forward PWM is between MIN_PWM and ZERO_PWM
        commanded_throttle = ZERO_PWM - cmd_strength;
        break;
      case 'b': // backward PWM is between ZERO_PWM and MAX_PWM
        commanded_throttle = ZERO_PWM + cmd_strength;
        break;
      default: //stop
        commanded_throttle = ZERO_PWM - 50 + ((cmd_strength - 100) / 10);
        stopped = true;
        break;
    }

    cmd_strength = (cmd_buffer[3] - 48 + 1) * INCREMENT;
    switch (cmd_buffer[2]) {
      case 'l': // left steer is between ZERO_PWM and MAX_PWM
        steering = ZERO_PWM + cmd_strength;
        break;
      case 'r': // right steer is between MIN_PWM and ZERO_PWM
        steering = ZERO_PWM - cmd_strength;
        break;
      default: //stop
        steering = ZERO_PWM - 50 + ((cmd_strength - 100) / 10);
        stopped = true;
    }

    if (true) {
      if (!stopped){
        Serial.print("ESP: Driving. Throttle: ");
        Serial.print(commanded_throttle);
        Serial.print(" Steering: ");
        Serial.print(steering);
      }
    else{
        // To investigate dead band s5 is ZERO_PWM while s0 is 
        // ZERO_PWM - 50 and S9 is ZERO_PWM + 50
        Serial.print("ESP: Stopped. Throttle: ");
        Serial.print(commanded_throttle);
        Serial.print(" Steering: ");
        Serial.print(steering);
      }
    }
  }
  else {
    digitalWrite(LED_PIN, LOW);
  }


  // state 0: wait to establish connection with RC controller
  if (state == 1){
    // pot read max value: 4096 (12bit)
    commanded_throttle = ZERO_PWM;
    applied_throttle = ZERO_PWM;
    steering = ZERO_PWM;

    // writes a duty cycle to the specified pwmchannel 
    // (which in this case was linked to pin 4)
    ledcWrite(PWM_CHNL_THROTTLE, applied_throttle);
    ledcWrite(PWM_CHNL_STEERING, steering);

    // Wait 2 seconds for RC controller to connect:
    current_time_ms = millis();
    if ((current_time_ms - start_time_ms) > WAIT_TIME_ms){
      state = 2;
    }
  }

  // stat 2: drive
  if (state == 2){
    applied_throttle = commanded_throttle;
    
    // For throttle control if the car is stopped we will accelerate above the
    // static friction threshold and then if necessary dial down the applied 
    // throttle to the commanded value:
    if ((prev_throttle == 0) && 
        (applied_throttle < MIN_THROTTLE_FROM_STOP) &&
        (jump_start == false)){
      // Accelerate above static friction for 100ms
      applied_throttle = START_THROTTLE;
      prev_time_ms = millis();
      jump_start = true;
    }

    // Check if we need to throttle-down:
    if (jump_start == true){
      current_time_ms = millis();
      if (current_time_ms - prev_time_ms > START_THROTTLE_TIME_ms){
        applied_throttle = commanded_throttle;
        jump_start = false;
      }
    }

    //Sanitize controls before sending them out to motors:
    applied_throttle = (applied_throttle <= MIN_PWM) ? MIN_PWM : applied_throttle;
    applied_throttle = (applied_throttle >= MAX_PWM) ? MAX_PWM : applied_throttle;
    steering = (steering <= MIN_PWM) ? MIN_PWM : steering;
    steering = (steering >= MAX_PWM) ? MAX_PWM : steering;

    // Send commands only when value change:
    if (prev_throttle != applied_throttle) {
      ledcWrite(PWM_CHNL_THROTTLE, applied_throttle);
      prev_throttle = applied_throttle;
    }
    if (prev_steering != steering) {
      ledcWrite(PWM_CHNL_STEERING, steering);
    }
  }

  iteration++;
}