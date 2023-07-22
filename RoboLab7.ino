#include "Buzzer.h"
#include "ESPMax.h"
#include "_espmax.h"
#include "ESP32PWMServo.h"
#include "SuctionNozzle.h"
#include "LobotSerialServoControl.h"

// Sound sensor placement

#define sensor_pin 32  // Define the pin for the sound sensor

void setup() {
  // Initialize driver libraries
  Buzzer_init();
  ESPMax_init();
  Nozzle_init();
  PWMServo_init();
  analogReadResolution(10);
  analogSetClockDiv(ADC_11db);
  Serial.begin(115200);
  Serial.println("start...");
  setBuzzer(100);              // Set the buzzer to sound for 100 milliseconds
  go_home(2000);               // Move the robotic arm to the initial position
  SetPWMServo(1, 1500, 2000);  // Set the suction nozzle servo to the initial position
}

void loop() {
  int num = 0;
  float pos[3];
  bool num_st = false;
  long int time_ms = millis();
  int angle_pul[3] = { 1800, 2000, 2200 };

  while (true) {
    float soundValue = analogRead(sensor_pin);
    if (soundValue > 50) {
      if (num == 0 || (millis() - time_ms) < 1000) {
        time_ms = millis();
        delay(80);
        num += 1;
      }
      Serial.println(soundValue);
    }

    if (num > 0 && (millis() - time_ms) > 1500) {
      if (num > 3) num = 3;
      num_st = true;
      Serial.println(num);
    }

    if (num_st) {    
      setBuzzer(100);  // Set the buzzer to sound for 100ms
      pos[0] = 0;
      pos[1] = -160;
      pos[2] = 100;
      set_position(pos, 1500);  // Move the arm above the colored block
      delay(1500);
      pos[0] = 0;
      pos[1] = -160;
      pos[2] = 86;
      set_position(pos, 800);  // Engage the suction nozzle to pick up the colored block
      Pump_on();               // Turn on the air pump to create suction
      delay(1000);
      pos[0] = 0;
      pos[1] = -160;
      pos[2] = 180;
      set_position(pos, 1000);  // Lift the robotic arm with the block
      delay(1000);
      pos[0] = 120;
      pos[1] = (-20 - 60 * (num - 1));
      pos[2] = 180;
      set_position(pos, 1500);  // Move the arm above the placement area
      delay(100);
      SetPWMServo(1, angle_pul[(num - 1)], 1000);  // Set the suction nozzle angle to align the block
      delay(500);
      pos[0] = 120;
      pos[1] = (-20 - 60 * (num - 1));
      pos[2] = 88;
      set_position(pos, 1000);  // Place the block in the designated area
      delay(1200);
      Valve_on();  // Turn off the air pump and open the electromagnetic valve
      pos[0] = 120;
      pos[1] = (-20 - 60 * (num - 1));
      pos[2] = 200;
      set_position(pos, 1000);  // Lift the robotic arm
      delay(1000);
      Valve_off();    // Close the electromagnetic valve
      go_home(1500);  // Reset the robotic arm to the initial position
      delay(100);
      SetPWMServo(1, 1500, 1500);  // Reset the suction nozzle angle
      num_st = false;
      num = 0;
    } else {
      delay(20);
    }
  }
}
