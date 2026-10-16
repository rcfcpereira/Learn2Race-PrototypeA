// References (full entries: docs/references.md at the repository root):
//   [R14] micro-ROS / micro_ros_arduino: micro-ROS client
//   [R63] ESP32 TRM / Arduino-ESP32 / PlatformIO: ESP32 Arduino
// More details: firmware/README.md

// Prototype A firmware entry point.
//
// Waveshare General Driver for Robots (ESP32) with two CHP-20GP-180ABHL geared
// motors. Everything of substance lives in RobotApp; this file only wires the
// Arduino entry points to it.

#include <Arduino.h>

#include "RobotApp.h"

void setup() {
  if (!g_app.begin()) {
    // begin() only fails if the serial transport cannot be installed, which is
    // not recoverable. Reboot rather than run with no link to the agent.
    delay(1000);
    ESP.restart();
  }
}

void loop() {
  g_app.spin();
}
