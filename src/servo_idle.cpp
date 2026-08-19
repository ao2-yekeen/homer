#include <Arduino.h>

// Stop all control output from the ESP32. GPIO 2 is left high-impedance so
// the neck servo receives no PWM command. No motors or ROS are initialised.
void setup() {
  pinMode(2, INPUT);
}

void loop() {
  delay(1000);
}
