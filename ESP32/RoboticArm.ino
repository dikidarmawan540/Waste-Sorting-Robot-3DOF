#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "pump.h"
#include "INA219.h"
#include "kineamtic.h"
#include "motion.h"
#include "command.h"

PumpController pump(PUMP_RELAY_PIN, PUMP_RELAY_ACTIVE_HIGH);
INA219Monitor ina219;
RobotKinematic kinematic;
MotionController motion;
CommandProcessor command(motion, kinematic, pump, ina219, Serial);

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  delay(500);

  Serial.println();
  Serial.println(F("=========================================="));
  Serial.println(F("Pemilah Sampah ESP32 Controller"));
  Serial.println(F("=========================================="));

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  pump.begin();
  motion.begin();

  if (ina219.begin()) {
    Serial.println(F("INA219 detected."));
  } else {
    Serial.println(F("WARNING: INA219 not detected. Touch detection disabled."));
  }

  command.begin();
  Serial.println(F(SERIAL_READY_TOKEN));
  Serial.println(F("EVENT:ESP32_READY"));
  Serial.println(F("Ready. Type HELP in Serial Monitor."));
}

void loop() {
  command.update();
  motion.run();
}
