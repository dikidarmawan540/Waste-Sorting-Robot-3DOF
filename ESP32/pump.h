#pragma once

#include <Arduino.h>

class PumpController {
public:
  PumpController(uint8_t relayPin, bool activeHigh)
    : _relayPin(relayPin), _activeHigh(activeHigh), _isOn(false) {}

  void begin() {
    pinMode(_relayPin, OUTPUT);
    off();
  }

  void on() {
    _isOn = true;
    digitalWrite(_relayPin, _activeHigh ? HIGH : LOW);
  }

  void off() {
    _isOn = false;
    digitalWrite(_relayPin, _activeHigh ? LOW : HIGH);
  }

  void toggle() {
    if (_isOn) off();
    else on();
  }

  bool isOn() const {
    return _isOn;
  }

private:
  uint8_t _relayPin;
  bool _activeHigh;
  bool _isOn;
};
