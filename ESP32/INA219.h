#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "config.h"

// =====================================================================
// Semua parameter dan logika kalibrasi INA219 ada di sini.
// config.h hanya menyimpan parameter non-kalibrasi (threshold, samples,
// alamat I2C). Untuk update kalibrasi: ubah CAL_OFFSET dan CAL_SLOPE
// di bawah ini saja, tidak perlu ubah file lain.
// =====================================================================

// Rumus kalibrasi (dari regresi linear data multimeter vs raw INA219):
//   calibrated = (raw - CAL_OFFSET) / CAL_SLOPE
// Sesuai script referensi: (avgCurrent - 0.0031) / 0.9722
static constexpr float CAL_OFFSET = 0.0031f;  // offset (intercept)
static constexpr float CAL_SLOPE  = 0.9722f;  // slope  (divisor)

class INA219Monitor {
public:
  INA219Monitor()
    : _ina(INA219_I2C_ADDR), _ready(false) {}

  bool begin() {
    _ready = _ina.begin();
    return _ready;
  }

  bool isReady() const {
    return _ready;
  }

  // Nilai mentah langsung dari chip, tanpa kalibrasi apapun.
  float rawCurrentA() {
    if (!_ready) return 0.0f;
    return _ina.getCurrent_mA() / 1000.0f;
  }

  // Nilai setelah regresi linear: (raw - CAL_OFFSET) / CAL_SLOPE.
  // Aktif permanen — ubah CAL_OFFSET/CAL_SLOPE di atas jika perlu re-kalibrasi.
  float calibratedCurrentA() {
    return (rawCurrentA() - CAL_OFFSET) / CAL_SLOPE;
  }

  // Rata-rata dari rawCurrentA() — untuk mengumpulkan data kalibrasi baru
  // (bandingkan INAavg dengan multimeter, lalu hitung CAL_OFFSET/CAL_SLOPE baru).
  float averageRawCurrentA(int samples = INA_CAL_SAMPLES, uint32_t sampleDelayMs = INA_CAL_SAMPLE_DELAY_MS) {
    if (!_ready || samples <= 0) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < samples; i++) {
      sum += rawCurrentA();
      delay(sampleDelayMs);
    }
    return sum / samples;
  }

  // Rata-rata dari calibratedCurrentA() — dipakai untuk streaming/cek kalibrasi.
  float averageCurrentA(int samples = INA_CAL_SAMPLES, uint32_t sampleDelayMs = INA_CAL_SAMPLE_DELAY_MS) {
    if (!_ready || samples <= 0) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < samples; i++) {
      sum += calibratedCurrentA();
      delay(sampleDelayMs);
    }
    return sum / samples;
  }


  void printStatus(Stream& out) {
    out.print(F("INA219 ready="));
    out.print(_ready ? F("YES") : F("NO"));
    out.print(F(" raw_A="));
    out.print(rawCurrentA(), 4);
    out.print(F(" calibrated_A="));
    out.print(calibratedCurrentA(), 4);
    out.print(F(" threshold_A="));
    out.println(INA_TOUCH_THRESHOLD_A, 4);
  }

private:
  Adafruit_INA219 _ina;
  bool _ready;
};
