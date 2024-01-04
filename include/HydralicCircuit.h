#ifndef HYDRALIC_CIRCUIT_h
#define HYDRALIC_CIRCUIT_h

#define MAX_CASE_PSI 75
#define MIN_CHARGE_PSI 250

#define MIN_IN 102.3
#define RAW_IN_TO_PSI_FACTOR 6.1

#include <Arduino.h>

class HydralicCircuit {
public:
  HydralicCircuit(uint8_t caseSensorPin, uint8_t chargeSensorPin,
                  uint8_t systemSensorPin, bool simulate);

  // alarms
  bool highCasePSI;
  bool lowChargePSI;

  uint8_t casePSI;
  uint16_t chargePSI;
  uint16_t systemPSI;
  uint16_t rawInToPSI(uint16_t rawIn);

  void run();
  void calibrate(uint8_t casePressureKp, uint16_t chargePressureKp,
                 uint16_t systemPressureKp);

private:
  bool simulatorMode;

  uint8_t caseSensorPin;
  uint8_t chargeSensorPin;
  uint8_t systemSensorPin;

  uint16_t casePressureKp;
  uint16_t chargePressureKp;
  uint16_t systemPressureKp;
};

#endif
