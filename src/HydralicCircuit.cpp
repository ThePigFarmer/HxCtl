#include "HydralicCircuit.h"

// init new instance and set pins
HydralicCircuit::HydralicCircuit(uint8_t caseSensorPin, uint8_t chargeSensorPin,
                                 uint8_t systemSensorPin, bool simulate) {
  this->caseSensorPin = caseSensorPin;
  this->chargeSensorPin = chargeSensorPin;
  this->systemSensorPin = systemSensorPin;

  simulatorMode = simulate;

  pinMode(this->caseSensorPin, INPUT);
  pinMode(this->chargeSensorPin, INPUT);
  pinMode(this->systemSensorPin, INPUT);
};

// loop read sensors, show data, convert to psi, and set public variables
void HydralicCircuit::run() {

  // read analog values (0-5v = 0-1023 in arduino analog input)
  uint16_t rawCasePressure = analogRead(caseSensorPin);
  uint16_t rawChargePressure = analogRead(chargeSensorPin);
  uint16_t rawSystemPressure = analogRead(systemSensorPin);

  // print raw stuff
  // Serial << "rawCsPress: " << rawCasePressure
  //       << "\nrawChrgPress: " << rawChargePressure << "\nrawSysPress: "
  //       << rawSystemPressure << "\n"

  if(simulatorMode) {
    casePSI = map(rawCasePressure, 0, 1023, 0, casePressureKp);
    chargePSI = map(rawChargePressure, 0, 1023, 0, chargePressureKp);
    systemPSI = map(rawSystemPressure, 0, 1023, 0, systemPressureKp);
  } else {
  // convert raw stuff to psi
  casePSI = rawInToPSI(rawCasePressure);
  chargePSI = rawInToPSI(rawChargePressure);
  systemPSI = rawInToPSI(rawSystemPressure);
}
  // print system stuff
  // Serial << "casePSI: " << casePSI << "\nchargPSI: " << chargePSI
  //        << "\nsysPSI: " << systemPSI << "\n";

  // alarms
  highCasePSI = (casePSI > MAX_CASE_PSI);
  lowChargePSI = (chargePSI < MIN_CHARGE_PSI);
}

// set the calibration "factors"
void HydralicCircuit::calibrate(uint8_t casePressureKp,
                                uint16_t chargePressureKp,
                                uint16_t systemPressureKp) {
  this->casePressureKp = casePressureKp;
  this->chargePressureKp = chargePressureKp;
  this->systemPressureKp = systemPressureKp;
}

uint16_t HydralicCircuit::rawInToPSI(uint16_t rawIn) {
  return uint16_t((rawIn - MIN_IN) * RAW_IN_TO_PSI_FACTOR);
}
