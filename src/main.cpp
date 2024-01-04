#include <Arduino.h>

#include "lcdPages.h"
#include "pins.h"
#include <AirSolinoid.h>
#include <Arduino.h>
#include <Button.h>
#include <DigitalOutput.h>
#include <HydralicCircuit.h>
#include <LiquidCrystal_I2C.h>


#define MAIN_LOOP_DELAY 2000
#define NUMBER_OF_HYDRALIC_CIRCUITS 4

// USE THESE FOR TESTING/DEVELOPING
// #define SHOW_BOTTOM_ROW_OF_LCD_IN_SERIAL_MONITOR
// #define SHOW_AIR_SOLINOID_STATUS
// #define SIMULATE_PRESSURE_SENSORS // use 0 - 5v for the psi range

// USE THESE TO SET OPTIONS
#define UI_BTN_HOLD_DELAY 500
#define RESUME_SWITCH_HOLD_DELAY 5000

// DON'T EDIT THESE!
#define SERIAL_BAUD_RATE monitor_speed
#define BTN_DEBOUNCE_DELAY 10
#ifdef SIMULATE_PRESSURE_SENSORS
#define SIMULATOR_MODE true
#else
#define SIMULATOR_MODE false
#endif

AirSolinoid airSolinoid(AIR_SOLINOID_PIN, false);

Button uiButton(UI_BUTTON_PIN, BTN_DEBOUNCE_DELAY, UI_BTN_HOLD_DELAY);
Button resumeSwitch(RESUME_SWITCH_PIN, BTN_DEBOUNCE_DELAY, RESUME_SWITCH_HOLD_DELAY);

DigitalOutput alarmBuzzer(ALARM_BUZZER_PIN, false);

HydralicCircuit chainRight(A0, A1, A2, SIMULATOR_MODE);
HydralicCircuit chainLeft(A3, A4, A5, SIMULATOR_MODE);
HydralicCircuit spinnerRight(A6, A7, A8, SIMULATOR_MODE);
HydralicCircuit spinnerLeft(A9, A10, A11, SIMULATOR_MODE);

// 16 * 4 lcd display
// LiquidCrystal lcd(12, 11, 10, 5, 4, 3, 2);
LiquidCrystal_I2C lcd(0x27, 20, 4);

// timer for loop
uint32_t mainLastLoopTime;

// charge pressure issue alerts for 2 seconds
uint32_t alertChargePressureStartTime;

// hydralic circuit stuff
void hydralicCircuitsSetup();
void hydralicCircuitsLoop();

// lcd stuff
uint16_t lcdBacklightBrightnessLevel[3] = {30, 90, 255};
void lcdSetup(uint8_t backlightPin);
void lcdSetupPage();
void lcdGotoNextBrightness();
void lcdGotoNextPage();
void lcdLoop();
void lcdUpdateBrightnessLevel();
// currents
int8_t lcdCurrentLcdBrightnessIndex = 2;
uint8_t lcdCurrentPage = CHARGE_PRESSURES_PAGE;

uint32_t ptoEngageTime;

// air solinoid stuff
void airSolinoidToggle();

void pressureMonitoringSystemLoop();

void casePressureLoop();
void chargePressureLoop();

bool caseAlert;
bool chargeAlert;

bool chargePressureFailed; // true if lowChargePSI occured

bool disableBuzzer = false;

void alarmLoop(bool caseAlert);

uint32_t alarmChargeAlertStartTime;
bool alertChargePressureIsRunning;

void userInputLoop();

void setup() {
  pinMode(LED_BUILTIN, 1);
  digitalWrite(LED_BUILTIN, 1); // show life
  Serial.begin(SERIAL_BAUD_RATE);

  Serial.print("started serial...\n");
  Serial.print(millis());
  airSolinoid.begin();
  airSolinoid.off(); // make sure it is off on startup
  Serial.print("started airsolinoid\n");
  Serial.print(millis());
  hydralicCircuitsSetup();
  Serial.print("started hydralic circuits setup...\n");
  Serial.print(millis());

  digitalWrite(LED_BUILTIN, 0); // show life
  lcdSetup(LCD_BACKLIGHT_PIN);
  digitalWrite(LED_BUILTIN, 1);

  Serial.print("started backlight\n");
  Serial.println(millis());

  lcdSetupPage();
}

void loop() {

  // hydralic circuit loop
  if ((millis() - mainLastLoopTime) > MAIN_LOOP_DELAY) {
    mainLastLoopTime = millis();
    hydralicCircuitsLoop();
    chargePressureLoop();
    casePressureLoop();

    lcdLoop();
  }

  alarmLoop(caseAlert); // more professional if this is instant

  userInputLoop();
}

void hydralicCircuitsSetup() {
// set calibration
#ifdef SIMULATE_PRESSURE_SENSORS
  chainRight.calibrate(99, 420, 4800);
  chainLeft.calibrate(99, 420, 4800);
  spinnerRight.calibrate(99, 420, 4800);
  spinnerLeft.calibrate(99, 420, 4800);
#endif
}

void hydralicCircuitsLoop() {
  chainRight.run();
  chainLeft.run();
  spinnerRight.run();
  spinnerLeft.run();
}

// -----------------------------------------------------------------------------
// lcd stuff here
// -----------------------------------------------------------------------------

void lcdGotoNextBrightness() {
  lcdCurrentLcdBrightnessIndex--;
  if (lcdCurrentLcdBrightnessIndex <= -1) {
    lcdCurrentLcdBrightnessIndex = 2;
  }

  lcdUpdateBrightnessLevel();
}

void lcdSetup(uint8_t backlightPin) {
  lcd.init();
  lcd.clear();

  // now start the pwm for variable brightness
  pinMode(backlightPin, OUTPUT);
  lcdUpdateBrightnessLevel();

  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PRECISION");
  lcd.setCursor(8, 1);
  lcd.print("AG");
  delay(200);
  lcd.setCursor(8, 1);
  lcd.print("  ");
  lcd.setCursor(8, 2);
  lcd.print("AG");
  delay(200);
  lcd.setCursor(8, 2);
  lcd.print("  ");
  lcd.setCursor(8, 1);
  lcd.print("AG");
  delay(200);
  lcd.setCursor(9, 2);
  lcd.print("SPREADERS");
  delay(300);
}

void lcdLoop() {
  char buffer[21]; // bottom row on lcd

  switch (lcdCurrentPage) {

  case CASE_PRESSURES_PAGE:
    // 18 chars
    sprintf(buffer, " %2u  %2u     %2u  %2u", spinnerLeft.casePSI,
            spinnerRight.casePSI, chainLeft.casePSI, chainRight.casePSI);
    break;

  case CHARGE_PRESSURES_PAGE:
    // 19 chars
    sprintf(buffer, " %3u %3u    %3u %3u", spinnerLeft.chargePSI,
            spinnerRight.chargePSI, chainLeft.chargePSI, chainRight.chargePSI);
    break;

  case SYSTEM_PRESSURES_PAGE:
    // 20 chars
    sprintf(buffer, "%4u %4u  %4u %4u", spinnerLeft.systemPSI,
            spinnerRight.systemPSI, chainLeft.systemPSI, chainRight.systemPSI);
    break;
  }

  lcd.setCursor(0, 3);
  lcd.print(buffer);

#ifdef SHOW_BOTTOM_ROW_OF_LCD_IN_SERIAL_MONITOR
  Serial.print("bottom of lcd: [");
  Serial.print(buffer);
  Serial.println("]");
#endif
}

void lcdSetupPage() {
  Serial.print("setting lcd page to: ");
  Serial.println(lcdCurrentPage);
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (lcdCurrentPage) {
  case CASE_PRESSURES_PAGE:
    lcd.println("   Case Pressures   ");
    break;

  case CHARGE_PRESSURES_PAGE:
    lcd.println("  Charge Pressures  ");
    break;

  case SYSTEM_PRESSURES_PAGE:
    lcd.println("  System Pressures  ");
    break;
  }

  lcd.setCursor(0, 1);
  lcd.print("spinners    chains");
  lcd.setCursor(0, 2);
  lcd.print("  L   R      L   R");
}

void lcdGotoNextPage() {
  lcdCurrentPage++;
  if (lcdCurrentPage >= 3) {
    lcdCurrentPage = 0;
  }

  lcdSetupPage();
}

// -----------------------------------------------------------------------------
// pressure monitoring system
// -----------------------------------------------------------------------------

void casePressureLoop() {
  if (airSolinoid.status() == HIGH) {

#ifdef SHOW_AIR_SOLINOID_STATUS
    Serial.print( "AirSolinoid is on\n";
#endif

    // see if bad things happened
    bool highCasePSIOccured = false;

    bool highCasePSI[4] = {spinnerLeft.highCasePSI, spinnerRight.highCasePSI,
                           chainLeft.highCasePSI, chainRight.highCasePSI};

    for (uint8_t i = 0; i < NUMBER_OF_HYDRALIC_CIRCUITS; i++) {
      // maybe there is a better way to do this

      if (highCasePSIOccured) {
        // do nothing... it has already done it before...
      } else if (highCasePSI[i]) {
        highCasePSIOccured = true;
      }
    }

    caseAlert = highCasePSIOccured; // it is more uniform this way

    // run alarm
  } else {
    caseAlert = false;
  }
}

void chargePressureLoop() {

  if (airSolinoid.status() == HIGH) {

    if ((millis() - ptoEngageTime) > 2000) {
      // see if bad things happened
      bool lowChargePSIOccured = false;

      bool lowChargePSI[4] = {spinnerLeft.lowChargePSI,
                              spinnerRight.lowChargePSI, chainLeft.lowChargePSI,
                              chainRight.lowChargePSI};

      for (uint8_t i = 0; i < NUMBER_OF_HYDRALIC_CIRCUITS; i++) {

        // maybe there is a better way to do this

        if (lowChargePSIOccured) {
          // do nothing... it has already done it before...
        } else if (lowChargePSI[i]) {
          lowChargePSIOccured = true;
        }
      }

      // here is where the stuff is done

      if (lowChargePSIOccured) {
        Serial.print("kill pto here\n");
        airSolinoid.off();
        chargePressureFailed = true;
      }

    } else {
      Serial.print("waiting for charge pressure to stabilize\n");
    }
  }
}

void userInputLoop() {

  // main pto on/off resume switch
  resumeSwitch.read();
  if (resumeSwitch.wasClicked()) {
    airSolinoidToggle();
  }

  if (resumeSwitch.wasHeld()) {
    disableBuzzer = !disableBuzzer;
    Serial.print("disable buzzer: ");
    Serial.println(disableBuzzer);
  }

  // display/page control button
  uiButton.read();
  if (uiButton.wasClicked()) {
    lcdGotoNextPage();
  }

  if (uiButton.wasHeld()) {
    lcdGotoNextBrightness();
  }
}

void alarmLoop(bool caseAlert) {

  if (chargePressureFailed) {
    if (!alertChargePressureIsRunning) {
      alertChargePressureStartTime = millis();
      alertChargePressureIsRunning = true;
      chargeAlert = true;
    }
  }

  if (alertChargePressureIsRunning) {

    if ((millis() - alertChargePressureStartTime) > 2000) {
      chargeAlert = false;
      chargePressureFailed = false;
      alertChargePressureIsRunning = false;
    }
  }

  bool shouldAlert = (caseAlert || chargeAlert);

  if (!disableBuzzer) {
  alarmBuzzer.write(shouldAlert);
  }

#ifdef SHOW_ALERTS
  Serial.print( "alert status: " << shouldAlert ? "it needs to beep" : "all is well" << endl;
#endif
}

void lcdUpdateBrightnessLevel() {
  analogWrite(LCD_BACKLIGHT_PIN,
              lcdBacklightBrightnessLevel[lcdCurrentLcdBrightnessIndex]);
  Serial.print("setting lcd backlight to: ");
  Serial.println(lcdBacklightBrightnessLevel[lcdCurrentLcdBrightnessIndex]);
}

void airSolinoidToggle() {
  airSolinoid.toggle();

  if (airSolinoid.status() == HIGH) {
    ptoEngageTime = millis();
  }
}
