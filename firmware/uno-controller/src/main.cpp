#include <Arduino.h>

/*
  Phase 2: Primary HVAC Controller (Arduino Uno)

  Overview:
  Controls an HVAC testbed based on temperature, CO2, and occupancy.
  Temperature drives fan duty, CO2 controls fresh air damper position.
  System operates only when occupied (derived from PIR motion with hold time).

  Control boundaries:
  The Arduino Uno is the sole authority for actuator control. Telemetry is emitted
  over serial for host-side logging and analysis.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <math.h>

#include "config.h"

// ======================================================================
//                      GLOBAL OBJECTS & STATE
// ======================================================================

Adafruit_BME280 bme;              // BME280 sensor object
Servo damperServo;                // Servo for damper control

bool bmeAvailable = false;        // BME280 successfully initialized?

unsigned long lastSampleMs = 0;   // Last time sensors were sampled
unsigned long lastMotionMs = 0;   // Last time PIR detected motion

SoftwareSerial T6613Serial(T6613_RX_PIN, T6613_TX_PIN);

// ======================================================================
//                   THERMISTOR CONFIGURATION
// ======================================================================

// NTC Thermistor Steinhart-Hart parameters
const float SERIES_RESISTOR     = 10000.0f; // 10kΩ series resistor
const float THERMISTOR_NOMINAL  = 10000.0f; // 10kΩ at 25°C
const float TEMPERATURE_NOMINAL = 25.0f;    // 25°C reference temp
const float B_COEFFICIENT       = 3950.0f;  // Beta coefficient for NTC
const float ADC_MAX             = 1023.0f;  // 10-bit ADC range

// ======================================================================
//                   FUNCTION DECLARATIONS
// ======================================================================

// Sensor reading functions
float readThermistorC();
bool  readBME280(float &tempC, float &humidity, float &pressure_hPa);
int   readT6613_CO2();

// Output control functions
void  runOutputSelfTest();
void  applyOutputs(uint8_t fanDuty, uint8_t damperAngle, bool occupied);

// Telemetry
void  printJson(float thermC, float bmeTempC, float bmeHum, float bmePress,
                int co2ppm, bool pirState, float indoorTempF,
                uint8_t fanDuty, uint8_t damperAngle, bool occupied);

// ======================================================================
//                                SETUP
// ======================================================================

void setup() {
  Serial.begin(115200);

  Serial.println(F("========================================"));
  Serial.println(F("Phase 2 HVAC Controller"));
  Serial.println(F("Primary: Arduino Uno"));
  Serial.println(F("========================================"));

  pinMode(PIR_PIN, INPUT);
  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(OCC_LED_PIN, OUTPUT);

  // Initialize outputs to safe state
  analogWrite(FAN_PWM_PIN, FAN_DUTY_OFF);
  digitalWrite(OCC_LED_PIN, LOW);

  damperServo.attach(DAMPER_SERVO_PIN);
  damperServo.write(DAMPER_CLOSED);

  Wire.begin();

  if (bme.begin(0x76)) {
    bmeAvailable = true;
    Serial.println(F("BME280 detected at 0x76"));
  } else if (bme.begin(0x77)) {
    bmeAvailable = true;
    Serial.println(F("BME280 detected at 0x77"));
  } else {
    Serial.println(F("BME280 not detected - thermistor will be used"));
  }

  T6613Serial.begin(19200);
  Serial.println(F("T6613 CO2 sensor initialized (19200 baud)"));

  Serial.println(F("Warming up sensors..."));
  delay(2000);

  runOutputSelfTest();

  Serial.println(F("========================================"));
  Serial.println(F("System ready - starting control loop"));
  Serial.println(F("========================================"));
}

// ======================================================================
//                                LOOP
// ======================================================================

void loop() {
  const unsigned long now = millis();

  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs = now;

  // Read thermistor temperature (always available)
  const float thermC = readThermistorC();

  // Read BME280 (temperature, humidity, pressure) if available
  float bmeTempC = NAN, bmeHum = NAN, bmePress = NAN;
  readBME280(bmeTempC, bmeHum, bmePress);

  // Read CO2 sensor (returns -1 on error)
  const int co2ppm = readT6613_CO2();

  // Read PIR state
  const bool pirState = (digitalRead(PIR_PIN) == HIGH);

  // Update last motion timestamp when PIR triggers
  if (pirState) {
    lastMotionMs = now;
  }

  // Derived occupancy: PIR active OR motion was recent
  const bool occupied = (now - lastMotionMs) <= OCCUPANCY_HOLD_MS;

  // Select primary temperature source
  const float indoorTempC = !isnan(bmeTempC) ? bmeTempC : thermC;

  float indoorTempF = NAN;
  if (!isnan(indoorTempC)) {
    indoorTempF = indoorTempC * 9.0f / 5.0f + 32.0f;
  }

  uint8_t fanDuty = FAN_DUTY_OFF;
  uint8_t damperAngle = DAMPER_CLOSED;

  if (occupied && !isnan(indoorTempF)) {

    // Temperature-based fan control
    if (indoorTempF <= (COOL_SETPOINT_F - COOL_DEADBAND_F)) {
      fanDuty = FAN_DUTY_OFF;
    } else if (indoorTempF >= (COOL_SETPOINT_F + COOL_DEADBAND_F)) {
      fanDuty = FAN_DUTY_HIGH;
    } else {
      fanDuty = FAN_DUTY_MED;
    }

    // CO2-based damper and fan override
    if (co2ppm >= 0) {
      if (co2ppm < CO2_LOW_PPM) {
        damperAngle = DAMPER_LOW;
      } else if (co2ppm < CO2_MED_PPM) {
        damperAngle = DAMPER_MED;
      } else {
        damperAngle = DAMPER_HIGH;
        if (fanDuty < FAN_DUTY_MED) {
          fanDuty = FAN_DUTY_MED;
        }
      }
    } else {
      // CO2 sensor failed: conservative default
      damperAngle = DAMPER_LOW;
    }

  } else {
    // Unoccupied or no valid temperature: idle outputs
    fanDuty = FAN_DUTY_OFF;
    damperAngle = DAMPER_CLOSED;
  }

  applyOutputs(fanDuty, damperAngle, occupied);

  printJson(thermC, bmeTempC, bmeHum, bmePress, co2ppm,
            pirState, indoorTempF, fanDuty, damperAngle, occupied);
}

// ======================================================================
//                        SENSOR FUNCTIONS
// ======================================================================

float readThermistorC() {
  const int adcValue = analogRead(THERMISTOR_PIN);

  // Open/short detection
  if (adcValue <= 0 || adcValue >= 1023) {
    return NAN;
  }

  // Compute resistance from divider ratio
  const float voltage_ratio = ADC_MAX / (float)adcValue;
  const float resistance = SERIES_RESISTOR / (voltage_ratio - 1.0f);

  float steinhart = resistance / THERMISTOR_NOMINAL;   // R/R0
  steinhart = log(steinhart);                          // ln(R/R0)
  steinhart /= B_COEFFICIENT;                          // (1/B)*ln(R/R0)
  steinhart += 1.0f / (TEMPERATURE_NOMINAL + 273.15f); // add 1/T0
  steinhart = 1.0f / steinhart;                        // invert to get T
  steinhart -= 273.15f;                                // Kelvin -> Celsius

  return steinhart;
}

bool readBME280(float &tempC, float &humidity, float &pressure_hPa) {
  if (!bmeAvailable) {
    tempC = NAN;
    humidity = NAN;
    pressure_hPa = NAN;
    return false;
  }

  tempC = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure_hPa = bme.readPressure() / 100.0f;

  return true;
}

int readT6613_CO2() {
  const byte readCmd[5] = {0xFF, 0xFE, 0x02, 0x02, 0x03};
  byte response[5];

  while (T6613Serial.available()) {
    T6613Serial.read();
  }

  T6613Serial.write(readCmd, 5);

  const unsigned long startTime = millis();
  int bytesRead = 0;

  while (bytesRead < 5 && (millis() - startTime) < CO2_TIMEOUT_MS) {
    if (T6613Serial.available()) {
      response[bytesRead++] = (byte)T6613Serial.read();
    }
  }

  if (bytesRead < 5) {
    return -1;
  }

  if (response[0] != 0xFF || response[1] != 0xFA || response[2] != 0x02) {
    return -1;
  }

  const int ppm = ((int)response[3] << 8) | response[4];
  return ppm;
}

// ======================================================================
//                      OUTPUT CONTROL FUNCTIONS
// ======================================================================

void runOutputSelfTest() {
  Serial.println(F("Output self-test: starting"));

  applyOutputs(FAN_DUTY_OFF, DAMPER_CLOSED, false);
  Serial.println(F("  [0/3] All outputs OFF"));
  delay(500);

  applyOutputs(FAN_DUTY_MED, DAMPER_LOW, true);
  Serial.println(F("  [1/3] Fan MED | Damper LOW | LED ON"));
  delay(1000);

  applyOutputs(FAN_DUTY_HIGH, DAMPER_MED, true);
  Serial.println(F("  [2/3] Fan HIGH | Damper MED | LED ON"));
  delay(1000);

  applyOutputs(FAN_DUTY_OFF, DAMPER_CLOSED, false);
  Serial.println(F("  [3/3] All outputs OFF"));

  delay(500);
}

void applyOutputs(uint8_t fanDuty, uint8_t damperAngle, bool occupied) {
  fanDuty = constrain(fanDuty, 0, 255);
  damperAngle = constrain(damperAngle, 0, 180);

  // Apply commanded fan duty cycle
  analogWrite(FAN_PWM_PIN, fanDuty);

  // Position commanded damper angle
  damperServo.write(damperAngle);

  if (LED_FOLLOWS_OCCUPANCY) {
    digitalWrite(OCC_LED_PIN, occupied ? HIGH : LOW);
  } else {
    digitalWrite(OCC_LED_PIN, LOW);
  }
}

// ======================================================================
//                           JSON OUTPUT
// ======================================================================

void printJson(
  float thermC,
  float bmeTempC,
  float bmeHum,
  float bmePress,
  int   co2ppm,
  bool  pirState,
  float indoorTempF,
  uint8_t fanDuty,
  uint8_t damperAngle,
  bool  occupied
) {
  const float thermF   = isnan(thermC)   ? NAN : (thermC   * 9.0f / 5.0f + 32.0f);
  const float bmeTempF = isnan(bmeTempC) ? NAN : (bmeTempC * 9.0f / 5.0f + 32.0f);

  Serial.print(F("{"));

  // Controller time for correlation across host reconnects
  Serial.print(F("\"ts_ms\":"));
  Serial.print(millis());

  Serial.print(F(",\"sensors\":{"));

  Serial.print(F("\"thermistorC\":"));
  isnan(thermC) ? Serial.print(F("null")) : Serial.print(thermC, 2);

  Serial.print(F(",\"thermistorF\":"));
  isnan(thermF) ? Serial.print(F("null")) : Serial.print(thermF, 2);

  Serial.print(F(",\"bmeTempC\":"));
  isnan(bmeTempC) ? Serial.print(F("null")) : Serial.print(bmeTempC, 2);

  Serial.print(F(",\"bmeTempF\":"));
  isnan(bmeTempF) ? Serial.print(F("null")) : Serial.print(bmeTempF, 2);

  Serial.print(F(",\"bmeHumidity\":"));
  isnan(bmeHum) ? Serial.print(F("null")) : Serial.print(bmeHum, 2);

  Serial.print(F(",\"bmePressure_hPa\":"));
  isnan(bmePress) ? Serial.print(F("null")) : Serial.print(bmePress, 2);

  Serial.print(F(",\"co2ppm\":"));
  (co2ppm < 0) ? Serial.print(F("null")) : Serial.print(co2ppm);

  Serial.print(F(",\"pir\":"));
  Serial.print(pirState ? F("true") : F("false"));

  Serial.print(F(",\"indoorTempF\":"));
  isnan(indoorTempF) ? Serial.print(F("null")) : Serial.print(indoorTempF, 2);

  Serial.print(F("}"));

  Serial.print(F(",\"outputs\":{"));

  Serial.print(F("\"fanDuty\":"));
  Serial.print(fanDuty);

  Serial.print(F(",\"damperAngle\":"));
  Serial.print(damperAngle);

  Serial.print(F(",\"occupied\":"));
  Serial.print(occupied ? F("true") : F("false"));

  Serial.print(F("}"));

  Serial.println(F("}"));
}
