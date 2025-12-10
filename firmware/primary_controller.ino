/*
  Phase 2: Primary HVAC Controller (Arduino Uno)
  
  OVERVIEW:
  Controls an HVAC system based on temperature, CO2, and occupancy.
  Temperature drives fan speed, CO2 controls fresh air damper position.
  System only operates when space is occupied (PIR motion detection).

  HARDWARE CONNECTIONS:
  Inputs:
    - Thermistor: A0 (10kΩ NTC in voltage divider: 5V -> 10kΩ -> A0 -> NTC -> GND)
    - BME280: I2C (A4=SDA, A5=SCL, 3.3V power)
    - T6613 CO2: UART via SoftwareSerial (D10=RX from sensor, D11=TX to sensor, 19200 baud)
    - PIR Motion: D2 (digital HIGH = motion detected)

  Outputs:
    - Fan: D5 (PWM to MOSFET gate, 0-255)
    - Damper: D9 (servo control, 0-180°)
    - Occupancy LED: D13 (indicates occupied state)

  CONTROL STRATEGY:
  - Occupied mode: Fan speed based on temperature deviation from setpoint
                   Damper position based on CO2 levels
                   High CO2 forces minimum medium fan speed
  - Unoccupied mode: All outputs idle (fan off, damper closed)
  - Occupancy persists for 30s after last motion detection
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <math.h>

// ======================================================================
//                         USER CONFIGURATION
// ======================================================================

// Temperature Control
const float COOL_SETPOINT_F   = 74.0f;  // Target indoor temperature (°F)
const float COOL_DEADBAND_F   = 2.0f;   // Dead band around setpoint (±°F)
                                        // Fan off below 72°F, high above 76°F

// Fan PWM Duty Cycles (0-255)
const uint8_t FAN_DUTY_OFF    = 0;      // Fan completely off
const uint8_t FAN_DUTY_LOW    = 80;     // ~31% duty cycle
const uint8_t FAN_DUTY_MED    = 160;    // ~63% duty cycle
const uint8_t FAN_DUTY_HIGH   = 255;    // 100% duty cycle

// CO2 Thresholds (ppm)
const uint16_t CO2_LOW_PPM    = 800;    // Below: minimal fresh air needed
const uint16_t CO2_MED_PPM    = 1200;   // Above: high fresh air + min medium fan
                                        // Range 800-1200: moderate fresh air

// Damper Servo Angles (degrees, 0-180)
const uint8_t DAMPER_CLOSED   = 0;      // No fresh air intake
const uint8_t DAMPER_LOW      = 45;     // Minimal fresh air
const uint8_t DAMPER_MED      = 90;     // Moderate fresh air
const uint8_t DAMPER_HIGH     = 120;    // Maximum fresh air

// Timing Configuration
const unsigned long SAMPLE_INTERVAL_MS  = 5000;   // Sensor read interval (5s)
const unsigned long OCCUPANCY_HOLD_MS   = 30000;  // Hold occupancy after motion (30s)
const unsigned long CO2_TIMEOUT_MS      = 1000;   // CO2 sensor UART timeout

// System Behavior
const bool LED_FOLLOWS_OCCUPANCY = true;  // LED indicates occupancy status

// ======================================================================
//                         PIN DEFINITIONS
// ======================================================================

// Analog Inputs
const uint8_t THERMISTOR_PIN  = A0;     // 10kΩ NTC thermistor voltage divider

// Digital Inputs
const uint8_t PIR_PIN         = 2;      // PIR motion sensor (HIGH = motion)

// Digital Outputs
const uint8_t FAN_PWM_PIN     = 5;      // PWM output to fan MOSFET
const uint8_t DAMPER_SERVO_PIN= 9;      // Servo control for fresh air damper
const uint8_t OCC_LED_PIN     = 13;     // Occupancy indicator LED

// CO2 Sensor UART (T6613)
const uint8_t T6613_RX_PIN    = 10;     // Arduino RX <- T6613 TX (Pin A)
const uint8_t T6613_TX_PIN    = 11;     // Arduino TX -> T6613 RX (Pin B)
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
//                      GLOBAL OBJECTS & STATE
// ======================================================================

Adafruit_BME280 bme;              // BME280 sensor object
Servo damperServo;                // Servo for damper control

bool bmeAvailable = false;        // BME280 successfully initialized?

unsigned long lastSampleMs = 0;   // Last time sensors were sampled
unsigned long lastMotionMs = 0;   // Last time PIR detected motion

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

// Utility functions
void  printJson(float thermC, float bmeTempC, float bmeHum, float bmePress,
                int co2ppm, bool pirState, float indoorTempF,
                uint8_t fanDuty, uint8_t damperAngle, bool occupied);

// ======================================================================
//                                SETUP
// ======================================================================

void setup() {
  // Initialize serial for debugging/data output
  Serial.begin(115200);
  while (!Serial) { ; }  // Wait for serial port (Leonardo/Micro only)

  Serial.println(F("========================================"));
  Serial.println(F("Phase 2 HVAC Controller"));
  Serial.println(F("========================================"));

  // Configure digital I/O pins
  pinMode(PIR_PIN, INPUT);          // PIR sensor input
  pinMode(FAN_PWM_PIN, OUTPUT);     // Fan PWM output
  pinMode(OCC_LED_PIN, OUTPUT);     // Occupancy LED
  
  // Initialize outputs to safe state
  analogWrite(FAN_PWM_PIN, 0);
  digitalWrite(OCC_LED_PIN, LOW);

  // Attach and center damper servo
  damperServo.attach(DAMPER_SERVO_PIN);
  damperServo.write(DAMPER_CLOSED);

  // Initialize I2C bus for BME280
  Wire.begin();
  
  // Try both common BME280 I2C addresses
  if (bme.begin(0x76)) {
    bmeAvailable = true;
    Serial.println(F("✓ BME280 detected at 0x76"));
  } else if (bme.begin(0x77)) {
    bmeAvailable = true;
    Serial.println(F("✓ BME280 detected at 0x77"));
  } else {
    Serial.println(F("⚠ BME280 not detected - will use thermistor only"));
  }

  // Initialize CO2 sensor UART
  T6613Serial.begin(19200);
  Serial.println(F("✓ T6613 CO2 sensor initialized (19200 baud)"));

  // Brief warmup delay for sensors
  Serial.println(F("Warming up sensors..."));
  delay(2000);

  // Test all outputs to verify connections
  runOutputSelfTest();

  Serial.println(F("========================================"));
  Serial.println(F("System ready - starting control loop"));
  Serial.println(F("========================================"));
}

// ======================================================================
//                                LOOP
// ======================================================================

void loop() {
  unsigned long now = millis();
  
  // Only run control logic at specified sample interval
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return;  // Not time yet
  }
  lastSampleMs = now;

  // ═══════════════════════════════════════════════════════════════════
  //                          READ ALL SENSORS
  // ═══════════════════════════════════════════════════════════════════
  
  // Read thermistor temperature (always available)
  float thermC = readThermistorC();

  // Read BME280 (temperature, humidity, pressure) if available
  float bmeTempC = NAN, bmeHum = NAN, bmePress = NAN;
  readBME280(bmeTempC, bmeHum, bmePress);

  // Read CO2 sensor (returns -1 on error)
  int co2ppm = readT6613_CO2();
  
  // Read PIR sensor state
  bool pirState = digitalRead(PIR_PIN);

  // ═══════════════════════════════════════════════════════════════════
  //                    DETERMINE OCCUPANCY STATE
  // ═══════════════════════════════════════════════════════════════════
  
  // Update last motion timestamp when PIR triggers
  if (pirState) {
    lastMotionMs = now;
  }

  // Consider occupied if PIR is active OR motion was recent
  bool occupied = (now - lastMotionMs) <= OCCUPANCY_HOLD_MS;

  // ═══════════════════════════════════════════════════════════════════
  //                  SELECT PRIMARY TEMPERATURE SOURCE
  // ═══════════════════════════════════════════════════════════════════
  
  // Prefer BME280 temperature (more accurate), fall back to thermistor
  float indoorTempC = !isnan(bmeTempC) ? bmeTempC : thermC;
  
  // Convert to Fahrenheit for control logic
  float indoorTempF = NAN;
  if (!isnan(indoorTempC)) {
    indoorTempF = indoorTempC * 9.0f / 5.0f + 32.0f;
  }

  // ═══════════════════════════════════════════════════════════════════
  //                        CONTROL LOGIC
  // ═══════════════════════════════════════════════════════════════════
  
  uint8_t fanDuty = FAN_DUTY_OFF;
  uint8_t damperAngle = DAMPER_CLOSED;

  if (occupied && !isnan(indoorTempF)) {
    
    // ─────────────────────────────────────────────────────────────────
    //              TEMPERATURE-BASED FAN CONTROL
    // ─────────────────────────────────────────────────────────────────
    // Below setpoint - deadband: Fan off (too cool)
    // Within deadband: Fan medium (maintaining temp)
    // Above setpoint + deadband: Fan high (too warm)
    
    if (indoorTempF <= (COOL_SETPOINT_F - COOL_DEADBAND_F)) {
      fanDuty = FAN_DUTY_OFF;
    } else if (indoorTempF >= (COOL_SETPOINT_F + COOL_DEADBAND_F)) {
      fanDuty = FAN_DUTY_HIGH;
    } else {
      fanDuty = FAN_DUTY_MED;
    }

    // ─────────────────────────────────────────────────────────────────
    //              CO2-BASED DAMPER & FAN OVERRIDE
    // ─────────────────────────────────────────────────────────────────
    // Low CO2 (<800 ppm): Minimal fresh air
    // Medium CO2 (800-1200 ppm): Moderate fresh air
    // High CO2 (>1200 ppm): Maximum fresh air + ensure fan circulates
    
    if (co2ppm >= 0) {  // Valid CO2 reading
      if (co2ppm < CO2_LOW_PPM) {
        damperAngle = DAMPER_LOW;
      } else if (co2ppm < CO2_MED_PPM) {
        damperAngle = DAMPER_MED;
      } else {
        // High CO2: max damper and ensure at least medium fan
        damperAngle = DAMPER_HIGH;
        if (fanDuty < FAN_DUTY_MED) {
          fanDuty = FAN_DUTY_MED;  // Override for air circulation
        }
      }
    } else {
      // CO2 sensor failed: use conservative default
      damperAngle = DAMPER_LOW;
    }
    
  } else {
    // ─────────────────────────────────────────────────────────────────
    //                    UNOCCUPIED OR NO TEMP
    // ─────────────────────────────────────────────────────────────────
    // When unoccupied or no valid temperature reading: idle all outputs
    fanDuty = FAN_DUTY_OFF;
    damperAngle = DAMPER_CLOSED;
  }

  // ═══════════════════════════════════════════════════════════════════
  //                    APPLY OUTPUTS & REPORT
  // ═══════════════════════════════════════════════════════════════════
  
  applyOutputs(fanDuty, damperAngle, occupied);
  
  printJson(thermC, bmeTempC, bmeHum, bmePress, co2ppm, 
            pirState, indoorTempF, fanDuty, damperAngle, occupied);
}

// ======================================================================
//                        SENSOR FUNCTIONS
// ======================================================================

/**
 * Read thermistor temperature using Steinhart-Hart equation
 * 
 * Circuit: 5V -> 10kΩ -> A0 -> NTC(10kΩ@25°C) -> GND
 * 
 * @return Temperature in Celsius, or NAN on error
 */
float readThermistorC() {
  int adcValue = analogRead(THERMISTOR_PIN);

  // Check for open/short circuit conditions
  if (adcValue <= 0 || adcValue >= 1023) {
    return NAN;
  }

  // Calculate NTC resistance from voltage divider
  // V_adc = V_supply * R_ntc / (R_series + R_ntc)
  // Therefore: R_ntc = R_series / (V_supply/V_adc - 1)
  float voltage_ratio = ADC_MAX / (float)adcValue;
  float resistance = SERIES_RESISTOR / (voltage_ratio - 1.0f);

  // Simplified Steinhart-Hart equation (B-parameter)
  // 1/T = 1/T0 + (1/B)*ln(R/R0)
  float steinhart = resistance / THERMISTOR_NOMINAL;   // R/R0
  steinhart = log(steinhart);                          // ln(R/R0)
  steinhart /= B_COEFFICIENT;                          // (1/B)*ln(R/R0)
  steinhart += 1.0f / (TEMPERATURE_NOMINAL + 273.15f); // add 1/T0
  steinhart = 1.0f / steinhart;                        // invert to get T
  steinhart -= 273.15f;                                // Kelvin -> Celsius

  return steinhart;
}

/**
 * Read BME280 temperature, humidity, and pressure
 * 
 * @param tempC Output: temperature in Celsius
 * @param humidity Output: relative humidity in %
 * @param pressure_hPa Output: barometric pressure in hPa
 * @return true if readings successful, false if BME280 unavailable
 */
bool readBME280(float &tempC, float &humidity, float &pressure_hPa) {
  if (!bmeAvailable) {
    tempC = NAN;
    humidity = NAN;
    pressure_hPa = NAN;
    return false;
  }

  tempC = bme.readTemperature();                    // °C
  humidity = bme.readHumidity();                    // %RH
  pressure_hPa = bme.readPressure() / 100.0f;       // Pa -> hPa

  return true;
}

/**
 * Read CO2 concentration from T6613 sensor via UART
 * 
 * Uses T6613 command protocol:
 * Command: [0xFF 0xFE 0x02 0x02 0x03] - Read gas PPM
 * Response: [0xFF 0xFA 0x02 HIGH_BYTE LOW_BYTE] - 5 bytes
 * 
 * @return CO2 concentration in ppm (0-5000 typical), or -1 on error
 */
int readT6613_CO2() {
  // T6613 "Read Gas PPM" command
  const byte readCmd[5] = {0xFF, 0xFE, 0x02, 0x02, 0x03};
  byte response[5];

  // Flush any stale data in receive buffer
  while (T6613Serial.available()) {
    T6613Serial.read();
  }

  // Send read command
  T6613Serial.write(readCmd, 5);

  // Wait for 5-byte response with timeout
  unsigned long startTime = millis();
  int bytesRead = 0;

  while (bytesRead < 5 && (millis() - startTime) < CO2_TIMEOUT_MS) {
    if (T6613Serial.available()) {
      response[bytesRead++] = T6613Serial.read();
    }
  }

  // Verify we received complete response
  if (bytesRead < 5) {
    return -1;  // Timeout - sensor not responding
  }

  // Validate response header
  // Expected: [0xFF 0xFA 0x02 DATA_HIGH DATA_LOW]
  if (response[0] != 0xFF || response[1] != 0xFA || response[2] != 0x02) {
    return -1;  // Invalid response format
  }

  // Extract 16-bit CO2 value (big-endian)
  int ppm = ((int)response[3] << 8) | response[4];
  
  return ppm;
}

// ======================================================================
//                      OUTPUT CONTROL FUNCTIONS
// ======================================================================

/**
 * Run a brief self-test sequence to verify all outputs are working
 * Useful for confirming hardware connections during startup
 */
void runOutputSelfTest() {
  Serial.println(F("\n--- Output Self-Test Sequence ---"));

  // Step 0: Everything off
  applyOutputs(FAN_DUTY_OFF, DAMPER_CLOSED, false);
  Serial.println(F("  [0/3] All outputs OFF"));
  delay(500);

  // Step 1: Medium operation
  applyOutputs(FAN_DUTY_MED, DAMPER_LOW, true);
  Serial.println(F("  [1/3] Fan: MED | Damper: LOW | LED: ON"));
  delay(1000);

  // Step 2: High operation
  applyOutputs(FAN_DUTY_HIGH, DAMPER_MED, true);
  Serial.println(F("  [2/3] Fan: HIGH | Damper: MED | LED: ON"));
  delay(1000);

  // Step 3: Return to idle
  applyOutputs(FAN_DUTY_OFF, DAMPER_CLOSED, false);
  Serial.println(F("  [3/3] All outputs OFF\n"));
  delay(500);
}

/**
 * Apply control outputs with bounds checking
 * 
 * @param fanDuty PWM duty cycle for fan (0-255)
 * @param damperAngle Servo angle for damper (0-180 degrees)
 * @param occupied Occupancy state (controls LED if LED_FOLLOWS_OCCUPANCY)
 */
void applyOutputs(uint8_t fanDuty, uint8_t damperAngle, bool occupied) {
  // Safety bounds checking (should never trigger with proper constants)
  fanDuty = constrain(fanDuty, 0, 255);
  damperAngle = constrain(damperAngle, 0, 180);

  // Apply fan PWM
  analogWrite(FAN_PWM_PIN, fanDuty);
  
  // Position damper servo
  damperServo.write(damperAngle);

  // Control occupancy LED
  if (LED_FOLLOWS_OCCUPANCY) {
    digitalWrite(OCC_LED_PIN, occupied ? HIGH : LOW);
  } else {
    digitalWrite(OCC_LED_PIN, LOW);  // LED always off if not following occupancy
  }
}

// ======================================================================
//                           JSON OUTPUT
// ======================================================================

/**
 * Output system state as JSON for logging/monitoring
 * Prints one JSON object per line with all sensor readings and output states
 */
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
  // Convert temperatures to Fahrenheit
  float thermF   = isnan(thermC)   ? NAN : (thermC   * 9.0f / 5.0f + 32.0f);
  float bmeTempF = isnan(bmeTempC) ? NAN : (bmeTempC * 9.0f / 5.0f + 32.0f);

  Serial.print(F("{"));

  // ───────────────────────────────────────────────────────────────────
  //                         SENSOR READINGS
  // ───────────────────────────────────────────────────────────────────
  Serial.print(F("\"sensors\":{"));

  // Thermistor readings
  Serial.print(F("\"thermistorC\":"));
  isnan(thermC) ? Serial.print(F("null")) : Serial.print(thermC, 2);
  Serial.print(F(",\"thermistorF\":"));
  isnan(thermF) ? Serial.print(F("null")) : Serial.print(thermF, 2);

  // BME280 temperature
  Serial.print(F(",\"bmeTempC\":"));
  isnan(bmeTempC) ? Serial.print(F("null")) : Serial.print(bmeTempC, 2);
  Serial.print(F(",\"bmeTempF\":"));
  isnan(bmeTempF) ? Serial.print(F("null")) : Serial.print(bmeTempF, 2);

  // BME280 humidity
  Serial.print(F(",\"bmeHumidity\":"));
  isnan(bmeHum) ? Serial.print(F("null")) : Serial.print(bmeHum, 2);

  // BME280 pressure
  Serial.print(F(",\"bmePressure_hPa\":"));
  isnan(bmePress) ? Serial.print(F("null")) : Serial.print(bmePress, 2);

  // CO2 sensor
  Serial.print(F(",\"co2ppm\":"));
  (co2ppm < 0) ? Serial.print(F("null")) : Serial.print(co2ppm);

  // PIR motion
  Serial.print(F(",\"pir\":"));
  Serial.print(pirState ? F("true") : F("false"));

  // Selected indoor temperature (for control)
  Serial.print(F(",\"indoorTempF\":"));
  isnan(indoorTempF) ? Serial.print(F("null")) : Serial.print(indoorTempF, 2);

  Serial.print(F("}"));  // End sensors

  // ───────────────────────────────────────────────────────────────────
  //                         OUTPUT STATES
  // ───────────────────────────────────────────────────────────────────
  Serial.print(F(",\"outputs\":{"));

  Serial.print(F("\"fanDuty\":"));
  Serial.print(fanDuty);

  Serial.print(F(",\"damperAngle\":"));
  Serial.print(damperAngle);

  Serial.print(F(",\"occupied\":"));
  Serial.print(occupied ? F("true") : F("false"));

  Serial.print(F("}"));  // End outputs

  Serial.println(F("}"));  // End JSON object
}