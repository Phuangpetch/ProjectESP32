/*
  ESP32 SMART MIXER + BLYNK IoT / Version 4.0
  Default: SAFE SIMULATION (ALL PHYSICAL OUTPUTS OFF)
  Board: ESP32 Dev Module

  60KTYZ AC MOTOR: GPIO25 -> K1 motor power, GPIO26 -> K2 direction.
  K1 / K2 are 12V coils driven by MOSFETs, ACTIVE HIGH GPIO.
  GPIO27 -> 12V DC water pump MOSFET (ACTIVE HIGH).
  SHT31 I2C: SDA 21, SCL 22. Float switch: GPIO33 to GND,
  CLOSED = water available. See validated electrical drawing.

  Warning: Only enable real hardware after professional review of
  220VAC reversing, emergency stop, protections and motor S1/S2 duty.
  SHT31 reads air relative humidity, NOT compost/material moisture.
*/

#define BLYNK_TEMPLATE_ID   "TMPL_REPLACE_ME"
#define BLYNK_TEMPLATE_NAME "ESP32 SMART MIXER"
#define BLYNK_AUTH_TOKEN    "REPLACE_WITH_YOUR_BLYNK_TOKEN"
#define WIFI_SSID           "YOUR_WIFI_NAME"
#define WIFI_PASS           "YOUR_WIFI_PASSWORD"

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_SHT31.h>
#include <math.h>

// ---------------------- HARDWARE PINS ----------------------
#define PIN_K1_MOTOR          25
#define PIN_K2_DIRECTION      26
#define PIN_PUMP              27
#define PIN_SHT_SDA           21
#define PIN_SHT_SCL           22
#define PIN_FLOAT            33
#define FLOAT_WATER_OK       LOW
#define OUTPUT_ON            HIGH   // MOSFET drivers: ACTIVE HIGH
#define OUTPUT_OFF           LOW
#define K2_CW_LEVEL          LOW    // K2 NC -> lower terminal, image Forward
#define K2_CCW_LEVEL         HIGH   // K2 NO -> upper terminal, image Reverse

// ----------------------- SAFETY MODES -----------------------
#define SIMULATION_MODE        1    // 1: generate demo values; all physical loads OFF
#define ENABLE_REAL_OUTPUTS    0    // 1: permit real GPIO outputs ONLY in real mode
#define MOTOR_DUTY_APPROVED    0    // 1 ONLY for motor approved for 30 min loaded duty
#define SIM_TIME_MULTIPLIER   60UL  // 30 real minutes -> 30 sec demonstration

// ------------------------ DEFAULTS -------------------------
#define DEFAULT_HUM_LOW       40
#define DEFAULT_HUM_HIGH      60
#define DEFAULT_AUTO_MODE      1    // 1=Auto, 0=Manual
#define DEFAULT_MIXER_ENABLE  1
#define DEFAULT_PUMP_ENABLE   1
#define MOTOR_WORK_MS         (30UL * 60UL * 1000UL)
#define MOTOR_REST_MS         (10UL * 60UL * 1000UL)
#define MOTOR_DIRECTION_MS    (60UL * 1000UL)
#define MOTOR_DEADTIME_MS      3000UL
#define CONTACT_SETTLE_MS      500UL
#define PUMP_MAX_ON_MS        (3UL * 60UL * 1000UL) // REAL-time safety limit
#define SENSOR_READ_MS         2000UL
#define SIM_SENSOR_MS          1000UL
#define BLYNK_VALUES_MS       5000UL
#define BLYNK_COUNTDOWN_MS   15000UL
#define BLYNK_STATUS_MS        1000UL
#define SERIAL_PRINT_MS       1000UL
#define NETWORK_RETRY_MS     10000UL
#define SHT_RETRY_MS         15000UL
#define SENSOR_STALE_MS      10000UL
#define RH_FILTER_ALPHA       0.35f

// ------------- BLYNK VIRTUAL PINS / DASHBOARD --------------
// V0  Master switch         Integer 0..1       RW
// V1  Mixer auto enable    Integer 0..1       RW
// V2  Pump auto enable     Integer 0..1       RW
// V3  Temperature (C)      Double  -40..125   Read
// V4  Air humidity (%)     Double  0..100     Read
// V5  Motor state          String             Read
// V6  Pump state           Integer 0..1       Read
// V7  Cycle phase          String             Read
// V8  Remaining seconds    Integer 0..3600    Read
// V9  Safety status        String             Read
// V10 Water OK             Integer 0..1       Read
// V11 AUTO mode            Integer 0..1       RW (0=Manual,1=Auto)
// V12 Manual motor         Integer 0..1       RW
// V13 Manual direction     Integer 0..1       RW (0=CW,1=CCW)
// V14 Manual pump          Integer 0..1       RW
// V15 Start pump RH (%)    Integer 10..90      RW (default 40)
// V16 Stop pump RH (%)     Integer 10..95      RW (default 60)
// V17 Test RH (%)          Integer 0..100      RW (default 35)
// V18 Test Temp (C)        Double  0..60       RW (default 29.5)
// V19 Test tank water      Integer 0..1       RW (default 1)
// V20 Test input manual    Integer 0..1       RW (0=auto demo,1=sliders)
// V21 Operating mode       String             Read
// V22 Countdown mm:ss      String             Read
// V23 Sensor OK            Integer 0..1       Read
// V24 Fault reset          Integer 0..1       Write (Push, system OFF)

Adafruit_SHT31 sht31 = Adafruit_SHT31();

bool masterOn = false;             // OFF on every boot, NEVER restore from cloud
bool autoMode = DEFAULT_AUTO_MODE;
bool mixerEnable = DEFAULT_MIXER_ENABLE;
bool pumpAutoEnable = DEFAULT_PUMP_ENABLE;
bool manualMotor = false;
bool manualPump = false;
bool manualCW = true;
int rhLow = DEFAULT_HUM_LOW;
int rhHigh = DEFAULT_HUM_HIGH;

bool simInputManual = false;
float testHumidity = 35.0f;
float testTemperature = 29.5f;
bool testWaterOK = true;
float demoHumidity = 35.0f;

bool shtFound = false;
bool sensorOK = false;
float temperatureC = NAN;
float humidityRH = NAN;
unsigned long lastSensorOK = 0;
unsigned long lastSensorRead = 0;
unsigned long lastShtRetry = 0;
uint8_t badSensorReadings = 0;

bool motorPowered = false;         // logical state; may be virtual in simulation
bool directionCW = true;
bool pumpPowered = false;          // logical state; may be virtual in simulation
bool pumpTimeout = false;
unsigned long pumpStartedReal = 0;

enum CyclePhase { CYCLE_OFF, CYCLE_WORK, CYCLE_REST };
CyclePhase cyclePhase = CYCLE_OFF;
uint32_t cycleStarted = 0;

enum MotorState { MOTOR_STOP, MOTOR_DEADTIME, MOTOR_SETTLE, MOTOR_CW, MOTOR_CCW };
MotorState motorState = MOTOR_STOP;
uint32_t motorStateAt = 0;
bool motorTargetCW = true;

unsigned long lastSimTick = 0;
unsigned long lastBlynkTelemetry = 0;
unsigned long lastBlynkCountdown = 0;
unsigned long lastBlynkStatus = 0;
unsigned long lastSerial = 0;
unsigned long lastNetworkTry = 0;
bool forceBlynk = true;
String prevMotor = "";
String prevCycle = "";
String prevFault = "";
String prevMode = "";
int prevPump = -1, prevWater = -1, prevSensor = -1;

// --------------------- CLOCK / OUTPUTS ---------------------
uint32_t controlNow() {
#if SIMULATION_MODE
  return (uint32_t)((uint64_t)millis() * SIM_TIME_MULTIPLIER);
#else
  return millis();
#endif
}

bool motorPermitted() {
#if SIMULATION_MODE
  return true;                    // VIRTUAL MOTOR ONLY
#else
  return (ENABLE_REAL_OUTPUTS && MOTOR_DUTY_APPROVED);
#endif
}

bool pumpPermitted() {
#if SIMULATION_MODE
  return true;                    // VIRTUAL PUMP ONLY
#else
  return ENABLE_REAL_OUTPUTS;
#endif
}

void writeLoad(uint8_t pin, bool active) {
#if !SIMULATION_MODE && ENABLE_REAL_OUTPUTS
  digitalWrite(pin, active ? OUTPUT_ON : OUTPUT_OFF);
#else
  digitalWrite(pin, OUTPUT_OFF);  // physical GPIO ALWAYS OFF in demo
#endif
}

void motorOff() {
  writeLoad(PIN_K1_MOTOR, false);
  motorPowered = false;
}

void applyDirection(bool clockwise) {
  if (motorPowered) return;       // NEVER change K2 when K1 is energized
#if !SIMULATION_MODE && ENABLE_REAL_OUTPUTS
  digitalWrite(PIN_K2_DIRECTION, clockwise ? K2_CW_LEVEL : K2_CCW_LEVEL);
#else
  digitalWrite(PIN_K2_DIRECTION, OUTPUT_OFF);
#endif
  directionCW = clockwise;
}

void motorOn() {
  if (!motorPermitted()) return;
  writeLoad(PIN_K1_MOTOR, true);
  motorPowered = true;
}

void pumpOff() {
  writeLoad(PIN_PUMP, false);
  pumpPowered = false;
}

void pumpOn() {
  if (pumpPowered || !pumpPermitted()) return;
  writeLoad(PIN_PUMP, true);
  pumpPowered = true;
  pumpStartedReal = millis();
}

bool waterOK() {
#if SIMULATION_MODE
  return testWaterOK;
#else
  return digitalRead(PIN_FLOAT) == FLOAT_WATER_OK;
#endif
}

bool validSensor() {
#if SIMULATION_MODE
  return sensorOK;
#else
  return sensorOK && ((uint32_t)(millis() - lastSensorOK) <= SENSOR_STALE_MS);
#endif
}

void stopMotorFSM() {
  motorOff();
  motorState = MOTOR_STOP;
}

void stopAll() {
  stopMotorFSM();
  pumpOff();
  cyclePhase = CYCLE_OFF;
}

void setMaster(bool enabled) {
  if (!enabled) {
    masterOn = false;
    stopAll();
    pumpTimeout = false;         // reset only via system OFF
    manualMotor = false;
    manualPump = false;
    if (Blynk.connected()) {
      Blynk.virtualWrite(V12, 0);
      Blynk.virtualWrite(V14, 0);
    }
  } else if (!masterOn) {
    masterOn = true;
    cyclePhase = CYCLE_OFF;       // fresh schedule, not old cloud state
  }
  forceBlynk = true;
}

// ----------------------- BLYNK INPUT -----------------------
BLYNK_WRITE(V0) { setMaster(param.asInt() == 1); }
BLYNK_WRITE(V1) {
  mixerEnable = (param.asInt() == 1);
  if (!mixerEnable && autoMode) { cyclePhase = CYCLE_OFF; stopMotorFSM(); }
}
BLYNK_WRITE(V2) {
  pumpAutoEnable = (param.asInt() == 1);
  if (!pumpAutoEnable && autoMode) pumpOff();
}
BLYNK_WRITE(V11) {
  bool newAuto = (param.asInt() == 1);
  if (newAuto != autoMode) {
    autoMode = newAuto;
    stopMotorFSM();
    pumpOff();
    cyclePhase = CYCLE_OFF;
    manualMotor = false;
    manualPump = false;
    if (Blynk.connected()) {
      Blynk.virtualWrite(V12, 0);
      Blynk.virtualWrite(V14, 0);
    }
  }
  forceBlynk = true;
}
BLYNK_WRITE(V12) {
  manualMotor = (masterOn && !autoMode && param.asInt() == 1);
  if (!manualMotor && Blynk.connected()) Blynk.virtualWrite(V12, 0);
}
BLYNK_WRITE(V13) { manualCW = (param.asInt() == 0); }
BLYNK_WRITE(V14) {
  manualPump = (masterOn && !autoMode && param.asInt() == 1);
  if (!manualPump && Blynk.connected()) Blynk.virtualWrite(V14, 0);
}
BLYNK_WRITE(V15) {
  int v = param.asInt();
  if (v >= 10 && v <= 90 && v < rhHigh) rhLow = v;
  else if (Blynk.connected()) Blynk.virtualWrite(V15, rhLow);
}
BLYNK_WRITE(V16) {
  int v = param.asInt();
  if (v >= 10 && v <= 95 && v > rhLow) rhHigh = v;
  else if (Blynk.connected()) Blynk.virtualWrite(V16, rhHigh);
}
BLYNK_WRITE(V17) {
  testHumidity = constrain(param.asFloat(), 0.0f, 100.0f);
}
BLYNK_WRITE(V18) {
  testTemperature = constrain(param.asFloat(), 0.0f, 60.0f);
}
BLYNK_WRITE(V19) { testWaterOK = (param.asInt() == 1); }
BLYNK_WRITE(V20) { simInputManual = (param.asInt() == 1); }
BLYNK_WRITE(V24) {
  if (param.asInt() == 1) {
    if (!masterOn) pumpTimeout = false;
    if (Blynk.connected()) Blynk.virtualWrite(V24, 0);
  }
}

BLYNK_CONNECTED() {
  // Push local safe settings, NEVER sync an old ON command from Blynk cloud.
  Blynk.virtualWrite(V0, masterOn ? 1 : 0);
  Blynk.virtualWrite(V1, mixerEnable ? 1 : 0);
  Blynk.virtualWrite(V2, pumpAutoEnable ? 1 : 0);
  Blynk.virtualWrite(V11, autoMode ? 1 : 0);
  Blynk.virtualWrite(V12, manualMotor ? 1 : 0);
  Blynk.virtualWrite(V13, manualCW ? 0 : 1);
  Blynk.virtualWrite(V14, manualPump ? 1 : 0);
  Blynk.virtualWrite(V15, rhLow);
  Blynk.virtualWrite(V16, rhHigh);
  Blynk.virtualWrite(V17, testHumidity);
  Blynk.virtualWrite(V18, testTemperature);
  Blynk.virtualWrite(V19, testWaterOK ? 1 : 0);
  Blynk.virtualWrite(V20, simInputManual ? 1 : 0);
  Blynk.virtualWrite(V24, 0);
  forceBlynk = true;
}

// ----------------- SIMULATED / REAL SENSOR -----------------
void updateSensor(unsigned long realNow) {
#if SIMULATION_MODE
  if ((uint32_t)(realNow - lastSimTick) < SIM_SENSOR_MS) return;
  lastSimTick = realNow;
  if (simInputManual) {
    temperatureC = testTemperature;
    humidityRH = testHumidity;
  } else {
    // In the demo humidity rises under spray and falls when spray is OFF.
    demoHumidity += pumpPowered ? 2.5f : -0.8f;
    demoHumidity = constrain(demoHumidity, 30.0f, 75.0f);
    humidityRH = demoHumidity;
    temperatureC = 29.5f + 0.5f * sinf((float)realNow / 8000.0f);
  }
  sensorOK = true;
  lastSensorOK = realNow;
#else
  if (!shtFound) {
    if ((uint32_t)(realNow - lastShtRetry) < SHT_RETRY_MS) return;
    lastShtRetry = realNow;
    shtFound = sht31.begin(0x44);
    if (!shtFound) { sensorOK = false; pumpOff(); return; }
  }
  if ((uint32_t)(realNow - lastSensorRead) < SENSOR_READ_MS) return;
  lastSensorRead = realNow;
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  if (isfinite(t) && isfinite(h) && t >= -40 && t <= 125 && h >= 0 && h <= 100) {
    temperatureC = t;
    humidityRH = sensorOK ? (RH_FILTER_ALPHA * h + (1.0f - RH_FILTER_ALPHA) * humidityRH) : h;
    sensorOK = true;
    badSensorReadings = 0;
    lastSensorOK = realNow;
  } else {
    if (badSensorReadings < 255) badSensorReadings++;
    if (badSensorReadings >= 3) {
      sensorOK = false;
      shtFound = false;
      lastShtRetry = realNow;
      pumpOff();
    }
  }
#endif
}

// ---------------------- MIXER SCHEDULE ---------------------
void updateCycle(uint32_t t) {
  if (!masterOn || !autoMode || !mixerEnable || !motorPermitted()) {
    cyclePhase = CYCLE_OFF;
    return;
  }
  if (cyclePhase == CYCLE_OFF) {
    cyclePhase = CYCLE_WORK;
    cycleStarted = t;
  } else if (cyclePhase == CYCLE_WORK && (uint32_t)(t - cycleStarted) >= MOTOR_WORK_MS) {
    cyclePhase = CYCLE_REST;
    cycleStarted = t;
  } else if (cyclePhase == CYCLE_REST && (uint32_t)(t - cycleStarted) >= MOTOR_REST_MS) {
    cyclePhase = CYCLE_WORK;
    cycleStarted = t;
  }
}

void updateMotor(bool desiredOn, bool desiredCW, uint32_t t) {
  if (!desiredOn || !motorPermitted()) {
    if (motorState != MOTOR_STOP) stopMotorFSM();
    return;
  }
  if (motorState == MOTOR_CW || motorState == MOTOR_CCW) {
    if (directionCW != desiredCW) {
      motorOff();                 // K1 OFF immediately
      motorTargetCW = desiredCW;
      motorState = MOTOR_DEADTIME;
      motorStateAt = t;
    }
    return;
  }
  if (motorState == MOTOR_STOP) {
    motorOff();
    motorTargetCW = desiredCW;
    motorState = MOTOR_DEADTIME;
    motorStateAt = t;
    return;
  }
  if (motorState == MOTOR_DEADTIME) {
    if (motorTargetCW != desiredCW) { motorTargetCW = desiredCW; motorStateAt = t; }
    if ((uint32_t)(t - motorStateAt) >= MOTOR_DEADTIME_MS) {
      applyDirection(motorTargetCW); // K2 changes only after deadtime
      motorState = MOTOR_SETTLE;
      motorStateAt = t;
    }
    return;
  }
  if (motorState == MOTOR_SETTLE) {
    if (motorTargetCW != desiredCW) {
      motorState = MOTOR_DEADTIME;
      motorStateAt = t;
    } else if ((uint32_t)(t - motorStateAt) >= CONTACT_SETTLE_MS) {
      motorOn();
      motorState = directionCW ? MOTOR_CW : MOTOR_CCW;
    }
  }
}

void updatePump() {
  if (!masterOn || !pumpPermitted() || !waterOK() || !validSensor() || pumpTimeout) {
    pumpOff();
    return;
  }
  bool wantPump = false;
  if (autoMode) {
    if (pumpAutoEnable) {
      if (pumpPowered) wantPump = (humidityRH < rhHigh);
      else wantPump = (humidityRH < rhLow);
    }
  } else {
    wantPump = manualPump;
  }
  if (!wantPump) { pumpOff(); return; }
  if (!pumpPowered) pumpOn();
  if (pumpPowered && PUMP_MAX_ON_MS > 0 &&
      (uint32_t)(millis() - pumpStartedReal) >= PUMP_MAX_ON_MS) {
    pumpOff();
    pumpTimeout = true;          // latched fault until master OFF/reset
  }
}

// ----------------------- STATUS / APP ----------------------
const char* motorLabel() {
  switch (motorState) {
    case MOTOR_CW:       return "CW";
    case MOTOR_CCW:      return "CCW";
    case MOTOR_DEADTIME: return "SAFE_STOP";
    case MOTOR_SETTLE:   return "RELAY_SETTLE";
    default:             return "OFF";
  }
}
const char* cycleLabel() {
  if (!masterOn) return "OFF";
  if (!autoMode) return "MANUAL";
  if (!mixerEnable) return "MIXER_DISABLED";
  if (!motorPermitted()) return "MOTOR_LOCKED";
  return (cyclePhase == CYCLE_WORK) ? "WORK" :
         (cyclePhase == CYCLE_REST) ? "REST" : "READY";
}
const char* warningLabel() {
  if (!masterOn) return "SYSTEM_OFF";
  if (!validSensor()) return "SENSOR_ERROR";
  if (pumpTimeout) return "PUMP_TIMEOUT";
  if (!waterOK()) return "WATER_LOW";
  if (!pumpPermitted()) return "OUTPUTS_LOCKED";
  if (!motorPermitted() && mixerEnable) return "MOTOR_DUTY_LOCK";
#if SIMULATION_MODE
  return "SIMULATION";
#else
  return "OK";
#endif
}
const char* modeLabel() {
#if SIMULATION_MODE
  return "SIM / NO PHYSICAL OUTPUT";
#else
  #if ENABLE_REAL_OUTPUTS
    return "REAL HARDWARE";
  #else
    return "REAL INPUT / OUTPUT LOCKED";
  #endif
#endif
}
uint32_t remainingSeconds(uint32_t t) {
  if (!autoMode || !masterOn || cyclePhase == CYCLE_OFF) return 0;
  uint32_t period = (cyclePhase == CYCLE_WORK) ? MOTOR_WORK_MS : MOTOR_REST_MS;
  uint32_t elapsed = (uint32_t)(t - cycleStarted);
  return (elapsed >= period) ? 0 : (period - elapsed + 999UL) / 1000UL;
}
String mmss(uint32_t s) {
  char txt[20];
  snprintf(txt, sizeof(txt), "%02lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
  return String(txt);
}

void sendBlynk(uint32_t t, unsigned long realNow) {
  if (!Blynk.connected()) return;
  if (forceBlynk || (uint32_t)(realNow - lastBlynkTelemetry) >= BLYNK_VALUES_MS) {
    lastBlynkTelemetry = realNow;
    if (validSensor()) {
      Blynk.virtualWrite(V3, temperatureC);
      Blynk.virtualWrite(V4, humidityRH);
    }
  }
  if (forceBlynk || (uint32_t)(realNow - lastBlynkCountdown) >= BLYNK_COUNTDOWN_MS) {
    lastBlynkCountdown = realNow;
    uint32_t remaining = remainingSeconds(t);
    Blynk.virtualWrite(V8, remaining);
    Blynk.virtualWrite(V22, mmss(remaining));
  }
  if (!forceBlynk && (uint32_t)(realNow - lastBlynkStatus) < BLYNK_STATUS_MS) return;
  lastBlynkStatus = realNow;
  String m = motorLabel(), c = cycleLabel(), f = warningLabel(), o = modeLabel();
  int p = pumpPowered ? 1 : 0;
  int w = waterOK() ? 1 : 0;
  int s = validSensor() ? 1 : 0;
  if (forceBlynk || m != prevMotor) { Blynk.virtualWrite(V5, m); prevMotor = m; }
  if (forceBlynk || p != prevPump) { Blynk.virtualWrite(V6, p); prevPump = p; }
  if (forceBlynk || c != prevCycle) { Blynk.virtualWrite(V7, c); prevCycle = c; }
  if (forceBlynk || f != prevFault) { Blynk.virtualWrite(V9, f); prevFault = f; }
  if (forceBlynk || w != prevWater) { Blynk.virtualWrite(V10, w); prevWater = w; }
  if (forceBlynk || o != prevMode) { Blynk.virtualWrite(V21, o); prevMode = o; }
  if (forceBlynk || s != prevSensor) { Blynk.virtualWrite(V23, s); prevSensor = s; }
  forceBlynk = false;
}

void printDebug(uint32_t t) {
  Serial.println("============================================");
  Serial.printf("SYSTEM       : %s\n", masterOn ? "ON" : "OFF");
  Serial.printf("MODE         : %s\n", autoMode ? "AUTO" : "MANUAL");
  Serial.printf("OUTPUT MODE  : %s\n", modeLabel());
  Serial.printf("TEMPERATURE  : %.1f C\n", temperatureC);
  Serial.printf("HUMIDITY     : %.1f %%\n", humidityRH);
  Serial.printf("RH CONTROL   : <%d ON / >=%d OFF\n", rhLow, rhHigh);
  Serial.printf("MOTOR        : %s\n", motorLabel());
  Serial.printf("CYCLE        : %s\n", cycleLabel());
  Serial.printf("REMAINING    : %s\n", mmss(remainingSeconds(t)).c_str());
  Serial.printf("WATER        : %s\n", waterOK() ? "OK" : "EMPTY");
  Serial.printf("PUMP         : %s\n", pumpPowered ? "ON" : "OFF");
  Serial.printf("SAFETY       : %s\n", warningLabel());
  Serial.printf("WIFI / BLYNK : %s / %s\n",
                WiFi.status() == WL_CONNECTED ? "ONLINE" : "OFFLINE",
                Blynk.connected() ? "ONLINE" : "OFFLINE");
}

void maintainConnection(unsigned long realNow) {
  if (WiFi.status() != WL_CONNECTED) {
    if ((uint32_t)(realNow - lastNetworkTry) >= NETWORK_RETRY_MS) {
      lastNetworkTry = realNow;
      WiFi.reconnect();
    }
    return;
  }
  if (Blynk.connected()) {
    Blynk.run();
  } else if ((uint32_t)(realNow - lastNetworkTry) >= NETWORK_RETRY_MS) {
    lastNetworkTry = realNow;
    Blynk.connect(500);
  }
}

// --------------------------- SETUP --------------------------
void setup() {
  Serial.begin(115200);
  // Latch OFF before enabling output mode, reducing startup glitches.
  digitalWrite(PIN_K1_MOTOR, OUTPUT_OFF);
  digitalWrite(PIN_K2_DIRECTION, K2_CW_LEVEL);
  digitalWrite(PIN_PUMP, OUTPUT_OFF);
  pinMode(PIN_K1_MOTOR, OUTPUT);
  pinMode(PIN_K2_DIRECTION, OUTPUT);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_FLOAT, INPUT_PULLUP);
  stopAll();

#if !SIMULATION_MODE
  Wire.begin(PIN_SHT_SDA, PIN_SHT_SCL);
  Wire.setClock(100000);
  shtFound = sht31.begin(0x44);
  lastShtRetry = millis();
#else
  sensorOK = true;
  temperatureC = 29.5f;
  humidityRH = 35.0f;
#endif

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Blynk.config(BLYNK_AUTH_TOKEN);

  Serial.println("ESP32 SMART MIXER v4 - BLYNK");
  Serial.println("BOOT: MASTER OFF; K1/K2/PUMP OFF");
#if SIMULATION_MODE
  Serial.println("SIMULATION ON: REAL OUTPUTS DISABLED");
#else
  if (!MOTOR_DUTY_APPROVED) Serial.println("WARNING: MOTOR LOCKED, CHECK DUTY RATING");
#endif
}

// ------------------------ MAIN LOOP -------------------------
void loop() {
  unsigned long realNow = millis();
  uint32_t t = controlNow();

  updateSensor(realNow);
  updateCycle(t);

  bool wantMotor = false;
  bool wantCW = true;
  if (masterOn && motorPermitted()) {
    if (autoMode && mixerEnable && cyclePhase == CYCLE_WORK) {
      wantMotor = true;
      uint32_t elapsed = (uint32_t)(t - cycleStarted);
      wantCW = ((elapsed / MOTOR_DIRECTION_MS) % 2UL) == 0;
    } else if (!autoMode && manualMotor) {
      wantMotor = true;
      wantCW = manualCW;
    }
  }
  updateMotor(wantMotor, wantCW, t);
  updatePump();
  maintainConnection(realNow);
  sendBlynk(t, realNow);

  if ((uint32_t)(realNow - lastSerial) >= SERIAL_PRINT_MS) {
    lastSerial = realNow;
    printDebug(t);
  }
}
