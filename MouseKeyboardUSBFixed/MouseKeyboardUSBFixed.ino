#include <Wire.h>
#include "USB.h"
#include "USBHIDMouse.h"
#include "USBHIDKeyboard.h"

// =====================================================
// USB HID
// =====================================================
USBHIDMouse mouse;
USBHIDKeyboard keyboard;

// =====================================================
// Shared I2C pins
// =====================================================
constexpr uint8_t SDA_PIN = 8;
constexpr uint8_t SCL_PIN = 9;

// =====================================================
// MPR121 configuration
// =====================================================
constexpr uint8_t MPR121_ADDR = 0x5A;

// status / data
constexpr uint8_t TOUCHSTATUS_L = 0x00;
constexpr uint8_t TOUCHSTATUS_H = 0x01;
constexpr uint8_t OOR_STATUS_L  = 0x02;
constexpr uint8_t OOR_STATUS_H  = 0x03;

// filtered data
constexpr uint8_t FILTDATA_0L   = 0x04;
constexpr uint8_t FILTDATA_0H   = 0x05;

// baseline data
constexpr uint8_t BASELINE_0    = 0x1E;

// filtering registers
constexpr uint8_t MHD_R         = 0x2B;
constexpr uint8_t NHD_R         = 0x2C;
constexpr uint8_t NCL_R         = 0x2D;
constexpr uint8_t FDL_R         = 0x2E;
constexpr uint8_t MHD_F         = 0x2F;
constexpr uint8_t NHD_F         = 0x30;
constexpr uint8_t NCL_F         = 0x31;
constexpr uint8_t FDL_F         = 0x32;
constexpr uint8_t NHD_T         = 0x33;
constexpr uint8_t NCL_T         = 0x34;
constexpr uint8_t FDL_T         = 0x35;

// thresholds / debounce / config
constexpr uint8_t ELE0_T        = 0x41;
constexpr uint8_t ELE0_R        = 0x42;
constexpr uint8_t DEBOUNCE_REG  = 0x5B;
constexpr uint8_t FIL_CFG_1     = 0x5C;
constexpr uint8_t FIL_CFG_2     = 0x5D;
constexpr uint8_t ELE_CFG       = 0x5E;
constexpr uint8_t SOFTRESET     = 0x80;

// =====================================================
// MPR121 tuning
// =====================================================
constexpr uint8_t TOUCH_THRESHOLD   = 40;   // more sensitive than before
constexpr uint8_t RELEASE_THRESHOLD = 20;
constexpr uint8_t DEBOUNCE_VALUE    = 0x11; // touch debounce=1, release debounce=1

// =====================================================
// Cirque TM035035 / Pinnacle I2C configuration
// =====================================================
constexpr uint8_t DR_PIN  = 10;
constexpr uint8_t SLAVE_ADDR = 0x2A;

constexpr uint8_t WRITE_MASK = 0x80;
constexpr uint8_t READ_MASK  = 0xA0;

constexpr uint8_t STATUS1_ADDR      = 0x02;
constexpr uint8_t SYSCONFIG_1_ADDR  = 0x03;
constexpr uint8_t FEEDCONFIG_1_ADDR = 0x04;
constexpr uint8_t FEEDCONFIG_2_ADDR = 0x05;
constexpr uint8_t Z_IDLE_ADDR       = 0x0A;
constexpr uint8_t PACKETBYTE_0_ADDR = 0x12;

constexpr uint8_t SYSCONFIG_1_DATA  = 0x00;
constexpr uint8_t FEEDCONFIG_1_DATA = 0x81; // relative mode
constexpr uint8_t FEEDCONFIG_2_DATA = 0x1C;
constexpr uint8_t Z_IDLE_COUNT      = 0x05;

// =====================================================
// Touch packet structure
// =====================================================
struct RelData {
  uint8_t buttons;
  int16_t xDelta;
  int16_t yDelta;
  int8_t wheel;
};

// =====================================================
// State
// =====================================================
uint16_t lastTouched = 0;

int accumulatedRawX = 0;
int accumulatedRawY = 0;

unsigned long lastSendMs = 0;
constexpr unsigned long SEND_INTERVAL_MS = 8;

constexpr int DEADZONE = 0;
constexpr int MAX_MOVE_PER_REPORT = 35;

// debug timing
unsigned long lastDebugMs = 0;
constexpr unsigned long DEBUG_INTERVAL_MS = 300;

// =====================================================
// Utility
// =====================================================
int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

// =====================================================
// MPR121 low-level
// =====================================================
bool mprWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPR121_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mprReadRegister(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(MPR121_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPR121_ADDR, (uint8_t)1) != 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

bool mprReadTouchStatus(uint16_t &status) {
  uint8_t lowByte = 0;
  uint8_t highByte = 0;

  if (!mprReadRegister(TOUCHSTATUS_L, lowByte)) return false;
  if (!mprReadRegister(TOUCHSTATUS_H, highByte)) return false;

  status = ((uint16_t)highByte << 8) | lowByte;
  status &= 0x0FFF;
  return true;
}

uint16_t mprReadFilteredData(uint8_t electrode) {
  if (electrode > 11) return 0;

  uint8_t lsb = 0;
  uint8_t msb = 0;
  uint8_t reg = FILTDATA_0L + electrode * 2;

  if (!mprReadRegister(reg, lsb)) return 0;
  if (!mprReadRegister(reg + 1, msb)) return 0;

  return (((uint16_t)(msb & 0x03)) << 8) | lsb;
}

uint16_t mprReadBaselineData(uint8_t electrode) {
  if (electrode > 11) return 0;

  uint8_t value = 0;
  if (!mprReadRegister(BASELINE_0 + electrode, value)) return 0;

  // datasheet: baseline register exposes upper 8 bits only, shift left by 2
  return ((uint16_t)value) << 2;
}

bool mprReadOOR(uint8_t &oorLow, uint8_t &oorHigh) {
  if (!mprReadRegister(OOR_STATUS_L, oorLow)) return false;
  if (!mprReadRegister(OOR_STATUS_H, oorHigh)) return false;
  return true;
}

bool setupMPR121() {
  // soft reset
  if (!mprWriteRegister(SOFTRESET, 0x63)) return false;
  delay(5);

  // stop mode before configuration
  if (!mprWriteRegister(ELE_CFG, 0x00)) return false;

  // baseline filter settings
  if (!mprWriteRegister(MHD_R, 0x01)) return false;
  if (!mprWriteRegister(NHD_R, 0x01)) return false;
  if (!mprWriteRegister(NCL_R, 0x00)) return false;
  if (!mprWriteRegister(FDL_R, 0x00)) return false;

  if (!mprWriteRegister(MHD_F, 0x01)) return false;
  if (!mprWriteRegister(NHD_F, 0x01)) return false;
  if (!mprWriteRegister(NCL_F, 0xFF)) return false;
  if (!mprWriteRegister(FDL_F, 0x02)) return false;

  // touched filtering
  if (!mprWriteRegister(NHD_T, 0x00)) return false;
  if (!mprWriteRegister(NCL_T, 0x00)) return false;
  if (!mprWriteRegister(FDL_T, 0x00)) return false;

  // thresholds for all 12 electrodes
  for (uint8_t i = 0; i < 12; i++) {
    if (!mprWriteRegister(ELE0_T + i * 2, TOUCH_THRESHOLD)) return false;
    if (!mprWriteRegister(ELE0_R + i * 2, RELEASE_THRESHOLD)) return false;
  }

  // debounce
  if (!mprWriteRegister(DEBOUNCE_REG, DEBOUNCE_VALUE)) return false;

  // global CDC / CDT / filter settings
  // 0x10 and 0x24 are datasheet defaults and are a stable starting point
  if (!mprWriteRegister(FIL_CFG_1, 0x10)) return false;
  if (!mprWriteRegister(FIL_CFG_2, 0x24)) return false;

  // enter run mode with 12 electrodes enabled and improved initial baseline loading
  // 0x8C = CL=10 + ELE_EN=12 electrodes
  if (!mprWriteRegister(ELE_CFG, 0x8C)) return false;

  delay(300);
  return true;
}

// =====================================================
// Pinnacle low-level RAP communication
// =====================================================
void rapWrite(uint8_t address, uint8_t data) {
  const uint8_t cmdByte = WRITE_MASK | address;

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(cmdByte);
  Wire.write(data);
  Wire.endTransmission(true);
}

bool rapReadBytes(uint8_t address, uint8_t* buffer, uint8_t count) {
  const uint8_t cmdByte = READ_MASK | address;

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(cmdByte);
  if (Wire.endTransmission(true) != 0) {
    return false;
  }

  const uint8_t received = Wire.requestFrom(SLAVE_ADDR, count, (uint8_t)true);
  if (received != count) {
    return false;
  }

  for (uint8_t i = 0; i < count; i++) {
    if (!Wire.available()) {
      return false;
    }
    buffer[i] = Wire.read();
  }

  return true;
}

// =====================================================
// Pinnacle init
// =====================================================
void pinnacleClearFlags() {
  rapWrite(STATUS1_ADDR, 0x00);
}

bool pinnacleEnableFeed(bool enable) {
  uint8_t value = 0;

  if (!rapReadBytes(FEEDCONFIG_1_ADDR, &value, 1)) {
    return false;
  }

  if (enable) {
    value |= 0x01;
  } else {
    value &= ~0x01;
  }

  rapWrite(FEEDCONFIG_1_ADDR, value);
  delay(5);
  return true;
}

bool pinnacleInit() {
  pinMode(DR_PIN, INPUT);

  delay(50);

  pinnacleClearFlags();
  delay(5);

  rapWrite(SYSCONFIG_1_ADDR, SYSCONFIG_1_DATA);
  delay(5);

  rapWrite(FEEDCONFIG_2_ADDR, FEEDCONFIG_2_DATA);
  delay(5);

  rapWrite(FEEDCONFIG_1_ADDR, FEEDCONFIG_1_DATA);
  delay(5);

  rapWrite(Z_IDLE_ADDR, Z_IDLE_COUNT);
  delay(5);

  return pinnacleEnableFeed(true);
}

bool pinnacleGetRelative(RelData& out) {
  uint8_t data[4] = {0, 0, 0, 0};

  if (!rapReadBytes(PACKETBYTE_0_ADDR, data, 4)) {
    return false;
  }

  pinnacleClearFlags();

  out.buttons = data[0] & 0x07;

  int16_t x = data[1] | ((data[0] & 0x10) << 4);
  int16_t y = data[2] | ((data[0] & 0x20) << 3);

  if (x & 0x100) x -= 512;
  if (y & 0x100) y -= 512;

  out.xDelta = x;
  out.yDelta = y;
  out.wheel = static_cast<int8_t>(data[3]);

  return true;
}

// =====================================================
// Mouse movement processing
// =====================================================
float applyAcceleration(int deltaX, int deltaY) {
  const int magnitude = abs(deltaX) + abs(deltaY);

  if (magnitude < 2)  return 1.0f;
  if (magnitude < 5)  return 2.0f;
  if (magnitude < 10) return 3.0f;
  return 4.5f;
}

void collectTouchpadMotion() {
  while (digitalRead(DR_PIN) == HIGH) {
    RelData rel{};

    if (!pinnacleGetRelative(rel)) {
      break;
    }

    accumulatedRawX += rel.xDelta;
    accumulatedRawY += rel.yDelta;
  }
}

void sendMouseMotionIfReady() {
  if (millis() - lastSendMs < SEND_INTERVAL_MS) {
    return;
  }

  lastSendMs = millis();

  int dx = accumulatedRawX;
  int dy = accumulatedRawY;

  accumulatedRawX = 0;
  accumulatedRawY = 0;

  if (dx >= -DEADZONE && dx <= DEADZONE) dx = 0;
  if (dy >= -DEADZONE && dy <= DEADZONE) dy = 0;

  if (dx == 0 && dy == 0) {
    return;
  }

  const float factor = applyAcceleration(dx, dy);

  int moveX = static_cast<int>(dx * factor);
  int moveY = static_cast<int>(dy * factor);

  moveX = clampInt(moveX, -MAX_MOVE_PER_REPORT, MAX_MOVE_PER_REPORT);
  moveY = clampInt(moveY, -MAX_MOVE_PER_REPORT, MAX_MOVE_PER_REPORT);

  if (moveX != 0 || moveY != 0) {
    mouse.move((int8_t)moveX, (int8_t)moveY);
  }
}

// =====================================================
// Keyboard / mouse button handling from MPR121
// =====================================================
void handleTouchPress(uint8_t electrode) {
  switch (electrode) {
    case 0:
      mouse.click(MOUSE_LEFT);
      break;

    case 1:
      mouse.click(MOUSE_RIGHT);
      break;

    case 2:
      keyboard.print(" ");
      break;

    case 3:
      keyboard.write(KEY_RETURN);
      break;

    case 4:
      keyboard.write(KEY_TAB);
      break;

    case 5:
      keyboard.write(KEY_BACKSPACE);
      break;

    case 6:
      keyboard.press(KEY_LEFT_CTRL);
      break;

    case 7:
      keyboard.press(KEY_LEFT_ALT);
      break;

    case 8:
      keyboard.press(KEY_LEFT_SHIFT);
      break;

    case 9:
      keyboard.write(KEY_UP_ARROW);
      break;

    case 10:
      keyboard.write(KEY_DOWN_ARROW);
      break;

    case 11:
      keyboard.press(KEY_LEFT_CTRL);
      keyboard.press(KEY_LEFT_GUI);
      keyboard.write('o');
      keyboard.releaseAll();
      break;
  }
}

void handleTouchRelease(uint8_t electrode) {
  switch (electrode) {
    case 6:
    case 7:
    case 8:
      keyboard.releaseAll();
      break;

    default:
      break;
  }
}

void processMPR121Keys() {
  uint16_t touched = 0;

  if (!mprReadTouchStatus(touched)) {
    return;
  }

  for (uint8_t i = 0; i < 12; i++) {
    bool nowTouched = touched & (1 << i);
    bool beforeTouched = lastTouched & (1 << i);

    if (nowTouched && !beforeTouched) {
      handleTouchPress(i);
    }

    if (!nowTouched && beforeTouched) {
      handleTouchRelease(i);
    }
  }

  lastTouched = touched;
}

// =====================================================
// Debug
// =====================================================
void printElectrodeDebug(uint8_t electrode) {
  uint16_t filtered = mprReadFilteredData(electrode);
  uint16_t baseline = mprReadBaselineData(electrode);
  int16_t diff = (int16_t)baseline - (int16_t)filtered;

  Serial.print("E");
  Serial.print(electrode);
  Serial.print("  filtered=");
  Serial.print(filtered);
  Serial.print("  baseline=");
  Serial.print(baseline);
  Serial.print("  diff=");
  Serial.print(diff);

  if (diff > TOUCH_THRESHOLD) {
    Serial.print("  -> above touch threshold");
  }

  Serial.println();
}

void printTouchStatusDebug() {
  uint16_t touched = 0;
  uint8_t oorL = 0;
  uint8_t oorH = 0;

  if (mprReadTouchStatus(touched)) {
    Serial.print("Touched mask: 0x");
    Serial.println(touched, HEX);
  } else {
    Serial.println("Touched mask: read failed");
  }

  if (mprReadOOR(oorL, oorH)) {
    Serial.print("OOR low: 0x");
    Serial.print(oorL, HEX);
    Serial.print("  OOR high: 0x");
    Serial.println(oorH, HEX);
  } else {
    Serial.println("OOR read failed");
  }

  // print only first 3 electrodes for less spam
  printElectrodeDebug(0);
  printElectrodeDebug(1);
  printElectrodeDebug(2);

  Serial.println("--------------------------------");
}

// =====================================================
// Arduino
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  mouse.begin();
  keyboard.begin();
  USB.begin();

  delay(2000);

  bool mprOk = setupMPR121();
  bool padOk = pinnacleInit();

  Serial.println();
  Serial.println("System startup:");
  Serial.print("MPR121 init: ");
  Serial.println(mprOk ? "OK" : "FAILED");
  Serial.print("Pinnacle init: ");
  Serial.println(padOk ? "OK" : "FAILED");
  Serial.println("IMPORTANT: Connect copper pad BEFORE power-on.");
  Serial.println();
}

void loop() {
  collectTouchpadMotion();
  sendMouseMotionIfReady();
  processMPR121Keys();

  if (millis() - lastDebugMs >= DEBUG_INTERVAL_MS) {
    lastDebugMs = millis();
    printTouchStatusDebug();
  }

  delay(1);
}