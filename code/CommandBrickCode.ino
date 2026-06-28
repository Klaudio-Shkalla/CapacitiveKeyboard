#include <Wire.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEHIDDevice.h>

// =====================================================
// BLE HID
// =====================================================
NimBLEHIDDevice* hid;
NimBLECharacteristic* keyboardInput;
NimBLECharacteristic* mouseInput;

bool deviceConnected = false;

// Keyboard modifier bits
constexpr uint8_t MOD_CTRL  = 0x01;
constexpr uint8_t MOD_SHIFT = 0x02;
constexpr uint8_t MOD_ALT   = 0x04;
constexpr uint8_t MOD_GUI   = 0x08;

uint8_t currentModifiers = 0;

// HID keycodes
constexpr uint8_t KEY_A_CODE         = 0x04;
constexpr uint8_t KEY_O_CODE         = 0x12;
constexpr uint8_t KEY_SPACE_CODE     = 0x2C;
constexpr uint8_t KEY_ENTER_CODE     = 0x28;
constexpr uint8_t KEY_TAB_CODE       = 0x2B;
constexpr uint8_t KEY_BACKSPACE_CODE = 0x2A;
constexpr uint8_t KEY_UP_CODE        = 0x52;
constexpr uint8_t KEY_DOWN_CODE      = 0x51;

// =====================================================
// Vibration motor
// =====================================================
constexpr uint8_t VIBRATION_PIN = 5;
constexpr unsigned long VIBRATION_DURATION_MS = 40;

bool vibrationActive = false;
unsigned long vibrationStartMs = 0;

// =====================================================
// Shared I2C pins
// =====================================================
constexpr uint8_t SDA_PIN = 8;
constexpr uint8_t SCL_PIN = 9;

// =====================================================
// MPR121 configuration
// =====================================================
constexpr uint8_t MPR121_ADDR = 0x5A;

constexpr uint8_t TOUCHSTATUS_L = 0x00;
constexpr uint8_t TOUCHSTATUS_H = 0x01;
constexpr uint8_t OOR_STATUS_L  = 0x02;
constexpr uint8_t OOR_STATUS_H  = 0x03;

constexpr uint8_t FILTDATA_0L   = 0x04;
constexpr uint8_t FILTDATA_0H   = 0x05;
constexpr uint8_t BASELINE_0    = 0x1E;

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

constexpr uint8_t ELE0_T        = 0x41;
constexpr uint8_t ELE0_R        = 0x42;
constexpr uint8_t DEBOUNCE_REG  = 0x5B;
constexpr uint8_t FIL_CFG_1     = 0x5C;
constexpr uint8_t FIL_CFG_2     = 0x5D;
constexpr uint8_t ELE_CFG       = 0x5E;
constexpr uint8_t SOFTRESET     = 0x80;

constexpr uint8_t TOUCH_THRESHOLD   = 40;
constexpr uint8_t RELEASE_THRESHOLD = 20;
constexpr uint8_t DEBOUNCE_VALUE    = 0x11;

// =====================================================
// Cirque Pinnacle configuration
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
constexpr uint8_t CALCONFIG_1_ADDR  = 0x07;

constexpr uint8_t SYSCONFIG_1_DATA  = 0x00;
constexpr uint8_t FEEDCONFIG_1_DATA = 0x81;
constexpr uint8_t FEEDCONFIG_2_DATA = 0x1F;
constexpr uint8_t Z_IDLE_COUNT      = 0x05;

constexpr uint8_t ERA_VALUE_ADDR    = 0x1B;
constexpr uint8_t ERA_HIGH_ADDR     = 0x1C;
constexpr uint8_t ERA_LOW_ADDR      = 0x1D;
constexpr uint8_t ERA_CONTROL_ADDR  = 0x1E;

constexpr uint8_t ADC_ATTENUATE_1X  = 0x00;
constexpr uint8_t ADC_ATTENUATE_2X  = 0x40;
constexpr uint8_t ADC_ATTENUATE_3X  = 0x80;
constexpr uint8_t ADC_ATTENUATE_4X  = 0xC0;

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

constexpr int DEADZONE = 1;
constexpr int MAX_MOVE_PER_REPORT = 18;

unsigned long lastDebugMs = 0;
constexpr unsigned long DEBUG_INTERVAL_MS = 1000;

// =====================================================
// BLE server callbacks
// =====================================================
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    Serial.println("BLE CONNECTED");
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    currentModifiers = 0;

    Serial.print("BLE DISCONNECTED, reason: ");
    Serial.println(reason);

    NimBLEDevice::startAdvertising();
  }
};

// =====================================================
// Utility
// =====================================================
int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

// =====================================================
// Vibration functions
// =====================================================
void startVibration() {
  digitalWrite(VIBRATION_PIN, LOW);   // ON because red = 3V3, blue = GPIO5
  vibrationActive = true;
  vibrationStartMs = millis();
}

void updateVibration() {
  if (vibrationActive && millis() - vibrationStartMs >= VIBRATION_DURATION_MS) {
    digitalWrite(VIBRATION_PIN, HIGH); // OFF
    vibrationActive = false;
  }
}

// =====================================================
// BLE HID output functions
// =====================================================
void sendKeyboardReport(uint8_t modifiers, uint8_t keycode) {
  if (!deviceConnected) return;

  uint8_t report[8] = {0};
  report[0] = modifiers;
  report[2] = keycode;

  keyboardInput->setValue(report, sizeof(report));
  keyboardInput->notify();
}

void tapKey(uint8_t keycode) {
  sendKeyboardReport(currentModifiers, keycode);
  delay(40);
  sendKeyboardReport(currentModifiers, 0);
}

void updateModifier(uint8_t modifier, bool pressed) {
  if (pressed) currentModifiers |= modifier;
  else currentModifiers &= ~modifier;

  sendKeyboardReport(currentModifiers, 0);
}

void releaseAllKeys() {
  currentModifiers = 0;
  sendKeyboardReport(0, 0);
}

void bleMouseMove(int8_t x, int8_t y, int8_t wheel = 0) {
  if (!deviceConnected) return;

  uint8_t report[4] = {0};
  report[0] = 0x00;
  report[1] = x;
  report[2] = y;
  report[3] = wheel;

  mouseInput->setValue(report, sizeof(report));
  mouseInput->notify();
}

void bleMouseClick(uint8_t buttonMask) {
  if (!deviceConnected) return;

  uint8_t report[4] = {0};

  report[0] = buttonMask;
  mouseInput->setValue(report, sizeof(report));
  mouseInput->notify();

  delay(60);

  report[0] = 0x00;
  mouseInput->setValue(report, sizeof(report));
  mouseInput->notify();
}

void openOnScreenKeyboard() {
  sendKeyboardReport(MOD_CTRL | MOD_GUI, KEY_O_CODE);
  delay(80);
  sendKeyboardReport(0, 0);
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

  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPR121_ADDR, (uint8_t)1) != 1) return false;

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

bool setupMPR121() {
  if (!mprWriteRegister(SOFTRESET, 0x63)) return false;
  delay(5);

  if (!mprWriteRegister(ELE_CFG, 0x00)) return false;

  if (!mprWriteRegister(MHD_R, 0x01)) return false;
  if (!mprWriteRegister(NHD_R, 0x01)) return false;
  if (!mprWriteRegister(NCL_R, 0x00)) return false;
  if (!mprWriteRegister(FDL_R, 0x00)) return false;

  if (!mprWriteRegister(MHD_F, 0x01)) return false;
  if (!mprWriteRegister(NHD_F, 0x01)) return false;
  if (!mprWriteRegister(NCL_F, 0xFF)) return false;
  if (!mprWriteRegister(FDL_F, 0x02)) return false;

  if (!mprWriteRegister(NHD_T, 0x00)) return false;
  if (!mprWriteRegister(NCL_T, 0x00)) return false;
  if (!mprWriteRegister(FDL_T, 0x00)) return false;

  for (uint8_t i = 0; i < 12; i++) {
    if (!mprWriteRegister(ELE0_T + i * 2, TOUCH_THRESHOLD)) return false;
    if (!mprWriteRegister(ELE0_R + i * 2, RELEASE_THRESHOLD)) return false;
  }

  if (!mprWriteRegister(DEBOUNCE_REG, DEBOUNCE_VALUE)) return false;
  if (!mprWriteRegister(FIL_CFG_1, 0x10)) return false;
  if (!mprWriteRegister(FIL_CFG_2, 0x24)) return false;
  if (!mprWriteRegister(ELE_CFG, 0x8C)) return false;

  delay(300);
  return true;
}

// =====================================================
// Pinnacle low-level
// =====================================================
void rapWrite(uint8_t address, uint8_t data) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(WRITE_MASK | address);
  Wire.write(data);
  Wire.endTransmission(true);
}

bool rapReadBytes(uint8_t address, uint8_t* buffer, uint8_t count) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(READ_MASK | address);

  if (Wire.endTransmission(true) != 0) return false;

  uint8_t received = Wire.requestFrom(SLAVE_ADDR, count, (uint8_t)true);
  if (received != count) return false;

  for (uint8_t i = 0; i < count; i++) {
    if (!Wire.available()) return false;
    buffer[i] = Wire.read();
  }

  return true;
}

void pinnacleClearFlags() {
  rapWrite(STATUS1_ADDR, 0x00);
  delayMicroseconds(50);
}

bool pinnacleEnableFeed(bool enable) {
  uint8_t value = 0;

  if (!rapReadBytes(FEEDCONFIG_1_ADDR, &value, 1)) return false;

  if (enable) value |= 0x01;
  else value &= ~0x01;

  rapWrite(FEEDCONFIG_1_ADDR, value);
  delay(5);
  return true;
}

bool eraReadByte(uint16_t address, uint8_t &data) {
  uint8_t control = 0xFF;

  if (!pinnacleEnableFeed(false)) return false;

  rapWrite(ERA_HIGH_ADDR, (uint8_t)(address >> 8));
  rapWrite(ERA_LOW_ADDR,  (uint8_t)(address & 0xFF));
  rapWrite(ERA_CONTROL_ADDR, 0x05);

  do {
    if (!rapReadBytes(ERA_CONTROL_ADDR, &control, 1)) return false;
  } while (control != 0x00);

  if (!rapReadBytes(ERA_VALUE_ADDR, &data, 1)) return false;

  pinnacleClearFlags();
  return true;
}

bool eraWriteByte(uint16_t address, uint8_t data) {
  uint8_t control = 0xFF;

  if (!pinnacleEnableFeed(false)) return false;

  rapWrite(ERA_VALUE_ADDR, data);
  rapWrite(ERA_HIGH_ADDR, (uint8_t)(address >> 8));
  rapWrite(ERA_LOW_ADDR,  (uint8_t)(address & 0xFF));
  rapWrite(ERA_CONTROL_ADDR, 0x02);

  do {
    if (!rapReadBytes(ERA_CONTROL_ADDR, &control, 1)) return false;
  } while (control != 0x00);

  pinnacleClearFlags();
  return true;
}

bool pinnacleSetAdcAttenuation(uint8_t adcGain) {
  uint8_t temp = 0;

  if (!eraReadByte(0x0187, temp)) return false;

  temp &= 0x3F;
  temp |= adcGain;

  return eraWriteByte(0x0187, temp);
}

bool pinnacleTuneEdgeSensitivity() {
  if (!eraWriteByte(0x0149, 0x04)) return false;
  if (!eraWriteByte(0x0168, 0x03)) return false;
  return true;
}

bool pinnacleForceCalibration() {
  uint8_t value = 0;

  if (!pinnacleEnableFeed(false)) return false;

  if (!rapReadBytes(CALCONFIG_1_ADDR, &value, 1)) return false;
  value |= 0x01;
  rapWrite(CALCONFIG_1_ADDR, value);

  do {
    if (!rapReadBytes(CALCONFIG_1_ADDR, &value, 1)) return false;
  } while (value & 0x01);

  pinnacleClearFlags();
  return true;
}

bool pinnacleInit() {
  pinMode(DR_PIN, INPUT);
  delay(200);

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

  bool adcOk = pinnacleSetAdcAttenuation(ADC_ATTENUATE_1X);
  bool calOk = pinnacleForceCalibration();
  bool edgeOk = pinnacleTuneEdgeSensitivity();
  bool feedOk = pinnacleEnableFeed(true);

  Serial.print("Pinnacle ADC tuning: ");
  Serial.println(adcOk ? "OK" : "FAILED");
  Serial.print("Pinnacle force calibration: ");
  Serial.println(calOk ? "OK" : "FAILED");
  Serial.print("Pinnacle edge tuning: ");
  Serial.println(edgeOk ? "OK" : "FAILED");

  return feedOk;
}

bool pinnacleGetRelative(RelData& out) {
  uint8_t data[4] = {0};

  if (!rapReadBytes(PACKETBYTE_0_ADDR, data, 4)) return false;

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
  int magnitude = abs(deltaX) + abs(deltaY);

  if (magnitude < 2)  return 0.5f;
  if (magnitude < 5)  return 0.8f;
  if (magnitude < 10) return 1.2f;
  if (magnitude < 20) return 1.7f;
  return 2.2f;
}

void collectTouchpadMotion() {
  while (digitalRead(DR_PIN) == HIGH) {
    RelData rel{};

    if (!pinnacleGetRelative(rel)) break;

    accumulatedRawX += rel.xDelta;
    accumulatedRawY += rel.yDelta;
  }
}

void sendMouseMotionIfReady() {
  if (!deviceConnected) return;

  if (millis() - lastSendMs < SEND_INTERVAL_MS) return;
  lastSendMs = millis();

  int dx = accumulatedRawX;
  int dy = accumulatedRawY;

  accumulatedRawX = 0;
  accumulatedRawY = 0;

  if (dx >= -DEADZONE && dx <= DEADZONE) dx = 0;
  if (dy >= -DEADZONE && dy <= DEADZONE) dy = 0;

  if (dx == 0 && dy == 0) return;

  float factor = applyAcceleration(dx, dy);

  int moveX = static_cast<int>(dx * factor);
  int moveY = static_cast<int>(dy * factor);

  moveX = clampInt(moveX, -MAX_MOVE_PER_REPORT, MAX_MOVE_PER_REPORT);
  moveY = clampInt(moveY, -MAX_MOVE_PER_REPORT, MAX_MOVE_PER_REPORT);

  bleMouseMove((int8_t)moveX, (int8_t)moveY);
}

// =====================================================
// MPR121 key handling
// =====================================================
void handleTouchPress(uint8_t electrode) {
  if (!deviceConnected) return;

  startVibration();

  switch (electrode) {
    case 0:
      bleMouseClick(0x01);
      break;

    case 1:
      bleMouseClick(0x02);
      break;

    case 2:
      tapKey(KEY_SPACE_CODE);
        
      break;

    case 3:
      tapKey(KEY_ENTER_CODE);
      break;

    case 4:
      tapKey(KEY_TAB_CODE);
      break;

    case 5:
      tapKey(KEY_BACKSPACE_CODE);
      break;

    case 6:
      updateModifier(MOD_CTRL, true);
      break;

    case 7:
      updateModifier(MOD_ALT, true);
      break;

    case 8:
      updateModifier(MOD_SHIFT, true);
      break;

    case 9:
      tapKey(KEY_UP_CODE);
      break;

    case 10:
      tapKey(KEY_DOWN_CODE);
      break;

    case 11:
      openOnScreenKeyboard();
      break;
  }
}

void handleTouchRelease(uint8_t electrode) {
  switch (electrode) {
    case 6:
      updateModifier(MOD_CTRL, false);
      break;

    case 7:
      updateModifier(MOD_ALT, false);
      break;

    case 8:
      updateModifier(MOD_SHIFT, false);
      break;

    default:
      break;
  }
}

void processMPR121Keys() {
  uint16_t touched = 0;

  if (!mprReadTouchStatus(touched)) return;

  for (uint8_t i = 0; i < 12; i++) {
    bool nowTouched = touched & (1 << i);
    bool beforeTouched = lastTouched & (1 << i);

    if (nowTouched && !beforeTouched) handleTouchPress(i);
    if (!nowTouched && beforeTouched) handleTouchRelease(i);
  }

  lastTouched = touched;
}

// =====================================================
// Debug
// =====================================================
void printTouchStatusDebug() {
  uint16_t touched = 0;

  if (mprReadTouchStatus(touched)) {
    Serial.print("Touched mask: 0x");
    Serial.println(touched, HEX);
  }

  Serial.print("BLE: ");
  Serial.println(deviceConnected ? "connected" : "not connected");
  Serial.println("--------------------------------");
}

// =====================================================
// BLE HID setup
// =====================================================
void setupBLEHID() {
  Serial.println("Starting BLE HID combo");

  NimBLEDevice::init("KLAUDIO_ASSISTIVE_BLE_001");

  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());

  hid = new NimBLEHIDDevice(server);

  keyboardInput = hid->getInputReport(1);
  mouseInput = hid->getInputReport(2);

  hid->setManufacturer("ESP32");
  hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->setHidInfo(0x00, 0x01);
  hid->setBatteryLevel(100);

  const uint8_t reportMap[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x01,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00,
    0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01,
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
    0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
    0xC0,

    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x02,
    0x09, 0x01, 0xA1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03,
    0x75, 0x01, 0x81, 0x02,
    0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08,
    0x95, 0x03, 0x81, 0x06,
    0xC0, 0xC0
  };

  hid->setReportMap((uint8_t*)reportMap, sizeof(reportMap));
  hid->startServices();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(hid->getHidService()->getUUID());
  advertising->setAppearance(0x03C0);
  advertising->start();

  Serial.println("BLE HID combo ready");
}

// =====================================================
// Arduino
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, HIGH); // motor OFF

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  bool mprOk = setupMPR121();
  bool padOk = pinnacleInit();

  Serial.println();
  Serial.println("System startup:");
  Serial.print("MPR121 init: ");
  Serial.println(mprOk ? "OK" : "FAILED");
  Serial.print("Pinnacle init: ");
  Serial.println(padOk ? "OK" : "FAILED");

  delay(1000);

  setupBLEHID();

  Serial.println("IMPORTANT: Do not touch the touchpad during startup/calibration.");
}

void loop() {
  if (!deviceConnected) {
    Serial.println("Waiting for BLE connection...");
    delay(1000);
    return;
  }

  collectTouchpadMotion();
  sendMouseMotionIfReady();
  processMPR121Keys();
  updateVibration();

  if (millis() - lastDebugMs >= DEBUG_INTERVAL_MS) {
    lastDebugMs = millis();
    printTouchStatusDebug();
  }

  delay(1);
}