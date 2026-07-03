#include <Arduino.h>
#include <Wire.h>

#define HAND_ID 0
#define SERIAL_BAUD 921600

#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ 400000

#define IMU_POWER_CTRL 1
#define IR_POWER_CTRL 42
#define PMOS_ON_LEVEL LOW

#define BAT_ADC_PIN 2
#define VBAT_DIVIDER_RATIO 1.454545f

#define PRINT_HZ 30
#define PRINT_INTERVAL_US (1000000UL / PRINT_HZ)

#define REG_ROLL 0x3D
#define REG_Q0 0x51

static const uint8_t IMU_ADDR_LIST[] = {
  0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60
};
#define IMU_COUNT 11

static uint32_t g_nextPrintUs = 0;
static uint32_t g_frameId = 0;

static bool ReadRegs(uint8_t addr7, uint8_t startReg, int16_t *buf, uint8_t count) {
  Wire.beginTransmission(addr7);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t need = count * 2;
  uint8_t got = Wire.requestFrom(addr7, need);
  if (got < need) return false;
  for (uint8_t i = 0; i < count; i++) {
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    buf[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

static uint8_t BatteryPercent() {
  uint32_t mv = analogReadMilliVolts(BAT_ADC_PIN);
  float vbat = (mv / 1000.0f) * VBAT_DIVIDER_RATIO;
  int pct = (int)lroundf((vbat - 3.30f) * 100.0f / (4.20f - 3.30f));
  return (uint8_t)constrain(pct, 0, 100);
}

static bool ReadPose(uint8_t addr, float euler[3], float quat[4]) {
  int16_t eulerRaw[3] = {};
  int16_t quatRaw[4] = {};
  bool okEuler = ReadRegs(addr, REG_ROLL, eulerRaw, 3);
  bool okQuat = ReadRegs(addr, REG_Q0, quatRaw, 4);
  if (!okEuler || !okQuat) return false;
  for (uint8_t i = 0; i < 3; i++) euler[i] = eulerRaw[i] / 32768.0f * 180.0f;
  for (uint8_t i = 0; i < 4; i++) quat[i] = quatRaw[i] / 32768.0f;
  return true;
}

static void PrintI2cScan() {
  Serial.println("I2C scan:");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found: 0x%02X\r\n", addr);
      found++;
    }
  }
  Serial.printf("Total I2C devices found: %u\r\n", found);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1200);

  digitalWrite(IMU_POWER_CTRL, PMOS_ON_LEVEL);
  pinMode(IMU_POWER_CTRL, OUTPUT);
  digitalWrite(IR_POWER_CTRL, PMOS_ON_LEVEL);
  pinMode(IR_POWER_CTRL, OUTPUT);
  delay(1200);

  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  analogReadResolution(12);

  Serial.println();
  Serial.println("=== IMU Direct Reader");
  Serial.printf("Mode: USB Text  Hand: left(%u)  Baud: %lu\r\n", HAND_ID, (unsigned long)SERIAL_BAUD);
  Serial.printf("IMU count: %u  Addr: 0x50~0x60  Target: %u Hz\r\n", IMU_COUNT, PRINT_HZ);
  PrintI2cScan();
  g_nextPrintUs = micros();
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - g_nextPrintUs) < 0) return;
  g_nextPrintUs += PRINT_INTERVAL_US;

  uint8_t bat = BatteryPercent();
  Serial.println("==== IMU Direct Reader");
  Serial.printf("Frame: %lu  Bat: %u%%\r\n", (unsigned long)g_frameId++, bat);

  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    uint8_t addr = IMU_ADDR_LIST[i];
    float euler[3] = {};
    float quat[4] = {};
    bool ok = ReadPose(addr, euler, quat);
    if (ok) {
      Serial.printf("[0x%02X] Roll:%9.2f Pitch:%9.2f Yaw:%9.2f Q:%9.4f %9.4f %9.4f %9.4f\r\n",
                    addr, euler[0], euler[1], euler[2], quat[0], quat[1], quat[2], quat[3]);
    } else {
      Serial.printf("[0x%02X] Read failed\r\n", addr);
    }
  }
}
