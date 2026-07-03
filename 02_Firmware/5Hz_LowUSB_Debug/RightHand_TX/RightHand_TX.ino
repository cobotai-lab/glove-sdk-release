#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define HAND_ID 1
#define HAND_NAME "right"

#define USB_TEXT_ENABLED 1
#define USB_TEXT_BAUD 921600
#define USB_TEXT_HZ 5
#define USB_TEXT_INTERVAL_MS (1000UL / USB_TEXT_HZ)

#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ 400000

#define IMU_POWER_CTRL 1
#define IR_POWER_CTRL 42
#define POWER_CTRL 41
#define PMOS_ON_LEVEL LOW

#define BAT_ADC_PIN 2
#define VBAT_DIVIDER_RATIO 1.454545f
#define LOW_BATTERY_VOLTAGE 3.50f
#define LOW_BATTERY_CHECK_MS 1000UL
#define LOW_BATTERY_HOLD_MS 8000UL
#define LOW_BATTERY_SAMPLES 8
#define POWER_OFF_HOLD_MS 3000UL
#define BATTERY_TREND_CHECK_MS 60000UL
#define BATTERY_DROP_MARGIN_V 0.01f
#define WIRELESS_ALIVE_TIMEOUT_MS 30000UL
#define WIRELESS_NO_RESPONSE_POWER_OFF_MS 900000UL

#define ESPNOW_CHANNEL 1
#define SEND_HZ 30
#define SEND_INTERVAL_US (1000000UL / SEND_HZ)

#define REG_GX 0x37
#define REG_ROLL 0x3D
#define REG_Q0 0x51
#define REG_SAVE 0x00
#define REG_CALSW 0x01
#define REG_HX 0x3A
#define REG_KEY 0x69

static const uint8_t IMU_ADDR_LIST[] = {
  0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60
};
#define IMU_COUNT 11

struct __attribute__((packed)) SerialFrame {
  uint8_t magic[2];
  uint8_t hand_id;
  uint8_t imu_count;
  uint8_t bat_percent;
  uint16_t frame_id;
  int16_t imu[IMU_COUNT][7];   // euler[3], quat[4]
  int16_t wrist_gyro[3];
  uint8_t checksum;
};

struct __attribute__((packed)) RadioPacket {
  uint32_t magic;
  uint16_t fail_mask;
  SerialFrame frame;
};

static_assert(sizeof(SerialFrame) == 168, "SerialFrame must be 168 bytes");
static_assert(sizeof(RadioPacket) <= 250, "ESP-NOW packet too large");

static const uint8_t RECEIVER_ADDR[] = {0x14, 0xC1, 0x9F, 0x2E, 0x79, 0xC0};
static const uint32_t RADIO_MAGIC = 0x48444D49; // "IMDH"

static RadioPacket g_packet;
static uint32_t g_nextSendUs = 0;
static uint32_t g_lastTextPrintMs = 0;
static uint16_t g_frameId = 0;

enum RunMode {
  MODE_NORMAL,
  MODE_MAG_CALIBRATING
};

static RunMode g_mode = MODE_NORMAL;
static uint8_t g_activeImus[IMU_COUNT];
static uint8_t g_activeCount = 0;
static uint32_t g_lastMagPrintMs = 0;
static char g_cmdBuf[80];
static uint8_t g_cmdLen = 0;
static uint32_t g_lastBatteryCheckMs = 0;
static uint32_t g_lowBatterySinceMs = 0;
static bool g_poweringOff = false;
static uint32_t g_lastBatteryTrendCheckMs = 0;
static float g_prevTrendVbat = 0.0f;
static bool g_batteryDecreasing = false;
static volatile uint32_t g_lastWirelessOkMs = 0;
static uint32_t g_lastWirelessCheckMs = 0;
static uint32_t g_wirelessNoResponseSinceMs = 0;

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

static bool WriteReg(uint8_t addr7, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  Wire.write((uint8_t)(val & 0xFF));
  Wire.write((uint8_t)(val >> 8));
  return Wire.endTransmission() == 0;
}

static bool ProbeImu(uint8_t addr7) {
  Wire.beginTransmission(addr7);
  return Wire.endTransmission() == 0;
}

static void UnlockActiveImus() {
  for (uint8_t i = 0; i < g_activeCount; i++) {
    WriteReg(g_activeImus[i], REG_KEY, 0xB588);
  }
  delay(100);
}

static void SetCalswActiveImus(uint16_t mode) {
  UnlockActiveImus();
  for (uint8_t i = 0; i < g_activeCount; i++) {
    bool ok = WriteReg(g_activeImus[i], REG_CALSW, mode);
    Serial.printf("$GLOVITY,CAL,CALSW,0x%02X,0x%02X,%s\r\n",
                  g_activeImus[i],
                  mode,
                  ok ? "OK" : "FAIL");
  }
  delay(100);
}

static void SaveActiveImus() {
  UnlockActiveImus();
  for (uint8_t i = 0; i < g_activeCount; i++) {
    bool ok = WriteReg(g_activeImus[i], REG_SAVE, 0x0000);
    Serial.printf("$GLOVITY,CAL,SAVE,0x%02X,%s\r\n",
                  g_activeImus[i],
                  ok ? "OK" : "FAIL");
  }
  delay(500);
}

static uint8_t ScanActiveImus() {
  g_activeCount = 0;
  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    uint8_t addr = IMU_ADDR_LIST[i];
    bool ok = ProbeImu(addr);
    Serial.printf("$GLOVITY,CAL,SCAN,0x%02X,%s\r\n", addr, ok ? "FOUND" : "MISS");
    if (ok) {
      g_activeImus[g_activeCount++] = addr;
    }
  }
  Serial.printf("$GLOVITY,CAL,SCAN_DONE,%u,%u\r\n", g_activeCount, IMU_COUNT);
  return g_activeCount;
}

static float ReadBatteryVoltage(uint8_t samples) {
  uint32_t totalMv = 0;
  if (samples == 0) samples = 1;
  for (uint8_t i = 0; i < samples; i++) {
    totalMv += analogReadMilliVolts(BAT_ADC_PIN);
    if (samples > 1) delay(2);
  }
  float mv = (float)totalMv / samples;
  return (mv / 1000.0f) * VBAT_DIVIDER_RATIO;
}

static uint8_t BatteryPercent() {
  float vbat = ReadBatteryVoltage(1);
  int pct = (int)lroundf((vbat - 3.30f) * 100.0f / (4.20f - 3.30f));
  return (uint8_t)constrain(pct, 0, 100);
}

static void AutoPowerOff(float vbat) {
  g_poweringOff = true;
  Serial.printf("battery low, auto power off: %.3fV\r\n", vbat);
  Serial.flush();

  esp_now_deinit();
  esp_wifi_stop();
  WiFi.mode(WIFI_OFF);

  digitalWrite(IR_POWER_CTRL, !PMOS_ON_LEVEL);
  digitalWrite(IMU_POWER_CTRL, !PMOS_ON_LEVEL);
  digitalWrite(POWER_CTRL, HIGH);
  delay(POWER_OFF_HOLD_MS);

  while (true) {
    digitalWrite(POWER_CTRL, HIGH);
    delay(1000);
  }
}

static void CheckLowBatteryPowerOff() {
  if (g_poweringOff) return;
  uint32_t nowMs = millis();
  if (nowMs - g_lastBatteryCheckMs < LOW_BATTERY_CHECK_MS) return;
  g_lastBatteryCheckMs = nowMs;

  float vbat = ReadBatteryVoltage(LOW_BATTERY_SAMPLES);
  if (vbat < LOW_BATTERY_VOLTAGE) {
    if (g_lowBatterySinceMs == 0) g_lowBatterySinceMs = nowMs;
    if (nowMs - g_lowBatterySinceMs >= LOW_BATTERY_HOLD_MS) {
      AutoPowerOff(vbat);
    }
  } else {
    g_lowBatterySinceMs = 0;
  }
}

static void UpdateBatteryTrend() {
  uint32_t nowMs = millis();
  if (g_prevTrendVbat > 0.0f && nowMs - g_lastBatteryTrendCheckMs < BATTERY_TREND_CHECK_MS) return;
  float vbat = ReadBatteryVoltage(LOW_BATTERY_SAMPLES);
  if (g_prevTrendVbat <= 0.0f) {
    g_prevTrendVbat = vbat;
    g_lastBatteryTrendCheckMs = nowMs;
    return;
  }
  g_lastBatteryTrendCheckMs = nowMs;
  g_batteryDecreasing = (vbat + BATTERY_DROP_MARGIN_V) < g_prevTrendVbat;
  g_prevTrendVbat = vbat;
}

static void CheckWirelessNoResponsePowerOff() {
  if (g_poweringOff) return;
  uint32_t nowMs = millis();
  if (nowMs - g_lastWirelessCheckMs < LOW_BATTERY_CHECK_MS) return;
  g_lastWirelessCheckMs = nowMs;
  UpdateBatteryTrend();

  uint32_t lastOk = g_lastWirelessOkMs;
  bool wirelessAlive = lastOk != 0 && (nowMs - lastOk) < WIRELESS_ALIVE_TIMEOUT_MS;
  if (!g_batteryDecreasing || wirelessAlive) {
    g_wirelessNoResponseSinceMs = 0;
    return;
  }

  if (g_wirelessNoResponseSinceMs == 0) g_wirelessNoResponseSinceMs = nowMs;
  if (nowMs - g_wirelessNoResponseSinceMs >= WIRELESS_NO_RESPONSE_POWER_OFF_MS) {
    float vbat = ReadBatteryVoltage(LOW_BATTERY_SAMPLES);
    Serial.printf("wireless no response while battery decreasing, auto power off: %.3fV\r\n", vbat);
    AutoPowerOff(vbat);
  }
}

static void OnEspNowSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  (void)txInfo;
  if (status == ESP_NOW_SEND_SUCCESS) {
    g_lastWirelessOkMs = millis();
  }
}

static uint8_t XorChecksum(const uint8_t *data, size_t len) {
  uint8_t x = 0;
  for (size_t i = 0; i < len; i++) x ^= data[i];
  return x;
}

static float EulerRawToDeg(int16_t raw) {
  return raw / 32768.0f * 180.0f;
}

static float QuatRawToFloat(int16_t raw) {
  return raw / 32768.0f;
}

static bool ReadOneImu(uint8_t addr, int16_t out7[7], int16_t gyro3[3]) {
  int16_t euler[3] = {};
  int16_t quat[4] = {};
  bool okEuler = ReadRegs(addr, REG_ROLL, euler, 3);
  bool okQuat = ReadRegs(addr, REG_Q0, quat, 4);
  if (gyro3) {
    int16_t gyro[3] = {};
    if (ReadRegs(addr, REG_GX, gyro, 3)) {
      gyro3[0] = gyro[0];
      gyro3[1] = gyro[1];
      gyro3[2] = gyro[2];
    }
  }
  if (!okEuler || !okQuat) return false;
  out7[0] = euler[0];
  out7[1] = euler[1];
  out7[2] = euler[2];
  out7[3] = quat[0];
  out7[4] = quat[1];
  out7[5] = quat[2];
  out7[6] = quat[3];
  return true;
}

static void BuildFrame() {
  SerialFrame &f = g_packet.frame;
  memset(&g_packet, 0, sizeof(g_packet));
  g_packet.magic = RADIO_MAGIC;
  f.magic[0] = 0xAA;
  f.magic[1] = 0x55;
  f.hand_id = HAND_ID;
  f.imu_count = IMU_COUNT;
  f.bat_percent = BatteryPercent();
  f.frame_id = g_frameId++;

  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    int16_t *gyro = (i == 0) ? f.wrist_gyro : nullptr;
    if (!ReadOneImu(IMU_ADDR_LIST[i], f.imu[i], gyro)) {
      g_packet.fail_mask |= (1U << i);
      memset(f.imu[i], 0, sizeof(f.imu[i]));
    }
  }
  f.checksum = XorChecksum((const uint8_t *)&f, sizeof(SerialFrame) - 1);
}

static void InitEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) ESP.restart();
  esp_now_register_send_cb(OnEspNowSent);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RECEIVER_ADDR, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

static void PrintUsbTextFrame(const RadioPacket &packet) {
#if USB_TEXT_ENABLED
  const SerialFrame &f = packet.frame;
  Serial.println("==== IMU Direct Reader");
  Serial.printf("Frame: %u  Bat: %u%%  Hand: %s(%u)  ESP-NOW: on  fail_mask: 0x%03X\r\n",
                f.frame_id,
                f.bat_percent,
                HAND_NAME,
                HAND_ID,
                packet.fail_mask);

  for (uint8_t i = 0; i < IMU_COUNT; i++) {
    uint8_t addr = IMU_ADDR_LIST[i];
    if (packet.fail_mask & (1U << i)) {
      Serial.printf("[0x%02X] Read failed\r\n", addr);
      continue;
    }

    const int16_t *v = f.imu[i];
    Serial.printf("[0x%02X] Roll:%9.2f Pitch:%9.2f Yaw:%9.2f Q:%9.4f %9.4f %9.4f %9.4f\r\n",
                  addr,
                  EulerRawToDeg(v[0]),
                  EulerRawToDeg(v[1]),
                  EulerRawToDeg(v[2]),
                  QuatRawToFloat(v[3]),
                  QuatRawToFloat(v[4]),
                  QuatRawToFloat(v[5]),
                  QuatRawToFloat(v[6]));
  }
#endif
}

static void StartMagCalibration() {
  if (g_mode == MODE_MAG_CALIBRATING) {
    Serial.println("$GLOVITY,CAL,ALREADY_RUNNING");
    return;
  }

  Serial.println("$GLOVITY,CAL,STARTING");
  if (ScanActiveImus() == 0) {
    Serial.println("$GLOVITY,CAL,ERROR,NO_IMU");
    g_mode = MODE_NORMAL;
    g_nextSendUs = micros();
    return;
  }

  SetCalswActiveImus(0x0007);
  g_mode = MODE_MAG_CALIBRATING;
  g_lastMagPrintMs = 0;
  Serial.println("$GLOVITY,CAL,STARTED");
}

static void StopMagCalibration() {
  if (g_mode != MODE_MAG_CALIBRATING) {
    Serial.println("$GLOVITY,CAL,ERROR,NOT_RUNNING");
    return;
  }

  Serial.println("$GLOVITY,CAL,STOPPING");
  SetCalswActiveImus(0x0000);
  SaveActiveImus();
  g_mode = MODE_NORMAL;
  g_nextSendUs = micros();
  g_lastTextPrintMs = millis();
  Serial.println("$GLOVITY,CAL,COMPLETE");
  Serial.println("$GLOVITY,MODE,NORMAL");
}

static void PrintMagCalibrationFrame() {
  Serial.printf("$GLOVITY,CAL,MAG,%lu", (unsigned long)(millis() / 1000));
  for (uint8_t i = 0; i < g_activeCount; i++) {
    int16_t mag[3] = {};
    if (ReadRegs(g_activeImus[i], REG_HX, mag, 3)) {
      Serial.printf(",0x%02X:%d:%d:%d", g_activeImus[i], mag[0], mag[1], mag[2]);
    } else {
      Serial.printf(",0x%02X:ERR", g_activeImus[i]);
    }
  }
  Serial.println();
}

static void LoopMagCalibration() {
  uint32_t nowMs = millis();
  if (nowMs - g_lastMagPrintMs >= 500) {
    g_lastMagPrintMs = nowMs;
    PrintMagCalibrationFrame();
  }
}

static void ProcessCommand(char *cmd) {
  while (*cmd == ' ' || *cmd == '\t') cmd++;
  for (char *p = cmd; *p; p++) {
    if (*p >= 'a' && *p <= 'z') *p = (char)(*p - 'a' + 'A');
  }

  if (strcmp(cmd, "$GLOVITY,CAL,START") == 0 || strcmp(cmd, "CAL,START") == 0) {
    StartMagCalibration();
  } else if (strcmp(cmd, "$GLOVITY,CAL,STOP") == 0 || strcmp(cmd, "CAL,STOP") == 0) {
    StopMagCalibration();
  } else if (strcmp(cmd, "$GLOVITY,CAL,STATUS") == 0 || strcmp(cmd, "CAL,STATUS") == 0) {
    Serial.printf("$GLOVITY,CAL,STATUS,%s,%u,%u\r\n",
                  g_mode == MODE_MAG_CALIBRATING ? "RUNNING" : "IDLE",
                  g_activeCount,
                  IMU_COUNT);
  } else if (strcmp(cmd, "$GLOVITY,NORMAL") == 0 || strcmp(cmd, "NORMAL") == 0) {
    if (g_mode == MODE_MAG_CALIBRATING) {
      StopMagCalibration();
    } else {
      Serial.println("$GLOVITY,MODE,NORMAL");
    }
  } else if (cmd[0] != '\0') {
    Serial.printf("$GLOVITY,CMD,UNKNOWN,%s\r\n", cmd);
  }
}

static void HandleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (g_cmdLen > 0) {
        g_cmdBuf[g_cmdLen] = '\0';
        ProcessCommand(g_cmdBuf);
        g_cmdLen = 0;
      }
    } else if (g_cmdLen < sizeof(g_cmdBuf) - 1) {
      g_cmdBuf[g_cmdLen++] = c;
    } else {
      g_cmdLen = 0;
      Serial.println("$GLOVITY,CMD,ERROR,TOO_LONG");
    }
  }
}

static void LoopNormal() {
  uint32_t now = micros();
  if ((int32_t)(now - g_nextSendUs) < 0) return;
  g_nextSendUs += SEND_INTERVAL_US;
  BuildFrame();
  esp_now_send(RECEIVER_ADDR, (const uint8_t *)&g_packet, sizeof(g_packet));

  uint32_t nowMs = millis();
  if (nowMs - g_lastTextPrintMs >= USB_TEXT_INTERVAL_MS) {
    g_lastTextPrintMs = nowMs;
    PrintUsbTextFrame(g_packet);
  }
}

void setup() {
  Serial.begin(USB_TEXT_BAUD);
  digitalWrite(POWER_CTRL, LOW);
  pinMode(POWER_CTRL, OUTPUT);
  digitalWrite(IMU_POWER_CTRL, PMOS_ON_LEVEL);
  pinMode(IMU_POWER_CTRL, OUTPUT);
  digitalWrite(IR_POWER_CTRL, PMOS_ON_LEVEL);
  pinMode(IR_POWER_CTRL, OUTPUT);
  delay(1200);
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  analogReadResolution(12);
  InitEspNow();
  g_nextSendUs = micros();
#if USB_TEXT_ENABLED
  Serial.println();
  Serial.println("=== DualHand IMU Glove");
  Serial.printf("Mode: ESP-NOW Binary + USB Text  Hand: %s(%u)  USB baud: %lu\r\n",
                HAND_NAME,
                HAND_ID,
                (unsigned long)USB_TEXT_BAUD);
  Serial.printf("ESP-NOW: %u Hz  USB Text: %u Hz  IMU: 0x50~0x60\r\n",
                SEND_HZ,
                USB_TEXT_HZ);
  Serial.println("Commands: $GLOVITY,CAL,START | $GLOVITY,CAL,STOP | $GLOVITY,CAL,STATUS");
#endif
}

void loop() {
  CheckLowBatteryPowerOff();
  CheckWirelessNoResponsePowerOff();
  HandleSerialCommands();
  if (g_mode == MODE_MAG_CALIBRATING) {
    LoopMagCalibration();
  } else {
    LoopNormal();
  }
}
