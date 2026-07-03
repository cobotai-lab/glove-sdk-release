#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ESPNOW_CHANNEL 1

// Change this if the receiver board LED is wired to another pin.
#define STATUS_LED_PIN 36
#define LED_ACTIVE_HIGH 1

#define HAND_COUNT 2
#define IMU_COUNT 11
#define HAND_TIMEOUT_MS 1000
#define LOW_BATTERY_PERCENT 15

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

static const uint32_t RADIO_MAGIC = 0x48444D49; // "IMDH"

struct PendingFrame {
  bool used;
  SerialFrame frame;
};

static PendingFrame g_queue[8];
static volatile uint8_t g_head = 0;
static volatile uint8_t g_tail = 0;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t g_lastRxMs[HAND_COUNT] = {};
static uint16_t g_failMask[HAND_COUNT] = {0x07FF, 0x07FF};
static uint8_t g_battery[HAND_COUNT] = {0, 0};

static uint8_t XorChecksum(const uint8_t *data, size_t len) {
  uint8_t x = 0;
  for (size_t i = 0; i < len; i++) x ^= data[i];
  return x;
}

static void SetLed(bool on) {
#if LED_ACTIVE_HIGH
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
#else
  digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);
#endif
}

static void PushFrame(const SerialFrame &frame) {
  portENTER_CRITICAL_ISR(&g_mux);
  uint8_t next = (uint8_t)((g_head + 1) % 8);
  if (next != g_tail) {
    g_queue[g_head].frame = frame;
    g_queue[g_head].used = true;
    g_head = next;
  }
  portEXIT_CRITICAL_ISR(&g_mux);
}

static bool PopFrame(SerialFrame &frame) {
  bool ok = false;
  portENTER_CRITICAL(&g_mux);
  if (g_tail != g_head && g_queue[g_tail].used) {
    frame = g_queue[g_tail].frame;
    g_queue[g_tail].used = false;
    g_tail = (uint8_t)((g_tail + 1) % 8);
    ok = true;
  }
  portEXIT_CRITICAL(&g_mux);
  return ok;
}

static void OnDataRecv(const esp_now_recv_info_t *, const uint8_t *data, int len) {
  if (len != (int)sizeof(RadioPacket)) return;

  RadioPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != RADIO_MAGIC) return;
  if (packet.frame.magic[0] != 0xAA || packet.frame.magic[1] != 0x55) return;
  if (packet.frame.imu_count != IMU_COUNT) return;
  if (packet.frame.hand_id >= HAND_COUNT) return;
  if (XorChecksum((const uint8_t *)&packet.frame, sizeof(SerialFrame) - 1) != packet.frame.checksum) return;

  uint8_t hand = packet.frame.hand_id;
  g_lastRxMs[hand] = millis();
  g_failMask[hand] = packet.fail_mask;
  g_battery[hand] = packet.frame.bat_percent;
  PushFrame(packet.frame);
}

static void InitEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) ESP.restart();
  esp_now_register_recv_cb(OnDataRecv);
}

static void UpdateStatusLed() {
  static uint32_t lastToggleMs = 0;
  static bool blink = false;

  uint32_t now = millis();
  bool anyRx = false;
  bool issue = false;

  for (uint8_t hand = 0; hand < HAND_COUNT; hand++) {
    bool fresh = g_lastRxMs[hand] != 0 && (now - g_lastRxMs[hand]) <= HAND_TIMEOUT_MS;
    anyRx = anyRx || fresh;
    if (!fresh || g_failMask[hand] != 0 || g_battery[hand] < LOW_BATTERY_PERCENT) {
      issue = true;
    }
  }

  if (!anyRx) {
    if (now - lastToggleMs >= 120) {
      lastToggleMs = now;
      blink = !blink;
      SetLed(blink);
    }
    return;
  }

  if (issue) {
    if (now - lastToggleMs >= 600) {
      lastToggleMs = now;
      blink = !blink;
      SetLed(blink);
    }
    return;
  }

  SetLed(true);
}

void setup() {
  Serial.begin(2000000);
  pinMode(STATUS_LED_PIN, OUTPUT);
  SetLed(false);
  InitEspNow();
}

void loop() {
  SerialFrame frame;
  while (PopFrame(frame)) {
    Serial.write((const uint8_t *)&frame, sizeof(frame));
  }
  UpdateStatusLed();
  delay(1);
}
