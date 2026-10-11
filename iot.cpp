/*
  =====================================================================
   iot.cpp  —  Wi-Fi + MQTT (HiveMQ Cloud, TLS พอร์ต 8883) สำหรับ Smart Breaker
  =====================================================================
   หลักการ
    - ทำงานใน FreeRTOS task แยก (คนละแกนกับ loop() ของ main)
      ต่อ Wi-Fi/TLS ช้าหรือค้างกี่วินาที ระบบป้องกันและปุ่มบนบอร์ดก็ไม่สะดุด
    - คุยกับ main.cpp ผ่านฟังก์ชันใน breaker_api.h เท่านั้น
    - ระบบป้องกันทำงานบนบอร์ดเสมอ ต่อให้ Wi-Fi/MQTT หลุดก็ยังตัดไฟได้เอง

   TOPIC (ทั้งหมดขึ้นต้นด้วย TOPIC_PREFIX ใน secrets.h)
     <prefix>/telemetry     บอร์ด -> มือถือ  JSON ค่าที่วัด+สถานะ (retained, ทุก 2 วินาที + ทันทีที่สถานะเปลี่ยน)
     <prefix>/status        บอร์ด -> มือถือ  "online" / "offline" (retained, offline ส่งโดย broker เมื่อบอร์ดหลุด)
     <prefix>/ack           บอร์ด -> มือถือ  ผลลัพธ์ของคำสั่งล่าสุด JSON
     <prefix>/cmd           มือถือ -> บอร์ด  คำสั่ง JSON (ดูด้านล่าง)
     <prefix>/cmd/relay     มือถือ -> บอร์ด  ข้อความธรรมดา on / off (สำหรับแอปที่ส่งข้อความง่ายๆ)

   คำสั่ง JSON ที่ <prefix>/cmd   ("id" ใส่หรือไม่ใส่ก็ได้ จะถูกส่งกลับใน ack)
     {"cmd":"relay","value":"on"}            หรือ "off" / true / false
     {"cmd":"set_overload","value":5.0}      กระแสตัดไฟ (A)
     {"cmd":"set_voltage","min":200,"max":240}
     {"cmd":"set_timer","value":60}          นาที
     {"cmd":"set_mode","value":"timer"}      "timer" หรือ "cont"
     {"cmd":"set_price","value":5.0}         บาท/หน่วย
     {"cmd":"reset_fault"}
     {"cmd":"reset_energy"}
     {"cmd":"get_status"}                    ขอ telemetry ทันที
  =====================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "breaker_api.h"
#include "secrets.h"

#ifndef IOT_ENABLED
#define IOT_ENABLED 1
#endif
#ifndef IOT_DEBUG
#define IOT_DEBUG 1          // 1 = พิมพ์สถานะลง Serial Monitor (115200) ตอนทดสอบ, 0 = ปิด
#endif

#if IOT_ENABLED

#if IOT_DEBUG
#define LOG(...) Serial.printf(__VA_ARGS__)
#else
#define LOG(...) do {} while (0)
#endif

// ===================== ค่าที่ปรับได้ =====================
static const unsigned long WIFI_RETRY_MS      = 15000;   // ต่อ Wi-Fi ไม่ติดภายในกี่ ms ให้เริ่มใหม่
static const unsigned long MQTT_RETRY_MIN_MS  = 5000;    // ต่อ MQTT ใหม่หลังพลาด (เริ่มที่ 5 วิ)
static const unsigned long MQTT_RETRY_MAX_MS  = 60000;   // ... เพิ่มเท่าตัวทุกครั้งจนถึง 60 วิ
static const unsigned long TELEMETRY_MS       = 2000;    // ส่งค่าทุกกี่ ms
static const unsigned long STATE_POLL_MS      = 200;     // เช็กว่าสถานะเปลี่ยนไหมทุกกี่ ms
static const uint16_t      MQTT_BUFFER_BYTES  = 768;
static const uint16_t      MQTT_KEEPALIVE_S   = 30;

// ===================== ตัวแปรภายใน =====================
static WiFiClientSecure netClient;
static PubSubClient     mqtt(netClient);

static char clientId[24];
static char tTelemetry[96], tStatus[96], tAck[96], tCmd[96], tCmdAll[96], tCmdRelay[96];

static bool          wifiStarted   = false;
static bool          wifiWasUp     = false;
static unsigned long lastWifiTry   = 0;
static unsigned long lastMqttTry   = 0;
static unsigned long mqttBackoff   = MQTT_RETRY_MIN_MS;
static unsigned long lastTelemetry = 0;
static unsigned long lastStateChk  = 0;
static bool          forceTelemetry = true;

// =====================================================================
//                           ฟังก์ชันช่วย
// =====================================================================
static const char* tf(bool b) { return b ? "true" : "false"; }

static const char* errText(uint8_t e) {
  switch (e) {
    case 1:  return "overload";
    case 2:  return "under_voltage";
    case 3:  return "over_voltage";
    case 4:  return "sensor";
    default: return "none";
  }
}

static const char* cmdName(BreakerCmdType t) {
  switch (t) {
    case BCMD_RELAY_ON:     return "relay_on";
    case BCMD_RELAY_OFF:    return "relay_off";
    case BCMD_SET_OVERLOAD: return "set_overload";
    case BCMD_SET_VOLTAGE:  return "set_voltage";
    case BCMD_SET_TIMER:    return "set_timer";
    case BCMD_SET_MODE:     return "set_mode";
    case BCMD_SET_PRICE:    return "set_price";
    case BCMD_RESET_FAULT:  return "reset_fault";
    case BCMD_RESET_ENERGY: return "reset_energy";
  }
  return "unknown";
}

// ---------- ส่ง telemetry (JSON) ----------
static void publishTelemetry(const BreakerStatus &s) {
  char buf[640];
  int n = snprintf(buf, sizeof(buf),
    "{\"v\":%.1f,\"i\":%.3f,\"p\":%.1f,\"kwh\":%.3f,\"pf\":%.2f,\"hz\":%.1f,"
    "\"relay\":%s,\"mode\":\"%s\",\"timer_left\":%d,\"recovery_left\":%d,"
    "\"error\":\"%s\",\"fault\":%s,\"overload_pending\":%s,\"recovery_locked\":%s,"
    "\"sensor_ok\":%s,\"remote_allowed\":%s,"
    "\"set\":{\"overload_a\":%.1f,\"v_min\":%.0f,\"v_max\":%.0f,\"timer_min\":%d,\"price\":%.2f},"
    "\"rssi\":%d,\"uptime\":%lu}",
    s.v, s.i, s.p, s.kwh, s.pf, s.hz,
    tf(s.relayOn), s.mode == 1 ? "timer" : "cont", s.timerSecLeft, s.recoverySecLeft,
    errText(s.error), tf(s.faultActive), tf(s.overloadPending), tf(s.recoveryLocked),
    tf(s.sensorOk), tf(s.remoteAllowed),
    s.overloadLimitA, s.vMin, s.vMax, s.timerMinutes, s.unitPrice,
    (int)WiFi.RSSI(), (unsigned long)(millis() / 1000));
  if (n > 0 && n < (int)sizeof(buf)) {
    mqtt.publish(tTelemetry, (const uint8_t*)buf, (unsigned int)n, true);   // retained
  }
}

// ---------- ส่งผลลัพธ์คำสั่ง (ack) ----------
static void publishAck(const char* cmd, bool ok, const char* result, uint32_t tag) {
  if (!mqtt.connected()) return;
  char buf[160];
  int n = snprintf(buf, sizeof(buf),
    "{\"cmd\":\"%s\",\"ok\":%s,\"result\":\"%s\",\"id\":%lu}",
    cmd, tf(ok), result, (unsigned long)tag);
  if (n > 0 && n < (int)sizeof(buf)) {
    mqtt.publish(tAck, (const uint8_t*)buf, (unsigned int)n, false);
  }
}

// ---------- ส่งคำสั่งให้บอร์ด (บอร์ดจะตรวจเงื่อนไขป้องกันเอง) ----------
static void submit(BreakerCmdType t, float a, float b, uint32_t tag) {
  BreakerCommand c;
  c.type = t; c.a = a; c.b = b; c.tag = tag;
  if (!breakerSubmitCommand(c)) publishAck(cmdName(t), false, "QUEUE_FULL", tag);
}

// ---------- ผลลัพธ์จากบอร์ด -> ack ----------
static void handleResults() {
  BreakerResult r;
  while (breakerPollResult(r)) {              // ดึงออกจากคิวเสมอ แม้ MQTT หลุด (กันคิวเต็ม)
    LOG("[IOT] result %s -> %s\n", cmdName(r.type), breakerResultText(r.code));
    publishAck(cmdName(r.type), r.code == BRES_OK, breakerResultText(r.code), r.tag);
    forceTelemetry = true;
  }
}

// =====================================================================
//                     รับข้อความจาก MQTT (เรียกใน mqtt.loop())
// =====================================================================
#define NUM_OK(key) (!doc[key].isNull() && !doc[key].is<const char*>())

static void onMessage(char* topic, uint8_t* payload, unsigned int length) {
  // ---------- ช่องทางง่าย: <prefix>/cmd/relay ส่งข้อความธรรมดา ----------
  if (strcmp(topic, tCmdRelay) == 0) {
    char t[16];
    unsigned int n = (length < sizeof(t) - 1) ? length : (unsigned int)(sizeof(t) - 1);
    for (unsigned int i = 0; i < n; i++) t[i] = (char)tolower((int)payload[i]);
    t[n] = '\0';
    if (!strcmp(t, "on") || !strcmp(t, "1") || !strcmp(t, "true"))        submit(BCMD_RELAY_ON, 0, 0, 0);
    else if (!strcmp(t, "off") || !strcmp(t, "0") || !strcmp(t, "false")) submit(BCMD_RELAY_OFF, 0, 0, 0);
    else publishAck("relay", false, "BAD_VALUE", 0);
    return;
  }
  if (strcmp(topic, tCmd) != 0) return;

  // ---------- คำสั่ง JSON ----------
  JsonDocument doc;
  if (deserializeJson(doc, payload, length)) {
    publishAck("unknown", false, "BAD_JSON", 0);
    return;
  }
  const char* cmd = doc["cmd"] | "";
  uint32_t tag = (uint32_t)(doc["id"] | 0);
  LOG("[IOT] cmd: %s\n", cmd);

  if (!strcmp(cmd, "relay")) {
    int state = -1;                                           // 1 = on, 0 = off
    if (doc["value"].is<bool>())      state = doc["value"].as<bool>() ? 1 : 0;
    else if (doc["value"].is<int>())  state = doc["value"].as<int>() ? 1 : 0;
    else {
      const char* sv = doc["value"] | "";
      if (!strcasecmp(sv, "on"))       state = 1;
      else if (!strcasecmp(sv, "off")) state = 0;
    }
    if (state < 0) publishAck("relay", false, "BAD_VALUE", tag);
    else           submit(state ? BCMD_RELAY_ON : BCMD_RELAY_OFF, 0, 0, tag);

  } else if (!strcmp(cmd, "set_overload")) {
    if (NUM_OK("value")) submit(BCMD_SET_OVERLOAD, doc["value"].as<float>(), 0, tag);
    else publishAck("set_overload", false, "BAD_VALUE", tag);

  } else if (!strcmp(cmd, "set_voltage")) {
    if (NUM_OK("min") && NUM_OK("max")) submit(BCMD_SET_VOLTAGE, doc["min"].as<float>(), doc["max"].as<float>(), tag);
    else publishAck("set_voltage", false, "BAD_VALUE", tag);

  } else if (!strcmp(cmd, "set_timer")) {
    if (NUM_OK("value")) submit(BCMD_SET_TIMER, doc["value"].as<float>(), 0, tag);
    else publishAck("set_timer", false, "BAD_VALUE", tag);

  } else if (!strcmp(cmd, "set_mode")) {
    const char* sv = doc["value"] | "";
    if (!strcasecmp(sv, "timer"))                                   submit(BCMD_SET_MODE, 1, 0, tag);
    else if (!strcasecmp(sv, "cont") || !strcasecmp(sv, "continuous")) submit(BCMD_SET_MODE, 0, 0, tag);
    else publishAck("set_mode", false, "BAD_VALUE", tag);

  } else if (!strcmp(cmd, "set_price")) {
    if (NUM_OK("value")) submit(BCMD_SET_PRICE, doc["value"].as<float>(), 0, tag);
    else publishAck("set_price", false, "BAD_VALUE", tag);

  } else if (!strcmp(cmd, "reset_fault")) {
    submit(BCMD_RESET_FAULT, 0, 0, tag);

  } else if (!strcmp(cmd, "reset_energy")) {
    submit(BCMD_RESET_ENERGY, 0, 0, tag);

  } else if (!strcmp(cmd, "get_status")) {
    forceTelemetry = true;
    publishAck("get_status", true, "OK", tag);

  } else {
    publishAck("unknown", false, "UNKNOWN_CMD", tag);
  }
}

// =====================================================================
//                          เชื่อมต่อ MQTT
// =====================================================================
static bool connectMqtt() {
  LOG("[IOT] MQTT connecting to %s:%d ...\n", MQTT_HOST, (int)MQTT_PORT);
  // Last Will: ถ้าบอร์ดหลุดโดยไม่ได้บอกลา broker จะประกาศ "offline" แทน
  bool ok = mqtt.connect(clientId, MQTT_USER, MQTT_PASS, tStatus, 1, true, "offline");
  if (!ok) {
    LOG("[IOT] MQTT failed, rc=%d  (-2 = ต่อ broker ไม่ได้, 4/5 = user/pass ผิด)\n", mqtt.state());
    return false;
  }
  mqtt.publish(tStatus, "online", true);
  mqtt.subscribe(tCmdAll, 1);
  forceTelemetry = true;
  LOG("[IOT] MQTT connected\n");
  return true;
}

// =====================================================================
//                     วนทำงาน 1 รอบ (เรียกจาก task)
// =====================================================================
static void iotStep() {
  unsigned long now = millis();

  // ---------- 1) Wi-Fi ----------
  if (WiFi.status() != WL_CONNECTED) {
    breakerSetNetState(0);
    wifiWasUp = false;
    if (mqtt.connected()) mqtt.disconnect();
    if (!wifiStarted || (now - lastWifiTry) >= WIFI_RETRY_MS) {
      wifiStarted = true;
      lastWifiTry = now;
      LOG("[IOT] WiFi connecting to \"%s\" ...\n", WIFI_SSID);
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#ifdef IOT_WIFI_TX_POWER
      WiFi.setTxPower(IOT_WIFI_TX_POWER);        // ลดกำลังส่งถ้าไฟเลี้ยงไม่พอ
#endif
    }
    handleResults();
    return;
  }

  if (!wifiWasUp) {                              // เพิ่งต่อ Wi-Fi ได้
    wifiWasUp = true;
    LOG("[IOT] WiFi OK  IP=%s  RSSI=%d\n", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
    configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");   // ซิงก์เวลา (จำเป็นถ้าตรวจใบรับรอง)
    lastMqttTry = now - mqttBackoff;             // ให้ลองต่อ MQTT ทันที
  }

  // ---------- 2) MQTT ----------
  if (!mqtt.connected()) {
    breakerSetNetState(1);
    if ((now - lastMqttTry) >= mqttBackoff) {
      lastMqttTry = now;
      if (connectMqtt()) {
        mqttBackoff = MQTT_RETRY_MIN_MS;
        breakerSetNetState(2);
      } else {
        mqttBackoff = (mqttBackoff * 2 > MQTT_RETRY_MAX_MS) ? MQTT_RETRY_MAX_MS : mqttBackoff * 2;
      }
    }
    handleResults();
    return;
  }

  breakerSetNetState(2);
  mqtt.loop();                                   // รับข้อความ/ส่ง keep-alive (onMessage ถูกเรียกที่นี่)
  handleResults();

  // ---------- 3) ส่ง telemetry ----------
  if ((now - lastStateChk) >= STATE_POLL_MS) {
    lastStateChk = now;
    static bool    lastRelay = false;
    static uint8_t lastErr = 0, lastMode = 0;
    BreakerStatus s = breakerGetStatus();
    if (s.relayOn != lastRelay || s.error != lastErr || s.mode != lastMode) {
      lastRelay = s.relayOn; lastErr = s.error; lastMode = s.mode;
      forceTelemetry = true;                     // สถานะสำคัญเปลี่ยน ส่งทันที
    }
    if (forceTelemetry || (now - lastTelemetry) >= TELEMETRY_MS) {
      forceTelemetry = false;
      lastTelemetry = now;
      publishTelemetry(s);
    }
  }
}

static void iotTask(void* arg) {
  (void)arg;
  for (;;) {
    iotStep();
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

// =====================================================================
//                        เริ่มระบบ IoT (เรียกจาก setup())
// =====================================================================
void iotSetup() {
#if IOT_DEBUG
  Serial.begin(115200);
#endif
  snprintf(clientId, sizeof(clientId), "breaker-%06X", (unsigned int)(ESP.getEfuseMac() & 0xFFFFFF));
  snprintf(tTelemetry, sizeof(tTelemetry), "%s/telemetry", TOPIC_PREFIX);
  snprintf(tStatus,    sizeof(tStatus),    "%s/status",    TOPIC_PREFIX);
  snprintf(tAck,       sizeof(tAck),       "%s/ack",       TOPIC_PREFIX);
  snprintf(tCmd,       sizeof(tCmd),       "%s/cmd",       TOPIC_PREFIX);
  snprintf(tCmdAll,    sizeof(tCmdAll),    "%s/cmd/#",     TOPIC_PREFIX);
  snprintf(tCmdRelay,  sizeof(tCmdRelay),  "%s/cmd/relay", TOPIC_PREFIX);

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);                        // ไม่เขียนค่า Wi-Fi ลง Flash ทุกครั้ง

#ifdef MQTT_CA_CERT
  netClient.setCACert(MQTT_CA_CERT);             // ตรวจใบรับรองของ broker (ปลอดภัยกว่า)
#else
  netClient.setInsecure();                       // ไม่ตรวจใบรับรอง: ต่อง่าย ใช้ได้กับ HiveMQ แต่ปลอดภัยน้อยกว่า
#endif

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(MQTT_BUFFER_BYTES);         // ค่าเริ่มต้น 256 ไบต์ ไม่พอสำหรับ JSON ของเรา
  mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
  mqtt.setSocketTimeout(5);

  // task แยก บนแกน 0 (loop() ของ Arduino อยู่แกน 1) stack 12 KB เผื่อ TLS
  xTaskCreatePinnedToCore(iotTask, "iot", 12288, NULL, 1, NULL, 0);
}

#else   // IOT_ENABLED == 0

void iotSetup() {}

#endif