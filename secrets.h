#pragma once
/*
  ⚠️ ไฟล์นี้เก็บรหัสผ่าน
     - ห้ามอัปโหลดขึ้น GitHub และห้ามส่งในแชทกลุ่ม (ใส่ไว้ใน .gitignore แล้ว)
     - ให้แก้ค่าด้านล่างเป็นของจริงของคุณ แล้วบันทึกก่อน Build
*/

// 1 = เปิดระบบ IoT, 0 = ปิดทั้งหมด (บอร์ดทำงานเดี่ยวได้ตามปกติ ไม่ต่อ Wi-Fi)
#define IOT_ENABLED 1

// ---------- Wi-Fi (ต้องเป็น 2.4 GHz) ----------
#define WIFI_SSID      "esp32_board"
#define WIFI_PASSWORD  "22331122"

// ---------- HiveMQ Cloud (ดูจากหน้า Cluster Details) ----------
#define MQTT_HOST      "5a04b7d592b04da29240bdd91de025de.s1.eu.hivemq.cloud"
#define MQTT_PORT      8883
#define MQTT_USER      "esp32_board"     // ของบอร์ด (ให้มือถือใช้อีกชุดหนึ่งต่างหาก)
#define MQTT_PASS      "22331122"

// ---------- ชื่อ topic นำหน้า: เปลี่ยนเป็นชุดตัวอักษรสุ่มของคุณเอง (กันคนอื่นเดา) ----------
#define TOPIC_PREFIX   "breaker/change-me-a1b2c3"

// ---------- ตัวเลือกเสริม ----------
// ลดกำลังส่ง Wi-Fi ถ้าบอร์ดรีเซ็ตเพราะไฟเลี้ยงไม่พอ (ลบ // ข้างหน้าเพื่อเปิดใช้)
// #define IOT_WIFI_TX_POWER WIFI_POWER_15dBm

// ปิดข้อความ debug ใน Serial Monitor (ค่าเริ่มต้น 1 = เปิด)
// #define IOT_DEBUG 0

// ตรวจใบรับรองของ broker (ปลอดภัยกว่า) ถ้าไม่ define จะใช้โหมด setInsecure()
// #define MQTT_CA_CERT "-----BEGIN CERTIFICATE-----\n...ใส่ใบรับรอง Root CA ของ HiveMQ...\n-----END CERTIFICATE-----\n"