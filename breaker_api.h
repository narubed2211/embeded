#pragma once
#include <Arduino.h>

/*
  =====================================================================
   breaker_api.h  —  "จุดเชื่อม" ระหว่างโค้ดบอร์ด (main.cpp) กับส่วน IoT (iot.cpp)
  =====================================================================
   main.cpp  = เจ้าของระบบป้องกัน/รีเลย์/จอ/ปุ่ม  (ห้ามทีม IoT แก้)
   iot.cpp   = Wi-Fi + MQTT (HiveMQ)               (ทีม IoT แก้ได้ตามสบาย)

   กฎเหล็กของทีม IoT (รายละเอียดเต็มดูในเอกสารแยก):
    1) ห้ามสั่งรีเลย์หรือขา GPIO ของบอร์ดโดยตรง สั่งผ่าน breakerSubmitCommand() เท่านั้น
    2) ห้ามเรียกฟังก์ชัน/อ่านตัวแปรใน main.cpp ตรงๆ ใช้เฉพาะฟังก์ชันในไฟล์นี้
    3) ห้าม while/รอ ค้างใน loop() ของ main  (iot.cpp ทำงานใน task แยกของตัวเอง)
    4) ห้ามปิด/แก้ Watchdog, ห้ามแก้ค่าป้องกันข้ามเงื่อนไข (บอร์ดตรวจและปฏิเสธเอง)
    5) รหัสผ่านทั้งหมดอยู่ใน secrets.h เท่านั้น ห้าม commit/แชร์
  =====================================================================
*/

// ---------------------------------------------------------------------
//  สถานะของเบรกเกอร์ (สำเนาล่าสุด อัปเดตทุก ~200 ms)
// ---------------------------------------------------------------------
struct BreakerStatus {
  float v, i, p, kwh, pf, hz;      // ค่าจาก PZEM (ถ้า sensorOk = false ค่าอาจเป็นค่าเก่า)
  bool  relayOn;                   // รีเลย์เปิดอยู่หรือไม่
  bool  sensorOk;                  // อ่าน PZEM ได้ปกติหรือไม่
  uint8_t mode;                    // 0 = CONT (ต่อเนื่อง), 1 = TIMER
  uint8_t error;                   // 0 = ไม่มี, 1 = overload, 2 = under-voltage, 3 = over-voltage, 4 = sensor
  bool  faultActive;               // true = ตัดไฟเพราะ error อยู่ (ต้องรีเซ็ต)
  bool  overloadPending;           // true = กระแสเกิน กำลังนับถอยหลังก่อนตัด
  bool  recoveryLocked;            // true = Auto-Recovery ครบจำนวนครั้ง ต้องรีเซ็ตเอง
  int   timerSecLeft;              // เวลา Timer ที่เหลือ (วินาที) 0 = ไม่ได้นับ
  int   recoverySecLeft;           // เวลารอ Auto-Recovery ที่เหลือ (วินาที) 0 = ไม่ได้รอ
  float overloadLimitA;            // ค่าตั้ง: กระแสตัดไฟ (A)
  float vMin, vMax;                // ค่าตั้ง: แรงดันต่ำสุด/สูงสุด (V)
  int   timerMinutes;              // ค่าตั้ง: เวลา Timer (นาที)
  float unitPrice;                 // ค่าตั้ง: ราคาค่าไฟ (บาท/หน่วย)
  bool  remoteAllowed;             // อนุญาตสั่งเปิด/ตั้งค่าจากระยะไกลหรือไม่ (ตั้งที่ตัวเครื่อง)
};

// ---------------------------------------------------------------------
//  คำสั่งจาก IoT -> บอร์ด
// ---------------------------------------------------------------------
enum BreakerCmdType : uint8_t {
  BCMD_RELAY_ON,        // เปิดรีเลย์ (ผ่านเงื่อนไขป้องกันเดียวกับปุ่ม BTN2)
  BCMD_RELAY_OFF,       // ปิดรีเลย์ (สั่งได้เสมอ)
  BCMD_SET_OVERLOAD,    // a = กระแส (A)         0.1 - 100
  BCMD_SET_VOLTAGE,     // a = V-Min, b = V-Max  ต้อง 100 <= a < b <= 300
  BCMD_SET_TIMER,       // a = นาที              1 - 1440
  BCMD_SET_MODE,        // a = 0 (CONT) / 1 (TIMER)
  BCMD_SET_PRICE,       // a = บาทต่อหน่วย       0 - 100
  BCMD_RESET_FAULT,     // ล้าง error / ยกเลิกการรอ Recovery (รีเลย์ยังคงปิด)
  BCMD_RESET_ENERGY     // รีเซ็ตค่าพลังงานสะสมใน PZEM
};

struct BreakerCommand {
  BreakerCmdType type;
  float a;
  float b;
  uint32_t tag;         // เลขอ้างอิงใดๆ (จะถูกส่งกลับมาในผลลัพธ์)
};

// ---------------------------------------------------------------------
//  ผลลัพธ์บอร์ด -> IoT
// ---------------------------------------------------------------------
enum BreakerResultCode : uint8_t {
  BRES_OK,
  BRES_BAD_VALUE,                // ค่าอยู่นอกขอบเขต
  BRES_DENIED_REMOTE_DISABLED,   // ที่ตัวเครื่องปิดการสั่งจากระยะไกลไว้
  BRES_DENIED_VOLTAGE,           // แรงดันผิดปกติ ไม่อนุญาตให้เปิด
  BRES_DENIED_SENSOR,            // ไม่มีข้อมูลจาก PZEM ไม่อนุญาตให้เปิด
  BRES_DENIED_FAULT,             // ระบบอยู่ในสถานะ error/recovery ต้องรีเซ็ตก่อน
  BRES_FAILED                    // ทำไม่สำเร็จ (เช่น PZEM ไม่ตอบ)
};

struct BreakerResult {
  BreakerCmdType type;
  BreakerResultCode code;
  uint32_t tag;
};

// ---------------------------------------------------------------------
//  ฟังก์ชันที่ iot.cpp เรียกใช้ได้ (ทั้งหมด thread-safe)
// ---------------------------------------------------------------------
BreakerStatus breakerGetStatus();                    // ขอสำเนาสถานะล่าสุด
bool breakerSubmitCommand(const BreakerCommand &c);  // ส่งคำสั่งเข้าคิว (false = คิวเต็ม)
bool breakerPollResult(BreakerResult &r);            // ดึงผลลัพธ์ที่เสร็จแล้ว (ไม่บล็อก, false = ไม่มี)
void breakerSetNetState(uint8_t state);              // แจ้งจอ OLED: 0 = ไม่ต่อ, 1 = Wi-Fi, 2 = MQTT
const char* breakerResultText(BreakerResultCode c);  // ข้อความของผลลัพธ์ เช่น "OK", "DENIED_VOLTAGE"

// ---------------------------------------------------------------------
//  ฟังก์ชันที่ main.cpp เรียก (เขียนไว้ใน iot.cpp)
// ---------------------------------------------------------------------
void iotSetup();   // เริ่ม task ของ IoT (เรียกครั้งเดียวใน setup())