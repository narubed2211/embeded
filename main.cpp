/*
  =====================================================================
   SMART BREAKER v2  (ESP32 + PZEM-004T v3 + SSD1306 + Relay)
  =====================================================================
  สิ่งที่เพิ่ม/แก้จากเวอร์ชันเดิม (ค่าที่ปรับได้อิสระยังอยู่ครบ)
   - ตรวจจับ PZEM หลุด (ตัดไฟเพื่อความปลอดภัย) + ไม่บล็อกลูปตอนอ่านพลาด
   - Overload มี "หน่วงเวลา" กัน inrush และ "ตัดทันที" ถ้าเกิน 1.5 เท่า
   - แรงดันตก/เกินต้องผิดปกติต่อเนื่องก่อนจึงตัด (กันสัญญาณกระตุก)
   - Auto-Recovery จำกัดจำนวนครั้ง (ไม่วนเปิด-ตัดไม่รู้จบ) + กลับมานับเวลา Timer ต่อ
   - บันทึกค่าที่ตั้งลง Flash (Preferences) ไฟดับแล้วค่าไม่หาย
   - หน้าตั้งค่าเพิ่ม: ราคาค่าไฟ, โหมดเสียง
   - ปุ่ม: debounce แยกต่อปุ่ม, กดค้างเพื่อเพิ่ม/ลดเร็วขึ้น, OFF ตัดทันทีที่กด,
           กด BTN1 ค้างเพื่อกลับหน้าหลัก, กดค้าง BTN3 รีเซ็ตค่าไฟพร้อม progress bar
   - หน้าตั้งค่า/Energy หมดเวลา 30 วิ แล้วกลับหน้าหลักเอง
   - จอวาดเฉพาะตอนข้อมูลเปลี่ยน (redraw flag) + I2C 400 kHz → ลื่นขึ้น
   - OLED เสีย/ไม่ต่อ ระบบป้องกันยังทำงาน (เดิมค้างใน for(;;))
   - LED 3 สี + Buzzer แบบ non-blocking
   - Hardware Watchdog

  ---------------------------------------------------------------------
   ขาใหม่ (เปลี่ยนได้ที่ #define ด้านล่าง)
     GPIO25 -> Buzzer (active buzzer; ถ้าเป็นแบบ 5V แนะนำขับผ่านทรานซิสเตอร์ NPN)
     GPIO19 -> LED1 ซ้าย  (ตามโครงสร้าง PCB)
     GPIO15 -> LED2 กลาง  (ตามโครงสร้าง PCB)
     GPIO23 -> LED3 ขวา   (ตามโครงสร้าง PCB)

  ความหมายของ LED (ออกแบบให้อ่านได้แม้ LED เป็นสีเดียวกันทั้ง 3 ดวง
  โดยแยกจาก "ตำแหน่ง" และ "จังหวะกระพริบ"  เรียง LED1-LED2-LED3 จากซ้ายไปขวา = GPIO19/15/23)
     LED1 กระพริบสั้นๆ ทุก 2 วิ ............ Standby (รีเลย์ปิด ปกติ)
     LED1 ติดค้าง ........................... รีเลย์เปิด ปกติ
     LED1 ติดค้าง + LED2 ติดค้าง ............ โหลดสูง (>= 90% ของ limit)
     LED1 ติดค้าง + LED2 กระพริบเร็ว ........ Overload กำลังนับถอยหลังก่อนตัด
     LED1 ติดค้าง + LED2 กระพริบช้า ......... Timer เหลือ <= 10 วิ
     LED2 ดวงเดียวกระพริบช้า ................ รอ Auto-Recovery / หา PZEM ไม่เจอ (ตอนรีเลย์ปิด)
     ทั้ง 3 ดวงกระพริบเร็วพร้อมกัน .......... Overload trip
     ไฟวิ่ง LED1 -> LED2 -> LED3 วนซ้ำ ...... แรงดันตก/เกิน trip
     ทั้ง 3 ดวงติดค้าง ....................... PZEM หลุด (Sensor error)
     (รายละเอียดชนิด error ดูได้บนจอ OLED เสมอ)
  =====================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PZEM004Tv30.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include "breaker_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ===================== IoT Global =====================
QueueHandle_t cmdQueue;
QueueHandle_t resQueue;
uint8_t netState = 0;

// ===================== จอ OLED =====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;           // false = ไม่มี/ใช้จอไม่ได้ (ระบบป้องกันยังทำงานปกติ)

// ===================== ขา I/O =====================
#define PIN_RELAY      26
#define PIN_BTN1       5
#define PIN_BTN2       2
#define PIN_BTN3       18
#define PIN_BUZZER     25
#define PIN_LED1       19   // ซ้าย   (ตาม PCB)
#define PIN_LED2       15   // กลาง   (ตาม PCB, Strapping Pin แต่ใช้เป็น LED ได้)
#define PIN_LED3       23   // ขวา    (ตาม PCB)

// โมดูลรีเลย์ส่วนมากเป็น Active-HIGH (ตามโค้ดเดิม) ถ้าของคุณเป็น Active-LOW ให้เปลี่ยนเป็น false
#define RELAY_ACTIVE_HIGH  true
#define LED_ACTIVE_HIGH    true
#define BUZZER_ACTIVE_HIGH false

PZEM004Tv30 pzem(Serial2, 17, 16);
Preferences prefs;

// ===================== ชนิดข้อมูล =====================
enum ScreenPage {
  PAGE_MAIN, PAGE_ENERGY, PAGE_SET_MODE,
  PAGE_SET_OVERLOAD, PAGE_SET_VMIN, PAGE_SET_VMAX,
  PAGE_SET_TIMER, PAGE_SET_PRICE, PAGE_SET_SOUND,
  PAGE_ERROR, PAGE_RECOVERY
};
enum OperatingMode { MODE_CONT, MODE_TIMER };
enum ErrorType { ERR_NONE, ERR_OVERLOAD, ERR_UNDER_V, ERR_OVER_V, ERR_SENSOR };

enum BtnEvent { EV_NONE, EV_DOWN, EV_REPEAT, EV_HOLD, EV_UP_SHORT, EV_UP };

struct Button {
  uint8_t pin;
  uint16_t holdMs;            // เวลากดค้างเพื่อให้เกิด EV_HOLD (0 = ไม่ใช้)
  bool rawLast;               // ค่าดิบรอบก่อน (true = กด)
  bool pressed;               // สถานะหลัง debounce (true = กด)
  bool holdFired;
  bool swallow;               // กดตอนจอดับ -> ใช้ปลุกจออย่างเดียว ไม่สั่งงาน
  unsigned long lastRawChange;
  unsigned long pressStart;
  unsigned long lastRepeat;
};

// ===================== ค่าคงที่ปรับจูนได้ =====================
const unsigned long PZEM_INTERVAL_MS     = 1000;    // อ่าน PZEM ทุกกี่ ms
const uint8_t       PZEM_FAIL_LIMIT      = 3;       // อ่านพลาดติดกันกี่ครั้งถึงถือว่า Sensor หลุด

const unsigned long OVERLOAD_DELAY_MS    = 3000;    // กระแสเกิน limit ต่อเนื่องกี่ ms จึงตัด
const float         OVERLOAD_INSTANT_FACTOR = 1.5;  // เกิน limit กี่เท่าให้ตัดทันที
const float         OVERLOAD_WARN_RATIO  = 0.9;     // เกินกี่ % ของ limit ให้ LED2 ติด
const unsigned long VOLT_DELAY_MS        = 1000;    // แรงดันผิดปกติต่อเนื่องกี่ ms จึงตัด

const int           RECOVERY_DELAY_S     = 180;     // ดีเลย์ 3 นาที (ป้องกันคอมเพรสเซอร์)
const uint8_t       MAX_RECOVERY_RETRY   = 3;       // Auto-Recovery ได้กี่ครั้ง (ก่อนต้องกดรีเซ็ตเอง)
const unsigned long RECOVERY_RESET_MS    = 600000UL;// เปิดต่อเนื่องนานเท่านี้ ถือว่าไฟนิ่ง นับครั้งใหม่

const unsigned long screenSaverDelay     = 180000;  // จอดับถ้าไม่กดปุ่ม 3 นาที
const unsigned long PAGE_TIMEOUT_MS      = 30000;   // หน้าตั้งค่า/Energy กลับหน้าหลักเองใน 30 วิ
const unsigned long ALARM_MAX_MS         = 30000;   // Buzzer แจ้ง trip ดังนานสุดกี่ ms

const unsigned long DEBOUNCE_MS          = 30;
const unsigned long REPEAT_DELAY_MS      = 500;     // กดค้างเกินนี้เริ่ม auto-repeat
const unsigned long HOME_HOLD_MS         = 1000;    // กด BTN1 ค้างเพื่อกลับหน้าหลัก
const unsigned long ENERGY_RESET_HOLD_MS = 2000;    // กด BTN3 ค้างเพื่อรีเซ็ตค่าไฟ
const unsigned long MIN_DRAW_INTERVAL_MS = 30;

const float OVERLOAD_MAX_A = 100.0;   // PZEM-004T v3 วัดได้สูงสุด 100 A
const float VMIN_FLOOR     = 100.0;   // ตั้ง V-Min ต่ำสุด (เหมือนเดิม)
const float VMAX_CEIL      = 300.0;   // ตั้ง V-Max สูงสุด
const int   TIMER_MAX_MIN  = 1440;    // ตั้ง Timer สูงสุด 24 ชม.

#define WDT_TIMEOUT_S 8

// ===================== พารามิเตอร์ระบบ (ปรับได้จากหน้าจอ + เก็บลง Flash) =====================
float unitPrice      = 5.0;    // 💵 ราคาค่าไฟต่อหน่วย (บาท)
float overloadLimitA = 5.0;    // ⚡ กระแสตัดไฟ (A)
float vGuardMin      = 200.0;  // 📉 ไฟตกตัดที่ (V)
float vGuardMax      = 240.0;  // 📈 ไฟเกินตัดที่ (V)
int   timerMinutes   = 60;     // ⏳ ตั้งเวลา (นาที)
OperatingMode currentMode = MODE_CONT;
uint8_t soundMode    = 2;      // 0 = ปิดเสียง, 1 = เฉพาะแจ้งเตือน, 2 = ทุกเสียง
bool settingsDirty   = false;

// ===================== สถานะระบบ =====================
ScreenPage currentPage  = PAGE_MAIN;
ErrorType  currentError = ERR_NONE;
bool relayState = false;
unsigned long relayOnSince = 0;

// Timer / Recovery
unsigned long lastTimerTick = 0;
int  activeTimerSeconds = 0;
int  resumeTimerSeconds = 0;     // เวลาที่เหลือก่อนโดนตัดเพราะแรงดัน (ไว้นับต่อหลัง Recovery)
unsigned long lastRecoveryTick = 0;
int  recoverySeconds = 0;
uint8_t recoveryAttempts = 0;
bool recoveryLocked = false;     // true = เกินจำนวนครั้ง ต้องกด BTN1 รีเซ็ตเอง

// ตัวจับเวลาผิดปกติ (0 = ไม่ได้นับ)
unsigned long oclSince  = 0;     // เริ่ม overload เมื่อไร
unsigned long vBadSince = 0;     // เริ่มแรงดันผิดปกติเมื่อไร

// ค่าจาก PZEM
unsigned long lastPzemCheck = 0;
float currentV = 0.0, currentI = 0.0, currentP = 0.0;
float currentEnergy = 0.0, currentHz = 0.0, currentPF = 0.0;
bool    dataValid = false;
uint8_t pzemFailCount = 0;

// จอ
bool screenOn = true;
unsigned long lastActivity = 0;
bool needRedraw = true;
unsigned long lastDraw = 0;
unsigned long lastHoldDraw = 0;
unsigned long lastBlinkDraw = 0;
char msgText[22] = "";
unsigned long msgUntil = 0;

// ปุ่ม
Button btn1 = { PIN_BTN1, HOME_HOLD_MS,         false, false, false, false, 0, 0, 0 };
Button btn2 = { PIN_BTN2, 0,                    false, false, false, false, 0, 0, 0 };
Button btn3 = { PIN_BTN3, ENERGY_RESET_HOLD_MS, false, false, false, false, 0, 0, 0 };

// ไฟทดสอบตอนเปิดเครื่อง
unsigned long bootTestUntil = 0;

// ===================== Buzzer: รูปแบบเสียง (ms สลับ ดัง,เงียบ,ดัง,... เริ่มที่ "ดัง") =====================
const uint16_t PAT_CLICK[]     = {25};
const uint16_t PAT_CONFIRM[]   = {70, 60, 160};
const uint16_t PAT_RELAY_ON[]  = {60, 60, 60};
const uint16_t PAT_RELAY_OFF[] = {200};
const uint16_t PAT_DENIED[]    = {110, 80, 110, 80, 110};
const uint16_t PAT_TICK[]      = {40};
const uint16_t PAT_TIMER_END[] = {150, 100, 150, 100, 350};
const uint16_t PAT_RECOVER[]   = {100, 80, 100};
const uint16_t PAT_WARN[]      = {80, 120};                       // เล่นซ้ำ
const uint16_t PAT_TRIP[]      = {250, 150, 250, 150, 250, 700};  // เล่นซ้ำ

// ลำดับความสำคัญ: เสียงที่สำคัญกว่าจะแทรกเสียงที่เบากว่าได้
#define PRIO_INFO   1
#define PRIO_NOTICE 2
#define PRIO_WARN   3
#define PRIO_ALARM  4

struct BeepPlayer {
  const uint16_t* pat;
  uint8_t len, idx, prio;
  bool repeat, active;
  unsigned long stepStart;
};
BeepPlayer bz = { NULL, 0, 0, 0, false, false, 0 };
bool warnBeeping = false;
bool alarmMuted = false;
unsigned long alarmUntil = 0;

#define PLAY(p, rep, prio) beep((p), (uint8_t)(sizeof(p) / sizeof((p)[0])), (rep), (prio))

// ===================== ประกาศฟังก์ชัน =====================
void buzzerWrite(bool on);
void ledWrite(uint8_t pin, bool on);
void beep(const uint16_t* pat, uint8_t len, bool repeat, uint8_t prio);
void stopBeep();
void endWarnBeep();
void silenceAlarm();
void updateBuzzer();
void updateIndicators();

void setRelay(bool on);
void enterError(ErrorType e, bool alarm);
void tripRelay(ErrorType e);
void resetFault();
bool voltageBad(float v);

void readPzemData();
void checkProtections();
void checkTimers();
void housekeeping();

BtnEvent pollButton(Button &b);
BtnEvent readButton(Button &b);
uint8_t holdMultiplier(const Button &b);
void handleButtons();
void onBtn1Short();
void onBtn1Hold();
void onBtn2(bool isRepeat);
void onBtn3(BtnEvent e);
void requestRelayOn();
void adjustSetting(int dir, uint8_t mult, bool isRepeat);
ScreenPage nextPage(ScreenPage p);
bool isSettingPage(ScreenPage p);
bool isLivePage(ScreenPage p);

void loadSettings();
void saveSettings();

void wakeScreen();
void requestRedraw();
void showMessage(const char* s);
void renderIfNeeded();
void drawBar(int x, int y, int w, int h, float frac);
void drawMain();
void drawEnergy();
void drawSettingValue(const char* title, const char* value, const char* unit);
void drawSetMode();
void drawSetSound();
void drawError();
void drawRecovery();
void processIotCommands();

// =====================================================================
//                               SETUP
// =====================================================================
void setup() {
  // ตั้งรีเลย์/เสียง/ไฟให้ "ปิด" ก่อนอย่างอื่น
  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false);
  pinMode(PIN_BUZZER, OUTPUT);     buzzerWrite(false);
  pinMode(PIN_LED1, OUTPUT);  ledWrite(PIN_LED1, false);
  pinMode(PIN_LED2, OUTPUT);  ledWrite(PIN_LED2, false);
  pinMode(PIN_LED3, OUTPUT);  ledWrite(PIN_LED3, false);

  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_BTN3, INPUT_PULLUP);

  Wire.begin(21, 22);
  Wire.setClock(400000);           // I2C 400 kHz วาดจอเร็วขึ้น ~4 เท่า
  displayOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (displayOK) {
    display.setTextWrap(false);
    display.clearDisplay();
    display.display();
  }

  loadSettings();

  readPzemData();                  // อ่านค่าแรกก่อนเริ่มทำงาน
  lastPzemCheck = millis();
  lastActivity  = millis();
  bootTestUntil = millis() + 600;  // ไฟ LED ติดทุกดวง 0.6 วิ เป็นการทดสอบหลอด
  PLAY(PAT_CONFIRM, false, PRIO_NOTICE);
  requestRedraw();

  // Hardware Watchdog: ถ้าโปรแกรมค้างเกินเวลา บอร์ดจะรีเซ็ตเอง (รีเลย์กลับเป็น OFF)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  esp_task_wdt_config_t wdtCfg;
  wdtCfg.timeout_ms     = WDT_TIMEOUT_S * 1000;
  wdtCfg.idle_core_mask = 0;
  wdtCfg.trigger_panic  = true;
  esp_task_wdt_reconfigure(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif

  // เริ่ม Queue และ IoT
  cmdQueue = xQueueCreate(10, sizeof(BreakerCommand));
  resQueue = xQueueCreate(10, sizeof(BreakerResult));
  iotSetup();

  esp_task_wdt_add(NULL);
}

// =====================================================================
//                                LOOP
// =====================================================================
void loop() {
  esp_task_wdt_reset();
  unsigned long now = millis();

  handleButtons();
  checkTimers();
  processIotCommands();

  if (now - lastPzemCheck >= PZEM_INTERVAL_MS) {
    lastPzemCheck = now;
    readPzemData();
    checkProtections();
    if (isLivePage(currentPage)) requestRedraw();
  }

  housekeeping();
  updateBuzzer();
  updateIndicators();
  renderIfNeeded();
}

// =====================================================================
//                    ฮาร์ดแวร์พื้นฐาน: รีเลย์ / LED / Buzzer
// =====================================================================
void setRelay(bool on) {
  relayState = on;
  digitalWrite(PIN_RELAY, (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
  if (on) {
    relayOnSince = millis();
  } else {
    oclSince = 0;
    vBadSince = 0;
    endWarnBeep();
  }
}

void buzzerWrite(bool on) {
  digitalWrite(PIN_BUZZER, (on == BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
}

void ledWrite(uint8_t pin, bool on) {
  digitalWrite(pin, (on == LED_ACTIVE_HIGH) ? HIGH : LOW);
}

// ---------- Buzzer แบบ non-blocking ----------
void beep(const uint16_t* pat, uint8_t len, bool repeat, uint8_t prio) {
  if (soundMode == 0) return;                     // ปิดเสียงทั้งหมด
  if (soundMode == 1 && prio < PRIO_WARN) return; // เฉพาะแจ้งเตือน
  if (bz.active && prio < bz.prio) return;        // เสียงที่สำคัญกว่ากำลังเล่นอยู่
  bz.pat = pat; bz.len = len; bz.idx = 0;
  bz.repeat = repeat; bz.prio = prio; bz.active = true;
  bz.stepStart = millis();
  buzzerWrite(true);
}

void stopBeep() {
  bz.active = false;
  bz.prio = 0;
  buzzerWrite(false);
}

void endWarnBeep() {
  if (warnBeeping) {
    warnBeeping = false;
    if (bz.active && bz.prio == PRIO_WARN) stopBeep();
  }
}

void silenceAlarm() {                              // กดปุ่มใดๆ เพื่อปิดเสียงแจ้งเตือน
  alarmMuted = true;
  if (bz.active && bz.prio >= PRIO_WARN) stopBeep();
}

void updateBuzzer() {
  if (!bz.active) return;
  unsigned long now = millis();

  // เสียง trip ดังนานสุด ALARM_MAX_MS แล้วหยุดเอง (LED ยังกระพริบต่อ)
  if (bz.prio == PRIO_ALARM && (long)(now - alarmUntil) >= 0) { stopBeep(); return; }

  if (now - bz.stepStart >= bz.pat[bz.idx]) {
    bz.idx++;
    bz.stepStart = now;
    if (bz.idx >= bz.len) {
      if (bz.repeat) bz.idx = 0;
      else { stopBeep(); return; }
    }
    buzzerWrite((bz.idx % 2) == 0);                // index คู่ = ดัง, คี่ = เงียบ
  }
}

// ---------- LED 3 ดวง (อ่านได้แม้เป็นสีเดียวกัน: ใช้ "ตำแหน่ง" + "จังหวะกระพริบ") ----------
void updateIndicators() {
  unsigned long now = millis();
  bool l1 = false, l2 = false, l3 = false;

  if (now < bootTestUntil) {
    l1 = l2 = l3 = true;                           // ทดสอบหลอดตอนเปิดเครื่อง
  } else if (currentPage == PAGE_ERROR) {
    if (currentError == ERR_OVERLOAD) {
      bool ph = ((now / 250) % 2) == 0;            // ทั้ง 3 ดวงกระพริบเร็วพร้อมกัน
      l1 = ph; l2 = ph; l3 = ph;
    } else if (currentError == ERR_SENSOR) {
      l1 = l2 = l3 = true;                         // ติดค้างทั้ง 3 ดวง
    } else {
      uint8_t step = (uint8_t)((now / 150) % 3);   // ไฟวิ่ง LED1 -> LED2 -> LED3 (แรงดันผิดปกติ)
      l1 = (step == 0); l2 = (step == 1); l3 = (step == 2);
    }
  } else if (currentPage == PAGE_RECOVERY) {
    l2 = ((now / 500) % 2) == 0;                   // LED2 ดวงเดียวกระพริบช้า
  } else if (relayState) {
    l1 = true;                                     // รีเลย์เปิด: LED1 ติดค้าง
    if (oclSince != 0)                                   l2 = ((now / 125) % 2) == 0;  // Overload นับถอยหลัง
    else if (currentI >= overloadLimitA * OVERLOAD_WARN_RATIO) l2 = true;              // โหลดสูง
    else if (currentMode == MODE_TIMER && activeTimerSeconds > 0 && activeTimerSeconds <= 10)
                                                         l2 = ((now / 500) % 2) == 0;  // Timer ใกล้หมด
  } else {
    l1 = (now % 2000) < 100;                       // Standby: LED1 กระพริบสั้นๆ
    if (!dataValid) l2 = ((now / 1000) % 2) == 0;  // หา PZEM ไม่เจอ: LED2 กระพริบช้า
  }

  ledWrite(PIN_LED1, l1);
  ledWrite(PIN_LED2, l2);
  ledWrite(PIN_LED3, l3);
}

// =====================================================================
//                         อ่าน PZEM + ตรวจป้องกัน
// =====================================================================
bool voltageBad(float v) {
  return (v > 0 && (v < vGuardMin || v > vGuardMax));
}

void readPzemData() {
  float v = pzem.voltage();        // อ่านแรงดันก่อน ถ้าพลาดไม่ต้องอ่านตัวอื่นต่อ (กันลูปค้างหลายร้อย ms)
  if (isnan(v)) {
    if (pzemFailCount < 255) pzemFailCount++;
    if (pzemFailCount >= PZEM_FAIL_LIMIT) {
      dataValid = false;
      if (relayState) {
        tripRelay(ERR_SENSOR);     // ไม่มีข้อมูลวัด = ไม่ปลอดภัย ตัดไฟ
      } else if (currentPage == PAGE_RECOVERY) {
        enterError(ERR_SENSOR, false);
      }
      requestRedraw();
    }
    return;
  }

  pzemFailCount = 0;
  dataValid = true;
  currentV = v;
  float t;
  t = pzem.current();   if (!isnan(t)) currentI = t;
  t = pzem.power();     if (!isnan(t)) currentP = t;
  t = pzem.energy();    if (!isnan(t)) currentEnergy = t;
  t = pzem.frequency(); if (!isnan(t)) currentHz = t;
  t = pzem.pf();        if (!isnan(t)) currentPF = t;
}

void tripRelay(ErrorType e) {
  bool vErr = (e == ERR_UNDER_V || e == ERR_OVER_V);
  resumeTimerSeconds = vErr ? activeTimerSeconds : 0;   // จำเวลาที่เหลือไว้นับต่อหลัง Recovery
  setRelay(false);
  activeTimerSeconds = 0;
  recoveryLocked = vErr && (recoveryAttempts >= MAX_RECOVERY_RETRY);
  enterError(e, true);
}

void enterError(ErrorType e, bool alarm) {
  currentError = e;
  currentPage = PAGE_ERROR;
  if (alarm) {
    alarmMuted = false;
    alarmUntil = millis() + ALARM_MAX_MS;
    PLAY(PAT_TRIP, true, PRIO_ALARM);
  }
  wakeScreen();
  requestRedraw();
}

void resetFault() {
  stopBeep();
  currentError = ERR_NONE;
  recoveryLocked = false;
  recoveryAttempts = 0;
  resumeTimerSeconds = 0;
  currentPage = PAGE_MAIN;
  PLAY(PAT_CONFIRM, false, PRIO_NOTICE);
  requestRedraw();
}

void checkProtections() {
  unsigned long now = millis();

  if (relayState) {
    // 1) ไฟตก/ไฟเกิน: ต้องผิดปกติต่อเนื่อง VOLT_DELAY_MS
    if (voltageBad(currentV)) {
      if (vBadSince == 0) vBadSince = now;
      if (now - vBadSince >= VOLT_DELAY_MS) {
        tripRelay(currentV < vGuardMin ? ERR_UNDER_V : ERR_OVER_V);
        return;
      }
    } else {
      vBadSince = 0;
    }

    // 2) กระแสเกิน: เกินมากตัดทันที / เกินเล็กน้อยรอ OVERLOAD_DELAY_MS
    if (currentI >= overloadLimitA * OVERLOAD_INSTANT_FACTOR) {
      tripRelay(ERR_OVERLOAD);
      return;
    }
    if (currentI >= overloadLimitA) {
      if (oclSince == 0) oclSince = now;
      if (now - oclSince >= OVERLOAD_DELAY_MS) {
        tripRelay(ERR_OVERLOAD);
        return;
      }
    } else {
      oclSince = 0;
    }

    // เสียงเตือนระหว่างนับถอยหลัง overload
    if (oclSince != 0 && !warnBeeping) {
      warnBeeping = true;
      if (!alarmMuted) PLAY(PAT_WARN, true, PRIO_WARN);
    } else if (oclSince == 0) {
      endWarnBeep();
    }

    // ไฟเสถียรนานพอ -> ล้างตัวนับ Recovery
    if (recoveryAttempts > 0 && (now - relayOnSince) >= RECOVERY_RESET_MS) recoveryAttempts = 0;
  }

  // 3) เริ่ม Auto-Recovery เมื่อแรงดันกลับมาปกติ
  if (currentPage == PAGE_ERROR && (currentError == ERR_UNDER_V || currentError == ERR_OVER_V)
      && !recoveryLocked && dataValid) {
    if (currentV >= vGuardMin && currentV <= vGuardMax) {
      stopBeep();
      currentPage = PAGE_RECOVERY;
      recoverySeconds = RECOVERY_DELAY_S;
      lastRecoveryTick = now;
      PLAY(PAT_RECOVER, false, PRIO_NOTICE);
      wakeScreen();
      requestRedraw();
    }
  }

  // ระหว่างรอ Recovery ถ้าแรงดันเพี้ยนอีก -> กลับไปหน้า Error
  if (currentPage == PAGE_RECOVERY && dataValid && voltageBad(currentV)) {
    enterError(currentV < vGuardMin ? ERR_UNDER_V : ERR_OVER_V, false);
  }
}

// =====================================================================
//                              ตัวนับเวลา
// =====================================================================
void checkTimers() {
  unsigned long now = millis();

  // 1) Timer Mode
  if (relayState && currentMode == MODE_TIMER && activeTimerSeconds > 0) {
    if (now - lastTimerTick >= 1000) {
      lastTimerTick += 1000;                         // += กันเวลาคลาดเคลื่อนสะสม
      if (now - lastTimerTick >= 1000) lastTimerTick = now;
      activeTimerSeconds--;
      if (activeTimerSeconds <= 0) {
        activeTimerSeconds = 0;
        setRelay(false);
        PLAY(PAT_TIMER_END, false, PRIO_NOTICE);
      } else if (activeTimerSeconds <= 10) {
        PLAY(PAT_TICK, false, PRIO_NOTICE);          // นับถอยหลัง 10 วิสุดท้าย
      }
      requestRedraw();
    }
  }

  // 2) Auto-Recovery (ดีเลย์ป้องกันคอมเพรสเซอร์)
  if (currentPage == PAGE_RECOVERY && now - lastRecoveryTick >= 1000) {
    lastRecoveryTick += 1000;
    if (now - lastRecoveryTick >= 1000) lastRecoveryTick = now;
    recoverySeconds--;
    if (recoverySeconds <= 0) {
      recoveryAttempts++;
      setRelay(true);
      if (currentMode == MODE_TIMER) {               // นับเวลา Timer ต่อจากที่เหลือ
        activeTimerSeconds = (resumeTimerSeconds > 0) ? resumeTimerSeconds : timerMinutes * 60;
        lastTimerTick = now;
      }
      resumeTimerSeconds = 0;
      currentError = ERR_NONE;
      currentPage = PAGE_MAIN;
      lastActivity = now;                            // ให้จอติดต่ออีกพักหนึ่ง
      PLAY(PAT_RELAY_ON, false, PRIO_NOTICE);
    } else if (recoverySeconds <= 3) {
      PLAY(PAT_TICK, false, PRIO_NOTICE);            // เตือน 3-2-1 ก่อนโหลดกลับมาทำงาน
    }
    requestRedraw();
  }
}

// =====================================================================
//                      งานเบ็ดเตล็ด: timeout / บันทึกค่า / จอดับ
// =====================================================================
void housekeeping() {
  unsigned long now = millis();

  // หน้าตั้งค่า/Energy ไม่มีการกด -> กลับหน้าหลัก
  if ((isSettingPage(currentPage) || currentPage == PAGE_ENERGY) && (now - lastActivity) > PAGE_TIMEOUT_MS) {
    currentPage = PAGE_MAIN;
    requestRedraw();
  }

  // ออกจากหน้าตั้งค่าแล้วค่อยบันทึกลง Flash (ดีเลย์ 15 วินาที เพื่อป้องกัน Flash พังจากการรับคำสั่งรัวๆ)
  static unsigned long dirtySince = 0;
  if (settingsDirty) {
    if (dirtySince == 0) dirtySince = now;
    if (now - dirtySince > 15000 && !isSettingPage(currentPage)) {
      saveSettings();
      settingsDirty = false;
      dirtySince = 0;
    }
  } else {
    dirtySince = 0;
  }

  // ข้อความชั่วคราวหมดเวลา
  if (msgText[0] != '\0' && (long)(now - msgUntil) >= 0) {
    msgText[0] = '\0';
    requestRedraw();
  }

  // ข้อความ HIGH LOAD กระพริบ
  if (currentPage == PAGE_MAIN && oclSince != 0 && now - lastBlinkDraw >= 300) {
    lastBlinkDraw = now;
    requestRedraw();
  }

  // Progress bar ตอนกด BTN3 ค้างในหน้า Energy
  if (currentPage == PAGE_ENERGY && btn3.pressed && !btn3.holdFired && !btn3.swallow && now - lastHoldDraw >= 80) {
    lastHoldDraw = now;
    requestRedraw();
  }

  // Screen Saver (ไม่ดับขณะ Error/Recovery และไม่ดับถ้าไม่มีจอ)
  if (displayOK && screenOn && currentPage != PAGE_ERROR && currentPage != PAGE_RECOVERY
      && (now - lastActivity) > screenSaverDelay) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    screenOn = false;
  }
}

void wakeScreen() {
  lastActivity = millis();
  if (!screenOn) {
    if (displayOK) display.ssd1306_command(SSD1306_DISPLAYON);
    screenOn = true;
    requestRedraw();
  }
}

// =====================================================================
//                                 ปุ่มกด
// =====================================================================
BtnEvent pollButton(Button &b) {
  unsigned long now = millis();
  bool raw = (digitalRead(b.pin) == LOW);
  if (raw != b.rawLast) { b.rawLast = raw; b.lastRawChange = now; }

  // debounce: ค่าต้องนิ่งนานเกิน DEBOUNCE_MS
  if (raw != b.pressed && (now - b.lastRawChange) >= DEBOUNCE_MS) {
    b.pressed = raw;
    if (raw) {
      b.pressStart = now;
      b.lastRepeat = now;
      b.holdFired = false;
      return EV_DOWN;
    }
    return b.holdFired ? EV_UP : EV_UP_SHORT;
  }

  if (b.pressed) {
    unsigned long held = now - b.pressStart;
    if (b.holdMs > 0 && !b.holdFired && held >= b.holdMs) {
      b.holdFired = true;
      return EV_HOLD;
    }
    if (held >= REPEAT_DELAY_MS) {
      unsigned long interval = (held >= 2000) ? 50 : 150;   // กดนานขึ้นวิ่งเร็วขึ้น
      if (now - b.lastRepeat >= interval) {
        b.lastRepeat = now;
        return EV_REPEAT;
      }
    }
  }
  return EV_NONE;
}

// ครอบ pollButton: จัดการ "กดครั้งแรกตอนจอดับ = ปลุกจออย่างเดียว" และบันทึกเวลา activity
BtnEvent readButton(Button &b) {
  BtnEvent e = pollButton(b);
  if (e == EV_NONE) return e;

  if (e == EV_DOWN) {
    if (!screenOn) {
      b.swallow = true;
      wakeScreen();
      return EV_NONE;
    }
    b.swallow = false;
  }
  if (b.swallow) {
    if (e == EV_UP || e == EV_UP_SHORT) b.swallow = false;
    return EV_NONE;
  }
  lastActivity = millis();
  return e;
}

// ตัวคูณความเร็วตอนกดค้างปรับค่า: >2 วิ = x5, >4 วิ = x10
uint8_t holdMultiplier(const Button &b) {
  unsigned long held = millis() - b.pressStart;
  if (held > 4000) return 10;
  if (held > 2000) return 5;
  return 1;
}

void handleButtons() {
  BtnEvent e1 = readButton(btn1);
  BtnEvent e2 = readButton(btn2);
  BtnEvent e3 = readButton(btn3);

  // เสียงคลิกทุกครั้งที่กดลง
  if (e1 == EV_DOWN || e2 == EV_DOWN || e3 == EV_DOWN) PLAY(PAT_CLICK, false, PRIO_INFO);

  // BTN1 : MODE / RESET (ทำงานตอนปล่อย เพื่อแยกกดสั้น/กดค้าง)
  if (e1 == EV_UP_SHORT)      onBtn1Short();
  else if (e1 == EV_HOLD)     onBtn1Hold();

  // BTN2 : UP / ON
  if (e2 == EV_DOWN || e2 == EV_REPEAT) onBtn2(e2 == EV_REPEAT);

  // BTN3 : DOWN / OFF / HOLD เพื่อรีเซ็ตค่าไฟ
  if (e3 != EV_NONE) onBtn3(e3);
}

void onBtn1Short() {
  if (currentPage == PAGE_ERROR || currentPage == PAGE_RECOVERY) {
    resetFault();                                   // รีเซ็ตสถานะฉุกเฉิน / ยกเลิก Recovery
  } else {
    currentPage = nextPage(currentPage);
    requestRedraw();
  }
}

void onBtn1Hold() {
  if (currentPage != PAGE_ERROR && currentPage != PAGE_RECOVERY && currentPage != PAGE_MAIN) {
    currentPage = PAGE_MAIN;                        // ทางลัดกลับหน้าหลัก
    PLAY(PAT_CONFIRM, false, PRIO_NOTICE);
    requestRedraw();
  }
}

void requestRelayOn() {
  if (!dataValid) {
    showMessage("NO SENSOR DATA");
    PLAY(PAT_DENIED, false, PRIO_NOTICE);
    return;
  }
  if (voltageBad(currentV)) {
    showMessage("V OUT OF RANGE");
    PLAY(PAT_DENIED, false, PRIO_NOTICE);
    return;
  }
  bool wasOn = relayState;
  recoveryAttempts = 0;
  setRelay(true);
  if (currentMode == MODE_TIMER) {                  // กด ON ซ้ำ = เริ่มนับ Timer ใหม่ (เหมือนเดิม)
    activeTimerSeconds = timerMinutes * 60;
    lastTimerTick = millis();
  }
  if (!wasOn) PLAY(PAT_RELAY_ON, false, PRIO_NOTICE);
  requestRedraw();
}

void onBtn2(bool isRepeat) {
  if (currentPage == PAGE_ERROR || currentPage == PAGE_RECOVERY) {
    silenceAlarm();
    return;
  }
  if (currentPage == PAGE_MAIN) {
    if (!isRepeat) requestRelayOn();
  } else if (currentPage != PAGE_ENERGY) {
    adjustSetting(+1, holdMultiplier(btn2), isRepeat);
  }
}

void onBtn3(BtnEvent e) {
  if (e == EV_HOLD) {                               // กดค้าง: รีเซ็ตค่าไฟ (เฉพาะหน้า Energy)
    if (currentPage == PAGE_ENERGY) {
      if (pzem.resetEnergy()) {
        currentEnergy = 0.0;
        showMessage("ENERGY RESET OK");
        PLAY(PAT_CONFIRM, false, PRIO_NOTICE);
      } else {
        showMessage("RESET FAILED");
        PLAY(PAT_DENIED, false, PRIO_NOTICE);
      }
      requestRedraw();
    }
    return;
  }
  if (e != EV_DOWN && e != EV_REPEAT) return;

  if (currentPage == PAGE_ERROR || currentPage == PAGE_RECOVERY) {
    silenceAlarm();
  } else if (currentPage == PAGE_MAIN) {
    if (e == EV_DOWN && relayState) {               // OFF ตัดทันทีที่กด
      setRelay(false);
      activeTimerSeconds = 0;
      PLAY(PAT_RELAY_OFF, false, PRIO_NOTICE);
      requestRedraw();
    }
  } else if (currentPage != PAGE_ENERGY) {
    adjustSetting(-1, holdMultiplier(btn3), e == EV_REPEAT);
  }
}

ScreenPage nextPage(ScreenPage p) {
  switch (p) {
    case PAGE_MAIN:         return PAGE_ENERGY;
    case PAGE_ENERGY:       return PAGE_SET_MODE;
    case PAGE_SET_MODE:     return PAGE_SET_OVERLOAD;
    case PAGE_SET_OVERLOAD: return PAGE_SET_VMIN;
    case PAGE_SET_VMIN:     return PAGE_SET_VMAX;
    case PAGE_SET_VMAX:     return (currentMode == MODE_TIMER) ? PAGE_SET_TIMER : PAGE_SET_PRICE;
    case PAGE_SET_TIMER:    return PAGE_SET_PRICE;
    case PAGE_SET_PRICE:    return PAGE_SET_SOUND;
    default:                return PAGE_MAIN;
  }
}

bool isSettingPage(ScreenPage p) {
  return (p == PAGE_SET_MODE || p == PAGE_SET_OVERLOAD || p == PAGE_SET_VMIN ||
          p == PAGE_SET_VMAX || p == PAGE_SET_TIMER || p == PAGE_SET_PRICE || p == PAGE_SET_SOUND);
}

bool isLivePage(ScreenPage p) {
  return (p == PAGE_MAIN || p == PAGE_ENERGY || p == PAGE_ERROR || p == PAGE_RECOVERY);
}

// dir = +1 / -1, mult = ตัวคูณจากการกดค้าง, isRepeat = มาจาก auto-repeat
void adjustSetting(int dir, uint8_t mult, bool isRepeat) {
  switch (currentPage) {
    case PAGE_SET_MODE:
      if (isRepeat) return;
      currentMode = (currentMode == MODE_CONT) ? MODE_TIMER : MODE_CONT;
      break;

    case PAGE_SET_OVERLOAD: {
      float v = overloadLimitA + dir * 0.1f * mult;
      v = roundf(v * 10.0f) / 10.0f;                       // กันทศนิยมคลาดเคลื่อนสะสม
      overloadLimitA = constrain(v, 0.1f, OVERLOAD_MAX_A);
      break;
    }
    case PAGE_SET_VMIN:
      vGuardMin = constrain(vGuardMin + dir * (float)mult, VMIN_FLOOR, vGuardMax - 1.0f);
      break;

    case PAGE_SET_VMAX:
      vGuardMax = constrain(vGuardMax + dir * (float)mult, vGuardMin + 1.0f, VMAX_CEIL);
      break;

    case PAGE_SET_TIMER:
      timerMinutes = constrain(timerMinutes + dir * (int)mult, 1, TIMER_MAX_MIN);
      break;

    case PAGE_SET_PRICE: {
      float v = unitPrice + dir * 0.05f * mult;
      v = roundf(v * 100.0f) / 100.0f;
      unitPrice = constrain(v, 0.0f, 100.0f);
      break;
    }
    case PAGE_SET_SOUND:
      if (isRepeat) return;
      soundMode = (uint8_t)((soundMode + dir + 3) % 3);
      if (soundMode == 0) stopBeep();
      else PLAY(PAT_CONFIRM, false, PRIO_WARN);            // เล่นเสียงตัวอย่าง
      break;

    default:
      return;
  }
  settingsDirty = true;
  requestRedraw();
}

// =====================================================================
//                         บันทึก/โหลดค่าตั้ง (Flash)
// =====================================================================
void loadSettings() {
  prefs.begin("breaker", false);
  unitPrice      = prefs.getFloat("price", unitPrice);
  overloadLimitA = prefs.getFloat("ol",    overloadLimitA);
  vGuardMin      = prefs.getFloat("vmin",  vGuardMin);
  vGuardMax      = prefs.getFloat("vmax",  vGuardMax);
  timerMinutes   = prefs.getInt("tmr",     timerMinutes);
  currentMode    = (prefs.getUChar("mode", (uint8_t)currentMode) == 1) ? MODE_TIMER : MODE_CONT;
  soundMode      = prefs.getUChar("snd",   soundMode);
  prefs.end();

  // กันค่าเพี้ยนจาก Flash
  if (!(overloadLimitA >= 0.1f && overloadLimitA <= OVERLOAD_MAX_A)) overloadLimitA = 5.0;
  if (!(vGuardMin >= VMIN_FLOOR && vGuardMax <= VMAX_CEIL && vGuardMin < vGuardMax)) { vGuardMin = 200.0; vGuardMax = 240.0; }
  if (timerMinutes < 1 || timerMinutes > TIMER_MAX_MIN) timerMinutes = 60;
  if (!(unitPrice >= 0.0f && unitPrice <= 100.0f)) unitPrice = 5.0;
  if (soundMode > 2) soundMode = 2;
}

void saveSettings() {
  prefs.begin("breaker", false);
  prefs.putFloat("price", unitPrice);
  prefs.putFloat("ol",    overloadLimitA);
  prefs.putFloat("vmin",  vGuardMin);
  prefs.putFloat("vmax",  vGuardMax);
  prefs.putInt("tmr",     timerMinutes);
  prefs.putUChar("mode",  (uint8_t)currentMode);
  prefs.putUChar("snd",   soundMode);
  prefs.end();
}

// =====================================================================
//                               หน้าจอ (HMI)
// =====================================================================
void requestRedraw() { needRedraw = true; }

void showMessage(const char* s) {
  strncpy(msgText, s, sizeof(msgText) - 1);
  msgText[sizeof(msgText) - 1] = '\0';
  msgUntil = millis() + 2000;
  requestRedraw();
}

void renderIfNeeded() {
  if (!needRedraw || !screenOn || !displayOK) return;
  unsigned long now = millis();
  if (now - lastDraw < MIN_DRAW_INTERVAL_MS) return;
  needRedraw = false;
  lastDraw = now;

  char buf[24];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  switch (currentPage) {
    case PAGE_MAIN:   drawMain();   break;
    case PAGE_ENERGY: drawEnergy(); break;
    case PAGE_SET_MODE:  drawSetMode();  break;
    case PAGE_SET_SOUND: drawSetSound(); break;
    case PAGE_SET_OVERLOAD:
      snprintf(buf, sizeof(buf), "%.1f A", overloadLimitA);
      drawSettingValue("== SET OVERLOAD ==", buf, NULL);
      break;
    case PAGE_SET_VMIN:
      snprintf(buf, sizeof(buf), "%.0f V", vGuardMin);
      drawSettingValue("== SET V-GUARD MIN ==", buf, NULL);
      break;
    case PAGE_SET_VMAX:
      snprintf(buf, sizeof(buf), "%.0f V", vGuardMax);
      drawSettingValue("== SET V-GUARD MAX ==", buf, NULL);
      break;
    case PAGE_SET_TIMER:
      snprintf(buf, sizeof(buf), "%d Min", timerMinutes);
      drawSettingValue("== SET TIMER ==", buf, NULL);
      break;
    case PAGE_SET_PRICE:
      snprintf(buf, sizeof(buf), "%.2f", unitPrice);
      drawSettingValue("== SET UNIT PRICE ==", buf, "THB per kWh");
      break;
    case PAGE_ERROR:    drawError();    break;
    case PAGE_RECOVERY: drawRecovery(); break;
  }
  display.display();
}

void drawBar(int x, int y, int w, int h, float frac) {
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int fw = (int)((w - 4) * frac);
  if (fw > 0) display.fillRect(x + 2, y + 2, fw, h - 4, SSD1306_WHITE);
}

void drawMain() {
  char buf[24];

  display.setCursor(0, 0);   
  if (netState == 2)      display.print("S-BREAKER [Cloud]");
  else if (netState == 1) display.print("S-BREAKER [Wi-Fi]");
  else                    display.print("SMART BREAKER");

  display.setCursor(98, 0);  display.print(currentMode == MODE_TIMER ? "TIMER" : "CONT.");

  display.setCursor(0, 10);
  if (dataValid) { display.print("V: "); display.print(currentV, 1); display.print(" V"); }
  else           { display.print("V: --.- V  NO SENSOR"); }

  display.setCursor(0, 19);
  if (dataValid) {
    display.print("I: "); display.print(currentI, 2); display.print(" A");
    int pct = (overloadLimitA > 0) ? (int)(currentI / overloadLimitA * 100.0f + 0.5f) : 0;
    if (pct > 999) pct = 999;
    snprintf(buf, sizeof(buf), "%3d%%", pct);
    display.setCursor(98, 19); display.print(buf);                 // % ของ limit
  } else {
    display.print("I: --.-- A");
  }

  display.setCursor(0, 28);
  if (dataValid) { display.print("P: "); display.print(currentP, 1); display.print(" W"); }
  else           { display.print("P: --.- W"); }

  display.setCursor(0, 38);
  display.print("Status: ");
  if (relayState) {
    if (currentMode == MODE_TIMER && activeTimerSeconds > 0) {
      snprintf(buf, sizeof(buf), "ON [%02d:%02d]", activeTimerSeconds / 60, activeTimerSeconds % 60);
      display.print(buf);
    } else {
      display.print("ON (Always)");
    }
  } else {
    display.print("OFF");
  }

  // แถบแสดงโหลดเทียบกับ limit
  drawBar(0, 47, 128, 7, dataValid && overloadLimitA > 0 ? currentI / overloadLimitA : 0);

  display.setCursor(0, 56);
  if (msgText[0] != '\0')      display.print(msgText);
  else if (oclSince != 0)      { if (((millis() / 300) % 2) == 0) display.print("!! HIGH LOAD !!"); }
  else                         display.print("BTN2:ON  BTN3:OFF");
}

void drawEnergy() {
  char buf[24];
  display.setCursor(0, 0);   display.print("== COST & METRICS ==");

  display.setCursor(0, 10);  display.print("Energy: "); display.print(currentEnergy, 3); display.print(" kWh");
  display.setCursor(0, 19);  display.print("Cost  : "); display.print(currentEnergy * unitPrice, 2); display.print(" THB");
  display.setCursor(0, 28);  display.print("PF: "); display.print(currentPF, 2);
                             display.print("  Hz: "); display.print(currentHz, 1);
  snprintf(buf, sizeof(buf), "Rate: %.2f THB/kWh", unitPrice);
  display.setCursor(0, 37);  display.print(buf);

  if (btn3.pressed && !btn3.holdFired && !btn3.swallow) {          // progress bar ตอนกดค้าง
    float frac = (float)(millis() - btn3.pressStart) / (float)ENERGY_RESET_HOLD_MS;
    drawBar(0, 50, 128, 9, frac);
  } else {
    display.setCursor(0, 56);
    if (msgText[0] != '\0') display.print(msgText);
    else                    display.print("Hold BTN3 2s: RESET");
  }
}

void drawSettingValue(const char* title, const char* value, const char* unit) {
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print(title);
  display.setTextSize(2);
  display.setCursor(0, 16); display.print(value);
  display.setTextSize(1);
  if (unit != NULL) { display.setCursor(0, 36); display.print(unit); }
  display.setCursor(0, 46); display.print("2:+  3:-  hold=fast");
  display.setCursor(0, 56); display.print("BTN1:NEXT  HOLD:HOME");
}

void drawSetMode() {
  display.setCursor(0, 0);  display.print("== SET RUN MODE ==");
  display.setCursor(0, 16); display.print(currentMode == MODE_CONT  ? "> [ CONTINUOUS ]" : "  [ CONTINUOUS ]");
  display.setCursor(0, 28); display.print(currentMode == MODE_TIMER ? "> [ TIMER ]"      : "  [ TIMER ]");
  display.setCursor(0, 46); display.print("BTN2/3: switch");
  display.setCursor(0, 56); display.print("BTN1:NEXT  HOLD:HOME");
}

void drawSetSound() {
  display.setCursor(0, 0);  display.print("== SET SOUND ==");
  display.setCursor(0, 14); display.print(soundMode == 2 ? "> [ ALL SOUNDS ]"  : "  [ ALL SOUNDS ]");
  display.setCursor(0, 25); display.print(soundMode == 1 ? "> [ ALERTS ONLY ]" : "  [ ALERTS ONLY ]");
  display.setCursor(0, 36); display.print(soundMode == 0 ? "> [ MUTE ]"        : "  [ MUTE ]");
  display.setCursor(0, 46); display.print("BTN2/3: change");
  display.setCursor(0, 56); display.print("BTN1:NEXT  HOLD:HOME");
}

void drawError() {
  char buf[24];
  display.setTextSize(2);
  display.setCursor(6, 0);
  switch (currentError) {
    case ERR_OVERLOAD: display.print("OVERLOAD!"); break;
    case ERR_UNDER_V:  display.print("UNDER-V!");  break;
    case ERR_OVER_V:   display.print("OVER-V!");   break;
    case ERR_SENSOR:   display.print("NO SENSOR"); break;
    default:           display.print("ERROR");     break;
  }
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("System Tripped.");

  display.setCursor(0, 30);
  if (dataValid) { snprintf(buf, sizeof(buf), "V:%.0f  I:%.1fA", currentV, currentI); display.print(buf); }
  else           { display.print("V:--  I:--"); }

  display.setCursor(0, 40);
  if (currentError == ERR_OVERLOAD) {
    snprintf(buf, sizeof(buf), "Limit: %.1f A", overloadLimitA); display.print(buf);
  } else if (currentError == ERR_SENSOR) {
    display.print(dataValid ? "Sensor OK. Press BTN1" : "Check PZEM wiring");
  } else if (recoveryLocked) {
    display.print("Retry limit! Manual");
  } else {
    snprintf(buf, sizeof(buf), "Normal: %.0f-%.0f V", vGuardMin, vGuardMax); display.print(buf);
  }

  display.setCursor(0, 56); display.print("BTN1:RESET  2/3:MUTE");
}

void drawRecovery() {
  char buf[24];
  display.setTextSize(2);
  display.setCursor(10, 0); display.print("RECOVERY");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("Voltage is Normal.");
  snprintf(buf, sizeof(buf), "Restart in: %d s", recoverySeconds);
  display.setCursor(0, 30); display.print(buf);
  drawBar(0, 40, 128, 8, 1.0f - (float)recoverySeconds / (float)RECOVERY_DELAY_S);
  display.setCursor(0, 56); display.print("BTN1: CANCEL");
}

// =====================================================================
//                           API สำหรับ IoT
// =====================================================================
BreakerStatus breakerGetStatus() {
  BreakerStatus s;
  s.v = currentV; s.i = currentI; s.p = currentP; s.kwh = currentEnergy; s.pf = currentPF; s.hz = currentHz;
  s.relayOn = relayState;
  s.sensorOk = dataValid;
  s.mode = (currentMode == MODE_TIMER) ? 1 : 0;
  s.error = currentError;
  s.faultActive = (currentPage == PAGE_ERROR || currentPage == PAGE_RECOVERY);
  s.overloadPending = (oclSince != 0);
  s.recoveryLocked = recoveryLocked;
  s.timerSecLeft = (currentMode == MODE_TIMER) ? activeTimerSeconds : 0;
  s.recoverySecLeft = (currentPage == PAGE_RECOVERY) ? recoverySeconds : 0;
  s.overloadLimitA = overloadLimitA;
  s.vMin = vGuardMin; s.vMax = vGuardMax;
  s.timerMinutes = timerMinutes;
  s.unitPrice = unitPrice;
  s.remoteAllowed = true;
  return s;
}

bool breakerSubmitCommand(const BreakerCommand &c) {
  if (!cmdQueue) return false;
  return xQueueSend(cmdQueue, &c, 0) == pdTRUE;
}

bool breakerPollResult(BreakerResult &r) {
  if (!resQueue) return false;
  return xQueueReceive(resQueue, &r, 0) == pdTRUE;
}

void breakerSetNetState(uint8_t state) {
  if (netState != state) {
    netState = state;
    requestRedraw();
  }
}

const char* breakerResultText(BreakerResultCode c) {
  switch (c) {
    case BRES_OK: return "OK";
    case BRES_BAD_VALUE: return "BAD_VALUE";
    case BRES_DENIED_REMOTE_DISABLED: return "DENIED_REMOTE";
    case BRES_DENIED_VOLTAGE: return "DENIED_VOLTAGE";
    case BRES_DENIED_SENSOR: return "DENIED_SENSOR";
    case BRES_DENIED_FAULT: return "DENIED_FAULT";
    case BRES_FAILED: return "FAILED";
    default: return "UNKNOWN";
  }
}

void processIotCommands() {
  if (!cmdQueue) return;
  BreakerCommand c;
  while (xQueueReceive(cmdQueue, &c, 0) == pdTRUE) {
    BreakerResult r;
    r.type = c.type; r.tag = c.tag; r.code = BRES_OK;

    if ((currentPage == PAGE_ERROR || currentPage == PAGE_RECOVERY) && c.type != BCMD_RESET_FAULT && c.type != BCMD_RELAY_OFF) {
      r.code = BRES_DENIED_FAULT;
      xQueueSend(resQueue, &r, 0);
      continue;
    }

    switch (c.type) {
      case BCMD_RELAY_ON:
        if (!dataValid) r.code = BRES_DENIED_SENSOR;
        else if (voltageBad(currentV)) r.code = BRES_DENIED_VOLTAGE;
        else requestRelayOn();
        break;
      case BCMD_RELAY_OFF:
        setRelay(false); activeTimerSeconds = 0; PLAY(PAT_RELAY_OFF, false, PRIO_NOTICE); requestRedraw();
        break;
      case BCMD_SET_OVERLOAD:
        if (c.a >= 0.1 && c.a <= OVERLOAD_MAX_A) { overloadLimitA = c.a; settingsDirty = true; requestRedraw(); } else r.code = BRES_BAD_VALUE;
        break;
      case BCMD_SET_VOLTAGE:
        if (c.a >= VMIN_FLOOR && c.b <= VMAX_CEIL && c.a < c.b) { vGuardMin = c.a; vGuardMax = c.b; settingsDirty = true; requestRedraw(); } else r.code = BRES_BAD_VALUE;
        break;
      case BCMD_SET_TIMER:
        if (c.a >= 1 && c.a <= TIMER_MAX_MIN) { timerMinutes = (int)c.a; settingsDirty = true; requestRedraw(); } else r.code = BRES_BAD_VALUE;
        break;
      case BCMD_SET_MODE:
        if (c.a == 0 || c.a == 1) { currentMode = (c.a == 1) ? MODE_TIMER : MODE_CONT; settingsDirty = true; requestRedraw(); } else r.code = BRES_BAD_VALUE;
        break;
      case BCMD_SET_PRICE:
        if (c.a >= 0.0 && c.a <= 100.0) { unitPrice = c.a; settingsDirty = true; requestRedraw(); } else r.code = BRES_BAD_VALUE;
        break;
      case BCMD_RESET_FAULT:
        if (currentPage == PAGE_ERROR || currentPage == PAGE_RECOVERY) resetFault();
        break;
      case BCMD_RESET_ENERGY:
        if (pzem.resetEnergy()) { currentEnergy = 0; requestRedraw(); } else r.code = BRES_FAILED;
        break;
    }
    xQueueSend(resQueue, &r, 0);
  }
}