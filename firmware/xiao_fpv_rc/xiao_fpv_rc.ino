/*
  XIAO FPV RC car — low latency firmware for Seeed Studio XIAO ESP32S3 Sense

  Video:   JPEG frames over a WebSocket with per-frame acknowledgement.
           The next frame is captured only after the browser has received the
           previous one, so frames never queue up and the picture shown is always
           the freshest the camera can give.
  Control: same WebSocket — steering, throttle with soft start, lights.
           Motor stops and steering centres if the browser goes quiet.
  Setup:   web page with settings is built into the firmware (no SD card needed),
           settings are kept in flash (NVS), firmware can be updated over WiFi.

  Arduino IDE: esp32 core 3.1.x, board XIAO_ESP32S3, PSRAM "OPI PSRAM",
  Partition Scheme "Default with spiffs (3MB APP/1.5MB SPIFFS)" (has OTA slots).

  License: GNU AGPL-3.0 — see LICENSE in the repository.
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <unistd.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "camera_pins.h"
#include "index_html.h"

#define FW_VERSION "1.0.0"
#define HOSTNAME "fpvcar"

// LEDC channels: camera XCLK uses timer 0 / channel 0, keep away from it
#define SERVO_CHANNEL 2   // timer 1
#define MOTOR_A_CHANNEL 4 // timer 2
#define MOTOR_B_CHANNEL 5 // timer 2 (same frequency as motor A)
#define SERVO_RES 14
#define MOTOR_RES 10

/* ======================= settings ======================= */

// WiFi
static char apSsid[33] = "FPV-Car";
static char apPass[65] = "12345678";
static uint8_t apChannel = 6;
static char staSsid[33] = "";
static char staPass[65] = "";
static uint8_t txPower = 0; // 0 max, 1 15dBm, 2 11dBm, 3 8.5dBm
// pins (XIAO D0..D3)
static int8_t servoPin = 1;
static int8_t motorAPin = 2;
static int8_t motorBPin = 3;
static int8_t lightsPin = 4;
// steering servo angles for stick full left / centre / full right
static uint8_t steerLeft = 45;
static uint8_t steerCenter = 90;
static uint8_t steerRight = 135;
// motor
static bool motorInvert = false;
static uint8_t minDuty = 15;    // % at which motor starts to move
static uint8_t maxDuty = 100;   // % at full stick
static uint16_t rampMs = 300;   // soft start 0 -> 100 %
static uint16_t pwmFreq = 5000; // Hz
static uint16_t failsafeMs = 1000;
// camera
static uint8_t frameSize = FRAMESIZE_HVGA;
static uint8_t jpegQuality = 15; // 8 (best) .. 63 (smallest)
static uint8_t maxFps = 30;
static bool camFlip = false;   // rotate 180
static bool camMirror = false; // mirror left / right

enum : uint8_t { T_U8, T_U16, T_I8, T_BOOL, T_STR };
struct Setting {
  const char* key;
  uint8_t type;
  void* ptr;
  int32_t minV;
  int32_t maxV; // for T_STR: buffer size
  bool reboot;  // change takes effect after restart
};

static const Setting settings[] = {
  {"apSsid", T_STR, apSsid, 0, sizeof(apSsid), true},
  {"apPass", T_STR, apPass, 0, sizeof(apPass), true},
  {"apChannel", T_U8, &apChannel, 1, 13, true},
  {"staSsid", T_STR, staSsid, 0, sizeof(staSsid), true},
  {"staPass", T_STR, staPass, 0, sizeof(staPass), true},
  {"txPower", T_U8, &txPower, 0, 3, false},
  {"servoPin", T_I8, &servoPin, -1, 48, true},
  {"motorAPin", T_I8, &motorAPin, -1, 48, true},
  {"motorBPin", T_I8, &motorBPin, -1, 48, true},
  {"lightsPin", T_I8, &lightsPin, -1, 48, true},
  {"steerLeft", T_U8, &steerLeft, 0, 180, false},
  {"steerCenter", T_U8, &steerCenter, 0, 180, false},
  {"steerRight", T_U8, &steerRight, 0, 180, false},
  {"motorInvert", T_BOOL, &motorInvert, 0, 1, false},
  {"minDuty", T_U8, &minDuty, 0, 100, false},
  {"maxDuty", T_U8, &maxDuty, 0, 100, false},
  {"rampMs", T_U16, &rampMs, 0, 3000, false},
  {"pwmFreq", T_U16, &pwmFreq, 100, 20000, false},
  {"failsafeMs", T_U16, &failsafeMs, 200, 5000, false},
  {"frameSize", T_U8, &frameSize, FRAMESIZE_QQVGA, FRAMESIZE_SVGA, false},
  {"quality", T_U8, &jpegQuality, 8, 63, false},
  {"maxFps", T_U8, &maxFps, 1, 60, false},
  {"camFlip", T_BOOL, &camFlip, 0, 1, false},
  {"camMirror", T_BOOL, &camMirror, 0, 1, false},
};
static const int NUM_SETTINGS = sizeof(settings) / sizeof(settings[0]);

static Preferences prefs;

/* ======================= runtime state ======================= */

static httpd_handle_t server = NULL;
static TaskHandle_t videoHandle = NULL;
static volatile int wsFd = -1;              // socket of the active driver
static volatile bool ackPending = false;    // waiting for browser to confirm last frame
static volatile uint32_t lastRxMs = 0;      // last message from driver
static volatile bool otaRunning = false;
static bool camOk = false;
static char camModel[12] = "none";
static bool staMode = false;

static volatile int steerCmd = 0;           // -100 .. 100 from stick
static volatile int throttleCmd = 0;        // -100 .. 100 from stick
static volatile int motorTarget = 0;        // signed duty %
static volatile int motorNow = 0;           // signed duty % applied
static bool lightsOn = false;

static volatile uint32_t frameCount = 0;
static volatile uint16_t fpsNow = 0;
static volatile uint32_t lastFrameBytes = 0;

/* ======================= helpers ======================= */

static int clampInt(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static const Setting* findSetting(const char* key) {
  for (int i = 0; i < NUM_SETTINGS; i++) {
    if (!strcmp(settings[i].key, key)) return &settings[i];
  }
  return NULL;
}

static void urlDecode(char* s) {
  // in place %XX and + decoding
  char* o = s;
  while (*s) {
    if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
      char hex[3] = {s[1], s[2], 0};
      *o++ = (char)strtol(hex, NULL, 16);
      s += 3;
    } else if (*s == '+') {
      *o++ = ' ';
      s++;
    } else {
      *o++ = *s++;
    }
  }
  *o = 0;
}

static void jsonEscape(String& out, const char* s) {
  for (; *s; s++) {
    char c = *s;
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); out += b; }
    else out += c;
  }
}

/* ======================= servo / motor / lights ======================= */

static uint32_t servoDuty(int angle) {
  // 0..180 deg -> 544..2400 us pulse at 50 Hz
  angle = clampInt(angle, 0, 180);
  uint32_t us = 544 + (uint32_t)angle * (2400 - 544) / 180;
  return us * ((1UL << SERVO_RES) - 1) / 20000;
}

static void writeServo(int angle) {
  if (servoPin >= 0) ledcWrite(servoPin, servoDuty(angle));
}

static int steerToAngle(int s) {
  // stick -100..100 -> servo angle through left / centre / right end points
  s = clampInt(s, -100, 100);
  if (s < 0) return steerCenter + (s * ((int)steerCenter - (int)steerLeft)) / 100;
  return steerCenter + (s * ((int)steerRight - (int)steerCenter)) / 100;
}

static void applyMotor(int duty) {
  // duty: signed %, + forward, - reverse, 0 coast
  if (motorAPin < 0 || motorBPin < 0) return;
  if (motorInvert) duty = -duty;
  uint32_t d = (uint32_t)abs(clampInt(duty, -100, 100)) * ((1UL << MOTOR_RES) - 1) / 100;
  if (duty > 0) { ledcWrite(motorBPin, 0); ledcWrite(motorAPin, d); }
  else if (duty < 0) { ledcWrite(motorAPin, 0); ledcWrite(motorBPin, d); }
  else { ledcWrite(motorAPin, 0); ledcWrite(motorBPin, 0); }
}

static int throttleToDuty(int t) {
  // stick -100..100 -> signed duty between minDuty and maxDuty
  t = clampInt(t, -100, 100);
  if (t == 0) return 0;
  int lo = min((int)minDuty, (int)maxDuty);
  int mag = lo + (abs(t) * ((int)maxDuty - lo)) / 100;
  return t > 0 ? mag : -mag;
}

static void setThrottle(int t) {
  throttleCmd = t;
  int duty = throttleToDuty(t);
  motorTarget = duty;
  if (duty == 0 || rampMs == 0) {
    // stop and no-ramp requests are applied at once
    applyMotor(duty);
    motorNow = duty;
  }
}

static void setSteer(int s) {
  steerCmd = s;
  writeServo(steerToAngle(s));
}

static void setLights(bool on) {
  lightsOn = on;
  if (lightsPin >= 0) digitalWrite(lightsPin, on ? HIGH : LOW);
}

static void stopCar() {
  setThrottle(0);
  setSteer(0);
}

static void setupOutputs() {
  if (servoPin >= 0) {
    ledcAttachChannel(servoPin, 50, SERVO_RES, SERVO_CHANNEL);
    writeServo(steerCenter);
  }
  if (motorAPin >= 0 && motorBPin >= 0) {
    ledcAttachChannel(motorAPin, pwmFreq, MOTOR_RES, MOTOR_A_CHANNEL);
    ledcAttachChannel(motorBPin, pwmFreq, MOTOR_RES, MOTOR_B_CHANNEL);
    applyMotor(0);
  }
  if (lightsPin >= 0) {
    pinMode(lightsPin, OUTPUT);
    digitalWrite(lightsPin, LOW);
  }
}

static int nextRampVal(int now, int tgt) {
  // accelerate gradually; slowing down, stopping and direction change: stop first / immediately
  if (tgt == now) return now;
  if (now != 0 && (tgt == 0 || (now > 0) != (tgt > 0))) return 0;
  if (abs(tgt) <= abs(now)) return tgt;
  int step = rampMs > 0 ? max(1, 100 * 10 / (int)rampMs) : 100;
  int mag = (now == 0) ? max((int)minDuty, step) : abs(now) + step;
  mag = min(mag, abs(tgt));
  return tgt > 0 ? mag : -mag;
}

static void controlTask(void* arg) {
  // every 10 ms: motor soft start and failsafe
  uint32_t fpsMark = millis();
  uint32_t fpsFrames = frameCount;
  while (true) {
    int now = motorNow, tgt = motorTarget;
    if (now != tgt) {
      int next = nextRampVal(now, tgt);
      applyMotor(next);
      motorNow = next;
    }
    // failsafe: driver silent or gone
    bool active = (throttleCmd != 0 || steerCmd != 0);
    if (active && (wsFd < 0 || millis() - lastRxMs > failsafeMs)) stopCar();
    // frames per second actually sent
    if (millis() - fpsMark >= 1000) {
      fpsNow = frameCount - fpsFrames;
      fpsFrames = frameCount;
      fpsMark = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ======================= camera ======================= */

static void applyCameraSettings() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  s->set_framesize(s, (framesize_t)frameSize);
  s->set_quality(s, jpegQuality);
  s->set_vflip(s, camFlip);
  s->set_hmirror(s, camFlip ^ camMirror);
}

static bool startCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_SVGA; // largest size allowed, buffers sized for it
  c.jpeg_quality = jpegQuality;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  switch (s->id.PID) {
    case OV2640_PID: strcpy(camModel, "OV2640"); break;
    case OV3660_PID: strcpy(camModel, "OV3660"); break;
    case OV5640_PID: strcpy(camModel, "OV5640"); break;
    default: snprintf(camModel, sizeof(camModel), "PID %x", s->id.PID); break;
  }
  applyCameraSettings();
  Serial.printf("Camera %s ready\n", camModel);
  return true;
}

static camera_fb_t* freshFrame() {
  // the driver keeps 2 buffers; if the first one is older than ~1 frame,
  // drop it and take the next so the browser always gets the newest picture
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return NULL;
  int64_t ts = (int64_t)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec;
  if (esp_timer_get_time() - ts > 40000) {
    esp_camera_fb_return(fb);
    fb = esp_camera_fb_get();
  }
  return fb;
}

/* ======================= video task ======================= */

static void videoTask(void* arg) {
  uint32_t lastSendMs = 0;
  while (true) {
    int fd = wsFd;
    if (fd < 0 || !camOk || otaRunning) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
      continue;
    }
    uint32_t now = millis();
    // wait for the browser to confirm the previous frame (timeout covers a lost ack)
    if (ackPending && now - lastSendMs < 500) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
      continue;
    }
    // frame rate limit
    uint32_t minGap = 1000 / (maxFps ? maxFps : 1);
    if (now - lastSendMs < minGap) {
      vTaskDelay(pdMS_TO_TICKS(minGap - (now - lastSendMs)));
      continue;
    }
    camera_fb_t* fb = freshFrame();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    httpd_ws_frame_t pkt = {};
    pkt.final = true;
    pkt.type = HTTPD_WS_TYPE_BINARY;
    pkt.payload = fb->buf;
    pkt.len = fb->len;
    ackPending = true;
    lastSendMs = millis();
    esp_err_t r = httpd_ws_send_frame_async(server, fd, &pkt);
    lastFrameBytes = fb->len;
    esp_camera_fb_return(fb);
    if (r != ESP_OK) {
      if (wsFd == fd) wsFd = -1;
      ackPending = false;
    } else {
      frameCount++;
    }
  }
}

/* ======================= web server ======================= */

static void handleWsText(int fd, char* msg) {
  if (fd != wsFd) return; // only the active driver controls the car
  lastRxMs = millis();
  switch (msg[0]) {
    case 'A': // frame received
      ackPending = false;
      if (videoHandle) xTaskNotifyGive(videoHandle);
      break;
    case 'H': // keepalive
      break;
    case 'S': // steering -100..100
      setSteer(clampInt(atoi(msg + 1), -100, 100));
      break;
    case 'T': // throttle -100..100
      setThrottle(clampInt(atoi(msg + 1), -100, 100));
      break;
    case 'L': // lights
      setLights(msg[1] == '1');
      break;
    default:
      break;
  }
}

static esp_err_t wsHandler(httpd_req_t* req) {
  int fd = httpd_req_to_sockfd(req);
  if (req->method == HTTP_GET) {
    // new connection: this browser becomes the driver
    wsFd = fd;
    ackPending = false;
    lastRxMs = millis();
    if (videoHandle) xTaskNotifyGive(videoHandle);
    Serial.printf("Driver connected (socket %d)\n", fd);
    return ESP_OK;
  }
  httpd_ws_frame_t pkt = {};
  esp_err_t r = httpd_ws_recv_frame(req, &pkt, 0); // get length
  if (r != ESP_OK) return r;
  if (pkt.len == 0) return ESP_OK;
  if (pkt.len > 128) return ESP_FAIL; // unexpected, drop connection
  uint8_t buf[130];
  pkt.payload = buf;
  r = httpd_ws_recv_frame(req, &pkt, pkt.len);
  if (r != ESP_OK) return r;
  buf[pkt.len] = 0;
  if (pkt.type == HTTPD_WS_TYPE_TEXT) handleWsText(fd, (char*)buf);
  return ESP_OK;
}

static void onSocketClose(httpd_handle_t hd, int fd) {
  if (fd == wsFd) {
    wsFd = -1;
    ackPending = false;
    Serial.println("Driver disconnected");
  }
  close(fd);
}

static esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static String settingsJson() {
  String j = "{";
  for (int i = 0; i < NUM_SETTINGS; i++) {
    const Setting& s = settings[i];
    j += '"'; j += s.key; j += "\":";
    switch (s.type) {
      case T_U8: j += (int)*(uint8_t*)s.ptr; break;
      case T_U16: j += (int)*(uint16_t*)s.ptr; break;
      case T_I8: j += (int)*(int8_t*)s.ptr; break;
      case T_BOOL: j += (int)(*(bool*)s.ptr ? 1 : 0); break;
      case T_STR: j += '"'; jsonEscape(j, (char*)s.ptr); j += '"'; break;
    }
    j += ',';
  }
  j += "\"version\":\"" FW_VERSION "\",";
  j += "\"camera\":\""; j += camModel; j += "\",";
  j += "\"mode\":\""; j += staMode ? "sta" : "ap"; j += "\",";
  j += "\"ip\":\""; j += staMode ? WiFi.localIP().toString() : WiFi.softAPIP().toString(); j += "\",";
  j += "\"rssi\":"; j += (int)(staMode ? WiFi.RSSI() : 0); j += ',';
  j += "\"fps\":"; j += (int)fpsNow; j += ',';
  j += "\"frameBytes\":"; j += (unsigned long)lastFrameBytes; j += ',';
  j += "\"lights\":"; j += (int)(lightsOn ? 1 : 0);
  j += '}';
  return j;
}

static esp_err_t configHandler(httpd_req_t* req) {
  String j = settingsJson();
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, j.c_str(), j.length());
}

static void applyLive(const Setting* s) {
  const char* k = s->key;
  if (!strcmp(k, "steerLeft")) writeServo(steerLeft);        // servo shows the end point being adjusted
  else if (!strcmp(k, "steerCenter")) writeServo(steerCenter);
  else if (!strcmp(k, "steerRight")) writeServo(steerRight);
  else if (!strcmp(k, "pwmFreq")) {
    if (motorAPin >= 0) ledcChangeFrequency(motorAPin, pwmFreq, MOTOR_RES);
    if (motorBPin >= 0) ledcChangeFrequency(motorBPin, pwmFreq, MOTOR_RES);
  }
  else if (!strcmp(k, "txPower")) {
    const wifi_power_t lv[] = {WIFI_POWER_19_5dBm, WIFI_POWER_15dBm, WIFI_POWER_11dBm, WIFI_POWER_8_5dBm};
    WiFi.setTxPower(lv[txPower]);
  }
  else if (!strcmp(k, "frameSize") || !strcmp(k, "quality") || !strcmp(k, "camFlip") || !strcmp(k, "camMirror")) {
    if (camOk) applyCameraSettings();
  }
  else if (!strcmp(k, "motorInvert")) setThrottle(0);
}

static bool setValue(const Setting* s, const char* val) {
  if (s->type == T_STR) {
    if (strlen(val) >= (size_t)s->maxV) return false;
    if (!strcmp(s->key, "apPass") && strlen(val) > 0 && strlen(val) < 8) return false; // WPA2 needs 8+
    strcpy((char*)s->ptr, val);
    return true;
  }
  int v = atoi(val);
  if (v < s->minV || v > s->maxV) return false;
  switch (s->type) {
    case T_U8: *(uint8_t*)s->ptr = v; break;
    case T_U16: *(uint16_t*)s->ptr = v; break;
    case T_I8: *(int8_t*)s->ptr = v; break;
    case T_BOOL: *(bool*)s->ptr = v != 0; break;
  }
  return true;
}

static esp_err_t setHandler(httpd_req_t* req) {
  // /set?k=<key>&v=<value>  — applied at once, saved with /save
  char query[200], key[24], val[100];
  bool ok = false, reboot = false;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
      httpd_query_key_value(query, "k", key, sizeof(key)) == ESP_OK &&
      httpd_query_key_value(query, "v", val, sizeof(val)) == ESP_OK) {
    urlDecode(val);
    const Setting* s = findSetting(key);
    if (s && setValue(s, val)) {
      ok = true;
      reboot = s->reboot;
      if (!reboot) applyLive(s);
    }
  }
  char resp[48];
  snprintf(resp, sizeof(resp), "{\"ok\":%s,\"reboot\":%s}", ok ? "true" : "false", reboot ? "true" : "false");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, resp);
}

static void saveSettings() {
  prefs.begin("fpvcar", false);
  for (int i = 0; i < NUM_SETTINGS; i++) {
    const Setting& s = settings[i];
    switch (s.type) {
      case T_U8: prefs.putUChar(s.key, *(uint8_t*)s.ptr); break;
      case T_U16: prefs.putUShort(s.key, *(uint16_t*)s.ptr); break;
      case T_I8: prefs.putChar(s.key, *(int8_t*)s.ptr); break;
      case T_BOOL: prefs.putBool(s.key, *(bool*)s.ptr); break;
      case T_STR: prefs.putString(s.key, (char*)s.ptr); break;
    }
  }
  prefs.end();
}

static void loadSettings() {
  prefs.begin("fpvcar", true);
  for (int i = 0; i < NUM_SETTINGS; i++) {
    const Setting& s = settings[i];
    if (!prefs.isKey(s.key)) continue; // keep default
    switch (s.type) {
      case T_U8: *(uint8_t*)s.ptr = prefs.getUChar(s.key); break;
      case T_U16: *(uint16_t*)s.ptr = prefs.getUShort(s.key); break;
      case T_I8: *(int8_t*)s.ptr = prefs.getChar(s.key); break;
      case T_BOOL: *(bool*)s.ptr = prefs.getBool(s.key); break;
      case T_STR: prefs.getString(s.key, (char*)s.ptr, s.maxV); break;
    }
  }
  prefs.end();
}

static esp_err_t saveHandler(httpd_req_t* req) {
  saveSettings();
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t rebootHandler(httpd_req_t* req) {
  httpd_resp_sendstr(req, "{\"ok\":true}");
  stopCar();
  delay(300);
  ESP.restart();
  return ESP_OK;
}

static esp_err_t resetHandler(httpd_req_t* req) {
  // factory defaults
  prefs.begin("fpvcar", false);
  prefs.clear();
  prefs.end();
  httpd_resp_sendstr(req, "{\"ok\":true}");
  stopCar();
  delay(300);
  ESP.restart();
  return ESP_OK;
}

static esp_err_t updateHandler(httpd_req_t* req) {
  // firmware update: raw .bin in POST body
  static uint8_t buf[4096];
  size_t remaining = req->content_len;
  otaRunning = true;
  stopCar();
  if (!Update.begin(remaining)) {
    otaRunning = false;
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, Update.errorString());
  }
  while (remaining > 0) {
    int r = httpd_req_recv(req, (char*)buf, min(remaining, sizeof(buf)));
    if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (r <= 0 || Update.write(buf, r) != (size_t)r) {
      Update.abort();
      otaRunning = false;
      httpd_resp_set_status(req, "500 Internal Server Error");
      return httpd_resp_sendstr(req, "upload failed");
    }
    remaining -= r;
  }
  if (!Update.end(true)) {
    otaRunning = false;
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_sendstr(req, Update.errorString());
  }
  httpd_resp_sendstr(req, "OK");
  Serial.println("Firmware updated, restarting");
  delay(500);
  ESP.restart();
  return ESP_OK;
}

static void startServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.max_uri_handlers = 10;
  cfg.stack_size = 8192;
  cfg.send_wait_timeout = 2;
  cfg.recv_wait_timeout = 10;
  cfg.lru_purge_enable = true;
  cfg.close_fn = onSocketClose;
  cfg.core_id = 0;
  if (httpd_start(&server, &cfg) != ESP_OK) {
    Serial.println("HTTP server start failed");
    return;
  }
  httpd_uri_t uris[] = {
    {.uri = "/", .method = HTTP_GET, .handler = indexHandler, .user_ctx = NULL},
    {.uri = "/ws", .method = HTTP_GET, .handler = wsHandler, .user_ctx = NULL, .is_websocket = true},
    {.uri = "/config", .method = HTTP_GET, .handler = configHandler, .user_ctx = NULL},
    {.uri = "/set", .method = HTTP_GET, .handler = setHandler, .user_ctx = NULL},
    {.uri = "/save", .method = HTTP_GET, .handler = saveHandler, .user_ctx = NULL},
    {.uri = "/reboot", .method = HTTP_GET, .handler = rebootHandler, .user_ctx = NULL},
    {.uri = "/reset", .method = HTTP_GET, .handler = resetHandler, .user_ctx = NULL},
    {.uri = "/update", .method = HTTP_POST, .handler = updateHandler, .user_ctx = NULL},
  };
  for (auto& u : uris) httpd_register_uri_handler(server, &u);
}

/* ======================= WiFi ======================= */

static void startWifi() {
  WiFi.persistent(false);
  staMode = false;
  if (strlen(staSsid)) {
    // home network if configured and reachable
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(HOSTNAME);
    WiFi.begin(staSsid, staPass);
    Serial.printf("Connecting to %s", staSsid);
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    staMode = WiFi.status() == WL_CONNECTED;
  }
  if (staMode) {
    Serial.printf("WiFi connected, open http://%s or http://" HOSTNAME ".local\n", WiFi.localIP().toString().c_str());
  } else {
    // car's own access point: phone connects directly, lowest latency
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSsid, strlen(apPass) >= 8 ? apPass : NULL, apChannel, 0, 2);
    Serial.printf("Access point \"%s\", open http://%s\n", apSsid, WiFi.softAPIP().toString().c_str());
  }
  // keep radio awake: no power save delays for video and control
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  const wifi_power_t lv[] = {WIFI_POWER_19_5dBm, WIFI_POWER_15dBm, WIFI_POWER_11dBm, WIFI_POWER_8_5dBm};
  WiFi.setTxPower(lv[txPower]);
  if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);
}

/* ======================= main ======================= */

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nXIAO FPV RC car " FW_VERSION);
  loadSettings();
  setupOutputs();                 // servo centred, motor off first
  camOk = startCamera();
  startWifi();
  startServer();
  xTaskCreatePinnedToCore(controlTask, "control", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(videoTask, "video", 6144, NULL, 4, &videoHandle, 1);
}

void loop() {
  // reconnect to home network if it dropped
  if (staMode && WiFi.status() != WL_CONNECTED) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 5000) {
      lastTry = millis();
      WiFi.reconnect();
    }
  }
  delay(100);
}
