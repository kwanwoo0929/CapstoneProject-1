#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <NimBLEDevice.h>
#include <WebServer.h>
#include "driver/i2s.h"

// ==================== 버튼 & 배터리 핀 ====================
#define BUTTON_PIN    1      // 사진 촬영 버튼
#define BUTTON2_PIN   6      // "누르는 동안" 음성 스트림 버튼 (필요 시 핀 변경)
#define BATTERY_PIN   4

// ==================== 카메라 핀맵 (XIAO ESP32-S3 Sense) ====================
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

// ==================== Wi-Fi (초기: AP 모드) ====================
#define AP_SSID       "XIAO_S3_CAM_AP"
#define AP_PASSWORD   "esp32s3cam123"

// 스마트폰 HTTP 서버로 JPEG/오디오 보낼 때
#define PHONE_IP      "192.168.4.2"   // 필요 시 폰에서 고정 IP로 지정
#define PHONE_PORT    8080
#define PHONE_PATH    "/upload"
#define PHONE_AUDIO_PATH "/audio_stream"   // 실시간 오디오 스트림 수신 엔드포인트

// ==================== HTTP 서버 ====================
WebServer server(80);

// ==================== BLE UUID ====================
#define SERVICE_UUID         "12345678-1234-1234-1234-1234567890ab"
#define CHAR_BUTTON_UUID     "12345678-1234-1234-1234-1234567890b1" // Notify(JSON)
#define CHAR_BATTERY_UUID    "12345678-1234-1234-1234-1234567890b2" // Notify(퍼센트만)

volatile bool g_btnFlag = false;
volatile uint32_t g_lastMsISR = 0;
NimBLECharacteristic* chButton   = nullptr;
NimBLECharacteristic* chBattery  = nullptr;

bool g_ready = false;

// ==================== I2S (스피커 출력) ====================
#define I2S_BCK_IO   7   // BCLK
#define I2S_LRCK_IO  8   // LRCK
#define I2S_DATA_IO  9   // DIN

void initI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = false
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_IO,
    .ws_io_num = I2S_LRCK_IO,
    .data_out_num = I2S_DATA_IO,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// ==================== I2S (마이크 입력) ====================
#define MIC_USE_PDM      1        
#define MIC_SAMPLE_RATE  16000
#define MIC_BITS         I2S_BITS_PER_SAMPLE_16BIT
#define MIC_CHANNELS_MONO 1
#define MIC_WS_IO      3   // LRCK/WS (I2S)
#define MIC_DATA_IO    2   // DOUT  (I2S/PDM)

bool g_micReady = false;

bool initI2S_MicRX() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX
#if MIC_USE_PDM
    | I2S_MODE_PDM
#endif
  );
  cfg.sample_rate = MIC_SAMPLE_RATE;
  cfg.bits_per_sample = MIC_BITS;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 1024;
  cfg.use_apll = false;

  i2s_pin_config_t pins = {};
#if MIC_USE_PDM
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = I2S_PIN_NO_CHANGE; 
  pins.ws_io_num  = MIC_WS_IO;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = MIC_DATA_IO;
#else
  pins.mck_io_num   = I2S_PIN_NO_CHANGE;
  pins.bck_io_num   = MIC_BCLK_IO;
  pins.ws_io_num    = MIC_WS_IO;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = MIC_DATA_IO;
#endif

  if (i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL) != ESP_OK) {
    Serial.println("[MIC] i2s_driver_install failed");
    return false;
  }
  if (i2s_set_pin(I2S_NUM_1, &pins) != ESP_OK) {
    Serial.println("[MIC] i2s_set_pin failed");
    return false;
  }
  if (i2s_set_clk(I2S_NUM_1, MIC_SAMPLE_RATE, (i2s_bits_per_sample_t)MIC_BITS,
                  MIC_CHANNELS_MONO == 1 ? I2S_CHANNEL_MONO : I2S_CHANNEL_STEREO) != ESP_OK) {
    Serial.println("[MIC] i2s_set_clk failed");
    return false;
  }
  g_micReady = true;
  return true;
}

// ==================== 버튼 ISR ====================
void IRAM_ATTR onButtonISR() {
  uint32_t now = millis();
  if (now - g_lastMsISR > 30) {
    g_btnFlag = true;
    g_lastMsISR = now;
  }
}

// 버튼2 (길게 누르는 동안 스트리밍) — 폴링 방식으로 처리

// ==================== 배터리 퍼센트 ====================
int readBatteryPercent() {
  int raw = analogRead(BATTERY_PIN);
  float v = (raw / 4095.0f) * 3.3f * 2.0f;              // 분압 가정
  int pct = map((int)(v * 1000), 3300, 4200, 0, 100);   // 3.30V~4.20V
  return constrain(pct, 0, 100);
}

// ==================== 카메라 초기화 ====================
bool init_camera() {
  camera_config_t c{};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;

  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;

  c.pin_xclk  = XCLK_GPIO_NUM;
  c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href  = HREF_GPIO_NUM;

  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;

  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = 24000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_SVGA;    // 800x600
  c.jpeg_quality = 12;
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("[ERR] esp_camera_init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

// ==================== HTTP: JPEG 업로드 (ESP32 -> 스마트폰) ====================
bool post_jpeg_to_phone(const uint8_t* buf, size_t len) {
  String url = String("http://") + PHONE_IP + ":" + String(PHONE_PORT) + PHONE_PATH;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    HTTPClient http;
    http.setConnectTimeout(8000);
    http.setTimeout(15000);
    if (!http.begin(url)) {
      Serial.println("[ERR] HTTP begin failed");
      return false;
    }
    http.addHeader("Content-Type", "image/jpeg");
    http.addHeader("Content-Length", String(len));
    int code = http.POST(buf, len);
    http.end();
    if (code == 200) return true;
    Serial.printf("[WARN] JPEG upload attempt %d failed, code=%d\n", attempt, code);
    delay(300);
  }
  return false;
}

// ==================== 촬영 & 전송 ====================
void capture_and_send() {
  camera_fb_t* fb = nullptr;
  for (int i = 0; i < 3 && fb == nullptr; ++i) fb = esp_camera_fb_get();
  if (!fb) { Serial.println("[ERR] capture failed"); return; }
  bool ok = post_jpeg_to_phone(fb->buf, fb->len);
  Serial.println(ok ? "[OK] Upload done" : "[ERR] Upload failed");
  esp_camera_fb_return(fb);
}

// ==================== HTTP: /audio 핸들러 (LLM 음성 재생) ====================
void handleAudioUpload() {
  WiFiClient client = server.client();
  client.setTimeout(5);
  uint8_t hdr[44]; size_t got = 0;
  while (got < 44) {
    int n = client.readBytes(hdr + got, 44 - got);
    if (n <= 0) { server.send(400, "text/plain", "Bad WAV header"); return; }
    got += n;
  }
  auto rd16 = [&](int o)->uint16_t { return hdr[o] | (hdr[o+1] << 8); };
  auto rd32 = [&](int o)->uint32_t {
    return (uint32_t)hdr[o] | ((uint32_t)hdr[o+1] << 8) |
           ((uint32_t)hdr[o+2] << 16) | ((uint32_t)hdr[o+3] << 24);
  };
  if (!(hdr[0]=='R'&&hdr[1]=='I'&&hdr[2]=='F'&&hdr[3]=='F'&&hdr[8]=='W'&&hdr[9]=='A'&&hdr[10]=='V'&&hdr[11]=='E')) {
    server.send(415, "text/plain", "Not a RIFF/WAVE"); return;
  }
  uint16_t audioFormat   = rd16(20);
  uint16_t numChannels   = rd16(22);
  uint32_t sampleRate    = rd32(24);
  uint16_t bitsPerSample = rd16(34);
  if (audioFormat != 1 || bitsPerSample != 16 || (numChannels != 1 && numChannels != 2)) {
    server.send(415, "text/plain", "Unsupported WAV (need PCM16, 1/2ch)"); return;
  }
  i2s_set_clk(I2S_NUM_0, sampleRate, I2S_BITS_PER_SAMPLE_16BIT,
              (i2s_channel_t)(numChannels==1 ? I2S_CHANNEL_MONO : I2S_CHANNEL_STEREO));
  uint8_t buf[1024];
  while (client.connected()) {
    int len = client.readBytes(buf, sizeof(buf));
    if (len <= 0) break;
    size_t written;
    i2s_write(I2S_NUM_0, buf, len, &written, portMAX_DELAY);
  }
  server.send(200, "text/plain", "Audio Upload OK");
}

// ==================== HTTP: /status, /snapshot, /upload ====================
void handleStatus() {
  char buf[128];
  snprintf(buf, sizeof(buf),
           "{\"battery\":%d,\"temp\":%.1f,\"ready\":%s}",
           readBatteryPercent(), 36.2, g_ready ? "true" : "false");
  server.send(200, "application/json", buf);
}

void handleSnapshot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(500, "text/plain", "capture failed"); return; }
  server.send_P(200, "image/jpeg", fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleUploadFinish() { server.send(200, "application/json", "{\"ok\":true}"); }
void handleUploadData() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    Serial.printf("[UPLOAD] start: %s\n", up.filename.c_str());
  } else if (up.status == UPLOAD_FILE_WRITE) {
    // up.buf / up.currentSize 활용 가능
  } else if (up.status == UPLOAD_FILE_END) {
    Serial.printf("[UPLOAD] done: %u bytes\n", up.totalSize);
  }
}

// ==================== READY 알림 공통 함수 ====================
void bleNotifyReadyIP(const IPAddress& ip, uint16_t port) {
  if (!chButton) return;
  char j[128];
  snprintf(j, sizeof(j),
           "{\"evt\":\"READY\",\"ip\":\"%s\",\"port\":%u}",
           ip.toString().c_str(), port);
  chButton->setValue((uint8_t*)j, strlen(j));
  chButton->notify();
}

// ==================== 오디오 스트리밍 (ESP32 → 스마트폰, chunked) ====================
WiFiClient g_audioClient;
bool g_streaming = false;
uint32_t g_lastChunkMs = 0;

bool startAudioStreamToPhone() {
  if (!g_micReady) {
    Serial.println("[AUDIO] Mic not ready");
    return false;
  }
  if (g_streaming) return true;

  if (!g_audioClient.connect(PHONE_IP, PHONE_PORT)) {
    Serial.println("[AUDIO] connect() failed");
    return false;
  }
  // HTTP chunked POST
  String req;
  req += "POST " + String(PHONE_AUDIO_PATH) + " HTTP/1.1\r\n";
  req += "Host: " + String(PHONE_IP) + ":" + String(PHONE_PORT) + "\r\n";
  req += "Content-Type: application/octet-stream\r\n";
  req += "Transfer-Encoding: chunked\r\n";
  req += "X-Audio-Format: PCM16LE; rate=16000; channels=1\r\n";
  req += "Connection: keep-alive\r\n\r\n";
  g_audioClient.print(req);

  g_streaming = true;
  g_lastChunkMs = millis();
  Serial.println("[AUDIO] Streaming started");
  return true;
}

void stopAudioStreamToPhone() {
  if (!g_streaming) return;

  // 종료 chunk
  g_audioClient.print("0\r\n\r\n");
  g_audioClient.stop();
  g_streaming = false;
  Serial.println("[AUDIO] Streaming stopped");
}

// 일정 크기 읽어서 chunk 로 내보내기
void pumpMicOnce() {
  if (!g_streaming) return;
  static uint8_t buf[1024];
  size_t bytesRead = 0;
  // I2S RX 한 번 읽기 (논블록 느낌으로 짧게)
  i2s_read(I2S_NUM_1, buf, sizeof(buf), &bytesRead, 10 /*ticks*/);
  if (bytesRead == 0) return;

  // chunk 프레임: <hex len>\r\n<data>\r\n
  char hdr[16];
  sprintf(hdr, "%X\r\n", (unsigned int)bytesRead);
  g_audioClient.print(hdr);
  g_audioClient.write(buf, bytesRead);
  g_audioClient.print("\r\n");

  // 주기적으로 서버 응답 체크
  if (millis() - g_lastChunkMs > 5000) {
    g_lastChunkMs = millis();
    if (!g_audioClient.connected()) {
      Serial.println("[AUDIO] server disconnected, stopping");
      stopAudioStreamToPhone();
    }
  }
}

// ==================== setup ====================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);  // 스트리밍 버튼
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonISR, FALLING);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  if (!init_camera()) {
    Serial.println("[ERR] Camera init failed");
  }
  initI2S();
  initI2S_MicRX();  // 마이크 RX 초기화

  // 초기: SoftAP + HTTP 라우트
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("[OK] AP SSID: "); Serial.println(AP_SSID);
  Serial.print("[OK] AP IP  : "); Serial.println(WiFi.softAPIP());
  server.on("/audio",    HTTP_POST, handleAudioUpload);
  server.on("/status",   HTTP_GET,  handleStatus);
  server.on("/snapshot", HTTP_GET,  handleSnapshot);
  server.on("/upload",   HTTP_POST, handleUploadFinish, handleUploadData);
  server.begin();
  g_ready = true;

  // BLE
  NimBLEDevice::init("AI_DOCENT_GLASS");
  NimBLEServer* pServer   = NimBLEDevice::createServer();
  NimBLEService* service  = pServer->createService(SERVICE_UUID);

  chButton   = service->createCharacteristic(CHAR_BUTTON_UUID,     NIMBLE_PROPERTY::NOTIFY);
  chBattery  = service->createCharacteristic(CHAR_BATTERY_UUID,    NIMBLE_PROPERTY::NOTIFY);

  service->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("[OK] BLE Advertising started");

  // READY 알림 (AP IP 기준)
  bleNotifyReadyIP(WiFi.softAPIP(), 80);
}

// ==================== loop ====================
void loop() {
  server.handleClient();

  // 버튼1: 짧게 눌림 → 사진 촬영 + 전송
  static uint32_t lastHandled = 0;
  if (g_btnFlag && (millis() - lastHandled > 150)) {
    g_btnFlag = false;
    lastHandled = millis();

    // 1) 버튼 이벤트 JSON Notify
    const char* j = "{\"evt\":\"BUTTON\",\"type\":\"SHORT\"}";
    chButton->setValue((uint8_t*)j, strlen(j));
    chButton->notify();

    // 2) 사진 촬영 + 전송
    capture_and_send();
  }

  // 버튼2: 누르는 동안 → 오디오 스트리밍
  static bool btn2Prev = true;
  bool btn2Now = digitalRead(BUTTON2_PIN); // INPUT_PULLUP이므로 0=눌림
  if (btn2Prev && (btn2Now == LOW)) {
    // 눌림 시작 → 스트리밍 시작
    startAudioStreamToPhone();
  } else if (!btn2Prev && (btn2Now == HIGH)) {
    // 떼었음 → 스트리밍 종료
    stopAudioStreamToPhone();
  }
  btn2Prev = btn2Now;

  // 스트리밍 중이면 마이크 펌프
  if (g_streaming) {
    pumpMicOnce();
  }

  // 배터리 주기 전송 (5초마다)
  static uint32_t lastBat = 0;
  if (millis() - lastBat > 5000) {
    lastBat = millis();
    uint8_t b = (uint8_t)readBatteryPercent();
    chBattery->setValue(&b, 1);
    chBattery->notify();
  }
}
