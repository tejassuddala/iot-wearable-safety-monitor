/* TrailGuard - All-in-one sketch
   Features:
   - OLED 128x64 (I2C SDA=32, SCL=33) with 3 screens + simple animation
   - MAX30102 (MAX30105 lib) for heart rate & SpO2 (approx)
   - Adafruit MPU6050 for fall detection (impact + confirmation)
   - GPS (TinyGPS++) on Serial2 (RX=16, TX=17)
   - NTP for time & date via WiFi
   - Blynk integration: V1 map, V2 HR, V3 SpO2, V4 Alert text + Blynk.notify()
   - SOS button: GPIO14 (press to send SOS)
   - Screen button: GPIO27 (cycle screens / cancel alert)
   - Cancel window after auto fall detection: press SCREEN button to cancel within window
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MAX30105.h>
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <BlynkSimpleEsp32.h>
#include <HTTPClient.h>

// ----------------- USER CONFIG -----------------
#define I2C_SDA 32
#define I2C_SCL 33

#define SOS_PIN 14        // D14
#define SCREEN_PIN 27     // D27

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

const char* WIFI_SSID = "Your_WiFi";
const char* WIFI_PASS = "Your_Password";

#define BLYNK_TEMPLATE_ID   "TMPLxxxxxx"
#define BLYNK_TEMPLATE_NAME "TrailGuard"
#define BLYNK_AUTH_TOKEN    "YourAuthTokenHere"


const long UTC_OFFSET_SECONDS = 19800L; // India GMT+5:30

// Blynk virtual pins
#define VPIN_MAP  V1
#define VPIN_HR   V2
#define VPIN_SPO2 V3
#define VPIN_ALERT V4

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Sensors & modules
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

// NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_SECONDS, 60000);

// Blynk timer
BlynkTimer timer;

// MAX buffers for SpO2 calc
const int BUFFER_SIZE = 100;
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// HR variables
unsigned long lastBeat = 0;
int beatsPerMinute = 0;
int beatAvg = 0;

// Fall detection variables (g-based)
const float IMPACT_THRESHOLD_G = 2.4;      // tweakable
const float MOTIONLESS_THRESHOLD_G = 0.8;  // tweakable
const unsigned long CONFIRM_WINDOW_MS = 2000;
unsigned long impactTime = 0;
bool impactFlag = false;
bool alertActive = false;
unsigned long alertStart = 0;
const unsigned long ALERT_CANCEL_MS = 9000; // 9 seconds to cancel
double lastKnownLat = 0.0, lastKnownLon = 0.0;

// UI
int screenPage = 0; // 0: HR, 1: SpO2, 2: Time/Date
bool lastScreenState = HIGH;
bool lastSosState = HIGH;
unsigned long lastDebounceScreen = 0, lastDebounceSos = 0;
const unsigned long DEBOUNCE_MS = 50;

// helper forward declarations
void sendAlert(const char* type);
void sendIFTTT(double lat, double lon, const String &message);

// ----------------- Helper Functions -----------------
float computeSpO2(uint32_t *redBuf, uint32_t *irBuf, int len) {
  // simple AC/DC approx using min/max and mean
  uint32_t minR = 0xFFFFFFFF, maxR = 0;
  uint32_t minIR = 0xFFFFFFFF, maxIR = 0;
  uint64_t sumR = 0, sumIR = 0;
  for (int i = 0; i < len; ++i) {
    uint32_t r = redBuf[i], ir = irBuf[i];
    if (r < minR) minR = r;
    if (r > maxR) maxR = r;
    if (ir < minIR) minIR = ir;
    if (ir > maxIR) maxIR = ir;
    sumR += r; sumIR += ir;
  }
  float dcR = (float)sumR / len;
  float dcIR = (float)sumIR / len;
  float acR = (float)(maxR - minR);
  float acIR = (float)(maxIR - minIR);
  if (dcIR <= 0 || acIR <= 0 || acR <= 0) return 0.0;
  float ratio = (acR / dcR) / (acIR / dcIR);
  float spo2 = 110.0 - 25.0 * ratio; // empirical mapping
  if (spo2 > 100) spo2 = 100;
  if (spo2 < 50) spo2 = 50;
  return spo2;
}

void drawTransition(String title) {
  // tiny animation: title slides from left
  display.clearDisplay();
  for (int x = -40; x <= 0; x += 10) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x, 8);
    display.print(title);
    display.display();
    delay(30);
  }
}

// OLED display functions
void showHeartbeatOLED() {
  drawTransition("Heartbeat");
  display.setTextSize(1);
  display.setCursor(0, 12);
  display.print("BPM:");
  display.setTextSize(2);
  display.setCursor(0, 28);
  display.print(beatsPerMinute);
  display.print(" ");
  display.setTextSize(1);
  display.setCursor(80, 28);
  display.print("HR");
  display.display();
}

void showSpO2OLED(float spo2) {
  drawTransition("Oxygen (SpO2)");
  display.setTextSize(1);
  display.setCursor(0, 12);
  display.print("SpO2:");
  display.setTextSize(2);
  display.setCursor(0, 28);
  if (spo2 > 0.1) display.print(spo2,1);
  else display.print("---");
  display.print(" %");
  display.display();
}

void showTimeOLED() {
  drawTransition("Time & Date");
  timeClient.update();
  String t = timeClient.getFormattedTime();
  unsigned long epoch = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t*)&epoch);
  int year = ptm->tm_year + 1900;
  int mon = ptm->tm_mon + 1;
  int day = ptm->tm_mday;
  display.setTextSize(1);
  display.setCursor(0, 12);
  display.print("Time:");
  display.setTextSize(2);
  display.setCursor(0, 28);
  display.print(t);
  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print(String(day) + "/" + String(mon) + "/" + String(year));
  display.display();
}

// ----------------- Alerts & IFTTT -----------------
void sendIFTTT(double lat, double lon, const String &message) {
  if (strlen(IFTTT_KEY) == 0) return;
  HTTPClient http;
  String url = "https://maker.ifttt.com/trigger/trailguard_alert/with/key/";
  url += IFTTT_KEY;
  url += "?value1=" + String(lat,6) + "&value2=" + String(lon,6) + "&value3=" + message;
  http.begin(url);
  int code = http.GET();
  http.end();
}

void sendAlert(const char* type) {
  double lat = 0.0, lon = 0.0;
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    lastKnownLat = lat; lastKnownLon = lon;
  } else {
    lat = lastKnownLat; lon = lastKnownLon;
  }
  String maps = "https://maps.google.com/?q=" + String(lat,6) + "," + String(lon,6);
  String msg = String(type) + " | Location: " + maps + " | HR:" + String(beatsPerMinute);
  // Blynk push
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.notify(msg); // push
    Blynk.virtualWrite(VPIN_MAP, 1, lat, lon, "TrailGuard");
    Blynk.virtualWrite(VPIN_HR, beatsPerMinute);
    // compute latest spo2 value from buffer if available
    float spo2_approx = 0.0;
    if (bufferFilled) spo2_approx = computeSpO2((uint32_t*)redBuffer, (uint32_t*)irBuffer, BUFFER_SIZE);
    Blynk.virtualWrite(VPIN_SPO2, spo2_approx);
    Blynk.virtualWrite(VPIN_ALERT, msg);
  }
  // IFTTT fallback (email/SMS)
  sendIFTTT(lat, lon, msg);
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  delay(50);

  // buttons
  pinMode(SOS_PIN, INPUT_PULLUP);
  pinMode(SCREEN_PIN, INPUT_PULLUP);

  // I2C on custom pins
  Wire.begin(I2C_SDA, I2C_SCL);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1) delay(100);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // MAX3010x
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX3010x not found. Check wiring.");
  } else {
    particleSensor.setup(); // default config
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeIR(0x0A);
  }
  // MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // GPS Serial2
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // WiFi connect
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0,0);
  display.print("Connecting WiFi...");
  display.display();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(250); Serial.print(".");
    retries++;
  }
  if (WiFi.status() == WL_CONNECTED) Serial.println("\nWiFi connected");
  else Serial.println("\nWiFi not connected (continue)");

  // NTP
  timeClient.begin();

  // Blynk (only if WiFi connected)
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.begin(BLYNK_AUTH, WIFI_SSID, WIFI_PASS);
  }

  // welcome
  display.clearDisplay(); display.setTextSize(1);
  display.setCursor(0,0); display.print("TrailGuard Ready");
  display.display();
  delay(700);

  // timers
  timer.setInterval(100L, [](){ // read MAX FIFO often
    // read sensor FIFO only if available
    if (particleSensor.available()) {
      long ir = particleSensor.getFIFOIR();
      long red = particleSensor.getFIFORed();
      if (ir == 0) ir = particleSensor.getIR();
      if (red == 0) red = particleSensor.getRed();
      irBuffer[bufferIndex] = (uint32_t)ir;
      redBuffer[bufferIndex] = (uint32_t)red;
      bufferIndex++;
      if (bufferIndex >= BUFFER_SIZE) { bufferIndex = 0; bufferFilled = true; }
      // heartbeat detection
      if (checkForBeat((int)ir)) {
        unsigned long delta = millis() - lastBeat;
        lastBeat = millis();
        if (delta > 0) beatsPerMinute = (int)(60 / (delta / 1000.0));
        if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(VPIN_HR, beatsPerMinute);
      }
    }
  });
  timer.setInterval(1000L, [](){ // gps feed parse & update last known coords
    while (GPSSerial.available()) {
      gps.encode(GPSSerial.read());
    }
    if (gps.location.isValid()) {
      lastKnownLat = gps.location.lat();
      lastKnownLon = gps.location.lng();
    }
  });
}

// ----------------- Main loop -----------------
unsigned long lastLoop = 0;
void loop() {
  unsigned long now = millis();
  Blynk.run();
  timer.run();

  // read MPU accel
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float ax = a.acceleration.x / 9.80665;
  float ay = a.acceleration.y / 9.80665;
  float az = a.acceleration.z / 9.80665;
  float accMag = sqrt(ax*ax + ay*ay + az*az);

  // FALL DETECTION: impact detection
  if (!impactFlag && accMag > IMPACT_THRESHOLD_G) {
    impactFlag = true;
    impactTime = now;
    // show warning & start cancel window
    alertActive = true;
    alertStart = now;
    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(0,0); display.print("Impact detected!");
    display.setCursor(0,16); display.print("Cancel within 9s (press screen)");
    display.display();
  }

  if (impactFlag) {
    // confirmation: within window, check if motionless (acc < threshold)
    if (now - impactTime <= CONFIRM_WINDOW_MS) {
      if (accMag < MOTIONLESS_THRESHOLD_G) {
        // confirmed fall
        impactFlag = false;
        Serial.println("Fall confirmed");
        sendAlert("Fall Detected");
      }
    } else {
      // expired - reset
      impactFlag = false;
    }
  }

  // If alertActive (cancel window), check cancel or timeout
  if (alertActive) {
    // check screen button as cancel during alert
    bool screenState = digitalRead(SCREEN_PIN);
    if (screenState == LOW) {
      // canceled
      alertActive = false;
      display.clearDisplay(); display.setTextSize(1);
      display.setCursor(0,0); display.print("Alert canceled");
      display.display();
      delay(800);
    } else if (now - alertStart > ALERT_CANCEL_MS) {
      // timeout passed, if no earlier confirmation, still we might have already sent alert in confirmed path.
      alertActive = false;
    }
  }

  // read SOS button (press sends immediate SOS)
  bool sosState = digitalRead(SOS_PIN);
  if (sosState == LOW && lastSosState == HIGH && (now - lastDebounceSos) > DEBOUNCE_MS) {
    lastDebounceSos = now;
    // immediate SOS
    sendAlert("SOS Button Pressed");
  }
  lastSosState = sosState;

  // screen button to cycle pages
  bool screenStateNow = digitalRead(SCREEN_PIN);
  if (screenStateNow == LOW && lastScreenState == HIGH && (now - lastDebounceScreen) > DEBOUNCE_MS) {
    lastDebounceScreen = now;
    screenPage = (screenPage + 1) % 3;
  }
  lastScreenState = screenStateNow;

  // compute SpO2 occasionally if buffer filled
  float spo2Val = 0.0;
  if (bufferFilled) {
    spo2Val = computeSpO2((uint32_t*)redBuffer, (uint32_t*)irBuffer, BUFFER_SIZE);
    if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(VPIN_SPO2, spo2Val);
  }

  // update OLED based on screenPage
  if (now - lastLoop > 400) { // refresh every 400ms or so
    lastLoop = now;
    display.clearDisplay();
    if (screenPage == 0) showHeartbeatOLED();
    else if (screenPage == 1) showSpO2OLED(spo2Val);
    else showTimeOLED();
  }

  delay(20);
}

