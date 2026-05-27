#include "LSM6DSOXSensor.h"
#include "lsm6dsox_activity_recognition_for_mobile.h"
#include <LittleFS_Mbed_RP2040.h>
#include <WiFiNINA.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#define SerialPort Serial

// ── Pin / bus ──────────────────────────────────────────────────────────────
#define INT_1 INT_IMU

#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif

// ── WiFi credentials ───────────────────────────────────────────────────────
const char* WIFI_SSID = "NatPark";
const char* WIFI_PASS = "arches10553";

// ── Globals ────────────────────────────────────────────────────────────────
volatile bool mems_event = false;
LSM6DSOXSensor AccGyr(&DEV_I2C, LSM6DSOX_I2C_ADD_L);
WiFiServer server(80);
LittleFS_MBED lfs;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -25200);

const char* LOG_FILE = MBED_LITTLEFS_FILE_PREFIX "/imu_log.csv";
uint8_t currentActivity = 255;
bool logging = false;
unsigned long sessionStartMillis = 0;
unsigned long sessionStartEpoch = 0;
unsigned long lastSerialPrint = 0;

// ── Forward declarations ───────────────────────────────────────────────────
void INT1Event_cb();
const char* activityLabel(uint8_t status);
void handleHTTP();
void epochToDateTime(unsigned long epoch, int &yr, int &mo, int &dy, int &hr, int &mn, int &sc);

// ── Epoch to date/time ─────────────────────────────────────────────────────
void epochToDateTime(unsigned long epoch, int &yr, int &mo, int &dy, int &hr, int &mn, int &sc) {
  sc = epoch % 60;
  mn = (epoch / 60) % 60;
  hr = (epoch / 3600) % 24;
  unsigned long days = epoch / 86400;
  yr = 1970;
  while (true) {
    bool leap = (yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0);
    unsigned long diy = leap ? 366 : 365;
    if (days < diy) break;
    days -= diy;
    yr++;
  }
  int monthDays[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  bool leap = (yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0);
  if (leap) monthDays[1] = 29;
  mo = 1;
  for (int i = 0; i < 12; i++) {
    if (days < (unsigned long)monthDays[i]) { mo = i + 1; break; }
    days -= monthDays[i];
  }
  dy = (int)days + 1;
}

// ── Setup ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(INT_1, OUTPUT);
  digitalWrite(INT_1, LOW);
  delay(200);

  DEV_I2C.begin();
  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();

  ucf_line_t* prog = (ucf_line_t*)lsm6dsox_activity_recognition_for_mobile;
  int32_t lines = sizeof(lsm6dsox_activity_recognition_for_mobile) / sizeof(ucf_line_t);
  Serial.print("Loading MLC program ("); Serial.print(lines); Serial.println(" lines)...");
  for (int32_t i = 0; i < lines; i++) {
    if (AccGyr.Write_Reg(prog[i].address, prog[i].data)) {
      Serial.print("MLC load error at line "); Serial.println(i);
      while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(250); }
    }
  }
  Serial.println("MLC loaded.");
  SerialPort.println("Program loaded inside the LSM6DSOX MLC");

  AccGyr.Disable_G();
  AccGyr.Enable_G();
  delay(200);

  if (!lfs.init()) {
    Serial.println("LittleFS init failed.");
    while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(500); }
  }
  Serial.println("LittleFS ready.");

  FILE* f = fopen(LOG_FILE, "w");
  if (!f) {
    Serial.println("Failed to open log file.");
    while (1);
  }
  fprintf(f, "timestamp,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,activity\n");
  fclose(f);
  Serial.println("Log file created.");

  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("HTTP server started.");

  timeClient.begin();
  timeClient.update();
  sessionStartEpoch = timeClient.getEpochTime();
  sessionStartMillis = millis();
  Serial.print("Time synced: "); Serial.println(timeClient.getFormattedTime());

  pinMode(INT_1, INPUT);
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  delay(3000);

  uint8_t mlc_out[8];
  AccGyr.Get_MLC_Output(mlc_out);
  currentActivity = mlc_out[0];
  logging = true;
  Serial.print("Initial activity: "); Serial.println(activityLabel(currentActivity));
}

// ── Loop ───────────────────────────────────────────────────────────────────
void loop() {
  if (mems_event) {
    mems_event = false;
    LSM6DSOX_MLC_Status_t status;
    AccGyr.Get_MLC_Status(&status);
    if (status.is_mlc1) {
      uint8_t mlc_out[8];
      AccGyr.Get_MLC_Output(mlc_out);
      currentActivity = mlc_out[0];
      Serial.print("Activity changed: ");
      Serial.println(activityLabel(currentActivity));
    }
  }

  int32_t accel[3], gyro[3];
  AccGyr.Get_X_Axes(accel);
  AccGyr.Get_G_Axes(gyro);

  unsigned long now = millis();
  unsigned long epochTime = sessionStartEpoch + (now - sessionStartMillis) / 1000;

  // Format timestamp
  int yr, mo, dy, hr, mn, sc;
  epochToDateTime(epochTime, yr, mo, dy, hr, mn, sc);
  char timestamp[20];
  snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d", yr, mo, dy, hr, mn, sc);

  // Buffer row
  static char rowBuf[4096] = "";
  char row[128];
  snprintf(row, sizeof(row), "%s,%ld,%ld,%ld,%ld,%ld,%ld,%s\n",
    timestamp,
    accel[0], accel[1], accel[2],
    gyro[0],  gyro[1],  gyro[2],
    activityLabel(currentActivity));
  strncat(rowBuf, row, sizeof(rowBuf) - strlen(rowBuf) - 1);

  // Serial Monitor once per second
  if (now - lastSerialPrint >= 1000) {
    lastSerialPrint = now;
    Serial.print("Time: "); Serial.print(timestamp);
    Serial.print(" | ax:"); Serial.print(accel[0]);
    Serial.print(" ay:"); Serial.print(accel[1]);
    Serial.print(" az:"); Serial.print(accel[2]);
    Serial.print(" | gx:"); Serial.print(gyro[0]);
    Serial.print(" gy:"); Serial.print(gyro[1]);
    Serial.print(" gz:"); Serial.print(gyro[2]);
    Serial.print(" | "); Serial.println(activityLabel(currentActivity));
  }

  // Flush every 2 seconds
  static unsigned long lastFlush = 0;
  if (now - lastFlush > 2000) {
    lastFlush = now;
    FILE* f = fopen(LOG_FILE, "a");
    if (f) {
      fprintf(f, "%s", rowBuf);
      fclose(f);
    }
    rowBuf[0] = '\0';
    handleHTTP();
  }

  delay(38);
}

// ── HTTP handler ───────────────────────────────────────────────────────────
void handleHTTP() {
  WiFiClient client = server.available();
  if (!client) return;

  unsigned long timeout = millis() + 2000;
  while (!client.available() && millis() < timeout);
  if (!client.available()) { client.stop(); return; }

  String request = client.readStringUntil('\r');
  client.readString();

  if (request.indexOf("GET /download") >= 0) {
    logging = false;
    FILE* f = fopen(LOG_FILE, "r");
    if (!f) {
      client.println("HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\n\r\nFile not found.");
      client.stop();
      logging = true;
      return;
    }
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/csv");
    client.println("Content-Disposition: attachment; filename=\"imu_log.csv\"");
    client.println("Connection: close");
    client.println();
    char buf[256];
    while (fgets(buf, sizeof(buf), f)) {
      client.print(buf);
    }
    fclose(f);
    client.stop();
    logging = true;
    Serial.println("CSV downloaded by client.");
  } else {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<!DOCTYPE html><html><body>");
    client.println("<h2>LSM6DSOX IMU Logger</h2>");
    client.print("<p>Current activity: <b>"); client.print(activityLabel(currentActivity)); client.println("</b></p>");
    client.println("<a href='/download'><button>Download CSV</button></a>");
    client.println("</body></html>");
    client.stop();
  }
}

// ── Interrupt callback ─────────────────────────────────────────────────────
void INT1Event_cb() {
  mems_event = true;
}

// ── Activity label ─────────────────────────────────────────────────────────
const char* activityLabel(uint8_t status) {
  switch (status) {
    case 0:  return "Stationary";
    case 1:  return "Walking";
    case 4:  return "Jogging";
    case 8:  return "Biking";
    case 12: return "Driving";
    default: return "Unknown";
  }
}