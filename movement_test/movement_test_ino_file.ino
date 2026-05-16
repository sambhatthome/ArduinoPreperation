#include "LSM6DSOXSensor.h"
#include "lsm6dsox_activity_recognition_for_mobile.h"
#include <LittleFS_Mbed_RP2040.h>
#include <WiFiNINA.h>
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

const char* LOG_FILE = MBED_LITTLEFS_FILE_PREFIX "/imu_log.csv";
uint8_t currentActivity = 255;   // 255 = uninitialized
bool logging = false;

// ── Forward declarations ───────────────────────────────────────────────────
void INT1Event_cb();
const char* activityLabel(uint8_t status);
void handleHTTP();

// ── Setup ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(LED_BUILTIN, OUTPUT);

  // Force INT1 low briefly to enable I2C on LSM6DSOX
  pinMode(INT_1, OUTPUT);
  digitalWrite(INT_1, LOW);
  delay(200);

  // I2C + IMU init
  DEV_I2C.begin();
  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();

  // Load MLC program
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

  // Re-enable gyroscope after MLC program load (UCF may have overwritten CTRL2_G)
  AccGyr.Disable_G();
  AccGyr.Enable_G();
  delay(200);

  // LittleFS
  if (!lfs.init()) {
    Serial.println("LittleFS init failed — check board/core version.");
    while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(500); }
  }
  Serial.println("LittleFS ready.");

  // Write CSV header (overwrites any previous session)
  FILE* f = fopen(LOG_FILE, "w");
  if (!f) {
    Serial.println("Failed to open log file for writing.");
    while (1);
  }
  fprintf(f, "timestamp_ms,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,activity\n");
  fclose(f);
  Serial.println("Log file created.");

  // WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("HTTP server started. Browse to the IP above to download the CSV.");

  // Interrupt
  pinMode(INT_1, INPUT);
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  delay(3000);

  // Log initial activity
  uint8_t mlc_out[8];
  AccGyr.Get_MLC_Output(mlc_out);
  currentActivity = mlc_out[0];
  logging = true;
  Serial.print("Initial activity: "); Serial.println(activityLabel(currentActivity));
}

// ── Loop ───────────────────────────────────────────────────────────────────
void loop() {
  // MLC check runs first, always, before any slow operations
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

  // Read IMU
  int32_t accel[3], gyro[3];
  AccGyr.Get_X_Axes(accel);
  AccGyr.Get_G_Axes(gyro);

  // Buffer the row in RAM
  static char rowBuf[4096] = "";
  char row[64];
  snprintf(row, sizeof(row), "%lu,%ld,%ld,%ld,%ld,%ld,%ld,%s\n",
    millis(),
    accel[0], accel[1], accel[2],
    gyro[0],  gyro[1],  gyro[2],
    activityLabel(currentActivity));
  strncat(rowBuf, row, sizeof(rowBuf) - strlen(rowBuf) - 1);

  // Flush to flash only every 2 seconds, not every loop
  static unsigned long lastFlush = 0;
  if (millis() - lastFlush > 2000) {
    lastFlush = millis();
    FILE* f = fopen(LOG_FILE, "a");
    if (f) {
      fprintf(f, "%s", rowBuf);
      fclose(f);
    }
    rowBuf[0] = '\0';

    // Only check HTTP after a flush, not every loop
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
  client.readString();  // consume rest of headers

  if (request.indexOf("GET /download") >= 0) {
    // Pause logging while sending
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
    // Landing page
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