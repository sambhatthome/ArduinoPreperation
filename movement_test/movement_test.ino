#include "LSM6DSOXSensor.h"
#include "lsm6dsox_activity_recognition_for_mobile.h"
#include <LittleFS_Mbed_RP2040.h>
#include <WiFiNINA.h>
#include <Chirale_TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "model.h"
#define SerialPort Serial

// ── Pin / bus ──────────────────────────────────────────────────────────────
#define INT_1 INT_IMU

#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif

// ── WiFi credentials ───────────────────────────────────────────────────────
const char* WIFI_SSID = "TP-Link_CEF4";
const char* WIFI_PASS = "23753112";
const char* DJANGO_HOST = "192.168.0.101";
const int DJANGO_PORT = 8000;
const char* DJANGO_ENDPOINT = "/api/imu/";

// ── Globals ────────────────────────────────────────────────────────────────
volatile bool mems_event = false;
LSM6DSOXSensor AccGyr(&DEV_I2C, LSM6DSOX_I2C_ADD_L);
WiFiServer server(80);
LittleFS_MBED lfs;

const char* LOG_FILE = MBED_LITTLEFS_FILE_PREFIX "/imu_log.csv";
uint8_t currentActivity = 255;
bool logging = false;
unsigned long sessionStartMillis = 0;
unsigned long sessionStartEpoch = 0;
unsigned long lastSerialPrint = 0;

// ── TFLite globals ─────────────────────────────────────────────────────────
const tflite::Model* tfl_model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* tfl_input = nullptr;
TfLiteTensor* tfl_output = nullptr;
constexpr int kTensorArenaSize = 16 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];
float lastPrediction = -1.0f;

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

  WiFiClient timeHttpClient;
  if (timeHttpClient.connect(DJANGO_HOST, DJANGO_PORT)) {
    timeHttpClient.println("GET /api/time/ HTTP/1.1");
    timeHttpClient.print("Host: "); timeHttpClient.println(DJANGO_HOST);
    timeHttpClient.println("Connection: close");
    timeHttpClient.println();
    delay(500);
    String response = timeHttpClient.readString();
    int idx = response.indexOf("\"epoch\":");
    if (idx >= 0) {
        sessionStartEpoch = response.substring(idx + 8).toInt();
        Serial.print("Time from Pi: "); Serial.println(sessionStartEpoch);
    }
    timeHttpClient.stop();
  }
  sessionStartMillis = millis();

  // ── TFLite setup ──────────────────────────────────────────────────────
  tfl_model = tflite::GetModel(model_tflite);
  if (tfl_model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }

  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
    tfl_model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors failed!");
    while (1);
  }
  tfl_input = interpreter->input(0);
  tfl_output = interpreter->output(0);
  Serial.println("TFLite model loaded.");

  pinMode(INT_1, INPUT);
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  delay(3000);

  uint8_t mlc_out[8];
  AccGyr.Get_MLC_Output(mlc_out);
  currentActivity = mlc_out[0];
  logging = true;
  Serial.print("Initial activity: "); Serial.println(activityLabel(currentActivity));
}

void postToDjango(const char* timestamp, int32_t ax, int32_t ay, int32_t az,
                  int32_t gx, int32_t gy, int32_t gz, const char* activity) {
  WiFiClient djangoClient;
  if (!djangoClient.connect(DJANGO_HOST, DJANGO_PORT)) {
    Serial.println("Django connection failed.");
    return;
  }

  char body[256];
  snprintf(body, sizeof(body),
    "{\"timestamp\":\"%s\",\"ax_mg\":%ld,\"ay_mg\":%ld,\"az_mg\":%ld,"
    "\"gx_dps\":%ld,\"gy_dps\":%ld,\"gz_dps\":%ld,\"activity\":\"%s\"}",
    timestamp, ax, ay, az, gx, gy, gz, activity);

  djangoClient.println("POST /api/imu/ HTTP/1.1");
  djangoClient.print("Host: "); djangoClient.println(DJANGO_HOST);
  djangoClient.println("Content-Type: application/json");
  djangoClient.print("Content-Length: "); djangoClient.println(strlen(body));
  djangoClient.println("Connection: close");
  djangoClient.println();
  djangoClient.println(body);

  unsigned long timeout = millis() + 3000;
  while (!djangoClient.available() && millis() < timeout);
  String response = djangoClient.readStringUntil('\n');
  if (response.indexOf("201") >= 0) {
    Serial.println("Posted to Django OK.");
  } else {
    Serial.print("Django response: "); Serial.println(response);
  }
  djangoClient.stop();
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

  static int32_t batchAccel[10][3];
  static int32_t batchGyro[10][3];
  static char batchTimestamps[10][20];
  static int batchCount = 0;
  static unsigned long lastBatchSend = 0;
  static unsigned long lastSample = 0;

  unsigned long now = millis();

  // Sample at 10Hz (every 100ms)
  if (now - lastSample >= 100) {
    lastSample = now;

    int32_t accel[3], gyro[3];
    AccGyr.Get_X_Axes(accel);
    AccGyr.Get_G_Axes(gyro);

    unsigned long epochTime = sessionStartEpoch + (now - sessionStartMillis) / 1000 - 25200;
    int yr, mo, dy, hr, mn, sc;
    epochToDateTime(epochTime, yr, mo, dy, hr, mn, sc);
    snprintf(batchTimestamps[batchCount], sizeof(batchTimestamps[0]),
      "%04d-%02d-%02d %02d:%02d:%02d", yr, mo, dy, hr, mn, sc);

    batchAccel[batchCount][0] = accel[0];
    batchAccel[batchCount][1] = accel[1];
    batchAccel[batchCount][2] = accel[2];
    batchGyro[batchCount][0] = gyro[0];
    batchGyro[batchCount][1] = gyro[1];
    batchGyro[batchCount][2] = gyro[2];

    // Run inference on latest reading
    float ax_n = max(0.0f, min(1.0f, ((float)accel[0] - (-1138.0f)) / (794.0f - (-1138.0f))));
    float ay_n = max(0.0f, min(1.0f, ((float)accel[1] - (-511.0f)) / (713.0f - (-511.0f))));
    float az_n = max(0.0f, min(1.0f, ((float)accel[2] - (-169.0f)) / (1921.0f - (-169.0f))));
    float gx_n = max(0.0f, min(1.0f, ((float)gyro[0] - (-47425.0f)) / (97448.0f - (-47425.0f))));
    float gy_n = max(0.0f, min(1.0f, ((float)gyro[1] - (-252157.0f)) / (210113.0f - (-252157.0f))));
    float gz_n = max(0.0f, min(1.0f, ((float)gyro[2] - (-69825.0f)) / (139615.0f - (-69825.0f))));

    tfl_input->data.f[0] = ax_n;
    tfl_input->data.f[1] = ay_n;
    tfl_input->data.f[2] = az_n;
    tfl_input->data.f[3] = gx_n;
    tfl_input->data.f[4] = gy_n;
    tfl_input->data.f[5] = gz_n;

    if (interpreter->Invoke() == kTfLiteOk) {
      lastPrediction = tfl_output->data.f[0];
      digitalWrite(LED_BUILTIN, lastPrediction <= 0.5 ? HIGH : LOW);
    }

    Serial.print("ax:"); Serial.print(accel[0]);
    Serial.print(" ay:"); Serial.print(accel[1]);
    Serial.print(" az:"); Serial.print(accel[2]);
    Serial.print(" | gx:"); Serial.print(gyro[0]);
    Serial.print(" gy:"); Serial.print(gyro[1]);
    Serial.print(" gz:"); Serial.print(gyro[2]);
    Serial.print(" | "); Serial.print(activityLabel(currentActivity));
    Serial.print(" | Pred: "); Serial.print(lastPrediction);
    Serial.println(lastPrediction > 0.5 ? " (Cruising)" : " (Not cruising)");

    batchCount++;
  }

  // Send batch every second
  if (now - lastBatchSend >= 1000 && batchCount > 0) {
    lastBatchSend = now;
    int countToSend = batchCount;
    batchCount = 0;

    String body = "[";
    for (int i = 0; i < countToSend; i++) {
      if (i > 0) body += ",";
      body += "{\"timestamp\":\"";
      body += batchTimestamps[i];
      body += "\",\"ax_mg\":";
      body += batchAccel[i][0];
      body += ",\"ay_mg\":";
      body += batchAccel[i][1];
      body += ",\"az_mg\":";
      body += batchAccel[i][2];
      body += ",\"gx_dps\":";
      body += batchGyro[i][0];
      body += ",\"gy_dps\":";
      body += batchGyro[i][1];
      body += ",\"gz_dps\":";
      body += batchGyro[i][2];
      body += ",\"activity\":\"";
      body += activityLabel(currentActivity);
      body += "\"}";
    }
    body += "]";

    WiFiClient djangoClient;
    djangoClient.setTimeout(200);
    if (djangoClient.connect(DJANGO_HOST, DJANGO_PORT)) {
      djangoClient.println("POST /api/imu/batch/ HTTP/1.1");
      djangoClient.print("Host: "); djangoClient.println(DJANGO_HOST);
      djangoClient.println("Content-Type: application/json");
      djangoClient.print("Content-Length: "); djangoClient.println(body.length());
      djangoClient.println("Connection: close");
      djangoClient.println();
      djangoClient.print(body);
      djangoClient.stop();
      Serial.print("Batch sent (");
      Serial.print(countToSend);
      Serial.println(" records)");
    } else {
      Serial.println("Django connection failed - skipping batch.");
    }
  }

  // Flush to flash every 2 seconds
  static unsigned long lastFlush = 0;
  if (now - lastFlush > 2000) {
    lastFlush = now;
    handleHTTP();
  }
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
    client.print("<p>Last prediction: <b>"); client.print(lastPrediction > 0.5 ? "Cruising" : "Not cruising"); client.println("</b></p>");
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