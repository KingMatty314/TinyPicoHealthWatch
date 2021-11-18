#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <TinyPICO.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <steps_counter.h>
#include <optical_oximeter.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Define LSB Sensitivity
#define ACCEL_FULL_SCALE_RANGE 16384.0
#define GYRO_FULL_SCALE_RANGE 131.0

// Screen Size
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// WiFi
// iPhone
//const char* ssid = "";
//const char* password = "";

// Networking
const char* server = "192.168.10.100";
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long timeStampAccel = millis();
unsigned long timeStampOximeter = millis();


// Time 
unsigned long lastNTPtime;
unsigned long lastEntryTime;
tm timeinfo;
time_t now;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
TinyPICO tp = TinyPICO();
MPU6050 accelgyro;
Pedometer pedometer = Pedometer(accelgyro);
Oximeter oximeter = Oximeter();

int16_t ax, ay, az;
int16_t gx, gy, gz;

float accel_x, accel_y, accel_z;

const int RedPin = 4;
const int IRPin = 5;
const int freq = 5000;
const int RedChannel = 4;
const int IRChannel = 0;
const int RedResolution = 10;
const int IRResolution = 10;
const int ButtonPin = 14;
bool recordingState = false;

void recordButttonState()
{
  int buttonState = digitalRead(ButtonPin);
  if (buttonState == HIGH && recordingState == false){
    recordingState = true;
    while (digitalRead(ButtonPin) == HIGH);
  }
  buttonState = digitalRead(ButtonPin);
  if (buttonState == HIGH && recordingState == true){
    recordingState = false;
    while (digitalRead(ButtonPin) == HIGH);
  }
}

void connectToNetwork()
{
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println("Establishing connection to WiFi...");
  }
  Serial.println("Connected to network"); 
}

void scanNetworks()
{
  int numberOfNetworks = WiFi.scanNetworks();
  Serial.print("Number of networks found: ");
  Serial.println(numberOfNetworks);
 
  for (int i = 0; i < numberOfNetworks; i++) {
 
    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));
 
    Serial.print("Signal strength: ");
    Serial.println(WiFi.RSSI(i));
 
    Serial.print("MAC address: ");
    Serial.println(WiFi.BSSIDstr(i));
  }
}

void printLocalTime()
{
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void getTimeUpdate(unsigned long sec)
{
  tm *ptm;
  if ((millis() - lastEntryTime) < (1000*sec)){
    now = lastNTPtime + (int)(millis() - lastEntryTime) / 1000;
  } else {
    lastEntryTime = millis();
    lastNTPtime = time(&now);
    now = lastNTPtime;
  }
  ptm = localtime(&now);
  timeinfo = *ptm;
}

void updateMainDisplay()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  // Display heart rate
  display.setCursor(0, 0);
  display.println("HR:165 bpm");
  // Display oxygenation
  display.setCursor(SCREEN_WIDTH-60, 0);
  display.println("Sp02: 98%");
  // Activity Level and steps
  display.setCursor(0, SCREEN_HEIGHT-20);
  display.println("Sitting");
  display.setCursor(0, SCREEN_HEIGHT-10);
  display.println("Steps:13,444");
  // Battery and Wifi Status
  display.setCursor(SCREEN_WIDTH-50, SCREEN_HEIGHT-10);
  display.println("Bat:40%");
  display.setCursor(SCREEN_WIDTH-40, SCREEN_HEIGHT-20);
  display.println("WiFi-C");
  // Display time
  display.setTextSize(1);
  if(!getLocalTime(&timeinfo)){
    display.setCursor(20, 25);
    display.println("Failed to obtain time");
  } else {
    display.setCursor(20, 20);
    display.println(&timeinfo, "%B %d %Y");
    display.setCursor(35, 30);
    display.println(&timeinfo, "%H:%M:%S");
  }
  display.display(); 
}

void sendAccel(float ax, float ay, float az)
{
  float dur = (float) (millis() - timeStampAccel) / 1000.0;
  char char_accel_x[8], char_accel_y[8], char_accel_z[8], time[8];
  dtostrf(dur, 1, 2, time);
  dtostrf(ax, 1, 2, char_accel_x);
  dtostrf(ay, 1, 2, char_accel_y);
  dtostrf(az, 1, 2, char_accel_z);
  client.publish("esp32watch/data/pedometer/time", time);
  client.publish("esp32watch/data/pedometer/accel_x", char_accel_x);
  client.publish("esp32watch/data/pedometer/accel_y", char_accel_y);
  client.publish("esp32watch/data/pedometer/accel_z", char_accel_z);
  timeStampAccel = millis();
}

void sendOximeter(float red, float ir){
  float dur = (float) (millis() - timeStampOximeter) / 1000.0;
  char char_red[8], char_ir[8], time[8];
  dtostrf(dur, 1, 2, time);
  dtostrf(red, 1, 2, char_red);
  dtostrf(ir, 1, 2, char_ir);
  client.publish("esp32watch/data/oximeter/time", time);
  client.publish("esp32watch/data/oximeter/red", char_red);
  client.publish("esp32watch/data/oximeter/ir", char_ir);
  timeStampOximeter = millis();
}

void sendDevice(float battery, int recording){
  char char_bat[8], char_record[4];
  dtostrf(battery, 1, 2, char_bat);
  itoa(recording, char_record, 4);
  client.publish("esp32watch/data/battery", char_bat);
  client.publish("esp32watch/data/recording", char_record);
}

void sendHealth(float heartbeat, int sp02, int steps, char* activity){
  char char_heart[8], char_sp02[6], char_steps[6];
  dtostrf(heartbeat, 1, 2, char_heart);
  dtostrf(sp02, 1, 2, char_sp02);
  dtostrf(steps, 1, 2, char_steps);
  client.publish("esp32watch/health/heartbeat", 0);
  client.publish("esp32watch/health/sp02", 0);
  client.publish("esp32watch/health/steps", 0);
  client.publish("esp32watch/health/activity", 0);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32_Watch")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
}

void setup() 
{
  Serial.begin(115200);
  // Setup push button
  pinMode(ButtonPin, INPUT);

  // initialize device
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.println("Initializing MPU...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // MPU6050 self calibration offset adjustment
  accelgyro.CalibrateAccel(10);
  accelgyro.CalibrateGyro(10);

  //Connect tow WiFi
  //scanNetworks();
  connectToNetwork();
  Serial.print("Mac Address: ") ;Serial.println(WiFi.macAddress());
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  
  // Grab time
  //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //lastNTPtime = time(&now);
  //lastEntryTime = millis();

  // Setup LED PWM
  ledcSetup(RedChannel, freq, RedResolution);
  ledcSetup(IRChannel, freq, IRResolution);
  ledcAttachPin(RedPin, RedChannel);
  ledcAttachPin(IRPin, IRChannel);

  // Configure Screen
  //if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
  //  Serial.println(F("SSD1306 allocation failed"));
  //  for(;;);
  //}

  // Connect to mqtt
  client.setServer(server, 1883);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Toggle between recording state and non-recording state
  recordButttonState();

  // read raw accel/gyro measurements from device
  if (accelgyro.testConnection()){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // display tab-separated accel/gyro x/y/z values
    //Serial.print(float(ax/ACCEL_FULL_SCALE_RANGE)); Serial.print("\t");
    //Serial.print(float(ay/ACCEL_FULL_SCALE_RANGE)); Serial.print("\t");
    //Serial.print(float(az/ACCEL_FULL_SCALE_RANGE)); Serial.print("\t");
    //Serial.print(float(gx/GYRO_FULL_SCALE_RANGE)); Serial.print("\t");
    //Serial.print(float(gy/GYRO_FULL_SCALE_RANGE)); Serial.print("\t");
    //Serial.println(float(gz/GYRO_FULL_SCALE_RANGE));
    accel_x = float(ax/ACCEL_FULL_SCALE_RANGE);
    accel_y = float(ay/ACCEL_FULL_SCALE_RANGE);
    accel_z = float(az/ACCEL_FULL_SCALE_RANGE);

    if (recordingState){
      sendAccel(accel_x, accel_y, accel_z);
    }
  }
  //Serial.print("X:"); Serial.print(accel_x); Serial.print(" ");
  //Serial.print("Y:"); Serial.print(accel_y); Serial.print(" ");
  //Serial.print("Z:"); Serial.println(accel_z); 


  //int steps = pedometer.count_steps();
  //Serial.println(steps);
  
  //int brightnessRed = (int)(pow(2, RedResolution)-1);
  //int brightnessIR = (int)(pow(2, IRResolution)-1);
  //ledcWrite(RedChannel, brightnessRed);
  //ledcWrite(IRChannel, brightnessIR);
  //delay(500);

  //int analogRed = analogRead(33);
  //int analogIR = analogRead(32);

  //Serial.print("Red Brightness Output: "); Serial.print(brightnessRed);
  //Serial.println(analogRed);
  //Serial.print("ir: "); Serial.println(analogIR);
  //Serial.print("IR Brightness Output: "); Serial.print(brightnessIR);
  //Serial.print(" IR Voltage Value: "); Serial.print(analogIR); Serial.println("mV");

  // Battery
  float voltage = tp.GetBatteryVoltage();
  sendDevice(voltage, recordingState);
  
  delay(50);

  // Display time
  //getTimeUpdate(3600);
  //printLocalTime();
  //updateMainDisplay();

  // Battery stuff
  //delay(1000);
}