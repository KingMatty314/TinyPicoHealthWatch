#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <I2Cdev.h>
#include <FreeRTOS.h>
#include <stdio.h>
#include <MPU6050.h>
#include <MAX30105.h>
#include <steps_counter.h>
#include <heart_rate.h>
#include <spO2.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Task Stack Size
# define STACK_SIZE 1024

// WiFi
const char* ssid = "UWB_Network";
const char* password = "123456789";

// Networking
const char* server = "192.168.10.100";
WiFiClient espClient;
PubSubClient client(espClient);

MPU6050 accelgyro;                        // Accelemeter Sensor
MAX30105 oximeterSensor;                  // Oximeter Sensor
Pedometer pedometer = Pedometer();        // Pedometer
HeartRate heartrate = HeartRate();        // Heartrate
Oxygenation oxygenation = Oxygenation();  // Oxygenation

// Constants
const int ButtonPin = 14;
bool recordingState = false;
const int RedLightThreshold = 30000;
const int IRLightThreshold = 50000;

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
/*
void connectToNetwork()
{
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println("Establishing connection to WiFi...");
  }
  Serial.println("Connected to network"); 
}

int byteArrayToInt(byte *data, int startIndex, int byteCount){
  int value = 0;
  for (int i = 0; i < byteCount; i++){
    int shift = i * 8;
    value += data[startIndex + i] << shift;
  }
  return value;
}

void sendAccel(float ax, float ay, float az)
{
  float accel = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  float dur = (float) (millis() - timeStampAccel) / 1000.0;
  char char_accel_x[8], char_accel_y[8], char_accel_z[8], char_accel[8], time[8];
  dtostrf(dur, 1, 2, time);
  dtostrf(ax, 1, 2, char_accel_x);
  dtostrf(ay, 1, 2, char_accel_y);
  dtostrf(az, 1, 2, char_accel_z);
  dtostrf(az, 1, 2, char_accel_z);
  client.publish("esp32watch/data/pedometer/time", time);
  client.publish("esp32watch/data/pedometer/accel_x", char_accel_x);
  client.publish("esp32watch/data/pedometer/accel_y", char_accel_y);
  client.publish("esp32watch/data/pedometer/accel_z", char_accel_z);
  client.publish("esp32watch/data/pedometer/accel", char_accel);
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
*/
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
  //connectToNetwork();
  //Serial.print("Mac Address: ") ;Serial.println(WiFi.macAddress());
  //Serial.print("IP: "); Serial.println(WiFi.localIP());

  // Connect to mqtt
  //client.setServer(server, 1883);
  //client.subscribe("esp32watch/data/time/hr");
  //client.subscribe("esp32watch/data/time/min");
  //client.subscribe("esp32watch/data/time/sec");
  //client.subscribe("esp32watch/data/time/day-of-week");
  //client.subscribe("esp32watch/data/time/month");
  //client.subscribe("esp32watch/data/time/day");
  //client.subscribe("esp32watch/data/time/year");
  //client.setCallback(ClockSynchronizationCallback);

  //if (!client.connected()) {
  //  Serial.println("Connecting MQTT Broker");
  //  reconnect();
  //}
  //client.loop();

  // Oximeter Sensor
  if (!oximeterSensor.begin(Wire, I2C_SPEED_FAST)){
    Serial.println("MAX30105 oximeter module was not found. Please check wiriing/power. ");
  }

  byte ledBrightness = 0x7f;
  byte sampleRate = 50;
  byte ledMode = 2;
  byte samapleAverage = 1;
  int pulseWidth = 411;
  int adcRange = 16384;
  oximeterSensor.setup(ledBrightness, samapleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

/*
/// RTOS Task Updates Users Step from Watch Accelerometer
void UpdateStepCounterTask(void *pvParameters){
  while (1){
    unsigned long StartTime = millis();         // Comment out future
    unsigned long ElapsedTime = 0;
    current_steps = pedometer.count_steps();
    ElapsedTime = millis() - StartTime;
    steps = ((current_steps + prior_steps)/2)*10;    // Steps are sampled very 10 seconds 
  }
}

/// RTOS Task Updates LCD Display on Watch Face
void DisplayLCDInfoTask(void *pvParameters){
  while(1){
    Serial.println("Displaying Data on Display");
  }
}

/// RTOS Task Updates Oximeter data includind heartbeat rate and oxygenation percentage
void UpdateStepOximeter(void *pvParameters){
  while(1){

  }

}

/// RTOS Task reporting all raw sensor data to MQTT broker
void ReportRawData(void *pvParameters){
  while(1){

  }

}

/// RTOS Task reporting health sensor data to MQTT broker
void ReportHealthSensorData(void *pvParameter){
  while(1){

  }
}

/// RTOS Task Loop MQTT Client to listen to Subsriber data from broker
void ListenBrokerSubsriberData(void *pvParameter){
  //while(1){
    client.loop();
  //}
}
*/

void loop() {
  float accelx, accely, accelz;
  int16_t ax, ay, az, gx, gy, gz;
  int32_t red;
  int32_t ir;
  int steps = 0;
  int heartbeat = 0;
  int oxyg = 0;
  int temp = 0;
  while (true){
    recordButttonState();

    // oximeter sample frequency 50Hz
    oximeterSensor.check();
    while (oximeterSensor.available()){
      red = oximeterSensor.getFIFORed();
      ir = oximeterSensor.getFIFOIR();
      temp = oximeterSensor.readTemperature();
      if (red > RedLightThreshold && ir > IRLightThreshold){
        // light is above threshold save
        heartrate.add_data(ir);
        oxygenation.add_data(ir, red);
      } else {
        // light is under threshold reset buffer
        heartrate.clear_data();
        oxygenation.clear_data();
      }
      // read next set of samples
      oximeterSensor.nextSample();  
    }

    // accelometer sample frequency 20Hz
    if (accelgyro.testConnection()){
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      accelx = float(ax / 16384.0);
      accely = float(ay / 16384.0);
      accelz = float(az / 16384.0);
      pedometer.add_data(accelx, accely, accelz);
    }

    // Check if buffers are full
    if (heartrate.is_buffer_full())
      heartbeat = heartrate.get_heart_rate();
    
    if (oxygenation.is_buffer_full()){
      oxyg = oxygenation.get_oxygenation();
    }

    if (pedometer.is_buffer_full()){
      steps += pedometer.get_count_steps();
    }

    // Data Fusion Algorthm
    // Activtity = DataFusion(heartbeat, temp, steps_delta)
    
    // Send Data Over in queue Networking communication task.


    ///////////  Print Data /////////////////
    // Debug comment once done.
    Serial.print("accelx: "); Serial.print(accelx); Serial.print(" accely: "); Serial.print(accely); Serial.print(" accelz: "); Serial.print(accelz);
    Serial.print(" ir: "); Serial.print(ir); Serial.print(" red: "); Serial.print(red); Serial.print(" temp: "); Serial.println(temp);
    Serial.print("Steps: "); Serial.print(steps); Serial.print(" HB: "); Serial.print(heartbeat); Serial.print(" O2: "); Serial.println(oxyg);
    Serial.print("Device State: "); Serial.println(recordingState);
    Serial.print("Buffer Index: "); Serial.println(heartrate.get_buffer_index());
    Serial.println();

    delay(100);
  }
}