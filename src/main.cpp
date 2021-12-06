#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <I2Cdev.h>
#include <FreeRTOS.h>
#include <esp_system.h>
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
# define STACK_SIZE 4096

// WiFi
const char* ssid = "UWB_Network";
const char* password = "123456789";

// Networking
const char* server = "192.168.10.100";
WiFiClient espClient;
PubSubClient client(espClient);

// Sensors
MPU6050 accelgyro;                        // Accelemeter Sensor
MAX30105 oximeterSensor;                  // Oximeter Sensor
Pedometer pedometer = Pedometer();        // Pedometer
HeartRate heartrate = HeartRate();        // Heartrate
Oxygenation oxygenation = Oxygenation();  // Oxygenation

// Watch Constants
const int ButtonPin = 14;
bool recordingState = false;
const int RedLightThreshold = 30000;
const int IRLightThreshold = 50000;

// Configuration of Oximeter Sensor
byte ledBrightness = 0x7f;
byte sampleRate = 200;            // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
byte ledMode = 2;                 // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
byte samapleAverage = 1;          // Options: 1, 2, 4, 8, 16, 32
int pulseWidth = 411;             // Options: 69, 118, 215, 411
int adcRange = 16384;             // Options: 2048, 4096, 8192, 16384

// Message Structure of HealthData
typedef struct HealtData {
  int heartrate;
  int spO2;
  int steps;
  char activity[8];
} HealthData;

// Message Structure of SensorData
typedef struct SensorData {
  int red;
  int ir;
  float accel;
} SensorData;

// Data transfer queue
const int health_data_queue_len = 10;
const int sensor_data_queue_len = 10;
QueueHandle_t health_data_queue;
QueueHandle_t sensor_data_queue;

// Thread Locks
xSemaphoreHandle health_data_lock;
xSemaphoreHandle sensor_data_lock; 

// Task Handlers
TaskHandle_t TaskHandleSendDataOverMQTTCommunication;
TaskHandle_t TaskHandleUpdateHealthWatchData;

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
    vTaskDelay(1000/portTICK_PERIOD_MS);
    Serial.println("Establishing connection to WiFi...");
  }
  Serial.println("Connected to network"); 
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
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }
  }
}

/// RTOS Task Updates LCD Display on Watch Face
void SendDataOverMQTTCommunication(void *pvParameters){
  SensorData sensorData;
  HealtData healthData;

  char char_heart[6], char_spO2[6], char_steps[6];
  char char_red[25], char_ir[25], char_accel[8];

  // Debugging timestamp
  unsigned long timeStamp = millis();
  unsigned long timeStampOld = millis();

  while(1){
    // Sample GPIO 
    recordButttonState();

    // Check client still connected
    if(!client.connected())
      reconnect();
    
    // Health Data
    if (xSemaphoreTake(health_data_lock, (TickType_t)1000)){
      if (xQueueReceive(health_data_queue, &(healthData), (TickType_t)10) == pdPASS){
        sprintf(char_heart, "%d", healthData.heartrate);
        sprintf(char_spO2, "%d", healthData.spO2);
        sprintf(char_steps, "%d", healthData.steps);
        client.publish("esp32watch/health/heartbeat", char_heart);
        client.publish("esp32watch/health/sp02", char_spO2);
        client.publish("esp32watch/health/steps", char_steps);
        //client.publish("esp32watch/health/activity", healthData.activity);
      }
      xSemaphoreGive(health_data_lock);
    }
    
    // Sensor Data 
    if (xSemaphoreTake(sensor_data_lock, (TickType_t)1000)){
      if (xQueueReceive(sensor_data_queue, &(sensorData), (TickType_t)10) == pdPASS){
        sprintf(char_red, "%d", sensorData.red);
        sprintf(char_ir, "%d", sensorData.ir);
        sprintf(char_accel, "%f", sensorData.accel);
        if (recordingState){
          client.publish("esp32watch/oximeter/red", char_red);
          client.publish("esp32watch/oximeter/ir", char_ir);
          client.publish("esp32watch/pedometer/accel", char_accel);
        }
      }
      xSemaphoreGive(sensor_data_lock);
    }

    ///////// Synchronication Check ///////////
    timeStampOld = timeStamp;
    timeStamp = millis();
    Serial.print("Task2 Dur: "); Serial.println(timeStamp - timeStampOld);
  }
}

/// RTOS Task Updates Oximeter data includind heartbeat rate and oxygenation percentage
void UpdateHealthWatchData(void *pvParameters){
  float accelx, accely, accelz, accel;
  int16_t ax, ay, az, gx, gy, gz;
  int32_t red;
  int32_t ir;
  int steps = 0;
  int heartbeat = 0;
  int oxyg = 0;
  int temp = 0;
  int countOximeter = 0;
  int countAccel = 0;

  // Debugging timestamp
  unsigned long timeStamp = millis();
  unsigned long timeStampOld = millis();

  // Mesesage Struct Sent Over Queue
  HealthData healthData;
  SensorData sensorData;

  while (true){
    // oximeter sample frequency 20Hz
    if (countOximeter >= 2){
      oximeterSensor.check();
      while (oximeterSensor.available()){
        red = oximeterSensor.getFIFORed();
        ir = oximeterSensor.getFIFOIR();
        //temp = oximeterSensor.readTemperature();
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
      countOximeter = 0;
    }

    // accelometer sample frequency 20Hz
    if (accelgyro.testConnection() && countAccel >= 5){
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      accelx = float(ax / 16384.0);
      accely = float(ay / 16384.0);
      accelz = float(az / 16384.0);
      accel = pedometer.add_data(accelx, accely, accelz);
    }

    // If count reads 0.05s and sends data off every 50Hz
    if (countAccel >= 5){

      // Check if buffers are full
      if (heartrate.is_buffer_full()){
        heartbeat = heartrate.get_heart_rate();
      }

      if (oxygenation.is_buffer_full()){
        oxyg = oxygenation.get_oxygenation();
      }

      if (pedometer.is_buffer_full()){
        steps += pedometer.get_count_steps();
      }

      
      // Transfer over activity data
      if (xSemaphoreTake(health_data_lock, (TickType_t)1000)){
        healthData.heartrate = heartbeat;
        healthData.spO2 = oxyg;
        healthData.steps = steps;
        if (xQueueSend(health_data_queue, &healthData, (TickType_t)10) != pdPASS){
          Serial.println("Failed to post the health data message after 10 ticks.");
        }
        xSemaphoreGive(health_data_lock);
      }
      
      // Transfer over sensor data 
      if (xSemaphoreTake(sensor_data_lock, (TickType_t)1000)){
        if (recordingState){
          sensorData.accel = accel;
          sensorData.ir = ir;
          sensorData.red = red;
          if(xQueueSend(sensor_data_queue, &sensorData, (TickType_t)10) != pdPASS){
            Serial.println("Failed to post sensor data message, after 10 ticks.");
          }
        }
        xSemaphoreGive(sensor_data_lock);
      }
      countAccel = 0;
    }
    
    // Data Fusion Algorthm
    // Activtity = DataFusion(heartbeat, temp, steps_delta)
    
    ///////////  Print Data /////////////////
    // Debug comment once done.
    //Serial.print("accelx: "); Serial.print(accelx); Serial.print(" accely: "); Serial.print(accely); Serial.print(" accelz: "); Serial.print(accelz);
    //Serial.print(" ir: "); Serial.print(ir); Serial.print(" red: "); Serial.print(red); Serial.print(" temp: "); Serial.println(temp);
    //Serial.print("Steps: "); Serial.print(steps); Serial.print(" HB: "); Serial.print(heartbeat); Serial.print(" O2: "); Serial.println(oxyg);
    //Serial.print("Device State: "); Serial.println(recordingState);
    //Serial.print("Buffer Index: "); Serial.println(heartrate.get_buffer_index());
    //Serial.println();

    ///////// Synchronication Check ///////////
    timeStampOld = timeStamp;
    timeStamp = millis();
    Serial.print("Task1 Dur: "); Serial.println(timeStamp - timeStampOld);
    

    vTaskDelay(10/portTICK_PERIOD_MS);
    countOximeter += 1;
    countAccel += 1;
  }
}

void setup() 
{
  Serial.begin(115200);
  // Setup push button
  pinMode(ButtonPin, INPUT);

  Serial.print("Clock Frequency: "); Serial.println(getCpuFrequencyMhz());

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
  connectToNetwork();
  Serial.print("Mac Address: ") ;Serial.println(WiFi.macAddress());
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // Connect to mqtt
  client.setServer(server, 1883);

  if (!client.connected()) {
    Serial.println("Connecting MQTT Broker");
    reconnect();
  }
  client.loop();

  // Oximeter Sensor
  if (!oximeterSensor.begin(Wire, I2C_SPEED_FAST)){
    Serial.println("MAX30105 oximeter module was not found. Please check wiriing/power. ");
  }

  // Setup MAX30102 for oximeter sampling
  oximeterSensor.setup(ledBrightness, samapleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  // Create Mutex Locks for data queue
  health_data_lock = xSemaphoreCreateMutex();
  sensor_data_lock = xSemaphoreCreateMutex();

  // Create communication queue between tasks
  health_data_queue = xQueueCreate(health_data_queue_len, sizeof(HealtData));
  sensor_data_queue = xQueueCreate(sensor_data_queue_len, sizeof(SensorData));

  // Create task
  xTaskCreate(UpdateHealthWatchData, "UpdateWatchData", STACK_SIZE, NULL, 5, &TaskHandleUpdateHealthWatchData);
  xTaskCreate(SendDataOverMQTTCommunication, "MQTTCommunication", STACK_SIZE, NULL, 1, &TaskHandleUpdateHealthWatchData);
  Serial.println("Setup Done!");
}

void loop(){
  /* Device should not enter here unless tasks are idling */
}

