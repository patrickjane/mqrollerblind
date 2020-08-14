// ************************************************************************************
// MQ Rollerblind
// ************************************************************************************
// ************************************************************************************
// IoT rollerblind control for Home Assistant using Arduino Uno + wifi + Arduino Motor shield
// with a DC motor + encoder
// ************************************************************************************
// Copyright (c) 2020 - Patrick Fial
// ************************************************************************************

// ************************************************************************************
// Includes
// ************************************************************************************

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h>
#elif defined(ARDUINO_ESP8266_ESP12)
  #include <ESP8266WiFi.h>
#endif

#include <MQTT.h>
#include "config.h"
#include "motor.h"

// ************************************************************************************
// config
// ************************************************************************************

/* config.h shall contain:

#define WIFI_SSID "myssid"
#define WIFI_PASS "mypass"

#define MQTT_ID "my_client_id"
#define MQTT_HOST "my_host"
#define MQTT_PORT 1883

#define TOPIC_STATE "ha/rollerblind/bedroom/status"            // will publish current status
#define TOPIC_POSITION "ha/rollerblind/bedroom/position"       // will publish current position when moving
#define TOPIC_CONFIG "ha/rollerblind/bedroom/cfg"              // (internal) will publish calibration results and current position w/ retain flag

#define TOPIC_CONTROL "ha/rollerblind/bedroom/control"         // will receive command to move
#define TOPIC_CALIBRATE "ha/rollerblind/bedroom/calibrate"     // will receive command to (re-)calibrate

Topics
------

TOPIC_STATE:
    Will publish (retained) the current state whenever it changes. State can change when a command is received 
    and the motor starts to move. 
    Valid states are: state_open, state_opening, state_closed, state_closing

TOPIC_POSITION:
    Will publish (retained)  a number between 100 and 0 representing the position in percent. 100 equals fully open/raised
    (at the top), 0 equals fully closed/lowered (at the bottom).

TOPIC_CONFIG:
    Will publish (retained) the calibration results (number of steps) of the cover length after finished calibration.

TOPIC_CONTROL: 
    Expects single number payload representing the position in percent, e.g. '30' (= 30% open).
    0 equals 0% open equals rollerblind fully closed (at the bottom). 100 equals 100% open equals
    rollerblind completely raised (at the top).
    If -1 is given as payload, the current movement will be stopped.

TOPIC_CALIBRATE:
      First message (with empty payload) starts calibration. Rollerblind is
      expected to be fully open (at the top), and will start going down/closing. Send second
      message (with empty payload) to mark the bottom point. Rollerblind will afterwards automatically
      go up again to the original top position.
      Note: the top point will NOT be calibrated and must be positioned correctly BEFORE starting
      the calibration.
*/

// ************************************************************************************
// Defines / Settings / Configuration
// ************************************************************************************

#define DIRECTION_PIN 12
#define BRAKE_PIN 9
#define SPEED_PIN 3
#define ENCODER_INTERRUPT 7

const char* topics[]  = { TOPIC_CONTROL, TOPIC_CALIBRATE, TOPIC_POSITION, TOPIC_CONFIG, 0 };
char buffer[100+1];

int connectMQTT();
int disconnectMQTT();
int connectWIFI();
int disconnectWIFI();
void finishInit();
void statusUpdate(bool withState = false);
void onMessageReceived(MQTTClient *client, char topic[], char payload[], int payload_length);

// ************************************************************************************
// Globals
// ************************************************************************************

long int start = 0;
long int boot = 0;

bool isInitializing = true;
int initPos = -1;
long int initSteps = -1;

bool shouldStop = false;
long int newCalibrationSteps = -1;

Motor motor(DIRECTION_PIN, BRAKE_PIN, SPEED_PIN, ENCODER_INTERRUPT);
WiFiClient wifiClient;
MQTTClient client;

// ************************************************************************************
// Setup
// ************************************************************************************

void setup() 
{
  Serial.begin(9600);

  connectWIFI();

  client.begin(MQTT_HOST, MQTT_PORT, wifiClient);
  client.setOptions(0, true, 360000);
  client.onMessageAdvanced(onMessageReceived);

  motor.setFlags(&newCalibrationSteps, &shouldStop);

  connectMQTT();
}

// ************************************************************************************
// Loop
// ************************************************************************************

void loop()
{
  long int now = millis();

  // initial setting of position & calibrated steps. will be received from retained
  // messages. also unsubscribe after pos/steps have been received.
  // force after 10s, or finish when pos & steps have been received.

  if ((boot && now - boot > 20000) || (initPos != -1 && initSteps != -1))
    finishInit();

  // stop flag set by motor?

  if (shouldStop)
  {
    shouldStop = false;
    motor.stop();

    statusUpdate(true);
  }

  // calibration done?

  if (newCalibrationSteps != -1)
    saveCalibration();

  // print motor position & publish status to respective MQTT topics

  if (motor.isMoving() && (!start || (now - start > 700) || (now < start)))
  {
    start = millis();
    statusUpdate();
  }

  // regularly check WIFI/MQTT connectivity

  if (WiFi.status() == WL_CONNECTED)
  {    
    client.loop();
  
    if (!client.connected())
    {
      Serial.println("[MQTT] Connection lost, trying to reconnect ...");
      connectMQTT();
    }    
  }
  else
  {
    Serial.println("[WIFI] Connection lost, trying to reconnect ...");

    disconnectWIFI();
    connectWIFI();
  }
}

// ************************************************************************************
// connect/disconnect WIFI
// ************************************************************************************

int connectWIFI()
{
  Serial.print("[WIFI] Connecting to network ");
  Serial.print(WIFI_SSID);
  Serial.print(" ...");

  while (WiFi.begin(WIFI_SSID, WIFI_PASS) != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println(" connected");
  return 0;
}

int disconnectWIFI()
{
  WiFi.disconnect();
}

// ************************************************************************************
// connect/disconnect MQTT
// ************************************************************************************

int connectMQTT() 
{
  Serial.print("[MQTT] Connecting to host ");
  Serial.print(MQTT_HOST);
  Serial.print(" ...");

  while (!client.connect(MQTT_ID)) 
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println(" connected");
  boot = millis();

  int i = -1;

  while (topics[++i])
  {
    Serial.print("[MQTT] Subscribing to topic: ");
    Serial.print(topics[i]);
    Serial.print(" ...");

    // subscribe to a topic
    
    if (!client.subscribe(topics[i]))
    {
      Serial.print(" failed to subscribe to ");
      Serial.print(topics[i]);
      Serial.println(", disconnecting");
      client.disconnect();
      return -1;
    }

    Serial.println(" done");
  }

  return 0;
}

int disconnectMQTT()
{
  client.disconnect();
  return 0;
}


// ************************************************************************************
// finishInit
// ************************************************************************************

void finishInit()
{
  if (!isInitializing)
    return;

  isInitializing = false;

  if (initPos != -1)
  {
    Serial.print("Initialize position with ");
    Serial.println(initPos);

    motor.setPosition(initPos);
  }

  if (initSteps != -1)
  {
    Serial.print("Initialize calibration with ");
    Serial.println(initSteps);
  
    motor.setCalibration(initSteps);
  
    if (initPos == -1)
      motor.setPosition(initSteps);
  }

  bool res = client.unsubscribe(TOPIC_CONFIG);

  if (!res)
    Serial.println("Failed to unsubscribe from TOPIC_CONFIG");
  
  res = client.unsubscribe(TOPIC_POSITION);

  if (!res)
    Serial.println("Failed to unsubscribe from TOPIC_POSITION");
}

// ************************************************************************************
// saveCalibration
// ************************************************************************************

void saveCalibration()
{
  Serial.print("Saving new calibration: ");
  Serial.println(newCalibrationSteps);

  snprintf(buffer, 100, "%d", motor.getCalibratedSteps());
  
  bool res = client.publish(TOPIC_CONFIG, buffer, true, 0);

  if (!res)
    Serial.println("Failed to publish calibration/config"); 

  newCalibrationSteps = -1;
}

// ************************************************************************************
// statusUpdate
// ************************************************************************************

void statusUpdate(bool withState)
{
  Serial.print("Step ");
  Serial.print(motor.getMotorPos());
  Serial.print(" - ");
  Serial.print(motor.getMotorPosPerc());
  Serial.println("%");

  if (motor.isCalibrating())
    return;

  snprintf(buffer, 100, "%d", motor.getMotorPos());

  bool res = client.publish(TOPIC_POSITION, buffer, true, 0);

  if (!res)
    Serial.println("Failed to publish position");

  if (withState)
  {
    const char* state = motor.getMotorPosPerc() >= 100 ? "state_open" : "state_closed";
    bool res = client.publish(TOPIC_STATE, state, true, 0);

    if (!res)
      Serial.println("Failed to publish state");  
  }
}

// ************************************************************************************
// onMessageReceived
// ************************************************************************************

void onMessageReceived(MQTTClient* client, char* topic, char* payload, int payload_length) 
{
  if (!topic)
  {
    Serial.println("Error: Received message without topic, must skip");
    return;  
  }

  if (!strcmp(topic, TOPIC_CALIBRATE))
  {
    Serial.println("Triggering calibration");
    motor.calibrate();
  }
  else if (!strcmp(topic, TOPIC_CONTROL))
  {
    if (!payload || !*payload || !isdigit(*payload))
    {
      if (payload && !strcmp(payload, "-1"))
      {
        Serial.println("Received stop command");
        shouldStop = true;
        return;
      }

      return;
    }
      
    int targetPerc = atof(payload);
    const char* state = motor.getMotorPosPerc() > targetPerc ? "state_closing" : "state_opening";

    bool res = client->publish(TOPIC_STATE, state, true, 0);

    if (!res)
      Serial.println("Failed to publish state");

    motor.goTo(targetPerc);
  }
  else if (!strcmp(topic, TOPIC_POSITION))
  {
    if (!payload || !*payload || !isdigit(*payload))
      return;

    initPos = atoi(payload);
  }
  else if (!strcmp(topic, TOPIC_CONFIG))
  {
    if (!payload || !*payload || !isdigit(*payload))
      return;

    initSteps = atol(payload);
  }
}
