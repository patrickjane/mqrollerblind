// ************************************************************************************
// rollerblind
// ************************************************************************************
// ************************************************************************************
// IoT rollerblind control using Arduino Uno + wifi + Arduino Motor shield
// with a DC motor + encoder
// ************************************************************************************

// ************************************************************************************
// Includes
// ************************************************************************************

#include "motor.h"

#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
  #include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
  #include <WiFi101.h>
#elif defined(ARDUINO_ESP8266_ESP12)
  #include <ESP8266WiFi.h>
#endif

#include <MQTT.h>

// ************************************************************************************
// config
// ************************************************************************************

#include "config.h"

// config.h shall contain:

// #define WIFI_SSID "myssid"
// #define WIFI_PASS "mypass"

// #define MQTT_ID "my_client_id"
// #define MQTT_HOST "my_host"
// #define MQTT_PORT 1883

// #define TOPIC_STATE "ha/rollerblind/bedroom/status"
// #define TOPIC_ONLINE "ha/rollerblind/bedroom/online"
// #define TOPIC_CONTROL "ha/rollerblind/bedroom/control"
// #define TOPIC_CALIBRATE "ha/rollerblind/bedroom/calibrate"

// ************************************************************************************
// Defines / Settings / Configuration
// ************************************************************************************

#define DIRECTION_PIN 12
#define BRAKE_PIN 9
#define SPEED_PIN 3
#define ENCODER_INTERRUPT 2

const char* topics[]  = { TOPIC_CONTROL, TOPIC_CALIBRATE, 0 };

int connectMQTT();
int disconnectMQTT();
int connectWIFI();
int disconnectWIFI();

void onMessageReceived(MQTTClient *client, char topic[], char payload[], int payload_length);

// ************************************************************************************
// Globals
// ************************************************************************************

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

  connectMQTT();
}

// ************************************************************************************
// Loop
// ************************************************************************************

void loop()
{
  if (WiFi.status() == 3)
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
    // attempt to connect to Wifi network
  
  Serial.print("[WIFI] Connecting to network ");
  Serial.print(WIFI_SSID);
  Serial.println(" ...");

  int dotsPrinted = 0;
  
  while (WiFi.begin(WIFI_SSID, WIFI_PASS) != WL_CONNECTED) 
  {
    // failed, retry
    Serial.print(".");
    dotsPrinted++;
    delay(1000);
  }

  if (dotsPrinted)
    Serial.print("\n[WIFI] Connected to network ");
  else
    Serial.print("[WIFI] Connected to network ");
    
  Serial.println(WIFI_SSID);
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
  Serial.println(" ...");

  while (!client.connect(MQTT_ID)) 
  {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("[MQTT] Connected");

  int i = -1;

  while (topics[++i])
  {
    Serial.print("[MQTT] Subscribing to topic: ");
    Serial.println(topics[i]);

    // subscribe to a topic
    
    // client.unsubscribe(topics[i]);
    
    if (!client.subscribe(topics[i]))
    {
      Serial.println("[MQTT] Failed to subscribe, disconnecting ...");
      client.disconnect();
      return -1;
    }
  }

  return 0;
}

int disconnectMQTT()
{
  client.disconnect();
  return 0;
}

// ************************************************************************************
// onMessageReceived
// ************************************************************************************

void onMessageReceived(MQTTClient *client, char topic[], char payload[], int payload_length) 
{
  Serial.println("GOT MESSAGE");
  dispatch(topic, payload);
}

// ************************************************************************************
// dispatch
// ************************************************************************************

void dispatch(const char* topic, const char* msg)
{
  Serial.print("Got message on topic: '");
  Serial.print(topic);
  Serial.println("'");
  
  if (!strcmp(topic, TOPIC_CALIBRATE))
  {
    motor.calibrate();
  }
  else if (!strcmp(topic, TOPIC_CONTROL))
  {
    if (!msg || !*msg || !isdigit(*msg))
      return;

    motor.goTo(atof(msg));
  }
}
