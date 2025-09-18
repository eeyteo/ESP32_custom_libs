#include "mqtt.h"
#include "config.h"
// Global variable to track operation mode
bool manualMode = false;
bool ledState = false; // Track current LED state

void publish_motion(bool motion, PubSubClient &mqttClient,const char* client_id, const char* topic) {
  // This function publishes motion state to the MQTT broker
  // It ensures the client is connected before publishing
  if (!mqttClient.connected()) {
    reconnect_mqtt(mqttClient, client_id);
  }
  mqttClient.loop();  // required for PubSubClient
  mqttClient.publish(topic, motion ? "ON" : "OFF", true);
}


void reconnect_mqtt(PubSubClient &mqttClient, const char* client_id) {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    bool connected;
        
    connected = mqttClient.connect(client_id, MQTT_USER, MQTT_PASS);
      
    if (connected) {
        Serial.println(" connected!");
    } else {
        Serial.print(" failed, rc=");
        Serial.println(mqttClient.state());
        delay(2000);
    }
  }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);
  
  if (String(topic) == "home/gym/sensor_mode/set") {
    if (message == "manual") {
      manualMode = true;
      Serial.println("Switched to MANUAL mode");
    } else if (message == "auto") {
      manualMode = false;
      Serial.println("Switched to AUTO mode");
    }
  }
  else if (String(topic) == "home/gym/led/set") {
    if (manualMode) {  // Only process LED commands in manual mode
      if (message == "ON") {
        Serial.println("Manual LED ON command");
        ledState = true;
      } else if (message == "OFF") {
        Serial.println("Manual LED OFF command");
        ledState = false;
      }
    } else {
      Serial.println("LED command ignored - not in manual mode");
    }
  }
}

void setupMQTT(PubSubClient &mqttClient) {
  mqttClient.setCallback(mqttCallback);
  // Subscribe to both topics
  mqttClient.subscribe("home/gym/sensor_mode/set");
  mqttClient.subscribe("home/gym/led/set");
}