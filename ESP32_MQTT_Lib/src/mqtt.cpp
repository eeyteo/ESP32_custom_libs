#include "mqtt.h"
#include "config.h"

void publish_motion(bool motion, PubSubClient &mqttClient,const char* state_topic) {
  if (!mqttClient.connected()) {
    reconnect_mqtt(mqttClient, state_topic);
  }
  mqttClient.loop();  // required for PubSubClient
  mqttClient.publish("home/gym/ld2411_motion", motion ? "ON" : "OFF", true);
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
