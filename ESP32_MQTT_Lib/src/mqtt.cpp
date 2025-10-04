#include "mqtt.h"
#include "config.h"


void reconnect_mqtt(PubSubClient &mqttClient, const char* client_id) {
  while (!mqttClient.connected()) {
    if (mqttClient.connect(client_id, MQTT_USER, MQTT_PASS)) {
      Serial.println("MQTT connected");
    } else {
      delay(2000);
    }
  }
}
