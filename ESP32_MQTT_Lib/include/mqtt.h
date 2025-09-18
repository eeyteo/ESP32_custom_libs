#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>
#include <Arduino.h>

void publish_motion(bool motion, PubSubClient &mqttClient, const char* client_id, const char* topic);
void reconnect_mqtt(PubSubClient &mqttClient, const char* client_id);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void setupMQTT(PubSubClient &mqttClient);
#endif
