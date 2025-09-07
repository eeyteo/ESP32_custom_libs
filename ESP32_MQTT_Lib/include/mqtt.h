#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>

void publish_motion(bool motion, PubSubClient &mqttClient, const char* state_topic);
void reconnect_mqtt(PubSubClient &mqttClient, const char* client_id);
#endif