#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>
#include <Arduino.h>


void reconnect_mqtt(PubSubClient &mqttClient, const char* client_id);
#endif
