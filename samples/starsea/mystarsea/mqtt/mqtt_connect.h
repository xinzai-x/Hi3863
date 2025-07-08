#ifndef MQTT_CONNECT_H
#define MQTT_CONNECT_H

void set_message_callback(void (*callback)(const char*, const char*));
errcode_t mqtt_connect(void);

#endif