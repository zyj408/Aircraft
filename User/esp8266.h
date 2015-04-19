#ifndef _ESP8266_H
#define _ESP8266_H

enum WIFI_STATUS
{
	IDLE = 0,
	AP_BEGIN,
	CONNECTING
};

struct ip_infomation
{
	uint8_t id;
	uint8_t mode;
	char    ip_add[20];
	uint8_t port;
};


void ESP8266_send_data(char* str);
#endif
