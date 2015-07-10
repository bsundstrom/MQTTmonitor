#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>

#define DEBUG_PRINT_ERR(...) do{printf("!!"); printf(__VA_ARGS__); printf(" @ line [%d] in function [%s]>\r\n",__LINE__,__FUNCTION__);}while(0)
#define DEBUG_PRINT_STATUS(...) do{printf(">>"); printf(__VA_ARGS__); printf("\r\n");}while(0)

/* Max size of UART buffer. */
#define MAIN_CHAT_BUFFER_SIZE 64

/* Max size of MQTT buffer. */
#define MAIN_MQTT_BUFFER_SIZE 128

/* Limitation of user name. */
#define MAIN_CHAT_USER_NAME_SIZE 64

/* Chat MQTT topic. */
#define MQTT_CLIENT_ID "SAMW25"
#define MQTT_TOPIC_STARTUP_TIME "bs/monitor/startup_time"
#define MQTT_TOPIC_SW0_LAST_OPENED "bs/monitor/sw0/lastopened"
#define MQTT_TOPIC_SW0_LAST_CLOSED "bs/monitor/sw0/lastclosed"
#define MQTT_TOPIC_HEARTBEAT "bs/monitor/heartbeat"

/*
 * A MQTT broker server which was connected.
 * test.mosquitto.org is public MQTT broker.
 */
static const char main_mqtt_broker[] = "test.mosquitto.org";

/** Wi-Fi Settings */
#define MAIN_WLAN_SSID        "yavin" /* < Destination SSID */
#define MAIN_WLAN_AUTH        M2M_WIFI_SEC_WPA_PSK /* < Security manner */
#define MAIN_WLAN_PSK         "starwars" /* < Password for Destination SSID */

/** Using NTP server information */
#define MAIN_WORLDWIDE_NTP_POOL_HOSTNAME        "pool.ntp.org"
#define MAIN_ASIA_NTP_POOL_HOSTNAME             "asia.pool.ntp.org"
#define MAIN_EUROPE_NTP_POOL_HOSTNAME           "europe.pool.ntp.org"
#define MAIN_NAMERICA_NTP_POOL_HOSTNAME         "north-america.pool.ntp.org"
#define MAIN_OCEANIA_NTP_POOL_HOSTNAME          "oceania.pool.ntp.org"
#define MAIN_SAMERICA_NTP_POOL_HOSTNAME         "south-america.pool.ntp.org"
#define MAIN_SERVER_PORT_FOR_UDP                (123)
#define MAIN_DEFAULT_ADDRESS                    0xFFFFFFFF /* "255.255.255.255" */
#define MAIN_DEFAULT_PORT                       (6666)
#define MAIN_WIFI_M2M_BUFFER_SIZE               1460

void extint_detection_callback(void);
void configure_extint_callbacks(void);
void configure_rtc_calendar(void);
void configure_extint_channel(void);
void set_next_rtc_alarm(uint32_t num_of_seconds);
void configure_rtc_callbacks(void);
void rtc_match_callback(void);
void GetTimeFromServer(void);
char* GenerateTimeStamp(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */
