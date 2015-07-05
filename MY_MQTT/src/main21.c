/**
 *
 * \file
 *
 * \brief WINC1500 MQTT chat example.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/** \mainpage
 * \section intro Introduction
 * This example demonstrates the use of the WINC1500 with the SAMD21 Xplained Pro
 * board to implement an MQTT based chat.
 * It uses the following hardware:
 * - the SAMD21 Xplained Pro.
 * - the WINC1500 on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC1500, connect to MQTT broker and chat with the other devices.
 * - mqtt.h : Implementation of MQTT 3.1
 *
 * \section usage Usage
 * -# Configure below code in the main.h for AP information to be connected.
 * \code
 *    #define MAIN_WLAN_SSID         "DEMO_AP"
 *    #define MAIN_WLAN_AUTH         M2M_WIFI_SEC_WPA_PSK
 *    #define MAIN_WLAN_PSK          "12345678"
 * \endcode
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application as the follows.
 * \code
 *    Baud Rate : 115200
 *    Data : 8bit
 *    Parity bit : none
 *    Stop bit : 1bit
 *    Flow control : none
 *    Line-Ending style : LF or CR+LF
 * \endcode
 * -# Start the application.
 * -# In the terminal window, First of all enter the user name through the terminal window.
 * -# And after the text of the following is displayed, please enjoy the chat.
 * -# Initialization operations takes a few minutes according to the network environment.
 * \code
 *    Preparation of the chat has been completed.
 * \endcode
 *
 * \section known_issue Known Issue
 * -# The user name cannot contain space (' ').
 * -# Cannot send more than 128 bytes.
 * -# User name must be unique. If someone uses the same user name, Which one will be disconnected.
 * -# USART interface has not error detection procedure. So sometimes serial input is broken.
 *
 * \section compinfo Compilation Information
 * This software was written for the GNU GCC compiler using Atmel Studio 6.2
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com">Atmel</A>.\n
 */

#include "asf.h"
#include "main.h"
#include "driver/include/m2m_wifi.h"
#include "iot/mqtt/mqtt.h"
#include "iot/sw_timer.h"
#include "socket/include/socket.h"

#define MAIN_PS_SLEEP_MODE M2M_PS_DEEP_AUTOMATIC

/* Application instruction phrase. */
#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- MY MQTT Monitor --"STRING_EOL \
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

char glb_topic[256];
char glb_msg[MAIN_CHAT_BUFFER_SIZE];
volatile uint8_t new_activity;
volatile uint8_t rtc_activity;
struct rtc_calendar_alarm_time alarm;

//! [rtc_module_instance]
struct rtc_module rtc_instance;

/** UART module for debug. */
static struct usart_module cdc_uart_module;

/** Instance of Timer module. */
struct sw_timer_module swt_module_inst;

/** User name of chat. */
char mqtt_user[64] = "";

/* Instance of MQTT service. */
static struct mqtt_module mqtt_inst;

/* Receive buffer of the MQTT service. */
static char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];

/** UART buffer. */
static char uart_buffer[MAIN_CHAT_BUFFER_SIZE];

/** Written size of UART buffer. */
static int uart_buffer_written = 0;

/** A buffer of character from the serial. */
static uint16_t uart_ch_buffer;

enum mqtt_connection_states
{
	
	DISCONNECTED,
	CONNECTING,
	CONNECTED,
	DISCONNECTING
} mqtt_connection_state;

static void uart_callback(const struct usart_module *const module)
{
	/* If input string is bigger than buffer size limit, ignore the excess part. */
	if (uart_buffer_written < MAIN_CHAT_BUFFER_SIZE) {
		uart_buffer[uart_buffer_written++] = uart_ch_buffer & 0xFF;
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] msg_type type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_callback(uint8 msg_type, void *msg_data)
{
	tstrM2mWifiStateChanged *msg_wifi_state;
	uint8 *msg_ip_addr;
	tstrM2MConnInfo     *pstrConnInfo = (tstrM2MConnInfo*)msg_data;

	switch (msg_type) 
	{	
	case M2M_WIFI_RESP_CONN_INFO:
		printf("CONNECTED AP INFO\n");
		printf("SSID                : %s\n",pstrConnInfo->acSSID);
		printf("SEC TYPE            : %d\n",pstrConnInfo->u8SecType);
		printf("Signal Strength     : %d\n", pstrConnInfo->s8RSSI);
		printf("Local IP Address    : %d.%d.%d.%d\r\n",
		pstrConnInfo->au8IPAddr[0] , pstrConnInfo->au8IPAddr[1], pstrConnInfo->au8IPAddr[2], pstrConnInfo->au8IPAddr[3]);
		break;
		
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
		msg_wifi_state = (tstrM2mWifiStateChanged *)msg_data;
		if (msg_wifi_state->u8CurrState == M2M_WIFI_CONNECTED) {
			/* If Wi-Fi is connected. */
			printf("Wi-Fi connected - Requesting DHCP...\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (msg_wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) {
			/* If Wi-Fi is disconnected. */
			printf("Wi-Fi disconnected\r\n");
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
					MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			/* Disconnect from MQTT broker. */
			/* Force close the MQTT connection, because cannot send a disconnect message to the broker when network is broken. */
			mqtt_disconnect(&mqtt_inst, 1);
		}

		break;

	case M2M_WIFI_REQ_DHCP_CONF:
		msg_ip_addr = (uint8 *)msg_data;
		printf("DHCP Complete.\r\nWi-Fi IP is %u.%u.%u.%u\r\n",
				msg_ip_addr[0], msg_ip_addr[1], msg_ip_addr[2], msg_ip_addr[3]);
		/* Try to connect to MQTT broker when Wi-Fi was connected. */
		mqtt_connect(&mqtt_inst, main_mqtt_broker);
		mqtt_connection_state = CONNECTING;
		printf("Connecting to MQTT broker...\n\r");
		m2m_wifi_get_connection_info();
		break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the Socket event.
 *
 * \param[in] Socket descriptor.
 * \param[in] msg_type type of Socket notification. Possible types are:
 *  - [SOCKET_MSG_CONNECT](@ref SOCKET_MSG_CONNECT)
 *  - [SOCKET_MSG_BIND](@ref SOCKET_MSG_BIND)
 *  - [SOCKET_MSG_LISTEN](@ref SOCKET_MSG_LISTEN)
 *  - [SOCKET_MSG_ACCEPT](@ref SOCKET_MSG_ACCEPT)
 *  - [SOCKET_MSG_RECV](@ref SOCKET_MSG_RECV)
 *  - [SOCKET_MSG_SEND](@ref SOCKET_MSG_SEND)
 *  - [SOCKET_MSG_SENDTO](@ref SOCKET_MSG_SENDTO)
 *  - [SOCKET_MSG_RECVFROM](@ref SOCKET_MSG_RECVFROM)
 * \param[in] msg_data A structure contains notification informations.
 */
static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data)
{
	mqtt_socket_event_handler(sock, msg_type, msg_data);
}

/**
 * \brief Callback of gethostbyname function.
 *
 * \param[in] doamin_name Domain name.
 * \param[in] server_ip IP of server.
 */
static void socket_resolve_handler(uint8_t *doamin_name, uint32_t server_ip)
{
	mqtt_socket_resolve_handler(doamin_name, server_ip);
}

#if(0)
void SetRTCTime(char *topic, char *msg)
{
	int8_t num;
	struct rtc_calendar_time time;
	rtc_calendar_get_time(&rtc_instance, &time);
	
	if (topic != NULL && msg != NULL)
	{
		if(strncmp(topic, "year", 4))
		{
			num = atoi(msg);
			if(num > 0)
			time.year = num;
		}
		if(strncmp(topic, "month", 5))
		{
			num = atoi(msg);
			if(num > 0)
			time.month = num;
		}
		if(strncmp(topic, "day", 3))
		{
			num = atoi(msg);
			if(num > 0)
			time.day = num;
		}
		if(strncmp(topic, "hour", 4))
		{
			num = atoi(msg);
			if(num > 0)
			time.hour = num;
		}
		if(strncmp(topic, "min", 4))
		{
			num = atoi(msg);
			if(num > 0)
			time.minute = num;
		}
		if(strncmp(topic, "sec", 4))
		{
			num = atoi(msg);
			if(num > 0)
			time.second = num;
		}
		rtc_calendar_set_time(&rtc_instance, &time);
	}
}
#endif

/**
 * \brief Callback to get the MQTT status update.
 *
 * \param[in] conn_id instance id of connection which is being used.
 * \param[in] type type of MQTT notification. Possible types are:
 *  - [MQTT_CALLBACK_SOCK_CONNECTED](@ref MQTT_CALLBACK_SOCK_CONNECTED)
 *  - [MQTT_CALLBACK_CONNECTED](@ref MQTT_CALLBACK_CONNECTED)
 *  - [MQTT_CALLBACK_PUBLISHED](@ref MQTT_CALLBACK_PUBLISHED)
 *  - [MQTT_CALLBACK_SUBSCRIBED](@ref MQTT_CALLBACK_SUBSCRIBED)
 *  - [MQTT_CALLBACK_UNSUBSCRIBED](@ref MQTT_CALLBACK_UNSUBSCRIBED)
 *  - [MQTT_CALLBACK_DISCONNECTED](@ref MQTT_CALLBACK_DISCONNECTED)
 *  - [MQTT_CALLBACK_RECV_PUBLISH](@ref MQTT_CALLBACK_RECV_PUBLISH)
 * \param[in] data A structure contains notification informations. @ref mqtt_data
 */
static void mqtt_callback(struct mqtt_module *module_inst, int type, union mqtt_data *data)
{
	switch (type) {
	case MQTT_CALLBACK_SOCK_CONNECTED:
	{
		/*
		 * If connecting to broker server is complete successfully, Start sending CONNECT message of MQTT.
		 * Or else retry to connect to broker server.
		 */
		if (data->sock_connected.result >= 0) {
			mqtt_connect_broker(module_inst, 1, NULL, NULL, mqtt_user, NULL, NULL, 0, 0, 0);
		} else {
			printf("Failed to connect to (%s)! Automatically retrying...\r\n", main_mqtt_broker);
			mqtt_connect(module_inst, main_mqtt_broker); /* Retry that. */
		}
	}
	break;

	case MQTT_CALLBACK_CONNECTED:
		if (data->connected.result == MQTT_CONN_RESULT_ACCEPT) {
			/* Subscribe chat topic. */
			mqtt_subscribe(module_inst, MAIN_CHAT_TOPIC "#", 0);
			/* Enable USART receiving callback. */
			usart_enable_callback(&cdc_uart_module, USART_CALLBACK_BUFFER_RECEIVED);
			printf("MQTT Connection Accepted.\r\n");
			mqtt_connection_state = CONNECTED;
		} else {
			/* Cannot connect for some reason. */
			printf("MQTT broker decline your access! error code %d\r\n", data->connected.result);
		}

		break;

	case MQTT_CALLBACK_RECV_PUBLISH:
		//SetRTCTime(data->recv_publish.topic, data->recv_publish.msg);
		break;

	case MQTT_CALLBACK_DISCONNECTED:
		/* Stop timer and USART callback. */
		printf("MQTT disconnected\r\n");
		usart_disable_callback(&cdc_uart_module, USART_CALLBACK_BUFFER_RECEIVED);
		mqtt_connection_state = DISCONNECTED;
		break;
	}
}

static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	usart_conf.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	usart_conf.baudrate    = 115200;

	stdio_serial_init(&cdc_uart_module, EDBG_CDC_MODULE, &usart_conf);
	/* Register USART callback for receiving user input. */
	usart_register_callback(&cdc_uart_module, (usart_callback_t)uart_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable(&cdc_uart_module);
}

static void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

static void configure_mqtt(void)
{
	struct mqtt_config mqtt_conf;
	int result;

	mqtt_get_config_defaults(&mqtt_conf);
	/* To use the MQTT service, it is necessary to always set the buffer and the timer. */
	mqtt_conf.timer_inst = &swt_module_inst;
	mqtt_conf.recv_buffer = mqtt_buffer;
	mqtt_conf.recv_buffer_size = MAIN_MQTT_BUFFER_SIZE;

	result = mqtt_init(&mqtt_inst, &mqtt_conf);
	if (result < 0) {
		printf("MQTT initialization failed. Error code is (%d)\r\n", result);
		while (1);
	}

	result = mqtt_register_callback(&mqtt_inst, mqtt_callback);
	if (result < 0) {
		printf("MQTT register callback failed. Error code is (%d)\r\n", result);
		while (1);
	}
}

void configure_extint_channel(void)
{
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin           = BUTTON_0_EIC_PIN;
	config_extint_chan.gpio_pin_mux       = BUTTON_0_EIC_MUX;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_BOTH;
	//	config_extint_chan.filter_input_signal = true;
	extint_chan_set_config(BUTTON_0_EIC_LINE, &config_extint_chan);
}

void extint_detection_callback(void)
{
	bool pin_state = port_pin_get_input_level(BUTTON_0_PIN);
	if(pin_state)
		strcpy(glb_msg, "Open");
	else if(!pin_state)
		strcpy(glb_msg, "Closed");
	new_activity = 1;
}

void configure_extint_callbacks(void)
{
	extint_register_callback(extint_detection_callback, BUTTON_0_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(BUTTON_0_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
}

void configure_rtc_calendar(void)
{
	/* Initialize RTC in calendar mode. */
	struct rtc_calendar_config config_rtc_calendar;
	
	rtc_calendar_get_config_defaults(&config_rtc_calendar);
	
	config_rtc_calendar.clock_24h     = true;
	config_rtc_calendar.alarm[0].time = alarm.time;
	config_rtc_calendar.alarm[0].mask = RTC_CALENDAR_ALARM_MASK_YEAR;
	rtc_calendar_init(&rtc_instance, RTC, &config_rtc_calendar);
	rtc_calendar_enable(&rtc_instance);
}

void configure_rtc_callbacks(void)
{
	rtc_calendar_register_callback(&rtc_instance, rtc_match_callback, RTC_CALENDAR_CALLBACK_ALARM_0);
	rtc_calendar_enable_callback(&rtc_instance, RTC_CALENDAR_CALLBACK_ALARM_0);
}

void rtc_match_callback(void)
{
	set_next_rtc_alarm(60);
	rtc_activity = 1;
}

void set_next_rtc_alarm(uint32_t num_of_seconds)
{
	struct rtc_calendar_time my_time;
	rtc_calendar_get_time(&rtc_instance, &my_time);
	alarm.time.day = my_time.day;
	alarm.time.hour = my_time.hour;
	alarm.time.minute = my_time.minute;
	alarm.time.month = my_time.month;
	alarm.time.pm = my_time.pm;
	alarm.time.year = my_time.year;
	alarm.time.second = my_time.second;	
	if(num_of_seconds < 60)
	{
		alarm.mask = RTC_CALENDAR_ALARM_MASK_SEC;
		alarm.time.second += num_of_seconds;
		alarm.time.second = alarm.time.second % 60;
	}
	else if(num_of_seconds < (60 * 60)) //time span is greater than an minute
	{
		alarm.mask = RTC_CALENDAR_ALARM_MASK_MIN;
		alarm.time.minute += num_of_seconds / 60;
		alarm.time.minute = alarm.time.minute % 60;		
	}
	else if(num_of_seconds < (3600 * 24)) //time span is greater than an hour
	{
		alarm.mask = RTC_CALENDAR_ALARM_MASK_HOUR;
		alarm.time.minute += num_of_seconds / 3600;
		alarm.time.minute = alarm.time.hour % (3600 * 24);		
	}
	else
	{
		printf("FAILURE TO SET ALARM - Value too high!"); //time span can't exceed 23:59:59 hours
		return;
	}
	rtc_calendar_set_alarm(&rtc_instance, &alarm, RTC_CALENDAR_ALARM_0);	
}

int main(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	char ping_msg[64];
	struct rtc_calendar_time my_time;
	/* Initialize the board. */
	system_init();

	configure_rtc_calendar();
	configure_rtc_callbacks();
	my_time.year   = 2015;
	my_time.month  = 1;
	my_time.day    = 1;
	my_time.hour   = 0;
	my_time.minute = 0;
	my_time.second = 0;
	rtc_calendar_set_time(&rtc_instance, &my_time);
	rtc_calendar_swap_time_mode(&rtc_instance);
	set_next_rtc_alarm(60);
	
	configure_extint_channel();
	configure_extint_callbacks();

	system_interrupt_enable_global();

	/* Initialize the UART console. */
	configure_console();

	/* Output example information */
	printf(STRING_HEADER);

	/* Initialize the Timer. */
	configure_timer();

	/* Initialize the MQTT service. */
	configure_mqtt();
	mqtt_connection_state = CONNECTING;

	/* Initialize the BSP. */
	nm_bsp_init();

	/* Setup user name first */
	strcpy(mqtt_user, "reed_switch");
	sprintf(glb_topic, "%s%s", MAIN_CHAT_TOPIC, mqtt_user);
	printf("Publishing to topic: %s\r\n", glb_topic);

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_callback; /* Set Wi-Fi event callback. */
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) 
	{
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1);
	}

	/* Initialize socket interface. */
	socketInit();
	registerSocketCallback(socket_event_handler, socket_resolve_handler);

	if (MAIN_PS_SLEEP_MODE == M2M_PS_MANUAL) 
	{
		printf("M2M_PS_MANUAL\r\n");
		m2m_wifi_set_sleep_mode(MAIN_PS_SLEEP_MODE, 1);
	} 
	else if (MAIN_PS_SLEEP_MODE == M2M_PS_DEEP_AUTOMATIC) 
	{
		printf("M2M_PS_DEEP_AUTOMATIC\r\n");
		tstrM2mLsnInt strM2mLsnInt;
		m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, 1);
		strM2mLsnInt.u16LsnInt = M2M_LISTEN_INTERVAL;
		m2m_wifi_set_lsn_int(&strM2mLsnInt);
	}
	
	system_set_sleepmode(SYSTEM_SLEEPMODE_IDLE_0);
	
	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	while (1) 
	{
		if (MAIN_PS_SLEEP_MODE == M2M_PS_MANUAL)
			m2m_wifi_request_sleep(1000);
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		sw_timer_task(&swt_module_inst);
		switch(mqtt_connection_state)
		{
			case DISCONNECTED:
				mqtt_connect(&mqtt_inst, main_mqtt_broker);
				printf("Connecting to MQTT Broker...\r\n");
				//TODO: sleep here...
				mqtt_connection_state = CONNECTING;
				break;
				
			case CONNECTED:
				if(new_activity)
				{
					new_activity = 0;
					rtc_calendar_get_time(&rtc_instance, &my_time);
					sprintf(ping_msg, "%s @ %d/%d/%d %d:%d:%d", glb_msg, my_time.day, my_time.month, my_time.year, my_time.hour, my_time.minute, my_time.second);
					printf("Sending: '%s' to %s\n", glb_msg, MAIN_CHAT_TOPIC);
					mqtt_publish(&mqtt_inst, glb_topic, ping_msg, strlen(ping_msg), 0, 1);
				}
				else if (rtc_activity)
				{
					rtc_activity = 0;
					/* Do something on RTC alarm match here */
					rtc_calendar_get_time(&rtc_instance, &my_time);
					sprintf(ping_msg, "Ping @ %d/%d/%d %d:%d:%d", my_time.month, my_time.day, my_time.year, my_time.hour, my_time.minute, my_time.second);
					printf("Sending ping message: %s", ping_msg);
					mqtt_publish(&mqtt_inst, "bs/monitor/ping", ping_msg, strlen(ping_msg), 0, 1);
				}
				else if(1)
				{
					if(mqtt_disconnect(&mqtt_inst, 1) == 0)
					{
						printf("Disconnecting from MQTT Broker...\r\n");
						mqtt_connection_state = DISCONNECTED;	
					}
					else
						printf("Disconnection from broker failed...\r\n");
					printf("---Going to sleep---\r\n");
					system_sleep();
					while((rtc_activity == 0) && (new_activity == 0));
					printf("Woke up!\n\r");
				}		
				break;
				
			default:
				break;		
		}
	}
}
