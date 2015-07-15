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

#include "asf.h"
#include "main.h"
#include "driver/include/m2m_wifi.h"
#include "iot/mqtt/mqtt.h"
#include "iot/sw_timer.h"
#include "socket/include/socket.h"

#define ENABLE_AUTO_TIME_SETTING
//#define ENABLE_SLEEPING
#define HEARTBEAT_FREQ_SECS (60 * 10)

#define MAIN_PS_SLEEP_MODE M2M_PS_DEEP_AUTOMATIC

volatile bool report_sw_open;
volatile bool report_sw_closed;
volatile uint8_t glb_rtc_activity;
struct rtc_calendar_alarm_time alarm;
struct rtc_calendar_time my_time;

//! [rtc_module_instance]
struct rtc_module rtc_instance;

/** UART module for debug. */
static struct usart_module cdc_uart_module;

/** Instance of Timer module. */
struct sw_timer_module swt_module_inst;

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

/*The following is for TimeServer functionality*/
/** UDP socket handlers. */
static SOCKET udp_socket = -1;
/** Receive buffer definition. */
static uint8_t gau8SocketBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];
/** Wi-Fi status variable. */
static bool gbConnectedWifi = false;
volatile bool gb_time_is_set = false;
bool glb_report_startup = 1;
volatile bool glb_service_1s_flag = 0;
volatile uint16_t glb_close_debounce_tmr;
volatile uint16_t glb_open_debounce_tmr;

struct tc_module tc_instance;

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
		if (msg_wifi_state->u8CurrState == M2M_WIFI_CONNECTED) 
		{	/* If Wi-Fi is connected. */
			DEBUG_PRINT_STATUS("Wi-Fi connected - Requesting DHCP...");
			m2m_wifi_request_dhcp_client();
		} 
		else if(msg_wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) 
		{	/* If Wi-Fi is disconnected. */
			DEBUG_PRINT_STATUS("Wi-Fi disconnected.");
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			/* Disconnect from MQTT broker... 
			Force close the MQTT connection, because cannot send a disconnect message to the broker when network is broken. */
			mqtt_disconnect(&mqtt_inst, 1);
		}
		break;

	case M2M_WIFI_REQ_DHCP_CONF:
		DEBUG_PRINT_STATUS("DHCP Complete.");
		/* Try to connect to MQTT broker when Wi-Fi was connected. */
		DEBUG_PRINT_STATUS("Requesting socket from MQTT broker...");
		mqtt_connect(&mqtt_inst, main_mqtt_broker);
		mqtt_connection_state = CONNECTING;
		DEBUG_PRINT_STATUS("Requesting WiFi connection info...");
		m2m_wifi_get_connection_info();
		DEBUG_PRINT_STATUS("Request has been sent.");
		break;

	default:
		break;
	}
}

static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg_data)
{
	mqtt_socket_event_handler(sock, msg_type, msg_data);
}

static void socket_resolve_handler(uint8_t *doamin_name, uint32_t server_ip)
{
	mqtt_socket_resolve_handler(doamin_name, server_ip);
}

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
			mqtt_connect_broker(module_inst, 1, NULL, NULL, MQTT_CLIENT_ID, NULL, NULL, 0, 0, 0);
			DEBUG_PRINT_STATUS("Requesting MQTT connection...");
		} else {
			DEBUG_PRINT_ERR("Failed to connect to (%s)! Automatically retrying...", main_mqtt_broker);
			mqtt_connect(module_inst, main_mqtt_broker); /* Retry that. */
		}
	}
	break;

	case MQTT_CALLBACK_CONNECTED:
		if (data->connected.result == MQTT_CONN_RESULT_ACCEPT) {
			/* Subscribe chat topic. */
			//mqtt_subscribe(module_inst, MAIN_CHAT_TOPIC "#", 0);
			/* Enable USART receiving callback. */
			//usart_enable_callback(&cdc_uart_module, USART_CALLBACK_BUFFER_RECEIVED);
			DEBUG_PRINT_STATUS("MQTT Connection Accepted.");
			mqtt_connection_state = CONNECTED;
		} else {
			/* Cannot connect for some reason. */
			DEBUG_PRINT_ERR("MQTT broker declined your access! error code %d", data->connected.result);
		}

		break;

	case MQTT_CALLBACK_RECV_PUBLISH:
		//Hook for receiving subscriptions
		break;

	case MQTT_CALLBACK_DISCONNECTED:
		/* Stop timer and USART callback. */
		DEBUG_PRINT_STATUS("MQTT disconnected\r\n");
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
	config_extint_chan.filter_input_signal = true;
	extint_chan_set_config(BUTTON_0_EIC_LINE, &config_extint_chan);
}

void extint_detection_callback(void)
{
	bool pin_state = port_pin_get_input_level(BUTTON_0_PIN);
	if(pin_state && !glb_open_debounce_tmr)
	{
		report_sw_open = 1;	
		glb_open_debounce_tmr = 5; //500ms debounce timer
	}
	else if(!glb_close_debounce_tmr)
	{
		report_sw_closed = 1;
		glb_close_debounce_tmr = 5; //500ms debounce timer
	}
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
	set_next_rtc_alarm(HEARTBEAT_FREQ_SECS);
	glb_rtc_activity = 1;
}

void set_next_rtc_alarm(uint32_t num_of_seconds)
{
	struct rtc_calendar_time alarm_time;
	rtc_calendar_get_time(&rtc_instance, &alarm_time);
	alarm.time.day = alarm_time.day;
	alarm.time.hour = alarm_time.hour;
	alarm.time.minute = alarm_time.minute;
	alarm.time.month = alarm_time.month;
	alarm.time.pm = alarm_time.pm;
	alarm.time.year = alarm_time.year;
	alarm.time.second = alarm_time.second;	
	if(num_of_seconds < 60)
	{
		alarm.mask = RTC_CALENDAR_ALARM_MASK_SEC;
		alarm.time.second += num_of_seconds;
		alarm.time.second = alarm.time.second % 60;
	}
	else if(num_of_seconds < (60 * 60)) //time span is greater than an minute, less than an hour
	{
		alarm.mask = RTC_CALENDAR_ALARM_MASK_MIN;
		alarm.time.minute += num_of_seconds / 60;
		alarm.time.minute = alarm.time.minute % 60;		
	}
	else if(num_of_seconds < (3600 * 24)) //time span is greater than an hour, less than a day
	{
		alarm.mask = RTC_CALENDAR_ALARM_MASK_HOUR;
		alarm.time.minute += num_of_seconds / 3600;
		alarm.time.minute = alarm.time.hour % (3600 * 24);		
	}
	else
	{
		DEBUG_PRINT_ERR("FAILURE TO SET ALARM - Value too high!"); //time span can't exceed 23:59:59 hours
		return;
	}
	rtc_calendar_set_alarm(&rtc_instance, &alarm, RTC_CALENDAR_ALARM_0);	
}

/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg Type of Socket notification.
 * \param[in] pvMsg A structure contains notification informations.
 */
static void TimeServerCallback(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	/* Check for socket event on socket. */
	int16_t ret;

	switch (u8Msg) {
	case SOCKET_MSG_BIND:
	{
		/* printf("socket_cb: socket_msg_bind!\r\n"); */
		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
		if (pstrBind && pstrBind->status == 0) {
			ret = recvfrom(sock, gau8SocketBuffer, MAIN_WIFI_M2M_BUFFER_SIZE, 0);
			if (ret != SOCK_ERR_NO_ERROR) {
				printf("socket_cb: recv error!\r\n");
			}
		} else {
			printf("socket_cb: bind error!\r\n");
		}

		break;
	}

	case SOCKET_MSG_RECVFROM:
	{
		/* printf("socket_cb: socket_msg_recvfrom!\r\n"); */
		tstrSocketRecvMsg *pstrRx = (tstrSocketRecvMsg *)pvMsg;
		if (pstrRx->pu8Buffer && pstrRx->s16BufferSize) {
			uint8_t packetBuffer[48];
			memcpy(&packetBuffer, pstrRx->pu8Buffer, sizeof(packetBuffer));

			if ((packetBuffer[0] & 0x7) != 4) {                   /* expect only server response */
				printf("socket_cb: Expecting response from Server Only!\r\n");
				return;                    /* MODE is not server, abort */
			} else {
				uint32_t secsSince1900 = packetBuffer[40] << 24 |
						packetBuffer[41] << 16 |
						packetBuffer[42] << 8 |
						packetBuffer[43];

				/* Now convert NTP time into everyday time.
				 * Unix time starts on Jan 1 1970. In seconds, that's 2208988800.
				 * Subtract seventy years.
				 */
				const uint32_t seventyYears = 2208988800UL;
				time_t rawtime = secsSince1900 - seventyYears;

				/* Print the hour, minute and second.
				 * GMT is the time at Greenwich Meridian.
				 */
				struct tm * ptm;
				printf ("The current GMT time is:\r\n%s", ctime(&rawtime));
				ptm = localtime ( &rawtime );
				my_time.hour = ptm->tm_hour;
				my_time.minute = ptm->tm_min;
				my_time.second = ptm->tm_sec;
				my_time.year = ptm->tm_year + 1900;
				my_time.day = ptm->tm_mday;
				my_time.month = ptm->tm_mon + 1;
				//RTC is actually set in main().
				
				ret = close(sock);
				if (ret == SOCK_ERR_NO_ERROR) {
					udp_socket = -1;
					
				gb_time_is_set = true;
				}
			}
		}
	}
	break;

	default:
		break;
	}
}

/**
 * \brief Callback to get the ServerIP from DNS lookup.
 *
 * \param[in] pu8DomainName Domain name.
 * \param[in] u32ServerIP Server IP.
 */
static void TimeServerResolveCallback(uint8_t *pu8DomainName, uint32_t u32ServerIP)
{
	struct sockaddr_in addr;
	int8_t cDataBuf[48];
	int16_t ret;

	memset(cDataBuf, 0, sizeof(cDataBuf));
	cDataBuf[0] = '\x1b'; /* time query */

	printf("resolve_cb: DomainName %s\r\n", pu8DomainName);

	if (udp_socket >= 0) {
		/* Set NTP server socket address structure. */
		addr.sin_family = AF_INET;
		addr.sin_port = _htons(MAIN_SERVER_PORT_FOR_UDP);
		addr.sin_addr.s_addr = u32ServerIP;

		/*Send an NTP time query to the NTP server*/
		ret = sendto(udp_socket, (int8_t *)&cDataBuf, sizeof(cDataBuf), 0, (struct sockaddr *)&addr, sizeof(addr));
		if (ret != M2M_SUCCESS) {
			printf("resolve_cb: failed to send  error!\r\n");
			return;
		}
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType Type of Wi-Fi notification.
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters.
 */
static void TimeServerWiFiCallback(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) 
		{
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) 
		{
			gbConnectedWifi = false;
			printf("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		/* Turn LED0 on to declare that IP address received. */
		printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		gbConnectedWifi = true;

		/* Obtain the IP Address by network name */
		gethostbyname((uint8_t *)MAIN_WORLDWIDE_NTP_POOL_HOSTNAME);
		break;
	}

	default:
	{
		break;
	}
	}
}

void GetTimeFromServer(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	struct sockaddr_in addr_in;
	
	/* Initialize the WINC1500 BSP. */
	nm_bsp_init();	

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = TimeServerWiFiCallback;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}

	/* Initialize Socket module */
	socketInit();

	/* Register socket handler, resolve handler */
	registerSocketCallback(TimeServerCallback, TimeServerResolveCallback);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
			MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	while (gb_time_is_set == false) 
	{
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);

		if (gbConnectedWifi) 
		{
			/*
			 * Create the socket for the first time.
			 */
			if (udp_socket < 0) 
			{
				udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
				if (udp_socket < 0) 
				{
					printf("main: UDP Client Socket Creation Failed.\r\n");
					continue;
				}

				/* Initialize default socket address structure. */
				addr_in.sin_family = AF_INET;
				addr_in.sin_addr.s_addr = _htonl(MAIN_DEFAULT_ADDRESS);
				addr_in.sin_port = _htons(MAIN_DEFAULT_PORT);

				bind(udp_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in));
			}
		}
	}
	close(udp_socket);
	socketDeinit();
	if(m2m_wifi_disconnect() != M2M_SUCCESS)
		DEBUG_PRINT_ERR("Unable to cleanly disconnect the WiFi connection.");
	m2m_wifi_deinit(NULL);
}

char* GenerateTimeStamp(void)
{
	static char timestamp[30];
	rtc_calendar_get_time(&rtc_instance, &my_time);
	sprintf(timestamp, "%.2d/%.2d/%d %d:%.2d:%.2d GMT", my_time.month, my_time.day, my_time.year, my_time.hour, my_time.minute, my_time.second);
	return timestamp;	
}

void Service_1s(void)
{
	glb_service_1s_flag = 0;
	/*Place code to be serviced every 1 second here:*/
	
}

/*Callback should be fired every 100ms*/
void tc_callback(struct tc_module *const module_inst)
{
	static uint_fast8_t tmr_cntr_1s = 0;
	if(tmr_cntr_1s > 9)
	{
		tmr_cntr_1s = 0;
		glb_service_1s_flag = 1;
	}
	else
		tmr_cntr_1s++;
		
	if(glb_open_debounce_tmr)	
		glb_open_debounce_tmr--;
	if(glb_close_debounce_tmr)
		glb_close_debounce_tmr--;
}

void configure_tc(void)
{
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);
	config_tc.counter_size = TC_COUNTER_SIZE_16BIT;
	config_tc.clock_source = GCLK_GENERATOR_1;
	config_tc.clock_prescaler = TC_CLOCK_PRESCALER_DIV16;
	config_tc.counter_8_bit.period = 205;

	tc_init(&tc_instance, CONF_TC_MODULE, &config_tc);

	tc_enable(&tc_instance);
}

void configure_tc_callbacks(void)
{
	tc_register_callback(&tc_instance, tc_callback,TC_CALLBACK_OVERFLOW);
	tc_enable_callback(&tc_instance, TC_CALLBACK_OVERFLOW);
}
int main(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	char msg_payload[64];
	
	system_init();

	configure_console();
	DEBUG_PRINT_STATUS("***Application Starting***");
	DEBUG_PRINT_STATUS("Compiled on "__DATE__ " @ "__TIME__);
	
#ifdef ENABLE_AUTO_TIME_SETTING
	DEBUG_PRINT_STATUS("Retrieving time from server... RTC will be automatically set.");
	GetTimeFromServer();
#else
	DEBUG_PRINT_STATUS("RTC Time is NOT automatically set!!!");
	//since we don't get the actual time, fill with dummy time
	my_time.hour = 0;
	my_time.minute = 0;
	my_time.second = 0;
	my_time.year = 1900;
	my_time.day = 1;
	my_time.month = 1;
#endif

	/* Initialize the WINC1500 BSP. */
	nm_bsp_init();
	
	DEBUG_PRINT_STATUS("TimeClient closed. Starting MQTT Phase...");
	
	configure_rtc_calendar();
	configure_rtc_callbacks();
	rtc_calendar_set_time(&rtc_instance, &my_time);
	set_next_rtc_alarm(60);
	
	configure_extint_channel();
	configure_extint_callbacks();

	/* Initialize the Timer. */
	configure_timer();

	/* Initialize the MQTT service. */
	configure_mqtt();
	mqtt_connection_state = CONNECTING;

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));
	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_callback; /* Set Wi-Fi event callback. */
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) 
	{
		DEBUG_PRINT_ERR("Call Error: retval=%d", ret);
		while (1);
	}

	/* Initialize socket interface. */
	socketInit();
	registerSocketCallback(socket_event_handler, socket_resolve_handler);
	
	//Setup sleep mode
	if (MAIN_PS_SLEEP_MODE == M2M_PS_MANUAL) 
	{
		DEBUG_PRINT_STATUS("Sleep mode is M2M_PS_MANUAL");
		m2m_wifi_set_sleep_mode(MAIN_PS_SLEEP_MODE, 1);
	} 
	else if (MAIN_PS_SLEEP_MODE == M2M_PS_DEEP_AUTOMATIC) 
	{
		DEBUG_PRINT_STATUS("Sleep mode is M2M_PS_DEEP_AUTOMATIC");
		tstrM2mLsnInt strM2mLsnInt;
		m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, 1);
		strM2mLsnInt.u16LsnInt = M2M_LISTEN_INTERVAL;
		m2m_wifi_set_lsn_int(&strM2mLsnInt);
	}
	system_set_sleepmode(SYSTEM_SLEEPMODE_IDLE_0);
	
	/* Connect to router. */
	DEBUG_PRINT_STATUS("Connecting to WiFi...");
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
	DEBUG_PRINT_STATUS("Entering main loop...");
	while (1) 
	{
		/* Handle pending events from network controller. */
		m2m_wifi_handle_events(NULL);
		/* Checks the timer timeout. */
		sw_timer_task(&swt_module_inst);
		Service_1s(); //check if we need to service 1s stuff, service if needed.
		switch(mqtt_connection_state)
		{
			case DISCONNECTED:
				mqtt_connect(&mqtt_inst, main_mqtt_broker);
				DEBUG_PRINT_STATUS("Connecting to MQTT Broker...");
				mqtt_connection_state = CONNECTING;
				break;
				
			case CONNECTED:
				if(report_sw_open)
				{
					report_sw_open = 0;
					sprintf(msg_payload, "Opened @ %s", GenerateTimeStamp());					
					DEBUG_PRINT_STATUS("Sending via MQTT: '%s' to %s", msg_payload, MQTT_TOPIC_SW0_LAST_OPENED);
					mqtt_publish(&mqtt_inst, MQTT_TOPIC_SW0_LAST_OPENED, msg_payload, strlen(msg_payload), 0, 1);
				}
				else if(report_sw_closed)
				{
					report_sw_closed = 0;
					sprintf(msg_payload, "Closed @ %s", GenerateTimeStamp());	
					DEBUG_PRINT_STATUS("Sending via MQTT: '%s' to %s", msg_payload, MQTT_TOPIC_SW0_LAST_CLOSED);
					mqtt_publish(&mqtt_inst, MQTT_TOPIC_SW0_LAST_CLOSED, msg_payload, strlen(msg_payload), 0, 1);
				}
				else if (glb_rtc_activity)
				{
					glb_rtc_activity = 0;
					rtc_calendar_get_time(&rtc_instance, &my_time);
					sprintf(msg_payload, "Ping @ %s", GenerateTimeStamp());
					DEBUG_PRINT_STATUS("Sending via MQTT: %s", msg_payload);
					mqtt_publish(&mqtt_inst, MQTT_TOPIC_HEARTBEAT, msg_payload, strlen(msg_payload), 0, 1);
				}
				else if (glb_report_startup)
				{
					glb_report_startup = 0;
					sprintf(msg_payload, "Start-up @ %s", GenerateTimeStamp());
					DEBUG_PRINT_STATUS("Sending via MQTT: %s", msg_payload);
					mqtt_publish(&mqtt_inst, MQTT_TOPIC_STARTUP_TIME, msg_payload, strlen(msg_payload), 0, 1);					
				}
#ifdef ENABLE_SLEEPING
				if(mqtt_disconnect(&mqtt_inst, 1) == 0)
				{
					DEBUG_PRINT_STATUS("Disconnecting from MQTT Broker...");
					mqtt_connection_state = DISCONNECTED;	
				}
				else
					DEBUG_PRINT_ERR("Disconnection from broker failed.");
				DEBUG_PRINT_STATUS("---Waiting for activity...");
				if (MAIN_PS_SLEEP_MODE == M2M_PS_MANUAL)
					m2m_wifi_request_sleep(1000);
				system_sleep();
				while((glb_rtc_activity == 0) && (report_sw_open == 0) && (report_sw_closed == 0));
				DEBUG_PRINT_STATUS("Activity detected!");
#endif //ENABLE_SLEEPING
				break;
				
			default:
				break;		
		}
	}
}
