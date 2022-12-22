#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <zephyr/random/rand32.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#if defined(CONFIG_NRF_MODEM_LIB)
#include <nrf_modem_at.h>
#endif /* CONFIG_NRF_MODEM_LIB */
#include <modem/lte_lc.h>
#include <zephyr/logging/log.h>
#if defined(CONFIG_MODEM_KEY_MGMT)
#include <modem/modem_key_mgmt.h>
#endif
#if defined(CONFIG_LWM2M_CARRIER)
#include <lwm2m_carrier.h>
#endif
#include <dk_buttons_and_leds.h>

#include "certificates.h"

// Start:GNSS simple import modules/////////////////////////////////////////////////////

#include <nrf_modem_gnss.h>
#include "coordinates.h"

// End:GNSS simple import modules//////////////////////////////////////////////////////

// Start:GNSS simple declerations/////////////////////////////////////////////////////

/* STEP 5 - Define the PVT data frame variable */
static struct nrf_modem_gnss_pvt_data_frame pvt_data;
static struct nrf_modem_gnss_pvt_data_frame current_pvt;
static struct nrf_modem_gnss_pvt_data_frame last_pvt;

/* STEP 12.1 - Declare helper variables to find the TTFF */
static int64_t gnss_start_time;
static bool first_fix = false;

static K_SEM_DEFINE(lte_connected, 0, 1);
// End:GNSS simple declerations//////////////////////////////////////////////////////

LOG_MODULE_REGISTER(mqtt_simple, CONFIG_MQTT_SIMPLE_LOG_LEVEL);

/* Buffers for MQTT client. */
static uint8_t rx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static uint8_t tx_buffer[CONFIG_MQTT_MESSAGE_BUFFER_SIZE];
static uint8_t payload_buf[CONFIG_MQTT_PAYLOAD_BUFFER_SIZE];

/* The mqtt client struct */
static struct mqtt_client client;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

/* File descriptor */
static struct pollfd fds;

// Start:GNSS simple Functions/////////////////////////////////////////////////////

static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type)
	{
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
			(evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING))
		{
			break;
		}
		LOG_INF("Network registration status: %s",
				evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "Connected - home network" : "Connected - roaming");
		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_INF("RRC mode: %s", evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle");
		break;
	default:
		break;
	}
}

static void modem_configure_2(void) // change name, function already exists in MQTT
{
	LOG_INF("Connecting to LTE network");

	int err = lte_lc_init_and_connect_async(lte_handler);
	if (err)
	{
		LOG_ERR("Modem could not be configured, error: %d", err);
		return;
	}
	k_sem_take(&lte_connected, K_FOREVER);
	LOG_INF("Connected to LTE network");
	dk_set_led_on(DK_LED2);
}

/* STEP 6 - Define a function to log fix data in a readable format */
// static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data, float *latitude_uni, float *longitude_uni)

// gnss_event_handler(int event) original location/////////////////////////////////

// End:GNSS simple Functions//////////////////////////////////////////////////////

#if defined(CONFIG_MQTT_LIB_TLS)
static int certificates_provision(void)
{
	int err = 0;

	LOG_INF("Provisioning certificates");

#if defined(CONFIG_NRF_MODEM_LIB) && defined(CONFIG_MODEM_KEY_MGMT)

	err = modem_key_mgmt_write(CONFIG_MQTT_TLS_SEC_TAG,
							   MODEM_KEY_MGMT_CRED_TYPE_CA_CHAIN,
							   CA_CERTIFICATE,
							   strlen(CA_CERTIFICATE));
	if (err)
	{
		LOG_ERR("Failed to provision CA certificate: %d", err);
		return err;
	}

#elif defined(CONFIG_BOARD_QEMU_X86) && defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)

	err = tls_credential_add(CONFIG_MQTT_TLS_SEC_TAG,
							 TLS_CREDENTIAL_CA_CERTIFICATE,
							 CA_CERTIFICATE,
							 sizeof(CA_CERTIFICATE));
	if (err)
	{
		LOG_ERR("Failed to register CA certificate: %d", err);
		return err;
	}

#endif

	return err;
}
#endif /* defined(CONFIG_MQTT_LIB_TLS) */

#if defined(CONFIG_LWM2M_CARRIER)
K_SEM_DEFINE(carrier_registered, 0, 1);
int lwm2m_carrier_event_handler(const lwm2m_carrier_event_t *event)
{
	switch (event->type)
	{
	case LWM2M_CARRIER_EVENT_BSDLIB_INIT:
		LOG_INF("LWM2M_CARRIER_EVENT_BSDLIB_INIT");
		break;
	case LWM2M_CARRIER_EVENT_CONNECTING:
		LOG_INF("LWM2M_CARRIER_EVENT_CONNECTING");
		break;
	case LWM2M_CARRIER_EVENT_CONNECTED:
		LOG_INF("LWM2M_CARRIER_EVENT_CONNECTED");
		break;
	case LWM2M_CARRIER_EVENT_DISCONNECTING:
		LOG_INF("LWM2M_CARRIER_EVENT_DISCONNECTING");
		break;
	case LWM2M_CARRIER_EVENT_DISCONNECTED:
		LOG_INF("LWM2M_CARRIER_EVENT_DISCONNECTED");
		break;
	case LWM2M_CARRIER_EVENT_BOOTSTRAPPED:
		LOG_INF("LWM2M_CARRIER_EVENT_BOOTSTRAPPED");
		break;
	case LWM2M_CARRIER_EVENT_REGISTERED:
		LOG_INF("LWM2M_CARRIER_EVENT_REGISTERED");
		k_sem_give(&carrier_registered);
		break;
	case LWM2M_CARRIER_EVENT_DEFERRED:
		LOG_INF("LWM2M_CARRIER_EVENT_DEFERRED");
		break;
	case LWM2M_CARRIER_EVENT_FOTA_START:
		LOG_INF("LWM2M_CARRIER_EVENT_FOTA_START");
		break;
	case LWM2M_CARRIER_EVENT_REBOOT:
		LOG_INF("LWM2M_CARRIER_EVENT_REBOOT");
		break;
	case LWM2M_CARRIER_EVENT_ERROR:
		LOG_ERR("LWM2M_CARRIER_EVENT_ERROR: code %d, value %d",
				((lwm2m_carrier_event_error_t *)event->data)->code,
				((lwm2m_carrier_event_error_t *)event->data)->value);
		break;
	default:
		LOG_WRN("Unhandled LWM2M_CARRIER_EVENT: %d", event->type);
		break;
	}

	return 0;
}
#endif /* defined(CONFIG_LWM2M_CARRIER) */

/**@brief Function to print strings without null-termination
 */
static void data_print(uint8_t *prefix, uint8_t *data, size_t len)
{
	char buf[len + 1];

	memcpy(buf, data, len);
	buf[len] = 0;
	LOG_INF("%s%s", (char *)prefix, (char *)buf);
}

/**@brief Function to publish data on the configured topic
 */
static int data_publish(struct mqtt_client *c, enum mqtt_qos qos,
						uint8_t *data, size_t len)
{
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = CONFIG_MQTT_PUB_TOPIC;
	param.message.topic.topic.size = strlen(CONFIG_MQTT_PUB_TOPIC);
	param.message.payload.data = data;
	param.message.payload.len = len;
	param.message_id = sys_rand32_get();
	param.dup_flag = 0;
	param.retain_flag = 0;

	data_print("Publishing: ", data, len);
	LOG_INF("to topic: %s len: %u",
			CONFIG_MQTT_PUB_TOPIC,
			(unsigned int)strlen(CONFIG_MQTT_PUB_TOPIC));

	return mqtt_publish(c, &param);
}

/**@brief Function to subscribe to the configured topic
 */
static int subscribe(void)
{
	struct mqtt_topic subscribe_topic = {
		.topic = {
			.utf8 = CONFIG_MQTT_SUB_TOPIC,
			.size = strlen(CONFIG_MQTT_SUB_TOPIC)},
		.qos = MQTT_QOS_1_AT_LEAST_ONCE};

	const struct mqtt_subscription_list subscription_list = {
		.list = &subscribe_topic,
		.list_count = 1,
		.message_id = 1234};

	LOG_INF("Subscribing to: %s len %u", CONFIG_MQTT_SUB_TOPIC,
			(unsigned int)strlen(CONFIG_MQTT_SUB_TOPIC));

	return mqtt_subscribe(&client, &subscription_list);
}

/**@brief Function to read the published payload.
 */
static int publish_get_payload(struct mqtt_client *c, size_t length)
{
	int ret;
	int err = 0;

	/* Return an error if the payload is larger than the payload buffer.
	 * Note: To allow new messages, we have to read the payload before returning.
	 */
	if (length > sizeof(payload_buf))
	{
		err = -EMSGSIZE;
	}

	/* Truncate payload until it fits in the payload buffer. */
	while (length > sizeof(payload_buf))
	{
		ret = mqtt_read_publish_payload_blocking(
			c, payload_buf, (length - sizeof(payload_buf)));
		if (ret == 0)
		{
			return -EIO;
		}
		else if (ret < 0)
		{
			return ret;
		}

		length -= ret;
	}

	ret = mqtt_readall_publish_payload(c, payload_buf, length);
	if (ret)
	{
		return ret;
	}

	return err;
}

/**@brief MQTT client event handler
 */
void mqtt_evt_handler(struct mqtt_client *const c,
					  const struct mqtt_evt *evt)
{
	int err;

	switch (evt->type)
	{
	case MQTT_EVT_CONNACK:
		if (evt->result != 0)
		{
			LOG_ERR("MQTT connect failed: %d", evt->result);
			break;
		}

		LOG_INF("MQTT client connected");
		subscribe();
		break;

	case MQTT_EVT_DISCONNECT:
		LOG_INF("MQTT client disconnected: %d", evt->result);
		break;

	case MQTT_EVT_PUBLISH:
	{
		const struct mqtt_publish_param *p = &evt->param.publish;

		LOG_INF("MQTT PUBLISH result=%d len=%d",
				evt->result, p->message.payload.len);
		err = publish_get_payload(c, p->message.payload.len);

		if (p->message.topic.qos == MQTT_QOS_1_AT_LEAST_ONCE)
		{
			const struct mqtt_puback_param ack = {
				.message_id = p->message_id};

			/* Send acknowledgment. */
			mqtt_publish_qos1_ack(&client, &ack);
		}

		if (err >= 0)
		{
			data_print("Received: ", payload_buf,
					   p->message.payload.len);
			/* Echo back received data */
			// data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,//commented out to prevent echo
			// payload_buf, p->message.payload.len);
		}
		else if (err == -EMSGSIZE)
		{
			LOG_ERR("Received payload (%d bytes) is larger than the payload buffer "
					"size (%d bytes).",
					p->message.payload.len, sizeof(payload_buf));
		}
		else
		{
			LOG_ERR("publish_get_payload failed: %d", err);
			LOG_INF("Disconnecting MQTT client...");

			err = mqtt_disconnect(c);
			if (err)
			{
				LOG_ERR("Could not disconnect: %d", err);
			}
		}
	}
	break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0)
		{
			LOG_ERR("MQTT PUBACK error: %d", evt->result);
			break;
		}

		LOG_INF("PUBACK packet id: %u", evt->param.puback.message_id);
		break;

	case MQTT_EVT_SUBACK:
		if (evt->result != 0)
		{
			LOG_ERR("MQTT SUBACK error: %d", evt->result);
			break;
		}

		LOG_INF("SUBACK packet id: %u", evt->param.suback.message_id);
		break;

	case MQTT_EVT_PINGRESP:
		if (evt->result != 0)
		{
			LOG_ERR("MQTT PINGRESP error: %d", evt->result);
		}
		break;

	default:
		LOG_INF("Unhandled MQTT event type: %d", evt->type);
		break;
	}
}

/**@brief Resolves the configured hostname and
 * initializes the MQTT broker structure
 */
static int broker_init(void)
{
	int err;
	struct addrinfo *result;
	struct addrinfo *addr;
	struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM};

	err = getaddrinfo(CONFIG_MQTT_BROKER_HOSTNAME, NULL, &hints, &result); // this is producing a negative result for some reason
	if (err)
	{
		LOG_ERR("getaddrinfo failed: %d", err);
		return -ECHILD;
	}

	addr = result;

	/* Look for address of the broker. */
	while (addr != NULL)
	{
		/* IPv4 Address. */
		if (addr->ai_addrlen == sizeof(struct sockaddr_in))
		{
			struct sockaddr_in *broker4 =
				((struct sockaddr_in *)&broker);
			char ipv4_addr[NET_IPV4_ADDR_LEN];

			broker4->sin_addr.s_addr =
				((struct sockaddr_in *)addr->ai_addr)
					->sin_addr.s_addr;
			broker4->sin_family = AF_INET;
			broker4->sin_port = htons(CONFIG_MQTT_BROKER_PORT);

			inet_ntop(AF_INET, &broker4->sin_addr.s_addr,
					  ipv4_addr, sizeof(ipv4_addr));
			LOG_INF("IPv4 Address found %s", ipv4_addr);

			break;
		}
		else
		{
			LOG_ERR("ai_addrlen = %u should be %u or %u",
					(unsigned int)addr->ai_addrlen,
					(unsigned int)sizeof(struct sockaddr_in),
					(unsigned int)sizeof(struct sockaddr_in6));
		}

		addr = addr->ai_next;
	}

	/* Free the address. */
	freeaddrinfo(result);

	return err;
}

#if defined(CONFIG_NRF_MODEM_LIB)
#define IMEI_LEN 15
#define CGSN_RESPONSE_LENGTH (IMEI_LEN + 6 + 1) /* Add 6 for \r\nOK\r\n and 1 for \0 */
#define CLIENT_ID_LEN sizeof("nrf-") + IMEI_LEN
#else
#define RANDOM_LEN 10
#define CLIENT_ID_LEN sizeof(CONFIG_BOARD) + 1 + RANDOM_LEN
#endif /* defined(CONFIG_NRF_MODEM_LIB) */

/* Function to get the client id */
static const uint8_t *client_id_get(void)
{
	static uint8_t client_id[MAX(sizeof(CONFIG_MQTT_CLIENT_ID),
								 CLIENT_ID_LEN)];

	if (strlen(CONFIG_MQTT_CLIENT_ID) > 0)
	{
		snprintf(client_id, sizeof(client_id), "%s",
				 CONFIG_MQTT_CLIENT_ID);
		goto exit;
	}

#if defined(CONFIG_NRF_MODEM_LIB)
	char imei_buf[CGSN_RESPONSE_LENGTH + 1];
	int err;

	err = nrf_modem_at_cmd(imei_buf, sizeof(imei_buf), "AT+CGSN");
	if (err)
	{
		LOG_ERR("Failed to obtain IMEI, error: %d", err);
		goto exit;
	}

	imei_buf[IMEI_LEN] = '\0';

	snprintf(client_id, sizeof(client_id), "nrf-%.*s", IMEI_LEN, imei_buf);
#else
	uint32_t id = sys_rand32_get();
	snprintf(client_id, sizeof(client_id), "%s-%010u", CONFIG_BOARD, id);
#endif /* !defined(NRF_CLOUD_CLIENT_ID) */

exit:
	LOG_DBG("client_id = %s", (char *)client_id);

	return client_id;
}

/**@brief Initialize the MQTT client structure
 */
static int client_init(struct mqtt_client *client)
{
	int err;

	mqtt_client_init(client);

	err = broker_init();
	if (err)
	{
		LOG_ERR("Failed to initialize broker connection");
		return err;
	}

	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_evt_handler;
	client->client_id.utf8 = client_id_get();
	client->client_id.size = strlen(client->client_id.utf8);
	client->password = NULL;
	client->user_name = NULL;
	client->protocol_version = MQTT_VERSION_3_1_1;

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	/* MQTT transport configuration */
#if defined(CONFIG_MQTT_LIB_TLS)
	struct mqtt_sec_config *tls_cfg = &(client->transport).tls.config;
	static sec_tag_t sec_tag_list[] = {CONFIG_MQTT_TLS_SEC_TAG};

	LOG_INF("TLS enabled");
	client->transport.type = MQTT_TRANSPORT_SECURE;

	tls_cfg->peer_verify = CONFIG_MQTT_TLS_PEER_VERIFY;
	tls_cfg->cipher_count = 0;
	tls_cfg->cipher_list = NULL;
	tls_cfg->sec_tag_count = ARRAY_SIZE(sec_tag_list);
	tls_cfg->sec_tag_list = sec_tag_list;
	tls_cfg->hostname = CONFIG_MQTT_BROKER_HOSTNAME;

#if defined(CONFIG_NRF_MODEM_LIB)
	tls_cfg->session_cache = IS_ENABLED(CONFIG_MQTT_TLS_SESSION_CACHING) ? TLS_SESSION_CACHE_ENABLED : TLS_SESSION_CACHE_DISABLED;
#else
	/* TLS session caching is not supported by the Zephyr network stack */
	tls_cfg->session_cache = TLS_SESSION_CACHE_DISABLED;

#endif

#else
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
#endif

	return err;
}

/**@brief Initialize the file descriptor structure used by poll.
 */
static int fds_init(struct mqtt_client *c)
{
	if (c->transport.type == MQTT_TRANSPORT_NON_SECURE)
	{
		fds.fd = c->transport.tcp.sock;
	}
	else
	{
#if defined(CONFIG_MQTT_LIB_TLS)
		fds.fd = c->transport.tls.sock;
#else
		return -ENOTSUP;
#endif
	}

	fds.events = POLLIN;

	return 0;
}

#if defined(CONFIG_DK_LIBRARY)
static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	if (has_changed & button_states &
		BIT(CONFIG_BUTTON_EVENT_BTN_NUM - 1))
	{
		int ret;

		ret = data_publish(&client,
						   MQTT_QOS_1_AT_LEAST_ONCE,
						   "oof,beans",				  // this is the message it is sending(its a string)/ OG: CONFIG_BUTTON_EVENT_PUBLISH_MSG
						   sizeof("oof, beans") - 1); // OG:
		if (ret)
		{
			LOG_ERR("Publish failed: %d", ret);
		}
	}
}
#endif

/**@brief Configures modem to provide LTE link. Blocks until link is
 * successfully established.
 */
static int modem_configure(void)
{
#if defined(CONFIG_LTE_LINK_CONTROL)
	/* Turn off LTE power saving features for a more responsive demo. Also,
	 * request power saving features before network registration. Some
	 * networks rejects timer updates after the device has registered to the
	 * LTE network.
	 */
	LOG_INF("Disabling PSM and eDRX");
	lte_lc_psm_req(true); // changing both psm and edrx to true allowed me to create a fix and get the MQTT working
	lte_lc_edrx_req(true);

	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT))
	{
		/* Do nothing, modem is already turned on
		 * and connected.
		 */
	}
	else
	{
#if defined(CONFIG_LWM2M_CARRIER)
		/* Wait for the LWM2M_CARRIER to configure the modem and
		 * start the connection.
		 */
		LOG_INF("Waitng for carrier registration...");
		k_sem_take(&carrier_registered, K_FOREVER);
		LOG_INF("Registered!");
#else  /* defined(CONFIG_LWM2M_CARRIER) */
		int err;

		LOG_INF("LTE Link Connecting...");
		err = lte_lc_init_and_connect();
		if (err)
		{
			LOG_INF("Failed to establish LTE connection: %d", err);
			return err;
		}
		LOG_INF("LTE Link Connected!");
#endif /* defined(CONFIG_LWM2M_CARRIER) */
	}
#endif /* defined(CONFIG_LTE_LINK_CONTROL) */

	return 0;
}

// Start: MQTT functionality function////////////////////////////////////////////////////////////

static void pass_lat_lon_MQTT(float arr[])
{
	LOG_INF("test function to send lat lon/////////////////////////////////////////////////////////////");
	int counter;
	int err;
	uint32_t connect_attempt = 0;

#if defined(CONFIG_MQTT_LIB_TLS)
	err = certificates_provision();
	if (err != 0)
	{
		LOG_ERR("Failed to provision certificates");
		return;
	}

#endif // defined(CONFIG_MQTT_LIB_TLS)
/*
while (counter > 100)//this loop breaks the program
{
	counter = counter + 1;
	LOG_ERR("should be looping");
}
*/

/*
	do // this is what is making it break
	{
		err = modem_configure(); // OG: err = modem_configure();
		if (err)
		{
			LOG_INF("Retrying in %d seconds",
					CONFIG_LTE_CONNECT_RETRY_DELAY_S);
			k_sleep(K_SECONDS(CONFIG_LTE_CONNECT_RETRY_DELAY_S));
		}
	} while (err);

	err = client_init(&client);
	if (err != 0)
	{
		LOG_ERR("client_init: %d", err);
		return;
	}

#if defined(CONFIG_DK_LIBRARY)
	dk_buttons_init(button_handler);
#endif
*/
do_connect:
	if (connect_attempt++ > 0)
	{
		LOG_INF("Reconnecting in %d seconds...",
				CONFIG_MQTT_RECONNECT_DELAY_S);
		k_sleep(K_SECONDS(CONFIG_MQTT_RECONNECT_DELAY_S));
	}
	err = mqtt_connect(&client);
	if (err != 0)
	{
		LOG_ERR("mqtt_connect %d", err);
		goto do_connect;
	}

	err = fds_init(&client);
	if (err != 0)
	{
		LOG_ERR("fds_init: %d", err);
		return;
	}

	while (1)
	{
		int i = 0;
		i = i++;

		err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
		if (err < 0)
		{
			LOG_ERR("poll: %d", errno);
			break;
		}

		err = mqtt_live(&client);
		if ((err != 0) && (err != -EAGAIN))
		{
			LOG_ERR("ERROR: mqtt_live: %d", err);
			break;
		}

		if ((fds.revents & POLLIN) == POLLIN)
		{
			err = mqtt_input(&client);
			if (err != 0)
			{
				LOG_ERR("mqtt_input: %d", err);
				break;
			}
		}

		if ((fds.revents & POLLERR) == POLLERR)
		{
			LOG_ERR("POLLERR");
			break;
		}

		if ((fds.revents & POLLNVAL) == POLLNVAL)
		{
			LOG_ERR("POLLNVAL");
			break;
		}

		// on pause till if figure out how to successfully send one message
		int ret;

		// char message1[15] = ("%.06f", arr[0]);
		char message1[10];
		char message2[11];
		gcvt(arr[0], 6, message1);
		gcvt(arr[1], 6, message2);
		// uint8_t message1;
		//  char message2[15] = "%.06f", arr[1];
		//   strcat(message1, message1);
		ret = data_publish(&client,
						   MQTT_QOS_1_AT_LEAST_ONCE,
						   ("%s,%s", message1, message2), // this is the message it is sending(its a string)/ OG: CONFIG_BUTTON_EVENT_PUBLISH_MSG
						   sizeof(message1) - 1);
		if (ret)
		{
			LOG_ERR("Publish failed: %d", ret);
		}

		if (ret != -128)
		{
			LOG_INF("ret: %d", ret);
			break; // added
		}

		if (i > 60)
		{
			LOG_INF("counter should be > 20: %d", i);
			break;
		}
	}

	LOG_INF("Disconnecting MQTT client...");

	err = mqtt_disconnect(&client);
	if (err)
	{
		LOG_ERR("Could not disconnect MQTT client: %d", err);
	}
	// goto do_connect;
}
// End: MQTT functionality function//////////////////////////////////////////////////////////////

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data, float arr[]) //, float arr[]
{
	LOG_INF("Latitude:       %.06f", pvt_data->latitude);
	LOG_INF("Longitude:      %.06f", pvt_data->longitude);
	LOG_INF("Altitude:       %.01f m", pvt_data->altitude);
	LOG_INF("Time (UTC):     %02u:%02u:%02u.%03u",
			pvt_data->datetime.hour,
			pvt_data->datetime.minute,
			pvt_data->datetime.seconds,
			pvt_data->datetime.ms);

	// latCoord = pvt_data->latitude;
	// lonCoord = pvt_data->longitude;

	//*latitude_uni = (pvt_data->latitude);
	//*longitude_uni = (pvt_data->longitude);
}

static void gnss_event_handler(int event) // loops for some reason
{
	int err;
	float arr[2];
	int retval;
	// nrf_modem_gnss_prio_mode_enable();
	//  Note: after each event triggers, the main code (above) triggers too
	switch (event)
	{
	/* STEP 7 - On a PVT event, confirm if PVT data is a valid fix */
	case NRF_MODEM_GNSS_EVT_PVT:
		LOG_INF("Searching..."); // prints searching
		/* STEP 15 - Print satellite information */
		int num_satellites = 0;
		for (int i = 0; i < 12; i++)
		{
			if (pvt_data.sv[i].signal != 0)
			{
				LOG_INF("sv: %d, cn0: %d", pvt_data.sv[i].sv, pvt_data.sv[i].cn0);
				num_satellites++;
			}
		}
		LOG_INF("Number of current satellites: %d", num_satellites); // prints number of satellites
		err = nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT);
		if (err)
		{
			LOG_ERR("nrf_modem_gnss_read failed, err %d", err);
			return;
		}
		// LOG_INF("priority result: %d //////////////////", );										// prints searching
		LOG_INF("NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID result: %d", NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID); // prints searching
		// if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID)										// pvt_data.flags &
		//{

		retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
		if (retval == 0)
		{
			dk_set_led_on(DK_LED1);
			current_pvt = last_pvt;
			// LOG_INF("current pvt: %f", current_pvt);
			//  LOG_INF("last pvt: %f", last_pvt);
			LOG_INF("current_pvt.latitude: %f", current_pvt.latitude);
			LOG_INF("current_pvt.latitude: %f", last_pvt.latitude);
			print_fix_data(&current_pvt, arr);
			// print_fix_data(&pvt_data, arr);
			LOG_INF("Time to first fix: %2.1lld s", (k_uptime_get() - gnss_start_time) / 1000);
			/* STEP 12.3 - Print the time to first fix */
			if (!first_fix)
			{
				LOG_INF("Time to first fix: %2.1lld s", (k_uptime_get() - gnss_start_time) / 1000);
				first_fix = true;
			}
			return;
		}
		break;
	/* STEP 7.2 - Log when the GNSS sleeps and wakes up */
	case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
		LOG_INF("GNSS has woken up");
		break;
	case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
		LOG_INF("GNSS enter sleep after fix");
		// nrf_modem_gnss_stop();
		break;
	default:
		break;
	}
}

void main(void)
{

	int err;
	uint32_t connect_attempt = 0;
	float latitude_uni = 0.0;
	float longitude_uni = 0.0;
	// latCoord = 1.0;
	// lonCoord = 1.0;
	float arr[2];
	arr[0] = 1.345554;
	LOG_INF("The MQTT simple sample started");

	// test passing and changing lat lon floats with pointers (it works)
	/*
	LOG_INF("after first function, should be zeroes: %.06f,%.06f", latitude_uni, latitude_uni);
	pointerPracticeMQTTFunction(&latitude_uni, &longitude_uni);
	LOG_INF("after first function, should be lat: %.06f", latitude_uni);
	LOG_INF("after first function, should be lon: %.06f", longitude_uni);
	*/

	// test to see if mqtt function will work here
	// pass_lat_lon_MQTT(arr);

	// Start: GNSS Simple main code////////////////////////////////////////////////////////////

	if (dk_leds_init() != 0)
	{
		LOG_ERR("Failed to initialize the LEDs Library");
	}

	// modem_configure_2();
	do // commenting this out allowed the gps to get a fix but mqtt wont connect to broker
	{
		err = modem_configure();
		if (err)
		{
			LOG_INF("Retrying in %d seconds",
					CONFIG_LTE_CONNECT_RETRY_DELAY_S);
			k_sleep(K_SECONDS(CONFIG_LTE_CONNECT_RETRY_DELAY_S));
		}
	} while (err);

	// STEP 8 - Activate modem and deactivate LTE
	if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL) != 0)
	{
		LOG_ERR("Failed to activate GNSS functional mode");
		return;
	}
	/*
		// deactivating lte
		LOG_INF("Deactivating LTE"); // deactivating lte is causing us to not get a fix, but turning it off doesn't allow a gps fix
		if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE) != 0)
		{
			LOG_ERR("Failed to activate GNSS functional mode");
			return;
		}
	*/
	// STEP 9 - Register the GNSS event handler

	if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0)
	{
		LOG_ERR("Failed to set GNSS event handler");
		return;
	}

	/*
		if (nrf_modem_gnss_event_handler_set(gnss_event_handler) == 0) // eliminates initial print of empty coordinates
		{
			// print_fix_data(&pvt_data, arr);
			latitude_uni = arr[0];
			longitude_uni = arr[1];
			LOG_INF("after handler function, should be lat: %.06f", latitude_uni);
			LOG_INF("after handler function, should be lon: %.06f", longitude_uni);
		}
	*/

	// STEP 10 - Set the GNSS fix interval and GNSS fix retry period
	if (nrf_modem_gnss_fix_interval_set(CONFIG_GNSS_PERIODIC_INTERVAL) != 0)
	{
		LOG_ERR("Failed to set GNSS fix interval");
		return;
	}

	LOG_INF("first/////////////////////////////////////////");

	if (nrf_modem_gnss_fix_retry_set(CONFIG_GNSS_PERIODIC_TIMEOUT) != 0)
	{
		LOG_ERR("Failed to set GNSS fix retry");
		return;
	}

	LOG_INF("second/////////////////////////////////////////");
	/*
		// STEP 11 - Start the GNSS receiver
		LOG_INF("Starting GNSS");
		if (nrf_modem_gnss_start() != 0)
		{
			LOG_ERR("Failed to start GNSS");
			return;
		}
		*/

	nrf_modem_gnss_start();

	LOG_INF("third/////////////////////////////////////////");

	// STEP 12.2 - Log the current system uptime
	gnss_start_time = k_uptime_get();

#if defined(CONFIG_MQTT_LIB_TLS)
	err = certificates_provision();
	if (err != 0)
	{
		LOG_ERR("Failed to provision certificates");
		return;
	}
#endif // defined(CONFIG_MQTT_LIB_TLS)
	   /*
		   do // commenting this out allowed the gps to get a fix but mqtt wont connect to broker
		   {
			   err = modem_configure();
			   if (err)
			   {
				   LOG_INF("Retrying in %d seconds",
						   CONFIG_LTE_CONNECT_RETRY_DELAY_S);
				   k_sleep(K_SECONDS(CONFIG_LTE_CONNECT_RETRY_DELAY_S));
			   }
		   } while (err);
	   */
	err = client_init(&client);
	if (err != 0)
	{
		LOG_ERR("client_init: %d", err);
		return;
	}

#if defined(CONFIG_DK_LIBRARY)
	dk_buttons_init(button_handler);
#endif

do_connect:
	if (connect_attempt++ > 0)
	{
		LOG_INF("Reconnecting in %d seconds...",
				CONFIG_MQTT_RECONNECT_DELAY_S);
		k_sleep(K_SECONDS(CONFIG_MQTT_RECONNECT_DELAY_S));
	}
	err = mqtt_connect(&client);
	if (err != 0)
	{
		LOG_ERR("mqtt_connect %d", err);
		goto do_connect;
	}

	err = fds_init(&client);
	if (err != 0)
	{
		LOG_ERR("fds_init: %d", err);
		return;
	}

	while (1)
	{
		int i = 0;
		i = i++;

		err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
		if (err < 0)
		{
			LOG_ERR("poll: %d", errno);
			break;
		}

		err = mqtt_live(&client);
		if ((err != 0) && (err != -EAGAIN))
		{
			LOG_ERR("ERROR: mqtt_live: %d", err);
			break;
		}

		if ((fds.revents & POLLIN) == POLLIN)
		{
			err = mqtt_input(&client);
			if (err != 0)
			{
				LOG_ERR("mqtt_input: %d", err);
				break;
			}
		}

		if ((fds.revents & POLLERR) == POLLERR)
		{
			LOG_ERR("POLLERR");
			break;
		}

		if ((fds.revents & POLLNVAL) == POLLNVAL)
		{
			LOG_ERR("POLLNVAL");
			break;
		}

		int ret;
		char message2[9];
		char messageB[10];
		float message1;
		float messageA;
		float num = current_pvt.latitude;
		// sprintf(message2, "%f", current_pvt.latitude);
		LOG_ERR("before conversion message: %f", current_pvt.latitude); // this works
		LOG_ERR("after conversion message: %d", message1);
		message1 = current_pvt.latitude;
		messageA = current_pvt.longitude;

		// uint8_t message1;
		// char message2[15] = "%.06f", arr[1];
		// snprintf(message2, 50, "%f", num);//causing some weird os error
		gcvt(message1, 9, message2);
		gcvt(messageA, 10, messageB);

		LOG_ERR("after conversion message: %d", message1);
		if (message1 >= 1.0)
		{
			ret = data_publish(&client,
							   MQTT_QOS_1_AT_LEAST_ONCE,
							   ("%d", message2), // this is the message it is sending(its a string)/ OG: CONFIG_BUTTON_EVENT_PUBLISH_MSG
							   sizeof(message2));

			ret = data_publish(&client,
							   MQTT_QOS_1_AT_LEAST_ONCE,
							   ("%d", messageB), // this is the message it is sending(its a string)/ OG: CONFIG_BUTTON_EVENT_PUBLISH_MSG
							   sizeof(messageB));
		}

		if (ret)
		{
			LOG_ERR("Publish failed: %d", ret);
		}

		if (ret != -128)
		{
			LOG_INF("ret: %d", ret);
			break; // added
		}
	}

	LOG_INF("Disconnecting MQTT client...");

	err = mqtt_disconnect(&client);
	if (err)
	{
		LOG_ERR("Could not disconnect MQTT client: %d", err);
	}
	goto do_connect;
}
/*
//practice nested function for changing values
static void pointerPracticeNestedFunction(float *latitude_uni, float *longitude_uni, float arr[])
{

	arr[0] = 4.55440382;
	arr[1] = -7.83274926;

	return 0;
}

//practice first functions for passing values
static void pointerPracticeMQTTFunction(float *latitude_uni, float *longitude_uni)
{
	float arr[2]; // array of size 2 (2 variables)

	LOG_INF("after first function, should be 1.175484836474: %.06f", *latitude_uni);
	LOG_INF("after first function, should be -2.8273272849: %.06f", *longitude_uni);

	pointerPracticeNestedFunction(&latitude_uni, &longitude_uni, arr); // pointerPracticeNestedFunction has to be declared above this function, otherwise: implecit declaration error

	*latitude_uni = arr[0];
	*longitude_uni = arr[1];

	return 0;
}
*/