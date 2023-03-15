/*
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    mqtt_functions.c
 * @brief   Application entry point.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "board.h"
#include "lwip/opt.h"
#include "mqtt_functions.h"
#include "gpio_functions.h"
#include "fsl_debug_console.h"


#if LWIP_IPV4 && LWIP_RAW && LWIP_NETCONN && LWIP_DHCP && LWIP_DNS

#include "lwip/api.h"
#include "lwip/dhcp.h"
#include "lwip/netdb.h"
#include "lwip/tcpip.h"
#include "lwip_mqtt_id.h"
#include "lwip/timeouts.h"
#include "lwip/netifapi.h"
#include "netif/ethernet.h"
#include "lwip/prot/dhcp.h"
#include "lwip/apps/mqtt.h"
#include "enet_ethernetif.h"

#include "ctype.h"
#include "stdio.h"

#include "fsl_phy.h"
#include "fsl_enet_mdio.h"
#include "fsl_phyksz8081.h"
#include "fsl_device_registers.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* @TEST_ANCHOR */

/* MAC address configuration. */
#ifndef configMAC_ADDR
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x07 \
    }
#endif

/* Address of PHY Interface */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* MDIO Operations */
#define EXAMPLE_MDIO_OPS enet_ops

/* PHY Operations */
#define EXAMPLE_PHY_OPS phyksz8081_ops

/* ENET Clock Frequency */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)

#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network Interface Initialization Function */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

/*! @brief MQTT server host name or IP address. */
#define EXAMPLE_MQTT_SERVER_HOST "driver.cloudmqtt.com"

/*! @brief MQTT server port number. */
#define EXAMPLE_MQTT_SERVER_PORT 18591

/*! @brief Stack size of the temporary initialization thread. */
#define APP_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary initialization thread. */
#define APP_THREAD_PRIO DEFAULT_THREAD_PRIO

/* Maximum Allowed Messages */
#ifndef MAX_MESSAGES
#define MAX_MESSAGES 10
#endif

/* Maximum Allowed Messages */
#ifndef MAX_NUM_OF_CHARS
#define MAX_NUM_OF_CHARS 50
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t  phyHandle  = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};

/*! @brief MQTT client data. */
static mqtt_client_t *mqtt_client;

/*! @brief MQTT client ID string. */
static char client_id[40];

/*! @brief MQTT client information. */
static const struct mqtt_connect_client_info_t mqtt_client_info = {
    .client_id   = (const char *)&client_id[0],
    .client_user = "Jorge",
    .client_pass = "clase2023",
    .keep_alive  = 100,
    .will_topic  = NULL,
    .will_msg    = NULL,
    .will_qos    = 0,
    .will_retain = 0,
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    .tls_config = NULL,
#endif
};

/*! @brief MQTT broker IP address. */
static ip_addr_t mqtt_addr;

/*! @brief Indicates connection to MQTT broker. */
static volatile bool connected = false;

/* Whether the SW button is pressed */
extern volatile bool g_Sw2Press;
extern volatile bool g_Sw3Press;

uint8_t ARRAY_SIZE = 10;
uint8_t isThereGas = 0;
uint8_t indx = 0;

static char currentValues [MAX_MESSAGES][MAX_NUM_OF_CHARS] = {"10", "500", "750", "310", "200", "550", "820", "120", "70", "960"};
static char temperatureValues [MAX_MESSAGES][MAX_NUM_OF_CHARS] = {"10", "52", "45", "82", "73", "40", "22", "38", "65", "13"};

/*******************************************************************************
 * Private Prototypes
 ******************************************************************************/
static void connect_to_mqtt (void *ctx);

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*!
 * @brief Called when subscription request finishes.
 */
static void mqtt_topic_subscribed_cb (void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF ("Subscribed to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF ("Failed to subscribe to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Called when there is a message on a subscribed topic.
 */
static void mqtt_incoming_publish_cb (void *arg, const char *topic, u32_t tot_len)
{
    LWIP_UNUSED_ARG (arg);

    PRINTF ("Received %u bytes from the topic \"%s\": \"", tot_len, topic);
}

/*!
 * @brief Called when received incoming published message fragment.
 */
static void mqtt_incoming_data_cb (void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    int i;

    LWIP_UNUSED_ARG (arg);

    if (data[0] >= '0' && data[0] <= '9')
    	PRINTF ("Motor Speed: Duty Cycle = ");


    for (i = 0; i < len; i++)
    {
        if (isprint (data[i]))
        {
            PRINTF ("%c", (char)data[i]);
        }
        else
        {
            PRINTF ("\\x%02x", data[i]);
        }
    }

    if (data[0] >= '0' && data[0] <= '9')
    	PRINTF ("%%");

    if (flags & MQTT_DATA_FLAG_LAST)
    {
        PRINTF ("\"\r\n");
    }

    if (data[6] == 'F')
    {
    	LED_Off ( );
    	LED_Blue ( );
    }
    else if (data[6] == 'B')
    {
    	LED_Off ( );
    	LED_Red ( );
    }
    else if (data[6] == 'L')
    {
    	LED_Off ( );
    	LED_Yellow ( );
    }
    else if (data[6] == 'R')
    {
    	LED_Off ( );
    	LED_Purple ( );
    }
}

/*!
 * @brief Subscribe to MQTT topics.
 */
static void mqtt_subscribe_topics (mqtt_client_t *client)
{
    static const char *topics[] = {"rescue/speed", "rescue/direction"};
    int qos[]                   = {1, 1};
    err_t err;
    int i;

    mqtt_set_inpub_callback (client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, LWIP_CONST_CAST (void *, &mqtt_client_info));

    for (i = 0; i < ARRAY_SIZE (topics); i++)
    {
        err = mqtt_subscribe (client, topics[i], qos[i], mqtt_topic_subscribed_cb, LWIP_CONST_CAST (void *, topics[i]));

        if (err == ERR_OK)
        {
            PRINTF ("Subscribing to the topic \"%s\" with QoS %d...\r\n", topics[i], qos[i]);
        }
        else
        {
            PRINTF ("Failed to subscribe to the topic \"%s\" with QoS %d: %d.\r\n", topics[i], qos[i], err);
        }
    }
}

/*!
 * @brief Called when connection state changes.
 */
static void mqtt_connection_cb (mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    const struct mqtt_connect_client_info_t *client_info = (const struct mqtt_connect_client_info_t *)arg;

    connected = (status == MQTT_CONNECT_ACCEPTED);

    switch (status)
    {
        case MQTT_CONNECT_ACCEPTED:
            PRINTF ("MQTT client \"%s\" connected.\r\n", client_info -> client_id);
            mqtt_subscribe_topics (client);
            break;

        case MQTT_CONNECT_DISCONNECTED:
            PRINTF ("MQTT client \"%s\" not connected.\r\n", client_info -> client_id);
            /* Try to reconnect 1 second later */
            sys_timeout (1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_TIMEOUT:
            PRINTF("MQTT client \"%s\" connection timeout.\r\n", client_info -> client_id);
            /* Try again 1 second later */
            sys_timeout (1000, connect_to_mqtt, NULL);
            break;

        case MQTT_CONNECT_REFUSED_PROTOCOL_VERSION:
        case MQTT_CONNECT_REFUSED_IDENTIFIER:
        case MQTT_CONNECT_REFUSED_SERVER:
        case MQTT_CONNECT_REFUSED_USERNAME_PASS:
        case MQTT_CONNECT_REFUSED_NOT_AUTHORIZED_:
            PRINTF ("MQTT client \"%s\" connection refused: %d.\r\n", client_info -> client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout (10000, connect_to_mqtt, NULL);
            break;

        default:
            PRINTF ("MQTT client \"%s\" connection status: %d.\r\n", client_info -> client_id, (int)status);
            /* Try again 10 seconds later */
            sys_timeout (10000, connect_to_mqtt, NULL);
            break;
    }
}

/*!
 * @brief Starts connecting to MQTT broker. To be called on tcpip_thread.
 */
static void connect_to_mqtt (void *ctx)
{
    LWIP_UNUSED_ARG (ctx);

    PRINTF ("Connecting to MQTT broker at %s...\r\n", ipaddr_ntoa (&mqtt_addr));

    mqtt_client_connect (mqtt_client, &mqtt_addr, EXAMPLE_MQTT_SERVER_PORT, mqtt_connection_cb, LWIP_CONST_CAST (void *, &mqtt_client_info), &mqtt_client_info);
}

/*!
 * @brief Called when publish request finishes.
 */
static void mqtt_message_published_cb (void *arg, err_t err)
{
    const char *topic = (const char *)arg;

    if (err == ERR_OK)
    {
        PRINTF ("Published to the topic \"%s\".\r\n", topic);
    }
    else
    {
        PRINTF ("Failed to publish to the topic \"%s\": %d.\r\n", topic, err);
    }
}

/*!
 * @brief Publishes a message. To be called on tcpip_thread.
 */
static void publish_message (void *ctx)
{
	static const char *topics[] = {"rescue/gas", "rescue/current", "rescue/temperature"};
	int qos[]                   = {1, 1, 1};

	static const char *message1 = "Alarm: Gas";
	static const char *message2 = "No Gas";
	char *message3 = &currentValues[indx][0];
	char *message4 = &temperatureValues[indx][0];

	LWIP_UNUSED_ARG(ctx);

	if (g_Sw2Press)
	{
		isThereGas = isThereGas^1;

		if (isThereGas)
		{
			LED_Off ( );
			LED_Aqua ( );
			mqtt_publish (mqtt_client, topics[0], message1, strlen(message1), qos[0], 0, mqtt_message_published_cb, (void *)topics[0]);
		}

		else
		{
			LED_Off ( );
			mqtt_publish (mqtt_client, topics[0], message2, strlen(message2), qos[0], 0, mqtt_message_published_cb, (void *)topics[0]);
		}

		/* Reset state of button. */
		g_Sw2Press = false;
	}

	if (g_Sw3Press)
	{
		mqtt_publish (mqtt_client, topics[1], message3, strlen(message3), qos[1], 0, mqtt_message_published_cb, (void *)topics[1]);
		mqtt_publish (mqtt_client, topics[2], message4, strlen(message4), qos[2], 0, mqtt_message_published_cb, (void *)topics[2]);

		/* Reset state of button. */
		g_Sw3Press = false;

		/* Update index */
		if (indx < (ARRAY_SIZE - 1))
			indx ++;
		else
			indx = 0;
	}
}

static void generate_client_id (void)
{
    uint32_t mqtt_id[MQTT_ID_SIZE];
    int res;

    get_mqtt_id (&mqtt_id[0]);

    res = snprintf (client_id, sizeof (client_id), "nxp_%08lx%08lx%08lx%08lx", mqtt_id[3], mqtt_id[2], mqtt_id[1], mqtt_id[0]);

    if ((res < 0) || (res >= sizeof (client_id)))
    {
        PRINTF("snprintf failed: %d\r\n", res);

        while (1)
        {
        }
    }
}

/*******************************************************************************
 * Functions
 ******************************************************************************/
/*!
 * @brief Rescue Robot.
 */
void rescue_app (err_t err)
{
    if (err == ERR_OK)
    {
        /* Start connecting to MQTT broker from tcpip_thread */
        err = tcpip_callback (connect_to_mqtt, NULL);
        if (err != ERR_OK)
        {
            PRINTF ("Failed to invoke broker connection on the tcpip_thread: %d.\r\n", err);
        }
    }
    else
    {
        PRINTF ("Failed to obtain IP address: %d.\r\n", err);
    }

	while (1)
	{
	    if (connected)
	    {
	        err = tcpip_callback (publish_message, NULL);
	        if (err != ERR_OK)
	        {
	           PRINTF ("Failed to invoke publishing of a message on the tcpip_thread: %d.\r\n", err);
	        }
	    }

	    sys_msleep (1000U);
	}
}

/*!
 * @brief Application thread.
 */
static void app_thread (void *arg)
{
    struct netif *netif = (struct netif *)arg;
    struct dhcp *dhcp;
    err_t err;

    /* Wait for address from DHCP */
    PRINTF ("Getting IP address from DHCP...\r\n");

    do
    {
        if (netif_is_up(netif))
        {
            dhcp = netif_dhcp_data(netif);
        }
        else
        {
            dhcp = NULL;
        }

        sys_msleep (20U);

    } while ((dhcp == NULL) || (dhcp->state != DHCP_STATE_BOUND));

    PRINTF ("\r\nIPv4 Address : %s\r\n", ipaddr_ntoa (&netif -> ip_addr));
    PRINTF ("IPv4 Subnet mask : %s\r\n", ipaddr_ntoa (&netif -> netmask));
    PRINTF ("IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa (&netif -> gw));

    /*
     * Check if we have an IP address or host name string configured.
     * Could just call netconn_gethostbyname() on both IP address or host name,
     * but we want to print some info if goint to resolve it.
     */
    if (ipaddr_aton (EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr) && IP_IS_V4 (&mqtt_addr))
    {
        /* Already an IP address */
        err = ERR_OK;
    }
    else
    {
        /* Resolve MQTT broker's host name to an IP address */
        PRINTF ("Resolving \"%s\"...\r\n", EXAMPLE_MQTT_SERVER_HOST);
        err = netconn_gethostbyname (EXAMPLE_MQTT_SERVER_HOST, &mqtt_addr);
    }

    /* Run Rescue Robot Application */
    rescue_app ( err );

    vTaskDelete (NULL);
}

/*!
 * @brief MQTT Initialization.
 */
void mqtt_init (void)
{
    static struct netif netif;
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;

    ethernetif_config_t enet_config = {
        .phyHandle  = &phyHandle,
        .macAddress = configMAC_ADDR,
    };

    generate_client_id ( );

    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    IP4_ADDR (&netif_ipaddr, 0U, 0U, 0U, 0U);
    IP4_ADDR (&netif_netmask, 0U, 0U, 0U, 0U);
    IP4_ADDR (&netif_gw, 0U, 0U, 0U, 0U);

    tcpip_init (NULL, NULL);

    LOCK_TCPIP_CORE ( );
    mqtt_client = mqtt_client_new ( );
    UNLOCK_TCPIP_CORE ( );

    if (mqtt_client == NULL)
    {
        PRINTF ("mqtt_client_new() failed.\r\n");

        while (1)
        {
        }
    }

    netifapi_netif_add (&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN, tcpip_input);
    netifapi_netif_set_default (&netif);
    netifapi_netif_set_up (&netif);

    netifapi_dhcp_start (&netif);

    PRINTF ("\r\n************************************************\r\n");
    PRINTF (" MQTT client example\r\n");
    PRINTF ("************************************************\r\n");

    if (sys_thread_new ("app_task", app_thread, &netif, APP_THREAD_STACKSIZE, APP_THREAD_PRIO) == NULL)
    {
        LWIP_ASSERT ("stack_init(): Task creation failed.", 0);
    }
}

#endif /* LWIP_IPV4 && LWIP_RAW && LWIP_NETCONN && LWIP_DHCP && LWIP_DNS */

