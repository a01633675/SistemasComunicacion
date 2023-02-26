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
 * @file    my_layer.c
 * @brief   Application entry point.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "board.h"
#include "fsl_aes.h"
#include "fsl_crc.h"
#include "lwip/opt.h"
#include "fsl_debug_console.h"

#include "my_layer.h"

#if LWIP_NETCONN

#include "tcpecho.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/netifapi.h"
#include "netif/ethernet.h"
#include "enet_ethernetif.h"

#include "fsl_phy.h"
#include "fsl_enet_mdio.h"
#include "fsl_phyksz8081.h"
#include "fsl_device_registers.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* @TEST_ANCHOR */

/* IP Address Configuration */
#ifndef CRC_SIZE
#define CRC_SIZE 4
#endif

/* IP Address Configuration */
#ifndef configIP_PORT
#define configIP_PORT 7
#endif

/* Maximum Allowed Messages */
#ifndef MAX_MESSAGES
#define MAX_MESSAGES 8
#endif

/* Maximum Allowed Messages */
#ifndef MAX_NUM_OF_CHARS
#define MAX_NUM_OF_CHARS 100
#endif

/* IP Address Configuration */
#ifndef configIP_ADDR0
#define configIP_ADDR0 192
#endif
#ifndef configIP_ADDR1
#define configIP_ADDR1 168
#endif
#ifndef configIP_ADDR2
#define configIP_ADDR2 0
#endif
#ifndef configIP_ADDR3
#define configIP_ADDR3 102
#endif

/* Network Mask Configuration */
#ifndef configNET_MASK0
#define configNET_MASK0 255
#endif
#ifndef configNET_MASK1
#define configNET_MASK1 255
#endif
#ifndef configNET_MASK2
#define configNET_MASK2 255
#endif
#ifndef configNET_MASK3
#define configNET_MASK3 0
#endif

/* Gateway Address Configuration */
#ifndef configGW_ADDR0
#define configGW_ADDR0 192
#endif
#ifndef configGW_ADDR1
#define configGW_ADDR1 168
#endif
#ifndef configGW_ADDR2
#define configGW_ADDR2 0
#endif
#ifndef configGW_ADDR3
#define configGW_ADDR3 100
#endif

/* MAC Address Configuration */
#ifndef configMAC_ADDR
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x11 \
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

/*******************************************************************************
 * Variables
 ******************************************************************************/
static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t  phyHandle  = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};

CRC_Type *base = CRC0;

uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };
uint8_t iv[]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

struct AES_ctx ctx;

char *rx_data_ptr;
char *tx_data_ptr;

char tx_data [MAX_MESSAGES][MAX_NUM_OF_CHARS] = {

		"Hola. Como estas?",
		"Soy Alejandro Triana",
		"Tengo 23 anos ",
		"Vivo en Guadalajara",
		"Trabajo en NXP",
		"Me gusta el futbol",
		"Estudio en el ITESO",
		"Adios"
};

/*******************************************************************************
 * Private Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Private Functions
 ******************************************************************************/
static void InitCrc32 (CRC_Type *base, uint32_t seed)
{
    crc_config_t config;

    config.polynomial         = 0x04C11DB7U;
    config.seed               = seed;
    config.reflectIn          = true;
    config.reflectOut         = true;
    config.complementChecksum = true;
    config.crcBits            = kCrcBits32;
    config.crcResult          = kCrcFinalChecksum;

    CRC_Init(base, &config);
}

struct netconn* my_socket_create (void)
{
    static struct netif netif;
    struct netconn *conn;

    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;

    ethernetif_config_t enet_config =
    {
        .phyHandle  = &phyHandle,
        .macAddress = configMAC_ADDR,
    };

    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    IP4_ADDR (&netif_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR (&netif_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR (&netif_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);

    tcpip_init (NULL, NULL);

    netifapi_netif_add (&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN, tcpip_input);
    netifapi_netif_set_default (&netif);
    netifapi_netif_set_up (&netif);

    PRINTF ("\r\n************************************************\r\n");
    PRINTF (" TCP Echo example\r\n");
    PRINTF ("************************************************\r\n");
    PRINTF (" IPv4 Address     : %u.%u.%u.%u\r\n", ((u8_t *)&netif_ipaddr)[0], ((u8_t *)&netif_ipaddr)[1],
           ((u8_t *)&netif_ipaddr)[2], ((u8_t *)&netif_ipaddr)[3]);
    PRINTF (" IPv4 Subnet mask : %u.%u.%u.%u\r\n", ((u8_t *)&netif_netmask)[0], ((u8_t *)&netif_netmask)[1],
           ((u8_t *)&netif_netmask)[2], ((u8_t *)&netif_netmask)[3]);
    PRINTF (" IPv4 Gateway     : %u.%u.%u.%u\r\n", ((u8_t *)&netif_gw)[0], ((u8_t *)&netif_gw)[1],
           ((u8_t *)&netif_gw)[2], ((u8_t *)&netif_gw)[3]);
    PRINTF ("************************************************\r\n");
    PRINTF ("\r\n");

    /* Create a new connection identifier. */
#if LWIP_IPV6

    conn = netconn_new(NETCONN_TCP_IPV6);
    netconn_bind(conn, IP6_ADDR_ANY, configIP_PORT);

#else /* LWIP_IPV6 */

    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, IP_ADDR_ANY, configIP_PORT);

#endif /* LWIP_IPV6 */

    LWIP_ERROR("tcpecho: invalid conn", (conn != NULL), return NULL;);

    /* Tell connection to go into listening mode. */
    netconn_listen(conn);

    return conn;
}

struct netconn* my_socket_accept (struct netconn *conn)
{
	struct netconn *newconn;
	err_t err;

	/* Grab new connection */
	err = netconn_accept(conn, &newconn);

	if (err == ERR_OK)

		return newconn;

	else
		return NULL;
}

char* my_socket_receive (struct netconn *conn)
{
	uint32_t checksum32, rx_checksum32;
	struct netbuf *buf;
	char *rx_data_ptr;
	char *cp_data;
	void *data;
	err_t err;
    u16_t len;

    /* Receive message. */
    err = netconn_recv (conn, &buf);

    if (err == ERR_OK)
    {
    	/* Copy message data */
    	do {
    		netbuf_data (buf, &data, &len);
    		cp_data = (char*)data;

    	} while (netbuf_next(buf) >= 0);

    	/* Clean buffer. */
    	netbuf_delete (buf);
    }

    /* Store the data + CRC */
    char rx_data [len - CRC_SIZE];
    char rx_crc  [CRC_SIZE];

    PRINTF ("mylayer INFO: rx - encrypted message: ");

    /* Get the body of the message */
    for (int i = 0; i < (len - CRC_SIZE); i++)
    {
    	rx_data [i] = *cp_data++;
    	PRINTF ("%x.", rx_data [i]);
    }

    PRINTF ("\r\n");
    PRINTF ("mylayer INFO: rx - crc bytes: ");

    /* Get the 4 bytes of the CRC 32 */
    for (int i = 0; i < CRC_SIZE; i++)
    {
    	rx_crc [i] = *cp_data++;
    	rx_checksum32 = (rx_checksum32 << 8) + rx_crc[i];
    	PRINTF ("%x.", rx_crc[i]);
    }

    PRINTF (", rx crc32: ");
    PRINTF ("%u", rx_checksum32);

    /* Calculate the CRC from the body */
    InitCrc32(base, 0xFFFFFFFFU);
    CRC_WriteData (base, (uint8_t *)&rx_data[0], (len - CRC_SIZE));
    checksum32 = CRC_Get32bitResult (base);
    PRINTF (", calc crc32: ");
    PRINTF ("%u", checksum32);
    PRINTF ("\r\n");

    if (rx_checksum32 == checksum32)
    {
        /* Decrypt the message */
    	AES_init_ctx_iv (&ctx, key, iv);
        AES_CBC_decrypt_buffer (&ctx, (uint8_t *)&rx_data[0], (len - CRC_SIZE));

        PRINTF ("mylayer INFO: rx - data: ");
        for (int i = 0; i < (len - CRC_SIZE); i++)
        {
        	PRINTF ("%c", rx_data[i]);
        }

    	PRINTF ("\r\n");

    	rx_data_ptr = &rx_data[0];
    }

    else
    {
    	PRINTF ("mylayer INFO: rx - crc error!");
    	PRINTF ("\r\n");

    	rx_data_ptr = NULL;
    }

    return rx_data_ptr;
}

err_t my_socket_send (struct netconn *conn, char *data)
{
	uint32_t checksum32;
	u16_t padded_len;
	char *cp_data;
	u16_t len;
	err_t err;

	cp_data = (char*)data;
    len = strlen(data);

    if (data != NULL)
    {
        /* Store the data + CRC */
        char tx_data [len];
        char tx_crc  [CRC_SIZE];

        PRINTF ("mylayer INFO: tx - sending data: ");

        /* Get the body of the message */
        for (int i = 0; i < len; i++)
        {
        	tx_data [i] = *data++;
        	PRINTF ("%c", tx_data[i]);
        }

        /* Padded Data */
        padded_len = len + (16 - (len % 16));
        char tx_padded_data [padded_len + CRC_SIZE];

        for (int i = 0; i < (padded_len + CRC_SIZE); i++)
        {
        	tx_padded_data[i] = 0;
        }

        memcpy (&tx_padded_data[0], &tx_data[0], len);

        /* Encrypt the message */
        AES_init_ctx_iv (&ctx, key, iv);
        AES_CBC_encrypt_buffer (&ctx, (uint8_t *)&tx_padded_data[0], padded_len);

        PRINTF ("\r\n");
        PRINTF ("mylayer INFO: tx - encrypted message: ");

        /* Print encrypted message */
        for (int i = 0; i < padded_len; i++)
        {
        	PRINTF ("%x.", tx_padded_data[i]);
        }

        /* Calculate the CRC from the body */
    	InitCrc32(base, 0xFFFFFFFFU);
        CRC_WriteData (base, (uint8_t *)&tx_padded_data[0], padded_len);
        checksum32 = CRC_Get32bitResult (base);

        PRINTF ("\r\n");
        PRINTF ("mylayer INFO: tx - crc32: ");
        PRINTF ("%u", checksum32);
        PRINTF (" crc bytes: ");

        /* Get the 4 bytes of the CRC 32 */
        for (int i = 0; i < CRC_SIZE; i++)
        {
        	tx_crc[i] = (checksum32 >> (8 * (CRC_SIZE - i - 1)));
        	PRINTF ("%x.", tx_crc[i]);
        }

        PRINTF ("\r\n");

        /* Add CRC to the message */
        for (int i = 0; i < CRC_SIZE; i++)
        {
            tx_padded_data[padded_len + i] = tx_crc[i];
        }

    	/* Transmit message. */
        cp_data = &tx_padded_data[0];
    	err = netconn_write (conn, (void*)cp_data, (padded_len + CRC_SIZE), NETCONN_COPY);

    	return err;
    }

    else
    {
    	PRINTF ("mylayer INFO: tx no data! ");
    	PRINTF ("\r\n");
    	err = ERR_BUF;
    	return err;
    }
}

void my_socket_close (struct netconn *conn)
{
	/* Close connection and discard connection identifier. */
	netconn_close (conn);
	netconn_delete (conn);
}

static void my_tcpecho_app (void *arg)
{
	struct netconn *conn, *newconn;
	err_t err;

    LWIP_UNUSED_ARG(arg);

    /* Create new socket. */
    conn = my_socket_create ( );

	while (1)
	{
		/* Accept new connection. */
		newconn = my_socket_accept (conn);

		/* Process the new connection. */
		if (newconn != NULL)
		{
			uint8_t indx = 0;

			while (indx < MAX_MESSAGES)
			{
				PRINTF ("TEST: ");
				PRINTF ("%u", indx);
				PRINTF ("\r\n");

				PRINTF ("======================== Rx DATA ========================\r\n");

				/* Receive message. */
				rx_data_ptr = my_socket_receive (newconn);
				if (rx_data_ptr != NULL)
				{
					PRINTF ("======================== Tx DATA ========================\r\n");

					/* Transmit message. */
					tx_data_ptr = &tx_data[indx][0];
					err = my_socket_send (newconn, tx_data_ptr);

					PRINTF ("\r\n\n");
				}

				/* Update index. */
				indx ++;
			}

			/* Close connection. */
			my_socket_close (newconn);
		}
	}
}

/*******************************************************************************
 * Functions
 ******************************************************************************/
void my_layer_init (void)
{
	sys_thread_new ("my_tcpecho_app", my_tcpecho_app, NULL, DEFAULT_THREAD_STACKSIZE, DEFAULT_THREAD_PRIO);
}

#endif /* LWIP_NETCONN */
