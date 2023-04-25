/* CoAP server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * WARNING
 * libcoap is not multi-thread safe, so only this thread must make any coap_*()
 * calls.  Any external (to this thread) data transmitted in/out via libcoap
 * therefore has to be passed in/out by xQueue*() via this thread.
 */

#include <string.h>
#include <sys/socket.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "nvs_flash.h"

#include "protocol_examples_common.h"

#include "coap3/coap.h"

#ifndef CONFIG_COAP_SERVER_SUPPORT
#error COAP_SERVER_SUPPORT needs to be enabled
#endif /* COAP_SERVER_SUPPORT */

/* The examples use simple Pre-Shared-Key configuration that you can set via
   'idf.py menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_COAP_PSK_KEY "some-agreed-preshared-key"

   Note: PSK will only be used if the URI is prefixed with coaps://
   instead of coap:// and the PSK must be one that the server supports
   (potentially associated with the IDENTITY)
*/
#define EXAMPLE_COAP_PSK_KEY CONFIG_EXAMPLE_COAP_PSK_KEY

/* The examples use CoAP Logging Level that
   you can set via 'idf.py menuconfig'.

   If you'd rather not, just change the below entry to a value
   that is between 0 and 7 with
   the config you want - ie #define EXAMPLE_COAP_LOG_DEFAULT_LEVEL 7
*/
#define EXAMPLE_COAP_LOG_DEFAULT_LEVEL CONFIG_COAP_LOG_DEFAULT_LEVEL

const static char *TAG = "CoAP_server";

static char espressif_data[100];
static int espressif_data_len = 0;

#ifdef CONFIG_COAP_MBEDTLS_PKI
/* CA cert, taken from coap_ca.pem
   Server cert, taken from coap_server.crt
   Server key, taken from coap_server.key

   The PEM, CRT and KEY file are examples taken from
   https://github.com/eclipse/californium/tree/master/demo-certs/src/main/resources
   as the Certificate test (by default) for the coap_client is against the
   californium server.

   To embed it in the app binary, the PEM, CRT and KEY file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
 */
extern uint8_t ca_pem_start[] asm("_binary_coap_ca_pem_start");
extern uint8_t ca_pem_end[]   asm("_binary_coap_ca_pem_end");
extern uint8_t server_crt_start[] asm("_binary_coap_server_crt_start");
extern uint8_t server_crt_end[]   asm("_binary_coap_server_crt_end");
extern uint8_t server_key_start[] asm("_binary_coap_server_key_start");
extern uint8_t server_key_end[]   asm("_binary_coap_server_key_end");
#endif /* CONFIG_COAP_MBEDTLS_PKI */

#define INITIAL_DATA "Hello World!"
#define INITIAL_DATA_DIRECTION "F"
#define INITIAL_DATA_SPEED "50"

#define MAX_NUM_OF_CHARS 50
#define MAX_MESSAGES 10

static char direction_data[100];
static int direction_data_len = 0;

static char speed_data[100];
static int speed_data_len = 0;

static char temperatureValues [MAX_MESSAGES][MAX_NUM_OF_CHARS] = {"10", "52", "45", "82", "73", "40", "22", "38", "65", "13"};
uint32_t temperature_indx = 0;

static char currentValues [MAX_MESSAGES][MAX_NUM_OF_CHARS] = {"10", "500", "750", "310", "200", "550", "820", "120", "70", "960"};
uint32_t current_indx = 0;

bool gas_status = false;

/*
 * The resource handler
 */

/* GET Handlers */
static void
hnd_espressif_get_direction(coap_resource_t *resource,
                  coap_session_t *session,
                  const coap_pdu_t *request,
                  const coap_string_t *query,
                  coap_pdu_t *response)
{
    coap_pdu_set_code(response, COAP_RESPONSE_CODE_CONTENT);
    coap_add_data_large_response(resource, session, request, response,
                                 query, COAP_MEDIATYPE_TEXT_PLAIN, 60, 0,
                                 (size_t)direction_data_len,
                                 (const u_char *)direction_data,
                                 NULL, NULL);

    /* Imprimir datos en terminal */
    printf("Direction: ");

    if (direction_data[0] == 'F')
        printf("Front");

    else if (direction_data[0] == 'B')
        printf("Back");

    else if (direction_data[0] == 'L')
        printf("Left");

    else if (direction_data[0] == 'R')
        printf("Right");

    else
        printf("DIRECTION ERROR");

    printf("\r\n");
}

static void
hnd_espressif_get_speed(coap_resource_t *resource,
                  coap_session_t *session,
                  const coap_pdu_t *request,
                  const coap_string_t *query,
                  coap_pdu_t *response)
{
    if (speed_data[0] < '0' || speed_data[0] > '9')
    {
        snprintf(speed_data, sizeof(speed_data), "SPEED ERROR");
        speed_data_len = strlen(speed_data);
    }

    coap_pdu_set_code(response, COAP_RESPONSE_CODE_CONTENT);
    coap_add_data_large_response(resource, session, request, response,
                                 query, COAP_MEDIATYPE_TEXT_PLAIN, 60, 0,
                                 (size_t)speed_data_len,
                                 (const u_char *)speed_data,
                                 NULL, NULL);

    /* Imprimir datos en terminal */
    printf("Velocidad - Duty Cycle: ");
    printf("%s", (char *)speed_data);
    printf(" %%");
    printf("\r\n");
}

static void
hnd_espressif_get_temperature(coap_resource_t *resource,
                  coap_session_t *session,
                  const coap_pdu_t *request,
                  const coap_string_t *query,
                  coap_pdu_t *response)
{
    char temperature_data[100];
    int temperature_data_len = 0;

    snprintf(temperature_data, sizeof(temperature_data), temperatureValues[temperature_indx]);
    temperature_data_len = strlen(temperature_data);

    coap_pdu_set_code(response, COAP_RESPONSE_CODE_CONTENT);
    coap_add_data_large_response(resource, session, request, response,
                                 query, COAP_MEDIATYPE_TEXT_PLAIN, 60, 0,
                                 (size_t)temperature_data_len,
                                 (const u_char *)temperature_data,
                                 NULL, NULL);

    /* Imprimir datos en terminal */
    printf("Temperatura Ambiente: ");
    printf("%s", (char *)temperature_data);
    printf(" °C\r\n");

    /* Actualizar indice */
    temperature_indx++;

    /* Reiniciar indice */
    if (temperature_indx >= 10)
        temperature_indx = 0;
}

static void
hnd_espressif_get_current(coap_resource_t *resource,
                  coap_session_t *session,
                  const coap_pdu_t *request,
                  const coap_string_t *query,
                  coap_pdu_t *response)
{
    char current_data[100];
    int current_data_len = 0;

    snprintf(current_data, sizeof(current_data), currentValues[current_indx]);
    current_data_len = strlen(current_data);

    coap_pdu_set_code(response, COAP_RESPONSE_CODE_CONTENT);
    coap_add_data_large_response(resource, session, request, response,
                                 query, COAP_MEDIATYPE_TEXT_PLAIN, 60, 0,
                                 (size_t)current_data_len,
                                 (const u_char *)current_data,
                                 NULL, NULL);

    /* Imprimir datos en terminal */
    printf("Consumo de Corriente: ");
    printf("%s", (char *)current_data);
    printf(" mA\r\n");

    /* Actualizar indice */
    current_indx++;

    /* Reiniciar indice */
    if (current_indx >= 10)
        current_indx = 0;
}

static void
hnd_espressif_get_gas(coap_resource_t *resource,
                  coap_session_t *session,
                  const coap_pdu_t *request,
                  const coap_string_t *query,
                  coap_pdu_t *response)
{
    char gas_data[100];
    int gas_data_len = 0;

    if (gas_status)
        snprintf(gas_data, sizeof(gas_data), "No Gas");

    else
        snprintf(gas_data, sizeof(gas_data), "ALARM: Gas");

    gas_data_len = strlen(gas_data);

    coap_pdu_set_code(response, COAP_RESPONSE_CODE_CONTENT);
    coap_add_data_large_response(resource, session, request, response,
                                 query, COAP_MEDIATYPE_TEXT_PLAIN, 60, 0,
                                 (size_t)gas_data_len,
                                 (const u_char *)gas_data,
                                 NULL, NULL);

    /* Imprimir datos en terminal */
    printf("%s", (char *)gas_data);
    printf("\r\n");

    gas_status = gas_status ^ 1;
}

/* PUT Handlers */
static void
hnd_espressif_put_direction(coap_resource_t *resource,
                  coap_session_t *session,
                  const coap_pdu_t *request,
                  const coap_string_t *query,
                  coap_pdu_t *response)
{
    size_t size;
    size_t offset;
    size_t total;
    const unsigned char *data;

    coap_resource_notify_observers(resource, NULL);

    if (strcmp (direction_data, INITIAL_DATA_DIRECTION) == 0) {
        coap_pdu_set_code(response, COAP_RESPONSE_CODE_CREATED);
    } else {
        coap_pdu_set_code(response, COAP_RESPONSE_CODE_CHANGED);
    }

    /* coap_get_data_large() sets size to 0 on error */
    (void)coap_get_data_large(request, &size, &data, &offset, &total);

    if (size == 0) {      /* re-init */
        snprintf(direction_data, sizeof(direction_data), INITIAL_DATA_DIRECTION);
        direction_data_len = strlen(direction_data);
    } else {
        espressif_data_len = size > sizeof (direction_data) ? sizeof (direction_data) : size;
        memcpy (direction_data, data, direction_data_len);
    }
}

static void
hnd_espressif_put_speed(coap_resource_t *resource,
                  coap_session_t *session,
                  const coap_pdu_t *request,
                  const coap_string_t *query,
                  coap_pdu_t *response)
{
    size_t size;
    size_t offset;
    size_t total;
    const unsigned char *data;

    coap_resource_notify_observers(resource, NULL);

    if (strcmp (speed_data, INITIAL_DATA_SPEED) == 0) {
        coap_pdu_set_code(response, COAP_RESPONSE_CODE_CREATED);
    } else {
        coap_pdu_set_code(response, COAP_RESPONSE_CODE_CHANGED);
    }

    /* coap_get_data_large() sets size to 0 on error */
    (void)coap_get_data_large(request, &size, &data, &offset, &total);

    if (size == 0) {      /* re-init */
        snprintf(speed_data, sizeof(speed_data), INITIAL_DATA_SPEED);
        speed_data_len = strlen(speed_data);
    } else {
        speed_data_len = size > sizeof (speed_data) ? sizeof (speed_data) : size;
        memcpy (speed_data, data, speed_data_len);
    }
}

/* DELETE Handlers */
static void
hnd_espressif_delete(coap_resource_t *resource,
                     coap_session_t *session,
                     const coap_pdu_t *request,
                     const coap_string_t *query,
                     coap_pdu_t *response)
{
    coap_resource_notify_observers(resource, NULL);
    snprintf(espressif_data, sizeof(espressif_data), INITIAL_DATA);
    espressif_data_len = strlen(espressif_data);
    coap_pdu_set_code(response, COAP_RESPONSE_CODE_DELETED);
}

#ifdef CONFIG_COAP_MBEDTLS_PKI

static int
verify_cn_callback(const char *cn,
                   const uint8_t *asn1_public_cert,
                   size_t asn1_length,
                   coap_session_t *session,
                   unsigned depth,
                   int validated,
                   void *arg
                  )
{
    coap_log(LOG_INFO, "CN '%s' presented by server (%s)\n",
             cn, depth ? "CA" : "Certificate");
    return 1;
}
#endif /* CONFIG_COAP_MBEDTLS_PKI */

static void
coap_log_handler (coap_log_t level, const char *message)
{
    uint32_t esp_level = ESP_LOG_INFO;
    char *cp = strchr(message, '\n');

    if (cp)
        ESP_LOG_LEVEL(esp_level, TAG, "%.*s", (int)(cp-message), message);
    else
        ESP_LOG_LEVEL(esp_level, TAG, "%s", message);
}

static void coap_example_server(void *p)
{
    /* CoAP context and server address */
    coap_context_t *ctx = NULL;
    coap_address_t serv_addr;

    /* Define resources */
    coap_resource_t *resource_direction   = NULL; 
    coap_resource_t *resource_speed       = NULL; 
    coap_resource_t *resource_temperature = NULL; 
    coap_resource_t *resource_current     = NULL; 
    coap_resource_t *resource_gas         = NULL; 

    snprintf(direction_data, sizeof(direction_data), INITIAL_DATA_DIRECTION);
    direction_data_len = strlen(direction_data);

    snprintf(speed_data, sizeof(speed_data), INITIAL_DATA_SPEED);
    speed_data_len = strlen(speed_data);

    coap_set_log_handler(coap_log_handler);
    coap_set_log_level(EXAMPLE_COAP_LOG_DEFAULT_LEVEL);

    while (1) {
        coap_endpoint_t *ep = NULL;
        unsigned wait_ms;
        int have_dtls = 0;

        /* Prepare the CoAP server socket */
        coap_address_init(&serv_addr);
        serv_addr.addr.sin6.sin6_family = AF_INET6;
        serv_addr.addr.sin6.sin6_port   = htons(COAP_DEFAULT_PORT);

        ctx = coap_new_context(NULL);
        if (!ctx) {
            ESP_LOGE(TAG, "coap_new_context() failed");
            continue;
        }
        coap_context_set_block_mode(ctx,
                                    COAP_BLOCK_USE_LIBCOAP|COAP_BLOCK_SINGLE_BODY);
#ifdef CONFIG_COAP_MBEDTLS_PSK
        /* Need PSK setup before we set up endpoints */
        coap_context_set_psk(ctx, "CoAP",
                             (const uint8_t *)EXAMPLE_COAP_PSK_KEY,
                             sizeof(EXAMPLE_COAP_PSK_KEY) - 1);
#endif /* CONFIG_COAP_MBEDTLS_PSK */

#ifdef CONFIG_COAP_MBEDTLS_PKI
        /* Need PKI setup before we set up endpoints */
        unsigned int ca_pem_bytes = ca_pem_end - ca_pem_start;
        unsigned int server_crt_bytes = server_crt_end - server_crt_start;
        unsigned int server_key_bytes = server_key_end - server_key_start;
        coap_dtls_pki_t dtls_pki;

        memset (&dtls_pki, 0, sizeof(dtls_pki));
        dtls_pki.version = COAP_DTLS_PKI_SETUP_VERSION;
        if (ca_pem_bytes) {
            /*
             * Add in additional certificate checking.
             * This list of enabled can be tuned for the specific
             * requirements - see 'man coap_encryption'.
             *
             * Note: A list of root ca file can be setup separately using
             * coap_context_set_pki_root_cas(), but the below is used to
             * define what checking actually takes place.
             */
            dtls_pki.verify_peer_cert        = 1;
            dtls_pki.check_common_ca         = 1;
            dtls_pki.allow_self_signed       = 1;
            dtls_pki.allow_expired_certs     = 1;
            dtls_pki.cert_chain_validation   = 1;
            dtls_pki.cert_chain_verify_depth = 2;
            dtls_pki.check_cert_revocation   = 1;
            dtls_pki.allow_no_crl            = 1;
            dtls_pki.allow_expired_crl       = 1;
            dtls_pki.allow_bad_md_hash       = 1;
            dtls_pki.allow_short_rsa_length  = 1;
            dtls_pki.validate_cn_call_back   = verify_cn_callback;
            dtls_pki.cn_call_back_arg        = NULL;
            dtls_pki.validate_sni_call_back  = NULL;
            dtls_pki.sni_call_back_arg       = NULL;
        }
        dtls_pki.pki_key.key_type = COAP_PKI_KEY_PEM_BUF;
        dtls_pki.pki_key.key.pem_buf.public_cert = server_crt_start;
        dtls_pki.pki_key.key.pem_buf.public_cert_len = server_crt_bytes;
        dtls_pki.pki_key.key.pem_buf.private_key = server_key_start;
        dtls_pki.pki_key.key.pem_buf.private_key_len = server_key_bytes;
        dtls_pki.pki_key.key.pem_buf.ca_cert = ca_pem_start;
        dtls_pki.pki_key.key.pem_buf.ca_cert_len = ca_pem_bytes;

        coap_context_set_pki(ctx, &dtls_pki);
#endif /* CONFIG_COAP_MBEDTLS_PKI */

        ep = coap_new_endpoint(ctx, &serv_addr, COAP_PROTO_UDP);
        if (!ep) {
            ESP_LOGE(TAG, "udp: coap_new_endpoint() failed");
            goto clean_up;
        }
        if (coap_tcp_is_supported()) {
            ep = coap_new_endpoint(ctx, &serv_addr, COAP_PROTO_TCP);
            if (!ep) {
                ESP_LOGE(TAG, "tcp: coap_new_endpoint() failed");
                goto clean_up;
            }
        }
#if defined(CONFIG_COAP_MBEDTLS_PSK) || defined(CONFIG_COAP_MBEDTLS_PKI)
        if (coap_dtls_is_supported()) {
#ifndef CONFIG_MBEDTLS_TLS_SERVER
            /* This is not critical as unencrypted support is still available */
            ESP_LOGI(TAG, "MbedTLS DTLS Server Mode not configured");
#else /* CONFIG_MBEDTLS_TLS_SERVER */
            serv_addr.addr.sin6.sin6_port = htons(COAPS_DEFAULT_PORT);
            ep = coap_new_endpoint(ctx, &serv_addr, COAP_PROTO_DTLS);
            if (!ep) {
                ESP_LOGE(TAG, "dtls: coap_new_endpoint() failed");
                goto clean_up;
            }
            have_dtls = 1;
#endif /* CONFIG_MBEDTLS_TLS_SERVER */
        }
        if (coap_tls_is_supported()) {
#ifndef CONFIG_MBEDTLS_TLS_SERVER
            /* This is not critical as unencrypted support is still available */
            ESP_LOGI(TAG, "MbedTLS TLS Server Mode not configured");
#else /* CONFIG_MBEDTLS_TLS_SERVER */
            serv_addr.addr.sin6.sin6_port = htons(COAPS_DEFAULT_PORT);
            ep = coap_new_endpoint(ctx, &serv_addr, COAP_PROTO_TLS);
            if (!ep) {
                ESP_LOGE(TAG, "tls: coap_new_endpoint() failed");
                goto clean_up;
            }
#endif /* CONFIG_MBEDTLS_TLS_SERVER */
        }
        if (!have_dtls) {
            /* This is not critical as unencrypted support is still available */
            ESP_LOGI(TAG, "MbedTLS (D)TLS Server Mode not configured");
        }
#endif /* CONFIG_COAP_MBEDTLS_PSK || CONFIG_COAP_MBEDTLS_PKI */

        /* Direction Resource */
        resource_direction = coap_resource_init(coap_make_str_const("Espressif/direction"), 0);
        if (!resource_direction) {
            ESP_LOGE(TAG, "coap_resource_init() failed");
            goto clean_up;
        }
        coap_register_handler(resource_direction, COAP_REQUEST_GET, hnd_espressif_get_direction);
        coap_register_handler(resource_direction, COAP_REQUEST_PUT, hnd_espressif_put_direction);
        coap_register_handler(resource_direction, COAP_REQUEST_DELETE, hnd_espressif_delete);
        /* We possibly want to Observe the GETs */
        coap_resource_set_get_observable(resource_direction, 1);
        coap_add_resource(ctx, resource_direction);

        /* Speed Resource */
        resource_speed = coap_resource_init(coap_make_str_const("Espressif/speed"), 0);
        if (!resource_speed) {
            ESP_LOGE(TAG, "coap_resource_init() failed");
            goto clean_up;
        }
        coap_register_handler(resource_speed, COAP_REQUEST_GET, hnd_espressif_get_speed);
        coap_register_handler(resource_speed, COAP_REQUEST_PUT, hnd_espressif_put_speed);
        coap_register_handler(resource_speed, COAP_REQUEST_DELETE, hnd_espressif_delete);
        /* We possibly want to Observe the GETs */
        coap_resource_set_get_observable(resource_speed, 1);
        coap_add_resource(ctx, resource_speed);

        /* Temperature Resource */
        resource_temperature = coap_resource_init(coap_make_str_const("Espressif/temperature"), 0);
        if (!resource_temperature) {
            ESP_LOGE(TAG, "coap_resource_init() failed");
            goto clean_up;
        }
        coap_register_handler(resource_temperature, COAP_REQUEST_GET, hnd_espressif_get_temperature);
        coap_register_handler(resource_temperature, COAP_REQUEST_DELETE, hnd_espressif_delete);
        /* We possibly want to Observe the GETs */
        coap_resource_set_get_observable(resource_temperature, 1);
        coap_add_resource(ctx, resource_temperature);

        /* Current Resource */
        resource_current = coap_resource_init(coap_make_str_const("Espressif/current"), 0);
        if (!resource_current) {
            ESP_LOGE(TAG, "coap_resource_init() failed");
            goto clean_up;
        }
        coap_register_handler(resource_current, COAP_REQUEST_GET, hnd_espressif_get_current);
        coap_register_handler(resource_current, COAP_REQUEST_DELETE, hnd_espressif_delete);
        /* We possibly want to Observe the GETs */
        coap_resource_set_get_observable(resource_current, 1);
        coap_add_resource(ctx, resource_current);

        /* Gas Resource */
        resource_gas = coap_resource_init(coap_make_str_const("Espressif/gas"), 0);
        if (!resource_gas) {
            ESP_LOGE(TAG, "coap_resource_init() failed");
            goto clean_up;
        }
        coap_register_handler(resource_gas, COAP_REQUEST_GET, hnd_espressif_get_gas);
        coap_register_handler(resource_gas, COAP_REQUEST_DELETE, hnd_espressif_delete);
        /* We possibly want to Observe the GETs */
        coap_resource_set_get_observable(resource_gas, 1);
        coap_add_resource(ctx, resource_gas);

#if defined(CONFIG_EXAMPLE_COAP_MCAST_IPV4) || defined(CONFIG_EXAMPLE_COAP_MCAST_IPV6)
        esp_netif_t *netif = NULL;
        for (int i = 0; i < esp_netif_get_nr_of_ifs(); ++i) {
            char buf[8];
            netif = esp_netif_next(netif);
            esp_netif_get_netif_impl_name(netif, buf);
#if defined(CONFIG_EXAMPLE_COAP_MCAST_IPV4)
            coap_join_mcast_group_intf(ctx, CONFIG_EXAMPLE_COAP_MULTICAST_IPV4_ADDR, buf);
#endif /* CONFIG_EXAMPLE_COAP_MCAST_IPV4 */
#if defined(CONFIG_EXAMPLE_COAP_MCAST_IPV6)
            /* When adding IPV6 esp-idf requires ifname param to be filled in */
            coap_join_mcast_group_intf(ctx, CONFIG_EXAMPLE_COAP_MULTICAST_IPV6_ADDR, buf);
#endif /* CONFIG_EXAMPLE_COAP_MCAST_IPV6 */
        }
#endif /* CONFIG_EXAMPLE_COAP_MCAST_IPV4 || CONFIG_EXAMPLE_COAP_MCAST_IPV6 */

        wait_ms = COAP_RESOURCE_CHECK_TIME * 1000;

        while (1) {
            int result = coap_io_process(ctx, wait_ms);
            if (result < 0) {
                break;
            } else if (result && (unsigned)result < wait_ms) {
                /* decrement if there is a result wait time returned */
                wait_ms -= result;
            }
            if (result) {
                /* result must have been >= wait_ms, so reset wait_ms */
                wait_ms = COAP_RESOURCE_CHECK_TIME * 1000;
            }
        }
    }
clean_up:
    coap_free_context(ctx);
    coap_cleanup();

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(coap_example_server, "coap", 8 * 1024, NULL, 5, NULL);
}
