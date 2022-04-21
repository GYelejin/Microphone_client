#include "driver/adc.h"
#include "driver/i2s.h"
#include "esp_adc_cal.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_partition.h"
#include "esp_rom_sys.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR "10.0.20.93"
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *payload = "Message from ESP32 ";

static const char *TAG = "ad/da";
#define V_REF 1100
#define ADC1_TEST_CHANNEL (ADC1_CHANNEL_7)

#define PARTITION_NAME "storage"

/*---------------------------------------------------------------
                            EXAMPLE CONFIG
---------------------------------------------------------------*/
// enable record sound and save in flash
#define RECORD_IN_FLASH_EN (1)
// enable replay recorded sound in flash
#define REPLAY_FROM_FLASH_EN (1)

// i2s number
#define EXAMPLE_I2S_NUM (0)
// i2s sample rate
#define EXAMPLE_I2S_SAMPLE_RATE (8000)
// i2s data bits
#define EXAMPLE_I2S_SAMPLE_BITS (16)
// enable display buffer for debug
#define EXAMPLE_I2S_BUF_DEBUG (0)
// I2S read buffer length
#define EXAMPLE_I2S_READ_LEN (16 * 1024)
// I2S data format
#define EXAMPLE_I2S_FORMAT (I2S_CHANNEL_FMT_RIGHT_LEFT)
// I2S channel number
#define EXAMPLE_I2S_CHANNEL_NUM                                                \
  ((EXAMPLE_I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
// I2S built-in ADC unit
#define I2S_ADC_UNIT ADC_UNIT_1
// I2S built-in ADC channel
#define I2S_ADC_CHANNEL ADC1_CHANNEL_0

// flash record size, for recording 5 seconds' data
#define FLASH_RECORD_SIZE                                                      \
  (EXAMPLE_I2S_CHANNEL_NUM * EXAMPLE_I2S_SAMPLE_RATE *                         \
   EXAMPLE_I2S_SAMPLE_BITS / 8 * 5)
#define FLASH_ERASE_SIZE                                                       \
  (FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE == 0)                                 \
      ? FLASH_RECORD_SIZE                                                      \
      : FLASH_RECORD_SIZE +                                                    \
            (FLASH_SECTOR_SIZE - FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE)
// sector size of flash
#define FLASH_SECTOR_SIZE (0x1000)
// flash read / write address
#define FLASH_ADDR (0x200000)

/**
 * @brief I2S ADC/DAC mode init.
 */
void example_i2s_init(void) {
  int i2s_num = EXAMPLE_I2S_NUM;
  i2s_config_t i2s_config = {
      .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX |
              I2S_MODE_DAC_BUILT_IN | I2S_MODE_ADC_BUILT_IN,
      .sample_rate = EXAMPLE_I2S_SAMPLE_RATE,
      .bits_per_sample = EXAMPLE_I2S_SAMPLE_BITS,
      .communication_format = I2S_COMM_FORMAT_STAND_MSB,
      .channel_format = EXAMPLE_I2S_FORMAT,
      .intr_alloc_flags = 0,
      .dma_buf_count = 2,
      .dma_buf_len = 1024,
      .use_apll = 1,
  };
  // install and start i2s driver
  i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
  // init DAC pad
  i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
  // init ADC pad
  i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);
}

/*
 * @brief erase flash for recording
 */
void example_erase_flash(void) {
#if RECORD_IN_FLASH_EN
  printf("Erasing flash \n");
  const esp_partition_t *data_partition = NULL;
  data_partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
  if (data_partition != NULL) {
    printf("partiton addr: 0x%08x; size: %d; label: %s\n",
           data_partition->address, data_partition->size,
           data_partition->label);
  }
  printf("Erase size: %d Bytes\n", FLASH_ERASE_SIZE);
  ESP_ERROR_CHECK(
      esp_partition_erase_range(data_partition, 0, FLASH_ERASE_SIZE));
#else
  printf("Skip flash erasing...\n");
#endif
}

/**
 * @brief debug buffer data
 */
void example_disp_buf(uint8_t *buf, int length) {
#if EXAMPLE_I2S_BUF_DEBUG
  printf("======\n");
  for (int i = 0; i < length; i++) {
    printf("%02x ", buf[i]);
    if ((i + 1) % 8 == 0) {
      printf("\n");
    }
  }
  printf("======\n");
#endif
}

/**
 * @brief Reset i2s clock and mode
 */
void example_reset_play_mode(void) {
  i2s_set_clk(EXAMPLE_I2S_NUM, EXAMPLE_I2S_SAMPLE_RATE, EXAMPLE_I2S_SAMPLE_BITS,
              EXAMPLE_I2S_CHANNEL_NUM);
}

/**
 * @brief Set i2s clock for example audio file
 */
void example_set_file_play_mode(void) {
  i2s_set_clk(EXAMPLE_I2S_NUM, 8000, EXAMPLE_I2S_SAMPLE_BITS, 1);
}

/**
 * @brief I2S ADC/DAC example
 *        1. Erase flash
 *        2. Record audio from ADC and save in flash
 *        3. Read flash and replay the sound via DAC
 *        4. Play an example audio file(file format: 8bit/8khz/single channel)
 *        5. Loop back to step 3
 */
void example_i2s_adc_dac(void *arg) {
  const esp_partition_t *data_partition = NULL;
  data_partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
  if (data_partition != NULL) {
    printf("partiton addr: 0x%08x; size: %d; label: %s\n",
           data_partition->address, data_partition->size,
           data_partition->label);
  } else {
    ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n",
             PARTITION_NAME);
    vTaskDelete(NULL);
  }
  // 1. Erase flash
  example_erase_flash();
  int i2s_read_len = EXAMPLE_I2S_READ_LEN;
  int flash_wr_size = 0;
  size_t bytes_read, bytes_written;

  // 2. Record audio from ADC and save in flash
#if RECORD_IN_FLASH_EN
  char *i2s_read_buff = (char *)calloc(i2s_read_len, sizeof(char));
  uint8_t *flash_write_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));
  i2s_adc_enable(EXAMPLE_I2S_NUM);
  while (flash_wr_size < FLASH_RECORD_SIZE) {
    // read data from I2S bus, in this case, from ADC.
    i2s_read(EXAMPLE_I2S_NUM, (void *)i2s_read_buff, i2s_read_len, &bytes_read,
             portMAX_DELAY);
    example_disp_buf((uint8_t *)i2s_read_buff, 64);
    // save original data from I2S(ADC) into flash.
    esp_partition_write(data_partition, flash_wr_size, i2s_read_buff,
                        i2s_read_len);
    flash_wr_size += i2s_read_len;
    esp_rom_printf("Sound recording %u%%\n",
                   flash_wr_size * 100 / FLASH_RECORD_SIZE);
  }
  i2s_adc_disable(EXAMPLE_I2S_NUM);
  free(i2s_read_buff);
  i2s_read_buff = NULL;
  free(flash_write_buff);
  flash_write_buff = NULL;
#endif

  uint8_t *flash_read_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));
  uint8_t *i2s_write_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));
  while (1) {
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
#if defined(CONFIG_EXAMPLE_IPV4)
      struct sockaddr_in dest_addr;
      dest_addr.sin_addr.s_addr = inet_addr(host_ip);
      dest_addr.sin_family = AF_INET;
      dest_addr.sin_port = htons(PORT);
      addr_family = AF_INET;
      ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
      struct sockaddr_in6 dest_addr = {0};
      inet6_aton(host_ip, &dest_addr.sin6_addr);
      dest_addr.sin6_family = AF_INET6;
      dest_addr.sin6_port = htons(PORT);
      dest_addr.sin6_scope_id =
          esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
      addr_family = AF_INET6;
      ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
      struct sockaddr_storage dest_addr = {0};
      ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol,
                                          &addr_family, &dest_addr));
#endif
      int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
      if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        break;
      }
      ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

      int err = connect(sock, (struct sockaddr *)&dest_addr,
                        sizeof(struct sockaddr_in6));
      if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        break;
      }
      ESP_LOGI(TAG, "Successfully connected");

      while (1) {
        // 3. Read flash and replay the sound via DAC
        for (int rd_offset = 0; rd_offset < flash_wr_size;
             rd_offset += FLASH_SECTOR_SIZE) {
          // read I2S(ADC) original data from flash
          esp_partition_read(data_partition, rd_offset, flash_read_buff,
                             FLASH_SECTOR_SIZE);
          // send data

          send(sock, &flash_read_buff, sizeof(flash_read_buff), 0);
        }
        if (err < 0) {
          ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
          break;
        }

        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        // Error occurred during receiving
        if (len < 0) {
          ESP_LOGE(TAG, "recv failed: errno %d", errno);
          break;
        }
        // Data received
        else {
          rx_buffer[len] =
              0; // Null-terminate whatever we received and treat like a string
          ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
          ESP_LOGI(TAG, "%s", rx_buffer);
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
      }

      if (sock != -1) {
        ESP_LOGE(TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
      }
    }
  }
}

void adc_read_task(void *arg) {
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_11db);
  esp_adc_cal_characteristics_t characteristics;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF,
                           &characteristics);
  while (1) {
    uint32_t voltage;
    esp_adc_cal_get_voltage(ADC1_TEST_CHANNEL, &characteristics, &voltage);
    ESP_LOGI(TAG, "%d mV", voltage);
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = EXAMPLE_ESP_WIFI_SSID,
              .password = EXAMPLE_ESP_WIFI_PASS,
              /* Setting a password implies station will connect to all security
               * modes including WEP/WPA. However these modes are deprecated and
               * not advisable to be used. Incase your Access point doesn't
               * support WPA2, these mode can be enabled by commenting below
               * line */
              .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID,
             EXAMPLE_ESP_WIFI_PASS);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

void app_main(void) {
  example_i2s_init();
  // Initialize NVS
  esp_log_level_set("I2S", ESP_LOG_INFO);
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  wifi_init_sta();

  xTaskCreate(example_i2s_adc_dac, "i2s_adc", 1024 * 4, NULL, 5, NULL);
  xTaskCreate(adc_read_task, "ADC read task", 2048, NULL, 5, NULL);
}
