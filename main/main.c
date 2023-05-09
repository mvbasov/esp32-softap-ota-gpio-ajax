#include <string.h>

#include <freertos/FreeRTOS.h>
#include "driver/gpio.h"
#include <esp_http_server.h>
#include <freertos/task.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include "esp_log.h"
#include <nvs_flash.h>
#include <sys/param.h>
#include <esp_wifi.h>

#define tag "OnboardBlink"
#define WIFI_SSID "ESP32 OTA Update"
#define ESP_INTR_FLAG_DEFAULT 0
#define ONBOARD_LED_GPIO 2
#define MAX_PERIOD 10
#define MIN_PERIOD 1
#define ONBOARD_BOOT_KEY_GPIO 0
#define EXAMPLE_HTTP_QUERY_KEY_MAX_LEN  (64)

/*
 * Serve OTA update portal (index.html)
 */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t update_html_start[] asm("_binary_update_html_start");
extern const uint8_t update_html_end[] asm("_binary_update_html_end");
extern const uint8_t ctl_html_start[] asm("_binary_control_html_start");
extern const uint8_t ctl_html_end[] asm("_binary_control_html_end");

TaskHandle_t taskOnboardBlink = NULL;
char lineChar[50];
bool led = false;
bool blink = true;
bool key = true;
uint8_t period = 2;

/*
 * Handle GPIO interrupts
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
        uint32_t gpio_num = (uint32_t) arg;
	/* This handle for GPIO interrupt processing as example
	 * The handler modify global variable. Real application may use queue for it
	 * The global variable may be replaced by direct check of GPIO state
	 */
        switch (gpio_num) {
                case ONBOARD_BOOT_KEY_GPIO:
			key = (bool) gpio_get_level(ONBOARD_BOOT_KEY_GPIO);
                        break;
                default:
        }
}

/*
 * HTTP Server
 */
// URL: / or /index.html
esp_err_t index_get_handler(httpd_req_t *req)
{
	httpd_resp_send(req, (const char *) index_html_start, index_html_end - index_html_start);
	return ESP_OK;
}
httpd_uri_t index_get = {
	.uri	  = "/",
	.method   = HTTP_GET,
	.handler  = index_get_handler,
	.user_ctx = NULL
};
httpd_uri_t index_html_get = {
	.uri	  = "/index.html",
	.method   = HTTP_GET,
	.handler  = index_get_handler,
	.user_ctx = NULL
};

// URL: /update.html
esp_err_t update_get_handler(httpd_req_t *req)
{
	httpd_resp_send(req, (const char *) update_html_start, update_html_end - update_html_start);
	return ESP_OK;
}
httpd_uri_t update_get = {
	.uri	  = "/update.html",
	.method   = HTTP_GET,
	.handler  = update_get_handler,
	.user_ctx = NULL
};

// URL: /control.html
esp_err_t control_get_handler(httpd_req_t *req)
{
	char*  buf = NULL;
	size_t buf_len;

	buf_len = httpd_req_get_url_query_len(req) + 1;
	if (buf_len > 1) {
        	buf = malloc(buf_len);
		if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
			ESP_LOGI(tag, "Found URL query => %s", buf);
			char param[EXAMPLE_HTTP_QUERY_KEY_MAX_LEN] = {0};
			if (httpd_query_key_value(buf, "led", param, sizeof(param)) == ESP_OK) {
				ESP_LOGI(tag, "Found URL query parameter => led = %s", param);
				if (strcmp(param, "0") == 0) led = false; else led = true;
				ESP_LOGI(tag, "LED state: %d", led);
			}
			if (httpd_query_key_value(buf, "blink", param, sizeof(param)) == ESP_OK) {
				ESP_LOGI(tag, "Found URL query parameter => blink = %s", param);
				if (strcmp(param, "false") == 0) blink = false; else blink = true;
				ESP_LOGI(tag, "Blink state: %d", blink);
			}
			if (httpd_query_key_value(buf, "period", param, sizeof(param)) == ESP_OK) {
				ESP_LOGI(tag, "Found URL query parameter => period = %s", param);
				period = (uint8_t)atoi(param);
				if (period > MAX_PERIOD) period = MAX_PERIOD;
				if (period < MIN_PERIOD) period = MIN_PERIOD;
				ESP_LOGI(tag, "Period: %d", period);
			}
		}
	}
	free(buf);
	httpd_resp_send(req, (const char *) ctl_html_start, ctl_html_end - ctl_html_start);
	return ESP_OK;
}
httpd_uri_t control_get = {
	.uri	  = "/control.html",
	.method   = HTTP_GET,
	.handler  = control_get_handler,
	.user_ctx = NULL
};

// URL: /update 
/*
 * Handle OTA file upload
 */
esp_err_t update_post_handler(httpd_req_t *req)
{
	char buf[1000];
	esp_ota_handle_t ota_handle;
	int remaining = req->content_len;

	const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
	ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));

	while (remaining > 0) {
		int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));

		// Timeout Error: Just retry
		if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
			continue;

		// Serious Error: Abort OTA
		} else if (recv_len <= 0) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
			return ESP_FAIL;
		}

		// Successful Upload: Flash firmware chunk
		if (esp_ota_write(ota_handle, (const void *)buf, recv_len) != ESP_OK) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash Error");
			return ESP_FAIL;
		}

		remaining -= recv_len;
	}

	// Validate and switch to new OTA image and reboot
	if (esp_ota_end(ota_handle) != ESP_OK || esp_ota_set_boot_partition(ota_partition) != ESP_OK) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation / Activation Error");
			return ESP_FAIL;
	}

	httpd_resp_sendstr(req, "Firmware update complete, rebooting now!\n");

	vTaskDelay(500 / portTICK_PERIOD_MS);
	esp_restart();

	return ESP_OK;
}
httpd_uri_t update_post = {
	.uri	  = "/update",
	.method   = HTTP_POST,
	.handler  = update_post_handler,
	.user_ctx = NULL
};

// URL: /state 
esp_err_t state_get_handler(httpd_req_t *req)
{
	sprintf(&lineChar[0], "{\"key\":%d, \"led\":%d, \"blink\":%d, \"period\":%hu }", key?1:0, led?1:0, blink?1:0, period);
	httpd_resp_send(req, (const char *) &lineChar, strlen(lineChar));
	ESP_LOGI(tag, "Key: %d; LED: %d; Blink: %d; Period: %d", key, led, blink, period);
	return ESP_OK;
}
httpd_uri_t state_get = {
	.uri	  = "/state",
	.method   = HTTP_GET,
	.handler  = state_get_handler,
	.user_ctx = NULL
};

/*
 * Setup web server pages
 */
static esp_err_t http_server_init(void)
{
	static httpd_handle_t http_server = NULL;

	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	if (httpd_start(&http_server, &config) == ESP_OK) {
		httpd_register_uri_handler(http_server, &index_get);
		httpd_register_uri_handler(http_server, &index_html_get);
		httpd_register_uri_handler(http_server, &update_get);
		httpd_register_uri_handler(http_server, &control_get);
		httpd_register_uri_handler(http_server, &update_post);
		httpd_register_uri_handler(http_server, &state_get);
	}

	return http_server == NULL ? ESP_FAIL : ESP_OK;
}

/*
 * WiFi configuration
 */
static esp_err_t softap_init(void)
{
	esp_err_t res = ESP_OK;

	res |= esp_netif_init();
	res |= esp_event_loop_create_default();
	esp_netif_create_default_wifi_ap();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	res |= esp_wifi_init(&cfg);

	wifi_config_t wifi_config = {
		.ap = {
			.ssid = WIFI_SSID,
			.ssid_len = strlen(WIFI_SSID),
			.channel = 6,
			.authmode = WIFI_AUTH_OPEN,
			.max_connection = 3
		},
	};

	res |= esp_wifi_set_mode(WIFI_MODE_AP);
	res |= esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
	res |= esp_wifi_start();

	return res;
}

/*
 * Blink onboard LED task
 */
void onboardBlink(void *ignore)
{
        while(1){
		if(blink)
			led = !led;
                if(led) {
			gpio_set_level(ONBOARD_LED_GPIO, 1);
                } else {
			gpio_set_level(ONBOARD_LED_GPIO, 0);
		}
                vTaskDelay(((period*1000/2))/portTICK_PERIOD_MS);
        }
}

void app_main(void) {
	esp_err_t ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);
	ESP_ERROR_CHECK(softap_init());
	ESP_ERROR_CHECK(http_server_init());

	/* Mark current app as valid */
	const esp_partition_t *partition = esp_ota_get_running_partition();
	printf("Currently running partition: %s\r\n", partition->label);

	esp_ota_img_states_t ota_state;
	if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK) {
		if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
			esp_ota_mark_app_valid_cancel_rollback();
		}
	}

        /* GPIO SETUP */
	//zero-initialize the config structure.
        gpio_config_t io_conf = {};
        //interrupt disabled
        io_conf.intr_type = GPIO_INTR_DISABLE;
        //bit mask of the pins
        io_conf.pin_bit_mask = 1ULL<<ONBOARD_LED_GPIO;
        //set as input mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io_conf);

	//interrupt of both edge
	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	//bit mask of the pins
	io_conf.pin_bit_mask = 1UL<<ONBOARD_BOOT_KEY_GPIO;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	//io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	//install gpio isr service
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        //hook isr handler for specific gpio pin
        gpio_isr_handler_add(ONBOARD_BOOT_KEY_GPIO, gpio_isr_handler, (void*)ONBOARD_BOOT_KEY_GPIO);

	gpio_set_level(ONBOARD_LED_GPIO, 1);
	ESP_LOGI(tag, "Start task: toggle onboard LED");
        xTaskCreate(onboardBlink, "toggle onboard LED", 2048, NULL, 10, &taskOnboardBlink);

	while(1) vTaskDelay(10);
}
