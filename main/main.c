/*
    SAU
*/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "camera.h"


#define CONFIG_EXAMPLE_UART_TXD 1
#define CONFIG_EXAMPLE_UART_RXD 3
#define CONFIG_EXAMPLE_UART_PORT_NUM 0
#define CONFIG_EXAMPLE_UART_BAUD_RATE 115200
#define CONFIG_EXAMPLE_TASK_STACK_SIZE 100000


#define SAU_TXD (CONFIG_EXAMPLE_UART_TXD)
#define SAU_RXD (CONFIG_EXAMPLE_UART_RXD)
#define SAU_RTS (UART_PIN_NO_CHANGE)
#define SAU_CTS (UART_PIN_NO_CHANGE)

#define SAU_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define SAU_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define SAU_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

// 10240 20480 30720 - for mysterious reasons if the program is big enough, the uart_read_bytes will return 0 at some late point.
//#define SAU_SEND_BUF_SIZE 1024
#define SAU_SEND_BUF_SIZE 30720
#define SAU_RCV_BUF_SIZE 30720

// Using the repeating structure we make sure "wxyz" is easily received (offset-independent)
#define SAU_READY_MARKER "wxyz3wxyz2wxyz1wxyz0"
#define SAU_READY_MARKER_LEN 20

#define LED_GPIO 4

static const char *TAG = "SAU";

static void sau_led_on() {
   gpio_set_level(LED_GPIO, 1);
}

static void sau_led_off() {
    gpio_set_level(LED_GPIO, 0);
}

static void sau_led_blink() {
    sau_led_on();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    sau_led_off();
    vTaskDelay(200 / portTICK_PERIOD_MS);
}

static void sau_led_blink_n(int n) {
    for (int i = 0; i < n; i++) {
        sau_led_blink();
    }
}

static void sau_long_delay() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

static int sau_uart_read(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait) {
    uint32_t num_rcv = 0;
    while (num_rcv < length) {
        int rcv = uart_read_bytes(uart_num, (uint8_t*)buf + num_rcv, length - num_rcv, ticks_to_wait);
        if (rcv < 0) {
            return -1;
        }
        if (0 == rcv) {
            ESP_LOGW(TAG, "uart_read_bytes returned 0");
        }
        num_rcv += rcv;
    }
    return 0;
}

static int sau_uart_write(uart_port_t uart_num, const void *buf, uint32_t length) {
    uint32_t num_sent = 0;
    while (num_sent < length) {
        int sent = uart_write_bytes(uart_num, (uint8_t*)buf + num_sent, length - num_sent);
        if (sent < 0) {
            ESP_LOGE(TAG, "uart_write_bytes failed with return value %d", sent);
            return -1;
        }
        if (0 == sent) {
            ESP_LOGW(TAG, "uart_write_bytes returned 0");
        }
        num_sent += sent;
    }
    return 0;
}

/**
 * @returns 0 on success, -1 on failure
 */
static int sau_frame() {
    //d sau_long_delay(); sau_led_blink_n(2);
    
    camera_fb_t *fb = NULL;
    esp_err_t err = camera_frame_take(&fb);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "camera_frame_take failed, err=%d", err);
        return -1;
    }
    uint32_t frame_size = (uint32_t)fb->len;
    assert (frame_size < 1000000U);
    assert(sizeof(uint32_t) == 4U);
    //e sau_long_delay(); sau_led_blink_n(1);
    //int rv = uart_write_bytes(SAU_PORT_NUM, (const void*)&frame_size, 4U);
    int rv = sau_uart_write(SAU_PORT_NUM, (const void*)&frame_size, 4U);
    //if (4 != rv) {
    if (-1 == rv) {
        //ESP_LOGE(TAG, "uart_write_bytes unexpectedly returned %d (expected 4)", rv);
        ESP_LOGE(TAG, "sau_uart_write failed.");
        return -1;
    }
    //e sau_long_delay(); sau_led_blink_n(2);
    uint8_t* frame_data = fb->buf;
    //const char* frame_data = "hello";
    //frame_size = 5;
    for (uint32_t i=0; i < frame_size; i += SAU_SEND_BUF_SIZE) {
        uint32_t bytes_to_send = (frame_size - i) < SAU_SEND_BUF_SIZE ? (frame_size - i) : SAU_SEND_BUF_SIZE;
        rv = sau_uart_write(SAU_PORT_NUM, (const void*)(frame_data + i), bytes_to_send);
        if (-1 == rv) {
            ESP_LOGE(TAG, "sau_uart_write failed");
            return -1;
        }
    }
    //e sau_long_delay(); sau_led_blink_n(3);

    //rv = uart_write_bytes(SAU_PORT_NUM, (const void*)frame_data, frame_size);
    // if (frame_size != rv) {
    //     ESP_LOGE(TAG, "uart_write_bytes unexpectedly returned %d (expected %lu)", rv, frame_size);
    //     return -1;
    // }
    err = camera_frame_release(fb);

    if (ESP_OK != err) {
        ESP_LOGE(TAG, "camera_frame_release failed, err=%d", err);
        return -1;
    }
    //e sau_long_delay(); sau_led_blink_n(4);
    return 0;
}

static int sau_firmware_update() {
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 (unsigned int)configured->address, (unsigned int)running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, (unsigned int)running->address);
    uint32_t update_size = 0;
    assert(sizeof(uint32_t) == 4U);
    // Read size of update
    int rv = sau_uart_read(SAU_PORT_NUM, (void*)&update_size, 4, portMAX_DELAY);
    // if (4 != rv) {
    //     ESP_LOGE(TAG, "uart_read_bytes unexpectedly returned %d (expected 4)", rv);
    //     return -1;
    // }
    if (-1 == rv) {
        ESP_LOGE(TAG, "sau_uart_read failed");
        return -1;
    }
    //ESP_LOGI(TAG, "update_size=%u", update_size);

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition((const esp_partition_t*)NULL);
    //ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             //update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);
    esp_ota_handle_t ota_handle = 0;
    //sau_long_delay(); sau_led_blink_n(3);
    if (ESP_OK != esp_ota_begin(update_partition, (size_t)update_size, &ota_handle)) {
        ESP_LOGE(TAG, "esp_ota_begin failed");
        return -1;
    }
    //sau_long_delay(); sau_led_blink_n(4);
    // Read new firmware chunk by chunk from uart and pass it to esp_ota_write
    char update_data_buf[SAU_RCV_BUF_SIZE+1] = { 0 };
    char ota_write_data[SAU_RCV_BUF_SIZE+1] = { 0 };

    ESP_LOGI(TAG, "update_size=%lu", update_size);
    ESP_LOGI(TAG, "SAU_RCV_BUF_SIZE=%d", SAU_RCV_BUF_SIZE);
    for (uint32_t i = 0; i < update_size; i += SAU_RCV_BUF_SIZE) {
        assert(i < update_size);
        uint32_t bytes_to_read = (update_size - i) < SAU_RCV_BUF_SIZE ? (update_size - i) : SAU_RCV_BUF_SIZE;
        // 20 ms timeout, TODO change if needed
        //memset(update_data_buf, 0, SAU_RCV_BUF_SIZE);
        //memset(ota_write_data, 0, SAU_RCV_BUF_SIZE);
        //if ((i << 1) > update_size) {
            ESP_LOGI(TAG, "Reading %lu / %lu", i, update_size);
        //}
        rv = sau_uart_read(SAU_PORT_NUM, update_data_buf, bytes_to_read, 100 / portTICK_PERIOD_MS);
        // if (bytes_to_read != rv) {
        //     ESP_LOGE(TAG, "uart_read_bytes unexpectedly returned %d (expected %d)", rv, SAU_RCV_BUF_SIZE);
        //     sau_long_delay(); sau_led_blink_n(4);
        //     return -1;
        //     //break;
        // }
        if (-1 == rv) {
            ESP_LOGE(TAG, "sau_uart_read failed");
            //d sau_long_delay(); sau_led_blink_n(4);
            return -1;
        }
        assert(bytes_to_read <= SAU_RCV_BUF_SIZE);
        memcpy(ota_write_data, update_data_buf, bytes_to_read);
        // ESP_LOGI(TAG, "Writing %lu / %lu", i, update_size);
        if (ESP_OK != esp_ota_write(ota_handle, (const void*)ota_write_data, bytes_to_read)) {
            ESP_LOGE(TAG, "esp_ota_write failed");
            //d sau_long_delay(); sau_led_blink_n(6);
            return -1;
        }
    }

    ESP_LOGI(TAG, "Total bytes written: %lu", update_size);
    //d sau_long_delay(); sau_led_blink_n(3);

    esp_err_t err = esp_ota_end(ota_handle);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "esp_ota_end failed! err=0x%x. ", err);
        return -1;
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
        return -1;
    }
    esp_restart();
    return 0;
}

static void sau_task(void *arg)
{
    //Wait 2 seconds
    vTaskDelay(2000 / portTICK_PERIOD_MS);


    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = SAU_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(SAU_PORT_NUM, SAU_RCV_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(SAU_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SAU_PORT_NUM, SAU_TXD, SAU_RXD, SAU_RTS, SAU_CTS));

    // uart_hw_flowcontrol_t uart_hw_flowctrl = UART_HW_FLOWCTRL_DISABLE;
    // //uart_get_hw_flow_ctrl(SAU_PORT_NUM, &uart_hw_flowctrl);
    // esp_err_t err = uart_set_hw_flow_ctrl(SAU_PORT_NUM, uart_hw_flowctrl, 0);
    // if (ESP_OK != err) {
    //     ESP_LOGE(TAG, "uart_set_hw_flow_ctrl failed, err=%d", err);
    // }
    // err = uart_set_sw_flow_ctrl(SAU_PORT_NUM, false, 0, 0);
    // if (ESP_OK != err) {
    //     ESP_LOGE(TAG, "uart_set_sw_flow_ctrl failed, err=%d", err);
    // }

    ESP_LOGI(TAG, "UART%d configured:", SAU_PORT_NUM);
    // Send SAU_READY_MARKER via uart
    const char* readyMarker = SAU_READY_MARKER;
    //int rv = uart_write_bytes(SAU_PORT_NUM, readyMarker, SAU_READY_MARKER_LEN);
    int rv = sau_uart_write(SAU_PORT_NUM, readyMarker, SAU_READY_MARKER_LEN);
    //if (SAU_READY_MARKER_LEN != rv) {
    if (-1 == rv) {
        //ESP_LOGE(TAG, "uart_write_bytes unexpectedly returned %d (expected %d)", rv, SAU_READY_MARKER_LEN);
        ESP_LOGE(TAG, "sau_uart_write failed.");
    }

    char request;
    //e sau_led_blink_n(5);
    while (1) {
        // Read request (wait indefinitely)
        //sau_long_delay(); sau_led_blink_n(3);
        int len = uart_read_bytes(SAU_PORT_NUM, &request, 1, portMAX_DELAY);
        //sau_long_delay(); sau_led_blink_n(2);

        if (len != 1) {
            ESP_LOGW(TAG, "uart_read_bytes unexpectedly returned %d (expected 1)", len);
        }

        int rv = 0;
        switch (request) {
            case 'f':
                rv = sau_frame();
                if (0 != rv) {
                    ESP_LOGE(TAG, "sau_frame failed with rv=%d", rv);
                }
                sau_led_off();
                break;
            case 'u':
                rv = sau_firmware_update();
                if (0 != rv) {
                    ESP_LOGE(TAG, "sau_firmware_update failed with rv=%d", rv);
                }
                break;
            default:
                break;
        }
        // // Write data back to the UART
        // uart_write_bytes(SAU_PORT_NUM, (const char *) data, len);
        // if (len) {
        //     data[len] = '\0';
        //     ESP_LOGI(TAG, "Recv str: %s", (char *) data);
        // }
    }
}

void app_main(void)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "SAU");
    sau_led_off();
    camera_init(/*FRAMESIZE_UXGA, 12*/);
    esp_ota_mark_app_valid_cancel_rollback();
    xTaskCreate(sau_task, "sau_task", SAU_TASK_STACK_SIZE, NULL, 10, NULL);
}