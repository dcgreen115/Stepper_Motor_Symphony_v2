#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#define BUF_SIZE 1024

static void echo_task(void) {
    // Configure UART parameters and install the driver
    uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // A temporary buffer for the incoming data
    uint8_t* data = malloc(BUF_SIZE);

    while (true) {
        // Read data
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);

        // Write it back
        uart_write_bytes(UART_NUM_0, (const char*) data, len);
    }
}

void app_main(void) {
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
}


