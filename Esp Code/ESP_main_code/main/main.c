#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "C:/Espressif/frameworks/esp-idf-v5.2.1/examples/get-started/blink/build/config/sdkconfig.h""

#define UART_NUM UART_NUM_0
#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3
#define BLINK_GPIO 12
#define SERVO_PIN 26

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microseconds
#define SERVO_MIN_DEGREE -90       // Minimum angle
#define SERVO_MAX_DEGREE 90        // Maximum angle



#define STEP_MOTOR_GPIO_EN       0
#define STEP_MOTOR_GPIO_DIR      25
#define STEP_MOTOR_GPIO_STEP     26
#define STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution


static const char *TAG = "example";
static uint8_t s_servo_state = 0; // 0: stopped, 1: sweeping
static uint8_t s_led_state = 1;   // Initial state: LED on

void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 460800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);
}

void configure_led(void) {
    ESP_LOGI(TAG, "Example configured to control GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, s_led_state); // Initially, set LED state
}

void configure_servo(void) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;  
    pwm_config.cmpr_a = 0;      
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void sweep_servo(void *pvParameters) {
    while (1) {
        if (s_servo_state == 1) { // If servo should sweep
            for (int angle = SERVO_MIN_DEGREE; angle <= SERVO_MAX_DEGREE; angle += 5) {
                uint32_t duty_us = (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * (angle - SERVO_MIN_DEGREE) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_us);
                vTaskDelay(pdMS_TO_TICKS(20)); // Delay for servo to reach the desired position
            }
            for (int angle = SERVO_MAX_DEGREE; angle >= SERVO_MIN_DEGREE; angle -= 5) {
                uint32_t duty_us = (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * (angle - SERVO_MIN_DEGREE) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_us);
                vTaskDelay(pdMS_TO_TICKS(20)); // Delay for servo to reach the desired position
            }
        }
        else { // Stop the servo
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void led_task(void *pvParameters) {
    configure_led();

    while (1) {
        uint8_t data;
        if (uart_read_bytes(UART_NUM, &data, 1, 1000 / portTICK_PERIOD_MS) > 0) {
            if (data == '1') {
                ESP_LOGI(TAG, "Received '1', turning LED OFF and starting servo sweep");
                s_servo_state = 1; // Start servo sweep
                s_led_state = 0;    // Turn off LED
                gpio_set_level(BLINK_GPIO, s_led_state); // Update LED state
            } else if (data == '0') {
                ESP_LOGI(TAG, "Received '0', turning LED ON and stopping servo");
                s_servo_state = 0; // Stop servo
                s_led_state = 1;   // Turn on LED
                gpio_set_level(BLINK_GPIO, s_led_state); // Update LED state
            }
        }
    }
}

void app_main() {
    uart_init();
    configure_servo();
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(sweep_servo, "sweep_servo", 2048, NULL, 5, NULL);
}
