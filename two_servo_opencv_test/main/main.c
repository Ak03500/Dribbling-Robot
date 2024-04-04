#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "example";

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        0     // Maximum angle

#define SERVO_PULSE_GPIO_X         26        // GPIO for the first servo
#define SERVO_PULSE_GPIO_Y         14        // GPIO for the second servo
#define LED_GPIO                   13        // GPIO for the LED
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void configure_uart() {
    uart_config_t uart_config = {
        .baud_rate = 460800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
}

void configure_led() {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

void servo_task(void *pvParameters) {
    configure_uart();
    configure_led();

    mcpwm_cmpr_handle_t comparator_x = NULL;
    mcpwm_cmpr_handle_t comparator_y = NULL;

    while (1) {
        uint8_t data;
        if (uart_read_bytes(UART_NUM_0, &data, 1, portMAX_DELAY) > 0) {
            if (data == '0') {
                ESP_LOGI(TAG, "Received '0'. Starting X servo and turning on LED.");
                int angle_x = 0;   // Initial angle for servo X
                int step_x = -1;    // Step size for servo X movement

                while (angle_x >= SERVO_MIN_DEGREE - 10) {
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_x, example_angle_to_compare(angle_x)));
                    angle_x += step_x;
                    vTaskDelay(pdMS_TO_TICKS(50)); // Reduced delay for smoother sweep
                }
                gpio_set_level(LED_GPIO, 1); // Turn on LED
            } else if (data == '1') {
                ESP_LOGI(TAG, "Received '1'. Starting Y servo and turning off LED.");
                int angle_y = SERVO_MIN_DEGREE; // Initial angle for servo Y
                int step_y = 2;                  // Step size for servo Y movement

                while (angle_y >= SERVO_MIN_DEGREE - 78) { // Adjusted range for servo Y
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_y, example_angle_to_compare(angle_y)));
                    angle_y -= step_y; // Decrease angle for Y servo
                    vTaskDelay(pdMS_TO_TICKS(200)); // Reduced delay for smoother sweep
                }
                gpio_set_level(LED_GPIO, 0); // Turn off LED
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Configure UART...");
    configure_uart();

    ESP_LOGI(TAG, "Create timer and operator for servo X");
    mcpwm_timer_handle_t timer_x = NULL, timer_y = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_x));
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_y));

    mcpwm_oper_handle_t oper_x = NULL, oper_y = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group as the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_x));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_y));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_x, timer_x));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_y, timer_y));

    ESP_LOGI(TAG, "Create comparator and generator from the operator for servo X");
    mcpwm_cmpr_handle_t comparator_x = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_x, &comparator_config, &comparator_x));

    mcpwm_gen_handle_t generator_x = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO_X,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_x, &generator_config, &generator_x));

    // set the initial compare value for servo X, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_x, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event for servo X");
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_x,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_x,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_x, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer for servo X");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer_x));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_x, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Create comparator and generator from the operator for servo Y");
    mcpwm_cmpr_handle_t comparator_y = NULL;
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_y, &comparator_config, &comparator_y));

    mcpwm_gen_handle_t generator_y = NULL;
    generator_config.gen_gpio_num = SERVO_PULSE_GPIO_Y;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_y, &generator_config, &generator_y));

    // set the initial compare value for servo Y, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_y, example_angle_to_compare(SERVO_MIN_DEGREE)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event for servo Y");
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_y,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_y,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_y, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer for servo Y");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer_y));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_y, MCPWM_TIMER_START_NO_STOP));

    xTaskCreate(servo_task, "servo_task", 2048, NULL, 5, NULL);
}
