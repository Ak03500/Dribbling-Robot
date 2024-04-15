#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h> 
#include <stdio.h>
#include "pid.h"
#include <time.h>
static const char *TAG = "example";

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90     // Maximum angle
#define UART_TX_PIN GPIO_NUM_1
#define UART_RX_PIN GPIO_NUM_3
#define UART_NUM UART_NUM_0
#define SERVOX_PULSE_GPIO        26        // GPIO for the first servo
#define SERVOY_PULSE_GPIO         14        // GPIO for the second servo
#define LED_GPIO                   13        // GPIO for the LED
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms
#define BUF_SIZE 1024
PID x;
PID y;
double setpointX = 31;
double setpointY = 22;


static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

int map_pid_output_to_pwm(double pid_output) {
    // Example conversion: 1500 µs is the center position of the servo.
    int center_pwm = 1500;
    double scale = 10.0;  // This scale factor would be adjusted based on your specific hardware setup
    int pwm_signal = center_pwm + (int)(pid_output * scale);

    // Clamping the PWM signal to ensure it stays within the servo range
    if (pwm_signal > 1700) {
        pwm_signal = 1700;
    } else if (pwm_signal < 1200) {
        pwm_signal = 1200;
    }

    return pwm_signal;
}


void extract_numbers(uint8_t *data, int *num1, int *num2) {
    char *token = strtok((char *)data, ",");  // Split the data at the comma
    if (token != NULL) {
        *num1 = atoi(token);  // Convert the first part to an integer
        token = strtok(NULL, ",");  // Get the next part of the string
        if (token != NULL) {
            *num2 = atoi(token);  // Convert the second part to an integer
        }
    }
}
static void parse_uart_command(const char* cmd, int* pwmX, int* pwmY) {
    // Expecting input in the format "PWMX:1500,PWMY:1500"
    sscanf(cmd, "PWMX:%d,PWMY:%d", pwmX, pwmY);
}

void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = 460800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
}



// void set_ServoY_angle(void *parameter){
// uart_init();
//  ESP_LOGI(TAG, "Create timer and operator");
//     mcpwm_timer_handle_t timer = NULL;
//     mcpwm_timer_config_t timer_config = {
//         .group_id = 0,
//         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//         .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
//         .period_ticks = SERVO_TIMEBASE_PERIOD,
//         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

//     mcpwm_oper_handle_t oper = NULL;
//     mcpwm_operator_config_t operator_config = {
//         .group_id = 0, // operator must be in the same group to the timer
//     };
//     ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

//     ESP_LOGI(TAG, "Connect timer and operator");
//     ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

//     ESP_LOGI(TAG, "Create comparator and generator from the operator");
//     mcpwm_cmpr_handle_t comparator = NULL;
//     mcpwm_comparator_config_t comparator_config = {
//         .flags.update_cmp_on_tez = true,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

//     mcpwm_gen_handle_t generator = NULL;
//     mcpwm_generator_config_t generator_config = {
//         .gen_gpio_num = SERVOY_PULSE_GPIO,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

//     // set the initial compare value, so that the servo will spin to the center position
//     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

//     ESP_LOGI(TAG, "Set generator action on timer and compare event");
//     // go high on counter empty
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
//                                                               MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
//     // go low on compare threshold
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
//                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

//     ESP_LOGI(TAG, "Enable and start timer");
//     ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
//     ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
//     //int angle = *(int *)parameter;
//     //int step = 2;
//      while (1) {
//         uint8_t data;
//         if (uart_read_bytes(UART_NUM, &data, 1, pdMS_TO_TICKS(1000)) > 0) {
//             ESP_LOGI(TAG, "Received data: %d", data);
//             if (data == '0') {
//                 // Move Servo 1 and stop Servo 2
//                 ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(40)));
//             } else if (data == '1') {
//                 // Stop Servo 1 and move Servo 2
//                  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
//             }
//         }
//     }
// }

// void set_ServoX_angle(void *parameter){
//  uart_init();
//  ESP_LOGI(TAG, "Create timer and operator");
//     mcpwm_timer_handle_t timer = NULL;
//     mcpwm_timer_config_t timer_config = {
//         .group_id = 0,
//         .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
//         .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
//         .period_ticks = SERVO_TIMEBASE_PERIOD,
//         .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

//     mcpwm_oper_handle_t oper = NULL;
//     mcpwm_operator_config_t operator_config = {
//         .group_id = 0, // operator must be in the same group to the timer
//     };
//     ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

//     ESP_LOGI(TAG, "Connect timer and operator");
//     ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

//     ESP_LOGI(TAG, "Create comparator and generator from the operator");
//     mcpwm_cmpr_handle_t comparator = NULL;
//     mcpwm_comparator_config_t comparator_config = {
//         .flags.update_cmp_on_tez = true,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

//     mcpwm_gen_handle_t generator = NULL;
//     mcpwm_generator_config_t generator_config = {
//         .gen_gpio_num = SERVOX_PULSE_GPIO,
//     };
//     ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

//     // set the initial compare value, so that the servo will spin to the center position
//     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

//     ESP_LOGI(TAG, "Set generator action on timer and compare event");
//     // go high on counter empty
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
//                                                               MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
//     // go low on compare threshold
//     ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
//                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

//     ESP_LOGI(TAG, "Enable and start timer");
//     ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
//     ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
//     //int angle = *(int *)parameter;
//     //int step = 2;
//     while (1) {
//         uint8_t data;
//         if (uart_read_bytes(UART_NUM, &data, 1, pdMS_TO_TICKS(1000)) > 0) {
//             ESP_LOGI(TAG, "Received data: %d", data);
//             if (data == '1') {
//                 // Move Servo 1 and stop Servo 2
//                 ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(40)));
//             } else if (data == '0') {
//                 // Stop Servo 1 and move Servo 2
//                  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
//             }
//         }
//     }
// }

void control_servos(void *pvParameters){
    uart_init();
    PID_Init(&x, 0.1, 0.02, 0.1);  // Initialize PID controllers with tuning parameters
    PID_Init(&y, 0.1, 0.02, 0.1);
    PID_SetSetpoint(&x, setpointX);
    PID_SetSetpoint(&y, setpointY);
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVOX_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////define Y servo //////////////////////////////////////

    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer2 = NULL;
    mcpwm_timer_config_t timer_config2 = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config2, &timer2));

    mcpwm_oper_handle_t oper2 = NULL;
    mcpwm_operator_config_t operator_config2 = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config2, &oper2));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer2));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator2 = NULL;
    mcpwm_comparator_config_t comparator_config2 = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config2, &comparator2));

    mcpwm_gen_handle_t generator2 = NULL;
    mcpwm_generator_config_t generator_config2 = {
        .gen_gpio_num = SERVOY_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &generator_config2, &generator2));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator2, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer2));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer2, MCPWM_TIMER_START_NO_STOP));
  

    // uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    // while (1) {
    //     int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE - 1, portTICK_PERIOD_MS);
    //     if (len > 0) {
    //         ESP_LOGI(TAG, "Received: %s", (char *)data);
    //         int pwmX;
    //         int pwmY;
    //         //ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(20)));
    //         if (sscanf((char *)data, "%d,%d", &pwmX, &pwmY) == 2) {
    //             ESP_LOGI(TAG, "Parsed PWMX: %d, PWMY: %d", pwmX, pwmY);
    //             if (pwmX == 1) {
    //                 // Move X servo
    //                 ESP_LOGI(TAG, "Moving X servo");
    //                 ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(20)));
    //                 ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(20)));
    //             } 
    //         } else {
    //             ESP_LOGE(TAG, "Failed to parse PWM values.");
    //         }
    //         if (len) {
    //         data[len] = '\0';
    //         ESP_LOGI(TAG, "Recv str: %s", (char *) data);
    //     }
    //     }
    // }
    // while (1) {
    //     uint8_t data;
    //     if (uart_read_bytes(UART_NUM, &data, 1, pdMS_TO_TICKS(1000)) > 0) {
    //         //ESP_LOGI(TAG, "Received data: %d", data);
    //         if (data == 'A') {
    //             // Move Servo 1 and stop Servo 2
    //             ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(40))); //move X servo to 40
    //             ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(0))); //move Y servo to 0
    //         } else if (data == 'B') {
    //             // Stop Servo 1 and move Servo 2
    //              ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0))); //move X servo to 0
    //              ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, example_angle_to_compare(40))); //move Y servo to 0
    //         } 
    //     }
    // }

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    while (1) {
       //int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS) > 0) {
            //ESP_LOGI(TAG, "Received data: %d", data)
            int posx = 0;
            int posy = 0;
            extract_numbers(data, &posx, &posy);
            // if ((numx < 1600 || numx > 1300) && (numy < 1600 || numy > 1300) ){
            //     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, numy));
            //     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, numx));
            // }
            // char x1 = data[0];
            // char x2 = data[1];
            // char x3 = data[2];
            // char x4 = data[3];
            // char y1 = data[5];
            // char y2 = data[6];
            // char y3 = data[7];
            // char y4 = data[8];
            int pidx = PID_Update(&x, posx, (double)clock() / CLOCKS_PER_SEC);
            int pidy = PID_Update(&y, posy, (double)clock() / CLOCKS_PER_SEC);

            int pwmx = map_pid_output_to_pwm(-pidx);
            int pwmy = map_pid_output_to_pwm(-pidy);
            // int control_signal_x = PID_Update(&x, errorX, (double)clock() / CLOCKS_PER_SEC);
            // int control_signal_y = PID_Update(&y, errorY, (double)clock() / CLOCKS_PER_SEC);
            // int combinedx = (x1 - '0')*1000 + (x2 - '0')*100 + (x3 - '0')*10 + (x4 - '0');
            // int combinedy = (y1 - '0')*1000 + (y2 - '0')*100 + (y3 - '0')*10 + (y4 - '0');
            if ((pwmx < 1600 || pwmx > 1300) && (pwmy < 1600 || pwmy > 1300) ){
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pwmy));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, pwmx));
            }
           
        }
        
    }

    
}
void app_main(void)
{
   
    ESP_LOGI(TAG, "Starting UART Servo Control Task...");
    xTaskCreate(control_servos, "servoX_task", 4096,NULL, 4, NULL);
   

}