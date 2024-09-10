#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/socket.h>
#include <netdb.h>
#include <cJSON.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "rom/ets_sys.h"

#define PORT 100
#define NUM_MOTORS 6
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 4
#define I2C_MASTER_SCL_IO 5
#define I2C_MASTER_FREQ_HZ 100000
#define STEPS_PER_REVOLUTION 200
#define STEPS_PER_DEGREE (STEPS_PER_REVOLUTION / 360.0)
#define TCA9548A_ADDRESS 0x70
#define AS5600_ADDRESS 0x36
#define MICROSTEPPING 8

// Global variables to track steps for J2 and J3
long int step_count_J2 = 0;
long int step_count_J3 = 0;

float STEPS_PER_REV_J2 = 200 * 50 * (4397.0 / 4913.0);  // Steps per revolution with 50:1 reducer
float STEPS_PER_REV_J3 = 200 * 5.18;                    // Steps per revolution with 5:1 reducer

static const char *TAG = "ROBOT_CONTROL";

#define WIFI_SSID "WIFI NAME"
#define WIFI_PASS "WIFI PASSWORD"

// Define stepper motor GPIO pins
static const gpio_num_t STEP_PINS[NUM_MOTORS] = {GPIO_NUM_48, GPIO_NUM_0, GPIO_NUM_36, GPIO_NUM_38, GPIO_NUM_1};
static const gpio_num_t DIR_PINS[NUM_MOTORS] = {GPIO_NUM_47, GPIO_NUM_45, GPIO_NUM_35, GPIO_NUM_37, GPIO_NUM_2};

// Home positions encoder angles
float HOME_ENCODER[NUM_MOTORS] = {47.0, 0, 0, 67.0, 80.0, 0.0};


void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Selects the I2C channel on the TCA9548A multiplexer
esp_err_t tca9548a_select_channel(uint8_t channel)
{
    uint8_t control_reg = (1 << channel);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TCA9548A_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control_reg, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

// Returns the AS5600 angle in degrees
float read_angle()
{
    uint8_t high_byte, low_byte;
    uint16_t raw_angle;

    // Read the low byte
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0D, true);  // Register address for raw angle (7:0)
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &low_byte, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Read the high byte
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x0C, true);  // Register address for raw angle (11:8)
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &high_byte, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Combine the high and low bytes
    raw_angle = ((uint16_t)high_byte << 8) | low_byte;

    // 12-bit resolution = 4096 steps over 360 degrees
    return (raw_angle * 0.087890625);  // Multiply by 360/4096
}


void move_stepper_to_angle(float target_angles[6]) {
    float angle_difs[6] = {0};
    int dirs[6] = {0};
    float steps[6] = {0};

    // Calculate angle differences and directions for each motor
    for (int motor_index = 0; motor_index < 6; motor_index++) {
        tca9548a_select_channel(motor_index);

        // Read the angle from the encoder and subtract it by the initial home position's offset
        float current_angle = read_angle() - HOME_ENCODER[motor_index];

        if (current_angle < 0) {
            current_angle += 360.0;
        }

        // Reverse the direction of the encoder reading
        current_angle = 360.0 - current_angle;


        // Calculating angle difference for J2 and J3 using the step counter
        if (motor_index == 1) {
            // For J2, calculate angle using step counter considering 50:1 reducer
            current_angle = (((float)step_count_J2 / STEPS_PER_REV_J2) * 360.0) / 8.0;

            if ((current_angle >= 0.0) && (current_angle <= 180.0)) {
                if (target_angles[motor_index] >= current_angle) {
                    dirs[motor_index] = 0;
                    angle_difs[motor_index] = target_angles[motor_index] - current_angle;
                }
                else {
                    dirs[motor_index] = 1;
                    angle_difs[motor_index] = current_angle - target_angles[motor_index];  
                }
            }
            else if (current_angle < 0) {
                if (target_angles[motor_index] >= current_angle) {
                    dirs[motor_index] = 0;
                    angle_difs[motor_index] = -1 * (current_angle - target_angles[motor_index]);    
                }
                else {
                    dirs[motor_index] = 1;
                    angle_difs[motor_index] = -1 * (target_angles[motor_index] - current_angle);  
                }
            }
        }
        else if (motor_index == 2) {
            // For J3, calculate angle using step counter considering 5:1 reducer
            current_angle = (((float)step_count_J3 / STEPS_PER_REV_J3) * 360.0) / 8.0;

            if ((current_angle >= 0.0) && (current_angle <= 180.0)) {
                if (target_angles[motor_index] >= current_angle) {
                    dirs[motor_index] = 1;
                    angle_difs[motor_index] = target_angles[motor_index] - current_angle;
                }
                else {
                    dirs[motor_index] = 0;
                    angle_difs[motor_index] = current_angle - target_angles[motor_index];  
                }
            }
            else if (current_angle < 0) {
                if (target_angles[motor_index] >= current_angle) {
                    dirs[motor_index] = 1;
                    angle_difs[motor_index] = -1 * (current_angle - target_angles[motor_index]);    
                }
                else {
                    dirs[motor_index] = 0;
                    angle_difs[motor_index] = -1 * (target_angles[motor_index] - current_angle);  
                }
            }
        }
        // Calculating angle difference for J1, J4, J5, J6
        else {
            if ((current_angle >= 0.0) && (current_angle <= 180.0)) {
                if (target_angles[motor_index] >= current_angle) {
                    dirs[motor_index] = 1;
                    angle_difs[motor_index] = target_angles[motor_index] - current_angle;
                }
                else {
                    dirs[motor_index] = 0;
                    angle_difs[motor_index] = current_angle - target_angles[motor_index];     
                }
            }
            else if (current_angle > 180.0) {
                current_angle = current_angle - 360.0;
                if (target_angles[motor_index] >= current_angle) {
                    dirs[motor_index] = 1;
                    angle_difs[motor_index] = -1 * (current_angle - target_angles[motor_index]);    
                }
                else {
                    dirs[motor_index] = 0;
                    angle_difs[motor_index] = -1 * (target_angles[motor_index] - current_angle);  
                }
            }
        }

        // Calculate steps based on angle differences
        if (motor_index == 1) {
            steps[motor_index] = angle_difs[motor_index] * (STEPS_PER_REV_J2 / 360.0) * 8;
        } 
        else if (motor_index == 2) {
            steps[motor_index] = angle_difs[motor_index] * (STEPS_PER_REV_J3 / 360.0) * 8;
        } 
        else {
            steps[motor_index] = angle_difs[motor_index] * STEPS_PER_DEGREE * 8;
        }
    }

    // Move each motor the required number of steps
    long max_steps = 0;
    for (int motor_index = 0; motor_index < 6; motor_index++) {
        if (steps[motor_index] > max_steps) {
            max_steps = steps[motor_index];
        }
    }

    for (long step = 0; step < max_steps; step++) {
        for (int motor_index = 0; motor_index < 6; motor_index++) {
            if (step < steps[motor_index]) {
                gpio_set_level(DIR_PINS[motor_index], dirs[motor_index]);
                gpio_set_level(STEP_PINS[motor_index], 1);
                ets_delay_us(2000);
                gpio_set_level(STEP_PINS[motor_index], 0);
                ets_delay_us(2000);

                // Update step count based on direction
                if (motor_index == 1) {
                    step_count_J2 += (dirs[motor_index] == 0) ? 1 : -1;
                } 
                else if (motor_index == 2) {
                    step_count_J3 += (dirs[motor_index] == 1) ? 1 : -1;
                }
            }
        }
    }
}


static void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}


static void handle_robot_control(int sock)
{
    char rx_buffer[1024];
    while (1) {
        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len > 0) {
            rx_buffer[len] = 0;  // Null-terminate the received data
            ESP_LOGI(TAG, "Received: %s", rx_buffer);

            // Parse JSON
            cJSON *root = cJSON_Parse(rx_buffer);
            if (root != NULL) {
                double theta_1 = cJSON_GetObjectItem(root, "theta_1")->valuedouble;
                double theta_2 = cJSON_GetObjectItem(root, "theta_2")->valuedouble;
                double theta_3 = cJSON_GetObjectItem(root, "theta_3")->valuedouble;
                double theta_4 = cJSON_GetObjectItem(root, "theta_4")->valuedouble;
                double theta_5 = cJSON_GetObjectItem(root, "theta_5")->valuedouble;
                double theta_6 = cJSON_GetObjectItem(root, "theta_6")->valuedouble;

                ESP_LOGI(TAG, "Joint Angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", theta_1, theta_2, theta_3, theta_4, theta_5, theta_6);

                float target_angles[NUM_MOTORS] = {theta_1, theta_2, theta_3, theta_4, theta_5, theta_6};

                move_stepper_to_angle(target_angles);

                cJSON_Delete(root);

                // Send confirmation back to Python
                const char *ack_msg = "{\"status\": \"Completed\"}";
                send(sock, ack_msg, strlen(ack_msg), 0);
            }
        }
        // Wait for the next set of angles
    }
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    listen(listen_sock, 1);

    while (1) {
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 source_addr;
        uint addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        handle_robot_control(sock);

        shutdown(sock, 0);
        close(sock);
    }
    close(listen_sock);
    vTaskDelete(NULL);
}


void app_main(void) {
    // Initialize Wi-Fi
    nvs_flash_init();
    wifi_init_sta();
    
    // Initialize I2C
    i2c_master_init();

    // Initialize GPIO for stepper motors
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_set_direction(STEP_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_direction(DIR_PINS[i], GPIO_MODE_OUTPUT);
    }

    // Home motors
    float HOME_ANGLES[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
    move_stepper_to_angle(HOME_ANGLES);
    
    // Start the TCP server
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}