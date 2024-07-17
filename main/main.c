/*
 * ESP32 
 * 
 * This library provides functions to interface with an ultrasonic sensor
 * and control a servo motor for creating a radar-like scanning motion.
 * 
 * Components used:
 * - ESP32 microcontroller
 * - HC-SR04 Ultrasonic Sensor
 * - Servo Motor
 * 
 * Pin Configuration:
 * - TRIG_PIN: GPIO pin connected to the trigger pin of the ultrasonic sensor
 * - ECHO_PIN: GPIO pin connected to the echo pin of the ultrasonic sensor
 * - SERVO_PIN: GPIO pin connected to the signal pin of the servo motor
 * 
 * Author: M.Saeed
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"

// GPIO pin for ultrasonic sensor trigger
#define TRIG_PIN 5  
// GPIO pin for ultrasonic sensor echo
#define ECHO_PIN 18 
// GPIO pin for servo motor     
#define SERVO_PIN 15     

// Tag for logging
static const char *TAG = "Radar"; 

/**
 * @brief Initializes the GPIO pins for the ultrasonic sensor.
 */
void init_ultrasonic_sensor()
{
    gpio_pad_select_gpio(TRIG_PIN);
    // Set TRIG_PIN as output
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT); 

    gpio_pad_select_gpio(ECHO_PIN);
    // Set ECHO_PIN as input
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT); 
}

/**
 * @brief Measures the distance using the ultrasonic sensor.
 * 
 * @return Distance in centimeters.
 */
uint32_t measure_distance()
{
    gpio_set_level(TRIG_PIN, 0); 
    // Ensure TRIG_PIN is low initially
    ets_delay_us(2);                    
    // Small delay
    gpio_set_level(TRIG_PIN, 1);        
    // Send 10us pulse to TRIG_PIN
    ets_delay_us(10);                   
    // Wait 10us
    gpio_set_level(TRIG_PIN, 0);        
    // Turn off TRIG_PIN


    while (gpio_get_level(ECHO_PIN) == 0); 
    // Wait for the pulse to start
    int64_t start_time = esp_timer_get_time();

    while (gpio_get_level(ECHO_PIN) == 1); 
    // Wait for the pulse to end
    int64_t end_time = esp_timer_get_time();

    // Calculate distance based on time difference
    uint32_t distance = ((end_time - start_time) * 0.034 / 2);
    return distance;
}

/**
 * @brief Initializes the servo motor using LEDC peripheral.
 */
void init_servo()
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_16_BIT,
        .freq_hz = 50,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = SERVO_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);
}

/**
 * @brief Sets the angle of the servo motor.
 * 
 * @param angle Desired angle in degrees (0 to 150).
 */
void set_servo_angle(int angle)
{
    int duty = (angle * (8192 / 300)) + 4096; 
    // Calculate duty cycle for the servo motor
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/**
 * @brief Task that performs radar-like scanning motion.
 * 
 * This task initializes the ultrasonic sensor and servo motor,
 * and performs a continuous scan from 0 to 150 degrees and back,
 * measuring distance at each angle.
 * 
 * @param pvParameter Unused task parameter.
 */
void radar_task(void *pvParameter)
{
    init_ultrasonic_sensor(); 
    // Initialize ultrasonic sensor GPIO
    init_servo();             
    // Initialize servo motor using LEDC

    while (1)
    {
        // Scan from 0 to 150 degrees
        for (int angle = 0; angle <= 150; angle += 10)
        {
            set_servo_angle(angle);                     
            // Set servo to current angle
            vTaskDelay(pdMS_TO_TICKS(500));             
            // Delay for servo movement
            uint32_t distance = measure_distance();     
            // Measure distance
            ESP_LOGI(TAG, "Angle: %d, Distance: %d cm", angle, distance); 
            // Log angle and distance
        }

        // Scan back from 150 to 0 degrees
        for (int angle = 150; angle >= 0; angle -= 10)
        {
            set_servo_angle(angle);                     
            // Set servo to current angle
            vTaskDelay(pdMS_TO_TICKS(500));             
            // Delay for servo movement
            uint32_t distance = measure_distance();     
            // Measure distance
            ESP_LOGI(TAG, "Angle: %d, Distance: %d cm", angle, distance); 
            // Log angle and distance
        }
    }
}

/**
 * @brief Main application entry point.
 */
void app_main()
{
    xTaskCreate(&radar_task, "radar_task", 2048, NULL, 5, NULL); 
    // Create radar task
}
