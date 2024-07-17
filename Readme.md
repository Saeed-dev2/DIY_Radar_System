# DIY Radar System

This project demonstrates a radar system using an ESP32, an ultrasonic sensor (HC-SR04), and a servo motor. The system measures distances at different angles and logs the results.

## Components

- ESP32 Development Board
- HC-SR04 Ultrasonic Sensor
- Servo Motor
- Jumper Wires
- Breadboard

## Pin Configuration

| Component          | Pin Name       | ESP32 Pin |
|--------------------|----------------|-----------|
| Ultrasonic Sensor  | Trig           | GPIO 5    |
|                    | Echo           | GPIO 18   |
| Servo Motor        | Signal         | GPIO 15   |

## Code Explanation

1. **Initialization**:
   - The `init_ultrasonic_sensor` function configures the GPIO pins for the ultrasonic sensor.
   - The `init_servo` function configures the LEDC module to control the servo motor.

2. **Distance Measurement**:
   - The `measure_distance` function sends a trigger signal to the ultrasonic sensor and measures the echo time to calculate the distance.

3. **Servo Control**:
   - The `set_servo_angle` function calculates the duty cycle for a given angle and updates the servo motor position.

4. **Radar Task**:
   - The `radar_task` function sweeps the servo motor from 0 to 150 degrees and back, measuring and logging the distance at each angle.

## How to Use

1. **Setup**:
   - Connect the components as per the pin Configration.
   - Flash the code to your ESP32 using the ESP-IDF.

2. **Run**:
   - Power up the ESP32.
   - Open a serial monitor to view the logged distance measurements.

## Requirements

- ESP-IDF (Espressif IoT Development Framework)

## Installation

1. **Install ESP-IDF**:
   - Follow the instructions from the [official ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).

2. **Clone the Repository**:
   ```sh
   git clone https://github.com/yourusername/esp32-radar-system.git
   cd esp32-radar-system
3. **Build and Flash**:
    ```sh
    idf.py build
    idf.py flash
    idf.py monitor
## Author
`M.Saeed`

## Acknowledgements
Special thanks to the ESP-IDF and FreeRTOS communities for their excellent resources and support.