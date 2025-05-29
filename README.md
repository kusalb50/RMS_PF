![20250108_105850 (1)](https://github.com/user-attachments/assets/0f70d1c2-0c2c-458c-a4a9-42cfed6c68f7)

# Power Factor Monitoring System using ESP32

This Arduino-based project demonstrates a real-time power factor monitoring system utilizing an ESP32 microcontroller. It analyzes analog input signals from voltage and current sensors to calculate and display RMS voltage, RMS current, and the power factor (PF) of an AC load. The system is designed for educational purposes, embedded systems learning, and rapid prototyping.

## ⚙️ Hardware Components

The following hardware components are essential for building this system:

* **ESP32 NodeMCU (Wroom-32):** A powerful dual-core microcontroller with integrated Wi-Fi capabilities, serving as the brain of the system.
* **ZMPT101B Voltage Sensor:** Provides accurate and isolated AC voltage measurements.
* **ACS712-20A Current Sensor:** Measures AC current up to 20 Amperes.
* **16x2 LCD Display:** A standard character LCD used to display live voltage, current, and power factor values directly on the device.
* **Custom-built Test Rig:** Includes necessary power terminals, a voltage regulator, display mounting, and other components for a portable, plug-and-play setup.

## 📚 Libraries Used

This project leverages several key Arduino and ESP-IDF libraries to manage multitasking, networking, sensor data processing, and display functions:

* **`freertos/FreeRTOS.h`, `/task.h`, `/semphr.h`:** Enables multi-core task management on the ESP32. These are crucial for separating data acquisition (ISR) from data processing, display, and data upload tasks, ensuring efficient and real-time operation.
* **`WiFi.h`, `WiFiClient.h`:** Used to establish Wi-Fi connectivity, allowing the ESP32 to upload power factor and RMS data to a remote server or cloud service.
* **`HTTPClient.h`:** Facilitates making HTTP POST/GET requests for uploading collected data to platforms like Blynk, ThingSpeak, or a custom server.
* **`Filters.h`:** Employed for real-time RMS calculation of analog voltage and current signals. This library simplifies signal filtering and ensures accurate energy measurement.
* **`cmath`:** Provides essential mathematical functions such as `sqrt()` and trigonometric calculations, which are fundamental for power factor computations.
* **`esp_timer.h`:** Utilizes the ESP32’s high-resolution timer functions for precise sampling and timing within the Interrupt Service Routine (ISR).
* **`Wire.h`:** Enables I2C communication between the ESP32 and the LCD display module.
* **`LiquidCrystal_I2C.h`:** Controls the I2C LCD display, simplifying text and data output.

## 🏗️ System Overview

The system is designed for efficient and accurate real-time power factor monitoring:

* **Sampling:** An Interrupt Service Routine (ISR) is meticulously configured to sample analog voltage and current signals precisely and simultaneously, ensuring high data integrity.
* **Dual-core Processing:**
    * **Core 0:** Dedicated to handling real-time data sampling, ensuring accurate and uninterrupted signal capture.
    * **Core 1:** Responsible for processing the sampled data. This includes:
        * Computing RMS values for both voltage and current.
        * Deriving the power factor based on the phase difference between voltage and current waveforms.
        * Displaying the calculated data locally on the 16x2 LCD display.
        * Uploading the results via Wi-Fi to a remote monitoring platform (e.g., Blynk, ThingSpeak, or a custom server).
* **RMS Calculation:** A dedicated RMS library (`Filters.h`) is utilized to simplify periodic recalculations and ensure efficient CPU usage, optimizing performance.

## ✨ Features

* **Real-time Monitoring:** Provides live monitoring of AC voltage, current, and power factor.
* **Accurate Calculations:** Ensures precise RMS voltage, RMS current, and phase difference calculations.
* **Wi-Fi Data Upload:** Supports remote data uploading for continuous monitoring and analysis.
* **Portable Design:** Implemented as a plug-and-play test rig, making it easy to evaluate various types of electrical loads.

## 💡 Use Case

This project is ideal for:

* **Educational Demonstrations:** A practical tool for teaching concepts related to AC circuits, power factor, and embedded systems.
* **Embedded Systems Learning and Prototyping:** Serves as an excellent foundation for students and hobbyists to learn about sensor integration, real-time data processing, and IoT applications with the ESP32.
