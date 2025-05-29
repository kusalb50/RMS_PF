![20250108_105850 (1)](https://github.com/user-attachments/assets/0f70d1c2-0c2c-458c-a4a9-42cfed6c68f7)

Power Factor Monitoring System using ESP32
This Arduino-based project analyzes analog input signals from a voltage sensor (ZMPT101B) and a current sensor (ACS712-20A) to calculate the RMS voltage, RMS current, and power factor (PF) of an AC load in real time.
Hardware Components
  ESP32 NodeMCU (Wroom-32) – Dual-core microcontroller with WiFi capabilities.
  ZMPT101B Voltage Sensor – Accurate AC voltage measurement with isolation.
  ACS712-20A Current Sensor – Measures AC current up to 20A.
  16X2 LCD display – To show live values on the device.
Libraries Used
This project relies on several key libraries to handle multitasking, networking, sensor data processing, and display functions:
  freertos/FreeRTOS.h,/task.h/semphr.h -Enable multi-core task management on the ESP32. Used to separate data acquisition and processing/display/upload tasks using FreeRTOS functionality.
  WiFi.h, WiFiClient.h -Establish WiFi connectivity to upload power factor and RMS data to a server or cloud service.
  HTTPClient.h - Used to make HTTP POST/GET requests for uploading data.
  Filters.h - Used for real-time RMS calculation of analog voltage and current signals. Simplifies signal filtering and accurate energy measurement.
  cmath - Provides mathematical functions such as sqrt() and trigonometric calculations used in power factor computations.
  esp_timer.h - Utilizes ESP32’s high-resolution timer functions for precise sampling and timing in the ISR.
  Wire.h - Enables I2C communication between the ESP32 and LCD display module.
  LiquidCrystal_I2C.h - Controls I2C LCD.  
Custom-built test rig – Includes power terminals, voltage regulator, display, and mounting components.

System Overview
Sampling: An Interrupt Service Routine (ISR) is used to sample analog voltage and current signals precisely and simultaneously.
Dual-core Processing:
  Core 0 handles real-time data sampling to ensure accurate signal capture.
  Core 1 processes the sampled data to:
    Compute RMS values for both voltage and current
    Derive the power factor based on phase difference
    Display the data locally
    Upload the results via WiFi to a remote monitoring platform (e.g., Blynk, ThingSpeak, or custom server).
    RMS Calculation: A dedicated RMS library is used to simplify periodic recalculations and ensure efficient CPU usage.

Features
  Real-time monitoring of AC voltage, current, and power factor
  Accurate RMS and phase difference calculation
  WiFi-based remote data uploading
  Portable plug-and-play test rig for evaluating various types of electrical loads
Use Case:
  Educational demonstrations
  Embedded systems learning and prototyping
