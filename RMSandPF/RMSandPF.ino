//libraries
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Filters.h>  // Library to use for RMS calculations
#include <iostream>
#include <cmath>
#include "esp_timer.h"  // Include the ESP32 timer library
#include <vector>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define ZMPT101B_Pin 35  // Voltage sensor pin
#define ACS712_Pin 34    // Current Sensor pin
//wifi settings
char ssid[] = "Eng-Student";
char pass[] = "3nG5tuDt";
// RMS Filter parameters
float VtestFrequency = 50;  // Test signal frequency for voltage (Hz)
float CtestFrequency = 50;  // Test signal frequency for current (Hz)
// Separate window lengths for voltage and current
float VwindowLength = 100 / VtestFrequency;  // Window length for voltage averaging
float CwindowLength = 40 / CtestFrequency;   // Window length for current averaging
// Variables for Voltage Measurement
float V_RawValue = 0;
float V_TRMS;                  // Estimated actual voltage in Volts
float V_intercept = 0;         // Calibration intercept (adjust as per your setup)
float V_slope = 0.8696703961;  // Calibration slope for voltage sensor//1.9030282972,4.5247072832
// Variables for Current Measurement
float C_RawValue = 0;
float C_TRMS;                                       // Estimated actual current in Amps
float C_intercept = -0.03;                          // Calibration intercept for current sensor
float C_slope = 0.0130316987266323489569222432945;  // Calibration slope for ACS712 current sensor//0.0115733737,0.0040750213
float P_factor = 0.0;
// Timing variables
unsigned long printPeriod = 1500;  // Measure every 1 second
unsigned long previousMillis = 0;
// Running statistics objects for both current and voltage
RunningStatistics voltageStats;  // For voltage sensor
RunningStatistics currentStats;  // For current sensor
//Sensor data Sampling Parameters
const int numSamples = 100;  // Number of samples to collect
// Arrays to store the samples
volatile int sampleIndex = 0;  // Index to keep track of collected samples
int voltageSamples[numSamples];
int currentSamples[numSamples];
// Vectors to store the peak indices
std::vector<int> voltagePeakIndices;  // Dynamic array to store indices of voltage peaks
std::vector<int> currentPeakIndices;  // Dynamic array to store indices of current peaks
// Semaphore handle
SemaphoreHandle_t xSemaphore;
//for  averages
float ReadytoUploadCloudTD = 0;
float averageDifference = 0;
// ESP32 timer handle for sampling
esp_timer_handle_t timerHandle;
unsigned long lastSampleTime = 0;  // Variable to keep track of the last sampling time
bool samplingReady = false;  // Flag to indicate sampling completion
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void IRAM_ATTR onTimer(void* arg) {
  // Check if we are still collecting samples
  if (sampleIndex < numSamples) {
    // Directly write to the arrays
    voltageSamples[sampleIndex] = analogRead(ZMPT101B_Pin);
    currentSamples[sampleIndex] = analogRead(ACS712_Pin);
    sampleIndex++;
  } else {
    esp_timer_stop(timerHandle);
    // protection for shared resources
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    samplingReady = true;
    Serial.println("End of sampling");
  }

}


void calculateRMS(void* parameter) {
  while (true) {
    if (samplingReady) {
      // Take the semaphore before accessing the shared arrays
      if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
            // Find the max value index for voltage
        int voltagePeakIndex = findMaxValueIndex(voltageSamples, numSamples);
        // Push the index to the voltage peak indices array
        voltagePeakIndices.push_back(voltagePeakIndex);
        // Find the max value index for current
        int currentPeakIndex = findMaxValueIndex(currentSamples, numSamples);
        // Push the index to the current peak indices array
        currentPeakIndices.push_back(currentPeakIndex);
        // Perform the first round of RMS calculations
        ReadVoltage();  // Calculate RMS voltage from sampled data
        ReadCurrent();  // Calculate RMS current from sampled data
        // Release the semaphore so that other parts of the task or sampling can access it
        xSemaphoreGive(xSemaphore);
        lcd.setCursor(0, 0);
        lcd.print("Found max values");
      }
      // Calculate the difference between corresponding elements if both arrays have the same number of elements
      if (voltagePeakIndices.size() == 3 && currentPeakIndices.size() == 3) {
        int sumOfDifferences = 0;
        int numberOfPairs = voltagePeakIndices.size();
        // Loop through each pair of elements
        for (int i = 0; i < numberOfPairs; ++i) {
          int difference = voltagePeakIndices[i] - currentPeakIndices[i];
          sumOfDifferences += abs(difference);  // Add the absolute value of the difference
        }
        // Calculate the average difference
        averageDifference = static_cast<float>(sumOfDifferences) / numberOfPairs;
        ReadytoUploadCloudTD = averageDifference;
        voltagePeakIndices.clear();  // Clear the voltage peak indices array
        currentPeakIndices.clear();  // Clear the current peak indices array
      }
      // Reset the samplingReady flag to prepare for the next sampling cycle
      samplingReady = false;
      lcd.setCursor(0, 1);
      lcd.print("vectors updated");
    }
    if(!samplingReady && (unsigned long)(millis() - previousMillis) >= printPeriod) {
      previousMillis = millis();  // Update time
      // Take the semaphore before accessing the shared arrays
      if (xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
        // Perform the first round of RMS calculations
        ReadVoltage();  // Calculate RMS voltage from sampled data
        ReadCurrent();  // Calculate RMS current from sampled data
        // Release the semaphore so that other parts of the task or sampling can access it
        xSemaphoreGive(xSemaphore);
      }
      // Calculate calibrated RMS values
      V_TRMS = voltageStats.sigma() * V_slope + V_intercept;
      C_TRMS = currentStats.sigma() * C_slope + C_intercept;
      float angle_r = ReadytoUploadCloudTD * 0.0002 * 314.0;
      P_factor = abs(cos(angle_r));
      lcd.setCursor(0, 0);
      lcd.print(V_TRMS);
      lcd.setCursor(6, 0);
      lcd.print("V");
      lcd.setCursor(8, 0);
      lcd.print(C_TRMS);
      lcd.setCursor(15, 0);
      lcd.print("I");
      lcd.setCursor(0, 1);
      lcd.print("PowerFac: ");
      lcd.setCursor(10, 1);
      lcd.print(P_factor);
    }
    // Add a delay to allow the task to run at a reasonable interval
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
int findMaxValueIndex(const int data[], int dataSize) {
  int maxIndex = 0;
  int maxValue = data[0];
  for (int i = 1; i < dataSize; ++i) {
    if (data[i] > maxValue) {
      maxValue = data[i];
      maxIndex = i;
    }
  }
  return maxIndex;  // Return the index of the max value
}
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("V,I RMS and PF");

  // Initialize the semaphore as a binary semaphore
  xSemaphore = xSemaphoreCreateBinary();
  if (xSemaphore == NULL) {
    Serial.println("Failed to create semaphore");
    while (true);  // Halt the program if the semaphore fails to create
  }
  // Configure the ESP32 timer
  const esp_timer_create_args_t timerArgs = {
    .callback = &onTimer,
    .name = "SamplingTimer"
  };
  esp_timer_create(&timerArgs, &timerHandle);
  // Start the first sampling cycle
  lastSampleTime = millis();  // Record the start time
      // Initialize statistics for voltage and current sensors with different window lengths
  voltageStats.setWindowSecs(VwindowLength);
  currentStats.setWindowSecs(CwindowLength);
  // Create tasks pinned to specific cores
  xTaskCreatePinnedToCore(calculateRMS, "RMS Calculation Task", 14000, NULL, 1, NULL, 1);  // Run on Core 1 
}
void loop() {
  // Check if 12 seconds have passed since the last sample collection
  if ((millis() - lastSampleTime) >= 12000) {  // 12 seconds = 12000 ms
    lastSampleTime = millis();                 // Reset the start time
    //WiFi.disconnect(true);
    sampleIndex = 0;                             // Reset the sample index
    samplingReady = false;                       // Reset the sampling flag
    esp_timer_start_periodic(timerHandle, 200);  // Start the timer (200µs interval = 5kHz)
    }
}
void ReadVoltage() {
  // Read voltage sampled array values
  for (int j = 0; j < 5000; j++) {
    for (int i = 0; i < numSamples; ++i) { voltageStats.input(voltageSamples[i]); }
  }
  // Log value to the statistics function
}

void ReadCurrent() {
  for (int j = 0; j < 2000; j++) {
    for (int i = 0; i < numSamples; ++i) { currentStats.input(currentSamples[i]); }
  }
  // Log value to the statistics function
}



