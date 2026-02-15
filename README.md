# ESP32-C3 Blood Pressure Waveform Monitor

This repository contains the firmware and design details for a portable, battery-powered medical monitoring device. It is designed to capture, process, and visualize real-time blood pressure cuff waveforms.

---

## 🛠 Hardware Architecture

* **Microcontroller**: **ESP32-C3**, chosen for its integrated Wi-Fi/Bluetooth capabilities and efficient power consumption for battery-operated use.
* **Pressure Sensor**: **HX710B** 24-bit ADC, specifically designed for high-precision atmospheric pressure sensing.
* **Display**: **ST7789 LCD** (128x160 resolution) providing a clear graphical interface for waveform visualization.
* **Motor Control**: An external blowing motor driven by a **motor shield** to manage cuff inflation.
* **User Input**: Three physical tactile buttons for manual motor and inflation control.

---

## 🚀 Key Features

### 1. High-Precision Waveform Capture

The system utilizes a bit-banging driver for the **HX710B** to retrieve raw pressure data. To extract the arterial pulse from the total cuff pressure, a digital **High-Pass Filter (HPF)** is applied with an alpha value of **0.90**. This removes the "DC" baseline (static pressure) and highlights the "AC" oscillometric signal.

### 2. Manual Inflation Control

The device features a dedicated motor control task that manages cuff inflation via PWM (30kHz frequency).

* **Button 1 (Start)**: Activates the motor at maximum duty cycle (255) to begin blowing.
* **Button 2 (Increase/Adjust)**: In the current firmware implementation, this button allows for speed reduction or fine-tuning of the motor output.
* **Button 3 (Stop)**: Immediately cuts power to the motor for safety and measurement stability.

### 3. Real-Time Graphical UI

Powered by **LVGL 8.3.11**, the interface features a dynamic line chart that plots up to **700 data points**. The chart is configured with a wide range (-100,000 to 100,000 units) to accommodate the filtered signal swings.

---

## 💻 Firmware Overview

The code is built on the **ESP-IDF** framework using **FreeRTOS** to handle concurrent operations:

* **`hx710b_sensor_task`**: High-priority task for data sampling and digital filtering.
* **`motor_control_task`**: Dedicated task for monitoring button states and updating PWM duty cycles.
* **`lvgl_port_task`**: Manages the display refresh and UI rendering logic.
* **`chart_update_task`**: Safely transfers filtered sensor data to the UI using mutexes to ensure thread safety.

---

## 🔌 Pin Configuration

| Component | ESP32-C3 Pin | Function |
| --- | --- | --- |
| **HX710B DOUT** | GPIO 1 | Data Output |
| **HX710B SCK** | GPIO 21 | Serial Clock |
| **Motor Enable** | GPIO 2 | PWM Speed Control |
| **Motor IN1** | GPIO 10 | Motor Direction |
| **Button 1** | GPIO 8 | Start Motor |
| **Button 2** | GPIO 0 | Decrease Speed |
| **Button 3** | GPIO 20 | Stop Motor |


