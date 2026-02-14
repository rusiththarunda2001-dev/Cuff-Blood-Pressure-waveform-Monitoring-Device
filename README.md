# Cuff-Blood-Pressure-waveform-Monitoring-Device
This ESP32-C3 monitor captures blood pressure waveforms using an HX710B sensor. It uses a High-Pass Filter ($\alpha=0.90$) to isolate arterial pulses from cuff pressure. Data is displayed via LVGL on an ST7789 LCD. Features include multi-threaded FreeRTOS tasks and PWM motor control for cuff inflation.
