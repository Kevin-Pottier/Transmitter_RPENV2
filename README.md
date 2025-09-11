
# RPEN V2 – Transmitter ✨
## STM32L4 Discovery Kit (STM32L476G-DISCO)


The project leverages the **STM32L4 Discovery Kit (STM32L476-DISCO)**, a feature-rich development board built around the STM32L476VGT6 microcontroller. This kit is ideal for low-power, sensor-rich applications and includes:

- **STM32L476VGT6 MCU:**
  - ARM Cortex-M4, 80 MHz, ultra-low-power
  - 1 Mbyte Flash, 128 Kbytes RAM (LQFP100)
- **Display:**
  - LCD with 24 segments, 4 commons (DIP 28)
- **Sensors:**
  - 9-axis motion: MEMS accelerometer, gyroscope, magnetometer
  - MEMS digital microphone
- **Memory:**
  - 128-Mbit Quad-SPI Flash
- **MCU Current Ammeter:**
  - 4 ranges, auto calibration

This hardware platform enables efficient motion capture, user interaction, and feedback, all with minimal power consumption and a wide range of prototyping options.

## STM32L476 MCU Power Modes & Current Consumption

The STM32L476 MCU offers several power modes to optimize energy usage. Below is a summary of typical current consumption values (from STMicroelectronics datasheet, VDD = 3.0V, 25°C):

| Power Mode         | Description                                 | Typical Current |
|--------------------|---------------------------------------------|-----------------|
| Run (80 MHz)       | Full-speed operation                        | ~3.8 mA         |
| Sleep              | CPU stopped, peripherals running            | ~844.9 µA       |
| Low-power Run      | 32 kHz-2 MHz, low voltage, peripherals on   | ~425.9 µA       |
| Low-power Sleep    | CPU clock stopped, low-power peripherals on | ~93.3 µA        |
| Stop 2             | Lowest stop mode, minimal context retained  | ~1.3 µA         |
| Standby            | RAM off, only RTC/backup domain on          | ~150 nA         |
| Shutdown           | Lowest power, almost everything off         | ~60 nA          |

> **Note:** Actual current depends on configuration, peripherals, and temperature. Refer to the [STM32L476 datasheet](https://www.st.com/resource/en/datasheet/stm32l476vg.pdf) for detailed values and conditions.

The **Transmitter** is the pen module of the RPEN V2 project.  
It captures motion, user input, and configuration data, then transmits them wirelessly to the **Receiver**, which renders the drawing on a screen.

## Features

- **Motion Sensing:** High-performance IMU for smooth and precise tracking.  
- **User Controls:** Buttons to change mode, size, or color.  
- **Feedback System:** RGB LEDs (and future haptics) to provide real-time feedback.  
- **Wireless Communication:** Sends data frames (motion, size, color, mode) to the Receiver with low latency.  

## Planned Improvements

- Better noise filtering and calibration.  
- Stronger microcontroller for improved responsiveness.  
- More ergonomic design for user comfort.  

## Roadmap

- [ ] Finalize hardware component selection.  
- [ ] Develop data frame protocol.  
- [ ] Implement advanced filtering algorithms.  
- [ ] Synchronize with the Receiver module.  

## Developer

- **Kévin Pottier**  
  - GitHub: [Kevin-Pottier](https://github.com/Kevin-Pottier)  
  - Email: kevin.pottier@reseau.eseo.fr  

## License

This project is licensed under the MIT License – see the [LICENSE](./LICENSE) file for details.
