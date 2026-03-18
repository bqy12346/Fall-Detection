# Fall-Detection

## Overview
This project is a mini embedded systems project based on the ARM-based NUCLEO-L476RG, an embedded system development board featuring a high-performance low-power microcontroller of type STM32L476RG. It implements a fall detection system designed to support elderly people in their daily lives.

## Features
- **Dual-Algorithm Fall Detection**: Utilizes an Acceleration Vector Magnitude algorithm to detect instantaneous increases in total gravitational force, and a Tilt Angle Calculation using trigonometry to determine the board's vertical inclination.
- **Adjustable Thresholds**: Users can modify threshold values to dtermine the optimal angle for all detection.
- **Performance Profiling**: Measures computational efficiency using the DWT cycle counter. The vecto magnitude algorithm takes approximately 134 clock cycles, while the angle calculation takes about 460 clock cycles.
- **Low-Power Operation**: Implements an energy-efficient duty cycle by alternating between an active data-acquisition state and a 5-second Sleep Mode.
- **Real-Time Energy Evaluation**: Actively calculates and logs energy consumption, demonstrating that sleep mode consumes significantly less energy and dominates the period to save power.

## Hardware
- MCU: NUCLEO-L476RG (STM32L476RG)
- Sensors: Inertial Measurement Unit (IMU) located on the PBL Shield, communicating via SPI.
- Actuators: Onboard LED (LED2) for visual fall alarms, and User Button (B1) for hardware interrupts and wakeups.
- Power supply: USB (5V) / 3.3V internal NUCLEO regulator

## Software
- Language: C
- IDE: STM32CubeIDE
- Framework / HAL: STM32 Hardware Abstraction Layer (HAL)

## Project Structure
- `Core/Src/main.c`: Core application logic, including SPI sensor reading, fall detection algorithms, performance profiling, and power management.
- `Core/Src/stm32l4xx_it.c`: Interrupt service routines (e.g., EXTI lines for user buttons).
- `Core/Src/stm32l4xx_hal_msp.c`: MCU Support Package for hardware-level peripheral initializations (SPI, UART, GPIO).
- `Core/Src/system_stm32l4xx.c`: System clock configuration (setting the system frequency to 80 MHz ).
- `docs/`: Contains project documentation, such as the energy evaluation and algorithms report.

## How to Build
1. Clone this repository to your local machine.
2. Open the project directory using STM32CubeIDE.
3. Ensure the project properties are configured for the STM32L476RG microcontroller.
4. Click on the Build (hammer) icon in the IDE toolbar to compile the source code and generate the binaries.

## How to Run
1. Attach the PBL Shield to your NUCLEO-L476RG board.
2. Connect the NUCLEO board to your PC via a mini-USB cable.
3. Open a serial terminal program and connect to the board's COM port with the following settings: `115200` baud rate, `8` data bits, `No` parity, `1` stop bit.

## Demo
Here is a short video demonstrating the fall detection system in action:


https://github.com/user-attachments/assets/7257fd28-9d2a-40cf-8805-17f7a69ad629




## Author
Boxuan Yu
