# OmniBot STM32 Firmware

This directory contains the firmware code for the STM32F407VG Discovery board used in the OmniBot mecanum wheel robot.

## Overview

The firmware handles the low-level control of the four mecanum wheels, reading encoder values, and communicating with the Raspberry Pi via serial (UART).

## Features

- PWM generation for motor control
- Quadrature encoder reading
- Serial communication protocol with the Raspberry Pi
- PID control for wheel velocity

## Development Environment

- STM32CubeIDE 1.15.0+
- STM32CubeMX for peripheral configuration
- STM32F4 HAL libraries

## Hardware Connections

### Motor Drivers (L298N)

- Motor 1 (Front Left): PA0, PA1 (PWM), PB0, PB1 (Direction)
- Motor 2 (Front Right): PA2, PA3 (PWM), PB2, PB3 (Direction)
- Motor 3 (Rear Left): PA6, PA7 (PWM), PB4, PB5 (Direction)
- Motor 4 (Rear Right): PA8, PA9 (PWM), PB6, PB7 (Direction)

### Encoders

- Encoder 1 (Front Left): PC0, PC1
- Encoder 2 (Front Right): PC2, PC3
- Encoder 3 (Rear Left): PC4, PC5
- Encoder 4 (Rear Right): PC6, PC7

### Serial Communication

- UART2: PA2 (TX), PA3 (RX) - Connected to Raspberry Pi

## Building and Flashing

1. Open the project in STM32CubeIDE
2. Build the project
3. Connect the STM32F407VG Discovery board via ST-LINK
4. Flash the firmware using the IDE or OpenOCD:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/omnibot_firmware.elf verify reset exit"
```

## Communication Protocol

The firmware communicates with the Raspberry Pi using a simple text-based protocol over UART:

### Commands from Raspberry Pi to STM32

- `<FL,FR,RL,RR>\n`: Set wheel velocities (rad/s)
- `<CMD_VEL,x,y,z>\n`: Set robot velocity (m/s, m/s, rad/s)
- `<HEARTBEAT>\n`: Heartbeat message

### Messages from STM32 to Raspberry Pi

- `<ENCODERS,fl,fr,rl,rr>\n`: Encoder values (ticks)
- `<STATUS,message>\n`: Status messages

## License

MIT License