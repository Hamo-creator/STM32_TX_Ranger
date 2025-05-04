# STM32 Ranger TX

A custom RC transmitter project based on STM32F4 microcontroller, implementing the CRSF (Crossfire) protocol for RC control.

## Features

- **Control Channels**:
  - 4 main control channels (Aileron, Elevator, Throttle, Rudder)
  - 8 AUX channels for additional functions
  - Configurable stick offsets and reversals
  - Simple Moving Average (SMA) filtering for smooth control

- **CRSF Protocol**:
  - Full CRSF (Crossfire) protocol implementation
  - Configurable packet rates (50Hz to 500Hz)
  - Adjustable power output (10mW to 250mW)
  - Dynamic power control
  - Model matching capability
  - WiFi connectivity for updates

- **Safety Features**:
  - Battery voltage monitoring
    - Warning at 7.4V (2S LiPo)
    - Critical at 7.0V
    - USB power detection at 5.2V
  - Stick movement detection with timeout warning
  - Configurable failsafe settings

- **User Interface**:
  - Audio feedback through passive buzzer
  - Startup melody
  - Stick movement warning melody
  - Multiple power/rate settings accessible through stick combinations

## Hardware Requirements

- STM32F4xx microcontroller
- 4 analog inputs for control sticks
- Multiple digital inputs for switches
- Passive buzzer for audio feedback
- Battery voltage monitoring circuit
- UART interface for CRSF communication

## Configuration

### Stick Commands
The transmitter supports special stick combinations for configuration:
- **Up Left**: Rate/Power setting 1 (250Hz / 100mW / Dynamic)
- **Up Right**: Rate/Power setting 2 (50Hz / 100mW)
- **Down Left**: Start TX bind
- **Down Right**: Start TX module WiFi

### Power Settings
Two default configurations:
1. Setting 1:
   - Packet Rate: 250Hz
   - Power: 100mW
   - Dynamic Power: On

2. Setting 2:
   - Packet Rate: 50Hz
   - Power: 100mW
   - Dynamic Power: Off

## Building and Flashing

1. Clone the repository
2. Open the project in STM32CubeIDE
3. Build the project
4. Flash to your STM32F4 device

## Dependencies

- STM32 HAL Library
- CRSF Protocol Library
- Standard C Library

## License

This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgments

- Based on the CRSF protocol implementation
- Inspired by the work of kkbin505: https://github.com/kkbin505/Arduino-Transmitter-for-ELRS
- Uses STM32 HAL libraries for hardware abstraction 
