# Robot Driver Firmware

Embedded controller firmware for a pulse/direction motor-control board with Modbus TCP communication.

## What This Repository Contains

- `Core/` - STM32 application source, headers, and startup code.
- `Drivers/` - STM32 HAL/CMSIS driver files.
- `Middlewares/` - middleware components.
- `LWIP/` - networking stack configuration/files.
- `EWARM/` - IAR Embedded Workbench project files.
- `Docs/` - project documentation.
- `RobotDriver.ioc` - STM32CubeMX configuration.

## Development

Use STM32CubeMX for configuration review and the embedded project files/toolchain for building and flashing. Keep custom application logic inside protected user-code sections when regenerating generated files.

## Hardware Notes

Before running motors, verify pulse/direction wiring, drive enable logic, limit switches, network settings, and emergency-stop behavior.

## Safety

Motion-control firmware can move hardware unexpectedly. Test with drives disabled or motors unloaded first, then validate command limits and failsafe handling before connecting to a real mechanism.