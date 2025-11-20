# Developer Reference

## Architecture
The library consists of:
- **XDuinoRails_MotorDriver**: Main interface class (PIMPL idiom).
- **XDuinoRails_MotorDriver_Impl**: Implementation logic.
- **pi_controller**: PI control logic.
- **HAL**: Hardware-specific implementations (external dependency).

## Build System
PlatformIO is used for building and testing.
- `platformio.ini`: Defines build environments.
- `library.json`: Library metadata.

## Testing
Native tests are located in `test/test_native`. They mock the HAL and verify core logic.
Run tests with: `python3 -m platformio test -e native_test`

## Contributing
Please follow the guidelines in `CONTRIBUTE.md`.
