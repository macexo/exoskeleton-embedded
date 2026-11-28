# Host firmware tests

These scripts run on your laptop or development computer to validate firmware.
They are separate from scripts intended to run on the Raspberry Pi.

From the repository root, with Python and GCC on PATH, run:

```sh
python src/stm32/joint-controller/Scripts/host_tests/test_motor_telemetry.py
```

The test compiles the motor-service code from `Core/Src/main.c` with mocked
HAL and CAN functions. It uses a temporary directory for generated files and
does not connect to hardware or command motors.
