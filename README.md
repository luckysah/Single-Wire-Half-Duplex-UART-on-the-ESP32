# Single-Wire Half-Duplex UART on the ESP32

Some devices use single-wire half-duplex UART: sending and receiving share one data wire, taking turns. The ESP32 has separate TX and RX pins, so this project joins them into one wire, sets TX to open-drain so it lets go of the line when idle, and uses a BSS138 level shifter to go from 3.3 V to 5 V. As the example, the ESP32 talks to smart servo motors (DYNAMIXEL XL430) directly, without the maker's controller board.

**For the full build guide, wiring, schematic and results, visit the project page:**
https://luckysah.github.io/projects/esp32-half-duplex-uart/

## What's in this repo

| File | What it does |
|---|---|
| `components/dxl/dxl.c` | The single-wire UART driver: sends commands, removes the echo, reads the motor's reply |
| `components/dxl/include/dxl.h` | List of the driver functions (ping, read, write) |
| `components/mpu6050/` | Driver for the MPU6050 sensor, used by the test |
| `hw_test/main/hw_test_main.c` | Test program: checks the wiring and both motors, prints PASS / FAIL |

## How to use

1. Wire the board as shown on the project page.
2. Install [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/).
3. Download this repo and run:

```bash
cd hw_test
idf.py set-target esp32s3
idf.py build flash monitor
```

The results print in the terminal. Keep the wheels off the ground, because the test spins each motor slowly.
