# Sensor Reading (HAL)

This sub-project is the physical gateway to the drone's environment. It implements the Hardware Abstraction Layer (HAL) interfaces.

## Hardware Drivers
* **IMU (Inertial Measurement Unit)**: Handles I2C or SPI transactions to read high-speed raw accelerometer and gyroscope data.
* **ToF/LiDAR**: Handles communication with the downward-facing distance sensor for altitude holding.
* **Power Monitoring**: Reads the ADC (Analog-to-Digital Converter) to monitor battery voltage and current.

## Architecture
These files are heavily tied to the ESP32-S3 and use ESP-IDF specific libraries (`driver/i2c.h`, `driver/spi_master.h`). They format the raw data and pass it to the agnostic `math-physics` filters.
