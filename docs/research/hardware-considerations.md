# Hardware Considerations & Notes

## Compute & Weight Budget
* **ESP32-S3**: Excellent choice for the FCU. Dual-core allows pinning the critical flight control loop to Core 0, while Wi-Fi/ESP-NOW and UART comms run on Core 1.
* **Vision Processor**: The RPi CM4 is powerful for OpenCV/SLAM but risks exceeding the <250g weight limit due to the carrier board, heatsink, and global shutter camera.
    * *Alternative*: A Raspberry Pi Zero 2 W is much lighter, though SLAM capabilities would be limited. If using the CM4, a custom, ultra-light carrier board is mandatory.
    * *Networking Cost*: A wireless-capable CM4 is significantly more expensive. To save budget, the CM4 will act as an offline compute node. The ESP32-S3 provides "free" networking for the system.

## Communications
* **CM4 <-> ESP32 (UART)**: Sending raw UART strings for trajectory/velocity setpoints is fragile.
    * *Recommendation*: Implement **COBS (Consistent Overhead Byte Stuffing)** framing combined with a robust serialization format like **Nanopb (Protocol Buffers)** or a packed C-struct with CRC checks.

## ESCs & Motors
* **4-in-1 ESC**: Highly recommended for a 250g drone. Designing custom ESCs is time-prohibitive for a project where the core goal is autonomous navigation.
