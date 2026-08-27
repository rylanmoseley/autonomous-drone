# External Interaction

This sub-project manages all communication going into and out of the Flight Controller, excluding raw sensor reads.

## Communication Channels
* **Vision Processor (CM4)**: 
  * *Protocol*: UART, framed with COBS (Consistent Overhead Byte Stuffing).
  * *Purpose*: Receives 3D displacement vectors and the identified shape sequence from the CM4. The ESP32 handles all networking; the CM4 is strictly an offline compute node.
* **Telemetry (Ground Station)**:
  * *Protocol*: Wi-Fi / ESP-NOW.
  * *Purpose*: Broadcasts live flight data (attitude, battery, mode) to the operator.
* **ESCs (Electronic Speed Controllers)**:
  * *Protocol*: CAN bus or UART (FOC).
  * *Purpose*: Sends high-speed throttle commands to the motors and receives RPM/current telemetry back.
* **Side-Path E-Stop**:
  * *Protocol*: SPI (Sub-GHz radio like RFM69HCW).
  * *Purpose*: Listens for a hardware-level disarm command independent of the 2.4GHz spectrum.
