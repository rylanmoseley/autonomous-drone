# Unresolved System Requirements (For Project Sponsor)

As we finalize the hardware and software architecture for the autonomous drone, the following requirements need explicit definition from the project sponsor.

### 1. Safety Requirements
* **Question**: Are physical propeller guards mandatory for the final demonstration?
* **Context**: Prop guards add weight and reduce aerodynamic efficiency, severely impacting our tight 250g budget. If they are not strictly required for safety by the venue/sponsor, omitting them gives us much more payload capacity for the Vision Processor.

### 2. Budget Constraints
* **Question**: What is the hard maximum Bill of Materials (BOM) cost?
* **Context**: We are currently estimating a roughly $500 budget. An RPi CM4, global shutter camera, ToF sensor, and a CAN/UART 4-in-1 ESC can quickly eat into this. We need to confirm if this $500 figure is a hard cap or if it is negotiable based on performance needs.
