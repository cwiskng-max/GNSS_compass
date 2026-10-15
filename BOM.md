# Bill of Materials (BOM)

Below is the list of components and materials required to build the GNSS Compass.

| Item | Component | Description / Sourcing Reference |
| :--- | :--- | :--- |
| **1** | **GNSS Receiver** | **Unicore UM982 Dual-Antenna GNSS RTK Module**<br>Centimeter-level high-precision tracker, all-constellation (GPS / BDS / GLONASS / Galileo / QZSS).<br>*Source:* Amazon (e.g., *MJ GPS RTK Module MJRTK-UM982*). |
| **2** | **Rotary Display** | **Elecrow 2.1" ESP32 Rotary Display (480×480 HMI IPS)**<br>Round knob capacitive touch display with ESP32-S3 (16MB Flash, 8MB OPI PSRAM).<br>*Source:* Amazon / Elecrow (*CrowPanel 2.1-inch Rotary Display*). |
| **3** | **Antennas** | **2× High-Precision Tri-band Helix Antennas**<br>Multi-frequency active antennas (GPS, GLONASS, BeiDou, Galileo).<br>*Source:* Amazon (e.g., *Slinkdsco Helix RTK GNSS Active Antenna*). |
| **4** | **RF Extension Cables** | **SMA Female Bulkhead to SMA Male RG316 Cables (1m / 3ft, 2-Pack)**<br>*Source:* Amazon (e.g., *BINGFU SMA RG316 Extension Cables*). |
| **5** | **Voltage Regulator** | **LM7805 5V Linear Regulator** (TO-220)<br>*Alternative:* Step-down DC-DC buck converter module, e.g., [Amazon](https://www.amazon.com/dp/B08NV3JCBC) (runs much cooler than a linear regulator). |
| **6** | **Capacitors** | **2× 10µF Tantalum Capacitors** (input and output decoupling for discrete LM7805; not required if using a self-contained buck converter module). |
| **7** | **Power Connector** | **DC Power Plug and Panel-Mount Jack** (standard 2.1mm / 5.5mm). |
| **8** | **Interconnects** | **GX12 3-Pin Aviation Connectors** (male wire mount, female chassis mount). |
| **9** | **Wiring** | **4-Conductor 22 AWG Cable**. |
| **10** | **Receiver Enclosure** | **Hammond 1590A Diecast Aluminum Enclosure** (to house the UM982 receiver module and 5V regulator). |
| **11** | **Display Enclosure** | **3D-Printed Display Housing**<br>*Source:* Printables (*Elecrow CrowPanel 2.1inch-HMI ESP32 Rotary Display 480*480 IPS Round Touch Knob Screen*). |
