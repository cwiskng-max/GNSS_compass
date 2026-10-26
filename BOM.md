# Bill of Materials (BOM)

## 1. Electronics & Enclosures

| Item | Component | Description / Sourcing Reference |
| :--- | :--- | :--- |
| **1** | **GNSS Receiver** | **Unicore UM982 Dual-Antenna GNSS RTK Module**<br>Centimeter-level high-precision tracker, all-constellation (GPS / BDS / GLONASS / Galileo / QZSS).<br>*Source:* Amazon (e.g., *MJ GPS RTK Module MJRTK-UM982*). |
| **2** | **Rotary Display** | **Elecrow 2.1" ESP32 Rotary Display (480×480 HMI IPS)**<br>Round knob capacitive touch display with ESP32-S3 (16MB Flash, 8MB OPI PSRAM).<br>*Source:* Amazon / Elecrow (*CrowPanel 2.1-inch Rotary Display*). |
| **3** | **Antennas** | **2× High-Precision Tri-band Helix Antennas**<br>Multi-frequency active antennas (GPS, GLONASS, BeiDou, Galileo).<br>*Source:* Amazon (e.g., *Slinkdsco Helix RTK GNSS Active Antenna*). |
| **4** | **RF Extension Cables** | **SMA Female Bulkhead to SMA Male RG316 Cables (1m / 3ft, 2-Pack)**<br>*Source:* Amazon (e.g., *BINGFU SMA RG316 Extension Cables*).<br>*Note:* If following the mast/boom mounting build in [Section 2](#2-mast-mounting--hardware), do not use these cables; use the separate SMA bulkhead couplers and 2-foot right-angle SMA male cables listed in Section 2 instead. |
| **5** | **Voltage Regulator** | **LM7805 5V Linear Regulator** (TO-220)<br>*Alternative:* Step-down DC-DC buck converter module, e.g., [Amazon](https://www.amazon.com/dp/B08NV3JCBC) (runs much cooler than a linear regulator). |
| **6** | **Capacitors** | **2× 10µF 50V Tantalum Capacitors** (input and output decoupling for discrete LM7805; not required if using a self-contained buck converter module). |
| **7** | **Power Connector** | **DC Power Plug and Panel-Mount Jack** (standard 2.1mm / 5.5mm). |
| **8** | **Interconnects** | **GX12 3-Pin Aviation Connectors** (male wire mount, female chassis mount). |
| **9** | **Wiring** | **4-Conductor 22 AWG Cable**. |
| **10** | **Receiver Enclosure** | **Hammond 1590A Diecast Aluminum Enclosure** (to house the UM982 receiver module and LM7805 regulator).<br>*Note:* A **Hammond 1590B** is recommended if the buck converter is used. |
| **11** | **Display Enclosure** | **3D-Printed Display Housing**<br>*Source:* Printables (*Elecrow CrowPanel 2.1inch-HMI ESP32 Rotary Display 480*480 IPS Round Touch Knob Screen*). |

---

## 2. Mast, Mounting & Hardware

| Item | Component | Description / Sourcing Reference |
| :--- | :--- | :--- |
| **12** | **Cross-Boom / Mounting Pole** | **1m Carbon Fiber Tube (25mm OD, 23mm ID, 1000mm length)**<br>Rigid, lightweight boom maintaining the required baseline between the two helix antennas.<br>*Source:* [Amazon](https://www.amazon.com/dp/B08DNRYPRN) (*WHABEST 25mm Carbon Fiber Tube*). |
| **13** | **Antenna Mounts** | **25mm Carbon Fiber Motor Mount Clamp Set**<br>Only 1 kit is required — it is split in half, one clamp used for each end of the 25mm boom.<br>*Source:* [Amazon](https://www.amazon.com/dp/B0F8ZK9HZ3) (*25mm Carbon Fiber Tube Clamps*).<br>*Tip:* See [Assembly & Fitment Notes](#assembly--fitment-notes) below for a clamp fitment tip. |
| **14** | **Cross-Pipe Clamp** | **Stainless Steel Cross-Pipe / Crossover Tube Clamp**<br>Secures the 25mm horizontal boom perpendicular to the vertical mast. Select the clamp size matching the outside diameter of the mast it will be attached to.<br>*Source:* [Amazon](https://www.amazon.com/dp/B0GR49PL1N) (*HARFINGTON 304 Stainless Steel Crossover Pipe Clamp*). |
| **15** | **Bulkhead SMA Connectors** | **2× SMA Female to Female Bulkhead Adapters**<br>Waterproof panel-mount female-to-female barrel couplers for mounting to the antenna brackets.<br>*Source:* [Amazon](https://www.amazon.com/dp/B0D66Y7C6J) (*exgoofit SMA Bulkhead Adapter 2-Pack*). |
| **16** | **Fender Washers** | **4× 1/4" Fender Washers**<br>Stainless steel fender washers for securing the SMA bulkhead connectors through the antenna mounting plates (2 per connector).<br>*Source:* Local hardware store / home center (no link provided). |
| **17** | **RF Jumper Cables** | **2× SMA Right-Angle Male to SMA Male RG316 Coax Cables (2 ft)**<br>Connects the bulkhead SMA adapters on each antenna mount to the receiver enclosure.<br>*Source:* [eBay item 272870603280](https://www.ebay.com/itm/272870603280) (*CablesOnline — select 2 ft length, Qty: 2*). |

### Assembly & Fitment Notes

While mechanical assembly is straightforward and intuitive, keep in mind this practical fitment tip:

* **Antenna Mount Clamp Fit:** Due to slight manufacturing tolerances, the linked 25mm motor mount bracket (Item 13) may not clamp down completely tight onto the 25mm carbon fiber tube out of the box.
  * **Shim method:** Wrap a strip of aluminum foil 3 to 4 times around the pole where the bracket mounts, then tighten the clamp over it.
  * **Sanding method:** Alternatively, sand or file down the mating gap surfaces of the C-shaped clamp so the halves close together more tightly (only about 1 mm of material needs to be removed).
