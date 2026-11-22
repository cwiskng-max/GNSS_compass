# Reference Target Lists

Ready-to-paste `target add` command lists for pre-loading target presets over USB-C or BLE UART before heading into the field.

## How to Use

1. Connect to the compass serial CLI at `115200` baud (or over BLE UART).
2. Copy one or more of the code blocks below and paste them into your terminal.
   * **Duplicates are safe:** `target add` updates an existing 6-character grid in place rather than creating duplicates (case-insensitive).
   * **Labels:** Up to 16 characters are stored per label; spaces are allowed.
   * **Capacity:** Default capacity is `100` presets (configurable from `10` to `500` via `target max <N>`).
3. Run `target list` to verify your loaded presets. See [SERIAL.md](SERIAL.md#pre-trip-batch-preloading) for shell batch-upload instructions.

---

## California & Southwest Microwave Sites / Beacons

Sorted north to south within each list. `DM04ms` (Frazier) is included in **Beacons** as well as both **Frazier and North** and **Frazier and South** so each regional block is self-contained (duplicates are merged automatically).

### 10 GHz Beacons

```text
target add CM88wj Vaca .325
target add CM97bl Allison 10369.0
target add CM97ae Leeson 10367.997
target add DM04ms Frazier .310
target add DM14hf Heaps .330
target add DM03tr Palos Verdes.300
target add DM33rn White Tank .370
target add DM12jv San Diego .360
```

### Frazier and North

```text
target add CM99gs Concow
target add CM89pk St John
target add CM88ws Bald
target add CM97av Diablo
target add CM97bv Diablo Top
target add CM87wj Stanford W6YX
target add CM87vj Stanford Dish
target add CM97jk Diablo Grande
target add CM97ei Hamilton
target add CM97kh I-5 SB Vista
target add CM97bd Umunhum
target add CM97bc Loma Prieta
target add DM25gw Potosi
target add DM04ms Frazier .310
```

### Frazier and South

```text
target add DM04ms Frazier .310
target add DM13et Pleasant Pk
target add DM13co Signal Pk
target add DM13jc Double Pk
target add DM12mq San Miguel
target add DM12no Otay
target add DM52bp Heliograph
target add DM42ok Lemmon
```

### Site Notes

* **`CM97av` (Diablo):** Commonly used — West Diablo, overflow parking lot or picnic tables by restrooms.
* **`CM97bv` (Diablo Top):** Less used — East Diablo summit/museum, also Devil's Elbow.

---

## Adding More Reference Lists

Additional regional or contest target lists can be added as new top-level sections in this file. Keep each `target add <GRID> [LABEL]` line free of comments and keep labels to 16 characters or fewer so they are not truncated on-device.
