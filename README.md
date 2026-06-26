# Magnetic D-Pad BLE Keyboard (ESP32-S3)

This project transforms an ESP32-S3 and two 3D Hall-effect sensors (MLX90393 and TMAG5273) into a touchless, fully magnetic Bluetooth Low Energy (BLE) keyboard. 

Designed to operate through thick materials (like a 1cm plexiglass panel), the system tracks the physical movement of a magnetic slider across X and Y axes to emulate a 4-way D-pad (Up/Down/Left/Right) for two independent inputs.

## Features
* **Touchless 4-Way Input:** Uses 3D magnetic tracking to determine slider position.
* **Dual Independent Sliders:** Supports an Adafruit MLX90393 (Right) and a SparkFun TMAG5273 (Left) on the same I2C bus.
* **BLE Keyboard:** Acts as a standard Bluetooth HID Keyboard.
* **Over-The-Air (OTA) Updates:** Flash new code wirelessly.
* **Live Telnet Debugging:** Connect via Wi-Fi to monitor raw magnetic deviations and tune thresholds without USB cables.
* **Auto-Calibration:** Locks onto the ambient magnetic baseline at startup to eliminate environmental drift.

---

## Hardware Requirements
* **Microcontroller:** ESP32-S3 (e.g., Waveshare ESP32-S3 Zero)
* **Sensor 1:** Adafruit MLX90393 (I2C Address: `0x0C`)
* **Sensor 2:** SparkFun TMAG5273 A2-Version (I2C Address: `0x35`)
* **Magnets:** Small neodymium disk magnets attached to physical slider knobs.
* **Pull-up Resistors:** Not required if using standard Adafruit/SparkFun breakout boards (they are built-in).

### Wiring (I2C Bus)
Both sensors run in parallel on the exact same pins.
* **3.3V** -> VCC (Both Sensors)
* **GND** -> GND (Both Sensors)
* **GP8** -> SDA (Both Sensors)
* **GP9** -> SCL (Both Sensors)

---

## The Mechanics & Logic

Working with magnetic fields through thick material requires precise math to filter out noise and detect true physical movement. Here is how the code's engine works:

### 1. The Baseline Calibration (Finding "Zero")
When the ESP32 boots up, it expects the magnetic sliders to be perfectly centered. It takes `20` rapid readings from both sensors and averages them. This creates the **Baseline** (`baselineX` and `baselineY`). 
* *Logic:* By establishing a baseline, the code ignores the Earth's natural magnetic field and any static metal nearby. It only cares about *changes* from this zero state.

### 2. The Delta (DX / DY)
During the main loop, the code continuously reads the current magnetic field and subtracts the baseline:
`DX = Current_X - Baseline_X`
* If the magnet slides **UP**, the field strength increases, creating a highly **Positive DX**.
* If the magnet slides **DOWN**, the field flips or weakens, creating a highly **Negative DX**.
* The same logic applies to the **Y-Axis** (Left/Right).

### 3. State Machines & Thresholds
To prevent "bouncing" (multiple rapid key presses from one slide), the code uses a State Machine with three distinct zones:
1. **Trigger Thresholds:** The `DX` or `DY` must cross a high value (e.g., `450.0` for MLX or `2.0mT` for TMAG) to register a keypress. Once triggered, the state locks.
2. **The Cooldown:** A `150ms` timer prevents overlapping reads during fast physical movements.
3. **The Deadzone:** To trigger the *same* key again, the user cannot just hover near the threshold. The magnet must physically return to the center (the `DEADZONE` threshold) to reset the state machine back to `0`.

---

## Calibration Guide

Because magnetic strength varies based on your specific magnets and the thickness of your plexiglass, you **must** tune the thresholds for your build. 

1. Ensure both sliders are centered and power on the device.
2. Connect to the ESP32 via Serial Monitor (115200 baud) or via Telnet (Port 23) if OTA is enabled.
3. Move the slider all the way UP. Note the `Raw DX` value printed in the console.
4. Move the slider all the way DOWN. Note the negative `Raw DX` value.
5. Set your `_POS_THRES` and `_NEG_THRES` in the code to about **50% to 60%** of those peak values.
6. Set the `DEADZONE` to about **15% to 20%** of the peak value to ensure it resets cleanly when the user lets go.

*Note: The MLX90393 outputs in raw bits (e.g., `450.0`), while the TMAG5273 outputs in milliteslas (e.g., `2.0 mT`). Their threshold numbers will look vastly different!*

---

## Dependencies
You will need to install the following libraries via the Arduino Library Manager:
* `Adafruit MLX90393`
* `SparkFun TMAG5273 Arduino Library`
* `ESP32 BLE Arduino` (Built into the ESP32 Core)

## License
MIT License. Feel free to use, modify, and distribute this code for your own custom keyboard projects!
