# ESP-Fly Custom Implementation

This project utilizes the unmodified hardware design from the [Seeed Projects ESP-Fly](https://github.com/Seeed-Projects/Co-Create_ESP-FLY).

---

## 🛠️ Hardware & Assembly

* **Base Design:** [Seeed Projects ESP-Fly](https://github.com/Seeed-Projects/Co-Create_ESP-FLY)
* **Assembly Instructions:** Watch the [ESP-FLY Tutorial & Build Guide](https://youtu.be/3Y_drsQtMs4) for detailed step-by-step assembly instructions.

---

## 📐 Axis & Orientation Reference

To ensure accurate IMU calibration and flight control dynamics, the components need to be aligned according to the reference frame below:

| Axis / Direction | Board / Component Feature |
| :--- | :--- |
| **$-Y$ Direction** | USB-C port on the ESP32 & battery connector on the IMU/Motor Driver module |
| **$-Z$ Direction** | Outward face of the IMU / Motor Driver module |

---

## 💡 Firmware & Library Recommendations

The stock ESP-Fly library bundles an internal MPU6050 library directly into its codebase, which makes debugging difficult. It is strongly recommended to bypass the bundled driver and use standard, well-maintained libraries for your development environment:

* **ESP-IDF Framework:** Use the official [Espressif MPU6050 Component (v1.2.1)](https://components.espressif.com/components/espressif/mpu6050/versions/1.2.1/readme).
* **Arduino IDE:** Use the official [Adafruit MPU6050 Library](https://github.com/adafruit/Adafruit_MPU6050).
