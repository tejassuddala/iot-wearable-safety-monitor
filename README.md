# iot-wearable-safety-monitor
A full-stack IoT wearable using an ESP32 to detect falls (MPU6050) and monitor vitals (MAX30102), sending instant GPS location alerts to a caregiver's phone via the Blynk platform.
# IoT-Based Wearable Safety Monitor

![Platform](https://img.shields.io/badge/Platform-ESP32-blue.svg)
![Framework](https://img.shields.io/badge/Framework-Arduino-00979D.svg)
![Cloud](https://img.shields.io/badge/Cloud-Blynk-2ED9D9.svg)

A wearable IoT device designed for hikers and adventurers. It detects falls, monitors vital signs (heart rate & SpO₂), and automatically sends an emergency alert with the user's live GPS location to a caregiver's smartphone via the Blynk platform.



---

## ## Features

* **Automatic Fall Detection:** Uses an MPU6050 gyroscope and accelerometer to detect a sudden impact followed by a period of inactivity.
* **Live Vitals Monitoring:** An integrated MAX30102 sensor tracks the user's heart rate and SpO₂ levels in real-time.
* **GPS Location Tracking:** A Neo-6M GPS module provides precise latitude and longitude coordinates for the user.
* **Manual SOS Button:** A dedicated push button allows the user to manually trigger an emergency alert.
* **Instant Cloud Alerts:** Connects to Wi-Fi to send all sensor data (GPS, vitals, alert status) to the Blynk IoT platform.
* **Mobile Dashboard:** A caregiver's phone, running the Blynk app, receives an instant push notification and can view the user's location on a map.
* **On-Device Display:** A local SSD1306 OLED screen displays current heart rate, device status, and alerts.

---

## ## Hardware Required

* **Main Controller:** ESP32 Development Board
* **Fall Sensor:** MPU6050 Gyroscope + Accelerometer
* **Health Sensor:** MAX30102 Heart Rate & SpO₂ Sensor
* **Location Sensor:** Neo-6M GPS Module
* **Display:** SSD1306 I2C OLED Display
* **Input:** Push Buttons (x2) for SOS and Mode
* **Power:**
    * 800mAh Li-ion Battery (or similar)
    * TP4056 Type-C Charging Module
    * Rocker Switch (for main power)
* **Prototyping:** Perfboard, connecting wires, 3D-Printed Case

---

## ## Software & Dependencies

* **IDE:** [Arduino IDE](https://www.arduino.cc/en/software)
* **IoT Platform:** [Blynk IoT](https://blynk.io/)
* **Arduino Libraries:**
    * `BlynkSimpleEsp32.h`
    * `Wire.h`
    * `Adafruit_GFX.h`
    * `Adafruit_SSD1306.h`
    * `MPU6050.h`
    * `TinyGPS++.h`

---

## ## How It Works

The system operates in a continuous loop, monitoring all sensors.

1.  **Fall Detection:** The ESP32 constantly reads data from the MPU6050. The algorithm is designed to detect a fall by looking for a sharp, sudden spike in acceleration (the impact) followed by a predefined period of inactivity (user is not moving).
2.  **Alert Trigger:** An alert is triggered in two ways:
    * **Automatically:** When the fall detection algorithm's conditions are met.
    * **Manually:** When the user presses the dedicated SOS button.
3.  **Data Collection:** Once an alert is triggered, the ESP32 immediately fetches the current vitals from the MAX30102 and the latest coordinates from the Neo-6M GPS module.
4.  **Cloud Communication:** The device connects to the pre-configured Wi-Fi network and sends an event to the Blynk cloud. It pushes the GPS data, heart rate, and alert status to virtual pins.
5.  **Caregiver Notification:** The Blynk app, set up on a caregiver's phone, receives the event as an instant push notification. The app's dashboard displays the user's location on a map widget and their live heart rate.

---

## ## Setup & Installation

### ### 1. Hardware

1.  Assemble all components on the perfboard according to the circuit diagram.
2.  Pay special attention to the I2C bus (SDA/SCL lines), which is shared by the MPU6050, MAX30102, and OLED display.
3.  House the perfboard, battery, and modules within the 3D-printed enclosure.

*(You would typically add your own schematic/wiring diagram image here)*

### ### 2. Blynk Cloud Setup

1.  Go to the [Blynk Console](https://blynk.cloud/).
2.  Create a new **Template** (e.g., "Safety Monitor").
3.  Define **Datastreams** (Virtual Pins) to receive data from the ESP32:
    * `V0`: Heart Rate (Integer)
    * `V1`: Latitude (Double)
    * `V2`: Longitude (Double)
    * `V3`: SpO₂ (Integer)
    * `V4`: Fall Alert (Event, String)
4.  Set up **Events** in the "Events" tab. Create a "Fall Alert" event (e.g., type `Warning`) and enable notifications.
5.  Create a **Device** from your new template. This will give you your `BLYNK_AUTH_TOKEN`.
6.  Set up the **Mobile Dashboard** in the Blynk app with a Map widget (set to V1/V2), a Gauge for heart rate (V0), and a notification widget.

### ### 3. Code Configuration

1.  Open the `.ino` sketch file in the Arduino IDE.
2.  Install all the required libraries listed under **Software & Dependencies** using the Library Manager.
3.  In the code, update the following variables with your credentials:

    ```cpp
    // Your Blynk Auth Token (from your device in the Blynk Console)
    char auth[] = "YOUR_BLYNK_AUTH_TOKEN";

    // Your WiFi credentials
    char ssid[] = "YOUR_WIFI_SSID";
    char pass[] = "YOUR_WIFI_PASSWORD";
    ```

4.  Select the correct board (e.g., "ESP32 Dev Module") and port.
5.  Upload the code to your ESP32.

---

## ## Usage

1.  Turn on the device using the main power switch.
2.  The OLED display will show the device status (e.g., "Connecting to WiFi...").
3.  Once connected, place the device on your wrist, ensuring the MAX30102 sensor has good contact with your skin. The OLED will begin to show your heart rate.
4.  The device is now armed. It will automatically detect falls or can be triggered manually with the SOS button.

---

## ## Future Improvements

* Integrate a **SIM module** (like SIM800L or A9G) to send alerts via SMS, removing the dependency on Wi-Fi.
* Design a **custom PCB** to replace the perfboard, making the device much smaller and more reliable.
* Improve the **power management** for longer battery life.
