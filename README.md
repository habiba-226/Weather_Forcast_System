# 🌦️ Smart Weather Monitoring System

A real-time weather dashboard built using **ESP32**, **MQTT**, **Blynk**, and a custom **JavaScript Web App**. The system gathers sensor data, pushes it to the cloud, and displays it live on both a web interface and mobile app. It even predicts weather trends based on atmospheric pressure changes!

---

## 📦 Features

- 📡 Real-time sensor readings:
  - Temperature (DHT11 & BMP180)
  - Humidity
  - Rain % (Analog sensor)
  - Light (LDR)
  - Pressure & Sea Level Pressure (BMP180)
  - Altitude
- 🌍 Live dashboards:
  - Mobile dashboard via **Blynk**
  - Web dashboard via **MQTT + JavaScript**
- ⏰ Custom alert thresholds for:
  - Temperature
  - Rain %
- 🔮 Weather prediction based on pressure trend:
  - 📈 Rising → Likely clear weather
  - 📉 Falling → Possible rain or storm
  - ➖ Stable → No major change expected
- 📊 Real-time charts for historical data
- 📩 Email or SMS alert integration via Blynk

---

## 🛠️ Tech Stack

- **Hardware:**
  - ESP32
  - DHT11
  - BMP180
  - LDR
  - Rain Sensor
- **Cloud:**
  - [HiveMQ MQTT Broker](https://www.hivemq.com/mqtt-cloud-broker/)
  - [Blynk IoT Platform](https://blynk.io)
- **Frontend:**
  - HTML, CSS, JavaScript
  - MQTT.js for real-time MQTT client

---

## 📁 Files in This Repo

- `FINAL.ino` – Contains the complete ESP32 firmware code for sensor reading, MQTT publishing, and Blynk integration.
- `final.html` – The full code for the real-time web dashboard (subscribe to MQTT topics and visualize data).

---

## 🚀 How It Works

1. ESP32 collects weather data from the sensors.
2. Data is published to MQTT topics (HiveMQ).
3. The web dashboard (`final.html`) subscribes to these topics using `mqtt.min.js`.
4. Data is updated live in the browser.
5. Pressure readings are tracked to predict whether the weather is **rising**, **falling**, or **stable**.
6. Data is also visualized in Blynk for mobile monitoring and alerting.

---

## 🔔 Alert System

Users can set thresholds for temperature and rain percentage directly from the dashboard. If these are exceeded, alerts appear live, and Blynk can send email or mobile notifications.

---
