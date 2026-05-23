# ESPHome Pool Display

An ESPHome component that connects an **ACT1025 BLE LED matrix display** (64×16 pixels) to Home Assistant, showing real-time pool measurements: temperature, pH and ORP.

![Pool Display](https://img.shields.io/badge/ESPHome-2026.5.0-blue) ![Framework](https://img.shields.io/badge/Framework-Arduino-green) ![Display](https://img.shields.io/badge/Display-ACT1025-orange)

## Display layout

```
+------------------+----------------+-----------------+
|     28.5°        |      7.4       |       694       |
|      TEMP        |       PH       |       ORP       |
+------------------+----------------+-----------------+
```

| Column | Value | Colour | Normal range |
|--------|-------|--------|--------------|
| Left   | Temperature (°C) | Cyan   | 10°C – 35°C  |
| Centre | pH value         | Green  | 7.0 – 7.6    |
| Right  | ORP (mV)         | Yellow | 650 – 800 mV |

Values turn **red** when outside the normal range.

## Hardware

| Component | Details |
|-----------|---------|
| Display | BK-Light ACT1025, 64×16 RGB LED matrix |
| Controller | Olimex ESP32-POE |
| Connection | Bluetooth Low Energy (BLE) |
| Framework | Arduino (ESPHome 2026.5.0) |

## File structure

```
/config/esphome/
├── pool-display.yaml
└── components/
    └── pool_display/
        ├── __init__.py
        └── pool_display.h
```

## Installation

### 1. Copy the component files

Copy the `components/` folder and `pool-display.yaml` to your ESPHome configuration directory (`/config/esphome/`).

### 2. Find the MAC address of your display

Use the **nRF Connect** app (Android/iOS) to scan for BLE devices. Look for a device named `LED_BLE_XXXXXX`. The MAC address is shown below the device name.

### 3. Update the configuration

Edit `pool-display.yaml` and update the following:

```yaml
# BLE MAC address of your ACT1025 display
ble_client:
  - mac_address: "0D:D0:1E:CD:2F:D2"  # ← replace with your MAC address

# Home Assistant sensor entities
sensor:
  - platform: homeassistant
    entity_id: sensor.your_temperature_sensor   # ← replace

  - platform: homeassistant
    entity_id: sensor.your_ph_sensor            # ← replace

  - platform: homeassistant
    entity_id: sensor.your_orp_sensor           # ← replace
```

Also update your WiFi credentials and API key in `secrets.yaml`.

### 4. Compile and flash

Open the ESPHome dashboard in Home Assistant and click **Install** on the pool-display device.

## Home Assistant entities

| Entity | Type | Description |
|--------|------|-------------|
| Panel Connected | Binary sensor | BLE connection status |
| Display Rotated | Binary sensor | Current rotation state |
| Panel Power | Switch | Turn display on/off |
| Display Brightness | Number (0–100) | Brightness control |
| Rotate 180° | Button | Rotate display 180° (saved after reboot) |

## Usage

### Adjusting brightness
Use the **Display Brightness** slider in Home Assistant (default: 70).

### Rotating the screen
Press the **Rotate 180°** button. The setting is saved to flash memory and persists after reboot.

### Turning off the display
Use the **Panel Power** switch. The BLE connection remains active.

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Display shows boot screen | Wait 10–20 seconds for BLE reconnect |
| Values not updating | Check ESPHome is added in HA integrations |
| Panel Connected = OFF | Restart ESP32, check display is powered on |
| Screen is upside down | Press Rotate 180° button |

## Protocol

The component communicates with the ACT1025 using the BK-Light BLE protocol:
- Service UUID: `0x00FA`
- Write characteristic: `0xFA02`
- Notify characteristic: `0xFA03`
- Images are sent as compressed PNG wrapped in the BK-Light frame format

## License

MIT License — feel free to use and modify.