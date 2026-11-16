# Robot Emo

Custom ESP32-S3-DevKitC-1 board profile for the N16R8 module, based on the
Bread Compact Wi-Fi board implementation.

## Hardware

- INMP441 microphone: WS GPIO4, SCK GPIO5, DIN GPIO6
- MAX98357A speaker: DOUT GPIO7, BCLK GPIO15, LRCK GPIO16
- SSD1306 128x64 OLED: SDA GPIO41, SCL GPIO42, I2C address `0x3C`
- BOOT button: GPIO0
- Optional on-board LED: GPIO48

The board profile does not initialize touch or volume buttons, a lamp, motors,
or a heart-rate sensor.

## Build and flash on Windows

From an ESP-IDF PowerShell with ESP-IDF v6.0.3 exported:

```powershell
Set-Location C:\dev\xiaozhi-esp32
idf.py --version
python scripts\build.py robot-emo --name robot-emo --language vi-VN
idf.py -p COMx flash monitor
```

Replace `COMx` with the device's serial port. The ESP32-S3 defaults already
select 16 MB flash and octal PSRAM; this board only overrides the OLED
resolution. Pass `--language vi-VN` when building to select Vietnamese.

## Controls

During startup, click BOOT to enter Wi-Fi provisioning. After startup, click
BOOT to toggle chat/listening, matching the Bread Compact Wi-Fi behavior.
