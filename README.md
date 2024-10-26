# ps2_horifs_adapter

A DIY adapter that can receive input from a usb Stick/HOTAS output as an PlayStation 2 Hori FlighStick (HP2-13).

## Supported devices
* Saitek X52
* Logitech Force 3D Pro
* Logitech Extreme 3D Pro
* Microsoft SideWinder FFB2

## Mapping
| FlighStick             | X52          | Force 3D Pro / Extrene 3d Pro | SideWinder FFB2 |
|------------------------|--------------|-------------------------------|-----------------|
| Main Stick             | Stick        | Stick                         | Stick           |
| Throttle               | Throttle     | Throttle                      | Throttle        |
| Rudder                 | Stick Twist  | Stick Twist                   | Stick Twist     |
| Sub Stick (analog hat) | Mouse        | -                             | -               |
| Hat (digital)          | Hat          | Hat                           | Hat             |
| A/Triangle (analog)    | C + Slider   | 3                             | 3               |
| B/Square (analog)      | B + Slider   | 4                             | 4               |
| Trigger                | Trigger      | 1 (Trigger)                   | 1 (Trigger)     |
| Launch                 | Fire         | 2                             | 2               |
| Start                  | E            | 11                            | 7               |
| Select                 | Mouse Button | 12                            | 6               |
| Sub Stick Click        | A            | 5                             | 5               |


## Building
Requires a Raspberry Pi Pico (RP2040) board and a USB Type-A port for input.

Check the wiring guidance [here](https://github.com/sekigon-gonnoc/Pico-PIO-USB/discussions/7).

USB Pins can be changed. Just need to set them in code.

Define the `D+` pin on sketch. `D-` will be `D+` + 1.

Required configuration are on the main sketch file as `PIN_USB_HOST_DP`

Firmware builds under Arduino IDE.

Required libs. Install using Arduino IDE.

[arduino-pico (3.9.4)](https://github.com/earlephilhower/arduino-pico#installing-via-arduino-boards-manager)<br/>
[Pico-PIO-USB (0.5.3)](https://github.com/sekigon-gonnoc/Pico-PIO-USB)<br/>
[Adafruit_TinyUSB_Arduino (3.3.4)](https://github.com/adafruit/Adafruit_TinyUSB_Arduino)

Configure IDE as:
* Board: Raspberry Pi Pico
* CPU Speed: 120MHz
* USB Stack: Adafruit TinyUSB
 
Needs some modifications on Adafruit_TinyUSB_Arduino lib.

* Add to src/arduino/Adafruit_USBD_Device.h
```
void setDeviceClass(uint8_t bcd);
void setDeviceSubClass(uint8_t bcd);
void setDeviceProtocol(uint8_t bcd);
void ClearStringIndexes(void);
```

* Add to src/arduino/Adafruit_USBD_Device.cpp
```
void Adafruit_USBD_Device::setDeviceClass(uint8_t bcd) {
  _desc_device.bDeviceClass = bcd;
}
void Adafruit_USBD_Device::setDeviceSubClass(uint8_t bcd) {
  _desc_device.bDeviceSubClass = bcd;
}
void Adafruit_USBD_Device::setDeviceProtocol(uint8_t bcd) {
  _desc_device.bDeviceProtocol = bcd;
}
void Adafruit_USBD_Device::ClearStringIndexes(void) {
  _desc_device.iManufacturer = 0;
  _desc_device.iProduct = 0;
  _desc_device.iSerialNumber = 0;
}
```

## Ready to use binaries
Will be released shortly.

## Credits

Unofficial windows [driver](https://www.tamanegi.org/prog/hfsd/)

Vespa for all the amazing support and help with doing remote tests and diagnostics

## Disclaimer
Code and wiring directions are provided to you 'as is' and without any warranties. Use at your own risk.