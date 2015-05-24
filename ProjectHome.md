C18 Project based on Microchip USB Framework. Implements an  USB HID mouse device that's controlled by motion (tilt and pan). Can be used with 18F2555/18F4555 devices or can be adapted to other PIC microcontrollers with USB support.

### Requirements ###

  * MCC18 compiler version 3.21 or higher.

  * Microchip USB firmware framework version 2.3 or higher.

### Notes ###

  * Microchip changed it's framework in the past, if current version is higher than 2.3, code might need updates.

  * The .mcp project references some files from  USB framework, it is assumed that the framework and the C18 compiler are installed in the default locations: C:\Microchip Solutions\    C:\MCC18\ . If you installed in a different directory just update the missing file references in your project file to your path.

### Hardware Circuit Schematics ###

  * See: http://starlino.com/usb_gamepad.html (USB Mouse uses same hardware as USB Gamepad)