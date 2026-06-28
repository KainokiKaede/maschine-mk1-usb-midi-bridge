# Maschine MK1 USB-MIDI Bridge

Teensy 4.1 firmware that turns a Native Instruments Maschine MK1 into a USB-MIDI controller.

## Hardware

- [Teensy 4.1](https://www.pjrc.com/store/teensy41.html)
- [USB Host Cable For Teensy 3.6 or Teensy 4.1](https://www.pjrc.com/store/cable_usb_host_t36.html)
- 2.54 mm pitch pin header, 5 pins
- Maschine MK1

Use Arduino IDE with Teensyduino installed. Select **Teensy 4.1** and a USB type that includes MIDI, such as **MIDI** or **Serial + MIDI**.

## Getting Started

1. Obtain the Teensy 4.1, USB host cable, 2.54 mm pitch pin header, and Maschine MK1, then assemble them so the Maschine is connected to the Teensy USB host port.
2. Follow the [Teensy download and install guide](https://www.pjrc.com/teensy/td_download.html) to make Arduino IDE able to compile and upload code for Teensy.
3. Compile the  code and upload it to the Teensy 4.1.
4. You now have a USB-MIDI Maschine MK1!

## MIDI Mapping

- Pads send notes on channel 10.
- Group buttons A-H switch fixed pad-note banks.
- Top display buttons send CC 14-21 on channel 1.
- While the shift button is held, other mapped buttons and encoders send on channel 2.
- Extra button CCs are CC 30-41, CC 45, and CC 46.
- Encoders 1-8 send CC 22-29 on channel 1. Volume encoder sends CC 42, tempo encoder sends CC 43, and swing encoder sends CC 44.
- The LCDs show the eight top-button CC numbers, the eight knob CC numbers, and bars for the current knob values. Channel 1 and channel 2 knob values are stored separately; holding Shift shows the channel 2 values.
- Transport and common buttons send the notes/CCs listed in `BUTTON_MAP`.
- Incoming USB MIDI CC 14-21 controls the matching top-button LEDs.
- Incoming USB MIDI CC 22-29 updates the visible encoder values.

The sketch also supports routing generated controller MIDI to the Maschine DIN MIDI output. Routing options are build-time defines near the top of the sketch.

## Notes

This implementation used the Linux `snd-usb-caiaq` driver and CABL's Maschine MK1 implementation as protocol references.
