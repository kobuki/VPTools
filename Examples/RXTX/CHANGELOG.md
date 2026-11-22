# RXTX changelog

The entries follow the version string the sketch prints at startup (`# RXTX vYYYYMMDDNN ...`). Changes to the DavisRFM69 library are listed in the main [CHANGELOG](../../CHANGELOG.md) under the WxReceiver versions.

## v2026112105, 2026-11-21 (40804ff)

First published version.

### Working features

- Receives Davis transmitters (tested with two: ISS and anemometer transmitter) and mixes them into one VP2 ISS stream, with a source station for each packet type and for wind.
- Sends Davis "no sensor" values for a source silent longer than the timeout (`n`).
- Learns transmit timing from the received stations (`c1`). The mean interval stays within 0.2 ppm of its target.
- Corrects the transmit frequency from the software AFC's learned offset (`f1`). The SDR measured it within 0.5 kHz of a real transmitter.
- Uses Davis-like on-air framing (`g1`, default), which the SDR showed matches Davis in lead, preamble and deviation.
- Supports RFM69 W/CW and HW/HCW modules (`h`).
- Has been tested on the EU and US bands with a Davis Envoy.
- Prints `I` lines for received packets and `T` lines for transmitted packets, including timing and frequency diagnostics.
- Settings are saved to EEPROM only with the `e` command (magic 0x5258). Settings saved by earlier development builds are ignored, and the defaults are loaded instead.