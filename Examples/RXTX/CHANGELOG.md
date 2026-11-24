# RXTX changelog

The entries follow the version string the sketch prints at startup (`# RXTX vYYYYMMDDNN ...`). Changes to the DavisRFM69 library are listed in the main [CHANGELOG](../../CHANGELOG.md) under the WxReceiver versions.

## v2026112403, 2026-11-24

- **Fixed: byte 4's wind bits come from the wind source.** Bit 1 is the lowest bit of the 9-bit wind direction, bit 2 means no anemometer. Both were taken from each packet type's own source: when that wasn't the wind source, a receiver's wind direction could be off by one step (about 0.7°), and a wind source without an anemometer was sent as calm instead of as no wind.

## v2026112402, 2026-11-24

- **Bit 2 of byte 4 ("no anemometer") follows the wind source:** cleared while the wind source is fresh, set while there is no wind. The "no sensor" values of the UV and solar packets (`FF C5 00`) set it, so a receiver dropped the wind on those packets: it showed no wind speed and direction until the next packet.

## v2026112401, 2026-11-24

- No functional change. The VP2 packet type sequence and the "no sensor" values moved to `DavisRFM69.h` (`VP2_TX_SEQ`, `VP2_NO_SENSOR`), shared with AnemometerTX.

## v2026112302, 2026-11-23

- **LED blinks on traffic:** 20 ms for each received packet, 100 ms for each transmission, on D9 (the Moteino LED). Set `LED` to -1 in the sketch for a board without one. The blink is switched from the main loop without blocking, so it doesn't affect receive or transmit timing.

## v2026112301, 2026-11-23 (716de69)

- **`m` takes the Davis firmware's band numbers, 0 to 8:** 0 US, 1 EU, 2 AU, 3 NZ, 4 JP, 5 IN, 6 R2, 7 RU, 8 TW. NZ moved from `m5` to `m3`. A board that saved `m5` starts on the India band: set `m3` and save with `e`.
- **Frequency tables from the Davis firmware**, see the main [CHANGELOG](../../CHANGELOG.md).

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