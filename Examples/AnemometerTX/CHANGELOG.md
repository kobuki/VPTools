# AnemometerTX changelog

The entries follow the version string the sketch prints at startup (`# AnemometerTX vYYYYMMDDNN ...`). Changes to the DavisRFM69 library are listed in the [library CHANGELOG](../../CHANGELOG.md).

## v2026112404, 2026-11-24

- **The startup line shows the library version:** `# AnemometerTX v2026112404 (VPTools v2026112401) settings from EEPROM`.

## v2026112403, 2026-11-24

First tested release. The sketch was rewritten; compared with the legacy code:

- **Commands as RXTX's,** saved to EEPROM with `e`: band `m` (the Davis band numbers 0 to 8), transmitter ID `i` (0 = off), transmit timing `c`/`p`, RFM69 module type `h`, on-air framing `g`, plus `d`, `r` and `?`. Defaults `m1 i3 c1 p0 h1 g1`.
- **VP2 ISS packet sequence and timing:** wind speed and the 9-bit wind direction in every packet, the 10-minute gust in the gust packets, Davis' "no sensor" values in the other packet types, with byte 4's "no anemometer" bit cleared.
- **Gust packets as the ISS sends them:** byte 3 the highest wind speed in the last 10 gust periods (a gust period is the 20-packet sequence), byte 5 its age in gust periods (upper nibble) and its direction in 16 sectors (lower nibble, `(dir + 8) >> 4`).
- **Wind speed** is the mean over all rotations since the previous packet, read 100 ms before each transmission; 0 when no rotation came for 2.25 s (below 1 mph); when the cups slow down, the time since the last rotation is used as the period. Build option `-DWIND_GATE_US=2250000` counts the rotations in a 2.25 s gate before each transmission instead, as the ISS does.
- **Wind speed is sent raw.** Receivers apply the direction-dependent speed correction themselves.
- **Debouncing on both switch edges:** a rotation is the first edge after the switch has rested open for 5 ms, so contact bounce on closing and on opening doesn't count.
- **Transmit timing:** `c0` runs on the board's clock with a fixed correction `p` (boards with a quartz crystal). `c1` measures the board's clock against the RFM69's crystal at startup and then every 30 min, and corrects the transmit interval with it (boards with a ceramic resonator); `p` then corrects for the RFM69 crystal's own error. The RFM69's RX timeout unit is 1095 ppm longer than the 16 bit times the datasheet gives; the sketch corrects for it (`CAL_UNIT_PPM`).
- **The radio only transmits** and sleeps between packets.
- **Output:** `T` lines for transmitted packets, and with `c1` `K` and `C` lines for the clock measurements, see the sketch header.

Library changes:

- `stopReceiver()` followed by `enableTx()` runs a transmitter only: the radio sleeps between packets instead of returning to RX.
- New `timeRxTimeout()` (clock measurement against the RFM69 crystal) and `txTimeLeft()`.
- The VP2 packet sequence, the "no sensor" values and byte 4's wind bits are in `DavisRFM69.h` (`VP2_TX_SEQ`, `VP2_NO_SENSOR`, `VP2_BYTE4_*`), shared with RXTX.

## Legacy versions

Untested legacy code without a version string.
