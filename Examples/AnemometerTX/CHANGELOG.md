# AnemometerTX changelog

The entries follow the version string the sketch prints at startup (`# AnemometerTX vYYYYMMDDNN ...`).

## v2026112401, 2026-11-24

- **Gust packets carry the 10-minute gust,** as the ISS sends it: byte 3 the highest wind speed in the last 10 gust periods (the 20-packet sequence, 51–60 s depending on the ID), byte 5 its age in gust periods (upper nibble) and its direction in 16 sectors (lower nibble), byte 4 = 1. The format was worked out from a day of a real ISS's packets: the direction nibble is `(dir + 8) >> 4` of the gust's reading in all 894 gust packets, and a speed drops out of the window after 10 gust periods. They were sent with the "no sensor" values.
- **Contact bounce when the switch opens no longer counts as a rotation.** The debounce only ignored edges within 5 ms of the previous closure, so chatter at the end of a longer closure doubled the speed (10 mph read as 19–21 on the bench). Now both edges interrupt, and a rotation is the first edge after the switch has rested open for 5 ms.
- **Build option `-DWIND_GATE_US=2250000`:** wind speed as the ISS measures it, the rotations counted in a 2.25 s gate that ends before each transmission (1 rotation = 1 mph). The default is still the mean over all rotations since the previous packet.
- The VP2 packet type sequence and the "no sensor" values moved to `DavisRFM69.h` (`VP2_TX_SEQ`, `VP2_NO_SENSOR`), shared with RXTX.

## v2026112301, 2026-11-23

Rewrite of the sketch.

- **Commands as RXTX's,** saved to EEPROM with `e`: band `m`, transmitter ID `i` (0 = off), timing `c`/`p`, module type `h`, framing `g`, plus `d`, `r` and `?`. Default `m1 i3 c1 p0 h1 g1`.
- **VP2 ISS type sequence with Davis' "no sensor" values** in bytes 3..5. They were sent as zeros.
- **The radio sleeps between transmissions.** The receiver no longer runs.
- **Transmit timing:** `c0` runs on the board's clock with a fixed correction `p` (boards with a quartz crystal). `c1` measures the board's clock against the RFM69's crystal and corrects the transmit interval with it (Moteinos with a ceramic resonator); `p` then corrects for the RFM69 crystal's own error. A calibration runs at startup and then every 30 min: a burst of 4 measurements, whose median is applied. Each measurement times a 2.2 s RFM69 RX timeout in one transmit gap and a 0.07 s one in the next, and takes their difference. Measurements more than 1000 ppm off count as failed.
  - The RFM69's timeout unit is 1095 ppm longer than the 16 bit times the datasheet gives, at every bitrate; measured on an RFM69HCW and an RFM69CW against NTP time, ±5 ppm. The sketch corrects for it (`CAL_UNIT_PPM`).
  - On a Moteino with a resonator, measurements scattered by 3.5 ppm and burst medians by 2.5 ppm; with the correction, the transmit interval was 1.6 ppm from nominal over 15 min, against NTP time.
- **Wind speed drops to 0 when the cups stop.** It kept the last speed, because the calm check compared against a time stamp that was set only at startup.
- **Wind speed is the mean over all rotations since the previous packet**, with no rotation lost between packets, read 100 ms before each transmission. It was read after each transmission, so every packet carried data one interval old. When no pulse has come for longer than the last rotation period, that time is used as the period.
- **The direction-dependent speed correction is removed.** Its table lookup read past the end of the table above about 20 mph.
- New `T` and `K` output lines, see the sketch header.

Library changes:

- `stopReceiver()` followed by `enableTx()` runs a transmitter only: the radio sleeps between packets instead of returning to RX.
- New `timeRxTimeout()` (clock measurement against the RFM69 crystal) and `txTimeLeft()`.
