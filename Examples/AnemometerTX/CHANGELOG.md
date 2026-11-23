# AnemometerTX changelog

The entries follow the version string the sketch prints at startup (`# AnemometerTX vYYYYMMDDNN ...`).

## v2026112301, 2026-11-23

Rewrite of the sketch.

- **Commands as RXTX's,** saved to EEPROM with `e`: band `m`, transmitter ID `i` (0 = off), timing `c`/`p`, module type `h`, framing `g`, plus `d`, `r` and `?`. Default `m1 i3 c1 p0 h1 g1`.
- **VP2 ISS type sequence with Davis' "no sensor" values** in bytes 3..5. They were sent as zeros.
- **The radio sleeps between transmissions.** The receiver no longer runs.
- **Transmit timing:** `c0` runs on the board's clock with a fixed correction `p` (boards with a quartz crystal). `c1` measures the board's clock against the RFM69's crystal and corrects the transmit interval with it (Moteinos with a ceramic resonator); `p` then corrects for the RFM69 crystal's own error. A calibration runs at startup and then every 30 min: a burst of 4 measurements, whose median is applied. Each measurement times a 2.2 s RFM69 RX timeout in one transmit gap and a 0.07 s one in the next, and takes their difference. Measurements more than 1000 ppm off count as failed. Its accuracy on hardware is not measured yet.
- **Wind speed drops to 0 when the cups stop.** It kept the last speed, because the calm check compared against a time stamp that was set only at startup.
- **Wind speed is the mean over all rotations since the previous packet**, with no rotation lost between packets, read 100 ms before each transmission. It was read after each transmission, so every packet carried data one interval old. When no pulse has come for longer than the last rotation period, that time is used as the period.
- **The direction-dependent speed correction is removed.** Its table lookup read past the end of the table above about 20 mph.
- New `T` and `K` output lines, see the sketch header.

Library changes:

- `stopReceiver()` followed by `enableTx()` runs a transmitter only: the radio sleeps between packets instead of returning to RX.
- New `timeRxTimeout()` (clock measurement against the RFM69 crystal) and `txTimeLeft()`.
