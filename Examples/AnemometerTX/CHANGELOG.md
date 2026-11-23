# AnemometerTX changelog

The entries follow the version string the sketch prints at startup (`# AnemometerTX vYYYYMMDDNN ...`).

## v2026112301, 2026-11-23

Rewrite of the sketch.

- **Commands as RXTX's,** saved to EEPROM with `e`: band `m`, transmitter ID `i` (0 = off), timing `c`/`p`, module type `h`, framing `g`, plus `d`, `r` and `?`. Default `m1 i3 c1 p0 h1 g1`.
- **VP2 ISS type sequence with Davis' "no sensor" values** in bytes 3..5. They were sent as zeros.
- **The radio sleeps between transmissions.** The receiver no longer runs.
- **Transmit timing:** `c0` runs on the board's clock with a fixed correction `p` (boards with a quartz crystal). `c1` measures the board's clock against the RFM69's crystal every 60 s and corrects the transmit interval with it (Moteinos with a ceramic resonator). Each measurement times two RFM69 RX timeouts, 1.25 s and 0.31 s, and takes their difference. Its accuracy on hardware is not measured yet.
- **Wind speed drops to 0 when the cups stop.** It kept the last speed, because the calm check compared against a time stamp that was set only at startup.
- **Wind speed is the mean over all rotations since the previous packet**, with no rotation lost between packets, read 100 ms before each transmission. It was read after each transmission, so every packet carried data one interval old. When no pulse has come for longer than the last rotation period, that time is used as the period.
- **The direction-dependent speed correction is removed.** Its table lookup read past the end of the table above about 20 mph.
- New `T` and `K` output lines, see the sketch header.

Library changes:

- `stopReceiver()` followed by `enableTx()` runs a transmitter only: the radio sleeps between packets instead of returning to RX.
- `setTxTiming()` and `txPpm()` take and return `int32_t`, so corrections beyond ±2048 ppm fit; the interval correction is computed in 64 bits.
- New `timeRxTimeout()` (clock measurement against the RFM69 crystal) and `txTimeLeft()`.
