# WxReceiver changelog

The entries follow the version string the sketch prints at startup (`# WxReceiver vYYYYMMDDNN (VPTools vYYYYMMDDNN) ready to accept commands`; the library version in brackets since v2026112401). Library changes are in the [library CHANGELOG](../../CHANGELOG.md), whose versions up to v2026112301 match the WxReceiver versions.

## v2026112501, 2026-11-25

- Library v2026112501: fixed late or missing discovery of a station far in frequency from the tracked ones.

## v2026112401, 2026-11-24

- **The startup banner shows the library version:** `# WxReceiver v2026112401 (VPTools v2026112401) ready to accept commands`.
- Library v2026112401. Reception is unchanged.

## v2026112301, 2026-11-23

- **`m` takes the Davis firmware's band numbers, 0 to 8:** 0 US, 1 EU, 2 AU, 3 NZ, 4 JP, 5 IN, 6 R2, 7 RU, 8 TW. NZ moved from `m5` to `m3`; `m5` is now India.
- Library v2026112301: refined frequency tables and five new bands.

## v2026111401, 2026-11-14 (cb211a9)

- **BMP388 oversampling and IIR filter now take effect.** The sketch set 8x temperature and 4x pressure oversampling and IIR filter coefficient 3 before `begin()`, which resets all three to off, so the sensor always ran without them. They are now set after `begin_I2C()`. Before the fix, pressure readings 30 s apart on one receiver differed by 0.08 hPa rms, and by up to 0.22 hPa.
- **BMP388 readings are current.** The library's `performReading()` starts a forced conversion but reads the result registers straight away, so each reading returned the conversion started 30 s earlier. The sketch now starts a conversion, waits 40 ms for it and then reads it.
- **BME280 takes one forced measurement per reading.** The sensor now sleeps between readings. Before, it ran in normal mode, and after each reading the sketch wrote 0 to `ctrl_meas`, potentially blocking measurements.
- **Sensor libraries updated.** Adafruit BMP3XX 2.x is now required. It also supports the BMP390, which 1.x rejects by its chip ID. SparkFun BME280 2.x is now required. It reads the H6 humidity calibration value as signed (1.x read it as unsigned, giving wrong humidity on chips where it is negative) and reads each value in one burst.

## v2026110601, 2026-11-06 (f21d50d)

- Library v2026110601: faster discovery when the receiver is off frequency.

## v2026110301, 2026-11-03 (634a972)

- Library v2026110301: US and EU frequency tables corrected by -2.0 ppm.

## v2026110101, 2026-11-01 (8b17062)

- Library v2026110101: foreign packets no longer hijack a tracked station. A packet that passes only the repeater CRC is now accepted only with repeater pass-through on (`r1`).

## v2026103101, 2026-10-31 (d34725d)

- Library v2026103101: fixed runaway miss counting and stuck rediscovery.

## v2026102803, 2026-10-28 (82e2ebf, b19e33d)

- Library v2026102803: discovery ignores FEI readings close to the RSSI threshold; each station's AFC starts from the frequency of its first packet.

## v2026102802, 2026-10-28 (d7352e1, 4b76198)

- Library v2026102802: measured discovery offset.

## v2026102801, 2026-10-28 (b9e0b67 … 379940b, b0bb4fd)

This is the first version change since v2019122001. It also covers two master commits that had no version bump; builds of those still report v2019122001.

- **New command `a` for the AFC mode:**
  - `a0`: off, fixed frequency table;
  - `a1`..`a50`: software AFC, correction limited to ±1..50 kHz;
  - `ah`: the RFM69's built-in HW AFC.

  The default is `a12`. The setting shows in the `?` status line, for example `# OK t3 f1 o3 m1 x200 r1 a12 running`.
- **FEI field in the `I` output line.** The last field is now the transmitter's offset from the frequency table in kHz, as the software AFC measured it. It is 0 with the AFC off, and the raw FEI with `ah`.
- Library v2026102801: software AFC, RX retuning and RSSI threshold fixes, re-measured EU and shifted US frequency tables, NZ hop order, wide reception bandwidth.

## v2019122001, 2019-12-20

- BMP388 support.

## v2018021601, 2018-02-16

- Version bump for release.

## v2017121901, 2017-12-19

- Barometer read interval set to 30 s.
- Library v2017121901: discovery channel and RX timing, RSSI threshold handling.

## v2017091301 / v2017091302, 2017-09-13

- Repeater pass-through (`r` command) to help repeater reception. The flag also shows in the info line.
- Library v2017091301: timer-driven transmitting, `timeBase`.

## v2017061401 / v2017061601, 2017-06-14

- Consecutive line-end characters are skipped by the serial command interpreter. Sources reformatted without tabs.

## v2017052401 … v2017052802, 2017-05-24..28

- BMP280 support, and standalone BMP180 support for temperature and humidity.
- Each sensor chip is selectable on its own: the Si7021 is decoupled from the BMP180.
- I2C address selection defines.
- Fixed the emulated pressure value and the sensor output.

## v2017042101, 2017-04-21

- The first BME280 reading is discarded, because it can be off.

## v2017021801 / v2017021901, 2017-02-18..19

- BME280 support (needs the SparkFun BME280 library). The sensor sleeps after each measurement.
- Fixed rain rate. `radio.initialize()` moved earlier in startup.
- Library v2017021801: ATmega644P/1284P and ATmega32U4 support, restart without stations.

## v2016071301, 2016-07-13

The first WxReceiver release, a receiver compatible with the weewx Meteostick driver, built on the 2016 library rework.
