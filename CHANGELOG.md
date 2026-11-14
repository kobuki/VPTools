# Changelog

The entries follow the version string the WxReceiver sketch prints at startup (`# WxReceiver vYYYYMMDDNN ready to accept commands`). Most changes live in the DavisRFM69 library, which all examples share.

## v2026111401, 2026-11-14 (cb211a9)

- **BMP388 oversampling and IIR filter now take effect.** The sketch set 8x temperature and 4x pressure oversampling and IIR filter coefficient 3 before `begin()`, which resets all three to off, so the sensor always ran without them. They are now set after `begin_I2C()`. Before the fix, pressure readings 30 s apart on one receiver differed by 0.08 hPa rms, and by up to 0.22 hPa.
- **BMP388 readings are current.** The library's `performReading()` starts a forced conversion but reads the result registers straight away, so each reading returned the conversion started 30 s earlier. The sketch now starts a conversion, waits 40 ms for it and then reads it.
- **BME280 readings update again.** After each reading the sketch wrote 0 to `ctrl_meas`, which puts the sensor to sleep with measurement disabled, and nothing restarted it, so every reading after the first was the same. This is from the code and not confirmed on hardware. The sketch now takes one forced measurement per reading, and the sensor sleeps in between.
- **Sensor libraries updated.** Adafruit BMP3XX 2.x is now required. It also supports the BMP390, which 1.x rejects by its chip ID. SparkFun BME280 2.x is now required. It reads the H6 humidity calibration value as signed (1.x read it as unsigned, giving wrong humidity on chips where it is negative) and reads each value in one burst.

## v2026110601, 2026-11-06 (f21d50d)

- **Faster discovery when the receiver is off frequency.** The software AFC sometimes measures a transmitter's offset on a preamble without getting its packet. When that happens, discovery now moves to the transmitter's *next* hop channel and waits there, one transmit interval away. Before, it stayed put until the transmitter came round again, a whole hop cycle later. On the US band with a receiver about 9 kHz off, acquisition after a restart dropped from 158–258 s to 32–119 s.

## v2026110301, 2026-11-03 (634a972)

- **US and EU frequency tables corrected by -2.0 ppm.** The tables were measured with an RTL-SDR stick that, calibrated afterwards against GSM cells, reads 2.0 ppm high. US entries moved by -30 FRF steps (-1.83 kHz) and EU by -28 steps (-1.71 kHz). With software AFC on this changes little; with AFC off it puts the receiver closer to the real transmitter frequencies.

## v2026110101, 2026-11-01 (8b17062)

- **Foreign packets no longer hijack a tracked station.** A foreign or noise packet that passes the Davis CRC by chance and carries a configured station's ID used to re-anchor that station's timing and hop channel. The receiver then ran out of step until it resynced, which took minutes. Now such a packet is rejected when either check fails:
  - **Trailer check:** a non-repeater packet must end in `FF FF`, allowing at most 2 bit errors (`TRAILER_MAX_ZERO_BITS`).
  - **Early-arrival check:** a packet arriving more than 20 ms early (`EARLY_PACKET_THRESH`) for each interval since the station was last heard is not accepted. Clock error between receiver and transmitter builds up with every missed interval, so the limit grows with misses. It tolerates up to about 0.8% clock error, which covers a ceramic resonator.
- **Repeater handling simplified.** The unused per-station `repeaterId` is gone. A packet that passes only the repeater CRC is now accepted only with repeater pass-through on (`r1`).

## v2026103101, 2026-10-31 (d34725d)

- **Fixed runaway miss counting.** The check for a late packet used unsigned arithmetic. When a station's last-received time lay ahead of the clock, the lateness wrapped to a huge value, so every 1 ms timer tick counted a miss. The station was dropped and resynced within about 50 ms, even though nothing was wrong.

  This happened because the timer interrupt could read a half-written 32-bit timestamp: the SPI access in the radio interrupt re-enables interrupts. The fix has several parts:
  - lateness is signed;
  - a timestamp more than an interval ahead is re-anchored;
  - the station update in the radio interrupt runs with interrupts off.

  The bug only became reachable with the radio interrupt reordering in v2026102801.
- **Fixed stuck rediscovery.** The station counter no longer underflows, and "all stations lost" now means no station is tracked. With nothing tracked, the receiver always retunes to the discovery channel, which also rearms a stuck receiver.

## v2026102803, 2026-10-28 (82e2ebf, b19e33d)

- **Discovery ignores FEI readings close to the RSSI threshold.** Readings 3–8 dB above it count only when two agree within 1.5 kHz, and weaker ones are ignored. Readings on noise had detuned discovery and stretched one startup to 12 minutes.
- **Each station's AFC starts from the frequency its first packet came through on.** Before, it started from 0.

## v2026102802, 2026-10-28 (d7352e1, 4b76198)

- **Measured discovery offset.** A transmitter that is several kHz off the table is often heard (RSSI triggers on its preamble) but not decoded. During discovery the FEI measured on such a preamble now becomes the correction for the discovery channel, so the next attempt is tuned to the transmitter.

## v2026102801, 2026-10-28 (b9e0b67 … 379940b, b0bb4fd)

This is the first version change since v2019122001. It also covers two master commits that had no version bump; builds of those still report v2019122001.

**Why software AFC:** with the AFC off, the demodulator only decodes within about ±2 kHz of the transmitter: partly at ±4 kHz, not at all at ±6 kHz. A wider bandwidth doesn't help. Real offsets were much larger. The US table was 26 kHz off and the EU channels up to 14 kHz. On top of that, receiver crystals sit 5–10 kHz off, an offset that differs per board and drifts with temperature. EU reception relied on the RFM69's HW AFC, which kept it at about 98% for years. HW AFC is the AFC built into the radio chip: when a signal appears, the chip measures its frequency error and retunes itself for that packet, with no help from the firmware. The US band barely worked. With the reworked receive path, HW AFC lost packets: on one receiver it got 94% on EU and 35% on US, where the software AFC gets 99% and 98%. A fixed table can't cover per-board offsets, so the receiver measures each transmitter's offset on its preamble and follows it per station.

- **Software AFC, new command `a`.** The RFM69's FEI is measured on each packet's preamble and averaged into a per-station correction. The receiver tunes with that correction whenever it expects that station. Setting values:
  - `a0`: off, fixed frequency table;
  - `a1`..`a50`: software AFC, correction limited to ±1..50 kHz;
  - `ah`: the RFM69's built-in HW AFC.

  The default is `a12`. The setting shows in the `?` status line, for example `# OK t3 f1 o3 m1 x200 r1 a12 running`.
- **FEI field in the `I` output line.** The last field is now the transmitter's offset from the frequency table in kHz, as the software AFC measured it. It is 0 with the AFC off, and the raw FEI with `ah`.
- **HW AFC is off by default.** It still acquires transmitters fastest, within seconds, but it loses more packets than the software AFC: a few percent on EU, most of them on US. A likely cause is that RSSI triggers on the unmodulated carrier leading every Davis packet, so HW AFC measures on that instead of the preamble. This is not confirmed.
- **Fixed RX retuning:**
  - A new frequency now takes effect: the radio passes through SYNTH mode, because frequency changes in RX mode only apply after leaving RX.
  - The first register write after a packet (FRFMSB) is no longer swallowed by the FIFO burst; the packet handler runs after the chip select is released.
  - Discovery actually moves the receiver to the next discovery channel.
- **Fixed RSSI threshold.** The threshold was written to the read-only RSSI value register, so it never applied. The default is now raw 190 (-95 dBm), the same as the init table.
- **Fixed a reset at receiver start.** A DIO0 interrupt left pending by the still-running radio fired before `selfPointer` was set.
- **EU frequency table re-measured.** It was taken with an SDR against real transmitters. The old table assumed 115.2 kHz channel spacing; the real spacing is about 120.15 kHz, and channels were 6–14 kHz off.
- **US frequency table shifted by -26.3 kHz** (f6cf493, on master) to match measured transmitter frequencies.
- **NZ frequency table hop order fixed.** It was the reverse of the transmission sequence.
- **Wide reception bandwidth** (61e39ac, on master, 2025-02-07). It lets the receiver pick up SIM transmitters made from 2024 on.

## v2019122001, 2019-12-20

- BMP388 support.

## v2018021601, 2018-02-16

- Version bump for release.

## v2017121901, 2017-12-19

- Discovery channel changed, and RX timing values tuned.
- Fixed a mix-up between txChannel and txId in the library and the examples.
- The RX RSSI threshold is stored and applied whenever the radio switches to RX.
- Barometer read interval set to 30 s.

## v2017091301 / v2017091302, 2017-09-13

- Repeater pass-through (`r` command) to help repeater reception. The flag also shows in the info line.
- Transmit timing moved to a timer interrupt; the sketch fills packet data from a callback (`attachTxCallback`). Concurrent receive and transmit is possible (RXTX example, testing stage).
- 1 ms timer moved from TimerOne to MsTimer2. TimerOne now times transmissions exactly.
- `timeBase`, to correct for crystal or resonator error.
- More interrupt safeguards. Frequency offset correction functions, and modem parameters tuned for more robust transmitting.

## v2017061401 / v2017061601, 2017-06-14

- Consecutive line-end characters are skipped by the serial command interpreter. Sources reformatted without tabs, and unused sources removed.

## v2017052401 … v2017052802, 2017-05-24..28

- BMP280 support, and standalone BMP180 support for temperature and humidity.
- Each sensor chip is selectable on its own: the Si7021 is decoupled from the BMP180.
- I2C address selection defines.
- Fixed the emulated pressure value and the sensor output.

## v2017042101, 2017-04-21

- The first BME280 reading is discarded, because it can be off.

## v2017021801 / v2017021901, 2017-02-18..19

- BME280 support (needs the SparkFun BME280 library). The sensor sleeps after each measurement.
- ATmega644P/1284P (Moteino Mega) and ATmega32U4 support: chip-dependent DIO0 interrupt pin and number, overridable in the constructor (GitHub issue #3).
- Fixed a restart bug when the receiver started without stations, for example in transmit-only use (#2).
- Fixed rain rate. `radio.initialize()` moved earlier in startup.

## v2016071301, 2016-07-13

The first WxReceiver release, a receiver compatible with the weewx Meteostick driver. It was built on the 2016 library rework:
- All reception logic moved into the DavisRFM69 class, so it can be used as a library.
- Stations resync individually, and a full resync happens only when all are lost.
- Discovery searches on one channel while still tracking stations already found. Lost stations get a channel shift on resync.
- Per-station statistics, a raw RSSI threshold helper, and handling of the repeater CRC.
- Fixes to wind direction, wind speed error correction, rain rate and solar/UV values.

## Library history before WxReceiver (2015–2016)

- **2015-02-28:** baseline, based on DeKay's DavisRFM69. External dependencies were moved into the project.
- **2015-04:** generic packet decoding for all packet types, Vue support, and decoded wind speed.
- **2015-09:**
  - runtime frequency band selection;
  - FEI and packet time delta in the packet data;
  - bandwidth tuned for console retransmits, with a 10-byte FIFO for repeater info.
- **2015-12:** `setBandwidth()` to switch between 25 and 50 kHz receive bandwidth.
- **2016-02:** New Zealand frequency table.
