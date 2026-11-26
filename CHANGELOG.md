# DavisRFM69 library changelog

Changes to the library: `DavisRFM69`, `PacketFifo` and `TimerOne`. The changes of the example sketches are in their own changelogs: [WxReceiver](Examples/WxReceiver/CHANGELOG.md), [RXTX](Examples/RXTX/CHANGELOG.md), [AnemometerTX](Examples/AnemometerTX/CHANGELOG.md).

The library version is `VPTOOLS_VERSION` in `DavisRFM69.h`, and the example sketches print it at startup after their own version, for example `# WxReceiver v2026112401 (VPTools v2026112401) ready to accept commands`. Before v2026112401 the library had no version string of its own; those entries are headed by the WxReceiver version that first shipped the change.

## v2026112501, 2026-11-25

- **Fixed: with software AFC, a station far in frequency from the tracked ones was found late or not at all.** Discovery listened with the correction of the last packet received, so every packet of a tracked station undid the offset that discovery had just measured for a missing one. Transmitters can be 10 kHz apart, and the receiver can't decode one while tuned for the other. Discovery now has its own correction:
  - It starts from the tracked stations' correction.
  - A discovery offset reading replaces it, and packets of tracked stations don't overwrite it for `DISCOVERY_STEP` (150 s). Each new reading restarts that time.
  - Readings are no longer taken while the receiver waits for a tracked station's packet on the discovery channel.
  - A station dropped after `RESYNC_THRESHOLD` misses is searched for at its own last correction.
- **Fixed: after a discovery reading the receiver waited two channels ahead instead of one** (since v2026110601). The 150 s discovery step fired right after every reading, so the transmitter came by one interval later than intended.
- **Fixed: with software AFC, a station whose first packet had no valid FEI reading was lost again** (since v2026102801). It got no correction of its own and was tracked at the discovery correction, which can be too far from it, until it was dropped after `RESYNC_THRESHOLD` misses. Its first correction is now the one its first packet was received with.
- **Changed: a station's first correction is its first reading in full,** not half way between it and the correction the packet was received with (since v2026102801). A station found far from the discovery correction was mistuned for several hops and lost most of its packets meanwhile.
- **Fixed: with software AFC, a packet that failed its CRC check on the discovery channel was not used as a discovery reading** (since v2026110601). A missing station received too far off to decode comes in like that: its preamble's offset is now a discovery reading, like an RSSI trigger without packet. On the US band, with a second transmitter about 9 kHz from the first, the second was found 3–114 s after the first in 10 of 10 restarts (before: 123 s, 376 s and not within 15 minutes in 3).

## v2026112401, 2026-11-24

- **Added: library version** `VPTOOLS_VERSION`.
- **Added: transmitter-only mode.** `stopReceiver()` followed by `enableTx()` runs a transmitter only: the radio sleeps between packets instead of returning to RX.
- **Added: `timeRxTimeout(bitrate, n)`** measures the board's clock against the RFM69's crystal: it times the RFM69's RX timeout of n units with `micros()`. The timeout unit is 1095 ppm longer than the 16 bit times the datasheet gives.
- **Added: `txTimeLeft()`**, the time until the next transmission.
- **Added: VP2 packet definitions** shared by the transmitting sketches: the ISS packet type sequence `VP2_TX_SEQ`, the "no sensor" values `VP2_NO_SENSOR`, and byte 4's wind bits `VP2_BYTE4_WIND_DIR_LSB` (lowest bit of the 9-bit wind direction) and `VP2_BYTE4_NO_WIND` (no anemometer), valid in the packet types of `VP2_BYTE4_WIND_TYPES`.

Reception is unchanged from v2026112301.

## v2026112301, 2026-11-23

- **Refined old frequency tables and added five new bands:** Japan (16 channels), India (19), a second Russian variant (2), Russia (5) and Taiwan (19), with the constants `FREQ_BAND_JP`, `FREQ_BAND_IN`, `FREQ_BAND_R2`, `FREQ_BAND_RU` and `FREQ_BAND_TW`.
- **Band numbers follow the Davis numbering.** The `FREQ_BAND_*` values are now Davis' band numbers, 0 to 8 (as used in the BAND command through console or Envoy serial). `FREQ_BAND_EU` is now 1 and `FREQ_BAND_AU` 2 (they were 2 and 1), and `FREQ_BAND_NZ` is 3. Code using the names needs no change.
- **`setBand()` keeps the discovery channel inside the band's table.** The discovery channel started at index 2, past the end of the 2-channel table. Invalid band numbers are ignored.

Changes for transmitting, used by RXTX:

- **Added: transmit timing follows the received stations** (`setTxTiming()`, `txPpm()`): the board measures the stations' intervals with its own clock and runs its transmit interval with the same perceived error. The Timer1 period is dithered, so the mean interval hits the target.
- **Added: transmit frequency correction** (`setTxAfc()`): the transmit frequency is shifted by the offset the software AFC learned on the received stations.
- **Added: Davis framing** (`setTxFrame()`): `TX_FRAME_DAVIS`, the default, sends an 18-bit lead, a 32-bit preamble and 10.4 kHz deviation as Davis transmitters do; `TX_FRAME_LEGACY` keeps the old 32-bit lead and 48-bit preamble.
- **Changed:** transmitted packets have their own buffer, separate from the receiver's, and the transmitter ID is set in byte 0 before the CRC. Transmitting works without any station configured.
- **Fixed:** a restart (`initialize()`, `setStations()`) kept the count of found stations, so after a band or station change at run time, discovery never ran again.
- **Fixed:** after a transmission the receiver returned to channel 0 of an untracked station instead of the channel it was on.
- **Fixed:** `enableTx()` sent an extra packet right away when a Timer1 overflow was pending.

## v2026110601, 2026-11-06 (f21d50d)

- **Faster discovery when the receiver is off frequency.** The software AFC sometimes measures a transmitter's offset on a preamble without getting its packet. When that happens, discovery now moves to the transmitter's *next* hop channel and waits there, one transmit interval away. Before, it stayed put until the transmitter came round again, a whole hop cycle later. On the US band with a receiver about 9 kHz off, acquisition after a restart dropped from 158–258 s to 32–119 s.

## v2026110301, 2026-11-03 (634a972)

- **US and EU frequency tables corrected by -2.0 ppm.** The tables were measured with an RTL-SDR stick that, calibrated afterwards against GSM cells, reads 2.0 ppm high. US entries moved by -30 FRF steps (-1.83 kHz) and EU by -28 steps (-1.71 kHz). With software AFC on this changes little; with AFC off it puts the receiver closer to the real transmitter frequencies.

## v2026110101, 2026-11-01 (8b17062)

- **Foreign packets no longer hijack a tracked station.** A foreign or noise packet that passes the Davis CRC by chance and carries a configured station's ID used to re-anchor that station's timing and hop channel. The receiver then ran out of step until it resynced, which took minutes. Now such a packet is rejected when either check fails:
  - **Trailer check:** a non-repeater packet must end in `FF FF`, allowing at most 2 bit errors (`TRAILER_MAX_ZERO_BITS`).
  - **Early-arrival check:** a packet arriving more than 20 ms early (`EARLY_PACKET_THRESH`) for each interval since the station was last heard is not accepted. Clock error between receiver and transmitter builds up with every missed interval, so the limit grows with misses. It tolerates up to about 0.8% clock error, which covers a ceramic resonator.
- **Repeater handling simplified.** The unused per-station `repeaterId` is gone. A packet that passes only the repeater CRC is now accepted only with repeater pass-through on.

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

This is the first version change since v2019122001. It also covers two master commits that had no version bump.

**Why software AFC:** with the AFC off, the demodulator only decodes within about ±2 kHz of the transmitter: partly at ±4 kHz, not at all at ±6 kHz. A wider bandwidth doesn't help. Real offsets were much larger. The US table was 26 kHz off and the EU channels up to 14 kHz. On top of that, receiver crystals sit 5–10 kHz off, an offset that differs per board and drifts with temperature. EU reception relied on the RFM69's HW AFC, which kept it at about 98% for years. HW AFC is the AFC built into the radio chip: when a signal appears, the chip measures its frequency error and retunes itself for that packet, with no help from the firmware. The US band barely worked. With the reworked receive path, HW AFC lost packets: on one receiver it got 94% on EU and 35% on US, where the software AFC gets 99% and 98%. A fixed table can't cover per-board offsets, so the receiver measures each transmitter's offset on its preamble and follows it per station.

- **Software AFC** (`setAfc()`). The RFM69's FEI is measured on each packet's preamble and averaged into a per-station correction. The receiver tunes with that correction whenever it expects that station. Modes: `RF69_AFC_OFF` (fixed frequency table), 1..`RF69_AFC_MAX` (software AFC, correction limited to ±1..50 kHz) and `RF69_AFC_HW` (the RFM69's built-in HW AFC).
- **HW AFC is no longer used by default.** It still acquires transmitters fastest, within seconds, but it loses more packets than the software AFC: a few percent on EU, most of them on US. A likely cause is that RSSI triggers on the unmodulated carrier leading every Davis packet, so HW AFC measures on that instead of the preamble. This is not confirmed.
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

## v2017121901, 2017-12-19

- Discovery channel changed, and RX timing values tuned.
- Fixed a mix-up between txChannel and txId in the library and the examples.
- The RX RSSI threshold is stored and applied whenever the radio switches to RX.

## v2017091301 / v2017091302, 2017-09-13

- Repeater pass-through, to help repeater reception.
- Transmit timing moved to a timer interrupt; the sketch fills packet data from a callback (`attachTxCallback`). Concurrent receive and transmit is possible.
- 1 ms timer moved from TimerOne to MsTimer2. TimerOne now times transmissions exactly.
- `timeBase`, to correct for crystal or resonator error.
- More interrupt safeguards. Frequency offset correction functions, and modem parameters tuned for more robust transmitting.

## v2017061401 / v2017061601, 2017-06-14

- Sources reformatted without tabs, and unused sources removed.

## v2017021801 / v2017021901, 2017-02-18..19

- ATmega644P/1284P (Moteino Mega) and ATmega32U4 support: chip-dependent DIO0 interrupt pin and number, overridable in the constructor (GitHub issue #3).
- Fixed a restart bug when the receiver started without stations, for example in transmit-only use (#2).

## v2016071301, 2016-07-13

The library rework that WxReceiver was built on:
- All reception logic moved into the DavisRFM69 class, so it can be used as a library.
- Stations resync individually, and a full resync happens only when all are lost.
- Discovery searches on one channel while still tracking stations already found. Lost stations get a channel shift on resync.
- Per-station statistics, a raw RSSI threshold helper, and handling of the repeater CRC.
- Fixes to wind direction, wind speed error correction, rain rate and solar/UV values.

## Before WxReceiver (2015–2016)

- **2015-02-28:** baseline, based on DeKay's DavisRFM69. External dependencies were moved into the project.
- **2015-04:** generic packet decoding for all packet types, Vue support, and decoded wind speed.
- **2015-09:**
  - runtime frequency band selection;
  - FEI and packet time delta in the packet data;
  - bandwidth tuned for console retransmits, with a 10-byte FIFO for repeater info.
- **2015-12:** `setBandwidth()` to switch between 25 and 50 kHz receive bandwidth.
- **2016-02:** New Zealand frequency table.
