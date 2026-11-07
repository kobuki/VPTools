# Davis bursts as seen on air

Captured on 2026-11-07 with an RTL-SDR stick. Settings: 2.4 Msps, gain 29.7. The stick reads about 2 ppm high, and no correction was applied.

- `eu.u8`: EU band, centre 867.8 MHz, 60 s. Only station 2 (the anemometer) decodes from the VM's location. Station 1 (the ISS) is too weak there.
- `us.u8`: US band, centre 915.1 MHz, 180 s. US test transmitter (packet ID field 0, so station 1).

| File | Carrier | Station | Packet (firmware byte order) |
|---|---|---|---|
| `eu-1.png` | 868.4215 MHz | 2 | E1 00 72 80 03 00 0F 8C FF FF |
| `eu-3.png` | 868.3013 MHz | 2 | C1 01 72 00 03 00 AB 8F FF FF |
| `us-1.png` | 914.0050 MHz | 1 | 60 00 00 FF C5 00 79 DA FF FF |
| `us-2.png` | 916.2963 MHz | 1 | E0 00 00 80 05 00 4F 97 FF FF |

## Panels

1. **Spectrogram**: the two FSK tones over the burst.
2. **Instantaneous frequency, whole burst**, plotted against the carrier centre:
   - raw demodulator output (grey) and a ½-bit smoothed version (blue);
   - a dashed ideal 2-FSK trace built from the decoded bits and aligned to the signal at sub-bit resolution;
   - shading for the lead carrier, preamble, sync word, data and trailer;
   - each data byte labelled in firmware order. Bytes go on air LSB first.
3. **Zoom** on the lead carrier, preamble and sync word, with the '1' and '0' tone levels marked. The tones are named as the RFM69 sees them.
4. **Envelope power**, relative to the median over the data section.

## What the plots show

- **The same layout on both bands:**
  - 18 bits of carrier on the '1' tone, lasting 0.94 ms;
  - 32 alternating preamble bits;
  - the sync word 0xCB89;
  - 10 data bytes;
  - a short trailer on the '1' tone.

  The whole burst lasts about 7.8 ms at 19.2 kbit/s.
- **Deviation** is ±9.6–10 kHz.
- **Single-bit pulses** come out rounded and don't quite reach full deviation, so the transmitter shapes or filters its modulation.
- **Long runs of one tone** overshoot to about 12 kHz and then settle back to about 10 kHz.
