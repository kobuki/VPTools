RXTX
----

RXTX receives several Davis transmitters and retransmits their data mixed into a single VP2 ISS packet stream on a transmitter ID of its own.

The older Vantage consoles, the Envoy and most older software that supports Davis stations take each sensor from a fixed transmitter type. Only one ISS is allowed, so data from separate transmitters can't always be combined there. For example, you might want wind, gust, UV and solar from a separate transmitter with a SIM board, and temperature, humidity and rain from the ISS. RXTX takes each packet type from the station you configure and sends the result as one ISS. The console or Envoy then only needs that one ISS channel.

If a source hasn't been heard for longer than the timeout, its packet types go out with Davis' "no sensor" values. The console then shows dashes instead of stale data.

## Hardware

Any ATmega328P board with an RFM69 module (a Moteino is suggested) for the band works. No hardware changes are needed for a Moteino. Set `h` to match the module: the W/CW modules use PA0, the HW/HCW modules use PA_BOOST. With the wrong setting the board transmits only leakage, which a console may not pick up.

The board talks over the serial port at 115200 baud. Commands end with CR or LF.

## Commands and settings

Each setting takes effect at once and is echoed in the status line, but it is **not saved**. Settings persist only after the `e` command, which writes them to EEPROM. At the next start the saved settings are loaded. Anything changed since the last `e` is lost when the board resets or loses power. Without saved settings, the board starts with the defaults below.

| Command | Default | Meaning |
|---|---|---|
| `m<0..8>` | `m1` | Band: 0 US, 1 EU, 2 AU, 3 NZ, 4 JP, 5 IN, 6 R2, 7 RU, 8 TW. |
| `t<mask>` | `t3` | Stations to receive, as a bit mask: bit 0 = ID 1 … bit 7 = ID 8. `t3` = IDs 1 and 2. |
| `i<0..8>` | `i3` | Transmitter ID to send as. `i0` switches the transmitter off, so the board only receives. |
| `x<raw>` | `x190` | RSSI threshold, raw value (dBm × −2): 190 = −95 dBm. |
| `a<n\|h>` | `a12` | Receive AFC: `a0` off, `a1`..`a50` software AFC with the correction limited to ±n kHz, `ah` the RFM69's built-in HW AFC. |
| `c<0\|1>` | `c1` | Transmit timing: 0 = fixed at `p`; 1 = learned from the received stations, starting from `p`. |
| `p<ppm>` | `p0` | Transmit timing correction in ppm (integer). With `c1` it is only the starting value. |
| `f<0\|1>` | `f1` | Transmit frequency correction: 1 = shift the transmit frequency by the offset the software AFC measured on the received stations. |
| `w<0..8>` | `w2` | Station ID that wind speed and direction come from. They are in every packet. 0 = no wind sensor. |
| `s<T><0..8>` | see below | Source station ID for packet type T. T is the hex type nibble from byte 0. For example, `s92` takes gust from ID 2. 0 = always send "no sensor". |
| `n<1..255>` | `n180` | Seconds without data from a source before its "no sensor" values are sent. The rarest types come only once every 20 packets (about 52 s), so keep it well above that. |
| `h<0\|1>` | `h1` | RFM69 module: 0 = W/CW (PA0, +13 dBm), 1 = HW/HCW (PA_BOOST, +20 dBm). |
| `g<0\|1>` | `g1` | On-air framing: 0 = legacy (32-bit lead, 48-bit preamble); 1 = as measured on Davis transmitters (18-bit lead, 32-bit preamble, 10.4 kHz deviation). |
| `e` | | Save the settings to EEPROM. |
| `d` | | Load the defaults (not saved). |
| `r` | | Reset the board. |
| `?` | | Show the settings. |

These are the packet types the VP2 ISS sequence sends, with their default sources:

| Type | Data | Default source |
|---|---|---|
| `4` | UV | ID 2 |
| `5` | rain rate | ID 1 |
| `6` | solar radiation | ID 2 |
| `8` | temperature | ID 1 |
| `9` | gust | ID 2 |
| `A` | humidity | ID 1 |
| `E` | rain | ID 1 |

The status line shows all settings, for example:

```
# OK m1 t3 i3 x190 a12 c1 p0 f1 w2 n180 h1 g1 s 4:2 5:1 6:2 8:1 9:2 A:1 E:1
```

The battery-low bit of a sent packet is set if any source that contributed to that packet reports a low battery.

## Example: ISS on ID 1, anemometer transmitter on ID 2, sent as ID 3

This is a sensible setup that matches the defaults. The ISS on ID 1 provides temperature, humidity and rain. The anemometer transmitter on ID 2 provides wind and gust, plus UV and solar. RXTX sends all of it as ID 3. On the EU band the defaults already do this. On the US band, change only the band:

```
d
m0
e
```

Written out in full, that is:

```
m0 t3 i3 w2 s42 s52 s62 s81 s92 sA1 sE1 e
```

Send each command on its own line.

On the console or Envoy, set ID 3 as the ISS and switch IDs 1 and 2 off. Otherwise the console also takes data straight from the real transmitters.

### Acquisition time and the transmitter ID

After a power-up or `NEWSETUP`, the Envoy took much longer to lock on to some IDs than others. ID 1 was the fastest, at about 30–40 s. ID 3 took about 4 minutes. Real Davis transmitters and RXTX behaved the same way. We didn't test the other IDs, but the case most likely holds for those as well.

So if a faster startup matters, IDs can be moved around: put the ISS on ID 2, other transmitters to any other id, like ID 3 with the DIP switches and let RXTX send as ID 1:

```
t6 i1 w2 s42 s53 s62 s83 s92 sA3 sE3 e
```

Over the long term this makes little difference. Acquisition only happens after a restart. Once locked, the console receives RXTX like a real transmitter, whatever the ID.

## Timing and frequency

RXTX needs no per-board calibration, but it needs at least one real transmitter in reception range:

- **Timing (`c1`):** the board measures the received stations' intervals with its own clock and runs its transmit interval with the same perceived error. Its transmissions then keep step with the real transmitters as the console sees them. The value is a relative error between the clocks, not the board's absolute clock error. Samples more than 50 ppm from the estimate are rejected once the estimate has settled. The transmit interval is dithered in 128 µs timer steps so that its mean hits the target.
- **Frequency (`f1`):** the transmit frequency is shifted by the mean offset that the software AFC learned on the received stations. If the board's crystal is several kHz off, its transmissions are just as far off from the frequency table, and this shift corrects that. It needs the software AFC (`a1`..`a50`).

## Output

Received packets are printed as `I` lines, in the same format as the WxReceiver sketch:

```
I 125 52 00 00 FF 75 00 C3 1B  -47 2687384 7
```

The fields are `1` and the channel (here 25), the 8 packet bytes, RSSI (dBm), time since the station's previous packet (µs) and the AFC's frequency offset (kHz).

Transmitted packets are printed as `T` lines:

```
T 36 80 00 00 28 CD 00 9B 4F FF FF 2562432 2562446 -20.8 18884 0 6.9
```

The fields are the channel, the 10 bytes sent, the transmit interval that just ended and the interval it was scheduled for (µs), the learned timing (ppm), the number of timing samples and of rejected samples, and the transmit frequency correction (kHz).

## Measured results

Tested with a Davis Envoy (VP2 firmware 3.15) and real Davis transmitters:

- **Reception:** once locked, the Envoy received RXTX as reliably as a real transmitter, tested on bands: EU, US, AU, NZ.
- **Timing:** the learned timing was stable, and the mean transmit interval was within 0.2 ppm of its target.
- **SDR check:** RXTX's frequency was within 0.5 kHz of the real transmitter, with the same hop order and interval jitter below 0.1 ms. With `g1` the lead, preamble and deviation match a Davis transmitter.

## Limitations

- Each transmission blocks reception for about 10 ms.
- Timing and frequency correction need at least one real transmitter to be received. Without one, timing stays at `p` and the frequency at the table.
- The console or Envoy may need minutes to acquire, depending on the ID (see above).
- The IN, JP, R2, RU and TW bands haven't been tested on air.
