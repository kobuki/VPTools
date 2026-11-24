AnemometerTX
------------

AnemometerTX reads a Davis anemometer and transmits it as a Davis transmitter, like the SIM board in an ISS or an anemometer transmitter kit.

The packets follow the VP2 ISS packet sequence and timing. Wind speed and direction go in every packet and the 10-minute gust in the gust packets. All other packet types carry Davis' "no sensor" values, so a console shows dashes for them. The radio only transmits and sleeps between packets.

## Hardware

Any ATmega328P board with an RFM69 module (a Moteino is suggested) for the band works. Set `h` to match the module: the W/CW modules use PA0, the HW/HCW modules use PA_BOOST. With the wrong setting the board transmits only leakage, which a console may not pick up.

| Pin | Connection |
|---|---|
| D3 (INT1) | Anemometer reed switch, the other side to GND. The internal pull-up is used. |
| A0 | Wind vane potentiometer wiper. |
| D5 | Wind vane potentiometer supply, the other end of the potentiometer to GND. Driven only while the vane is read. |
| D4 | Through an extra resistor to the potentiometer's D5 end, for detecting the potentiometer's dead zone. |
| D9 | LED (Moteino): blinks for each transmission. Set `LED` to -1 in the sketch for a board without one. |

In the Davis anemometer cable, black is the reed switch, red is ground, green is the vane wiper and yellow is the vane supply.

The board talks over the serial port at 115200 baud. Commands end with CR or LF.

## Commands and settings

Each setting takes effect at once and is echoed in the status line, but it is **not saved**. Settings persist only after the `e` command, which writes them to EEPROM. At the next start the saved settings are loaded. Anything changed since the last `e` is lost when the board resets or loses power. Without saved settings, the board starts with the defaults below.

| Command | Default | Meaning |
|---|---|---|
| `m<0..8>` | `m1` | Band: 0 US, 1 EU, 2 AU, 3 NZ, 4 JP, 5 IN, 6 R2, 7 RU, 8 TW. |
| `i<0..8>` | `i3` | Transmitter ID to send as. `i0` switches the transmitter off. |
| `c<0\|1>` | `c1` | Transmit timing: 0 = the board's own clock, corrected by `p`; 1 = calibrated against the RFM69's crystal, plus `p`. See below. |
| `p<ppm>` | `p0` | Transmit timing correction in ppm (integer). With `c1` it corrects the RFM69 crystal's own error. |
| `h<0\|1>` | `h1` | RFM69 module: 0 = W/CW (PA0, +13 dBm), 1 = HW/HCW (PA_BOOST, +20 dBm). |
| `g<0\|1>` | `g1` | On-air framing: 0 = legacy (32-bit lead, 48-bit preamble); 1 = as Davis transmitters (18-bit lead, 32-bit preamble, 10.4 kHz deviation). |
| `e` | | Save the settings to EEPROM. |
| `d` | | Load the defaults (not saved). |
| `r` | | Reset the board. |
| `?` | | Show the settings. |

The status line shows all settings, for example:

```
# OK m1 i3 c1 p0 h1 g1
```

## Example: anemometer on ID 2, US band

```
m0
i2
e
```

On the console or Envoy, set up ID 2 as the station that provides wind. If the console allows only one ISS and wind should be combined with another transmitter's data, RXTX can mix them into one ISS.

## Wind

- **Speed** is the mean over all rotations since the previous packet, measured 100 ms before each transmission, in whole mph (1 rotation per second = 2.25 mph). With no rotation for 2.25 s the speed is 0. While the cups slow down, the time since the last rotation counts as the current period, so the speed falls before the calm limit is reached.
- With the build option `-DWIND_GATE_US=2250000`, the speed is the number of rotations in a 2.25 s gate that ends before each transmission instead, as the ISS measures it.
- **Contact bounce:** a rotation is the first edge after the switch has rested open for 5 ms (`DEBOUNCE_US`). Bounce when the switch closes or opens doesn't count.
- **Speed correction:** the speed is sent raw. Consoles and receivers apply their direction-dependent speed correction themselves.
- **Direction** is the vane's raw 8-bit value, 0 in the potentiometer's dead zone.
- **Gust:** the gust packets carry the highest speed in the last 10 gust periods (a gust period is the 20-packet sequence, 51–60 s depending on the ID), its age in gust periods and its direction in 16 sectors, as an ISS sends them. Consoles take their 10-minute gust from these packets.

## Transmit timing

A console expects a transmitter to send every (40 + ID) / 16 seconds, 2.5625 s for ID 1 to 3.0 s for ID 8. The board's clock sets this interval.

- **`c0`:** the interval runs on the board's own clock, corrected by `p`. For boards with a quartz crystal, which is accurate enough as it is.
- **`c1`:** the board measures its own clock against the RFM69's crystal and corrects the interval with the result. This is for boards with a ceramic resonator, like the Moteino, which is too far off and drifts with temperature. A calibration runs at startup and then every 30 min (`CAL_PERIOD_S`): a burst of 4 measurements (`CAL_BURST`), whose median is applied. Each measurement uses two transmit gaps. Measurements more than 1000 ppm off count as failed. `p` then corrects for the RFM69 crystal's own error, if needed.

## Output

Transmitted packets are printed as `T` lines:

```
T 47 80 00 06 FF C1 00 19 BF FF FF 2562432 2562501 0.4 27 0 0.0
```

The fields are the channel, the 10 bytes sent, the transmit interval that just ended and the interval it was scheduled for (µs), the timing correction in use (ppm), the number of calibrations, the number of failed measurements, and 0.0 (for the same format as RXTX's).

With `c1`, each clock measurement is printed as a `K` line and each calibration as a `C` line:

```
K 2203700 70268 -7.9
C -5.1 4 -5.1
```

`K`: the long and the short measured time (µs) and the clock error it gives (ppm), or `fail`. `C`: the median applied (ppm) or `fail`, the number of measurements, and the timing correction now in use (ppm).

## Measured results

Tested with a Davis Envoy (VP2 firmware 3.15):

- **Reception:** the Envoy received 99.1–99.6 % of the packets on the EU and US bands, without resyncs.
- **Timing:** with `c1`, the transmit interval was within 5 ppm of nominal over a night on a board with a crystal, and within 2 ppm over 15 minutes on a Moteino with a ceramic resonator. The Envoy kept receiving with intervals from 1500 ppm short to about 400 ppm long.
- **Wind speed:** the sent speed matched the rotation rate exactly from 1 to 150 mph, also with contact bounce on either edge.
- **Gust:** the Envoy's 10-minute gust followed the gust packets.

## Limitations

- The wind vane input hasn't been tested yet.
- The direction is sent with 8 bits. Davis receivers decode 9 bits (the lowest one is bit 1 of byte 4), so half the possible resolution is unused.
- The battery-low bit is never set.
- The IN, JP, R2, RU and TW bands haven't been tested on air.
