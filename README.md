# VPTools

VPTools is a Davis(TM) weather station compatible transceiver library. It can receive from multiple RF transmitters (ISS, ATK, etc.) It tries to serve as a basis for similar hobby projects. Example sketches provide simple, parseable and RAW packet output among others. The receiver library is extensively tested on the EU and US bands (as of Nov. 2026). Other bands (AU, NZ) will probably work but not tested by the author, lacking equipment on those bands.

## Examples

### WxReceiver

The WxReceiver sketch is a receiver meant to be used with the custom driver, [weewx-meteoRX](https://github.com/kobuki/weewx-meteoRX), which was originally based on the [WeeWx driver](https://github.com/matthewwall/weewx-meteostick) written by Matthew Wall et al for the Meteostick.

**NOTE:** It should still be compatible with the original driver, but compatibility is not verified any more for new WxReceiver versions.

### RXTX

RXTX receives several Davis transmitters and retransmits their data mixed into a single VP2 ISS packet stream on a transmitter ID of its own. The older Vantage consoles, the Envoy and most older software that supports Davis stations take each sensor from a fixed transmitter type. Only one ISS is allowed, so data from separate transmitters can't always be combined there. This sketch aims to help with this.

### ISSRx

The ISSRx sketch is a receiver with a simple interface configurable in source code. _WIP_

### AnemometerTX

The AnemometerTX sketch is a simple anemometer transmitter for Davis anemometers. It's a work in progress and needs optimisations for low power operation, but can serve as basis for doing so. It can easily be adapted to emulate any Davis transmitter. _WIP_

---

### Notes

- Some parts were originally based on [code by DeKay](https://github.com/dekay/DavisRFM69) - for technical details on packet formats, etc. see his (now outdated) [wiki](https://github.com/dekay/DavisRFM69/wiki) and the source code of this repo.
- From October 2026 onward, Claude Code was used to assist in coding and the rigorous testing required to bring these solutions alive.
