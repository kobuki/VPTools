VPTools
-------

VPTools is a Davis(TM) weather station compatible transceiver library. It is a work in progress. It can receive from multiple RF transmitters (ISS, ATK, etc.) It tries to serve as a basis for similar hobby projects. Example sketches provide simple, parseable and RAW packet output among others.

* The ISSRx sketch is a receiver with a simple interface configurable in source code.
* The AnemometerTX sketch is a simple anemometer transmitter for Davis anemometers. It's a work in progress and needs optimisations for low power operation, but can serve as a base for doing so. It can easily be adapted to emulate any Davis transmitter.
* The WxReceiver sketch is a receiver compatible with the [WeeWx driver](https://github.com/matthewwall/weewx-meteostick) written by Matthew Wall et al for the Meteostick.

Originally based on [code by DeKay](https://github.com/dekay/DavisRFM69) - for technical details on packet formats, etc. see his (now outdated) [wiki](https://github.com/dekay/DavisRFM69/wiki) and the source code of this repo.
