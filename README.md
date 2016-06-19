VPTools
-------

VPTools is a Davis(TM) weather station compatible transceiver library. It is a work in progress. It can receive from multiple RF transmitters (ISS, ATK, etc.) and provides a simple, parseable and a RAW output. It also tries to serve as a basis for similar hobby projects.

The ISSRx sketch is the receiver with a simple interface configurable in source code.

The AnemometerTX sketch is a simple anemometer transmitter for Davis anemometers. It's a work in progress and needs optimisations for low power operation, but can serve as a base for doing so. It can easily be adapted to emulate any Davis transmitter.

Original receiver code by DeKay: https://github.com/dekay/DavisRFM69

For technical details on packet formats, etc. see his (somewhat outdated) wiki: https://github.com/dekay/DavisRFM69/wiki
