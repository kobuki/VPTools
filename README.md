VPTools
-------

VPTools is a Davis(TM) weather station compatible transceiver library. It is a work in progress.

I decided to create a new, separate repo instead of forking the original. I originally only planned to contribute to the original repo by DeKay which was created with a full-fledged Davis console emulator in mind. I still wish to contribute wherever I can, but I needed a faster moving reposiotory where I can freely try my stuff and share with a few interested folks at WXForums.

Differences in the receiver code from the original repo include:

- simple, parseable output for all packet types
- multiple station receiver code is stable and functional, but work in progress

The ISSRx sketch is the receiver.

The AnemometerTX sketch is a simple anemometer transmitter for Davis anemometers. It's also a work in progress and needs optimisations for low power operation, but can serve as a base for doing so. It can easily be adapted for any type of transmitter a Davis console or Envoy, etc. is capable of receiving.

I'm going to update this description when I see it appropriate (mostly when something mentionable is committed).

Receiver code is originally based on https://github.com/dekay/DavisRFM69 by DeKay. I left the credits in all files. For (mostly) up to date technical details see his Wiki: https://github.com/dekay/DavisRFM69/wiki
