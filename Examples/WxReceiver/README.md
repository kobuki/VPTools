WxReceiver.ino
--------------

This sketch is a receiver compatible with the WeeWx driver written by Matthew Wall et al for the Meteostick: https://github.com/matthewwall/weewx-meteostick

It emulates a subset of the publicly documented commands for the stick. It only supports 8 and 10-byte raw output, which the WeeWx driver currently uses as its main and only fully developed modus operandi. Thus it's not really meant as a replacement, but a cheap alternative to other available Davis Vantage device receivers for anyone wishing to use WeeWx.

The sketch requires either a LowPowerLab Weathershield (when used on a Moteino) or any similar Arduino-compatible device equipped with:
* a Hope RF RFM69 series 868/915 MHz transceiver for the proper band for the country
* a SiLabs Si7021 T/H sensor combined with a Bosch Sensortec BMP180 barometric pressure sensor (WeatherShield)
* a Bosch Sensortec BME280 T/H/P sensor (or WeatherShield R2)
* a Bosch Sensortec BMP280 P/T sensor

Or it can be easily adapted for any suitable sensor. They're optional, and can be emulated or code simply removed if observing internal climate conditions is not needed.
