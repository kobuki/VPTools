WxReceiver.ino
--------------

This sketch is a receiver compatible with Matthew Wall's Meteostick driver for WeeWx: https://github.com/matthewwall/weewx-meteostick

It emulates a subset of the publicly documented commands for the stick. It only supports 8 and 10-byte raw output, which the WeeWx driver currently uses as its main and only fully developed modus operandi. Thus it's not really meant as a replacement, but a cheap alternative to any other available device having a WeeWx driver on the market today. It also requires moderate electronic skills to build and properly house the receiver. Output is simply provided on the serial port with 115200/8N1 as usual.

The sketch requires either a Moteino Weathershield (when used on a Moteino) or any similar Arduino-compatible device equipped with:
* a Hope RF RFM69 series transceiver for the proper band for the country
* a SiLabs Si7021 T/H sensor
* a Bosch Sensortec BMP180 barometric pressure sensor

It can be easily adapted for any suitable sensor. They're optional, and can be emulated or code simply removed if one doesn't need internal climate conditions.
