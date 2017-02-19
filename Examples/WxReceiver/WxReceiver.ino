#include <Arduino.h>

#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>
#include <avr/wdt.h>

#include "DavisRFM69.h"
#include "PacketFifo.h"

//#define SENSOR_TYPE_EMULATED
#define SENSOR_TYPE_BME280
//#define SENSOR_TYPE_SI7021_BMP180

#ifdef SENSOR_TYPE_SI7021_BMP180
#include <SFE_BMP180.h>
#include <SI7021.h>
#else
#include <SparkFunBME280.h>
#endif

#define NAME_VERSION F("WxReceiver v2017021901")

#define LED 9 // Moteinos have LEDs on D9
#define SERIAL_BAUD 115200
#define BARO_DELAY 60 * 1000000 // 60 secs

#define S_OK true
#define S_ERR false

#define SERBUF_LEN 8

DavisRFM69 radio;

#ifdef SENSOR_TYPE_SI7021_BMP180
SI7021 si7021;
SFE_BMP180 bmp180;
#else
BME280 bme280;
#endif

bool localEcho = false;
bool running = false;

byte cfgNumStations = 1;
byte cfgStationsByte = 1;
byte cfgBandRaw = 1;
byte cfgBand = FREQ_BAND_EU;
byte cfgRfSensitivity = 190;
byte cfgOutput = 0;

// id, type, active
Station stations[8] = {
  { 0, STYPE_ISS, true },
};

void setup() {
  Serial.begin(SERIAL_BAUD);

#ifndef SENSOR_TYPE_EMULATED

#ifdef SENSOR_TYPE_SI7021_BMP180

  si7021.begin();
  si7021.setHeater(0);
  bmp180.begin();

#else // BME280

  bme280.settings.commInterface = I2C_MODE;
  bme280.settings.I2CAddress = 0x76; // can be 0x77

  bme280.settings.runMode = 3; // normal mode
  bme280.settings.tStandby = 0; // 0.5ms
  bme280.settings.filter = 0; // filter off
  bme280.settings.tempOverSample = 1; // x1
  bme280.settings.pressOverSample = 1; // x1
  bme280.settings.humidOverSample = 1; // x1
  delay(10); // safe sensor init

#endif // SENSOR_TYPE_SI7021_BMP180

#endif // SENSOR_TYPE_EMULATED

  blinkLed(LED, 750);
  printBanner();
}

void loop() {
  if (running) {
    if (radio.fifo.hasElements()) {
      blinkLed(LED, 25);
      printIPacket(radio.fifo.dequeue());
    }
    printBPacket();
  }
  processSerial();
}

void startReceiver() {
  radio.initialize(cfgBand);
  radio.setStations(stations, cfgNumStations);
  radio.setRssiThresholdRaw(cfgRfSensitivity);
  radio.setBandwidth(RF69_DAVIS_BW_NARROW);
  running = true;
}

void stopReceiver() {
  running = false;
  radio.stopReceiver();
  radio.fifo.flush();
}

void printBanner()
{
  Serial.print(F("# "));
  Serial.print(NAME_VERSION);
  Serial.println(F(" ready to accept commands"));
  Serial.println('?');
}

void printStatus(bool st, char *info = NULL)
{
  Serial.print(st ? F("# OK") : F("# ERR"));
  if (info != NULL) {
    Serial.print(' ');
    Serial.print(info);
  }
  Serial.println();
}

void printStatusF(bool st, const __FlashStringHelper *info = NULL)
{
  Serial.print(st ? F("# OK") : F("# ERR"));
  if (info != NULL) {
    Serial.print(' ');
    Serial.print(info);
  }
  Serial.println();
}

void processSerial()
{
  int c;
  static int i;
  static char s[SERBUF_LEN];
  
  if (Serial.available()) {
    c = Serial.read();

    if (c == '\r' || c == '\n') {

      // ack empty command with OK
      if (i == 0) {
        printStatus(S_OK);
        return;
      }
      
      s[i] = 0;
      i = 0;

      if (localEcho) Serial.println();

      switch(s[0]) {
        case 'r':
          cmdReset();
          break;
    
        case 't':
          cmdTransmitter(s + 1);
          break;
    
        case 'f':
          cmdFilter(s + 1);
          break;
          
        case 'o':
          cmdOutput(s + 1);
          break;
    
        case 'm':
          cmdMode(s + 1);
          break;

        case 'x':
          cmdRFSensitivity(s + 1);
          break;

        case '?':
          cmdShowDefaults();
          break;

        default:
          printStatusF(S_ERR, F("bad command"));
      }

    } else {
      if (localEcho) Serial.print((char)c);
      if (i < SERBUF_LEN - 1) {
        s[i++] = c;
      } else {
        s[0] = 0;
        i = 0;
        Serial.println();
        printStatusF(S_ERR, F("bad command"));
      }
    }

  }

}

void softReset()
{
  wdt_enable(WDTO_15MS);
  for(;;) {}
}

void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();
  return;
}

uint32_t t = 0, lastBaro = 0, lastReceived = 0, lastMissed = 0;

// B 25470 311109 309 99544 46 7
void printBPacket() {
  t = micros();

  if (t - lastBaro > BARO_DELAY) {

    blinkLed(LED, 250);
    lastBaro = t;

#ifdef SENSOR_TYPE_EMULATED

    Serial.print(F("B 0 0 250 10132 "));

#else

#ifdef SENSOR_TYPE_SI7021_BMP180

    double P = 0, T = 0;
    char st;
    int tempC100 = si7021.getCelsiusHundredths();
    int humidity = si7021.getHumidityPercent();
    
    st = bmp180.startTemperature();
    if (st != 0) {
      delay(st);
      st = bmp180.getTemperature(T);
      if (st != 0) {
        st = bmp180.startPressure(3);
        if (st != 0) {
          delay(st);
          st = bmp180.getPressure(P, T);
        }
      }
    }

    Serial.print(F("B 0 0 "));
    Serial.print(tempC100 / 10);
    Serial.print(' ');
    Serial.print((uint32_t)(P * 100));
    Serial.print(' ');

#else // BME280

    bme280.begin();
    double tempC = bme280.readTempC();
    int humidity = round(bme280.readFloatHumidity());
    double P = bme280.readFloatPressure();
    bme280.writeRegister(BME280_CTRL_MEAS_REG, 0x00); 

    Serial.print(F("B 0 0 "));
    Serial.print(round(tempC * 10.0));
    Serial.print(' ');
    Serial.print(round(P));
    Serial.print(' ');

#endif // SENSOR_TYPE_SI7021_BMP180

#endif // SENSOR_TYPE_EMULATED

    Serial.print(radio.packets - lastReceived + radio.lostPackets - lastMissed);
    Serial.print(' ');
    Serial.print(radio.lostPackets - lastMissed);
    Serial.print(' ');

#ifdef SENSOR_TYPE_EMULATED
    Serial.println(45);
#else
    Serial.println(humidity);
#endif

    lastReceived = radio.packets;
    lastMissed = radio.lostPackets;
  }

}

void cmdReset()
{
  printStatusF(S_OK, F("resetting"));
  stopReceiver();
  softReset();
}

void cmdTransmitter(char *s)
{
  int b = atoi(s);
  if (b == 0 || b > 255) {
    printStatus(S_ERR);
    return;
  }
  cfgStationsByte = b;
  bool oldr = running;
  if (oldr) stopReceiver();
  Serial.print(F("# OK listening to "));
  byte si = 0;
  for (int i = 1; i <= 8; i++) {
    if (b & 1) {
      Serial.print(i);
      Serial.print(' ');
      stations[si].id = i - 1;
      stations[si].type = STYPE_ISS; // whatever is good here
      stations[si].active = true;
      si++;
    }
    b >>= 1;
  }
  cfgNumStations = si;
  Serial.println();
  if (oldr) startReceiver();
}

void cmdFilter(char *s)
{
  if (s[0] == '1') {
    printStatusF(S_OK, F("filter on"));
  } else {
    printStatusF(S_ERR, F("unimplemented"));
  }
}

void cmdOutput(char *s)
{
  switch (s[0]) {
    case '0':
      cfgOutput = 0;
      printStatusF(S_OK, F("output raw 8B"));
      break;
    case '3':
      cfgOutput = 3;
      printStatusF(S_OK, F("output raw 10B"));
      break;
    default:
      printStatusF(S_ERR, F("unimplemented"));
      return;
  }
}

void cmdMode(char *s)
{
  switch (s[0]) {
    case '0':
      cfgBand = FREQ_BAND_US;
      printStatusF(S_OK, F("band US"));
      break;
    case '1':
      cfgBand = FREQ_BAND_EU;
      printStatusF(S_OK, F("band EU"));
      break;
    case '2':
      cfgBand = FREQ_BAND_AU;
      printStatusF(S_OK, F("band AU"));
      break;
    case '5':
      cfgBand = FREQ_BAND_NZ;
      printStatusF(S_OK, F("band NZ"));
      break;
    default:
      printStatusF(S_ERR);
      return;
  }
  cfgBandRaw = s[0] - '0';
  startReceiver();
}

void cmdRFSensitivity(char *s)
{
  byte sens = atoi(s);
  if (sens > 0 && sens <= 255) {
    cfgRfSensitivity = sens;
    radio.setRssiThresholdRaw(sens);
    Serial.print(F("# OK sensitivity "));
    Serial.println(sens);
  } else {
    printStatusF(S_ERR);
  }
}

void cmdShowDefaults()
{
  Serial.print(F("# OK "));
  Serial.print('t');
  Serial.print(cfgStationsByte);
  Serial.print(F(" f1 o"));
  Serial.print(cfgOutput);
  Serial.print(F(" m"));
  Serial.print(cfgBandRaw);
  Serial.print(F(" x"));
  Serial.print(cfgRfSensitivity);
  Serial.println(running ? F(" running") : F(" stopped"));
}

// I 104 61 0 0 FF C3 0 96 DC  -64 2624972 242
void printIPacket(RadioData* rd) {
  int val;
  byte* packet = rd->packet;

  Serial.print(F("I "));
  
  Serial.print('1');
  if (rd->channel < 10) Serial.print('0');
  Serial.print(rd->channel);
  Serial.print(' ');

  printHex(packet, cfgOutput == 0 ? 8 : 10);
  Serial.print(F("  "));
  Serial.print(-rd->rssi);
  Serial.print(' ');
  Serial.print(rd->delta);
  Serial.print(' ');
  Serial.println(round(rd->fei * RF69_FSTEP / 1000));

// Other possible informational values:
//  Serial.print(packet[0] & 0x7); // station
//  Serial.print(radio.packets); // total packets
//  Serial.print(radio.lostPackets); // total lost packets
//  Serial.print((float)(radio.packets * 100.0 / (radio.packets + radio.lostPackets))); // total good/(good+bad) packet ratio
//  Serial.print((char*)(packet[0] & 0x8 ? "err" : "ok")); // ISS battery status
}

void printHex(volatile byte* packet, byte len) {
  for (byte i = 0; i < len; i++) {
    if (!(packet[i] & 0xf0)) Serial.print('0');
    Serial.print(packet[i], HEX);
    if (i < len - 1) Serial.print(' ');
  }
}

void printFreeRam() {
  extern int __heap_start, *__brkval;
  int16_t v;
  Serial.print(F("free mem: "));
  Serial.println((int16_t) &v - (__brkval == 0 ? (int16_t) &__heap_start : (int16_t) __brkval));
}

void blinkLed(byte pin, int blinkDelay)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(blinkDelay);
  digitalWrite(pin, LOW);
}

