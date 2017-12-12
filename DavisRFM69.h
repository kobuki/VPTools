// Driver definition for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from a Davis Instrument
// wireless Integrated Sensor Suite (ISS)
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
//
// As I consider this to be a derived work from the RFM69W library from LowPowerLab,
// it is released under the same Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/3.0/
//
// In accordance with the CC-BY-SA, many modifications by GitHub user "kobuki".

#ifndef DAVISRFM69_h
#define DAVISRFM69_h

#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater

#include "PacketFifo.h"

#define DAVIS_PACKET_LEN     10 // ISS has fixed packet lengths of eight bytes, including CRC and trailing repeater info
#define SPI_CS               SS // SS is the SPI slave select pin, for instance D10 on atmega328
// #define RF69_IRQ_PIN          2 // INT0 on AVRs should be connected to DIO0 (ex on Atmega328 it's D2)
// INT0 on AVRs should be connected to RFM69's DIO0 (ex on ATmega328 it's D2, on ATmega644/1284 it's D2)
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          0
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__)
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          2
#elif defined(__AVR_ATmega32U4__)
  #define RF69_IRQ_PIN          3
  #define RF69_IRQ_NUM          0
#else
  #define RF69_IRQ_PIN          2
  #define RF69_IRQ_NUM          0
#endif

#define CSMA_LIMIT          -90 // upper RX signal sensitivity threshold in dBm for carrier sense access
#define RF69_MODE_SLEEP       0 // XTAL OFF
#define RF69_MODE_STANDBY     1 // XTAL ON
#define RF69_MODE_SYNTH       2 // PLL ON
#define RF69_MODE_RX          3 // RX MODE
#define RF69_MODE_TX          4 // TX MODE

#define RF69_DAVIS_BW_NARROW  1
#define RF69_DAVIS_BW_WIDE    2

#define null                  0
#define COURSE_TEMP_COEF    -90 // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_FSTEP 61.03515625  // == FXOSC/2^19 = 32mhz/2^19 (p13 in DS)

#define RESYNC_THRESHOLD 50       // max. number of lost packets from a station before rediscovery
#define LATE_PACKET_THRESH 5000   // packet is considered missing after this many micros
#define POST_RX_WAIT 2000         // RX "settle" delay
#define MAX_STATIONS 8            // max. stations this code is able to handle

#define DISCOVERY_MINGAP 70000L
#define DISCOVERY_GUARD  60000L
#define DISCOVERY_STEP   150000000L
#define SPI_OP_TIMEOUT   15000L

// Station data structure for managing radio reception
typedef struct __attribute__((packed)) Station {
  byte id;                  // station ID (set with the DIP switch on original equipment)
                            // set it ONE LESS than advertised station id, eg. 0 for station 1 (default) etc.
  byte type;                // STYPE_XXX station type, eg. ISS, standalone anemometer transmitter, etc.
  bool active;              // true when the station is actively listened to but ignored
  byte repeaterId;          // repeater id when packet is coming via a repeater, otherwise 0
                            // repeater IDs A..H are stored as 0x8..0xf here
  byte channel;             // rx channel the next packet of the station is expected on
  uint32_t lastRx;          // last time a packet is seen or should have been seen when missed
  uint32_t lastSeen;        // last factual reception time
  uint32_t interval;        // packet transmit interval for the station: (41 + id) / 16 * 1M microsecs
  uint32_t numResyncs;      // number of times discovery of this station started because of packet loss
  uint32_t packets;         // total number of received packets after (re)restart
  uint32_t missedPackets;   // total number of misssed packets after (re)restart
  byte lostPackets;         // missed packets since a packet was last seen from this station
};

class DavisRFM69 {
  public:
    static volatile byte DATA[DAVIS_PACKET_LEN];  // recv/xmit buf, including header, CRC, and RSSI value
    static volatile byte _mode; // should be protected?
    static volatile bool _packetReceived;
    static volatile byte CHANNEL;
    static volatile int RSSI;
    static volatile int16_t FEI;
    static volatile byte band;

    static volatile uint32_t packets;
    static volatile uint32_t lostPackets;
    static volatile uint32_t numResyncs;
    static volatile uint32_t lostStations;
    static volatile byte stationsFound;
    static volatile byte curStation;
    static volatile byte numStations;
    static volatile byte discChannel;
    static volatile uint32_t lastDiscStep;
    static volatile int16_t freqCorr;
    static volatile uint32_t lastTx;
    static volatile uint32_t txDelay;
    static volatile uint32_t realTxDelay;
    static volatile uint32_t timeBase;
    static volatile byte txId;
    static volatile byte txChannel;
    static volatile byte rxRssi;
    static volatile bool repeaterPT;

    static PacketFifo fifo;
    static Station *stations;

    DavisRFM69(byte slaveSelectPin=SPI_CS, byte interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, byte interruptNum=RF69_IRQ_NUM) {
      _slaveSelectPin = slaveSelectPin;
      _interruptPin = interruptPin;
      _interruptNum = interruptNum;
      _mode = RF69_MODE_STANDBY;
      _packetReceived = false;
      _powerLevel = 31;
      _isRFM69HW = isRFM69HW;
    }

    void setChannel(byte channel);
    static uint16_t crc16_ccitt(volatile byte *buf, byte len, uint16_t initCrc = 0);

    void initialize(byte freqBand);
    bool canSend();
    void send(const byte* buffer, byte channel);
    bool receiveDone();
    void setFrequency(uint32_t FRF);
    void setCS(byte newSPISlaveSelect);
    int readRSSI(bool forceTrigger = false);
    void setHighPower(bool onOFF = true); // have to call it after initialize for RFM69HW
    void setPowerLevel(byte level); // reduce/increase transmit power level
    void sleep();
    byte readTemperature(byte calFactor=0); // get CMOS temperature (8bit)
    void rcCalibration(); // calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]

    // allow hacking registers by making these public
    byte readReg(byte addr);
    void writeReg(byte addr, byte val);
    void readAllRegs();
    void setBand(byte newBand);
    void setBandwidth(byte bw);
    byte getBandTabLength();

    byte nextChannel(byte channel);
    int findStation(byte id);
    void handleRadioInt();
    void initStations();
    void nextStation();

    static void handleTimerInt();
    static void handleTxInt();
    static void setStations(Station *_stations, byte n);
    void stopReceiver();
    void setRssiThreshold(int rssiThreshold);
    void setRssiThresholdRaw(int rssiThresholdRaw);
    void setFreqCorr(int16_t value);
    void setTimeBase(uint32_t value);
    void setRepeaterPT(bool value = false);
    void enableTx(void (*function)(byte* buffer), byte ID);
    void disableTx();

  protected:
    static volatile bool txMode;
    void (*txCallback)(byte* buffer);

    void virtual interruptHandler();
    void sendFrame(const byte* buffer);
    byte reverseBits(byte b);

    static void isr0();

    static DavisRFM69* selfPointer;
    byte _slaveSelectPin;
    byte _interruptPin;
    byte _interruptNum;
    byte _powerLevel;
    bool _isRFM69HW;

    void receiveBegin();
    void setMode(byte mode);
    void setHighPowerRegs(bool onOff);
    void setTxMode(bool mode);
    void select();
    void unselect();
};

// FRF_MSB, FRF_MID, and FRF_LSB for the 51 North American, Australian, New Zealand & 5 European channels
// used by Davis in frequency hopping

#define FREQ_TABLE_LENGTH_US 51
#define FREQ_TABLE_LENGTH_AU 51
#define FREQ_TABLE_LENGTH_EU 5
#define FREQ_TABLE_LENGTH_NZ 51

#define FREQ_BAND_US 0
#define FREQ_BAND_AU 1
#define FREQ_BAND_EU 2
#define FREQ_BAND_NZ 3

typedef uint8_t FRF_ITEM[3];

static const __attribute__((progmem)) FRF_ITEM FRF_US[FREQ_TABLE_LENGTH_US] =
{
  {0xE3, 0xDA, 0x7C},
  {0xE1, 0x98, 0x71},
  {0xE3, 0xFA, 0x92},
  {0xE6, 0xBD, 0x01},
  {0xE4, 0xBB, 0x4D},
  {0xE2, 0x99, 0x56},
  {0xE7, 0x7D, 0xBC},
  {0xE5, 0x9C, 0x0E},
  {0xE3, 0x39, 0xE6},
  {0xE6, 0x1C, 0x81},
  {0xE4, 0x5A, 0xE8},
  {0xE1, 0xF8, 0xD6},
  {0xE5, 0x3B, 0xBF},
  {0xE7, 0x1D, 0x5F},
  {0xE3, 0x9A, 0x3C},
  {0xE2, 0x39, 0x00},
  {0xE4, 0xFB, 0x77},
  {0xE6, 0x5C, 0xB2},
  {0xE2, 0xD9, 0x90},
  {0xE7, 0xBD, 0xEE},
  {0xE4, 0x3A, 0xD2},
  {0xE1, 0xD8, 0xAA},
  {0xE5, 0x5B, 0xCD},
  {0xE6, 0xDD, 0x34},
  {0xE3, 0x5A, 0x0A},
  {0xE7, 0x9D, 0xD9},
  {0xE2, 0x79, 0x41},
  {0xE4, 0x9B, 0x28},
  {0xE5, 0xDC, 0x40},
  {0xE7, 0x3D, 0x74},
  {0xE1, 0xB8, 0x9C},
  {0xE3, 0xBA, 0x60},
  {0xE6, 0x7C, 0xC8},
  {0xE4, 0xDB, 0x62},
  {0xE2, 0xB9, 0x7A},
  {0xE5, 0x7B, 0xE2},
  {0xE7, 0xDE, 0x12},
  {0xE6, 0x3C, 0x9D},
  {0xE3, 0x19, 0xC9},
  {0xE4, 0x1A, 0xB6},
  {0xE5, 0xBC, 0x2B},
  {0xE2, 0x18, 0xEB},
  {0xE6, 0xFD, 0x42},
  {0xE5, 0x1B, 0xA3},
  {0xE3, 0x7A, 0x2E},
  {0xE5, 0xFC, 0x64},
  {0xE2, 0x59, 0x16},
  {0xE6, 0x9C, 0xEC},
  {0xE2, 0xF9, 0xAC},
  {0xE4, 0x7B, 0x0C},
  {0xE7, 0x5D, 0x98}
};

static const __attribute__((progmem)) FRF_ITEM FRF_AU[FREQ_TABLE_LENGTH_AU] =
{
   {0xE5, 0x84, 0xDD},
   {0xE6, 0x43, 0x43},
   {0xE7, 0x1F, 0xCE},
   {0xE6, 0x7F, 0x7C},
   {0xE5, 0xD5, 0x0E},
   {0xE7, 0x5B, 0xF7},
   {0xE6, 0xC5, 0x81},
   {0xE6, 0x07, 0x2B},
   {0xE6, 0xED, 0xA1},
   {0xE6, 0x61, 0x58},
   {0xE5, 0xA3, 0x02},
   {0xE6, 0xA7, 0x8D},
   {0xE7, 0x3D, 0xB2},
   {0xE6, 0x25, 0x3F},
   {0xE5, 0xB7, 0x0A},
   {0xE6, 0x93, 0x85},
   {0xE7, 0x01, 0xDB},
   {0xE5, 0xE9, 0x26},
   {0xE7, 0x70, 0x00},
   {0xE6, 0x57, 0x6C},
   {0xE5, 0x98, 0xF5},
   {0xE6, 0xB1, 0x99},
   {0xE7, 0x29, 0xDB},
   {0xE6, 0x11, 0x37},
   {0xE7, 0x65, 0xE3},
   {0xE5, 0xCB, 0x33},
   {0xE6, 0x75, 0x60},
   {0xE6, 0xD9, 0xA9},
   {0xE7, 0x47, 0xDF},
   {0xE5, 0x8E, 0xF9},
   {0xE6, 0x2F, 0x4B},
   {0xE7, 0x0B, 0xB6},
   {0xE6, 0x89, 0x68},
   {0xE5, 0xDF, 0x2B},
   {0xE6, 0xBB, 0xA5},
   {0xE7, 0x79, 0xFB},
   {0xE6, 0xF7, 0xAE},
   {0xE5, 0xFD, 0x2F},
   {0xE6, 0x4D, 0x4F},
   {0xE6, 0xCF, 0x8D},
   {0xE5, 0xAD, 0x0E},
   {0xE7, 0x33, 0xD7},
   {0xE6, 0x9D, 0x91},
   {0xE6, 0x1B, 0x33},
   {0xE6, 0xE3, 0xA5},
   {0xE5, 0xC1, 0x16},
   {0xE7, 0x15, 0xC2},
   {0xE5, 0xF3, 0x33},
   {0xE6, 0x6B, 0x64},
   {0xE7, 0x51, 0xDB},
   {0xE6, 0x39, 0x58}
};

static const __attribute__((progmem)) FRF_ITEM FRF_EU[FREQ_TABLE_LENGTH_EU] =
{
  {0xD9, 0x04, 0x45},
  {0xD9, 0x13, 0x04},
  {0xD9, 0x21, 0xC2},
  {0xD9, 0x0B, 0xA4},
  {0xD9, 0x1A, 0x63}
};

static const __attribute__((progmem)) FRF_ITEM FRF_NZ[FREQ_TABLE_LENGTH_NZ] =
{
  {0xE6, 0x45, 0x0E},
  {0xE7, 0x43, 0xC7},
  {0xE6, 0xF4, 0xAC},
  {0xE6, 0x9C, 0xEE},
  {0xE7, 0xA4, 0x7B},
  {0xE6, 0xC0, 0x31},
  {0xE7, 0xD0, 0x52},
  {0xE7, 0x20, 0x93},
  {0xE6, 0x68, 0x31},
  {0xE7, 0x67, 0x0A},
  {0xE6, 0xDA, 0x5E},
  {0xE7, 0xE1, 0xEC},
  {0xE7, 0x8A, 0x0C},
  {0xE6, 0x82, 0xA0},
  {0xE7, 0x0F, 0x1B},
  {0xE7, 0xBE, 0xE9},
  {0xE6, 0xB7, 0x3B},
  {0xE7, 0x4C, 0x6A},
  {0xE7, 0xFC, 0x5A},
  {0xE6, 0x4D, 0xF4},
  {0xE7, 0x92, 0xD1},
  {0xE6, 0xEB, 0xF8},
  {0xE6, 0x94, 0x39},
  {0xE7, 0xEA, 0xC1},
  {0xE7, 0x29, 0x79},
  {0xE6, 0x5F, 0x7D},
  {0xE7, 0x5E, 0x35},
  {0xE6, 0xC8, 0xC5},
  {0xE7, 0xB6, 0x25},
  {0xE6, 0xA5, 0xB2},
  {0xE6, 0xFD, 0x81},
  {0xE7, 0x6F, 0xCF},
  {0xE6, 0x79, 0xCB},
  {0xE7, 0x9B, 0xB6},
  {0xE7, 0x32, 0x2D},
  {0xE7, 0xC7, 0x7D},
  {0xE6, 0x8B, 0x54},
  {0xE7, 0x81, 0x37},
  {0xE6, 0xD1, 0x89},
  {0xE7, 0x55, 0x60},
  {0xE7, 0xD9, 0x17},
  {0xE6, 0x56, 0xA8},
  {0xE7, 0x06, 0x35},
  {0xE7, 0xAD, 0x2F},
  {0xE6, 0xAE, 0x77},
  {0xE7, 0x3B, 0x12},
  {0xE7, 0xF3, 0x85},
  {0xE6, 0x71, 0x06},
  {0xE7, 0x17, 0xCF},
  {0xE6, 0xE3, 0x12},
  {0xE7, 0x78, 0xA4}
};

static const FRF_ITEM *bandTab[4] = {
  FRF_US,
  FRF_AU,
  FRF_EU,
  FRF_NZ
};

static const uint8_t bandTabLengths[4] = {
  FREQ_TABLE_LENGTH_US,
  FREQ_TABLE_LENGTH_AU,
  FREQ_TABLE_LENGTH_EU,
  FREQ_TABLE_LENGTH_NZ
};

// added these here because upstream removed them
#define REG_TESTAFC        0x71

#define RF_FDEVMSB_9900    0x00
#define RF_FDEVLSB_9900    0xa1

#define RF_FDEVMSB_10000   0x00
#define RF_FDEVLSB_10000   0xa4

#define RF_AFCLOWBETA_ON   0x20
#define RF_AFCLOWBETA_OFF  0x00 // Default

// Davis VP2 standalone station types
#define STYPE_ISS         0x0  // ISS
#define STYPE_TEMP_ONLY   0x1  // Temperature Only Station
#define STYPE_HUM_ONLY    0x2  // Humidity Only Station
#define STYPE_TEMP_HUM    0x3  // Temperature/Humidity Station
#define STYPE_WLESS_ANEMO 0x4  // Wireless Anemometer Station
#define STYPE_RAIN        0x5  // Rain Station
#define STYPE_LEAF        0x6  // Leaf Station
#define STYPE_SOIL        0x7  // Soil Station
#define STYPE_SOIL_LEAF   0x8  // Soil/Leaf Station
#define STYPE_SENSORLINK  0x9  // SensorLink Station (not supported for the VP2)
#define STYPE_OFF         0xA  // No station – OFF
#define STYPE_VUE         0x10 // pseudo station type for the Vue ISS
                               // since the Vue also has a type of 0x0

// aliases
#define STYPE_ATK         STYPE_WLESS_ANEMO

// Below are known, publicly documented packet types for the VP2 and the Vue.

// VP2 packet types
#define VP2P_UV           0x4 // UV index
#define VP2P_RAINSECS     0x5 // seconds between rain bucket tips
#define VP2P_SOLAR        0x6 // solar irradiation
#define VP2P_TEMP         0x8 // outside temperature
#define VP2P_WINDGUST     0x9 // 10-minute wind gust
#define VP2P_HUMIDITY     0xA // outside humidity
#define VP2P_UNKNOWN1     0xC // unknown packet type:
                              //   packet[3] and pkt[5] are always zero;
                              //   packet[4] has values 0-3 (ATK) or 5 (temp/hum)
#define VP2P_RAIN         0xE // rain bucket tips counter
#define VP2P_SOIL_LEAF    0xF // soil/leaf station

// Vue packet types
#define VUEP_VCAP         0x2 // supercap voltage
#define VUEP_UNKNOWN1     0x3 // unknown Vue packet type
#define VUEP_VSOLAR       0x7 // solar panel voltage

#endif  // DAVISRFM_h
