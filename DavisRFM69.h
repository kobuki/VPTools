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

#define RF69_AFC_OFF          0     // fixed frequency table, no AFC
                                    // 1..RF69_AFC_MAX: software AFC, per station correction limited to +/- that many kHz
#define RF69_AFC_MAX          50
#define RF69_AFC_HW           255   // RFM69 auto AFC, can lock onto the flat carrier around Davis packets
#define RF69_FEI_INVALID      ((int16_t)0x8000)

// Davis packets start with ~18 bits of flat carrier on the '1' tone, then 32 bits preamble, 16 bits sync, 80 bits data.
// RSSI triggers early on the carrier, where FEI reads 0; FEI started 0.7-1.1 ms later reads the true offset.
#define AFC_FEI_DELAY         900       // RSSI trigger to FEI start (micros)
#define AFC_PKT_MIN           5000L     // RSSI trigger to PayloadReady when RSSI triggered on the packet's own carrier
#define AFC_PKT_MAX           8000L     // (7.45 ms nominal, but micros() loses up to ~1 ms around the FEI wait)
#define AFC_MAX_STEP          ((int16_t)(2000 / RF69_FSTEP))  // limit of a single FEI sample's pull once converged
// While discovering, an RSSI trigger without payload still measures the offset on the preamble (the packet itself
// was likely lost to that offset). Readings this much stronger than the RSSI threshold (raw, 0.5 dB units) are used
// right away, weaker ones only if two agree, so noise triggers don't detune the receiver. Readings below the weak
// margin are ignored: noise triggers sit right at the threshold and agree by chance often enough to matter.
#define AFC_DISC_MARGIN       16
#define AFC_DISC_WEAK         6
#define AFC_DISC_AGREE        ((int16_t)(1500 / RF69_FSTEP))

#define null                  0
#define COURSE_TEMP_COEF    -90     // puts the temperature reading in the ballpark, user can fine tune the returned value
#define RF69_FSTEP 61.03515625      // FXOSC/2^19 (see datasheet)

#define RESYNC_THRESHOLD 50         // max. number of lost packets from a station before rediscovery
#define LATE_PACKET_THRESH 5000L    // packet is considered missing after this delay (micros)
#define EARLY_PACKET_THRESH 20000L  // a tracked station's packet this much early, per interval since it was last seen, is foreign (micros)
#define TRAILER_MAX_ZERO_BITS 2     // a non-repeater packet's 0xFF 0xFF trailer may have this many bit errors
#define POST_RX_WAIT 2000L          // RX "settle" delay
#define MAX_STATIONS 8              // max. stations this code is able to handle

#define DISCOVERY_MINGAP 70000L     // time before expected transmission to keep discovery frequency on
#define DISCOVERY_GUARD  20000L     // time before expected transmission to switch on receiver
#define DISCOVERY_STEP   150000000L // time before skipping the discovery channel to avoid interference
#define SPI_OP_TIMEOUT   15000L     // SPI operation time guard

// Transmit timing follows the received stations: their intervals as this board's clock perceives them give the
// clock's error against theirs, and the transmitter runs its own interval with the same error. Units are 1/16 ppm.
#define TX_PPM_ONE        16          // 1 ppm
#define TX_PPM_MAX        (2000 * TX_PPM_ONE)  // samples beyond this are not station timing
#define TX_PPM_GATE       (50 * TX_PPM_ONE)    // once settled, samples further than this from the estimate are outliers
#define TX_PPM_FAST       16          // the first this many samples are averaged plainly
#define TX_PPM_SLOW       16          // after that each sample moves the estimate 1/TX_PPM_SLOW of the way
#define TX_PPM_MAX_SPAN   8           // longest gap between two packets of a station used as a sample (intervals)

// Station data structure for managing radio reception
typedef struct __attribute__((packed)) Station {
  byte id;                  // station ID (set with the DIP switch on original equipment)
                            // set it ONE LESS than advertised station id, eg. 0 for station 1 (default) etc.
  byte type;                // STYPE_XXX station type, eg. ISS, standalone anemometer transmitter, etc.
  bool active;              // true when the station is actively listened to but ignored
  byte channel;             // rx channel the next packet of the station is expected on
  uint32_t lastRx;          // last time a packet is seen or should have been seen when missed
  uint32_t lastSeen;        // last factual reception time
  uint32_t interval;        // packet transmit interval for the station: (41 + id) / 16 * 1M microsecs
  uint32_t numResyncs;      // number of times discovery of this station started because of packet loss
  uint32_t packets;         // total number of received packets after (re)restart
  uint32_t missedPackets;   // total number of misssed packets after (re)restart
  byte lostPackets;         // missed packets since a packet was last seen from this station
  int16_t afcCorr;          // software AFC: learned frequency correction in FSTEP units
  byte afcSamples;          // software AFC: number of FEI samples behind afcCorr (saturates)
};

// on-air framing of transmitted packets
#define TX_FRAME_LEGACY 0  // 32 bits '1' lead, 48 bits preamble, 8 bits '1' trailer, 10 kHz deviation
#define TX_FRAME_DAVIS  1  // default; as measured on Davis transmitters: 18 bits lead, 32 bits preamble, short trailer, 10.4 kHz
#define TX_LEN_LEGACY (DAVIS_PACKET_LEN + 13)  // 4 lead + 6 preamble + 2 sync + 10 packet + 1 trailer bytes
#define TX_LEN_DAVIS 19                        // 146 bits, padded to whole bytes

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
    static volatile uint32_t realTxDelay;   // the transmit interval that just ended, by micros()
    static volatile uint32_t txLastTarget;  // the interval it was scheduled for, including the timing correction (micros)
    static volatile bool txLearn;           // learn txPpm from the received stations, or keep it fixed
    static volatile int32_t txPpmAcc;       // transmit timing correction, 1/256 ppm
    static volatile uint16_t txPpmSamples;  // samples behind it (saturates)
    static volatile uint16_t txPpmRejects;  // samples rejected as outliers
    static volatile bool txAfc;             // correct the transmit frequency with the software AFC's learned offset
    static volatile int16_t txAfcApplied;   // transmit frequency correction in use (FSTEP)
    static volatile byte txFrame;           // TX_FRAME_LEGACY or TX_FRAME_DAVIS
    static volatile byte txData[DAVIS_PACKET_LEN];  // transmit buffer, separate from DATA which the receiver owns
    static volatile uint32_t timeBase;
    static volatile bool rxOn;              // the receiver runs between transmissions; false after stopReceiver()
    static volatile byte txId;
    static volatile byte txChannel;
    static volatile byte rxRssi;
    static volatile byte afcMode;
    static volatile int16_t afcApplied;
    static volatile int16_t afcLast;
    static volatile uint32_t rssiTime;
    static volatile bool feiWait;
    static volatile bool feiPending;
    static volatile byte feiRssi;
    static volatile int16_t discCand;
    static volatile bool discCandValid;
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
    static void initStations();
    void nextStation();

    static void handleTimerInt();
    static void handleTxInt();
    static void setStations(Station *_stations, byte n);
    static void resetAfc();
    void stopReceiver();
    void setRssiThreshold(int rssiThreshold);
    void setRssiThresholdRaw(int rssiThresholdRaw);
    void setFreqCorr(int16_t value);
    void setAfc(byte mode);
    void setTimeBase(uint32_t value);
    void setTxTiming(bool learn, int32_t ppm);  // ppm in 1/16 ppm: start value when learning, fixed value otherwise
    static int32_t txPpm() { return txPpmAcc / 16; }  // current correction, 1/16 ppm
    uint32_t timeRxTimeout(uint16_t bitrate, byte n);
    static uint32_t txTimeLeft();  // micros until the next transmission  // micros() until the RFM69's RX start timeout, 0 if not possible
    void setTxAfc(bool on);
    void setTxFrame(byte frame);  // TX_FRAME_LEGACY or TX_FRAME_DAVIS
    void setRepeaterPT(bool value = false);
    void enableTx(void (*function)(byte* buffer), byte ID);
    void disableTx();

  protected:
    static volatile bool txMode;
    static volatile int32_t txAcc;      // transmit schedule remainder below one Timer1 step (1/16 us)
    static volatile uint16_t txTick16;  // one Timer1 step (1/16 us)
    static void scheduleTx();
    static void learnTxTiming(byte id, uint32_t dt, byte n);
    static int16_t stationsAfc();
    static volatile uint32_t txNextTarget;  // the interval the running one is scheduled for (micros)
    void (*txCallback)(byte* buffer);

    void virtual interruptHandler();
    void sendFrame(const byte* buffer);
    static void putBits(byte *f, byte &nb, byte v, byte n);
    byte reverseBits(byte b);

    static void isr0();

    static DavisRFM69* selfPointer;
    byte _slaveSelectPin;
    byte _interruptPin;
    byte _interruptNum;
    byte _powerLevel;
    bool _isRFM69HW;

    void receiveBegin();
    void updateAfc(Station *s);
    void discoveryFei();
    static bool swAfc() { return afcMode != RF69_AFC_OFF && afcMode != RF69_AFC_HW; }
    void setMode(byte mode);
    void setHighPowerRegs(bool onOff);
    void setTxMode(bool mode);
    void select();
    void unselect();
};

// RFM69 FRF_MSB, FRF_MID and FRF_LSB of the channels Davis hops through, in hop order.

#define FREQ_TABLE_LENGTH_US 51
#define FREQ_TABLE_LENGTH_EU 5
#define FREQ_TABLE_LENGTH_AU 51
#define FREQ_TABLE_LENGTH_NZ 51
#define FREQ_TABLE_LENGTH_JP 16
#define FREQ_TABLE_LENGTH_IN 19
#define FREQ_TABLE_LENGTH_R2 2
#define FREQ_TABLE_LENGTH_RU 5
#define FREQ_TABLE_LENGTH_TW 19

#define FREQ_BAND_US 0
#define FREQ_BAND_EU 1
#define FREQ_BAND_AU 2
#define FREQ_BAND_NZ 3
#define FREQ_BAND_JP 4
#define FREQ_BAND_IN 5
#define FREQ_BAND_R2 6
#define FREQ_BAND_RU 7
#define FREQ_BAND_TW 8
#define FREQ_BAND_COUNT 9

typedef uint8_t FRF_ITEM[3];

// US, domain 1 (the default): Davis BAND 0, 51 channels, 902.35575-927.44325 MHz, 501.75 kHz apart
static const __attribute__((progmem)) FRF_ITEM FRF_US[FREQ_TABLE_LENGTH_US] =
{
  {0xE3, 0xD8, 0xC9},  // 911.38725
  {0xE1, 0x96, 0xC5},  // 902.35575
  {0xE3, 0xF8, 0xE5},  // 911.88900
  {0xE6, 0xBB, 0x5C},  // 922.92750
  {0xE4, 0xB9, 0x91},  // 914.89950
  {0xE2, 0x97, 0xAA},  // 906.36975
  {0xE7, 0x7C, 0x08},  // 925.93800
  {0xE5, 0x9A, 0x5A},  // 918.41175
  {0xE3, 0x38, 0x39},  // 908.87850
  {0xE6, 0x1A, 0xCD},  // 920.41875
  {0xE4, 0x59, 0x3B},  // 913.39425
  {0xE1, 0xF7, 0x1B},  // 903.86100
  {0xE5, 0x3A, 0x04},  // 916.90650
  {0xE7, 0x1B, 0xB2},  // 924.43275
  {0xE3, 0x98, 0x8F},  // 910.38375
  {0xE2, 0x37, 0x54},  // 904.86450
  {0xE4, 0xF9, 0xCB},  // 915.90300
  {0xE6, 0x5B, 0x06},  // 921.42225
  {0xE2, 0xD7, 0xE3},  // 907.37325
  {0xE7, 0xBC, 0x42},  // 926.94150
  {0xE4, 0x39, 0x1F},  // 912.89250
  {0xE1, 0xD6, 0xFE},  // 903.35925
  {0xE5, 0x5A, 0x21},  // 917.40825
  {0xE6, 0xDB, 0x79},  // 923.42925
  {0xE3, 0x58, 0x56},  // 909.38025
  {0xE7, 0x9C, 0x25},  // 926.43975
  {0xE2, 0x77, 0x8D},  // 905.86800
  {0xE4, 0x99, 0x75},  // 914.39775
  {0xE5, 0xDA, 0x93},  // 919.41525
  {0xE7, 0x3B, 0xCF},  // 924.93450
  {0xE1, 0xB6, 0xE1},  // 902.85750
  {0xE3, 0xB8, 0xAC},  // 910.88550
  {0xE6, 0x7B, 0x23},  // 921.92400
  {0xE4, 0xD9, 0xAE},  // 915.40125
  {0xE2, 0xB7, 0xC7},  // 906.87150
  {0xE5, 0x7A, 0x3D},  // 917.91000
  {0xE7, 0xDC, 0x5E},  // 927.44325
  {0xE6, 0x3A, 0xE9},  // 920.92050
  {0xE3, 0x18, 0x1D},  // 908.37675
  {0xE4, 0x19, 0x02},  // 912.39075
  {0xE5, 0xBA, 0x77},  // 918.91350
  {0xE2, 0x17, 0x37},  // 904.36275
  {0xE6, 0xFB, 0x96},  // 923.93100
  {0xE5, 0x19, 0xE7},  // 916.40475
  {0xE3, 0x78, 0x73},  // 909.88200
  {0xE5, 0xFA, 0xB0},  // 919.91700
  {0xE2, 0x57, 0x71},  // 905.36625
  {0xE6, 0x9B, 0x3F},  // 922.42575
  {0xE2, 0xF8, 0x00},  // 907.87500
  {0xE4, 0x79, 0x58},  // 913.89600
  {0xE7, 0x5B, 0xEC}   // 925.43625
};

// EU: Davis BAND 1, 5 channels, 868.05990-868.54050 MHz, 120.15 kHz apart
static const __attribute__((progmem)) FRF_ITEM FRF_EU[FREQ_TABLE_LENGTH_EU] =
{
  {0xD9, 0x03, 0xD5},  // 868.05990
  {0xD9, 0x13, 0x36},  // 868.30020
  {0xD9, 0x22, 0x98},  // 868.54050
  {0xD9, 0x0B, 0x86},  // 868.18005
  {0xD9, 0x1A, 0xE7}   // 868.42035
};

// Australia, Brazil: Davis BAND 2, 51 channels, 918.07830-925.90830 MHz, 156.60 kHz apart
static const __attribute__((progmem)) FRF_ITEM FRF_AU[FREQ_TABLE_LENGTH_AU] =
{
  {0xE5, 0x85, 0x03},  // 918.07830
  {0xE6, 0x43, 0x70},  // 921.05370
  {0xE7, 0x1F, 0xEE},  // 924.49890
  {0xE6, 0x7F, 0x92},  // 921.99330
  {0xE5, 0xD5, 0x31},  // 919.33110
  {0xE7, 0x5C, 0x10},  // 925.43850
  {0xE6, 0xC5, 0xBA},  // 923.08950
  {0xE6, 0x07, 0x4D},  // 920.11410
  {0xE6, 0xED, 0xD1},  // 923.71590
  {0xE6, 0x61, 0x81},  // 921.52350
  {0xE5, 0xA3, 0x14},  // 918.54810
  {0xE6, 0xA7, 0xA9},  // 922.61970
  {0xE7, 0x3D, 0xFF},  // 924.96870
  {0xE6, 0x25, 0x5F},  // 920.58390
  {0xE5, 0xB7, 0x20},  // 918.86130
  {0xE6, 0x93, 0x9E},  // 922.30650
  {0xE7, 0x01, 0xDD},  // 924.02910
  {0xE5, 0xE9, 0x3C},  // 919.64430
  {0xE7, 0x70, 0x1C},  // 925.75170
  {0xE6, 0x57, 0x7B},  // 921.36690
  {0xE5, 0x99, 0x0E},  // 918.39150
  {0xE6, 0xB1, 0xAF},  // 922.77630
  {0xE7, 0x29, 0xF4},  // 924.65550
  {0xE6, 0x11, 0x53},  // 920.27070
  {0xE7, 0x66, 0x16},  // 925.59510
  {0xE5, 0xCB, 0x2B},  // 919.17450
  {0xE6, 0x75, 0x8C},  // 921.83670
  {0xE6, 0xD9, 0xC6},  // 923.40270
  {0xE7, 0x48, 0x05},  // 925.12530
  {0xE5, 0x8F, 0x09},  // 918.23490
  {0xE6, 0x2F, 0x64},  // 920.74050
  {0xE7, 0x0B, 0xE3},  // 924.18570
  {0xE6, 0x89, 0x98},  // 922.14990
  {0xE5, 0xDF, 0x36},  // 919.48770
  {0xE6, 0xBB, 0xB5},  // 922.93290
  {0xE7, 0x7A, 0x22},  // 925.90830
  {0xE6, 0xF7, 0xD7},  // 923.87250
  {0xE5, 0xFD, 0x48},  // 919.95750
  {0xE6, 0x4D, 0x76},  // 921.21030
  {0xE6, 0xCF, 0xC0},  // 923.24610
  {0xE5, 0xAD, 0x1A},  // 918.70470
  {0xE7, 0x33, 0xF9},  // 924.81210
  {0xE6, 0x9D, 0xA3},  // 922.46310
  {0xE6, 0x1B, 0x59},  // 920.42730
  {0xE6, 0xE3, 0xCC},  // 923.55930
  {0xE5, 0xC1, 0x25},  // 919.01790
  {0xE7, 0x15, 0xE8},  // 924.34230
  {0xE5, 0xF3, 0x42},  // 919.80090
  {0xE6, 0x6B, 0x87},  // 921.68010
  {0xE7, 0x52, 0x0B},  // 925.28190
  {0xE6, 0x39, 0x6A}   // 920.89710
};

// New Zealand, Peru: Davis BAND 3, 51 channels, 921.07800-927.94050 MHz, 137.25 kHz apart
static const __attribute__((progmem)) FRF_ITEM FRF_NZ[FREQ_TABLE_LENGTH_NZ] =
{
  {0xE6, 0x44, 0xFE},  // 921.07800
  {0xE6, 0xEB, 0xE3},  // 923.68575
  {0xE7, 0xAD, 0x23},  // 926.70525
  {0xE7, 0x20, 0x98},  // 924.50925
  {0xE6, 0x8B, 0x44},  // 922.17600
  {0xE7, 0xE1, 0xD7},  // 927.52875
  {0xE7, 0x5E, 0x14},  // 925.47000
  {0xE6, 0xB7, 0x2F},  // 922.86225
  {0xE7, 0x81, 0x37},  // 926.01900
  {0xE7, 0x06, 0x3D},  // 924.09750
  {0xE6, 0x5F, 0x58},  // 921.48975
  {0xE7, 0x43, 0xBA},  // 925.05825
  {0xE7, 0xC7, 0x7D},  // 927.11700
  {0xE6, 0xD1, 0x89},  // 923.27400
  {0xE6, 0x70, 0xE9},  // 921.76425
  {0xE7, 0x32, 0x29},  // 924.78375
  {0xE7, 0x92, 0xC9},  // 926.29350
  {0xE6, 0x9C, 0xD5},  // 922.45050
  {0xE7, 0xF3, 0x68},  // 927.80325
  {0xE6, 0xFD, 0x75},  // 923.96025
  {0xE6, 0x56, 0x8F},  // 921.35250
  {0xE7, 0x4C, 0x83},  // 925.19550
  {0xE7, 0xB5, 0xEC},  // 926.84250
  {0xE6, 0xBF, 0xF8},  // 922.99950
  {0xE7, 0xEA, 0xA0},  // 927.66600
  {0xE6, 0x82, 0x7B},  // 922.03875
  {0xE7, 0x17, 0xCF},  // 924.37200
  {0xE7, 0x6F, 0xA6},  // 925.74450
  {0xE7, 0xD0, 0x46},  // 927.25425
  {0xE6, 0x4D, 0xC7},  // 921.21525
  {0xE6, 0xDA, 0x52},  // 923.41125
  {0xE7, 0x9B, 0x91},  // 926.43075
  {0xE7, 0x29, 0x60},  // 924.64650
  {0xE6, 0x94, 0x0C},  // 922.31325
  {0xE7, 0x55, 0x4C},  // 925.33275
  {0xE7, 0xFC, 0x31},  // 927.94050
  {0xE7, 0x8A, 0x00},  // 926.15625
  {0xE6, 0xAE, 0x66},  // 922.72500
  {0xE6, 0xF4, 0xAC},  // 923.82300
  {0xE7, 0x66, 0xDD},  // 925.60725
  {0xE6, 0x68, 0x21},  // 921.62700
  {0xE7, 0xBE, 0xB4},  // 926.97975
  {0xE7, 0x3A, 0xF2},  // 924.92100
  {0xE6, 0xC8, 0xC1},  // 923.13675
  {0xE7, 0x78, 0x6F},  // 925.88175
  {0xE6, 0x79, 0xB2},  // 921.90150
  {0xE7, 0xA4, 0x5A},  // 926.56800
  {0xE6, 0xA5, 0x9E},  // 922.58775
  {0xE7, 0x0F, 0x06},  // 924.23475
  {0xE7, 0xD9, 0x0E},  // 927.39150
  {0xE6, 0xE3, 0x1B}   // 923.54850
};

// Japan (band used in later firmware updates): Davis BAND 4, 16 channels, 928.15065-929.64915 MHz, 99.90 kHz apart
static const __attribute__((progmem)) FRF_ITEM FRF_JP[FREQ_TABLE_LENGTH_JP] =
{
  {0xE8, 0x09, 0xA4},  // 928.15065
  {0xE8, 0x4F, 0xF9},  // 929.24955
  {0xE8, 0x5C, 0xC2},  // 929.44935
  {0xE8, 0x10, 0x09},  // 928.25055
  {0xE8, 0x16, 0x6E},  // 928.35045
  {0xE8, 0x63, 0x27},  // 929.54925
  {0xE8, 0x1C, 0xD3},  // 928.45035
  {0xE8, 0x23, 0x37},  // 928.55025
  {0xE8, 0x30, 0x01},  // 928.75005
  {0xE8, 0x3C, 0xCA},  // 928.94985
  {0xE8, 0x49, 0x94},  // 929.14965
  {0xE8, 0x56, 0x5D},  // 929.34945
  {0xE8, 0x36, 0x66},  // 928.84995
  {0xE8, 0x43, 0x2F},  // 929.04975
  {0xE8, 0x69, 0x8C},  // 929.64915
  {0xE8, 0x29, 0x9C}   // 928.65015
};

// India: Davis BAND 5, 19 channels, 865.09980-866.90610 MHz, 100.35 kHz apart
static const __attribute__((progmem)) FRF_ITEM FRF_IN[FREQ_TABLE_LENGTH_IN] =
{
  {0xD8, 0xAD, 0x25},  // 866.70540
  {0xD8, 0x46, 0x63},  // 865.09980
  {0xD8, 0x8D, 0x09},  // 866.20365
  {0xD8, 0x99, 0xE1},  // 866.40435
  {0xD8, 0x4C, 0xCF},  // 865.20015
  {0xD8, 0x53, 0x3B},  // 865.30050
  {0xD8, 0xA0, 0x4D},  // 866.50470
  {0xD8, 0x59, 0xA8},  // 865.40085
  {0xD8, 0xB9, 0xFE},  // 866.90610
  {0xD8, 0x60, 0x14},  // 865.50120
  {0xD8, 0x6C, 0xEC},  // 865.70190
  {0xD8, 0x79, 0xC4},  // 865.90260
  {0xD8, 0x86, 0x9C},  // 866.10330
  {0xD8, 0x93, 0x75},  // 866.30400
  {0xD8, 0x73, 0x58},  // 865.80225
  {0xD8, 0x80, 0x30},  // 866.00295
  {0xD8, 0xA6, 0xB9},  // 866.60505
  {0xD8, 0xB3, 0x91},  // 866.80575
  {0xD8, 0x66, 0x80}   // 865.60155
};

// Russia, 2nd variant: Davis BAND 6, 2 channels, 868.05000-868.14990 MHz, 99.90 kHz apart
static const __attribute__((progmem)) FRF_ITEM FRF_R2[FREQ_TABLE_LENGTH_R2] =
{
  {0xD9, 0x03, 0x33},  // 868.05000
  {0xD9, 0x09, 0x98}   // 868.14990
};

// Russia: Davis BAND 7, 5 channels, 868.75020-869.14980 MHz, 99.90 kHz apart
static const __attribute__((progmem)) FRF_ITEM FRF_RU[FREQ_TABLE_LENGTH_RU] =
{
  {0xD9, 0x30, 0x03},  // 868.75020
  {0xD9, 0x3C, 0xCD},  // 868.95000
  {0xD9, 0x49, 0x96},  // 869.14980
  {0xD9, 0x36, 0x68},  // 868.85010
  {0xD9, 0x43, 0x32}   // 869.04990
};

// Taiwan: Davis BAND 8, 19 channels, 920.40165-924.59745 MHz, 233.10 kHz apart
static const __attribute__((progmem)) FRF_ITEM FRF_TW[FREQ_TABLE_LENGTH_TW] =
{
  {0xE7, 0x08, 0x66},  // 924.13125
  {0xE6, 0x19, 0xB5},  // 920.40165
  {0xE6, 0xBD, 0xCF},  // 922.96575
  {0xE6, 0xDB, 0xA5},  // 923.43195
  {0xE6, 0x28, 0xA0},  // 920.63475
  {0xE6, 0x37, 0x8B},  // 920.86785
  {0xE6, 0xEA, 0x90},  // 923.66505
  {0xE6, 0x46, 0x76},  // 921.10095
  {0xE7, 0x26, 0x3D},  // 924.59745
  {0xE6, 0x55, 0x61},  // 921.33405
  {0xE6, 0x73, 0x37},  // 921.80025
  {0xE6, 0x91, 0x0E},  // 922.26645
  {0xE6, 0xAE, 0xE4},  // 922.73265
  {0xE6, 0xCC, 0xBA},  // 923.19885
  {0xE6, 0x82, 0x22},  // 922.03335
  {0xE6, 0x9F, 0xF9},  // 922.49955
  {0xE6, 0xF9, 0x7B},  // 923.89815
  {0xE7, 0x17, 0x52},  // 924.36435
  {0xE6, 0x64, 0x4C}   // 921.56715
};

static const FRF_ITEM *bandTab[FREQ_BAND_COUNT] = {
  FRF_US,
  FRF_EU,
  FRF_AU,
  FRF_NZ,
  FRF_JP,
  FRF_IN,
  FRF_R2,
  FRF_RU,
  FRF_TW
};

static const uint8_t bandTabLengths[FREQ_BAND_COUNT] = {
  FREQ_TABLE_LENGTH_US,
  FREQ_TABLE_LENGTH_EU,
  FREQ_TABLE_LENGTH_AU,
  FREQ_TABLE_LENGTH_NZ,
  FREQ_TABLE_LENGTH_JP,
  FREQ_TABLE_LENGTH_IN,
  FREQ_TABLE_LENGTH_R2,
  FREQ_TABLE_LENGTH_RU,
  FREQ_TABLE_LENGTH_TW
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
#define STYPE_OFF         0xA  // No station � OFF
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
