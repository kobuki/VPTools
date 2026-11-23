// Driver implementation for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from a Davis Instrument
// wireless Integrated Sensor Suite (ISS)
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
//
// As I consider this to be a derived work for now from the RFM69W library from LowPowerLab,
// it is released under the same Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/3.0/
//
// In accordance with the CC-BY-SA, many modifications by GitHub user "kobuki".

#include "DavisRFM69.h"
#include "RFM69registers.h"
#include "TimerOne.h"
#include "MsTimer2.h"
#include "PacketFifo.h"

#include <SPI.h>

volatile byte DavisRFM69::DATA[DAVIS_PACKET_LEN];
volatile byte DavisRFM69::_mode;       // current transceiver state
volatile bool DavisRFM69::_packetReceived = false;
volatile byte DavisRFM69::CHANNEL = 0;
volatile byte DavisRFM69::band = 0;
volatile int DavisRFM69::RSSI;   // RSSI measured immediately after payload reception
volatile int16_t DavisRFM69::FEI;
volatile bool DavisRFM69::txMode = false;
volatile uint32_t DavisRFM69::lastTx = micros();
volatile uint32_t DavisRFM69::txDelay = 0;
volatile uint32_t DavisRFM69::realTxDelay = 0;
volatile uint32_t DavisRFM69::txLastTarget = 0;
volatile uint32_t DavisRFM69::txNextTarget = 0;
volatile bool DavisRFM69::txLearn = false;
volatile int32_t DavisRFM69::txPpmAcc = 0;
volatile uint16_t DavisRFM69::txPpmSamples = 0;
volatile uint16_t DavisRFM69::txPpmRejects = 0;
volatile bool DavisRFM69::txAfc = false;
volatile int16_t DavisRFM69::txAfcApplied = 0;
volatile byte DavisRFM69::txFrame = TX_FRAME_DAVIS;
volatile byte DavisRFM69::txData[DAVIS_PACKET_LEN];
volatile int32_t DavisRFM69::txAcc = 0;
volatile uint16_t DavisRFM69::txTick16 = 0;
volatile byte DavisRFM69::txId = 255;
volatile byte DavisRFM69::txChannel = 255;
volatile byte DavisRFM69::rxRssi = 190; // -95 dBm, same as REG_RSSITHRESH in the init table
volatile byte DavisRFM69::afcMode = RF69_AFC_OFF;
volatile int16_t DavisRFM69::afcApplied = 0;  // software AFC correction the receiver is currently tuned with
volatile int16_t DavisRFM69::afcLast = 0;     // last learned correction of any station, used for discovery
volatile uint32_t DavisRFM69::rssiTime = 0;   // software AFC: when RSSI triggered
volatile bool DavisRFM69::feiWait = false;    // software AFC: inside the wait for the preamble
volatile bool DavisRFM69::feiPending = false; // software AFC: FEI started, payload not yet received
volatile byte DavisRFM69::feiRssi = 0;        // software AFC: raw RSSI on the preamble at FEI start
volatile int16_t DavisRFM69::discCand = 0;    // software AFC: weak discovery reading waiting for a second one
volatile bool DavisRFM69::discCandValid = false;

volatile uint32_t DavisRFM69::packets = 0;
volatile uint32_t DavisRFM69::lostPackets = 0;
volatile uint32_t DavisRFM69::numResyncs = 0;
volatile uint32_t DavisRFM69::lostStations = 0;
volatile byte DavisRFM69::stationsFound = 0;
volatile byte DavisRFM69::curStation = 0;
volatile byte DavisRFM69::numStations = 0;
volatile byte DavisRFM69::discChannel = 2;
volatile uint32_t DavisRFM69::lastDiscStep;
volatile int16_t DavisRFM69::freqCorr = 0;
volatile uint32_t DavisRFM69::timeBase = 1000000;
volatile bool DavisRFM69::rxOn = false;
volatile bool DavisRFM69::repeaterPT = false;

PacketFifo DavisRFM69::fifo;
Station *DavisRFM69::stations;
DavisRFM69* DavisRFM69::selfPointer;

void DavisRFM69::initialize(byte freqBand)
{
  const byte CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_10 }, // Davis uses Gaussian shaping with BT=0.5
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_19200 }, // (0x06) Davis uses a datarate of 19.2 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_19200 }, // (0x83)
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_9900 }, // (0x00) Davis uses a deviation of 9.9 kHz
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_9900 }, // (0xa1)
    /* 0x07 to 0x09 are REG_FRFMSB to LSB. No sense setting them here. Done in main routine.
    /* 0x0B */ { REG_AFCCTRL, RF_AFCLOWBETA_OFF }, // TODO: Should use LOWBETA_ON, but having trouble getting it working
    /* 0x12 */ { REG_PARAMP, RF_PARAMP_25 }, // xxx
    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout=-18+OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout=-14+OutputPower (with PA1 and PA2)**
    // +20dBpaym formula: Pout=-11+OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)
    /* 0x18 */ { REG_LNA, RF_LNA_ZIN_50 | RF_LNA_GAINSELECT_AUTO }, // Not sure which is correct!
    // REG_RXBW 50 kHz fixes console retransmit reception but seems worse for SIM transmitters (to be confirmed with more testing)
    // Defaulting to narrow BW, since console retransmits are rarely used - use setBandwidth() to change this
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4 }, // Use 25 kHz BW (BitRate < 2 * RxBw)
    /* 0x1A */ { REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3 }, // Use double the bandwidth during AFC as reception
    /* 0x1B - 0x1D These registers are for OOK.  Not used */
    // No auto AFC: when RSSI triggers on the flat carrier around Davis packets, AFC locks one deviation off and the packet is lost
    /* 0x1E */ { REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON | RF_AFCFEI_AFCAUTO_OFF },
    /* 0x1F & 0x20 AFC MSB and LSB values, respectively */
    /* 0x21 & 0x22 FEI MSB and LSB values, respectively */
    /* 0x23 & 0x24 RSSI MSB and LSB values, respectively */
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 RegDioMapping2 */
    /* 0x27 RegIRQFlags1 */
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // Reset the FIFOs. Fixes a problem I had with bad first packet.
    /* 0x29 */ { REG_RSSITHRESH, 190 }, // real dBm = -(REG_RSSITHRESH / 2) -> 190 raw = -95 dBm
    /* 0x2a & 0x2b RegRxTimeout1 and 2, respectively */
    /* 0x2c RegPreambleMsb - use zero default */
    /* 0x2d */ { REG_PREAMBLELSB, 0x4 }, // Davis has four preamble bytes 0xAAAAAAAA -- use 6 for TX for this setup
    /* 0x2e */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_2 },  // Allow a couple erros in the sync word
    /* 0x2f */ { REG_SYNCVALUE1, 0xcb }, // Davis ISS first sync byte. http://madscientistlabs.blogspot.ca/2012/03/first-you-get-sugar.html
    /* 0x30 */ { REG_SYNCVALUE2, 0x89 }, // Davis ISS second sync byte.
    /* 0x31 - 0x36  REG_SYNCVALUE3 - 8 not used */
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_OFF | RF_PACKET1_ADRSFILTERING_OFF }, // Fixed packet length and we'll check our own CRC
    /* 0x38 */ { REG_PAYLOADLENGTH, DAVIS_PACKET_LEN }, // Davis sends 8 bytes of payload, including CRC that we check manually.
    //* 0x39 */ { REG_NODEADRS, nodeID }, // Turned off because we're not using address filtering
    //* 0x3a */ { REG_BROADCASTADRS, RF_BROADCASTADDRESS_VALUE }, // Not using this
    /* 0x3b REG_AUTOMODES - Automatic modes are not used in this implementation. */
    /* 0x3c */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | DAVIS_PACKET_LEN + 13 - 1 }, // TX on FIFO having enough bytes
    /* 0x3d */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x3e - 0x4d  AES Key not used in this implementation */
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // // TODO: Should use LOWBETA_ON, but having trouble getting it working
    /* 0x71 */ { REG_TESTAFC, 0 }, // AFC Offset for low mod index systems
    {255, 0}
  };

  digitalWrite(_slaveSelectPin, HIGH);
  pinMode(_slaveSelectPin, OUTPUT);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2); // max speed, except on Due which can run at system clock speed
  SPI.begin();

  do writeReg(REG_SYNCVALUE1, 0xaa); while (readReg(REG_SYNCVALUE1) != 0xaa);
  do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55);

  for (byte i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);
  setAfc(afcMode);

  setHighPower(_isRFM69HW); // called regardless if it's a RFM69W or RFM69HW
  setMode(RF69_MODE_STANDBY);
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
  // isr0() goes through selfPointer, and a DIO0 edge left pending by the still running radio fires right away
  selfPointer = this;
  attachInterrupt(_interruptNum, DavisRFM69::isr0, RISING);

  setBand(freqBand);
  setChannel(discChannel);

  fifo.flush();
  initStations();
  stationsFound = 0;  // a restart (new band or station set) starts discovering again
  resetAfc();
  lastDiscStep = micros();

  rxOn = true;
  MsTimer2::set(1, DavisRFM69::handleTimerInt);
  MsTimer2::start();
}

// Stops the receiver and the transmitter. Call enableTx() after it for a transmitter that sleeps between packets.
void DavisRFM69::stopReceiver() {
  Timer1.detachInterrupt();
  MsTimer2::stop();
  rxOn = false;
  setMode(RF69_MODE_SLEEP);
}

void DavisRFM69::setStations(Station *_stations, byte n) {
  noInterrupts();  // the receiver timer may already be running
  stations = _stations;
  numStations = n;
  initStations();
  stationsFound = 0;  // with a stale count, stationsFound >= numStations stopped discovery for good
  resetAfc();
  interrupts();
}

// AFC corrections survive resyncs (the crystal drift is still there), but not a restart or a new station set
void DavisRFM69::resetAfc() {
  for (byte i = 0; i < numStations; i++) {
    stations[i].afcCorr = 0;
    stations[i].afcSamples = 0;
  }
  afcLast = 0;
  discCandValid = false;
}

void DavisRFM69::handleTxInt() {
  uint32_t t = micros();
  scheduleTx();  // first, while interrupts are still off (SPI access turns them back on)
  realTxDelay = t - lastTx;
  lastTx = t;
  if (txAfc) {
    int16_t corr = stationsAfc();
    if (corr != RF69_FEI_INVALID) txAfcApplied = corr;  // no station learned yet: keep the last one
  }
  (*selfPointer->txCallback)((byte*)txData);
  selfPointer->send((byte*)txData, txChannel);
  txChannel = selfPointer->nextChannel(txChannel);
}

// Program the next transmit interval: nominal plus the timing correction. Timer1 counts in coarse steps (128 us
// at 16 MHz, ~48 ppm of an interval), so the remainder carries over and the steps average out to the exact target.
// Called from the Timer1 ISR at BOTTOM: ICR1 isn't double buffered, but the counter is far below any new TOP.
void DavisRFM69::scheduleTx() {
  int32_t corr16 = (int32_t)(txDelay / 100) * txPpm() / 10000;  // txDelay * ppm, in 1/16 us
  int32_t target16 = (int32_t)txDelay * 16 + corr16;
  txLastTarget = txNextTarget;
  txNextTarget = target16 / 16;
  txAcc += target16;
  uint16_t steps = txAcc / txTick16;
  txAcc -= (int32_t)steps * txTick16;
  ICR1 = steps;
}

// A station's packet came dt after its previous one, n intervals apart, by this board's clock.
// Called with interrupts off.
void DavisRFM69::learnTxTiming(byte id, uint32_t dt, byte n) {
  if (!txLearn || n > TX_PPM_MAX_SPAN) return;
  int32_t nominal = (int32_t)(41 + id) * 62500L * n;
  int32_t err = (int32_t)dt - nominal;
  if (err > nominal / 100 || err < -nominal / 100) return;  // not n intervals at all
  int32_t s = err * 256 / ((int32_t)(41 + id) * n);  // err / nominal in 1/16 ppm
  int32_t acc = s * 16;
  if (s > TX_PPM_MAX || s < -TX_PPM_MAX
      || (txPpmSamples >= TX_PPM_FAST && abs(acc - txPpmAcc) > TX_PPM_GATE * 16L)) {
    if (txPpmRejects < 0xffff) txPpmRejects++;
    return;
  }
  if (txPpmSamples < TX_PPM_FAST) {
    txPpmAcc += (acc - txPpmAcc) / (txPpmSamples + 1);  // plain mean of the samples so far
  } else {
    txPpmAcc += (acc - txPpmAcc) / TX_PPM_SLOW;
  }
  if (txPpmSamples < 0xffff) txPpmSamples++;
}

// Mean software AFC correction of the tracked stations, or RF69_FEI_INVALID if none has one yet.
// The receiver hears them tuned to the table plus this, so transmitting there lands where they are.
int16_t DavisRFM69::stationsAfc() {
  int32_t sum = 0;
  byte n = 0;
  for (byte i = 0; i < numStations; i++) {
    if (stations[i].interval > 0 && stations[i].afcSamples > 0) {
      sum += stations[i].afcCorr;
      n++;
    }
  }
  return n > 0 ? sum / n : RF69_FEI_INVALID;
}

// Handle missed packets. Called from a timer ISR every ms
void DavisRFM69::handleTimerInt() {
  if (txMode || stations == NULL || numStations == 0) return;

  uint32_t t = micros();
  bool readjust = false;

  if (feiPending && t - rssiTime > AFC_PKT_MAX) {
    feiPending = false;
    selfPointer->discoveryFei();
  }

  if (stations[curStation].interval > 0
      && stations[curStation].lastRx + stations[curStation].interval - t < DISCOVERY_GUARD
      && CHANNEL != stations[curStation].channel) {
    selfPointer->setChannel(stations[curStation].channel);
    return;
  }

  if (t - lastDiscStep > DISCOVERY_STEP) {
    // with no station tracked there is nothing else to listen to: always (re)tune, which also rearms a stuck receiver
    bool parked = stationsFound < numStations && (CHANNEL == discChannel || stationsFound == 0);
    discChannel = selfPointer->nextChannel(discChannel);
    lastDiscStep = t;
    if (parked) selfPointer->setChannel(discChannel);  // actually move the receiver to the new discovery channel
  }

  // find and adjust 'last seen' rx time on all stations with older reception timestamp than their period + threshold
  // that is, find missed packets
  for (byte i = 0; i < numStations; i++) {
    if (stations[i].interval == 0) continue;
    // signed: a lastRx ahead of t means not due yet. As unsigned it wrapped to a huge lateness, and since every
    // miss pushes lastRx another interval ahead, that counted a miss on each 1 ms tick until the station was dropped
    int32_t late = (int32_t)(t - stations[i].lastRx) - (int32_t)stations[i].interval;
    if (late < -(int32_t)stations[i].interval) {
      stations[i].lastRx = t;  // more than an interval ahead can't be right: re-anchor, normal miss counting follows
      continue;
    }
    if (late > LATE_PACKET_THRESH) {
      if (stations[curStation].active) lostPackets++;
      stations[i].lostPackets++;
      stations[i].missedPackets++;
      if (stations[i].lostPackets > RESYNC_THRESHOLD) {
        stations[i].numResyncs++;
        stations[i].interval = 0;
        stations[i].lostPackets = 0;
        lostStations++;
        if (stationsFound > 0) stationsFound--;
        if (stationsFound == 0) {
          numResyncs++;
          stationsFound = 0;
          lostStations = 0;
          selfPointer->initStations();
          selfPointer->setChannel(discChannel);
          return;
        }
      } else {
        stations[i].lastRx += stations[i].interval; // when packet should have been received
        stations[i].channel = selfPointer->nextChannel(stations[i].channel); // skip station's next channel in hop seq
        readjust = true;
      }
    }
  }

  if (readjust) {
    selfPointer->nextStation();
    selfPointer->setChannel(stations[curStation].channel);
  }
}

// Handle received packets, called from RFM69 ISR
void DavisRFM69::handleRadioInt() {
  uint32_t lastRx = micros();
  uint16_t rxCrc = word(DATA[6], DATA[7]);  // received CRC
  uint16_t calcCrc = DavisRFM69::crc16_ccitt(DATA, 6);  // calculated CRC

  bool repeaterCrcTried = false;

  // repeater packets checksum bytes (0..5) and (8..9), so try this at mismatch
  if (calcCrc != rxCrc) {
    calcCrc = DavisRFM69::crc16_ccitt(DATA + 8, 2, calcCrc);
    repeaterCrcTried = true;
  }

  // fifo.queue((byte*)DATA, CHANNEL, -RSSI, FEI, lastRx - stations[curStation].lastSeen);

  // packet passed crc?
  if (calcCrc == rxCrc && rxCrc != 0) {

    // station id is byte 0:0-2
    byte id = DATA[0] & 7;
    int stIx = findStation(id);

    // ignore the packet if we have no station configured for this id (at all; it can still be !active),
    // OR it passed only the repeater crc check without repeater pass-through (repeaters are not supported otherwise)
    if (stIx < 0 || (!repeaterPT && repeaterCrcTried)) {
      setChannel(CHANNEL);
      return;
    }

    // foreign packets passing the CRC by chance would re-anchor a station's timing: check the FF FF trailer and arrival time
    if (!repeaterCrcTried) {
      byte zeros = 0;
      for (byte b = 8; b < 10; b++) for (byte m = 1; m; m <<= 1) if (!(DATA[b] & m)) zeros++;
      if (zeros > TRAILER_MAX_ZERO_BITS) {
        setChannel(CHANNEL);
        return;
      }
    }
    noInterrupts();  // the timer ISR moves lastRx on, see below
    uint32_t expected = stations[stIx].lastRx + stations[stIx].interval;
    uint32_t interval = stations[stIx].interval;
    byte lost = stations[stIx].lostPackets;
    uint32_t lastSeen = stations[stIx].lastSeen;
    interrupts();
    if (interval > 0) {
      int32_t early = (int32_t)(expected - lastRx);
      if (early > EARLY_PACKET_THRESH * (lost + 1)) {
        setChannel(CHANNEL);
        return;
      }
    }

    // The SPI access above re-enabled interrupts, so the 1 ms timer ISR can run in the middle of this. Update the
    // station's state atomically: a 32-bit lastRx half written when the timer reads it can lie ahead of its clock.
    noInterrupts();
    if (stations[stIx].interval == 0) {
      stations[stIx].interval = (41 + id) * timeBase / 16; // Davis' official tx interval in us
      if (stationsFound < numStations) stationsFound++;
      if (lostStations > 0) lostStations--;
    }

    if (swAfc()) updateAfc(&stations[stIx]);
    // repeated packets come late by the repeater's delay
    if (interval > 0 && lastSeen > 0 && !repeaterCrcTried) learnTxTiming(id, lastRx - lastSeen, lost + 1);

    if (stations[stIx].active) {
      packets++;
      stations[stIx].packets++;
      fifo.queue((byte*)DATA, CHANNEL, -RSSI, FEI, stations[curStation].lastSeen > 0 ? lastRx - stations[curStation].lastSeen : 0);
    }

    stations[stIx].lostPackets = 0;
    stations[stIx].lastRx = stations[stIx].lastSeen = lastRx;
    stations[stIx].channel = nextChannel(CHANNEL);

    nextStation(); // skip curStation to next station expected to tx
    interrupts();

    if (stationsFound < numStations && stations[curStation].lastRx + stations[curStation].interval - lastRx > DISCOVERY_MINGAP) {
      setChannel(discChannel);
    } else {
      setChannel(stations[curStation].channel); // reset current radio channel
    }

  } else {
    setChannel(CHANNEL); // this always has to be done somewhere right after reception, even for ignored/bogus packets
  }
}

// Software AFC: RSSI triggered, but no payload followed. While discovering, that is most likely a packet lost to
// the very offset AFC is there to correct, so take the FEI measured on its preamble as the discovery correction.
void DavisRFM69::discoveryFei() {
  if (stationsFound >= numStations || CHANNEL != discChannel) return;  // tracking: the next hop rearms RSSI
  if (readReg(REG_AFCFEI) & RF_AFCFEI_FEI_DONE) {
    int32_t offset = (int32_t)afcApplied + (int16_t)word(readReg(REG_FEIMSB), readReg(REG_FEILSB));
    int32_t limit = afcMode * 1000L / RF69_FSTEP;
    bool follow = true;
    if (offset >= -limit && offset <= limit) {
      if (feiRssi + AFC_DISC_MARGIN <= rxRssi) {
        afcLast = offset;
        discCandValid = false;
      } else if (feiRssi + AFC_DISC_WEAK > rxRssi) {
        follow = false;  // too close to the threshold to tell from noise
      } else if (discCandValid && abs(offset - discCand) <= AFC_DISC_AGREE) {
        afcLast = (offset + discCand) / 2;
        discCandValid = false;
      } else {
        discCand = offset;
        discCandValid = true;
      }
    } else {
      follow = false;
    }
    if (follow) {
      // the transmitter hops to the next channel: wait there, one interval instead of a whole hop cycle away
      discChannel = nextChannel(discChannel);
      lastDiscStep = micros();
    }
  }
  setChannel(discChannel);  // rearm RSSI on the discovery channel, retuned if the correction changed
}

// Software AFC: FEI was measured on the packet's preamble, relative to the receiver tuned with afcApplied.
// Average it into the station's correction and limit that.
void DavisRFM69::updateAfc(Station *s) {
  if (FEI == RF69_FEI_INVALID) {  // no usable measurement: keep the correction, report the one in use
    FEI = afcApplied;
    return;
  }
  int32_t offset = (int32_t)afcApplied + FEI;
  int32_t diff = offset - s->afcCorr;
  int32_t corr;
  if (s->afcSamples == 0) {
    corr = offset - FEI / 2;  // the first packet came through at afcApplied: start there, not at 0, half way to the reading
  } else if (s->afcSamples < 4) {
    corr = s->afcCorr + diff / 2;  // converge fast at first
  } else {
    corr = s->afcCorr + constrain(diff, -AFC_MAX_STEP, AFC_MAX_STEP) / 8;  // then track slowly, ignoring outliers
  }
  int32_t limit = afcMode * 1000L / RF69_FSTEP;
  s->afcCorr = constrain(corr, -limit, limit);
  if (s->afcSamples < 255) s->afcSamples++;
  afcLast = s->afcCorr;
  FEI = offset;  // report the transmitter's offset from the frequency table
}

// Calculate the next hop of the specified channel
byte DavisRFM69::nextChannel(byte channel) {
  return ++channel % getBandTabLength();
}

// Find the station index in stations[] for station expected to tx the earliest and update curStation
void DavisRFM69::nextStation() {
  uint32_t earliest = 0xffffffff;
  uint32_t now = micros();
  for (int i = 0; i < numStations; i++) {
    uint32_t current = stations[i].lastRx + stations[i].interval - now;
    if (stations[i].interval > 0 && current < earliest) {
      earliest = current;
      curStation = i;
    }
  }
}

// Find station index in stations[] for a station ID (-1 if doesn't exist)
int DavisRFM69::findStation(byte id) {
  for (byte i = 0; i < numStations; i++) {
    if (stations[i].id == id) return i;
  }
  return -1;
}

// Reset station array to safe defaults
void DavisRFM69::initStations() {
  for (byte i = 0; i < numStations; i++) {
    stations[i].channel = 0;
    stations[i].lastRx = 0;
    stations[i].interval = 0;
    stations[i].lostPackets = 0;
    stations[i].lastRx = 0;
    stations[i].lastSeen = 0;
    stations[i].packets = 0;
    stations[i].missedPackets = 0;
  }
}

void DavisRFM69::interruptHandler() {
  if (txMode) return;

  RSSI = readRSSI();  // Read up front when it is most likely the carrier is still up
  if (_mode != RF69_MODE_RX) return;

  if (!(readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
    if (swAfc() && !feiWait && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_RSSI)) {
      // software AFC: DIO0 was mapped to Rssi, which triggers on the flat carrier leading every Davis packet.
      // FEI only works on the preamble that follows, so wait for it, then wait for the payload.
      // SPI access re-enables interrupts, feiWait guards against reentry if the timer ISR retunes meanwhile.
      rssiTime = micros();
      feiWait = true;
      writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
      delayMicroseconds(AFC_FEI_DELAY);
      writeReg(REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON | RF_AFCFEI_AFCAUTO_OFF | RF_AFCFEI_FEI_START);
      feiRssi = readReg(REG_RSSIVALUE);
      feiPending = true;
      feiWait = false;
    }
    return;
  }

  feiPending = false;  // the payload came, so no discovery reading from the timer
  if (afcMode == RF69_AFC_HW) {
    FEI = word(readReg(REG_FEIMSB), readReg(REG_FEILSB));  // measured by the auto AFC
  } else if (afcMode == RF69_AFC_OFF) {
    FEI = 0;  // never measured, report 0 as a placeholder
  } else {
    // valid only if RSSI triggered on this packet's leading carrier, ie. the payload followed in packet time
    uint32_t dt = micros() - rssiTime;
    bool valid = dt >= AFC_PKT_MIN && dt <= AFC_PKT_MAX && (readReg(REG_AFCFEI) & RF_AFCFEI_FEI_DONE);
    FEI = valid ? word(readReg(REG_FEIMSB), readReg(REG_FEILSB)) : RF69_FEI_INVALID;
  }
  setMode(RF69_MODE_STANDBY);
  select();   // Select RFM69 module, disabling interrupts
  SPI.transfer(REG_FIFO & 0x7f);
  for (byte i = 0; i < DAVIS_PACKET_LEN; i++) DATA[i] = reverseBits(SPI.transfer(0));
  unselect();  // Unselect RFM69 module, enabling interrupts
  // must run after unselect(): with CS still low, its first register write (FRFMSB) is swallowed by the FIFO burst
  _packetReceived = true;
  handleRadioInt();
}

bool DavisRFM69::canSend()
{
  if (_mode == RF69_MODE_RX && readRSSI() < CSMA_LIMIT) // if signal stronger than -100dBm is detected assume channel activity
  {
    setMode(RF69_MODE_STANDBY);
    return true;
  }
  return false;
}

void DavisRFM69::setTxMode(bool mode)
{
  if (mode) {
    txMode = mode;
    // the whole frame (lead, preamble, sync, packet, trailer) goes through the FIFO
    byte len = txFrame == TX_FRAME_DAVIS ? TX_LEN_DAVIS : TX_LEN_LEGACY;
    if (txFrame == TX_FRAME_DAVIS) {
      // 170 x 61 Hz = 10.4 kHz: on the SDR, Davis transmitters' deviation measured 3.5 % above our 10 kHz setting's
      writeReg(REG_FDEVMSB, 0x00);
      writeReg(REG_FDEVLSB, 0xaa);
    } else {
      writeReg(REG_FDEVMSB, RF_FDEVMSB_10000);
      writeReg(REG_FDEVLSB, RF_FDEVLSB_10000);
    }
    writeReg(REG_PREAMBLELSB, 0);
    writeReg(REG_SYNCCONFIG, RF_SYNC_OFF);
    writeReg(REG_PAYLOADLENGTH, len);
    writeReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | (len - 1));
  } else {
    writeReg(REG_FDEVMSB, RF_FDEVMSB_9900);
    writeReg(REG_FDEVLSB, RF_FDEVLSB_9900);
    writeReg(REG_PREAMBLELSB, 4);
    writeReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_2);
    writeReg(REG_PAYLOADLENGTH, DAVIS_PACKET_LEN);
    txMode = mode;
  }
}

// IMPORTANT: make sure buffer is at least 10 bytes
void DavisRFM69::send(const byte* buffer, byte channel)
{
  // back to where the receiver was: the expected station's channel, or the discovery channel. Retuning to
  // stations[curStation].channel instead kept discovery on channel 0, which also stopped its AFC offset readings
  byte rxChannel = CHANNEL;
  MsTimer2::stop();
  setTxMode(true);
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  setChannel(channel);
  sendFrame(buffer);
  setTxMode(false);
  if (rxOn) {
    setChannel(rxChannel);
    MsTimer2::start();
  } else {
    if (_isRFM69HW) setHighPowerRegs(false);
    setMode(RF69_MODE_SLEEP);  // transmitter only: sleep until the next packet
  }
}

// IMPORTANT: make sure buffer is at least 6 bytes
void DavisRFM69::sendFrame(const byte* buffer)
{
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  uint32_t t = micros();
  // Wait for ModeReady using 15 ms timeout
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) {
    if (micros() - t > SPI_OP_TIMEOUT) return;
  }
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"

  // the transmitter ID goes into byte 0 before the CRC, the battery flag (bit 3) stays
  byte frame[6];
  frame[0] = (buffer[0] & 0xf8) | txId;
  for (byte i = 1; i < 6; i++) frame[i] = buffer[i];
  // calculate crc on first 6 bytes (no repeater info)
  uint16_t crc = crc16_ccitt(frame, 6);
  if (txFrame == TX_FRAME_DAVIS) {
    // Davis' on-air structure, measured with an SDR: 18 bits lead on the '1' tone, 32 bits preamble,
    // sync word, the 10 packet bytes, then a short trailer. The FIFO takes whole bytes, so the bits are
    // packed: 18 + 32 + 16 + 80 = 146, padded with '1' bits to 152 (19 bytes).
    byte f[TX_LEN_DAVIS];
    byte nb = 0;
    memset(f, 0, sizeof(f));
    putBits(f, nb, 0xff, 8);
    putBits(f, nb, 0xff, 8);
    putBits(f, nb, 0x03, 2);
    for (byte i = 0; i < 4; i++) putBits(f, nb, 0xaa, 8);
    putBits(f, nb, 0xcb, 8);
    putBits(f, nb, 0x89, 8);
    for (byte i = 0; i < 6; i++) putBits(f, nb, reverseBits(frame[i]), 8);
    putBits(f, nb, reverseBits(crc >> 8), 8);
    putBits(f, nb, reverseBits(crc & 0xff), 8);
    putBits(f, nb, 0xff, 8);  // repeater info: none
    putBits(f, nb, 0xff, 8);
    putBits(f, nb, 0xff, TX_LEN_DAVIS * 8 - nb);  // trailer
    select();
    SPI.transfer(REG_FIFO | 0x80);
    for (byte i = 0; i < TX_LEN_DAVIS; i++) SPI.transfer(f[i]);
    unselect();
  } else {
    // write to FIFO
    select();
    SPI.transfer(REG_FIFO | 0x80);

    // The following byte sequence is an attempt to emulate the original Davis packet structure.

    // carrier on/start
    SPI.transfer(0xff);
    SPI.transfer(0xff);
    SPI.transfer(0xff);
    SPI.transfer(0xff);

    // preamble
    SPI.transfer(0xaa);
    SPI.transfer(0xaa);
    SPI.transfer(0xaa);
    SPI.transfer(0xaa);
    SPI.transfer(0xaa);
    SPI.transfer(0xaa);

    // sync word
    SPI.transfer(0xcb);
    SPI.transfer(0x89);

    for (byte i = 0; i < 6; i++)
      SPI.transfer(reverseBits(frame[i]));

    // transmit crc of first 6 bytes
    SPI.transfer(reverseBits(crc >> 8));
    SPI.transfer(reverseBits(crc & 0xff));

    // transmit dummy repeater info (always 0xff, 0xff for a normal packet without repeater)
    SPI.transfer(0xff);
    SPI.transfer(0xff);

    // add one extra for stability
    SPI.transfer(0xff);

    unselect();
  }

  setMode(RF69_MODE_TX);
  t = micros(); // wait SPI_OP_TIMEOUT ms max (transmission time is about 9.5 ms)
  while (digitalRead(_interruptPin) == 0 && micros() - t < SPI_OP_TIMEOUT); // wait for DIO0 to turn HIGH signalling transmission finish
  setMode(RF69_MODE_STANDBY);
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);
}

void DavisRFM69::setChannel(byte channel)
{
  CHANNEL = channel;

  byte a = pgm_read_byte(&bandTab[band][CHANNEL][0]);
  byte b = pgm_read_byte(&bandTab[band][CHANNEL][1]);
  byte c = pgm_read_byte(&bandTab[band][CHANNEL][2]);

  int32_t corr = freqCorr;
  if (txMode) {
    if (txAfc) corr += txAfcApplied;
  } else {
    // software AFC: the expected station's correction, or the last learned one while discovering
    afcApplied = 0;
    if (stations != NULL && swAfc()) {
      Station *s = &stations[curStation];
      afcApplied = (s->interval > 0 && channel == s->channel && s->afcSamples > 0) ? s->afcCorr : afcLast;
    }
    corr += afcApplied;
  }

  if (corr != 0) {
    uint32_t x = ((uint32_t)a<<16) | ((uint32_t)b<<8) | (uint32_t)c;
    x += corr;
    c =  x & 0x0000ff;
    b = (x & 0x00ff00) >> 8;
    a = (x & 0xff0000) >> 16;
  }

  writeReg(REG_FRFMSB, a);
  writeReg(REG_FRFMID, b);
  writeReg(REG_FRFLSB, c);

  if (!txMode) {
    // in RX the new frequency and a fresh AGC only take effect after leaving RX, so cycle through FS
    if (_mode == RF69_MODE_RX) setMode(RF69_MODE_SYNTH);
    receiveBegin();
    setRssiThresholdRaw(rxRssi);
  }
}

// The data bytes come over the air from the ISS least significant bit first. Fix them as we go. From
// http://www.ocf.berkeley.edu/~wwu/cgi-bin/yabb/YaBB.cgi?board=riddles_cs;action=display;num=1103355188
byte DavisRFM69::reverseBits(byte b)
{
  b = ((b & 0b11110000) >>4 ) | ((b & 0b00001111) << 4);
  b = ((b & 0b11001100) >>2 ) | ((b & 0b00110011) << 2);
  b = ((b & 0b10101010) >>1 ) | ((b & 0b01010101) << 1);

  return(b);
}

// Davis CRC calculation from http://www.menie.org/georges/embedded/
uint16_t DavisRFM69::crc16_ccitt(volatile byte *buf, byte len, uint16_t initCrc)
{
  uint16_t crc = initCrc;
  while( len-- ) {
    int i;
    crc ^= *(char *)buf++ << 8;
    for( i = 0; i < 8; ++i ) {
      if( crc & 0x8000 )
        crc = (crc << 1) ^ 0x1021;
      else
        crc = crc << 1;
    }
  }
  return crc;
}

void DavisRFM69::setMode(byte newMode)
{
  if (newMode == _mode) return;

  switch (newMode) {
    case RF69_MODE_TX:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      if (_isRFM69HW) setHighPowerRegs(true);
      break;
    case RF69_MODE_RX:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      if (_isRFM69HW) setHighPowerRegs(false);
      break;
    case RF69_MODE_SYNTH:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default: return;
  }

  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady

  _mode = newMode;
}

void DavisRFM69::sleep() {
  setMode(RF69_MODE_SLEEP);
}

void DavisRFM69::isr0() { selfPointer->interruptHandler(); }

void DavisRFM69::receiveBegin() {
  _packetReceived = false;
  feiPending = false;
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks

  // DIO0 is "PAYLOADREADY" in receive mode, or "RSSI" first for software AFC to time its FEI measurement
  writeReg(REG_DIOMAPPING1, swAfc() ? RF_DIOMAPPING1_DIO0_11 : RF_DIOMAPPING1_DIO0_01);
  setMode(RF69_MODE_RX);
}

bool DavisRFM69::receiveDone() {
  return _packetReceived;
}

void DavisRFM69::setRssiThreshold(int rssiThreshold) {
  setRssiThresholdRaw(abs(rssiThreshold) << 1);
}

void DavisRFM69::setRssiThresholdRaw(int rssiThresholdRaw) {
  writeReg(REG_RSSITHRESH, rssiThresholdRaw);
  rxRssi = rssiThresholdRaw;
}

int DavisRFM69::readRSSI(bool forceTrigger) {
  int rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // Wait for RSSI_Ready
  }
  rssi = -readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

byte DavisRFM69::readReg(byte addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}

void DavisRFM69::writeReg(byte addr, byte value)
{
  select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  unselect();
}

// Select the transceiver
void DavisRFM69::select() {
  noInterrupts();
  digitalWrite(_slaveSelectPin, LOW);
}

// Unselect the transceiver chip
void DavisRFM69::unselect() {
  digitalWrite(_slaveSelectPin, HIGH);
  interrupts();
}

void DavisRFM69::setHighPower(bool onOff) {
  _isRFM69HW = onOff;
  writeReg(REG_OCP, _isRFM69HW ? RF_OCP_OFF : RF_OCP_ON);
  if (_isRFM69HW) // turning ON
    writeReg(REG_PALEVEL, (readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON); // enable P1 & P2 amplifier stages
    else
      writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | _powerLevel); // enable P0 only
}

void DavisRFM69::setHighPowerRegs(bool onOff) {
  writeReg(REG_TESTPA1, onOff ? 0x5D : 0x55);
  writeReg(REG_TESTPA2, onOff ? 0x7C : 0x70);
}

void DavisRFM69::setCS(byte newSPISlaveSelect) {
  _slaveSelectPin = newSPISlaveSelect;
  pinMode(_slaveSelectPin, OUTPUT);
}

void DavisRFM69::readAllRegs()
{
  byte regVal;
  for (byte regAddr = 1; regAddr <= 0x4F; regAddr++)
  {
    select();
    SPI.transfer(regAddr & 0x7f); // send address + r/w bit
    regVal = SPI.transfer(0);
    unselect();

    if (!(regAddr & 0xf0)) Serial.print('0');
    Serial.print(regAddr, HEX);
    Serial.print(" - ");
    if (!(regVal & 0xf0)) Serial.print('0');
    Serial.print(regVal, HEX);
    Serial.print(" - ");
    Serial.println(regVal, BIN);
  }
  unselect();
}

byte DavisRFM69::readTemperature(byte calFactor) // returns centigrade
{
  setMode(RF69_MODE_STANDBY);
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING)) Serial.print('*');
  // 'complement'corrects the slope, rising temp = rising val
  // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction
  return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor;
}

void DavisRFM69::rcCalibration()
{
  writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}

void DavisRFM69::setBand(byte newBand)
{
  if (newBand >= FREQ_BAND_COUNT) return;
  band = newBand;
  discChannel %= getBandTabLength();  // the bands have 2 to 51 channels, keep it inside the new table
}

void DavisRFM69::setBandwidth(byte bw)
{
  switch (bw) {
    case RF69_DAVIS_BW_NARROW:
      writeReg(REG_RXBW,  RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4); // Use 25 kHz BW (BitRate < 2 * RxBw)
      writeReg(REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3); // Use double the bandwidth during AFC as reception
      break;
    case RF69_DAVIS_BW_WIDE:
      // REG_RXBW 50 kHz fixes console retransmit reception but seems worse for SIM transmitters (to be confirmed with more testing)
      writeReg(REG_RXBW,  RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3); // Use 50 kHz BW (BitRate < 2 * RxBw)
      writeReg(REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_2); // Use double the bandwidth during AFC as reception
      break;
    default:
      return;
  }
}

byte DavisRFM69::getBandTabLength()
{
  return bandTabLengths[band];
}

void DavisRFM69::setFreqCorr(int value)
{
  freqCorr = value * 1000.0 / RF69_FSTEP;
}

// RF69_AFC_OFF, RF69_AFC_HW, or software AFC limited to +/- mode kHz (1..RF69_AFC_MAX)
void DavisRFM69::setAfc(byte mode)
{
  if (mode > RF69_AFC_MAX && mode != RF69_AFC_HW) return;
  afcMode = mode;
  writeReg(REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON | (mode == RF69_AFC_HW ? RF_AFCFEI_AFCAUTO_ON : RF_AFCFEI_AFCAUTO_OFF));
}

void DavisRFM69::enableTx(void (*function)(byte* buffer), byte ID)
{
  txCallback = function;
  txId = ID;
  txChannel = 0;
  txDelay = (41 + txId) * timeBase >> 4;
  txAcc = 0;
  txNextTarget = txDelay;
  lastTx = micros();
  Timer1.initialize(txDelay);
  // one Timer1 step in 1/16 us: phase correct mode counts up and down, 2 * prescaler clocks
  static const uint16_t prescalers[] = { 0, 1, 8, 64, 256, 1024, 0, 0 };
  txTick16 = prescalers[TCCR1B & 7] * (32000000UL / (F_CPU / 1000)) / 1000;
  // Timer1 was already running (the Arduino core sets it up for PWM) with its overflow flag pending, which fired
  // a transmission right away: start the count over and clear the flag (1, not 0: TCNT1 = 0 sets it again)
  noInterrupts();
  TCNT1 = 1;
  TIFR1 = _BV(TOV1);
  interrupts();
  Timer1.attachInterrupt(DavisRFM69::handleTxInt, txDelay);
}

void DavisRFM69::disableTx()
{
  Timer1.detachInterrupt();
  txCallback = NULL;
  txId = 255;
  txChannel = 255;
  txDelay = 0;
}

void DavisRFM69::setTimeBase(uint32_t value)
{
  timeBase = value;
}

void DavisRFM69::setTxTiming(bool learn, int16_t ppm)
{
  noInterrupts();
  txLearn = learn;
  txPpmAcc = (int32_t)ppm * 16;
  txPpmSamples = 0;
  txPpmRejects = 0;
  interrupts();
}

uint32_t DavisRFM69::txTimeLeft()
{
  noInterrupts();
  int32_t left = txNextTarget - (micros() - lastTx);
  interrupts();
  return left < 0 ? 0 : left;
}

// Measure this board's clock against the RFM69's crystal. In RX, the RFM69 raises its Timeout flag
// n * 16 bit times after entering RX (RegRxTimeout1), and the bit time comes from the crystal: n * bitrate / 2 us
// at 32 MHz. Returns the micros() from the mode write to the flag, or 0 if the next transmission is due before it
// could finish, or the flag didn't come. The RSSI threshold is 0 dBm, so nothing received stops the timer.
// The start-up delay between the mode write and the timer's start is in the result: take the difference of two
// lengths to cancel it. The receiver must be stopped (stopReceiver()); leaves the radio in standby.
uint32_t DavisRFM69::timeRxTimeout(uint16_t bitrate, byte n)
{
  uint32_t expected = (uint32_t)n * bitrate / 2;
  uint32_t limit = expected + expected / 1000 + 5000;  // the board's clock is within 1000 ppm
  if (rxOn || n == 0) return 0;
  if (txCallback != NULL && txTimeLeft() < limit + 20000) return 0;

  setMode(RF69_MODE_STANDBY);
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);
  if (_isRFM69HW) setHighPowerRegs(false);
  writeReg(REG_BITRATEMSB, bitrate >> 8);
  writeReg(REG_BITRATELSB, bitrate & 0xff);
  writeReg(REG_RSSITHRESH, 0);
  writeReg(REG_RXTIMEOUT2, 0);
  writeReg(REG_RXTIMEOUT1, n);
  byte opmode = readReg(REG_OPMODE) & 0xE3;

  // the time stamps are taken the same way at both ends, with interrupts off, so the latencies cancel
  noInterrupts();
  digitalWrite(_slaveSelectPin, LOW);
  SPI.transfer(REG_OPMODE | 0x80);
  SPI.transfer(opmode | RF_OPMODE_RECEIVER);
  digitalWrite(_slaveSelectPin, HIGH);
  uint32_t t0 = micros();
  interrupts();
  uint32_t t1;
  byte flags;
  do {
    noInterrupts();
    digitalWrite(_slaveSelectPin, LOW);
    SPI.transfer(REG_IRQFLAGS1 & 0x7f);
    flags = SPI.transfer(0);
    digitalWrite(_slaveSelectPin, HIGH);
    t1 = micros();
    interrupts();
  } while (!(flags & RF_IRQFLAGS1_TIMEOUT) && t1 - t0 < limit);

  writeReg(REG_OPMODE, opmode | RF_OPMODE_STANDBY);
  writeReg(REG_RXTIMEOUT1, 0);
  writeReg(REG_BITRATEMSB, RF_BITRATEMSB_19200);
  writeReg(REG_BITRATELSB, RF_BITRATELSB_19200);
  writeReg(REG_RSSITHRESH, rxRssi);
  _mode = RF69_MODE_STANDBY;
  return (flags & RF_IRQFLAGS1_TIMEOUT) ? t1 - t0 : 0;
}

void DavisRFM69::setTxFrame(byte frame)
{
  txFrame = frame == TX_FRAME_DAVIS ? TX_FRAME_DAVIS : TX_FRAME_LEGACY;
}

// append the low n bits of v, MSB first, to the bit buffer f at bit position nb
void DavisRFM69::putBits(byte *f, byte &nb, byte v, byte n)
{
  while (n--) {
    if (v & (1 << n)) f[nb >> 3] |= 0x80 >> (nb & 7);
    nb++;
  }
}

void DavisRFM69::setTxAfc(bool on)
{
  txAfc = on;
  if (!on) txAfcApplied = 0;
}

void DavisRFM69::setRepeaterPT(bool value)
{
  repeaterPT = value;
}
