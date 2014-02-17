// Driver implementation for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
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

#include <DavisRFM69.h>
#include <../RFM69/RFM69registers.h>
#include <SPI.h>

volatile byte DavisRFM69::hopIndex = 0;

bool DavisRFM69::initialize(byte freqBand, byte nodeID, byte networkID)
{
  const byte CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_10 }, // Davis uses Gaussian shaping with BT=0.5
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_19200}, // Davis uses a datarate of 19.2 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_19200},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_4800}, // Davis uses a deviation of 4.8 kHz
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_4800},
    /* 0x07 to 0x09 are REG_FRFMSB to LSB. No sense setting them here. Done in main routine.
    /* 0x0B */ { REG_AFCCTRL, RF_AFCLOWBETA_OFF }, // TODO: Should use LOWBETA_ON, but having trouble getting it working
    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout=-18+OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout=-14+OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout=-11+OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, //over current protection (default is 95mA)
    /* 0x18 */ { REG_LNA, RF_LNA_ZIN_50 | RF_LNA_GAINSELECT_AUTO}, // Not sure which is correct!
    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4khz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4 }, // Use 25 kHz BW (BitRate < 2 * RxBw)
    /* 0x1A */ { REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3 }, // Use double the bandwidth during AFC as reception
    /* 0x1B - 0x1D These registers are for OOK.  Not used */
    /* 0x1E */ { REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_OFF | RF_AFCFEI_AFCAUTO_ON },
    /* 0x1F & 0x20 AFC MSB and LSB values, respectively */
    /* 0x21 & 0x22 FEI MSB and LSB values, respectively */
    /* 0x23 & 0x24 RSSI MSB and LSB values, respectively */
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, //DIO0 is the only IRQ we're using
    /* 0x26 RegDioMapping2 */
    /* 0x27 RegIRQFlags1 */
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // Reset the FIFOs. Fixes a problem I had with bad first packet.
    /* 0x29 */ { REG_RSSITHRESH, 200 }, //must be set to dBm = (-Sensitivity / 2) - default is 0xE4=228 so -114dBm
    /* 0x2a & 0x2b RegRxTimeout1 and 2, respectively */
    /* 0x2c RegPreambleMsb - use zero default */
    /* 0x2d */ { REG_PREAMBLELSB, 4 }, // Davis has four preamble bytes 0xAAAAAAAA
    /* 0x2e */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_2 },  // Allow a couple erros in the sync word
    /* 0x2f */ { REG_SYNCVALUE1, 0xcb }, // Davis ISS first sync byte. http://madscientistlabs.blogspot.ca/2012/03/first-you-get-sugar.html
    /* 0x30 */ { REG_SYNCVALUE2, 0x89 }, // Davis ISS second sync byte.
    /* 0x31 - 0x36  REG_SYNCVALUE3 - 8 not used */
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_OFF | RF_PACKET1_ADRSFILTERING_OFF }, // Fixed packet length and we'll check our own CRC
    /* 0x38 */ { REG_PAYLOADLENGTH, 8 }, // Davis sends 8 bytes of payload, including CRC that we check manually.
    //* 0x39 */ { REG_NODEADRS, nodeID }, // Turned off because we're not using address filtering
    //* 0x3a */ { REG_BROADCASTADRS, RF_BROADCASTADDRESS_VALUE }, // Not using this
    /* 0x3b REG_AUTOMODES - Automatic modes are not used in this implementation. */
    /* 0x3c */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | 0x07 }, // TX on FIFO having more than seven bytes
    /* 0x3d */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x3e - 0x4d  AES Key not used in this implementation */
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // // TODO: Should use LOWBETA_ON, but having trouble getting it working
    /* 0x71 */ { REG_TESTAFC, 0 }, // AFC Offset for low mod index systems
    {255, 0}
  };

  pinMode(_slaveSelectPin, OUTPUT);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2); //max speed, except on Due which can run at system clock speed
  SPI.begin();
  
  do writeReg(REG_SYNCVALUE1, 0xaa); while (readReg(REG_SYNCVALUE1) != 0xaa);
	do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55);
  
  for (byte i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);

  encrypt(0);		      // Disable encryption to get to a known state between resets

  setHighPower(_isRFM69HW); //called regardless if it's a RFM69W or RFM69HW
  setMode(RF69_MODE_STANDBY);
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
  attachInterrupt(0, RFM69::isr0, RISING);
  
  selfPointer = this;
  _address = 0; // Was nodeID in original implementation.  Not used here.
  return true;
}

void DavisRFM69::interruptHandler() {
  if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    setMode(RF69_MODE_STANDBY);
    select();
    SPI.transfer(REG_FIFO & 0x7f);
    PAYLOADLEN = 8;
    DATALEN = 8;
    
    for (byte i= 0; i < DATALEN; i++)
    {
      DATA[i] = reverseBits(SPI.transfer(0));
    }
    unselect();
    setMode(RF69_MODE_RX);
  }
  RSSI = readRSSI();
}

void DavisRFM69::send(byte toAddress, const void* buffer, byte bufferSize, bool requestACK)
{
  writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  while (!canSend()) receiveDone();
  sendFrame(toAddress, buffer, bufferSize, requestACK, false);
}

void DavisRFM69::sendFrame(byte toAddress, const void* buffer, byte bufferSize, bool requestACK, bool sendACK)
{
  setMode(RF69_MODE_STANDBY); //turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  if (bufferSize > MAX_DATA_LEN) bufferSize = MAX_DATA_LEN;

  unsigned int crc = crc16_ccitt((volatile byte *)buffer, 6);
  //write to FIFO
  select();
  SPI.transfer(REG_FIFO | 0x80);
  
  for (byte i = 0; i < bufferSize; i++)
    SPI.transfer(reverseBits(((byte*)buffer)[i]));

  SPI.transfer(reverseBits(crc >> 8));
  SPI.transfer(reverseBits(crc & 0xff));
  unselect();

  /* no need to wait for transmit mode to be ready since its handled by the radio */
  setMode(RF69_MODE_TX);
  while (digitalRead(_interruptPin) == 0); //wait for DIO0 to turn HIGH signalling transmission finish
  //while (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT == 0x00); // Wait for ModeReady
  setMode(RF69_MODE_STANDBY);
}

void DavisRFM69::setChannel(byte channel)
{
  hopIndex = channel;
  writeReg(REG_FRFMSB, pgm_read_byte(&FRF[channel][0]));
  writeReg(REG_FRFMID, pgm_read_byte(&FRF[channel][1]));
  writeReg(REG_FRFLSB, pgm_read_byte(&FRF[channel][2]));
}

void DavisRFM69::hop()
{
  hopIndex++;
  if (hopIndex > 50) 
  {
    hopIndex = 0;
  }
  setChannel(hopIndex);
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
unsigned int DavisRFM69::crc16_ccitt(volatile byte *buf, byte len)
{
  unsigned int crc = 0;
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
