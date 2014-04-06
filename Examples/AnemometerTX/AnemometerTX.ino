
// ----- Includes -----

#include <DavisRFM69.h>
#include <SPI.h>


// ----- Global Definitions -----

#define POT_PIN A0 // pot input pin
#define POT_VCC 5 // Vcc pin at pot end
#define POT_BLIND_VCC 4 // Vcc pin at additional R after pot for gap detection
#define ANALOG_WAIT_MS 1 // "stabilization" threshold
#define POT_GAP_DEGREES 5 // wind vane potentiometer dead zone, +-N straight at North in the middle
#define ANALOG_ROUNDING 0 // on Leonardo, set it to 1..2, since IO pins don't source exactly Vcc

#define GAP_DETECT_THRESH1 5 // fiddly stuff for gap detection
#define GAP_DETECT_THRESH2 50
#define GAP_DETECT_THRESH3 70

#define DEBOUNCE_THRESH 5000 // 5 ms in micros
#define WIND_DETECT_THRESH 2250000 // 2.25 s in micros
#define WIND_INTERRUPT 1 // int.1 -> pin 3

#define MIN_PULSE_COUNT 10 // below MIN_PULSE_COUNT pulses / TX period, use alternative algorithm
#define WIND_RPS_MPH 2.25  // rps to mph constant by Davis

//#define ORIGINAL_ISS_MPH 1 // dumb transmitter mode, don't interpolate readouts
//#define IS_RFM69HW //uncomment only for RFM69HW! Leave out if you have RFM69W!

#define TX_ID 1 // 0..7, Davis transmitter ID, set to a different value than the console and all other transmitters
                // IMPORTANT: set it ONE LESS than you'd set it on the ISS via the DIP switch; 1 here is 2 on the ISS/Davis console
#define TX_PERIOD (41 + TX_ID) * 1000000 / 16 // TX_PERIOD is a function of the ID and some constants, in micros
                                              // starts at 2.5625 and increments by 0.625 up to 3.0 for every increment in TX_ID

// ----- Constant Definitions -----

// Error correction values for 3 perpendicular wind directions,
// provided by Davis for OEM anemometer installations
static const float windtab[27][3] =
{
//    0°   90/270°  180°
  {  23.3,  17.8,  16.4 },  //  20 mph
  {  28.5,  22.3,  20.4 },  //  25 mph
  {  33.8,  27.1,  25.3 },  //  30 mph
  {  39.2,  31.6,  29.8 },  //  35 mph
  {  44.5,  35.9,  34.3 },  //  40 mph
  {  49.7,  41.2,  40.5 },  //  45 mph
  {  55.0,  45.5,  45.1 },  //  50 mph
  {  60.3,  50.2,  49.8 },  //  55 mph
  {  65.7,  54.7,  54.1 },  //  60 mph
  {  70.8,  59.0,  59.0 },  //  65 mph
  {  76.2,  64.4,  63.9 },  //  70 mph
  {  81.4,  69.0,  68.2 },  //  75 mph
  {  86.8,  73.6,  73.1 },  //  80 mph
  {  92.1,  77.6,  78.2 },  //  85 mph
  {  97.4,  82.0,  83.2 },  //  90 mph
  { 102.5,  86.9,  87.5 },  //  95 mph
  { 107.7,  92.1,  92.8 },  // 100 mph
  { 113.2,  96.9,  97.3 },  // 105 mph
  { 118.5, 101.5, 102.3 },  // 110 mph
  { 123.9, 106.2, 106.5 },  // 115 mph
  { 129.5, 110.6, 111.0 },  // 120 mph
  { 135.0, 115.4, 115.3 },  // 125 mph
  { 139.8, 120.3, 119.7 },  // 130 mph
  { 144.8, 125.0, 124.0 },  // 135 mph
  { 149.3, 129.8, 128.7 },  // 140 mph
  { 154.5, 134.1, 134.5 },  // 145 mph
  { 159.8, 137.9, 138.0 }   // 150 mph
};

// Observed sequence of transmitted ISS value types.
// The upper nibble is important, the lower nibble is the transmitter ID + battery flag.
// Type values for a standard VP2 ISS:
//   0x80 0xe0 0x40 0xa0 0x60 0x50 0x90
//   temp rain uv   rh   sol  unk  unk
// Wind speed and direction is transmitted in every packet at byte offsets 1 and 2.
static const byte txseq[20] = 
{
  0x80, 0xe0, 0x50, 0x40,
  0x80, 0xe0, 0x50, 0x90,
  0x80, 0xe0, 0x50, 0xa0,
  0x80, 0xe0, 0x50, 0xa0,
  0x80, 0xe0, 0x50, 0x60
};

// ----- Global Types and Variable Declarations -----

typedef struct ecpoint {
  float raw, real;
} ecpoint;

volatile unsigned long lastSwitchSensed; // last time the anemometer pulse was sensed, NOT debounced
volatile unsigned long lastPulseStart;   // last anemometer rotation pulse started at, in micros
volatile unsigned long rotationPeriod;   // duration of last measured period, in micros
volatile unsigned long sampleStart;      // first pulse started at, in micros during a TX_PERIOD
volatile int pulseCount;                 // number of sensed rotation pulses during a TX_PERIOD

byte vaneAngleRaw; // angle of the anemometer, raw byte value for the radio packet
byte windSpeed;    // calculated wind speed value in mph, with or without EC applied
int vaneAngle;     // real angle of the anemometer in degrees, used for EC
int rnd = 0;

ecpoint im[2][2]; // 2 row x 2 col ec points as interpolation matrix

DavisRFM69 radio;
unsigned long lastTx;  // last time a wind data radio transmission started
byte seqIndex;         // current packet type index in txseq
char hs[24];
char* hex = "0123456789abcdef";


// ----- Standard Arduino Routines -----

void setup() {
  Serial.begin(19200);
  pinMode(POT_VCC, INPUT); // hi-Z
  pinMode(POT_BLIND_VCC, INPUT); // hi-Z
  lastSwitchSensed = -WIND_DETECT_THRESH;
  rotationPeriod = WIND_DETECT_THRESH;
  lastTx = micros();
  attachInterrupt(WIND_INTERRUPT, windInterrupt, FALLING);
  radio.initialize();
  radio.setTxMode(true); // enable tx params, default is false, call this before the first setChannel()
  radio.setChannel(0); // Frequency / Channel is *not* set in the initialization. Do it right after.
#ifdef IS_RFM69HW
  radio.setHighPower(); // uncomment only for RFM69HW!
#endif
  seqIndex = 0;
  randomSeed(analogRead(A1));
}

void loop() {
  if (micros() - lastTx < TX_PERIOD + rnd) return;
  lastTx = micros();
  //rnd = -random(10000); // generate offset between -10 and +10 ms

  readVaneValue();
  vaneAngle = rawAngleToDegrees(vaneAngleRaw);
  byte mode = 0;
  unsigned long period;

  if (rotationPeriod == 0 || rotationPeriod >= WIND_DETECT_THRESH || lastTx > lastPulseStart + WIND_DETECT_THRESH) {
    rotationPeriod = 0;
    period = 0;
    windSpeed = 0;
  } else {
    if (pulseCount < MIN_PULSE_COUNT) {
      period = rotationPeriod;
#ifdef ORIGINAL_ISS_MPH
      windSpeed = calcWindSpeed(period, vaneAngle);
#else
      windSpeed = calcWindSpeedEC(period, vaneAngle);
#endif
      mode = 1;
    } else {
      period = (lastPulseStart - sampleStart) / (pulseCount - 1);
#ifdef ORIGINAL_ISS_MPH
      windSpeed = calcWindSpeed(period, vaneAngle);
#else
      windSpeed = calcWindSpeedEC(period, vaneAngle);
#endif
      mode = 2;
    }
  }
  
  byte oldsi = seqIndex;
  byte oldchan = radio.CHANNEL;

  sendRadioPacket();
  packetToHex();
  
  Serial.print("angle: ");
  Serial.print(vaneAngle);
  Serial.print("\tspeed");
  Serial.print(mode);
  Serial.print(": ");
  Serial.print(windSpeed);
  Serial.print("\tcount: ");
  Serial.print(pulseCount);
  Serial.print("\tperiod: ");
  Serial.print(period);
  Serial.print("\ttxseq: ");
  Serial.print(oldsi);
  Serial.print("\tchan: ");
  Serial.print(oldchan);
  Serial.print("\tpacket: ");
  Serial.println(hs);

  pulseCount = 0;
  sampleStart = 0;
}


// ----- Main Operative Functions -----

// Read wind vane direction as raw byte value
void readVaneValue() {
  long val, blindVal;

  pinMode(POT_BLIND_VCC, OUTPUT);
  digitalWrite(POT_BLIND_VCC, HIGH);
  delay(ANALOG_WAIT_MS); // stabilize output
  blindVal = analogRead(POT_PIN);
  pinMode(POT_VCC, OUTPUT);
  digitalWrite(POT_VCC, HIGH);
  pinMode(POT_BLIND_VCC, INPUT);
  delay(ANALOG_WAIT_MS);
  val = (analogReadAvg(POT_PIN, 4) + ANALOG_ROUNDING) >> 2;
  pinMode(POT_VCC, INPUT);
  if ( (blindVal - val > GAP_DETECT_THRESH3) || (val < GAP_DETECT_THRESH1 && blindVal > GAP_DETECT_THRESH2) ) {
    val = 0;
  }
  vaneAngleRaw = val;
}

// Read wind vane direction, normalized to degrees and accounting for the dead zone of the vane potmeter
int rawAngleToDegrees(long val) {
  if (val > 0) {
    return POT_GAP_DEGREES - 1 + (361 - 2 * POT_GAP_DEGREES) * val / 255;
  } else {
    return 0;
  }
}

// Dumb wind speed routine. Calculates wind speed using the original ISS formula
int calcWindSpeed(unsigned long period, int angle) {
  return round(1000000.0 / period * WIND_RPS_MPH); // (1 sec) / (period micros) * (majic constant by Davis)
}

// Normalize and interpolate raw wind figure using Davis' OEM calibration data
int calcWindSpeedEC(unsigned long period, int angle) {

  int eccolbase;
  if (angle > 180) angle = 360 - angle; // EC is symmetric between E/W
  eccolbase = (angle - 1) / 90; // avoiding oob errors; we just need the 2 fixed perp. angle column indices
  angle = angle == 180 ? 90 : angle % 90; // difference is always 90 between EC points, we only need the angle between them
                                          // 180 is special because EC values aren't equivalent at 0° and 180°

  float mph = 1000000.0 / period * WIND_RPS_MPH; // (1 sec) / (period micros) * (majic constant by Davis)

  // Find the raw EC values in the table for the raw wind speed in mph
  // for the 2 perpendicular angles above and below the vane angle.
  // Then we have im[][] filled for interpolation
  for (byte icol = 0; icol < 2; icol++) {
    if (mph <= windtab[0][eccolbase + icol]) { // no EC for values below 20 mph
        im[0][icol].raw = 1.0;
        im[0][icol].real = 1.0;
        im[1][icol].raw = windtab[0][eccolbase + icol];
        im[1][icol].real = 20.0;
    } else if (mph >= windtab[sizeof(windtab) - 1][eccolbase + icol]) { // no EC for values above 150 mph
        im[0][icol].raw = 150;
        im[0][icol].real = 150;
        im[1][icol].raw = 150;
        im[1][icol].real = 150;
    } else { // find EC row for raw value for current angle column
        byte i;
        for (i = 0; mph > windtab[i][eccolbase + icol]; i++);
        im[0][icol].raw = windtab[i - 1][eccolbase + icol];
        im[0][icol].real = 15 + i * 5;
        im[1][icol].raw = windtab[i][eccolbase + icol];
        im[1][icol].real = 20 + i * 5;
    }
  }

  return interpolate(mph, angle);
}

// Simple bilinear interpolation for a point in a quad represented by its 4 "corners" in im[][]
// assumption: angle is always between 0 and 90
int interpolate(float mph, int angle) {
  float mph1, mph2;

  if (im[0][0].raw == im[1][0].raw) {
    mph1 = im[0][0].real;
  } else {
    mph1 = im[0][0].real + (im[1][0].real - im[0][0].real) * (mph - im[0][0].raw) / (im[1][0].raw - im[0][0].raw);
  }
  if (im[0][1].raw == im[1][1].raw) {
    mph2 = im[0][1].real;
  } else {
    mph2 = im[0][1].real + (im[1][1].real - im[0][1].real) * (mph - im[0][1].raw) / (im[1][1].raw - im[0][1].raw);
  }

//  Serial.print("mph: ");
//  Serial.println(mph);
//
//  Serial.print(mph1);
//  Serial.print("\t");
//  Serial.println(mph2);
//
//  Serial.print(im[0][0].raw);
//  Serial.print("\t");
//  Serial.println(im[0][1].raw);
//  Serial.print(im[1][0].raw);
//  Serial.print("\t");
//  Serial.println(im[1][1].raw);
//
//  Serial.println();
//
//  Serial.print(im[0][0].real);
//  Serial.print("\t");
//  Serial.println(im[0][1].real);
//  Serial.print(im[1][0].real);
//  Serial.print("\t");
//  Serial.println(im[1][1].real);

  float ecmph = mph1 + angle / 90.0 * (mph2 - mph1);
  return round(ecmph);
}

// Send out radio packet containing the wind data and the transmitter ID.
// Every packet contains dummy data on other sensors in the proper transmit sequence.
void sendRadioPacket() {
  radio.DATA[0] = txseq[seqIndex] | TX_ID;
  if (++seqIndex >= sizeof(txseq)) seqIndex = 0;
  radio.DATA[1] = windSpeed;
  radio.DATA[2] = vaneAngleRaw;
  radio.DATA[3] = radio.DATA[4] = radio.DATA[5] = 0;
  unsigned int crc = radio.crc16_ccitt(radio.DATA, DAVIS_PACKET_LEN - 2);
  radio.send((const void*)radio.DATA, DAVIS_PACKET_LEN - 2);  
  radio.hop();
  //radio.setChannel(0);
}


// ----- Interrupt Routines -----

// The anemometer reed relay causes this interrupt to be raised at every revolution.
void windInterrupt(void) {
  unsigned long t = micros();
  if (lastPulseStart > 0 && (t - lastSwitchSensed) > DEBOUNCE_THRESH) {
    rotationPeriod = t - lastPulseStart;
    lastPulseStart = t;
    pulseCount++;
  }
  lastSwitchSensed = t;
  if (lastPulseStart == 0) lastPulseStart = t;
  if (sampleStart == 0) sampleStart = t;
}


// ----- Utility Functions -----

int analogReadAvg(byte port, byte n) {
  long val = 0;
  for (byte i = 0; i < n; i++) val += analogRead(port);
  val /= n;
  return val;
}

void packetToHex() {
  hs[23] = 0;
  int x = 0;
  for (byte i = 0; i < 8; i++) {
    hs[x++] = hex[radio.DATA[i] >> 4];
    hs[x++] = hex[radio.DATA[i] & 0x0f];
    if (i < 7) hs[x++] = ' ';
  }
}

