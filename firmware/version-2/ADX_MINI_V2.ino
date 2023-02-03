
// ============================================================================
//
// ADX_MINI_V2.ino  ::  ADX-MINI Control Program
//
// This control program was derived from the ADX control program.
// It is compatible with ADX, ADX-UNO, and ADX-MINI hardware.
// The user interface code has been modified to encode the band selection
// Supported bands are: 160m/80m/60m/40m/30m/20m/17m/15m/12m/10m/6m
// The user interface is described in a separate document.
//
// Libraries
// ---------
// Arduino Wire.h   I2C library - built-into Arduino IDE
// Arduino EEPROM.h library     - built-into Arduino IDE
// Si5351 library               - by Milldrum and Myers
//
// Acknowledgement
// ---------------
// The Arduino Digital Transceiver (ADX) hardware was designed by WB2CBA
// The ADX is a simple QRP radio in both its hardware implmentation and
// its user interface (3 buttons and 5 LEDs :) Yet it is highly capable
// when paired with the WSJT-X software.  Thanks Barb !!
//
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// ============================================================================

#define DEBUG     0                 // set to 1 for debugging
#define VERSION   "ADX-MINI 1.01a"  // firmware version
#define DATE      "Jan 16 2023"     // firmware date

// Arduino Pins
#define LTSW      2     // PD2   UI pushbutton <
#define RTSW      3     // PD3   UI pushbutton >
#define TXSW      4     // PD4   UI pushbutton TX
#define TXXLED   13     // PB5   TX   status LED
#define WSPLED    9     // PB1   WSPR status LED
#define JS8LED   10     // PB2   JS8  status LED
#define FT4LED   11     // PB3   FT4  status LED
#define FT8LED   12     // PB4   FT8  status LED
#define RXGATE    8     // PB0   RX Gate
#define ACMP      7     // PD7   analog comparator

// generic
#define OFF   0
#define ON    1
#define NO    0
#define YES   1
#define FALSE 0
#define TRUE  1

#include <Wire.h>
#include <EEPROM.h>
#include "si5351.h"

// prototype defs

void print_version();
void wait_ms(uint16_t dly);
void clrLED();
void blinkLED(uint8_t pin, uint8_t x);
void blinkTX();
void setLED(uint8_t data, uint8_t band);
void blink_band();
void blink_mode();
void mode_assign();
void band_assign();
void band_select();
void manualTX();
void si5351_cal();
void FSK_tone();
void check_VOX();
void check_UI();
void save_eeprom();
void read_eeprom();
void init_VFO();
void init_timer();
void initPins();

// eeprom addresses
#define DATA_ADDR    10
#define TEMP_ADDR    30
#define MODE_ADDR    40
#define SLOT_ADDR    50

// delay times (ms)
#define DLY1        250
#define DEBOUNCE     20

#define MS_LONG     300  // long press
uint32_t t0;             // event timer

// read pushbutton switches
#define LT_PRESSED  !digitalRead(LTSW)
#define RT_PRESSED  !digitalRead(RTSW)
#define TX_PRESSED  !digitalRead(TXSW)

uint32_t freq;
uint32_t base_freq;
uint32_t F_FT8;
uint32_t F_FT4;
uint32_t F_JS8;
uint32_t F_WSP;

// button events
#define NONE       0x00  // no event
#define CLICK      0x01  // click
#define LONG       0x02  // long press

uint8_t event = NONE;

// Si5351 xtal frequency (25 MHz)
#define SI5351_REF  25000000UL

// band groups
#define GROUP3  0xc
#define GROUP2  0x8
#define GROUP1  0x4
#define GROUP0  0x0

// band within group
#define XBAND3  0x3
#define XBAND2  0x2
#define XBAND1  0x1
#define XBAND0  0x0

#define BAND160  11
#define BAND80   10
#define BAND60    9
#define BAND40    8
#define BAND30    7
#define BAND20    6
#define BAND17    5
#define BAND15    4
#define BAND12    3
#define BAND10    2
#define BAND06    1

// encoded band LEDs
#define ENC160m GROUP3|XBAND3
#define ENC80m  GROUP3|XBAND2
#define ENC60m  GROUP3|XBAND1
#define ENC40m  GROUP3|XBAND0
#define ENC30m  GROUP2|XBAND3
#define ENC20m  GROUP2|XBAND2
#define ENC17m  GROUP2|XBAND1
#define ENC15m  GROUP1|XBAND3
#define ENC12m  GROUP1|XBAND2
#define ENC10m  GROUP1|XBAND1
#define ENC6m   GROUP0|XBAND3

// encoded bands
const int8_t bandENC[11] = {
 ENC6m,  ENC10m, ENC12m, ENC15m, ENC17m,
 ENC20m, ENC30m, ENC40m, ENC60m, ENC80m,
 ENC160m };

// calibration data
#define CAL_DATA_INIT  64000ULL
uint32_t cal_data = CAL_DATA_INIT;

// FSK transmit status
uint8_t FSKtx = FALSE;

// data valid status
uint8_t d2ICR = FALSE;

// CPU clock frequency
#define CPUXTL  1600000000ULL

#define MAX_MODE 8
#define MIN_MODE 1
#define MAX_SLOT 11
#define MIN_SLOT 1

#define WSP_MODE 8
#define JS8_MODE 4
#define FT4_MODE 2
#define FT8_MODE 1

uint8_t mode      = FT8_MODE;
uint8_t band_slot = BAND20;

Si5351 si5351;

// print the firmware version
void print_version() {
  Serial.print("\n\n");
  Serial.println(VERSION);
  Serial.println(DATE);
  Serial.print("\n\n");
}

// interruptable delay
void wait_ms(uint16_t dly) {
  uint32_t curTime = millis();
  uint32_t endTime = curTime + dly;
  while(curTime < endTime) {
    delayMicroseconds(100);
    curTime = millis();
  }
}

// wait for all buttons released
void wait4buttons() {
  while ((RT_PRESSED)||(LT_PRESSED)||(TX_PRESSED));
  delay(DEBOUNCE);
}

// clear all mode LEDs
void clrLED() {
  digitalWrite(FT8LED, OFF);
  digitalWrite(FT4LED, OFF);
  digitalWrite(JS8LED, OFF);
  digitalWrite(WSPLED, OFF);
}

// blink an LED
void blinkLED(uint8_t pin, uint8_t x) {
  uint8_t state=1;
  for (uint8_t i=0; i<(x<<1); i++) {
    digitalWrite(pin, state);
    state = !state;
    delay(DLY1);
  }
}

// blink the TX LED
void blinkTX() {
  clrLED();
  blinkLED(TXXLED, 2);
}

// set LEDs with data
void setLED(uint8_t data, uint8_t band) {
  if (band) data = bandENC[band-1];
  digitalWrite(FT8LED, bitRead(data, 0));
  digitalWrite(FT4LED, bitRead(data, 1));
  digitalWrite(JS8LED, bitRead(data, 2));
  digitalWrite(WSPLED, bitRead(data, 3));
}

// blink the selected band
void blink_band() {
  for (uint8_t i=0; i<2; i++) {
    clrLED();
    delay(DLY1);
    setLED(0, band_slot);
    delay(DLY1);
  }
}

// blink the selected mode
void blink_mode() {
  for (uint8_t i=0; i<2; i++) {
    clrLED();
    delay(DLY1);
    setLED(mode, 0);
    delay(DLY1);
  }
}

// mode assignments
void mode_assign() {
  switch (mode) {
    case WSP_MODE:
      base_freq = F_WSP;
      break;
    case JS8_MODE:
      base_freq = F_JS8;
      break;
    case FT4_MODE:
      base_freq = F_FT4;
      break;
    case FT8_MODE:
      base_freq = F_FT8;
      break;
    default:
      break;
  }
  freq = base_freq;
}

// band frequency assignments
void band_assign() {
  switch (band_slot) {
    case BAND160:
      // 160m/1.75Mhz
      F_FT8 = 3573000;
      F_FT4 = 3575000;
      F_JS8 = 3578000;
      F_WSP = 3568600;
      break;
    case BAND80:
      // 80m/3.5Mhz
      F_FT8 = 3573000;
      F_FT4 = 3575000;
      F_JS8 = 3578000;
      F_WSP = 3568600;
      break;
    case BAND60:
      // 60m/5 Mhz
      /*
      F_FT8 =
      F_FT4 =
      F_JS8 =
      F_WSP =
      */
      break;
    case BAND40:
      // 40m/7 Mhz
      F_FT8 = 7074000;
      F_FT4 = 7047500;
      F_JS8 = 7078000;
      F_WSP = 7038600;
      break;
    case BAND30:
      // 30m/10 Mhz
      F_FT8 = 10136000;
      F_FT4 = 10140000;
      F_JS8 = 10130000;
      F_WSP = 10138700;
      break;
    case BAND20:
      // 20m/14 Mhz
      F_FT8 = 14074000;
      F_FT4 = 14080000;
      F_JS8 = 14078000;
      F_WSP = 14095600;
      break;
    case BAND17:
      // 17m/18 Mhz
      F_FT8 = 18100000;
      F_FT4 = 18104000;
      F_JS8 = 18104000;
      F_WSP = 18104600;
      break;
    case BAND15:
      // 15m/21Mhz
      F_FT8 = 21074000;
      F_FT4 = 21140000;
      F_JS8 = 21078000;
      F_WSP = 21094600;
      break;
    case BAND12:
      // 12m/24Mhz
      F_FT8 = 24915000;
      F_FT4 = 24919000;
      F_JS8 = 24922000;
      F_WSP = 24924600;
      break;
    case BAND10:
      // 10m/28Mhz
      F_FT8 = 28074000;
      F_FT4 = 28180000;
      F_JS8 = 28078000;
      F_WSP = 28124600;
      break;
    case BAND06:
      // 6m/48Mhz
      /*
      F_FT8 =
      F_FT4 =
      F_JS8 =
      F_WSP =
      */
      break;
  }
  mode_assign();
}

// band select
void band_select() {
  bool done = FALSE;
  blink_band();
  while ((LT_PRESSED)||(RT_PRESSED));  // wait for release
  delay(DEBOUNCE);
  while (!done) {
    // check the < pushbutton
    if (LT_PRESSED) {
      t0 = millis();
      event = CLICK;
      // wait for release
      while (LT_PRESSED) {
        // check for long press
        if ((millis() - t0) > MS_LONG) { event = LONG; break; }
        wait_ms(1);
      }
      delay(DEBOUNCE);
      if (event == LONG) {
        // for long press of <
        // exit band-select mode
        done = TRUE;
        blink_mode();
      } else {
        // for short click of <
        // decrement the band slot
        band_slot = band_slot - 1;
        if (band_slot < MIN_SLOT) band_slot = MAX_SLOT;
        setLED(0, band_slot);
        band_assign();
      }
      while (LT_PRESSED);  // wait for release
      delay(DEBOUNCE);
    }
    // check the > pushbutton
    if (RT_PRESSED) {
      t0 = millis();
      event = CLICK;
      // wait for release
      while (RT_PRESSED) {
        // check for long press
        if ((millis() - t0) > MS_LONG) { event = LONG; break; }
        wait_ms(1);
      }
      delay(DEBOUNCE);
      if (event == LONG) {
        // for long press of <
        // exit band-select mode
        done = TRUE;
        blink_mode();
      } else {
        // for short click of <
        // increment the band slot
        band_slot = band_slot + 1;
        if (band_slot > MAX_SLOT) band_slot = MIN_SLOT;
        setLED(0, band_slot);
        band_assign();
      }
      while (RT_PRESSED);  // wait for release
      delay(DEBOUNCE);
    }
    // check the tx pushbutton
    if (TX_PRESSED) {
      t0 = millis();
      event = CLICK;
      // wait for release
      while (TX_PRESSED) {
        // check for long press
        if ((millis() - t0) > MS_LONG) { event = LONG; break; }
        wait_ms(1);
      }
      if (event == LONG) {
        // for long press of TX
        // manual TX
        clrLED();
        manualTX();
        setLED(0, band_slot);
      } else {
        // for short click of TX
        // save the band slot to eeprom
        blinkTX();
        EEPROM.put(SLOT_ADDR, band_slot);
        while (TX_PRESSED);  // wait for release
        delay(DEBOUNCE);
        // exit band-select mode and
        // return to mode-select mode
        done = TRUE;
        blink_mode();
      }
    }
  }
}

// manual TX
void manualTX() {
  digitalWrite(RXGATE, OFF);
  si5351.output_enable(SI5351_CLK1, 0);   // RX off
  digitalWrite(TXXLED, ON);
  si5351.set_freq(base_freq*100, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 1);   // TX on
  while (TX_PRESSED);  // wait for release
  delay(DEBOUNCE);
  digitalWrite(TXXLED, OFF);
  si5351.output_enable(SI5351_CLK0, 0);   // TX off
}

// si5351 calibration
void si5351_cal() {
  uint32_t cal_freq = 1000000UL;
  bool done = FALSE;
  setLED(0b1111, 0);   // set all LEDs while button pressed
  wait4buttons();      // wait for all buttons released
  setLED(0b1001, 0);   // set LEDs to indicate cal mode
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
  si5351.set_freq(cal_freq*100, SI5351_CLK2);
  si5351.set_clock_pwr(SI5351_CLK2, 1);
  si5351.output_enable(SI5351_CLK2, 1);
  while (!done) {
    // check the tx pushbutton
    if (TX_PRESSED) {
      // if TX pressed then
      // save the calibration data
      // and exit calibration mode
      done = TRUE;
      clrLED();            // clear all LEDs
      while (TX_PRESSED);  // wait for release
      delay(DEBOUNCE);
    }
    // check the > pushbutton
    else if (RT_PRESSED) {
      cal_data = cal_data - 100;
      si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
      si5351.set_freq(cal_freq*100, SI5351_CLK2);
      delay(DEBOUNCE);
    }
    // check the < pushbutton
    else if (LT_PRESSED) {
      cal_data = cal_data + 100;
      si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
      si5351.set_freq(cal_freq*100, SI5351_CLK2);
      delay(DEBOUNCE);
    }
  }
  si5351.output_enable(SI5351_CLK2, 0);
  si5351.set_clock_pwr(SI5351_CLK2, 0);
  EEPROM.put(DATA_ADDR, cal_data);
  blinkTX();  // blink TX LED when done
}

#define MAXVOX  15     // VOX timeout (ms)
uint32_t vox_timer;

#define MAXCNT  64000  // max timer1 count

// FSK zero-crossing frequency detection
void FSK_tone() {
  static uint16_t d1,d2;
  // check if d2 is valid
  if (!d2ICR) {
    d2 = ICR1;
    d2ICR = TRUE;
    return;
  }
  d1 = d2;
  d2 = ICR1;
  TIFR1 = (1<<ICF1);            // write one to clear
  uint16_t delta = d2 - d1;     // captured event period
  if (delta < MAXCNT) {         // check for valid period
    if (!FSKtx) {
      clrLED();
      digitalWrite(TXXLED, ON);
      digitalWrite(RXGATE, OFF);
      si5351.output_enable(SI5351_CLK1, 0);   // RX off
      si5351.output_enable(SI5351_CLK0, 1);   // TX on
      FSKtx = TRUE;
    }
    uint32_t codefreq = CPUXTL/delta;
    si5351.set_freq(((freq*100) + codefreq), SI5351_CLK0);
  }
  vox_timer = millis();  // reset the vox timer
}

// check for VOX timeout
void check_VOX() {
  if (FSKtx) {
    if (millis() - vox_timer > MAXVOX) {
      // if timeout then return to RX
      digitalWrite(TXXLED, OFF);
      setLED(mode, 0);
      si5351.output_enable(SI5351_CLK0, 0);   //TX off
      si5351.set_freq(freq*100, SI5351_CLK1);
      si5351.output_enable(SI5351_CLK1, 1);   //RX on
      digitalWrite(RXGATE, ON);
      FSKtx = FALSE;
      d2ICR = FALSE;
    }
  }
}

// check the UI buttons
void check_UI() {
  // check the < pushbutton
  if (LT_PRESSED) {
    t0 = millis();
    event = CLICK;
    // wait for release
    while (LT_PRESSED) {
      // check for long press
      if ((millis() - t0) > MS_LONG) { event = LONG; break; }
      wait_ms(1);
    }
    if (event == LONG) {
      // for long press of <
      // goto band-select mode
      band_select();
    } else {
      // for short click of <
      // left-shift the radio mode
      mode = mode<<1;
      if (mode > MAX_MODE) mode = MIN_MODE; // wrap
      mode_assign();
      setLED(mode, 0);
    }
    while (LT_PRESSED);  // wait for release
    delay(DEBOUNCE);
  }
  // check the > pushbutton
  if (RT_PRESSED) {
    mode = mode>>1;
    if (mode < MIN_MODE) mode = MAX_MODE; // wrap
    mode_assign();
    setLED(mode, 0);
    while (RT_PRESSED);  // wait for release
    delay(DEBOUNCE);
  }
  // check the tx pushbutton
  if (TX_PRESSED) {
    t0 = millis();
    event = CLICK;
    // wait for release
    while (TX_PRESSED) {
      // check for long press
      if ((millis() - t0) > MS_LONG) { event = LONG; break; }
      wait_ms(1);
    }
    if (event == LONG) {
      // for long press of TX
      // manual TX
      clrLED();
      manualTX();
      setLED(mode, 0);
    } else {
      // for short click of TX
      // save the radio mode
      blinkTX();
      setLED(mode, 0);
      EEPROM.put(MODE_ADDR, mode);
      // wait for release
      while (TX_PRESSED);
      delay(DEBOUNCE);
    }
  }
}

// write to the eeprom
void save_eeprom() {
  EEPROM.put(DATA_ADDR, cal_data);
  EEPROM.put(MODE_ADDR, mode);
  EEPROM.put(SLOT_ADDR, band_slot);
}

// read the eeprom
void read_eeprom() {
  while (LT_PRESSED);  // wait for release
  delay(DEBOUNCE);
  clrLED();
  EEPROM.get(DATA_ADDR, cal_data);
  EEPROM.get(MODE_ADDR, mode);
  EEPROM.get(SLOT_ADDR, band_slot);
  if ((cal_data < 100) || (cal_data > 100000)) {
    cal_data = CAL_DATA_INIT;
    EEPROM.put(DATA_ADDR, cal_data);
  }
  if ((mode < MIN_MODE) || (mode > MAX_MODE)) {
    mode = MAX_MODE;
    EEPROM.put(MODE_ADDR, mode);
  }
  if ((band_slot < MIN_SLOT) || (band_slot > MAX_SLOT)) {
    band_slot = MIN_SLOT;
    EEPROM.put(SLOT_ADDR, band_slot);
  }
}

// initialize the Set Si5351 VFO
void init_VFO() {
  si5351.init();
  si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
}

// initialize the timer
void init_timer() {
  TCCR1A = 0x00;
  TCCR1B = 0x81;      // falling edge capture + noise canceller
  ACSR |= (1<<ACIC);  // analog comparator input capture
  TIFR1 = (1<<ICF1);  // write one to clear
}

// initialize pins
void initPins() {
  pinMode(LTSW,   INPUT);
  pinMode(RTSW,   INPUT);
  pinMode(TXSW,   INPUT);
  pinMode(TXXLED, OUTPUT);
  pinMode(WSPLED, OUTPUT);
  pinMode(JS8LED, OUTPUT);
  pinMode(FT4LED, OUTPUT);
  pinMode(FT8LED, OUTPUT);
  pinMode(RXGATE, OUTPUT);
  pinMode(ACMP,   INPUT);
  digitalWrite(TXXLED, OFF);
  digitalWrite(WSPLED, OFF);
  digitalWrite(JS8LED, OFF);
  digitalWrite(FT4LED, OFF);
  digitalWrite(FT8LED, OFF);
  digitalWrite(RXGATE, OFF);
}

#define BAUD  115200

// Arduino setup function
void setup() {
  initPins();
  Serial.begin(BAUD);
  print_version();
  // if < button active then read eeprom
  // if > button active then calibrate
  if (!LT_PRESSED) read_eeprom();
  else save_eeprom();    // factory reset
  init_VFO();
  if (RT_PRESSED) si5351_cal();
  init_timer();
  band_assign();
  blink_band();          // blink band on LEDs
  delay(500);
  blink_mode();          // blink mode on LEDs
  wait4buttons();        // wait for all buttons released
}

#define EVENT (TIFR1 & (1<<ICF1))  // capture event

// Arduino main loop
void loop() {
  if (EVENT) FSK_tone();   // check for FSK tone input
  if (FSKtx) check_VOX();  // check for VOX timeout
  else check_UI();         // check UI buttons
}

