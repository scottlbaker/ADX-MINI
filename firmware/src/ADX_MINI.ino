
// ============================================================================
//
// ADX_MINI.ino  ::  ADX-MINI Control Program with CAT control
//
// This control program was derived from the ADX control program.
// It is compatible with ADX, ADX-UNO, and ADX-MINI hardware.
// Supported bands are: 160M/80M/60M/40M/30M/20M/17M/15M/12M/10M/6M
// The user interface is described in a separate document.
//
// (c) Scott Baker KJ7NLA
//
//  Arduino IDE settings:
//  board: Arduino UNO
//  bootloader: no bootloader
//  programmer: AVRISP mkII
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
// its user interface. Yet it is highly capable when paired with
// the WSJT-X software.  Thanks Barb !!
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

#define DEBUG1    0                     // set to 1 for debug level 1
#define DEBUG2    0                     // set to 1 for debug level 2
#define VERSION   "ADX-MINI 1.04d"      // firmware version
#define DATE      "Jul 29 2023"         // firmware date
#define NOTE1     "with CAT support"    // firmware notes

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
#define RX    0
#define TX    1
#define FALSE 0
#define TRUE  1
#define RIGHT 0
#define LEFT  1

#include <Wire.h>
#include <EEPROM.h>
#include "si5351.h"

// string prototype defs
char getc();
char gnac();
void getsemi();
char gcal(char ch);
uint8_t len(char *str);
uint8_t cmpstr(char *dst, char *src);
uint8_t alpha(char ch);
uint8_t numeric(char ch);
void catstr(char *dst, char c);
void uppercase(char *str);
uint32_t fs2int(char *str);

// prototype defs
void print_version();
void wait_ms(uint16_t dly);
void clrLED();
void blinkLED(uint8_t pin, uint8_t x);
void blinkTX();
void setLED(uint8_t data);
void display_band();
void display_mode();
void blink_band();
void blink_mode();
void blink_both();
void set_all();
void clr_all();
void blink_all();
void cylon(uint8_t left);
void mode_assign();
void band_assign();
void band_select();
void freq2band();
void manualTX();
void si5351_cal();
void FSK_tone();
void check_VOX();
void rx_mode();
void check_UI();
void check_CAT();
void CAT_VFO();
void CAT_control();
void save_eeprom();
void read_eeprom();
void init_VFO();
void init_timer();
void initPins();
void print_band();
void print_mode();
void print_cat_mode();
void print_cal_data();
void factory_reset();
void calibrate_mode();

// eeprom addresses
#define DATA_ADDR    10
#define MODE_ADDR    20
#define BAND_ADDR    30
#define CATX_ADDR    40

// delay times (ms)
#define DLY1        250
#define DEBOUNCE     20

#define MS_LONG     300  // long press
uint32_t t0;             // event timer

// read pushbutton switches
#define LT_PRESSED  !digitalRead(LTSW)
#define RT_PRESSED  !digitalRead(RTSW)
#define TX_PRESSED  !digitalRead(TXSW)

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
#define UNKNOWN   0

// encoded bands
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
#define ENC0m   GROUP0|XBAND0

const int8_t bandENC[12] = {
 ENC0m,  ENC6m,  ENC10m, ENC12m, ENC15m,
 ENC17m, ENC20m, ENC30m, ENC40m, ENC60m,
 ENC80m, ENC160m };

// calibration data
#define CAL_DATA_INIT  64000ULL
uint32_t cal_data = CAL_DATA_INIT;

#define MAX_MODE 8
#define MIN_MODE 1
#define MAX_BAND 11
#define MIN_BAND 1

#define WSP_MODE 8
#define JS8_MODE 4
#define FT4_MODE 2
#define FT8_MODE 1

uint8_t mode = FT8_MODE;
uint8_t band = BAND20;
uint8_t cat_mode = OFF;

Si5351 si5351;

#define CPUXTL  1600000000ULL // CPU clock
#define MAXCNT  64000  // max event period
#define MAXVOX  15     // VOX timeout (ms)

volatile uint8_t  d2ICR = FALSE;
volatile uint8_t  doFSK = NO;
volatile uint16_t d1;
volatile uint16_t d2;

// timer1 input capture interrupt routine
ISR (TIMER1_CAPT_vect) {
  // check if d2 is valid
  if (!d2ICR) {
    d2 = ICR1;
    d2ICR = TRUE;
    doFSK = NO;
  } else {
    d1 = d2;
    d2 = ICR1;
    doFSK = YES;
  }
}

// read char from the serial port
char getc() {
  while (!Serial.available());
  return(Serial.read());
}

// get next alpha char from serial buffer
char gnac() {
  char ch;
  uint16_t tc = 0;  // for timeout
  while (TRUE) {
    if (Serial.available()) {
      ch = Serial.read();
      if (alpha(ch)) return(ch);
    } else {
      if (tc++ > 1024) return('z');
    }
  }
}

// wait for semicolon from serial buffer
void getsemi() {
  char ch;
  while (TRUE) {
    if (Serial.available()) {
      ch = Serial.read();
      if (ch == ';') return;
    }
  }
}

// get + or - char from serial buffer
char gcal(char ch) {
  char new_ch;
  uint16_t tc = 0;  // for timeout
  while (TRUE) {
    if (Serial.available()) {
      new_ch = Serial.read();
      if ((new_ch=='+')||(new_ch=='-')||
          (new_ch=='=')||(new_ch=='.')){
        return(new_ch);
      }
    } else {
      if (tc++ > 1024) return(ch);
    }
  }
}

// return the length of string
uint8_t len(char *str) {
  uint8_t i=0;
  while (str[i++]);
  return i-1;
}

// compare command
uint8_t cmpstr(char *x, char *y) {
  if ((x[0] == y[0]) && (x[1] == y[1])) return(1);
  else return(0);
}

uint8_t alpha(char ch) {
  if ((ch >= 'a') && (ch <= 'z')) return(1);
  if ((ch >= 'A') && (ch <= 'Z')) return(1);
  return(0);
}

uint8_t numeric(char ch) {
  if ((ch >= '0') && (ch <= '9')) return(1);
  return(0);
}

// concatenate a character to a string
void catstr(char *dst, char ch) {
  uint8_t strlen= len(dst);
  dst[strlen++] = ch;
  dst[strlen] = '\0';
}

// convert command to upper case
void uppercase(char *str) {
  if ((str[0] >= 'a') && (str[0] <= 'z')) str[0] -= 32;
  if ((str[1] >= 'a') && (str[1] <= 'z')) str[1] -= 32;
}

// convert frequency string to integer
uint32_t fs2int(char *str) {
  uint32_t tmp = 0;
  uint32_t pwr10 = 1;
  char ch;
  for (uint8_t i=10; i>2; i--) {
    ch = str[i] - 48;
    tmp += ch * pwr10;
    pwr10 = ((pwr10<<3)+(pwr10<<1));
  }
  return(tmp);
}

// print the firmware version
void print_version() {
  Serial.println('\n');
  Serial.println(VERSION);
  Serial.println(DATE);
  Serial.println(NOTE1);
}

// send a string
void send(char *str) {
  getsemi(); // get semicolon
  Serial.print(str);
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
void setLED(uint8_t data) {
  digitalWrite(FT8LED, bitRead(data, 0));
  digitalWrite(FT4LED, bitRead(data, 1));
  digitalWrite(JS8LED, bitRead(data, 2));
  digitalWrite(WSPLED, bitRead(data, 3));
}

// set LEDs with the encoded band
void display_band() {
  uint8_t data = bandENC[band];
  setLED(data);
}

// set LEDs with the digital mode
void display_mode() {
  setLED(mode);
}

// blink the selected band
void blink_band() {
  for (uint8_t i=0; i<2; i++) {
    clrLED();
    delay(DLY1);
    display_band();
    delay(DLY1);
  }
}

// blink the selected mode
void blink_mode() {
  for (uint8_t i=0; i<2; i++) {
    clrLED();
    delay(DLY1);
    display_mode();
    delay(DLY1);
  }
}

// blink band then mode
void blink_both() {
  blink_band();  // blink band on LEDs
  delay(500);
  blink_mode();  // blink mode on LEDs
}

// set all LEDs
void set_all() {
  digitalWrite(FT8LED, ON);
  digitalWrite(FT4LED, ON);
  digitalWrite(JS8LED, ON);
  digitalWrite(WSPLED, ON);
  digitalWrite(TXXLED, ON);
}

// clear all LEDs
void clr_all() {
  digitalWrite(FT8LED, OFF);
  digitalWrite(FT4LED, OFF);
  digitalWrite(JS8LED, OFF);
  digitalWrite(WSPLED, OFF);
  digitalWrite(TXXLED, OFF);
}

// blink all the LEDs
void blink_all() {
  clr_all();
  delay(500);
  set_all();
  delay(500);
  clr_all();
}

// cylon blink all the LEDs
void cylon(uint8_t left) {
  clr_all();
  delay(500);
  if (left) {
    digitalWrite(FT8LED, ON);
    delay(100);
    digitalWrite(FT4LED, ON);
    delay(100);
    digitalWrite(TXXLED, ON);
    delay(100);
    digitalWrite(JS8LED, ON);
    delay(100);
    digitalWrite(WSPLED, ON);
  } else {
    digitalWrite(WSPLED, ON);
    delay(100);
    digitalWrite(JS8LED, ON);
    delay(100);
    digitalWrite(TXXLED, ON);
    delay(100);
    digitalWrite(FT4LED, ON);
    delay(100);
    digitalWrite(FT8LED, ON);
  }
  delay(500);
  clr_all();
  delay(500);
  blink_both();  // blink band then mode on LEDs
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
  rx_mode();
}

// band frequency assignments
void band_assign() {
  switch (band) {
    case BAND160:
      // 160M band
      F_FT8 = 1842000;
      F_FT4 = 1842000;
      F_JS8 = 1842000;
      F_WSP = 1842000;
      break;
    case BAND80:
      // 80M band
      F_FT8 = 3573000;
      F_FT4 = 3575000;
      F_JS8 = 3578000;
      F_WSP = 3568600;
      break;
    case BAND60:
      // 60M band
      /*
      F_FT8 =
      F_FT4 =
      F_JS8 =
      F_WSP =
      */
      break;
    case BAND40:
      // 40M band
      F_FT8 = 7074000;
      F_FT4 = 7047500;
      F_JS8 = 7078000;
      F_WSP = 7038600;
      break;
    case BAND30:
      // 30M band
      F_FT8 = 10136000;
      F_FT4 = 10140000;
      F_JS8 = 10130000;
      F_WSP = 10138700;
      break;
    case BAND20:
      // 20M band
      F_FT8 = 14074000;
      F_FT4 = 14080000;
      F_JS8 = 14078000;
      F_WSP = 14095600;
      break;
    case BAND17:
      // 17M band
      F_FT8 = 18100000;
      F_FT4 = 18104000;
      F_JS8 = 18104000;
      F_WSP = 18104600;
      break;
    case BAND15:
      // 15M band
      F_FT8 = 21074000;
      F_FT4 = 21140000;
      F_JS8 = 21078000;
      F_WSP = 21094600;
      break;
    case BAND12:
      // 12M band
      F_FT8 = 24915000;
      F_FT4 = 24919000;
      F_JS8 = 24922000;
      F_WSP = 24924600;
      break;
    case BAND10:
      // 10M band
      F_FT8 = 28074000;
      F_FT4 = 28180000;
      F_JS8 = 28078000;
      F_WSP = 28124600;
      break;
    case BAND06:
      // 6M band
      F_FT8 = 50313000;
      F_FT4 = 50318000;
      F_JS8 = 50318000;
      F_WSP = 50293000;
      break;
  }
  mode_assign();
}

// band select
void band_select() {
  bool done = NO;
  blink_band();
  while ((LT_PRESSED)||(RT_PRESSED));  // wait for release
  delay(DEBOUNCE);
  while (!done) {
    // check the < button
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
        // for long press of < button
        // exit band-select mode
        done = YES;
        blink_mode();
      } else {
        // for short click of < button
        // decrement the band
        band = band - 1;
        if (band < MIN_BAND) band = MAX_BAND;
        display_band();
        band_assign();
      }
      while (LT_PRESSED);  // wait for release
      delay(DEBOUNCE);
    }
    // check the > button
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
        // for long press of < button
        // exit band-select mode
        done = YES;
        blink_mode();
      } else {
        // for short click of < button
        // increment the band
        band = band + 1;
        if (band > MAX_BAND) band = MIN_BAND;
        display_band();
        band_assign();
      }
      while (RT_PRESSED);  // wait for release
      delay(DEBOUNCE);
    }
    // check the tx button
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
        // for long press of TX button
        // manual TX
        manualTX();
        display_band();
      } else {
        // for short click of TX button
        // save the band to eeprom
        blinkTX();
        EEPROM.put(BAND_ADDR, band);
        while (TX_PRESSED);  // wait for release
        delay(DEBOUNCE);
        // exit band-select mode
        done = YES;
        blink_mode();
      }
    }
  }
}

// frequency to band
void freq2band() {
  band = UNKNOWN;
  mode = UNKNOWN;
  // lookup the band
  if ((base_freq > 50000000) && (base_freq < 54000000)) {
    band = BAND06;
  } else if ((base_freq > 28000000) && (base_freq < 29700000)) {
    band = BAND10;
  } else if ((base_freq > 24890000) && (base_freq < 24990000)) {
    band = BAND12;
  } else if ((base_freq > 21000000) && (base_freq < 21450000)) {
    band = BAND15;
  } else if ((base_freq > 18070000) && (base_freq < 18170000)) {
    band = BAND17;
  } else if ((base_freq > 14000000) && (base_freq < 14350000)) {
    band = BAND20;
  } else if ((base_freq > 10100000) && (base_freq < 10150000)) {
    band = BAND30;
  } else if ((base_freq >  7000000) && (base_freq <  7300000)) {
    band = BAND40;
  } else if ((base_freq >  5300000) && (base_freq <  5500000)) {
    band = BAND60;
  } else if ((base_freq >  3500000) && (base_freq <  4000000)) {
    band = BAND80;
  } else if ((base_freq >  1800000) && (base_freq <  2000000)) {
    band = BAND160;
  }
  // lookup the mode
  switch (base_freq) {
    // 160M band
    case 1840000:
      mode = FT8_MODE;
      break;
    case 1842000:
      mode = JS8_MODE;
      break;
    case 1836000:
      mode = WSP_MODE;
      break;
    // 80M band
    case 3573000:
      mode = FT8_MODE;
      break;
    case 3575000:
      mode = FT4_MODE;
      break;
    case 3578000:
      mode = JS8_MODE;
      break;
    case 3568600:
      mode = WSP_MODE;
      break;
    // 40M band
    case 7074000:
      mode = FT8_MODE;
      break;
    case 7047500:
      mode = FT4_MODE;
      break;
    case 7078000:
      mode = JS8_MODE;
      break;
    case 7038600:
      mode = WSP_MODE;
      break;
    // 30M band
    case 10136000:
      mode = FT8_MODE;
      break;
    case 10140000:
      mode = FT4_MODE;
      break;
    case 10130000:
      mode = JS8_MODE;
      break;
    case 10138700:
      mode = WSP_MODE;
      break;
    // 20M band
    case 14074000:
      mode = FT8_MODE;
      break;
    case 14080000:
      mode = FT4_MODE;
      break;
    case 14078000:
      mode = JS8_MODE;
      break;
    case 14095600:
      mode = WSP_MODE;
      break;
    // 17M band
    case 18100000:
      mode = FT8_MODE;
      break;
    case 18104000:
      mode = FT4_MODE;
      break;
    /*
    case 18104000:
      mode = JS8_MODE;
      break;
    */
    case 18104600:
      mode = WSP_MODE;
      break;
    // 15M band
    case 21074000:
      mode = FT8_MODE;
      break;
    case 21140000:
      mode = FT4_MODE;
      break;
    case 21078000:
      mode = JS8_MODE;
      break;
    case 21094600:
      mode = WSP_MODE;
      break;
    // 12M band
    case 24915000:
      mode = FT8_MODE;
      break;
    case 24919000:
      mode = FT4_MODE;
      break;
    case 24922000:
      mode = JS8_MODE;
      break;
    case 24924600:
      mode = WSP_MODE;
      break;
    // 10M band
    case 28074000:
      mode = FT8_MODE;
      break;
    case 28180000:
      mode = FT4_MODE;
      break;
    case 28078000:
      mode = JS8_MODE;
      break;
    case 28124600:
      mode = WSP_MODE;
      break;
    // 6M band
    case 50313000:
      mode = FT8_MODE;
      break;
    case 50318000:
      mode = FT4_MODE;
      break;
    /*
    case 50318000:
      mode = JS8_MODE;
      break;
    */
    case 50293000:
      mode = WSP_MODE;
      break;
  }
  // update the frequency
  si5351.set_freq(base_freq*100, SI5351_CLK1);
  display_band();
}

// manual TX
void manualTX() {
  digitalWrite(RXGATE, OFF);
  si5351.output_enable(SI5351_CLK1, 0);   // RX off
  clrLED();
  digitalWrite(TXXLED, ON);
  si5351.set_freq(base_freq*100, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 1);   // TX on
  while (TX_PRESSED);  // wait for release
  delay(DEBOUNCE);
  rx_mode();           // back to rx mode
}

// si5351 calibration
void si5351_cal() {
  uint32_t cal_freq = 1000000UL;
  bool done = NO;
  cat_mode = OFF;
  setLED(0b1111);   // set all LEDs while button pressed
  wait4buttons();   // wait for all buttons released
  setLED(0b1001);   // set LEDs to indicate cal mode
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
  si5351.set_freq(cal_freq*100, SI5351_CLK2);
  si5351.set_clock_pwr(SI5351_CLK2, 1);
  si5351.output_enable(SI5351_CLK2, 1);
  while (!done) {
    // check the tx button
    if (TX_PRESSED) {
      // if TX pressed then
      // save the calibration data
      // and exit calibration mode
      done = YES;
      clrLED();            // clear all LEDs
      while (TX_PRESSED);  // wait for release
      delay(DEBOUNCE);
    }
    // check the > button
    else if (RT_PRESSED) {
      cal_data = cal_data - 100;
      si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
      si5351.set_freq(cal_freq*100, SI5351_CLK2);
      delay(DEBOUNCE);
    }
    // check the < button
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
  rx_mode();  // put radio in rx mode
}

uint8_t  FSKtx = FALSE;
uint8_t  tx_status = RX;
uint32_t vox_timer;

void FSK_tone() {
  if (!doFSK) return;
  doFSK = NO;
  // noInterrupts();
  // FSK frequency measurement
  uint16_t delta = d2 - d1;  // captured event period
  vox_timer = millis();      // reset the vox timer
  if (delta < MAXCNT) {      // check for valid period
    if (!FSKtx) {
      clrLED();
      digitalWrite(TXXLED, ON);
      tx_status = TX;
      digitalWrite(RXGATE, OFF);
      si5351.output_enable(SI5351_CLK1, 0);  // RX off
      si5351.output_enable(SI5351_CLK0, 1);  // TX on
      FSKtx = TRUE;
    }
    uint32_t code_freq = CPUXTL/delta;
    si5351.set_freq(((base_freq*100) + code_freq), SI5351_CLK0);
    vox_timer = millis();    // reset the vox timer
  }
}

// if VOX timeout then return to rx mode
void check_VOX() {
  if (FSKtx && (millis() - vox_timer > MAXVOX)) {
    FSKtx = FALSE;
    d2ICR = FALSE;
    rx_mode();
    display_mode();
  }
}

// put radio in rx mode
void rx_mode() {
  digitalWrite(TXXLED, OFF);
  tx_status = RX;
  si5351.output_enable(SI5351_CLK0, 0);   // TX off
  si5351.set_freq(base_freq*100, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);   // RX on
  digitalWrite(RXGATE, ON);
}

// check the UI buttons
void check_UI() {
  // check the < button
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
      // for long press of < button
      // goto band-select mode
      band_select();
    } else {
      // for short click of < button
      // left-shift the radio mode
      mode = mode<<1;
      if (mode > MAX_MODE) mode = MIN_MODE; // wrap
      band_assign();
      display_mode();
    }
    while (LT_PRESSED);  // wait for release
    delay(DEBOUNCE);
  }
  // check the > button
  if (RT_PRESSED) {
    t0 = millis();
    event = CLICK;
    // wait for release
    while (RT_PRESSED) {
      // check for long press
      if ((millis() - t0) > MS_LONG) { event = LONG; break; }
      wait_ms(1);
    }
    if (event == LONG) {
      // for long press of > button
      // turn on CAT mode and blink LEDs
      cat_mode = ON;
      print_cat_mode();
      cylon(LEFT);
      EEPROM.put(CATX_ADDR, cat_mode);
    } else {
      // for short click of > button
      // right-shift the radio mode
      mode = mode>>1;
      if (mode < MIN_MODE) mode = MAX_MODE; // wrap
      band_assign();
      display_mode();
    }
    while (RT_PRESSED);  // wait for release
    delay(DEBOUNCE);
  }
  // check the tx button
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
      // for long press of TX button
      // manual TX
      clrLED();
      manualTX();
      display_mode();
    } else {
      // for short click of TX button
      // save the radio mode
      blinkTX();
      display_mode();
      EEPROM.put(MODE_ADDR, mode);
      // wait for release
      while (TX_PRESSED);
      delay(DEBOUNCE);
    }
  }
}

// check for CAT control
void check_CAT() {
  if (Serial.available()) CAT_control();
  // check the > button
  if (RT_PRESSED) {
    t0 = millis();
    event = CLICK;
    // wait for release
    while (RT_PRESSED) {
      // check for long press
      if ((millis() - t0) > MS_LONG) { event = LONG; break; }
      wait_ms(1);
    }
    if (event == LONG) {
      // for long press of > button
      // turn off CAT mode and blink LEDs
      cat_mode = OFF;
      print_cat_mode();
      cylon(RIGHT);
      EEPROM.put(CATX_ADDR, cat_mode);
    } else {
      // for short click of > button
      display_band();
    }
    while (RT_PRESSED);  // wait for release
    delay(DEBOUNCE);
  }
  // check the < button
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
      // do nothing
    } else {
      // for short click of > button
      display_mode();
    }
    while (LT_PRESSED);  // wait for release
    delay(DEBOUNCE);
  }
}

// print an 11-digit frequency
void CAT_VFO() {
  if      (base_freq >= 10000000) Serial.print("000");
  else if (base_freq >=  1000000) Serial.print("0000");
  else                       Serial.print("00000");
  Serial.print(base_freq);
}

// The following CAT commands are implemented
//
// command get/set  name              operation
// ------- -------  ----------------  -----------------------
// IF        G -    radio status      returns frequency and other status
// ID        G -    radio ID          returns 019 = Kenwood TS2000
// FA        G S    frequency         gets or sets the ADX frequency
// AI        G S    auto-information  returns 0   = OFF
// MD        G S    radio mode        returns 2   = USB
// PS        G S    power-on status   returns 1   = ON
// XT        G S    XIT status        returns 0   = OFF
// TX        - S    transmit          returns 0 and set TX LED
// RX        - S    receive           returns 0 and clears TX LED
//
void CAT_control() {
  char cmd[3] = "zz";
  char param[20] = "";

  // get next alpha char from serial buffer
  char ch = gnac();
  if (ch == 'z') return;  // non-alpha char

  // get the command
  cmd[0] = ch;
  cmd[1] = getc();
  uppercase(cmd);

  // get frequency and other status
  if (cmpstr(cmd, "IF")) {
    send("IF");
    CAT_VFO();
    Serial.print("00000+000000000");
    if (tx_status) Serial.print('1');
    else Serial.print('0');
    Serial.print("20000000;");
  }

  // get radio ID
  else if (cmpstr(cmd, "ID")) send("ID019;");

  // get or set frequency
  else if (cmpstr(cmd, "FA")) {
    ch = getc();
    if (numeric(ch)) {
      // set frequency
      catstr(param, ch);
      for (uint8_t i=0; i<10; i++) {
        catstr(param, getc());
      }
      base_freq = fs2int(param);
      // set band and mode
      freq2band();
      getsemi(); // get semicolon
    } else {
      // get frequency
      Serial.print("FA");
      CAT_VFO();
      Serial.print(";");
    }
  }

  // get or set the radio mode
  else if (cmpstr(cmd, "MD")) {
    ch = getc();
    if (numeric(ch)) {
      // set radio mode
      // does nothing .. always 2
      getsemi();
    } else {
      // get auto-information status
      Serial.print("MD2;");
    }
  }

  // get or set auto-information status
  else if (cmpstr(cmd, "AI")) {
    ch = getc();
    if (numeric(ch)) {
      // set auto-information status
      // does nothing .. always 0
      getsemi();
    } else {
      // get auto-information status
      Serial.print("AI0;");
    }
  }

  // get or set the power (ON/OFF) status
  else if (cmpstr(cmd, "PS")) {
    ch = getc();
    if (numeric(ch)) {
      // set power (ON/OFF) status
      // does nothing .. always 1
      getsemi();
    } else {
      // get power (ON/OFF) status
      Serial.print("PS1;");
    }
  }

  // get or set the XIT (ON/OFF) status
  else if (cmpstr(cmd, "XT")) {
    ch = getc();
    if (numeric(ch)) {
      // set XIT (ON/OFF) status
      // does nothing .. always OFF
      getsemi();
    } else {
      // get XIT (ON/OFF) status
      Serial.print("XT0;");
    }
  }

  // CAT transmit
  else if (cmpstr(cmd, "TX")) {
    getsemi(); // get semicolon
    tx_status = TX;
  }

  // CAT receive
  else if (cmpstr(cmd, "RX")) {
    getsemi(); // get semicolon
    tx_status = RX;
  }

  // ===========================
  //  ADX-specific CAT commands
  // ===========================

  // factory reset
  else if (cmpstr(cmd, "FR")) {
    factory_reset();
  }
  // calibrate mode
  else if (cmpstr(cmd, "CM")) {
    calibrate_mode();
  }

}

// write to the eeprom
void save_eeprom() {
  Serial.println("Saving settings to EEPROM");
  EEPROM.put(DATA_ADDR, cal_data);
  EEPROM.put(MODE_ADDR, mode);
  EEPROM.put(BAND_ADDR, band);
  EEPROM.put(CATX_ADDR, cat_mode);
}

// read the eeprom
void read_eeprom() {
  Serial.println("Reading settings from EEPROM");
  while (LT_PRESSED);  // wait for release
  delay(DEBOUNCE);
  clrLED();
  EEPROM.get(DATA_ADDR, cal_data);
  EEPROM.get(MODE_ADDR, mode);
  EEPROM.get(BAND_ADDR, band);
  EEPROM.get(CATX_ADDR, cat_mode);
  if ((cal_data < 100) || (cal_data > 100000)) {
    cal_data = CAL_DATA_INIT;
    EEPROM.put(DATA_ADDR, cal_data);
  }
  if ((mode < MIN_MODE) || (mode > MAX_MODE)) {
    mode = MAX_MODE;
    EEPROM.put(MODE_ADDR, mode);
  }
  if ((band < MIN_BAND) || (band > MAX_BAND)) {
    band = MIN_BAND;
    EEPROM.put(BAND_ADDR, band);
  }
  print_band();
  print_mode();
  print_cat_mode();
  print_cal_data();
}

// initialize the Set Si5351 VFO
void init_VFO() {
  si5351.init();
  si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  band_assign();
}

// initialize the timer
void init_timer() {
  TCCR1A = 0x00;       // OC1A/OC1B disconnected
  TCCR1B = 0x81;       // falling edge capture + noise canceller
  ACSR  |= (1<<ACIC);  // analog comparator input capture
  TIFR1  = (1<<ICF1);  // clear interrupt flag
  TIMSK1 = (1<<ICIE1); // enable timer1 capture event interrupt
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

// debug print of current band
void print_band() {
  const char* band_label[]  = {
  "??",   "6M", "10M", "12M", "15M", "17M",
  "20M", "30M", "40M", "60M", "80M", "160M" };
  if (DEBUG1) {
    Serial.print("band = ");
    Serial.println(band_label[band]);
  }
}

// debug print of current mode
void print_mode() {
  if (DEBUG1) {
    Serial.print("mode = ");
    switch (mode) {
      case WSP_MODE:
        Serial.println("WSPR");
        break;
      case JS8_MODE:
        Serial.println("JS8");
        break;
      case FT4_MODE:
        Serial.println("FT4");
        break;
      case FT8_MODE:
        Serial.println("FT8");
        break;
      default:
        break;
    }
  }
}

// debug print of CAT mode
void print_cat_mode() {
  if (cat_mode) Serial.println("CAT mode is ON");
  else Serial.println("CAT mode is OFF");
}

// debug print of calibration data
void print_cal_data() {
  Serial.print("cal_data = ");
  Serial.println(cal_data);
}

// factory reset (CAT command)
void factory_reset() {
  cal_data = CAL_DATA_INIT;
  band = BAND20;
  mode = FT8_MODE;
  cat_mode = ON;
  Serial.println("factory reset");
  print_band();
  print_mode();
  print_cat_mode();
  print_cal_data();
  band_assign();
  blink_both();   // blink band then mode on LEDs
  save_eeprom();  // save factory settings
  rx_mode();
}

// calibrate si5351 (CAT command)
void calibrate_mode() {
  char ch;
  uint8_t up = FALSE;
  uint8_t dn = FALSE;
  uint8_t xx = 0;
  bool done = NO;
  uint32_t cal_freq = 1000000UL;
  setLED(0b1111);   // set all LEDs
  delay(500);
  setLED(0b1001);   // set LEDs to indicate cal mode
  Serial.println('\n');
  Serial.println("calibration mode");
  Serial.println("press + to increase cal frequency");
  Serial.println("press - to decrease cal frequency");
  Serial.println("press = to stop");
  Serial.println("press . to save and exit");
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
  si5351.set_freq(cal_freq*100, SI5351_CLK2);
  si5351.set_clock_pwr(SI5351_CLK2, 1);
  si5351.output_enable(SI5351_CLK2, 1);
  while (!done) {
    ch = gcal(ch);
    switch (ch) {
      case '+':      // increment
        up = TRUE;
        dn = FALSE;
        break;
      case '-':      // decrement
        up = FALSE;
        dn = TRUE;
        break;
      case '=':      // stop
        up = FALSE;
        dn = FALSE;
        break;
      case '.':      // exit
        done = TRUE;
        up = FALSE;
        dn = FALSE;
        break;
      default:
        break;
    }
    if (up||dn) {
      if (up) cal_data = cal_data - 10;
      if (dn) cal_data = cal_data + 10;
      si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
      si5351.set_freq(cal_freq*100, SI5351_CLK2);
      if (xx == 0) Serial.print(ch);
      if (xx++ == 100) xx = 0;
    }
  }
  si5351.output_enable(SI5351_CLK2, 0);
  si5351.set_clock_pwr(SI5351_CLK2, 0);
  EEPROM.put(DATA_ADDR, cal_data);
  Serial.println(' ');
  Serial.println("exiting calibration mode");
  blinkTX();  // blink TX LED when done
  rx_mode();  // put radio in rx mode
}

// Arduino setup function
void setup() {
  initPins();
  Serial.begin(115200);
  print_version();
  // if < button active then factory reset
  // if > button active then calibrate
  if (!LT_PRESSED) read_eeprom();
  else save_eeprom();
  init_VFO();
  if (RT_PRESSED) si5351_cal();
  init_timer();
  blink_both();    // blink band then mode on LEDs
  wait4buttons();  // wait for all buttons released
}

// Arduino main loop
void loop() {
  FSK_tone();              // measure FSK frequency
  if (FSKtx) check_VOX();  // check for VOX timeout
  if (cat_mode) check_CAT();
  else check_UI();
}

