
// ============================================================================
//
// ADX_MI3.ino  ::  ADX-MI3 Control Program with CAT control
//
// This control program was derived from the ADX control program.
// It is compatible with ADX-MI3 hardware.
// Supported bands are: 160M/80M/60M/40M/30M/20M/17M/15M/12M/10M/6M
// The user interface is described in a separate document.
//
// (c) Scott Baker KJ7NLA
//
// Libraries
// ---------
// i2c.h        - a simple I2C lib
// ee.h         - a simple EEPROM lib
// oled.h       - an OLED display lib
// font.h       - a font that I designed
// si5351.h     - by Milldrum and Myers
//
// Arduino IDE settings
// --------------------
// board: Arduino UNO
// bootloader: no bootloader
// programmer: AVRISP mkII
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

#define VERSION   "ADX-MI3"             // firmware version
#define DATE      "Oct 29 2024"         // firmware date

// Arduino Pins
#define RXD       0      // PD0   UART RX            (pin 30)
#define TXD       1      // PD1   UART TX            (pin 31)
#define B3        2      // PD2   band ID 3          (pin 32)
#define B2        9      // PB1   band ID 2          (pin 13)
#define B1        3      // PD3   band ID 1          (pin  1)
#define B0       10      // PB2   band ID 0          (pin 14)
#define TXLED     5      // PD5   Tx LED             (pin  9)
#define ACMP      7      // PD7   analog comparator  (pin 11)
#define RXGATE    8      // PB0   Rx Gate            (pin 12)
#define VBATT    20      // ADC6  battery voltage    (pin 19)
#define BUTTON    4      // PD4   UI pushbutton      (pin  2)

#include "i2c.h"
#include "ee.h"
#include "oled.h"
#include "font.h"
#include "si5351.h"

// generic
#define OFF      0
#define ON       1
#define NO       0
#define YES      1
#define FALSE    0
#define TRUE     1
#define RX       0    // tx/rx status
#define TX       1    // tx/rx status
#define FACTORY  0    // factory reset
#define SOFT     1    // soft reset
#define SERIAL   1    // print to UART
#define LOCAL    2    // print to OLED
#define BOTH     3    // print to UART and OLED

// string prototype defs
char getc();
char gnac();
void getsemi();
char gcal(char ch);
uint8_t len(char *str);
void send(char *str);
uint8_t cmpstr(char *dst, char *src);
void catstr(char *dst, char c);
void uppercase(char *str);
uint8_t alpha(char ch);
uint8_t numeric(char ch);
void getmode();
uint32_t fs2int(char *str);

// more prototype defs
void show_version(uint8_t x);
void show_help();
void show_cal();
void show_info();
void show_debug();
void show_queue();
void show_band(char* str);
void wait_ms(uint16_t dly);
void wait_us(uint16_t dly);
void blinkLED();
void error_blink();
void FSK_tone();
void check_VOX();
void readbuf();
inline void CAT_VFO();
inline void CAT_cmd();
void save_eeprom();
void init_VFO();
void init_freq();
void init_timer0();
void init_timer1();
void init_pins();
void init_i2c();
void init_uart();
void init_oled();
void init_check();
void init_adc();
void read_adc();
void set_tx_status(uint8_t x);
void tuning_hdr();
void tuning_mode();
void manualTX();
uint8_t check_band();
uint8_t freq2band(uint32_t freq);
void update_freq(uint32_t freq);
void check_timeout();
void check_UI();
void check_CAT();
void reset_xtimer();
void refresh();
void do_reset(uint8_t soft);
void run_calibrate();

// eeprom addresses
#define DATA_ADDR    10      // calibration data
#define FREQ_ADDR    20      // frequency

// Si5351 xtal frequency (25 MHz)
#define SI5351_REF  25000000UL

// band assignments
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

// band ID encodings
#define ID160M  0x0e
#define ID80M   0x0d
#define ID60M   0x0b
#define ID40M   0x0a
#define ID30M   0x09
#define ID20M   0x07
#define ID17M   0x06
#define ID15M   0x05
#define ID12M   0x03
#define ID10M   0x02
#define ID06M   0x01
#define IDXXM   0x0f

// band labels
const char* band_label[] = {
" ???","  6M"," 10M"," 12M"," 15M"," 17M",
" 20M"," 30M"," 40M"," 60M"," 80M","160M" };

// mode labels
const char* mode_label[] = {
"??? ","FT8 ","FT4 ","JS8 ","WSPR","JT65"};

// calibration data
#define CAL_DATA_INIT  64000ULL
uint32_t cal_data = CAL_DATA_INIT;

#define MAX_MODE 5
#define MIN_MODE 1
#define MAX_BAND 11
#define MIN_BAND 1

#define J65_MODE 5
#define WSP_MODE 4
#define JS8_MODE 3
#define FT4_MODE 2
#define FT8_MODE 1

uint8_t mode = FT8_MODE;
uint8_t band = BAND20;

// class instantiation
Si5351  si5351;
I2C     i2c;
EE      eeprom;
OLED    oled;

#define CPUXTL  1600000000ULL // CPU clock
#define MAXCNT  64000  // max event period
#define MAXVOX  15     // VOX timeout (ms)

volatile uint8_t  d2ICR = FALSE;
volatile uint8_t  doFSK = NO;
volatile uint16_t d1;
volatile uint16_t d2;

// delay times (ms)
#define DEBOUNCE          50
#define LED_BLINK        100
#define ONE_SECOND      1000
#define TWO_SECONDS     2000
#define THREE_SECONDS   3000
#define TEN_SECONDS    10000
#define HALF_MINUTE    30000
#define ONE_MINUTE     60000
#define TIMEOUT        HALF_MINUTE

// user interface macros
#define NBP  0  // no-button-pushed
#define BSC  1  // button-single-click
#define BPL  2  // button-push-long
#define DLP  3  // double-long-press
#define SLP  4  // super-long-press

#define SUPERPRESS  3500
#define XLPRESS     1200
#define LONGPRESS   500
#define UIKEY       !digitalRead(BUTTON)

// for display blank/timeout
uint8_t  display = ON;
uint32_t xtimer;

// band module ID fault detected
uint8_t band_fault;

// millisecond time
volatile uint32_t msTimer = 0;
volatile uint16_t loopCount = 0;

// timer 0 interrupt service routine
ISR(TIMER0_COMPA_vect) {
  msTimer++;
  loopCount++;
}

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

// get cal control char from serial buffer
char gcal(char ch) {
  char new_ch;
  uint16_t tc = 0;  // timeout counter
  while (TRUE) {
    if (Serial.available()) {
      new_ch = Serial.read();
      if ((new_ch=='+')||(new_ch=='-')||
          (new_ch=='/')||(new_ch=='\\')||
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

// send a command
void send(char *str) {
  getsemi(); // get semicolon
  Serial.print(str);
}

// compare command
uint8_t cmpstr(char *x, char *y) {
  if ((x[0] == y[0]) && (x[1] == y[1])) return(1);
  else return(0);
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

// check if char is alpha
uint8_t alpha(char ch) {
  if ((ch >= 'A') && (ch <= 'z')) return(1);
  return(0);
}

// check if char is numeric
uint8_t numeric(char ch) {
  if ((ch >= '0') && (ch <= '9')) return(1);
  return(0);
}

char *modestr = "                ";
float fpv = 0.0;

// concatenate mode, band, and vbatt to a string
void getmode() {
  char tmp[10];
  dtostrf(fpv, 4, 1, tmp);
  for (uint8_t i=0; i<4; i++) {
    modestr[i]    = mode_label[mode][i];
    modestr[i+5]  = band_label[band][i];
    modestr[i+11] = tmp[i];
  }
  modestr[15] = 'V';  // voltage
}

// convert frequency string to integer
uint32_t fs2int(char *str) {
  uint32_t acc = 0;
  uint32_t pwr10 = 1;
  uint8_t digit;
  for (uint8_t i=10; i>2; i--) {
    digit = (uint8_t)str[i] - 48;
    acc += digit * pwr10;
    pwr10 = ((pwr10<<3)+(pwr10<<1));
  }
  return(acc);
}

// print the firmware version
void show_version(uint8_t x) {
  if ((x == SERIAL) || (x == BOTH)) {
    // print to serial port
    Serial.print("  ");
    Serial.print(VERSION);
    Serial.print("\r\n  ");
    Serial.print(DATE);
    Serial.print("\r\n\n");
  }
  if ((x == LOCAL) || (x == BOTH)) {
    // print to OLED
    oled.clrScreen();
    oled.printline(0, VERSION);
    oled.printline(1, DATE);
    wait_ms(TWO_SECONDS);
    oled.clrScreen();
  }
}

#define HELP_MSG "\r\n\
  IF  G -  radio status\r\n\
  ID  G -  radio ID\r\n\
  FA  G S  frequency\r\n\
  AI  G S  auto-information\r\n\
  MD  G S  radio mode\r\n\
  PS  G S  power-on status\r\n\
  XT  G S  XIT status\r\n\
  TX  - S  transmit\r\n\
  RX  - S  receive\r\n\n\
  HE => print help\r\n\
  HH => print help\r\n\
  DD => debug on/off\r\n\
  II => print info\r\n\
  FR => factory reset\r\n\
  SR => soft reset\r\n\
  CM => calibration mode\r\n\n"

// print help message
void show_help() {
  Serial.print(HELP_MSG);
}

// print calibration data
void show_cal() {
  Serial.print("  cal_data = ");
  Serial.print(cal_data);
  Serial.print("\r\n\n");
}

uint8_t  DEBUG = FALSE;
uint8_t  FSKtx = FALSE;
uint8_t  tx_status = RX;
uint32_t vox_timer;
uint32_t base_freq = 0;

// print info to serial port
void show_info() {
  show_version(SERIAL);
  // print band
  Serial.print("  band = ");
  Serial.println(band_label[band]);
  // print frequency
  Serial.print("  freq = ");
  Serial.print(base_freq);
  Serial.print("\r\n");
  // print mode
  Serial.print("  mode = ");
  Serial.println(mode_label[mode]);
  show_cal();
}

// show debug status
void show_debug() {
  DEBUG = ! DEBUG;
  Serial.print("DEBUG=");
  Serial.print(DEBUG);
  Serial.println("");
}

// print a diagnostic message
void show_band(char* str) {
  oled.clrScreen();
  oled.printline(0, "BAND MODULE");
  oled.printline(1, str);
  Serial.print("BAND MODULE ");
  Serial.println(str);
}

// millisecond delay
void wait_ms(uint16_t dly) {
  uint32_t startTime = msTimer;
  while((msTimer - startTime) < dly) {
    wait_us(10);
  }
}

// microsecond delay
void wait_us(uint16_t x) {
  uint16_t t = ((x * 3) + (x>>1));
  for (int16_t i=0; i<t; i++) {
    asm("");
  }
}

// blink the LED
void blinkLED() {
  digitalWrite(TXLED,ON);
  wait_ms(LED_BLINK);
  digitalWrite(TXLED,OFF);
}

// blink the LED
void error_blink() {
  for (uint8_t i=0; i<2; i++) {
    digitalWrite(TXLED,ON);
    wait_ms(LED_BLINK);
    digitalWrite(TXLED,OFF);
    wait_ms(LED_BLINK);
  }
}

// FSK frequency measurement
void FSK_tone() {
  doFSK = NO;
  uint16_t delta = d2 - d1;  // captured event period
  vox_timer = msTimer;       // reset the vox timer
  if (delta < MAXCNT) {      // check for valid period
    if (!FSKtx) {
      set_tx_status(TX);
      FSKtx = TRUE;
    }
    uint32_t code_freq = CPUXTL/delta;
    si5351.set_freq(((base_freq*100) + code_freq), SI5351_CLK0);
    vox_timer = msTimer;     // reset the vox timer
  }
}

// if VOX timeout then return to rx mode
void check_VOX() {
  if (FSKtx && (msTimer - vox_timer > MAXVOX)) {
    FSKtx = FALSE;
    d2ICR = FALSE;
    set_tx_status(RX);
  }
}

// print (11-bit) VFO frequency
inline void CAT_VFO() {
  if      (base_freq >= 10000000) Serial.print("000");
  else if (base_freq >=  1000000) Serial.print("0000");
  else                            Serial.print("00000");
  Serial.print(base_freq);
}

// ==============================================================
// The following Kenwood TS-2000 CAT commands are implemented
//
// command get/set  name              operation
// ------- -------  ----------------  -----------------------
// IF        G -    radio status      returns frequency and other status
// ID        G -    radio ID          returns 019 = Kenwood TS-2000
// FA        G S    frequency         gets or sets the ADX frequency
// AI        G S    auto-information  returns 0   = OFF
// MD        G S    radio mode        returns 2   = USB
// PS        G S    power-on status   returns 1   = ON
// XT        G S    XIT status        returns 0   = OFF
// TX        - S    transmit          returns 0 and set TX LED
// RX        - S    receive           returns 0 and clears TX LED
//
// The following ADX-specific CAT commands are implemented
//
//  HE => print help
//  HH => print help
//  II => print info
//  DD => turn on/off debug
//  FR => factory reset
//  SR => soft reset
//  CM => calibration mode
// ==============================================================

// check for CAT control
void check_CAT() {
  if (Serial.available()) CAT_cmd();
}

void CAT_cmd() {
  char cmd[3] = "zz";
  char param[20] = "";

  // get next alpha char from serial buffer
  char ch = gnac();
  if (ch == 'z') return;  // non-alpha char

  // get the command
  cmd[0] = ch;
  cmd[1] = getc();
  uppercase(cmd);

  // ===========================
  //  TS-2000 CAT commands
  // ===========================

  //====================================
  //  IF           // (command)       2
  //  00014074000  // P1 (VF0)       11
  //  0000         // P2 (step size)  4
  //  +00000       // P3 (rit)        6
  //  00000        // P4->P7          5
  //  0/1          // P8 (Tx/Rx)      1
  //  20000000     // P9->P15         8
  //                       TOTAL  =  37
  //====================================

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
      freq2band(base_freq);
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

  // print help
  if (cmpstr(cmd, "HE")) {
    show_help();
  }

  // print help
  else if (cmpstr(cmd, "HH")) {
    show_help();
  }

  // toggle debug on/off
  else if (cmpstr(cmd, "DD")) {
    show_debug();
  }

  // print info
  else if (cmpstr(cmd, "II")) {
    show_info();
  }

  // factory reset
  else if (cmpstr(cmd, "FR")) {
    do_reset(FACTORY);
  }

  // soft reset
  else if (cmpstr(cmd, "SR")) {
    do_reset(SOFT);
  }

  // calibrate mode
  else if (cmpstr(cmd, "CM")) {
    run_calibrate();
  }

}

// write config data to the eeprom
void save_eeprom() {
  Serial.print("  Saving to EEPROM\r\n");
  eeprom.put32(DATA_ADDR, cal_data);
  eeprom.put32(FREQ_ADDR, base_freq);
}

// initialize the Si5351 VFO clocks
void init_VFO() {
  si5351.init();
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  set_tx_status(RX);
  si5351.output_enable(SI5351_CLK2, OFF);   // Cal off
  // clock drive strength
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);
}

// initialize the Si5351 frequency
void init_freq() {
  si5351.set_correction(cal_data, SI5351_PLL_INPUT_XO);
  si5351.set_freq(base_freq*100, SI5351_CLK1);
}

// initialize timer 0
void init_timer0() {
  TCCR0A = 0x02;          // CTC mode
  OCR0A  = 249;           // 1 ms count value
  TCCR0B = 0x03;          // use clk/64
  TIMSK0 = 0x02;          // interrupt on
}

// initialize timer 1
void init_timer1() {
  TCCR1A = 0x00;       // OC1A/OC1B disconnected
  TCCR1B = 0x81;       // falling edge capture + noise canceller
  ACSR  |= (1<<ACIC);  // analog comparator input capture
  TIFR1  = (1<<ICF1);  // clear interrupt flag
  TIMSK1 = (1<<ICIE1); // enable timer1 capture event interrupt
}

// initialize pins
void init_pins() {
  pinMode(TXLED,  OUTPUT);
  pinMode(RXGATE, OUTPUT);
  pinMode(TXD,    OUTPUT);
  pinMode(ACMP,   INPUT);
  pinMode(VBATT,  INPUT);
  pinMode(B3,     INPUT_PULLUP);
  pinMode(B2,     INPUT_PULLUP);
  pinMode(B1,     INPUT_PULLUP);
  pinMode(B0,     INPUT_PULLUP);
  pinMode(RXD,    INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
}

// initialize the I2C bus
void init_i2c() {
  i2c.begin();
}

// initialize the serial port
void init_uart() {
  #define BAUDRATE  115200
  Serial.begin(BAUDRATE);
}

// initialize the OLED
void init_oled() {
  oled.begin();
  display = ON;
  oled.onDisplay();
}

// check for factory reset during setup
void init_check() {
  if (UIKEY) do_reset(FACTORY);
  else do_reset(SOFT);
  show_version(BOTH);
}

// initialize the ADC
void init_adc() {
  analogReference(INTERNAL);
}

// measure battery voltage
void read_adc() {
  uint16_t val;
  val = analogRead(VBATT);
  fpv = ((float)val * 14.1) / 1024.0;
}

// set the Rx/Tx status
void set_tx_status(uint8_t x) {
  if (x == TX) {
    tx_status = TX;
    digitalWrite(TXLED,  ON);
    digitalWrite(RXGATE, OFF);
    si5351.output_enable(SI5351_CLK1, OFF);  // Rx off
    si5351.output_enable(SI5351_CLK0, ON);   // Tx on
  } else {
    tx_status = RX;
    si5351.output_enable(SI5351_CLK0, OFF);  // Tx off
    si5351.output_enable(SI5351_CLK1, ON);   // Rx on
    digitalWrite(TXLED,  OFF);
    digitalWrite(RXGATE, ON);
  }
}

// print tuning mode header
void tuning_hdr() {
  oled.clrScreen();
  oled.printline(0,"TUNING MODE");
}

// when in tuning mode push button for manual Tx
void tuning_mode() {
  uint8_t  event = NBP;
  uint32_t t0;
  while(event != BSC) {
    if (UIKEY) {
      event = BSC;
      t0 = msTimer;
      reset_xtimer();
      // check for long button press
      while (UIKEY && (event != BPL)) {
        if (msTimer > (t0 + LONGPRESS)) event = BPL;
        wait_ms(DEBOUNCE);
        check_CAT();
      }
      switch (event) {
        case BSC:         // short click to exit
          show_version(LOCAL);
          refresh();
          break;
        case BPL:         // long press to tune
          // check that the correct band module is installed
          // using the band module ID pins
          if (check_band()) manualTX();
          else tuning_hdr();
          break;
        default:
          break;
      }
    }
    reset_xtimer();
    check_CAT();          // check CAT interface
  }
}

// manual TX
void manualTX() {
  set_tx_status(TX);
  oled.printline(1,"XMIT");
  si5351.set_freq(base_freq*100, SI5351_CLK0);
  reset_xtimer();
  while (UIKEY) {
    wait_ms(DEBOUNCE);
    check_CAT();
  }
  wait_ms(DEBOUNCE);
  set_tx_status(RX);
  oled.clrLine(1);
}

// check the band ID
uint8_t check_band() {
  uint8_t pin_ID = 0;
  uint8_t bandID = UNKNOWN;
  band_fault = 0;
  reset_xtimer();
  if (digitalRead(B3)) pin_ID |= 0x08;
  if (digitalRead(B2)) pin_ID |= 0x04;
  if (digitalRead(B1)) pin_ID |= 0x02;
  if (digitalRead(B0)) pin_ID |= 0x01;
  switch (pin_ID) {
    case 0x0e:
      bandID = BAND160;
      break;
    case 0x0d:
      bandID = BAND80;
      break;
    case 0x0b:
      bandID = BAND60;
      break;
    case 0x0a:
      bandID = BAND40;
      break;
    case 0x09:
      bandID = BAND30;
      break;
    case 0x07:
      bandID = BAND20;
      break;
    case 0x06:
      bandID = BAND17;
      break;
    case 0x05:
      bandID = BAND15;
      break;
    case 0x03:
      bandID = BAND12;
      break;
    case 0x02:
      bandID = BAND10;
      break;
    case 0x01:
      bandID = BAND06;
      break;
    case 0x0f:
      band_fault = 1;
      break;
    case 0x0c:
      band_fault = 2;
      break;
    case 0x08:
      band_fault = 2;
      break;
    case 0x04:
      band_fault = 2;
      break;
    case 0x00:
      band_fault = 2;
      break;
    default:
      break;
  }
  // print error message
  switch (band_fault) {
    case 0:
      if (bandID == band) {
        return TRUE;
      } else {
        show_band(band_label[bandID]);
        oled.putstr(" != ");
        oled.putstr(band_label[band]);
        Serial.print("BAND = ");
        Serial.println(band_label[band]);
      }
      break;
    case 1:
      show_band("MISSING");
      break;
    case 2:
      show_band("UNKNOWN");
      break;
    default:
      break;
  }
  wait_ms(THREE_SECONDS);
  oled.clrScreen();
  return FALSE;
}

// frequency to band
uint8_t freq2band(uint32_t freq) {
  uint8_t freq_OK = NO;
  band = UNKNOWN;
  mode = UNKNOWN;
  // lookup the band
  if ((freq > 50000000) && (freq < 54000000)) {
    band = BAND06;
  } else if ((freq > 28000000) && (freq < 29700000)) {
    band = BAND10;
  } else if ((freq > 24890000) && (freq < 24990000)) {
    band = BAND12;
  } else if ((freq > 21000000) && (freq < 21450000)) {
    band = BAND15;
  } else if ((freq > 18070000) && (freq < 18170000)) {
    band = BAND17;
  } else if ((freq > 14000000) && (freq < 14350000)) {
    band = BAND20;
  } else if ((freq > 10100000) && (freq < 10150000)) {
    band = BAND30;
  } else if ((freq >  7000000) && (freq <  7300000)) {
    band = BAND40;
  } else if ((freq >  5300000) && (freq <  5500000)) {
    band = BAND60;
  } else if ((freq >  3500000) && (freq <  4000000)) {
    band = BAND80;
  } else if ((freq >  1800000) && (freq <  2000000)) {
    band = BAND160;
  }
  // lookup the mode
  switch (freq) {
    // 160M band
    case 1840000:
      freq_OK++;
      mode = FT8_MODE;
      break;
    case 1842000:
      freq_OK++;
      mode = JS8_MODE;
      break;
    case 1836000:
      freq_OK++;
      mode = WSP_MODE;
      break;
    case 1838000:
      freq_OK++;
      mode = J65_MODE;
      break;
    // 80M band
    case 3573000:
      freq_OK++;
      mode = FT8_MODE;
      break;
    case 3575000:
      freq_OK++;
      mode = FT4_MODE;
      break;
    case 3578000:
      freq_OK++;
      mode = JS8_MODE;
      break;
    case 3568600:
      freq_OK++;
      mode = WSP_MODE;
      break;
    case 3570000:
      freq_OK++;
      mode = J65_MODE;
      break;
    // 40M band
    case 7074000:
      freq_OK++;
      mode = FT8_MODE;
      break;
    case 7047500:
      freq_OK++;
      mode = FT4_MODE;
      break;
    case 7078000:
      freq_OK++;
      mode = JS8_MODE;
      break;
    case 7038600:
      freq_OK++;
      mode = WSP_MODE;
      break;
    case 7076000:
      freq_OK++;
      mode = J65_MODE;
      break;
    // 30M band
    case 10136000:
      freq_OK++;
      mode = FT8_MODE;
      break;
    case 10140000:
      freq_OK++;
      mode = FT4_MODE;
      break;
    case 10130000:
      freq_OK++;
      mode = JS8_MODE;
      break;
    case 10138700:
      freq_OK++;
      mode = WSP_MODE;
      break;
    case 10138000:
      freq_OK++;
      mode = J65_MODE;
      break;
    // 20M band
    case 14074000:
      freq_OK++;
      mode = FT8_MODE;
      break;
    case 14080000:
      freq_OK++;
      mode = FT4_MODE;
      break;
    case 14078000:
      freq_OK++;
      mode = JS8_MODE;
      break;
    case 14095600:
      freq_OK++;
      mode = WSP_MODE;
      break;
    case 14076000:
      freq_OK++;
      mode = J65_MODE;
      break;
    // 17M band
    case 18100000:
      freq_OK++;
      mode = FT8_MODE;
      break;
    case 18104000:
      freq_OK++;
      mode = FT4_MODE;
      break;
    /*
    case 18104000:
      freq_OK++;
      mode = JS8_MODE;
      break;
    */
    case 18104600:
      freq_OK++;
      mode = WSP_MODE;
      break;
    case 18102000:
      freq_OK++;
      mode = J65_MODE;
      break;
    // 15M band
    case 21074000:
      freq_OK++;
      mode = FT8_MODE;
      break;
    case 21140000:
      freq_OK++;
      mode = FT4_MODE;
      break;
    case 21078000:
      freq_OK++;
      mode = JS8_MODE;
      break;
    case 21094600:
      freq_OK++;
      mode = WSP_MODE;
      break;
    case 21076000:
      freq_OK++;
      mode = J65_MODE;
      break;
    // 12M band
    case 24915000:
      freq_OK++;
      mode = FT8_MODE;
      break;
    case 24919000:
      freq_OK++;
      mode = FT4_MODE;
      break;
    case 24922000:
      freq_OK++;
      mode = JS8_MODE;
      break;
    case 24924600:
      freq_OK++;
      mode = WSP_MODE;
      break;
    case 24917000:
      freq_OK++;
      mode = J65_MODE;
      break;
    // 10M band
    case 28074000:
      freq_OK++;
      mode = FT8_MODE;
      break;
    case 28180000:
      freq_OK++;
      mode = FT4_MODE;
      break;
    case 28078000:
      freq_OK++;
      mode = JS8_MODE;
      break;
    case 28124600:
      freq_OK++;
      mode = WSP_MODE;
      break;
    case 28076000:
      freq_OK++;
      mode = J65_MODE;
      break;
    // 6M band
    case 50313000:
      freq_OK++;
      mode = FT8_MODE;
      break;
    case 50318000:
      freq_OK++;
      mode = FT4_MODE;
      break;
    /*
    case 50318000:
      freq_OK++;
      mode = JS8_MODE;
      break;
    */
    case 50293000:
      freq_OK++;
      mode = WSP_MODE;
      break;
    case 50310000:
      freq_OK++;
      mode = J65_MODE;
      break;
  }
  return (freq_OK);
}

// update the VFO frequency
void update_freq(uint32_t freq) {
  if (!freq2band(freq)) {
    // requested frequency is not OK
    error_blink();
  }
  si5351.set_freq(freq*100, SI5351_CLK1);
  base_freq = freq;
  getmode();
  oled.clrScreen();
  oled.printline(0, modestr);
  oled.print32(base_freq);
}

// check for display timeout
void check_timeout() {
  if ((display == ON) && ((msTimer - xtimer) > TIMEOUT)) {
    display = OFF;
    oled.noDisplay();
  }
}

// check the UI pushbutton
void check_UI() {
  uint8_t  event = NBP;
  uint32_t t0;
  if (UIKEY) {
    event = BSC;
    t0 = msTimer;
    reset_xtimer();
    // check for long button press
    while (UIKEY && (event != BPL)) {
      if (msTimer > (t0 + LONGPRESS)) event = BPL;
      wait_ms(DEBOUNCE);
      check_CAT();
    }
    // check for short click
    if (event == BSC) refresh();
    if (event == BPL) {
      tuning_hdr();
      reset_xtimer();
      while (UIKEY && (event != DLP)) {
        if (msTimer > (t0 + XLPRESS)) event = DLP;
        wait_ms(DEBOUNCE);
        check_CAT();
      }
      if (event == DLP) {
        show_version(LOCAL);
        // wait for release
        while (UIKEY) {
          wait_ms(DEBOUNCE);
          check_CAT();
        }
        wait_ms(DEBOUNCE);
        refresh();
      } else {
        tuning_mode();  // event is BPL
      }
    }
  } else {
    check_timeout();   // check for display timeout
  }
}

// reset display timeout
void reset_xtimer() {
  xtimer = msTimer;
  if (display == OFF) {
    display = ON;
    oled.onDisplay();
  }
}

// print mode/band/freq
void refresh() {
  read_adc();
  reset_xtimer();
  update_freq(base_freq);
}

#define INIT_FREQ 14074000ULL  // 20M FT8

// reset (CAT command)
void do_reset(uint8_t soft) {
  oled.clrScreen();
  if (soft) {
    // soft reset
    oled.putstr("SOFT RESET");
    init_uart();
    Serial.print("  Soft Reset\r\n");
    Serial.print("  Reading EEPROM\r\n");
    cal_data = eeprom.get32(DATA_ADDR);
    update_freq(eeprom.get32(FREQ_ADDR));
  } else {
    // factory reset
    oled.putstr("FACTORY RESET");
    init_uart();
    Serial.print("  Factory Reset\r\n");
    cal_data = CAL_DATA_INIT;
    update_freq(INIT_FREQ);
    save_eeprom();
  }
  show_cal();
  set_tx_status(RX);
  wait_ms(TWO_SECONDS);
  refresh();
}

#define CAL_FREQ  100000000ULL

#define CAL_MSG  "\r\n\
  Adjust cal freq to 1 MHz\r\n\
  press + to increase cal freq\r\n\
  press - to decrease cal freq\r\n\
  press = to stop\r\n\
  press . to save and exit\r\n\
  press / to exit without saving\r\n\n"

// calibrate the VFO (CAT command)
void run_calibrate() {
  char ch;
  uint8_t up = FALSE;
  uint8_t dn = FALSE;
  uint8_t xx = 0;
  uint8_t done = NO;
  uint8_t save = YES;
  reset_xtimer();
  // print to serial port
  Serial.print(CAL_MSG);
  // print to OLED
  oled.clrScreen();
  oled.putstr("CALIBRATION MODE");
  // update the VFO
  set_tx_status(RX);
  si5351.set_freq(CAL_FREQ, SI5351_CLK2);
  si5351.set_clock_pwr(SI5351_CLK2, ON);
  si5351.output_enable(SI5351_CLK2, ON);
  ch = getc();
  while (!done) {
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
      case '/':      // exit without saving
      case '\\':
        save = NO;
      case '.':      // exit and save
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
      wait_us(100);
      si5351.set_freq(CAL_FREQ, SI5351_CLK2);
      if (xx == 0) Serial.print(ch);
      if (xx++ == 100) xx = 0;
    }
    ch = gcal(ch);
  }
  si5351.output_enable(SI5351_CLK2, OFF);
  si5351.set_clock_pwr(SI5351_CLK2, OFF);
  // print to serial port
  Serial.print("\r\n\  Exiting Calibration Mode\r\n");
  show_cal();
  // print to OLED
  oled.printline(0,"CAL COMPLETE");
  if (save) {
    Serial.print("  Saving to EEPROM\r\n");
    eeprom.put32(DATA_ADDR, cal_data);
  }
  wait_ms(TWO_SECONDS);
  refresh();
}

// main code starts here
int main() {
  // setup
  init();
  init_pins();
  init_timer0();
  init_timer1();
  init_adc();
  init_i2c();
  init_VFO();
  init_oled();
  init_check();
  init_freq();
  refresh();
  loopCount=0;
  // main loop
  while (TRUE) {
    check_CAT();                // check CAT interface
    check_UI();                 // check UI pushbutton
    if (doFSK) FSK_tone();      // measure FSK frequency
    if (FSKtx) check_VOX();     // check for VOX timeout
    if (loopCount > TWO_SECONDS) {
      // LED heartbeat
      if (tx_status != TX) blinkLED();
      loopCount=0;
    }
  }
  return 0;
}

