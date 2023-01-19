
// ============================================================================
//
// si5351.cpp - Si5351 library for Arduino
//
// Copyright (C) 2015 - 2019 Jason Milldrum <milldrum@gmail.com>
//                           Dana H. Myers <k6jq@comcast.net>
//
// Some tuning algorithms derived from clk-si5351.c in the Linux kernel.
// Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
// Rabeeh Khoury <rabeeh@solid-run.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see https://www.gnu.org/licenses/
//
// ============================================================================

#include <stdint.h>
#include "Arduino.h"
#include "Wire.h"
#include "si5351.h"

// Public functions

void Si5351::init(void) {
  uint8_t i;
  // 25 MHz XO ref osc
  xtal_freq[SI5351_PLL_INPUT_XO] = SI5351_XTAL_FREQ;
  // use XO ref osc as default for each PLL
  plla_ref_osc = SI5351_PLL_INPUT_XO;
  pllb_ref_osc = SI5351_PLL_INPUT_XO;
  // no clock divider
  clkin_div = SI5351_CLKIN_DIV_1;
  // set 8pF crystal load capacitance
  write_reg(SI5351_CRYSTAL_LOAD, 0x92);
  // the clock disable state is low
  write_reg(SI5351_CLK3_0_DISABLE_STATE, 0x00);
  write_reg(SI5351_OEB_PIN_ENABLE_CTRL,  0xFF);
  // init clocks
  for(i = 16; i < 19; i++) write_reg(i, 0x80);
  for(i = 16; i < 19; i++) write_reg(i, 0x0C);
  write_reg(SI5351_OUTPUT_ENABLE_CTRL, 0xFF);
  // set PLLA and PLLB to 800 MHz for automatic tuning
  set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  set_pll(SI5351_PLL_FIXED, SI5351_PLLB);
  // make PLL to CLK assignments for automatic tuning
  for(i = 0; i < 3; i++) {
    pll_assignment[i] = SI5351_PLLA;
    set_ms_source(i, SI5351_PLLA);
  }
  // reset the VCXO param
  write_reg(SI5351_VXCO_PARAMETERS_LOW, 0);
  write_reg(SI5351_VXCO_PARAMETERS_MID, 0);
  write_reg(SI5351_VXCO_PARAMETERS_HIGH, 0);
  // then reset the PLLs
  pll_reset(SI5351_PLLA);
  pll_reset(SI5351_PLLB);
}

void Si5351::set_freq(uint64_t freq, uint8_t clk) {
  struct Si5351RegSet ms_reg;
  uint64_t pll_freq;
  uint8_t int_mode = 0;
  uint8_t div_by_4 = 0;
  uint8_t r_div = 0;
  // select the proper R div value
  r_div = select_r_div(&freq);
  // calculate the synth parameters
  if (pll_assignment[clk] == SI5351_PLLA) {
    multisynth_calc(freq, plla_freq, &ms_reg);
  } else {
    multisynth_calc(freq, pllb_freq, &ms_reg);
  }
  // set multisynth registers
  set_ms(clk, ms_reg, int_mode, r_div, div_by_4);
}

void Si5351::set_pll(uint64_t pll_freq, uint8_t target_pll) {
  struct Si5351RegSet pll_reg;
  if (target_pll == SI5351_PLLA) {
    pll_calc(SI5351_PLLA, pll_freq, &pll_reg, ref_correction[plla_ref_osc], 0);
  } else {
    pll_calc(SI5351_PLLB, pll_freq, &pll_reg, ref_correction[pllb_ref_osc], 0);
  }
  // prepare an array for parameters to be written to
  uint8_t *params = new uint8_t[20];
  uint8_t i = 0;
  uint8_t temp;
  // registers 26-27
  temp = ((pll_reg.p3 >> 8) & 0xFF);
  params[i++] = temp;
  temp = (uint8_t)(pll_reg.p3  & 0xFF);
  params[i++] = temp;
  // register 28
  temp = (uint8_t)((pll_reg.p1 >> 16) & 0x03);
  params[i++] = temp;
  // registers 29-30
  temp = (uint8_t)((pll_reg.p1 >> 8) & 0xFF);
  params[i++] = temp;
  temp = (uint8_t)(pll_reg.p1  & 0xFF);
  params[i++] = temp;
  // register 31
  temp = (uint8_t)((pll_reg.p3 >> 12) & 0xF0);
  temp += (uint8_t)((pll_reg.p2 >> 16) & 0x0F);
  params[i++] = temp;
  // registers 32-33
  temp = (uint8_t)((pll_reg.p2 >> 8) & 0xFF);
  params[i++] = temp;
  temp = (uint8_t)(pll_reg.p2  & 0xFF);
  params[i++] = temp;
  // write the parameters
  if (target_pll == SI5351_PLLA) {
    write_bulk(SI5351_PLLA_PARAMETERS, i, params);
    plla_freq = pll_freq;
  } else if (target_pll == SI5351_PLLB) {
    write_bulk(SI5351_PLLB_PARAMETERS, i, params);
    pllb_freq = pll_freq;
  }
  delete params;
}

void Si5351::set_ms(uint8_t clk, struct Si5351RegSet ms_reg, uint8_t int_mode, uint8_t r_div, uint8_t div_by_4) {
  uint8_t *params = new uint8_t[20];
  uint8_t i = 0;
  uint8_t temp;
  uint8_t reg_val;
  // registers 42-43 for CLK0
  temp = (uint8_t)((ms_reg.p3 >> 8) & 0xFF);
  params[i++] = temp;
  temp = (uint8_t)(ms_reg.p3  & 0xFF);
  params[i++] = temp;
  // register 44 for CLK0
  reg_val = read_reg((SI5351_CLK0_PARAMETERS + 2) + (clk * 8));
  reg_val &= ~(0x03);
  temp = reg_val | ((uint8_t)((ms_reg.p1 >> 16) & 0x03));
  params[i++] = temp;
  // registers 45-46 for CLK0
  temp = (uint8_t)((ms_reg.p1 >> 8) & 0xFF);
  params[i++] = temp;
  temp = (uint8_t)(ms_reg.p1  & 0xFF);
  params[i++] = temp;
  // register 47 for CLK0
  temp = (uint8_t)((ms_reg.p3 >> 12) & 0xF0);
  temp += (uint8_t)((ms_reg.p2 >> 16) & 0x0F);
  params[i++] = temp;
  // registers 48-49 for CLK0
  temp = (uint8_t)((ms_reg.p2 >> 8) & 0xFF);
  params[i++] = temp;
  temp = (uint8_t)(ms_reg.p2  & 0xFF);
  params[i++] = temp;
  // write the parameters
  switch(clk) {
    case SI5351_CLK0:
      write_bulk(SI5351_CLK0_PARAMETERS, i, params);
      set_int(clk, int_mode);
      ms_div(clk, r_div, div_by_4);
      break;
    case SI5351_CLK1:
      write_bulk(SI5351_CLK1_PARAMETERS, i, params);
      set_int(clk, int_mode);
      ms_div(clk, r_div, div_by_4);
      break;
    case SI5351_CLK2:
      write_bulk(SI5351_CLK2_PARAMETERS, i, params);
      set_int(clk, int_mode);
      ms_div(clk, r_div, div_by_4);
      break;
    default:
      break;
  }
  delete params;
}

void Si5351::output_enable(uint8_t clk, uint8_t enable) {
  uint8_t reg_val;
  reg_val = read_reg(SI5351_OUTPUT_ENABLE_CTRL);
  if (enable == 1) {
    reg_val &= ~(1<<clk);
  } else {
    reg_val |= (1<<clk);
  }
  write_reg(SI5351_OUTPUT_ENABLE_CTRL, reg_val);
}

void Si5351::drive_strength(uint8_t clk, uint8_t drive) {
  uint8_t reg_val;
  const uint8_t mask = 0x03;
  reg_val = read_reg(SI5351_CLK0_CTRL + clk);
  reg_val &= ~(mask);
  reg_val |= drive;
  write_reg(SI5351_CLK0_CTRL + clk, reg_val);
}

void Si5351::set_correction(int32_t corr, uint8_t ref_osc) {
  ref_correction[ref_osc] = corr;
  // recalculate and set PLL freqs based on correction value
  set_pll(plla_freq, SI5351_PLLA);
  set_pll(pllb_freq, SI5351_PLLB);
}

void Si5351::pll_reset(uint8_t target_pll) {
  if (target_pll == SI5351_PLLA) {
    write_reg(SI5351_PLL_RESET, SI5351_PLL_RESET_A);
  } else if (target_pll == SI5351_PLLB) {
      write_reg(SI5351_PLL_RESET, SI5351_PLL_RESET_B);
  }
}

void Si5351::set_ms_source(uint8_t clk, uint8_t pll) {
  uint8_t reg_val;
  reg_val = read_reg(SI5351_CLK0_CTRL + clk);
  if (pll == SI5351_PLLA) {
    reg_val &= ~(SI5351_CLK_PLL_SELECT);
  } else if (pll == SI5351_PLLB) {
    reg_val |= SI5351_CLK_PLL_SELECT;
  }
  write_reg(SI5351_CLK0_CTRL + clk, reg_val);
  pll_assignment[clk] = pll;
}

void Si5351::set_int(uint8_t clk, uint8_t enable) {
  uint8_t reg_val;
  reg_val = read_reg(SI5351_CLK0_CTRL + clk);
  if (enable == 1) {
    reg_val |= (SI5351_CLK_INTEGER_MODE);
  } else {
    reg_val &= ~(SI5351_CLK_INTEGER_MODE);
  }
  write_reg(SI5351_CLK0_CTRL + clk, reg_val);

}

void Si5351::set_clock_pwr(uint8_t clk, uint8_t pwr) {
  uint8_t reg_val;
  reg_val = read_reg(SI5351_CLK0_CTRL + clk);
  if (pwr == 1)  {
    reg_val &= 0b01111111;
  } else {
    reg_val |= 0b10000000;
  }
  write_reg(SI5351_CLK0_CTRL + clk, reg_val);
}

void Si5351::write_bulk(uint8_t addr, uint8_t bytes, uint8_t *data) {
  Wire.beginTransmission(SI5351_I2C_ADDR);
  Wire.write(addr);
  for(int i = 0; i < bytes; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

void Si5351::write_reg(uint8_t addr, uint8_t data) {
  Wire.beginTransmission(SI5351_I2C_ADDR);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t Si5351::read_reg(uint8_t addr) {
  uint8_t reg_val = 0;
  Wire.beginTransmission(SI5351_I2C_ADDR);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(SI5351_I2C_ADDR, 1, 0);
  while(Wire.available()) {
    reg_val = Wire.read();
  }
  return reg_val;
}

// private functions

uint64_t Si5351::pll_calc(uint8_t pll, uint64_t freq, struct Si5351RegSet *reg, int32_t corr, uint8_t vcxo) {
  uint64_t ref_freq;
  if (pll == SI5351_PLLA) {
    ref_freq = xtal_freq[plla_ref_osc] * SI5351_FREQ_MULT;
  } else {
    ref_freq = xtal_freq[pllb_ref_osc] * SI5351_FREQ_MULT;
  }
  //ref_freq = 15974400ULL * SI5351_FREQ_MULT;
  uint32_t a, b, c, p1, p2, p3;
  uint64_t lltmp;
  // factor calibration value into nominal crystal frequency
  // measured in parts-per-billion
  ref_freq = ref_freq + (int32_t)((((((int64_t)corr) << 31) / 1000000000LL) * ref_freq) >> 31);
  // PLL bounds checking
  if (freq < SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT) {
    freq = SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT;
  }
  if (freq > SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT) {
    freq = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT;
  }
  // determine integer part of feedback equation
  a = freq / ref_freq;
  if (a < SI5351_PLL_A_MIN) {
    freq = ref_freq * SI5351_PLL_A_MIN;
  }
  if (a > SI5351_PLL_A_MAX) {
    freq = ref_freq * SI5351_PLL_A_MAX;
  }
  // find best approximation for b/c = fVCO mod fIN
  if (vcxo) {
    b = (((uint64_t)(freq % ref_freq)) * 1000000ULL) / ref_freq;
    c = 1000000ULL;
  } else {
    b = (((uint64_t)(freq % ref_freq)) * RFRAC_DENOM) / ref_freq;
    c = b ? RFRAC_DENOM : 1;
  }
  // calculate parameters
  p1 = 128 * a + ((128 * b) / c) - 512;
  p2 = 128 * b - c * ((128 * b) / c);
  p3 = c;
  // recalculate frequency as fIN * (a + b/c)
  lltmp = ref_freq;
  lltmp *= b;
  do_div(lltmp, c);
  freq = lltmp;
  freq += ref_freq * a;
  reg->p1 = p1;
  reg->p2 = p2;
  reg->p3 = p3;
  if (vcxo) {
    return (uint64_t)(128 * a * 1000000ULL + b);
  } else {
    return freq;
  }
}

uint64_t Si5351::multisynth_calc(uint64_t freq, uint64_t pll_freq, struct Si5351RegSet *reg) {
  uint64_t lltmp;
  uint32_t a, b, c, p1, p2, p3;
  uint8_t divby4 = 0;
  uint8_t ret_val = 0;
  // bounds checking
  if (freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT)  {
    freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;
  }
  if (freq < SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT)  {
    freq = SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT;
  }
  if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT)  {
    divby4 = 1;
  }
  if (pll_freq == 0) {
    // find largest integer divider for max
    // VCO frequency and given target frequency
    if (divby4 == 0) {
      lltmp = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT; // margin needed?
      do_div(lltmp, freq);
      if (lltmp == 5) {
        lltmp = 4;
      } else if (lltmp == 7) {
        lltmp = 6;
      }
      a = (uint32_t)lltmp;
    }
    else {
      a = 4;
    }
    b = 0;
    c = 1;
    pll_freq = a * freq;
  } else {
    // preset PLL, so return the actual freq for these params instead of PLL freq
    ret_val = 1;
    // determine integer part of feedback equation
    a = pll_freq / freq;
    if (a < SI5351_MULTISYNTH_A_MIN)    {
      freq = pll_freq / SI5351_MULTISYNTH_A_MIN;
    }
    if (a > SI5351_MULTISYNTH_A_MAX) {
      freq = pll_freq / SI5351_MULTISYNTH_A_MAX;
    }
    b = (pll_freq % freq * RFRAC_DENOM) / freq;
    c = b ? RFRAC_DENOM : 1;
  }
  // calculate parameters
  if (divby4 == 1) {
    p3 = 1;
    p2 = 0;
    p1 = 0;
  } else {
    p1 = 128 * a + ((128 * b) / c) - 512;
    p2 = 128 * b - c * ((128 * b) / c);
    p3 = c;
  }
  reg->p1 = p1;
  reg->p2 = p2;
  reg->p3 = p3;
  if (ret_val == 0) {
    return pll_freq;
  } else {
    return freq;
  }
}

void Si5351::ms_div(uint8_t clk, uint8_t r_div, uint8_t div_by_4) {
  uint8_t reg_val = 0;
  uint8_t reg_addr = 0;
  switch(clk) {
    case SI5351_CLK0:
      reg_addr = SI5351_CLK0_PARAMETERS + 2;
      break;
    case SI5351_CLK1:
      reg_addr = SI5351_CLK1_PARAMETERS + 2;
      break;
    case SI5351_CLK2:
      reg_addr = SI5351_CLK2_PARAMETERS + 2;
      break;
    default:
      break;
  }
  reg_val = read_reg(reg_addr);
  reg_val &= ~(0x7C);
  if (div_by_4 == 0) {
    reg_val &= ~(SI5351_OUTPUT_CLK_DIVBY4);
  } else {
    reg_val |= (SI5351_OUTPUT_CLK_DIVBY4);
  }
  reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);
  write_reg(reg_addr, reg_val);
}

// select R divider
uint8_t Si5351::select_r_div(uint64_t *freq) {
  uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;
  if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 2)) {
    r_div = SI5351_OUTPUT_CLK_DIV_128;
    *freq *= 128ULL;
  }
  else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 2) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 4)) {
    r_div = SI5351_OUTPUT_CLK_DIV_64;
    *freq *= 64ULL;
  }
  else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 4) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 8)) {
    r_div = SI5351_OUTPUT_CLK_DIV_32;
    *freq *= 32ULL;
  }
  else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 8) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 16)) {
    r_div = SI5351_OUTPUT_CLK_DIV_16;
    *freq *= 16ULL;
  }
  else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 16) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 32)) {
    r_div = SI5351_OUTPUT_CLK_DIV_8;
    *freq *= 8ULL;
  }
  else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 32) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 64)) {
    r_div = SI5351_OUTPUT_CLK_DIV_4;
    *freq *= 4ULL;
  }
  else if ((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 64) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128)) {
    r_div = SI5351_OUTPUT_CLK_DIV_2;
    *freq *= 2ULL;
  }
  return r_div;
}

// power down
void Si5351::powerDown(void) {
  // disable all clocks
  write_reg(SI5351_OUTPUT_ENABLE_CTRL, 0xff);
  for (uint8_t addr=16; addr<24; addr++) {
    write_reg(addr, 0x80);
  }
}

