// rx_200loc.ino - https://github.com/threeme3/usdx-sketch
//
//  Copyright 2019, 2020, 2021, 2022, 2023, 2024   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  License:   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  Install:   Adjust configuration definition below and select the include file corresponding to your hardware, add libraries LiquidCrystal/LiquidCrystal_I2C/U8g2 to Tools>Manage Libaries.
//             Select Tools>Board>Arduino Uno, Tools>Programmer>Arduino as ISP, Tools>Port>ttyUSBx/ttyACMx/COMx, and select Upload Using Programmer or Upload when using a bootloader.
//
#include <Wire.h>
#include <si5351.h>              // https://github.com/etherkit/Si5351Arduino
//#include "LiquidCrystal.h"     // https://github.com/arduino-libraries/LiquidCrystal
//#include "LiquidCrystal_I2C.h" // https://github.com/johnrickman/LiquidCrystal_I2C
#include <U8g2lib.h>             // https://github.com/olikraus/u8g2
#define F_XTAL  27000000UL       // 25/27MHz SI5351,    enter here actual crystal frequency
#define F_CPU_  20000000UL       // 16/20MHz ATMEGA328, enter here actual crystal frequency

// arduino pin defintions https://images.prismic.io/circuito/8e3a980f0f964cc539b4cbbba2654bb660db6f52_arduino-uno-pinout-diagram.png?auto=compress,format
#define ROT_A   6         //PD6 - encoder
#define ROT_B   7         //PD7 - encoder
#define SIDETONE 9        //PB1 - speaker output
#define ADC_Q   14        //PC0/A0 - RX Q input
#define ADC_I   15        //PC1/A1 - RX I input
#define BUTTONS 17        //PC3/A3 - encoder, right and left buttons (resp. thresholds 1VCC 0.82VCC 0.68VCC)

#ifdef LiquidCrystal_h
LiquidCrystal lcd(18, 4, 0, 1, 2, 3); // LCD1602 at parallel interface
#endif //LiquidCrystal_h

#ifdef LiquidCrystal_I2C_h
LiquidCrystal_I2C lcd(0x27, 16, 2);   // LCD1602 at standard I2C bus
#endif //LiquidCrystal_I2C_h

#ifdef U8G2LIB_HH
U8G2_SSD1306_128X64_NONAME_2_SW_I2C u8g2(U8G2_R0, 3, 2, U8X8_PIN_NONE);  // OLED at SCL(pin 3) SDA(pin 2) see complete list: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);     // OLED at standard I2C bus
#endif //U8G2LIB_HH

static Si5351 si5351;  // SI5351 at standard I2C bus

void set_freq(int32_t freq){
  const uint64_t pll_freq = 64800000000ULL;
  si5351.set_freq_manual(freq * 100ULL, pll_freq, SI5351_CLK0);
  si5351.set_freq_manual(freq * 100ULL, pll_freq, SI5351_CLK1);
  si5351.set_phase(SI5351_CLK0, pll_freq / (freq * 100ULL) ); //90 degrees phase difference
  si5351.set_phase(SI5351_CLK1, 0 );
  si5351.pll_reset(SI5351_PLLA);
#ifdef LiquidCrystal_h
  Wire.end(); // after Wire activity, finalize with Wire.end() so that LCD can regain control on RS pin
#endif //LiquidCrystal_h
}

static int16_t hilb_i(int16_t ac){
  static int16_t v[7];  // delay-line to match Hilbert transform on Q branch
  for(int8_t i = 0; i != 6; i++) v[i] = v[i+1]; v[6] = ac;
  return v[0]; 
}
static int16_t hilb_q(int16_t ac){
  static int16_t v[14];  // delay-line
  for(int8_t i = 0; i != 13; i++) v[i] = v[i+1]; v[13] = ac;
  return ((v[0] - v[13]) + (v[2] - v[12]) * 4) / 64 + ((v[4] - v[10]) + (v[6] - v[8])) / 8 + ((v[4] - v[10]) * 5 - (v[6] - v[8]) ) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 43dB side-band rejection in 650..3400Hz (@8kSPS) when used in image-rejection scenario; (Hilbert transform require 4 additional bits)
}

class Receiver {
  uint8_t _init = 0;
public:
  uint8_t log2(uint16_t x){ uint8_t y = 0; for(; x>>=1;) y++; return y; }

  void begin(){
    _init = 1;
    sdr_rx_00();
    init_adc(); init_dac();
    
    set_dac_sample_rate(78125);
    set_clk_interrupt(62500);  // start timer 2 interrupt clock used as ADC sample clock and subsequent DSP and audio output chain
    set_dac_audio_enable(true);  // speaker output enable
  }

  void end(){
    set_dac_audio_enable(false);     // speaker output disable
    clk_interrupt_enable(false);  // stop processing
  }
    
  int16_t agc_gain = 1 << 10;  
  void process_agc_fast_det(int16_t in){
    int16_t accum = (1 - abs(in >> 10));
    if((INT16_MAX - agc_gain) > accum) agc_gain = agc_gain + accum;
    if(agc_gain < 1) agc_gain = 1;
  }
  int16_t process_agc_fast(int16_t in){
    int16_t out = (agc_gain >= 1024) ? (agc_gain >> 10) * in : in;
    return out;
  }

  enum mode_t { USB, LSB };
  uint8_t mode;
  int8_t volume = 11;

  void process(int16_t i, int16_t q){
    static int16_t ac3;
    dac_upsample(ac3);

    int16_t qh = hilb_q(q >> 2);
    int16_t ih = hilb_i(i >> 2);

    int16_t ac = (mode == USB) ? -(ih - qh) : -(ih + qh);
    ac = process_agc_fast(ac);
    ac = ac >> (16-volume);
    process_agc_fast_det(ac << (16-volume));
    ac3 = min(max(ac, -(1<<9)), (1<<9)-1 );
  }

  void init_adc(){
    DIDR0 |= 0x07; // init ADC, disable digital input for ADC0,1,2
    ADCSRA = (1 << ADEN) | ((uint8_t)log2((uint8_t)(F_CPU_ / 13 / 96153 ))) & 0x07; ADCSRB = 0; // Enable ADC, ADC Prescaler (for normal conversions non-auto-triggered): ADPS = log2(F_CPU / 13 / Fs) - 1; ADSP=0..7 resulting in resp. conversion rate of 1536, 768, 384, 192, 96, 48, 24, 12 kHz
  }
  void init_dac(){
    TCCR1A = (1 << WGM11);  // init (DAC) PWM for AUDIO (OC1A) and PA Envelope (OC1B):  Timer 1: OC1A and OC1B in PWM mode, Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode)
    TCCR1B = (1 << CS10) | (1 << WGM13) | (1 << WGM12); // Mode 14 - Fast PWM;  CS10: clkI/O/1 (No prescaling)    
  }
  void set_dac_audio_enable(bool val){ if(val){ TCCR1A |= (1 << COM1A1); pinMode(SIDETONE, OUTPUT); } else { pinMode(SIDETONE, INPUT); TCCR1A &= ~(1 << COM1A1); } };  // enable PWM, speaker sees high-impedance when disabled
  void set_dac_sample_rate(uint32_t fs){ ICR1L = min(255, F_CPU_ / fs); ICR1H = 0x00; } // PWM value range (fs>78431):  Fpwm = F_CPU / [Prescaler * (1 + TOP)]

  void set_clk_interrupt(uint16_t fs){
    ASSR &= ~(1 << AS2);    // Timer 2 interrupt mode: clocked from CLK I/O (like Timer 0 and 1)
    TCNT2 = 0;
    TCCR2A = (1 << WGM21);  // WGM21: Mode 2 - CTC (Clear Timer on Compare Match)
    TCCR2B = (1 << CS22);   // Set C22 bits for 64 prescaler
    OCR2A = ((F_CPU_ / 64) / fs) - 1;   // OCRn = (F_CPU / pre-scaler / fs) - 1;
    clk_interrupt_enable(true);
  }
  void clk_interrupt_enable(bool val = true){ if(val) TIMSK2 |= (1 << OCIE2A); else TIMSK2 &= ~(1 << OCIE2A); } // enable timer compare interrupt clk_COMPA_vect

  static void set_adc_mux(uint8_t adcpin){ ADMUX = (adcpin-14)|(1<<REFS1)|(1<<REFS0); }
  static uint16_t get_adc(){ uint16_t adc = ADC; ADCSRA |= (1 << ADSC); return adc; }
  static int16_t get_adc(uint8_t adcpin){ set_adc_mux(adcpin); ADCSRA |= (1 << ADSC); return ADC - 511; }  // returns unbiased ADC input
  static int16_t sample_corr(int16_t ac){ static int16_t prev_adc; return (prev_adc + ac) / 2, prev_adc = ac; } // I/Q needs to be sampled at the same time, since there is only one ADC, I/Q samples are taken in sequence; this function corrects the unwanted time-offset by averaging

  void dac_upsample(int16_t ac){
    static int16_t ozd1, ozd2;  // output stage
    if(_init){ ac = 0; ozd1 = 0; ozd2 = 0; ozi1 = 0; ozi2 = 0; _init = 0; } // hack: on first sample init accumulators of further stages (to prevent instability)  
    int16_t od1 = ac - ozd1; // comb section
    ocomb = od1 - ozd2;
    interrupts();  // hack, since slow_dsp process exceeds rx sample-time, allow subsequent 7 interrupts for further rx sampling while processing, prevent nested interrupts with tc  
    ozd2 = od1;
    ozd1 = ac;
  }

  static int16_t ocomb, ozi1, ozi2;
  static int16_t set_duc_audio(){
    ozi1 = ocomb + ozi1;
    ozi2 = ozi1 + ozi2;          // Integrator section
    OCR1AL = min(max((ozi2>>5) + 128, 0), 255);
  }
};
Receiver recv;

typedef void (*func_t)(void);
volatile func_t func_ptr;
ISR(TIMER2_COMPA_vect){  // Timer2 COMPA interrupt
  func_ptr();
}

int16_t Receiver::ocomb, Receiver::ozi1, Receiver::ozi2, c[13];
void sdr_rx_00(){ int16_t ac = Receiver::sample_corr(Receiver::get_adc(ADC_I));     func_ptr = sdr_rx_01;  int16_t i_s1za0 = (ac + (c[0] + c[1]) * 3 + c[2]) >> 1; c[0] = ac; int16_t ac2 = (i_s1za0 + (c[3] + c[4]) * 3 + c[5]); c[3] = i_s1za0; recv.process(ac2, c[12]); }
void sdr_rx_02(){ int16_t ac = Receiver::sample_corr(Receiver::get_adc(ADC_I));     func_ptr = sdr_rx_03;  c[2] = c[1]; c[1] = ac; }
void sdr_rx_04(){ int16_t ac = Receiver::sample_corr(Receiver::get_adc(ADC_I));     func_ptr = sdr_rx_05;  c[5] = c[4]; c[4] = (ac + (c[0] + c[1]) * 3 + c[2]) >> 1; c[0] = ac; }
void sdr_rx_06(){ int16_t ac = Receiver::sample_corr(Receiver::get_adc(ADC_I));     func_ptr = sdr_rx_07;  c[2] = c[1]; c[1] = ac; }
void sdr_rx_01(){ int16_t ac = Receiver::get_adc(ADC_Q); Receiver::set_duc_audio(); func_ptr = sdr_rx_02;  c[8] = c[7]; c[7] = ac; }
void sdr_rx_03(){ int16_t ac = Receiver::get_adc(ADC_Q); Receiver::set_duc_audio(); func_ptr = sdr_rx_04;  c[11] = c[10]; c[10] = (ac + (c[6] + c[7]) * 3 + c[8]) >> 1; c[6] = ac; }
void sdr_rx_05(){ int16_t ac = Receiver::get_adc(ADC_Q); Receiver::set_duc_audio(); func_ptr = sdr_rx_06;  c[8] = c[7]; c[7] = ac; }
void sdr_rx_07(){ int16_t ac = Receiver::get_adc(ADC_Q); Receiver::set_duc_audio(); func_ptr = sdr_rx_00;  int16_t q_s1za0 = (ac + (c[6] + c[7]) * 3 + c[8]) >> 1; c[6] = ac; c[12] = (q_s1za0 + (c[9] + c[10]) * 3 + c[11]); c[9] = q_s1za0; }

template<uint8_t rota_pin = 6, uint8_t rotb_pin = 7> 
class Encoder {
  uint8_t last_state;
  uint8_t _pin_to_pio(uint8_t pin){ return (pin<8?16:pin<14?-8:-6)+pin; }  // translates Arduino pin to logical bit representation 0..23 mappable on PB/PC/PD0-7/PCINT0..23
  void set_interrupt_enable_pin(uint8_t pin){ *((volatile uint8_t *)&PCMSK0 + _pin_to_pio(pin)/8) |= 1 << _pin_to_pio(pin)%8, PCICR |= 1 << _pin_to_pio(pin)/8; }
public:
  int16_t step;
  volatile int32_t value;
  Encoder(const int32_t init_val = 7074000, const bool swap = false, const int16_t _step = 500) : value(init_val), step((swap) ? -_step : _step) {
    pinMode(rota_pin, INPUT_PULLUP); pinMode(rotb_pin, INPUT_PULLUP);
    set_interrupt_enable_pin(rota_pin); set_interrupt_enable_pin(rotb_pin);
  }
  void event(){
    last_state = (last_state << 4) | (digitalRead(rotb_pin) << 1) | digitalRead(rota_pin);
    if(last_state == 0x23) value += step; if(last_state == 0x32) value -= step;
  }
};
Encoder<ROT_A, ROT_B> enc;
ISR(PCINT2_vect){ // Interrupt on rotary encoder turn
  enc.event();
}

void setup(){
  digitalWrite(10, LOW); pinMode(10, OUTPUT); // potential PA bias
  digitalWrite(8, HIGH); pinMode(8, OUTPUT);  // potential RX switch
  pinMode(BUTTONS, INPUT);

#ifdef LiquidCrystal_h
  lcd.begin(16, 2);
#endif //LiquidCrystal_h
#ifdef LiquidCrystal_I2C_h
  lcd.init();
  lcd.backlight();
#endif //LiquidCrystal_I2C_h
#ifdef U8G2LIB_HH
  u8g2.setBusClock(800000);
  u8g2.begin();
  u8g2.setFont(u8g2_font_freedoomr10_tu);
#endif //U8G2LIB_HH

  Wire.begin();
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, F_XTAL, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  recv.begin();
}

void loop(){
  static int32_t freq;
  if(freq != enc.value){  // encoder change
    freq = enc.value; 
    set_freq(freq);
#ifdef U8G2LIB_HH
    u8g2.firstPage(); u8g2.setCursor(0, 12); u8g2.print(freq); u8g2.print(' '); u8g2.print( (recv.mode == Receiver::USB) ? 'U' : 'L'); u8g2.print(' '); u8g2.nextPage();
#else
    lcd.setCursor(0, 1); lcd.print(freq); lcd.print(' '); lcd.print( (recv.mode == Receiver::USB) ? 'U' : 'L'); lcd.print(' ');
#endif //U8G2LIB_HH
  }
  if(digitalRead(BUTTONS)){
    int32_t t0 = millis(); while(digitalRead(BUTTONS)); if((millis() - t0) > 500){  // long button press: change mode
      if(recv.mode++ >= 1) recv.mode = 0; freq++;
    } else {  // short press: change band
      static int32_t freqs[] = { 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000 };
      int i; for(i = 0; i != 9; i++) if(freqs[i] > (enc.value + 1700000)){ enc.value = freqs[i]; break; } if(i == 9) enc.value = freqs[0];
    }
    delayMicroseconds(1000);  // debounce
  }
}
