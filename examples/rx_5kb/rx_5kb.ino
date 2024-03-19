// rx_5kb.ino - https://github.com/threeme3/Arduino_uSDX_Library
//
//  Copyright 2019, 2020, 2021, 2022, 2023, 2024   Guido PE1NNZ <pe1nnz@amsat.org>
//
//  License:   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  Adjust configuration definition below and select the include file corresponding to your hardware, add libraries LiquidCrystal/LiquidCrystal_I2C/SSD1306Ascii to Tools>Manage Libaries, select Tools>Board>Arduino Uno, Tools>Programmer>Arduino as ISP, Tools>Port>ttyUSBx/ttyACMx/COMx, and select Upload Using Programmer or Upload when using a bootloader.
//
#include "LiquidCrystal.h"     // https://github.com/arduino-libraries/LiquidCrystal
//#include "LiquidCrystal_I2C.h" // https://github.com/johnrickman/LiquidCrystal_I2C
//#include "SSD1306Ascii.h"        // https://github.com/greiman/SSD1306Ascii
#define F_XTAL  27000000UL       // 25/27MHz SI5351,    enter here actual crystal frequency
#define F_CPU_  20000000UL       // 16/20MHz ATMEGA328, enter here actual crystal frequency

// arduino pin defintions https://images.prismic.io/circuito/8e3a980f0f964cc539b4cbbba2654bb660db6f52_arduino-uno-pinout-diagram.png?auto=compress,format
#define ROT_A   6         //PD6 - encoder
#define ROT_B   7         //PD7 - encoder
#define SIDETONE 9        //PB1 - speaker output
#define ADC_Q   14        //PC0/A0 - RX Q input
#define ADC_I   15        //PC1/A1 - RX I input
#define BUTTONS 17        //PC3/A3 - encoder, right and left buttons (resp. thresholds 1VCC 0.82VCC 0.68VCC)

template<uint8_t sda_pin = 2, uint8_t scl_pin = 3, uint8_t i2c_delay = 4>
class SoftWire {  // I2C bus https://www.nxp.com/docs/en/user-guide/UM10204.pdf
  #define pin_to_pio(pin) (( (pin)<8?16:(pin)<14?-8:-6)+(pin))  // translates Arduino pin to logical bit representation 0..23 mappable on PB/PC/PD0-7/PCINT0..23, see https://images.prismic.io/circuito/8e3a980f0f964cc539b4cbbba2654bb660db6f52_arduino-uno-pinout-diagram.png?auto=compress,format
  #define set_pio(regb, pin, val) if(val){ ((volatile uint8_t*)regb)[pin_to_pio(pin)/8*3] |=   1 << pin_to_pio(pin)%8; } else { ((volatile uint8_t*)regb)[pin_to_pio(pin)/8*3] &= ~(1 << pin_to_pio(pin)%8); }
  #define _delay() for(uint8_t i = 0; i != i2c_delay; i++){ asm("nop"); } //delayMicroseconds(2);
  #define scl_lo(){ set_pio(&DDRB, scl_pin, !false); }
  #define scl_hi(){ set_pio(&DDRB, scl_pin, !true); }
  #define sda_lo(){ set_pio(&DDRB, sda_pin, !false); }     // use void (smaller program space & distortion fix read)
  #define sda_hi(){ set_pio(&DDRB, sda_pin, !true); }
  #define writeBit(value){ if(value){ sda_hi(); } else { sda_lo(); } _delay(); scl_hi(); _delay(); scl_lo(); }
public:
  void begin(){
    set_pio(&PORTB, sda_pin, false); set_pio(&PORTB, scl_pin, false); // prepare to pull-down
    sda_hi();
    scl_hi();
  }
  void beginTransmission(uint8_t addr, bool read = false){  // (assumed) IDLE-condition: SDA_HI and SCL_HI
    if((sda_pin == 18) || (scl_pin == 19)){  // same pins that are potentially used by Wire/TWI -> perform "Wire.end()" to handle potential conflict
      TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA)); // disable twi module, acks, and twi interrupt
      PORTC &= ~(1<<PC4); PORTC &= ~(1<<PC5);       // deactivate internal pullups for twi, prepare to pull-down
      endTransmission();   // to support LCD on RS pin; extra STOP condition required, as bus likely gone in start-condition
    }
    sda_lo();  // START condition is defined by SDA_HI to SDA_LO while SCL_HI
    _delay(); scl_lo();
    write((addr << 1) | read);
  }
  uint8_t write(uint8_t data){
    for(uint8_t bit = 0x80; bit; bit >>= 1) writeBit(data & bit); //writeBit(data & bit);
    writeBit(true);   // recv ACK  ACK-condition is defined by SDA_HI by master, SDA_LO by slave
    return 1;
  }
  uint8_t write(uint8_t data[], uint8_t len){ uint8_t n = len; while (n--) write(*data++); return len; }
  uint8_t endTransmission(){
    sda_lo();  // STOP-condition is defined by SDA_LO to SDA_HI while SCL_HI
    _delay(); scl_hi();
    sda_hi();  // end of stop-condition
    _delay();  // to support LCD on RS pin;
    sda_lo();  // to support LCD on RS pin; this means we already entering i2c start-condition
    return 0;
  }
};

#ifdef SSD1306Ascii_h
SoftWire<2, 3> Wire2;  // secondary I2C bus at SDA(PD2), SCL(PD3) pins

class SSD1306AsciiAtWire2 : public SSD1306Ascii {
  uint8_t m_i2cAddr;
  void writeDisplay(uint8_t b, uint8_t mode){ Wire2.beginTransmission(m_i2cAddr); Wire2.write(mode == SSD1306_MODE_CMD ? 0x00 : 0x40); Wire2.write(b); Wire2.endTransmission(); }
public:
  void begin(const DevType* dev = &Adafruit128x64, const uint8_t i2cAddr = 0x3c){ m_i2cAddr = i2cAddr; init(dev); Wire2.begin(); }
  void begin(uint8_t cols, uint8_t rows){ begin(); setFont(lcdnums12x16); /*set2X();*/ }  // init alike LiquidCrystal
};
//SSD1306Ascii lcd;       // 128x64 OLED1306 at primary   I2C bus
SSD1306AsciiAtWire2 lcd;  // 128x64 OLED1306 at secondary I2C bus
#endif //SSD1306Ascii_h

#ifdef LiquidCrystal_h
LiquidCrystal lcd(18, 4, 0, 1, 2, 3);  // LCD1602 at parallel interface
#endif //LiquidCrystal_h

#ifdef LiquidCrystal_I2C_h
LiquidCrystal_I2C lcd(0x27, 16, 2);   // LCD1602 at primary I2C bus
#endif //LiquidCrystal_I2C_h

SoftWire<18, 19>  Wire1;  // fast 800kbps I2C bus replacing primary Wire/I2C bus SDA(PC4), SCL(PC5) pins
//#include <Wire.h>       // Wire library can be used instead of SoftWire on primary bus
//#define Wire1 Wire

class SI5351 {
  uint32_t fxtal = F_XTAL;
  int16_t msa;
  #define BB0(x) ((uint8_t)(x))           // Bash byte x of int32_t
  #define BB1(x) ((uint8_t)((x)>>8))
  #define BB2(x) ((uint8_t)((x)>>16))
  #define SI5351_ADDR  0x60
public:
  void SendRegister(uint8_t reg, uint8_t* data, uint8_t n){
    Wire1.beginTransmission(SI5351_ADDR);
    Wire1.write(reg);
    while (n--) Wire1.write(*data++);
    Wire1.endTransmission();
  }
  void SendRegister(uint8_t reg, uint8_t val){ SendRegister(reg, &val, 1); }

  void begin(){    // datasheet: https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf
    Wire1.begin();
    oe(0b00000000);  // Disable all CLK outputs
    SendRegister(24, 0b10010000); // CLK3 high-impedance when disabled, CLK0-1 low-state when disabled, CLK2 high-state so that cap to gate is charged when KEY_OUT is low, providing a negative bias when CLK2 is enabled
    SendRegister(25, 0b10101010); // CLK7-4 high-impedance when disabled
    for(int addr = 16; addr != 24; addr++) SendRegister(addr, 0b10000000);  // clock power-down when disabled
    SendRegister(187, 0);        // Disable fanout (power-safe)
    SendRegister( 15, 0);        // PLL input source
    SendRegister(149, 0);        // Disable spread spectrum enable
    SendRegister(183, 0b11010010);  // Internal CL = 10 pF (default)
  }

  void oe(uint8_t val){ SendRegister(3, ~val); }  // output enable, 1=CLK0, 2=CLK1, 3=CLK2

  enum ms_t { PLLA=0, PLLB=1, MSNA=-2, MSNB=-1, MS0=0, MS1=1, MS2=2, MS3=3, MS4=4, MS5=5 };

  void set_ms(int8_t n, uint32_t div_nom, uint32_t div_denom, uint8_t pll = PLLA, uint8_t _int = 0, uint16_t phase = 0, uint8_t rdiv = 0, uint8_t drive_strength = 3 /*0=2mA,1=4mA,2=6mA,3=8mA*/, uint8_t inv = 0, uint8_t as_ms0 = 0){
    uint16_t msa; uint32_t msb, msc, msp1, msp2, msp3;
    msa = div_nom / div_denom;     // integer part: msa must be in range 15..90 for PLL, 8+1/1048575..900 for MS
    if(msa == 4) _int = 1;  // To satisfy the MSx_INT=1 requirement of AN619, section 4.1.3 which basically says that for MS divider a value of 4 and integer mode must be used
    msb = (_int) ? 0 : (((uint64_t)(div_nom % div_denom)*0x10000) / div_denom); // fractional part
    msc = (_int) ? 1 : 0x10000;
    msp1 = 128*msa + 128*msb/msc - 512; // todo: in case of _int, set divby4 bit and ensure msp1 = 0 (msa = 0)
    msp2 = 128*msb - 128*msb/msc * msc;
    msp3 = msc;
    uint8_t ms_reg2 = BB2(msp1) | (rdiv<<4) | ((msa == 4)*0x0C);
    uint8_t ms_regs[8] = { BB1(msp3), BB0(msp3), ms_reg2, BB1(msp1), BB0(msp1), BB2(((msp3 & 0x0F0000)<<4) | msp2), BB1(msp2), BB0(msp2) };
    SendRegister(n*8+42, ms_regs, 8); // Write to MSx
    if(n < 0){
      SendRegister(n+16+8, 0x80|(0x40*_int)); // MSNx PLLn: 0x40=FBA_INT; 0x80=CLKn_PDN
    } else {
      SendRegister(n+16, ((pll)*0x20)|((as_ms0 && n) ? 0x08: 0x0C)|drive_strength|((_int)*0x40)|((inv)*0x10));  // MSx CLKn: 0x0C=PLLA,0x2C=PLLB local msynth; 3=8mA; 0x40=MSx_INT; 0x80=CLKx_PDN
      SendRegister(n+165, (!_int) * phase * msa / 90);      // when using: make sure to configure MS in fractional-mode, perform reset afterwards
    }
  }

  void set_freq(int32_t fout){  // Set a CLK0,1,2 to fout Hz with phase i=90, q=0 (on PLLA)
      uint8_t rdiv = 0; // CLK pin sees fout/(2^rdiv)
      if(fout < 500000){ rdiv = 7; fout *= 128; } // Divide by 128 for fout 4..500kHz
      uint16_t d; if(fout < 30000000) d = (16 * fxtal) / fout; else d = (32 * fxtal) / fout;  // Integer part  .. maybe 44?
      if(fout > 140000000) d = 4; // for f=140..300MHz; AN619; 4.1.3, this implies integer mode
      if(d % 2) d++;  // prefer even numbers for divider (AN619 p.4 and p.6)
      uint32_t fvcoa = d * fout; // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz

      set_ms(MSNA, fvcoa, fxtal);                   // PLLA in fractional mode
      set_ms(MS0,  fvcoa, fout, PLLA, 0, 90, rdiv, 3);  // Multisynth stage with integer divider but in frac mode due to phase setting
      set_ms(MS1,  fvcoa, fout, PLLA, 0,  0, rdiv, 3);
      set_ms(MS2,  fvcoa, fout, PLLA, 0,  0, rdiv, 3);
      if(msa != (fvcoa/fout)){ msa = fvcoa/fout; SendRegister(177, 0xA0); }
      oe(0b00000011);  // output enable CLK0,1
  }
};
static SI5351 si5351;

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
public:
  void begin(){
    sdr_rx_00();  //func_ptr = sdr_rx_00;
    init_adc(); init_dac();
    set_dac_sample_rate(78125);
    set_clk_interrupt(62500);  // start timer 2 interrupt clock used as ADC sample clock and subsequent DSP and audio output chain
    set_dac_audio_enable(true);  // speaker output enable
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
  static uint8_t mode;
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

  uint8_t log2(uint16_t x){ uint8_t y = 0; for(; x>>=1;) y++; return y; }

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
  void clk_interrupt_enable(bool val = true){ if(val) TIMSK2 |= (1 << OCIE2A); else TIMSK2 &= ~(1 << OCIE2A); } // enable timer compare interrupt TIMER2_COMPA_vect

  static void set_adc_mux(uint8_t adcpin){ ADMUX = (adcpin-14)|(1<<REFS1)|(1<<REFS0); }
  static int16_t get_adc(uint8_t adcpin){ set_adc_mux(adcpin); ADCSRA |= (1 << ADSC); return ADC - 511; }  // returns unbiased ADC input
  static int16_t sample_corr(int16_t ac){ static int16_t prev_adc; return (prev_adc + ac) / 2, prev_adc = ac; } // I/Q needs to be sampled at the same time, since there is only one ADC, I/Q samples are taken in sequence; this function corrects the unwanted time-offset by averaging

  void dac_upsample(int16_t ac){
    static int16_t ozd1, ozd2;  // output stage
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
uint8_t Receiver::mode;
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
    set_interrupt_enable_pin(rota_pin); set_interrupt_enable_pin(rotb_pin); //PCMSK2 |= (1 << PCINT22) | (1 << PCINT23); PCICR |= (1 << PCIE2);
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

#ifdef LiquidCrystal_I2C_h
  lcd.init();
  lcd.backlight();
#endif //LiquidCrystal_I2C_h
  lcd.begin(16, 2);

  si5351.begin();
  recv.begin();
}

void loop(){
  static int32_t freq;
  if(freq != enc.value){  // encoder change
    freq = enc.value; lcd.setCursor(0, 0); lcd.print(freq); lcd.print(' '); lcd.print( (recv.mode == recv.USB) ? 'U' : 'L'); lcd.print(' ');
    si5351.set_freq(freq);
  }
  if(digitalRead(BUTTONS)){
    int32_t t0 = millis(); while(digitalRead(BUTTONS)); if((millis() - t0) > 500){  // long button press: change mode
      if(recv.mode++ >= 1) recv.mode = 0; freq = 0;
    } else {  // short button press: change band
      static int32_t freqs[] = { 3573000, 5357000, 7074000, 10136000, 14074000, 18100000, 21074000, 24915000, 28074000 };
      int i; for(i = 0; i != 9; i++) if(freqs[i] > (enc.value + 1700000)){ enc.value = freqs[i]; break; } if(i == 9) enc.value = freqs[0];
    }
    delayMicroseconds(1000);  // debounce
  }
}
