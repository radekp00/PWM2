#include <SPI.h>
#include <Wire.h>
#include "Ucglib.h"

#define TIMEBASE_VALUE ((uint8_t) ceil(F_CPU*0.000001))
#define OVERSAMPLING_BITS 5 /* 5 bits extra */
#define OVERSAMPLING_MAX_VALUE ((uint32_t) ((1 << 12) - 1) << OVERSAMPLING_BITS) /* 12 + 5 bits = 17 bits */
#define ADC_SAMPNUM_CONFIG (OVERSAMPLING_BITS << 1) /* The SAMPNUM bit field setting match this formula */
#define ADC_SAMPLES (1 << ADC_SAMPNUM_CONFIG) /* 5 bits = 1024 samples */
#define ROVERSAMPLING_MAX_VALUE ((uint32_t) (((1 << 12) /2)- 1) << OVERSAMPLING_BITS) /* 12 + 5 bits = 17 bits */

static volatile uint32_t adc_reading;
static volatile int32_t radc_reading;
static volatile float voltage;
static volatile float rvoltage;
static volatile float resistance;

Ucglib_ST7735_18x128x160_HWSPI ucg(/*cd=*/ 3, /*cs=*/ 13, /*reset=*/ -1);

long int result;
uint8_t volpercent;
int intr = 0;
int started = 0;
int startedd = 0;
char charge = 0;
char interrup = 0;
char noAtomizer = 0;
char shortAtomizer = 0;
uint8_t power = 30;
uint8_t mode = 0;
uint8_t keyCounter = 0;
uint8_t keyKey = 0;
uint8_t keyMode = 0;
uint8_t fireing = 0;

typedef struct RgbColor
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
} RgbColor;

typedef struct HsvColor
{
  unsigned char h;
  unsigned char s;
  unsigned char v;
} HsvColor;

RgbColor HsvToRgb(HsvColor hsv)
{
  RgbColor rgb;
  unsigned char region, remainder, p, q, t;

  if (hsv.s == 0)
  {
    rgb.r = hsv.v;
    rgb.g = hsv.v;
    rgb.b = hsv.v;
    return rgb;
  }

  region = hsv.h / 43;
  remainder = (hsv.h - (region * 43)) * 6;

  p = (hsv.v * (255 - hsv.s)) >> 8;
  q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
  t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

  switch (region)
  {
  case 0:
    rgb.r = hsv.v; rgb.g = t; rgb.b = p;
    break;
  case 1:
    rgb.r = q; rgb.g = hsv.v; rgb.b = p;
    break;
  case 2:
    rgb.r = p; rgb.g = hsv.v; rgb.b = t;
    break;
  case 3:
    rgb.r = p; rgb.g = q; rgb.b = hsv.v;
    break;
  case 4:
    rgb.r = t; rgb.g = p; rgb.b = hsv.v;
    break;
  default:
    rgb.r = hsv.v; rgb.g = p; rgb.b = q;
    break;
  }

  return rgb;
}
RgbColor rgb;
HsvColor hsv;
void menuMain() {
  hsv.h = (volpercent + 235) % 255;
  hsv.s = 255;
  hsv.v = 255;
  rgb = HsvToRgb(hsv);
  ucg.setColor(255, 255, 255);
  ucg.setPrintPos(26, 20);
  ucg.print(voltage, 2);
  ucg.print("V");
  ucg.setPrintPos(26, 40);
  ucg.print(volpercent);
  ucg.print("%");
  ucg.drawBox(82, 3, 8, 2);
  ucg.drawFrame(76, 5, 20, 37);
  ucg.setColor(rgb.b, rgb.g, rgb.r);
  ucg.drawBox(78, 40 - volpercent / 3, 16, volpercent / 3);
  ucg.setFont(ucg_font_freedoomr25_tn);
  ucg.setPrintPos(40, 75);
  ucg.setColor(0, 0, 255);
  ucg.print(power);
  ucg.setFont(ucg_font_helvR12_tr);
  //ucg.setPrintPos(26, 75);
  ucg.print(" W");
  ucg.setColor(255, 255, 255);
  if (noAtomizer) {
    ucg.setColor(0, 255, 255);
  }
  if (shortAtomizer) {
    ucg.setColor(0, 0, 255);
  }
  ucg.setPrintPos(26, 95);
  ucg.print(resistance, 3);
  ucg.print("Ohm");


}
void menuCharge() {
  ucg.setColor(0, 0, 0);
  ucg.drawBox(26, 0, 80, 20);
  volpercent = 2;
  hsv.h = (volpercent + 235) % 255;
  hsv.s = 255;
  hsv.v = 255;
  rgb = HsvToRgb(hsv);
  ucg.setColor(255, 255, 255);
  ucg.drawBox(51, 38, 30, 4);
  ucg.drawFrame(36, 42, 60, 108);
  ucg.setColor(rgb.b, rgb.g, rgb.r);
  ucg.drawBox(40, 146 - volpercent, 52, volpercent);
  ucg.setColor(255, 255, 255);
  ucg.setPrintPos(26, 15);
  ucg.print(voltage);
  ucg.print("V");


}
void enterStandby() {
//out of memory
}
void powerOn() {
//out of memory
}

void powerOff() {
//out of memory
}

void measureBatteryVoltage() {
  ADC0.COMMAND &= ~ADC_DIFF_bm;
  ADC0.CTRLA = ADC_ENABLE_bm;
  ADC0.CTRLB = ADC_PRESC_DIV20_gc;
  ADC0.CTRLC = ADC_REFSEL_1024MV_gc | (TIMEBASE_VALUE << ADC_TIMEBASE_gp);
  ADC0.CTRLE = 128;
  ADC0.CTRLF = ADC_SAMPNUM_CONFIG | ADC_LEFTADJ_bm;
  ADC0.MUXPOS = ADC_MUXPOS_VDDDIV10_gc | ADC_VIA_PGA_gc;
  ADC0.COMMAND = ADC_MODE_BURST_gc;
  ADC0.PGACTRL = ADC_GAIN_2X_gc | 0b00010000 | ADC_ADCPGASAMPDUR_20CLK_gc |
                 ADC_PGAEN_bm;
  ADC0.COMMAND |= ADC_START_IMMEDIATE_gc;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  adc_reading = ADC0.RESULT >> OVERSAMPLING_BITS;
  voltage = (float)((adc_reading * 5.12) / OVERSAMPLING_MAX_VALUE);
  voltage = voltage + voltage * 0.028;
  volpercent = ((voltage - 3.0) * 84);
  ADC0.COMMAND = ADC_START_STOP_gc;
}

void measureAtomizerVoltage() {
  delay(100);
  ADC0.CTRLA = ADC_ENABLE_bm;

  ADC0.CTRLB = ADC_PRESC_DIV20_gc;
  ADC0.CTRLC = ADC_REFSEL_1024MV_gc | (TIMEBASE_VALUE << ADC_TIMEBASE_gp);
  ADC0.CTRLE = 128;
  ADC0.CTRLF = ADC_SAMPNUM_CONFIG;
  ADC0.MUXPOS = ADC_MUXPOS_AIN6_gc | ADC_VIA_PGA_gc;
  ADC0.MUXNEG = ADC_MUXNEG_AIN5_gc | ADC_VIA_PGA_gc;
  ADC0.COMMAND = ADC_MODE_BURST_gc;
  ADC0.PGACTRL = ADC_GAIN_8X_gc | 0b00010000 | ADC_ADCPGASAMPDUR_20CLK_gc |
                 ADC_PGAEN_bm;
  delay(100);
  ADC0.COMMAND |= ADC_START_IMMEDIATE_gc | ADC_DIFF_bm;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
  radc_reading = ADC0.RESULT;
  rvoltage = (float)(((radc_reading * 2.048) / ROVERSAMPLING_MAX_VALUE) / ADC_SAMPLES);
  //voltage = voltage + voltage*0.025;
  ADC0.COMMAND = ADC_START_STOP_gc;
  resistance = (float)((rvoltage * 47.0) / (voltage - rvoltage));
  noAtomizer = 0;
  shortAtomizer = 0;
  if (resistance < 0.1) {
    shortAtomizer = 1;
  }
  if (rvoltage > 0.063) {
    noAtomizer = 1;
    resistance = 0.0;
  }
}
void setup(void)
{
  SPI.swap(1);
  pinMode(7, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, INPUT);
  //pinMode(1, INPUT);
  //pinMode(2, INPUT);
  pinMode(6, OUTPUT);
  pinMode(0, OUTPUT);
  digitalWrite(6, LOW);
  digitalWrite(0, LOW);
  digitalWrite(7, HIGH);
  delay(2000);
  digitalWrite(7, LOW);
  delay(1);
  digitalWrite(16, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(7, HIGH);
  PORTB.PIN4CTRL = PORT_ISC_FALLING_gc | PORT_PULLUPEN_bm;
  delay(3000);
  ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.setFont(ucg_font_helvR12_tr);
  //ucg.begin(UCG_FONT_MODE_SOLID);
  ucg.clearScreen();


  PORTMUX.TCAROUTEA = 0b1;
//pinMode(6, OUTPUT);
//digitalWrite(6, HIGH);
  TCA0.SPLIT.CTRLB = TCA_SPLIT_LCMP0EN_bm;
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;
  TCA0.SPLIT.LPER = 0xFE;
  TCA0.SPLIT.LCMP0 = 0x05;
  TCA0.SPLIT.CTRLA = TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV2_gc;
  TCA0.SPLIT.CTRLA &= ~TCA_SPLIT_ENABLE_bm;

  Wire.begin();
  Wire.beginTransmission(0x28);
  Wire.write(0x21);
  Wire.write(0b11101100);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x27);
  Wire.write(0b11101100);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x29);
  Wire.write(0b11101100);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x00);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x32);
  Wire.write(0b00001000);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x33);
  Wire.write(0b00001000);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x35);
  Wire.write(0b00001000);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x36);
  Wire.write(0b00001000);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x37);
  Wire.write(0b00001000);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x1F);
  Wire.write(0b00111111);
  Wire.endTransmission();
  Wire.beginTransmission(0x28);
  Wire.write(0x28);
  Wire.write(0b000000);
  Wire.endTransmission();

}

void loop(void)
{
  sei();
  while (1) {

    if (interrup) {
      interrup = 0;
      Wire.beginTransmission(0x28);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(0x28, 8, 1);

      while (Wire.available() < 8); 
      int c = Wire.read(); 
      if ((c & 0b00000001) == 1) {
        //keyKey = c;
        keyCounter++;
        Wire.beginTransmission(0x28);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        startedd = 1;
        if (fireing == 1) {
          mode = 2;
        } else {
          mode = 1;
        }

      } else {
        charge = 1;
      }
    }


    switch (mode) {
    case 1:
      keyCounter = 0;
      measureBatteryVoltage();
      digitalWrite(0, HIGH);
      measureAtomizerVoltage();
      digitalWrite(0, LOW);
      if (!(shortAtomizer | noAtomizer)) {
        float pMax = ((voltage * voltage) / resistance);
        uint8_t ppp = (256 / (pMax / power));
        TCA0.SPLIT.LCMP0 = ppp;
        TCA0.SPLIT.CTRLA |= TCA_SPLIT_ENABLE_bm;
        fireing = 1;

      } else {
        mode = 2;
      }
      mode = 0;

      break;
    case 2:
      fireing = 0;
      TCA0.SPLIT.LCMP0 = 0x00;
      //TCA0.SPLIT.CTRLA &= ~TCA_SPLIT_ENABLE_bm;

      ucg.clearScreen();
      menuMain();
      mode = 0;
      break;
    default:
      break;
    }
    if (charge) {
      ucg.clearScreen();
      menuCharge();
      while (charge) {
        measureBatteryVoltage();
        menuCharge();
        delay(1000);
        Wire.beginTransmission(0x28);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        delay(10);
        if ((PORTB.IN & (PIN4_bm))) {
          charge = 0;
        }
      }
    }
    delay(1);
  }

  ucg.clearScreen();
  //menuCharge();
  //delay(2000);
  //ucg.clearScreen();
  menuMain();
  started = 1;
  sei();
  if (true) {
    if (false) {
      measureBatteryVoltage();
      ucg.clearScreen();
      menuMain();
      delay(2000);
    }
    if (startedd) {
      if (started) {
        intr++;
        ucg.setColor(0, 0, 0);
        ucg.drawBox(0, 115, 128, 128);
        ucg.setPrintPos(26, 135);
        ucg.setColor(255, 255, 255);
        Wire.beginTransmission(0x28);
        Wire.write(0x03);
        Wire.endTransmission();
        Wire.requestFrom(0x28, 8, 1);

        while (Wire.available() < 8); 
        int c = Wire.read(); 

        ucg.print(intr);
        ucg.print(digitalRead(5), BIN);
        ucg.setPrintPos(26, 155);
        ucg.print(c, BIN);
        Wire.beginTransmission(0x28);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        Wire.beginTransmission(0x28);
        Wire.write(0x03);
        Wire.write(0b00000000);
        Wire.endTransmission();
        startedd = 0;
      }
    }
    delay(10);
    if (charge) {
      ucg.clearScreen();
      menuCharge();
      while (charge) {
        measureBatteryVoltage();
        menuCharge();
        delay(1000);
        Wire.beginTransmission(0x28);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        delay(10);
        if ((PORTB.IN & (PIN4_bm))) {
          charge = 0;
        }
      }
    }

  }


  //ucg.setScale2x2();
  /// ucg.setPrintPos(13, 30);
  // ucg.print(voltage);
  // ucg.setPrintPos(45, 30);
  // ucg.print("O");
  // ucg.undoScale();


  //ucg.setColor(0, 0, 255, 255); ucg.setColor(1, 255, 255, 0); ucg.drawGradientLine(50, 40, 45, 0);
  delay(200);

}
ISR(PORTB_PORT_vect) {
  PORTB.INTFLAGS = 0xFF;
  interrup = 1;
}
