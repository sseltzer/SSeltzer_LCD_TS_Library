#include "TFTLCD.h"

// Graphics library by ladyada/adafruit with init code from Rossum 
// Modified by sseltzer.
// MIT license

#define DATAPORT1 PORTD
#define DATAPIN1 PIND
#define DATADDR1 DDRD

#define DATAPORT2 PORTB
#define DATAPIN2 PINB
#define DATADDR2 DDRB

#define PORTD_PINMASK 0xD0
#define PORTB_PINMASK 0x2F

#include "glcdfont.c"
#include <avr/pgmspace.h>
#include "pins_arduino.h"
#include "wiring_private.h"

static const uint16_t _regValues[] PROGMEM = {
  TFTLCD_START_OSC, 0x0001,     // start oscillator

  TFTLCD_DELAYCMD, 50,          // this will make a delay of 50 milliseconds

  TFTLCD_DRIV_OUT_CTRL, 0x0100, 
  TFTLCD_DRIV_WAV_CTRL, 0x0700,
  TFTLCD_ENTRY_MOD, 0x1030,
  TFTLCD_RESIZE_CTRL, 0x0000,
  TFTLCD_DISP_CTRL2, 0x0202,
  TFTLCD_DISP_CTRL3, 0x0000,
  TFTLCD_DISP_CTRL4, 0x0000,
  TFTLCD_RGB_DISP_IF_CTRL1, 0x0,
  TFTLCD_FRM_MARKER_POS, 0x0,
  TFTLCD_RGB_DISP_IF_CTRL2, 0x0,
  
  TFTLCD_POW_CTRL1, 0x0000,
  TFTLCD_POW_CTRL2, 0x0007,
  TFTLCD_POW_CTRL3, 0x0000,
  TFTLCD_POW_CTRL4, 0x0000,

  TFTLCD_DELAYCMD, 200,  
  
  TFTLCD_POW_CTRL1, 0x1690,
  TFTLCD_POW_CTRL2, 0x0227,

  TFTLCD_DELAYCMD, 50,  

  TFTLCD_POW_CTRL3, 0x001A,

  TFTLCD_DELAYCMD, 50,  

  TFTLCD_POW_CTRL4, 0x1800,
  TFTLCD_POW_CTRL7, 0x002A,

  TFTLCD_DELAYCMD,50,
  
  TFTLCD_GAMMA_CTRL1, 0x0000,    
  TFTLCD_GAMMA_CTRL2, 0x0000, 
  TFTLCD_GAMMA_CTRL3, 0x0000,
  TFTLCD_GAMMA_CTRL4, 0x0206,   
  TFTLCD_GAMMA_CTRL5, 0x0808,  
  TFTLCD_GAMMA_CTRL6, 0x0007,  
  TFTLCD_GAMMA_CTRL7, 0x0201,
  TFTLCD_GAMMA_CTRL8, 0x0000,  
  TFTLCD_GAMMA_CTRL9, 0x0000,  
  TFTLCD_GAMMA_CTRL10, 0x0000,  
 
  TFTLCD_GRAM_HOR_AD, 0x0000,  
  TFTLCD_GRAM_VER_AD, 0x0000,  
  TFTLCD_HOR_START_AD, 0x0000,
  TFTLCD_HOR_END_AD, 0x00EF,
  TFTLCD_VER_START_AD, 0X0000,
  TFTLCD_VER_END_AD, 0x013F,
   

  TFTLCD_GATE_SCAN_CTRL1, 0xA700,     // Driver Output Control (R60h)
  TFTLCD_GATE_SCAN_CTRL2, 0x0003,     // Driver Output Control (R61h)
  TFTLCD_GATE_SCAN_CTRL3, 0x0000,     // Driver Output Control (R62h)

  TFTLCD_PANEL_IF_CTRL1, 0X0010,     // Panel Interface Control 1 (R90h)
  TFTLCD_PANEL_IF_CTRL2, 0X0000,
  TFTLCD_PANEL_IF_CTRL3, 0X0003,
  TFTLCD_PANEL_IF_CTRL4, 0X1100,
  TFTLCD_PANEL_IF_CTRL5, 0X0000,
  TFTLCD_PANEL_IF_CTRL6, 0X0000,

  // Display On
  TFTLCD_DISP_CTRL1, 0x0133,     // Display Control (R07h)
};

uint16_t TFTLCD::width(void) {return _width;}
uint16_t TFTLCD::height(void) {return _height;}
uint8_t TFTLCD::getRotation(void) {return rotation;}

TFTLCD::TFTLCD(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset) {
  _cs = cs;       // Chip Select:     A3 with the Adafruit shield.
  _rs = cd;       // Register Select: A2 with the Adafruit shield.
  _wr = wr;       // LCD Write:       A1 with the Adafruit shield.
  _rd = rd;       // LCD Read:        A0 with the Adafruit shield.
  _reset = reset; //                  A4 with the Adafruit shield.
  
  rotation = 0;
  _width = TFTWIDTH;
  _height = TFTHEIGHT;

  // nCS - Chip Select signal HIGH:!ENABLED LOW:ENABLED
  digitalWrite(_cs, HIGH);
  pinMode(_cs, OUTPUT);  
  /* Per the ILI9325 Datasheet:
    A register select signal.
    Low: select an index or status register
    High: select a control register
    Fix to either IOVcc or GND level when not in use.
  */
  digitalWrite(_rs, HIGH);
  pinMode(_rs, OUTPUT);  
  
  /* Per the ILI9325 Datasheet:
    A write strobe signal and enables an operation to write data when the signal is low.
    Fix to either IOVcc or GND level when not in use.
    SPI Mode:
    Synchronizing clock signal in SPI mode.
  */
  digitalWrite(_wr, HIGH);
  pinMode(_wr, OUTPUT);  
  /* Per the ILI9325 Datasheet:
  A read strobe signal and enables an operation to read out data when the signal is low.
  Fix to either IOVcc or GND level when not in use.
  */
  digitalWrite(_rd, HIGH);
  pinMode(_rd, OUTPUT);  

  // Per the ILI9325 Datasheet: Initializes the ILI9325 with a low input. 
  // Be sure to execute a power-on reset after supplying power.
  digitalWrite(_reset, HIGH); 
  pinMode(_reset, OUTPUT); 

  csport = digitalPinToPort(_cs);
  rsport = digitalPinToPort(_rs);
  wrport = digitalPinToPort(_wr);
  rdport = digitalPinToPort(_rd);

  cspin = digitalPinToBitMask(_cs);
  rspin = digitalPinToBitMask(_rs);
  wrpin = digitalPinToBitMask(_wr);
  rdpin = digitalPinToBitMask(_rd);

  cursor_y = cursor_x = 0;
  textsize = 1;
  textcolor = 0xFFFF;
}

void TFTLCD::initDisplay(void) {
  uint16_t a, d;
  reset();
  for (uint8_t i = 0; i < sizeof(_regValues) / 4; i++) {
    a = pgm_read_word(_regValues + i*2);
    d = pgm_read_word(_regValues + i*2 + 1);
    if (a == 0xFF) delay(d);
    else writeRegister(a, d);
  }
}

void TFTLCD::reset(void) {
  if (_reset) digitalWrite(_reset, LOW);
  delay(2); 
  if (_reset) digitalWrite(_reset, HIGH);

  // resync
  writeData(0);
  writeData(0);
  writeData(0);  
  writeData(0);
}

void TFTLCD::setRotation(uint8_t x) {
  writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
  rotation = x;
  _width = TFTWIDTH; 
  _height = TFTHEIGHT;
  if (x % 2 == 1) {
    _width = TFTHEIGHT; 
    _height = TFTWIDTH;
  }
}

void TFTLCD::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
  if ((x < 0) || (y < 0)) return;
  // check rotation, move pixel around if necessary
  switch (rotation) {
  case 1:
    swap(x, y);
    x = TFTWIDTH - x - 1;
    break;
  case 2:
    x = TFTWIDTH - x - 1;
    y = TFTHEIGHT - y - 1;
    break;
  case 3:
    swap(x, y);
    y = TFTHEIGHT - y - 1;
    break;
  }
  if ((x >= TFTWIDTH) || (y >= TFTHEIGHT)) return;
  writeRegister(TFTLCD_GRAM_HOR_AD, x);
  writeRegister(TFTLCD_GRAM_VER_AD, y);
  writeCommand(TFTLCD_RW_GRAM);
  writeData(color);
}

void  TFTLCD::fillScreen(uint16_t color) {
  writeRegister(TFTLCD_GRAM_HOR_AD, 0);     // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, 0);     // GRAM Address Set (Vertical Address) (R21h)
  writeCommand(TFTLCD_RW_GRAM);         // Write Data to GRAM (R22h)
  *portOutputRegister(csport) &= ~cspin;
  *portOutputRegister(rsport) |= rspin;
  *portOutputRegister(rdport) |= rdpin;
  *portOutputRegister(wrport) |= wrpin;
  DDRD |= PORTD_PINMASK;
  DDRB |= PORTB_PINMASK;
  /* 
  This masking and two port data transmit is due to the fact that the 8 data pins are on 
	D4, D6, D7, D8, D9, D10, D11, D13. Not sure why all of the data pins are not on PORTD, 
	so all transmission could be completed using a single port instead of using PORTB and 
	PORTD via masking. See the schematic.
	
	Configure all data ahead of time so no bitwise operations are done in the draw loop. This
	decreases draw time by quite a bit.
  */
  // PORTB_PINMASK = 0010 1111     PORTD_PINMASK = 1101 0000
  uint8_t portd_highbyte = (PORTD & PORTB_PINMASK) | ((color >> 8) & PORTD_PINMASK);
  uint8_t portb_highbyte = (PORTB & PORTD_PINMASK) | ((color >> 8) & PORTB_PINMASK);
	uint8_t portd_lowbyte  = (PORTD & PORTB_PINMASK) | (color & PORTD_PINMASK);
	uint8_t portb_lowbyte  = (PORTB & PORTD_PINMASK) | (color & PORTB_PINMASK);
  uint8_t nwrpin = ~wrpin;
  /*
	In order to increase speed, we cut the number of iterations down by four times.
	This results in ~20 ms gain in draw time. Total draw time for my UNO R3 is ~145 ms
	for any color. Draw time for the previous fillScreen function was ~571 ms.
  
  Draw time could further be optimized if all datapins were on the same port. The gain
  would likely be ~20 ms.
  */
  uint32_t i = 19200; // 320 * 240 / 4.
  volatile uint8_t *wrportreg = portOutputRegister(wrport);
  while (i--) {
    PORTD = portd_highbyte; // Send high byte over port D. (LCD_DATA_5_5V, LCD_DATA_7_5V, LCD_DATA_8_5V)
    PORTB = portb_highbyte; // Send high byte over port B. (LCD_DATA_1_5V, LCD_DATA_2_5V, LCD_DATA_3_5V, LCD_DATA_4_5V, LCD_DATA_6_5V)
    *wrportreg &= nwrpin;   // Strobe to transmit the data.
    *wrportreg |=  wrpin;   // Strobe to transmit the data.
    PORTD = portd_lowbyte;  // Send high byte over port D.
    PORTB = portb_lowbyte;  // Send high byte over port B.
    *wrportreg &= nwrpin;   // Strobe to transmit the data.
    *wrportreg |=  wrpin;   // Strobe to transmit the data.
    // Repeat cycle 4 times.
    PORTD = portd_highbyte;
    PORTB = portb_highbyte;
    *wrportreg &= nwrpin;
    *wrportreg |=  wrpin;
    PORTD = portd_lowbyte;
    PORTB = portb_lowbyte;
    *wrportreg &= nwrpin;
    *wrportreg |=  wrpin;
    PORTD = portd_highbyte;
    PORTB = portb_highbyte;
    *wrportreg &= nwrpin;
    *wrportreg |=  wrpin;
    PORTD = portd_lowbyte;
    PORTB = portb_lowbyte;
    *wrportreg &= nwrpin;
    *wrportreg |=  wrpin;
    PORTD = portd_highbyte;
    PORTB = portb_highbyte;
    *wrportreg &= nwrpin;
    *wrportreg |=  wrpin;
    PORTD = portd_lowbyte;
    PORTB = portb_lowbyte;
    *wrportreg &= nwrpin;
    *wrportreg |=  wrpin;
  }
  *portOutputRegister(csport) |= cspin;
}
/*
  Slightly improved. 370 ms for a screen fill, down to 298 ms. It would be significant 
  enough for very complex draw operations with fast lines, but not otherwise very useful.
*/
void TFTLCD::drawFastLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color, uint8_t rotflag) {
  uint16_t newentrymod;

  switch (rotation) {
  case 0:
    if (rotflag)
      newentrymod = 0x1028;   // we want a 'vertical line'
    else 
      newentrymod = 0x1030;   // we want a 'horizontal line'
    break;
  case 1:
    swap(x, y);
    // first up fix the X
    x = TFTWIDTH - x - 1;
    if (rotflag)
      newentrymod = 0x1000;   // we want a 'vertical line'
    else 
      newentrymod = 0x1028;   // we want a 'horizontal line'
    break;
  case 2:
    x = TFTWIDTH - x - 1;
    y = TFTHEIGHT - y - 1;
    if (rotflag)
      newentrymod = 0x1008;   // we want a 'vertical line'
    else 
      newentrymod = 0x1020;   // we want a 'horizontal line'
    break;
  case 3:
    swap(x,y);
    y = TFTHEIGHT - y - 1;
    if (rotflag)
      newentrymod = 0x1030;   // we want a 'vertical line'
    else 
      newentrymod = 0x1008;   // we want a 'horizontal line'
    break;
  }
  
  writeRegister(TFTLCD_ENTRY_MOD, newentrymod);

  writeRegister(TFTLCD_GRAM_HOR_AD, x); // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, y); // GRAM Address Set (Vertical Address) (R21h)
  writeCommand(TFTLCD_RW_GRAM);  // Write Data to GRAM (R22h)
  volatile uint8_t *wrportreg = portOutputRegister(wrport);
  *portOutputRegister(csport) &= ~cspin;  // LOW
  *portOutputRegister(rsport) |= rspin;   // HIGH
  *portOutputRegister(rdport) |= rdpin;   // HIGH
  *wrportreg |=  wrpin; // HIGH
  DATADDR1 |= PORTD_PINMASK;
  DATADDR2 |= PORTB_PINMASK;
  while (length--) {
    PORTD = (PORTD & PORTB_PINMASK) | ((color >> 8) & PORTD_PINMASK);
    PORTB = (PORTB & PORTD_PINMASK) | ((color >> 8) & PORTB_PINMASK);
    *wrportreg &= ~wrpin; // LOW
    *wrportreg |=  wrpin; // HIGH
    PORTD = (PORTD & PORTB_PINMASK) | (color & PORTD_PINMASK);
    PORTB = (PORTB & PORTD_PINMASK) | (color & PORTB_PINMASK);
    *wrportreg &= ~wrpin; // LOW
    *wrportreg |=  wrpin; // HIGH
  }
  *portOutputRegister(csport) |= cspin; // HIGH
  writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
}
/*
// 7739 the 'simplified form' is less efficient
void TFTLCD::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  int16_t dx = abs(x1 - x0);
  int16_t dy = abs(y1 - y0);
  int8_t sx = (x0 < x1) ? 1 : -1;
  int8_t sy = (y0 < y1) ? 1 : -1;
  int16_t err = dx - dy;
  int16_t e2;
  while(true) {
    drawPixel(x0, y0, color);
    if (x0 == x1 && y0 == y1) break;
    e2 = 2 * err;
    if (e2 > -1 * dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}
*/
//7690
// bresenham's algorithm - thx wikpedia
void TFTLCD::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  // if you're in rotation 1 or 3, we need to swap the X and Y's
  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }
  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx = x1 - x0;
  int16_t dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep = (y0 < y1) ? 1 : -1;

  for (; x0<=x1; x0++) {
    if (steep) drawPixel(y0, x0, color);
    else       drawPixel(x0, y0, color);
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

uint16_t TFTLCD::lcdGetPixel(uint16_t x, uint16_t y) {
  writeRegister(TFTLCD_GRAM_HOR_AD, x);     // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, y);     // GRAM Address Set (Vertical Address) (R21h)
  writeCommand(TFTLCD_RW_GRAM);         // Write Data to GRAM (R22h)
  return readData();
}



void TFTLCD::goTo(int x, int y) {
  writeRegister(TFTLCD_GRAM_HOR_AD, x);     // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, y);     // GRAM Address Set (Vertical Address) (R21h)
  writeCommand(TFTLCD_RW_GRAM);         // Write Data to GRAM (R22h)
}

void TFTLCD::setCursor(uint16_t x, uint16_t y) {
  cursor_x = x;
  cursor_y = y;
}

void TFTLCD::setTextSize(uint8_t s) {
  textsize = s;
}

void TFTLCD::setTextColor(uint16_t c) {
  textcolor = c;
}

#if ARDUINO >= 100
size_t TFTLCD::write(uint8_t c) {
#else
void TFTLCD::write(uint8_t c) {
#endif
  if (c == '\n') {
    cursor_y += textsize*8;
    cursor_x = 0;
  } else if (c == '\r') {
    // skip em
  } else {
    drawChar(cursor_x, cursor_y, c, textcolor, textsize);
    cursor_x += textsize*6;
  }
#if ARDUINO >= 100
  return 1;
#endif
}

void TFTLCD::drawString(uint16_t x, uint16_t y, char *c, uint16_t color, uint8_t size) {
  while (c[0] != 0) {
    drawChar(x, y, c[0], color, size);
    x += size*6;
    c++;
  }
}
// draw a character
void TFTLCD::drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint8_t size) {
  for (uint8_t i =0; i<5; i++ ) {
    uint8_t line = pgm_read_byte(font+(c*5)+i);
    for (uint8_t j = 0; j<8; j++) {
      if (line & 0x1) {
	if (size == 1) // default size
	  drawPixel(x+i, y+j, color);
	else {  // big size
	  fillRect(x+i*size, y+j*size, size, size, color);
	} 
      }
      line >>= 1;
    }
  }
}

// draw a triangle!
void TFTLCD::drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color) {
  drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color); 
}

void TFTLCD::fillTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color) {
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  int32_t dx1, dx2, dx3; // Interpolation deltas
  int32_t sx1, sx2, sy; // Scanline co-ordinates

  sx2=(int32_t)x0 * (int32_t)1000; // Use fixed point math for x axis values
  sx1 = sx2;
  sy=y0;

  // Calculate interpolation deltas
  if (y1-y0 > 0) dx1=((x1-x0)*1000)/(y1-y0);
    else dx1=0;
  if (y2-y0 > 0) dx2=((x2-x0)*1000)/(y2-y0);
    else dx2=0;
  if (y2-y1 > 0) dx3=((x2-x1)*1000)/(y2-y1);
    else dx3=0;

  // Render scanlines (horizontal lines are the fastest rendering method)
  if (dx1 > dx2)
  {
    for(; sy<=y1; sy++, sx1+=dx2, sx2+=dx1)
    {
      drawHorizontalLine(sx1/1000, sy, (sx2-sx1)/1000, color);
    }
    sx2 = x1*1000;
    sy = y1;
    for(; sy<=y2; sy++, sx1+=dx2, sx2+=dx3)
    {
      drawHorizontalLine(sx1/1000, sy, (sx2-sx1)/1000, color);
    }
  }
  else
  {
    for(; sy<=y1; sy++, sx1+=dx1, sx2+=dx2)
    {
      drawHorizontalLine(sx1/1000, sy, (sx2-sx1)/1000, color);
    }
    sx1 = x1*1000;
    sy = y1;
    for(; sy<=y2; sy++, sx1+=dx3, sx2+=dx2)
    {
      drawHorizontalLine(sx1/1000, sy, (sx2-sx1)/1000, color);
    }
  }
}

uint16_t TFTLCD::Color565(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t c;
  c = r >> 3;
  c <<= 6;
  c |= g >> 2;
  c <<= 5;
  c |= b >> 3;

  return c;
}

// draw a rectangle
void TFTLCD::drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  // smarter version
  drawHorizontalLine(x, y, w, color);
  drawHorizontalLine(x, y+h-1, w, color);
  drawVerticalLine(x, y, h, color);
  drawVerticalLine(x+w-1, y, h, color);
}

// draw a rounded rectangle
void TFTLCD::drawRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color) {
  // smarter version
  drawHorizontalLine(x+r, y, w-2*r, color);
  drawHorizontalLine(x+r, y+h-1, w-2*r, color);
  drawVerticalLine(x, y+r, h-2*r, color);
  drawVerticalLine(x+w-1, y+r, h-2*r, color);
  // draw four corners
  drawCircleHelper(x+r, y+r, r, 1, color);
  drawCircleHelper(x+w-r-1, y+r, r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r, y+h-r-1, r, 8, color);
}


// fill a rounded rectangle
void TFTLCD::fillRoundRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t color) {
  // smarter version
  fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(x+r, y+r, r, 2, h-2*r-1, color);
}

// fill a circle
void TFTLCD::fillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
  writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
  drawVerticalLine(x0, y0-r, 2*r+1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}


// used to do circles and roundrects!
void TFTLCD::fillCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, uint16_t delta, uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    if (cornername & 0x1) {
      drawVerticalLine(x0+x, y0-y, 2*y+1+delta, color);
      drawVerticalLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      drawVerticalLine(x0-x, y0-y, 2*y+1+delta, color);
      drawVerticalLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}


// draw a circle outline

void TFTLCD::drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
  drawPixel(x0, y0+r, color);
  drawPixel(x0, y0-r, color);
  drawPixel(x0+r, y0, color);
  drawPixel(x0-r, y0, color);

  drawCircleHelper(x0, y0, r, 0xF, color);
}

void TFTLCD::drawCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t cornername, uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;


  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    if (cornername & 0x4) {
      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 + y, y0 + x, color);
    } 
    if (cornername & 0x2) {
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      drawPixel(x0 - y, y0 - x, color);
      drawPixel(x0 - x, y0 - y, color);
    }
  }
}

// fill a rectangle
void TFTLCD::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fillcolor) {
  // smarter version
  while (h--) drawHorizontalLine(x, y++, w, fillcolor);
}


void TFTLCD::drawVerticalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color) {
  if (x >= _width) return;
  drawFastLine(x,y,length,color,1);
}

void TFTLCD::drawHorizontalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color) {
  if (y >= _height) return;
  drawFastLine(x,y,length,color,0);
}

inline void TFTLCD::setWriteDir(void) {
  DATADDR2 |= PORTB_PINMASK;
  DATADDR1 |= PORTD_PINMASK;
}

inline void TFTLCD::setReadDir(void) {
  DATADDR2 &= ~PORTB_PINMASK;
  DATADDR1 &= ~PORTD_PINMASK;
}

inline void TFTLCD::write8(uint8_t d) {
  DATAPORT2 = (DATAPORT2 & PORTD_PINMASK) | (d & PORTB_PINMASK);
  DATAPORT1 = (DATAPORT1 & PORTB_PINMASK) | (d & PORTD_PINMASK); // top 6 bits
}

inline uint8_t TFTLCD::read8(void) {
 uint8_t d;
 d = DATAPIN1 & PORTD_PINMASK; 
 d |= DATAPIN2 & PORTB_PINMASK; 
 return d;
}
void TFTLCD::writeData(uint16_t data) {
  volatile uint8_t *wrportreg = portOutputRegister(wrport);
  *portOutputRegister(csport) &= ~cspin;  // LOW
  *portOutputRegister(rsport) |= rspin;   // HIGH
  *portOutputRegister(rdport) |= rdpin;   // HIGH
  *wrportreg |=  wrpin; // HIGH
  DATADDR1 |= PORTD_PINMASK;
  DATADDR2 |= PORTB_PINMASK;
  PORTD = (PORTD & PORTB_PINMASK) | ((data >> 8) & PORTD_PINMASK);
  PORTB = (PORTB & PORTD_PINMASK) | ((data >> 8) & PORTB_PINMASK);
  *wrportreg &= ~wrpin; // LOW
  *wrportreg |=  wrpin; // HIGH
  PORTD = (PORTD & PORTB_PINMASK) | (data & PORTD_PINMASK);
  PORTB = (PORTB & PORTD_PINMASK) | (data & PORTB_PINMASK);
  *wrportreg &= ~wrpin; // LOW
  *wrportreg |=  wrpin; // HIGH
  *portOutputRegister(csport) |= cspin; // HIGH
}

// this is a 'sped up' version, with no direction setting, or pin initialization
// not for external usage, but it does speed up stuff like a screen fill
inline void TFTLCD::writeData_unsafe(uint16_t data) {
  volatile uint8_t *wrportreg = portOutputRegister(wrport);

  write8(data >> 8);

  *wrportreg &= ~wrpin;
  //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);

  write8(data);

  *wrportreg &= ~wrpin;
  //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);
}

// the C/D pin is low during write
void TFTLCD::writeCommand(uint16_t cmd) {
  volatile uint8_t *wrportreg = portOutputRegister(wrport);

  *portOutputRegister(csport) &= ~cspin;
  //digitalWrite(_cs, LOW);
  *portOutputRegister(rsport) &= ~rspin;
  //digitalWrite(_rs, LOW);
  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);

  setWriteDir();
  write8(cmd >> 8);

  *wrportreg &= ~wrpin;
  //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);

  write8(cmd);

  *wrportreg &= ~wrpin;
  //digitalWrite(_wr, LOW);
  *wrportreg |=  wrpin;
  //digitalWrite(_wr, HIGH);

  *portOutputRegister(csport) |= cspin;
}

uint16_t TFTLCD::readData() {
 uint16_t d = 0;
 
  *portOutputRegister(csport) &= ~cspin;
  //digitalWrite(_cs, LOW);
  *portOutputRegister(rsport) |= rspin;
  //digitalWrite(_rs, HIGH);
  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  *portOutputRegister(wrport) |= wrpin;
  //digitalWrite(_wr, HIGH);
  
  setReadDir();

  *portOutputRegister(rdport) &= ~rdpin;
  //digitalWrite(_rd, LOW);

  delayMicroseconds(10);
  d = read8();
  d <<= 8;

  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  *portOutputRegister(rdport) &= ~rdpin;
  //digitalWrite(_rd, LOW);

  delayMicroseconds(10);
  d |= read8();

  *portOutputRegister(rdport) |= rdpin;
  //digitalWrite(_rd, HIGH);
  
  *portOutputRegister(csport) |= cspin;
  //digitalWrite(_cs, HIGH);
   
  return d;
}


/************************************* medium level data reading/writing */

uint16_t TFTLCD::readRegister(uint16_t addr) {
   writeCommand(addr);
   return readData();
}

void TFTLCD::writeRegister(uint16_t addr, uint16_t data) {
   writeCommand(addr);
   writeData(data);
}




