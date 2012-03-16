// Graphics library by ladyada/adafruit with init code from Rossum 
// Modified by sseltzer.
// MIT license
#include "TFTLCD.h"
#include "glcdfont.c"
#include <avr/pgmspace.h>
#include "pins_arduino.h"
#include "wiring_private.h"

#define DATAPORT1 PORTD
#define DATAPIN1 PIND
#define DATADDR1 DDRD

#define DATAPORT2 PORTB
#define DATAPIN2 PINB
#define DATADDR2 DDRB

#define PORTD_PINMASK 0xD0
#define PORTB_PINMASK 0x2F

// Adds delays during the init of the LCD 571ms to do an init and screen fill, 148ms if disabled.
//#define LONGINIT

uint16_t TFTLCD::width(void) {return _width;}
uint16_t TFTLCD::height(void) {return _height;}
uint8_t  TFTLCD::getRotation(void) {return rotation;}
uint16_t TFTLCD::color565(uint8_t r, uint8_t g, uint8_t b) {return (((((r >> 3) << 6) | (g >> 2)) << 5) | (b >> 3));}

TFTLCD::TFTLCD(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset) {
  _cs = cs;       // Chip Select:     A3 with the Adafruit shield.
  _rs = cd;       // Register Select: A2 with the Adafruit shield.
  _wr = wr;       // LCD Write:       A1 with the Adafruit shield.
  _rd = rd;       // LCD Read:        A0 with the Adafruit shield.
  _reset = reset; //                  A4 with the Adafruit shield.
  
  rotation = 0;
  _width = TFTWIDTH;
  _height = TFTHEIGHT;
  
  // Map the ports and pins
  csport = digitalPinToPort(_cs);
  rsport = digitalPinToPort(_rs);
  wrport = digitalPinToPort(_wr);
  rdport = digitalPinToPort(_rd);

  cspin  = digitalPinToBitMask(_cs);
  rspin  = digitalPinToBitMask(_rs);
  wrpin  = digitalPinToBitMask(_wr);
  nwrpin = ~wrpin; // These save tons and tons of execution cycles
  rdpin  = digitalPinToBitMask(_rd);
  nrdpin = ~rdpin;
  
  pinMode(_cs, OUTPUT);
  pinMode(_rs, OUTPUT);
  pinMode(_wr, OUTPUT);
  pinMode(_rd, OUTPUT);
  pinMode(_reset, OUTPUT); 
  
  /* Per the ILI9325 Datasheet:
  
    nCS - Chip Select signal HIGH:!ENABLED LOW:ENABLED
  
    A register select signal.
    Low: select an index or status register
    High: select a control register
    Fix to either IOVcc or GND level when not in use.
    
    A write strobe signal and enables an operation to write data when the signal is low.
    Fix to either IOVcc or GND level when not in use.
    SPI Mode:
    Synchronizing clock signal in SPI mode.
    
    A read strobe signal and enables an operation to read out data when the signal is low.
    Fix to either IOVcc or GND level when not in use.
    
    Initializes the ILI9325 with a low input. Be sure to execute a power-on reset after supplying power.
  */
  
  *portOutputRegister(csport) |= cspin;
  *portOutputRegister(rsport) |= rspin;
  *portOutputRegister(rdport) |= rdpin;
  *portOutputRegister(wrport) |= wrpin;
  digitalWrite(_reset, HIGH); // No need to map this
   
  cursor_y = cursor_x = 0;
  textsize = 1;
  textcolor = 0xFFFF;
  
  DATADDR2 |= PORTB_PINMASK;
  DATADDR1 |= PORTD_PINMASK;
}
void TFTLCD::initDisplay() {
  initDisplay(true, 0x0000);
}
// Init display and optionally fill the screen with a color before enabling the display.
void TFTLCD::initDisplay(boolean fill, uint16_t color) {
  reset();
  writeRegister(TFTLCD_START_OSC,         0x0001); //0000 0000 0000 0001 // start oscillator
  #ifdef LONGINIT
  delay(50);          // this will make a delay of 50 milliseconds
  #endif
  writeRegister(TFTLCD_DRIV_OUT_CTRL,     0x0100); //0000 0001 0000 0000
  writeRegister(TFTLCD_DRIV_WAV_CTRL,     0x0700); //0000 0111 0000 0000
  writeRegister(TFTLCD_ENTRY_MOD,         0x1030); //0001 0000 0011 0000
  writeRegister(TFTLCD_RESIZE_CTRL,       0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_DISP_CTRL2,        0x0202); //0000 0010 0000 0010
  writeRegister(TFTLCD_DISP_CTRL3,        0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_DISP_CTRL4,        0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_RGB_DISP_IF_CTRL1, 0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_FRM_MARKER_POS,    0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_RGB_DISP_IF_CTRL2, 0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_POW_CTRL1,         0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_POW_CTRL2,         0x0007); //0000 0000 0000 0111
  writeRegister(TFTLCD_POW_CTRL3,         0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_POW_CTRL4,         0x0000); //0000 0000 0000 0000
  #ifdef LONGINIT
  delay(200);
  #endif
  writeRegister(TFTLCD_POW_CTRL1,         0x1690); //0001 0110 1001 0000
  writeRegister(TFTLCD_POW_CTRL2,         0x0227); //0000 0010 0010 0111
  #ifdef LONGINIT
  delay(50);
  #endif
  writeRegister(TFTLCD_POW_CTRL3,         0x001A); //0000 0000 0001 1010
  #ifdef LONGINIT
  delay(50);
  #endif
  writeRegister(TFTLCD_POW_CTRL4,         0x1800); //0001 1000 0000 0000
  writeRegister(TFTLCD_POW_CTRL7,         0x002A); //0000 0000 0010 1010
  #ifdef LONGINIT
  delay(50);
  #endif
  writeRegister(TFTLCD_GAMMA_CTRL1,       0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_GAMMA_CTRL2,       0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_GAMMA_CTRL3,       0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_GAMMA_CTRL4,       0x0206); //0000 0010 0000 0110
  writeRegister(TFTLCD_GAMMA_CTRL5,       0x0808); //0000 1000 0000 1000
  writeRegister(TFTLCD_GAMMA_CTRL6,       0x0007); //0000 0000 0000 0111
  writeRegister(TFTLCD_GAMMA_CTRL7,       0x0201); //0000 0010 0000 0001
  writeRegister(TFTLCD_GAMMA_CTRL8,       0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_GAMMA_CTRL9,       0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_GAMMA_CTRL10,      0x0000); //0000 0000 0000 0000
  
  writeRegister(TFTLCD_GRAM_HOR_AD,       0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_GRAM_VER_AD,       0x0000); //0000 0000 0000 0000  
  writeRegister(TFTLCD_HOR_START_AD,      0x0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_HOR_END_AD,        0x00EF); //0000 0000 1110 1111
  writeRegister(TFTLCD_VER_START_AD,      0X0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_VER_END_AD,        0x013F); //0000 0001 0011 1111

  writeRegister(TFTLCD_GATE_SCAN_CTRL1,   0xA700); //1010 0111 0000 0000 // Driver Output Control (R60h)
  writeRegister(TFTLCD_GATE_SCAN_CTRL2,   0x0003); //0000 0000 0000 0011 // Driver Output Control (R61h)
  writeRegister(TFTLCD_GATE_SCAN_CTRL3,   0x0000); //0000 0000 0000 0000 // Driver Output Control (R62h)

  writeRegister(TFTLCD_PANEL_IF_CTRL1,    0X0010); //0000 0000 0001 0000 // Panel Interface Control 1 (R90h)
  writeRegister(TFTLCD_PANEL_IF_CTRL2,    0X0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_PANEL_IF_CTRL3,    0X0003); //0000 0000 0000 0011
  writeRegister(TFTLCD_PANEL_IF_CTRL4,    0X1100); //0001 0001 0000 0000
  writeRegister(TFTLCD_PANEL_IF_CTRL5,    0X0000); //0000 0000 0000 0000
  writeRegister(TFTLCD_PANEL_IF_CTRL6,    0X0000); //0000 0000 0000 0000
  if (fill) fillScreen(color);                     // Aesthetics option to fill the screen to a color before enabling the display.
  writeRegister(TFTLCD_DISP_CTRL1,        0x0133); //0000 0001 0011 0011 // Display Control (R07h)
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

void  TFTLCD::fillScreen(uint16_t color) {
  writeRegister(TFTLCD_GRAM_HOR_AD, 0);     // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, 0);     // GRAM Address Set (Vertical Address) (R21h)
  writeRegister(TFTLCD_RW_GRAM);         // Write Data to GRAM (R22h)
  
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
  /*
	In order to increase speed, we cut the number of iterations down by four times.
	This results in ~20 ms gain in draw time. Total draw time for my UNO R3 is ~145 ms
	for any color. Draw time for the previous fillScreen function was ~571 ms.
  
  Draw time could further be optimized if all datapins were on the same port. The gain
  would likely be ~20 ms.
  */
  uint32_t i = 19200; // 320 * 240 / 4.
  volatile uint8_t *wrportreg = portOutputRegister(wrport);
  *portOutputRegister(rsport) |= rspin;
  *portOutputRegister(csport) &= ~cspin;
  while (i--) {
    PORTD = portd_highbyte; // Send high byte over port D. (LCD_DATA_5_5V, LCD_DATA_7_5V, LCD_DATA_8_5V)
    PORTB = portb_highbyte; // Send high byte over port B. (LCD_DATA_1_5V, LCD_DATA_2_5V, LCD_DATA_3_5V, LCD_DATA_4_5V, LCD_DATA_6_5V)
    *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
    PORTD = portd_lowbyte;  // Send low byte over port D.
    PORTB = portb_lowbyte;  // Send low byte over port B.
    *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
    PORTD = portd_highbyte;
    PORTB = portb_highbyte;
    *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
    PORTD = portd_lowbyte;
    PORTB = portb_lowbyte;
    *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
    PORTD = portd_highbyte;
    PORTB = portb_highbyte;
    *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
    PORTD = portd_lowbyte;
    PORTB = portb_lowbyte;
    *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
    PORTD = portd_highbyte;
    PORTB = portb_highbyte;
    *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
    PORTD = portd_lowbyte;
    PORTB = portb_lowbyte;
    *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  }
  *portOutputRegister(csport) |= cspin;
}
/*
  Slightly improved. 370 ms for a screen fill, down to 298 ms. It would be significant 
  enough for very complex draw operations with fast lines, but not otherwise very useful.
  This has been further optimized to 222.
*/
void TFTLCD::drawFastLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color, uint8_t rotflag) {
  uint16_t entryMode;
  
  switch (rotation) {
  case 0:
    entryMode = (rotflag) ? 0x1028 : 0x1030;
    break;
  case 1:
    swap(x, y);
    x = TFTWIDTH - x - 1;
    entryMode = (rotflag) ? 0x1000 : 0x1030;
    break;
  case 2:
    x = TFTWIDTH - x - 1;
    y = TFTHEIGHT - y - 1;
    entryMode = (rotflag) ? 0x1008 : 0x1020;
    break;
  case 3:
    swap(x,y);
    y = TFTHEIGHT - y - 1;
    entryMode = (rotflag) ? 0x1030 : 0x1008;
    break;
  }
  writeRegister(TFTLCD_ENTRY_MOD, entryMode);
  writeRegister(TFTLCD_GRAM_HOR_AD, x); // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, y); // GRAM Address Set (Vertical Address) (R21h)
  writeRegister(TFTLCD_RW_GRAM);  // Write Data to GRAM (R22h)
  *portOutputRegister(csport) &= ~cspin;  // LOW
  *portOutputRegister(rsport) |= rspin; // Register Select Pin HIGH
  *portOutputRegister(rdport) |= rdpin;   // HIGH
  volatile uint8_t *wrportreg = portOutputRegister(wrport);
  DATADDR1 |= PORTD_PINMASK;
  DATADDR2 |= PORTB_PINMASK;
  uint8_t portd_highbyte = (PORTD & PORTB_PINMASK) | ((color >> 8) & PORTD_PINMASK);
  uint8_t portb_highbyte = (PORTB & PORTD_PINMASK) | ((color >> 8) & PORTB_PINMASK);
	uint8_t portd_lowbyte  = (PORTD & PORTB_PINMASK) | (color & PORTD_PINMASK);
	uint8_t portb_lowbyte  = (PORTB & PORTD_PINMASK) | (color & PORTB_PINMASK);
  while (length--) {
    PORTD = portd_highbyte;
    PORTB = portb_highbyte;
    *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
    PORTD = portd_lowbyte;
    PORTB = portb_lowbyte;
    *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  }
  *portOutputRegister(csport) |= cspin; // HIGH
  writeRegister(TFTLCD_ENTRY_MOD, 0x1030);
}

void TFTLCD::drawVerticalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color) {
  if (x >= _width) return;
  drawFastLine(x,y,length,color,1);
}

void TFTLCD::drawHorizontalLine(uint16_t x, uint16_t y, uint16_t length, uint16_t color) {
  if (y >= _height) return;
  drawFastLine(x,y,length,color,0);
}

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
    if (steep) writePixel(y0, x0, color, false);
    else       writePixel(x0, y0, color, false);
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
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

size_t TFTLCD::write(uint8_t c) {
  if (c == '\n') {
    cursor_y += textsize*8;
    cursor_x = 0;
  } else if (c == '\r') {
    // skip em
  } else {
    drawChar(cursor_x, cursor_y, c, textcolor, textsize);
    cursor_x += textsize*6;
  }
  return 1;
}

void TFTLCD::drawString(uint16_t x, uint16_t y, char *c, uint16_t color, uint8_t size) {
  while (c[0]) {
    drawChar(x, y, c[0], color, size);
    x += size * 6;
    c++;
  }
}
// draw a character
void TFTLCD::drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint8_t size) {
  for (uint8_t i =0; i<5; i++ ) {
    uint8_t line = pgm_read_byte(font + (c * 5) + i);
    for (uint8_t j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) writePixel(x + i, y + j, color, true); // default size
        else           fillRect(x + i * size, y + j * size, size, size, color); // big size
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
  //if (x >= _width) return;
  //drawFastLine(x0, y0-r, 2*r+1, color,1);
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
  writePixel(x0, y0 + r, color);
  writePixel(x0, y0 - r, color);
  writePixel(x0 + r, y0, color);
  writePixel(x0 - r, y0, color);
  drawCircleHelper(x0, y0, r, 0x0F, color);
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
    if (cornername & 0x04) {
      writePixel(x0 + x, y0 + y, color);
      writePixel(x0 + y, y0 + x, color);
    } 
    if (cornername & 0x02) {
      writePixel(x0 + x, y0 - y, color);
      writePixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x08) {
      writePixel(x0 - y, y0 + x, color);
      writePixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x01) {
      writePixel(x0 - y, y0 - x, color);
      writePixel(x0 - x, y0 - y, color);
    }
  }
}

// fill a rectangle
void TFTLCD::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fillcolor) {
  // smarter version
  while (h--) drawHorizontalLine(x, y++, w, fillcolor);
}

void TFTLCD::writePixel(uint16_t x, uint16_t y, uint16_t color) {
  writePixel(x, y, color, false);
}
void TFTLCD::writePixel(uint16_t x, uint16_t y, uint16_t color, bool rotflag) {
  if ((x < 0) || (y < 0)) return;
  if (rotflag) {
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
  }
  if ((x >= TFTWIDTH) || (y >= TFTHEIGHT)) return;
  
  *portOutputRegister(csport) &= ~cspin;  // LOW
  *portOutputRegister(rdport) |= rdpin;   // HIGH
  volatile uint8_t *wrportreg = portOutputRegister(wrport);
  DATADDR1 |= PORTD_PINMASK;
  DATADDR2 |= PORTB_PINMASK;
  uint8_t dataD = PORTD & PORTB_PINMASK;
  uint8_t dataB = PORTB & PORTD_PINMASK;
  // Write the command.
  *portOutputRegister(rsport) &= ~rspin;  // Register Select Pin LOW
  PORTD = dataD;                          // 0x0020 (Low byte is 0, simplify logic - set it equal to the preserved values of PORTD and PORTB).
  PORTB = dataB;                          // 0x0020 (Low byte is 0, simplify logic - set it equal to the preserved values of PORTD and PORTB).
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  PORTB |= TFTLCD_GRAM_HOR_AD;            // 0x0020 (PORTD does not need to change. Since PORTB == preserved values of the port, we can just | it with 0x20)
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  // Write the data.
  *portOutputRegister(rsport) |= rspin;   // Register Select Pin HIGH
  PORTD = dataD;
  PORTB = dataB;
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  PORTD = dataD | (x & PORTD_PINMASK);
  PORTB = dataB | (x & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  // Write the command.
  *portOutputRegister(rsport) &= ~rspin; // Register Select Pin LOW
  PORTD = dataD;
  PORTB = dataB;
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  PORTB |= TFTLCD_GRAM_VER_AD;
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  // Write the data.
  *portOutputRegister(rsport) |= rspin; // Register Select Pin HIGH
  PORTD = dataD | ((y >> 8) & PORTD_PINMASK);
  PORTB = dataB | ((y >> 8) & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  PORTD = dataD | (y & PORTD_PINMASK);
  PORTB = dataB | (y & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  *portOutputRegister(rsport) &= ~rspin; // Register Select Pin LOW
  PORTD = dataD;
  PORTB = dataB;
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  PORTB |= TFTLCD_RW_GRAM;
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  *portOutputRegister(rsport) |= rspin; // Register Select Pin HIGH
  PORTD = dataD | ((color >> 8) & PORTD_PINMASK);
  PORTB = dataB | ((color >> 8) & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  PORTD = dataD | (color & PORTD_PINMASK);
  PORTB = dataB | (color & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  *portOutputRegister(csport) |= cspin; // HIGH
}

uint16_t TFTLCD::readPixel(uint16_t x, uint16_t y) {
  writeRegister(TFTLCD_GRAM_HOR_AD, x);     // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister(TFTLCD_GRAM_VER_AD, y);     // GRAM Address Set (Vertical Address) (R21h)
  return readRegister(TFTLCD_RW_GRAM);
}

void TFTLCD::writeRegister(uint16_t address) {
  *portOutputRegister(csport) &= ~cspin;  // LOW
  *portOutputRegister(rsport) &= ~rspin;  // Register Select Pin LOW
  
  volatile uint8_t *wrportreg = portOutputRegister(wrport);
  PORTD = (PORTD & PORTB_PINMASK) | ((address >> 8) & PORTD_PINMASK);
  PORTB = (PORTB & PORTD_PINMASK) | ((address >> 8) & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  PORTD = (PORTD & PORTB_PINMASK) | (address & PORTD_PINMASK);
  PORTB = (PORTB & PORTD_PINMASK) | (address & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  
  *portOutputRegister(csport) |= cspin; // HIGH
}

void TFTLCD::writeRegister(uint16_t address, uint16_t data) {
  volatile uint8_t *wrportreg = portOutputRegister(wrport);
  *portOutputRegister(csport) &= ~cspin;  // LOW
  *portOutputRegister(rsport) &= ~rspin;  // Register Select Pin LOW
  
  // Write the address.
  PORTD = (PORTD & PORTB_PINMASK) | ((address >> 8) & PORTD_PINMASK);
  PORTB = (PORTB & PORTD_PINMASK) | ((address >> 8) & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  PORTD = (PORTD & PORTB_PINMASK) | (address & PORTD_PINMASK);
  PORTB = (PORTB & PORTD_PINMASK) | (address & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  
  // Write the data.
  *portOutputRegister(rsport) |= rspin; // Register Select Pin HIGH
  PORTD = (PORTD & PORTB_PINMASK) | ((data >> 8) & PORTD_PINMASK);
  PORTB = (PORTB & PORTD_PINMASK) | ((data >> 8) & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  PORTD = (PORTD & PORTB_PINMASK) | (data & PORTD_PINMASK);
  PORTB = (PORTB & PORTD_PINMASK) | (data & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  
  *portOutputRegister(csport) |= cspin; // HIGH
}

void TFTLCD::writeData(uint16_t data) {
  volatile uint8_t *wrportreg = portOutputRegister(wrport);
  *portOutputRegister(csport) &= ~cspin;  // LOW
  *portOutputRegister(rsport) |= rspin; // Register Select Pin HIGH
  *portOutputRegister(rdport) |= rdpin;   // HIGH
  DATADDR1 |= PORTD_PINMASK;
  DATADDR2 |= PORTB_PINMASK;
  PORTD = (PORTD & PORTB_PINMASK) | ((data >> 8) & PORTD_PINMASK);
  PORTB = (PORTB & PORTD_PINMASK) | ((data >> 8) & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  PORTD = (PORTD & PORTB_PINMASK) | (data & PORTD_PINMASK);
  PORTB = (PORTB & PORTD_PINMASK) | (data & PORTB_PINMASK);
  *wrportreg |= (*wrportreg &= nwrpin); // Strobe Write Pin
  *portOutputRegister(csport) |= cspin; // HIGH
}

uint16_t TFTLCD::readRegister(uint16_t address) {
  writeRegister(address);
  *portOutputRegister(csport) &= ~cspin;
  *portOutputRegister(rsport) |= rspin;
  volatile uint8_t *rdportreg = portOutputRegister(rdport);
  DATADDR1 &= ~PORTD_PINMASK; 
  DATADDR2 &= ~PORTB_PINMASK;
  
  *rdportreg &= nrdpin;
  delayMicroseconds(100); // Data wont read correctly without the delay.
  uint16_t d = (DATAPIN1 & PORTD_PINMASK) | (DATAPIN2 & PORTB_PINMASK);
  d <<= 8;
  *rdportreg |= rdpin;
  
  *rdportreg &= nrdpin;
  delayMicroseconds(100); // Data wont read correctly without the delay.
  d |= (DATAPIN1 & PORTD_PINMASK) | (DATAPIN2 & PORTB_PINMASK);
  *rdportreg |= rdpin;
  
  *portOutputRegister(csport) |= cspin;
  DATADDR1 |= PORTD_PINMASK;
  DATADDR2 |= PORTB_PINMASK;
  return d;
}
