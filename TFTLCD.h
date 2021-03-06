/*
  sseltzer graphics library based on the Adafruit/Rossum library.
  For the original library, please visit: https://github.com/adafruit/TFTLCD-Library
  For this library, please visit: https://github.com/sseltzer/SSeltzer_LCD_TS_Library
  MIT license
*/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define TFTLCD_START_OSC			    0x00
#define TFTLCD_DRIV_OUT_CTRL		  0x01
#define TFTLCD_DRIV_WAV_CTRL		  0x02
#define TFTLCD_ENTRY_MOD			    0x03
#define TFTLCD_RESIZE_CTRL			  0x04
#define TFTLCD_DISP_CTRL1			    0x07
#define TFTLCD_DISP_CTRL2			    0x08
#define TFTLCD_DISP_CTRL3			    0x09
#define TFTLCD_DISP_CTRL4			    0x0A
#define TFTLCD_RGB_DISP_IF_CTRL1	0x0C
#define TFTLCD_FRM_MARKER_POS		  0x0D
#define TFTLCD_RGB_DISP_IF_CTRL2	0x0F
#define TFTLCD_POW_CTRL1			    0x10
#define TFTLCD_POW_CTRL2			    0x11
#define TFTLCD_POW_CTRL3			    0x12
#define TFTLCD_POW_CTRL4			    0x13
#define TFTLCD_GRAM_HOR_AD			  0x20
#define TFTLCD_GRAM_VER_AD			  0x21
#define TFTLCD_RW_GRAM				    0x22
#define TFTLCD_POW_CTRL7			    0x29
#define TFTLCD_FRM_RATE_COL_CTRL	0x2B
#define TFTLCD_GAMMA_CTRL1			  0x30
#define TFTLCD_GAMMA_CTRL2			  0x31
#define TFTLCD_GAMMA_CTRL3			  0x32
#define TFTLCD_GAMMA_CTRL4			  0x35 
#define TFTLCD_GAMMA_CTRL5			  0x36
#define TFTLCD_GAMMA_CTRL6			  0x37
#define TFTLCD_GAMMA_CTRL7			  0x38
#define TFTLCD_GAMMA_CTRL8			  0x39
#define TFTLCD_GAMMA_CTRL9			  0x3C
#define TFTLCD_GAMMA_CTRL10			  0x3D
#define TFTLCD_HOR_START_AD			  0x50
#define TFTLCD_HOR_END_AD			    0x51
#define TFTLCD_VER_START_AD			  0x52
#define TFTLCD_VER_END_AD			    0x53
#define TFTLCD_GATE_SCAN_CTRL1		0x60
#define TFTLCD_GATE_SCAN_CTRL2		0x61
#define TFTLCD_GATE_SCAN_CTRL3		0x6A
#define TFTLCD_PART_IMG1_DISP_POS	0x80
#define TFTLCD_PART_IMG1_START_AD	0x81
#define TFTLCD_PART_IMG1_END_AD		0x82
#define TFTLCD_PART_IMG2_DISP_POS	0x83
#define TFTLCD_PART_IMG2_START_AD	0x84
#define TFTLCD_PART_IMG2_END_AD		0x85
#define TFTLCD_PANEL_IF_CTRL1		  0x90
#define TFTLCD_PANEL_IF_CTRL2		  0x92
#define TFTLCD_PANEL_IF_CTRL3		  0x93
#define TFTLCD_PANEL_IF_CTRL4		  0x95
#define TFTLCD_PANEL_IF_CTRL5		  0x97
#define TFTLCD_PANEL_IF_CTRL6		  0x98

#define swap(a, b){int16_t t = a; a = b; b = t;}

class TFTLCD : public Print {
 public:
  static const uint16_t TFTWIDTH = 240;
  static const uint16_t TFTHEIGHT = 320;
 
  TFTLCD(uint8_t cs, uint8_t rs, uint8_t wr, uint8_t rd, uint8_t reset);
  
  void initDisplay();
  void initDisplay(boolean fill, uint16_t color);
  void reset(void);
  
  void writeRegister(uint16_t address);
  void writeRegister(uint16_t address, uint16_t data);
  void writeData(uint16_t data);
  uint16_t readRegister(uint16_t address);
  
  void writePixel(uint16_t x, uint16_t y, uint16_t color);
  void writePixel(uint16_t x, uint16_t y, uint16_t color, bool rotflag);
  uint16_t readPixel(uint16_t x, uint16_t y);
  
  void fillScreen(uint16_t color);
  
  void drawFastLine(uint16_t x0, uint16_t y0, uint16_t l, uint16_t color, uint8_t flag);
  void drawVerticalLine(uint16_t x0, uint16_t y0, uint16_t length, uint16_t color);
  void drawHorizontalLine(uint16_t x0, uint16_t y0, uint16_t length, uint16_t color);
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
  
  void setRotation(uint8_t x);
  uint8_t getRotation();
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
  uint16_t width();
  uint16_t height();
  
  void drawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
  void fillTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color);
  void drawRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t color);
  void fillRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t color);
  void drawRoundRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t radius, uint16_t color);
  void fillRoundRect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t radius, uint16_t color);
  void drawCircle(uint16_t x0, uint16_t y0, uint16_t r,	uint16_t color);
  void fillCircle(uint16_t x0, uint16_t y0, uint16_t r,	uint16_t color);

  void setCursor(uint16_t x, uint16_t y);
  void setTextColor(uint16_t c);
  void setTextSize(uint8_t s);
  
  virtual size_t write(uint8_t);
  void drawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint8_t s = 1);
  void drawString(uint16_t x, uint16_t y, char *c, uint16_t color, uint8_t s = 1);

 private:
  void drawCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t corner, uint16_t color);
  void fillCircleHelper(uint16_t x0, uint16_t y0, uint16_t r, uint8_t corner, uint16_t delta, uint16_t color);

  uint8_t _cs, _rs, _reset, _wr, _rd;

  uint8_t csport, rsport, wrport, rdport;
  uint8_t cspin, ncspin, rspin, nrspin, wrpin, nwrpin, rdpin, nrdpin;

  uint16_t _width, _height;
  uint8_t textsize;
  uint16_t cursor_x, cursor_y;
  uint16_t textcolor;
  uint8_t rotation;
};
