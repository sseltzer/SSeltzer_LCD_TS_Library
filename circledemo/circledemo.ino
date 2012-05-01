#include "TFTLCD.h"
#include "TouchScreen.h"

// These are the pins for the shield!
#define YP A1  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 7   // can be a digital pin
#define XP 6   // can be a digital pin

#define TS_MINX 135
#define TS_MINY 115
#define TS_MAXX 900
#define TS_MAXY 940
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0 

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 0);
TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, 0);

int x = 0;
int y = 0;
int xLast = 0;
int yLast = 0;
int xMod = 1;
int yMod = 1;
int radius = 8;
int red = tft.color565(0xFF, 0x00, 0x00);
void setup(void) {
  tft.initDisplay();
  tft.println("Hello World!");
}
void loop() {
  delay(10);
  tft.drawCircle(x, y, radius, 0x0000);
  if      (x == 0)   xMod = 1;
  else if (x == 240) xMod = -1;
  if      (y == 0)   yMod = 1;
  else if (y == 320) yMod = -1;
  y += yMod;
  x += xMod;
  tft.fillCircle(x, y, radius, red);
}