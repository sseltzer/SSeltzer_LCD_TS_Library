// Modified by sseltzer.
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

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 0);

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0 

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0 
#define WHITE           0xFFFF


TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, 0);

#define BOXSIZE 40
#define PENRADIUS 4
int oldcolor, currentcolor;

void setup(void) {
  //Serial.begin(9600); // Serial slows things down. Taking it out.
  //Serial.println("Paint!");
  
  tft.reset();
  
  //uint16_t identifier = tft.readRegister(0x0);
  //if (identifier == 0x9325) Serial.println("Found ILI9325");

  tft.initDisplay(); 
  //tft.fillScreen(BLACK);
  
  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
  tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, YELLOW);
  tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, GREEN);
  tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, WHITE);
  tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, BLUE);
  tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, MAGENTA);
 // tft.fillRect(BOXSIZE*6, 0, BOXSIZE, BOXSIZE, WHITE);
 
 tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
 currentcolor = RED;
 
  pinMode(13, OUTPUT);
}

#define MINPRESSURE 10
#define MAXPRESSURE 1000

void loop()
{
  digitalWrite(13, HIGH);
  Point p = ts.getPoint();
  digitalWrite(13, LOW);
  digitalWrite(XM, HIGH);
  digitalWrite(YP, HIGH);
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
	
    if (p.y < (TS_MINY-5)) tft.fillRect(0, BOXSIZE, tft.width(), tft.height()-BOXSIZE, BLACK);
    
    // turn from 0->1023 to tft.width
    p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
    p.y = map(p.y, TS_MINY, TS_MAXY, tft.height(), 0);
    
    if (p.y < BOXSIZE) {
       oldcolor = currentcolor;
       if (p.x < BOXSIZE) { 
         currentcolor = RED; 
         tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
       } 
       else if (p.x < BOXSIZE*2) {
         currentcolor = YELLOW; 
         tft.drawRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, WHITE);
       }
       else if (p.x < BOXSIZE*3) {
         currentcolor = GREEN; 
         tft.drawRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, WHITE);
       }
       else if (p.x < BOXSIZE*4) {
         currentcolor = WHITE; 
         tft.drawRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, WHITE);
       }
       else if (p.x < BOXSIZE*5) {
         currentcolor = BLUE; 
         tft.drawRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, WHITE);
       }
       else if (p.x < BOXSIZE*6) {
         currentcolor = MAGENTA; 
         tft.drawRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, WHITE);
       }
       if (oldcolor != currentcolor) {
          if (oldcolor == RED) tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
          if (oldcolor == YELLOW) tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, YELLOW);
          if (oldcolor == GREEN) tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, GREEN);
          if (oldcolor == WHITE) tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, WHITE);
          if (oldcolor == BLUE) tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, BLUE);
          if (oldcolor == MAGENTA) tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, MAGENTA);
       }
    }
    if (((p.y-PENRADIUS) > BOXSIZE) && ((p.y+PENRADIUS) < tft.height())) {
      tft.fillCircle(p.x, p.y, PENRADIUS, currentcolor);
      //tft.fillRect(p.x, p.y, PENRADIUS, PENRADIUS, currentcolor);
    }
  }
}
