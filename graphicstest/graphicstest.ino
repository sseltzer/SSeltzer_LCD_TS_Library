// Modified by sseltzer.
// The control pins can connect to any pins but we'll use the 
// analog lines since that means we can double up the pins
// with the touch screen (see the TFT paint example)
#define LCD_CS A3    // Chip Select goes to Analog 3
#define LCD_RS A2    // Register Select goes to Analog 2
#define LCD_WR A1    // LCD Write goes to Analog 1
#define LCD_RD A0    // LCD Read goes to Analog 0

// you can also just connect RESET to the arduino RESET pin
#define LCD_RESET A4

/* For the 8 data pins:
Duemilanove/Diecimila/UNO/etc ('168 and '328 chips) microcontoller:
D0 connects to digital 8
D1 connects to digital 9
D2 connects to digital 2
D3 connects to digital 3
D4 connects to digital 4
D5 connects to digital 5
D6 connects to digital 6
D7 connects to digital 7

For Mega's use pins 22 thru 29 (on the double header at the end)
*/

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0 
#define WHITE           0xFFFF

#include "TFTLCD.h"

TFTLCD tft(LCD_CS, LCD_RS, LCD_WR, LCD_RD, LCD_RESET);

unsigned long startTime = 0;
unsigned long endTime = 0;

// 45506
// 20018 ms new code
// 14246 ms New new code
void setup(void) {
/*
  tft.initDisplay();
  startTime = millis();
testFillRoundRect();
  endTime = millis();
  tft.setCursor(10, 100);
  tft.setTextSize(2);
  tft.println(endTime - startTime);
/*

  //tft.initDisplay();
  /*
  startTime = micros();
  endTime = micros();
  tft.setCursor(10, 50);
  tft.setTextSize(2);
  tft.println(endTime - startTime);
  /*
  startTime = millis();
  testdrawcircles(10, WHITE);
  //testFastLine(BLUE);
  endTime = millis();
  tft.setCursor(10, 50);
  tft.setTextSize(2);
  tft.println(endTime - startTime);
  int i = 0;
  for (; i < 240; i++)  {
    if (tft.readPixel(i, 0) != 0x0000) {
    tft.println(i + " " + tft.readPixel(i, 0));
    break;
    }
  }
  */

  startTime = millis();
  tft.reset();
  tft.initDisplay();
  testtext(RED);
  delay(2000);
  testlines(CYAN);
  delay(500);
  testfastlines(RED, BLUE);
  delay(500);
  testdrawrects(GREEN);
  delay(500);
  testfillrects(YELLOW, MAGENTA);
  delay(500);
  tft.fillScreen(BLACK);
  testfillcircles(10, MAGENTA);
  testdrawcircles(10, WHITE);
  delay(500); 
  testtriangles();
  delay(500); 
  testfilltriangles();
  delay(500); 
  testRoundRect();
  delay(500); 
  testFillRoundRect();
  endTime = millis();
  tft.setCursor(10, 50);
  tft.setTextSize(2);
  tft.println(endTime - startTime);
  //*/
  /*
  uint16_t color = BLUE;
  startTime = millis();
  tft.fillScreen(color);
  endTime = millis();
  tft.setCursor(10, 10);
  tft.setTextSize(2);
  tft.println(endTime - startTime);
  tft.println(" ");
  tft.println(tft.lcdGetPixel(10, 50), HEX);
  delay(2000);
  int i = 240;
  startTime = millis();
  //while (i-- > 0) tft.drawPixel(i, 100, BLACK);
  //tft.drawLine(75, 75, 200, 200, BLACK);
  while (i-- > 0) tft.drawLine(i, 0, i, 320, BLACK);
  endTime = millis();
  tft.setCursor(10, 120);
  tft.println(endTime - startTime);
  tft.println(" ");
  */
}

void loop(void) {
  /*
  startTime = millis();
  tft.fillScreen(BLACK);
  endTime = millis();
  tft.setCursor(10, 10);
  tft.setTextSize(2);
  tft.println(endTime - startTime);
  delay(2000);
  */
}



void testFillRoundRect() {
  tft.fillScreen(BLACK);
  
  for (uint16_t x=tft.width(); x > 20 ; x-=6) {
    tft.fillRoundRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, x/8,  tft.color565(0, x, 0));
 }
}

void testRoundRect() {
  tft.fillScreen(BLACK);
  
  for (uint16_t x=0; x < tft.width(); x+=6) {
    tft.drawRoundRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, x/8, tft.color565(x, 0, 0));
 }
}

void testtriangles() {
  tft.fillScreen(BLACK);
  for (uint16_t i=0; i<tft.width()/2; i+=5) {
    tft.drawTriangle(tft.width()/2, tft.height()/2-i,
                     tft.width()/2-i, tft.height()/2+i,
                     tft.width()/2+i, tft.height()/2+i, tft.color565(0, 0, i));
  }
}

void testfilltriangles() {
  tft.fillScreen(BLACK);
  
  for (uint16_t i=tft.width()/2; i>10; i-=5) {
    tft.fillTriangle(tft.width()/2, tft.height()/2-i,
                     tft.width()/2-i, tft.height()/2+i,
                     tft.width()/2+i, tft.height()/2+i, 
                     tft.color565(0, i, i));
    tft.drawTriangle(tft.width()/2, tft.height()/2-i,
                     tft.width()/2-i, tft.height()/2+i,
                     tft.width()/2+i, tft.height()/2+i, tft.color565(i, i, 0));    
  }
}
void testtext(uint16_t color) {
  tft.fillScreen(BLACK);
  tft.setCursor(0, 20);
  tft.setTextColor(color);
  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
}

void testfillcircles(uint8_t radius, uint16_t color) {
  for (uint16_t x=radius; x < tft.width(); x+=radius*2) {
    for (uint16_t y=radius; y < tft.height(); y+=radius*2) {
      tft.fillCircle(x, y, radius, color);
    }
  }  
}

void testdrawcircles(uint8_t radius, uint16_t color) {
  for (uint16_t x=0; x < tft.width()+radius; x+=radius*2) {
    for (uint16_t y=0; y < tft.height()+radius; y+=radius*2) {
      tft.drawCircle(x, y, radius, color);
    }
  }  
}


void testfillrects(uint16_t color1, uint16_t color2) {
 tft.fillScreen(BLACK);
 for (uint16_t x=tft.width()-1; x > 6; x-=6) {
   //Serial.println(x, DEC);
   tft.fillRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color1);
   tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color2);
 }
}

void testdrawrects(uint16_t color) {
 tft.fillScreen(BLACK);
 for (uint16_t x=0; x < tft.width(); x+=6) {
   tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color);
 }
}

void testfastlines(uint16_t color1, uint16_t color2) {
   tft.fillScreen(BLACK);
   for (uint16_t y=0; y < tft.height(); y+=5) {
     tft.drawHorizontalLine(0, y, tft.width(), color1);
   }
   for (uint16_t x=0; x < tft.width(); x+=5) {
     tft.drawVerticalLine(x, 0, tft.height(), color2);
   }
  
}

void testlines(uint16_t color) {
   tft.fillScreen(BLACK);
   for (uint16_t x=0; x < tft.width(); x+=6) {
     tft.drawLine(0, 0, x, tft.height()-1, color);
   }
   for (uint16_t y=0; y < tft.height(); y+=6) {
     tft.drawLine(0, 0, tft.width()-1, y, color);
   }
   
   tft.fillScreen(BLACK);
   for (uint16_t x=0; x < tft.width(); x+=6) {
     tft.drawLine(tft.width()-1, 0, x, tft.height()-1, color);
   }
   for (uint16_t y=0; y < tft.height(); y+=6) {
     tft.drawLine(tft.width()-1, 0, 0, y, color);
   }
   
   tft.fillScreen(BLACK);
   for (uint16_t x=0; x < tft.width(); x+=6) {
     tft.drawLine(0, tft.height()-1, x, 0, color);
   }
   for (uint16_t y=0; y < tft.height(); y+=6) {
     tft.drawLine(0, tft.height()-1, tft.width()-1, y, color);
   }

   tft.fillScreen(BLACK);
   for (uint16_t x=0; x < tft.width(); x+=6) {
     tft.drawLine(tft.width()-1, tft.height()-1, x, 0, color);
   }
   for (uint16_t y=0; y < tft.height(); y+=6) {
     tft.drawLine(tft.width()-1, tft.height()-1, 0, y, color);
   }
}


void testBars() {
  uint16_t i,j;
  for(i=0; i < tft.height(); i++)
  {
    for(j=0; j < tft.width(); j++)
    {
      if(i>279) tft.writeData(WHITE);
      else if(i>239) tft.writeData(BLUE);
      else if(i>199) tft.writeData(GREEN);
      else if(i>159) tft.writeData(CYAN);
      else if(i>119) tft.writeData(RED);
      else if(i>79) tft.writeData(MAGENTA);
      else if(i>39) tft.writeData(YELLOW);
      else tft.writeData(BLACK);
    }
  }
}
