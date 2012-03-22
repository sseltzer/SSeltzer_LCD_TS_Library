// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// (c) ladyada / adafruit
// Modified by sseltzer.
// Code under MIT License

#include "pins_arduino.h"
#include "wiring_private.h"
#include <avr/pgmspace.h>
#include "TouchScreen.h"

Point::Point(void) {
  x = y = 0;
}
Point::Point(int16_t x0, int16_t y0, int16_t z0) {
  x = x0;
  y = y0;
  z = z0;
}

bool Point::operator==(Point p1) {return ((p1.x == x) && (p1.y == y) && (p1.z == z));}
bool Point::operator!=(Point p1) {return ((p1.x != x) || (p1.y != y) || (p1.z != z));}

TouchScreen::TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym) {TouchScreen(xp, yp, xm, ym, 0);}
TouchScreen::TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym, uint16_t resistanceX) {
  _yp = yp;
  _xm = xm;
  _ym = ym;
  _xp = xp;
  _resistanceX = resistanceX;
}

Point TouchScreen::getPoint(void) {
  int x, y, z, sample1, sample2;
  
  uint8_t xp_port = digitalPinToPort(_xp);
  uint8_t yp_port = digitalPinToPort(_yp);
  uint8_t xm_port = digitalPinToPort(_xm);
  uint8_t ym_port = digitalPinToPort(_ym);

  uint8_t xp_pin = digitalPinToBitMask(_xp);
  uint8_t yp_pin = digitalPinToBitMask(_yp);
  uint8_t xm_pin = digitalPinToBitMask(_xm);
  uint8_t ym_pin = digitalPinToBitMask(_ym);

  // Read x.
  pinMode(_yp, INPUT);
  pinMode(_ym, INPUT);
  *portOutputRegister(yp_port) &= ~yp_pin;
  *portOutputRegister(ym_port) &= ~ym_pin;
  
  pinMode(_xp, OUTPUT);
  pinMode(_xm, OUTPUT);
  *portOutputRegister(xp_port) |= xp_pin;
  *portOutputRegister(xm_port) &= ~xm_pin;
  
  sample1 = analogRead(_yp);
  sample2 = analogRead(_yp);
  if (sample1 != sample2) return Point(-1, -1, -1);
  x = (1023-(sample1+sample2)/2);
  
  // Read y.
  pinMode(_xp, INPUT);
  pinMode(_xm, INPUT);
  *portOutputRegister(xp_port) &= ~xp_pin;

  pinMode(_yp, OUTPUT);
  pinMode(_ym, OUTPUT);
  *portOutputRegister(yp_port) |= yp_pin;
  
  sample1 = analogRead(_xm);
  sample2 = analogRead(_xm);
  if (sample1 != sample2) return Point(-1, -1, -1);
  y = (1023-(sample1+sample2)/2);
  
  // Read z.
  // Set X+ to ground
  pinMode(_xp, OUTPUT);
  *portOutputRegister(xp_port) &= ~xp_pin;

  // Set Y- to VCC
  *portOutputRegister(ym_port) |= ym_pin;

  // Hi-Z X- and Y+
  *portOutputRegister(yp_port) &= ~yp_pin;
  pinMode(_yp, INPUT);

  sample1 = analogRead(_xm);
  sample2 = analogRead(_yp);

  if (_resistanceX != 0) z = ((((sample2 / sample1) - 1) * x * _resistanceX) / 1024);
  else z = (1023 - (sample2 - sample1));

  pinMode(_xp, OUTPUT);
  pinMode(_xm, OUTPUT);
  pinMode(_yp, OUTPUT);
  pinMode(_ym, OUTPUT);
  return Point(x, y, z);
}