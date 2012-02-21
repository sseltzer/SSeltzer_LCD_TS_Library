// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// (c) ladyada / adafruit
// Modified by sseltzer.
// Code under MIT License

class Point {
 public:
  Point(void);
  Point(int16_t x, int16_t y, int16_t z);
  
  bool operator==(Point);
  bool operator!=(Point);

  int16_t x, y, z;
};

class TouchScreen {
 public:
  TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym);
  TouchScreen(uint8_t xp, uint8_t yp, uint8_t xm, uint8_t ym, uint16_t rx);
  Point getPoint();

private:
  uint8_t _yp, _ym, _xm, _xp;
  uint16_t _resistanceX;
};
