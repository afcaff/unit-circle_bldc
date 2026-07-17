/*
Adrian June 2026
FOC motor controller project + round unit circle dial
-- simpleFOC miniv1.0
-- stm32 nucleo f446re
-- 2804 hollow shaft gimbal motor
-- AS5600 encoder (I2C)
-- DIYables round GC9A01 240x240 TFT (SPI)
*/

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

// pins for display
#define TFT_SCK   PB13   // SPI2 clock  display SCL 
#define TFT_MOSI  PB15   // SPI2 data   display SDA .  look at pin outs to  find pb13 and pb15
#define TFT_MISO  PB14   // unused but set so SPI2 is selected
#define TFT_CS    PB12
#define TFT_DC    PA8
#define TFT_RST   PA9

Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_RST);

// colors
#define BLACK  0x0000
#define WHITE  0xFFFF
#define GRAY   0x7BEF
#define RED    0xF800
#define CYAN   0x07FF
#define YELLOW 0xFFE0

// dial geometry
#define CX 120           // center x (screen is 240x240)
#define CY 120           // center y
#define R  118           // outer circle radius
#define HUB 5            // center hub radius
#define L  82            // needle length  
#define W  6             // needle half-width at the hub

// dial behaviors
#define ANGLE_DIR    (+1.0f)  
#define ANGLE_OFFSET (0.0f)   // radians 
#define ANGLE_EPS    (0.01f)  //  skip redraw for changes smaller than this
#define DISPLAY_MS   40       // ~25 fps. 


// motor encoder, bldc motor 'type', pwm pins on nucleo
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(13, 12, 11, 10);

void serialReceiveUserCommand();

float lastAngle = 0;
bool  firstDraw = true;
unsigned long lastDraw = 0;

// the unit circle angles. len = how far the tick reaches inward from R.
struct Tick { float deg; uint8_t len; };
const Tick TICKS[] = {
  {0,13},{30,6},{45,10},{60,6},{90,13},{120,6},{135,10},{150,6},
  {180,13},{210,6},{225,10},{240,6},{270,13},{300,6},{315,10},{330,6}
};


// print text centered on (cx,cy), size 1
void printCentered(int cx, int cy, const char* t, uint16_t color)
{
  int w = (int)strlen(t) * 6;   // 6 px per char at text size 1
  tft.setTextSize(1);
  tft.setTextColor(color);
  tft.setCursor(cx - w / 2, cy - 4);
  tft.print(t);
}

// the two axes through the center 
void drawAxes()
{
  tft.drawLine(CX - R, CY, CX + R, CY, GRAY);   // horizontal 
  tft.drawLine(CX, CY - R, CX, CY + R, GRAY);   // vertical  
}

// draw or erase the needle at angle a
void drawNeedle(float a, uint16_t color)
{
  float c = cosf(a), s = sinf(a);
  int tx = CX + (int)lroundf(L * c);
  int ty = CY - (int)lroundf(L * s);
  int b1x = CX + (int)lroundf(W * s), b1y = CY + (int)lroundf(W * c);
  int b2x = CX - (int)lroundf(W * s), b2y = CY - (int)lroundf(W * c);
  tft.fillTriangle(b1x, b1y, b2x, b2y, tx, ty, color);
}

// the digital readout badge in the middle
void drawReadout(float a)
{
  float piUnits = a / _PI;                 // 0.00 .. 2.00
  int bx = CX - 35, by = CY + 42, bw = 70, bh = 22;
  tft.fillRoundRect(bx, by, bw, bh, 4, BLACK);
  tft.drawRoundRect(bx, by, bw, bh, 4, GRAY);
  tft.setTextSize(1);
  tft.setTextColor(YELLOW);
  tft.setCursor(bx + 10, by + 7);
  tft.print(piUnits, 2);
  tft.print(" PI");
}

void drawStaticScene()
{
  tft.fillScreen(BLACK);

  // outer circle, 2px so it reads on the round bezel
  tft.drawCircle(CX, CY, R, WHITE);
  tft.drawCircle(CX, CY, R - 1, WHITE);

  drawAxes();

  // unit circle hash marks
  for (uint8_t i = 0; i < 16; i++) {
    float rad = TICKS[i].deg * DEG_TO_RAD;
    uint16_t col = (TICKS[i].len >= 10) ? WHITE : GRAY;   // dim the minor ticks
    int x0 = CX + (int)lroundf(cosf(rad) * R);
    int y0 = CY - (int)lroundf(sinf(rad) * R);
    int x1 = CX + (int)lroundf(cosf(rad) * (R - TICKS[i].len));
    int y1 = CY - (int)lroundf(sinf(rad) * (R - TICKS[i].len));
    tft.drawLine(x0, y0, x1, y1, col);
  }

  printCentered(CX + 94, CY,      "0",     CYAN);   // east
  printCentered(CX,      CY - 94, "PI/2",  CYAN);   // north
  printCentered(CX - 94, CY,      "PI",    CYAN);   // west
  printCentered(CX,      CY + 94, "3PI/2", CYAN);   // south
}

// dynamic update: erase old needle, restore axes, draw new needle 
void updateDial()
{
  float raw = motor.shaft_angle;                 // actual position, radians (accumulates)
  float a = ANGLE_DIR * raw + ANGLE_OFFSET;
  a = fmodf(a, _2PI);                            // wrap onto the circle: [0, 2pi)
  if (a < 0) a += _2PI;

  if (!firstDraw && fabsf(a - lastAngle) < ANGLE_EPS) return;  // threshold for change

  if (!firstDraw) drawNeedle(lastAngle, BLACK);  
  drawAxes();                                     
  drawNeedle(a, RED);                            
  tft.fillCircle(CX, CY, HUB, WHITE);            
  drawReadout(a);                               // center readout

  lastAngle = a;
  firstDraw = false;
}

void setup()
{
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // point the SPI object at the SPI2 pins BEFORE begin(); the STM32 core
  // picks the SPI peripheral from the pins, so PB13/14/15 => SPI2
  SPI.setSCLK(TFT_SCK);
  SPI.setMOSI(TFT_MOSI);
  SPI.setMISO(TFT_MISO);
  tft.begin();
  tft.setRotation(2);
  drawStaticScene();

  // motor init
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12.0;
  driver.init();
  driver.enable();
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::angle;

  motor.PID_velocity.P = 0.12;
  motor.PID_velocity.I = 2.5;
  motor.velocity_limit = 8;

  motor.voltage_limit = 6;
  motor.LPF_velocity.Tf = 0.02;
  motor.P_angle.P = 20;

  // motor.sensor_direction = Direction::CCW;
  motor.init();
  motor.initFOC();

  if (motor.motor_status == FOCMotorStatus::motor_ready) {
    Serial.println(F("initFOC success i ready"));
  } else {
    Serial.println(F("initFOC failed"));
  }

  motor.target = 0;
  Serial.println(F("send integer 'x' --> motor turns x(pi) radians"));
}

void loop()
{
  motor.loopFOC();
  motor.move();
  serialReceiveUserCommand();

  // display update is time sliced so it never disrupts the FOC loop
  if (millis() - lastDraw >= DISPLAY_MS) {
    lastDraw = millis();
    updateDial();
  }
}

void serialReceiveUserCommand()
{
  static String received_char;

  while (Serial.available())
  {
    char in_char = (char)Serial.read();
    received_char += in_char;

    if (in_char == '\n')
    {
      int slash = received_char.indexOf('/');

      if (slash >= 0)
      {
        float num = received_char.substring(0, slash).toFloat();
        float den = received_char.substring(slash + 1).toFloat();
        if (den != 0) motor.target = (num / den) * _PI;
        Serial.print(F("x = "));
        Serial.print(received_char);
        Serial.print(F("  target = "));
        Serial.println(motor.target);
      }
      else {
        float n = received_char.toFloat();
        motor.target = n * _PI;
        Serial.print(F("x = "));
        Serial.print(received_char);
        Serial.print(F("  target = "));
        Serial.println(motor.target);
      }
      received_char = "";
    }
  }
}