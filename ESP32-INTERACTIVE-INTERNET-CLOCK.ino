/*******************************************************************

This clock not only displays the time but also interacts with our motion and look like they’re affectted by gravity.
This matrix clock displays some of LEDs as little grains of sand which are driven by a MPU6050 (Accelerometer + Gyro)
and NODEMCU-32S.
This program is referenced to the following sources:
https://learn.adafruit.com/matrix-led-sand
https://github.com/witnessmenow/Falling-Sand-Matri...
https://github.com/adafruit/Adafruit_PixelDust
https://github.com/porrey/ledmatrixide

 *******************************************************************/
//--------------------------------------------------------------------------
#include <Wire.h>
#include "logo.h"
#include "MPU6050.h"

// https://github.com/witnessmenow/Arduino-MPU6050

#define double_buffer // this must be enabled to stop flickering
#include <PxMatrix.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

// https://github.com/2dom/PxMatrix
// https://github.com/adafruit/Adafruit-GFX-Library

#include "imgBufferGFX.h"

#define N_GRAINS     512 // Number of grains of sand
#define WIDTH        64 // Display width in pixels
#define HEIGHT       64 // Display height in pixels
#define MAX_FPS      45 // Maximum redraw rate, frames/second

// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH  * 256 - 1) // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * 256 - 1) // Maximum Y coordinate
struct Grain {
  int16_t  x,  y; // Position
  int16_t vx, vy; // Velocity
  uint16_t pos;
  uint16_t colour;
} grain[N_GRAINS];

// Wifi
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
const char* ssid     = "FTP-XXXXXX";      // SSID of local network
const char* password = "12345678";        // Password on network
#define NTP_OFFSET    25200               // In seconds - Time offset
#define NTP_INTERVAL  60 * 1000           // In miliseconds
#define NTP_ADDRESS   "1.asia.pool.ntp.org"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

// ----------------------------
// Wiring and Display setup
// For ESP32 (NODEMCU-32S)
// ----------------------------
#define P_LAT   22
#define P_A     19
#define P_B     23
#define P_C     18
#define P_D     5
#define P_E     15
#define P_OE    2

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// This defines the 'on' time of the display is us. The larger this number,
// the brighter the display. If too large the ESP will crash
uint8_t display_draw_time = 10; //10-50 is usually fine

PxMATRIX display(64, 64, P_LAT, P_OE, P_A, P_B, P_C, P_D, P_E);

#define NUM_COLOURS 5

uint16_t myRED      = display.color565(255, 0, 0);
uint16_t myGREEN    = display.color565(0, 255, 0);
uint16_t myBLUE     = display.color565(0, 0, 255);
uint16_t myMAGENTA  = display.color565(255, 0, 255);
uint16_t myYELLOW   = display.color565(255, 255, 0);
uint16_t myCYAN     = display.color565(0, 255, 255);
uint16_t myBLACK    = display.color565(0, 0, 0);
uint16_t myWHITE    = display.color565(255, 255, 255);

uint16_t myCOLORS[6]={myRED,myGREEN,myCYAN,myMAGENTA,myYELLOW,myBLUE};

MPU6050 mpu;
uint32_t        prevTime   = 0;      // Used for frames-per-second throttle
uint16_t        backbuffer = 0,      // Index for double-buffered animation
                img[WIDTH * HEIGHT]; // Internal 'map' of pixels

ImgBufferGFX imgWrapper(img, WIDTH, HEIGHT);

float xOffset = -1350; 
float yOffset = -2590;

void pixelTask(void *param) {

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_4G, MPU6050_ADDRESS, 27, 26)) // SDA - GPIO27, SCL - GPIO26
  {
    //Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

 Vector accelVector = mpu.readRawAccel();

  float xOffset = (accelVector.XAxis * -1) * -1;
  float yOffset = (accelVector.YAxis * -1) * -1;
  
  while (true) {
  uint32_t t;
  while (((t = micros()) - prevTime) < (1000000L / MAX_FPS));
  prevTime = t;

  Vector accelVector = mpu.readRawAccel();

  float accelX = (accelVector.XAxis * -1) + xOffset;
  float accelY = (accelVector.YAxis * -1) + yOffset;
  float accelZ = accelVector.ZAxis;

  int16_t ax = -accelX / 256,       // Transform accelerometer axes
          ay =  -accelY / 256,      // to grain coordinate space
          az = abs(accelZ) / 2048;  // Random motion factor
  az = (az >= 3) ? 1 : 4 - az;      // Clip & invert
  ax -= az;                         // Subtract motion factor from X, Y
  ay -= az;
  int16_t az2 = az * 2 + 1;         // Range of random motion to add back in

  // ...and apply 2D accel vector to grain velocities...
  int32_t v2; // Velocity squared
  float   v;  // Absolute velocity
  for (int i = 0; i < N_GRAINS; i++) {
    grain[i].vx += ax + random(az2); // A little randomness makes
    grain[i].vy += ay + random(az2); // tall stacks topple better!
    // Terminal velocity (in any direction) is 256 units -- equal to
    // 1 pixel -- which keeps moving grains from passing through each other
    // and other such mayhem.  Though it takes some extra math, velocity is
    // clipped as a 2D vector (not separately-limited X & Y) so that
    // diagonal movement isn't faster
    v2 = (int32_t)grain[i].vx * grain[i].vx + (int32_t)grain[i].vy * grain[i].vy;
    if (v2 > 65536) { // If v^2 > 65536, then v > 256
      v = sqrt((float)v2); // Velocity vector magnitude
      grain[i].vx = (int)(256.0 * (float)grain[i].vx / v); // Maintain heading
      grain[i].vy = (int)(256.0 * (float)grain[i].vy / v); // Limit magnitude
    }
  }

  // ...then update position of each grain, one at a time, checking for
  // collisions and having them react.  This really seems like it shouldn't
  // work, as only one grain is considered at a time while the rest are
  // regarded as stationary.  Yet this naive algorithm, taking many not-
  // technically-quite-correct steps, and repeated quickly enough,
  // visually integrates into something that somewhat resembles physics.
  // (I'd initially tried implementing this as a bunch of concurrent and
  // "realistic" elastic collisions among circular grains, but the
  // calculations and volument of code quickly got out of hand for both
  // the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)

  uint16_t       i, bytes, oldidx, newidx, delta;
  int16_t        newx, newy;

  for (i = 0; i < N_GRAINS; i++) {
    newx = grain[i].x + grain[i].vx; // New position in grain space
    newy = grain[i].y + grain[i].vy;
    if (newx > MAX_X) {              // If grain would go out of bounds
      newx         = MAX_X;          // keep it inside, and
      grain[i].vx /= -2;             // give a slight bounce off the wall
    } else if (newx < 0) {
      newx         = 0;
      grain[i].vx /= -2;
    }
    if (newy > MAX_Y) {
      newy         = MAX_Y;
      grain[i].vy /= -2;
    } else if (newy < 0) {
      newy         = 0;
      grain[i].vy /= -2;
    }

    oldidx = (grain[i].y / 256) * WIDTH + (grain[i].x / 256); // Prior pixel #
    newidx = (newy      / 256) * WIDTH + (newx      / 256); // New pixel #
    if ((oldidx != newidx) &&         // If grain is moving to a new pixel...
        img[newidx]) {                // but if that pixel is already occupied...
      delta = abs(newidx - oldidx);   // What direction when blocked?
      if (delta == 1) {               // 1 pixel left or right)
        newx         = grain[i].x;    // Cancel X motion
        grain[i].vx /= -2;            // and bounce X velocity (Y is OK)
        newidx       = oldidx;        // No pixel change
      } else if (delta == WIDTH) {    // 1 pixel up or down
        newy         = grain[i].y;    // Cancel Y motion
        grain[i].vy /= -2;            // and bounce Y velocity (X is OK)
        newidx       = oldidx;        // No pixel change
      } else { // Diagonal intersection is more tricky...
        // Try skidding along just one axis of motion if possible (start w/
        // faster axis).  Because we've already established that diagonal
        // (both-axis) motion is occurring, moving on either axis alone WILL
        // change the pixel index, no need to check that again.
        if ((abs(grain[i].vx) - abs(grain[i].vy)) >= 0) { // X axis is faster
          newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
          if (!img[newidx]) {             // That pixel's free!  Take it!  But...
            newy         = grain[i].y;    // Cancel Y motion
            grain[i].vy /= -2;            // and bounce Y velocity
          } else { // X pixel is taken, so try Y...
            newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
            if (!img[newidx]) {           // Pixel is free, take it, but first...
              newx         = grain[i].x;  // Cancel X motion
              grain[i].vx /= -2;          // and bounce X velocity
            } else { // Both spots are occupied
              newx         = grain[i].x;  // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;          // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;      // Not moving
            }
          }
        } else { // Y axis is faster, start there
          newidx = (newy / 256) * WIDTH + (grain[i].x / 256);
          if (!img[newidx]) { // Pixel's free!  Take it!  But...
            newx         = grain[i].x;    // Cancel X motion
            grain[i].vy /= -2;            // and bounce X velocity
          } else { // Y pixel is taken, so try X...
            newidx = (grain[i].y / 256) * WIDTH + (newx / 256);
            if (!img[newidx]) { // Pixel is free, take it, but first...
              newy         = grain[i].y; // Cancel Y motion
              grain[i].vy /= -2;         // and bounce Y velocity
            } else { // Both spots are occupied
              newx         = grain[i].x; // Cancel X & Y motion
              newy         = grain[i].y;
              grain[i].vx /= -2;         // Bounce X & Y velocity
              grain[i].vy /= -2;
              newidx       = oldidx;     // Not moving
            }
          }
        }
      }
    }
    grain[i].x  = newx; // Update grain position
    grain[i].y  = newy;
    img[oldidx] = 0;    // Clear old spot (might be same as new, that's OK)
    img[newidx] = 255;  // Set new spot
    grain[i].pos = newidx;
    //Serial.println(newidx);
  }
  }
}

void IRAM_ATTR display_updater() {
  display.display(display_draw_time);
}

void display_update_enable(bool is_enable)
{
  if (is_enable)
  {
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &display_updater, true);
    timerAlarmWrite(timer, 2000, true);
    timerAlarmEnable(timer);
  }
  else
  {
    timerDetachInterrupt(timer);
    timerAlarmDisable(timer);
  }
}

// SETUP - RUNS ONCE AT PROGRAM START --------------------------------------

void setup(void) {
  int i, j, bytes;

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
  delay(500);
  Serial.print(".");
  }
  timeClient.begin();
  //Serial.begin(115200);
  //Serial.println("Initialize MPU6050");
  // Define your display layout here, e.g. 1/32 step
  display.begin(32);
  display.setMuxDelay(0,1,0,0,0);   // Delay multiplexing - Important for some panel with slow multiplexer
  display.setFastUpdate(true);
  display.clearDisplay();

  display_update_enable(true);
  //display.showBuffer();

  memset(img, 0, sizeof(img)); // Clear the img[] array

  imgWrapper.setCursor(0, 26);
  imgWrapper.setFont(&FreeSansBold9pt7b);
  imgWrapper.setTextColor(myWHITE);
  //imgWrapper.setTextSize(2);  
  imgWrapper.print("CLOCK");
  
  for (i = 0; i < N_GRAINS; i++) { // For each sand grain...

    int imgIndex = 0;
    do {
      grain[i].x = random(WIDTH  * 256); // Assign random position within
      grain[i].y = random(HEIGHT * 256); // the 'grain' coordinate space
      // Check if corresponding pixel position is already occupied...
      for (j = 0; (j < i) && (((grain[i].x / 256) != (grain[j].x / 256)) ||
                              ((grain[i].y / 256) != (grain[j].y / 256))); j++);
      imgIndex = (grain[i].y / 256) * WIDTH + (grain[i].x / 256);
    } while (img[imgIndex] != 0); // Keep retrying until a clear spot is found
    img[imgIndex] = 255; // Mark it
    grain[i].pos = (grain[i].y / 256) * WIDTH + (grain[i].x / 256);
    grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
    
    grain[i].colour = myCOLORS[i%NUM_COLOURS];
  }

  TaskHandle_t xHandle = NULL;
  xTaskCreatePinnedToCore(pixelTask, "PixelTask1", 5000, 0, (2 | portPRIVILEGE_BIT), &xHandle, 0);
}

// MAIN LOOP - RUNS ONCE PER FRAME OF ANIMATION ----------------------------

void loop() {

  display.clearDisplay();
  for (int i = 0; i < N_GRAINS; i++) {
    int yPos = grain[i].pos / WIDTH;
    int xPos = grain[i].pos % WIDTH;
    display.drawPixel(xPos , yPos, grain[i].colour);
  }

  timeClient.update();
  //int dd = timeClient.getDay();
  //int hh = timeClient.getHours();
  //int mm = timeClient.getMinutes();
  //int ss = timeClient.getSeconds();
  //String formattedTime = timeClient.getFormattedTime();

  unsigned long rawTime = timeClient.getEpochTime();
  unsigned long hours = (rawTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (rawTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  unsigned long seconds = rawTime % 60;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  String hhmm = hoursStr + ":" + minuteStr ;

  display.setCursor(2, 16);
  display.setFont(&FreeSansBold12pt7b);
  display.setTextColor(myBLUE);
  //display.setTextSize(2); 
  display.print(hhmm);
  
  display.setCursor(0, 32);
  display.setFont(&FreeSansBold9pt7b);
  display.setTextColor(myWHITE);
  //display.setTextSize(2);  
  display.print("CLOCK");  
  
  display.setCursor(12, 60);
  display.setFont(&FreeSansBold18pt7b);
  display.setTextColor(myBLUE);
  //display.setTextSize(2); 
  display.print(secondStr);
  display.showBuffer();
}
