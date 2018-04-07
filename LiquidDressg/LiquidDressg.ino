/*
   This is a modified version of the Visualize101.ino example sketch
       https://github.com/arduino-libraries/MadgwickAHRS/blob/master/examples/Visualize101/Visualize101.ino

   Modified starting March 10th, 2018 by Jane Hacker (JAH)

   XYZ_M = Macro
   xyz_g = Global
   xyz_t = Type

   Sorry, comment styles are kinda all over :/
*/

//**********
// Libraries
//**********
#include <math.h>               // For things like `atan2` and `PI`
#include <CurieIMU.h>           // Should be installed along with "Intel Curie Boards" in Boards Manager
#include <MadgwickAHRS.h>       // Search for "Madgwick" in Library Manager
#include <Adafruit_NeoPixel.h>  // Search for "Adafruit NeoPixel" in Library Manager


//***********************
// Definitions and Macros
//***********************
#define PIX_PIN        6          // PWM pin for NeoPix data
#define PIXELS 300
#define PIX_NUM        (float)6  // Number of NeoPix in circle
#define PIX_BRIGHTNESS 32         // The brightness of the NeoPix

#define DEG_PER_PIX  (PIX_NUM / 360)
#define DEG2PIX_M(a) (DEG_PER_PIX * a)


//****************************//
#define TAIL_LEN           5  // Length of the following tail
#define PIX_OVERFLOW_M(a)  (a > 5 - 1 ? a - 6 : a)
#define PIX_UNDERFLOW_M(a) (a < 0 ? a + 6 : a)
//****************************//


//***********************
// Type Definitions
//***********************
typedef uint32_t color_t;


//********
// Objects
//********

// For IMU
Madgwick          filter_g;

// For NeoPix
Adafruit_NeoPixel pixels_g   = Adafruit_NeoPixel(PIXELS, PIX_PIN, NEO_GRB + NEO_KHZ800);


//*********************
// Basic type variables
//*********************

// For IMU
float             accelScale, gyroScale;
unsigned long     microsPerReading, microsPrevious;

// For NeoPix
int               lastLitPix_g = 0;
color_t           onColor_g    = pixels_g.Color(0, 0, 150);
color_t           offColor_g   = pixels_g.Color(0, 0, 0);

// Color values for tail
color_t           midColor_g   = pixels_g.Color(150, 0, 0);
color_t           endColor_g   = pixels_g.Color(0, 0, 150);



//******************
// Arduino Functions
//******************
void setup() {
  while (!Serial);
  Serial.begin(9600);

  // Start the IMU
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);

  // Start the filter for IMU output
  filter_g.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);

  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // Initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious   = micros();

  // Start and initialize NeoPix
  pixels_g.begin();
  pixels_g.setBrightness(PIX_BRIGHTNESS);
  pixels_g.show();
}


void loop() {
  float roll, pitch, heading;
  unsigned long microsNow;
  // check if it's time to read data and update the filter
  microsNow = micros();

  if (microsNow - microsPrevious >= microsPerReading) {
    // Get values from IMU and update filter
    readImuUpdateFilter();

    // Get heading, pitch and roll
    roll    = filter_g.getRoll();
    pitch   = filter_g.getPitch();
    heading = filter_g.getYaw();

    // Print out heading, pitch and roll
    //debugPrintHPR(heading, pitch, roll);

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // This is where the magic starts
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    useFilteredImuData(pitch, roll);

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}


/*************************
  JAH additional functions
 *************************/

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// This is where the magic happens
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void useFilteredImuData(float pitch, float roll) {
  // Tilt caculations
  int tiltToward = (((atan2(pitch, roll) / PI) * 180) + 180);
  // Print tilting toward 0-360
  Serial.print("Tilting toward: ");
  Serial.print(tiltToward);
  Serial.println("deg");
  int pix2Light = DEG2PIX_M(tiltToward);
  // Print the NeoPix to light from 0 to PIX_NUM (default 16) on a NeoPix ring
  Serial.print("Light NeoPix (0-indexed): ");
  Serial.println(pix2Light);

  //
  // Neopix update
  //

  /*
      // For single point & w/o clear
      // Turn off last NeoPix that was on
      pixels_g.setPixelColor(lastLitPix_g, offColor_g);
      lastLitPix_g = pix2Light;
      drawPix(pix2Light);
  */
  // For a little tail
  lightTail(pix2Light, lastLitPix_g);

  //TODO: Passing in a global to be later modified in the function w/o using the reference; PICK ONE
}

void drawPix(int toLight) {
  pixels_g.setPixelColor(toLight, onColor_g);
  pixels_g.show();
}



// I just moved these out of the loop function
void readImuUpdateFilter() {
  int   aix, aiy, aiz;
  int   gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;

  // read raw data from CurieIMU
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  // update the filter, which computes orientation
  filter_g.updateIMU(gx, gy, gz, ax, ay, az);
}


/***************************
   Original sketch functions
 ***************************/
float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}


/***************************
   extraneous
 ***************************/

//****************************// Kinda boring actualy

void lightTail(int toLight, int lastLit) {
  // Only if there is a change
  if (lastLit != toLight) {
    pixels_g.clear();
    if (PIX_NUM - 1 == toLight && 0 == lastLit) {
      lightUpDown(1, toLight);
    }
    else if (PIX_NUM - 1 == lastLit && 0 == toLight) {
      lightUpDown(0, toLight);
    }
    else if (toLight < lastLit) {
      lightUpDown(1, toLight);
    }
    else {
      lightUpDown(0, toLight);
    }

    // Update last lit pixel
    lastLitPix_g = toLight;

    // Turn on NeoPix at (0-indexed) toLight
    //drawPix(toLight*20);
    boop(toLight);
  }
}
void boop( int toLight) {

  // test for now
  if (toLight == 0) {
    for (int i = 0; i < 25; ++i) {
      drawPix(i);
    }
  }
  if (toLight == 1) {
    for (int i = 26; i < 50; ++i) {
      drawPix(i);
    }
  }
  if (toLight == 2) {
    for (int i = 51; i < 75; ++i) {
      drawPix(i);
    }
  }
  if (toLight == 3) {
    for (int i = 75; i < 100; ++i) {
      drawPix(i);
    }
  }
}

void lightUpDown(bool up, int p2l) {
  //  for(int i = 1; i < TAIL_LEN; ++i) {
  //    int     p = PIX_OVERFLOW_M(p2l + i);
  //    color_t c = pixels_g.Color(0,0,150);
  //    //color_t c = i == 2 ? midColor_g : endColor_g;
  //    pixels_g.setPixelColor(p, c);
  //  }
  ////    if(!up)
  ////         = PIX_UNDERFLOW_M(p2l - i);
  ////  }
}
// I just moved these out of the loop function
void debugPrintHPR(float h, float p, float r) {
  Serial.print("Orientation: ");
  Serial.print(h);
  Serial.print(" ");
  Serial.print(p);
  Serial.print(" ");
  Serial.println(r);
  Serial.print(" ");
}

