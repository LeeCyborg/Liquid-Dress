/*
 * This is a modified version of the Visualize101.ino example sketch
 *     https://github.com/arduino-libraries/MadgwickAHRS/blob/master/examples/Visualize101/Visualize101.ino
 * 
 * Modified starting March 10th, 2018 by Jane Hacker (JAH)
 * 
 * XYZ_M = Macro
 * xyz_g = Global
 * xyz_t = Type
 * 
 * Sorry, comment styles are kinda all over :/
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
#define PIX_NUM        (float)16  // Number of NeoPix in circle
#define PIX_BRIGHTNESS 32         // The brightness of the NeoPix

#define DEG_PER_PIX  (PIX_NUM / 360)
#define DEG2PIX_M(a) (DEG_PER_PIX * a)


//****************************//
#define TAIL_LEN           4  // Length of the following tail
#define PIX_OVERFLOW_M(a)  (a > PIX_NUM - 1 ? a - PIX_NUM : a)
#define PIX_UNDERFLOW_M(a) (a < 0 ? a + PIX_NUM : a)
//****************************//
//****************************//
#undef LOSE_MY_MIND        //Change `undef` to `define` to include wackyness

#ifdef LOSE_MY_MIND
#define PIX_MEM_LEN 3  //Remember the index of the last three NeoPixels
#endif
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
Adafruit_NeoPixel pixels_g   = Adafruit_NeoPixel(PIX_NUM, PIX_PIN, NEO_GRB + NEO_KHZ800);


//*********************
// Basic type variables
//*********************

// For IMU
float             accelScale, gyroScale;
unsigned long     microsPerReading, microsPrevious;

// For NeoPix
int               lastLitPix_g = 0;
color_t           onColor_g    = pixels_g.Color(0,150,0);
color_t           offColor_g   = pixels_g.Color(0,0,0);

// Color values for tail
color_t           midColor_g   = pixels_g.Color(150,0,0);
color_t           endColor_g   = pixels_g.Color(0,0,150);

#ifdef LOSE_MY_MIND
// Make an array of ints to store last pix index
int *             lastLitPixs_g = (int*)calloc(PIX_MEM_LEN, sizeof(int));
#endif


//******************
// Arduino Functions
//******************
void setup() {
  while(!Serial);
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
 *JAH additional functions
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

#ifdef LOSE_MY_MIND
    //For being bored
    pixels_g.clear();
    yayPointers(pix2Light);
    drawPix(pix2Light);
#else
    // For a little tail
    lightTail(pix2Light, lastLitPix_g); //TODO: Passing in a global to be later modified in the function w/o using the reference; PICK ONE
#endif
}

void drawPix(int toLight) {
    pixels_g.setPixelColor(toLight, onColor_g);
    pixels_g.show();
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
 * Original sketch functions
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
 * extraneous
 ***************************/

//****************************// Kinda boring actualy

void lightTail(int toLight, int lastLit) {
  // Only if there is a change
  if(lastLit != toLight) {
        pixels_g.clear();
        
        if(PIX_NUM - 1 == toLight && 0 == lastLit) {
            lightUpDown(1, toLight);
        }
        else if(PIX_NUM - 1 == lastLit && 0 == toLight) {
            lightUpDown(0, toLight);
        }
        else if(toLight < lastLit) {
            lightUpDown(1, toLight);
        }
        else {
            lightUpDown(0, toLight);
        }
    
        // Update last lit pixel
        lastLitPix_g = toLight;
        
        // Turn on NeoPix at (0-indexed) toLight
        drawPix(toLight);
    }
}

 void lightUpDown(bool up, int p2l) {
  for(int i = 1; i < TAIL_LEN; ++i) {
    int     p = PIX_OVERFLOW_M(p2l + i);
    color_t c = i > 1 ? midColor_g : endColor_g;
    
    if(!up)
        p = PIX_UNDERFLOW_M(p2l - i);
        
    pixels_g.setPixelColor(p, c);
  }
}
    
#ifdef LOSE_MY_MIND
//****************************// Ignore this if you value your sanity

void yayPointers(int pix2Light) {
    /*
     * Let's have a little fun >)
     */
     if(pix2Light != lastLitPixs_g[0] &&
        (  pix2Light > lastLitPixs_g[0] + 1 ||
           pix2Light < lastLitPixs_g[0] - 1)) 
        push(lastLitPixs_g, pix2Light);
        
    showPixArray();

    /* What's that? How can you use bracket notation?! It wasn't declared as an array type?!
     * Have you figured out arrays' secret yet? ;)
     */
     pixels_g.setPixelColor(lastLitPixs_g[0], midColor_g);
     pixels_g.setPixelColor(lastLitPixs_g[1], endColor_g);
     pixels_g.setPixelColor(lastLitPixs_g[2], offColor_g);
}


void showPixArray() {
    Serial.print("{ ");
    
    /* Get the value/data stored at address stored in the pointer "lastLitPixs_g" (using `*`) */
    Serial.print(*lastLitPixs_g); // Get the value stored in the *first* element of the array
    Serial.print(", ");
    
    /* Here we get the _address_ which is stored in the _pointer_ which is simply `lastLitPixs_g`...
       (just so you know, the pointer just holds the address of the _fisrt_element_of_the_array_)
       ... then we add the size of an `int` (`sizeof(int)`) to the address...
       ... and get the value/data at that address (`*`)...
     */
    Serial.print( *(int*)((size_t)lastLitPixs_g + sizeof(int)) );  // Get the value stored in the *second* element of the array
    /* ... basically we stepped over the data in the first element by changing the address by the size of the data :) */
    Serial.print(", ");
    
    /* But we don't need to!
       The compiler knows what size of type we are dealing with...
       ... so adding `2` is the same as adding `sizeof(int) * 2` to the address...
       ... this is known as "Pointer Arithmetic"!
      */
    Serial.print(*(lastLitPixs_g + 2)); // Get the value stored in the *third* element of the array
    Serial.println(" }");
}


// Oh yes, fun to be had!
int * push(int * a, int i) {
    memmove(a + 1, a, sizeof(int) * (PIX_MEM_LEN -1));
    *a = i;

    return a;
}
//***************************
#endif
