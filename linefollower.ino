#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>


#include <Wire.h>
#include "Adafruit_TCS34725.h"

//Neopixels
#include <Adafruit_NeoPixel.h>

//Motorshield
#include <Adafruit_MotorShield.h>

//Defines
//// Which pin on the Arduino is connected to the NeoPixels?
#define PIN 9 // Pin that the NeoPixels are attached to
#define NUMPIXELS 2 // number of NeoPixels
#define BUTTON0 5 //defines left button
#define BUTTON1 6 // defines right button
#define MOTORPOWER 229  //defines default motor power
#define BUTTONDELAY 1500 //minimum time in milliseconds needed between button clicks

//Encoders
//Right encoder
#define encoderMotor1ChannelA 10
#define encoderMotor1ChannelB 11
//Left encoder
#define encoderMotor2ChannelA 12
#define encoderMotor2ChannelB 13

 
volatile int lastEncodedR = 0;
volatile long encoderValueR = 0;
volatile int lastEncodedL = 0;
volatile long encoderValueL = 0;
 
long lastencoderValueR = 0;
long lastencoderValueL = 0;
 
int lastMSBR = 0;
int lastLSBR = 0;
int lastMSBL = 0;
int lastLSBL = 0;



//TFT screen setup
// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

//Neopixels setup
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//Color sensor setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);

//Motorshield setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myLeftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myRightMotor = AFMS.getMotor(2);
//

//Globals
float kp = 121;
float ki = 0.399;
float kd = 0.0989;
float oldError = 0;
float sumError = 0;
uint16_t nominalLux = 100;
uint16_t blackLux = 255;
uint16_t whiteLux = 0;
int colorSwitch0 = 0;
int colorSwitch1 = 0;
bool checkButton0 = LOW;
bool checkButton1 = LOW;
unsigned long stopwatch = 0;
unsigned long button1Click = 0;
unsigned long button0Click = 0;
int runPID = 0;



void ITR_BUTTON0 () {
  checkButton0 = HIGH;
}

void ITR_BUTTON1 () {
  checkButton1 = HIGH;
}

void IRAM_ATTR updateEncoderL(){
  int MSB = digitalRead(encoderMotor2ChannelA); //MSB = most significant bit
  int LSB = digitalRead(encoderMotor2ChannelB); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedL << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueL ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueL --;
 
  lastEncodedL = encoded; //store this value for next time
 
}
 
void IRAM_ATTR updateEncoderR(){
  int MSB = digitalRead(encoderMotor1ChannelA); //MSB = most significant bit
  int LSB = digitalRead(encoderMotor1ChannelB); //LSB = least significant bit
 
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedR << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
	encoderValueR ++;
  }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
	encoderValueR --;
  }
 
  lastEncodedR = encoded; //store this value for next time
 
}


uint16_t measureLux () {
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  lux = tcs.calculateLux(r, g, b);
  return lux;
  }

long PID(){
  int error = 0;
  int deltaError = 0;
  long PID = 0;
  error = measureLux() - nominalLux;
  sumError += error;
  deltaError = error - oldError;
  oldError = error;
  PID = error*kp + sumError*ki + deltaError*kd;
  return PID;
}

void setup() {
  // Put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(BUTTON0, INPUT_PULLUP);
  attachInterrupt(BUTTON0, ITR_BUTTON0, RISING);
  pinMode(BUTTON1, INPUT_PULLUP);
  attachInterrupt(BUTTON1, ITR_BUTTON1, RISING);

  // Initialize NeoPixels
  pixels.begin();

  // Setup the initial colors on the buttons
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); //right from behind
  pixels.setPixelColor(1, pixels.Color(0, 0, 255)); //left from behind
  pixels.show();   // Send the updated pixel colors to the hardware.
  
  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  tft.init(135, 240); // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(3);

  tft.println("TFT Works");

  //Check if the color sensor is working
  if (tcs.begin()) {
    tft.println("Found sensor");
  } else {
    tft.println("No TCS34725 found ... check your connections");
    //while (1);
  }

  AFMS.begin();

  //Check for the motor shield
  //tft.println("Adafruit Motorshield v2 - DC Motor test!");
  /*if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    tft.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  tft.println("Motor Shield found.");*/

  tft.println("Motor shield up");
  delay(5000);

  tft.fillScreen(ST77XX_BLACK);
  stopwatch = millis();  //start the stopwatc

  pinMode(encoderMotor1ChannelA, INPUT_PULLUP);
  pinMode(encoderMotor1ChannelB, INPUT_PULLUP);
  pinMode(encoderMotor2ChannelA, INPUT_PULLUP);
  pinMode(encoderMotor2ChannelB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderMotor1ChannelA), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderMotor1ChannelB), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderMotor2ChannelA), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderMotor2ChannelB), updateEncoderL, CHANGE);


}

void loop() {
  // put your main code here, to run repeatedly: 
  uint16_t lux=0;
  long PIDValue = 0;
  if (checkButton0 == HIGH){
    button0Click = millis();
    if ((button0Click - stopwatch) > BUTTONDELAY){
      stopwatch = millis();
      if (colorSwitch0 == 1){
        uint16_t sumWhite = 0;
        for (int x =0;x<10; x++){
          lux = measureLux();
          sumWhite += lux;
        }
        whiteLux = sumWhite/10;
        tft.fillScreen(ST77XX_BLACK);
        tft.setCursor(0, 30);
        tft.println("White = ");
        tft.println(whiteLux);
        pixels.setPixelColor(0, pixels.Color(0, 255, 0));
        pixels.show(); 
        checkButton0 = LOW;
        colorSwitch0=2;
        nominalLux = (blackLux+whiteLux)/2;
        tft.println("Nominal = ");
        tft.println(nominalLux);
        delay(5000);
      }
    }
    if (colorSwitch0 == 0){
      uint16_t sumBlack = 0;
      for (int x =0;x<10; x++){
        lux = measureLux();
        sumBlack += lux;
      }
      blackLux = sumBlack/10;
      tft.setCursor(0, 30);
      tft.println("Black = ");
      tft.println(blackLux);
      pixels.setPixelColor(0, pixels.Color(255, 255, 0));
      pixels.show(); 
      checkButton0 = LOW;
      colorSwitch0=1;
      delay(5000);
    }
    checkButton0=LOW;
  }
  if (checkButton1 == HIGH) {
    button1Click = millis();
    if ((button1Click - stopwatch) > BUTTONDELAY){
      stopwatch = millis();
      switch (colorSwitch1) {
        case 0:
          runPID = 1;
          break;
        case 1:
          runPID = 0;
          myLeftMotor->setSpeed(0);
          myLeftMotor->setSpeed(0);
          myLeftMotor->run(RELEASE);
          myRightMotor->run(RELEASE); 
          pixels.setPixelColor(1, pixels.Color(0, 0, 255));
          pixels.show(); 
          break;
          
        default:
          break;  
      } 
    if (colorSwitch1==0) {
        colorSwitch1=1;
      }
      else {
        colorSwitch1=0;
      }
      
    }
    checkButton1 = LOW;
  }

  if (runPID == 1){
     int leftSpeed = 0;
    int rightSpeed = 0;

    PIDValue = PID();
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 30);
    tft.println("PID Value = ");
    tft.println(PIDValue);
       leftSpeed = MOTORPOWER-PIDValue;
    if (leftSpeed < -255) {
        leftSpeed = -255;
      }
    if (leftSpeed > 255) {
        leftSpeed = 255;
      }
    rightSpeed = MOTORPOWER+PIDValue;
    if (rightSpeed < -255) {
        rightSpeed = -255;
      }
    if (rightSpeed > 255) {
        rightSpeed = 255;
      }
    if (leftSpeed < 0){
      myLeftMotor->run(BACKWARD);
      myRightMotor->run(FORWARD);
      myLeftMotor->setSpeed(-1*leftSpeed);
      myRightMotor->setSpeed(rightSpeed);
      } 
    else if (rightSpeed <0) {
       myLeftMotor->run(FORWARD);
       myRightMotor->run(BACKWARD);
       myLeftMotor->setSpeed(leftSpeed);
       myRightMotor->setSpeed(-1*rightSpeed);
       } 
     else {
       myLeftMotor->run(FORWARD);
       myRightMotor->run(FORWARD);
       myLeftMotor->setSpeed(leftSpeed);
       myRightMotor->setSpeed(rightSpeed);
     }
     pixels.setPixelColor(1, pixels.Color(255, 0, 255));
     pixels.show(); 
  }


  

  
  //Serial.println(lux,DEC);
  
  //tft.fillScreen(ST77XX_BLACK);
  delay(10);
}
