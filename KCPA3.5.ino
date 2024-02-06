#include <Adafruit_GFX.h>         // Core graphics library
#include <Adafruit_ST7789.h>      // Hardware-specific library for ST7789
#include <SdFat.h>                // SD card & FAT filesystem library
#include <Adafruit_SPIFlash.h>    // SPI / QSPI flash library
#include <Adafruit_ImageReader.h> // Image-reading functions
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
// #include <SD.h>                //substituted with SDFat
#include "Adafruit_FRAM_SPI.h"
#include <sstream>
#include <iostream>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_ZeroFFT.h"

using namespace std;

//---------- FFT ----------

//set this to 0 to disable autoranging and scale to FFT_MAX
#define AUTOSCALE 1

#if AUTOSCALE == 0
#define FFT_MAX 512
#endif

#define DATA_SIZE 1024 //power of 2, lower = faster

//the sample rate.
#define FS 2360

#define NUM_REFERENCE_LINES 6

#define GRAPH_OFFSET 0

#define GRAPH_WIDTH (240)
#define GRAPH_HEIGHT (135 - GRAPH_OFFSET)

#define GRAPH_MIN (135 - 2)
#define GRAPH_MAX (135 - GRAPH_HEIGHT)
static float xScale;

int16_t spec_data[DATA_SIZE];

//---------- SD CARD AND TFT ----------
#define SD_CS    9 // SD card select pin
#define TFT_CS  12 // TFT select pin
#define TFT_DC   10 // TFT display/command pin
#define TFT_RST  11 // Or set to -1 and connect to Arduino RESET pin

SdFat                SD;         // SD card filesystem
Adafruit_ImageReader reader(SD); // Image-reader object, pass in SD filesys
File myFile;

Adafruit_ST7789      tft    = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_Image       img;        // An image loaded into RAM
int32_t              width  = 0, // BMP image dimensions
                     height = 0;


//---------- BLUETOOTH ----------

// rewrite to name based on serial #
// #define SETBLUETOOTHNAME  "AT+GAPDEVNAME=_P523_tone_generator"
#define SETBLUETOOTHNAME  "AT+GAPDEVNAME=_P523_mic1"
// #define SETBLUETOOTHNAME  "AT+GAPDEVNAME=_P523_mic2"
// #define SETBLUETOOTHNAME  "AT+GAPDEVNAME=_P523_mic3"
// #define SETBLUETOOTHNAME  "AT+GAPDEVNAME=_P523_mic4"

#define name_start 14

// Size of the read buffer for incoming data
#define BUFSIZE 160   

// If set to 'true' enables debug output
#define VERBOSE_MODE  true  

// Feather M4 pins
#define BLUEFRUIT_SPI_CS               A5 // OLD: 10
#define BLUEFRUIT_SPI_IRQ              A4 // OLD: 5
#define BLUEFRUIT_SPI_RST              A3 // OLD: 6 

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ,
BLUEFRUIT_SPI_RST);

#define ASCII_CR 0x0A // carriage return is 0x0A.

char iPhone_incoming[BUFSIZE+1];
int iphone_incoming_index = 0;
char iPhone_last_message[BUFSIZE+1];
int iPhone_last_message_length = 0;

//---------- ANALOG INPUTS ----------
int analogPin = A0;

//---------- LED pins ----------
#define RED_LED_PIN 13
int LED_pin = RED_LED_PIN;
#define LED_OFF LOW
#define LED_ON HIGH

//---------- SETUP ----------

void setup(void) {

  ImageReturnCode stat; // Status from image-reading functions
  Serial.begin(9600);
  while(!Serial);       // Wait for Serial Monitor before continuing
  tft.init(135, 240);           // Init ST7789 320x240

  // SD card
  tft.print(F("Initializing filesystem..."));
  if(!SD.begin(SD_CS, SD_SCK_MHZ(10))) { // Breakouts require 10 MHz limit due to longer wires
    Serial.println(F("SD begin() failed"));
    for(;;); // Fatal error, do not continue
  }
  Serial.println(F("OK!"));

  Serial.print(F("Loading startup.bmp to screen..."));
  reader.drawBMP("/startup.bmp", tft, 0, 0);


  analogReadResolution(12);


  //---------- BLUETOOTH ----------
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setRotation(3);
  tft.setCursor(1, 1);
  tft.println(__FILE__);
  tft.print(__DATE__);
  tft.print("  ");
  tft.println(__TIME__);

  // set the LED-illuminating pin to be a digital output.
  pinMode(LED_pin, OUTPUT);
  // make sure the LED is off.
  digitalWrite(LED_pin, LED_OFF);

  /* Initialise the BLE module */
  tft.print(F("Bluetooth Setup..."));
  if ( !ble.begin(VERBOSE_MODE) ){
    tft.println(F("Couldn't find Bluefruit"));
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  tft.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    tft.print(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() )
    {
      tft.println(F("Couldn't factory reset"));
      error(F("Couldn't factory reset"));
    }
  }

  // Disable command echo from Bluefruit
  ble.echo(false);
  tft.print("Requesting Bluefruit info: ");
  ble.info();

  tft.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));

  // debug info is a little annoying after this point!
  ble.verbose(false);  

  // Wait for connection to iPhone
  tft.print("waiting for connection...");
  while (! ble.isConnected()) {delay(500);}
  tft.println("BLE is now connected.");

  // now try changing the name of the BLE device.
  Serial.println("About to change bluetoothLE module name via the command\n   ");
  Serial.println(SETBLUETOOTHNAME);
  ble.println(SETBLUETOOTHNAME);

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  // see Adafruit_BluefruitLE_SPI.cpp.
  // mode choices are  BLUEFRUIT_MODE_COMMAND and BLUEFRUIT_MODE_DATA

  Serial.println("Setup Complete");
  tft.print("Setup Complete");

  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(1,1);
  tft.println(F("Commands:"));
  tft.println(F("run - full test"));
  tft.println(F("# - manual data taking"));
  tft.println(F("set # - set run # (legacy)"));
  tft.println(F("sp - audio spectrum (restart to exit)"));
}


int startTime = micros();
const int size = 86000;
double freq = 35000;
int run = 0;


void loop() {
  // Check for user input from the serial monitor input field
  char n, inputs[BUFSIZE+1];

  
  if (Serial.available()){
    n = Serial.readBytes(inputs, BUFSIZE);
    // inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending to iPhone: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
  }

  // Echo received-from-iPhone data
  while (ble.available()){
    int c = ble.read();

    // Serial.print((char)c);

    iPhone_incoming[iphone_incoming_index] = (char)c;
    iphone_incoming_index++;
    if(iphone_incoming_index > BUFSIZE) {iphone_incoming_index = 0;}

    iPhone_last_message[iPhone_last_message_length] = (char)c;
    iPhone_last_message_length++;
    if(iPhone_last_message_length > BUFSIZE) {iPhone_last_message_length = 0;}

    if(c == ASCII_CR){
      Serial.print("Command: '");

      // add a null after the message, where the carriage return is.
      iPhone_last_message[iPhone_last_message_length - 1] = '\0';

      Serial.print(iPhone_last_message);
      Serial.println("'");
      iPhone_last_message_length = 0;

      if(is_number(iPhone_last_message)){ // manual data taking
            tft.fillScreen(ST77XX_BLACK);
            tft.println(F("started manual data taking"));
            int run_start = micros();
            for(int i=1; i<16; i++){
              takeData(extractIntegerWords(iPhone_last_message), i);
              while (micros()-run_start < 7000000*i){}
            }
         } 

      else if (iPhone_last_message[0] == 's' && iPhone_last_message[1] == 'e' && iPhone_last_message[2] == 't'){ // set run number
            run = extractIntegerWords(iPhone_last_message);
            tft.println("Set run number");
           } 

      else if (iPhone_last_message[0] == 'r' && iPhone_last_message[1] == 'u' && iPhone_last_message[2] == 'n'){ // run the default set
            tft.fillScreen(ST77XX_BLACK);
            tft.println(F("started data taking sequence"));
            int run_start = micros();
            for(int i=1; i<6; i++){
              delay(1000);
              takeData(330, i);
              while (micros()-run_start < 7500000*i){}
            }
            for(int i=1; i<6; i++){
              delay(1000);
              takeData(440, i);
              while (micros()-run_start < 7500000*5 + 7500000*i){}
            }
            for(int i=1; i<6; i++){
              delay(1000);
              takeData(550, i);
              while (micros()-run_start < 7500000*10 + 7500000*i){}
            }
           } 

      else if (iPhone_last_message[0] == 's' && iPhone_last_message[1] == 'p'){ // easter egg
            spec();
           } 

      else{ 
            Serial.println("unrecognized command");
            ble.println("unrecognized command");
         }
    }
  }

}



//---------- A small helper ----------
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


//---------- Blink the red LED ----------
void blink_LED(uint32_t blink_ms){
  // blink the LED for the specofied number of milliseconds

  uint32_t start_time = millis();

  // make sure the LED is off, initially.
  digitalWrite(LED_pin, LED_OFF);

  while(millis() - start_time < blink_ms)
  {
    digitalWrite(LED_pin, LED_ON);
    delay(50);
    digitalWrite(LED_pin, LED_OFF);
    delay(50);
  }
}

//---------- Check if string is a number ----------
bool is_number(const std::string& s){
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}

//---------- get the first number from a string ----------
int extractIntegerWords(string str){
  stringstream ss;

  /* Storing the whole string into string stream */
  ss << str;

  /* Running loop till the end of the stream */
  string temp;
  int found;
  while (!ss.eof()) {
      /* extracting word by word from stream */
      ss >> temp;

      /* Checking the given word is integer or not */
      if (stringstream(temp) >> found){
          return found;
      }
    }
  return 0;
}

//---------- data taking and saving ----------
void takeData (double run_freq, int run){
  uint16_t data[size];
  ble.println("starting data taking");    
  // wait for sound to start up  
  blink_LED(300);
  digitalWrite(LED_pin, HIGH);

  // take the data
  startTime = micros();
  for (int i = 0; i<size; i++){
    data[i] = analogRead(A0);
  }
  freq = size/((micros() - startTime) * 0.000001);

  // save the data
  stringstream stream;
  stream<<run_freq<<" "<<run;
  string str1, str2;
  stream>>str1>>str2;
  string title = str2 + "a" + str1 +".txt";
  tft.println(title.c_str());
  SD.remove(title.c_str());
  myFile = SD.open(title.c_str(), FILE_WRITE);
  if(myFile){
    myFile.println("raw data");
    for (int j=0; j<size;j++){
      myFile.println(data[j]);
    }
    myFile.println(freq);
    myFile.close();
    ble.println("data saved");
    // blink_LED(100);
  }
  else{
    tft.println("Error saving data");
  }
  digitalWrite(LED_pin, LOW);
}

//---------- FOR PLOTTING SPECTRUM ----------
void drawReference(){
  //draw some reference lines
  uint16_t refStep = DATA_SIZE / 2 / NUM_REFERENCE_LINES;
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_RED);
  for(int i=0; i<DATA_SIZE/2 - refStep; i+=refStep){
    uint16_t fc = FFT_BIN(i, FS, DATA_SIZE);
    uint16_t xpos = map(i, 0, DATA_SIZE/2, 0, GRAPH_WIDTH);
    tft.setCursor(xpos, 0);
    tft.drawLine(xpos, GRAPH_MIN, xpos, GRAPH_MAX, ST77XX_RED);
    
    tft.print(fc);
  }
}

//----------SPECTROGRAM----------
void spec(){
  while(true){
    int32_t avg = 0;
    for(int i=0; i<DATA_SIZE; i++){
      int16_t val = analogRead(A0);
      avg += val;
      spec_data[i] = val;
    }

    //remove DC offset and gain up to 16 bits
    avg = avg/DATA_SIZE;
    for(int i=0; i<DATA_SIZE; i++) spec_data[i] = (spec_data[i] - avg) * 64;
      
    //run the FFT
    ZeroFFT(spec_data, DATA_SIZE);

  #if AUTOSCALE
    //get the maximum value
    float maxVal = 0;
    //data is only meaningful up to sample rate/2, discard the other half
    for(int i=0; i<DATA_SIZE/2; i++) if(spec_data[i] > maxVal) maxVal = spec_data[i];

    //normalize to the maximum returned value
    for(int i=0; i<DATA_SIZE/2; i++)
      spec_data[i] =(float)spec_data[i] / maxVal * GRAPH_HEIGHT;
  #endif

    tft.fillScreen(ST77XX_BLACK);
    drawReference();

    //draw all points
    int16_t lasty, thisy;
    thisy = GRAPH_MIN;
    for(int i=1; i<GRAPH_WIDTH; i++){
      uint16_t ix = map(i, 0, GRAPH_WIDTH, 0, DATA_SIZE/2);
      
  #if AUTOSCALE
      thisy = constrain(map(spec_data[ix], 0, GRAPH_HEIGHT, GRAPH_MIN, GRAPH_MAX), GRAPH_OFFSET, GRAPH_MIN);
  #else
      thisy = constrain(map(spec_data[ix], 0, FFT_MAX, GRAPH_MIN, GRAPH_MAX), GRAPH_OFFSET, GRAPH_MIN);
  #endif
  
      tft.drawLine(i - 1, lasty, i, thisy, ST77XX_GREEN);
      lasty = thisy;
    }
  }
}

//---------- Data analysis ----------
bool analyze(const std::string& s){
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}
