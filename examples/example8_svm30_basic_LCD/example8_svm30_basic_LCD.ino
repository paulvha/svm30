/*  example how to read information from SVM30 with display on LCD (https://www.sparkfun.com/products/16396)
 *  
 *  October 27, 2020 / paulvha@hotmail.com
 *   add LCD display
 *
 *  September 20, 2019 / paulvha@hotmail.com
 *   initial version : 1.0
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE. 
 * 
 * 
 * connected to MEGA2560, DUE
 *                    DUE
 * SVM30              MEGA2560      LCD
 * 1. SCL  -----------   SCL -----  SC
 * 2. GND  -----------   GND -----  GND
 * 3. VCC  -----------   +5V -----  RAW 3V3 -9
 * 4. SDA  -----------   SDA -----  DA
 * 
 * NO NEED FOR PULL RESISTORS, SVM30 already has them.
 * 
 * Sparkfun ESP32 thing
 ******************************************************************************************
 * WARNING: THE SVM30 NEEDS BETWEEN 4.5V AND 5.5V (TYPICAL 5V). RUNNING ON 3v3 MAKES IT UNSTABLE.
 * The SVM30 seems to have on-board pull-up resistors to +5V
 * THE ESP32 PINS CAN HANDLE UP TO 3v3 ONLY AND AS SUCH YOU USE A BI-DIRECTIONAL LEVEL CONVERTER. 
 * e.g. https://www.sparkfun.com/products/12009
 *****************************************************************************************
 * SVM30    LCD               ESP32
 * 1. SCL -- SC  ----lll------  SCL (pin 22) 
 * 2. GND -- GND ------------  GND 
 * 3. VCC -- RAW3V3-9--------  +5V/VUSB 
 * 4. SDA -- SA  ----lll-----  SDA (pin 21)
 * 
 * Sparkfun ESP32
 *
 *    Make sure :
 *      - use the VUSB pin to connect to the level converter (HV) and VCC of the SVM30
 *      - use the 3V3 pin to connect to the level converter (LV)
 *      - Select the Sparkfun ESP32 thing board before compiling
 *      - connect GND, SDA, SCL lines to the level converter and SVM30
 *      - The serial monitor is NOT active (will cause upload errors)
 *      - Press GPIO 0 switch during connecting after compile to start upload to the board
 *      - Press reset on the ESP32 thing after the sketch is loaded
 * 
 *
 * connected to Artemis /Apollo3 
 ******************************************************************************************
 * WARNING: THE SVM30 NEEDS BETWEEN 4.5V AND 5.5V (TYPICAL 5V). RUNNING ON 3v3 MAKES IT UNSTABLE.
 * THE Artemis PINS CAN HANDLE UP TO 3v3 ONLY AND AS SUCH YOU USE A BI-DIRECTIONAL LEVEL CONVERTER. 
 * e.g. https://www.sparkfun.com/products/12009
 ***************************************************************************************** 
 * SVM30     LCD               Artemis / ATP / Apollo3
 * 1. SCL -- SC  ----lll------  SCL  
 * 2. GND -- GND ------------  GND 
 * 3. VCC -- RAW3V3-9--------  +5V
 * 4. SDA -- SA  ----lll-----  SDA 
 *
 ************************************************************************************************
 * Tested on Arduino UNO with LCD but the compiler indicates : Low memory available, stability problems may occur.
 * This has been a recipe for failure in the past and I would not recommend it. Maybe a Sparkfun Redboard or other 
 * Uno formfactor has larger memory and I expect it to work.
 * 
 * Not tested on ESP8266 as you need 5V to run stable and the ESP8266 is 3V3 only.
 ************************************************************************************************
 *
 * LCD info
 * https://www.sparkfun.com/products/16396
 *
 * MAKE SURE TO INSTALL http://librarymanager/All#SparkFun_SerLCD.
 * 
 * The SparkFun SerLCD is an AVR-based, serial enabled LCD that provides a simple and cost 
 * effective solution for adding a 16x2 Black on RGB Liquid Crystal Display into your project. 
 * Both the SVM30 and LCD are connected on the same WIRE device.
 *
 * The Qwiic adapter should be attached to the display as follows. If you have a model (board or LCD)
 * without QWiic connect, or connect is indicated.
 * Display  / Qwiic Cable Color        LCD -connection without Qwiic
 * GND      / Black                    GND
 * RAW      / Red                      3V3 -9 v
 * SDA      / Blue                     I2c DA
 * SCL      / Yellow                   I2C CL
 *
 * Note: If you connect directly to a 5V Arduino instead, you *MUST* use
 * a level-shifter on SDA and SCL to convert the i2c voltage levels down to 3.3V for the display.
 *
 * If ONLYONBUTTON is set, connect a push-button switch between pin BUTTONINPUT and ground.
 */

/////////////////////////////////////////////////////////////
/* define driver debug
 * false : no messages
 * true : display driver debug messages
 *//////////////////////////////////////////////////////////////
#define DEBUG false

/////////////////////////////////////////////////////////////
/* define number of seconds between display results
 * 
 *//////////////////////////////////////////////////////////////
#define DELAY 10

//////////////////////////////////////////////////////////////////////////
//                SELECT THE INTERFACES                                 //
//////////////////////////////////////////////////////////////////////////
#define SVM30WIRE Wire
#define LCDCON Wire

//////////////////////////////////////////////////////////////////////////
//                SELECT LCD settings                                   //
//////////////////////////////////////////////////////////////////////////
#define LCDTEMPCELSIUS  true     // set to false to display temperature in Fahrenheit

#define LCDBACKGROUNDCOLOR  1    // Normal background color: 1 = white, 2 = red, 3 = green 4 = blue 5 = off

#define CO2LIMIT  700            // Background LCD color will start LCDBACKGROUNDCOLOR and turn to red if 
                                 // CO2 is above this limit to return to LCDBACKGROUNDCOLOR background when below. 
                                 // set to zero to disable
                                 
#define ONLYONLIMIT false        // only display the results on the LCD display if the CO2LIMIT is exceeded
                                 // set to false disables this option.
                                 // do NOT select together with ONLYONBUTTON
                                 // make sure to set CO2LIMIT > 0 (compile will fail)
                               
#define ONLYONBUTTON false       // only display the results on the LCD display (red) if the CO2LIMIT
                                 // is exceeded OR for LCDTIMEOUT seconds if a button is pushed
                                 // set to false disables this option
                                 // do NOT select together with ONLYONLIMIT
                                 // if CO2LIMIT is zero the LCD will only display when button is pressed  
#if ONLYONBUTTON == true                    
#define BUTTONINPUT  10         // Digital input where button is connected for ONLYONBUTTON between GND
                                 // is ignored if ONLYONBUTTON is set to false
                                 // Artemis / Apollo3 set as D27
                                 // ESP32, Arduino set as 10

#define LCDTIMEOUT 10            // Number of seconds LCD is displayed after button was pressed 
#endif                           // is ignored if ONLYONBUTTON is set to false        
                                                               
//////////////////////////////////////////////////////////////////////////
//////////////// NO CHANGES BEYOND THIS POINT NEEDED /////////////////////
//////////////////////////////////////////////////////////////////////////

// checks will happen at pre-processor time to 
#if ONLYONLIMIT == true && ONLYONBUTTON == true 
#error you can NOT set BOTH ONLYONLIMIT and ONLYONBUTTON to true
#endif

#if ONLYONLIMIT == true && CO2LIMIT == 0
#error you MUST set CO2LIMIT when ONLYONLIMIT to true
#endif

// function prototypes (sometimes the pre-processor do not create prototypes themself on ESPxx)
void read_id();
void read_featureSet();
void Errorloop(char *mess);
void read_values();
void read_baseline();
void KeepTrigger(uint8_t del);
bool checkButton();
void lcdinit();
void lcdsetbackground();
void printLCD(bool dd);
int FromFloat(char *buf, double number, uint8_t digits);
 
#include <svm30.h>
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD

// create instance
SVM30 svm;         // Initialize the library with default I2C address 0x70
SerLCD lcd;         // Initialize the library with default I2C address 0x72

struct svm_values v;

byte ppm[8] = {     // ppm custom character
  0b11100,
  0b10100,
  0b11100,
  0b10000,
  0b00111,
  0b00101,
  0b00111,
  0b00100
};

#if LCDTEMPCELSIUS == true 
byte deg[8] = {     // celsius custom character
  0b11100,
  0b10100,
  0b11100,
  0b00000,
  0b01111,
  0b01100,
  0b01100,
  0b01111
};
#else
byte deg[8] = {   // Fahrenheit custom character
  0b11100,
  0b10100,
  0b11100,
  0b00000,
  0b01111,
  0b01100,
  0b01110,
  0b01100
};
#endif

void setup() {
  
  Serial.begin(115200);

  Serial.println(F("Hi there, this is example 8.\nReading information from the SVM30 + LCD"));

  // enable debug messages
  svm.EnableDebugging(DEBUG);
  
  LCDCON.begin();
  SVM30WIRE.begin();
    
  // initialize LCD
  lcdinit();
  
  if (! svm.begin(&SVM30WIRE)){
    lcd.setBacklight(255, 0, 0);    // bright red
    lcd.clear();
    lcd.write("SVM30 comms");
    lcd.setCursor(0, 1);            // pos 50, line 1
    lcd.write("Error");
    Errorloop((char *) "could not connect with SVM30.");
  }

  Serial.print(F("Driver version : "));
  Serial.println(svm.GetDriverVersion());
    
  // try to detect SVM30 sensors
  if (svm.probe()) Serial.println(F("Detected SVM30."));
  else {
    lcd.setBacklight(255, 0, 0);    // bright red
    lcd.clear();
    lcd.write("SVM30 probe");
    lcd.setCursor(0, 1);            // pos 50, line 1
    lcd.write("Error");
    Errorloop((char *)"could not detect SVM30 sensors");
  } 

  // read and display the ID
  read_id();

  // read SGP30 feature set
  read_featureSet();

  // set celsius or Fahrenheit
  svm.SetTempCelsius(LCDTEMPCELSIUS);
}

void loop() {
  
  // read measurement values
  read_values();
  
  // read SGP30 baseline
  read_baseline(); 
  
  // wait x seconds
  KeepTrigger(DELAY);
}

/*
 * @brief : keep triggering SGP30 while waiting
 * 
 * @param del : number of seconds to wait
 *
 * source datasheet SVM30:
 * The on-chip baseline compensation algorithm has been optimized 
 * for 1HZ sampling rate. The sensor shows best performance when 
 * used with this sampling rate.
 * 
 */
void KeepTrigger(uint8_t del)
{
  uint8_t w_seconds = del;
  unsigned long startMillis;
 
  if (w_seconds == 0) w_seconds = 1;
  
  while (w_seconds--)
  {
    startMillis = millis();
    
    if (! svm.TriggerSGP30())
      Errorloop((char *)"Error during trigger waiting");
      
    //capture button pressed  
    checkButton();
      
    // this gives 1Hz /1000ms (aboutisch)
    while(millis() - startMillis < 1000);
  }
}

/*
 * @brief : read and display the values from the SVM30
 * 
 */
void read_values() {

  if (! svm.GetValues(&v)) {
    lcd.setBacklight(255, 0, 0);    // bright red
    lcd.clear();
    lcd.write("Error during");
    lcd.setCursor(0, 1);            // pos 0, line 1
    lcd.write("reading");
    Errorloop((char *) "Error during reading values");
  }
  
  Serial.print(F("CO2 equivalent : "));
  Serial.print(v.CO2eq);

  Serial.print(F(", TVOC : "));
  Serial.print(v.TVOC);

  Serial.print(F(", H2_signal : "));
  Serial.print(v.H2_signal);

  Serial.print(F(", Ethanol_signal : "));
  Serial.print(v.Ethanol_signal);

  Serial.print(F(", Humidity : "));
  Serial.print((float) v.humidity/1000);

  Serial.print(F(", temperature : "));
  Serial.print((float) v.temperature/1000);

  Serial.print(F(", absolute humidity : "));
  Serial.print(v.absolute_hum);

  printLCD(true);
}

/*
 * @brief: read and display the id of the SGP30 and SHTC1 sensors
 */
void read_id() {
  
  uint16_t buf[3];    // SGP30 needs 3 words, SHTC1 is 1 word
  char  id[15];
  
  if ( ! svm.GetId(SGP30, buf))
      Errorloop((char *) "could not read SGP30 id");

  Serial.print(F("\nSGP30 id : "));
  sprintf(id, "%04x %04x %04x", buf[0], buf[1], buf[2]);
  Serial.println(id);

  if (svm.GetId(SHTC1, buf) == false)
      Errorloop((char *) "could not read SHTC1 id");

  Serial.print(F("SHTC1 id : "));
  // only bit 5:0 matter (source: datasheet)
  sprintf(id, "%04x", buf[0] & 0x3f);
  Serial.println(id);
}

/*
 * @brief: read and display the product feature set of the SGP30 sensor
 */
void read_featureSet(){

  char buf[2];
  
  if ( ! svm.GetFeatureSet(buf))
      Errorloop((char *) "could not read SGP30 feature set");

  Serial.print(F("\nSGP30 product type : "));
  Serial.print((buf[0] & 0xf0), HEX);
  
  Serial.print(F(", Product version : "));
  Serial.println(buf[1], HEX);
  Serial.println();
}

/*
 * @brief: read the baselines of the SGP30 sensor
 * see example3 for extended information about baselines
 */
void read_baseline(){

  uint16_t baseline;
  
  if (! svm.GetBaseLine_CO2(&baseline))
      Errorloop((char *)"could not read SGP30 CO2 baseline");

  Serial.print(F(" CO2 equivalent baseline : 0x"));
  Serial.print(baseline, HEX);

  if (! svm.GetBaseLine_TVOC(&baseline))
      Errorloop((char *)"could not read SGP30 TVOC baseline");
  
  Serial.print(F(", TVOC baseline : 0x"));
  Serial.println(baseline, HEX);
}

// checks for button pressed to set the LCD on and keep on for 
// LCDTIMEOUT seconds after button has been released.
// return true to turn on OR false to turn / stay off.
bool checkButton()
{
#if ONLYONBUTTON == true
  static unsigned long startTime = 0;
  
  // button pressed ?
  if (! digitalRead(BUTTONINPUT)){
    startTime = millis();
  }

  if (startTime > 0) {
    if (millis() - startTime < (LCDTIMEOUT*1000))  return true;
    else  startTime = 0;      // reset starttime
  }    

  return false;

#endif //ONLYONBUTTON
  return true;  
}

// initialize the LCD
void lcdinit()
{
  lcd.begin(LCDCON);

  lcd.createChar(0, ppm);     // create custom characters
  lcd.createChar(1, deg);

  lcdsetbackground();         // set background

#if ONLYONBUTTON == true
  pinMode(BUTTONINPUT,INPUT_PULLUP);
#endif
}

// set requested background color
void lcdsetbackground()
{
  
#if ONLYONLIMIT == true
  lcd.setBacklight(0, 0, 0);  // off
  return;
#endif

#if ONLYONBUTTON == true
  if(! checkButton()) {
    lcd.setBacklight(0, 0, 0);  // off
    return;
  }
#endif //ONLYONBUTTON

  switch(LCDBACKGROUNDCOLOR){
    
    case 2:   // red
      lcd.setBacklight(255, 0, 0); // bright red
      break;
    case 3:   // green
      lcd.setBacklight(0, 255, 0); // bright green
      break;
    case 4:   // blue
      lcd.setBacklight(0, 0, 255); // bright blue
      break;
    case 5:   // off
      lcd.setBacklight(0, 0, 0);
      break;
    case 1:   // white
    default:
      lcd.setBacklight(255, 255, 255); // bright white
  }
}

// print results on LCD
// @parameter dd : true is display new data else no-data indicator
void printLCD(bool dd)
{
  char buf[10];
  static bool limitWasSet = false;
  static bool MeasureInd = true;
  
  // change background on limit (if limit was set)
#if CO2LIMIT > 0 
  
  if (v.CO2eq > CO2LIMIT){
    // change once..
    if(! limitWasSet){
      lcd.setBacklight(255, 0, 0); // bright red
      limitWasSet = true;
    }
  }
  else if (limitWasSet){
    lcdsetbackground();           // reset to original request
    limitWasSet = false;
  }
#endif //CO2LIMIT
  
// only display if limit has been reached  
#if ONLYONLIMIT == true
  if(! limitWasSet) {
    lcd.clear();
    return;
  }
#endif //ONLYONLIMIT

// only display if button was pressed or limit has been reached
#if ONLYONBUTTON == true
  if(! checkButton() && ! limitWasSet) {
    lcd.clear();
    lcd.setBacklight(0, 0, 0);  // off
    return;
  }
#endif //ONLYONBUTTON

  // if no data available indicate with . and return
  if (!dd) {
    lcd.setCursor(15, 0);            // pos 15, line 0
    
    // display measurement indicator 
    if (MeasureInd)  lcd.write(".");
    else lcd.write(" ");
    
    MeasureInd = !MeasureInd;
    return;
  }
  
  // just in case next no-data, start display .
  MeasureInd = true;
  
  lcd.clear();
  lcd.write("Co2: Temp: Hum:");

  lcd.setCursor(0, 1);            // pos 0, line 1
  
  sprintf(buf,"%d",v.CO2eq);
  lcd.write(buf);
  lcd.writeChar(0);               // add custom ppm

  lcd.setCursor(5, 1);            // pos 5, line 1
  FromFloat(buf, (float) v.temperature/1000 ,1);
  lcd.write(buf);
  lcd.writeChar(1);               // add customer degree

  lcd.setCursor(11, 1);           // pos 11, line 1
  FromFloat(buf, (float) v.humidity/1000 ,1);
  lcd.write(buf);
  lcd.write('%');
}

// This is a workaround as sprintf on Artemis/Apollo3 is not recognizing %f (returns empty)
// based on source print.cpp/ printFloat
int FromFloat(char *buf, double number, uint8_t digits) 
{ 
  char t_buf[10];
  buf[0] = 0x0;
  
  if (isnan(number)) {
    strcpy(buf,"nan");
    return 3;
  }
  
  if (isinf(number)) {
    strcpy(buf,"inf");
    return 3;
  }

  if (number > 4294967040.0 || number <-4294967040.0) {
    strcpy(buf,"ovf");
    return 3;
  }
    
  // Handle negative numbers
  if (number < 0.0)
  {
     strcat(buf,"-");
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;

  sprintf(t_buf,"%ld", int_part);
  strcat(buf,t_buf);
  
  if (digits > 0) {
    
    // Print the decimal point, but only if there are digits beyond
    strcat(buf,".");  
  
    // Extract digits from the remainder one at a time
    while (digits-- > 0)
    {
      remainder *= 10.0;
      unsigned int toPrint = (unsigned int)(remainder);
      sprintf(t_buf,"%d", toPrint);
      strcat(buf,t_buf);
      remainder -= toPrint; 
    } 
  }

  return (int) strlen(buf);
}

/**
 *  @brief : continued loop after fatal error
 *  @param mess : message to display
 */
void Errorloop(char *mess)
{
  Serial.println(mess);
  Serial.println(F("Program on hold"));
  for(;;) delay(100000);
}
