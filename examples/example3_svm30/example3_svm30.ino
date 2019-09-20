/*  example3 update baseline and continued read information from SVM30
 *   
 *  By: paulvha@hotmail.com
 *  Date: September , 2019
 *  Version : 1.0
 *   
 *  Baseline handling on the SGP32 seems to be half documented and partly 
 *  documented in error.
 *  
 *  1. The order is of reading and sending is TVOC, CO2eq (the datasheet is
 *  mixing the order up for reading)
 *  
 *  2. When sending the baseline values, each word needs to be followed with a
 *  CRC (not defined in the datasheet)
 *  
 *  3. Setting a baseline value of zero on CO2 causes CO2 baseline to be the
 *  same as TVOC. Setting values of zero should be avoided (not documented)
 *  
 *  4. TVOC baseline can be set by sending only 1 word/baseline + CRC. (Not documented)
 *  
 *  5. When setting baseline during the first 15 seconds afer power-up/reset, update 
 *  baseline for CO2 first, then TVOC. Else setting will fail.
 *  
 *  6. After setting the baseline value, it will NOT change/ updated. Once it 
 *  is updated the Baseline stays what it is. (not documented)
 *  
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
 * connected to MEGA2560
 * 
 * SVM30              MEGA2560
 * 1. SDL  -----------   SDL
 * 2. GND  -----------   GND
 * 3. VCC  -----------   +5V
 * 4. SDA  -----------   SDA
 * 
 * connected to UNO (might cause low memory issues)
 * 
 * SVM30                UNO
 * 1. SDL  -----------   A5
 * 2. GND  -----------   GND
 * 3. VCC  -----------   +5V
 * 4. SDA  -----------   A4
 * 
 * 
 * connected to ESP32
 ******************************************************************************************
 * WARNING: THE SVM30 NEEDS BETWEEN 4.5V AND 5.5V (TYPICAL 5V). RUNNING ON 3v3 MAKES IT UNSTABLE.
 * THE ESP32 PINS CAN HANDLE UP TO 3v3 ONLY AND AS SUCH YOU USE A BI-DIRECTIONAL LEVEL CONVERTER. 
 * e.g. https://www.sparkfun.com/products/12009
 *****************************************************************************************
 * SVM30               ESP32
 * 1. SDL  ----lll-----  SCL (pin 22)
 * 2. GND  -----------   GND
 * 3. VCC  -----------   +5V/VUSB
 * 4. SDA  ----lll-----  SDA (pin 21)
 * 
 * Sparkfun ESP32
 *
 *    Make sure :
 *      - use the VUSB pin to connect to the level converter (HV) and VCC of the SVM30
 *      - use the 3V3 pin to connect to the level converter (LV)
 *      - To select the Sparkfun ESP32 thing board before compiling
 *      - connect GND, SDA, SCL lines to the level converter and SVM30
 *      - The serial monitor is NOT active (will cause upload errors)
 *      - Press GPIO 0 switch during connecting after compile to start upload to the board
 *      - Press reset on the ESP32 thing after the sketch is loaded
 * 
 *
 * connected to ESP8266 
 ******************************************************************************************
 * WARNING: THE SVM30 NEEDS BETWEEN 4.5V AND 5.5V (TYPICAL 5V). RUNNING ON 3v3 MAKES IT UNSTABLE.
 * THE ESP8266 PINS CAN HANDLE UP TO 3v3 ONLY AND AS SUCH YOU USE A BI-DIRECTIONAL LEVEL CONVERTER. 
 * e.g. https://www.sparkfun.com/products/12009
 ***************************************************************************************** 
 * SVM30              ESP8266
 * 1. SDL  ----lll----   SDL
 * 2. GND  -----------   GND
 * 3. VCC  -----------   VIN
 * 4. SDA  ----lll----   SDA
 *
 * Sparkfun ESP8266 thing
 *
 *    Make sure :
 *      - connect with USB/Serial converter to serial connector for communication with PC (set to provide 3V3)
 *      - connect an EXTRA USB cable to the USB connector for providing 5V
 *      - use the VIN pin to connect to the level converter (HV) and VCC of the SVM30
 *      - use the 3V3 pin to connect to the level converter (LV)
 *      - connect GND, SDA, SCL lines to the level converter and SVM30
 *      - To select the Sparkfun ESP8266 thing board before compiling
 *      - The serial monitor is NOT active (will cause upload errors)
 *      - close the on-board DTR link during connecting after compile to start upload to the board
 *
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

/////////////////////////////////////////////////////////////
/* define new baselines
 * 
 *//////////////////////////////////////////////////////////////
#define BASE_CO2  0x8F8A
#define BASE_TVOC 0x90B3

///////////////////////////////////////////////////////////////
/////////// NO CHANGES BEYOND THIS POINT NEEDED ///////////////
///////////////////////////////////////////////////////////////

// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void read_id();
void read_featureSet();
void Errorloop(char *mess);
void read_values();
void read_baseline(bool ind);
void update_baseline(uint16_t base_co2, uint16_t base_TVOC, bool ind );
void KeepTrigger(uint8_t del);

#include <svm30.h>

// create instance
SVM30 svm;

// store values
struct svm_values v;

void setup() {
  
  Serial.begin(115200);

  Serial.println(F("Hi there, this is example 3.\nUpdate of the baseline while reading information from the SVM30"));
  
  // enable debug messages
  svm.EnableDebugging(DEBUG);
  
  svm.begin();

  Serial.print(F("Driver version : "));
  Serial.println(svm.GetDriverVersion());
    
  // try to detect SVM30 sensors
  if (svm.probe() == false) Errorloop("could not detect SVM30 sensors");
  else Serial.println(F("SVM30 detected"));

  // read and display the ID
  read_id();

  // read SGP30 feature set
  read_featureSet();
}

void loop() {

  static bool OnlyOnce = true;
  
  // read measurement values
  read_values();

  // read SGP30 baseline (see remark at read_baseline about true/false)
  read_baseline(true);

  // update one time
  if (OnlyOnce){
    update_baseline(BASE_CO2, BASE_TVOC, false);
    OnlyOnce = false;
  }
   
  // wait x seconds
  KeepTrigger(DELAY);
}

/*
 * @brief : update baseline values
 * 
 * @param ind :
 *  true : use individual baseline calls to set
 *  false : set with one call the baselines
 *  
 *  The 'ind' parameter was only added to show the example
 */

void update_baseline(uint16_t base_co2, uint16_t base_TVOC, bool ind ){
 
  uint32_t base32;
  
  if (ind){
    
    //Serial.println("set individual baseline");
    
    if (base_co2 != 0) {
      if (! svm.SetBaseLine_CO2(base_co2))
          Errorloop("Error during updating baseline CO2");
      } 
    else
      Serial.println("Skipping update baseline CO2 equivalent");
  
    if (base_TVOC != 0) {
      if (! svm.SetBaseLine_TVOC(base_TVOC))
          Errorloop("Error during updating baseline TVOC");
      }
    else
      Serial.println("Skipping update baseline TVOC");
  }
  else {
    
    //Serial.println("set baselines in one call");
    
    if (base_co2 == 0 || base_TVOC == 0 ) {
      Serial.println("Skipping update baselines");
      return;
    }
    
    base32 = base_TVOC << 16 | base_co2;
    
    if (! svm.SetBaseLines(base32))
        Errorloop("Error during updating baselines");
  } 
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
      Errorloop("Error during trigger waiting");
      
    // this gives 1Hz /1000ms (aboutisch)
    while(millis() - startMillis < 1000);
  }
}

/*
 * @brief : read and display the values from the SVM30
 * 
 */
void read_values() {

  if (! svm.GetValues(&v))
      Errorloop("Error during reading values");

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
}

/*
 * @brief: read and display the id of the SGP30 and SHTC1 sensors
 */
void read_id() {
  
  uint16_t buf[3];    // SGP30 needs 3 words, SHTC1 is 1 word
  char  id[15];
  
  if ( ! svm.GetId(SGP30, buf))
      Errorloop("could not read SGP30 id");

  Serial.print(F("\nSGP30 id : "));
  sprintf(id, "%04x %04x %04x", buf[0], buf[1], buf[2]);
  Serial.println(id);

  if (svm.GetId(SHTC1, buf) == false)
      Errorloop("could not read SHTC1 id");

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
      Errorloop("could not read SGP30 feature set");

  Serial.print(F("\nSGP30 product type : "));
  Serial.print((buf[0] & 0xf0), HEX);
  
  Serial.print(F(", Product version : "));
  Serial.println(buf[1], HEX);
  Serial.println();
}

/*
 * @brief: read the baselines of the SGP30 sensor
 * 
 * @param ind :
 *  true : obtain with individual calls
 *  false : obtain with one call
 *  
 *  The 'ind' parameter was only added to show the example
 */
void read_baseline(bool ind){

  uint16_t baseline;
  uint32_t base32;
  
  if(ind) {
    
    // Serial.println("read individual");
    
    if (! svm.GetBaseLine_CO2(&baseline))
        Errorloop("could not read SGP30 CO2 baseline");
  
    Serial.print(F(" CO2 equivalent baseline : 0x"));
    Serial.print(baseline, HEX);
  
    if (! svm.GetBaseLine_TVOC(&baseline))
        Errorloop("could not read SGP30 TVOC baseline");
    
    Serial.print(F(", TVOC baseline : 0x"));
    Serial.println(baseline, HEX);
  }
  
  else {
    
    //Serial.println("read with one call");
    
    if (! svm.GetBaseLines(&base32))
        Errorloop("could not read SGP30 baselines");
  
    Serial.print(F(" CO2 equivalent baseline : 0x"));
    baseline = base32 & 0xffff;
    Serial.print(baseline, HEX);
    
    Serial.print(F(", TVOC baseline : 0x"));
    baseline = base32 >> 16;
    Serial.println(baseline, HEX);
  }
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
