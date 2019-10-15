/************************************************************************************
 *  Copyright (c) October 2019, version 1.0     Paul van Haastrecht
 *
 *  =========================  Highlevel description ================================
 *
 *  This basic reading example sketch to connect an SVM30 for providing data, and
 *  sent over LORA radio to TheThingsNetwork (TTN).
 *
 *  This sketch was developed and tested on Feather Lora 32U4.
 *
 *  =========================  Hardware connections =================================
 *
 *  FOR DETAILED INFORMATION SEE THE SVM30.odt IN THE EXTRAS DIRECTORY
 *
 *  SVM30              Feather 32U4
 *  1. SCL  ---------- SCL
 *  2. GND  ---------- GND
 *  3. VCC  ---------  VUSB
 *  4. SDA  ---------- SDA
 *
 *  ================================= PARAMETERS =====================================
 *
 *  From line 88 there are optional and MANDATORY configuration parameters for the program
 *
 *  ================================= DATA FORMAT =====================================
 *
 * In do_send() the info that is sent to TTN is defined as follows:
 *
 * buffer[0]  = MSB SVM_id
 * buffer[1]  = LSB SVM_id
 * buffer[2]  = MSB temp_int
 * buffer[3]  = LSB temp_int
 * buffer[4]  = MSB rh_int
 * buffer[5]  = LSB rh_int
 * buffer[6]  = MSB CO2_Avg_int
 * buffer[7]  = LSB CO2_Avg_int
 * buffer[8]  = MSB TVOC_Avg_int
 * buffer[9]  = LSB TVOC_Avg_int
 *
 * SVM_ID is the last 4 digits of the SGP30 serial number that is on the SVM30
 * temp_int is last read temperature * 100 ( 22.74 C => 2274)
 * rh_int is last read humidity * 100 ( 49.42% => 4942)
 * CO2_Avg_int is average reading of the number of CO2 samples taken between trigger on the event() from LMIC (defined by TX_INTERVAL)
 * TVOC_Avg_int is average reading of the number of TVOC samples taken between trigger on the event() from LMIC (defined by TX_INTERVAL)
 *
 * On TTN console you will see HEX numbers, e.g.:
 *  9E5608E2134E01CA0009
 *
 *  0x9E56  SVM_ID
 *  0x08E2  temperature ( = 2274 decimal => 22.74 C)
 *  0x134E  humidity    ( = 4942 decimal => 49.42%)
 *  0x01CA  CO2_AVg     ( = 458)
 *  0x0009  TVOC_AVG    ( = 9)
 *
 *  ================================== SOFTWARE ======================================
 *  The following libraries are necessary to be installed:
 *  LMIC  : https://github.com/matthijskooijman/arduino-lmic
 *  SVM30 : https://github.com/paulvha/svm30
 *
 *  ================================ Disclaimer ======================================
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *                            *************************
 *
 *  The lMIC code is based on ttn_ABP sketch from Thomas Telkamp and Matthijs Kooijman
 *
 *  Permission is hereby granted, free of charge, to anyone
 *  obtaining a copy of this document and accompanying files,
 *  to do whatever they want with them without any restriction,
 *  including, but not limited to, copying, modification and redistribution.
 *  NO WARRANTY OF ANY KIND IS PROVIDED.
 *  ===================================================================================
 *
 *  NO support, delivered as is, have fun, good luck !!
 */

#include <lmic.h>
#include <hal/hal.h>          // needed for lmic_pins
#include <svm30.h>

/////////////////////////////////////////////////////////////
/* define SVM30 driver debug
 * 0 : no messages
 * 1 : request sending and receiving
 *//////////////////////////////////////////////////////////////
#define SVM30_DEBUG 0

// Set to 1 for sketch debug / result information
#define SKETCH_DEBUG 0

// Schedule TX every this many seconds (Is not precize and might become longer due to duty cycle).
// 30 seconds should be the minimum not to overload communication band usage.
#define TX_INTERVAL  40

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//////////////////////////// LMIC parameter setting //////////////////////////

// LoRaWAN NwkSKey, network session key from TTN (use the same format as this example)
static const PROGMEM u1_t NWKSKEY[16] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

// LoRaWAN AppSKey, application session key From TTN (use the same format as this example)
static const u1_t PROGMEM APPSKEY[16] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

// LoRaWAN end-device address (DevAddr) from TTN (use the same format as this example)
static const u4_t DEVADDR = 0x03FF0001;

// Pin mapping 
// WARNING !!!!  WARNING !!!! WARNING !!!! WARNING !!!! WARNING !!!! WARNING !!!! WARNING !!!! WARNING !!!! WARNING !!!! WARNING !!!!
// make sure to connect IO1 (next to TX) to pin 6 (physical pin 2, second pin from SCL pin) !!!!

const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {7, 6, LMIC_UNUSED_PIN},
};

///////////////////////////////////////////////////////////////
/////////// NO CHANGES BEYOND THIS POINT NEEDED ///////////////
///////////////////////////////////////////////////////////////

// create instance
SVM30 svm;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;     // needed in do_send

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
////////////////////////////// PROGRAM VARIABLES ////////////////////////////

float SVM_Temp = 0.0;       // store temperature
float SVM_Humi = 0.0;       // store humidity

unsigned int SVM_id = 0;    // SVM30 ID to include in message to TTN

unsigned int Counter_Pm = 0;// number of samples in total
float fCO2_Total = 0.0, fTVOC_Total = 0.0;

int16_t temp_int,rh_int;    // store data to sent
int16_t CO2_Avg_int, TVOC_Avg_int;

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/**
 * @brief: call back from LMIC library
 */
void onEvent (ev_t ev) {

  //Serial.println("onEvent ");

  Serial.print(os_getTime());
  Serial.print(":");

  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println("EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      Serial.println("EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      Serial.println("EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println("EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      Serial.println("EV_JOINING");
      break;
    case EV_JOINED:
      Serial.println("EV_JOINED");
      break;
    case EV_RFU1:
      Serial.println("EV_RFU1");
      break;
    case EV_JOIN_FAILED:
      Serial.println("EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      Serial.println("EV_REJOIN_FAILED");
      break;
    case EV_TXCOMPLETE:

      if (SKETCH_DEBUG) {
        Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
        if (LMIC.txrxFlags & TXRX_ACK)
          Serial.println("Received ack");
        if (LMIC.dataLen) {
          Serial.println("Received ");
          Serial.println(LMIC.dataLen);
          Serial.println(" bytes of payload");
        }
      }

      // Schedule next transmission, this does not mean it is handled right away
      // So calculate now and sent this later with do_send()!!!
      calculate_data();
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println("EV_LOST_TSYNC");
      break;
    case EV_RESET:
      Serial.println("EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println("EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      Serial.println("EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      Serial.println("EV_LINK_ALIVE");
      break;
    default:
      Serial.println("Unknown event");
      break;
  }
}

/**
 * @brief : calculate the results to be sent later
 */
void calculate_data()
{
  temp_int = SVM_Temp * 100;        // 24.43 becomes 2443
  rh_int = SVM_Humi * 100 ;         // 43.50 becomes 4350

  CO2_Avg_int =  fCO2_Total / Counter_Pm;
  TVOC_Avg_int = fTVOC_Total / Counter_Pm;

  // Zero Counters
  fCO2_Total = fTVOC_Total = 0;
  Counter_Pm = 0;
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
/**
 * @brief : read data from SVM30 and calculate totals
 */

void ProcessData()
{
  struct svm_values v;

  if (! svm.GetValues(&v))
      Errorloop("Error during reading values");

  fCO2_Total += (float)  v.CO2eq;
  fTVOC_Total += (float) v.TVOC;
  SVM_Humi = (float) v.humidity/1000;
  SVM_Temp = (float) v.temperature/1000;
  Counter_Pm++;

  if (SKETCH_DEBUG) {
      Serial.print("SVM_id; ");
      Serial.println(SVM_id, HEX);
      disp_info("\nCounter_Pm:", &Counter_Pm, false);
      disp_info("CO2 Total  :", &fCO2_Total, true);
      disp_info("TVOC Total:",&fTVOC_Total, true);
      disp_info("CO2_Sensor  :", &v.CO2eq, false);
      disp_info("TVOC_Sensor:",&v.TVOC, false);
      disp_info("Temperature:", &SVM_Temp, true);
      disp_info("Humidity:", &SVM_Humi, true);
      Serial.println();
  }
}

/**
 * @brief : prepare sent buffer and send it
 */
void do_send(osjob_t* j) {

  uint8_t buffer[15];

  //Serial.println(F("ENTER do_send"));

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    return;
  }

  // Prepare upstream data transmission at the next possible time.
  buffer[0] = SVM_id >> 8;
  buffer[1] = SVM_id;
  buffer[2] = temp_int >> 8;
  buffer[3] = temp_int;
  buffer[4] = rh_int >> 8;
  buffer[5] = rh_int;
  buffer[6] = CO2_Avg_int >> 8;
  buffer[7] = CO2_Avg_int;
  buffer[8] = TVOC_Avg_int >> 8;
  buffer[9] = TVOC_Avg_int;

  LMIC_setTxData2(1, buffer, 10, 0);

  if (SKETCH_DEBUG) {
    Serial.print("\t\tPacket queued\nSVM_ID: ");
    Serial.println(SVM_id, HEX);
    disp_info("Temp:\t", &temp_int, false);
    disp_info("RH:\t", &rh_int, false);
    disp_info("CO2_Avg:",&CO2_Avg_int, false);
    disp_info("TVOC_Avg:",&TVOC_Avg_int, false);
  }

  // Next TX is scheduled after TX_COMPLETE event.
}

/**
 *  @brief : display information to reduce the number of Serial.print() for memory reasons
 *  @param m : message to display
 *  @param *val : pointer to value to display
 *  @type : if true display val as float, else display as uint16_t
 *
 */
void disp_info(char * m, void *val, bool type)
{
   Serial.print(m);
   Serial.print("\t");
   if(type) {
    float ft = *(float *) val;
    Serial.println(ft);
   }
   else {
    uint16_t t = *(uint16_t *) val;
    Serial.print(t);
    Serial.print("\t0x");           // Hex output to compare against TTN
    Serial.println(t, HEX);         // console info
  }
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

void setup() {

  Serial.begin(115200);

  /* ADDED WHILE(!Serial) TO WAIT FOR SERIAL CONSOLE TO BE AVAILABLE AND
   * DISPLAY THE COMPLETE INFORMATION FROM START. IT ALSO HELPED DURING UPLOAD
   *
   * IF YOU ARE APPLY POWER TO THE VUSB WITHOUT CONNECTING A
   * SERIAL MONITOR, COMMENT THIS LINE OUT OTHERWISE THE SKETCH
   * DOES NOT GO BEYOND THIS POINT.
   */

  while(!Serial);

  Serial.println(F("Hi there, this is example 6.\nReading information from the SVM30 and sending to TTN"));

///////////////////// SVM30 init /////////////////////////////

  // enable driver debug messages
  svm.EnableDebugging(SVM30_DEBUG);

  Serial.print("connecting to SVM30: ");

  svm.begin();

  Serial.print(F("Driver version : "));
  Serial.println(svm.GetDriverVersion());

  // try to detect SVM30 sensors
  if ( ! svm.probe() ) Errorloop("could not detect SVM30 sensors");
  else Serial.println(F("SVM30 detected"));

  // read device serial number
  GetDeviceInfo();

///////////////////// LMIC init /////////////////////////////

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // this VERIABLE is defined in config.h
  // *************************************************************
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF12, 14);

  // Start job
  do_send(&sendjob);
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

void loop() {

  delay(1000);       // This is a one sec delay.

  ProcessData();     // read from SVM30 the calculate totals

  os_runloop_once(); // trigger LMIC library
}

/**
 * @brief : read serial number SVM30 / SGP30
 */
void GetDeviceInfo()
{
  uint16_t buf[3];    // SGP30 needs 3 words
  char id[20];

  if ( ! svm.GetId(SGP30, buf))
      Errorloop("could not read SGP30 id");

  if(SKETCH_DEBUG) {
    Serial.print(F("\nSVM30 / SGP30 id : "));
    sprintf(id, "%04x %04x %04x", buf[0], buf[1], buf[2]);
    Serial.println(id);
  }

  // use last 4 digits
  SVM_id = buf[2];
}

/**
 *  @brief : continued loop after fatal error
 *  @param mess : message to display
 */
void Errorloop(char *mess)
{
  Serial.println(mess);
  Serial.println("Program on hold");
  for(;;) delay(100000);
}
