/**
 * SVM30 Library Header file
 *
 * Copyright (c) September 2019, Paul van Haastrecht
 *
 * SVM30 is a sensor from Sensirion AG.
 * All rights reserved.
 *
 * Development environment specifics:
 * Arduino IDE 1.9
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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **********************************************************************
 * Version 1.0   / September 2019
 * - Initial version by paulvha
 *
 *********************************************************************
 */

#include "svm30.h"
const char * SVM30_VERSION = VERSION;

/**
 * @brief constructor and initialize variables
 */
SVM30::SVM30(void) {
  _Send_BUF_Length = 0;
  _Receive_BUF_Length = 0;
  _SVM30_Debug = false;
  _started = false;
  _wait = DEFAULT_WAIT;        // wait time after sending command
}

/**
 * @brief start SGP30 (if not started already)
 *
 * @return
 *   true on success else false
 */

bool SVM30::StartSGP30() {

    if (! _started) {

        PrepSendBuffer(SGP30_ADDRESS, SGP30_Init_Air_Quality);

        // send request (no response expected)
        if (SendToSVM() != ERR_OK) {
            if (_SVM30_Debug) printf("Error during requesting init Air Quality\n");
            return(false);
        }

        _started = true;
    }

    return(true);
}

/**
 * @brief : Initialize the communication
 *
 * @return :
 *   true on success else false
 */
bool SVM30::begin() {
    Wire.begin();

    if (! reset(SGP30)) return(false);

    // start SGP30
    return(StartSGP30());
}

/**
 * @brief : Enable or disable the printing of debug messages.
 *
 * @param act :
 *  false : no debug messages
 *  true : debug messages
 */
void SVM30::EnableDebugging(bool act) {
    _SVM30_Debug = act;
}

/**
 * @brief : return driver version.
 *
 * @return : driver information string
 */
const char * SVM30::GetDriverVersion() {
    return (SVM30_VERSION);
}

/**
 * @brief : reset SGP30 or SHTC1
 *
 * @param device : I2C address of device
 *
 * @return :
 *   true on success else false
 */
bool SVM30::reset(uint8_t device) {
    if (device == SGP30_ADDRESS){
        PrepSendBuffer(RESET_ADDRESS, RESET_CMD);
        if (_SVM30_Debug) printf("WARNING: reset ALL devices on I2C\n");
    }

    else if (device == SHTC1_ADDRESS)
        PrepSendBuffer(SHTC1_ADDRESS, SHTC1_Reset);

    else
        return(false);

    // send Request to sensor(s)
    if (SendToSVM() != ERR_OK) {
        if (_SVM30_Debug) printf("Error during reset\n");
        return(false);
    }

    // give time to settle reset
    delay(500);

    _started = false;

    return(true);
}

/**
 * @brief : read feature set from SGP30
 *
 * @param buf : buffer to store feature set number (min 2)
 *
 * @return :
 *   true on success else false
 */
bool SVM30::GetFeatureSet(char *buf) {
    PrepSendBuffer(SGP30_ADDRESS, SGP30_Get_Feature_Set);

    // send Request and read from sensor
    if (RequestFromSVM(2) != ERR_OK) {
        if (_SVM30_Debug) printf("Error during reading Feature set\n");
        return(false);
    }

    // copy feature set
    buf[0] = _Receive_BUF[0];
    buf[1] = _Receive_BUF[1];

    return(true);
}

/**
 * @brief : Measure test on SGP30
 *
 * @return :
 *   true on success else false
 */
bool SVM30::MeasureTest() {

    bool restart = false;

    if (_started) {
       if (! reset(SGP30))  return(false);
       restart = true;
    }

    PrepSendBuffer(SGP30_ADDRESS, SGP30_Measure_Test);

    // set for longer wait time
    _wait = MEASURE_WAIT;

    // send Request and read from sensor
    if (RequestFromSVM(2) != ERR_OK) {
        if (_SVM30_Debug) printf("Error during measurement test\n");
        return(false);
    }

    // set for default wait time
    _wait = DEFAULT_WAIT;

    // check the return code
    if (_Receive_BUF[0] != (SGP30_TestOK >> 8 & 0xff) || _Receive_BUF[1] != (SGP30_TestOK & 0xff) ) {
        if (_SVM30_Debug) printf("Error in measurement test return code\n");
        return(false);
    }

    if (restart)  return(StartSGP30());

    return(true);
}

/**
 * @brief : get BOTH baselines (TVOC and CO2)
 *
 * @param : pointer to baseline to store results
 *
 * @return :
 *   true on success else false
 */
bool SVM30::GetBaseLines(uint32_t *baseline) {
    uint16_t base;

    // first get TVOC
    if ( ! GetBaseLine(&base, true)) return(false);
    *baseline = base << 16;

    // get CO2
    if ( ! GetBaseLine(&base, false)) return(false);
    *baseline = *baseline | base;

    return(true);
}

/**
 * @brief read baseline from SGP30
 *
 * @param baseline : store baseline
 *
 * @param tvoc :
 *      true get tvoc baseline
 *      false get CO2eq baseline
 *
 * source : datasheet
 * The sensor responds with 2 data bytes (MSB first) and 1 CRC byte
 * for each of the two values in the order CO 2 eq and TVOC.
 *
 * REMARK: The order is TVOC first then CO2 (seen in sample code)
 *
 * @return :
 *   true on success else false
 */
bool SVM30::GetBaseLine(uint16_t *baseline , bool tvoc) {
    PrepSendBuffer(SGP30_ADDRESS, SGP30_Get_Baseline);

    // send Request and read from sensor
    if (RequestFromSVM(4) != ERR_OK) {
        if (_SVM30_Debug) printf("Error during reading baseline\n");
        return(false);
    }

    // copy baseline
    if (tvoc)   *baseline = byte_to_uint16(0);
    else *baseline = byte_to_uint16(2);

    return(true);
}

/**
 * @brief : set BOTH baselines (TVOC and CO2)
 *
 * @param baseline
 *
 * @return :
 *   true on success else false
 */
bool SVM30::SetBaseLines(uint32_t baseline) {
    uint16_t base;

    // first set CO2
    base = baseline & 0xffff;
    if ( ! SetBaseLine(base, false)) return(false);

    // set TVOC
    base = (baseline >> 16) & 0xffff;
    if ( ! SetBaseLine(base, true)) return(false);

    return(true);
}

/**
 * @brief set baseline on SGP30
 *
 * @param baseline: baseline to set
 *
 * @param tvoc :
 *      true set tvoc baseline
 *      false set CO2eq baseline
 *
 * source datasheet:
 * After a power-up or soft reset, the baseline of the baseline compensation
 * algorithm can be restored by sending first an “Init_air_quality” command followed
 * by a “Set_baseline” command with the two baseline values as parameters
 * in the order as (TVOC, CO 2 eq)
 *
 * WARNING: The datasheet is NOT completly correct, as a CRC has to
 * be added after each baseline.
 *
 * @return :
 *   true on success else false
 */
bool SVM30::SetBaseLine(uint16_t baseline, bool tvoc) {
    char    data[4];
    uint16_t base, len;

    /* Setting a baseline of 0x0000 on CO2, will result in CO2
     * being set the same as TVOC. Setting TVOC to 0x0000 is ignored
     * by the SGP30
     *
     * Hence a baseline of 0x0000 will be treated as an error */
     if (baseline == 0x0) {
         if (_SVM30_Debug) printf("Error during setting baseline. Baseline can NOT be zero\n");
         return(false);
     }

    if (tvoc) {
        data[0] = (baseline >> 8) & 0xff;// MSB  update TVOC
        data[1] = baseline & 0xff;       // LSB
        len = 2;
    }
    else {  // CO2

        // first read current baseline TVOC
        if ( !GetBaseLine_TVOC(&base)) return(false);

        data[0] = (base >> 8) & 0xff;    // MSB keep TVOC
        data[1] = base & 0xff;           // LSB
        data[2] = (baseline >> 8) & 0xff;// MSB update Co2
        data[3] = baseline & 0xff;       // LSB
        len = 4;
    }

    PrepSendBuffer(SGP30_ADDRESS, SGP30_Set_Baseline, data, len);

    // send Request to sensor
    if (SendToSVM() != ERR_OK) {
        if (_SVM30_Debug) printf("Error during setting baseline\n");
        return(false);
    }

    return(true);
}

/**
 * @brief set humidity on SGP30
 *
 * @param humidity: absolute humidity value to set
 *
 * @return :
 *   true on success else false
 */
bool SVM30::SetHumidity(float humidity) {
    char    data[2];

    if (humidity > 256000 || humidity < 0) {
        if (_SVM30_Debug) printf("Invalid humidity\n");
        return (false);
    }

    // convert to 8.8 fixed point
    uint16_t ConvHum = ConvAbsolute(humidity);

    data[0] = (ConvHum >> 8) & 0xff; //MSB
    data[1] = ConvHum & 0xff;        //LSB

    PrepSendBuffer(SGP30_ADDRESS, SGP30_Set_Humidity, data, 2);

    // send Request to sensor
    if (SendToSVM() != ERR_OK) {
        if (_SVM30_Debug) printf("Error during setting humidity\n");
        return(false);
    }

    return(true);
}

/**
 * @brief read ID number from SGP30 or SHTC1
 *
 * @param device : I2C address of device (SGP30 or SHTC1)
 * @param buf : buffer to store ID (SGP30 : 3 words, SHTC1 : 1 word)
 *
 * @return :
 *   true on success else false
 */
bool SVM30::GetId(uint8_t device, uint16_t *buf) {
    uint8_t i = 0, j = 0, len;

    if (device == SGP30_ADDRESS) {

        /* The get serial ID command returns 3 words (6 bytes) and
         * every word (2 bytes) is followed by an 8-bit CRC checksum.
         * Together the 3 words constitute a unique serial ID with a length of 48 bits.*/

        len = 6;
        PrepSendBuffer(SGP30_ADDRESS, SGP30_Read_ID);
    }

    else if (device == SHTC1_ADDRESS) {

       /* After the SHTC1 has acknowledged the proper reception of
        * the command, the master can send an I2C read header and
        * the SHTC1 will submit the 16-bit ID followed by 8 bits of CRC.
        * REMARK : only bit 5:0 are valid for SHTC1 ID (source: datasheet)*/

        len = 2;
        PrepSendBuffer(SHTC1_ADDRESS, SHTC1_Read_ID);
    }

    else
        return(false);

    // send Request and read from sensor
    if (RequestFromSVM(len)){
        if (_SVM30_Debug) printf("Error during get ID\n");
        return(false);
    }

    // copy received ID number
    for (i = 0; i < len; i += 2)  buf[j++] = byte_to_uint16(i);

    return(true);
}

/**
 * @brief : check if SVM30 sensors are available (read ID)
 *
 * @return :
 *   true on success else false
 */
bool SVM30::probe() {
    uint16_t buf[3];        // SGP30 has 3 words, SHTC1 has 1 word

    if (GetId(SGP30_ADDRESS, buf) != true){
         if (_SVM30_Debug) printf("Error during probe SGP30 at address 0x%x\n", SGP30_ADDRESS );
         return(false);
    }

    if (GetId(SHTC1_ADDRESS, buf) != true) {
        if (_SVM30_Debug) printf("Error during probe SHTC1 at address 0x%x\n", SHTC1_ADDRESS);
        return(false);
    }

    return(true);
}

/**
 * @brief : Trigger a read on the SGP30
 *
 * The on-chip baseline compensation algorithm has been optimized
 * for 1HZ sampling rate. The sensor shows best performance when
 * used with this sampling rate.
 *
 * Sample rate to be implemented in the sketch (see examples)
 *
 * @return :
 *   true on success else false
 */
bool SVM30::TriggerSGP30() {
    // SGP30 measurement started already?
    if ( ! _started)
        if (! StartSGP30()) return(false);

    // get TVOC and CO2 equivalent data
    PrepSendBuffer(SGP30_ADDRESS, SGP30_Measure_Air_Quality);

    // send Request and read from sensor
    if (RequestFromSVM(4) != ERR_OK) {
        if (_SVM30_Debug) printf("Error during reading TVOC and CO2\n");
        return(false);
    }

    return(true);
}

/**
 * @brief : read all measurement values from the sensor and store in structure
 * @param v: pointer to structure to store
 *
 * @return :
 *   true on success else false
 */
bool SVM30::GetValues(struct svm_values *v) {
    memset(v,0x0,sizeof(struct svm_values));

    /** data from SGP30  */
    if (TriggerSGP30() == false) return(false);

    v->CO2eq = byte_to_uint16(0);
    v->TVOC  = byte_to_uint16(2);

    // get raw H2 signal and Ethanol signal
    PrepSendBuffer(SGP30_ADDRESS, SGP30_Measure_Raw_Signals);

    // send Request and read from sensor
    if (RequestFromSVM(4) != ERR_OK) {
        if (_SVM30_Debug) printf("Error during reading Raw signals\n");
        return(false);
    }

    v->H2_signal = byte_to_uint16(0);
    v->Ethanol_signal  = byte_to_uint16(2);

    /** data from SHTC1 */
    PrepSendBuffer(SHTC1_ADDRESS, SHTC1_Read_Temp_First);

    // send Request and read from sensor
    if (RequestFromSVM(4) != ERR_OK) {
        if (_SVM30_Debug) printf("Error during reading SHTC1\n");
        return(false);
    }

    // get the raw values from the SHTC
    v->r_temperature = byte_to_uint16(0);
    v->r_humidity  = byte_to_uint16(2);

    // convert to useable temperature and humidity
    shtc1_conv(&v->temperature, &v->humidity, v->r_temperature, v->r_humidity);

    // calculate absolute humidity
    v->absolute_hum = calc_absolute_humidity(v->temperature, v->humidity);

    return(true);
}

/**
 * @brief : translate 2 bytes to uint16
 * @param x : offset in _Receive_BUF
 *
 * assumed is MSB first, LSB second byte
 *
 * @return : uint16_t number
 */
uint16_t SVM30::byte_to_uint16(int x) {
    uint16_t val;
    val =  _Receive_BUF[x] << 8 | _Receive_BUF[x+1];
    return val;
}

/**
 * @brief : Fill buffer to send over I2C communication
 * @param device : I2C address to use (either SGP30 or SHTC1)
 * @param cmd: I2C commmand for device
 * @param param : additional parameters to add
 * @param len : length of parameters to add (zero if none)
 *
 */
void SVM30::PrepSendBuffer(uint8_t device, uint16_t cmd, char *param, uint8_t len) {
    uint8_t     i = 0, j, c;

    _I2C_address = device;

    // add command
    _Send_BUF[i++] = cmd >> 8 & 0xff;   //0 MSB
    _Send_BUF[i++] = cmd & 0xff;        //1 LSB

    // additional parameters to add?
    if (len != 0) {

        for (j = 0, c = 0 ; j < len; j++) {

            // add bytes
            _Send_BUF[i++] = param[j];

            // add CRC after each 2 bytes
            if(++c == 2){
                _Send_BUF[i] = CalcCrC(&_Send_BUF[i - 2]);
                i++;
                c = 0;
            }
        }
    }

    _Send_BUF_Length = i;
}

/**
 * @brief : send a prepared command (with PrepsendBuffer())
 *
 * @return :
 * Ok ERR_OK
 * else error
 */
uint8_t SVM30::SendToSVM() {
    if (_Send_BUF_Length == 0) return(ERR_DATALENGTH);

    if (_SVM30_Debug) {
        printf("Sending to 0x%x: ",_I2C_address );
        for(byte i = 0; i < _Send_BUF_Length; i++)
            printf("0x%02X ", _Send_BUF[i]);
    }

    Wire.beginTransmission(_I2C_address);
    Wire.write(_Send_BUF, _Send_BUF_Length);
    Wire.endTransmission();

    _Send_BUF_Length = 0;

    // give time to settle
    delay(_wait);

    return(ERR_OK);
}

/**
 * @brief : sent command/request and read from SVM30 sensor
 * @param cnt: number of data bytes to get
 *
 * @return :
 * OK   ERR_OK
 * else error
 */
uint8_t SVM30::RequestFromSVM(uint8_t cnt) {
    uint8_t ret;

#if defined STABIILITY  // see SVM30.h
    uint8_t retry = RETR_CNT;
RETRY:
#endif

    // sent Request
    ret = SendToSVM();
    if (ret != ERR_OK) {
        if (_SVM30_Debug) printf("Can not sent request\n");
        return(ret);
    }

    // read from Sensor
    ret = ReadFromSVM(cnt);

    if (ret != ERR_OK) {

#if defined STABIILITY // see SVM30.h
        // Optional retry mechanism to improve stability
        if (--retry > 0) {
           // printf("R\  N");

            if (_I2C_address == SGP30_ADDRESS) reset(SGP30);
            else reset(SHTC1);

            goto RETRY;
        }
#endif
        if (_SVM30_Debug)  printf("Error during reading. Errorcode: 0x%02X\n", ret);
    }

    if (_SVM30_Debug){
       printf(", Received: ");
       for(byte i = 0; i < _Receive_BUF_Length; i++) printf("0x%02X ",_Receive_BUF[i]);
       printf("length: %d\n",_Receive_BUF_Length);
    }

    return(ret);
}

/**
 * @brief       : receive from Sensor
 * @param count : number of data bytes to read
 *
 * @return :
 * OK   ERR_OK
 * else error
 */
uint8_t SVM30::ReadFromSVM(uint8_t count) {
    uint8_t data[3];
    uint8_t i, j;

    j = i = _Receive_BUF_Length = 0;

    // 2 data bytes  + crc
    Wire.requestFrom(_I2C_address, uint8_t (count / 2 * 3));

    delay(5);

    while (Wire.available()) { // wait till all arrive

        data[i++] = Wire.read();

        // 2 bytes RH, 1 CRC
        if( i == 3) {

            if (data[2] != CalcCrC(&data[0])){
                if (_SVM30_Debug){
                    printf("I2C CRC error: Expected 0x%02X, calculated 0x%02X\n",data[2] & 0xff,CalcCrC(&data[0]) &0xff);
                }
                return(ERR_PROTOCOL);
            }

            _Receive_BUF[_Receive_BUF_Length++] = data[0];
            _Receive_BUF[_Receive_BUF_Length++] = data[1];

            i = 0;

            if (_Receive_BUF_Length >= count) break;
        }
    }

    if (i != 0) {
        if (_SVM30_Debug) printf("Error: Data counter %d\n",i);
        while (j < i) _Receive_BUF[_Receive_BUF_Length++] = data[j++];
    }

    if (_Receive_BUF_Length == 0) {
        if (_SVM30_Debug)  printf("Error: Received NO bytes\n");
        return(ERR_PROTOCOL);
    }

    if (_Receive_BUF_Length == count) return(ERR_OK);

    if (_SVM30_Debug)
        printf("Error: Expected bytes : %d, Received bytes %d\n", count,_Receive_BUF_Length);

    return(ERR_DATALENGTH);
}

/**
 * @brief : calculate CRC for I2c comms
 * @param data : 2 databytes to calculate the CRC from
 *
 * Source : datasheet SPS30
 *
 * return CRC
 */
uint8_t SVM30::CalcCrC(uint8_t *data) {
    uint8_t crc = 0xFF;
    for(int i = 0; i < 2; i++) {
        crc ^= data[i];
        for(uint8_t bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ 0x31u;
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

/**
 * @brief : calculate the absolute humidity from relative humidity [%RH *1000]
 * and temperature [mC]
 *
 * @param temperature : temperature in mC
 * @param humidity : humidity in %RH * 1000
 *
 * return : Absolute value
 */

float SVM30::calc_absolute_humidity(int32_t temperature, int32_t humidity) {
    float Temp = (float) temperature/1000;
    float Hum = (float) humidity/1000;

    if (Hum == 0) return(0);

    float AbsoluteHumidity = (6.112 * pow(2.71828,((17.67 * Temp)/(Temp + 243.5))) * Hum * 2.1674) / (273.15 + Temp);

    return(AbsoluteHumidity);
}

/**
 * Convert relative humidity [%RH] and temperature [C] to
 * absolute humidity [g/m^3] that can be used as input for
 * the SetHumidity() call.
 *
 * source: datasheet
 * The 2 data bytes represent humidity values as a fixed-point 8.8bit
 * number with a minimum value of 0x0001 (=1/256 g/m 3 ) and a maximum
 * value of 0xFFFF (255 g/m 3 + 255/256 g/m 3 ).
 * For instance, sending a value of 0x0F80 corresponds to a humidity
 * value of 15.50 g/m3 (15 g/m3 + 128/256 g/m 3 ).
 */
uint16_t SVM30::ConvAbsolute(float AbsoluteHumidity) {
    uint16_t val;
    int val1;

    // get top 8 bits (MSB)
    val = (uint16_t) AbsoluteHumidity;

    // get bottom 8 bits (LSB)
    val1 = (AbsoluteHumidity - (float) val) * 100;

    val = val << 8 | val1;
    // printf("val 0x%x, val1 %d / 0x%x\n", val,val1, val1);

    return(val);
}

/********************************************************************
 * FOLLOWING CODE IS TAKEN FROM
 *
 * https://github.com/Sensirion/embedded-sht/blob/master/shtc1/shtc1.c
 *
 * The code is slightly modified to enable compile and integration in
 * the rest of library.
 *
 **********************************************************************
 * Copyright (c) 2017, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 **********************************************************************/

/**
 * @brief : Convert relative to humidity [%RH*1000] and temperature [mC] to useable
 * temperature and humidity
 */
void SVM30::shtc1_conv(int32_t *temperature, int32_t *humidity, uint16_t temp, uint16_t hum) {

    /**
     * formulas for conversion of the sensor signals, optimized for fixed point
     * algebra:
     * Temperature = 175 * S_T / 2^16 - 45
     * Relative Humidity = 100 * S_RH / 2^16
     */
    *temperature = ((21875 * (int32_t)temp) >> 13) - 45000;
    *humidity = ((12500 * (int32_t)hum) >> 13);
}
