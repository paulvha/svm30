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
 * Version 1.0 / September 2019 / paulvha
 * - Initial version
 *
 * Version 1.1 / October 2019 / paulvha
 * - added dewPoint and heatindex
 * - added Temperature selection (Fahrenheit / Celsius)
 * - updated examples
 * - added examples 5 and 6
 *
 * Version 1.2 / August 2020 / paulvha
 * it seems that older product version(level 9) of the SGP30 / SVM30 fail to read raw data.
 * added support to exclude reading the raw data.
 *
 * - added raw boolean (default true) to include(true) / exclude (false) raw data
 * - added read-delay setting based on the kind of command request.
 * - added support for inceptive baseline of the SGP30 (requires level 34 at least)
 *********************************************************************
 */
#ifndef SVM30_H
#define SVM30_H

#include "Arduino.h"            // Needed for Stream
#include "printf.h"             // for debug
#include "Wire.h"               // for I2c

// set driver version
#define VERSION "1.2 / August 2020";

/* structure to return measurement values */
typedef struct svm_values
{
    uint16_t   r_humidity;    // SHTC1 (raw data from sensor)
    uint16_t   r_temperature; // SHTC1 (raw data from sensor)
    int32_t    humidity;      // SHTC1 (converted humidity)
    int32_t    temperature;   // SHTC1 (converted temperature in C)
    float      absolute_hum;  // calculated absolute humidity in g/m3.
    uint16_t   CO2eq;         // SGP30
    uint16_t   TVOC;          // SGP30
    uint16_t   H2_signal;     // SGP30 Raw H2 signal
    uint16_t   Ethanol_signal;// SGP30 Raw ethanol signal
    float      heat_index;    // calculated heat index
    float      dew_point;     // calculated dew point
};

/*
 * During testing I did run into bad connections causing errors.
 * This could also happen if you have a long line between the AVR and SVM30.
 * In order to improve stability, you can add the retry option in
 * RequestFromSVM().
 *
 * Remove the comment '//' from before #define
 *
 * RETRY_CNT is the number of retry attemps
 */
//#define STABIILITY 1
#define RETR_CNT 5

/*************************************************************/
/* internal driver error codes */
#define ERR_OK          0x00
#define ERR_DATALENGTH  0X01
#define ERR_UNKNOWNCMD  0x02
#define ERR_ACCESSRIGHT 0x03
#define ERR_PARAMETER   0x04
#define ERR_OUTOFRANGE  0x28
#define ERR_CMDSTATE    0x43
#define ERR_TIMEOUT     0x50
#define ERR_PROTOCOL    0x51

/* source : Datasheet SVM30
 * A sensor reset can be generated using the “General Call” mode
 * according to I2C-bus specification. It is important to understand
 * that a reset generated in this way is not device specific.
 * All devices on the same I2C bus that support the General Call
 * mode will perform a reset. This has to be used for SGP30 */
#define RESET_ADDRESS   0x0
#define RESET_CMD       0x6

/* SGP30 information   */
#define SGP30_ADDRESS 0x58          // I2c address
#define SGP30 SGP30_ADDRESS

#define SGP30_Init_Air_Quality      0x2003
#define SGP30_Measure_Air_Quality   0x2008
#define SGP30_Get_Baseline          0x2015
#define SGP30_Set_Baseline          0x201E
#define SGP30_Set_Humidity          0x2061
#define SGP30_Measure_Test          0x2032
#define SGP30_TestOK                0xD400
#define SGP30_Get_Feature_Set       0x202F
#define SGP30_Measure_Raw_Signals   0x2050
#define SGP30_Get_tvoc_inceptive_baseline 0x20B3        // Datasheet SGP30 May 2020 - requires level 34 - added 1.2
#define SGP30_Set_tvoc_inceptive_baseline 0x2077        // Datasheet SGP30 May 2020 - requires level 34 - added 1.2
#define SGP30_Read_ID               0x3682

/* SHTC1 information  */
#define SHTC1_ADDRESS 0x70          // I2c address
#define SHTC1 SHTC1_ADDRESS

/* source : Datasheet SVM30
 * The SHTC1 provides the possibility to define the sensor behavior
 * during measurement as well as the transmission sequence of
 * measurement results. These characteristics are  defined  by
 * the  appropriate measurement command. The commands used here do NOT
 * require clock stretching as some boards do not support that in a
 * good / right manner.
 * Each measurement command triggers both a temperature and a humidity measurement.*/
#define SHTC1_Read_Temp_First           0x7866  // polling no clock stretching
#define SHTC1_Read_Humidity_First       0x58E0  // polling no clock stretching (info only not used)
#define SHTC1_CS_Read_Temp_First        0x7CA2  // clock stretching (info only not used
#define SHTC1_CS_Read_Humidity_First    0x5C24  // clock stretching (info only not used)
#define SHTC1_Read_ID                   0xEFC8
#define SHTC1_Reset                     0x805D

// wait times (mS) after sending command to sensor
#define DEFAULT_WAIT 50             // can be adjusted if you get timeouts
#define MEASURE_WAIT 500            // although 220mS should be enough, it seems to need more time
/***************************************************************/

class SVM30
{
  public:

    SVM30(void);

    /**
     * @brief  Enable or disable the printing of debug messages.
     *
     * @param act :
     *  false : no debug messages
     *  true : enable debug messages
     */
    void EnableDebugging(bool act);

    /**
     * @brief Initialize the communication & start SGP30 (use wire)
     *
     * @return :
     *   true on success else false
     */
    bool begin();

    /**
     * @brief Manual assigment I2C communication port added 1.2
     *
     * @param port : I2C communication channel to be used
     *
     * User must have performed the wirePort.begin() in the sketch.
     */
    bool begin(TwoWire *wirePort);

    /**
     * @brief check if SVM30 sensors are available (read ID)
     *
     * @return :
     *   true on success else false
     */
    bool probe();

    /**
     * @brief : return driver version.
     */
    const char * GetDriverVersion();

    /**
     * @brief : Trigger a read on the SGP30
     *
     * source : Datasheet SVM30
     * The on-chip baseline compensation algorithm has been optimized
     * for 1HZ sampling rate. The sensor shows best performance when
     * used with this sampling rate. The 1 HZ sequences needs to be
     * implemented in the sketch (see examples)
     *
     * @return :
     *   true on success else false
     */
    bool TriggerSGP30();

    /**
     * @brief read ID number from SGP30 or SHTC1
     *
     * @param device : I2C address of device
     * @param id : buffer to store ID (min 3 words)
     *
     * @return :
     *   true on success else false
     */
    bool GetId(uint8_t device, uint16_t *buf);

    /**
     * @brief reset SGP30 or SHTC1
     *
     * @param device : I2C address of device
     *
     * @return :
     *   true on success else false
     */
    bool reset(uint8_t device);

    /**
     * @brief read feature set from SGP30
     *
     * @param buf : buffer to store feature set number (min 2)
     *
     * @return :
     *   true on success else false
     */
    bool GetFeatureSet(char *buf);

    /**
     * @brief Measure  SELF test on SGP30
     *
     * @return :
     *   true on success else false
     */
    bool MeasureTest();

    /**
     * @brief get individual baseline from SGP30
     *
     * @param baseline : store baseline
     *
     * @return :
     *   true on success else false
     */
    bool GetBaseLine_CO2(uint16_t *baseline){return(GetBaseLine(baseline, false));}
    bool GetBaseLine_TVOC(uint16_t *baseline){return(GetBaseLine(baseline, true));}

    /**
     * @brief : get BOTH baselines (TVOC and CO2) from SGP30
     *
     * @param : pointer to baseline to store results
     *
     * @return :
     *   true on success else false
     */
    bool GetBaseLines(uint32_t *baseline);

    /**
     * @brief set individual baseline on SGP30
     *
     * @param baseline: baseline to set
     *
     * @return :
     *   true on success else false
     *
     * source datasheet:
     * Approximately in the first 15 seconds of operation after start
     * the call will fail unless a previous baseline was restored.
     *
     * REMARK : Sending baseline values of zero will be treated as an error.
     */
    bool SetBaseLine_CO2(uint16_t baseline) {return(SetBaseLine(baseline,false));}
    bool SetBaseLine_TVOC(uint16_t baseline){return(SetBaseLine(baseline, true));}

    /**
     * @brief : set BOTH baselines (TVOC and CO2) on SGP30
     *
     * @param baseline (as retrieved with GetBaselines()
     *
     * @return :
     *   true on success else false
     */
    bool SetBaseLines(uint32_t baseline);

    /**
     * @brief : get Inceptivebaseline  (impact TVOC only)
     *
     * See datasheet May 2020 SGP30
     *
     * !!! REQUIRES LEVEL 34 FEATURE SET !!!
     *
     * @return :
     *   true on success else false
     */
    bool GetInceptiveBaseLine_TVOC(uint16_t *baseline);
    bool SetInceptiveBaseLine_TVOC(uint16_t baseline);

    /**
     * @brief set humidity on SGP30
     *
     * @param humidity: humidity value to set
     *
     * Sending a humidity value of 0x0000 can be used to turn off
     * the humidity compensation. The humidity value used for
     * compensation is set to its default value (11.57g/m3)
     *
     * @return :
     *   true on success else false
     */
    bool SetHumidity(float humidity);

    /**
     * @brief : read all measurement values from the sensor and store in structure
     * @param v: pointer to structure to store
     * @param raw: if true it will try to read the raw values from the SGP30 (update 1.2)
     *
     * It seems that older version (level 9) of the SGP30 do not support reading raw, hence the "raw" -option
     * has been added as an option to exclude. By default it will read to stay backward compatible
     *
     * @return :
     *   true on success else false
     */
    bool GetValues(struct svm_values *v, bool raw = true);

    /**
     * @brief : Set temperature.
     *
     * @param act :
     *  true set to Celsius,
     *  false is Fahrenheit
     *
     */
    void SetTempCelsius(bool act);

  private:

    /** shared variables */
    uint8_t _Receive_BUF[40];   // buffers
    uint8_t _Send_BUF[10];      // 2 command + max 6 data
    uint8_t _Receive_BUF_Length;
    uint8_t _Send_BUF_Length;
    uint8_t _I2C_address;       // I2C address to use (SGP30 or SHTC1)
    TwoWire *_i2cPort;          // holds the I2C port
    bool    _SelectTemp;        // select temperature (true = celsius)
    bool     _SVM30_Debug;      // program debug level
    bool    _started;           // indicate the SGP30 measurement has started
    uint8_t _wait;              // wait time after sending command
    unsigned long   ReadDelay = 10;  // time to wait after requesting from update 1.2

    /** supporting routines */
    bool StartSGP30();
    uint16_t byte_to_uint16(int x);
    void calc_absolute_humidity(struct svm_values *v);
    uint16_t ConvAbsolute(float AbsoluteHumidity);
    bool SetBaseLine(uint16_t baseline, bool tvoc);
    bool GetBaseLine(uint16_t *baseline, bool tvoc);
    void computeHeatIndex(struct svm_values *v);
    void calc_dewpoint(struct svm_values *v);

    /** I2C communication */
    void PrepSendBuffer(uint8_t I2C_add, uint16_t cmd, char *param = NULL, uint8_t len = 0);
    uint8_t RequestFromSVM(uint8_t count);
    uint8_t ReadFromSVM(uint8_t cnt);
    uint8_t SendToSVM();
    uint8_t CalcCrC(uint8_t data[2]);

    /********************************************************************
     * FOLLOWING CODE IS TAKEN FROM
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

    void shtc1_conv(int32_t *temperature, int32_t *humidity, uint16_t temp, uint16_t hum);
};

#endif /* SVM30_H */
