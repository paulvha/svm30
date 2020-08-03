# Sensirion SVM30

## ===========================================================

A program to set instructions and get information from an SVM30. It has been
tested to run on ESP32, MEGA2560, ESP8266, UNO and Feather Lora 32U4.
<br> A detailed description of the options and findings are available in the SVM30.ODT in the extra directory

<br>A version for the Raspberry Pi is available on ![https://github.com/paulvha/SVM30_on_raspberry](https://github.com/paulvha/SVM30_on_raspberry)

## Getting Started
As part of a larger project I am looking at analyzing and understanding the air quality.
I have done a number of projects on air-sensors. The SVM30 sensor is a new kid on the block
that looks interesting. This is a working driver + examples.

A word of warning: the SVM30 needs a female plug, which is different depending on the rpoduct version of your board

SVM30-Y  Yeonho Electronics, 20037WR-04 <br>
SVM30-J  Scondar SCT2001WR -S-4P compatible to JST part no. S4B-PH-SM4-TB <br>

## Prerequisites
Only for example6 : LMIC  : ![https://github.com/paulvha/SPS30_lora](https://github.com/paulvha/SPS30_lora)
<br>Otherwise no special library dependencies. The standard libraries will work.

## Software installation
Obtain the zip and install like any other.

## Program usage
### Program options
<br> 6 examples are included:
 - 1. read all the values from the SVM30
 - 2. update humidity while reading all the values from the SVM30
 - 3. update baseline and then read all the values from the SVM30
 - 4. perform a self-test on the SGP30
 - 5. Read basic information, add functions for heatindex, dewpoint and Celsius/Fahrenheit selection
 - 6. Sketch to read the data from the SVM30 and connect Feather Lora 32U4 to TTN
<br>Please see the description in the top of the sketches as well as the SVM30.ODT

## Versioning

### version 1.0 / September 20 2019
 * Initial version Arduino, ESP32, UNO, ESP8266

### Version 1.1 / October 2019
 * added dewPoint and heatindex
 * added temperature selection (Fahrenheit / Celsius)
 * updated examples
 * added example 5 and 6

## Author
 * Paul van Haastrecht (paulvha@hotmail.com)

## License
This project is licensed under the GNU GENERAL PUBLIC LICENSE 3.0

## Acknowledgements
Make sure to read the SVM30, SHTC1 and SGP30 datasheets from Sensirion.
They provide good starting point for information and are added in the extras directory.<br>
Philipp : Extra clarification on about female plug
