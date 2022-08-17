# Sensirion SVM30

## ===========================================================

A program to set instructions and get information from an SVM30. It has been
tested to run on ESP32, MEGA2560, ESP8266, UNO, Apollo3 and Feather Lora 32U4.
It will also work on Wire1 for a DUE, because of the onboard pull-up resistors on the standard Wire. use example7
<br> A detailed description of the options and findings are available in the SVM30.ODT in the extra directory

<br>A version for the Raspberry Pi is available on ![https://github.com/paulvha/SVM30_on_raspberry](https://github.com/paulvha/SVM30_on_raspberry)

## Getting Started
As part of a larger project I am looking at analyzing and understanding the air quality.
I have done a number of projects on air-sensors. The SVM30 sensor was a new kid on the block
that looks interesting. This is a working driver + examples.

A word of warning: the SVM30 needs a female plug, which is different depending on the product version of your board

SVM30-Y  Yeonho Electronics, 20037WR-04 <br>
SVM30-J  Scondar SCT2001WR -S-4P compatible to JST part no. S4B-PH-SM4-TB <br>

## Prerequisites
For example6 : LMIC  : ![https://github.com/paulvha/SPS30_lora](https://github.com/paulvha/SPS30_lora)
For example8 : LCD   : defined in top of sketch

<br>Otherwise no special library dependencies. The standard libraries will work.

## Software installation
Obtain the zip and install like any other.

## Program usage
### Program options
<br> 8 examples are included:
 - 1. read all the values from the SVM30
 - 2. update humidity while reading all the values from the SVM30
 - 3. update baseline and then read all the values from the SVM30
 - 4. perform a self-test on the SGP30
 - 5. Read basic information, add functions for heatindex, dewpoint and Celsius/Fahrenheit selection
 - 6. Sketch to read the data from the SVM30 and connect Feather Lora 32U4 to TTN
 - 7. like example1 but you can now select different Wire-channels (Needed for DUE)
 - 8. lime example1 but with an LCD display
<br>Please see the description in the top of the sketches as well as the SVM30.ODT

## Versioning

### Version 1.3.1 / August 2022 / paulvha
 * fixed _pf_buffer_ warning on ESP32 when compiling with -Werror=all

### Version 1.3 / October 2020 / paulvha
 * Added support for Artemis / Apollo3 boards
 * fixed some typo's and some small coding issues
 * added example8 (display including a LCD screen)

### Version 1.2 / August 2020
 * it seems that older product version(level 9) of the SGP30 / SVM30 fail to read raw data.
 * added raw boolean (default true) to include(true) / exclude (false) raw data in case of older product version.
 * added read-delay setting based on the kind of command request to improve stability
 * added example7 to select and I2C/ Wire channel (Wire1 is needed for Arduino Due)
 * added functions for inceptive baseline of the SGP30 (requires level 34 at least). Documented in SGP30 datasheet May 2020.

### Version 1.1 / October 2019
 * added dewPoint and heatindex
 * added temperature selection (Fahrenheit / Celsius)
 * updated examples
 * added example 5 and 6

### version 1.0 / September 20 2019
 * Initial version Arduino, ESP32, UNO, ESP8266


## Author
 * Paul van Haastrecht (paulvha@hotmail.com)

## License
This project is licensed under the GNU GENERAL PUBLIC LICENSE 3.0

## Acknowledgements
Make sure to read the SVM30, SHTC1 and SGP30 datasheets from Sensirion.
They provide good starting point for information and are added in the extras directory.<br>
Philipp : Extra clarification on about female plug.
Amy Kim : Help diagnosing older product version raw-reading issue.
