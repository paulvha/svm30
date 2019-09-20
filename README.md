# Sensirion SVM30

## ===========================================================

A program to set instructions and get information from an SVM30. It has been
tested to run on ESP32, MEGA2560, ESP8266 and UNO.
<br> A detailed description of the options and findings will follow later

## Getting Started
As part of a larger project I am looking at analyzing and understanding the air quality.
I have done a number of projects on air-sensors. The SVM30 sensor is a new kid on the block
that looks interesting. This is the first version of a working driver + examples.
More work is happening to create examples, compare against other sensors and document the finds.

A word of warning: the SVM30 needs a female plug of ZHR-4 from JST Sales America Inc.
You might have one that fits (e.g. other sensor use this as well) but I have been able to find a source for that : AliExpress

## Prerequisites
No special library dependencies. The standard libraries will work

## Software installation
Obtain the zip and install like any other

## Program usage
### Program options
<br> 4 examples are included:
 - read all the values from the SVM30
 - update humidity while reading all the values from the SVM30
 - update baseline and then read all the values from the SVM30
 - perform a self-test on the SGP30

<br>Please see the description in the top of the sketch.

## Versioning

### version 1.0 / September 20 2019
 * Initial version Arduino, ESP32, UNO, ESP8266

## Author
 * Paul van Haastrecht (paulvha@hotmail.com)

## License
This project is licensed under the GNU GENERAL PUBLIC LICENSE 3.0

## Acknowledgements
Make sure to read the datasheet from Sensirion. They provide good starting point for information and are
added in the extras directory.<br>


