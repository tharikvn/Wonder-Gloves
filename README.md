# Wonder-Gloves
A wearable glove that converts sign language to speech, thus making communication for differently abled more effective.

# Overview
Arduino is the chosen platform for executing this project, and the relevant source files reside in the 'Src/' directory.
Also an Android Test App was created through which the sign language was converted to speech, which was created using an
online tool called MIT App Inventor. The exported project from the online tool is present in the 'TestApp/' directory.

This project was tested on Arduino Mega 2560 Rev3 since it required 7 analog pins, 5 x Flex Sensors, MPU6050 (Acc. + Gyro).

Note: This code is was later modified for making 5 GPIO pins to be set as HIGH (5V output) for providing Vin (Voltage input)
for each Flex Sensors.

This project was completed in 2017 so the source code might be outdated when using MIT App Inventor and Arduino IDE.
