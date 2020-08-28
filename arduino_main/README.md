# Arduino
## The role of Arduino
We're using a Arduino Mega 2560 for this application. It's powered by the USB type B port which connects to our Raspberry Pi.
It controls 4 DC motors, a servo motor, and receives velocity data from the encoder, and distance information from three TOF distance sensors.
## Schematics

## Arduino source code files
### arduino_main.ino
### encoder_class.h
An encoder is installed on the left front wheel of the robot. The path finding algorith request that the robot has two types of movements: one with specified direction and distance (i.e. "move forward 30cm", or "turn right 90 degrees"), and the other with direction only.
\nThe former returns a confirm message "c" to the Raspberry Pi when the motion is complete, while the latter returns the distance/degree moved before the robot receives a "halt" message.
### servo_sweeper.h
### tof.h
### track.h


