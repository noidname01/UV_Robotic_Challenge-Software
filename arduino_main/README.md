# Arduino
## The role of Arduino
We're using a Arduino Mega 2560 for this application. It's powered by the USB type B port which connects to our Raspberry Pi.
It controls 4 DC motors, a servo motor, and receives velocity data from the encoder, and distance information from three TOF distance sensors.
## Schematics

## Arduino source code files
### arduino_main.ino

### encoder_class.h
We use a photo interrupter and a 3D printed disk to make an encoder, which is installed on the left front wheel of the robot. The path finding algorith request that the robot has two types of movements: one with specified direction and distance (e.g. "move forward 30cm" or "turn right 90 degrees", etc.), and the other with direction only (e.g. "move forward").

The former returns a **confirm message "c"** to the Raspberry Pi when the motion is completed, while the latter returns **the distance/degree moved** before the robot receives a "halt" message.
### servo_sweeper.h
Since we rely on the main loop speed to obtain the correct encoder information, freezing up the processor when servo motor turns isn't an option. Therefore, we call the "Update" function on every loop to **update the servo motor position by a small amount**.

### tof.h
We use three TOF distance sensors (VL53L0X-V2) to measure distance; they transmit distance data via I2C protocal, which shares the "SCL" and "SDA" pins. In the "initTOF" function, we wake up three sensors one by one (by setting XSHUT pin high) and specify three unique addresses to properly boot the sensors.

### track.h
Defines functions("forward", "backward", "leftTurn", "rightTurn", "halt") to control movements. (Note: We emitted "rightShift" and "leftShift" to simplify the path finding algorithm)


