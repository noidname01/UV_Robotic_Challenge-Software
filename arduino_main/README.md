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
```C++
// Upon finishing a movement with direction and distance/degree
    if(clicks_left <= 0 && non_stop == 0){
      non_stop = 1;
      return 1;
    }
    else return 0;
```
```C++
// called when finished moving an unspecified distance and returned the distance/degree
double getDistOrDegree()
  {
    if(non_stop == 1){
      if(curMovement == Forward || curMovement == Backward){
        Serial.println(clicks_walked * 1.0 / distToClicks_s);
        return clicks_walked * 1.0 / distToClicks_s;
      }
      else if(curMovement == LeftTurn || curMovement == RightTurn){
        Serial.println(clicks_walked * 1.0 / distToClicks_r);
        return clicks_walked * 1.0 / distToClicks_r;
      }
    }
    else 
      return 0;
  }
```

### servo_sweeper.h
Since we rely on the main loop speed to obtain the correct encoder information, freezing up the processor when servo motor turns isn't an option. Therefore, we call the "Update" function on every loop to **update the servo motor position by a small amount**.

### tof.h
We use three TOF distance sensors (VL53L0X-V2) to measure distance; they transmit distance data via I2C protocal, which shares the "SCL" and "SDA" pins. In the "initTOF" function, we wake up three sensors one by one (by setting XSHUT pin high) and specify three unique addresses to properly boot the sensors.
```C++
// all reset
  digitalWrite(SHT_LOX_F, LOW);    
  digitalWrite(SHT_LOX_R, LOW);
  digitalWrite(SHT_LOX_L, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX_F, HIGH);    
  digitalWrite(SHT_LOX_R, HIGH);
  digitalWrite(SHT_LOX_L, HIGH);
  delay(10);
  ///////////////////////////////////////////////
  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX_F, HIGH);
  digitalWrite(SHT_LOX_R, LOW);
  digitalWrite(SHT_LOX_L, LOW);

  // initing LOX1
  if(!lox_F.begin(LOX_F_ADDRESS)) {
    Serial.print(F("Failed to boot F VL53L0X at address "));
    Serial.println(LOX_F_ADDRESS);
  }
  delay(10);
  ///////////////////////////////////////////////
  // activating LOX2
  digitalWrite(SHT_LOX_R, HIGH);
  delay(10);
  
  //initing LOX2
  if(!lox_R.begin(LOX_R_ADDRESS)) {
    Serial.println(F("Failed to boot R VL53L0X at address "));
    Serial.println(LOX_R_ADDRESS);
    delay(200);
  }
  delay(10);
```

### track.h
Defines functions("forward", "backward", "leftTurn", "rightTurn", "halt") to control movements. (Note: We emitted "rightShift" and "leftShift" to simplify the path finding algorithm)

