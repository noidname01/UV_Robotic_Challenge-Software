# Arduino firmware

## The role of Arduino
We use a Arduino Mega 2560 for this application. It's powered by the USB type B port connected to our Raspberry Pi.
It controls 4 DC motors, a servo motor, and receives velocity/distance/degree data from the encoder, and distance information from three TOF distance sensors.

## Communication with Raspberry Pi
We apply serial communication (UART) over USB to communicate with Raspberry Pi.

## Schematics
Systems architecture:
![Systems architecture](https://github.com/noidname01/UV_Robotic_Challenge-Software/blob/master/arduino_main/System%20architecture%20diagram.png)
A very detailed schematic
![Detailed schematic](https://github.com/noidname01/UV_Robotic_Challenge-Software/blob/master/arduino_main/Module%20wiring%20map.png)
## Arduino source code

### arduino_main.ino
We check the pir sensors for the presence of human first thing in the main loop. Sends a signal when the presence of human is detected, and when people are cleared.

We carry on checking if the distance of the TOF sensors are abnormal. If there is an obstacle or edge in front, then add an effective wall to the `obstacle.py` node. (not yet implemented)

If every precautional sensor values are normal, we continue on receiving and parsing commands from Raspberry Pi, sweep the servo motor by a small angle, and update the encoder and TOF distance sensor value.

To make the encoder work properly, we normally have to keep the run time for one loop under 53 ms. Since this is nearly impossible if we update the TOF distance sensor on every loop, we decide to update the encoder several times in the loop, after each process that takes a relatively long time.
```C++
    UpdateEncoderR();
    sweeper1.doSweep();
    UpdateEncoderR();
    readTOF_F();
    UpdateEncoderR();
    readTOF_R();
    UpdateEncoderR();
    readTOF_L();
    UpdateEncoderR();
```

### encoder_class.h
We use a photo interrupter and a 3D printed disk to make an encoder, which is installed on the left front wheel of the robot. The path searching algorithm request that the robot has two types of movements: one with specified direction and distance (e.g. "move forward 30 cm" or "turn right 90 degrees", etc.), and another with direction only (e.g. "move forward").

The former returns a **confirm message "c"** to the Raspberry Pi when the motion is completed, while the latter returns **the distance/degree moved** before the robot receives a "halt" message.
```C++
    // Upon finishing a movement with direction and distance/degree
    if(clicks_left <= 0 && non_stop == 0){
      non_stop = 1;
      return 1;
    }
    else return 0;
```

When finished moving an unspecified distance and returned the distance/degree:
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
```C++
void Update()
{
  if((millis() - lastUpdate) > updateInterval)  // time to update
  {
    lastUpdate = millis();
    pos += increment;
    servo.write(pos);
    //Serial.println(pos);
    // Sweeps in a 120 degree angle
    if ((pos >= 150) || (pos <= 30)) // end of sweep
    {
      // reverse direction
      increment = -increment;
    }
  }
}
```

### tof.h
We use three TOF distance sensors (VL53L0X-V2) to measure distance; the sensors share I2C bus. In the `initTOF` function, we wake up three sensors one by one (by pulling `XSHUT` pin high) and specify three unique addresses to properly boot the sensors.
```C++
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
Defines functions (`forward`, `backward`, `leftTurn`, `rightTurn`, `halt`) to control movements. (Note: We omitted `rightShift` and `leftShift` to simplify the path finding algorithm)
