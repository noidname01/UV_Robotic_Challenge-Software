#include <Servo.h>

// Class for sweeping servo motor repeatedly
class Sweeper
{
  Servo servo;              // the servo
  int pos;              // current servo position 
  int increment;        // increment to move for each interval
  int updateInterval;      // interval between updates
  unsigned long lastUpdate; // last update of position
     
public: 
  bool sweeping;

  Sweeper(int interval)
  {
    updateInterval = interval;
    increment = 10;
    sweeping = 0;
    pos = 90;
  }

  void startSweep()
  {
    sweeping = 1;
  }

  void stopSweep()
  {
    sweeping = 0;
  }

  // called by the main loop in each loop
  void doSweep()
  {
    if(sweeping){
      Update();
    }
  }
      
  void Attach(int pin)
  {
    servo.attach(pin);
    servo.write(90);
  }
      
  void Detach()
  {
    servo.detach();
  }

  // rotate the servo by a small degree
  void Update()
  {
    if((millis() - lastUpdate) > updateInterval)  // time to update
    {
      lastUpdate = millis();
      pos += increment;
      servo.write(pos);
      //Serial.println(pos);
      if ((pos >= 150) || (pos <= 30)) // end of sweep
      {
        // reverse direction
        increment = -increment;
      }
    }
  }

  // get current angle of the servo
  int get_pos()
  {
    return pos;
  }
};
