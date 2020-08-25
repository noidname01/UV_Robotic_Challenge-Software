#include <Servo.h>
     
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
      
  void Update()
  {
    if((millis() - lastUpdate) > updateInterval)  // time to update
    {
      lastUpdate = millis();
      pos += increment;
      servo.write(pos);
      //Serial.println(pos);
      if ((pos >= 135) || (pos <= 45)) // end of sweep
      {
        // reverse direction
        increment = -increment;
      }
    }
  }

  int get_pos()
  {
    return pos;
  }
};
     
extern Sweeper sweeper1(1);
/*
void setup() 
{ 
  Serial.begin(9600);
  sweeper1.Attach(9);
  sweeper2.Attach(10);
} 
     
     
void loop() 
{ 
  sweeper1.Update();
      
  if(digitalRead(2) == HIGH)
  {
     sweeper2.Update();
     led1.Update();
  }
      
  led2.Update();
  led3.Update();
}
*/
