#ifndef ENCODER
#define ENCODER
#define TIMEOUT 200

class Encoder
{
  const int pin;
  const int report_interval;
  unsigned long prev_report;
  const int slotNum;
  long prev_time[4];
  unsigned long timeout_timer; 
  //bool undetected;
  bool prev_state;
  int clicks_left;
  bool non_stop;
  
public:
  Encoder(int p, int t, int n)
  :pin(p), report_interval(t), slotNum(n)
  {
    //undetected = 0;
    prev_state = 0;
    prev_report = millis();
    timeout_timer = millis();
    prev_time[1] = prev_time[2] = prev_time[3] = -100000;
    prev_time[0] = millis();
    //attachInterrupt(digitalPinToInterrupt(pin), isr, RISING);
    clicks_left = 0;
    non_stop = 1;
    
  }
  
  bool report()
  {
    Serial.print(clicks_left);
    Serial.println(" clicks left");
    if( prev_state == 0 && digitalRead(pin) == 1 )
    {
      prev_time[3] = prev_time[2];
      prev_time[2] = prev_time[1];
      prev_time[1] = prev_time[0];
      prev_time[0] = timeout_timer = millis();
      //undetected = 0;
      if(clicks_left > 0)
        clicks_left--;
    }
    
    prev_state = digitalRead(pin);
    if(millis() - timeout_timer > TIMEOUT){
      prev_time[1] = prev_time[2] = prev_time[3] = -100000;
      prev_time[0] = millis();
    }
    if( (millis() - prev_report) > report_interval ){
      double omega = 2*PI/ (prev_time[0] - prev_time[3]) /12;
      //Serial.print("Omega = ");
      //Serial.println(omega*1000);
      //prev_report = millis();
    }
    
    if(clicks_left <= 0 && non_stop == 0){
      non_stop = 1;
      return 1;
    }
    else return 0;
  }
  void goStraight(int d)
  {
    non_stop = 0;
    clicks_left = d;
  }
  
  void revolve(int d)
  {
    non_stop = 0;
    clicks_left = d;
  }

  void forrest_gump()
  {
    clicks_left = 0;
    non_stop = 1;
  }

  void reset_click()
  {
    clicks_left = 0;
  }

//  void isr()
//  {
//    undetected = 1;
//  }
};

#endif
