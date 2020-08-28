#ifndef ENCODER
#define ENCODER
#define TIMEOUT 200

double distToClicks_s = 1.0/ (PI*5.67)*36;
double distToClicks_r = 0.68;

enum Movement{
  Forward = 0,
  Backward,
  RightTurn,
  LeftTurn,
  Halt
};

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
  int clicks_walked;
  Movement curMovement;
  
public:
  Encoder(int p, int t, int n)
  :pin(p), report_interval(t), slotNum(n)
  {
    
    prev_state = 0;
    prev_report = millis();
    timeout_timer = millis();
    prev_time[1] = prev_time[2] = prev_time[3] = -100000;
    prev_time[0] = millis();
    
    //attachInterrupt(digitalPinToInterrupt(pin), isr, RISING);
    
    clicks_left = 0;
    non_stop = 1;
    clicks_walked = 0;
    curMovement = Halt;
    

    
  }
  
  // Called by the main loop in each loop
  // Returns 1 when finished moving for a specified distance/degree; returns 0 otherwise
  bool report()
  {
    //Serial.print(clicks_left);
    //Serial.println(" clicks left");
    if( prev_state == 0 && digitalRead(pin) == 1 )
    {
      prev_time[3] = prev_time[2];
      prev_time[2] = prev_time[1];
      prev_time[1] = prev_time[0];
      prev_time[0] = timeout_timer = millis();
      //undetected = 0;
      if(clicks_left > 0 && non_stop == 0)
        clicks_left--;
      else if(non_stop == 1)
        clicks_walked++;
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
      prev_report = millis();
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

  // move in the specified direction for an indefinite distance/degree
  void forrest_gump( Movement m )
  {
    curMovement = m;
    clicks_left = 0;
    non_stop = 1;
    clicks_walked = 0;
  }


  int getClicks()
  {
    return clicks_walked;
  }

  void reset_click()
  {
    clicks_left = 0;
    clicks_walked = 0;
  }

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

//  void isr()
//  {
//    undetected = 1;
//  }

};

#endif
