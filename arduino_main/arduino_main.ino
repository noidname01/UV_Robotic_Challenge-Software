#include <Servo.h>
#include <string.h>

#define MotorLF_I1      28//I1（left back）
#define MotorLF_I2      26//I2（left back）
#define MotorRF_I3      24//I3（right back）
#define MotorRF_I4      22//I4（right back）
#define MotorLB_I1      38//I1（left front）
#define MotorLB_I2      40//I2（left front）
#define MotorRB_I3      30//I3（right front 
#define MotorRB_I4      32//I4（right front）
#define ServoPin        31
#define EncoderR_pin    18

#define ENA     2//left front
#define ENB     3//right front
#define ENC     4//left back
#define END     5//right back

#define Relay_1         47
#define Relay_2         49
#define Relay_3         51
#define Relay_4         53

#define Pir_1 33
#define Pir_2 35
#define Pir_3 37
#define Pir_4 39

#include "motorControl.h"
#include "encoder_class.h"
#include "servo_sweeper.h"
#include "tof.h"


Encoder EncoderR(EncoderR_pin, 10, 36);
Sweeper sweeper1(1);

bool noPeople = 0;
bool overCliff = 0;

void LightsOn()
{
  digitalWrite(Relay_1, LOW);
  digitalWrite(Relay_2, LOW);
  delay(1500);
  digitalWrite(Relay_3, LOW);
  delay(1500);
  digitalWrite(Relay_4, LOW);
}

void LightsOff()
{
  digitalWrite(Relay_1, HIGH);
  digitalWrite(Relay_2, HIGH);
  digitalWrite(Relay_3, HIGH);
  digitalWrite(Relay_4, HIGH);

}

void UpdateEncoderR()
{
  // Sends confirm message "c" when finishes a movement with direction and distance/degree
    if( EncoderR.report() ){
      halt();
      sweeper1.stopSweep();
      overCliff = 0;
      Serial.println("c");
    }
}

void setup(){
  Serial.begin(9600);
  pinMode(MotorLF_I1, OUTPUT);
  pinMode(MotorLF_I2, OUTPUT);
  pinMode(MotorRF_I3, OUTPUT);
  pinMode(MotorRF_I4, OUTPUT);
  pinMode(MotorLB_I1, OUTPUT);
  pinMode(MotorLB_I2, OUTPUT);
  pinMode(MotorRB_I3, OUTPUT);
  pinMode(MotorRB_I4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(END, OUTPUT);
  pinMode(EncoderR_pin, INPUT);
//  pinMode(EncoderL, INPUT);
  pinMode(ServoPin, OUTPUT);

  pinMode(Relay_1, OUTPUT);
  pinMode(Relay_2, OUTPUT);
  pinMode(Relay_3, OUTPUT);
  pinMode(Relay_4, OUTPUT);

  pinMode(Pir_1, OUTPUT);
  pinMode(Pir_2, OUTPUT);
  pinMode(Pir_3, OUTPUT);
  pinMode(Pir_4, OUTPUT);
  
  LightsOff();

  sweeper1.Attach(ServoPin);
  initTOF();
}


void loop() {
  //Serial.println('S');
  
  bool pirRead[4];
  pirRead[0] = digitalRead(Pir_1);
  pirRead[1] = digitalRead(Pir_2);
  pirRead[2] = digitalRead(Pir_3);
  pirRead[3] = digitalRead(Pir_4);
  
  // Check the pir sensor for human presence first thing in the loop;
  bool lastPeopleState = noPeople;
  for(int i=0; i<4; i++){
    if( !pirRead[i] ){
      LightsOff();
      //halt();
      // Turn UVC lamps off but keep warning light on
      digitalWrite(Relay_1, LOW);
      noPeople = 0;
      break;
    }
    else{
      noPeople = 1;
    }
  }
  // Signal Raspberry Pi to stop sending commands
  if(lastPeopleState == 1 && noPeople == 0){
    Serial.println("Human");
  }
  // Signal Raspberry Pi to resume the proccess
  else if(lastPeopleState == 0 && noPeople == 1){
    
    switch(EncoderR.curMovement)
    {
      case Forward:
        forward(100);
        break;
      case Backward:
        backward(100);
        break;
      case RightTurn:
        rightturn(100);
        break;
      case LeftTurn:
        leftturn(100);
        break;
      case Halt:
        break;
    }
    Serial.println("Resume");
    
  }


  //Serial.println('1');




  
  // Continue the whole proccess only if nobody is around
  if(/*noPeople == 1 */true)
  {
    
      
    bool enc_state = digitalRead(EncoderR_pin);
    //Serial.println(enc_state);
  
  //  Serial.println('1');
    //Serial.print("Range Status:");
    //Serial.println(measure_F.RangeStatus);
    if(measure_F.RangeMilliMeter < 150){
      
      if(measure_F.RangeStatus != 4) {
        //Serial.print("F too close\n");
        //Serial.println(measure_F.RangeMilliMeter);
        //halt();
      }
    }
    /*
    else {
      Serial.print("F normal\n");
      Serial.println(measure_F.RangeMilliMeter);
    }
    */
    // If left wheel is close to a "cliff"
    if(measure_R.RangeMilliMeter < 93 || measure_R.RangeMilliMeter > 114 ){
      //Serial.print("R too close or too Far!\n");
      //Serial.println(measure_R.RangeMilliMeter);
      
      if(EncoderR.curMovement != ReverseMov)
        halt();
       
      if(!overCliff){
        Serial.println("er");
        overCliff = 1;
      }
        

      // move backwards if previously moving forward
      if(EncoderR.curMovement == Forward || EncoderR.curMovement == Backward){
        backward(80);
        EncoderR.goStraight(Backward, 8.0*distToClicks_s);
      }
      // turn back to the original position if previously turning
      else
        EncoderR.turnBack();
    }
    
    if(measure_L.RangeMilliMeter < 85 || measure_L.RangeMilliMeter > 106){
     // Serial.print("L too close or too Far!\n");
     // Serial.println(measure_L.RangeMilliMeter);
     
      if(EncoderR.curMovement != ReverseMov)
        halt();
        
      if(!overCliff){
        Serial.println("el");
        overCliff = 1;
      }
        
      if(EncoderR.curMovement == Forward || EncoderR.curMovement == Backward){
        backward(80);
        EncoderR.goStraight(Backward, 8.0*distToClicks_s);
      }
      else 
        EncoderR.turnBack();
    }
    
    
    
    
   // Serial.println('2');
  
  
    // Receive and parse Raspberry Pi commands
    if(Serial.available()){
        
      char movement = Serial.read();
      int dist = 0;
      dist = atoi(Serial.readString().c_str());
      int clicks_s = round( dist*distToClicks_s );
      int clicks_r = round( dist*distToClicks_r );

      switch(movement){
        case 'f':
          forward(100);
          sweeper1.startSweep();
          
          if(dist != 0){
            EncoderR.goStraight(Forward, clicks_s);
          }
          else  EncoderR.forrest_gump( Forward );
          
          break;
          
        case 'b':
          backward(100);
          sweeper1.stopSweep();
          
          if(dist != 0){
            EncoderR.goStraight(Backward, clicks_s);
          }
          else  EncoderR.forrest_gump( Backward );
          
          break;
          
        case 'l':
          leftturn(100);
          sweeper1.startSweep();
  
          if(dist != 0){
            EncoderR.revolve(LeftTurn, clicks_r);
          }
          else  EncoderR.forrest_gump( LeftTurn );
          
          break;
          
        case 'r':
          rightturn(100);        
          sweeper1.startSweep();
  
          if(dist != 0){
            EncoderR.revolve(RightTurn, clicks_r);
          }
          else  EncoderR.forrest_gump( RightTurn );
          
          break;
          
        case 'm':
          leftshift(100);
          break;
          
        case 'n':
          rightshift(100);
          break;
          
        case '1':
          MotorWriting(60, 0, 0, 0);
          delay(100);
          MotorWriting(0, 0, 0, 0);
          break;
          
        case '2':
          MotorWriting(0, 60, 0, 0);
          delay(100);
          MotorWriting(0, 0, 0, 0);
          break;
          
        case '3':
          MotorWriting(0, 0, 60, 0);
          delay(100);
          MotorWriting(0, 0, 0, 0);
          break;
          
        case '4':
          MotorWriting(0, 0, 0, 60);
          delay(100);
          MotorWriting(0, 0, 0, 0);
          break;
          
        case 'h':
          halt();
          sweeper1.stopSweep();
          EncoderR.getDistOrDegree();
          EncoderR.curMovement = Halt;
          EncoderR.reset_click();
          Serial.println("c");
          
          break;
          
        case 's': 
          sweeper1.startSweep();
          break;
  
        case 'o':
          LightsOn();
          break;
          
        case 'k':
          LightsOff();
          break;
          
        default:
          break;
      }
    }
  //Serial.println('3');

    UpdateEncoderR();
    
  //Serial.println('4');
    sweeper1.doSweep();
  //Serial.println('5');
    
    UpdateEncoderR();
  //Serial.println('6');
    
    //readTOF_F();
    
    //UpdateEncoderR();
    
    readTOF_R();
 // Serial.println('7');
    
    UpdateEncoderR();
 // Serial.println('8');

    readTOF_L();
  //Serial.println('9');
    
    UpdateEncoderR();
  //  readTOFs();
  //Serial.println('E');
  }
}
