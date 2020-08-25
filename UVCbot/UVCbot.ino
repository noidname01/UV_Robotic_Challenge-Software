#include <Servo.h>
#include <string.h>

//#include <SoftwareSerial.h>

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

#include "track.h"
#include "encoder_class.h"
#include "servo_sweeper.h"
#include "tof.h"

Encoder EncoderR(EncoderR_pin, 10, 36);

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

void setup(){
  Serial.begin(9600);
  Serial.println("hi");
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

  LightsOff();

  sweeper1.Attach(ServoPin);
  initTOF();
}


void loop() {
  /*
  forward(70);
  delay(3000);
  backward(70);
  delay(3000);
  leftturn(70);
  delay(10000);
  rightturn(70);
  delay(10000);
 // leftspin(70);
 // delay(3000);
 // rightspin(70);
 // delay(3000);
  MotorWriting(0, 0, 0, 0);
  delay(2000);
  */
//  Serial.println('S');
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
    else
      Serial.print("F normal\n");
  }
  /*
  else {
    Serial.print("F normal\n");
    Serial.println(measure_F.RangeMilliMeter);
  }
  */  
  if(measure_R.RangeMilliMeter < 80 || measure_R.RangeMilliMeter > 120 ){
    if(measure_R.RangeStatus != 4) {
      //Serial.print("R too close or too Far!\n");
      //Serial.println(measure_R.RangeMilliMeter);
      //halt();
    }
    else
      Serial.print("R normal\n");
  }
  
  if(measure_L.RangeMilliMeter < 80 || measure_L.RangeMilliMeter > 120){
    if(measure_L.RangeStatus != 4) {
     // Serial.print("L too close or too Far!\n");
     // Serial.println(measure_L.RangeMilliMeter);
     // halt();
    }
    else
      Serial.print("L normal\n");
  }
  
  //Serial.println('2');


  
  if(Serial.available()){
      
    char movement = Serial.read();
    int dist = 0;
    dist = atoi(Serial.readString().c_str());
    int clicks_s = round( dist*1.0/ (PI*5.67)*36 );
    int clicks_r = round( dist*0.68);
    
    switch(movement){
      case 'f':
        forward(100);
        sweeper1.startSweep();
        
        if(dist != 0){
          EncoderR.goStraight(clicks_s);
        }
        else  EncoderR.forrest_gump();
        
        break;
        
      case 'b':
        backward(100);
        sweeper1.stopSweep();
        
        if(dist != 0){
          EncoderR.goStraight(clicks_s);
        }
        else  EncoderR.forrest_gump();
        
        break;
        
      case 'l':
        leftturn(100);
        sweeper1.startSweep();

        if(dist != 0){
          EncoderR.revolve(clicks_r);
        }
        else  EncoderR.forrest_gump();
        
        break;
        
      case 'r':
        rightturn(100);        
        sweeper1.startSweep();

        if(dist != 0){
          EncoderR.revolve(clicks_r);
        }
        else  EncoderR.forrest_gump();
        
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

        EncoderR.reset_click();
        
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
  if( EncoderR.report() ){
    halt();
    sweeper1.stopSweep();
    
    Serial.println('c');
  }
  //Serial.println('4');
  sweeper1.doSweep();
//  Serial.println('5');
  //readTOFs();
//  Serial.println('E');
}
