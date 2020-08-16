//#include <SoftwareSerial.h>

#define MotorLF_I1      26//I1（left front）
#define MotorLF_I2      28//I2（left front）
#define MotorRF_I3      22//I3（right front）
#define MotorRF_I4      24//I4（right front）
#define MotorLB_I1      36//I1（left back）
#define MotorLB_I2      34//I2（left back）
#define MotorRB_I3      32//I3（right back 
#define MotorRB_I4      30//I4（right back）

#define ENA     2//left front
#define ENB     3//right front
#define ENC     4//left back
#define END     5//right back

#include "track.h"
//#include "communicate.h"
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
  while(Serial.available()){
      
    char command = Serial.read();
    switch(command){
      case 'f':
        forward(120);
        break;
      case 'b':
        backward(120);
        break;
      case 'l':
        leftturn(120);
        break;
      case 'r':
        rightturn(120);
        break;
      case 'm':
        leftshift(120);
        break;
      case 'n':
        rightshift(120);
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
        break;
      default:
        break;
    }
  }
  /*
  if     (ask()==BACKWARD){backward(70);}
  else if(ask()==FORWARD){forward(70);}
  else if(ask()==LEFTTURN){leftturn(70);}
  else if(ask()==RIGHTTURN){rightturn(70);}
  else if(ask()==LEFTSHIFT){leftspin(70);}
  else if(ask()==RIGHTSHIFT){rightspin(70);}
  else if(ask()==HALT){MotorWriting(0, 0, 0, 0);}
  //send_msg('c');
  delay(200);*/
}
