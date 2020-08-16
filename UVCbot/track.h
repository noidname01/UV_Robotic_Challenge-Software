//#include <SoftwareSerial.h>
#include <Wire.h>

void MotorWriting(double vLF, double vRF, double vLB, double vRB){
  if(vRF<0){
    vRF = -vRF;
    digitalWrite(MotorRF_I3, HIGH);
    digitalWrite(MotorRF_I4, LOW);
    }
  else{
    digitalWrite(MotorRF_I3, LOW);
    digitalWrite(MotorRF_I4, HIGH);
    }
  if(vLF<0){
    vLF = -vLF;
    digitalWrite(MotorLF_I1, LOW);
    digitalWrite(MotorLF_I2, HIGH);
    }
  else{
    digitalWrite(MotorLF_I1, HIGH);
    digitalWrite(MotorLF_I2, LOW);
    }
  if(vRB<0){
    vRB = -vRB;
    digitalWrite(MotorRB_I3, HIGH);
    digitalWrite(MotorRB_I4, LOW);
    }
  else{
    digitalWrite(MotorRB_I3, LOW);
    digitalWrite(MotorRB_I4, HIGH);
    }
  if(vLB<0){
    vLB = -vLB;
    digitalWrite(MotorLB_I1, LOW);
    digitalWrite(MotorLB_I2, HIGH);
    }
  else{
    digitalWrite(MotorLB_I1, HIGH);
    digitalWrite(MotorLB_I2, LOW);
    }
  analogWrite(ENA, vLF);
  analogWrite(ENB, vRF);
  analogWrite(ENC, vLB);
  analogWrite(END, vRB);
  }


void forward(double v){MotorWriting(v, v, v, v);}
void backward(double v){MotorWriting(-v, -v, -v, -v);}
void leftturn(double v){MotorWriting(v, -v, -v, v);}
void rightturn(double v){MotorWriting(-v, v, v, -v);}
void leftshift(double v){MotorWriting(v, -v, v, -v);}
void rightshift(double v){MotorWriting(-v, v, -v, v);}
void halt(){ MotorWriting(0, 0, 0, 0); }

/*
void MotorWriting(double vLF, double vRF){
  if(vRF<0){
    vRF = -vRF;
    digitalWrite(MotorRF_I3, HIGH);
    digitalWrite(MotorRF_I4, LOW);
    }
  else{
    digitalWrite(MotorRF_I3, LOW);
    digitalWrite(MotorRF_I4, HIGH);
    }
  if(vLF<0){
    vLF = -vLF;
    digitalWrite(MotorLF_I1, LOW);
    digitalWrite(MotorLF_I2, HIGH);
    }
  else{
    digitalWrite(MotorLF_I1, HIGH);
    digitalWrite(MotorLF_I2, LOW);
    }
  
  analogWrite(ENA, vLF);
  analogWrite(ENB, vRF);
  
  }


void foward(double v){MotorWriting(v, v);}
void backward(double v){MotorWriting(-v, -v);}
void leftturn(double v){MotorWriting(v, -v);}
void rightturn(double v){MotorWriting(-v, v);}
void rotate(double v){MotorWriting(v, -v, v, -v);}
*/
