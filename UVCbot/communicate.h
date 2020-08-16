#include<SoftwareSerial.h>
enum CMD {FORWARD, BACKWARD, LEFTTURN, RIGHTTURN, LEFTSPIN, RIGHTSPIN, HALT};

CMD ask(){
  CMD message;
  char cmd;
  if(Serial.available()){
    cmd = Serial.read();
    if     (cmd =='f'){message = FORWARD;}
    else if(cmd =='b'){message = BACKWARD;} 
    else if(cmd =='l'){message = LEFTTURN;}
    else if(cmd =='r'){message = RIGHTTURN;}
    else if(cmd =='1'){message = LEFTSPIN;}
    else if(cmd =='2'){message = RIGHTSPIN;}
    else if(cmd =='h'){message = HALT;}
    Serial.println(message);
  }
    #ifdef DEBUG
    Serial.print("cmd : ");
    Serial.println(cmd);
    #endif
  return message;
  }

void send_msg(const char& msg){
    Serial.write(msg);
    // Serial.print("I send: ");
    // Serial.println(msg);
  }
