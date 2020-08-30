#define DEBUGTOF 1
#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
//#define LOX_F_ADDRESS 0x34
//#define LOX_R_ADDRESS 0x35
//#define LOX_L_ADDRESS 0x36

// set the pins to shutdown
#define SHT_LOX_F 46
#define SHT_LOX_R 48
#define SHT_LOX_L 50

unsigned int LOX_F_ADDRESS = 0x31;
unsigned int LOX_R_ADDRESS = 0x32;
unsigned int LOX_L_ADDRESS = 0x34;

// objects for the vl53l0x
Adafruit_VL53L0X lox_F = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_R = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_L = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure_F;
VL53L0X_RangingMeasurementData_t measure_R;
VL53L0X_RangingMeasurementData_t measure_L;


int myturn = 0;

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void initTOF()
{
  pinMode(SHT_LOX_F, OUTPUT);
  pinMode(SHT_LOX_R, OUTPUT);
  pinMode(SHT_LOX_L, OUTPUT);
  // all reset
  digitalWrite(SHT_LOX_F, LOW);    
  digitalWrite(SHT_LOX_R, LOW);
  digitalWrite(SHT_LOX_L, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX_F, HIGH);    
  digitalWrite(SHT_LOX_R, HIGH);
  digitalWrite(SHT_LOX_L, HIGH);
  delay(10);
  ///////////////////////////////////////////////
  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX_F, HIGH);
  digitalWrite(SHT_LOX_R, LOW);
  digitalWrite(SHT_LOX_L, LOW);

  // initing LOX1
  if(!lox_F.begin(LOX_F_ADDRESS)) {
   // Serial.print(F("Failed to boot F VL53L0X at address "));
   // Serial.println(LOX_F_ADDRESS);
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
  ///////////////////////////////////////////////
  // activating LOX3
  digitalWrite(SHT_LOX_L, HIGH);
  delay(10);

  //initing LOX3
  while(!lox_L.begin(LOX_L_ADDRESS)) {
    //if(LOX_L_ADDRESS == LOX_F_ADDRESS || LOX_L_ADDRESS == LOX_R_ADDRESS) continue;
    
    Serial.println(F("Failed to boot L VL53L0X at address "));
    Serial.println(LOX_L_ADDRESS);
    //LOX_L_ADDRESS++;
    delay(200);
    

  }
}

void readTOF_F()
{
  lox_F.rangingTest(&measure_F, false);
}

void readTOF_R()
{
  lox_R.rangingTest(&measure_R, false);
}

void readTOF_L()
{
  lox_L.rangingTest(&measure_L, false);
}

void readTOFs() {
  //Serial.println("Enter readTOFs");

  // Take turns to measure distance to reduce main loop time
  if(myturn == 0)
    lox_F.rangingTest(&measure_F, false); // pass in 'true' to get debug data printout!
  else if(myturn == 1)
    lox_R.rangingTest(&measure_R, false); // pass in 'true' to get debug data printout!
  else if(myturn == 2)
    lox_L.rangingTest(&measure_L, false); // pass in 'true' to get debug data printout!
    
  myturn = (myturn + 1) % 3;

  // print sensor F reading
                    #if (DEBUGTOF == 1)
  Serial.print("F: ");
                    #endif


  if(measure_F.RangeStatus != 4) {     // if not out of range
                    #if (DEBUGTOF == 1)
    Serial.print(measure_F.RangeMilliMeter);
                    #endif    
  } 
  else {
    Serial.print("Out of range");
  }
                    #if (DEBUGTOF == 1)
  Serial.print(" ");
                    #endif

  // print sensor R reading
                    #if (DEBUGTOF == 1)
  Serial.print("R: ");
                    #endif
  if(measure_R.RangeStatus != 4) {
                    #if (DEBUGTOF == 1)
    Serial.print(measure_R.RangeMilliMeter);
                    #endif
  } else {
    Serial.print("Out of range");
  }
                    #if (DEBUGTOF == 1)
  Serial.print(" ");
                    #endif
  // print sensor L reading
                    #if (DEBUGTOF == 1)
  Serial.print("L: ");
                    #endif
  if(measure_L.RangeStatus != 4) {
                    #if (DEBUGTOF == 1)
  Serial.print(measure_L.RangeMilliMeter);
                    #endif
  } else {
    Serial.print("Out of range");
  }
                    #if (DEBUGTOF == 1)
  Serial.println();
                    #endif
}
