#include <Adafruit_MCP9808.h>
#include <Wire.h>
#include <ModbusRtu.h>
#include  <MCP342X.h>

#define debug false

typedef struct {
    uint8_t pinNum;
    bool pinVal;
    uint16_t r_value;
} pinInit_t;

typedef struct {
    pinInit_t pins[];
    uint8_t powerPCT;
} heaterPower;

pinInit_t optoPins[] {
    {3, LOW, 0},
    // {4, LOW, 806}, //changed to 51k to lower min power, though, now smallest power increment is doubled....
    {5, LOW, 1500},
    {6, LOW, 3160},
    {7, LOW, 6800},
    {8, LOW, 13300},
    {9, LOW, 27000},
    {10, LOW, 51000},
    {4, LOW, 51000}    
};

void pct2opto(uint8_t pct){
  uint8_t scaled = pct*255/100;
  uint8_t twoKeeper = 1;
  for (uint8_t i = 1; i<8; i++){
    twoKeeper*=2;
      if(scaled & twoKeeper){
         digitalWrite(optoPins[i].pinNum, HIGH);        
      }else{digitalWrite(optoPins[i].pinNum, LOW);}
  }
}

void heaterState(bool on){
  if(on){digitalWrite(optoPins[0].pinNum, HIGH);}
  else{digitalWrite(optoPins[0].pinNum, LOW);}
}


// data array for modbus network sharing
//[2] is used for BTtemperature, [4] for heater power 0-100, [5] for heater on/off 0-1
uint16_t au16data[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1 };  

Modbus slave(1,0,0);  

MCP342X myADC(0x69);
Adafruit_MCP9808  tempSensor;

// coefficients for inverse lookup (given mV, find C)
const double Kcoeff_inv[10][3] = {
         { 0.0000000E+00,  0.000000E+00, -1.318058E+02 },
         { 2.5173462E+01,  2.508355E+01,  4.830222E+01 }, 
         { -1.1662878E+00,  7.860106E-02, -1.646031E+00 },
         { -1.0833638E+00, -2.503131E-01,  5.464731E-02 },
         { -8.9773540E-01,  8.315270E-02, -9.650715E-04 },
         { -3.7342377E-01, -1.228034E-02,  8.802193E-06 },
         { -8.6632643E-02,  9.804036E-04, -3.110810E-08 },
         { -1.0450598E-02, -4.413030E-05,  0.000000E+00 },
         { -5.1920577E-04,  1.057734E-06,  0.000000E+00 },
         { 0.0000000E+00, -1.052755E-08,  0.000000E+00 }
};

// coefficients for direct lookup (given C, find mV)
const double Kcoeff_dir[11][2] = {
         {  0.000000000000E+00, -0.176004136860E-01 },
         {  0.394501280250E-01,  0.389212049750E-01 },
         {  0.236223735980E-04,  0.185587700320E-04 },
         { -0.328589067840E-06, -0.994575928740E-07 },
         { -0.499048287770E-08,  0.318409457190E-09 },
         { -0.675090591730E-10, -0.560728448890E-12 },
         { -0.574103274280E-12,  0.560750590590E-15 },
         { -0.310888728940E-14, -0.320207200030E-18 },
         { -0.104516093650E-16,  0.971511471520E-22 },
         { -0.198892668780E-19, -0.121047212750E-25 },
         { -0.163226974860E-22,  0.0  }
};

double _poly( float x, const double* coeff, uint8_t cols, uint8_t rows ) {
  float temp = 0;
  float mult = 1;
  for (int i = 0; i < rows; i++) {
    temp += mult *  *(coeff+i*cols);
    mult *= x;
  }
  return temp;
}

double getCfromMV(float mv){
  return _poly( mv, &Kcoeff_inv[0][1], 3,10);
}

double getMVfromC(float C){
  return _poly( C, &Kcoeff_dir[0][1], 2,11);
}

void setup() {
  if (debug) {
    Serial.begin(9600);
    Serial.println("begin");    
  }else{
    slave.begin(19200);   // 19200 baud, 8-bits, non, 1-bit stop
  }

  uint8_t i; //Init optocouplers
  for( i = 0; i < sizeof(optoPins)/sizeof(pinInit_t); ++i ){
    pinMode(optoPins[i].pinNum, OUTPUT);
    digitalWrite(optoPins[i].pinNum, optoPins[i].pinVal);
  } 

  heaterState(false); //turn heater off to start, turn on in Artisan software

  Wire.begin();  // join I2C bus
  TWBR = 12;  // 400 kHz (maximum)
  
    
  if (!tempSensor.begin()) { //init temp sensor
    while (1) { delay(10); }
  }

  myADC.configure( 0x80 | MCP342X_MODE_CONTINUOUS | //init ADC for K thermocouple
                   MCP342X_CHANNEL_1 |
                   MCP342X_SIZE_16BIT |
                   MCP342X_GAIN_8X
                 );

  delay(500);
}

void loop() {
  static int16_t result; //get mV from K thermocouple
  myADC.startConversion();
  myADC.getResult(&result);
  float mv = result*62.5E-3/8.;

  sensors_event_t event;  //get on PCB temp
  tempSensor.getEvent(&event);
  uint16_t coldTemp = event.temperature;
  
  double temp = getCfromMV(mv+getMVfromC(coldTemp)); //Calculate cold joint compensated K thermocouple Temp.
  au16data[2] = temp*100; 

  if (debug){
    Serial.print("Cold temp: "); Serial.println(event.temperature);
    Serial.print("Probe temp: "); Serial.println(temp);
    Serial.println();
    digitalWrite(optoPins[0].pinNum, HIGH);
    delay(2000);
    for(uint8_t i=0;i<101;i+=5){
      pct2opto(i);
      Serial.println(i);
      delay(1000);      
    }
    delay(2000);  
    digitalWrite(optoPins[0].pinNum, LOW);
  }else{
    slave.poll( au16data, 16 ); 
    pct2opto(au16data[4]); 
    heaterState(au16data[5]);
  }
}

