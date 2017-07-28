/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <AddicoreRFID.h>
#include <SPI.h>
#include "EmonLib.h"                   
#include <SoftwareSerial.h> //import SoftwareSerial lib 


/*-----( Declare Constants )-----*/
#define rxEsp8266Pin 6 
#define txEsp8266Pin 7
#define SSID "38 Apple" // insert your SSID
#define PASS "PTCRUn9f" // insert your password

//#define  uchar unsigned char
//#define uint  unsigned int
//4 bytes tag serial number, the first 5 bytes for the checksum byte
//uchar serNumA[5];

//uchar fifobytes;
//uchar fifoValue;

//AddicoreRFID myRFID; // create AddicoreRFID object to control the RFID module

//const int chipSelectPin = 10;
//const int NRSTPD = 5;

//Maximum length of the array
//#define MAX_LEN 16

/*-----( Declare objects )-----*/
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

EnergyMonitor emon1;                   // Create an instance
SoftwareSerial esp8266Module =  SoftwareSerial(rxEsp8266Pin, txEsp8266Pin);


/*-----( Declare Variables )-----*/
//Sound
int digitalSound = 2; //Pin for Digital Output - DO
int analogSound = A0; // Pin for Analog Output - AO
int threshold = 100; //Set minimum threshold for LED lit

//Vibration
int vibrationINPUT = 3;  
int previousVibration = LOW; 

//Current power draw
int analogCurrentPower = 2;

//Accelerometer
int CS=10; //Assign the Chip Select signal to pin 10.
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1
char values[10]; //This buffer will hold values read from the ADXL345 registers.


int noisevalue;
int vibrationValue;
int accX;
int accY;
int accZ;

double total;
int count;


void setup() {
  //pinMode(vibrationINPUT, INPUT); 
  emon1.current(analogCurrentPower, 60);             // Current: input pin, calibration.
  
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(115200);
  Serial.println("Setting...");

  esp8266Module.begin(9600);

   boolean connected=false;
   for(int i=0;i<5;i++)
   {
      if(connectToWifi(SSID,PASS))
      {
        connected = true;
        break;
      }
    }
  
  /*Accelerometer setup*/
  //pinMode(CS, OUTPUT);
  //digitalWrite(CS, HIGH);
  //writeRegister(DATA_FORMAT, 0x01); //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  //writeRegister(POWER_CTL, 0x08);  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register. 

  /*
  pinMode(chipSelectPin,OUTPUT);              // Set digital pin 10 as OUTPUT to connect it to the RFID /ENABLE pin 
  digitalWrite(chipSelectPin, LOW);         // Activate the RFID reader
  pinMode(NRSTPD,OUTPUT);                     // Set digital pin 10 , Not Reset and Power-down
  digitalWrite(NRSTPD, HIGH);
  */
  
  //myRFID.AddicoreRFID_Init();  
  /*
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); // finish with backlight on  
  */

}

void loop() {
  const unsigned long oneMinute = 1 * 60 * 1000UL;
  static unsigned long lastSampleTime = 0 - oneMinute;  // initialize such that a reading is due the first time through loop()
  
  //noisevalue = analogRead(analogSound);  //Read the analog value
  //vibrationValue = digitalRead(vibrationINPUT); 
  
  double Irms = emon1.calcIrms(1480);  // Calculate Irms only
  double apparentPower = Irms*230.0;
  
  //Print so the output is going to be: vibration(bool);noiselevel(double);apparentPower(double);accX(double);accY(double);accZ(double)
  // If the switch changed, due to bounce or pressing...
  //if (vibrationValue != previousVibration) {
  //  Serial.print(1);
  //} else {
  //  Serial.print(0);    
  //}
  //Serial.print(";"); 
  //Serial.print(noisevalue); 
  
  //Serial.print(";"); 
  //Serial.print(apparentPower); 
  //Serial.print(";");
  //getAccelerometerData();
  //Serial.print(";");
  //Serial.print(millis());
  //Serial.println(";end"); 
  
  //previousVibration = vibrationValue;

  total += apparentPower;
  count += 1;

  unsigned long now = millis();
  if (now - lastSampleTime >= oneMinute)
  {
    lastSampleTime += oneMinute;
    double mean = total / count;
    Serial.println(mean);
    sendInformation("http://kse624project4.herokuapp.com", "/api/v1.0/submit/-KKD6n_yZ9mE6fbMIVpW/" + String(mean,2));
    count = 0;
    total = 0.0;
   }

  /*
  int digitalValue = digitalRead(DO); 
  String stringTop = "Analog: " + sensorvalue;
  String stringBottom = "Digital: " + digitalValue;
  
  if (sensorvalue >= threshold)  {
    lcd.clear();
     lcd.setCursor(0,0); //Start at character 0 on line 0
     lcd.print(sensorvalue);
      delay(1000);
     lcd.setCursor(0,1);
     lcd.print(digitalValue);

  }else {
    //lcd.clear();
  }

  handleRFID();*/

}

/*Wifi methods*/
bool connectToWifi(String networkId, String networkPassword) {
  String cmd = F("AT+CWJAP=\"");
  cmd += networkId;
  cmd += F("\",\"");
  cmd += networkPassword;
  cmd += F("\"");
  Serial.println(cmd);
  esp8266Module.println(cmd);
  delay(5000);
  if (esp8266Module.find("OK"))
  {
    Serial.println(F("CONNECTED TO WIFI"));
    return true;
  }
  else
  {
    Serial.println(F("NOT CONNECTED TO WIFI"));
    return false;
  }
}

bool sendInformation(String parameters) {
  String cmd = F("AT+CIPSTART=\"TCP\",\"");
  cmd += "46.137.127.234"; //Ip adress to the server
  cmd += F("\",80");
  esp8266Module.println(cmd);
  if (esp8266Module.find("CONNECT"))
  {
    Serial.println(F("Connected to server"));
  }else {
    Serial.println(F("NOT connected to server"));
    return false;
  }

  //Start sending get request

  //sendInformation("http://kse624project4.herokuapp.com", "/api/v1.0/submit/-KKD6n_yZ9mE6fbMIVpW/" + String(mean,2) + "/" + String(var_v,2) + "/" + String(max_v,2) + "/" + String(min_v,2));
  
  String first_cmd = "GET https://kse624project4.herokuapp.com" + parameters +" HTTP/1.1";
  String second_cmd = "Host: kse624project4.herokuapp.com";
  cmd  = first_cmd + second_cmd
  
  esp8266Module.print("AT+CIPSEND=");
  esp8266Module.println(cmd.length()+6);

  if (esp8266Module.find(">"))
  {
    Serial.println(F("found > prompt - issuing GET request"));
    //esp8266Module.println(cmd);
    esp8266Module.println(first_cmd);
    esp8266Module.println(second_cmd);
    esp8266Module.println("");
    esp8266Module.println("");
    
    if(esp8266Module.available()) {
      esp8266Module.read();
    }

    return true;
  }
  else
  {
    Serial.println(F("No '>' prompt received after AT+CPISEND"));
    return false;
  }
}

/*Accelerometer*/
void getAccelerometerData() {
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  accX = ((int)values[1]<<8)|(int)values[0];
  //The Y value is stored in values[2] and values[3].
  accY = ((int)values[3]<<8)|(int)values[2];
  //The Z value is stored in values[4] and values[5].
  accZ = ((int)values[5]<<8)|(int)values[4];

  Serial.print(accX, DEC);
  Serial.print(';');
  Serial.print(accY, DEC);
  Serial.print(';');
  Serial.print(accZ, DEC);
}

void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

/*RFID card*/
/*
void handleRFID() {

  uchar i, tmp, checksum1;
  uchar status;
  uchar str[MAX_LEN];
  uchar RC_size;
  uchar blockAddr;  //Selection operation block address 0 to 63
  String mynum = "";

   str[1] = 0x4400;
  
  //Find tags, return tag type
  status = myRFID.AddicoreRFID_Request(PICC_REQIDL, str); 
  if (status == MI_OK)
  {
     Serial.println("RFID tag detected");
     Serial.print(str[0],BIN);
     Serial.print(" , ");
     Serial.print(str[1],BIN);
     Serial.println(" ");

     lcd.clear();
     lcd.setCursor(0,0); //Start at character 0 on line 0
     lcd.print("RFID tag detected");
  }

  //Anti-collision, return tag serial number 4 bytes
  status = myRFID.AddicoreRFID_Anticoll(str);
  if (status == MI_OK)
  {
            checksum1 = str[0] ^ str[1] ^ str[2] ^ str[3];
            Serial.println("The tag's number is  : ");
          //Serial.print(2);
          Serial.print(str[0]);
            Serial.print(" , ");
          Serial.print(str[1],BIN);
            Serial.print(" , ");
          Serial.print(str[2],BIN);
            Serial.print(" , ");
          Serial.print(str[3],BIN);
            Serial.print(" , ");
          Serial.print(str[4],BIN);
            Serial.print(" , ");
            Serial.println(checksum1,BIN);
            
            // Should really check all pairs, but for now we'll just use the first
            if(str[0] == 156)                      //You can change this to the first byte of your tag by finding the card's ID through the Serial Monitor
            {
                Serial.print("Hello Craig!\n");
            } else if(str[0] == 244) {             //You can change this to the first byte of your tag by finding the card's ID through the Serial Monitor
                Serial.print("Hello Erin!\n");
            }
            Serial.println();
            delay(1000);
  }
  myRFID.AddicoreRFID_Halt();      //Command tag into hibernation     
  
}
*/
