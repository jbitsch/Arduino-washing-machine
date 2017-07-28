/*-----( Import needed libraries )-----*/
#include <Wire.h>  // Comes with Arduino IDE
#include <SPI.h>
#include "EmonLib.h"                   
#include <SoftwareSerial.h>
#include "Statistic.h"

/*-----( Declare Constants )-----*/
#define rxEsp8266Pin 6 
#define txEsp8266Pin 7
#define analogCurrentPower 2
#define SSID "" // insert your SSID
#define PASS "" // insert your password

/*-----( Declare objects )-----*/
EnergyMonitor emon1;                   // Create an instance
SoftwareSerial esp8266Module =  SoftwareSerial(rxEsp8266Pin, txEsp8266Pin);
Statistic myStats;

void setup() {
  emon1.current(analogCurrentPower, 60);             // Current: input pin, calibration.
  
  //Initiate an SPI communication instance.
  SPI.begin();

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

    myStats.clear(); //explicitly start clean
}

void loop() {
  const unsigned long oneMinute = 1 * 60 * 1000UL;
  static unsigned long lastSampleTime = 0 - oneMinute;  // initialize such that a reading is due the first time through loop()
  
  double Irms = emon1.calcIrms(1480);  // Calculate Irms only
  double apparentPower = Irms*230.0;

  myStats.add(apparentPower);

  unsigned long now = millis();
  
  if (now - lastSampleTime >= oneMinute) //myStats.count()
  {
    lastSampleTime += oneMinute;
    double mean = myStats.average();
    double minimum = myStats.minimum();
    double maximum = myStats.maximum();
    double variance = myStats.variance();
    
    sendInformation("/api/v1.0/submit/-KKD6n_yZ9mE6fbMIVpW/" + String(mean,2) +"/" + String(variance,2) +"/" + String(maximum,2) +"/" + String(minimum,2));
    //clear the stats..
    myStats.clear();
    
   }

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
  String first_cmd = "GET https://kse624project4.herokuapp.com" + parameters +" HTTP/1.1";
  String second_cmd = "Host: kse624project4.herokuapp.com";
  cmd  = first_cmd + second_cmd;
  
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
