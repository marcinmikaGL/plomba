#include <DFRobot_sim808.h>
#include <math.h>
#include <LowPower.h>
#include <OneWire.h>
#include <SD.h>

#define PIN_PWR 12
#define interval 180000
#define plombNumber 694
//const uint8_t CHIP_SELECT = 10;

OneWire  ds(8);
DFRobot_SIM808 sim808(&Serial);
//-- konfiguracja

String latitude;
String longitude;
String date;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int tempB = 0;
char incomingByte;
char instrukcja[120];
File myFile;


///////////////////////////////////////////////////////////////
//
//                       INITIALIZATION
//
///////////////////////////////////////////////////////////////
void setup() {

  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  Serial.begin(9600);
  delay(1000);
  pinMode(PIN_PWR, OUTPUT);
  sim808.powerUpDown(PIN_PWR);
  while (!SD.begin(10)) //SD card initialization
  {
    delay(100);
  }
  delay(5000);
  myFile = SD.open(F("logi.txt"), FILE_WRITE);
  myFile.println(F("SD INITIALIZATION DONE"));
  myFile.close();
  delay(100);
  //******** Initialize sim808 module *************
  while (!sim808.init()) {
    Clear();
    delay(1000);
  }

  ConfigureGPS(); //GPS initialization
  delay(20000);
  myFile = SD.open(F("logi.txt"), FILE_WRITE);
  myFile.println(F("GPS CONFIGURATION DONE"));
  myFile.close();
  delay(100);
  ConfigureNet(); //net connection initialization
  SendCommand(F("AT+HTTPINIT"), 2000);
  SendCommand(F("AT+HTTPPARA=\"CID\",1"), 2000);
  String message = String(F("\"http://url:90/api/DevicesApi?li=")) + String(plombNumber) + String(F("&md=start\""));
  SendCommand(String(F("AT+HTTPPARA=\"URL\",")) + message, 5000);
  SendCommand(F("AT+HTTPACTION=0"), 5000);
  SendCommand(F("AT+HTTPTERM"), 3000);
  SendCommand(F("AT+CIPCLOSE=0"), 3000);
  delay(100);
  myFile = SD.open(F("logi.txt"), FILE_WRITE);
  myFile.println(F("NET CONFIGURATION DONE"));
  myFile.print(String(F("Dane [czas,bateria,swiatlowod,jakos gsm,jasnosc,temperatura")));
  myFile.println(String(F(",temperatura cpu,dl. geo.,szer.geo.,odpowiedz serwera]")));
  myFile.close();
}


///////////////////////////////////////////////////////////////
//
//                         MAIN LOOP
//
///////////////////////////////////////////////////////////////
void loop()
{
  currentMillis = millis();

  //Jeśli da się pobrać położenie, zrób to i przelicz
  if (sim808.getGPS())
  {
    latitude =  String(sim808.GPSdata.lat, 6);
    longitude = String(sim808.GPSdata.lon, 6);
    date = String(sim808.GPSdata.year) + "-" + RoundInt(sim808.GPSdata.month) + "-" + RoundInt(sim808.GPSdata.day) + "%20" + RoundInt(sim808.GPSdata.hour) + ":" + RoundInt(sim808.GPSdata.minute) + ":" + RoundInt(sim808.GPSdata.second);
    float laf = latitude.toFloat();
    float longf = longitude.toFloat();
    double laffract = fmod(laf, 1) * 10 / 6; //Bez tego myli się o kilka km
    double longffract = fmod(longf, 1) * 10 / 6;
    laf = floor(laf) + (float)laffract;
    longf = floor(longf) + (float)longffract;
    latitude =  String(laf, 6);
    longitude = String(longf, 6);
  }
  //Jeśli minął zadany odstep czasu
  if (currentMillis - previousMillis > interval)
  {
    sim808.detachGPS();
    float batteryLevel = CheckBattery();

    //Wyłączenie, jeśli bateria jest zbyt słaba
    if (batteryLevel < 6.4)
    {
      SendCommand(F("AT+CIPSTART=\"TCP\",\"url\",\"90\""), 5000);
      SendCommand(F("AT+HTTPINIT"), 2000);
      SendCommand(F("AT+HTTPPARA=\"CID\",1"), 2000);
      String message = "\"";
      String(F("\"http://url:90/api/DevicesApi?li=")) + String(plombNumber) + String(F("&md=start\""));
      SendCommand(String(F("\"http://url:90/api/DevicesApi?li=")) + String(plombNumber) + String(F("&md=poweroff\"")), 5000);
      SendCommand(F("AT+HTTPACTION=0"), 5000);
      SendCommand(F("AT+HTTPTERM"), 3000);
      SendCommand(F("AT+CIPCLOSE=0"), 3000);
      Clear();
      myFile = SD.open(F("logi.txt"), FILE_WRITE);
      myFile.println(F("Battery low"));
      myFile.close();
      Clear();
      sim808.powerUpDown(PIN_PWR);
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
      while (true)
      {        
        delay(600000);
      }
    }

    //Światło wewnątrz plomby
    int lightR = analogRead(A1);

    //Temperatury
    if ((int)temp() != 0)
      tempB = (int)temp() ;
    int tempC = (int)GetTemp();
    Clear();
    
    //Siła sygnału GSM
    String quality = SendCommandGetResponse(F("AT+CSQ"), 2000);
    Clear();    
    if (quality.length() > 18) //obcinanie żeby wyjąć CSQ:XX
      quality = quality.substring(10, 17);
    else
      quality=F("CSQ:  ");
    Clear();
    
    //Status światłowodu
    analogRead(A1);
    int result = CheckState(); 
    int result2 = CheckState();
    if (result == 1 || result2 == 1)
      result = 1;

    SendCommand(F("AT+CIPSTART=\"TCP\",\"url\",\"90\""), 5000);
    SendCommand(F("AT+HTTPINIT"), 2000);
    SendCommand(F("AT+HTTPPARA=\"CID\",1"), 2000);
    SendMoje(batteryLevel, result, quality, lightR, tempB, tempC, 3000);
    String sendResult = SendCommandGetResponse(F("AT+HTTPACTION=0"), 5000);
    SendCommand(F("AT+HTTPTERM"), 3000);
    SendCommand(F("AT+CIPCLOSE=0"), 3000);

    previousMillis = currentMillis;
    Clear();
    delay(100);
    myFile = SD.open(F("logi.txt"), FILE_WRITE);
    myFile.print(date + ",");
    myFile.print(String(batteryLevel) + ",");
    myFile.print(String(result) + ",");
    myFile.print(quality + ",");
    myFile.print(String(lightR) + ",");
    myFile.print(String(tempB) + ",");
    myFile.print(String(tempC) + ",");
    myFile.print(longitude + ",");
    myFile.print(latitude + ",");
    myFile.println(sendResult);
    myFile.close();

    if (sendResult.indexOf(F(",200,")) < 0)
    {
      myFile = SD.open(F("logi.txt"), FILE_WRITE);
      myFile.println(F("Reset try"));
      myFile.close();
      sim808.powerUpDown(PIN_PWR);
      delay(50000);
      sim808.powerUpDown(PIN_PWR);
      delay(10000);
      SendCommand(F("AT+CFUN=1,1"), 2000);
      delay(10000);
      byte count=0;
      while (!sim808.init()) {
        count++;        
        Clear();
        myFile = SD.open(F("logi.txt"), FILE_WRITE);
        myFile.println(F("Initialization try"));
        myFile.close();
        delay(10000);
        if(count>20){
          count=0;
          sim808.powerUpDown(PIN_PWR);
          delay(110000);
          sim808.powerUpDown(PIN_PWR);
          delay(10000);
          SendCommand(F("AT+CFUN=1,1"), 2000);
          delay(10000);
        }
      }
      ConfigureGPS(); //GPS initialization
      delay(20000);
      ConfigureNet(); //net connection initialization
      delay(10000);
      SendCommand(F("AT+HTTPINIT"), 2000);
      SendCommand(F("AT+HTTPPARA=\"CID\",1"), 2000);
      String message = String(F("\"http://url:90/api/DevicesApi?li=")) + String(plombNumber) + String(F("&md=reset\""));
      SendCommand(String(F("AT+HTTPPARA=\"URL\",")) + message, 5000);
      SendCommand(F("AT+HTTPACTION=0"), 5000);
      SendCommand(F("AT+HTTPTERM"), 3000);
      SendCommand(F("AT+CIPCLOSE=0"), 3000);
      delay(1000);
      previousMillis = millis();
      currentMillis = previousMillis;
    }

    sim808.attachGPS();
    Clear();
  }
}
int CheckState()
{
  int result = 0;
  delay(50);
  String test = F("test");
  for (int i = 0; i < test.length(); i++)
  {
    char myChar = test.charAt(i);
    for (int j = 7; j >= 0; j--)
    {
      analogRead(A2);
      byte bytes = bitRead(myChar, j);
      if (bytes == 1)
      {
        digitalWrite(6, HIGH);
        digitalWrite(7, HIGH);
        delay(30);
        int val = analogRead(A2);
        if (val < 70)
          return 0;
      }
      else
      {
        digitalWrite(6, LOW);
        digitalWrite(7, LOW);

        delay(30);
        int val = analogRead(A2);
        if (val > 30)
          return 0;
      }
    }
  }
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  return 1;
}

float CheckBattery()
{
  return analogRead(A0) * (10.0 / 1023.0);
}

String RoundInt(int value)
{
  if (value < 10)
    return "0" + String(value);
  return String(value);
}

void SendCommand(String command, int timeout)
{
  Serial.println(command);
  delay(timeout);
  String summ = "";
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    delay(1);
    summ += incomingByte;
  }
  //      if(debugSerialOn)
  //        debugSerial.println(summ);
}

void SendMoje(float battery, int lock, String quality, int swiatlo, int tempb, int tempc, int timeout)
{
  Serial.print(F("AT+HTTPPARA=\"URL\",\"http://url:90/api/DevicesApi?lo="));
  //Serial.print(longitude+"&la="+latitude+"&utc="+year+"-"+month+"-"+day+"%20"+hour+":"+minute+":"+second);
  Serial.print(longitude + "&la=" + latitude + "&u=" + date);
  Serial.print("&b=" + String(battery));
  Serial.print("&s=" + String(swiatlo));
  Serial.print("&f=" + String(lock));
  Serial.print(F("&c=0&p=0"));
  Serial.print("&l=" + String(plombNumber));
  Serial.print("&q=" + quality);
  Serial.print("&t=" + String(tempb));
  Serial.print("&tc=" + String(tempc));
  Serial.println("\"");
  delay(timeout);
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
  }
}



String SendCommandGetResponse(String command, int timeout)
{
  Clear();
  Serial.println(command);
  delay(timeout);
  String summ = "";
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    delay(1);
    summ += incomingByte;
  }
  Clear();
  return summ;
}


//Konfiguracja GPS
void ConfigureGPS()
{
  while (!sim808.attachGPS()) {
    delay(1000);
  }
  Clear();
}

//Konfiguracja sieci - połączenia GPRS
void ConfigureNet()
{
  SendCommand(F("AT+CGREG=1,5"), 5000);
  SendCommand(F("AT+SABR=0,1"), 5000);
  SendCommand(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""), 2000);
  SendCommand(F("AT+SAPBR=3,1,\"APN\",\"Internet\""), 2000);
  SendCommand(F("AT+SAPBR=1,1"), 10000);
  SendCommand(F("AT+SAPBR=2,1"), 2000);
}

void Clear()
{
  while (Serial.read() > 0)
  {}
}

float temp() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;

  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
  }


  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(800);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);

  for ( i = 0; i < 9; i++) {
    data[i] = ds.read();
  }


  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  return celsius;
}

double GetTemp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celsius.
  return (t);
}
