#include <quaternionFilters.h>
#include <MPU9250.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <SD.h>
#include <config.h>
#include <TimeLib.h>

//Declare Variables
const long logInterval = 100; //interval in milisec to log/stream data
long lastLogTime = 0;
const long wifiReconnInterval = 30000; //Interval for scanning the network and trying to reconnect
long lastWifiReconn = 0;
bool ledState = 0;
bool MPUonline = 0;
int wifiRetry;
String stringOne;
char charBuf[96];
IPAddress timeServerIP;
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
String logFile;
byte secondDS = 0;
byte minuteDS = 0;
byte hourDS = 0;
byte dayOfWeekDS = 0;
byte dayOfMonthDS = 0;
byte monthDS = 0;
byte yearDS = 0;
#define FTPWRITE
char outBuf[128];
char outCount;
//FTP stuff
const char* host = "172.16.0.12";
const char* userName = "telem1";
const char* passwordFTP = "telem12345";
String fileName;

// Pin definitions
const int intPin = D6;  // These can be changed, 2 and 3 are the Arduinos ext int pins
const int outLed = D1; //Led used for signaling stuff
const int chipSelect = D8;  //used for CS SDcard
const int i2cSDA = D2; //i2c SDA pin
const int i2cSCL = D4; //i2c SCL pin

//Configuration parameters:
const IPAddress IP_Remote(172, 16, 0, 10); //IP address to stream live data to
const char* ntpServerName = "ro.pool.ntp.org"; //NTP time server
const unsigned int localUdpPort = 4210; //UDP local port
const bool stream = 0; //if set to 1 start streaming UDP to IP_Remote
const bool logToSD = 1; //if set to 1 start logging data to SD card
const bool wifiON = 1; //if 0 then don't connect to WiFi
#define SerialDebug true  // Set to true to get Serial output for debugging

#define DS3231_I2C_ADDRESS 0x68

//Initialize libraries
WiFiUDP Udp;
MPU9250 myIMU;
// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;
WiFiClient client;
WiFiClient dclient;


void setup()
{
  Serial.begin(38400);
  pinMode(outLed, OUTPUT);

  //Initialize i2c pins
  Wire.begin(i2cSDA,i2cSCL);

  //Connect to Wifi
  if (wifiON) {
    connWiFi();
    delay(500);
  }

  // set the initial time here:
  // DS3231 seconds, minutes, hours, day, date, month, year
  // setDS3231time(30,21,00,5,25,02,17);
  readDS3231time(&secondDS, &minuteDS, &hourDS, &dayOfWeekDS, &dayOfMonthDS, &monthDS,  &yearDS);
  if (SerialDebug) {
    Serial.print("DS3231 time: ");
    Serial.print(hourDS, DEC);
    Serial.print(":");
    Serial.print(minuteDS, DEC);
    Serial.print(":");
    Serial.print(secondDS, DEC);
    Serial.print(" Weekday:");
    Serial.print(dayOfWeekDS, DEC);
    Serial.print(" Day:");
    Serial.print(dayOfMonthDS, DEC);
    Serial.print(" Month:");
    Serial.print(monthDS, DEC);
    Serial.print(" Year:");
    Serial.println(yearDS, DEC);
  }
  setTime(hourDS,minuteDS,secondDS,dayOfMonthDS,monthDS,yearDS);
  time_t t = now();
  if (SerialDebug){
    Serial.print("System time: ");
    Serial.print(hour(t));
    Serial.print(":");
    Serial.print(minute(t));
    Serial.print(":");
    Serial.print(second(t));
    Serial.print(" Weekday:");
    Serial.print(weekday(t));
    Serial.print(" Day:");
    Serial.print(day(t));
    Serial.print(" Month:");
    Serial.print(month(t));
    Serial.print(" Year:");
    Serial.println(year(t) - 2000);
  }
  //Query NTP and set local time
  queryNTP();


  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(outLed, OUTPUT);
  digitalWrite(outLed, HIGH);

  //Initialize the MPU 9250
  startMPU9250();

  //If we log to SD card then initialize it
  if(logToSD){
    initSDcard();
    logFile = String(now()).substring(0,7) + ".log";
    Serial.print("Writing to file: ");
    Serial.println(logFile);
    }
}

void loop()
{
  if (millis() - lastWifiReconn >= wifiReconnInterval) {
    //Scan for our SSID and if in range Connect to Wifi
    scanForSSID();
    lastWifiReconn = millis();
  }

  if (MPUonline){
    getMPU9250values();
  }

  stringOne = String(now()) + ",";
  stringOne = stringOne + String((int)1000*myIMU.ax) + "," + String((int)1000*myIMU.ay) + "," + String((int)1000*myIMU.az) + ",";
  stringOne = stringOne + String(myIMU.gx) + "," + String(myIMU.gy) + "," + String(myIMU.gz) + ",";
  stringOne = stringOne + String(myIMU.mx) + "," + String(myIMU.my) + "," + String(myIMU.mz);
  stringOne.toCharArray(charBuf, 96);

  if (millis() - lastLogTime >= logInterval) {
    if (logToSD) {
      File dataFile = SD.open(logFile, FILE_WRITE);
      dataFile.println(charBuf);
      dataFile.close();
      if(SerialDebug) {
        //Serial.println(charBuf);
      }
    }
    //start UDP stream if active
    if (stream) {
      Udp.beginPacket(IP_Remote,4445);
      Udp.write(charBuf);
      Udp.endPacket();
      }
    lastLogTime = millis();
    toggleLed();
    if(SerialDebug) {
      //Serial.println(charBuf);
    }
  }
}


//Start functions definition
void connWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  for (wifiRetry = 0; wifiRetry < 10; wifiRetry++) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiRetry = 0;
      Serial.println();
      Serial.print("Connected, IP address: ");
      Serial.println(WiFi.localIP());
      break;
    }
    delay(500);
    Serial.print(".");
  }

}

void scanForSSID() {
  if(WiFi.status() != WL_CONNECTED){
    if(WiFi.scanComplete() == -2) {
      Serial.println("Starting network scan....");
      WiFi.scanNetworks(1, 1);
    }

    int networksFound = WiFi.scanComplete();
    if(networksFound > 0){
      for (int i = 0; i < networksFound; i++) {
        if(WiFi.SSID(i) == String(ssid)) {
          Serial.print(ssid);
          Serial.println(" is in range");
          connWiFi();
          queryNTP ();
        }
      }
      WiFi.scanDelete();
    }
  }
}

void toggleLed() {
  if (ledState) {
    digitalWrite(outLed, HIGH);
    ledState = 0;
  }
  else {
    digitalWrite(outLed, LOW);
    ledState = 1;
  }
}

void startMPU9250 () {
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");
    MPUonline = 1;

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    //while(1) ; // Loop forever if communication doesn't happen
  }
  delay(3000);
  Serial.println("ax,ay,az,gx,gy,gz,mx,my,mz,q0,qx,qy,qz,yaw,pitch,roll,rate");
}

void getMPU9250values() {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);


  // Serial print and/or display at 0.5 s rate independent of data rates
  myIMU.delt_t = millis() - myIMU.count;

  // print to serial once per second independent of read rate
  if (myIMU.delt_t > 1000)
  {
    if(SerialDebug)
    {
      Serial.print(now());
      Serial.print(" ");
      Serial.print((int)1000*myIMU.ax);
      Serial.print("  ");
      Serial.print((int)1000*myIMU.ay);
      Serial.print("  ");
      Serial.print((int)1000*myIMU.az);
      Serial.print("  | ");
      Serial.print(myIMU.gx, 2);
      Serial.print("  ");
      Serial.print(myIMU.gy, 2);
      Serial.print("  ");
      Serial.print(myIMU.gz, 2);
      Serial.print("  | ");
      Serial.print((int)myIMU.mx);
      Serial.print("  ");
      Serial.print((int)myIMU.my);
      Serial.print("  ");
      Serial.print((int)myIMU.mz);
      Serial.print("  |  ");
      Serial.print(*getQ());
      Serial.print("  ");
      Serial.print(*(getQ() + 1));
      Serial.print("  ");
      Serial.print(*(getQ() + 2));
      Serial.print("  ");
      Serial.print(*(getQ() + 3));
      Serial.print("  |  ");
      Serial.print(myIMU.yaw, 2);
      Serial.print("  ");
      Serial.print(myIMU.pitch, 2);
      Serial.print("  ");
      Serial.print(myIMU.roll, 2);
      Serial.print("  ");
      Serial.println((float)myIMU.sumCount/myIMU.sum, 2);
      Serial.printf("Settings heap size: %u\n", ESP.getFreeHeap());
    }
    myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                  *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
    myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                  *(getQ()+2)));
    myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                  *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                  - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
    myIMU.pitch *= RAD_TO_DEG;
    myIMU.yaw   *= RAD_TO_DEG;
    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    myIMU.yaw   -= 8.5;
    myIMU.roll  *= RAD_TO_DEG;
    if(SerialDebug)
    {
      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(myIMU.yaw, 2);
      Serial.print(", ");
      Serial.print(myIMU.pitch, 2);
      Serial.print(", ");
      Serial.println(myIMU.roll, 2);
      Serial.print("rate = ");
      Serial.print((float)myIMU.sumCount/myIMU.sum, 2);
      Serial.println(" Hz");
    }
    myIMU.count = millis();
    myIMU.sumCount = 0;
    myIMU.sum = 0;

  }
}

void initSDcard() {
  if (logToSD){
  Serial.print("\nInitializing SD card...");
    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!card.init(SPI_HALF_SPEED, chipSelect)) {
      Serial.println("initialization failed. No SD logging.");
      digitalWrite(outLed, HIGH);
      return;
    } else {
      Serial.println("Wiring is correct and a card is present.");
    }

    // print the type of card
    Serial.print("\nCard type: ");
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
      break;
      case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
      break;
      case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
      default:
        Serial.println("Unknown");
      }

      // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
      if (!volume.init(card)) {
        Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
        return;
      }


      // print the type and size of the first FAT-type volume
      uint32_t volumesize;
      Serial.print("\nVolume type is FAT");
      Serial.println(volume.fatType(), DEC);
      Serial.println();

      volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
      volumesize *= volume.clusterCount();       // we'll have a lot of clusters
      volumesize *= 512;                            // SD card blocks are always 512 bytes
      Serial.print("Volume size (bytes): ");
      Serial.println(volumesize);
      Serial.print("Volume size (Kbytes): ");
      volumesize /= 1024;
      Serial.println(volumesize);
      Serial.print("Volume size (Mbytes): ");
      volumesize /= 1024;
      Serial.println(volumesize);


      Serial.println("\nFiles found on the card (name, date and size in bytes): ");
      root.openRoot(volume);

      // list all files in the card with date and size
      root.ls(LS_R | LS_DATE | LS_SIZE);

      if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        return;
      }
      Serial.println("card initialized.");
    }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

//Get NTP time and update the DS3231 RTC
void queryNTP (){
  if(WiFi.status() != WL_CONNECTED){
    Serial.println("We are not connected to WiFi, not syncing to NTP.");
    return;
  }
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  Serial.print("Resolving: ");
  Serial.print(ntpServerName);
  Serial.print(" -> ");
  Serial.println(timeServerIP);

  Serial.println("Starting UDP");
  Udp.begin(localUdpPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);
  int cb;
  int ntpRetries = 7;
  Serial.print("Waiting for NTP server");
  while (!cb && ntpRetries > 0) {
    Serial.print(".");
    cb = Udp.parsePacket();
    delay(500);
    ntpRetries--;
  }
  if(cb) {
    Serial.print("packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = " );
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if ( ((epoch % 3600) / 60) < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ( (epoch % 60) < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second

    //Set system time
    setTime(epoch);

    Serial.print("System time is: ");
    Serial.println(now());
    time_t t = now();
    setDS3231time(second(t),minute(t),hour(t),weekday(t),day(t),month(t),year(t) - 2000);
    Serial.print(second(t));
    Serial.print(hour(t));
    Serial.print(minute(t));
    Serial.print(weekday(t));
    Serial.print(day(t));
    Serial.print(month(t));
    Serial.println(year(t) - 2000);
  }
  Serial.println("");
}

void countMillis() {
  //Will be used to add millilisec support
}

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

//DS3231 functions
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte
dayOfMonth, byte month, byte year)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}
void readDS3231time(byte *second,byte *minute,byte *hour,byte *dayOfWeek,byte *dayOfMonth,byte *month,byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

//display time from DS3231
void displayTime()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,  &year);
  // send it to the serial monitor
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (minute<10)
  {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second<10)
  {
    Serial.print("0");
  }
  Serial.print(second, DEC);
  Serial.print(" ");
  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
  Serial.print(" Day of week: ");
  switch(dayOfWeek){
  case 1:
    Serial.println("Sunday");
    break;
  case 2:
    Serial.println("Monday");
    break;
  case 3:
    Serial.println("Tuesday");
    break;
  case 4:
    Serial.println("Wednesday");
    break;
  case 5:
    Serial.println("Thursday");
    break;
  case 6:
    Serial.println("Friday");
    break;
  case 7:
    Serial.println("Saturday");
    break;
  }
}

byte doFTP()
{
    File fh = SD.open(fileName, FILE_READ);
    if (!fh) {
      Serial.println("file open failed");
    }
  if (client.connect(host,21)) {
    client.setNoDelay(1);
    Serial.println(F("Command connected"));
  }
  else {
    fh.close();
    Serial.println(F("Command connection failed"));
    return 0;
  }
  if(!eRcv()) return 0;
  client.print("USER ");
  client.println(userName);
  if(!eRcv()) return 0;
  client.print("PASS ");
  client.println(passwordFTP);
  if(!eRcv()) return 0;
  client.println("SYST");
  if(!eRcv()) return 0;
  client.println("Type I");
  if(!eRcv()) return 0;
  client.println("PASV");
  if(!eRcv()) return 0;
  char *tStr = strtok(outBuf,"(,");
  int array_pasv[6];
  for ( int i = 0; i < 6; i++) {
    tStr = strtok(NULL,"(,");
    array_pasv[i] = atoi(tStr);
    if(tStr == NULL)
    {
      Serial.println(F("Bad PASV Answer"));

    }
  }

  unsigned int hiPort,loPort;
  hiPort=array_pasv[4]<<8;
  loPort=array_pasv[5]&255;
  Serial.print(F("Data port: "));
  hiPort = hiPort|loPort;
  Serial.println(hiPort);
  if(dclient.connect(host, hiPort)){
    Serial.println("Data connected");
  }
  else{
    Serial.println("Data connection failed");
    client.stop();
    fh.close();
  }
  client.print("STOR ");
  client.println(fileName);
  if(!eRcv())
  {
    dclient.stop();
    return 0;
  }
  Serial.println(F("Writing"));
  byte clientBuf[64];
  int clientCount = 0;
  while(fh.available())
  {
    clientBuf[clientCount] = fh.read();
    clientCount++;

    if(clientCount > 63)
    {
      dclient.write((const uint8_t *)clientBuf, 64);
      clientCount = 0;
    }
  }
  if(clientCount > 0) dclient.write((const uint8_t *)clientBuf, clientCount);
  dclient.stop();
  Serial.println(F("Data disconnected"));
  client.println();
  if(!eRcv()) return 0;
  client.println("QUIT");
  if(!eRcv()) return 0;
  client.stop();
  Serial.println(F("Command disconnected"));
  fh.close();
  Serial.println(F("File closed"));
  return 1;
}

byte eRcv()
{
  byte respCode;
  byte thisByte;
  while(!client.available()) delay(1);
  respCode = client.peek();
  outCount = 0;
  while(client.available())
  {
    thisByte = client.read();
    Serial.write(thisByte);

    if(outCount < 127)
    {
      outBuf[outCount] = thisByte;
      outCount++;
      outBuf[outCount] = 0;
    }
  }
  if(respCode >= '4')
  {
    efail();
    return 0;
  }

  return 1;
}

void efail()
{
  byte thisByte = 0;
  client.println(F("QUIT"));
  while(!client.available()) delay(1);
  while(client.available())
  {
    thisByte = client.read();
    Serial.write(thisByte);
  }
  client.stop();
  Serial.println(F("Command disconnected"));
}

//Upload all LOG files to FTP server; the remove function does not work
void uploadFiles() {
  SD.begin(chipSelect);
  File root = SD.open("/");
  delay(1000);
  while(true) {
    File entry = root.openNextFile();
    if (! entry) {
      // no more files
      Serial.println("Done uploading files.");
      break;
    }
    fileName = entry.name();
    long fileSize = entry.size();
    Serial.println(fileName);
    //Get the index of the dot
    int dotPos = fileName.indexOf(".");
    //Check if the file is a LOG file and upload it
    if (fileName.substring(dotPos + 1) == "LOG") {
      Serial.print("Uploading: ");
      Serial.print(fileName);
      Serial.print("\t\t Size: ");
      Serial.println(fileSize, DEC);
      if(doFTP()) {
        entry.close();
        Serial.println(F("FTP OK"));
        Serial.print("Deleting: ");
        Serial.println(fileName);
        fileName = fileName + '\0';
        char charBuf[fileName.length()];
        fileName.toCharArray(charBuf, fileName.length());
        if (SD.remove(charBuf)) {
          Serial.print("Deleted ");
          Serial.println(fileName);
        }
      else Serial.println(F("FTP FAIL"));
      }
    }
  }
}
