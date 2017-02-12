#include <quaternionFilters.h>
#include <MPU9250.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <SPI.h>
#include <SD.h>
#include <config.h>

// Pin definitions
const int intPin = D6;  // These can be changed, 2 and 3 are the Arduinos ext int pins
const int outLed = D1; //Led used for signaling stuff
const int chipSelect = D8;  //used for CS SDcard
const int i2cSDA = D4; //i2c SDA pin
const int i2cSCL = D2; //i2c SCL pin

//Declare Variables
const long logInterval = 60; //interval in milisec to log/stream data
long lastLogTime = 0;
bool ledState = 0;
int wifiRetry;
String stringOne;
char charBuf[96];

//Configuration parameters:
const IPAddress IP_Remote(172, 16, 0, 10); //IP address to stream data to
const unsigned int localUdpPort = 4210; //UDP port to stream data
const bool stream = 1; //if set to 1 start streaming UDP to IP_Remote
const bool logToSD = 1; //if set to 1 start logging data to SD card
#define SerialDebug true  // Set to true to get Serial output for debugging

//Initialize libraries
WiFiUDP Udp;
MPU9250 myIMU;
// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

void setup()
{
  Serial.begin(38400);
  pinMode(outLed, OUTPUT);
  connWiFi();
  Wire.begin(i2cSDA,i2cSCL);


  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(outLed, OUTPUT);
  digitalWrite(outLed, HIGH);

  startMPU9250();

  initSDcard();

}

void loop()
{
  getMPU9250values();

  stringOne = String(millis()) + ",";
  stringOne = stringOne + String((int)1000*myIMU.ax) + "," + String((int)1000*myIMU.ay) + "," + String((int)1000*myIMU.az) + ",";
  stringOne = stringOne + String(myIMU.gx) + "," + String(myIMU.gy) + "," + String(myIMU.gz) + ",";
  stringOne = stringOne + String(myIMU.mx) + "," + String(myIMU.my) + "," + String(myIMU.mz);
  stringOne.toCharArray(charBuf, 96);

  if (millis() - lastLogTime >= logInterval) {
    if (logToSD) {
      File dataFile = SD.open("telem.txt", FILE_WRITE);
      dataFile.println(charBuf);
      dataFile.close();
    }
    //start UDP stream if active
    if (stream) {
      Udp.beginPacket(IP_Remote,4445);
      Udp.write(charBuf);
      Udp.endPacket();
      }
    lastLogTime = millis();
    toggleLed();
  }
}


//Start functions definition
void connWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  for (wifiRetry = 0; wifiRetry < 4; wifiRetry++) {
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
    while(1) ; // Loop forever if communication doesn't happen
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

  // print to serial once per half-second independent of read rate
  if (myIMU.delt_t > 500)
  {
    if(SerialDebug)
    {
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
      Serial.printf("settings heap size: %u\n", ESP.getFreeHeap());
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
      Serial.println("initialization failed. Things to check:");
      Serial.println("* is a card inserted?");
      Serial.println("* is your wiring correct?");
      Serial.println("* did you change the chipSelect pin to match your shield or module?");
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
