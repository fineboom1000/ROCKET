#include <Arduino.h>
#include <TinyMPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <BMP280_DEV.h>

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 115200;

float temperature, pressure, altitude;
BMP280_DEV bmp280;


TinyGPSPlus gps;


MPU6050 mpu (Wire);

SoftwareSerial ss(RXPin, TXPin);

 

#include <SPI.h>

#include <RH_RF95.h>

 



#define RFM95_CS 8

#define RFM95_RST 4

#define RFM95_INT 7

 

 

#if defined(ESP8266)

  

  #define RFM95_CS  2    

  #define RFM95_RST 16   

  #define RFM95_INT 15   

 

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)

 

  #define RFM95_CS      8

  #define RFM95_INT     3

  #define RFM95_RST     4

 

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)

  #define RFM95_INT     9  

  #define RFM95_CS      10  

  #define RFM95_RST     11  

 

#elif defined(ESP32)  

  

  #define RFM95_RST     27   

  #define RFM95_CS      33   

  #define RFM95_INT     12   

 

#elif defined(ARDUINO_NRF52832_FEATHER)

  

  #define RFM95_RST     7   // "A"

  #define RFM95_CS      11   // "B"

  #define RFM95_INT     31   // "C"

  #define LED           17

 

#elif defined(TEENSYDUINO)

  

  #define RFM95_RST     9   // "A"

  #define RFM95_CS      10   // "B"

  #define RFM95_INT     4    // "C"

#endif

 


#define RF95_FREQ 915.0

 



RH_RF95 rf95(RFM95_CS, RFM95_INT);

 



#define LED 13
void PrintGets () {
  
  Serial.println("--- Offsets:");
  Serial.print("GyroX Offset = ");
  Serial.println(mpu.GetGyroXOffset());
  Serial.print("GyroY Offset = ");
  Serial.println(mpu.GetGyroYOffset());
  Serial.print("GyroZ Offset = ");
  Serial.println(mpu.GetGyroZOffset());
  
  Serial.println("--- Raw data:");
  Serial.print("Raw AccX = ");
  Serial.println(mpu.GetRawAccX());
  Serial.print("Raw AccY = ");
  Serial.println(mpu.GetRawAccY());
  Serial.print("Raw AccZ = ");
  Serial.println(mpu.GetRawAccZ());
  Serial.print("Raw GyroX = ");
  Serial.println(mpu.GetRawGyroX());
  Serial.print("Raw GyroY = ");
  Serial.println(mpu.GetRawGyroY());
  Serial.print("Raw GyroZ = ");
  Serial.println(mpu.GetRawGyroZ());
  
  Serial.println("--- Readable data:");
  Serial.print("AccX = ");
  Serial.print(mpu.GetAccX());
  Serial.println(" m/s²");
  Serial.print("AccY = ");
  Serial.print(mpu.GetAccY());
  Serial.println(" m/s²");
  Serial.print("AccZ = ");
  Serial.print(mpu.GetAccZ());
  Serial.println(" m/s²");
  Serial.print("GyroX = ");
  Serial.print(mpu.GetGyroX());
  Serial.println(" degrees/second");
  Serial.print("GyroY = ");
  Serial.print(mpu.GetGyroY());
  Serial.println(" degrees/second");
  Serial.print("GyroZ = ");
  Serial.print(mpu.GetGyroZ());
  Serial.println(" degrees/second");
  
  Serial.println("--- Accel angles:");
  Serial.print("AccelAngX = ");
  Serial.println(mpu.GetAngAccX());
  Serial.print("AccelAngY = ");
  Serial.println(mpu.GetAngAccY());
  
  Serial.println("--- Gyro angles:");
  Serial.print("GyroAngX = ");
  Serial.println(mpu.GetAngGyroX());
  Serial.print("GyroAngY = ");
  Serial.println(mpu.GetAngGyroY());
  Serial.print("GyroAngZ = ");
  Serial.println(mpu.GetAngGyroZ());
  
  Serial.println("--- Filtered angles:");
  Serial.print("FilteredAngX = ");
  Serial.println(mpu.GetAngX());
  Serial.print("FilteredAngY = ");
  Serial.println(mpu.GetAngY());
  Serial.print("FilteredAngZ = ");
  Serial.println(mpu.GetAngZ());
  
  Serial.println("--- Angle filter coefficients:");
  Serial.print("Accelerometer percentage = ");
  Serial.print(mpu.GetFilterAccCoeff());
  Serial.println('%');
  Serial.print("Gyroscope percentage = ");
  Serial.print(mpu.GetFilterGyroCoeff());
  Serial.println('%');
}

void setup()
{
  
  mpu.Initialize();

  Serial.begin(115200);
  ss.begin(GPSBaud);
  bmp280.begin();
  bmp280.setPresOversampling(OVERSAMPLING_X4);    
  bmp280.setTempOversampling(OVERSAMPLING_X1);    
  bmp280.setIIRFilter(IIR_FILTER_4);              

  Serial.println("=====================================");
  Serial.println("Starting calibration...");
  mpu.Calibrate();
  Serial.println("Calibration complete!");
 
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to Calgary  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));


  pinMode(LED, OUTPUT);

  pinMode(RFM95_RST, OUTPUT);

  digitalWrite(RFM95_RST, HIGH);

 

  Serial.begin(115200);

  while (!Serial) {

    delay(1);

  }

  delay(100);

 

  Serial.println("Feather LoRa RX Test!");

 

  

  digitalWrite(RFM95_RST, LOW);

  delay(10);

  digitalWrite(RFM95_RST, HIGH);

  delay(10);

 

  while (!rf95.init()) {

    Serial.println("LoRa radio init failed");

    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");

    while (1);

  }

  Serial.println("LoRa radio init OK!");

 

  

  if (!rf95.setFrequency(RF95_FREQ)) {

    Serial.println("setFrequency failed");

    while (1);

  }

  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

 



  rf95.setTxPower(23, false);

}

void loop()
{
  static const double CALGARY_LAT = 51.049999, CALGARY_LON = -114.066666;

  bmp280.startForcedConversion();
  delay(100);
  if (bmp280.getMeasurements(temperature, pressure, altitude))
  {
    Serial.print(temperature);       
    Serial.print(F("*C   "));
    Serial.print(pressure);    
    Serial.print(F("hPa   "));
    Serial.print(altitude);
    Serial.println(F("m"));  
  }
  Serial.println();
  delay(100); 

  mpu.Execute();
  PrintGets();
  delay(100); 
  Serial.println();

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToCALGARY =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      CALGARY_LAT, 
      CALGARY_LON) / 1000;
  printInt(distanceKmToCALGARY, gps.location.isValid(), 9);

  double courseToCALGARY =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      CALGARY_LAT, 
      CALGARY_LON);

  printFloat(courseToCALGARY, gps.location.isValid(), 7, 2);

  const char *cardinalToCalgary = TinyGPSPlus::cardinal(courseToCALGARY);

  printStr(gps.location.isValid() ? cardinalToCalgary : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(800);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  pinMode(LED, OUTPUT);

  pinMode(RFM95_RST, OUTPUT);

  digitalWrite(RFM95_RST, HIGH);

 

  Serial.begin(115200);

  while (!Serial) {

    delay(1);

  }

  delay(100);

 

  Serial.println("Feather LoRa RX Test!");

 

  

  digitalWrite(RFM95_RST, LOW);

  delay(10);

  digitalWrite(RFM95_RST, HIGH);

  delay(10);

 

  while (!rf95.init()) {

    Serial.println("LoRa radio init failed");

    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");

    while (1);

  }

  Serial.println("LoRa radio init OK!");

 

  

  if (!rf95.setFrequency(RF95_FREQ)) {

    Serial.println("setFrequency failed");

    while (1);

  }

  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

 

 

  rf95.setTxPower(23, false);

}










static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
