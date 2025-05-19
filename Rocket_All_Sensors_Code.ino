#include <Arduino.h>
#include <TinyMPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <BMP280_DEV.h>

#include <LoRa.h>
// green pin 4 RX, yellow pin 3 yellow is tx
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 115200;

float temperature, pressure, altitude;
BMP280_DEV bmp280;

// The TinyGPSPlus object
TinyGPSPlus gps;

//Constructing MPU-6050
MPU6050 mpu (Wire);

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

/*
 *  Method that prints everything for the accelerometer
 */
void PrintGets () {
  // Shows offsets
  LoRa.println("--- Offsets:");
   LoRa.print("GyroX Offset = ");
   LoRa.println(mpu.GetGyroXOffset());
   LoRa.print("GyroY Offset = ");
   LoRa.println(mpu.GetGyroYOffset());
   LoRa.print("GyroZ Offset = ");
   LoRa.println(mpu.GetGyroZOffset());
  // Shows raw data
  LoRa.println("--- Raw data:");
   LoRa.print("Raw AccX = ");
   LoRa.println(mpu.GetRawAccX());
   LoRa.print("Raw AccY = ");
   LoRa.println(mpu.GetRawAccY());
   LoRa.print("Raw AccZ = ");
   LoRa.println(mpu.GetRawAccZ());
   LoRa.print("Raw GyroX = ");
   LoRa.println(mpu.GetRawGyroX());
   LoRa.print("Raw GyroY = ");
   LoRa.println(mpu.GetRawGyroY());
   LoRa.print("Raw GyroZ = ");
   LoRa.println(mpu.GetRawGyroZ());
  // Show readable data
   LoRa.println("--- Readable data:");
   LoRa.print("AccX = ");
   LoRa.print(mpu.GetAccX());
   LoRa.println(" m/s²");
   LoRa.print("AccY = ");
   LoRa.print(mpu.GetAccY());
   LoRa.println(" m/s²");
   LoRa.print("AccZ = ");
   LoRa.print(mpu.GetAccZ());
   LoRa.println(" m/s²");
   LoRa.print("GyroX = ");
   LoRa.print(mpu.GetGyroX());
   LoRa.println(" degrees/second");
   LoRa.print("GyroY = ");
   LoRa.print(mpu.GetGyroY());
   LoRa.println(" degrees/second");
   LoRa.print("GyroZ = ");
   LoRa.print(mpu.GetGyroZ());
   LoRa.println(" degrees/second");
  // Show angles based on accelerometer only
   LoRa.println("--- Accel angles:");
   LoRa.print("AccelAngX = ");
   LoRa.println(mpu.GetAngAccX());
   LoRa.print("AccelAngY = ");
   LoRa.println(mpu.GetAngAccY());
  // Show angles based on gyroscope only
   LoRa.println("--- Gyro angles:");
   LoRa.print("GyroAngX = ");
   LoRa.println(mpu.GetAngGyroX());
   LoRa.print("GyroAngY = ");
   LoRa.println(mpu.GetAngGyroY());
   LoRa.print("GyroAngZ = ");
   LoRa.println(mpu.GetAngGyroZ());
  // Show angles based on both gyroscope and accelerometer
   LoRa.println("--- Filtered angles:");
   LoRa.print("FilteredAngX = ");
   LoRa.println(mpu.GetAngX());
   LoRa.print("FilteredAngY = ");
   LoRa.println(mpu.GetAngY());
   LoRa.print("FilteredAngZ = ");
   LoRa.println(mpu.GetAngZ());
  // Show filter coefficients
   LoRa.println("--- Angle filter coefficients:");
   LoRa.print("Accelerometer percentage = ");
   LoRa.print(mpu.GetFilterAccCoeff());
   LoRa.println('%');
   LoRa.print("Gyroscope percentage = ");
   LoRa.print(mpu.GetFilterGyroCoeff());
   LoRa.println('%');
}

  // software serial lora pins
 
 #define rxPin 10
 #define txPin 11
// Sets up a new SoftwareSerial object
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);
 
void setup()
{

  // Define pin modes for TX and RX
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    // buad rate begin!
    mySerial.begin(115200);
    // should we make it myLoRa begin
  //LoRa.beginPacket();
  
  //Initialization
  mpu.Initialize();

   LoRa.beginPacket(115200);
  ss.begin(GPSBaud);
  bmp280.begin();
  bmp280.setPresOversampling(OVERSAMPLING_X4);    // Set the pressure oversampling to X4
  bmp280.setTempOversampling(OVERSAMPLING_X1);    // Set the temperature oversampling to X1
  bmp280.setIIRFilter(IIR_FILTER_4);              // Set the IIR filter to setting 4 

   LoRa.println("=====================================");
   LoRa.println("Starting calibration...");
  mpu.Calibrate();
   LoRa.println("Calibration complete!");
 
   LoRa.println();
   LoRa.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
   LoRa.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to Calgary  ----  RX    RX        Fail"));
   LoRa.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
}

void loop()
{
Serial.print("test");
    //start
  if (mySerial.available() > 0) {
        mySerial.read();
    }
  static const double CALGARY_LAT = 51.049999, CALGARY_LON = -114.066666;

  bmp280.startForcedConversion();
  delay(100);
  if (bmp280.getMeasurements(temperature, pressure, altitude))// Check if the measurement is complete
  {
     LoRa.print(temperature);     // Display the results    
     LoRa.print(F("*C   "));
     LoRa.print(pressure);    
     LoRa.print(F("hPa   "));
     LoRa.print(altitude);
     LoRa.println(F("m"));  
  }
   LoRa.println();
  delay(100); // 0.1 sec delay

  mpu.Execute();
  PrintGets();
  delay(100); // 0.1 sec delay
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
   LoRa.println();
  
  smartDelay(800);

  if (millis() > 5000 && gps.charsProcessed() < 10)
     LoRa.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
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
       LoRa.print('*');
     LoRa.print(' ');
  }
  else
  {
     LoRa.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
       LoRa.print(' ');
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
   LoRa.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
     LoRa.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
     LoRa.print(sz);
  }
  
  if (!t.isValid())
  {
     LoRa.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
     LoRa.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
     LoRa.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
