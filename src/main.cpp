#include <QMC5883LCompass.h>
#include <ReefwingMSP.h>
#include <UbxGpsNavPvt.h>

UbxGpsNavPvt<HardwareSerial> gps(Serial1);
SerialPIO fcSerial(D3, SerialPIO::NOPIN);

QMC5883LCompass compass;
ReefwingMSP msp;

typedef struct  __attribute__ ((packed)){
  uint8_t  instance;
  uint32_t timeMs;
  int16_t  magX; // mGauss, front
  int16_t  magY; // mGauss, right
  int16_t  magZ; // mGauss, down
} mspSensorCompassDataMessage_t;

typedef struct __attribute__((packed)) {
    uint8_t  instance;                  // sensor instance number to support multi-sensor setups
    uint16_t gpsWeek;                   // GPS week, 0xFFFF if not available [NOT USED]
    uint32_t msTOW; // [NOT USED]
    uint8_t  fixType;
    uint8_t  satellitesInView;
    uint16_t horizontalPosAccuracy;     // [cm]
    uint16_t verticalPosAccuracy;       // [cm]
    uint16_t horizontalVelAccuracy;     // [cm/s] [NOT USED]
    uint16_t hdop;
    int32_t  longitude;
    int32_t  latitude;
    int32_t  mslAltitude;       // cm
    int32_t  nedVelNorth;       // cm/s
    int32_t  nedVelEast;
    int32_t  nedVelDown;
    uint16_t groundCourse;      // deg * 100, 0..36000
    uint16_t trueYaw;           // deg * 100, values of 0..36000 are valid. 65535 = no data available [NOT USED]
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
} mspSensorGpsDataMessage_t;

#define MSP2_SENSOR_GPS             0x1F03
#define MSP2_SENSOR_COMPASS         0x1F04
#define MSP2_SENSOR_BAROMETER       0x1F05

#define COMPUTER_BAUDRATE 115200
#define GPS_BAUDRATE 115200

void onGpsUpdate()
{
  mspSensorGpsDataMessage_t g;
  g.fixType = gps.fixType;
  g.year = gps.year;
  g.month = gps.month;
  g.day = gps.day;
  g.hour = gps.hour;
  g.min = gps.min;
  g.sec = gps.sec;
  g.longitude = gps.lon;
  g.latitude = gps.lat;
  g.mslAltitude = gps.hMSL / 10;
  g.groundCourse = gps.heading / 1000;
  g.horizontalPosAccuracy = gps.hAcc / 10;
  g.verticalPosAccuracy = gps.vAcc / 10;
  g.horizontalVelAccuracy = gps.sAcc / 10;
  g.trueYaw = 65535;
  g.gpsWeek = 0xFFFF;
  g.satellitesInView = gps.numSV;
  g.hdop = sqrt(pow(gps.pDOP, 2)/3.25);
  msp.send(MSP2_SENSOR_GPS, &g, sizeof(g));
}

void onCompassUpdate()
{
  int x = compass.getX();
  int y = compass.getY();
  int z = compass.getZ();
  mspSensorCompassDataMessage_t c;
  c.magX = x;
  c.magY = y;
  c.magZ = z;
  msp.send(MSP2_SENSOR_COMPASS, &c, sizeof(c));
}

// the setup function runs once when you press reset or power the board
void setup() 
{
  compass.init();
  
  Serial.begin(COMPUTER_BAUDRATE);
  while (!Serial);

  gps.begin(GPS_BAUDRATE);
  while (!Serial1);

  fcSerial.begin(GPS_BAUDRATE);
  while (!fcSerial);

  msp.begin(Serial);
}
 
// the loop function runs over and over again forever
void loop() 
{
  // if(Serial1.available()) {
  //   Serial.write(Serial1.read());
  // }
  // if(Serial.available()) {
  //   Serial1.write(Serial.read());
  // }
  if (gps.ready())
  {
    onGpsUpdate();
    onCompassUpdate();
  }
}