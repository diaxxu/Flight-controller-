
// Hardware: ESP32 + MPU6500 + QMC5883L + NEO-M8 GPS + RC input
// Feature: Manual flight with switch-based Return To Home

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include <Servo.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define GPS_RX 16
#define GPS_TX 17

#define THROTTLE_PIN 25
#define RUDDER_PIN 26
#define ELEVATOR_PIN 27
#define RC_RTH_CHANNEL 34

Servo throttleServo;
Servo rudderServo;
Servo elevatorServo;

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
QMC5883LCompass compass;

// GPS Coordinates
double homeLat = 0.0, homeLon = 0.0;
double targetLat = 37.7749; // Replace with real waypoint
double targetLon = -122.4194;

bool modeRTH = false;
bool homeLocked = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  throttleServo.attach(THROTTLE_PIN);
  rudderServo.attach(RUDDER_PIN);
  elevatorServo.attach(ELEVATOR_PIN);

  compass.init();
  compass.setCalibration(-1435, 2719, -1559, 2649, -1895, 2556); // Example, calibrate yours

  pinMode(RC_RTH_CHANNEL, INPUT);

  
  Serial.println("Waiting for GPS fix...");
  while (!gps.location.isValid()) {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
  }

  homeLat = gps.location.lat();
  homeLon = gps.location.lng();
  homeLocked = true;
  Serial.println("Home location locked!");
}

void loop() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  int rthSignal = analogRead(RC_RTH_CHANNEL);
  if (rthSignal > 2000) modeRTH = true; 
  else modeRTH = false;

  compass.read();
  float heading = compass.getAzimuth();

  if (!gps.location.isValid()) return;

  double currentLat = gps.location.lat();
  double currentLon = gps.location.lng();

  double destLat = modeRTH ? homeLat : targetLat;
  double destLon = modeRTH ? homeLon : targetLon;

  double targetHeading = getBearing(currentLat, currentLon, destLat, destLon);
  float yawError = normalizeAngle(targetHeading - heading);

  int rudderPos = 90 + yawError * 1.5; 
  rudderPos = constrain(rudderPos, 45, 135);

  rudderServo.write(rudderPos);
  elevatorServo.write(90);       
  throttleServo.write(120);      

  Serial.print("Mode: "); Serial.println(modeRTH ? "RTH" : "TO TARGET");
  Serial.print("Yaw Error: "); Serial.println(yawError);
  Serial.print("Heading: "); Serial.println(heading);
  delay(100);
}


double getBearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  return fmod((degrees(atan2(y, x)) + 360), 360);
}

float normalizeAngle(float angle) {
  if (angle > 180) angle -= 360;
  if (angle < -180) angle += 360;
  return angle;
}
