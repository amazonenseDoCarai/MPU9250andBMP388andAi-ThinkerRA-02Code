#include <Wire.h>
#include <TinyGPSPlus.h>
#include <MPU9250.h>

// =====================
// GPS
// =====================
#define GPS_RX 9
#define GPS_TX 8
#define GPS_BAUD 9600

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// =====================
// MPU9250 (I2C)
// =====================
MPU9250 mpu;

// =====================
// Setup
// =====================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Inicializando...");

  // I2C padrão ESP32 (SDA=21, SCL=22 normalmente)
  Wire.begin();

  // Inicializa MPU9250
  if (!mpu.setup(0x68)) {
    Serial.println("Erro ao iniciar MPU9250!");
    while (1);
  }
  Serial.println("MPU9250 iniciado com sucesso");

  // Inicializa GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS iniciado");
}

// =====================
// Loop principal
// =====================
void loop() {

  // =====================
  // Leitura GPS
  // =====================
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    Serial.println("=== GPS ===");
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);

    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);

    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());

    Serial.print("Satélites: ");
    Serial.println(gps.satellites.value());
  }

  // =====================
  // Leitura MPU9250
  // =====================
  if (mpu.update()) {
    Serial.println("=== MPU9250 ===");

    Serial.print("Accel X: ");
    Serial.print(mpu.getAccX());
    Serial.print(" Y: ");
    Serial.print(mpu.getAccY());
    Serial.print(" Z: ");
    Serial.println(mpu.getAccZ());

    Serial.print("Gyro X: ");
    Serial.print(mpu.getGyroX());
    Serial.print(" Y: ");
    Serial.print(mpu.getGyroY());
    Serial.print(" Z: ");
    Serial.println(mpu.getGyroZ());

    Serial.print("Mag X: ");
    Serial.print(mpu.getMagX());
    Serial.print(" Y: ");
    Serial.print(mpu.getMagY());
    Serial.print(" Z: ");
    Serial.println(mpu.getMagZ());
  }

  Serial.println("----------------------");

  delay(500);
}