/*
   LILYGO T-Beam Supreme
   Sensores:
   - MPU9250 (I2C)
   - BME280  (I2C)
   - QMI8658 (SPI)
   - GPS     (UART)

   Monitor Série Arduino IDE
*/

#include <Wire.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BME280.h>

// =====================================================
// GPS
// =====================================================
#define GPS_RX 9
#define GPS_TX 8
#define GPS_WAKEUP 7
#define GPS_1PPS 6

HardwareSerial GPSserial(1);
TinyGPSPlus gps;

// =====================================================
// QMI8658 SPI
// =====================================================
#define QMI_SCLK 36
#define QMI_MISO 37
#define QMI_MOSI 35
#define QMI_CS   34

SPIClass qmiSPI(FSPI);

// =====================================================
// MPU9250
// =====================================================
MPU9250_asukiaaa mpu;

// =====================================================
// BME280
// =====================================================
Adafruit_BME280 bme;

// =====================================================
// QMI8658 REGISTERS
// =====================================================
#define QMI8658_WHO_AM_I 0x00
#define QMI8658_CTRL2    0x03
#define QMI8658_CTRL3    0x04
#define QMI8658_CTRL7    0x08
#define QMI8658_AX_L     0x35

// =====================================================
// QMI8658 SPI FUNCTIONS
// =====================================================
void qmiWriteReg(uint8_t reg, uint8_t value)
{
  digitalWrite(QMI_CS, LOW);
  qmiSPI.transfer(reg & 0x7F);
  qmiSPI.transfer(value);
  digitalWrite(QMI_CS, HIGH);
}

uint8_t qmiReadReg(uint8_t reg)
{
  digitalWrite(QMI_CS, LOW);
  qmiSPI.transfer(reg | 0x80);
  uint8_t value = qmiSPI.transfer(0x00);
  digitalWrite(QMI_CS, HIGH);
  return value;
}

void qmiReadBytes(uint8_t reg, uint8_t *buffer, uint8_t len)
{
  digitalWrite(QMI_CS, LOW);
  qmiSPI.transfer(reg | 0x80);

  for (int i = 0; i < len; i++)
  {
    buffer[i] = qmiSPI.transfer(0x00);
  }

  digitalWrite(QMI_CS, HIGH);
}

void initQMI8658()
{
  pinMode(QMI_CS, OUTPUT);
  digitalWrite(QMI_CS, HIGH);

  qmiSPI.begin(QMI_SCLK, QMI_MISO, QMI_MOSI, QMI_CS);

  delay(100);

  uint8_t whoami = qmiReadReg(QMI8658_WHO_AM_I);

  Serial.print("QMI8658 WHO_AM_I: 0x");
  Serial.println(whoami, HEX);

  // Accelerometer
  qmiWriteReg(QMI8658_CTRL2, 0x15);

  // Gyroscope
  qmiWriteReg(QMI8658_CTRL3, 0x35);

  // Enable ACC + GYRO
  qmiWriteReg(QMI8658_CTRL7, 0x03);

  delay(100);
}

void readQMI8658(float &ax, float &ay, float &az,
                 float &gx, float &gy, float &gz)
{
  uint8_t data[12];

  qmiReadBytes(QMI8658_AX_L, data, 12);

  int16_t rawAx = (data[1] << 8) | data[0];
  int16_t rawAy = (data[3] << 8) | data[2];
  int16_t rawAz = (data[5] << 8) | data[4];

  int16_t rawGx = (data[7] << 8) | data[6];
  int16_t rawGy = (data[9] << 8) | data[8];
  int16_t rawGz = (data[11] << 8) | data[10];

  ax = rawAx / 8192.0;
  ay = rawAy / 8192.0;
  az = rawAz / 8192.0;

  gx = rawGx / 64.0;
  gy = rawGy / 64.0;
  gz = rawGz / 64.0;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("===== T-Beam Supreme =====");

  // =====================================================
  // I2C GENERICO
  // =====================================================
  // Usa os pinos SDA/SCL padrão do ESP32-S3
  Wire.begin();

  // =====================================================
  // GPS
  // =====================================================
  pinMode(GPS_WAKEUP, OUTPUT);
  digitalWrite(GPS_WAKEUP, HIGH);

  pinMode(GPS_1PPS, INPUT);

  GPSserial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("GPS iniciado");

  // =====================================================
  // MPU9250
  // =====================================================
  mpu.setWire(&Wire);

  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  Serial.println("MPU9250 iniciado");

  // =====================================================
  // BME280
  // =====================================================
  if (!bme.begin(0x76))
  {
    Serial.println("BME280 nao encontrado!");
  }
  else
  {
    Serial.println("BME280 iniciado");
  }

  // =====================================================
  // QMI8658
  // =====================================================
  initQMI8658();

  Serial.println("QMI8658 iniciado");
}

void loop()
{
  // =====================================================
  // GPS
  // =====================================================
  while (GPSserial.available())
  {
    gps.encode(GPSserial.read());
  }

  // =====================================================
  // MPU9250
  // =====================================================
  mpu.accelUpdate();
  mpu.gyroUpdate();
  mpu.magUpdate();

  // =====================================================
  // QMI8658
  // =====================================================
  float qax, qay, qaz;
  float qgx, qgy, qgz;

  readQMI8658(qax, qay, qaz, qgx, qgy, qgz);

  // =====================================================
  // SERIAL OUTPUT
  // =====================================================
  Serial.println("========================================");

  // MPU9250
  Serial.println("MPU9250");

  Serial.print("ACC: ");
  Serial.print(mpu.accelX(), 3);
  Serial.print(", ");
  Serial.print(mpu.accelY(), 3);
  Serial.print(", ");
  Serial.println(mpu.accelZ(), 3);

  Serial.print("GYRO: ");
  Serial.print(mpu.gyroX(), 3);
  Serial.print(", ");
  Serial.print(mpu.gyroY(), 3);
  Serial.print(", ");
  Serial.println(mpu.gyroZ(), 3);

  // QMI8658
  Serial.println("QMI8658");

  Serial.print("ACC: ");
  Serial.print(qax, 3);
  Serial.print(", ");
  Serial.print(qay, 3);
  Serial.print(", ");
  Serial.println(qaz, 3);

  Serial.print("GYRO: ");
  Serial.print(qgx, 3);
  Serial.print(", ");
  Serial.print(qgy, 3);
  Serial.print(", ");
  Serial.println(qgz, 3);

  // BME280
  Serial.println("BME280");

  Serial.print("Temperatura: ");
  Serial.print(bme.readTemperature());
  Serial.println(" C");

  Serial.print("Pressao: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Umidade: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("Altitude: ");
  Serial.print(bme.readAltitude(1013.25));
  Serial.println(" m");

  // GPS
  Serial.println("GPS");

  if (gps.location.isValid())
  {
    Serial.print("LAT: ");
    Serial.println(gps.location.lat(), 6);

    Serial.print("LON: ");
    Serial.println(gps.location.lng(), 6);
  }
  else
  {
    Serial.println("Sem fix GPS");
  }

  Serial.print("SAT: ");
  Serial.println(gps.satellites.value());

  Serial.print("1PPS: ");
  Serial.println(digitalRead(GPS_1PPS));

  delay(1000);
}