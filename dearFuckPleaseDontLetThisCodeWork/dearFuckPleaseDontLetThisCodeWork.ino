#include <SoftWire.h>

#define SDA_PIN 6
#define SCL_PIN 7

SoftWire sw(SDA_PIN, SCL_PIN);

// MPU9250 I2C addresses
#define MPU9250_ADDR 0x68
#define AK8963_ADDR  0x0C

// MPU9250 registers
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define INT_PIN_CFG  0x37

// AK8963 registers
#define AK8963_CNTL1 0x0A
#define AK8963_ST1   0x02
#define AK8963_XOUT_L 0x03

void writeRegister(uint8_t addr, uint8_t reg, uint8_t data)
{
  sw.beginTransmission(addr);
  sw.write(reg);
  sw.write(data);
  sw.endTransmission();
}

uint8_t readRegister(uint8_t addr, uint8_t reg)
{
  sw.beginTransmission(addr);
  sw.write(reg);
  sw.endTransmission(false);

  sw.requestFrom(addr, (uint8_t)1);
  return sw.read();
}

void readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *dest)
{
  sw.beginTransmission(addr);
  sw.write(reg);
  sw.endTransmission(false);

  sw.requestFrom(addr, count);

  for (uint8_t i = 0; i < count; i++)
    dest[i] = sw.read();
}

void setup()
{
  Serial.begin(115200);

  sw.begin();
  sw.setClock(100000);

  delay(100);

  // Wake MPU9250
  writeRegister(MPU9250_ADDR, PWR_MGMT_1, 0x00);
  delay(100);

  // Enable bypass to access magnetometer
  writeRegister(MPU9250_ADDR, INT_PIN_CFG, 0x02);
  delay(10);

  // Magnetometer setup
  writeRegister(AK8963_ADDR, AK8963_CNTL1, 0x00);
  delay(10);
  writeRegister(AK8963_ADDR, AK8963_CNTL1, 0x16); // Continuous mode 2 (100Hz)
  delay(10);

  Serial.println("MPU9250 Initialized");
}

void loop()
{
  uint8_t rawData[14];
  readBytes(MPU9250_ADDR, ACCEL_XOUT_H, 14, rawData);

  int16_t ax = (rawData[0] << 8) | rawData[1];
  int16_t ay = (rawData[2] << 8) | rawData[3];
  int16_t az = (rawData[4] << 8) | rawData[5];

  int16_t gx = (rawData[8] << 8) | rawData[9];
  int16_t gy = (rawData[10] << 8) | rawData[11];
  int16_t gz = (rawData[12] << 8) | rawData[13];

  uint8_t magStatus = readRegister(AK8963_ADDR, AK8963_ST1);

  int16_t mx = 0, my = 0, mz = 0;

  if (magStatus & 0x01)
  {
    uint8_t magData[7];
    readBytes(AK8963_ADDR, AK8963_XOUT_L, 7, magData);

    mx = (magData[1] << 8) | magData[0];
    my = (magData[3] << 8) | magData[2];
    mz = (magData[5] << 8) | magData[4];
  }

  Serial.print("ACCEL: ");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az);

  Serial.print(" | GYRO: ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.print(gz);

  Serial.print(" | MAG: ");
  Serial.print(mx); Serial.print(", ");
  Serial.print(my); Serial.print(", ");
  Serial.print(mz);

  Serial.println();

  delay(1000);
}