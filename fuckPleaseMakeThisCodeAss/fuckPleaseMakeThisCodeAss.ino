#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>

float accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// ===== GPS =====
TinyGPSPlus gps;
HardwareSerial GPSserial(1);

// ===== SPI IMU (QMI8658) =====
#define IMU_CS   34
#define IMU_SCK  36
#define IMU_MISO 37
#define IMU_MOSI 35

// Registradores
#define REG_CTRL1 0x02
#define REG_ACC_X_L 0x35

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

// ===== FUNÇÕES SPI =====

#define REG_CTRL1 0x02
#define REG_CTRL2 0x03
#define REG_CTRL3 0x04
#define REG_CTRL7 0x08



void initIMU() {
  // Reset
  imuWrite(0x60, 0xB0);
  delay(50);

  // Accelerometer: 2g, 100Hz
  imuWrite(REG_CTRL2, 0x01);

  // Gyroscope: 512dps, 100Hz
  imuWrite(REG_CTRL3, 0x01);

  // Enable accel + gyro
  imuWrite(REG_CTRL7, 0x03);

  delay(100);
}

void imuWrite(uint8_t reg, uint8_t val) {
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(reg & 0x7F); // write
  SPI.transfer(val);
  digitalWrite(IMU_CS, HIGH);
}



void calibrateIMU(int samples = 500) {
  Serial.println("Calibrando IMU... NÃO MEXA!");

  long accXsum = 0, accYsum = 0, accZsum = 0;
  long gyroXsum = 0, gyroYsum = 0, gyroZsum = 0;

  for (int i = 0; i < samples; i++) {
    imuRead();

    accXsum += accX;
    accYsum += accY;
    accZsum += accZ;

    gyroXsum += gyroX;
    gyroYsum += gyroY;
    gyroZsum += gyroZ;

    delay(5);
  }

  accOffsetX = accXsum / (float)samples;
  accOffsetY = accYsum / (float)samples;

  // Z precisa considerar gravidade (~1g)
  accOffsetZ = (accZsum / (float)samples);

  gyroOffsetX = gyroXsum / (float)samples;
  gyroOffsetY = gyroYsum / (float)samples;
  gyroOffsetZ = gyroZsum / (float)samples - 16384.0;

  Serial.println("Calibração concluída!");
}

void imuReadBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(reg | 0x80);
  for (int i = 0; i < len; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(IMU_CS, HIGH);
}

void imuRead() {
  uint8_t buf[12];

  imuReadBurst(0x35, buf, 12);

  accX = (int16_t)(buf[1] << 8 | buf[0]);
  accY = (int16_t)(buf[3] << 8 | buf[2]);
  accZ = (int16_t)(buf[5] << 8 | buf[4]);

  gyroX = (int16_t)(buf[7] << 8 | buf[6]);
  gyroY = (int16_t)(buf[9] << 8 | buf[8]);
  gyroZ = (int16_t)(buf[11] << 8 | buf[10]);

}

// ===== SETUP =====
uint8_t readReg(uint8_t reg) {
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(IMU_CS, HIGH);
  return val;
}

void setup() {
  Wire.begin(42, 41);

  Wire.beginTransmission(0x34); // AXP2101
  Wire.write(0x12);
  Wire.write(0xFF); // ativa tudo
  Wire.endTransmission();
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  Serial.begin(115200);
  delay(1000);

  Serial.println("T-Beam Supreme V3 SPI IMU + GPS");

  // GPS (pinos corretos)
  GPSserial.begin(115200, SERIAL_8N1, 9, 8);

  // SPI
  SPI.begin(IMU_SCK, IMU_MISO, IMU_MOSI, IMU_CS);
  pinMode(IMU_CS, OUTPUT);
  digitalWrite(IMU_CS, HIGH);
  initIMU();
  delay(500);

  calibrateIMU();

  // Inicializar IMU
  imuWrite(REG_CTRL1, 0x60); // enable accel + gyro

  Serial.println("IMU SPI inicializada");
}


// ===== LOOP =====
void loop() {

  // ===== GPS =====
  gps.encode(GPSserial.read());

  Serial.println("=== GPS ===");
  Serial.print("Lat: "); Serial.println(gps.location.lat(), 6);
  Serial.print("Lng: "); Serial.println(gps.location.lng(), 6);
  Serial.print("Sat: "); Serial.println(gps.satellites.value());

  // ===== IMU =====
  uint8_t buf[12];
  imuReadBurst(REG_ACC_X_L, buf, 12);

  accX = buf[1] << 8 | buf[0];
  accY = buf[3] << 8 | buf[2];
  accZ = buf[5] << 8 | buf[4];

  gyroX = buf[7] << 8 | buf[6];
  gyroY = buf[9] << 8 | buf[8];
  gyroZ = buf[11] << 8 | buf[10];

  float gyroXc = accX - accOffsetX;
  float gyroYc = accY - accOffsetY;
  float gyroZc = accZ - accOffsetZ;

  float accXc = gyroX - gyroOffsetX;
  float accYc = gyroY - gyroOffsetY;
  float accZc = gyroZ - gyroOffsetZ;

  float alpha = 0.1; // 0 = suave, 1 = sem filtro

  static float accXf = 0, accYf = 0, accZf = 0;

  accXf = alpha * accXc + (1 - alpha) * accXf;
  accYf = alpha * accYc + (1 - alpha) * accYf;
  accZf = alpha * accZc + (1 - alpha) * accZf;

  float accX_ms2 = accXf / 16384.0 * 9.81;
  float accY_ms2 = accYf / 16384.0 * 9.81;
  float accZ_ms2 = accZf / 16384.0 * 9.81;

  float gyroX_dps = gyroXc / 64.0;
  float gyroY_dps = gyroYc / 64.0;
  float gyroZ_dps = gyroZc / 64.0;

  Serial.println("=== IMU (SPI) ===");

  Serial.print("ACC X: "); Serial.print(accX_ms2);
  Serial.print(" m/s² ");
  Serial.print(" Y: "); Serial.print(accY_ms2);
  Serial.print(" m/s² ");
  Serial.print(" Z: "); Serial.print(accZ_ms2);
  Serial.println(" m/s² ");
  Serial.println("m/s² - metros por segundo ao quadrado");
  Serial.println("");


  Serial.print("GYRO X: "); Serial.print(gyroX_dps);
  Serial.print(" dps ");
  Serial.print(" Y: "); Serial.print(gyroY_dps);
  Serial.print(" dps ");
  Serial.print(" Z: "); Serial.print(gyroZ_dps);
  Serial.println(" dps ");
  Serial.println("dps - degrees per second (graus por segundo)");

  Serial.println("----------------------");

  delay(1000);
}