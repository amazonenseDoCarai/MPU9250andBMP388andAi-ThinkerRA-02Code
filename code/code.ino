#include <SPI.h>
#include <TinyGPS++.h>
#include <Wire.h>

// ===== GPS =====
TinyGPSPlus gps;
HardwareSerial GPSserial(1);

// ===== IMU QMI8658 (SPI) =====
#define IMU_CS   34   // Chip select do IMU
#define SPI_SCK  36
#define SPI_MISO 37
#define SPI_MOSI 35

// Registradores QMI8658
#define REG_CTRL2   0x03
#define REG_CTRL3   0x04
#define REG_CTRL7   0x08
#define REG_ACC_X_L 0x35

// Variáveis raw
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

// offsets
float accOffsetX=0, accOffsetY=0, accOffsetZ=0;
float gyroOffsetX=0, gyroOffsetY=0, gyroOffsetZ=0;

// filtro simples
float accXf=0, accYf=0, accZf=0;
float alpha=0.1;

// ==== SPI de baixo nível ====
void imuWrite(uint8_t reg, uint8_t val){
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(IMU_CS, HIGH);
}

void imuReadBurst(uint8_t reg, uint8_t *buf, uint8_t len){
  digitalWrite(IMU_CS, LOW);
  SPI.transfer(reg | 0x80);
  for (int i=0;i<len;i++){
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(IMU_CS, HIGH);
}

// ==== Inicializa IMU ====
void initIMU(){
  digitalWrite(IMU_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, IMU_CS);
  delay(100);

  // Configura escala e habilita
  imuWrite(REG_CTRL2, 0x01); // accel ±2g
  imuWrite(REG_CTRL3, 0x01); // gyro ±512 dps
  imuWrite(REG_CTRL7, 0x03); // habilita acel + gyro

  delay(100);
}

// ==== Lê a IMU ====
void readIMU(){
  uint8_t buf[12];
  imuReadBurst(REG_ACC_X_L, buf, 12);

  accX = (int16_t)(buf[1]<<8 | buf[0]);
  accY = (int16_t)(buf[3]<<8 | buf[2]);
  accZ = (int16_t)(buf[5]<<8 | buf[4]);

  gyroX = (int16_t)(buf[7]<<8 | buf[6]);
  gyroY = (int16_t)(buf[9]<<8 | buf[8]);
  gyroZ = (int16_t)(buf[11]<<8 | buf[10]);
}

// ==== Calibração simples ====
void calibrateIMU(int samples=300){
  long ax=0, ay=0, az=0;
  long gx=0, gy=0, gz=0;

  for(int i=0;i<samples;i++){
    readIMU();
    ax += accX; ay += accY; az += accZ;
    gx += gyroX; gy += gyroY; gz += gyroZ;
    delay(5);
  }

  accOffsetX = ax / (float)samples;
  accOffsetY = ay / (float)samples;
  accOffsetZ = az / (float)samples;

  gyroOffsetX = gx / (float)samples;
  gyroOffsetY = gy / (float)samples;
  gyroOffsetZ = gz / (float)samples;
}

// ==== SETUP ====
void setup(){
  Serial.begin(115200);
  delay(1000);

  // Ativa energia de sensores via AXP2101
  Wire.begin(17,18);
  Wire.beginTransmission(0x34);
  Wire.write(0x12);
  Wire.write(0xFF);
  Wire.endTransmission();

  Serial.println("Iniciando GPS+IMU...");

  // GPS
  GPSserial.begin(9600, SERIAL_8N1, 9, 8);

  // IMU
  pinMode(IMU_CS, OUTPUT);
  initIMU();
  calibrateIMU();
}

// ==== LOOP ====
void loop(){
  while (GPSserial.available()){
    gps.encode(GPSserial.read());
  }

  if (gps.location.isUpdated()){
    Serial.println("=== GPS ===");
    Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
    Serial.print("Satélites: "); Serial.println(gps.satellites.value());
    Serial.println("-------------------------");
  }

  readIMU();

  float accXc = accX - accOffsetX;
  float accYc = accY - accOffsetY;
  float accZc = accZ - accOffsetZ;

  accXf = alpha * accXc + (1 - alpha)*accXf;
  accYf = alpha * accYc + (1 - alpha)*accYf;
  accZf = alpha * accZc + (1 - alpha)*accZf;

  Serial.println("=== IMU ===");
  Serial.print("Accel(g) X: "); Serial.print(accXf/16384.0);
  Serial.print(" Y: "); Serial.print(accYf/16384.0);
  Serial.print(" Z: "); Serial.println(accZf/16384.0);

  Serial.println("-------------------------");
  delay(500);
}