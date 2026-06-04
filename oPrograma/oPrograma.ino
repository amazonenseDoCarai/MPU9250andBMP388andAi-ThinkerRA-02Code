#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SensorQMI8658.hpp>
#include <SensorQMC5883L.hpp> 
#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

// ==========================================
// DEFINIÇÃO DE PINOS - T-BEAM SUPREME
// ==========================================
#define I2C_SDA_PMU     42
#define I2C_SCL_PMU     41
#define I2C_SDA_SENSORS 17
#define I2C_SCL_SENSORS 18

#define SD_SCK     36
#define SD_MISO    37
#define SD_MOSI    35
#define SD_CS      47
#define QMI_CS     34

#define GPS_RX     9
#define GPS_TX     8
#define GPS_EN     7

// ==========================================
// INSTÂNCIAS GLOBAIS
// ==========================================
XPowersPMU PMU;
Adafruit_BME280 bme;
TinyGPSPlus gps;
SensorQMI8658 qmi;
SensorQMC5883L qmc; 
HardwareSerial GPSserial(2);
SPIClass spi_sensors(HSPI);

File dataFile;
const char* fileName = "/missao_voo.csv";

// Controlo de tempo
unsigned long lastFastTask = 0; 
unsigned long lastSlowTask = 0; 

// Estado dos sensores
bool sdOK = false, bmeOK = false, qmiOK = false, qmcOK = false;

// Estrutura exigida pela biblioteca e variáveis de leitura
MagnetometerData magData;
float magX = 0.0, magY = 0.0, magZ = 0.0;

void setup() {
  Serial.begin(115200);
  delay(2000); 

  Serial.println("\n=== INICIANDO SISTEMA T-BEAM SUPREME ===");

  // 1. INICIAR PMU (ENERGIA)
  Wire1.begin(I2C_SDA_PMU, I2C_SCL_PMU);
  if (PMU.begin(Wire1, AXP2101_SLAVE_ADDRESS, I2C_SDA_PMU, I2C_SCL_PMU)) {
    PMU.setALDO1Voltage(3300); PMU.enableALDO1(); 
    PMU.setALDO2Voltage(3300); PMU.enableALDO2(); 
    PMU.setALDO4Voltage(3300); PMU.enableALDO4(); 
    Serial.println("[OK] Gestão de Energia (PMU)");
  }

  // 2. INICIAR SPI E CARTÃO SD
  spi_sensors.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (SD.begin(SD_CS, spi_sensors)) {
    sdOK = true;
    Serial.println("[OK] Cartão SD Inicializado");
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("ms,Lat,Lon,Alt_GPS,Temp_C,Pres_hPa,Hum_%,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,MagX,MagY,MagZ");
      dataFile.close();
    }
  }

  // 3. INICIAR BARRAMENTO I2C
  Wire.begin(I2C_SDA_SENSORS, I2C_SCL_SENSORS);
  
  if (bme.begin(0x77, &Wire) || bme.begin(0x76, &Wire)) {
    bmeOK = true;
    Serial.println("[OK] Sensor BME280 (Clima)");
  }

  if (qmc.begin(Wire, I2C_SDA_SENSORS, I2C_SCL_SENSORS)) {
    qmcOK = true;
    Serial.println("[OK] Sensor QMC5883L (Magnetómetro)");
  }

  // 4. INICIAR IMU VIA SPI
  if (qmi.begin(spi_sensors, QMI_CS, SD_MOSI, SD_MISO, SD_SCK)) {
    qmiOK = true;
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_896_8Hz);
    Serial.println("[OK] Sensor QMI8658 (IMU)");
  }

  // 5. INICIAR GPS
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);
  GPSserial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[OK] Módulo GPS");

  Serial.println("=== SETUP COMPLETO ===\n");
}

void loop() {
  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
  }

  unsigned long now = millis();

  // -----------------------------------------------------
  // TAREFA RÁPIDA: IMU e Magnetómetro (10Hz)
  // -----------------------------------------------------
  if (now - lastFastTask >= 100) {
    lastFastTask = now;
    
    float ax = 0.0, ay = 0.0, az = 0.0;
    float gx = 0.0, gy = 0.0, gz = 0.0;

    if (qmiOK) {
      qmi.getAccelerometer(ax, ay, az);
      qmi.getGyroscope(gx, gy, gz);
    }
    
    if (qmcOK) {
      // Lê os dados para dentro da estrutura
      qmc.readData(magData); 
      
      // HACK C++: Ler os 3 eixos da memória contornando os nomes ocultos da biblioteca
      int16_t* arrayMemoria = (int16_t*)&magData;
      magX = (float)arrayMemoria[0];
      magY = (float)arrayMemoria[1];
      magZ = (float)arrayMemoria[2];
    }

    if (sdOK) {
      dataFile = SD.open(fileName, FILE_APPEND);
      if (dataFile) {
        dataFile.printf("%lu,,,,,,,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", now, ax, ay, az, gx, gy, gz, magX, magY, magZ);
        dataFile.close();
      }
    }
  }

  // -----------------------------------------------------
  // TAREFA LENTA: Clima e GPS (0.5Hz)
  // -----------------------------------------------------
  if (now - lastSlowTask >= 2000) {
    lastSlowTask = now;

    float t = bmeOK ? bme.readTemperature() : 0.0;
    float p = bmeOK ? bme.readPressure() / 100.0F : 0.0;
    float h = bmeOK ? bme.readHumidity() : 0.0;
    
    double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    double lon = gps.location.isValid() ? gps.location.lng() : 0.0;
    double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;

    Serial.printf("GPS: %.6f,%.6f | Temp: %.1fC | MagX: %.2f\n", lat, lon, t, magX);

    if (sdOK) {
      dataFile = SD.open(fileName, FILE_APPEND);
      if (dataFile) {
        dataFile.printf("%lu,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,,,,,,,,,\n", now, lat, lon, alt, t, p, h);
        dataFile.close();
      }
    }
  }
}