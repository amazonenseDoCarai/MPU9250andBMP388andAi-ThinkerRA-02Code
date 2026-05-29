#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SensorQMI8658.hpp>
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
// INSTÂNCIAS
// ==========================================
XPowersPMU PMU;
Adafruit_BME280 bme;
TinyGPSPlus gps;
SensorQMI8658 qmi;
HardwareSerial GPSserial(2);
SPIClass spi_sensors(HSPI);

File dataFile;
const char* fileName = "/datalog.csv";

// Temporização
unsigned long lastFastTask = 0;
unsigned long lastSlowTask = 0;

bool sdOK = false, bmeOK = false, qmiOK = false;

void setup() {
  Serial.begin(115200);
  unsigned long t_start = millis();
  while (!Serial && (millis() - t_start < 3000));

  Serial.println("\n\n--- INICIALIZANDO (GPS + BME + IMU) ---");

  // 1. ENERGIA (PMU)
  Wire1.begin(I2C_SDA_PMU, I2C_SCL_PMU);
  if (PMU.begin(Wire1, AXP2101_SLAVE_ADDRESS, I2C_SDA_PMU, I2C_SCL_PMU)) {
    PMU.setALDO1Voltage(3300); PMU.enableALDO1(); // Sensores
    PMU.setALDO2Voltage(3300); PMU.enableALDO2(); // SD Card
    PMU.setALDO4Voltage(3300); PMU.enableALDO4(); // GPS
    Serial.println("[OK] Energia configurada.");
  }

  // 2. SPI E CARTÃO SD
  spi_sensors.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (SD.begin(SD_CS, spi_sensors)) {
    sdOK = true;
    Serial.println("[OK] Cartão SD pronto.");
    
    // Criar cabeçalho sem magnetómetro
    dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("ms,Lat,Lon,Alt_GPS,Temp_C,Pres_hPa,Hum_%,AccX,AccY,AccZ,GyrX,GyrY,GyrZ");
      dataFile.close();
    }
  } else {
    Serial.println("[AVISO] SD não encontrado. A continuar sem gravação.");
  }

  // 3. I2C SENSORES (BME280)
  Wire.begin(I2C_SDA_SENSORS, I2C_SCL_SENSORS);
  if (bme.begin(0x77, &Wire) || bme.begin(0x76, &Wire)) {
    bmeOK = true;
    Serial.println("[OK] BME280 pronto.");
  }

  // 4. IMU QMI8658 (SPI)
  if (qmi.begin(spi_sensors, QMI_CS, SD_MOSI, SD_MISO, SD_SCK)) {
    qmiOK = true;
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_896_8Hz);
    Serial.println("[OK] IMU pronto.");
  }

  // 5. GPS
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);
  GPSserial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[OK] GPS Iniciado.");
}

void loop() {
  // LER GPS (Processamento contínuo)
  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
  }

  unsigned long now = millis();

  // TAREFA RÁPIDA (10Hz - 100ms): Acelerómetro e Giroscópio
  if (now - lastFastTask >= 100) {
    lastFastTask = now;
    
    float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    if (qmiOK) {
      qmi.getAccelerometer(ax, ay, az);
      qmi.getGyroscope(gx, gy, gz);
      
      Serial.printf("[IMU] Acc:%.2f %.2f %.2f | Gyr:%.2f %.2f\n", ax, ay, az, gx, gy);

      if (sdOK) {
        dataFile = SD.open(fileName, FILE_APPEND);
        if (dataFile) {
          // Grava apenas as colunas de movimento (as outras ficam vazias nesta linha)
          dataFile.printf("%lu,,,,,,,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                          now, ax, ay, az, gx, gy, gz);
          dataFile.close();
        }
      }
    }
  }

  // TAREFA LENTA (2 segundos): GPS e Ambiente
  if (now - lastSlowTask >= 2000) {
    lastSlowTask = now;

    float t=0, p=0, h=0;
    if (bmeOK) {
      t = bme.readTemperature();
      p = bme.readPressure() / 100.0F;
      h = bme.readHumidity();
    }

    double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    double lon = gps.location.isValid() ? gps.location.lng() : 0.0;
    double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;

    Serial.println("\n--- UPDATE AMBIENTE/GPS ---");
    Serial.printf("POS: %.6f, %.6f | ALT: %.1fm\n", lat, lon, alt);
    Serial.printf("BME: %.1fC | %.1fhPa | %.1f%%\n\n", t, p, h);

    if (sdOK) {
      dataFile = SD.open(fileName, FILE_APPEND);
      if (dataFile) {
        // Grava apenas colunas de GPS e Clima
        dataFile.printf("%lu,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,,,,,,\n", 
                        now, lat, lon, alt, t, p, h);
        dataFile.close();
      }
    }
  }
}