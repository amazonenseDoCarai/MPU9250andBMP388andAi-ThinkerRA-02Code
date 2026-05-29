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
#define GPS_BAUD   9600 

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

// ==========================================
// TEMPORIZAÇÃO (Multitasking)
// ==========================================
unsigned long lastIMU  = 0;
unsigned long lastENV  = 0;
unsigned long lastSync = 0;

const unsigned long intervalIMU  = 100;   // 10Hz (Acelerómetro/Giroscópio)
const unsigned long intervalENV  = 2000;  // 0.5Hz (GPS e BME280)
const unsigned long intervalSync = 5000;  // 5s (Guardar ficheiro fisicamente no SD)

// Variáveis de estado
bool sdCardOK = false;
bool bmeOK = false;
bool qmiOK = false;

void setup() {
  Serial.begin(115200);
  
  // Espera pelo Monitor Série abrir, mas com um limite de 3 segundos
  unsigned long t_start = millis();
  while (!Serial && (millis() - t_start < 3000));
  
  Serial.println("\n\n==========================================");
  Serial.println("  T-BEAM SUPREME - DATALOGGER MULTI-FREQ");
  Serial.println("==========================================");

  // 1. INICIAR ENERGIA (PMU)
  Wire1.begin(I2C_SDA_PMU, I2C_SCL_PMU);
  if (!PMU.begin(Wire1, AXP2101_SLAVE_ADDRESS, I2C_SDA_PMU, I2C_SCL_PMU)) {
    Serial.println("[ERRO] PMU AXP2101 não encontrado.");
  } else {
    Serial.println("[OK] PMU Iniciado.");
    PMU.setALDO1Voltage(3300); PMU.enableALDO1(); 
    PMU.setALDO2Voltage(3300); PMU.enableALDO2(); 
    PMU.setALDO3Voltage(3300); PMU.enableALDO3();
    PMU.setALDO4Voltage(3300); PMU.enableALDO4(); 
  }

  // 2. INICIAR I2C SENSOR CLIMA (BME280)
  Wire.begin(I2C_SDA_SENSORS, I2C_SCL_SENSORS);
  if (!bme.begin(0x77) && !bme.begin(0x76)) {
    Serial.println("[ERRO] BME280 não encontrado.");
  } else {
    Serial.println("[OK] BME280 Iniciado.");
    bmeOK = true;
  }

  // 3. INICIAR BARRAMENTO SPI SECUNDÁRIO
  spi_sensors.begin(SD_SCK, SD_MISO, SD_MOSI);

  // 4. INICIAR IMU (QMI8658) via SPI
  if (!qmi.begin(spi_sensors, QMI_CS, SD_MOSI, SD_MISO, SD_SCK)) {
    Serial.println("[ERRO] QMI8658 não encontrado.");
  } else {
    Serial.println("[OK] QMI8658 Iniciado.");
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz);
    
    // CORREÇÃO AQUI: Usar GYR_ em vez de GYRO_
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_896_8Hz);
    
    qmiOK = true;
  }

  // 5. INICIAR CARTÃO SD
  if (!SD.begin(SD_CS, spi_sensors)) {
    Serial.println("[ERRO] Cartão SD não detetado ou falhou.");
  } else {
    Serial.println("[OK] Cartão SD Iniciado.");
    sdCardOK = true;
    
    dataFile = SD.open("/dados_voo.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Tempo_ms,Lat,Lon,Alt_m,Temp_C,Pres_hPa,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z");
      dataFile.flush();
      Serial.println("[OK] Ficheiro /dados_voo.csv pronto.");
    } else {
      Serial.println("[ERRO] Não foi possível criar o ficheiro no SD.");
      sdCardOK = false;
    }
  }

  // 6. INICIAR GPS
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, HIGH);
  GPSserial.setRxBufferSize(1024);
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(500);
  GPSserial.println("$PMTK886,3*2B");
  Serial.println("[OK] GPS Iniciado (Aguardando FIX...)");
  
  Serial.println("==========================================");
  Serial.println("  A INICIAR LEITURAS...");
  Serial.println("==========================================\n");
}

void loop() {
  unsigned long now = millis();

  // ---------------------------------------------------------
  // TAREFA 0: LER GPS
  // ---------------------------------------------------------
  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
  }

  // ---------------------------------------------------------
  // TAREFA 1: LEITURA RÁPIDA - IMU
  // ---------------------------------------------------------
  if (now - lastIMU >= intervalIMU) {
    lastIMU = now;
    
    if (qmiOK) {
      float ax, ay, az, gx, gy, gz;
      qmi.getAccelerometer(ax, ay, az);
      qmi.getGyroscope(gx, gy, gz);

      Serial.printf("[IMU] %lu ms | Acc: %.2f, %.2f, %.2f | Gyr: %.2f, %.2f, %.2f\n", now, ax, ay, az, gx, gy, gz);

      if (sdCardOK && dataFile) {
        dataFile.printf("%lu,,,,,,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", now, ax, ay, az, gx, gy, gz);
      }
    }
  }

  // ---------------------------------------------------------
  // TAREFA 2: LEITURA LENTA - AMBIENTE
  // ---------------------------------------------------------
  if (now - lastENV >= intervalENV) {
    lastENV = now;

    float temp = 0.0, press = 0.0;
    if (bmeOK) {
      temp = bme.readTemperature();
      press = bme.readPressure() / 100.0F;
    }

    double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    double lon = gps.location.isValid() ? gps.location.lng() : 0.0;
    double alt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    int sats = gps.satellites.value();

    Serial.println("\n------------------------------------------------");
    Serial.printf("[AMBIENTE] Temp: %.2f C | Pressao: %.2f hPa\n", temp, press);
    Serial.printf("[GPS] Sats: %d | Lat: %.6f | Lon: %.6f | Alt: %.1f m\n", sats, lat, lon, alt);
    Serial.println("------------------------------------------------\n");

    if (sdCardOK && dataFile) {
      dataFile.printf("%lu,%.6f,%.6f,%.2f,%.2f,%.2f,,,,,,\n", now, lat, lon, alt, temp, press);
    }
  }

  // ---------------------------------------------------------
  // TAREFA 3: SINCRONIZAR CARTÃO SD
  // ---------------------------------------------------------
  if (now - lastSync >= intervalSync) {
    lastSync = now;
    if (sdCardOK && dataFile) {
      dataFile.flush();
      Serial.println(">>> [SD] Dados sincronizados no cartao.");
    }
  }
}

