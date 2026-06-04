#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Biblioteca para o QMI8658
#include <SensorQMI8658.hpp>
#include <SensorQMC6310.hpp>

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

XPowersPMU PMU;
Adafruit_BME280 bme;
TinyGPSPlus gps;
SensorQMI8658 qmi; // Instância do IMU
SensorQMC6310 qmc;

HardwareSerial GPSserial(2);

SX1262 radio = new Module(10, 1, 5, 4);

// Pinos
#define GPS_RX_PIN 9
#define GPS_TX_PIN 8
#define GPS_EN_PIN 7
#define GPS_BAUD   9600 

#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_CS    10

#define I2C_SDA_SENSORS 17
#define I2C_SCL_SENSORS 18
#define I2C_SDA_PMU     42
#define I2C_SCL_PMU     41

String packet;
unsigned long ultimaTransmissao = 0;
const unsigned long intervaloTransmissao = 5000;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n====================================");
  Serial.println("T-Beam Supreme: GPS + BME280 + QMI8658");
  Serial.println("====================================");

  // Inicialização dos barramentos I2C
  Wire.begin(I2C_SDA_SENSORS, I2C_SCL_SENSORS); // Sensores
  Wire1.begin(I2C_SDA_PMU, I2C_SCL_PMU);        // PMU

  // 1. Iniciar PMU (Energia)
  if (!PMU.begin(Wire1, AXP2101_SLAVE_ADDRESS, I2C_SDA_PMU, I2C_SCL_PMU)) {
    Serial.println("Falha ao iniciar PMU!");
    while(1);
  }
  
  // Ligar rails de energia
  PMU.setALDO1Voltage(3300); PMU.enableALDO1(); // VDD Sensores
  PMU.setALDO2Voltage(3300); PMU.enableALDO2();
  PMU.setALDO3Voltage(3300); PMU.enableALDO3();
  PMU.setALDO4Voltage(3300); PMU.enableALDO4(); // GPS

  pinMode(GPS_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  delay(500);

  // 2. Iniciar BME280
  if (!bme.begin(0x76) && !bme.begin(0x77)) {
    Serial.println("Falha ao iniciar BME280");
  } else {
    Serial.println("BME280 OK");
  }

  // 3. Iniciar QMI8658 (IMU)
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA_SENSORS, I2C_SCL_SENSORS)) {
    Serial.println("Falha ao iniciar QMI8658");
  } else {
    Serial.println("QMI8658 OK");
    // Configurações básicas: 4G de sensibilidade e 512 dps para o giro
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_896_8Hz, SensorQMI8658::LPF_MODE_0);
  }
  if (!qmc.begin(Wire, QMC6310U_SLAVE_ADDRESS, I2C_SDA_SENSORS, I2C_SCL_SENSORS)) {
    Serial.println("Falha ao iniciar QMC6310");
  } else {
    Serial.println("QMC6310 OK");
    qmc.configMagnetometer();
  }

  // 4. Iniciar GPS
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(500);
  GPSserial.println("$PMTK886,3*2B"); // Balloon Mode
  Serial.println("GPS OK (Balloon Mode)");

  // 5. Iniciar LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  int state = radio.begin(868.0, 125.0, 9, 7, 0x12, 22, 8, 1.6);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa OK");
  } else {
    Serial.print("Erro LoRa: "); Serial.println(state);
  }
}

void loop() {
  // Ler GPS continuamente
  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
  }

  if (millis() - ultimaTransmissao >= intervaloTransmissao) {
    ultimaTransmissao = millis();

    // Leitura BME280
    float temp = bme.readTemperature();
    float pres = bme.readPressure() / 100.0F;

    // Leitura QMI8658 (Acelerómetro e Giroscópio)
    IMUdata acc;
    IMUdata gyr;
    qmi.getAccelerometer(acc.x, acc.y, acc.z);
    qmi.getGyroscope(gyr.x, gyr.y, gyr.z);

    // Montar Pacote
    packet = "LAT:" + String(gps.location.lat(), 6) +
             ",LON:" + String(gps.location.lng(), 6) +
             ",ALT:" + String(gps.altitude.meters(), 1) +
             ",TEMP:" + String(temp, 1) +
             ",PRES:" + String(pres, 1) +
             ",AX:" + String(acc.x, 2) +
             ",AY:" + String(acc.y, 2) +
             ",AZ:" + String(acc.z, 2) +
             ",GX:" + String(gyr.x, 2) +
             ",GY:" + String(gyr.y, 2) +
             ",GZ:" + String(gyr.z, 2);

    // Monitor Serial
    Serial.println("\n--- Enviando Dados ---");
    Serial.println(packet);
    Serial.printf("Sats: %d | AccX: %.2f | AccY: %.2f | AccZ: %.2f | GyrX: %.2f | GyrY: %.2f | GyrZ: %.2f\n", gps.satellites.value(), acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z);

    // Enviar via LoRa
    int state = radio.transmit(packet);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("Sucesso LoRa");
    } else {
      Serial.printf("Erro LoRa: %d\n", state);
    }
    
  }
}