#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Biblioteca para os sensores da LilyGO
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

// LoRa instanciado no barramento SPI padrão
SX1262 radio = new Module(10, 1, 5, 4);

// Pinos GPS
#define GPS_RX_PIN 9
#define GPS_TX_PIN 8
#define GPS_EN_PIN 7
#define GPS_BAUD   9600 

// PINOS SPI 1: Dedicado ao LoRa
#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_CS    10

// PINOS SPI 2: Barramento nativo do IMU e SD Card na placa
#define IMU_MOSI   35
#define IMU_SCLK   36
#define IMU_MISO   37
#define IMU_CS     34
#define SD_CS      47

// Criação do segundo barramento SPI para o QMI8658
SPIClass imuSPI(HSPI);

// Pinos I2C
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
  Serial.println("T-Beam Supreme: Dual SPI Bus Ativo");
  Serial.println("====================================");

  // Inicialização dos barramentos I2C
  Wire.begin(I2C_SDA_SENSORS, I2C_SCL_SENSORS); // Sensores (BME280, QMC6310)
  Wire1.begin(I2C_SDA_PMU, I2C_SCL_PMU);        // PMU

  // 1. Iniciar PMU (Energia)
  if (!PMU.begin(Wire1, AXP2101_SLAVE_ADDRESS, I2C_SDA_PMU, I2C_SCL_PMU)) {
    Serial.println("Falha ao iniciar PMU!");
    while(1);
  }
  
  // Ligar rails de energia (fundamental para alimentar os barramentos internos)
  PMU.setALDO1Voltage(3300); PMU.enableALDO1(); // VDD Sensores
  PMU.setALDO2Voltage(3300); PMU.enableALDO2();
  PMU.setALDO3Voltage(3300); PMU.enableALDO3();
  PMU.setALDO4Voltage(3300); PMU.enableALDO4(); // GPS

  pinMode(GPS_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  delay(500);

  // 2. Iniciar BME280 (I2C)
  if (!bme.begin(0x76) && !bme.begin(0x77)) {
    Serial.println("Falha ao iniciar BME280");
  } else {
    Serial.println("BME280 OK");
  }

  // 3. Inicializar o SEGUNDO barramento SPI (específico do IMU)
  imuSPI.begin(IMU_SCLK, IMU_MISO, IMU_MOSI, IMU_CS);

  // Iniciar QMI8658 passando a nossa instância personalizada de SPI
  if (!qmi.begin(imuSPI, IMU_CS)) {
    Serial.println("Falha ao iniciar QMI8658 via SPI Próprio");
  } else {
    Serial.println("QMI8658 (SPI Dedicado) OK");
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
    qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_896_8Hz, SensorQMI8658::LPF_MODE_0);
    qmi.enableAccelerometer();
    qmi.enableGyroscope();
  }

  // Iniciar QMC6310 (Magnetómetro continua no I2C)
  if (!qmc.begin(Wire, QMC6310U_SLAVE_ADDRESS, I2C_SDA_SENSORS, I2C_SCL_SENSORS)) {
    Serial.println("Falha ao iniciar QMC6310");
  } else {
    Serial.println("QMC6310 OK");
    qmc.configMagnetometer(OperationMode::CONTINUOUS_MEASUREMENT,
                           MagFullScaleRange::FS_8G,
                           50.0f,
                           MagOverSampleRatio::OSR_8,
                           MagDownSampleRatio::DSR_1);
  }

  // 4. Iniciar GPS
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(500);
  GPSserial.println("$PMTK886,3*2B"); // Balloon Mode
  Serial.println("GPS OK (Balloon Mode)");

  // 5. Iniciar LoRa no primeiro barramento SPI (Global/Padrão)
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

    // Leitura QMI8658 (A biblioteca gerencia a comunicação via imuSPI de forma transparente)
    IMUdata acc;
    IMUdata gyr;
    MagnetometerData data;
    qmi.getAccelerometer(acc.x, acc.y, acc.z);
    qmi.getGyroscope(gyr.x, gyr.y, gyr.z);
    float balls = qmc.readData(data);

    float magx = MagnetometerUtils::gaussToMicroTesla(data.magnetic_field.x);
    float magy = MagnetometerUtils::gaussToMicroTesla(data.magnetic_field.y);
    float magz = MagnetometerUtils::gaussToMicroTesla(data.magnetic_field.z);  

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
             ",GZ:" + String(gyr.z, 2) +
             ",MX:" + String(magx, 2) +
             ",MY:" + String(magy, 2) +
             ",MZ:" + String(magz, 2);

    // Monitor Serial
    Serial.println("\n--- Enviando Dados ---");
    Serial.println(packet);
    Serial.printf("Sats: %d | AccX: %.2f | AccY: %.2f | AccZ: %.2f | GyrX: %.2f | GyrY: %.2f | GyrZ: %.2f | MagX: %.2f | MagY: %.2f | MagZ: %.2f\n", gps.satellites.value(), acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z, magx, magy, magz);

    // Enviar via LoRa
    int state = radio.transmit(packet);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("Sucesso LoRa");
    } else {
      Serial.printf("Erro LoRa: %d\n", state);
    }
    //Guardar num cartão SD
    if(!SD.begin(47)){
      Serial.println("it dont wanna");
    } else {
      Serial.println("it do wanna");
    }
  }
}