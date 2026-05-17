#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

XPowersPMU PMU;

Adafruit_BME280 bme;

TinyGPSPlus gps;

HardwareSerial GPSserial(2);

SX1262 radio = new Module(
  10,
  1,
  5,
  4
);

#define GPS_RX_PIN 9
#define GPS_TX_PIN 8
#define GPS_EN_PIN 7
#define GPS_BAUD   9600 

#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_CS    10

#define I2C_SDA 42
#define I2C_SCL 41

String packet;
unsigned long ultimaTransmissao = 0;
const unsigned long intervaloTransmissao = 5000;

void setup() {

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("====================================");
  Serial.println("T-Beam Supreme LoRa GPS BME280");
  Serial.println("====================================");

  Wire.begin(17, 18);
  Wire1.begin(I2C_SDA, I2C_SCL);
  Serial.println("I2C iniciado");

  if (!PMU.begin(Wire1, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
    Serial.println("Falha ao iniciar PMU AXP2101!");
    Serial.println("Verifique as bibliotecas. O GPS não terá energia!");
    while(1) { delay(100); }
  } else {
    Serial.println("PMU AXP2101 iniciado com sucesso!");

    PMU.setALDO4Voltage(3300);
    PMU.enableALDO4();
    
    PMU.setALDO3Voltage(3300); 
    PMU.enableALDO3();
    PMU.setALDO2Voltage(3300);
    PMU.enableALDO2();
    PMU.setALDO1Voltage(3300);
    PMU.enableALDO1();
  }

  pinMode(GPS_EN_PIN, OUTPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  delay(1000);

  bme.begin(0x77);
  if (!bme.begin(0x76) && !bme.begin(0x77)) {
      Serial.println("Falha ao iniciar BME280");
  } else {
      Serial.println("BME280 iniciado");
  }

  // =====================================================
  // GPS
  // =====================================================
  GPSserial.end();
  delay(500);
  GPSserial.setRxBufferSize(4096);
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(1000);
  Serial.println("GPS iniciado");

  Serial.print("Inicializando LoRa ... ");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  int state = radio.begin(868.0, 125.0, 9, 7, 0x12, 22, 8, 1.6);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("OK");
  } else {
    Serial.print("Erro LoRa: ");
    Serial.println(state);
    while (true) { delay(1000); }
  }

  Serial.println("Aguardando FIX GPS...");
}

void loop() {

  while (GPSserial.available() > 0) {
    char c = GPSserial.read();
    gps.encode(c);
  }

  if (millis() - ultimaTransmissao >= intervaloTransmissao) {
    ultimaTransmissao = millis();

    double latitude  = gps.location.isValid() ? gps.location.lat() : 0.0;
    double longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
    double altitude  = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;

    float temperature = bme.readTemperature();
    float humidity    = bme.readHumidity();
    float pressure    = bme.readPressure() / 100.0F;

    packet =
      "LAT:"  + String(latitude, 6) +
      ",LON:" + String(longitude, 6) +
      ",ALT:" + String(altitude, 2) +
      ",TMP:" + String(temperature, 1) +
      ",HUM:" + String(humidity, 1) +
      ",PRS:" + String(pressure, 1);

    Serial.println();
    Serial.println("================================");
    Serial.print("Latitude: "); Serial.println(latitude, 6);
    Serial.print("Longitude: "); Serial.println(longitude, 6);
    Serial.print("Altitude GPS: "); Serial.print(altitude, 2); Serial.println(" m");
    Serial.print("Temperatura: "); Serial.print(temperature); Serial.println(" C");
    Serial.print("Humidade: "); Serial.print(humidity); Serial.println(" %");
    Serial.print("Pressão: "); Serial.print(pressure); Serial.println(" hPa");
    
    Serial.print("Satélites: "); Serial.println(gps.satellites.value());
    Serial.print("Chars processados: "); Serial.println(gps.charsProcessed());
    Serial.print("GPS válido: "); Serial.println(gps.location.isValid() ? "SIM" : "NAO");
    
    Serial.println();
    Serial.print("Pacote: "); Serial.println(packet);

    int state = radio.transmit(packet);
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println("LoRa enviado com sucesso");
    } else {
      Serial.print("Erro envio LoRa: "); Serial.println(state);
    }
    Serial.println("================================");
  }
}