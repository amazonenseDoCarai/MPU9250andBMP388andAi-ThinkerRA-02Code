#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// =====================================================
// BME280
// =====================================================
Adafruit_BME280 bme;

// =====================================================
// GPS
// =====================================================
TinyGPSPlus gps;
HardwareSerial GPSserial(1);

// =====================================================
// SX1262 LoRa
// Ajustar se necessário conforme a revisão da placa
// =====================================================
#define LORA_CS    18
#define LORA_DIO1  14
#define LORA_RST   23
#define LORA_BUSY  33

SX1262 radio = new Module(
  10,  // CS
  1,   // DIO1
  5,   // RST
  4    // BUSY
);

// =====================================================
// Variável pacote
// =====================================================
String packet;

// =====================================================
// SETUP
// =====================================================
void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("====================================");
  Serial.println("T-Beam Supreme LoRa GPS BME280");
  Serial.println("====================================");

  // =====================================================
  // I2C
  // Usa automaticamente os pinos SDA/SCL da board
  // =====================================================
  Wire.begin(17, 18);

  Serial.println("I2C iniciado");

  // =====================================================
  // BME280
  // =====================================================
  if (!bme.begin(0x76)) {

    Serial.println("BME280 não encontrado!");
    Serial.println("Tentando endereço 0x77...");

    if (!bme.begin(0x77)) {

      Serial.println("BME280 falhou.");
      while (true);
    }
  }

  Serial.println("BME280 iniciado");

  // =====================================================
  // GPS
  // RX=34
  // TX=12
  // =====================================================
  GPSserial.begin(9600, SERIAL_8N1, 34, 12);

  Serial.println("GPS iniciado");

  // =====================================================
  // LoRa SX1262
  // =====================================================
  Serial.print("Inicializando LoRa ... ");

  SPI.begin(12, 13, 11, 10); // SCLK, MISO, MOSI, CS
  int state = radio.begin(
    868.0,   // frequência MHz
    125.0,   // bandwidth
    9,       // spreading factor
    7,       // coding rate
    0x12,    // sync word
    22,      // potência TX
    8,       // preâmbulo
    1.6      // TCXO
  );

  if (state == RADIOLIB_ERR_NONE) {

    Serial.println("OK");

  } else {

    Serial.print("Erro LoRa: ");
    Serial.println(state);

    while (true);
  }
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  // =====================================================
  // Ler GPS
  // =====================================================
  while (GPSserial.available()) {

    gps.encode(GPSserial.read());
  }

  float latitude  = 0.0;
  float longitude = 0.0;

  if (gps.location.isValid()) {

    latitude  = gps.location.lat();
    longitude = gps.location.lng();
  }

  // =====================================================
  // Ler BME280
  // =====================================================
  float temperature = bme.readTemperature();
  float humidity    = bme.readHumidity();
  float pressure    = bme.readPressure() / 100.0F;

  // =====================================================
  // Criar pacote LoRa
  // =====================================================
  packet =
    "LAT:"  + String(latitude, 6) +
    ",LON:" + String(longitude, 6) +
    ",TMP:" + String(temperature, 1) +
    ",HUM:" + String(humidity, 1) +
    ",PRS:" + String(pressure, 1);

  // =====================================================
  // Mostrar no Serial Monitor
  // =====================================================
  Serial.println();
  Serial.println("================================");

  Serial.print("Latitude: ");
  Serial.println(latitude, 6);

  Serial.print("Longitude: ");
  Serial.println(longitude, 6);

  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Humidade: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Pressão: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.println();
  Serial.print("Pacote: ");
  Serial.println(packet);

  // =====================================================
  // Enviar via LoRa
  // =====================================================
  int state = radio.transmit(packet);

  if (state == RADIOLIB_ERR_NONE) {

    Serial.println("LoRa enviado com sucesso");

  } else {

    Serial.print("Erro envio LoRa: ");
    Serial.println(state);
  }

  delay(5000);
}