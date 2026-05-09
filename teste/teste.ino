#include <Wire.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

void setup() {

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("Teste BME280");

  Wire.begin();

  bool status = bme.begin(0x76);

  if (!status) {

    Serial.println("0x76 falhou");

    status = bme.begin(0x77);

    if (!status) {

      Serial.println("BME280 NÃO encontrado");

      while (1);
    }
  }

  Serial.println("BME280 OK");
}

void loop() {

  Serial.print("Temp: ");
  Serial.print(bme.readTemperature());
  Serial.println(" C");

  Serial.print("Hum: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("Press: ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.println();

  delay(2000);
}