#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// ================= PINOS SPI =================
#define SPI_SCLK 36
#define SPI_MISO 37
#define SPI_MOSI 35

#define SD_CS    47
#define IMU_CS   34

// ================= OBJETOS =================
SPIClass spi(FSPI);
File logFile;

// ================= TIMERS =================
unsigned long lastSDWrite = 0;
const unsigned long SD_INTERVAL = 100;

// =========================================================
void selectSD() {
  digitalWrite(IMU_CS, HIGH);
  digitalWrite(SD_CS, LOW);
}

void selectIMU() {
  digitalWrite(SD_CS, HIGH);
  digitalWrite(IMU_CS, LOW);
}

void deselectAll() {
  digitalWrite(SD_CS, HIGH);
  digitalWrite(IMU_CS, HIGH);
}

// =========================================================
void setup() {
  Serial.begin(115200);

  pinMode(SD_CS, OUTPUT);
  pinMode(IMU_CS, OUTPUT);

  deselectAll();

  spi.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);

  // ================= SD INIT =================
  selectSD();
  if (!SD.begin(SD_CS, spi)) {
    Serial.println("Erro SD!");
  } else {
    Serial.println("SD OK");
  }
  deselectAll();

  // ================= IMU INIT =================
  selectIMU();
  // Aqui você inicializaria o QMI8658 via SPI
  Serial.println("IMU pronto (placeholder)");
  deselectAll();
}

// =========================================================
void loop() {
  unsigned long now = millis();

  // =====================================================
  // 📊 LEITURA IMU (SPI)
  // =====================================================
  selectIMU();

  // ⚠️ Substituir por leitura real do QMI8658 via SPI
  float ax = random(-100, 100) / 10.0;
  float ay = random(-100, 100) / 10.0;
  float az = random(-100, 100) / 10.0;

  deselectAll();

  // =====================================================
  // 💾 GRAVAÇÃO SD
  // =====================================================
  if (now - lastSDWrite >= SD_INTERVAL) {
    lastSDWrite = now;

    selectSD();

    logFile = SD.open("/log.csv", FILE_APPEND);
    if (logFile) {
      logFile.printf("%lu,%.2f,%.2f,%.2f\n", now, ax, ay, az);
      logFile.close();
    }

    deselectAll();
  }
}