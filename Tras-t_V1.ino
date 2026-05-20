#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ST7735.h"
#include <MPU6050.h>
#include "SPI.h"

#define REPORTING_PERIOD_MS 1000
#define TFT_CS 5
#define TFT_RST 4
#define TFT_DC 2

// objetos para los sensores
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
MPU6050 mpu;
PulseOximeter pox;

uint32_t lastReport_CP = 0;
uint32_t lastReport = 0;
uint32_t lastBeat = 0;

//variables del MPU
int16_t ax, ay, az;
int16_t gx, gy, gz;
float accelX, accelY, accelZ; //valores en m/s2
float gyroX, gyroY; //valores en °/s
float accelMagnitude;

// Variables globales para el conteo de pasos
float gyroZ = 0;       // Processed gyroscope Z-axis reading (float)
float filt1 = 0;       // Filtered gyro data

// Parametros de filtro para el movimiento promedio
const int MA_SIZE = 5;                // Moving average window size
float gyroBuffer[MA_SIZE] = {0};      // Buffer for gyroZ
int bufferIndex = 0;                   // Current index in the buffer
float gyroSum = 0;                     // Sum of the buffer elements

// variables de conteo de pasos
int stepCount = 0;                     // Total de numero de pasos
float threshold = 0.6;                 // Umbral de detección
float threshold_acc = 9;             // umbral de detección para accel

// variables para la cadencia de pasos
int cad_pasos_act = 0;
float duracion_reporte_cadP = 10000;

// funcion para promediar el giro z
void filterData() {
    // Subtract the oldest value from the sum
    gyroSum -= gyroBuffer[bufferIndex];
    
    // Add the new gyroZ value to the buffer and sum
    gyroBuffer[bufferIndex] = gyroZ;
    gyroSum += gyroZ;
    
    // Move to the next index, wrapping around if necessary
    bufferIndex = (bufferIndex + 1) % MA_SIZE;
    
    // Calculate the moving average
    filt1 = gyroSum / MA_SIZE;
}

void onBeatDetected() {
  //Serial.println("♥ Latido detectado");
}

// funcion para detectar los pasos bajo un determinado umbral
void detectSteps() {
    static float prevFilt1 = 0; // Previous filtered gyroscope reading

    // Detect a step when the filtered gyroZ crosses the threshold from below
    if (filt1 > threshold && accelMagnitude > threshold_acc && prevFilt1 <= threshold) {
        stepCount++;
    }

    // Update the previous filtered gyroscope reading
    prevFilt1 = filt1;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // iniciamos el display
  tft.initR(INITR_BLACKTAB); // Inicializa la pantalla
  tft.setRotation(1);        // Ajusta la orientación (0–3)
  tft.fillScreen(ST77XX_BLACK); // Limpia la pantalla en negro

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 30);

  // Iniciamos el sensor MAX30100
  Serial.println("Inicializando MAX30100...");
  if (!pox.begin()) {
    Serial.println("ERROR: MAX30100 no encontrado");
    while (1);
  }

  pox.setOnBeatDetectedCallback(onBeatDetected);

  // Corriente de LEDs (importante para SpO2)
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

  // Inicializar MPU6050
  Serial.println("Inicializando MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Error: No se detecta el MPU6050");
    while (1);
  }
}

void loop() {
  pox.update();
  if (millis() - lastReport > REPORTING_PERIOD_MS) {
    Serial.println("--------------------------------------");
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 30);

    tft.println("Pulso: ");
    tft.print(pox.getHeartRate());
    tft.print(" bpm | SpO2: ");
    tft.print(pox.getSpO2());
    tft.println(" %");

    //obtenemos datos del giroscopio
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convertir aceleración a m/s²
    accelX = (ax / 16384.0) * 9.81;
    accelY = (ay / 16384.0) * 9.81;
    accelZ = (az / 16384.0) * 9.81;
    accelMagnitude = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));

    // Convertir giroscopio a grados/segundo
    gyroX = gx / 131.0;
    gyroY = gy / 131.0;
    gyroZ = gz / 131.0;

    filterData(); // promediamos el giro Z actual
    detectSteps(); // detectamos si hay un paso

    Serial.print("Acelerómetro: ");
    Serial.print("X="); Serial.print(accelX);
    Serial.print(" Y="); Serial.print(accelY);
    Serial.print(" Z="); Serial.println(accelZ);

    Serial.print("Giroscopio: ");
    Serial.print("X="); Serial.print(gyroX);
    Serial.print(" Y="); Serial.print(gyroY);
    Serial.print(" Z="); Serial.println(gyroX);

    Serial.print("Steps counted: ");
    Serial.println(stepCount);

    lastReport = millis();
  }

  if (millis() - lastReport_CP > duracion_reporte_cadP){

    cad_pasos_act = stepCount/(duracion_reporte_cadP/60000);
    Serial.print("Cadencia de pasos (pasos/min):");
    Serial.println(cad_pasos_act);

    stepCount = 0;
    lastReport_CP = millis();
  }
}