#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <stdio.h>

#define BME_ADDRESS 0x76 // I2C address of BME280 (change to 0x77 if needed)
#define NEOPIXEL_PIN PA8 // Pin where NeoPixels are connected
#define NEOPIXEL_COUNT 4 // Number of NeoPixels

#define USER_BUTTON_PIN PC13 // Pin for user button
#define INTERVAL_MS ((uint32_t)5000) // Interval for sensor readings
#define ATM_PA 1013.25 // Standard atmospheric pressure in hPa
#define SENSOR_READ_INTERVAL_MS 5000 // Interval for sensor readings
#define BUTTON_DEBOUNCE_DELAY_MS 50   // Debounce delay for the button

Adafruit_BME280 bme = Adafruit_BME280(); // Create BME280 object
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

volatile int displayMode=0; // 0: Temp, 1: Humidity, 2: Pressure, 3: Altitude
uint32_t lastButtonPress=0;
uint32_t lastTime=0;
uint32_t lastSensorReadTime=0;
uint32_t lastButtonPressTime=0;


//Serial begin give baudrate
void printSerialValues(float tempC,float tempF,float humidity, float pressure);
void printValues(float tempC,float tempF,float humidity, float pressureAtm);
void updateNeopixels(int mode);
void updateSSD(float value, int mode);
float convertCtoF(float c);
float convertPatoAtm(float pa);
void ButtonTimerInterrupt(){
    lastButtonPressTime = millis();
    displayMode = (displayMode + 1) % 4; // Cycle through modes
    updateNeopixels(displayMode);
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect. Needed for native USB only
  displayMode= (displayMode + 1) %4; //go through sensors
  updateNeopixels(displayMode);
  pinMode(USER_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(USER_BUTTON_PIN), ButtonTimerInterrupt, FALLING);

  Wire.begin();
  if (!bme.begin(BME_ADDRESS)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1);
  }
}


void loop() { 
  float tempC = bme.readTemperature();
  float tempF = convertCtoF(tempC);
  float humidity = bme.readHumidity();
  float pressure = bme.readPressure(); // Convert to hPa
  float tempf = convertCtoF(tempC);
  float pressureAtm=convertPatoAtm(pressure);

  delay(SENSOR_READ_INTERVAL_MS);
  printValues(tempC, tempF, humidity, pressureAtm);
}


void printValues(float tempC,float tempF,float humidity, float pressureAtm) {
  char buffer[50];
  sprintf(buffer, "Temp(C) = %.2f, Temp(F) = %.2f, RelHum(%%) = %.2f, Press(atm) = %.4f", tempC, tempF, humidity, pressureAtm);
  Serial.println(buffer);
}

float convertCtoF(float c) {
  return c * 9.0 / 5.0 + 32.0;
}

float convertPatoAtm(float pa) {
  return pa / 1013.25;
  return pa / 1013.25F;
}

void updateNeopixels(int mode) {
  uint32_t color;
  switch(mode) {
    case 0: color=pixels.Color(255,0,0); break; // Red for temperature
    case 1: color=pixels.Color(0,0,255); break; // Blue for humidity
    case 2: color=pixels.Color(0,255,0); break; // Green for pressure
    case 3: color=pixels.Color(255,255,0); break; // Yellow for altitude
    default: color=pixels.Color(255,255,255); break; // White as fallback
  }
  for(int i=0;i<NEOPIXEL_COUNT;i++) {
    pixels.setPixelColor(i,color);
  }
  pixels.show();
}

void updateSSD(float value, int mode) {
  // Placeholder function to update SSD display
  // Implementation depends on specific SSD library used
  // For example: display.println(value); display.display();
}
