#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>

#define BME_ADDRESS 0x76 // I2C address of BME280 (change to 0x77 if needed)

#define NEOPIXEL_PIN PA8 // Pin where NeoPixels are connected
#define NEOPIXEL_COUNT 4 // Number of NeoPixels

#define USER_BUTTON_PIN PC13 // Pin for user button
#define INTERVAL_MS ((uint32_t)5000) // Interval for sensor readings
#define ATM_PA 1013.25 // Standard atmospheric pressure in hPa

Adafruit_BME280 bme; // Create BME280 object
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);


HardwareTimer *LogTimer = new HardwareTimer(TIM2); // Timer for logging
HardwareTimer *ButtonTimer = new HardwareTimer(TIM3); // Timer for button 


volatile bool log_data_flag = false;
uint32_t lastButtonPress=0;
volatile int displayMode=0; // 0: Temp, 1: Humidity, 2: Pressure, 3: Altitude
uint32_t lastTime=0;
uint32_t lastButtonPress=0;


//Serial begin give baudrate
void printSensorValues(float tempC,float tempF,float humidity, float pressure);
void updateNeopixels(int mode);
void updateSSD(float value, int mode);
float convertCtoF(float c);
float convertPatoAtm(float pa);

void LogTimerInterruptHandler(){
  log_data_flag = true;
}

void ButtonTimerInterrupt(){
  if(digitalRead(USER_BUTTON_PIN)==LOW){  //if button pressed

    displayMode= (displayMode + 1) %4; //go through sensors
    updateNeopixels(displayMode);
  }
}


void setup() {
  Serial.begin(115200);

}


void loop() { 
  float tempC=bme.readTemperature();
  float humidity=bme.readHumidity();
  float pressure=bme.readPressure()/100.0F; // Convert to hPa
  float tempf=convertCtoF(tempC);
  float pressureAtm=convertPatoAtm(pressure);



}


void printValues(float tempC,float tempF,float humidity, float pressureAtm) {
  Serial.print(F("Temp(C) = "));
    Serial.print(tempC, 2);

    Serial.print(F(", Temp(F) = "));
    Serial.print(tempF, 2);

    Serial.print(F(", RelHum(%) = "));
    Serial.print(humidity, 2);

    Serial.print(F(", Press(atm) = "));
    Serial.print(pressureAtm, 4); 

    Serial.println();
}

float convertCtoF(float c) {
  return c * 9.0 / 5.0 + 32.0;
}


float convertPatoAtm(float pa) {
  return pa / 1013.25;
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
}
