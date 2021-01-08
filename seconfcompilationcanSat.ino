#include<Wire.h>
#include<Adafruit_BMP280.h>
Adafruit_BMP280 bmp;  //I2C
float pressure;
void setup() {
  Serial.begin(115200);
  bmp1();


}

void loop() {
    Serial.print("xx"); //team ID
    Serial.print("S1") ;     //science payload 1
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature(),1);
    Serial.println(" *C");
    pressure = bmp.readPressure(); //reading pressure
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25),1); /* Adjusted to local forecast! */
    Serial.println(" m");

}

void bmp1(){
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}               
