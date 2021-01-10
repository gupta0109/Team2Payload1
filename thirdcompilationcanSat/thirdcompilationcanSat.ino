#include<Wire.h>
#include<Adafruit_BMP280.h>
#include<stdlib.h>
Adafruit_BMP280 bmp;  //I2C
float altitude, temp;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
float rpmX,rpmY,rpmZ;
int packet_count=0;
char altstr[5];
char tempstr[5];
char packstr[5];
void setup() {
  Serial.begin(115200);
  Wire.begin();//RPM Measurement
  setupMPU();
  bmp1();
}

void loop() {
    Serial.print("0000,");//Team ID
    Serial.print(" ,");//Mission Time
    packet_count++;
    dtostrf(packet_count,5,0,packstr);
    Serial.println(packstr);
    Serial.print(",");
    Serial.print("S1,");//science payload 1 
    delay(115200);
    recordGyroRegisters();//RPM Measurement
    RPMMeasurement();
    BMPMeasurement();
}

void bmp1(){
  if (!bmp.begin(0x76)) {
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}               
void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}
void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rpmX=rotX/6;
  rotY = gyroY / 131.0; 
  rpmY=rotY/6;
  rotZ = gyroZ / 131.0;
  rpmZ=rotZ/6;
}
void RPMMeasurement() {
  Serial.print("Gyro (RPM)");
  Serial.print(" X=");
  Serial.print(rpmX);
  Serial.print(" Y=");
  Serial.print(rpmY);
  Serial.print(" Z=");
  Serial.print(rpmZ);
}
void BMPMeasurement()
{
    temp=bmp.readTemperature();
    dtostrf(temp,5,1,tempstr);
    Serial.println(tempstr);
    altitude=bmp.readAltitude(1013.25);
    dtostrf(altitude,5,1,altstr);
    Serial.println(altstr);
    
}
