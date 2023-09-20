#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <math.h>

#include <SPI.h>
#include <SD.h> 

//bmp includes
#include <Adafruit_BMP280.h>

//defines
#define BMP_ADRESS 0x76
#define BMP_CHIPID 0x58
#define BUZZID 3
#define CS 10

#define GRAVCONST 9.80665
#define alpha 0.5

//Sensors:
MPU6050 mpu(Wire); 
Adafruit_BMP280 bmp; 

// variables for saving
float alt; 
float gx, gy, gz, prevax=0, prevay=0, prevaz=0;
float noise_controlled_ax, noise_controlled_ay, noise_controlled_az; 
float r, p; 
int time; 
bool checkforlaunch = true; 
float initalt, maxAlt, currAlt;

File dataFile, eventFile; 
// Function to write data to a file in CSV format
void writeDataToFile() {
  dataFile = SD.open("SENSOR.csv", FILE_WRITE);
  if (dataFile) {
    // Combine all data into one CSV-formatted string
    // Write the data string to the file
    dataFile.println(String(noise_controlled_ax) + "," + String(noise_controlled_ay) + "," + String(noise_controlled_az) + "," +
                        String(r) + "," + String(p) + "," + String(alt));
    dataFile.close();
  } else {
    tone(BUZZID, 500, 3); 
  }
  if (currAlt >= maxAlt) {
    maxAlt = currAlt; 
  }
  else if (currAlt <=maxAlt) {
    dataFile.close(); 
    eventFile = SD.open("EVENT.TXT", FILE_WRITE); 
    if (eventFile) {
      eventFile.println("Apogee Reached at: " + String(maxAlt) + "," + String(time/100)); 
      eventFile.close();
    } 
  }
  
}

// Time interval between sensor readings (in seconds)

void MPUJOB(); 
void BMPJOB(); 
void CalibrateSensors(); 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(); 
  //calibrate MPU, BMP, MICROSD
  CalibrateSensors();      // Make sure backlight is on
  maxAlt = initalt; 
}

void loop() {
  // put your main code here, to run repeatedly:
  // buzz the buzzer: 
  analogWrite(BUZZID, 200); //TODO: Open this later in flight 
  MPUJOB(); 
  BMPJOB();
  writeDataToFile();
  time = millis(); 
  delay(100); 
}


///@brief Calibrate mpu, bmp and microsd
void CalibrateSensors() {
  tone(BUZZID, 1200, 40); 

  mpu.begin(); 
  mpu.calcGyroOffsets();  
  
  if (!bmp.begin(BMP_ADRESS, BMP_CHIPID)) {
    tone(BUZZID, 1400, 1);
    delay(100); 
    tone(BUZZID, 1400, 1);  
    while (1) delay(10); 
  } 

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,   
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 

  initalt = bmp.readAltitude(1013.25);
   
  if (!SD.begin(CS)) {
    tone(BUZZID, 1400, 3); 
  } else {
      SD.remove("SENSOR.CSV"); 
      SD.remove("EVENT.TXT"); 
  }
}

// ----* Sensors *-----
///@brief Gathers altitude, temperature data. 
void BMPJOB() {
  alt = bmp.readAltitude(1013.25) - initAlt; 
  currAlt = alt;
}

/// @brief Gather MPU6050 Data and perform writes onto separate AX, AZ, AY files on MICROSD
void MPUJOB() {
  mpu.update(); 
  gx = mpu.getGyroX() * 0.01745329251; 
  gy = mpu.getGyroY() * 0.01745329251; 
  gz = mpu.getGyroZ() * 0.01745329251; 
  // apply complementary filter to reduce noise
  noise_controlled_ax = alpha * (mpu.getAccX() * GRAVCONST)  + (1 - alpha) * prevax; 
  noise_controlled_ay = alpha * (mpu.getAccY() * GRAVCONST) + (1 - alpha) * prevay; 
  noise_controlled_az = alpha * (mpu.getAccZ() * GRAVCONST) + (1-alpha) * prevaz; 
  // Roll and Pitch (combining gyro and accel to calculate final roll and pitvh calues)
  r = alpha * (gx + tanf(p) * (sin(r) * gy + cosf(r) * gz) ) + (1.0f - alpha) * (r + (0.1f/ 1000.0f) * (gx + tanf(p) * (sin(r) * gy + cosf(r) * gz) )); 
  p = alpha * (cosf(r) * gy - sinf(r) * gz) + (1.0f - alpha) * (p +(0.1f / 1000.0f) * (cosf(r) * gy - sinf(r) * gz)); 
  //only open when rocket is moving
  prevax = noise_controlled_ax; 
  prevay = noise_controlled_ay; 
  prevaz = noise_controlled_az;
}

