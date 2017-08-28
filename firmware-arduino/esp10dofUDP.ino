/*
 * Copyright (C) 2016 XXXXXXXX
 *
 * This file is subject to the terms and conditions defined in file 'LICENSE.txt', 
 * which is part of this source code package.
 */

/**
 * @file    esp_10dof.h
 * @brief   Main file
 */
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

//============================  Change these to match your setup ===============================
const char* ssid = "esp";                      // your network SSID (name)
const char* pass = "password";                    // your network password
static const char *RemoteUDP = "192.168.100.114";   // IP of target device/computer
static const int RemoteUdpPort = 9999;              // remote port number of target device/computer
//===============================================================================================

unsigned int localPort = 8888;
char packetBuffer[255];
//unsigned int localPort = 9999;


#include <Wire.h>
#include <SPI.h>
#include "lsm9ds1.h"
//#include "bmp280.h"
#include "imu.h"

#define LED 2


/* Udp class instance to let us send and receive packets over Udp*/
WiFiUDP Udp;

/* LSM9DS1 class instance */
LSM9DS1 imu;

/* BMP280 class instance */
//BMP280 bar;

/* I2C address */
#define LSM9DS1_M   0x1E  // I2C 7-bit address for magnetometer
#define LSM9DS1_AG  0x6B  // I2C 7-bit address for accelerometer/gyroscope

/* timer interrupt settings */
#define PERIOD_MS 500 		  // timer interrupt period in ms

/* macro to convert ms to esp ticks */
#define MS_TO_ESPTICK(x)					( x * 80000L )	// 80MHz == 1sec

const uint32_t period_ms = MS_TO_ESPTICK(PERIOD_MS);

bool tick_flag = false;
uint8_t led_toggle = 0, led_counter = 0;
float accel, gyro, mag, temp, pressure, altitude;


//======================================================
void wifisetup() {

  WiFi.begin(ssid, pass);  //setting up Station
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.print(":");
  Serial.println(localPort);

  Udp.begin(localPort);
}

//======================================================
void Udpsend() {

Udp.beginPacket(RemoteUDP, RemoteUdpPort);

  // print gyro
  Udp.print("Gyro: ");
  Udp.print(imu_gx, 2);
  Udp.print(", ");
  Udp.print(imu_gy, 2);
  Udp.print(", ");
  Udp.print(imu_gz, 2);
  Udp.print(" deg/s ");

  // print accel
  Udp.print("Accel: ");
  Udp.print(imu_ax, 2);
  Udp.print(", ");
  Udp.print(imu_ay, 2);
  Udp.print(", ");
  Udp.print(imu_az, 2);
  Udp.print(" g ");

  // print mag
  Udp.print("Accel: ");
  Udp.print(imu_mx, 2);
  Udp.print(", ");
  Udp.print(imu_my, 2);
  Udp.print(", ");
  Udp.print(imu_mz, 2);
  Udp.print(" gauss");

 Udp.endPacket(); 
}

//======================================================

/**
 * @brief   Initializes on-board sensors
 * @return  Nothing
 */
void timer_setup()
{
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(timer0_ISR);
  timer0_write(ESP.getCycleCount() + period_ms);
  interrupts();
}

/**
 * @brief   Print sensor data
 * @return  Nothing
 */
void printData()
{
  // print gyro
  Serial.print("Gyro: ");
  Serial.print(imu_gx, 2);
  Serial.print(", ");
  Serial.print(imu_gy, 2);
  Serial.print(", ");
  Serial.print(imu_gz, 2);
  Serial.println(" deg/s");

  // print accel
  Serial.print("Accel: ");
  Serial.print(imu_ax, 2);
  Serial.print(", ");
  Serial.print(imu_ay, 2);
  Serial.print(", ");
  Serial.print(imu_az, 2);
  Serial.println(" g");

  // print mag
  Serial.print("Accel: ");
  Serial.print(imu_mx, 2);
  Serial.print(", ");
  Serial.print(imu_my, 2);
  Serial.print(", ");
  Serial.print(imu_mz, 2);
  Serial.println(" gauss");

/*
  // print barometer
  Serial.print("Temperature: ");
  Serial.print(temp, 2);
  Serial.print(" *C, ");
  Serial.print("Pressure: ");
  Serial.print(pressure, 2);
  Serial.print(" Pa, ");
  Serial.print("Altitude: ");
  Serial.print(altitude, 2);
  Serial.println(" m, ");
*/
  Serial.println();
}

/*****************************************************************************
 * Setup
 ****************************************************************************/
void setup()
{
  pinMode(LED, OUTPUT);     				// initialize LED pin as output
  Serial.begin(115200);

  sensor_setup();										// configure sensors
  timer_setup();										// configure timer

  wifisetup();  
}

/*****************************************************************************
 * Main Loop
 ****************************************************************************/
void loop()
{
  if (tick_flag) {
  	tick_flag = false;

    getAccel(0);
    getGyro(0);
    getMag(0);

//  	temp = bar.readTemperature();
//    pressure = bar.readPressure();
//    altitude = bar.readAltitude(1013.25);

  	printData();
    Udpsend();
  }
}




/*****************************************************************************
 * Interrupt Service Routines (ISR)
 ****************************************************************************/

/**
 * @brief   Timer o interrupt service routine
 */
void timer0_ISR ()
{
  tick_flag = true;
  if (led_counter++ >= 1000/period_ms) {
  	led_counter = 0;
  	led_toggle = (led_toggle == 1) ? 0 : 1;
  	digitalWrite(LED, led_toggle);
  }
  timer0_write(ESP.getCycleCount() + period_ms);
}

/*****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief   Initializes on-board sensors
 * @return  Nothing
 */
void sensor_setup()
{
  /* initialize 9-dof imu */
  // Before initializing the IMU, there are a few settings we may need to adjust.
  // Use the settings struct to set the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
/*
//------------------------------------------------------ 
  // The above lines will only take effect AFTER calling imu.begin(), which verifies
  // communication with the IMU and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1)
      ;
  }
*/
//------------------------------------------------------
  /* initialize barometer 
  if (!bar.begin())
  {
    Serial.println("Failed to communicate with BMP280.");
    while (1)
      ;
*/  }


