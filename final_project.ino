#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <SoftwareSerial.h>
#define BME280_ADDRESS (0x77) // I2C address of the sensor
#define BME280_REGISTER_CHIPID (0xD0) // ChipID register address
#define BME280_REGISTER_CONTROLHUMID (0xF2) // Humidity configuration register address
#define BME280_REGISTER_CONTROL (0xF4) // Measurement configuration register address
#define BME280_REGISTER_TEMPDATA (0xFA) // Temperature data register address
#define BME280_REGISTER_PRESSUREDATA (0xF7)
#define BME280_REGISTER_HUMDATA (0xFD)
#define DEBUG true

//-----------------------------------------------------------------------------------------------------

typedef struct
{
  uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
  uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3; int16_t dig_P4; int16_t dig_P5; int16_t dig_P6;
  int16_t dig_P7; int16_t dig_P8; int16_t dig_P9;
  uint8_t dig_H1; int16_t dig_H2; uint8_t dig_H3; int16_t dig_H4; int16_t dig_H5; int8_t dig_H6;
} bme280_calib_data;

bme280_calib_data _bme280_calib = {28398, 26500, 50, 36953, -10772, 3024, 7550, 6, -7, 9900, -10230, 4285,
                                   75, 368, 0, 302, 0, 30
                                  };
int32_t t_fine;
uint32_t raw_pressure;
uint32_t raw_temperature;

float humidity;
double pressure;
float temperature;

//-----------------------------------------------------------------------------------------------------

float compensateTemperature(int32_t adc_T) 
{
  int32_t var1, var2;
  adc_T >>= 4;
  var1 = ((((adc_T >> 3) - ((int32_t)_bme280_calib.dig_T1 << 1))) *
          ((int32_t)_bme280_calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bme280_calib.dig_T1))) >> 12) *
          ((int32_t)_bme280_calib.dig_T3)) >> 14;
  t_fine = var1 + var2;
  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

float compensatePressure(int32_t adc_P) 
{
  int64_t var1, var2, p;
  adc_P >>= 4;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bme280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bme280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bme280_calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_bme280_calib.dig_P1) >> 33;
  if (var1 == 0) {
    return 0;
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bme280_calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)_bme280_calib.dig_P7) << 4);
  return (float)p / 256;
}

float compensateHumidity(int32_t adc_H) 
{
  int32_t v_x1_u32r;
  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)_bme280_calib.dig_H4) << 20) -
                  (((int32_t)_bme280_calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
               (((((((v_x1_u32r * ((int32_t)_bme280_calib.dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)_bme280_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                  ((int32_t)2097152)) * ((int32_t)_bme280_calib.dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                             ((int32_t)_bme280_calib.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  float h = (v_x1_u32r >> 12);
  return h / 1024.0;
}

//-----------------------------------------------------------------------------------------------------

void readStuff()
{
  Wire.beginTransmission((uint8_t)BME280_ADDRESS);
  Wire.write((uint8_t)BME280_REGISTER_TEMPDATA);
  Wire.endTransmission();
  
  Wire.requestFrom((uint8_t)BME280_ADDRESS, (byte)3);
  uint32_t raw_temperature;
  raw_temperature = Wire.read();
  raw_temperature <<= 8;
  raw_temperature |= Wire.read();
  raw_temperature <<= 8;
  raw_temperature |= Wire.read();

  temperature = compensateTemperature(raw_temperature);
  Serial.print("Temperature: ");
  Serial.println(temperature);

  Wire.beginTransmission((uint8_t)BME280_ADDRESS);
  Wire.write((uint8_t)BME280_REGISTER_PRESSUREDATA);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)BME280_ADDRESS, (byte)3);

  raw_pressure = Wire.read();
  raw_pressure <<= 8;
  raw_pressure |= Wire.read();
  raw_pressure <<= 8;
  raw_pressure |= Wire.read();

  pressure = compensatePressure(raw_pressure);
  Serial.print("Pressure: ");
  Serial.println(pressure);

  Wire.beginTransmission((uint8_t)BME280_ADDRESS);
  Wire.write((uint8_t)BME280_REGISTER_HUMDATA);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)BME280_ADDRESS, (byte)3);
  uint32_t raw_hum;
  raw_hum = Wire.read();
  raw_hum <<= 8;
  raw_hum |= Wire.read();

  humidity = compensateHumidity(raw_hum);
  Serial.print("Humidity: ");
  Serial.println(humidity);

  Serial.println("------------------------------------------------------------------");
  delay (1000);
}

void setTheStuffUp()
{
  Serial1.begin(115200);
  Serial.begin(115200);
  Wire.begin();
  delay(500);

  Wire.beginTransmission((uint8_t)BME280_ADDRESS);
  Wire.write((uint8_t)BME280_REGISTER_CHIPID);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BME280_ADDRESS, (byte)1);
  uint8_t value = Wire.read();
  Serial.print("Registru Chip_Id (0xD0): ");
  Serial.println(value);

  Wire.beginTransmission((uint8_t)BME280_ADDRESS);
  Wire.write((uint8_t)BME280_REGISTER_CONTROLHUMID);
  Wire.write((uint8_t)0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission((uint8_t)BME280_ADDRESS);
  Wire.write((uint8_t)BME280_REGISTER_CONTROL);
  Wire.write((uint8_t)0xB7);
  Wire.endTransmission();
}

//-----------------------------------------------------------------------------------------------------
/* The functions relevant to wifi module */

String sendData(String command, const int timeout, boolean debug) 
{
  String response = "";
  Serial1.print(command); // send command to the esp8266
  long int time = millis();
  while ((time + timeout) > millis()) 
  {
    while (Serial1.available()) 
    {
      char c = Serial1.read(); // read next char
      response += c;
    }
  }
  
  if (debug) 
  {
    Serial.print(response);
  }
  return response;
}

unsigned long readSensor()
{
  return millis();
}

float readHumidity()
{
  Wire.requestFrom((uint8_t)BME280_ADDRESS, (byte)3);
  uint32_t rawhum;
  rawhum = Wire.read();
  rawhum <<= 8;
  rawhum |= Wire.read();
  float humidity1 = compensateHumidity(rawhum);
  return humidity1;
}

double readPressure()
{
  double pressure1 = compensatePressure(raw_pressure);
  return pressure1;
}

float readTemp()
{
  Wire.requestFrom((uint8_t)BME280_ADDRESS, (byte)3);
  raw_temperature = Wire.read();
  raw_temperature <<= 8;
  raw_temperature |= Wire.read();
  raw_temperature <<= 8;
  raw_temperature |= Wire.read();
  float temperature = compensateTemperature(raw_temperature);
  return temperature;
}

void initializeWiFiThing()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  sendData("AT+RST\r\n", 2000, false); // reset module
  sendData("AT+CWMODE=2\r\n", 1000, false); // configure as access point
  sendData("AT+CIFSR\r\n", 1000, DEBUG); // get ip address
  sendData("AT+CWSAP?\r\n", 2000, DEBUG); // get SSID info (network name)
  sendData("AT+CIPMUX=1\r\n", 1000, false); // configure for multiple connections
  sendData("AT+CIPSERVER=1,80\r\n", 1000, false); // turn on server on port 80
}

void doTheWiFiThing()
{
  if (Serial1.available()) 
  {
    if (Serial1.find("+IPD,")) 
    {
      delay(500);
      int connectionId = Serial1.read() - 48; // read() function returns
      // ASCII decimal value and 0 (the first decimal number) starts at 48
      String webpage = "<h1>Hello World!</h1><a href=\"/l0\"><button>ON</button></a>";
      String cipSend = "AT+CIPSEND=";
      cipSend += connectionId;
      cipSend += ",";
      webpage += "<a href=\"/l1\"><button>OFF</button></a>";

      if (readSensor() > 0)
      {
        webpage += "<h2>Temperature:</h2>";
        char mytemp[5];
        String stTemp;
        float themp=readTemp();
        stTemp = String(temperature);
        webpage += stTemp;
        
        char mypressure[5];
        String pres;
        double thePressure = readPressure();
        sprintf(mypressure,"%d",thePressure);
        webpage += "<h2>Pressure: </h2>";
        pres = String(thePressure);
        webpage += pres;

        char myhum[5];
        String humid;
        float theHumidity = readHumidity();
        sprintf(myhum,"%d",theHumidity);
        webpage += "<h2>Humidity: </h2>";
        humid = String(humidity);
        webpage += humid;
      }
      cipSend += webpage.length();
      cipSend += "\r\n";
      sendData(cipSend, 100, DEBUG);
      sendData(webpage, 150, DEBUG);

      String closeCommand = "AT+CIPCLOSE=";
      closeCommand += connectionId; // append connection id
      closeCommand += "\r\n";
      sendData(closeCommand, 300, DEBUG);
    }
  }
}


//-----------------------------------------------------------------------------------------------------

void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  Wire.begin();
  delay(500);

  Wire.beginTransmission((uint8_t)BME280_ADDRESS);
  Wire.write((uint8_t)BME280_REGISTER_CHIPID);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BME280_ADDRESS, (byte)1);
  uint8_t value = Wire.read();
  Serial.print("Registru Chip_Id (0xD0): ");
  Serial.println(value);

  Wire.beginTransmission((uint8_t)BME280_ADDRESS);
  Wire.write((uint8_t)BME280_REGISTER_CONTROLHUMID);
  Wire.write((uint8_t)0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission((uint8_t)BME280_ADDRESS);
  Wire.write((uint8_t)BME280_REGISTER_CONTROL);
  Wire.write((uint8_t)0xB7);
  Wire.endTransmission();
  initializeWiFiThing();
}

void loop() {
  readStuff();
  doTheWiFiThing();
}