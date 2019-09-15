
#include "DHT.h"

// Temperature / Humidity sensor configuration
#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

// Soil moisture sensor configuration

#define soilMeasurePin A0 // Analog pin connected to the sensor's signal outptu
// Rather than powering the sensor through the 3.3V or 5V pins, 
// we'll use a digital pin to power the sensor. This will 
// prevent corrosion of the sensor as it sits in the soil. 
// Variable for soil moisture power pin
#define soilPowerPin 7

// Relay configuration
#define relaySignalPin 8 // Variable for pump relay signal pin

// Variable definitions temp / humidity

// Interval between temperature measurements in milliseconds
unsigned long tempMeasurementInterval = 6000;
unsigned long lastTemperatureMeasurementMillis = 0;

// Variable definitions misting
int soilMoisture = 0; // Value for storing moisture measurement 

// Interval between soil mouisture measurements in milliseconds
unsigned long soilMeasurementInterval = 600000;


// Interval the pump will run for in milliseconds
unsigned long mistingInterval = 120000; 

// If this threshold is reached, we conclude the moisture sensor is not in soil. 
// In this case we will switch to initermittent misting at fixed intervals
int soilMoistureSensorOutOfSoilThreshold = 0;

// Threshold value for soil moisture, if soil moisture levels drop below this, we will mist.
int soilMoistureThreshold = 200; 

unsigned long mistingStartMillis = 0;
unsigned long waitingPeriodStartMillis = 0;

bool isMisting = false;
bool isWaiting = false;

void setup() 
{
  Serial.begin(9600); // open serial over USB

  pinMode(soilPowerPin, OUTPUT); // Set D7 as an OUTPUT
  digitalWrite(soilPowerPin, LOW); // Set to LOW so no power is flowing through the sensor

  pinMode(relaySignalPin, OUTPUT); // Set D8 as an OUTPUT
  digitalWrite(relaySignalPin, LOW); // Set to LOW so relay is switched OFF

  dht.begin();
}

void loop() 
{
  unsigned long currentMillis = millis();
  mistingLoop(currentMillis);
  tempHumidityLoop(currentMillis);
}

// Temp / Humidity code
void tempHumidityLoop(unsigned long currentMillis) {
  if (currentMillis > lastTemperatureMeasurementMillis + tempMeasurementInterval || lastTemperatureMeasurementMillis == 0) {
    measureTemperatureAndHumidity();
    lastTemperatureMeasurementMillis = currentMillis;
  }
}

void measureTemperatureAndHumidity() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)){
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(F("Heat index: "));
  Serial.print(hic);
  Serial.println(F("°C "));
}

// Misting code
void mistingLoop(unsigned long currentMillis) {
   if (isWaiting == false && isMisting == false) {
     soilMoisture = readSoil();
     if (soilMoisture < soilMoistureThreshold) {
        startMisting(currentMillis);
     } else {
        startWaiting(currentMillis);
     }
  } else if (isWaiting == true) {
    if (currentMillis > waitingPeriodStartMillis + soilMeasurementInterval) {
        stopWaiting();
    }
  } else if (isMisting == true) {
    if (currentMillis > mistingStartMillis + mistingInterval) {
        stopMisting();
        startWaiting(currentMillis); // always wait one cycle directly after misting.
    }
  }
}

void startMisting(unsigned long currentMillis) {
   isMisting = true;
   mistingStartMillis = currentMillis;
   startPump();
}

void stopMisting() {
  stopPump();
  isMisting = false;
}

void startWaiting(unsigned long currentMillis) {
  isWaiting = true;
  waitingPeriodStartMillis = currentMillis;
}

void stopWaiting() {
  isWaiting = false;
}

// Function used to run the pump (by switching the relay)
void startPump() 
{
  digitalWrite(relaySignalPin, HIGH); // turn relay ON
}

// Function used to stop the pump (by switching the relay)
void stopPump() 
{
  digitalWrite(relaySignalPin, LOW);  // turn relay OFF
}

// Function used to get the soil moisture content
int readSoil()
{
  digitalWrite(soilPowerPin, HIGH); // turn power to sensor ON
  delay(10); // wait 10 milliseconds 
  int val = analogRead(soilMeasurePin); // Read the SIG value form sensor 
  digitalWrite(soilPowerPin, LOW);  // Turn power to sensor OFF
  Serial.print("Soil Moisture = ");
  Serial.println(val);
  return val; // Send current moisture value
}
