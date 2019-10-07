
#include "DHT.h"

// PIN definitions

// LED configuration
#define normalOperationLEDPin 3
#define forceMistingLEDPin 4
#define stopMistingLEDPin 5
#define soilSensorErrorLEDPin 6

// Push button configuration
#define buttonPin 9

#define soilMeasurePin A0 // Analog pin connected to the sensor's signal outptu
// Rather than powering the sensor through the 3.3V or 5V pins, 
// we'll use a digital pin to power the sensor. This will 
// prevent corrosion of the sensor as it sits in the soil. 
// Variable for soil moisture power pin
#define soilPowerPin 7

// Relay configuration
#define relaySignalPin 8 // Variable for pump relay signal pin


// Temperature / Humidity sensor configuration
#define DHTPIN 11     // Digital pin connected to the DHT sensor
#define DHTPowerPin 10  // Digital pin to power the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

// Push button vaiables

// operation mode of the system, based on push button presses
// 0 = normal
// 1 = force misting
// 2 = stop misting
int operationMode = 0;       
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button


// Variable definitions temp / humidity

// Interval between temperature measurements in milliseconds
unsigned long tempMeasurementInterval = 60000;
unsigned long lastTemperatureMeasurementMillis = 0;

// Variable definitions misting
int soilMoisture = 0; // Value for storing moisture measurement 

// Interval between soil mouisture measurements in milliseconds
unsigned long soilMeasurementInterval = 600000;

// Interval between mistings when sensor is nor working in milliseconds
unsigned long sensorlessWaitingInterval = 43200000; // 12 hours

// Interval the pump will run for in milliseconds
unsigned long mistingInterval = 120000;

// If this threshold is reached, we conclude the moisture sensor is not in soil. 
// In this case we will switch to initermittent misting at fixed intervals
int soilMoistureSensorOutOfSoilThreshold = 0;

// Threshold value for soil moisture, if soil moisture levels drop below this, we will mist.
int soilMoistureMistingThreshold = 200; 
// Threshold value for soil moisture, if soil moisture levels drop below this, we assume sensor has been removed from soil or is broken
// In that case we switch to misting on fixed schedule
int soilMoistureSensorErrorThreshold = 1;

unsigned long mistingStartMillis = 0;
unsigned long waitingPeriodStartMillis = 0;

bool isMisting = false;
bool isWaiting = false;
bool soilSensingFailed = false;

void setup() 
{
  Serial.begin(9600); // open serial over USB

  pinMode(soilPowerPin, OUTPUT); 
  digitalWrite(soilPowerPin, LOW); // Set to LOW so no power is flowing through the sensor

  pinMode(relaySignalPin, OUTPUT); 
  digitalWrite(relaySignalPin, LOW); // Set to LOW so relay is switched OFF

  pinMode(DHTPowerPin, OUTPUT);
  digitalWrite(DHTPowerPin, LOW); // Set to LOW so no power is flowing through the sensor
  
  pinMode(buttonPin, INPUT);
  
  // Configure LEDs
  pinMode(normalOperationLEDPin, OUTPUT);
  digitalWrite(normalOperationLEDPin, HIGH); // Default is normal operation

  pinMode(forceMistingLEDPin, OUTPUT);
  digitalWrite(forceMistingLEDPin, LOW); // Default is normal operation

  pinMode(stopMistingLEDPin, OUTPUT);
  digitalWrite(stopMistingLEDPin, LOW); // Default is normal operation

  pinMode(soilSensorErrorLEDPin, OUTPUT); 
  digitalWrite(soilSensorErrorLEDPin, LOW); // Default is normal operation

  dht.begin();
}

void loop() 
{
  unsigned long currentMillis = millis();
  if (currentMillis < mistingStartMillis || currentMillis < waitingPeriodStartMillis) {
    // if millis() has overflown (happens after about 50 days), need to reset our variables
    reset();
  }
  mistingLoop(currentMillis);
  tempHumidityLoop(currentMillis);
  readButtonState(currentMillis);
}

void reset() {
  mistingStartMillis = 0;
  waitingPeriodStartMillis = 0;
  lastTemperatureMeasurementMillis = 0;
  isWaiting = false;
  isMisting = false;
}

// Temp / Humidity code

void tempHumidityLoop(unsigned long currentMillis) {
  if (currentMillis > lastTemperatureMeasurementMillis + tempMeasurementInterval || lastTemperatureMeasurementMillis == 0) {
    measureTemperatureAndHumidity();
    lastTemperatureMeasurementMillis = currentMillis;
  }
}

void measureTemperatureAndHumidity() {
  // Power-up the sensor
  digitalWrite(DHTPowerPin, HIGH); // turn power to sensor ON
  delay(1000); // sensor needs a second to get ready 
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

   digitalWrite(DHTPowerPin, LOW);  // Turn power to sensor OFF
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

// Operation mode code

void readButtonState(unsigned long currentMillis) {
  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    lastButtonState = buttonState;
    if (buttonState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      operationMode = (operationMode + 1) % 3;
      Serial.print("operationMode: ");
      Serial.println(operationMode);
      updateToNewMode(currentMillis);
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
}

void updateToNewMode(unsigned long currentMillis) {
   resetLEDs();
   if (operationMode == 0) { // normal
      digitalWrite(normalOperationLEDPin, HIGH);
      // Reset so that misting loop sample soil moisture and goes back to normal from there.
      stopMisting();
      isWaiting = false;
   } else if (operationMode == 1) { // force misting
      startMisting(currentMillis);
      digitalWrite(forceMistingLEDPin, HIGH);
   } else if (operationMode == 2) { // stop misting
      stopMisting(); 
      digitalWrite(stopMistingLEDPin, HIGH);
   }
}

void updateErrorLED() {
  if(soilSensingFailed) {
    digitalWrite(soilSensorErrorLEDPin, HIGH);
  } else {
    digitalWrite(soilSensorErrorLEDPin, LOW);
  }
}

void resetLEDs() {
  digitalWrite(normalOperationLEDPin, LOW);
  digitalWrite(forceMistingLEDPin, LOW);
  digitalWrite(stopMistingLEDPin, LOW);
}

// Misting code

void mistingLoop(unsigned long currentMillis) {
   // Only execture misting loop if we are in normal operation mode
   if (operationMode != 0){ return; }
   if (isWaiting == false && isMisting == false) {
     soilMoisture = readSoil();
     if (soilMoisture <= soilMoistureSensorErrorThreshold) {
        soilSensingFailed = true;
        // If we can't get a proper reading from the soil sensor, mist according to the fixed schedule
        if (currentMillis > mistingStartMillis + sensorlessWaitingInterval) {
            startMisting(currentMillis);
        } else {
            startWaiting(currentMillis);
        }
     } else if (soilMoisture < soilMoistureMistingThreshold) {
        soilSensingFailed = false;
        startMisting(currentMillis);
     } else {
        soilSensingFailed = false;
        startWaiting(currentMillis);
     }
     updateErrorLED();
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
