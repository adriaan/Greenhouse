int soilMoisture = 0; // Value for storing moisture measurement 

// Variable for the soil moisture sensor signal pin
int soilMeasurePin = A0; 

// Rather than powering the sensor through the 3.3V or 5V pins, 
// we'll use a digital pin to power the sensor. This will 
// prevent corrosion of the sensor as it sits in the soil. 
// Variable for soil moisture power pin
int soilPowerPin = 7; 

int relaySignalPin = 8; // Variable for pump relay signal pin

// Interval between soil mouisture measurements in milliseconds
unsigned long soilMeasurementInterval = 600000 

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
}

void loop() 
{
  unsigned long currentMillis = millis();
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
        startWaiting(); // always wait one cycle directly after misting.
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
