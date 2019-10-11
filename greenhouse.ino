#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "DHT.h"

// --------- //
// LoRa Code //
// --------- //

// LoRa payload

static uint8_t payload[6];

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x24, 0x35, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xD5, 0xB7, 0xAD, 0xFB, 0xE9, 0x50, 0x9C, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x71, 0x23, 0x2F, 0x1A, 0x53, 0x71, 0x13, 0x06, 0x5E, 0xE3, 0x38, 0xE6, 0x4D, 0x14, 0x9E, 0x33 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        Serial.println(F("Packet queued"));
        Serial.print(F("Sending packet on frequency: "));
        Serial.println(LMIC.freq);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// --------- ----//
// End LoRa Code //
// ------------- //

// PIN definitions
// Dragino LoRa shield uses all pins except 3/4/5 and A0/A1/A2/A3/A4/A5
// https://wiki.dragino.com/index.php?title=Lora_Shield

// LED configuration
#define normalOperationLEDPin A1
#define forceMistingLEDPin A2
#define stopMistingLEDPin A3
#define soilSensorErrorLEDPin A4

// Push button configuration
#define buttonPin A5

#define soilMeasurePin A0 // Analog pin connected to the sensor's signal outptu
// Rather than powering the sensors through the 3.3V or 5V pins, 
// we'll use a digital pin to power the sensors. 
// This is more energy efficient and will prevent corrosion of the sensor as it sits in the soil. 
#define sensorPowerPin 3

// Relay configuration
#define relaySignalPin 4 // Variable for pump relay signal pin

// Temperature / Humidity sensor configuration
#define DHTPIN 5     // Digital pin connected to the DHT sensor
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

// Variable definitions misting
int soilMoisture = 0; // Value for storing moisture measurement 

// Interval between sensor measurements in milliseconds
unsigned long measurementInterval = 60000;

// Interval between mistings when sensor is not working in milliseconds
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
  // ---------- //
  // LoRa setup //
  // ---------- //
    Serial.begin(115200);
    delay(100);     // per sample code on RF_95 test
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

    // ---------------- //
    // Greenhouse setup //
    // ---------------- //
 

  pinMode(sensorPowerPin, OUTPUT); 
  digitalWrite(sensorPowerPin, LOW); // Set to LOW so no power is flowing through the sensor

  pinMode(relaySignalPin, OUTPUT); 
  digitalWrite(relaySignalPin, LOW); // Set to LOW so relay is switched OFF

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
  unsigned long now;
    now = millis();

//    // LoRa
//    if ((now & 512) != 0) {
//      digitalWrite(13, HIGH);
//    }
//    else {
//      digitalWrite(13, LOW);
//    }
      
    os_runloop_once();

  // greenhouse
  if (now < mistingStartMillis || now < waitingPeriodStartMillis) {
    // if millis() has overflown (happens after about 50 days), need to reset our variables
    reset();
  }
  mistingLoop(now);
  readButtonState(now);
}

void reset() {
  mistingStartMillis = 0;
  waitingPeriodStartMillis = 0;
  isWaiting = false;
  isMisting = false;
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
     measureTemperatureAndHumidity();
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
    if (currentMillis > waitingPeriodStartMillis + measurementInterval) {
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

// Sensor readings code

// Function for measuring temperature and humidity
void measureTemperatureAndHumidity() {
  // Power-up the sensor
  digitalWrite(sensorPowerPin, HIGH); // turn power to sensor ON
  delay(1000); // sensor needs a second to get ready 
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float rHumidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float temperature = dht.readTemperature();

   digitalWrite(sensorPowerPin, LOW);  // Turn power to sensor OFF
  // Check if any reads failed and exit early (to try again).
  if (isnan(rHumidity) || isnan(temperature)){
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.print(F("Humidity: "));
  Serial.print(rHumidity);
  Serial.print(F("%  Temperature: "));
  Serial.print(temperature);
  Serial.print(F("°C "));

  // Put temp and humidity in payload
  // adjust for the f2sflt16 range (-1 to 1)
  temperature = temperature / 100;
  rHumidity = rHumidity / 100;
        
  // float -> int
  // note: this uses the sflt16 datum (https://github.com/mcci-catena/arduino-lmic#sflt16)
  uint16_t payloadTemp = LMIC_f2sflt16(temperature);
  // int -> bytes
  byte tempLow = lowByte(payloadTemp);
  byte tempHigh = highByte(payloadTemp);
  // place the bytes into the payload
  payload[0] = tempLow;
  payload[1] = tempHigh;

  // float -> int
  uint16_t payloadHumid = LMIC_f2sflt16(rHumidity);
  // int -> bytes
  byte humidLow = lowByte(payloadHumid);
  byte humidHigh = highByte(payloadHumid);
  payload[2] = humidLow;
  payload[3] = humidHigh;
}


// Function used to get the soil moisture content
int readSoil()
{
  digitalWrite(sensorPowerPin, HIGH); // turn power to sensor ON
  delay(10); // wait 10 milliseconds 
  int val = analogRead(soilMeasurePin); // Read the SIG value form sensor 
  digitalWrite(sensorPowerPin, LOW);  // Turn power to sensor OFF
  Serial.print("Soil Moisture = ");
  Serial.println(val);

  //put moisture reading in payload
  payload[4] = highByte(val);
  payload[5] = lowByte(val);
  
  return val; // Send current moisture value
}
