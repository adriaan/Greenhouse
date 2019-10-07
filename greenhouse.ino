// TODO
// - add LoRa code
// - not sure exactly how all LoRa code works, start by just merging the sample code with this sketch and see if works...
// - design and implement LoRa payload
// - send payload

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "DHT.h"

// --------- //
// LoRa Code //
// --------- //


//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0x43, 0xFB, 0xCC, 0x13, 0x7D, 0x23, 0x16, 0xF1, 0xB2, 0x24, 0x6A, 0x28, 0x93, 0x90, 0x2F, 0xE5 };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0x48, 0x87, 0xAC, 0x36, 0x94, 0xB8, 0xD1, 0xF6, 0x45, 0xDF, 0x84, 0x1D, 0x3D, 0xA9, 0x53, 0x56 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed.
static const u4_t DEVADDR = 0x260617F3; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 5;

// Pin mapping
// TL Modifications:
// Specifically for Arduino Uno + Dragino LoRa Shield US900
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
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
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
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
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
      while (!Serial); // wait for Serial to be initialized
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

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    // Specify to operate on AU915 sub-band 2
    #elif defined(CFG_au921)
    Serial.println(F("Loading AU915/AU921 Configuration..."));
    // Set to AU915 sub-band 2
    LMIC_selectSubBand(1); 
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
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

    // LoRa
    if ((now & 512) != 0) {
      digitalWrite(13, HIGH);
    }
    else {
      digitalWrite(13, LOW);
    }
      
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
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

   digitalWrite(sensorPowerPin, LOW);  // Turn power to sensor OFF
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


// Function used to get the soil moisture content
int readSoil()
{
  digitalWrite(sensorPowerPin, HIGH); // turn power to sensor ON
  delay(10); // wait 10 milliseconds 
  int val = analogRead(soilMeasurePin); // Read the SIG value form sensor 
  digitalWrite(sensorPowerPin, LOW);  // Turn power to sensor OFF
  Serial.print("Soil Moisture = ");
  Serial.println(val);
  return val; // Send current moisture value
}
