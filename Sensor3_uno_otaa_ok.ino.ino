
#include <LowPower.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "DHT.h"

#define DHTPIN 7 
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

bool sleeping = false;

float voltage;


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
//static const PROGMEM u1_t NWKSKEY[16] ={ 0xD8, 0xB6, 0xDB, 0xC0, 0xCF, 0xB8, 0x3B, 0xC0, 0x9A, 0x6D, 0xE1, 0x54, 0xAF, 0x8E, 0x2A, 0xE7 };
//
//// LoRaWAN AppSKey, application session key
//// This is the default Semtech key, which is used by the prototype TTN
//// network initially.
//static const u1_t PROGMEM APPSKEY[16] = { 0x16, 0xBC, 0xDE, 0xD6, 0x97, 0x09, 0xF0, 0xDC, 0x50, 0xA8, 0xB1, 0xD6, 0x0D, 0x06, 0x65, 0x45 };
//
//// LoRaWAN end-device address (DevAddr)
//// See http://thethingsnetwork.org/wiki/AddressSpace
//static const u4_t DEVADDR = 0x26041A90; // <-- Change this address for every node!
static const u1_t PROGMEM APPEUI[8]={ 0xE1, 0x7B, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x22, 0x11, 0x33, 0x22, 0x11, 0x33, 0x22, 0x11 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x07, 0x61, 0x93, 0x91, 0xAE, 0x21, 0x71, 0xBF, 0xB8, 0xF6, 0xF1, 0x66, 0x95, 0x28, 0xEC, 0xD5 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
//void os_getArtEui (u1_t* buf) { }
//void os_getDevEui (u1_t* buf) { }
//void os_getDevKey (u1_t* buf) { }

// static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,                                                                                                                                                                                                                                                                                                                                                                                                                              
    .rst = 5,
    .dio = {2, 3, LMIC_UNUSED_PIN},
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
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
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
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j) {
  byte buffer[32];  
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  //float b = 4.8;
//  while (isnan(t) || isnan(h)){   //check for valid data, else read again
//    delay(5000);
//    t = dht.readTemperature();  
//    h = dht.readHumidity();
//    Serial.println("DHT error");
//    String message = "{Error DHT}";
//    message.getBytes(buffer, message.length()+1);
//  // Check if there is not a current TX/RX job running
//  if (LMIC.opmode & OP_TXRXPEND) {
//    Serial.println(F("OP_TXRXPEND, not sending"));
//  } else {
//    // Prepare upstream data transmission at the next possible time.
//    LMIC_setTxData2(1, (uint8_t*) buffer, message.length() , 0);
//    Serial.println("Sending: "+message);
//  }
//  }

//read battery
  int sensorValue = analogRead(A0);
  float voltage= sensorValue * (5 / 1023.0); //2 from voltage divider
  
  String message = String(t)+ ":" + String(h) + ":" + String(voltage);
//String message = "Sensor1";
  message.getBytes(buffer, message.length()+1);
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (uint8_t*) buffer, message.length() , 0);
    Serial.println("Sending: "+message);
  }
}

// initial job
static void initfunc (osjob_t* j) {
    // reset MAC state
    LMIC_reset();
    // start joining
    LMIC_startJoining();
    // init done - onEvent() callback will be invoked...
}

void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));
    dht.begin();
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
}


void loop() {
    os_runloop_once();
}

//void loop() {
//   do_send(&sendjob);    // Sent sensor values
//      while(sleeping == false)
//      {
//        os_runloop_once();
//      }
//      sleeping = false;
//      sleep();
//}


////Sleep Ten Minutes
//void sleep()
//{
//  for (int i = 0; i < 75; i++) { 
//     LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
//  }
//}
