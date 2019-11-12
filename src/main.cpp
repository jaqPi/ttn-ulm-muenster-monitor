/*
Code is based on the the ABP example of the arduino-lmic library
which is licensed under the MIT license
Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino
*/

#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "LowPower.h"
#include <VL6180X.h>


#include <Credentials.h> // TODO

#define DEBUG // toggle serial output
#define SINGLE_VALUES
#define numberOfMeasurements 50 // max 255!



#ifdef DEBUG
  #define print(x) Serial.print(x);
  #define println(x) Serial.println(x);
#else
  #define print(x)
  #define println(x)
#endif

Adafruit_BME280 bme; // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below
VL6180X sensor;


struct Measurement
{
    double mean, standardDeviation;
    uint8_t successfulMeasurements;
};


void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

const unsigned TX_INTERVAL = 900; // in seconds
const int SLEEP_CYCLES = (int) (TX_INTERVAL / 8);


//float measureDistance();


// Pin mapping
const lmic_pinmap lmic_pins = {
        .nss = 10, // ulm node 10
        .rxtx = LMIC_UNUSED_PIN,
        .rst = LMIC_UNUSED_PIN,
        .dio = {4, 5, 6}, // TTN Ulm Minster node {4, 5 ,6}
};

void printValues() {
    print("Temperature = ");
    print(bme.readTemperature());
    println(" *C");

    print("Pressure = ");

    print(bme.readPressure());
    println(" hPa");

    print("Humidity = ");
    print(bme.readHumidity());
    println(" %");

    println();
}

struct Measurement measureDistance() {
    uint8_t measurementSeries[numberOfMeasurements];

    print("Start series with ");
    print(numberOfMeasurements);
    println(" measurements");
    println();

    sensor.startRangeContinuous();
    double meanDistance = 0.0;
    uint8_t successfulMeasurements = 0;
    double variance = 0.0;
    double standardDeviation = 0.0;

    for (uint8_t i = 0; i < numberOfMeasurements; i++) {
        uint8_t currentDistance = sensor.readRangeContinuous();
        if (!sensor.timeoutOccurred()) {
            measurementSeries[successfulMeasurements] = currentDistance;
            successfulMeasurements += 1;

            #ifdef SINGLE_VALUES
                // Print current value to Seriel 
                if(i == numberOfMeasurements - 1) {
                    println(currentDistance);  
                }
                else {
                    print(currentDistance);
                    print(",");
                }
            #endif
        }
        delay(100);
    }
    sensor.stopContinuous();

    // Calculate stats only if there was a successful measurement
    if (successfulMeasurements > 0)
    {
        // Mean distance
        for(uint8_t i = 0; i < successfulMeasurements; i++) {
            meanDistance += measurementSeries[i];
        }
        meanDistance = meanDistance/successfulMeasurements;

        // Variance
        for(uint8_t i = 0; i < successfulMeasurements; i++) {
            variance += sq(measurementSeries[i] - meanDistance);
        }
        variance = variance/(successfulMeasurements-1);

        // Standard deviation
        standardDeviation = sqrt(variance);
    }

    print("Successful measurements: ");
    print(successfulMeasurements);
    print("/");
    print(numberOfMeasurements);
    println();
    print("Mean Distance: ");
    print(meanDistance);
    println();

    print("StandardDeviation: ");
    print(standardDeviation);
    println();
    
	struct Measurement measurement = { meanDistance, standardDeviation, successfulMeasurements };

    return measurement;
}


void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      println(F("OP_TXRXPEND, not sending"));
    } else {
        // temp -> 2 byte
        // pressure -> 2 byte
        // humidity -> 2 byte
        // distance -> 2 byte
        // sum -> 8 byte
        byte payload[11];

        // Only needed in forced mode. Force update of BME values
        bme.takeForcedMeasurement();

        #ifdef DEBUG
          printValues();
        #endif

        // temp
        int temp = round(bme.readTemperature() * 100);
        payload[0] = highByte(temp);
        payload[1] = lowByte(temp);

        // pressure
        int pressure = round(bme.readPressure()/100);
        payload[2] = highByte(pressure);
        payload[3] = lowByte(pressure);

        // humidity
        int humidity = round(bme.readHumidity() * 100);
        payload[4] = highByte(humidity);
        payload[5] = lowByte(humidity);

        // distance
        Measurement measurement = measureDistance();
        payload[6] = highByte(round(measurement.mean * 100));
        payload[7] = lowByte(round(measurement.mean * 100));

        payload[8] = highByte(round(measurement.standardDeviation * 100));
        payload[9] = lowByte(round(measurement.standardDeviation * 100));

        payload[10] = measurement.successfulMeasurements;



        
        LMIC_setTxData2(1, (uint8_t*)payload, sizeof(payload), 0);

        println(F("Packet queued"));
    }
}

void onEvent (ev_t ev) {
    print(os_getTime());
    print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
                println(F("Received ack"));
            if (LMIC.dataLen) {
                println(F("Received "));
                println(LMIC.dataLen);
                println(F(" bytes of payload"));
            }

            // Now preparing to go into sleep mode. The LMIC library already
            // powers down the RFM95, see
            // https://www.thethingsnetwork.org/forum/t/how-to-put-rfm95-to-sleep/9427

            // Following code is adapted by
            // https://github.com/rocketscream/MiniUltraPro/blob/master/ttn-otaa-sleep.ino

            // Ensure all debugging messages are sent before sleep
            #ifdef DEBUG
              Serial.flush();
            #endif

            // Going into sleep for more than 8 s – any better idea?
            for(int i = 0; i < SLEEP_CYCLES; i++) {
              LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
            }

            // Schedule next transmission to be immediately after this
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);

            break;
        case EV_LOST_TSYNC:
            println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            println(F("EV_LINK_ALIVE"));
            break;
        default:
            println(F("Unknown event"));
            break;
    }
}

void setup() {
    #ifdef DEBUG
      Serial.begin(9600);
    #endif
    println(F("Starting"));


    // Setup BME280, use address 0x77 (default) or 0x76
    if (!bme.begin(0x76)) {
      println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
    }

    // Set BME in force mode to reduce power consumption
    // force mode = measure, store results, and go into sleep mode
    // until next measurement, see
    // - http://tinkerman.cat/low-power-weather-station-bme280-moteino/
    // - https://github.com/adafruit/Adafruit_BME280_Library/blob/master/examples/advancedsettings/advancedsettings.ino
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

    // init tof sensor
    sensor.init();
    sensor.configureDefault();
    sensor.setTimeout(500);
    // stop continuous mode if already active
    sensor.stopContinuous();
    // in case stopContinuous() triggered a single-shot
    // measurement, wait for it to complete
    delay(300);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

#ifdef PROGMEM
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
#elif defined(CFG_us915)
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
}

void loop() {
    os_runloop_once();
}