/*
Code is based on the the ABP example of the arduino-lmic library
which is licensed under the MIT license
Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
https://github.com/matthijskooijman/arduino-lmic/blob/master/examples/ttn-abp/ttn-abp.ino
*/

#include "arduino_lmic.h"
#include <hal/hal.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <VL6180X.h>


#include <Credentials.h> // TODO

// #define SINGLE_VALUES
const uint8_t numberOfMeasurements = 50; // max 255!



#ifdef DEBUG
  #define print(x) Serial.print(x);
  #define println(x) Serial.println(x);
#else
  #define print(x)
  #define println(x)
#endif

Adafruit_BME280 bme; // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below
VL6180X sensor;


struct Stats
{
    double mean, standardDeviation;
};

struct Measurement
{
    double meanDistance, standardDeviationDistance, meanAmbientLight, standardDeviationAmbientLight;
    uint8_t successfulMeasurementsDistance, successfulMeasurementsAmbientLight;
};


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

const unsigned TX_INTERVAL = 5*60; // in seconds
const int SLEEP_CYCLES = (int) (TX_INTERVAL / 8);


// Pin mapping
// Adapted for Feather M0 per p.10 of [feather]
// Pin mapping for Adafruit Feather M0 LoRa
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};

void printValues() {
    // print("Temperature = ");
    // print(bme.readTemperature());
    // println(" *C");

    // print("Pressure = ");

    // print(bme.readPressure());
    // println(" hPa");

    // print("Humidity = ");
    // print(bme.readHumidity());
    // println(" %");

    // println();
}

float measureBatteryVoltage() {
    float measuredvbat = analogRead(A0);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    return measuredvbat;
}

double calcMean(uint8_t successfulMeasurements, uint16_t measurementSeries[numberOfMeasurements]) {
    double mean = 0.0;

    if (successfulMeasurements > 0)
    {
        // Mean
        for(uint8_t i = 0; i < successfulMeasurements; i++) {
            mean += measurementSeries[i];
        }
        mean = mean/successfulMeasurements;
    }

    return mean;
}

double calcSD(uint8_t successfulMeasurements, uint16_t measurementSeries[numberOfMeasurements], double mean) {
    double standardDeviation = 0.0;
    if (successfulMeasurements > 0)
    {
        double variance = 0.0;
        // Variance
        for(uint8_t i = 0; i < successfulMeasurements; i++) {
            variance += sq(measurementSeries[i] - mean);
        }
        variance = variance/(successfulMeasurements-1);

        // Standard deviation
        standardDeviation = sqrt(variance);
    }

     return standardDeviation;
}

struct Stats calcStats(uint8_t successfulMeasurements, uint16_t measurementSeries[numberOfMeasurements]) {
    double mean = calcMean(successfulMeasurements, measurementSeries);
    return Stats { mean, calcSD(successfulMeasurements, measurementSeries, mean)};
}


struct Measurement measureDistanceAndAmbientLight() {
    uint16_t measurementSeriesDistance[numberOfMeasurements];
    uint16_t measurementSeriesAmbientLight[numberOfMeasurements];

    print("Start ");
    println(numberOfMeasurements);

    // calibrate sensor in terms of temperature
    // disabled
    //sensor.writeReg(sensor.SYSRANGE__VHV_RECALIBRATE, 0x01);    


    sensor.startInterleavedContinuous();
    uint8_t successfulMeasurementsDistance = 0;
    uint8_t successfulMeasurementsAmbientLight = 0;

    for (uint8_t i = 0; i < numberOfMeasurements; i++) {
        // Ambient Light
        uint16_t currentAmbientLight = sensor.readAmbientContinuous();
        if (!sensor.timeoutOccurred()) {
            measurementSeriesAmbientLight[successfulMeasurementsAmbientLight] = currentAmbientLight;
            successfulMeasurementsAmbientLight += 1;

            #ifdef SINGLE_VALUES
                // Print current value to Serial
                if(i == numberOfMeasurements - 1) {
                    println(currentAmbientLight);
                }
                else {
                    print(currentAmbientLight);
                    print(",");
                }
            #endif
        }

        // Distance
        uint16_t currentDistance = (uint16_t) sensor.readRangeContinuous();
        if (!sensor.timeoutOccurred()) {
            measurementSeriesDistance[successfulMeasurementsDistance] = currentDistance;
            successfulMeasurementsDistance += 1;

            #ifdef SINGLE_VALUES
                // Print current value to Serial
                if(i == numberOfMeasurements - 1) {
                    println(currentDistance);
                }
                else {
                    print(currentDistance);
                    print(",");
                }
            #endif
        }
    }
    sensor.stopContinuous();

    struct Stats statsAmbientLight = calcStats(successfulMeasurementsAmbientLight, measurementSeriesAmbientLight);
    struct Stats statsDistance = calcStats(successfulMeasurementsDistance, measurementSeriesDistance);



    // print("Distsucc: ");
    // print(successfulMeasurementsDistance);
    // print("/");
    // print(numberOfMeasurements);
    // println();
    print("DistM:");
    print(statsDistance.mean);
    println();

    print("DistSD:");
    print(statsDistance.standardDeviation);
    println();

    print("ALSsuc:");
    print(successfulMeasurementsAmbientLight);
    print("/");
    print(numberOfMeasurements);
    println();
    print("ALSM:");
    print(statsAmbientLight.mean);
    println();

    print("ALSSD:");
    print(statsAmbientLight.standardDeviation);
    println();

	struct Measurement measurement = { statsDistance.mean, statsDistance.standardDeviation, statsAmbientLight.mean, statsAmbientLight.standardDeviation, successfulMeasurementsDistance, successfulMeasurementsAmbientLight };

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
        // distanceSD -> 2 byte
        // distanceSucc -> 1 byte
        // ambientLight -> 2 byte
        // ambientLightSD -> 2 byte
        // ambietnLightSucc -> 1 byte
        // batteryVoltage -> 2 byte

        // sum -> 18 byte
        byte payload[18];

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
        Measurement measurement = measureDistanceAndAmbientLight();
        int meanDistance = round(measurement.meanDistance * 100);
        payload[6] = highByte(meanDistance);
        payload[7] = lowByte(meanDistance);

        int standardDeviationDistance = round(measurement.standardDeviationDistance * 100);
        payload[8] = highByte(standardDeviationDistance);
        payload[9] = lowByte(standardDeviationDistance);

        payload[10] = measurement.successfulMeasurementsDistance;

        // ambientLight
        int meanAmbientLight = round(measurement.meanAmbientLight * 100);
        payload[11] = highByte(meanAmbientLight);
        payload[12] = lowByte(meanAmbientLight);

        int standardDeviationAmbientLight = round(measurement.standardDeviationAmbientLight * 100);
        payload[13] = highByte(standardDeviationAmbientLight);
        payload[14] = lowByte(standardDeviationAmbientLight);

        payload[15] = measurement.successfulMeasurementsAmbientLight;

        //int batteryVoltage = round(measureBatteryVoltage() * 100);
        // temporary set battery voltage to zero
        int batteryVoltage = 0;
        payload[16] = highByte(batteryVoltage);
        payload[17] = lowByte(batteryVoltage);


        LMIC_setTxData2(1, (uint8_t*)payload, sizeof(payload), 0);
        println(F("Pckt qd"));
    }
}

void onEvent (ev_t ev) {
    // print(os_getTime());
    // print(": ");
    print(os_getTime());
    println(": ");
    switch(ev) {
#ifndef MINSTER_NODE
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            println("EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            println("EV_REJOIN_FAILED");
            break;
#endif
        case EV_TXCOMPLETE:
            println("EV_TXCOMPLETE");
#ifndef MINSTER_NODE
            if (LMIC.txrxFlags & TXRX_ACK)
              println("Rcvd ack");
            if (LMIC.dataLen) {
              println("Received ");
              println(LMIC.dataLen);
              println(" bytes of payload");
            }
#endif
            // println(F("EV_TXCOMPLETE"));
            // if (LMIC.txrxFlags & TXRX_ACK)
            //     println(F("Received ack"));
            // if (LMIC.dataLen) {
            //     println(F("Received "));
            //     println(LMIC.dataLen);
            //     println(F(" bytes of payload"));
            // }

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
            // disabled due to sensor vulnerability to voltage
            //for(int i = 0; i < SLEEP_CYCLES; i++) {
            //  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
            //}

            // Schedule next transmission to be immediately after this
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

            break;
#ifndef MINSTER_NODE
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            print(F("Unknown event: "));
            println((unsigned) ev);
            break;
#endif
    }
}

void setup() {
    #ifdef DEBUG
      Serial.begin(9600);
    #endif
    println(F("Strtng"));


    // Setup BME280, use address 0x77 (default) or 0x76
    if (!bme.begin(0x76)) {
      println(F("noBME"));
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

    // disable auto calibrate (to do it manually before every series)
    sensor.writeReg(sensor.SYSRANGE__VHV_REPEAT_RATE, 0x00);    
    // calibrate single time (actually the sensor should have done it during start up)
    sensor.writeReg(sensor.SYSRANGE__VHV_RECALIBRATE, 0x01);    


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
