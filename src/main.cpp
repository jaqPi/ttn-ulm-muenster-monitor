/*
Code is based on the the OTAA example of the arduino-lmic library

 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI

which is licensed under the MIT license
https://github.com/mcci-catena/arduino-lmic/blob/master/examples/ttn-otaa/ttn-otaa.ino
*/

#include "arduino_lmic.h"
#include <hal/hal.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <VL6180X.h>
#include "measurements.h"
#include "sensors.h"
#include <Statistics.h>


#include <Credentials.h> // TODO

// #define SINGLE_VALUES
const uint8_t numberOfMeasurements = 50; // max 255!

VL6180X sensorVL6180X;
const SensorConfig configInterleaved = { INTERLEAVED, MANUAL, 30, 50, 500};
const TofSensor tofSensor = TofSensor { sensorVL6180X, configInterleaved, 0, 0x29 };

#ifdef DEBUG
  #define print(x) Serial.print(x);
  #define println(x) Serial.println(x);
#else
  #define print(x)
  #define println(x)
#endif

Adafruit_BME280 bme; // I2C, depending on your BME, you have to use address 0x77 (default) or 0x76, see below


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

const unsigned TX_INTERVAL = 5*60; // in seconds
const int SLEEP_CYCLES = (int) (TX_INTERVAL / 8);


// Pin mapping
#ifdef MINSTER_NODE
// Pin mapping for Minster Node
const lmic_pinmap lmic_pins = {
        .nss = 10, // ulm node 10
        .rxtx = LMIC_UNUSED_PIN,
        .rst = LMIC_UNUSED_PIN,
        .dio = {4, 5, 6}, // TTN Ulm Minster node {4, 5 ,6}
};
#elif FEATHER_M0
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
#endif

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

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      println(F("OP_TXRXPEND, not sending"));
    } else {
        // temp -> 2 byte
        // pressure -> 2 byte
        // humidity -> 2 byte
        // distance -> 2 byte
        // distanceMedian -> 2 byte
        // distanceSD -> 2 byte
        // distanceSucc -> 1 byte
        // ambientLight -> 2 byte
        // ambientLightMedian -> 2 byte
        // ambientLightSD -> 2 byte
        // ambietnLightSucc -> 1 byte
        // batteryVoltage -> 2 byte

        // sum -> 22 byte
        byte payload[22];

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

        measurement_t currentMeasurement = measureDistanceAndAmbientLight(&tofSensor, numberOfMeasurements);

        // distance
        int meanDistance = round(currentMeasurement.distance.mean * 100);
        payload[6] = highByte(meanDistance);
        payload[7] = lowByte(meanDistance);

        int standardDeviationDistance = round(currentMeasurement.distance.standardDeviation * 100);
        payload[8] = highByte(standardDeviationDistance);
        payload[9] = lowByte(standardDeviationDistance);

        int medianDistance = round(currentMeasurement.distance.median * 100);
        payload[10] = highByte(medianDistance);
        payload[11] = lowByte(medianDistance);

        payload[12] = currentMeasurement.successfulMeasurementsDistance;

        // ambientLight
        int meanAmbientLight = round(currentMeasurement.light.mean * 100);
        payload[13] = highByte(meanAmbientLight);
        payload[14] = lowByte(meanAmbientLight);

        int standardDeviationAmbientLight = round(currentMeasurement.light.standardDeviation * 100);
        payload[15] = highByte(standardDeviationAmbientLight);
        payload[16] = lowByte(standardDeviationAmbientLight);

        int medianLight = round(currentMeasurement.light.median * 100);
        payload[17] = highByte(medianLight);
        payload[18] = lowByte(medianLight);

        payload[19] = currentMeasurement.successfulMeasurementsAmbientLight;

        //int batteryVoltage = round(measureBatteryVoltage() * 100);
        // temporary set battery voltage to zero
        int batteryVoltage = 0;
        payload[20] = highByte(batteryVoltage);
        payload[21] = lowByte(batteryVoltage);


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

void scanI2C() {
  println("I2C scanner. Scanning ...");
  byte count = 0;


  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      print("Found address: ");
      print(i);
      print(" (0x");
      print(i);
      println(")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  println("Done.");
  print("Found ");
  print(count);
  println(" device(s).");
}

void setup() {
    #ifdef DEBUG
      Serial.begin(9600);
      while(!Serial);
    #endif
    println(F("Strtng"));
    Wire.begin();
    delay(5000);

    #ifdef DEBUG
      scanI2C();
    #endif
    

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


      tofSensor.sensor.init();
      tofSensor.sensor.configureDefault();

      // Reduce range max convergence time and ALS integration
      // time to 30 ms and 50 ms, respectively, to allow 10 Hz
      // operation (as suggested by table "Interleaved mode
      // limits (10 Hz operation)" in the datasheet).
      tofSensor.sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, tofSensor.config.rangeMaxConvergenceTime);
      tofSensor.sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, tofSensor.config.alsMaxIntegrationPeriod);
      tofSensor.sensor.setTimeout(tofSensor.config.timeout);

      // stop continuous mode if already active
      tofSensor.sensor.stopContinuous();
      // in case stopContinuous() triggered a single-shot
      // measurement, wait for it to complete
      delay(300);

      if(tofSensor.config.calibrationMode == MANUAL) {
        // disable auto calibrate (to do it manually before every series)
        tofSensor.sensor.writeReg(VL6180X::SYSRANGE__VHV_REPEAT_RATE, 0x00);    
        // calibrate single time (actually the sensor should have done it during start up)
        tofSensor.sensor.writeReg(VL6180X::SYSRANGE__VHV_RECALIBRATE, 0x01);   
      }

      println(", configuration completed");

     // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();


    LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
