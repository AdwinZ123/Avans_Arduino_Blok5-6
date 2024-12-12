/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/
#include <Arduino.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "CQRobotTDS.h"

#include "types.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "EEPROM.h"

#include <Wire.h>

#include "Elektrischegeleidingssensor.h"
#include "Troebelheidsensor.h"
#include "Phsensor.h"
#include "Zuurstofsensor.h"
#include "Temperatuursensor.h"

#define TEMPERATUURSENSORPIN 13 // Nummer: 17 van rechts
#define ZUURSTOFSENSORPIN 12 // Nummer: 16 van rechts
#define PHSENSORPIN 34 // Nummer: 8 van rechts
#define TROEBELHEIDSENSORPIN 35 // Nummer 9 van rechts
#define ELEKTRISCHEGELEIDINGSSENSORPIN 36 // Nummer 4 van rechts

Elektrischegeleidingssensor elektrischegeleidingssensor(ELEKTRISCHEGELEIDINGSSENSORPIN);
Troebelheidsensor troebelheidsensor(TROEBELHEIDSENSORPIN);
Phsensor phsensor(PHSENSORPIN);
Zuurstofsensor zuurstofsensor(ZUURSTOFSENSORPIN);
Temperatuursensor temperatuursensor(TEMPERATUURSENSORPIN);

#define EEPROM_SIZE 128

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

GpsDataState_t gpsState = {};

#define TASK_SERIAL_RATE 100

uint32_t nextSerialTaskTs = 0;

#define BUILDINLED 25

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0xF0, 0xAF, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0xFB, 0xDA, 0x08, 0x0F, 0x51, 0xDE, 0xC4, 0xEB, 0x2B, 0x7B, 0x71, 0xB1, 0x54, 0x64, 0xE7, 0x58};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

void do_send(osjob_t *);

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping TTGO LoRa32 V1.0:
const lmic_pinmap lmic_pins = {
    .nss = LORA_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST,
    .dio = {LORA_IRQ, A5, A4},
};

void printHex2(unsigned v)
{
    Serial.print("0123456789ABCDEF"[v >> 4]);
    Serial.print("0123456789ABCDEF"[v & 0xF]);
}

void onEvent(ev_t ev)
{
    Serial.println("onEvent");
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
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
            Serial.print("AppSKey: ");
            for (size_t i = 0; i < sizeof(artKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(artKey[i]);
            }
            Serial.println("");
            Serial.print("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(nwkKey[i]);
            }
            Serial.println();
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
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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

        // Zet de ingebouwde LED aan wanneer een bericht ontvangen is via de down-link
        digitalWrite(BUILDINLED, HIGH);

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
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;

    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

template <class T>
int EEPROM_writeAnything(int ee, const T &value)
{
    const byte *p = (const byte *)(const void *)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.write(ee++, *p++);
    return i;
}

template <class T>
int EEPROM_readAnything(int ee, T &value)
{
    byte *p = (byte *)(void *)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return i;
}

float *readGps()
{
    static int p0 = 0;

    gpsState.originLat = gps.location.lat();
    gpsState.originLon = gps.location.lng();
    gpsState.originAlt = gps.altitude.meters();

    long writeValue;
    writeValue = gpsState.originLat * 1000000;
    EEPROM_writeAnything(0, writeValue);
    writeValue = gpsState.originLon * 1000000;
    EEPROM_writeAnything(4, writeValue);
    writeValue = gpsState.originAlt * 1000000;
    EEPROM_writeAnything(8, writeValue);
    EEPROM.commit();

    gpsState.distMax = 0;
    gpsState.altMax = -999999;
    gpsState.spdMax = 0;
    gpsState.altMin = 999999;

    while (SerialGPS.available() > 0)
    {
        gps.encode(SerialGPS.read());
    }

    if (gps.satellites.value() > 4)
    {
        gpsState.dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), gpsState.originLat, gpsState.originLon);

        if (gpsState.dist > gpsState.distMax && abs(gpsState.prevDist - gpsState.dist) < 50)
        {
            gpsState.distMax = gpsState.dist;
        }

        gpsState.prevDist = gpsState.dist;

        if (gps.altitude.meters() > gpsState.altMax)
        {
            gpsState.altMax = gps.altitude.meters();
        }

        if (gps.speed.mps() > gpsState.spdMax)
        {
            gpsState.spdMax = gps.speed.mps();
        }

        if (gps.altitude.meters() < gpsState.altMin)
        {
            gpsState.altMin = gps.altitude.meters();
        }
    }

    if (nextSerialTaskTs < millis())
    {

        Serial.print("LAT=");
        Serial.println(gps.location.lat(), 6);
        Serial.print("LONG=");
        Serial.println(gps.location.lng(), 6);
        Serial.print("ALT=");
        Serial.println(gps.altitude.meters());
        Serial.print("Sats=");
        Serial.println(gps.satellites.value());
        Serial.print("DST: ");
        Serial.println(gpsState.dist, 1);

        nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    }

    static float arr[2];
    arr[0] = gps.location.lat();
    arr[1] = gps.location.lng();

    return arr;
}

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        // PUT HERE YOUR CODE TO READ THE SENSORS AND CONSTRUCT THE TTS PAYLOAD
        float temperatuurWaarde = temperatuursensor.Meet();
        float elektrischegeleidingsWaarde = elektrischegeleidingssensor.Meet(temperatuurWaarde);
        float troebelheidWaarde = troebelheidsensor.Meet();
        float phWaarde = phsensor.Meet(temperatuurWaarde);
        int zuurstofWaarde = zuurstofsensor.Meet(temperatuurWaarde);
        float *gpsLocaties = readGps();
        float gpsLocatieLat = gpsLocaties[0];
        float gpsLocatieLng = gpsLocaties[1];

        // hier moet de payload worden opgebouwd
        Serial.println();
        Serial.println();
        Serial.println("Nieuwe test");

        String phPayload = String(phWaarde, HEX);
        String troebelheidPayload = String(troebelheidWaarde, HEX);
        String elektrischegeleidingsPayload = String(elektrischegeleidingsWaarde, HEX);
        String zuurstofPayload = String(zuurstofWaarde, HEX);
        String temperatuurPayload = String(temperatuurWaarde, HEX);

        unsigned char byteArray[8] = {0x8C};

        int pHPayloadRangeValue = phWaarde * 10;
        int temperaturePayloadRangeValue = temperatuurWaarde * 20;
        int oxygenPayloadRangeValue = zuurstofWaarde * 10;
        // int temperaturePayloadRangeValue = temperatuurWaarde * 20;

        String temperatureResult = String(temperaturePayloadRangeValue);
        String egvResult = String(elektrischegeleidingsWaarde);
        String turbidityResult = String(troebelheidWaarde);

        while (temperatureResult.length() < 4)
        {
            temperatureResult = "0" + temperatureResult;
        }

        while (egvResult.length() < 4)
        {
            egvResult = "0" + egvResult;
        }

        while (turbidityResult.length() < 4)
        {
            turbidityResult = "0" + turbidityResult;
        }

        Serial.println("_______________________" + temperatureResult);

        String payloadByte2 = temperatureResult.substring(0, 2);
        String payloadByte3 = temperatureResult.substring(2, 4);

        String payloadByte5 = egvResult.substring(0, 2);
        String payloadByte6 = egvResult.substring(2, 4);

        String payloadByte7 = turbidityResult.substring(0, 2);
        String payloadByte8 = turbidityResult.substring(2, 4);

        Serial.println("_______________________");

        Serial.println(payloadByte2 + " | " + payloadByte3);

        Serial.println("_______________________");

        String hexString = ""; // De resulterende string met hex-waarden
        String payloadByte1String =  String("0x") + String(pHPayloadRangeValue, HEX);
        String payloadByte2String = String("0x") + payloadByte2;
        String payloadByte3String = String("0x") + payloadByte3;
        String payloadByte4String = String("0x") + String("0x") + String(oxygenPayloadRangeValue, HEX);
        String payloadByte5String = String("0x") + payloadByte5;
        String payloadByte6String = String("0x") + payloadByte6;
        String payloadByte7String = String("0x") + payloadByte7;
        String payloadByte8String = String("0x") + payloadByte8;

        String hexStrings[] =   {payloadByte1String,
                                payloadByte2String,
                                payloadByte3String,
                                payloadByte4String,
                                payloadByte5String,
                                payloadByte6String,
                                payloadByte7String,
                                payloadByte8String};

         for (int i = 0; i < sizeof(hexStrings); i++) {

        mydata[i] = (uint8_t) strtol(hexStrings[i].c_str(), NULL, 16);

    }   


        for (int i = 2; i < 4; i++)
        {
            // Haal elk karakter op en zet het om naar een getal
            int num = payloadByte2String.charAt(i) - '0';
            // Zet het getal om naar een hexadecimale string en voeg toe aan hexString
            hexString += String(num, HEX);
        }

        // TODO: Add the other values to the payload

        // Print the payload to the console
        Serial.print("Payload: ");
        for (size_t i = 0; i < sizeof(mydata) - 1; ++i)
        {
            if (i != 0)
                Serial.print("-");
            printHex2(mydata[i]);
        }
        Serial.println();

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup()
{
    Serial.begin(115200);
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

    pinMode(BUILDINLED, OUTPUT);


    while (!EEPROM.begin(EEPROM_SIZE))
    {
        true;
    }

    long readValue;
    EEPROM_readAnything(0, readValue);
    gpsState.originLat = (double)readValue / 1000000;

    EEPROM_readAnything(4, readValue);
    gpsState.originLon = (double)readValue / 1000000;

    EEPROM_readAnything(8, readValue);
    gpsState.originAlt = (double)readValue / 1000000;

    // Serial.begin(9600);
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
}

void loop()
{
    os_runloop_once();
    // float temperatuurWaarde = temperatuursensor.Meet();
    // delay(1000);
}
