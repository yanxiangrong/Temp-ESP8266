#include <Arduino.h>
#include "DHT.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ikcp.h>
#include <EEPROM.h>


#define DHTPIN 5
#define DHTPWR 4
#define DHTTYPE DHT11

ADC_MODE(ADC_VCC)

const char *ssids[] = {"ASUS", "HNIT_Student", "HNIT_Teacher"};
int ssidIndex = 0;
const char *passwords[] = {"acm2607.", nullptr, nullptr};
const char *host = "192.168.1.137";
//const char *host = "report.esp.yandage.top";

const uint16_t port = 67;
const uint16_t id = 2;

DHT dht(DHTPIN, DHTTYPE);
WiFiUDP Udp;
char incomingPacket[1024];
char sendBuf[1024];
ikcpcb *mKCP;
uint16_t localUdpPort;
uint32_t bootCount;
unsigned long lastSend = 0;

void connectWiFi() {
    Serial.println();
    Serial.print(F("Connecting to WiFi "));
    Serial.println(ssids[ssidIndex]);

    WiFi.begin(ssids[ssidIndex], passwords[ssidIndex]);
    WiFi.setSleepMode(WIFI_LIGHT_SLEEP);

    int i = 20;
    while (WiFi.status() != WL_CONNECTED and i) {
        delay(500);
        Serial.print(".");

        i--;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("");
        Serial.println(F("WiFi connect fail"));
        ssidIndex++;
        if (ssidIndex == (sizeof(ssids) / sizeof(ssids[0]))) {
            ssidIndex = 0;
        }
        return;
    }
    Serial.println("");
    Serial.println(F("WiFi connected"));
    Serial.print(F("IP address: "));
    Serial.println(WiFi.localIP());
}

int udpSendOut(const char *pBuf, int lSize, ikcpcb *pKCP, void *pCTX) {
    digitalWrite(LED_BUILTIN, LOW);

    Udp.beginPacket(host, port);
    Udp.write(pBuf, lSize);
    Udp.endPacket();
    Serial.printf("Send %d bytes UDP packet\n", lSize);
    delay(20);

    digitalWrite(LED_BUILTIN, HIGH);
    return 0;
}

//void setSeed() {
//    uint32_t value = 0;
//    for (int i = 0; i < 4; ++i) {
//        value <<= 8;
//        value += EEPROM.read(i);
//    }
//    if (value == 0xFFFFFFFF) {
//        value = EspClass::getChipId();
//    }
//
//    Serial.print(F("EEPROM seed number: "));
//    Serial.println(value);
//
//    randomSeed(value);
//
//    value++;
//    for (int i = 3; i >= 0; --i) {
//        EEPROM.write(i, (uint8_t) (value & 0xFF));
//        value >>= 8;
//    }
//    if (EEPROM.commit()) {
//        Serial.println("EEPROM successfully committed");
//    } else {
//        Serial.println("ERROR! EEPROM commit failed");
//    }
//}

void deepSleep() {
    Serial.end();
    digitalWrite(DHTPWR, LOW);
    pinMode(DHTPWR, INPUT);
    EspClass::deepSleep(180e6);
}

void sendData() {
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        sprintf(sendBuf, "{\"id\":%d,\"error\":{"
                         "\"code\":1,"
                         "\"string\":\"DHT sensor error\""
                         "}}\n",
                id
        );

        ikcp_send(mKCP, sendBuf, (int) strlen(sendBuf));
        return;
    }

    Serial.print(F("Humidity: "));
    Serial.print(h);
    Serial.print(F("%  Temperature: "));
    Serial.print(t);
    Serial.print(F("C\n"));
    sprintf(sendBuf, "{\"id\":%d,\"data\":{\"humidity\":%.2f,\"temperature\":%.2f}}\n",
            id, h, t);
    ikcp_send(mKCP, sendBuf, (int) strlen(sendBuf));
}

void sendStatus() {
    String resetReason = EspClass::getResetReason();
    uint32_t freeHeap = EspClass::getFreeHeap();
    int heapFragmentation = EspClass::getHeapFragmentation();
    int maxFreeBlockSize = EspClass::getMaxFreeBlockSize();
    uint32_t chipId = EspClass::getChipId();
    String coreVersion = EspClass::getCoreVersion();
    String sdkVersion = EspClass::getSdkVersion();
    int cpuFreq = EspClass::getCpuFreqMHz();
    uint32_t sketchSize = EspClass::getSketchSize();
    uint32_t freeSketchSpace = EspClass::getFreeSketchSpace();
    String sketchMD5 = EspClass::getSketchMD5();
    uint32_t flashChipId = EspClass::getFlashChipId();
    uint32_t flashChipSize = EspClass::getFlashChipSize();
    uint32_t flashChipRealSize = EspClass::getFlashChipRealSize();
    uint32_t flashChipSpeed = EspClass::getFlashChipSpeed();
    bool flashHealthy = EspClass::checkFlashCRC();
    int vcc = EspClass::getVcc();

    sprintf(sendBuf, "{\"id\":%d,\"status\":{"
                     "\"reset_reason\":\"%s\","
                     "\"free_Heap\":%u,"
                     "\"heap_fragmentation\":%d,"
                     "\"max_free_block_size\":%d,"
                     "\"chip_id\":%u,"
                     "\"core_version\":%s,"
                     "\"sdk_version\":%s,"
                     "\"cpu_freq\":%d,"
                     "\"sketch_size\":%u,"
                     "\"free_sketch_space\":%u,"
                     "\"sketch_md5\":\"%s\","
                     "\"flash_chip_id\":%u,"
                     "\"flash_chip_size\":%u,"
                     "\"flash_chip_real_size\":%u,"
                     "\"flash_chip_speed\":%u,"
                     "\"flash_crc_check\":%s,"
                     "\"vcc\":%d"
                     "}}\n",
            id,
            resetReason.c_str(),
            freeHeap,
            heapFragmentation,
            maxFreeBlockSize,
            chipId,
            coreVersion.c_str(),
            sdkVersion.c_str(),
            cpuFreq,
            sketchSize,
            freeSketchSpace,
            sketchMD5.c_str(),
            flashChipId,
            flashChipSize,
            flashChipRealSize,
            flashChipSpeed,
            (flashHealthy ? "true" : "false"),
            vcc
    );

    ikcp_send(mKCP, sendBuf, (int) strlen(sendBuf));
}

void setup() {
    unsigned long dhtBegin;
    uint32_t conv;
    int vcc;

    vcc = EspClass::getVcc();
//    if (vcc < 3200) {
//        EspClass::deepSleep(UINT64_MAX);
//    }
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(DHTPWR, OUTPUT);
    pinMode(16, OUTPUT_OPEN_DRAIN);
    Serial.begin(9600);
    delay(500);
    Serial.println();
    Serial.println();
    Serial.print(F("Client ID: "));
    Serial.println(id);
    Serial.print(F("CPU freq: "));
    Serial.print(EspClass::getCpuFreqMHz());
    Serial.println(F("MHz"));
    Serial.print(F("Vcc: "));
    Serial.print(vcc);
    Serial.println(F("mV"));

    EspClass::rtcUserMemoryRead(0, &bootCount, sizeof(bootCount));
    bootCount++;
    EspClass::rtcUserMemoryWrite(0, &bootCount, sizeof(bootCount));
    Serial.print(F("Boot count: "));
    Serial.println(bootCount);

    Serial.println(F("DHT begin"));
    digitalWrite(DHTPWR, HIGH);
    dht.begin();
    dhtBegin = millis();

//    Serial.println(F("EEPROM begin"));
//    EEPROM.begin(4);
//    setSeed();
//    EEPROM.end();

    connectWiFi();

    localUdpPort = 1024 + EspClass::random() % 3976;
    Serial.print(F("UDP begin, localUdpPort: "));
    Serial.println(localUdpPort);
    Udp.begin(localUdpPort);

//    conv = random();
    conv = EspClass::random();
    Serial.print(F("KCP create, conv: "));
    Serial.println(conv);
    mKCP = ikcp_create(conv, nullptr);
    ikcp_nodelay(mKCP, 0, 40, 0, 0);
    ikcp_wndsize(mKCP, 16, 16);
    mKCP->rx_minrto = 100;
    mKCP->output = udpSendOut;

    if (millis() - dhtBegin < 2000) {
        Serial.println(F("Wait dht"));
        delay(2000 - (millis() - dhtBegin));
    }

    Serial.println(F("Init done"));
    Serial.println();
    Serial.println();
    digitalWrite(LED_BUILTIN, HIGH);

    if (WiFi.isConnected()) {
        ikcp_update(mKCP, millis());
        sendData();
        if (bootCount % 20 == 0) {
            sendStatus();
        }
        lastSend = millis();
    }
}

void loop() {
    if (WiFi.isConnected()) {
        int packetSize = Udp.parsePacket();
        if (packetSize) {
            Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(),
                          Udp.remotePort());
            int len = Udp.read(incomingPacket, 255);
            if (len > 0) {
                incomingPacket[len] = '\0';
            }
            ikcp_input(mKCP, incomingPacket, len);
        }

        ikcp_update(mKCP, millis());
    }

    if (ikcp_waitsnd(mKCP) == 0 or (millis() - lastSend) > 10e3) {
        Serial.println(F("Go deep sleep"));
        delay(500);
        deepSleep();
    }

    delay(10);
}