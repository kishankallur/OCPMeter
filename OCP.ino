#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31865.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <stdlib.h>

Adafruit_ADS1115 ads;

#define MAX31865_CS_PIN 5
#define MAX31865_RDY_PIN 4
Adafruit_MAX31865 rtd(MAX31865_CS_PIN);

const double RTD_NOMINAL = 100.0;
const double RTD_REFERENCE = 430.0;

const double VOLTS_PER_BIT = 0.000125;
const double VMID_CALIBRATED = 1.65017;
const int OVERSAMPLE_COUNT = 512;

const int TRIM_COUNT = 64;
int16_t adcSamples[OVERSAMPLE_COUNT];

int compareInt16(const void *a, const void *b) {
  return (int)*(const int16_t *)a - (int)*(const int16_t *)b;
}

// Nordic UART Service (NUS) - the de facto default ESP32 BLE UUID set,
// recognized out of the box by nRF Connect / Serial Bluetooth Terminal etc.
#define SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *pTxCharacteristic;
volatile bool deviceConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    pServer->startAdvertising();
  }
};

void setup(void) {
  Serial.begin(115200);
  Serial.println("Initializing...");

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!ads.begin(0x48)) {
    Serial.println("ERROR: ADS1115 not detected, check wiring.");
    while (1);
  }

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  pinMode(MAX31865_RDY_PIN, INPUT);
  rtd.begin(MAX31865_2WIRE);

  BLEDevice::init("ADS1115-OCP");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX included for a complete/standard NUS service; unused for now (no
  // inbound commands yet).
  pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("Ready. Advertising as ADS1115-OCP.");
}

void loop(void) {
  for (int i = 0; i < OVERSAMPLE_COUNT; ++i)
    adcSamples[i] = ads.readADC_SingleEnded(0);

  qsort(adcSamples, OVERSAMPLE_COUNT, sizeof(adcSamples[0]), compareInt16);

  double raw = 0.0;
  for (int i = TRIM_COUNT; i < OVERSAMPLE_COUNT - TRIM_COUNT; ++i)
    raw += (double)adcSamples[i];
  int keptCount = OVERSAMPLE_COUNT - 2 * TRIM_COUNT;

  double adc_measured_volts = raw / (double)keptCount * VOLTS_PER_BIT;
  double ocp_volts = (adc_measured_volts - VMID_CALIBRATED) * 2.0;

  Serial.print("OCP: ");
  Serial.print(ocp_volts, 5);
  Serial.println(" V");

  double temp_c = NAN;
  uint8_t fault = rtd.readFault();
  if (fault) {
    Serial.print("MAX31865 fault 0x");
    Serial.println(fault, HEX);
    rtd.clearFault();
  } else {
    temp_c = rtd.temperature(RTD_NOMINAL, RTD_REFERENCE);
    Serial.print("Temp: ");
    Serial.print(temp_c, 2);
    Serial.println(" C");
  }

  if (deviceConnected) {
    char json[96];
    if (fault) {
      snprintf(json, sizeof(json),
               "{\"ocp\":%.5f,\"temp_c\":null,\"fault\":%u}", ocp_volts,
               fault);
    } else {
      snprintf(json, sizeof(json),
               "{\"ocp\":%.5f,\"temp_c\":%.2f,\"fault\":0}", ocp_volts,
               temp_c);
    }
    pTxCharacteristic->setValue(json);
    pTxCharacteristic->notify();
  }

  delay(100);
}
