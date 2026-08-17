#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <stdlib.h>

Adafruit_ADS1115 ads;

// GAIN_ONE multiplier is exactly 0.125 mV per bit
const double VOLTS_PER_BIT = 0.000125;

// warm up for a few mins and measure voltage between A1 and gnd
const double V_REF_CALIBRATED = 3.29832;
const int OVERSAMPLE_COUNT = 512;

// samples sorted and the extreme TRIM_COUNT at each tail dropped before
// averaging, so supply-noise glitch spikes don't drag the mean around
const int TRIM_COUNT = 64;
int16_t adcSamples[OVERSAMPLE_COUNT];

int compareInt16(const void *a, const void *b) {
  return (int)*(const int16_t *)a - (int)*(const int16_t *)b;
}

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
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
    while (1)
      ;
  }

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  BLEDevice::init("ADS1115-OCP");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("Ready. Advertising as ADS1115-OCP.");
}

void loop(void) {
  for (int i = 0; i < OVERSAMPLE_COUNT; ++i)
    adcSamples[i] = ads.readADC_Differential_0_1();

  qsort(adcSamples, OVERSAMPLE_COUNT, sizeof(adcSamples[0]), compareInt16);

  double raw = 0.0;
  for (int i = TRIM_COUNT; i < OVERSAMPLE_COUNT - TRIM_COUNT; ++i)
    raw += (double)adcSamples[i];
  int keptCount = OVERSAMPLE_COUNT - 2 * TRIM_COUNT;

  double differential_measured_volts = raw / (double)keptCount * VOLTS_PER_BIT;
  double ocp_volts = differential_measured_volts + V_REF_CALIBRATED;

  Serial.print("OCP: ");
  Serial.print(ocp_volts, 5);
  Serial.println(" V");

  if (deviceConnected) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%.5f", ocp_volts);
    pCharacteristic->setValue(buf);
    pCharacteristic->notify();
  }

  delay(100);
}
