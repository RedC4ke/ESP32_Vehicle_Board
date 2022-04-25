#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Arduino.h>
#include <driver/mcpwm.h>

#define SERVICE_UUID                 "00002137-0000-1000-8000-00805F9B34FB"
#define TURN_CHARACTERISTIC_UUID     "00006901-0000-1000-8000-00805F9B34FB"
#define MOVEMENT_CHARACTERISTIC_UUID "00006902-0000-1000-8000-00805F9B34FB"

#define MOTOR_0_FORWARD 15
#define MOTOR_0_BACKWARD 2

BLECharacteristic turnCharacteristic(TURN_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
BLECharacteristic movementCharacteristic(MOVEMENT_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);

class TurnCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* data = pCharacteristic->getData();

      if (data != nullptr) {
        
      }
    }
};

class MovementCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* data = pCharacteristic->getData();

      if (data != nullptr) {
        if (*data > 100) {
          mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, *data - 100.1);
        } else if (*data == 100) {
          mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
          mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
        } else {
          mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, *data);
        }
        Serial.print(*data);
      }
    }
};


void setup() {
  Serial.begin(115200);

  //BLUETOOTH SETUP
  BLEDevice::init("DaddyMobile");
  BLEDevice::setMTU(500);
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pService->addCharacteristic(&turnCharacteristic);
  pService->addCharacteristic(&movementCharacteristic);

  turnCharacteristic.setCallbacks(new TurnCallbacks());
  movementCharacteristic.setCallbacks(new MovementCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  //MOTOR CONTROL SETUP
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_0_FORWARD);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_0_BACKWARD);

  mcpwm_config_t mcpwm_config;
    mcpwm_config.frequency = 2000;
    mcpwm_config.cmpr_a = 0;
    mcpwm_config.cmpr_b = 0;
    mcpwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_config.counter_mode = MCPWM_UP_COUNTER;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &mcpwm_config);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

}

void loop() {

}