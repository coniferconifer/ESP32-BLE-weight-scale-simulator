/*
   BLE Weight scale simulator for ESP32 devkitC
   ESP32-BLE-weight-scale-simulator.ino
   Copyright(C) 2021 coniferconifer
   MIT License
   
   Reference "Reading Xiaomi Mi Smart scale using an ESP32"
   https://www.pangodream.es/read-xiaomi-mi-smart-scale-using-an-esp32

  this program behaves as a weight scaler with following charactristics
  
  #define SERVICE_UUID           "0000181d-0000-1000-8000-00805f9b34fb"  // BLE Weight scale service
  #define CHARACTERISTIC_UUID_TX "00002a9d-0000-1000-8000-00805f9b34fb"  // Weight measurement charactristic

  #define SERVICE_UUID_BATT "0000180f-0000-1000-8000-00805f9b34fb"
  #define CHARACTERISTIC_UUID_BATT "00002a19-0000-1000-8000-00805f9b34fb"

  you can change weight value via serial port

  scale -s 88  // change scale to 88kg
  # scale -s 63  (set scale value to 63kg )

  This device sleeps after 120sec by default and awakend by touch to GPIO32

  NimBLE is used for BLE stack. 
  Import NimBLE library from Aruduino IDE library manager   
   Aruduino IDE ->Tool->Library

   

*/
#define VERSION "0.90210722"
#include "NimBLEDevice.h"

#include <Ticker.h>      // timer 
// this software uses SimpleCLI
// Inlcude Library Copyright (c) 2019 Stefan Kremser
//   This software is licensed under the MIT License. See the license file for details.
//   Source: github.com/spacehuhn/SimpleCLI
#include <SimpleCLI.h>
#define SERIAL_TIMEOUT 3000 //msec
// Create CLI Object
SimpleCLI cli;
// Commands
Command cmdScale;
Command cmdHelp;
Command cmdReboot;
#define BACKSPACE 0x08



RTC_DATA_ATTR int bootCount = 0; // Sleep wakeup counter

#define SERVER_NAME "MI_SCALE"
#define SERIAL_SPEED 115200 //USB serial port speed
#define TIME_TO_SLEEPMS 120000 // time to sleep in msec
//#define TIME_TO_SLEEPMS 10000000 // for DEBUG , time to sleep in msec
/* BLE Peripheral definitio */
#define SERVICE_UUID           "0000181d-0000-1000-8000-00805f9b34fb"  // BLE Weight scale service
#define CHARACTERISTIC_UUID_TX "00002a9d-0000-1000-8000-00805f9b34fb"  // Weight measurement charactristic

#define SERVICE_UUID_BATT "0000180f-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_BATT "00002a19-0000-1000-8000-00805f9b34fb"

#define BatteryService NimBLEUUID((uint16_t)0x180F)
#define MAX_DESCRIPTOR_LENGTH 25
NimBLEDescriptor *commandDescriptor;
NimBLEDescriptor *batteryLevelDescriptor;

NimBLECharacteristic *pCharacteristicTX;   // response charactristic
NimBLECharacteristic *pCharacteristicBATT;

bool deviceConnected = false;           // BLE device state

Ticker  ticker;
bool bReadyTicker = false;
const int iIntervalTime = 10;           // Notify interval in sec
// sleep
long lastCommand = 0; // last access time in msec
char weightPacket[] = {0x02, 156, 64 , 227, 7, 12 , 7, 5 , 23, 38 };
// https://www.pangodream.es/read-xiaomi-mi-smart-scale-using-an-esp32

const int readyPin = GPIO_NUM_13; // LED indicator to show the system is up and running
const int touchPin = GPIO_NUM_33; // touch Pin

void printHelp() {
  Serial.println("\r\nScale emulator");
  Serial.println(" you can change weight by \"scale -s 60\" via serial port");

}
void doInitializeCLI() {
  printHelp();
  cmdScale = cli.addCmd("scale");// set sleep time in sec
  cmdScale.addArg("s"); //  scale -s 10

  cmdReboot = cli.addCmd("reboot");//reboot
  cmdHelp = cli.addCommand("?");
  Serial.print("# ");
}
#define KEY_BUFFERSIZE 80
char inputBuffer[KEY_BUFFERSIZE];
int  inputBufferPointer = 0;
void checkCLI() {
  String input;
  if (Serial.available()) {// CR=\r LF=\n  teraterm's default is CR by return key
    char c = Serial.read();
    Serial.print(c);//echo back
    if ( inputBufferPointer < KEY_BUFFERSIZE - 2 ) { //prevent buffer overflow
      inputBuffer[inputBufferPointer++] = c;
    }
    if (c == BACKSPACE) { //Backspace key
      inputBufferPointer = inputBufferPointer - 2;
      if (inputBufferPointer < 0 ) inputBufferPointer = 0;//prevent buffer underflow
    }
    if (c == '\r' || c == '\n') {
      inputBuffer[inputBufferPointer++] = 0x00;
      input = inputBuffer;//char array to String
      inputBufferPointer = 0;
      if (input.length() > 0) {
        Serial.print("# ");
        Serial.println(input);
        cli.parse(input);
      }
    }

    if (cli.available()) {
      Command c = cli.getCmd();
      int argNum = c.countArgs();
      Serial.print("> ");
      Serial.print(c.getName());
      Serial.print(' ');

      for (int i = 0; i < argNum; ++i) {
        Argument arg = c.getArgument(i);
        // if(arg.isSet()) {
        Serial.print(arg.toString());
        Serial.print(' ');
        // }
      }

      Serial.println();

      if (c == cmdScale) {
        int scaleValue = c.getArgument("s").getValue().toInt();
        Serial.println("set scale to  " + String(scaleValue) + "kg" );
        scaleValue = scaleValue * 200;
        weightPacket[1] = scaleValue % 256;
        weightPacket[2] = scaleValue / 256;
        indicateWeight();
        Serial.print("# ");
      } else if (c == cmdHelp) {
        Serial.println("Help:");
        printHelp();
        Serial.print("# ");
      } else if (c == cmdReboot) {
        Serial.println("Rebooting now");
        delay(3000);
        ESP.restart();
      }
    }

    if (cli.errored()) {
      CommandError cmdError = cli.getError();

      Serial.print("ERROR: ");
      Serial.println(cmdError.toString());

      if (cmdError.hasCommand()) {
        Serial.print("Did you mean \"");
        Serial.print(cmdError.getCommand().toString());
        Serial.println("\"?");
      }
      Serial.print("# ");
    }
  }
}

/*********************< Callback classes and functions >**********************/
class funcServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
      Serial.println("connected");
      deviceConnected = true;
    }
    void onDisconnect(NimBLEServer* pServer) {
      Serial.println("disconnected");
      deviceConnected = false;
    }
};


// battery dummy service
uint8_t batLevel[] = {99};

class funcReceiveCallbackBattery: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic *pCharacteristicBATT) {
      pCharacteristicBATT->setValue( (uint8_t*)batLevel, 1);
    }

};

//  timer kick
void kickRoutine() {
  bReadyTicker = true;
}

void setup() {
  doInitialize();
  doInitializeCLI();
  String serverName = SERVER_NAME;

  NimBLEDevice::init(serverName.c_str());

  // Server setup

  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new funcServerCallbacks());
  // Service setup
  NimBLEService *pService = pServer->createService(SERVICE_UUID);
  doPrepare(pService);
  // battery service
  NimBLEService *pService2 = pServer->createService(SERVICE_UUID_BATT);
  doPrepare2(pService2);
  //
  pService->start();
  NimBLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  // pAdvertising->start();
  pService2->start();
  //  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_BATT);
  pAdvertising->start();



  // start timer interrupt
  ticker.attach(iIntervalTime, kickRoutine);

  Serial.println("Waiting to connect ...");
  Serial.print("# ");
}

void loop() {

  if (deviceConnected ) {
    // main task when timer fires
    if (bReadyTicker) {
      doMainProcess(); //
      bReadyTicker = false;
    }
  }

  if ((millis() - lastCommand) > TIME_TO_SLEEPMS ) { //goto sleep after TIMEOUT
    Serial.println("goto deep sleep... touch the pad at GPIO32 to wakeup");
    esp_deep_sleep_start();
  }

  checkCLI();
}

void doInitialize() {
  Serial.begin(SERIAL_SPEED);
  Serial.setTimeout(SERIAL_TIMEOUT);
  //
  if ( bootCount == 0 ) {
    delay(1000);
  }

  Serial.printf("\r\nBLE-Weight scale emulator Version %s\r\n", VERSION);
  setupBoot(); //enable touch input

  pinMode(readyPin, OUTPUT);
  digitalWrite(readyPin, HIGH);
}

void callback() {
  //Serial.println("touch");
}
void setupBoot()
{
  bootCount++;
  Serial.printf("wakeup count: %d ", bootCount);

  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
  static int threshold = 16;
  pinMode(touchPin, INPUT);  //touch pad should be connected to GPIO_NUM_32
  //due to Arduino IDE's bug ?
  touchAttachInterrupt(GPIO_NUM_33, callback, threshold);
  esp_sleep_enable_touchpad_wakeup();

}



class funcReceiveCallback: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic *pCharacteristicTX) {
      pCharacteristicTX->setValue((uint8_t*)weightPacket, sizeof(weightPacket));
      lastCommand = millis();
    }
};
void doPrepare(NimBLEService *pService) {
  pCharacteristicTX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        NIMBLE_PROPERTY::READ
                        |  NIMBLE_PROPERTY::INDICATE
                        |  NIMBLE_PROPERTY::NOTIFY
                      );

  commandDescriptor = pCharacteristicTX->createDescriptor(NimBLEUUID((uint16_t)0x2901), NIMBLE_PROPERTY::READ, MAX_DESCRIPTOR_LENGTH);
  commandDescriptor->setValue("weight scale");
  pCharacteristicTX->setCallbacks(new funcReceiveCallback());

}


//battery service

void doPrepare2(NimBLEService *pService) {
  // Battery service
  pCharacteristicBATT = pService->createCharacteristic(
                          CHARACTERISTIC_UUID_BATT,
                          NIMBLE_PROPERTY::NOTIFY |
                          NIMBLE_PROPERTY::READ
                        );
  batteryLevelDescriptor = pCharacteristicBATT->createDescriptor(NimBLEUUID((uint16_t)0x2901), NIMBLE_PROPERTY::READ, MAX_DESCRIPTOR_LENGTH);
  batteryLevelDescriptor->setValue("battery level");
  pCharacteristicBATT->setCallbacks(new funcReceiveCallbackBattery());
}

// main logic  //
void indicateWeight() {
  pCharacteristicTX->setValue((uint8_t*)&weightPacket, sizeof(weightPacket));
  pCharacteristicTX->indicate();
  pCharacteristicTX->notify();
  Serial.printf("%d:Weight %f kg\r\n", millis(), (float)(weightPacket[2] * 256 + weightPacket[1]) * 0.005);
}
void doMainProcess() {
  indicateWeight();
  
  batLevel[0] = batLevel[0] - random(10);
  batLevel[0]=batLevel[0]%101;//0-100%
  pCharacteristicBATT->setValue((uint8_t*)&batLevel, 1);
  pCharacteristicBATT->notify();
  Serial.printf("%d:batLevel %d\r\n", millis(), batLevel[0]);

}
