
#include <Arduino.h>
#include <UrbanGardenBusClient.h>
#include <UrbanGardenBusConfig.h>

UrbanGardenBusClient client;

// lorawan specific stuff
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // this is in little endian format
static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // this is in little endian format
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// pass any LoRaWAN to our UrbanGardenBusClient object  
void onEvent(ev_t ev){
  client.onEvent(ev);
}

// get value function for generictemperaturesensor on slot 0
float getValueForGenerictemperaturesensorOnSlot0(){
  // read your sensor here 
  return 420.0; // replace this value with your sensor reading
}

// get value function for genericsoilhumidity on slot 1
float getValueForGenericsoilhumidityOnSlot1(){
  // read your sensor here 
  return 420.0; // replace this value with your sensor reading
}

float applyCalibrationValueForGenericsoilhumidityOnSlot1(float value, float calibrationValue){
  // apply your calibration value to your sensor reading here
  // value is the raw value that we got from our sensor
  // calibrationValue is the value we calibrated our sensor with
  // as an example you can subtract the calibration value from 
  // the sensor reading and return it like this:
  return value - calibrationValue;
}
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Running GardenBusClient (LoRa32 version)");
  client = UrbanGardenBusClient(
    1, // your individual node id, don't change it!
    30000 // tick rate in milliseconds
  ); //initialize the UrbanGardenBusClient
  
  
  UrbanGardenSensor generictemperaturesensorOnSlot0;
  generictemperaturesensorOnSlot0.slot=0;
  generictemperaturesensorOnSlot0.sensorModelId = 1;
  generictemperaturesensorOnSlot0.getValueFunction = getValueForGenerictemperaturesensorOnSlot0;
  
  Serial.print("Registering generictemperaturesensorOnSlot0: ");
  if (client.registerSensor(generictemperaturesensorOnSlot0,10000)){
    Serial.println("success");
  }else{
    Serial.println("failed");
  }
  
  UrbanGardenSensor genericsoilhumidityOnSlot1;
  genericsoilhumidityOnSlot1.slot=1;
  genericsoilhumidityOnSlot1.sensorModelId = 2;
  genericsoilhumidityOnSlot1.getValueFunction = getValueForGenericsoilhumidityOnSlot1;
  
  // one way to apply a (hardcoded) calibration value for a sensor:
  genericsoilhumidityOnSlot1.calibrationValue = 100.0;
  genericsoilhumidityOnSlot1.needsCalibration=true;
  genericsoilhumidityOnSlot1.applyCalibrationFunction = applyCalibrationValueForGenericsoilhumidityOnSlot1;
  
  
  Serial.print("Registering genericsoilhumidityOnSlot1: ");
  if (client.registerSensor(genericsoilhumidityOnSlot1,10000)){
    Serial.println("success");
  }else{
    Serial.println("failed");
  }
  
  // or for dynamically requesting a calibration value that is saved at the headstation:
  client.requestCalibrationForSensor(genericsoilhumidityOnSlot1, 10000);
}

void loop() {
  client.do_loop();   
}