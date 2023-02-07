#include "UrbanGardenBusClient.h"
#include "UrbanGardenBusConfig.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LoraMessage.h>
#include <LoraEncoder.h>


const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}
LoraMessage message;

void UrbanGardenBusClient::onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
                for (size_t i=0; i<sizeof(artKey); ++i) {
                    if (i != 0)
                        Serial.print("-");
                    printHex2(artKey[i]);
                }
                Serial.println("");
                Serial.print("NwkSKey: ");
                for (size_t i=0; i<sizeof(nwkKey); ++i) {
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
            if (LMIC.dataLen) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            if(this->lorawanEnabled){
              //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            
                os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), &do_send);

            }
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
            Serial.println((unsigned) ev);
            break;
    }
}

UrbanGardenBusClient::UrbanGardenBusClient(){}

UrbanGardenBusClient::UrbanGardenBusClient(int16_t nodeId, int tickRate){
    this->nodeId = nodeId;
    this->tickRate = tickRate;
    os_init();
    LMIC_reset();
    CAN.setPins(GPIO_NUM_4,GPIO_NUM_12);
    if (!CAN.begin(CAN_SPEED)) {
        Serial.println("Starting CAN failed!");
        while (1);
    }
    this->sendLeavePacket();
    this->sendEntryPacket();

    this->isInitialized = true;
}

void UrbanGardenBusClient::sendEntryPacket(){
    uint8_t idBytes[2];
    memcpy(idBytes, &this->nodeId, 2);
    Serial.print("Sending ENTRY packet... ");
    CAN.beginPacket(100);
    CAN.write(ENTRY_PACKET);
    CAN.write(idBytes[1]);
    CAN.write(idBytes[0]);
    CAN.endPacket();
    Serial.println("sent");
}

void UrbanGardenBusClient::printSensorMap(){
    // Print slots and sensors to console
    for (auto const &ent : this->sensors){
        Serial.print("Slot:");
        Serial.println(ent.first);
        Serial.print("Sensor id:");
        Serial.println(ent.second.sensorModelId);
    }
}

void UrbanGardenBusClient::sendLeavePacket(){
    uint8_t idBytes [2];
    memcpy(idBytes,&this->nodeId,2);
    Serial.print("Sending LEAVE packet... ");
    CAN.beginPacket(100);
    CAN.write(LEAVE_PACKET);
    CAN.write(idBytes[1]);
    CAN.write(idBytes[0]);
    CAN.endPacket();
    Serial.println("sent");
}

void UrbanGardenBusClient::sendAlivePacket(){
    uint8_t idBytes [2];
    memcpy(idBytes,&nodeId,2);
    Serial.print("Sending ALIVE packet... ");
    CAN.beginPacket(100);
    CAN.write(ALIVE_PACKET);
    CAN.write(idBytes[1]);
    CAN.write(idBytes[0]);
    CAN.endPacket();
    Serial.println("sent");
    lastAlivePacket = millis();
}

int16_t UrbanGardenBusClient::convertBytesToInt(uint8_t *data){
    int16_t i;
    memcpy(&i, data, sizeof(data));
    return i;
}

void UrbanGardenBusClient::handleEntryPacket(){
    uint8_t twoByteBuffer [2] = {this->packetBytes[2],this->packetBytes[1]};
    int nodeIdFromPacket = convertBytesToInt(twoByteBuffer);
    if(nodeIdFromPacket==HEADSTATION_ID){
        Serial.println("Got ENTRY_PACKET from headstation. Restarting node...");
        ESP.restart();
    }
}

void UrbanGardenBusClient::handleValueRequest(){
    Serial.println("Now handling value request");
    uint8_t twoByteBuffer [2] = {this->packetBytes[2],this->packetBytes[1]};
    int nodeIdFromPacket = convertBytesToInt(twoByteBuffer);
    if(this->nodeId==nodeIdFromPacket){
        int sensorSlot = packetBytes[3];
        auto it = this->sensors.find(sensorSlot);
        //Serial.println("Printing sensors...");
        //printSensorMap();
        if(it==this->sensors.end()){
            Serial.println("Requested sensor slot is not available");
        }else{
            // send VALUE_REQUEST_ACK packet
            memcpy(twoByteBuffer, &this->nodeId,2);
            Serial.print("Sending VALUE_REQUEST_ACK packet... ");
            CAN.beginPacket(100);
            CAN.write(VALUE_REQUEST_ACK);
            CAN.write(twoByteBuffer[1]);
            CAN.write(twoByteBuffer[0]);
            CAN.write(sensorSlot);
            CAN.endPacket();
            Serial.println("sent");

            Serial.print("Accessing sensor on slot ");
            Serial.print(sensorSlot);
            Serial.print(":");
            float value = it->second.getValueFunction();
            if (it->second.needsCalibration){
                Serial.print("(requires calibration with value ");
                Serial.print(it->second.calibrationValue);
                Serial.print(")");
                value = it->second.applyCalibrationFunction(value,it->second.calibrationValue);
            }
            Serial.println(value);
            uint8_t fourByteBuffer [4];
            memcpy(fourByteBuffer,&value,4);
            CAN.beginPacket(100);
            CAN.write(VALUE_RESPONSE);
            CAN.write(twoByteBuffer[1]);
            CAN.write(twoByteBuffer[0]);
            CAN.write(sensorSlot);
            CAN.write(fourByteBuffer[0]);
            CAN.write(fourByteBuffer[1]);
            CAN.write(fourByteBuffer[2]);
            CAN.write(fourByteBuffer[3]);
            CAN.endPacket();
        }
    }
}

bool UrbanGardenBusClient::registerSensor(UrbanGardenSensor newSensor, int timeout){
  //default timeout is 30 seconds (or 30.000 milliseconds)
    uint8_t buffer[2];
    memcpy(buffer,&this->nodeId,2);
    Serial.print("Sending SENSOR_REGISTER packet... ");
    CAN.beginPacket(100);
    CAN.write(SENSOR_REGISTER_PACKET);
    CAN.write(buffer[1]); // node id byte 1
    CAN.write(buffer[0]); // node id byte 2
    uint16_t modelId = newSensor.sensorModelId;
    memcpy(buffer,&modelId,2);
    CAN.write(buffer[1]); // sensor model id byte 1
    CAN.write(buffer[0]); // sensor model id byte 2
    CAN.write(newSensor.slot);
    CAN.endPacket();
    Serial.println("sent");

    bool registrationSuccessful = false;
    long timestamp = millis();
    while(!registrationSuccessful){
        if(millis()-timestamp>=timeout){
            break;
        }
        if (waitForCanPacket()) {
            if(packetBytes[0]==SENSOR_REGISTER_ACK_PACKET){
                uint8_t twoByteBuffer [2] = {this->packetBytes[2],this->packetBytes[1]};
                int nodeIdFromPacket = convertBytesToInt(twoByteBuffer);
                twoByteBuffer[0] = packetBytes[4];
                twoByteBuffer[1] = packetBytes[3];
                int sensorModelId = convertBytesToInt(twoByteBuffer);
                twoByteBuffer[0] = packetBytes[5];
                twoByteBuffer[1] = 0;
                int sensorSlot = convertBytesToInt(twoByteBuffer);
                Serial.print("Received a sensor register ack packet for node id ");
                Serial.println(nodeIdFromPacket);
                if (this->nodeId==nodeIdFromPacket &&
                    newSensor.sensorModelId==sensorModelId &&
                    newSensor.slot == sensorSlot){
                    registrationSuccessful = true;
                }
            }
            else{
                Serial.print("Tried to register sensor but received");
                Serial.println(packetBytes[0]);
            }
        }
    }
    //UrbanGardenSensor fml;
    //fml.slot = newSensor.slot;
    //fml.sensorModelId = newSensor.sensorModelId;
    //fml.getValueFunction= newSensor.getValueFunction;
    //fml.needsCalibration= newSensor.needsCalibration;
    //fml.applyCalibrationFunction=newSensor.applyCalibrationFunction;
    ////this->sensors.insert(std::pair<int, UrbanGardenSensor>(fml.slot, fml));
    //
    
    //this->sensors.insert({newSensor.slot, newSensor});
    this->sensors.insert(std::pair<int,UrbanGardenSensor>(newSensor.slot,newSensor));
    return registrationSuccessful;
}

bool UrbanGardenBusClient::handleCalibrationResponse(){
    bool calibrationSuccessful = false;

    if(this->packetBytes[0]==CALIBRATION_RESPONSE){
        uint8_t twoByteBuffer [2] = {this->packetBytes[2],this->packetBytes[1]};
        int nodeIdFromPacket = convertBytesToInt(twoByteBuffer);
        if (nodeIdFromPacket == this->nodeId){
            int sensorSlot = this->packetBytes[3];
            auto it = sensors.find(sensorSlot);
            if(it==this->sensors.end()){
                Serial.print("Slot ");
                Serial.print(sensorSlot);
                Serial.print(" is not available. Sending CALIBRATION_ERR_INVALID_SLOT packet. .. ");
                CAN.beginPacket(100);
                CAN.write(CALIBRATION_ERR_INVALID_SLOT);
                CAN.write(twoByteBuffer[1]);
                CAN.write(twoByteBuffer[0]);
                CAN.write(sensorSlot);
                CAN.endPacket();
                Serial.println("sent");
            }else{
                Serial.print("Sensor slot for calibration is ");
                Serial.println(sensorSlot);
                uint8_t fourByteBuffer [4] = {this->packetBytes[4],
                    this->packetBytes[5],
                    this->packetBytes[6],
                    this->packetBytes[7]
                };
                float calibrationValue;
                memcpy(&calibrationValue, fourByteBuffer, 4);
                Serial.print("Received calibration value: ");
                Serial.println(calibrationValue);
                this->sensors.find(sensorSlot)->second.calibrationValue = calibrationValue;
                calibrationSuccessful = true;
                memcpy(twoByteBuffer,&this->nodeId,2);
                Serial.print("Sending CALIBRATION_ACK packet... ");
                CAN.beginPacket(100);
                CAN.write(CALIBRATION_ACK);
                CAN.write(twoByteBuffer[1]);
                CAN.write(twoByteBuffer[0]);
                CAN.write(sensorSlot);
                CAN.endPacket();
                Serial.println("sent");
            }
        }else{
            Serial.println("Ignoring CALIBRATION_RESPONSE (it is for another node)");
        }
    }
    return calibrationSuccessful;
}

bool UrbanGardenBusClient::requestCalibrationForSensor(UrbanGardenSensor &sensor, float timeout){
    uint8_t twoByteBuffer [2];
    memcpy(twoByteBuffer,&this->nodeId,2);
    Serial.print("Sending CALIBRATION_REQUEST packet... ");
    CAN.beginPacket(100);
    CAN.write(CALIBRATION_REQUEST);
    CAN.write(twoByteBuffer[1]);
    CAN.write(twoByteBuffer[0]);
    memcpy(twoByteBuffer,&sensor.sensorModelId,2);
    CAN.write(twoByteBuffer[1]);
    CAN.write(twoByteBuffer[0]);
    CAN.write(sensor.slot);
    CAN.endPacket();
    Serial.println("sent");
    long timestamp = millis();
    bool calibrationSuccessful = false;
    while(!calibrationSuccessful){
        if(millis()-timestamp>=timeout){
            Serial.println("Calibration request timeout");
            break;
        }
        if (waitForCanPacket()) {
            calibrationSuccessful = handleCalibrationResponse();
        }
    }
    return calibrationSuccessful;
}

bool UrbanGardenBusClient::waitForCanPacket(){
    int packetSize = CAN.parsePacket();
    if (packetSize) {
        for (int i = 0; i<packetSize; i++){
            packetBytes[i] = CAN.read();
            Serial.print(packetBytes[i]);
        }
        if (packetBytes[0]==ENTRY_PACKET){
            handleEntryPacket();
        }
        uint8_t twoByteBuffer [2] = {packetBytes[2],packetBytes[1]};
        int nodeIdFromPacket = convertBytesToInt(twoByteBuffer);
        Serial.print("Received a packet from node with id ");
        Serial.println(nodeIdFromPacket);
        if (nodeIdFromPacket==HEADSTATION_ID){ //check if headstation sent ALIVE packet
            this->lastHeadstationAlivePacket = millis();
            Serial.print("Received a headstation ALIVE packet at millis: ");
            Serial.println(this->lastHeadstationAlivePacket);
            if (lorawanEnabled){
                Serial.println("Disabling LORAWAN connectivity");
                LMIC_reset();
                lorawanEnabled = false;
            }
        }
        return true;
    }
    return false;
}

bool UrbanGardenBusClient::headstationTimeout(){
    //return true;
    return millis()-this->lastHeadstationAlivePacket>=HEADSTATION_TIMEOUT;
}

LoraMessage UrbanGardenBusClient::fillLoraMessage(){
    for (auto const &ent : this->sensors){
        auto it = this->sensors.find(ent.first);
        float value = it->second.getValueFunction();
        
        if (it->second.needsCalibration){
            Serial.println(it->second.calibrationValue);
            value = it->second.applyCalibrationFunction(value,it->second.calibrationValue);
            Serial.println(value);
        }
        message.addRawFloat(value);
    }
}

void UrbanGardenBusClient::do_send(osjob_t* j){
    Serial.println("do_send");
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
        //Serial.println(F("Rescheduling TX"));

        //os_setTimedCallback(&sendjob, os_getTime(), do_send);
        
    } else {
        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        //Serial.println(F("Packet queued"));
        
        int length = message.getLength();
        
        for (int i = 0; i<length;i++){
          Serial.print(message.getBytes()[i]);
        }
        Serial.println();

        LMIC_setTxData2(1, message.getBytes(), message.getLength(), 0);
        

    }

    // Next TX is scheduled after TX_COMPLETE event.
}

void UrbanGardenBusClient::do_loop(){
    if(!this->isInitialized){
        Serial.println("UrbanGardenBusClient has to be initialized before looping.");
        while(1){}
    }
    os_runloop_once();
    if (millis()-this->lastAlivePacket>=this->tickRate){
        sendAlivePacket();
    }
    bool packetPresent = this->waitForCanPacket();
    if (packetPresent){
        Serial.println("CANBUS packet received");
    }
    if(this->headstationTimeout() && !this->lorawanEnabled){
        Serial.println("HEADSTATION TIMEOUT, FALLING BACK TO LORAWAN CONNECTIVITY");
        //os_init();
        lorawanEnabled=true; //prevents the following line to be executed over and over again
        // Start job (sending automatically starts OTAA too)
        //
        this->fillLoraMessage();
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), &do_send);
        
        //os_setTimedCallback(this->sendjob,os_getTime(),do_send);
    }

    if (packetPresent) {
        switch(packetBytes[0]){
            case VALUE_REQUEST:
                handleValueRequest();
            break;
            case CALIBRATION_RESPONSE:
                Serial.println("Handling calibration response");
                handleCalibrationResponse();
            break;    
        }
        packetPresent = false;
    }  
}
