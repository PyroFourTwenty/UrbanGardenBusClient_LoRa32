#include <Arduino.h>
#include <map>
#include <CAN.h>
#include <lmic.h>
#include <LoraMessage.h>

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6

struct UrbanGardenSensor{
  int8_t slot;
  int16_t sensorModelId;
  float (*getValueFunction)(); //https://www.section.io/engineering-education/function-pointers-in-c++/
  float (*applyCalibrationFunction)(float, float);
  float calibrationValue;
  bool needsCalibration = false;
};

class UrbanGardenBusClient{
    private:
        int16_t nodeId;
        int tickRate;
        bool waitForCanPacket();
        int16_t convertBytesToInt(uint8_t *data);
        float convertBytesToFloat(uint8_t *data);
        LoraMessage fillLoraMessage();
        void handleValueRequest();
        uint8_t packetBytes [8];
        long lastAlivePacket;
        long lastHeadstationAlivePacket = 0;
        bool lorawanEnabled;
        bool headstationTimeout();
        unsigned TX_INTERVAL = 10;
        static const u1_t PROGMEM internalAPPEUI[8];
        static const u1_t PROGMEM internalDEVEUI[8];
        static const u1_t PROGMEM internalAPPKEY[16];
        static void do_send(osjob_t* j);
        bool isInitialized;
    public:
        osjob_t sendjob;

        void onEvent(ev_t ev);
        void sendEntryPacket();
        void printSensorMap();
        void sendLeavePacket();
        void sendAlivePacket();
        bool registerSensor(UrbanGardenSensor newSensor, int timeout = 30.0E3*6);
        bool requestCalibrationForSensor(UrbanGardenSensor &sensor, float timeout=30.0E3*6);
        bool handleCalibrationResponse();
        void handleEntryPacket();
        std::map<int, UrbanGardenSensor> sensors;
        void do_loop();
        UrbanGardenBusClient();
        UrbanGardenBusClient(int16_t nodeId, int tickRate);
};