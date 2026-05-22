
#ifndef PhotoMux_h
#define PhotoMux_h

#include <Arduino.h>
#include <limits.h>

// - Sides around the robot
enum Side { FRONT, LEFT, RIGHT, BACK };

class PhotoMux {
public:

    // - Sensor: mux index + analog channel
    struct Sensor {
        uint8_t muxIndex;   // - mux number
        uint8_t channel;    // - analog pin (A0..)
    };

    // - Constructor: store pin mappings
    PhotoMux(const uint8_t selectPins[3], const uint8_t muxPins[4]);

    // - init pins
    void begin();

    // - configure sensors for a side
    void configureSide(Side side, const Sensor* sensors, uint8_t count);
    // - set detection thresholds range
    void setThresholdRange(Side side, int minThreshold, int maxThreshold);

    // - detection helpers
    bool isLineDetected(Side side); // - compare avg to threshold
    float getAverage(Side side);    // - averaged reading
    float getRawAverage(Side side); // - raw averaged reading
    int readSensor(uint8_t muxIndex, uint8_t channel); // - read one sensor

private:
    uint8_t _selectPins[3];     // - mux select pins
    uint8_t _muxPins[4];        // - analog input pins

    int minThresholds[4];       // - per-side minimum thresholds
    int maxThresholds[4];       // - per-side maximum thresholds

    // - sensor arrays per side
    Sensor* frontSensors;
    uint8_t frontCount;

    Sensor* leftSensors;
    uint8_t leftCount;

    Sensor* rightSensors;
    uint8_t rightCount;

    Sensor* backSensors;
    uint8_t backCount;

    int max_channel = 3;


    // - low-level helpers
    void selectChannel(uint8_t channel); // - set mux selects
    
    float readAverage(const Sensor* sensors, uint8_t size); // - avg on array

    void readWithoutMux(const Sensor* sensors, uint8_t size, float& average, float& rawAverage); // - direct read

    /* How to use readWithoutMux:
    Sensors on A0, A1
    Sensor frontSensors[] = {
    {0, A0},
    {0, A1}
    };
    float frontAvg, frontRaw;
    readWithoutMux(frontSensors, 2, frontAvg, frontRaw);
    frontAvg now holds the average reading of the front sensors
    LEFT 
    Sensors on A2, A3
    Sensor leftSensors[] = {
    {0, A2},
    {0, A3}
    };
    float leftAvg, leftRaw;
    readWithoutMux(leftSensors, 2, leftAvg, leftRaw);
    */
};

#endif