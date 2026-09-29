#pragma once

#include "SerialIO.h"

#include "device.h"

typedef struct
{
    uint32_t lat;       // Latitude in decimal degrees
    uint32_t lon;       // Longitude in decimal degrees
    uint32_t alt;       // Altitude in meters
    uint32_t speed;     // Speed in km/h
    uint16_t heading;   // Heading in degrees, positive. 0 is north.
    uint8_t satellites; // Number of satellites

    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} GpsData;

class HardwareSerial;

class SerialGPS final : public SerialIO {
public:
    explicit SerialGPS(Stream &out, Stream &in, HardwareSerial *hw = nullptr) 
        : SerialIO(&out, &in), _hwPort(hw) {}
    ~SerialGPS() override = default;

    typedef void (*gpsFieldParser_t)(SerialGPS *ctx, uint8_t fieldIdx, char *field);

    void sendQueuedData(uint32_t maxBytesToSend) override;
    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override { return DURATION_IMMEDIATELY; }
private:
    void processBytes(uint8_t *bytes, uint16_t size) override;
    void sendTelemetryFrame();
    void sendGpsTimeTelemetryFrame();
    bool isValidChecksum(char *sentence, uint8_t size);
    void processSentence(char *sentence, uint8_t size);
    void splitSentenceFields(char *sentence, uint8_t size, gpsFieldParser_t callback);

    static void fieldParseGGA(SerialGPS *ctx, uint8_t fieldIdx, char *field);
    static void fieldParseVTG(SerialGPS *ctx, uint8_t fieldIdx, char *field);
    static void fieldParseRMC(SerialGPS *ctx, uint8_t fieldIdx, char *field);

    void sendDiagnosticTelemetryFrame(bool streamLost);

    GpsData gpsData = {0};
    // NMEA 0183 has a maximum 82 byte sentence, including the end delimiter \r\n
    char nmeaBuffer[83] = {0};
    uint8_t nmeaBufferIndex = 0;

    HardwareSerial *_hwPort = nullptr;
    bool hasGga = false;
    uint8_t currentBaudIndex = 0;
    bool baudLocked = false;
    uint32_t lastBaudSwitchMs = 0;
    uint32_t validPacketsCount = 0;
    uint32_t rawBytesCount = 0;
    uint32_t csumErrors = 0;
    uint32_t lastValidPacketMs = 0;
    uint32_t lastDiagFrameMs = 0;
};

