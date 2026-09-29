#include "SerialGPS.h"
#include <HardwareSerial.h>

#include "CRSFRouter.h"
#include <crsf_protocol.h>

static const uint32_t gpsBaudRates[] = { 9600, 115200, 38400, 57600 };
static const uint8_t gpsBaudRatesCount = sizeof(gpsBaudRates) / sizeof(gpsBaudRates[0]);

void SerialGPS::sendQueuedData(uint32_t maxBytesToSend)
{
    uint32_t now = millis();
    if (!baudLocked && _hwPort != nullptr)
    {
        if (lastBaudSwitchMs == 0)
        {
            lastBaudSwitchMs = now;
        }
        else if (now - lastBaudSwitchMs >= 2500)
        {
            lastBaudSwitchMs = now;
            currentBaudIndex = (currentBaudIndex + 1) % gpsBaudRatesCount;
            uint32_t newBaud = gpsBaudRates[currentBaudIndex];
            _hwPort->flush();
            _hwPort->updateBaudRate(newBaud);
            if (_hwPort != nullptr)
            {
                while (_hwPort->available())
                {
                    _hwPort->read();
                }
            }
            nmeaBufferIndex = 0;
            DBGLN("GPS Auto-Baud: trying %u baud", newBaud);
        }
    }

    bool streamLost = (validPacketsCount > 0 && (now - lastValidPacketMs > 3000));
    if (validPacketsCount == 0 || streamLost)
    {
        if (now - lastDiagFrameMs >= 1000)
        {
            lastDiagFrameMs = now;
            sendDiagnosticTelemetryFrame(streamLost);
        }
    }
}


/***
* @brief Parses a decimal string with optional decimal point and returns the value scaled by the given factor as an integer
*   Ex: "0.442" with scale 100 returns 44
*   Ex: "123.456" with scale 1000 returns 123456
*/
static int32_t parseDecimalToScaled(const char* str, int32_t scale) {
    char *end;
    int32_t whole = strtol(str, &end, 10);
    int32_t result = whole * scale;

    if (*end == '.') {
        const char* dec = end + 1;
        int32_t divisor = 1;
        int32_t decimalPart = 0;

        // Count decimal places in scale
        int32_t scaleDecimals = 0;
        int32_t tempScale = scale;
        while (tempScale > 1) {
            scaleDecimals++;
            tempScale /= 10;
        }

        // Process up to scaleDecimals digits
        for (int i = 0; i < scaleDecimals && dec[i] != '\0'; i++) {
            decimalPart = decimalPart * 10 + (dec[i] - '0');
            divisor *= 10;
        }

        // Scale the decimal part
        if (divisor > 1) {
            while (divisor < scale) {
                decimalPart *= 10;
                divisor *= 10;
            }
            result += decimalPart;
        }
    }
    return result;
}

/***
 * @brief Parse NMEA Decimal Degrees Minutes value and return Decimal Degrees * 1e7.
 * @param field Points to beginning of DDMM[M] string, must not be blank!
 */
static int32_t nmeaDdmToDd(const char *field)
{
    // Latitude is DDMM.MMMMM, Longitude is DDDMM.MMMM
    // Start by getting the part before the decimal, and divide by 100 to remove the MM part
    int32_t degrees = atoi(field) / 100;

    // Minutes is always two digits. Find the decimal and look 2 characters before it
    char *minutes = strchr(field, '.');
    if (minutes == nullptr)
        return 0;
    int32_t minutesPart = parseDecimalToScaled(minutes - 2, 10000000) / 60;

    return degrees * 10000000 + minutesPart;
}

bool SerialGPS::isValidChecksum(char *sentence, uint8_t size)
{
    // Could also check for the \r\n but we know it at least has the \n to get here
    if (size < 6 || sentence[0] != '$' || sentence[size-5] != '*')
    {
        return false;
    }

    // Checksum in a NMEA packet starts after the $ and stops before the *XX and is a simple XOR
    uint8_t csumCalculated = 0;
    for (unsigned b=1; b<size-5; ++b)
    {
        csumCalculated ^= sentence[b];
    }

    uint8_t csumSentence = strtol((char *)&sentence[size-4], nullptr, 16);
    if (csumCalculated != csumSentence)
    {
        return false;
    }

    if (!baudLocked)
    {
        baudLocked = true;
        DBGLN("GPS Baud locked at %u", gpsBaudRates[currentBaudIndex]);
    }
    validPacketsCount++;
    lastValidPacketMs = millis();
    return true;
}

void SerialGPS::splitSentenceFields(char *sentence, uint8_t size, gpsFieldParser_t callback)
{
    uint8_t fieldIdx = 0;
    char *fieldStart = sentence;

    //sentence[size] = 0; DBG(sentence);
    for (unsigned i=0; i<size; ++i)
    {
        if (sentence[i] == ',' || sentence[i] == '*')
        {
            sentence[i] = 0;
            callback(this, fieldIdx, fieldStart);
            fieldStart = &sentence[i+1];
            ++fieldIdx;
        }
    }
}

void SerialGPS::fieldParseGGA(SerialGPS *ctx, uint8_t fieldIdx, char *field)
{
    const bool blank = (field[0] == '\0');

    switch (fieldIdx)
    {
        case 2:
            ctx->gpsData.lat = (blank) ? 0 : nmeaDdmToDd(field);
            break;
        case 3:
            if (field[0] == 'S')
                ctx->gpsData.lat = -ctx->gpsData.lat;
            break;
        case 4:
            ctx->gpsData.lon = (blank) ? 0 : nmeaDdmToDd(field);
            break;
        case 5:
            if (field[0] == 'W')
                ctx->gpsData.lon = -ctx->gpsData.lon;
            break;
        case 7:
            ctx->gpsData.satellites = atoi(field);
            break;
        case 9:
            ctx->gpsData.alt = (blank) ? 0 : parseDecimalToScaled(field, 100);;
            break;

    }
}

void SerialGPS::fieldParseVTG(SerialGPS *ctx, uint8_t fieldIdx, char *field)
{
    const bool blank = (field[0] == '\0');

    switch (fieldIdx)
    {
        case 1:
            ctx->gpsData.heading = (blank) ? 0 : parseDecimalToScaled(field, 100);
            break;
        case 7:
            ctx->gpsData.speed = (blank) ? 0 : parseDecimalToScaled(field, 100);
            break;
    }
}

void SerialGPS::fieldParseRMC(SerialGPS *ctx, uint8_t fieldIdx, char *field)
{
    const bool blank = (field[0] == '\0');
    if (blank) return;

    switch (fieldIdx) {
        case 1: // Time: HHMMSS.ss
        {
            uint32_t time_ms = parseDecimalToScaled(field, 1000);
            ctx->gpsData.millisecond = time_ms % 1000;
            time_ms /= 1000;
            ctx->gpsData.second = time_ms % 100;
            time_ms /= 100;
            ctx->gpsData.minute = time_ms % 100;
            time_ms /= 100;
            ctx->gpsData.hour = time_ms % 100;
            break;
        }
        case 3: // Latitude: DDMM.MMMMM
        {
            ctx->gpsData.lat = nmeaDdmToDd(field);
            break;
        }
        case 4: // N/S
        {
            if (field[0] == 'S')
                ctx->gpsData.lat = -ctx->gpsData.lat;
            break;
        }
        case 5: // Longitude: DDDMM.MMMMM
        {
            ctx->gpsData.lon = nmeaDdmToDd(field);
            break;
        }
        case 6: // E/W
        {
            if (field[0] == 'W')
                ctx->gpsData.lon = -ctx->gpsData.lon;
            break;
        }
        case 7: // Speed over ground: knots -> scaled km/h * 100 (matching VTG)
        {
            int32_t knots100 = parseDecimalToScaled(field, 100);
            ctx->gpsData.speed = (uint32_t)((knots100 * 1852LL + 500) / 1000);
            break;
        }
        case 8: // Track angle in degrees -> scaled by 100
        {
            ctx->gpsData.heading = parseDecimalToScaled(field, 100);
            break;
        }
        case 9: // Date: DDMMYY
        {
            uint32_t date = atoi(field);
            ctx->gpsData.year = 2000 + date % 100;
            date /= 100;
            ctx->gpsData.month = date % 100;
            date /= 100;
            ctx->gpsData.day = date % 100;
            break;
        }
    }
}

void SerialGPS::processSentence(char *sentence, uint8_t size)
{
    if (sentence[3] == 'G' && sentence[4] == 'G' && sentence[5] == 'A') {
        splitSentenceFields(sentence, size, &fieldParseGGA);
        hasGga = true;
        sendTelemetryFrame();
    }
    else if (sentence[3] == 'V' && sentence[4] == 'T' && sentence[5] == 'G') {
        splitSentenceFields(sentence, size, &fieldParseVTG);
        // VTG usually comes before GGA, only generate one telemetry frame with both combined
        //sendTelemetryFrame();
    }
    else if (sentence[3] == 'R' && sentence[4] == 'M' && sentence[5] == 'C') {
        splitSentenceFields(sentence, size, &fieldParseRMC);
        sendGpsTimeTelemetryFrame();
        // If receiver does not emit GGA, still send telemetry frame from RMC data!
        if (!hasGga) {
            sendTelemetryFrame();
        }
    }
}


void SerialGPS::processBytes(uint8_t *bytes, uint16_t size)
{
    rawBytesCount += size;
    for (uint16_t i = 0; i < size; i++) {
        const char c = bytes[i];
        if (nmeaBufferIndex < sizeof(nmeaBuffer)) {
            nmeaBuffer[nmeaBufferIndex++] = c;
        } else {
            // Buffer overflow without newline (garbage data or mismatched baud)
            csumErrors++;
            nmeaBufferIndex = 0;
        }
        if (c == '\n') {
            // Note that the buffer/size includes the \r\n
            if (isValidChecksum(nmeaBuffer, nmeaBufferIndex)) {
                processSentence(nmeaBuffer, nmeaBufferIndex);
            } else if (nmeaBufferIndex > 5) {
                csumErrors++;
            }
            nmeaBufferIndex = 0;
        }
    }
}

void SerialGPS::sendGpsTimeTelemetryFrame()
{
    // Don't send if there hasn't been an update to the year field
    if (gpsData.year == 0)
        return;

    CRSF_MK_FRAME_T(crsf_sensor_gps_time_t) crsftime{};
    crsftime.p.year = htobe16(gpsData.year);
    crsftime.p.month = gpsData.month;
    crsftime.p.day = gpsData.day;
    crsftime.p.hour = gpsData.hour;
    crsftime.p.minute = gpsData.minute;
    crsftime.p.second = gpsData.second;
    crsftime.p.millisecond = htobe16(gpsData.millisecond);
    crsfRouter.SetHeaderAndCrc(&crsftime.h, CRSF_FRAMETYPE_GPS_TIME, CRSF_FRAME_SIZE(sizeof(crsf_sensor_gps_time_t)));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsftime.h);

    gpsData.year = 0;
}

void SerialGPS::sendTelemetryFrame()
{
    CRSF_MK_FRAME_T(crsf_sensor_gps_t) crsfgps{};
    crsfgps.p.latitude = htobe32(gpsData.lat);
    crsfgps.p.longitude = htobe32(gpsData.lon);
    crsfgps.p.altitude = htobe16((int16_t)(gpsData.alt / 100 + 1000));
    crsfgps.p.groundspeed = htobe16((uint16_t)(gpsData.speed / 10));
    crsfgps.p.satellites_in_use = gpsData.satellites;
    crsfgps.p.gps_heading = htobe16(gpsData.heading);
    crsfRouter.SetHeaderAndCrc(&crsfgps.h, CRSF_FRAMETYPE_GPS, CRSF_FRAME_SIZE(sizeof(crsf_sensor_gps_t)));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfgps.h);
}

void SerialGPS::sendDiagnosticTelemetryFrame(bool streamLost)
{
    uint8_t diagState = 0;
    if (streamLost)
    {
        diagState = 4; // Lost / Timeout (>3s since last valid packet)
    }
    else if (rawBytesCount == 0)
    {
        diagState = 0; // No bytes received (wire broken / no power / wrong pin)
    }
    else if (!baudLocked)
    {
        diagState = (csumErrors > 0) ? 2 : 1; // 1 = scanning baud, 2 = checksum fail
    }
    else
    {
        diagState = 3; // Baud locked, waiting for valid sentences
    }

    uint32_t currentBaud = gpsBaudRates[currentBaudIndex];
    uint16_t speedScaled = (uint16_t)(currentBaud / 100);

    CRSF_MK_FRAME_T(crsf_sensor_gps_t) crsfgps{};
    crsfgps.p.latitude = 0;
    crsfgps.p.longitude = 0;
    crsfgps.p.groundspeed = htobe16(speedScaled);
    crsfgps.p.gps_heading = htobe16((uint16_t)(csumErrors % 36000));
    crsfgps.p.altitude = htobe16((int16_t)(1000 + (rawBytesCount % 10000)));
    crsfgps.p.satellites_in_use = diagState;

    crsfRouter.SetHeaderAndCrc(&crsfgps.h, CRSF_FRAMETYPE_GPS, CRSF_FRAME_SIZE(sizeof(crsf_sensor_gps_t)));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfgps.h);
}