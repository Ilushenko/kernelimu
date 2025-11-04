#include "IMUKernel.h"

#define xLog(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#ifdef DEBUGLOG
# define xDebug(fmt, ...) Serial.printf("[%s %s:%i] ", __FUNCTION__, __FILENAME__, __LINE__); Serial.printf(fmt, ##__VA_ARGS__)
#else
# define xDebug(fmt, ...)
#endif

il::IMUKernel::IMUKernel(HardwareSerial& serial) : _serial(serial)
{
}

il::IMUKernel::~IMUKernel()
{
}

bool il::IMUKernel::begin()
{
    // Stop
    if (!setCommand(MSG_STOP)) return false;
    waitAvailable(2000);
    // Get DevInfo
    if (!setCommand(MSG_DEVINFO)) return false;
    if (!waitAvailable(2000)) return false;
    if (!readSerialData()) return false;
    // Params
    // Get Data Rate
    if (!setCommand(RAM_READ, PRM_DATA_RATE, 2)) return false;
    if (!waitAvailable(2000)) return false;
    if (!readSerialData()) return false;
    // Get Baud Rate
    if (!setCommand(RAM_READ, PRM_BAUD_RATE, 1)) return false;
    if (!waitAvailable(2000)) return false;
    if (!readSerialData()) return false;
    // Get Auto Start
    if (!setCommand(RAM_READ, PRM_AUTO_START, 1)) return false;
    if (!waitAvailable(2000)) return false;
    if (!readSerialData()) return false;
    // Average output data
    if (!setCommand(RAM_READ, PRM_AVG_OUTDATA, 1)) return false;
    if (!waitAvailable(2000)) return false;
    if (!readSerialData()) return false;
    // Initial alignment time
    if (!setCommand(RAM_READ, PRM_INIT_ATIME, 2)) return false;
    if (!waitAvailable(2000)) return false;
    if (!readSerialData()) return false;
    // Alignment Angles
    if (!setCommand(RAM_READ, PRM_ALIGN_ANG, sizeof(float) * 3)) return false;
    if (!waitAvailable(2000)) return false;
    if (!readSerialData()) return false;

    return true;
}

bool il::IMUKernel::stop()
{
    _currentCmd = 0;
    return setCommand(MSG_STOP);
}

void il::IMUKernel::update()
{
    uint8_t id = 0;
    while (_serial.available() > 0) {
        id = readSerialData();
        if (_callback != nullptr) _callback(id, &_orientation);
    }
}

bool il::IMUKernel::provide(const uint8_t cmd)
{
    if (_currentCmd == cmd) return true;
    switch (cmd) {
    case MSG_ORIENTATION:
    case MSG_GA:
    case MSG_GA_M:
    case MSG_GAA:
    case MSG_GAA_M:
    case MSG_QUAT:
    case MSG_CALIB_HR:
        break;
    default:
        xDebug("Unsupport prividing format: %02X\n", cmd);
        return false;
    }
    if (_currentCmd != 0) {
        if (!stop()) return false;
    }
    if (!setCommand(cmd)) return false;
    _currentCmd = cmd;
    return true;    
}

bool il::IMUKernel::setDataRate(uint16_t rate)
{
    if (rate == _dataRate) return true;

    if (rate > 2000) rate = 2000;
    if (rate > 1000 && rate < 2000) rate = 1000;
    if (rate > 500 && rate < 1000) rate = 500;
    if (rate > 400 && rate < 500) rate = 400;
    if (rate > 250 && rate < 400) rate = 250;
    if (rate > 200 && rate < 250) rate = 200;
    if (rate > 125 && rate < 200) rate = 125;
    if (rate > 100 && rate < 125) rate = 100;
    if (rate > 80 && rate < 100) rate = 80;
    if (rate > 50 && rate < 80) rate = 50;
    if (rate > 40 && rate < 50) rate = 40;
    if (rate > 25 && rate < 40) rate = 25;
    if (rate > 20 && rate < 25) rate = 20;
    if (rate > 16 && rate < 20) rate = 16;
    if (rate > 10 && rate < 16) rate = 10;
    if (rate == 9) rate = 8;
    if (rate == 7) rate = 5;
    if (rate == 6) rate = 5;
    if (rate == 3) rate = 2;

    if (_currentCmd != 0) setCommand(MSG_STOP);
    if (!setCommand(RAM_SAVE, PRM_DATA_RATE, sizeof(rate), reinterpret_cast<const uint8_t*>(&rate))) return false;
    if (!waitAvailable(10000)) return false;
    if (readSerialData() != MSG_SAVE_PARAM) return false;
    _dataRate = rate;
    xDebug("Set Data Rate: %u\n", rate);
    if (_currentCmd != 0) setCommand(_currentCmd);
    return true;
}

bool il::IMUKernel::setBaudRate(uint8_t rate)
{
    if (rate == _baudRate) return true;

    switch (rate) {
    case BPS_DEFAULT:
        rate = BPS_115200;
        break;
    case BPS_4800:
    case BPS_9600:
    case BPS_14400:
    case BPS_19200:
    case BPS_38400:
    case BPS_57600:
    case BPS_115200:
    case BPS_230400:
    case BPS_460800:
    case BPS_921600:
    case BPS_2000000:
    case BPS_375000:
    case BPS_1843200:
    case BPS_3686400:
    case BPS_1000000:
    case BPS_4000000:
        break;
    default:
        xDebug("Unssupport Baud Rate ID:%u\n", rate);
        return false;
    }

    if (_currentCmd != 0) setCommand(MSG_STOP);
    if (!setCommand(RAM_SAVE, PRM_BAUD_RATE, sizeof(rate), reinterpret_cast<const uint8_t*>(&rate))) return false;
    if (!waitAvailable(10000)) return false;
    if (readSerialData() != MSG_SAVE_PARAM) return false;
    _baudRate = rate;
    xDebug("Set Baud Rate: %u\n", rate);
    if (_currentCmd != 0) setCommand(_currentCmd);
    return true;
}

bool il::IMUKernel::setAutoStart(const uint8_t id)
{
    if (id == _autoStart) return true;

    switch (id) {
    case MSG_ORIENTATION:
    case MSG_GA:
    case MSG_GA_M:
    case MSG_GAA:
    case MSG_GAA_M:
    case MSG_QUAT:
    case MSG_CALIB_HR:
        break;
    default:
        xDebug("Unsupport Auto Start Command: %02X\n", id);
        return false;
    }

    if (_currentCmd != 0) setCommand(MSG_STOP);
    if (!setCommand(RAM_SAVE, PRM_AUTO_START, sizeof(id), reinterpret_cast<const uint8_t*>(&id))) return false;
    if (!waitAvailable(10000)) return false;
    if (readSerialData() != MSG_SAVE_PARAM) return false;
    _autoStart = id;
    xDebug("Set Auto Start: %02X\n", id);
    if (_currentCmd != 0) setCommand(_currentCmd);
    return true;
}

bool il::IMUKernel::setAvgOutData(const bool averaged)
{
    if (averaged == static_cast<bool>(_avgOutData)) return true;

    const uint8_t outdata = averaged ? 0x01 : 0x00;
    if (_currentCmd != 0) setCommand(MSG_STOP);
    if (!setCommand(RAM_SAVE, PRM_AVG_OUTDATA, sizeof(outdata), reinterpret_cast<const uint8_t*>(&outdata))) return false;
    if (!waitAvailable(10000)) return false;
    if (readSerialData() != MSG_SAVE_PARAM) return false;
    _avgOutData = outdata;
    xDebug("Set Output Data: %s\n", (averaged ? "Averaged" : "Instant"));
    if (_currentCmd != 0) setCommand(_currentCmd);
    return true;
}

bool il::IMUKernel::setInitAlignTime(const uint16_t sec)
{
    if (sec == _initAlignTime) return true;

    if (_currentCmd != 0) setCommand(MSG_STOP);
    if (!setCommand(RAM_SAVE, PRM_INIT_ATIME, sizeof(sec), reinterpret_cast<const uint8_t*>(&sec))) return false;
    if (!waitAvailable(10000)) return false;
    if (readSerialData() != MSG_SAVE_PARAM) return false;
    _initAlignTime = sec;
    xDebug("Set Initial alignment time: %u\n", sec);
    if (_currentCmd != 0) setCommand(_currentCmd);
    return true;
}

bool il::IMUKernel::setAlignAngles(const float &yaw, const float &pitch, const float &roll)
{
    if (yaw == _alignAngle[0] && pitch == _alignAngle[1] && roll == _alignAngle[2]) return true;

    const float angle[3] = { yaw, pitch, roll };
    if (_currentCmd != 0) setCommand(MSG_STOP);
    if (!setCommand(RAM_SAVE, PRM_ALIGN_ANG, sizeof(angle), reinterpret_cast<const uint8_t*>(&angle))) return false;
    if (!waitAvailable(10000)) return false;
    if (readSerialData() != MSG_SAVE_PARAM) return false;
    _alignAngle[0] = angle[0];
    _alignAngle[1] = angle[1];
    _alignAngle[2] = angle[2];
    xDebug("Set alignment Angles[ yaw: %f, pitch: %f, roll: %f]\n", angle[0], angle[1], angle[2]);
    if (_currentCmd != 0) setCommand(_currentCmd);
    return true;
}

bool il::IMUKernel::setCommand(const uint8_t *payload, uint16_t size)
{
    uint8_t cmd[32] = { HEAD0, HEAD1, HEAD_TYPE_CMD, 0x00 };
    cmd[4] = static_cast<uint8_t>((size + 6) & 0xFF);
    cmd[5] = static_cast<uint8_t>((size + 6) >> 8);
    memcpy(&cmd[6], payload, size);
    _checksum = 0;
    for (uint16_t i = 2; i < size + 6; ++i) _checksum += cmd[i];
    cmd[6 + size] = static_cast<uint8_t>(_checksum & 0xFF);
    cmd[7 + size] = static_cast<uint8_t>(_checksum >> 8);
    return _serial.write(&cmd[0], size + 8) == (size + 8);
}

bool il::IMUKernel::setCommand(const uint16_t cmd, const uint16_t param, uint16_t size, const uint8_t *value)
{
    switch (param) {
    case PRM_DATA_RATE:
    case PRM_INIT_ATIME:
    case PRM_ALIGN_ANG:
    case PRM_BAUD_RATE:
    case PRM_AUTO_START:
    case PRM_AVG_OUTDATA:
        break;
    default:
        xDebug("Unsupport Param ID\n");
        return false;
    }
    uint8_t payload[24] = { 0 };
    payload[0] = static_cast<uint8_t>(cmd & 0xFF);
    payload[1] = static_cast<uint8_t>(cmd >> 8);
    payload[2] = static_cast<uint8_t>(param & 0xFF);
    payload[3] = static_cast<uint8_t>(param >> 8);
    payload[4] = static_cast<uint8_t>(size & 0xFF);
    payload[5] = static_cast<uint8_t>(size >> 8);
    if (cmd == RAM_READ) {
        _param = param;
        return setCommand(&payload[0], 6);
    }
    if (value == nullptr) return false;
    memcpy(&payload[6], value, size);
    return setCommand(&payload[0], size + 6);
}

uint8_t il::IMUKernel::readSerialData()
{
    uint8_t data = 0;
    // 0xAA
    if (_serial.read(&data, 1) < 1) {
        xDebug("Error read byte\n");
        return 0;
    }
    if (data != HEAD0) return 0;
    // 0x55
    if (_serial.read(&data, 1) != 1) {
        xDebug("Error read byte\n");
        return 0;
    }
    if (data != HEAD1) return 0;
    // 0x01    
    if (_serial.read(&data, 1) != 1) {
        xDebug("Error read byte\n");
        return 0;
    }
    if (data != HEAD_TYPE_MSG) return 0;
    // Kernel Data ID
    if (_serial.read(&data, 1) != 1) {
        xDebug("Error read byte\n");
        return 0;
    }
    // Message length
    uint16_t lenght = 0;
    if (_serial.read(reinterpret_cast<uint8_t*>(&lenght), sizeof(uint16_t)) != 2) {
        xDebug("Error read size\n");
        return 0;
    }
    enum {
        PAYLOAD_IDX = 6,
        PAYLOAD_SIZE = 192
    };
    const uint8_t id = data;
    const uint16_t size = lenght - PAYLOAD_IDX;
    if (size > PAYLOAD_SIZE) {
        xDebug("Message Size is too long\n");
        return 0;
    }
    // Payload
    uint8_t payload[PAYLOAD_SIZE] = { 0 };
    if (_serial.read(&payload[0], size) != size) {
        xDebug("Error read payload\n");
        return 0;
    }
    // Checksum
    uint16_t checksum = 0;
    if (_serial.read(reinterpret_cast<uint8_t*>(&checksum), sizeof(uint16_t)) != 2) {
        xDebug("Error read checksum\n");
        return 0;
    }
    // Verify checksum
    uint16_t verify = HEAD_TYPE_MSG + data + static_cast<uint8_t>(lenght & 0xFF) + static_cast<uint8_t>(lenght >> 8);
    for (uint16_t i = 0; i < size; ++i) verify += payload[i];
    //xDebug("Checksum: %u Verify: %u\n", checksum, verify);
    if (checksum != verify) {
        xDebug("Checksum Error! Contain: %u Computed: %u", checksum, verify);
        return 0;
    }
    // Parse
    switch (id) {
    case MSG_DEVINFO:
        if (sizeof(DevInfo) != size) {
            xDebug("DevInfo wrong size: %u\n", size);
            return 0;
        }
        memcpy(&_info, &payload[0], size);
        break;
    case MSG_GETBIT:
        if (sizeof(_bit) != size) {
            xDebug("GetBit wrong size: %u\n", size);
            return 0;
        }
        memcpy(&_bit[0], &payload[0], size);
        break;
    case MSG_READ_PARAM:
        switch (_param) {
        case PRM_DATA_RATE:
            if (sizeof(uint16_t) != size) {
                xDebug("Data Rate wrong size: %u\n", size);
                return 0;
            }
            memcpy(&_dataRate, &payload[0], size);
            break;
        case PRM_BAUD_RATE:
            if (sizeof(uint8_t) != size) {
                xDebug("Baud Rate wrong size: %u\n", size);
                return 0;
            }
            memcpy(&_baudRate, &payload[0], size);
            break;
        case PRM_AUTO_START:
            if (sizeof(uint8_t) != size) {
                xDebug("Auto Start wrong size: %u\n", size);
                return 0;
            }
            memcpy(&_autoStart, &payload[0], size);
            break;
        case PRM_AVG_OUTDATA:
            if (sizeof(uint8_t) != size) {
                xDebug("Average Output Data wrong size: %u\n", size);
                return 0;
            }
            memcpy(&_avgOutData, &payload[0], size);
            break;
        case PRM_INIT_ATIME:
            if (sizeof(uint16_t) != size) {
                xDebug("Initial Alignment Time wrong size: %u\n", size);
                return 0;
            }
            memcpy(&_initAlignTime, &payload[0], size);
            break;
        case PRM_ALIGN_ANG:
            if (sizeof(float) * 3 != size) {
                xDebug("Alignment Angles wrong size: %u\n", size);
                return 0;
            }
            memcpy(&_alignAngle[0], &payload[0], size);
            break;
        }
        _param = 0;
        break;
    case MSG_SAVE_PARAM:
        if (sizeof(uint16_t) != size) {
            xDebug("Checksum wrong size: %u\n", size);
            return 0;
        }
        if (memcmp(&_checksum, &payload[0], size) != 0) {
            xDebug("Checksum is Wrong\n");
            return 0;
        }
        break;
    case MSG_ORIENTATION:
        if (sizeof(OrientationData) != size) {
            xDebug("Orientation Data wrong size: %u\n", size);
            return 0;
        }
        memcpy(&_orientation, &payload[0], size);
        break;
    case MSG_GA:
        if (sizeof(GAData) != size) {
            xDebug("GAData Data wrong size: %u\n", size);
            return 0;
        }
        memcpy(&_GA_data, &payload[0], size);
        break;        
    case MSG_GA_M:
        if (sizeof(GAmData) != size) {
            xDebug("GAmData Data wrong size: %u\n", size);
            return 0;
        }
        memcpy(&_GA_m_data, &payload[0], size);
        break;
    case MSG_GAA:
        if (sizeof(GAAData) != size) {
            xDebug("GAAData Data wrong size: %u\n", size);
            return 0;
        }
        memcpy(&_GAA_data, &payload[0], size);
        break;        
    case MSG_GAA_M:
        if (sizeof(GAAmData) != size) {
            xDebug("GAAmData Data wrong size: %u\n", size);
            return 0;
        }
        memcpy(&_GAA_m_data, &payload[0], size);
        break;        
    case MSG_QUAT:
        if (sizeof(QuaternionData) != size) {
            xDebug("QuaternionData Data wrong size: %u\n", size);
            return 0;
        }
        memcpy(&_quaternion, &payload[0], size);
        break;        
    case MSG_CALIB_HR:
        if (sizeof(CalibratedHRData) != size) {
            xDebug("CalibratedHRData Data wrong size: %u\n", size);
            return 0;
        }
        memcpy(&_hrdata, &payload[0], size);
        break;
    }
    return id;
}

bool il::IMUKernel::waitAvailable(const unsigned long timeout)
{
    unsigned long ms = millis();
    if (static_cast<unsigned long>(ms + timeout) == 0) ms = 0;
    while (_serial.available() == 0) {
        if (millis() - ms > timeout) return false;
        yield();
    }
    return true;
}
