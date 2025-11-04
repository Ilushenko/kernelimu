#ifndef __IMU_DATA_H__
#define __IMU_DATA_H__

#include <cstdint>

/// \defgroup ildata Inertial Labs KERNEL IMU Protocol
/// \brief KERNEL IMU Protocol: enumerations and structures
/// \details Structures and enumerators that ensure the operation of KERNEL Inertial Measurement Unit (IMU)
/// \anchor strualign
/// \note All structures must defined with 1-byte alignment
/// \li Use \c __attribute__((__packed__)) by \b GCC
/// \li Use \c \#pragma \c pack(push,1) and \c \#pragma \c pack(pop) directives by \b Win32

/// \namespace il KERNEL IMU Protocol
/// \brief Provides working KERNEL IMU driver
/// \details Classes, structures and enumerators that ensure the operation of the KERNEL IMU driver
namespace il {
    /// \brief Header
    /// \details First 3 bytes of all commands and messages to/from the Inertial Labs KERNEL IMU
    /// \ingroup ildata
    enum {
        HEAD0           = 0xAA, ///< Header 0
        HEAD1           = 0x55, ///< Header 1
        HEAD_TYPE_CMD   = 0x00, ///< Message type: command
        HEAD_TYPE_MSG   = 0x01  ///< Message type: transferring data
    };
    /// \brief Kernel Messages (1 Byte ID)
    /// \details KERNEL control commands and appropriate output data format
    /// \ingroup ildata
    enum {
        MSG_ORIENTATION     = 0x33, ///< Orientation Data
        MSG_GA              = 0x8F, ///< GA Data
        MSG_GA_M            = 0x9B, ///< GAm Data
        MSG_GAA             = 0xA5, ///< GAA Data
        MSG_GAA_M           = 0xA6, ///< GAAm Data
        MSG_QUAT            = 0x82, ///< Quaternion Data
        MSG_CALIB_HR        = 0x81, ///< Calibrated High Resolution Data
        MSG_STOP            = 0xFE, ///< Stop Command
        MSG_USRDEF_DATA     = 0x95, ///< User Defined Data Commands
        MSG_USRDEF_CONFIG   = 0x96, ///< User Defined Configuration
        MSG_USRDEF_STRUCT   = 0x97, ///< User Defined Structure
        MSG_DEVINFO         = 0x12, ///< GetDevInfo Command
        MSG_GETBIT          = 0x1A, ///< GetBIT Command
        MSG_READ_PARAM      = 0xB1, ///< ReadRAM First bytye Command
        MSG_SAVE_PARAM      = 0xB2  ///< SaveFlash First bytye Command
    };
    /// \brief Read/Save Parameter Commands (2 Byte ID)
    /// \details KERNEL commands for read and save device parameters
    /// \ingroup ildata
    enum {
        RAM_READ        = 0xB1 | (0xFF << 8),   ///< Read RAM Command
        RAM_SAVE        = 0xB2 | (0xFF << 8)    ///< Save Flash Command
    };
    /// \brief Kernel Device Parameters (2 Bytes ID)
    /// \details Available parameters for read and change
    /// \ingroup ildata
    enum {
        PRM_DATA_RATE   = 0x12,                 ///< Data rate. Word (2 byte). Output data rate in Hertz (Hz)
        PRM_INIT_ATIME  = 0x14,                 ///< Initial alignment time, Word (2 byte). The initial alignment time in seconds.
        PRM_ALIGN_ANG   = 0x3A | (0x02 << 8),   ///< Alignment angles. 3*float (12 byte). Angles of IMU mounting on the carrier object in degrees.
        PRM_BAUD_RATE   = 0xB2 | (0x03 << 8),   ///< Baud rate. 1 byte. See \c Kernel \c Baud \c Rate enumerator
        PRM_AUTO_START  = 0xA9 | (0x03 << 8),   ///< Auto Start. 1 byte. \c 0x00 - Disabled (default), or code of output data format (See \c Kernel \c Commands enumerator)
        PRM_AVG_OUTDATA = 0xE1 | (0x03 << 8)    ///< Average output data. 1 byte. The output of averaged data: \c 0x00 - Instant (default), \c 0x01 - Averaged
    };
    /// \brief Kernel Baud Rate
    /// \details Baud rate ID
    /// \note The \c 1843200, \c 3686400, and \c 4000000 baud rate values were designed for specific purposes only, and a particular adaptor for data/commands transmission is required. Do not apply such values
    /// \ingroup ildata
    enum {
        BPS_DEFAULT = 0x00, ///< Default: 115200 bps
        BPS_4800    = 0x01, ///< 4800 bps
        BPS_9600    = 0x02, ///< 9600 bps
        BPS_14400   = 0x03, ///< 14400 bps
        BPS_19200   = 0x04, ///< 19200 bps
        BPS_38400   = 0x05, ///< 38400 bps
        BPS_57600   = 0x06, ///< 57600 bps
        BPS_115200  = 0x07, ///< 115200 bps
        BPS_230400  = 0x08, ///< 230400 bps
        BPS_460800  = 0x09, ///< 460800 bps
        BPS_921600  = 0x0A, ///< 921600 bps
        BPS_2000000 = 0x0B, ///< 2000000 bps
        BPS_375000  = 0x0C, ///< 375000 bps
        BPS_1843200 = 0x0D, ///< 1843200 bps (for specific purposes only)
        BPS_3686400 = 0x0E, ///< 3686400 bps (for specific purposes only)
        BPS_1000000 = 0x0F, ///< 1000000 bps
        BPS_4000000 = 0x10  ///< 4000000 bps (for specific purposes only)
    };
    /// \brief KG and KA constants
    /// \details Scale factors for KERNEL Orientation Data for 2000 Hz
    /// \ingroup ildata
    enum {
        KG = 10,    ///< Gyro range, deg/sec
        KA = 4000   ///< Accelerometer range, g
    };
    /// \struct  il::DevInfo IMUData.h
    /// \brief GetDevInfo Data
    /// \details GetDevInfo command response
    /// \ingroup ildata
    struct __attribute__((__packed__)) DevInfo {
        char idSN[8];           ///< IMU device (KERNEL) s/n
        char idFW[40];          ///< IMU (KERNEL) firmware version
        uint8_t reserved1;      ///< Reserved
        uint8_t imuType;        ///< IMU type
        char imuSN[8];          ///< IMU device (KERNEL) s/n (duplicated)
        char imuFW[40];         ///< IMU (KERNEL) firmware version (duplicated)
        uint8_t reserved2[68];  ///< Reserved
    };
    /// \struct il::OrientationData IMUData.h
    /// \brief KERNEL Orientation Data
    /// \details Provides data output from the KERNEL IMU in the form of:
    /// \li 3 orientation angles (heading, pitch, and roll);
    /// \li calibrated outputs of the 6 sensors (gyros and accelerometers) that give information about current angular rates, and linear accelerations of the IMU.
    /// \details \ref strualign "Structure alignment"
    /// \ingroup ildata
    struct __attribute__((__packed__)) OrientationData {
        uint16_t heading;       ///< Heading (deg * 100)
        int16_t pitch;          ///< Pitch (deg * 100)
        int16_t roll;           ///< Roll (deg * 100)
        int16_t gyroX;          ///< Angular rate X (deg/s * il::KG)
        int16_t gyroY;          ///< Angular rate Y (deg/s * il::KG)
        int16_t gyroZ;          ///< Angular rate Z (deg/s * il::KG)
        int16_t accX;           ///< Accelerations X (g * il::KA)
        int16_t accY;           ///< Accelerations Y (g * il::KA)
        int16_t accZ;           ///< Accelerations Z (g * il::KA)
        int16_t magX;           ///< 0 for the IMU (not measured)
        int16_t magY;           ///< 0 for the IMU (not measured)
        int16_t magZ;           ///< 0 for the IMU (not measured)
        uint32_t reserved1;     ///< Reserved
        uint16_t usw;           ///< Unit Status Word
        uint16_t reserved2;     ///< Reserved
        int16_t temperature;    ///< Temperature, сelsius * 10
    };
    /// \struct il::GAData IMUData.h
    /// \brief KERNEL GA Data
    /// \details Provides the IMU calibrated output of gyros and accelerometers with high resolution
    /// \details Contains information about current angular rates and linear accelerations of the KERNEL IMU
    /// \details \ref strualign "Structure alignment"
    /// \ingroup ildata
    struct __attribute__((__packed__)) GAData {
        int32_t gyroX;          ///< Angular rate X (deg/s * 1.0e5)
        int32_t gyroY;          ///< Angular rate Y (deg/s * 1.0e5)
        int32_t gyroZ;          ///< Angular rate Z (deg/s * 1.0e5)
        int32_t accX;           ///< Accelerations X (g * 1.0e6)
        int32_t accY;           ///< Accelerations Y (g * 1.0e6)
        int32_t accZ;           ///< Accelerations Z (g * 1.0e6)
        uint16_t counter;       ///< Frequency of the internal computations of the device
        uint16_t usw;           ///< Unit Status Word
        uint16_t reserved;      ///< Reserved
        int16_t temperature;    ///< Temperature, сelsius * 10
    };
    /// \struct il::GAmData IMUData.h
    /// \brief KERNEL GAm Data
    /// \details Provides the KERNEL IMU calibrated output of gyros and accelerometers with high resolution
    /// \details contains information about current angular rates and linear accelerations of the IMU
    /// \details \ref strualign "Structure alignment"
    /// \ingroup ildata
    struct __attribute__((__packed__)) GAmData {
        int32_t gyroX;          ///< Angular rate X (deg/s * 1.0e5)
        int32_t gyroY;          ///< Angular rate Y (deg/s * 1.0e5)
        int32_t gyroZ;          ///< Angular rate Z (deg/s * 1.0e5)
        int32_t accX;           ///< Accelerations X (g * 1.0e6)
        int32_t accY;           ///< Accelerations Y (g * 1.0e6)
        int32_t accZ;           ///< Accelerations Z (g * 1.0e6)
        uint16_t timeFlag;      ///< Time Flag
        uint32_t secondFract;   ///< Second fraction
        int16_t temperature;    ///< Temperature, сelsius * 10
    };
    /// \struct il::GAAData IMUData.h
    /// \brief KERNEL GAA Data
    /// \details Based on the KERNEL GA Data format
    /// \details The only difference is that to the message structure added data from two additional accelerometers
    /// \details \ref strualign "Structure alignment"
    /// \note This output data format is ONLY available for the KERNEL-120 and KERNEL-220 models
    /// \ingroup ildata
    struct __attribute__((__packed__)) GAAData {
        int32_t gyroX;          ///< Angular rate X (deg/s * 1.0e5)
        int32_t gyroY;          ///< Angular rate Y (deg/s * 1.0e5)
        int32_t gyroZ;          ///< Angular rate Z (deg/s * 1.0e5)
        int32_t accX1;          ///< Accelerations X1 (g * 1.0e6)
        int32_t accY1;          ///< Accelerations Y1 (g * 1.0e6)
        int32_t accZ1;          ///< Accelerations Z1 (g * 1.0e6)
        int32_t accX2;          ///< Accelerations X2 (g * 1.0e6)
        int32_t accY2;          ///< Accelerations Y2 (g * 1.0e6)
        int32_t accZ2;          ///< Accelerations Z2 (g * 1.0e6)
        uint16_t reserved1;     ///< Reserved
        uint16_t usw;           ///< Unit Status Word
        uint16_t reserved2;     ///< Reserved
        int16_t temperature;    ///< Temperature, сelsius * 10
    };
    /// \struct il::GAAmData IMUData.h
    /// \brief KERNEL GAAm Data
    /// \details Based on the KERNEL GAm Data format
    /// \details The only difference is that to the message structure added data from two additional accelerometers
    /// \details \ref strualign "Structure alignment"
    /// \note This output data format is ONLY available for the KERNEL-120 and KERNEL-220 models
    /// \ingroup ildata
    struct __attribute__((__packed__)) GAAmData {
        int32_t gyroX;          ///< Angular rate X (deg/s * 1.0e5)
        int32_t gyroY;          ///< Angular rate Y (deg/s * 1.0e5)
        int32_t gyroZ;          ///< Angular rate Z (deg/s * 1.0e5)
        int32_t accX1;          ///< Accelerations X1 (g * 1.0e6)
        int32_t accY1;          ///< Accelerations Y1 (g * 1.0e6)
        int32_t accZ1;          ///< Accelerations Z1 (g * 1.0e6)
        int32_t accX2;          ///< Accelerations X2 (g * 1.0e6)
        int32_t accY2;          ///< Accelerations Y2 (g * 1.0e6)
        int32_t accZ2;          ///< Accelerations Z2 (g * 1.0e6)        
        uint16_t timeFlag;      ///< Time Flag
        uint32_t secondFract;   ///< Second fraction        
        int16_t temperature;    ///< Temperature, сelsius * 10
    };
    /// \struct il::QuaternionData IMUData.h
    /// \brief KERNEL Quaternion Data
    /// \details provides data in the following format:
    /// \li Euler Orientation Angles
    /// \li Quaternion of Orientation
    /// \details \ref strualign "Structure alignment"
    /// \ingroup ildata
    struct __attribute__((__packed__)) QuaternionData {
        uint16_t heading;       ///< Heading (deg * 100)
        int16_t pitch;          ///< Pitch (deg * 100)
        int16_t roll;           ///< Roll (deg * 100)
        int16_t q[4];           ///< Quaternion of orientation * 10000
        uint8_t reserved1[14];  ///< Reserved
        uint16_t usw;           ///< Unit Status Word
        uint16_t reserved2;     ///< Reserved
        int16_t temperature;    ///< Temperature, сelsius * 10
    };
    /// \struct il::CalibratedHRData IMUData.h
    /// \brief KERNEL Calibrated HR Data
    /// \details Provides data from the KERNEL in the form of:
    /// \li 3 orientation angles (relative heading, pitch and roll) in high resolution
    /// \li calibrated outputs of the 6 sensors (gyros, accelerometers) that give information about current angular rates, and linear accelerations of the KERNEL in high resolution
    /// \li A counter
    /// \li USW
    /// \li Temperature
    /// \details \ref strualign "Structure alignment"
    /// \ingroup ildata
    struct __attribute__((__packed__)) CalibratedHRData {
        int32_t heading;        ///< Heading (deg * 1000)
        int32_t pitch;          ///< Pitch (deg * 1000)
        int32_t roll;           ///< Roll (deg * 1000)
        int32_t gyroX;          ///< Angular rate X (deg/s * 1.0e5)
        int32_t gyroY;          ///< Angular rate Y (deg/s * 1.0e5)
        int32_t gyroZ;          ///< Angular rate Z (deg/s * 1.0e5)
        int32_t accX;           ///< Accelerations X (g * 1.0e6)
        int32_t accY;           ///< Accelerations Y (g * 1.0e6)
        int32_t accZ;           ///< Accelerations Z (g * 1.0e6)
        int16_t magX;           ///< 0 for the IMU (not measured)
        int16_t magY;           ///< 0 for the IMU (not measured)
        int16_t magZ;           ///< 0 for the IMU (not measured)
        uint16_t counter;       ///< Frequency of the internal computations of the device
        uint16_t reserved1;     ///< Reserved
        uint16_t usw;           ///< Unit Status Word
        uint16_t reserved2;     ///< Reserved
        int16_t temperature;    ///< Temperature, сelsius * 10
    };
    /// \brief Calculate Max Data Rate
    /// \param baudrate COM port baud rate (bits/s)
    /// \param pkgid Id of Package Data (See \c Kernel \c Messages enumerator)
    /// \note The KERNEL can only output at data rates that are factors of \c 2000 (1, 2, 4, 5, 8, 10, 16, 20, 25, 40, 50, 80, 100, 125, 200, 250, 400, 500, 1000, 2000 Hz)
    /// \note Calculated maximum data rates must be rounded down to the nearest factor of \c 2000 that is listed above
    /// \return Maximum Output Data Rate
    /// \ingroup ildata
    inline uint16_t maxDataRate(const uint32_t baudrate, const uint8_t pkgid)
    {
        switch (pkgid) {
        case MSG_ORIENTATION:
            return static_cast<uint16_t>(static_cast<double>(baudrate) / (sizeof(OrientationData) * 11));
        case MSG_GA:
            return static_cast<uint16_t>(static_cast<double>(baudrate) / (sizeof(GAData) * 11));
        case MSG_GA_M:
            return static_cast<uint16_t>(static_cast<double>(baudrate) / (sizeof(GAmData) * 11));
        case MSG_GAA:
            return static_cast<uint16_t>(static_cast<double>(baudrate) / (sizeof(GAAData) * 11));
        case MSG_GAA_M:
            return static_cast<uint16_t>(static_cast<double>(baudrate) / (sizeof(GAAmData) * 11));
        case MSG_QUAT:
            return static_cast<uint16_t>(static_cast<double>(baudrate) / (sizeof(QuaternionData) * 11));
        case MSG_CALIB_HR:
            return static_cast<uint16_t>(static_cast<double>(baudrate) / (sizeof(CalibratedHRData) * 11));
        }
        return static_cast<uint16_t>(static_cast<double>(baudrate) / 572); // Biggest Providing size (52) * bits_per_byte (11)
    }
}

#endif // __IMU_DATA_H__