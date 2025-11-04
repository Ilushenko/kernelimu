#ifndef __IMU_KERNEL_H__
#define __IMU_KERNEL_H__

#include "IMUData.h"
#include "HardwareSerial.h"

/// \defgroup ildriver Driver
/// \brief KERNEL IMU driver
/// \details Classes and functions provide working Inertial Labs KERNEL Inertial Measurement Unit (IMU)

namespace il {
    /// \brief Callback IMU Data function
    /// \details User defined function for handle IMU Data
    /// \param id IMU Data identifier
    /// \param data IMU Data pointer
    /// \ingroup ildriver
    typedef void(*callbackData)(const uint8_t id, const void* data);
    /// \class il::IMUKernel IMUKernel.h
    /// \brief Kernel IMU driver
    /// \details Provide read and save to flash IMU parameters
    /// \details Provide output data formats are available during IMU operation
    /// \ingroup ildriver
    class IMUKernel {
        IMUKernel() = delete;
        IMUKernel(const IMUKernel&) = delete;
        IMUKernel& operator = (const IMUKernel&) = delete;
    public:
        /// \brief Constructor
        /// \param serial Serial interface reference
        explicit IMUKernel(HardwareSerial& serial);
        /// \brief Destructor
        ~IMUKernel();
    public:
        /// \brief Start working IMU
        /// \details Read IMU parameters
        /// \details Call this method at the beginning of the program
        /// \return Result of read parameters
        bool begin();
        /// \brief Stop working IMU
        /// \details Call this method at the end of the program
        /// \return Result of execute stop IMU commad
        bool stop();
        /// \brief Update data of IMU
        /// \details To be called in the main program loop
        void update();
        /// \brief Start providing output data
        /// \details Start providing new output data - automatic stopping preview output data providing
        /// \details Call this method at the beginning of the program after IMUKernel::begin and setting parameters
        /// \param cmd See \b Kernel \b Commands enumerator
        /// \return Commad execution result
        bool provide(const uint8_t cmd);
    public:
        /// @{
        /// \name Settings
        /// \brief Setters of KERNEL parameters

        /// \brief Set callback function for handling IMU Data
        /// \param func Function pointer
        inline void setCallback(callbackData func) { _callback = func; }
        /// \brief Save output data rate
        /// \param rate Output data rate in Hz
        /// \return Save flash result
        bool setDataRate(uint16_t rate);
        /// \brief Save baud rate
        /// \param rate Baud rate ID (see \b Kernel \b Baud \b Rate enumerator)
        /// \return Save flash result
        bool setBaudRate(uint8_t rate);
        /// \brief Save auto start output data format
        /// \param id Output data format. \с 0x00 - Disabled, \с 0xXX - see \b Kernel \b Commands enumerator
        /// \return Save flash result
        bool setAutoStart(const uint8_t id);
        /// \brief Save average output data
        /// \param averaged \c true - Averaged, \c false - Instant
        /// \return Save flash result
        bool setAvgOutData(const bool averaged);
        /// \brief Save initial alignment time
        /// \param sec Time in seconds
        /// \return Save flash result
        bool setInitAlignTime(const uint16_t sec);
        /// \brief Save angles of IMU mounting on the carrier object
        /// \param yaw Heading (yaw) andgle in degrees
        /// \param pitch Pitch andgle in degrees
        /// \param roll Roll andgle in degrees
        /// \return Save flash result
        bool setAlignAngles(const float& yaw, const float& pitch, const float& roll);
        /// @}
    public:
        /// @{
        /// \name Parameters
        /// \brief Getters of KERNEL parameters

        /// \return Output data rate in Hz
        inline uint16_t dataRate() const { return _dataRate; }
        /// \return Baud rate ID (see \b Kernel \b Baud \b Rate enumerator)
        inline uint8_t baudRate() const { return _baudRate; }
        /// \return Auto start output data format. \с 0x00 - Disabled, \с 0xXX - see \b Kernel \b Commands enumerator
        inline uint8_t autoStart() const { return _autoStart; }
        /// \return Average output data: \c true - Averaged, \c false - Instant
        inline bool averageOutputData() const { return static_cast<bool>(_avgOutData); }
        /// \return Шnitial alignment time шт іусщтві
        inline uint16_t initialAlignmentTime() const { return _initAlignTime; }
        /// \return Alignment рeading (yaw) andgle in degrees
        inline float alignYaw() const { return _alignAngle[0]; }
        /// \return Alignment зitch andgle in degrees
        inline float alignPitch() const { return _alignAngle[1]; }
        /// \return Alignment кoll andgle in degrees
        inline float alignRoll() const { return _alignAngle[2]; }
        /// @}
    public:
        /// @{
        /// \name IMU Data
        /// \brief Getters of KERNEL IMU structures

        /// \return GetDevInfo Data
        inline const DevInfo& getDeviceInfo() const { return _info; }
        /// \return KERNEL Orientation Data
        inline const OrientationData& getOrientationData() const { return _orientation; }
        /// \return KERNEL GA Data
        inline const GAData& getGAData() const { return _GA_data; }
        /// \return KERNEL GAm Data
        inline const GAmData& getGAmData() const { return _GA_m_data; }
        /// \return KERNEL GAA Data
        inline const GAAData& getGAAData() const { return _GAA_data; }
        /// \return KERNEL GAAm Data
        inline const GAAmData& getGAAmData() const { return _GAA_m_data; }
        /// \return KERNEL Quaternion Data
        inline const QuaternionData& getQuaternionData() const { return _quaternion; }
        /// \return KERNEL Calibrated HR Data
        inline const CalibratedHRData& getCalibratedHRData() const { return _hrdata; }
        /// @}
    private:
        /// \brief Сommands have a byte structure
        /// \param cmd Сode identifying the command (see \b Kernel \b Commands enumerator)
        /// \return Commad execution result
        inline bool setCommand(const uint8_t cmd) { return setCommand(&cmd, 1); }
        /// \brief Сommands have a byte structure
        /// \param payload Сode identifying the command or \b SaveFlash or \b ReadRAM command payload structure
        /// \param size Payload size
        /// \return Commad execution result
        bool setCommand(const uint8_t* payload, uint16_t size);
        /// \brief Сommands have a byte structure
        /// \param cmd See in \b Read/Save \b Parameter \b Commands enumerator
        /// \param param See in \b Kernel \b Device \b Parameters enumerator
        /// \param size Read/Save value size
        /// \param value Param value (\c nullptr for read command)
        /// \return Commad execution result
        bool setCommand(const uint16_t cmd, const uint16_t param, uint16_t size, const uint8_t* value = nullptr);
        /// \brief Read data from serial and parse it
        /// \details Handle output formats, read device parameters and save parameter to flash confirmation
        /// \return Сode identifying the output format (see \b Kernel \b Commands enumerator)
        uint8_t readSerialData();
        /// \brief Wait for serial is available
        /// \param timeout Timeout in milliseconds
        /// \return \c true - is serial available or \c false of timeout
        bool waitAvailable(const unsigned long timeout);
    private:
        uint8_t _currentCmd{ 0 };       // Current Message in Providing
        uint16_t _checksum{ 0 };        // Last command checksum
        uint16_t _param{ 0 };           // Requested Param Id
        uint16_t _dataRate{ 0 };        // Output data rate in Hertz (Hz)
        uint8_t _baudRate{ 0 };         // Baud rate
        uint8_t _autoStart{ 0 };        // Auto Start: 0x00 - Disabled, 0xXX - Output data format.
        uint8_t _avgOutData{ 0 };       // Average output data: 0x00 - Instant (default), 0x01 - Averaged (for less than 2000 Hz)
        uint16_t _initAlignTime{ 0 };   // Initial alignment time in seconds.
        float _alignAngle[3]{ 0 };      // Angles of IMU mounting on the carrier object in degrees. [0] - heading (yaw), [1] - pitch, [2] - roll
    private:
        uint8_t _bit[4]{ 0 };               // Payload of the KERNEL response to the GetBIT command
        DevInfo _info{ 0 };                 // Payload of the KERNEL response to the GetDevInfo command
        OrientationData _orientation{ 0 };  // Payload at KERNEL Orientation Data
        GAData _GA_data{ 0 };               // Payload at KERNEL GA Data
        GAmData _GA_m_data{ 0 };            // Payload at KERNEL GAm Data
        GAAData _GAA_data{ 0 };             // Payload at KERNEL GAA Data
        GAAmData _GAA_m_data{ 0 };          // Payload at KERNEL GAAm Data
        QuaternionData _quaternion{ 0 };    // Payload at KERNEL Quaternion Data
        CalibratedHRData _hrdata{ 0 };      // Payload at KERNEL Calibrated HR Data
    private:
        callbackData _callback{ nullptr };
        HardwareSerial& _serial;
    };
}

#endif // __IMU_KERNEL_H__