#ifndef MODBUS_CONTROL_H
#define MODBUS_CONTROL_H

#include "modbus.h"
#include <stdint.h>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <atomic>
#include <chrono>


class ModbusControl {
public:
    // Function pointer types for connection methods
    using ConnectFunc = std::function<bool(modbus_t*)>;
    using DisconnectFunc = std::function<void(modbus_t*)>;

    // Constructors and destructor
    ModbusControl(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave);
    ModbusControl(const char* device, int baud, char parity, int data_bit, int stop_bit, int slave, uint32_t to_sec, uint32_t to_usec);
    ModbusControl(const char* ip, int port, int slave);
    ModbusControl(const char* ip, int port, int slave, uint32_t to_sec, uint32_t to_usec);
    ModbusControl(const std::string& ip_address, int port, int slave_address = 1);
    ~ModbusControl();

    // Connection management
    bool connect();
    void disconnect();
    bool isConnected() const;

    // Modbus operations
    bool readCoils(int addr, int nb, uint8_t* dest);
    bool writeCoil(int addr, uint8_t value);
    bool writeMultipleCoils(int addr, int nb, const uint8_t* values);
    bool readDiscreteInputs(int addr, int nb, uint8_t* values);
    bool readHoldingRegisters(int addr, int nb, uint16_t* dest);
    bool readInputRegisters(int addr, int nb, uint16_t* dest);
    bool readInputRegisters(int addr, int nb, uint16_t* dest, double& time_buffer, double& duration_buffer);
    bool writeRegister(int addr, uint16_t value);
    bool writeRegisters(int addr, int nb, const uint16_t* values);
    bool readHoldingRegisters(int addr, int nb, uint16_t* dest, double& time_buffer, double& duration_buffer);
    bool writeHoldingRegisters(int addr, int nb, const uint16_t* data);
    void setDisableCerr(bool disable) { disable_cerr = disable; }
       // Common utility methods for devices
    uint16_t readDiscreteInputsAsWord(int startAddr, int count);
    bool writeCoilsFromWord(int startAddr, int count, uint16_t value);
    bool readHoldingRegistersAsInt32(int addr, int32_t& value);
    bool writeInt32AsHoldingRegisters(int addr, int32_t value);
    bool readHoldingRegistersAsDouble(int addr, double& value);
    bool readDiscreteInput(int addr);
    
    // Common data combination utilities
    static int32_t combineTwoInt16ToInt32(int16_t high_bits, int16_t low_bits);
    static uint32_t combineTwoUint16ToUint32(uint16_t high_bits, uint16_t low_bits);
    static float combineTwoInt16ToFloat32(int16_t high_bits, int16_t low_bits);
    static float combineTwoUint16ToFloat32(uint16_t high_bits, uint16_t low_bits);
    static double combineTwoInt16ToDouble(int16_t high_bits, int16_t low_bits);

    // Common run control methods
    virtual void run_once() {
        // Default implementation: do nothing
    }
    void run();
    void stop();
    
    // Virtual method to get run loop sleep time in milliseconds
    virtual int getRunLoopSleepMs() const {
        return 10; // Default 100Hz
    }

protected:
    std::thread run_thread_;
    bool running_thread_{false};
    
    // Thread management helper
    std::mutex data_mutex_;

private:
    // Helper methods
    void setupRTUFunctions();
    void setupTCPFunctions();
    bool ensureConnection();
    void handleError(const char* context);
    void handleError(const std::string& message);

    // Member variables
    modbus_t* ctx = nullptr;
    bool connected = false;
    std::mutex mutex_;
    ConnectFunc connectFunc;
    DisconnectFunc disconnectFunc;
    std::string ip_address_;
    int port_;
    int slave_address_;
    bool disable_cerr = false;
};

#endif // MODBUS_CONTROL_H