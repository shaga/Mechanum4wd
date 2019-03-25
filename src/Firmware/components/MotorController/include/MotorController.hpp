#ifndef __MOTOR_CONTROLLER_HPP__
#define __MOTOR_CONTROLLER_HPP__

#include "esp_err.h"
#include "driver/i2c.h"

class MotorController {
public:
    static const uint8_t DriverId;
    static const uint8_t SpeedLevelMax;

    typedef enum {
        DirFore = 0,
        DirBack = 1,
    } EMotorDir;

    static bool is_i2c_initialized_;

    MotorController(uint8_t addr);


    esp_err_t initialize();
    bool isReady();
    bool isBusy();

    esp_err_t setEnable(bool is_enable);
    esp_err_t drive(uint8_t motor, EMotorDir dir, uint8_t speed);
    esp_err_t setInversion(uint8_t motor, uint8_t is_invert);
    esp_err_t setBridging(uint8_t driver, uint8_t is_bridging);

private:
    static const i2c_port_t I2cPortNum;
    static const uint32_t I2cClockHz;

    enum {
        RegFid = 0x00,
        RegId = 0x01,
        RegSlaveAddr = 0x02,
        RegConfigBits = 0x03,
        RegUI2cRdErr = 0x04,
        RegUI2cWrErr = 0x05,
        RegUBufDumped = 0x06,
        RegEI2cRdErr = 0x07,
        RegEI2cWrErr = 0x08,
        RegLoopTime = 0x09,
        RegSlvPollCnt = 0x0A,
        RegSlvTopAddr = 0x0B,
        RegMstEErr = 0x0C,
        RegMstEStatus = 0x0D,
        RegFSafeFaults = 0x0E,
        RegRegOorCnt = 0x0F,
        RegRegRoWriteCnt = 0x10,
        RegGenTestWord = 0x11,
        RegMotorAInvert = 0x12,
        RegMotorBInvert = 0x13,
        RegBridge = 0x14,
        RegLocalMasterLock = 0x15,
        RegLocalUserLock = 0x16,
        RegMstEInFn = 0x17,
        RegUPortClkdivU = 0x18,
        RegUPortClkdivL = 0x19,
        RegUPortClkdivCtrl = 0x1A,
        RegEPortClkdivU = 0x1B,
        RegEPortClkdivL = 0x1C,
        RegEPortClkddivCtrl = 0x1D,
        RegUBusUartBaud = 0x1E,
        RegFsafeCtrl = 0x1F,
        RegMaDrive = 0x20,
        RegMbDrive = 0x21,
        RegS1aDrive = 0x22,
        RegS1bDrive = 0x23,
        RegS2aDrive = 0x24,
        RegS2bDrive = 0x25,
        RegS3aDrive = 0x26,
        RegS3bDrive = 0x27,
        RegS4aDrive = 0x28,
        RegS4bDrive = 0x29,
        RegS5aDrive = 0x2A,
        RegS5bDrive = 0x2B,
        RegS6aDrive = 0x2C,
        RegS6bDrive = 0x2D,
        RegS7aDrive = 0x2E,
        RegS7bDrive = 0x2F,
        RegS8aDrive = 0x30,
        RegS8bDrive = 0x31,
        RegS9aDrive = 0x32,
        RegS9bDrive = 0x33,
        RegS10aDrive = 0x34,
        RegS10bDrive = 0x35,
        RegS11aDrive = 0x36,
        RegS11bDrive = 0x37,
        RegS12aDrive = 0x38,
        RegS12bDrive = 0x39,
        RegS13aDrive = 0x3A,
        RegS13bDrive = 0x3B,
        RegS14aDrive = 0x3C,
        RegS14bDrive = 0x3D,
        RegS15aDrive = 0x3E,
        RegS15bDrive = 0x3F,
        RegS16aDrive = 0x40,
        RegS16bDrive = 0x41,
        RegInv29 = 0x50,
        RegInv1017 = 0x51,
        RegInv1825 = 0x52,
        RegInv2633 = 0x53,
        RegBridgeSlvL = 0x54,
        RegBridgeSlvH = 0x55,
        RegDriverEnable = 0x70,
        RegDriverUpdateRate = 0x71,
        RegForceUpdate = 0x72,
        RegEBusSpeed = 0x73,
        RegMasterLock = 0x74,
        RegUserLock = 0x75,
        RegFsafeTime = 0x76,
        RegStatus1 = 0x77,
        RegControl1 = 0x78,
        RegRemAddr = 0x79,
        RegRemOffset = 0x7A,
        RegRemDataWr = 0x7B,
        RegRemDataRd = 0x7C,
        RegRemWrite = 0x7D,
        RegRemRead = 0x7E,

        MasterLockKey = 0x9B,
        UserLockKey = 0x5C,
        FiremwareVersion = 0x05,
        PollAddress = 0x4A,
        MaxPollLimit = 0xC8,

        Status1BitEnumeration = 0x01,
        Status1BitBusy = 0x02,
        Status1BitRemRead = 0x04,
        Status1BitRemWrite = 0x08,

        Contro1BitFullReset = 0x01,
        Control1BitReEnumerate = 0x02,

        FsafeCtrlDriveKill = 0x01,
        FsafeCtrlRestart = 0x06,
        FsafeCtrlReboot = 0x02,
        FsafeCtrlReEnum = 0x04,
        FsafeCtrlCycleUser = 0x08,
        FsafeCtrlCycleExp = 0x10,

        MstMInRestart = 0x03,
        MstMInReboot = 0x01,
        MstMInReEnum = 0x02,
        MstMInCycleUser = 0x04,
        MstMInCycleExp = 0x08,
    };

    esp_err_t initI2c();
    esp_err_t initDriver();

    esp_err_t writeRegister(uint8_t offset, uint8_t value);
    esp_err_t readRegister(uint8_t offset, uint8_t *value);

    uint8_t addr_;

};


#endif //__MOTOR_CONTROLLER_HPP__
