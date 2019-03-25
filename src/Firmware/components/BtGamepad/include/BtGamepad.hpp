#ifndef __BT_GAMEPAD_HPP__
#define __BT_GAMEPAD_HPP__

#include <inttypes.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include "btstack_config.h"
#include "btstack.h"

typedef enum {
    PadConnectionFailed = -1,
    PadDisconnected = 0,
    PadConnected = 1,
} EBtGamepadConnection_t;

typedef enum {
    HatUp = 1,
    HatUpRight = 2,
    HatRight = 3,
    HatDownRight = 4,
    HatDown = 5,
    HatDownLeft = 6,
    HatLeft = 7,
    HatUpLeft = 8,
    HatNone = 0x0f,
} EGamepadHat_t;

typedef enum {
    ButtonB = 0x001,
    ButtonA = 0x002,
    ButtonY = 0x004,
    ButtonX = 0x008,
    ButtonL1 = 0x010,
    ButtonR1 = 0x020,
    Select = 0x040,
    Start = 0x080,
    ButtonL2 = 0x100,
    ButtonR2 = 0x200,
} EGamepadButtons_t;

typedef enum {
    AnalogLH = 0,
    AnalogLV,
    AnalogRH,
    AnalogRV,
    AnalogCount,
} EGamepadStick_t;

typedef void (*GamepadConnectionCallback_t)(EBtGamepadConnection_t);
typedef void (*GamepadUpdateStateCallback_t)(EGamepadHat_t, EGamepadButtons_t, int8_t*);


class BtGamepad {
public:
    static BtGamepad* getInstance();
    void initialize();
    void registerCallbacks(GamepadConnectionCallback_t connection_callback, GamepadUpdateStateCallback_t update_state_callback);

private:
    BtGamepad();
};

#endif //__BT_GAMEPAD_HPP__
