#ifndef __MECHANUM_DRIVER_HPP__
#define __MECHANUM_DRIVER_HPP__

#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

struct MotorIo {
    ledc_timer_t    pwm_timer;
    ledc_channel_t  pwm_channel;
    gpio_num_t      pwm;
    gpio_num_t      dir;
    uint32_t        fore_level;
};

class MechanumDriver {
public:
    MechanumDriver();

    esp_err_t Initialize();
    esp_err_t Drive(float fb, float lr, float ro);
    esp_err_t Stop();
    bool IsInitialized() { return is_initialized_; }

private:
    enum MotorInfo{
        kFrontLeft,
        kFrontRight,
        kBackLeft,
        kBackRignt,
        kMotorCount
    };

    enum UseTimerInfo {
        kFront,
        kBack,
        kTimerCount,
    };

    static const uint32_t kPwmFrequency;
    static const ledc_timer_t kUseTimer[kTimerCount];
    static const MotorIo kMotorIo[kMotorCount];
    static const uint32_t kPwmSpeedBase;
    static const uint32_t kPwmSpeedRange;

    esp_err_t InitPwmTimer();
    esp_err_t InitMotorIo();
    esp_err_t SetMotorSpeed(int id, float speed);

    bool is_initialized_;
};

inline float SetValueInRange(float src) {
    if (src < -1.0f) return -1.0f;
    else if (src > 1.0f) return 1.0f;
    return src;
}

#endif //__MECHANUM_DRIVER_HPP__
