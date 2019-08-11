#include "MechanumDriver.hpp"

const uint32_t MechanumDriver::kPwmFrequency = 1000;

const ledc_timer_t MechanumDriver::kUseTimer[] = {LEDC_TIMER_0, LEDC_TIMER_1};

const MotorIo MechanumDriver::kMotorIo[] = {
    {kUseTimer[kBack], LEDC_CHANNEL_2, GPIO_NUM_16, GPIO_NUM_17, 1},
    {kUseTimer[kBack], LEDC_CHANNEL_3, GPIO_NUM_14, GPIO_NUM_27, 0},
    {kUseTimer[kFront], LEDC_CHANNEL_0, GPIO_NUM_22, GPIO_NUM_23, 1},
    {kUseTimer[kFront], LEDC_CHANNEL_1, GPIO_NUM_18, GPIO_NUM_19, 0},
};

const uint32_t MechanumDriver::kPwmSpeedBase = 0xcfff;
const uint32_t MechanumDriver::kPwmSpeedRange = 0x3000;

MechanumDriver::MechanumDriver() : is_initialized_(false) {

}

esp_err_t MechanumDriver::Initialize() {
    esp_err_t ret = ESP_OK;

    ret = InitPwmTimer();

    if (ret == ESP_OK) {
        ret = InitMotorIo();
    }

    is_initialized_ = ret == ESP_OK;

    return ret;
}

esp_err_t MechanumDriver::Drive(float fb, float lr, float ro) {
    esp_err_t ret = ESP_OK;

    fb = SetValueInRange(fb);
    lr = SetValueInRange(lr);
    ro = SetValueInRange(ro);

    for (int i = 0; i < kMotorCount; i++) {
        SetMotorSpeed(i, fb);
    }

    return ret;
}

esp_err_t MechanumDriver::Stop() {
    esp_err_t ret = ESP_OK;

    for (int i = 0; i < kMotorCount; i++) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, kMotorIo[i].pwm_channel, 0); 
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, kMotorIo[i].pwm_channel);
    }

    return ret;
}

esp_err_t MechanumDriver::InitPwmTimer() {
    esp_err_t ret = ESP_OK;

    ledc_timer_config_t timer_config;
    timer_config.duty_resolution = LEDC_TIMER_16_BIT;
    timer_config.freq_hz = kPwmFrequency;
    timer_config.speed_mode = LEDC_HIGH_SPEED_MODE;

    for (int i = 0; i < kTimerCount && ret == ESP_OK; i++) {
        timer_config.timer_num = kUseTimer[i];
        ret = ledc_timer_config(&timer_config);
    }

    return ret;
}

esp_err_t MechanumDriver::InitMotorIo() {
    esp_err_t ret = ESP_OK;

    ledc_channel_config_t pwm_config;
    pwm_config.duty = 0;
    pwm_config.hpoint = 0;
    pwm_config.speed_mode = LEDC_HIGH_SPEED_MODE;
    pwm_config.intr_type = LEDC_INTR_DISABLE;

    gpio_config_t dir_gpio_config;
    dir_gpio_config .mode = GPIO_MODE_OUTPUT;
    dir_gpio_config.pin_bit_mask = 0;
    dir_gpio_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    dir_gpio_config.pull_up_en = GPIO_PULLUP_ENABLE;
    dir_gpio_config.intr_type = GPIO_INTR_DISABLE;

    for (int i = 0; i < kMotorCount && ret == ESP_OK; i++) {
        dir_gpio_config.pin_bit_mask |= (0x01 << kMotorIo[i].dir);

        pwm_config.channel = kMotorIo[i].pwm_channel;
        pwm_config.gpio_num = kMotorIo[i].pwm;
        pwm_config.timer_sel = kMotorIo[i].pwm_timer;

        ret = ledc_channel_config(&pwm_config);
    }

    if (ret == ESP_OK) {
       ret = gpio_config(&dir_gpio_config);
    }

    return ret;
}

esp_err_t MechanumDriver::SetMotorSpeed(int id, float speed) {
    esp_err_t ret = ESP_OK;

    speed = SetValueInRange(speed);

    if (speed < 0) {
        speed *= -1;
        ret = gpio_set_level(kMotorIo[id].dir, !kMotorIo[id].fore_level);
    } else {
        ret = gpio_set_level(kMotorIo[id].dir, kMotorIo[id].fore_level);
    }

    if (ret != ESP_OK) return ret;

    
    if (speed < 0.001) {
        ret = ledc_set_duty(LEDC_HIGH_SPEED_MODE, kMotorIo[id].pwm_channel, 0);
    } else {
        uint32_t duty = kPwmSpeedBase + kPwmSpeedRange * speed;

        ret = ledc_set_duty(LEDC_HIGH_SPEED_MODE, kMotorIo[id].pwm_channel, duty);
    }

    ledc_update_duty(LEDC_HIGH_SPEED_MODE, kMotorIo[id].pwm_channel);

    return ret;
}