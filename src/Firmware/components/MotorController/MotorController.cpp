

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "MotorController.hpp"

const uint8_t MotorController::DriverId = 0xA9;
const uint8_t MotorController::SpeedLevelMax = 255;
const uint32_t MotorController::I2cClockHz = 100000;
const i2c_port_t MotorController::I2cPortNum = I2C_NUM_0;

bool MotorController::is_i2c_initialized_ = false;

MotorController::MotorController(uint8_t addr) : addr_ (addr) {

}

esp_err_t MotorController::initialize() {
    esp_err_t ret = ESP_OK;

    if (!is_i2c_initialized_) {
        ret = initI2c();
    }

    if (ret == ESP_OK) {
        while ((ret = initDriver()) == ESP_ERR_INVALID_RESPONSE) {
            vTaskDelay(100/portTICK_RATE_MS);
        }
    }

    while (!isReady());

    while (isBusy());

    //setEnable(true);

    return ret;
}

bool MotorController::isReady() {
    esp_err_t ret;
    uint8_t state;

    ret = readRegister(RegStatus1, &state);

    return ret == ESP_OK && (state & 0x01) == 0x01;
}

bool MotorController::isBusy() {
    esp_err_t ret;
    uint8_t state;

    ret = readRegister(RegStatus1, &state);

    return ret == ESP_OK && (state & (Status1BitBusy | Status1BitRemRead | Status1BitRemWrite)) != 0;
}

esp_err_t MotorController::setEnable(bool is_enable) {
    return writeRegister(RegDriverEnable, is_enable ? 0x01 : 0x00);
}

esp_err_t MotorController::drive(uint8_t motor, EMotorDir dir, uint8_t level) {
    if (motor >= 34) return ESP_ERR_INVALID_ARG;
    level = (uint8_t)(0x80 + (dir == DirFore ? 1 : -1) * (level >> 1));
    return writeRegister(RegMaDrive + motor, level);
}

esp_err_t MotorController::initDriver() {
    if (!is_i2c_initialized_) return ESP_ERR_INVALID_STATE;

    uint8_t id;
    esp_err_t ret = readRegister(RegId, &id);

    if (ret == ESP_OK && id != DriverId) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }

    return ret;
}

esp_err_t MotorController::setInversion(uint8_t motor, uint8_t is_invert) {
    uint8_t reg;
    uint8_t current;
    esp_err_t ret = ESP_OK;

    is_invert = is_invert > 0 ? 0x01 : 0x00;

    if (motor < 2) {
        if (motor == 0) {
            ret = writeRegister(RegMotorAInvert, is_invert);
        } else {
            ret = writeRegister(RegMotorBInvert, is_invert);
        }
    } else if (motor >= 34) {
        ret = ESP_ERR_INVALID_ARG;
    } else {
        if (motor < 10) {
            reg = RegInv29;
            motor -= 2;
        } else if (motor < 18) {
            reg = RegInv1017;
            motor -= 10;
        } else if (motor < 26) {
            reg = RegInv1825;
            motor -= 18;
        } else {
            reg = RegInv2633;
            motor -= 26;
        }

        ret = readRegister(reg, &current);

        if (ret == ESP_OK) {
            ret = writeRegister(reg, (current & ~(0x01 << motor)) | (is_invert << motor));
        }
    }

    return ret;
}

esp_err_t MotorController::initI2c() {
    if (is_i2c_initialized_) return ESP_OK;

    esp_err_t ret = ESP_OK;

    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = GPIO_NUM_22;
    config.sda_pullup_en = GPIO_PULLUP_DISABLE;
    config.scl_io_num = GPIO_NUM_21;
    config.scl_pullup_en = GPIO_PULLUP_DISABLE;
    config.master.clk_speed = I2cClockHz;

    ret = i2c_param_config(I2cPortNum, &config);

    if (ret == ESP_OK) {
        ret = i2c_driver_install(I2cPortNum, I2C_MODE_MASTER, 0, 0, 0);
    }

    return ret;
}

esp_err_t MotorController::writeRegister(uint8_t offset, uint8_t value) {
    esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ret = i2c_master_start(cmd);

    if (ret == ESP_OK) {
        ret = i2c_master_write_byte(cmd, (addr_ << 1) | I2C_MASTER_WRITE, true);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_write_byte(cmd, offset, true);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_write_byte(cmd, value, true);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_stop(cmd);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_cmd_begin(I2cPortNum, cmd, 1000 / portTICK_RATE_MS);
    }

    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t MotorController::readRegister(uint8_t offset, uint8_t *value) {
    esp_err_t ret = ESP_OK;

    if (value == NULL) return ESP_ERR_INVALID_ARG;

    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();

    ret = i2c_master_start(cmd);

    if (ret == ESP_OK) {
        ret = i2c_master_write_byte(cmd, (addr_ << 1) | I2C_MASTER_WRITE, true);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_write_byte(cmd, offset, true);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_stop(cmd);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_cmd_begin(I2cPortNum, cmd, 1000 / portTICK_RATE_MS);
    }

    if (ret == ESP_OK) {
        i2c_cmd_link_delete(cmd);

        cmd = i2c_cmd_link_create();
    }

    if (ret == ESP_OK) {
        ret = i2c_master_start(cmd);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_write_byte(cmd, (addr_ << 1) | I2C_MASTER_READ, true);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_stop(cmd);
    }

    if (ret == ESP_OK) {
        ret = i2c_master_cmd_begin(I2cPortNum, cmd, 1000 / portTICK_RATE_MS);
    }

    i2c_cmd_link_delete(cmd);

    return ret;
}

