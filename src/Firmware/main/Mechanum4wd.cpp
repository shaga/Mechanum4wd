/*
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#define __BTSTACK_FILE__ "hid_host_demo.c"

/*
 * hid_host_demo.c
 */

/* EXAMPLE_START(hid_host_demo): HID Host Demo
 *
 * @text This example implements an HID Host. For now, it connnects to a fixed device, queries the HID SDP
 * record and opens the HID Control + Interrupt channels
 */

#include <inttypes.h>
#include <stdio.h>
#include <math.h>

#include "btstack_config.h"
#include "btstack.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "MotorController.hpp"
#include "BtGamepad.hpp"

//#define DEBUG_ONLY_PAD

TaskHandle_t motor_control_task = NULL;

//static BtGamepad* pgamepad = nullptr;
static MotorController front_controller(0x59);
static MotorController back_controller(0x58);


static bool is_connected = false;

static bool is_updated = false;

static int fl = 0;
static int fr = 0;
static int bl = 0;
static int br = 0;

/* @section Main application configuration
 *
 * @text In the application configuration, L2CAP is initialized 
 */

extern "C" {
    int btstack_main(int argc, const char * argv[]);
}

static void updateMotorSpeed(void* params) {
#ifndef DEBUG_ONLY_PAD
    while (true) {
        if (is_updated) {
            is_updated = false;
            front_controller.drive(1, fl >= 0 ? MotorController::DirFore : MotorController::DirBack, (uint8_t)abs(fl));
            front_controller.drive(0, fr >= 0 ? MotorController::DirFore : MotorController::DirBack, (uint8_t)abs(fr));
            back_controller.drive(1, bl >= 0 ? MotorController::DirFore : MotorController::DirBack, (uint8_t)abs(bl));
            back_controller.drive(0, br >= 0 ? MotorController::DirFore : MotorController::DirBack, (uint8_t)abs(br));
            vTaskDelay(20 / portTICK_PERIOD_MS);
        } else {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
#endif
}

static void initLed() {
    esp_err_t ret = ESP_OK;

    ledc_timer_config_t config;
    config.duty_resolution = LEDC_TIMER_16_BIT;
    config.freq_hz = 1;
    config.speed_mode = LEDC_HIGH_SPEED_MODE;
    config.timer_num = LEDC_TIMER_0;

    ledc_timer_config(&config);

    ledc_channel_config_t channel_config;
    channel_config.channel = LEDC_CHANNEL_0;
    channel_config.duty = 0;
    channel_config.gpio_num = 17;
    channel_config.speed_mode = LEDC_HIGH_SPEED_MODE;
    channel_config.hpoint = 0;
    channel_config.timer_sel = LEDC_TIMER_0;

    ledc_channel_config(&channel_config);
}

static void setLedFreq(uint32_t freq) {
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, freq);
}

static void setLedDuty(uint8_t duty) {
    if (duty > 100) duty = 100;

    uint32_t value = 0xffff * duty / 100;

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, value);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

static void updateGamepadConnection(EBtGamepadConnection_t state) {
    if (state == EBtGamepadConnection_t::PadConnected) {
        setLedDuty(0);
        is_connected = true;
#ifndef DEBUG_ONLY_PAD
        xTaskCreate(updateMotorSpeed, "motor_control", 1024, NULL, tskIDLE_PRIORITY, &motor_control_task);
#endif
    } else {
        setLedDuty(100);
        is_connected = false;
#ifndef DEBUG_ONLY_PAD
        if (motor_control_task != NULL)
            vTaskDelete(motor_control_task);
#endif
        motor_control_task = NULL;
    }
}

static void updateGamepadState(EGamepadHat_t hat, EGamepadButtons_t buttons, int8_t* sticks) {
    //if (!is_connected) return;

    is_updated = true;

    fl = 0 + sticks[EGamepadStick_t::AnalogLV] * 2 - sticks[EGamepadStick_t::AnalogLH] * 2 - sticks[EGamepadStick_t::AnalogRH] * 2;
    fr = 0 - sticks[EGamepadStick_t::AnalogLV] * 2 - sticks[EGamepadStick_t::AnalogLH] * 2 - sticks[EGamepadStick_t::AnalogRH] * 2;
    bl = 0 + sticks[EGamepadStick_t::AnalogLV] * 2 + sticks[EGamepadStick_t::AnalogLH] * 2 - sticks[EGamepadStick_t::AnalogRH] * 2;
    br = 0 - sticks[EGamepadStick_t::AnalogLV] * 2 + sticks[EGamepadStick_t::AnalogLH] * 2 - sticks[EGamepadStick_t::AnalogRH] * 2;

    if (fl > 254) fl = 254;
    else if (fl < -254) fl = -254;
    if (fr > 254) fr = 254;
    else if (fr < -254) fr = -254;
    if (bl > 254) bl = 254;
    else if (bl < -254) bl = -254;
    if (br > 254) br = 254;
    else if (br < -254) br = -254;
#ifdef DEBUG_ONLY_PAD
    printf("LH:%d/LV:%d/RH:%d\n", sticks[EGamepadStick_t::AnalogLH], sticks[EGamepadStick_t::AnalogLV], sticks[EGamepadStick_t::AnalogRH]);
    printf("FL:%d/FR:%d/BL:%d/BR:%d\n", fl, fr, br, bl);
#endif
}

int btstack_main(int argc, const char * argv[]){

    (void)argc;
    (void)argv;

    
#ifndef DEBUG_ONLY_PAD
    initLed();

    setLedDuty(50);

    setLedDuty(90);

    front_controller.initialize();

    front_controller.setEnable(true);

    front_controller.drive(0, MotorController::DirFore, 0);
    front_controller.drive(1, MotorController::DirFore, 0);

    setLedDuty(50);

    back_controller.initialize();

    back_controller.setEnable(true);

    back_controller.drive(0, MotorController::DirFore, 0);
    back_controller.drive(1, MotorController::DirFore, 0);

    setLedDuty(10);
#endif

    BtGamepad* pad = BtGamepad::getInstance();
    pad->registerCallbacks(updateGamepadConnection, updateGamepadState);
    pad->initialize();

    return 0;
}

/* EXAMPLE_END */
