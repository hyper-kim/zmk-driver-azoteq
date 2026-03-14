/*
 * Copyright (c) 2020 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT azoteq_iqs5xx

#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include "iqs5xx.h"

static int iqs_regdump_err = 0;

// Default config
struct iqs5xx_reg_config iqs5xx_reg_config_default () {
    struct iqs5xx_reg_config regconf;
    regconf.activeRefreshRate =         2;    // 🔥 3에서 2로 단축 (빠른 스캔)
    regconf.idleRefreshRate =           10;   // 20에서 10으로 단축
    // 🔥 모든 제스처 개방
    regconf.singleFingerGestureMask =   0x00; 
    regconf.multiFingerGestureMask =    0x00; 
    regconf.tapTime =                   150;  
    regconf.tapDistance =               20;   
    regconf.touchMultiplier =           0;
    regconf.debounce =                  0;
    regconf.i2cTimeout =                10;   
    regconf.filterSettings =            MAV_FILTER;  
    regconf.filterDynBottomBeta =        8;   
    regconf.filterDynLowerSpeed =        5;   
    regconf.filterDynUpperSpeed =        250; 
    // 🔥 10에서 25로 늘림 (살짝 만졌을 때 제스처가 아닌 커서 이동으로 인식하게 함)
    regconf.initScrollDistance =        255;   
    return regconf;
}
/* 🔥 해킹 1: 읽기 실패하면 1ms마다 최대 100번 재시도 (문 열릴 때까지 대기) 🔥 */
static int iqs5xx_seq_read(const struct device *dev, const uint16_t start, uint8_t *read_buf, const uint8_t len) {
    const struct iqs5xx_data *data = dev->data;
    uint16_t nstart = (start << 8 ) | (start >> 8);
    int ret = -1;
    
    for (int i = 0; i < 3; i++) {
        ret = i2c_write_read(data->i2c, AZOTEQ_IQS5XX_ADDR, &nstart, sizeof(nstart), read_buf, len);
        if (ret == 0) return 0; // 성공하면 즉시 탈출
        k_usleep(1000); // 1ms 대기 후 재시도
    }
    return ret;
}

/* 🔥 해킹 2: 쓰기 실패하면 1ms마다 최대 100번 재시도 🔥 */
static int iqs5xx_write(const struct device *dev, const uint16_t start_addr, const uint8_t *buf, uint32_t num_bytes) {
    const struct iqs5xx_data *data = dev->data;
    uint8_t addr_buffer[2];
    struct i2c_msg msg[2];

    addr_buffer[1] = start_addr & 0xFF;
    addr_buffer[0] = start_addr >> 8;
    msg[0].buf = addr_buffer;
    msg[0].len = 2U;
    msg[0].flags = I2C_MSG_WRITE;

    msg[1].buf = (uint8_t *)buf;
    msg[1].len = num_bytes;
    msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    int ret = -1;
    for (int i = 0; i < 3; i++) {
        ret = i2c_transfer(data->i2c, msg, 2, AZOTEQ_IQS5XX_ADDR);
        if (ret == 0) return 0; // 성공하면 즉시 탈출
        k_usleep(1000); // 1ms 대기 후 재시도
    }
    return ret;
}

static int iqs5xx_reg_dump (const struct device *dev) {
    return iqs5xx_write(dev, IQS5XX_REG_DUMP_START_ADDRESS, _iqs5xx_regdump, IQS5XX_REG_DUMP_SIZE);
}

static int iqs5xx_sample_fetch (const struct device *dev) {
    uint8_t buffer[44];
    struct iqs5xx_data *data = dev->data;
    const struct iqs5xx_config *config = dev->config;

    int res = iqs5xx_seq_read(dev, GestureEvents0_adr, buffer, 44);
    iqs5xx_write(dev, END_WINDOW, 0, 1);

    if (res < 0) return res;

    data->raw_data.gestures0 =      buffer[0];
    data->raw_data.gestures1 =      buffer[1];
    data->raw_data.system_info0 =   buffer[2];
    data->raw_data.system_info1 =   buffer[3];
    data->raw_data.finger_count =   buffer[4];

    int16_t raw_rx = (int16_t)(buffer[5] << 8 | buffer[6]);
    int16_t raw_ry = (int16_t)(buffer[7] << 8 | buffer[8]);

    struct coord_transform rel_transformed = apply_coordinate_transform(raw_rx, raw_ry, config);
    data->raw_data.rx = rel_transformed.x;
    data->raw_data.ry = rel_transformed.y;

    for(int i = 0; i < 5; i++) {
        const int p = 9 + (7 * i);
        data->raw_data.fingers[i].ax = buffer[p + 0] << 8 | buffer[p + 1];
        data->raw_data.fingers[i].ay = buffer[p + 2] << 8 | buffer[p + 3];
        data->raw_data.fingers[i].strength = buffer[p + 4] << 8 | buffer[p + 5];
        data->raw_data.fingers[i].area= buffer[p + 6];
        apply_finger_transform(&data->raw_data.fingers[i], config);
    }
    return 0;
}

/* 🔥 해킹 3: 에러 카운터, 강제 리셋 로직 모조리 삭제. 무조건 10ms마다 불도저처럼 전진 🔥 */
static void iqs5xx_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, work);

    k_mutex_lock(&data->i2c_mutex, K_MSEC(100));

    /* 🔥 추가된 꼼수: 가짜 통신으로 칩셋 뺨 때려서 깨우기 (Wake-up Ping) 🔥 */
    uint8_t dummy = 0;
    i2c_write(data->i2c, &dummy, 0, AZOTEQ_IQS5XX_ADDR); 
    k_usleep(2000); // 뺨 때리고 2ms 동안 정신 차릴 때까지 기다려줌
    int ret = iqs5xx_sample_fetch(data->dev);
    
    // 에러 나든 말든(ret < 0) 신경 안 씀! 통신 성공했을 때만 데이터 전송
    if (ret == 0 && data->data_ready_handler != NULL) {
        data->data_ready_handler(data->dev, &data->raw_data);
    }
    
    k_mutex_unlock(&data->i2c_mutex);
    
    k_work_reschedule(&data->work, K_MSEC(5));
}

int iqs5xx_trigger_set(const struct device *dev, iqs5xx_trigger_handler_t handler) {
    struct iqs5xx_data *data = dev->data;
    data->data_ready_handler = handler;
    return 0;
}

int iqs5xx_registers_init (const struct device *dev, const struct iqs5xx_reg_config *config) {
    struct iqs5xx_data *data = dev->data;
    k_mutex_lock(&data->i2c_mutex, K_MSEC(5000));

    uint8_t buf = RESET_TP;
    iqs5xx_write(dev, SystemControl1_adr, &buf, 1);
    iqs5xx_write(dev, END_WINDOW, 0, 1);
    iqs_regdump_err = iqs5xx_reg_dump(dev);

    uint8_t wbuff[16];
    *((uint16_t*)wbuff) = SWPEND16(config->activeRefreshRate);
    iqs5xx_write(dev, ActiveRR_adr, wbuff, 2);

    *((uint16_t*)wbuff) = SWPEND16(config->idleRefreshRate);
    iqs5xx_write(dev, IdleRR_adr, wbuff, 2);

    iqs5xx_write(dev, SFGestureEnable_adr, &config->singleFingerGestureMask, 1);
    iqs5xx_write(dev, MFGestureEnable_adr, &config->multiFingerGestureMask, 1);

    *((uint16_t*)wbuff) = SWPEND16(config->tapTime);
    iqs5xx_write(dev, TapTime_adr, wbuff, 2);

    *((uint16_t*)wbuff) = SWPEND16(config->tapDistance);
    iqs5xx_write(dev, TapDistance_adr, wbuff, 2);

    iqs5xx_write(dev, GlobalTouchSet_adr, &config->touchMultiplier, 1);
    iqs5xx_write(dev, ProxDb_adr, &config->debounce, 1);
    iqs5xx_write(dev, TouchSnapDb_adr, &config->debounce, 1);

    wbuff[0] = 0;
    iqs5xx_write(dev, HardwareSettingsA_adr, wbuff, 1);
    iqs5xx_write(dev, I2CTimeout_adr, &config->i2cTimeout, 1);
    iqs5xx_write(dev, FilterSettings0_adr, &config->filterSettings, 1);
    iqs5xx_write(dev, DynamicBottomBeta_adr, &config->filterDynBottomBeta, 1);
    iqs5xx_write(dev, DynamicLowerSpeed_adr, &config->filterDynLowerSpeed, 1);

    *((uint16_t*)wbuff) = SWPEND16(config->filterDynUpperSpeed);
    iqs5xx_write(dev, DynamicUpperSpeed_adr, wbuff, 2);

    *((uint16_t*)wbuff) = SWPEND16(config->initScrollDistance);
    iqs5xx_write(dev, ScrollInitDistance_adr, wbuff, 2);

    iqs5xx_write(dev, END_WINDOW, 0, 1);
    k_mutex_unlock(&data->i2c_mutex);

    return 0; // 에러 무시
}

static int iqs5xx_init(const struct device *dev) {
    struct iqs5xx_data *data = dev->data;
    data->dev = dev;
    data->i2c = DEVICE_DT_GET(DT_BUS(DT_DRV_INST(0)));

    if (!data->i2c) return -ENODEV;

    k_mutex_init(&data->i2c_mutex);
    k_work_init_delayable(&data->work, iqs5xx_work_cb);
    
    struct iqs5xx_reg_config iqs5xx_registers = iqs5xx_reg_config_default();
    iqs5xx_registers_init(dev, &iqs5xx_registers);

    /* 불도저 엔진 시동! */
    k_work_reschedule(&data->work, K_MSEC(10));
    return 0;
}

static struct iqs5xx_data iqs5xx_data_0 = { .data_ready_handler = NULL };

static const struct iqs5xx_config iqs5xx_config_0 = {
    .dr = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), dr_gpios, {}),
    .invert_x = DT_INST_PROP(0, invert_x),
    .invert_y = DT_INST_PROP(0, invert_y),
    .rotate_90 = DT_INST_PROP(0, rotate_90),
    .rotate_180 = DT_INST_PROP(0, rotate_180),
    .rotate_270 = DT_INST_PROP(0, rotate_270),
    .sensitivity = (uint8_t)MIN(255, MAX(64, DT_INST_PROP_OR(0, sensitivity, 128))),
};

DEVICE_DT_INST_DEFINE(0, iqs5xx_init, NULL, &iqs5xx_data_0, &iqs5xx_config_0,
                      POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, NULL);
