#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int fd;
    uint8_t addr; // 7-bit I2C address (0x68 or 0x69)
} mpu6050_t;

// Инициализация: открывает /dev/i2c-X, выставляет адрес 0x68, будит чип.
mpu6050_t *mpu6050_init(const char *devpath);

// Освобождение ресурсов
int mpu6050_cleanup(mpu6050_t *m);

// Чтение 16-битного регистрового слова (addr и addr+1), big-endian
int16_t mpu6050_read_word(mpu6050_t *m, uint8_t addr);

// Сырые значения (без масштабирования в g/deg/s)
void mpu6050_get_gyro_raw(mpu6050_t *m, int16_t *gx, int16_t *gy, int16_t *gz);
void mpu6050_get_accl_raw(mpu6050_t *m, int16_t *ax, int16_t *ay, int16_t *az);

// Простые set/get для регистров конфигурации
void    mpu6050_set_config(mpu6050_t *m, uint8_t config);       // REG 0x1A
uint8_t mpu6050_get_config(mpu6050_t *m);

void    mpu6050_set_gyro_config(mpu6050_t *m, uint8_t config);  // REG 0x1B
uint8_t mpu6050_get_gyro_config(mpu6050_t *m);

void    mpu6050_set_accl_config(mpu6050_t *m, uint8_t config);  // REG 0x1C
uint8_t mpu6050_get_accl_config(mpu6050_t *m);

#ifdef __cplusplus
}
#endif

#endif
