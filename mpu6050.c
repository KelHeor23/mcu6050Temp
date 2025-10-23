#include "mpu6050.h"
#include "utils.h"

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#define MPU6050_I2C_ADDR_DEFAULT 0x68
#define REG_PWR_MGMT_1  0x6B
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C

mpu6050_t *mpu6050_init(const char *devpath)
{
    mpu6050_t *m = (mpu6050_t *)calloc(1, sizeof(mpu6050_t));
    if (!m) {
        prerr("calloc mpu6050_t failed");
        return NULL;
    }

    m->fd = open(devpath, O_RDWR);
    if (m->fd < 0) {
        prerr("open(%s) failed", devpath);
        free(m);
        return NULL;
    }

    m->addr = MPU6050_I2C_ADDR_DEFAULT;

    if (ioctl(m->fd, I2C_SLAVE, m->addr) < 0) {
        prerr("ioctl(I2C_SLAVE, 0x%02X) failed", m->addr);
        close(m->fd);
        free(m);
        return NULL;
    }

    // Разбудить чип (сбросить бит SLEEP в PWR_MGMT_1)
    if (i2c_smbus_write_byte_data(m->fd, REG_PWR_MGMT_1, 0x00) < 0) {
        prerr("Failed to write PWR_MGMT_1");
        close(m->fd);
        free(m);
        return NULL;
    }

    // Базовая конфигурация по умолчанию (необязательно)
    // DLPF = 3 (0x03) как неплохой компромисс
    i2c_smbus_write_byte_data(m->fd, REG_CONFIG, 0x03);

    return m;
}

int mpu6050_cleanup(mpu6050_t *m)
{
    if (!m) return 0;
    if (m->fd >= 0) close(m->fd);
    free(m);
    return 0;
}

static inline int read_byte_checked(int fd, uint8_t reg)
{
    int v = i2c_smbus_read_byte_data(fd, reg);
    if (v < 0) {
        prerr("I2C read error at reg 0x%02X", reg);
    }
    return v;
}

int16_t mpu6050_read_word(mpu6050_t *m, uint8_t addr)
{
    int high = read_byte_checked(m->fd, addr);
    int low  = read_byte_checked(m->fd, (uint8_t)(addr + 1));

    if (high < 0 || low < 0) {
        // Сообщение уже выведено, вернём 0 как безопасное значение
        return 0;
    }

    // Данные big-endian, знаковые
    return (int16_t)((high << 8) | (low & 0xFF));
}

void mpu6050_get_gyro_raw(mpu6050_t *m, int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (!m) { *gx = *gy = *gz = 0; return; }
    // Регистры: 0x43..0x48
    *gx = mpu6050_read_word(m, 0x43);
    *gy = mpu6050_read_word(m, 0x45);
    *gz = mpu6050_read_word(m, 0x47);
}

void mpu6050_get_accl_raw(mpu6050_t *m, int16_t *ax, int16_t *ay, int16_t *az)
{
    if (!m) { *ax = *ay = *az = 0; return; }
    // Регистры: 0x3B..0x40
    *ax = mpu6050_read_word(m, 0x3B);
    *ay = mpu6050_read_word(m, 0x3D);
    *az = mpu6050_read_word(m, 0x3F);
}

void mpu6050_set_config(mpu6050_t *m, uint8_t config)
{
    if (!m) return;
    if (i2c_smbus_write_byte_data(m->fd, REG_CONFIG, config) < 0)
        prerr("Failed to write CONFIG (0x1A)");
}

uint8_t mpu6050_get_config(mpu6050_t *m)
{
    if (!m) return 0;
    int v = i2c_smbus_read_byte_data(m->fd, REG_CONFIG);
    if (v < 0) { prerr("Failed to read CONFIG (0x1A)"); return 0; }
    return (uint8_t)v;
}

void mpu6050_set_gyro_config(mpu6050_t *m, uint8_t config)
{
    if (!m) return;
    if (i2c_smbus_write_byte_data(m->fd, REG_GYRO_CFG, config) < 0)
        prerr("Failed to write GYRO_CONFIG (0x1B)");
}

uint8_t mpu6050_get_gyro_config(mpu6050_t *m)
{
    if (!m) return 0;
    int v = i2c_smbus_read_byte_data(m->fd, REG_GYRO_CFG);
    if (v < 0) { prerr("Failed to read GYRO_CONFIG (0x1B)"); return 0; }
    return (uint8_t)v;
}

void mpu6050_set_accl_config(mpu6050_t *m, uint8_t config)
{
    if (!m) return;
    if (i2c_smbus_write_byte_data(m->fd, REG_ACCEL_CFG, config) < 0)
        prerr("Failed to write ACCEL_CONFIG (0x1C)");
}

uint8_t mpu6050_get_accl_config(mpu6050_t *m)
{
    if (!m) return 0;
    int v = i2c_smbus_read_byte_data(m->fd, REG_ACCEL_CFG);
    if (v < 0) { prerr("Failed to read ACCEL_CONFIG (0x1C)"); return 0; }
    return (uint8_t)v;
}
