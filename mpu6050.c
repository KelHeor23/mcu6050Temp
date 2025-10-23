#include "mpu6050.h"
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

////////////////////////////////////////////


mpu6050_t *mpu6050_init(const char *dev)
{
        mpu6050_t *m = (mpu6050_t *)malloc(sizeof(mpu6050_t));
        if (!m)
        {
                prerr("malloc failed");
                return NULL;
        }

        m->addr = 0x68; // исправлено на ваш адрес
        strncpy(m->device, dev, 63);
        m->device[63] = '\0';

        if ((m->fd = open(m->device, O_RDWR)) < 0)
        {
                prerr("cannot open %s. (%s)", m->device, strerror(errno));
                free(m);
                return NULL;
        }

        if (ioctl(m->fd, I2C_SLAVE, m->addr) < 0)
        {
                prerr("ioctl failed to set device address (%s)", strerror(errno));
                close(m->fd);
                free(m);
                return NULL;
        }

        // Пробуждаем MPU6050 (снимаем с режима сна)
        if (i2c_smbus_write_byte_data(m->fd, 0x6B, 0x00) < 0)
        {
                prerr("Failed to wake up MPU6050");
                close(m->fd);
                free(m);
                return NULL;
        }

        // Добавляем задержку для стабилизации после пробуждения
        usleep(100000);

        return m;
}


////////////////////////////////////////////


int16_t mpu6050_read_word(mpu6050_t *m, uint16_t addr)
{
        int high = i2c_smbus_read_byte_data(m->fd, addr);
        int low = i2c_smbus_read_byte_data(m->fd, addr + 1);

        if (high < 0 || low < 0)
        {
                prerr("I2C read error at addr 0x%X", addr);
                return 0; // или можно вернуть ошибку каким-то иным способом
        }

        return (int16_t)((high << 8) | low);
}


////////////////////////////////////////////


void mpu6050_get_gyro_raw(mpu6050_t *m, int16_t *gx, int16_t *gy, int16_t *gz)
{
        *gx = mpu6050_read_word(m, 0x43);
        *gy = mpu6050_read_word(m, 0x45);
        *gz = mpu6050_read_word(m, 0x47);
}


////////////////////////////////////////////


void mpu6050_get_accl_raw(mpu6050_t *m, int16_t *ax, int16_t *ay, int16_t *az)
{
        *ax = mpu6050_read_word(m, 0x3B);
        *ay = mpu6050_read_word(m, 0x3D);
        *az = mpu6050_read_word(m, 0x3F);
}
