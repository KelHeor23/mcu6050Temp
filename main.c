#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "mpu6050.h"

int main()
{
    mpu6050_t *m = mpu6050_init("/dev/i2c-0");
    if (!m)
    {
        printf("Ошибка инициализации MPU6050\n");
        return 1;
    }

    int16_t gx = 0, gy = 0, gz = 0;
    int16_t ax = 0, ay = 0, az = 0;

    while (1)
    {
        mpu6050_get_gyro_raw(m, &gx, &gy, &gz);
        mpu6050_get_accl_raw(m, &ax, &ay, &az);

        printf("%d,%d,%d,%d,%d,%d\n", ax, ay, az, gx, gy, gz);

        usleep(100000); // 100ms delay
    }

    mpu6050_cleanup(m);
    return 0;
}
