#include <stdio.h>
#include <unistd.h>
#include "mpu6050.h"

int main(void)
{
    // Замените путь, если у вас i2c-1
    mpu6050_t *m = mpu6050_init("/dev/i2c-0");
    if (!m) {
        fprintf(stderr, "MPU6050 init failed\n");
        return 1;
    }

    // Пример: убедимся, что чип «проснулся»
    // (mpu6050_init уже делает write в PWR_MGMT_1 = 0x00)
    // При желании можно выставить DLPF:
    // mpu6050_set_config(m, 0x03);

    printf("ax,ay,az,gx,gy,gz\n");
    while (1) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050_get_accl_raw(m, &ax, &ay, &az);
        mpu6050_get_gyro_raw(m, &gx, &gy, &gz);
        printf("%d,%d,%d,%d,%d,%d\n", ax, ay, az, gx, gy, gz);
        fflush(stdout);
        usleep(100000); // 100 ms
    }

    mpu6050_cleanup(m);
    return 0;
}
