#include <stdio.h>
#include <unistd.h>
#include "mpu6050.h"

int main(void)
{
    mpu6050_t *m = mpu6050_init("/dev/i2c-2");
    if (!m) {
        fprintf(stderr, "MPU6050 init failed\n");
        return 1;
    }

    printf("ax,ay,az,gx,gy,gz\n");
    while (1) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050_get_accl_raw(m, &ax, &ay, &az);
        mpu6050_get_gyro_raw(m, &gx, &gy, &gz);
        printf("ax = %.2f ay = %.2f, az = %.2f gx = %.2f gy = %.2f gz = %.2f\n",
            (float)ax / 16384, (float)ay / 16384, (float)az / 16384,
            (float)gx / 131, (float)gy / 131, (float)gz / 131);
        fflush(stdout);
        usleep(100000); // 100 ms
    }

    mpu6050_cleanup(m);
    return 0;
}
