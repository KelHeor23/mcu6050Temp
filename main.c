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

    mpu6050_scale_t S = {0};
    mpu6050_bias_t  B = {0};

    mpu6050_read_and_print_scales(m, &S);

    // 3 секунды калибровка в покое, 100 Гц
    if (mpu6050_calibrate_bias(m, &S, 3.0, 100.0, &B) != 0) {
        fprintf(stderr, "Calibration failed\n");
        mpu6050_cleanup(m);
        return 1;
    }

    printf("ax_ms2,ay_ms2,az_ms2,gx_dps,gy_dps,gz_dps\n");
    while (1) {
        int16_t rax, ray, raz, rgx, rgy, rgz;
        double ax, ay, az, gx, gy, gz;

        mpu6050_get_accl_raw(m, &rax, &ray, &raz);
        mpu6050_get_gyro_raw(m, &rgx, &rgy, &rgz);

        mpu6050_convert_raw_si(&S, rax, ray, raz, rgx, rgy, rgz,
                               &ax, &ay, &az, &gx, &gy, &gz);

        // Применяем bias: после этого в покое аксель ~0 (g удалена), гира ~0
        mpu6050_apply_bias(&B, &ax, &ay, &az, &gx, &gy, &gz);

        printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", ax, ay, az, gx, gy, gz);
        fflush(stdout);
        usleep(10000); // 100 Гц
    }

    mpu6050_cleanup(m);
    return 0;
}
