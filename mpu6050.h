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

// ===== High-level conversion & calibration helpers =====

typedef struct {
    // accelerometer
    double acc_lsb_per_g;      // 16384, 8192, 4096, 2048
    double acc_ms2_per_lsb;    // 9.80665 / acc_lsb_per_g

    // gyroscope
    double gyr_lsb_per_dps;    // 131, 65.5, 32.8, 16.4
    double gyr_dps_per_lsb;    // 1.0 / gyr_lsb_per_dps
    double gyr_rads_per_lsb;   // (M_PI/180) * gyr_dps_per_lsb

    // raw bits from ACCEL_CONFIG/GYRO_CONFIG
    uint8_t afs_sel;           // 0..3
    uint8_t gfs_sel;           // 0..3
} mpu6050_scale_t;

typedef struct {
    // biases in SI units
    double bax, bay, baz;      // m/s^2   (в эту bias входит и гравитация, т.е. после вычитания в покое ~0)
    double bgx, bgy, bgz;      // deg/s
} mpu6050_bias_t;

/** Читает ACCEL_CONFIG/GYRO_CONFIG, вычисляет масштабные коэффициенты и печатает их. */
void mpu6050_read_and_print_scales(mpu6050_t *m, mpu6050_scale_t *S);

/** Конвертирует сырые значения в m/s^2 и deg/s (без вычитания bias). */
void mpu6050_convert_raw_si(const mpu6050_scale_t *S,
                            int16_t ax, int16_t ay, int16_t az,
                            int16_t gx, int16_t gy, int16_t gz,
                            double *ax_ms2, double *ay_ms2, double *az_ms2,
                            double *gx_dps, double *gy_dps, double *gz_dps);

/** Калибрует смещения за duration_sec в покое (обычно 3 c) с fs_hz (обычно 100 Гц).
 *  Возвращает 0 при успехе. Для акселя получает bias ~ среднее (включая g), таким образом
 *  после коррекции a_corr = a - bias ≈ 0 в покое (т.е. гравитация вычтена).
 */
int mpu6050_calibrate_bias(mpu6050_t *m, const mpu6050_scale_t *S,
                           double duration_sec, double fs_hz, mpu6050_bias_t *B);

/** Применяет bias к значениям в SI. */
static inline void mpu6050_apply_bias(const mpu6050_bias_t *B,
                                      double *ax_ms2, double *ay_ms2, double *az_ms2,
                                      double *gx_dps, double *gy_dps, double *gz_dps)
{
    *ax_ms2 -= B->bax;
    *ay_ms2 -= B->bay;
    *az_ms2 -= B->baz;   // после вычитания в покое ≈ 0 (гравитация удалена)
    *gx_dps -= B->bgx;
    *gy_dps -= B->bgy;
    *gz_dps -= B->bgz;
}

#ifdef __cplusplus
}
#endif

#endif
