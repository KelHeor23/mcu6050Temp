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
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

static void decode_afs(uint8_t afs_sel, mpu6050_scale_t *S)
{
    // AFS_SEL: 0=±2g, 1=±4g, 2=±8g, 3=±16g
    static const double lsb_per_g[4] = {16384.0, 8192.0, 4096.0, 2048.0};
    S->afs_sel = afs_sel & 0x03;
    S->acc_lsb_per_g   = lsb_per_g[S->afs_sel];
    S->acc_ms2_per_lsb = 9.80665 / S->acc_lsb_per_g;
}

static void decode_gfs(uint8_t gfs_sel, mpu6050_scale_t *S)
{
    // FS_SEL: 0=±250, 1=±500, 2=±1000, 3=±2000 deg/s
    static const double lsb_per_dps[4] = {131.0, 65.5, 32.8, 16.4};
    S->gfs_sel        = gfs_sel & 0x03;
    S->gyr_lsb_per_dps  = lsb_per_dps[S->gfs_sel];
    S->gyr_dps_per_lsb  = 1.0 / S->gyr_lsb_per_dps;
    S->gyr_rads_per_lsb = (M_PI / 180.0) * S->gyr_dps_per_lsb;
}

void mpu6050_read_and_print_scales(mpu6050_t *m, mpu6050_scale_t *S)
{
    uint8_t a_cfg = mpu6050_get_accl_config(m);
    uint8_t g_cfg = mpu6050_get_gyro_config(m);
    uint8_t afs = (a_cfg >> 3) & 0x03;
    uint8_t gfs = (g_cfg >> 3) & 0x03;

    decode_afs(afs, S);
    decode_gfs(gfs, S);

    const char *afs_names[4] = { "±2g", "±4g", "±8g", "±16g" };
    const char *gfs_names[4] = { "±250 dps", "±500 dps", "±1000 dps", "±2000 dps" };

    fprintf(stderr, "[INFO] ACCEL range: %s (%.0f LSB/g, %.9f m/s^2 per LSB)\n",
            afs_names[S->afs_sel], S->acc_lsb_per_g, S->acc_ms2_per_lsb);

    fprintf(stderr, "[INFO] GYRO  range: %s (%.1f LSB/(deg/s), %.9f deg/s per LSB)\n",
            gfs_names[S->gfs_sel], S->gyr_lsb_per_dps, S->gyr_dps_per_lsb);
}

void mpu6050_convert_raw_si(const mpu6050_scale_t *S,
                            int16_t ax, int16_t ay, int16_t az,
                            int16_t gx, int16_t gy, int16_t gz,
                            double *ax_ms2, double *ay_ms2, double *az_ms2,
                            double *gx_dps, double *gy_dps, double *gz_dps)
{
    *ax_ms2 = ((double)ax) * S->acc_ms2_per_lsb;
    *ay_ms2 = ((double)ay) * S->acc_ms2_per_lsb;
    *az_ms2 = ((double)az) * S->acc_ms2_per_lsb;

    *gx_dps = ((double)gx) * S->gyr_dps_per_lsb;
    *gy_dps = ((double)gy) * S->gyr_dps_per_lsb;
    *gz_dps = ((double)gz) * S->gyr_dps_per_lsb;
}

int mpu6050_calibrate_bias(mpu6050_t *m, const mpu6050_scale_t *S,
                           double duration_sec, double fs_hz, mpu6050_bias_t *B)
{
    if (!m || !S || !B || fs_hz <= 0.0 || duration_sec <= 0.0) return -1;

    const double dt_us = 1e6 / fs_hz;
    const size_t N = (size_t)(duration_sec * fs_hz);

    double sax=0, say=0, saz=0;
    double sgx=0, sgy=0, sgz=0;

    for (size_t i = 0; i < N; ++i) {
        int16_t rax, ray, raz, rgx, rgy, rgz;
        double ax, ay, az, gx, gy, gz;

        mpu6050_get_accl_raw(m, &rax, &ray, &raz);
        mpu6050_get_gyro_raw(m, &rgx, &rgy, &rgz);

        mpu6050_convert_raw_si(S, rax, ray, raz, rgx, rgy, rgz,
                               &ax, &ay, &az, &gx, &gy, &gz);

        sax += ax;  say += ay;  saz += az;
        sgx += gx;  sgy += gy;  sgz += gz;

        usleep((useconds_t)dt_us);
    }

    // Средние в SI
    double axm = sax / (double)N;
    double aym = say / (double)N;
    double azm = saz / (double)N;

    double gxm = sgx / (double)N;
    double gym = sgy / (double)N;
    double gzm = sgz / (double)N;

    // Гиро-bias: просто среднее (deg/s)
    B->bgx = gxm;
    B->bgy = gym;
    B->bgz = gzm;

    // Аксель: хотим, чтобы после вычитания в покое было ≈ 0 м/с^2 (т.е. удалить g в текущей ориентации).
    // Это эквивалентно "bias = среднее", т.к. среднее в покое содержит гравитацию.
    B->bax = axm;
    B->bay = aym;
    B->baz = azm;

    // Диагностика
    double anorm = sqrt(axm*axm + aym*aym + azm*azm);
    fprintf(stderr, "[CAL] accel mean = (%.5f, %.5f, %.5f) m/s^2 |norm|=%.5f (ожидание ~9.81)\n",
            axm, aym, azm, anorm);
    fprintf(stderr, "[CAL] gyro  mean = (%.5f, %.5f, %.5f) deg/s\n", gxm, gym, gzm);
    fprintf(stderr, "[CAL] biases set. После вычитания в покое accel≈(0,0,0), gyro≈(0,0,0).\n");

    return 0;
}

