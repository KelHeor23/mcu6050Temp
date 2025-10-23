#ifndef UTILS_H
#define UTILS_H

#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Печать ошибки с errno
#define prerr(fmt, ...) \
do { fprintf(stderr, "[ERR] " fmt " (%s)\n", ##__VA_ARGS__, strerror(errno)); } while (0)

    // Безопасный wrapper: логирует, если res == -1 (errno уже установлен)
    int sysguard(int res, char *msg);

// Время в секундах с наносекундной точностью
double get_time_sec();

#ifdef __cplusplus
}
#endif

#endif
