#include "utils.h"

double get_time_sec()
{
    struct timespec now = {0, 0};
#if defined(CLOCK_MONOTONIC_RAW)
    clock_gettime(CLOCK_MONOTONIC_RAW, &now);
#else
    clock_gettime(CLOCK_MONOTONIC, &now);
#endif
    return (double)now.tv_sec + 1e-9 * now.tv_nsec;
}

int sysguard(int res, char *msg)
{
    if (res == -1) {
        fprintf(stderr, "[ERR] %s (%s)\n", msg, strerror(errno));
    }
    return res;
}
