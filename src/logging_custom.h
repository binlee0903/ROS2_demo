#include <time.h>
#include <fstream>

#define BILLION 1000000000ULL

// #include "./logging_custom.h"
// unsigned long long timedelay = 0;
// unsigned long long total_time = 0;
//   unsigned long long total_count = 0;

//   struct timespec clock_data[2];
//   clock_gettime(CLOCK_MONOTONIC, &clock_data[0]);

//   clock_gettime(CLOCK_MONOTONIC, &clock_data[1]);

//   calclock(clock_data, &total_time, &total_count);

//   printf("----------------------------------------\n");
//   printf("누적된 총 시간: %llu ns\n", total_time);
//   printf("총 측정 횟수: %llu\n", total_count);

//   if (total_count > 0) {
//         printf("평균 소요 시간: %llu ns\n", total_time / total_count);
//   }

#define calclock(timevalue, total_time, total_count, time_delay) do { \
    unsigned long long temp_s, temp_ns; \
    struct timespec *myclock = (struct timespec*)(timevalue); \
    \
    if (myclock[1].tv_nsec >= myclock[0].tv_nsec) { \
        temp_s = myclock[1].tv_sec - myclock[0].tv_sec; \
        temp_ns = myclock[1].tv_nsec - myclock[0].tv_nsec; \
    } else { \
        temp_s = myclock[1].tv_sec - myclock[0].tv_sec - 1; \
        temp_ns = BILLION + myclock[1].tv_nsec - myclock[0].tv_nsec; \
    } \
    \
    time_delay = BILLION * temp_s + temp_ns; \
    \
    __sync_fetch_and_add(total_time, time_delay); \
    __sync_fetch_and_add(total_count, 1); \
} while(0)
