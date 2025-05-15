#include <time.h>
#include <cstdint>
#include <fstream>

class TimeUtil
{
public:
    TimeUtil() = default;
    ~TimeUtil() = default;

    void StartTimer()
    {
        timespec time;

        clock_gettime(CLOCK_MONOTONIC, &time);
    }

    uint64_t EndTimer();

    void WriteTimeInfoToFile(std::ifstream* is, )
    {
        
    }

private:
    uint64_t mStartTime;
    uint64_t mEndTime;
    uint64_t mCurrentTime;
};

