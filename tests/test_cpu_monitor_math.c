#include <assert.h>
#include "../main/cpu_monitor_math.h"
int main(void)
{
    uint16_t busy;
    assert(cpu_monitor_busy(0, 180000, 1000000, &busy) && busy == 820);
    assert(cpu_monitor_busy(0, 620000, 1000000, &busy) && busy == 380);
    assert(cpu_monitor_busy(0, 0, 1000000, &busy) && busy == 1000);
    assert(cpu_monitor_busy(0, 1000000, 1000000, &busy) && busy == 0);
    assert(cpu_monitor_busy(0xFFFFFFFFULL, 0xFFFFFFFFULL + 250000, 1000000, &busy) && busy == 750);
    assert(cpu_monitor_busy(500, 1000500, 2000000, &busy) && busy == 500);
    assert(!cpu_monitor_busy(10, 9, 1000, &busy));
    assert(!cpu_monitor_busy(0, 1001, 1000, &busy));
    assert(!cpu_monitor_busy(0, 0, 0, &busy));
    assert(!cpu_monitor_busy(0, 1, UINT64_MAX, &busy));
    return 0;
}
