#include "../include/marg.h"
#include "../sensor/marg_sensor.h"

marg_ret_t marg_init(void) {
    if(!marg_sensor_init()) {
        return MARG_ERROR_INIT;
    }
#if defined(CONFIG_MARG_PINNED_TO_CORE_0)
#elif defined(CONFIG_MARG_PINNED_TO_CORE_1)
#else
#endif
    return MARG_NOERROR;
}
