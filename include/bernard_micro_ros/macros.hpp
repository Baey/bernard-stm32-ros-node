#ifndef _MACROS_HPP
#define _MACROS_HPP

#include <rcl/rcl.h>

/// @brief Macro to check the return value of a ROS function call.
#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            return false;            \
        }                            \
    }

/// @brief Macro to execute code every given milliseconds.
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

#endif // _MACROS_HPP