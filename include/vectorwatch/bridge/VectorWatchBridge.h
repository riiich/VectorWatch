#pragma once
#include <stdint.h>
#if defined(_WIN32)
# if defined(VECTORWATCH_BRIDGE_EXPORTS)
#  define VW_API __declspec(dllexport)
# else
#  define VW_API __declspec(dllimport)
# endif
#else
# define VW_API __attribute__((visibility("default")))
#endif
#ifdef __cplusplus
extern "C" {
#endif
/* ABI v1. All strings are UTF-8. Every returned string must be freed with
   vw_free_string. A run has one owner: serialize read/stop/destroy calls.
   No exceptions cross this boundary. Errors return null and set *error.
   Destroy cancels and joins all native threads. */
typedef struct vw_options {
    const char* scenario;
    double speed;
    uint32_t seed;
    uint32_t uncertainty_seed;
    int32_t has_seed;
    int32_t probabilistic;
    int32_t samples;
    int32_t workers;
    int32_t uncertainty; /* 0 low, 1 medium, 2 high */
    int32_t outcome; /* 0 any, 1 collision, 2 pass */
} vw_options;
VW_API char* vw_scenarios(char** error);
VW_API void* vw_create(const vw_options* options, char** error);
VW_API char* vw_read(void* run, char** error);
VW_API void vw_stop(void* run);
VW_API void vw_destroy(void* run);
VW_API void vw_free_string(char* text);
#ifdef __cplusplus
}
#endif
