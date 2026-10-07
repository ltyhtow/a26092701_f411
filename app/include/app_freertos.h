#ifndef APP_FREERTOS_H
#define APP_FREERTOS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* One boot attempt only, before starting the scheduler. Failure requires reset. */
int32_t app_synctasks_init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_FREERTOS_H */
