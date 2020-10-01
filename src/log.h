#pragma once

#ifndef LOG_LOCAL_LEVEL
#ifdef CONFIG_LOG_DEFAULT_LEVEL
#define LOG_LOCAL_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#endif
#endif

#ifndef LOG_LOCAL_LEVEL
#define _e(...)
#define _w(...)
#define _i(...)
#define _d(...)
#define _v(...)

#else
#ifndef TAG
#define TAG "NO NAME"
#endif

#include "esp_log.h"
#define _e(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)
#define _w(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define _i(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define _d(format, ...) ESP_LOGD(TAG, format, ##__VA_ARGS__)
#define _v(format, ...) ESP_LOGV(TAG, format, ##__VA_ARGS__)

#endif
