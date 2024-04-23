#pragma once

#include <stdint.h>
#include "packets.h"


#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#else
#define IRAM_ATTR
#endif


class OSD
{
private:

    struct OSDBuffer buffer;
    bool changed;

public:
    OSD();
    IRAM_ATTR void* getBuffer();
    bool isChanged();
};

extern OSD g_osd;