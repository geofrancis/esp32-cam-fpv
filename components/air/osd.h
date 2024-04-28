#pragma once

#include <stdint.h>
#include "packets.h"
#include "main.h"


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
    int64_t delayedUpdate;

public:
    OSD();
    IRAM_ATTR void* getBuffer();
    bool isChanged();
    void clear();
    void commit();
    void writeString(unsigned int row, unsigned int col, int isExtChar, uint8_t* str, int len);
};

extern OSD g_osd;
