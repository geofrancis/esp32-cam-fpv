#pragma once

#include "fontwalksnail.h"

#define OSD_COLS 53
#define OSD_ROWS 20

//======================================================
//======================================================
class OSD
{
private:
    FontWalksnail* font;
    uint16_t screen[OSD_ROWS][OSD_COLS];

public:
    OSD();
    void init();
    void draw();
};

