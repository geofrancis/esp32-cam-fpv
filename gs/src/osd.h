#pragma once

#include "fontwalksnail.h"
#include "packets.h"

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
    void update(uint16_t* pScreen);
};

