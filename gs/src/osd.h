#pragma once

#include "fontwalksnail.h"
#include "packets.h"

//======================================================
//======================================================
class OSD
{
private:
    FontWalksnail* font;
    OSDBuffer buffer;

public:
    OSD();
    void init();
    void draw();
    void update(void* pScreen);
};

