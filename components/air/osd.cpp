#include <cstring>
#include "osd.h"

OSD g_osd;

//==============================================================
//==============================================================
OSD::OSD()
{
    memset( &this->buffer, 0, OSD_BUFFER_SIZE );
    this->changed = false;
}

//==============================================================
//==============================================================
void* OSD::getBuffer()
{
    for ( int i = 0; i < 10; i++)
    {
        this->buffer.screenLow[10][10+i]++;
    }
    return &this->buffer;
}

//==============================================================
//==============================================================
bool OSD::isChanged()
{
    this->changed = true;
    
    bool res = this->changed;
    this->changed = false;
    return res;
}