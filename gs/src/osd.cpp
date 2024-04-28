#include "osd.h"
#include "imgui.h"


//======================================================
//======================================================
OSD::OSD()
{
    memset( &this->buffer, 0, OSD_BUFFER_SIZE );
}

//======================================================
//======================================================
void OSD::init()
{
    this->font = new FontWalksnail("assets/INAV_default_24.png");
}

//======================================================
//======================================================
void OSD::draw()
{
    ImVec2 screenSize = ImGui::GetIO().DisplaySize;

    float fxs = screenSize.x / OSD_COLS;
    float fys = screenSize.y / OSD_ROWS;

    int ixs = (int) fxs;
    int iys = (int) fys;

    int mx = ((int)screenSize.x - OSD_COLS * ixs) / 2;
    int my = ((int)screenSize.y - OSD_ROWS * iys) / 2;

    int y = my;
    for ( int row = 0; row < OSD_ROWS; row++ )
    {
        int x = mx;
        int mask = 1;
        int col8 = 0;
        for ( int col = 0; col < OSD_COLS; col++ )
        {
            uint16_t c = this->buffer.screenLow[row][col];
            if ( (this->buffer.screenHigh[row][col8] & mask) !=0 )
            {
                c += 0x100;
            }
            if ( c != 0 )
            {
                this->font->drawChar(c, x, y, ixs, iys);
            }
            x += ixs;
            mask <<= 1;
            if ( mask == 0x100 )
            {
                mask = 1;
                col8++;
            }
        }
        y += iys;
    }
}

//======================================================
//======================================================
void OSD::update(void* pScreen)
{
    memcpy(&this->buffer, pScreen, OSD_BUFFER_SIZE);
}

