#include "osd.h"
#include "imgui.h"


//======================================================
//======================================================
OSD::OSD()
{
    //int c = 1;
    for ( int row = 0; row < OSD_ROWS; row++ )
    {
        for ( int col = 0; col < OSD_COLS; col++ )
        {
            this->screen[row][col] = 0;//c++ % 512;
        }
    }
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
        for ( int col = 0; col < OSD_COLS; col++ )
        {
            uint16_t c = this->screen[row][col];
            if ( c != 0 )
            {
                this->font->drawChar(c, x, y, ixs, iys);
            }
            x += ixs;
        }
        y += iys;
    }
}
