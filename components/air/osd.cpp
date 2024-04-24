#include <cstring>
#include "esp_timer.h"

#include "osd.h"

#ifdef UART_MSP_OSD

OSD g_osd;

//==============================================================
//==============================================================
OSD::OSD()
{
    this->delayedUpdate = 0;
    this->clear();
}

//==============================================================
//==============================================================
void OSD::clear()
{
    memset( &this->buffer, 0, OSD_BUFFER_SIZE );
    
    this->delayedUpdate = esp_timer_get_time() + 1000000;
}

//==============================================================
//==============================================================
void OSD::commit()
{
    this->delayedUpdate = 0;
    this->changed = false;
}


//==============================================================
//==============================================================
void OSD::writeString(unsigned int row, unsigned int col, int isExtChar, uint8_t* str, int len)
{
    if ( row >= OSD_ROWS ) return;

    uint8_t flag = isExtChar ? 0xff : 0;
    
    while ( (len > 0) && (col < OSD_COLS) )
    {
        this->buffer.screenLow[row][col] = *str++;

        int col8 = col >> 3;
        int sh = col & 0x7;
        uint8_t m = 1 << sh;
        this->buffer.screenHigh[row][col8] = (this->buffer.screenHigh[row][col8] & ~m) | (m & flag);

        len--;
        col++;
    }
}

//==============================================================
//==============================================================
void* OSD::getBuffer()
{
    return &this->buffer;
}

//==============================================================
//==============================================================
bool OSD::isChanged()
{
    if ( this->delayedUpdate !=0 )
    {
        if ( esp_timer_get_time() >= this->delayedUpdate )
        {
            this->delayedUpdate = 0;
            this->changed = true;
        }
    }

    /*
    for ( int i = 0; i < 10; i++)
    {
        this->buffer.screenLow[10][10+i]++;
    }
    this->changed = true;
    */

    bool res = this->changed;
    this->changed = false;
    return res;
}

#endif //UART_MSP_OSD