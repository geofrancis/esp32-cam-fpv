 #pragma once

#include <unistd.h>
#include <cstdint>  

#define AVI_HEADER_LEN 240 // AVI header length
#define CHUNK_HDR 8 // bytes per jpeg hdr in AVI 
#define DVR_MAX_FRAMES  65000   //25fps = 43 minues


extern const uint8_t dcBuf[]; // 00dc

extern uint8_t aviHeader[AVI_HEADER_LEN];

extern void prepAviIndex(bool isTL = false);
extern void finalizeAviIndex(uint16_t frameCnt, bool isTL = false);
extern size_t writeAviIndex(uint8_t* clientBuf, size_t buffSize, bool isTL = false);
extern void buildAviHdr(uint8_t FPS, uint8_t frameType, bool clip16x9, uint16_t frameCnt, bool isTL = false);
extern void buildAviIdx(size_t dataSize, bool isVid = true, bool isTL = false);
extern bool openAvi();



