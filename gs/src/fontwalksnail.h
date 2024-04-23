#pragma once

 #include <stdint.h>

//======================================================
//======================================================
class FontWalksnail 
{
private:

  int fontTextureId = 0;

  unsigned int charWidth;
  unsigned int charHeight;

  unsigned int fontTextureWidth;
  unsigned int fontTextureHeight;

  void calculateTextureHeight(unsigned int imageWidth, unsigned int imageHeight);

public:
	FontWalksnail(const char* fileName);
	~FontWalksnail();

  void drawChar(uint16_t code, float x1, float y1, float width, float height);
  void destroy();

};
