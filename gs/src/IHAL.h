#pragma once

#include "imgui.h"
#include <functional>

class IHAL
{
public:
    virtual ~IHAL() = default;

    virtual bool init() = 0;
    virtual void shutdown() = 0;

    virtual void* get_window() = 0;
    virtual void* get_main_context() = 0;
    virtual void* lock_main_context() = 0;
    virtual void unlock_main_context() = 0;

    virtual ImVec2 get_display_size() const = 0;
    virtual void set_backlight(float brightness) = 0; //0..1
    virtual void set_video_channel(unsigned int channel, unsigned int id)=0;
    virtual void add_render_callback(std::function<void()> func)=0;
    virtual bool process() = 0;
};
