#include "Comms.h"
#include <iostream>
#include <string>
#include <deque>
#include <mutex>
#include <algorithm>
#include <cstdio>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include "Clock.h"
#include "IHAL.h"
#include "PI_HAL.h"
#include "imgui.h"
#include "HUD.h"
#include "Log.h"
#include "Video_Decoder.h" 
#include "crc.h"
#include "packets.h"
#include <thread>
#include "imgui_impl_opengl3.h"
#include "main.h"

#ifdef TEST_LATENCY
extern "C"
{
#include "pigpio.h"
}
#endif
/*

Changed on the PI:

- Disable the compositor from raspi-config. This will increase FPS
- Change from fake to real driver: dtoverlay=vc4-fkms-v3d to dtoverlay=vc4-kms-v3d

*/

std::unique_ptr<IHAL> s_hal;
Comms s_comms;
Video_Decoder s_decoder;

/* This prints an "Assertion failed" message and aborts.  */
void __assert_fail(const char* __assertion, const char* __file, unsigned int __line, const char* __function)
{
    printf("assert: %s:%d: %s: %s", __file, __line, __function, __assertion);
    fflush(stdout);
    //    abort();
}

static std::thread s_comms_thread;

static std::mutex s_ground2air_config_packet_mutex;
static Ground2Air_Config_Packet s_ground2air_config_packet;

#ifdef TEST_LATENCY
static uint32_t s_test_latency_gpio_value = 0;
static Clock::time_point s_test_latency_gpio_last_tp = Clock::now();
#endif
float video_fps = 0;
static void comms_thread_proc()
{
    Clock::time_point last_stats_tp = Clock::now();
    Clock::time_point last_comms_sent_tp = Clock::now();
    uint8_t last_sent_ping = 0;
    Clock::time_point last_ping_sent_tp = Clock::now();
    Clock::duration ping_min = std::chrono::seconds(999);
    Clock::duration ping_max = std::chrono::seconds(0);
    Clock::duration ping_avg = std::chrono::seconds(0);
    size_t ping_count = 0;
    size_t sent_count = 0;
    size_t total_data = 0;
    int16_t min_rssi = 0;

    std::vector<uint8_t> video_frame;
    uint32_t video_frame_index = 0;
    uint8_t video_next_part_index = 0;

    struct RX_Data
    {
        std::array<uint8_t, AIR2GROUND_MTU> data;
        size_t size;
        int16_t rssi = 0;
    };

    RX_Data rx_data;

    while (true)
    {
        if (Clock::now() - last_stats_tp >= std::chrono::milliseconds(1000))
        {
            if (ping_count == 0)
            {
                ping_count = 0;
                ping_min = std::chrono::seconds(0);
                ping_max = std::chrono::seconds(0);
                ping_avg = std::chrono::seconds(0);
            }

            LOGI("Sent: {}, RX len: {}, RSSI: {}, Latency: {}/{}/{},vfps:{}", sent_count, total_data, min_rssi, 
                std::chrono::duration_cast<std::chrono::milliseconds>(ping_min).count(),
                std::chrono::duration_cast<std::chrono::milliseconds>(ping_max).count(),
                std::chrono::duration_cast<std::chrono::milliseconds>(ping_avg).count() / ping_count,video_fps);

            ping_min = std::chrono::seconds(999);
            ping_max = std::chrono::seconds(0);
            ping_avg = std::chrono::seconds(0);
            sent_count = 0;
            ping_count = 0;
            total_data = 0;
            min_rssi = 0;
            last_stats_tp = Clock::now();
        }

        if (Clock::now() - last_comms_sent_tp >= std::chrono::milliseconds(500))
        {
            std::lock_guard<std::mutex> lg(s_ground2air_config_packet_mutex);
            auto& config = s_ground2air_config_packet;
            config.ping = last_sent_ping; 
            config.type = Ground2Air_Header::Type::Config;
            config.size = sizeof(config);
            config.crc = 0;
            config.crc = crc8(0, &config, sizeof(config)); 
            s_comms.send(&config, sizeof(config), true);
            last_comms_sent_tp = Clock::now();
            last_ping_sent_tp = Clock::now();
            sent_count++;
        }

#ifdef TEST_LATENCY
        if (s_test_latency_gpio_value == 0 && Clock::now() - s_test_latency_gpio_last_tp >= std::chrono::milliseconds(200))
        {
            s_test_latency_gpio_value = 1;
            gpioWrite(17, s_test_latency_gpio_value);
            s_test_latency_gpio_last_tp = Clock::now();
#   ifdef TEST_DISPLAY_LATENCY
            s_decoder.inject_test_data(s_test_latency_gpio_value);
#   endif
        }
        if (s_test_latency_gpio_value != 0 && Clock::now() - s_test_latency_gpio_last_tp >= std::chrono::milliseconds(50))
        {
            s_test_latency_gpio_value = 0;
            gpioWrite(17, s_test_latency_gpio_value);
            s_test_latency_gpio_last_tp = Clock::now();
#   ifdef TEST_DISPLAY_LATENCY
            s_decoder.inject_test_data(s_test_latency_gpio_value);
#   endif
        }
#endif        

#ifdef TEST_DISPLAY_LATENCY
        std::this_thread::yield();

        //pump the comms to avoid packages accumulating
        s_comms.process();
        s_comms.receive(rx_data.data.data(), rx_data.size);
#else
        //receive new packets
        do
        {
            s_comms.process();
            if (!s_comms.receive(rx_data.data.data(), rx_data.size))
            {
                std::this_thread::yield();
                break;
            }

            rx_data.rssi = (int16_t)s_comms.get_input_dBm();

            //filter bad packets
            Air2Ground_Header& air2ground_header = *(Air2Ground_Header*)rx_data.data.data();
            if (air2ground_header.type != Air2Ground_Header::Type::Video)
            {
                LOGE("Unknown air packet: {}", air2ground_header.type);
                break;
            }

            uint32_t video_packet_size = air2ground_header.size;
            if (video_packet_size > rx_data.size)
            {
                LOGE("Video frame {}: data too big: {} > {}", video_frame_index, video_packet_size, rx_data.size);
                break;
            }

            if (video_packet_size < sizeof(Air2Ground_Video_Packet))
            {
                LOGE("Video frame {}: data too small: {} > {}", video_frame_index, video_packet_size, sizeof(Air2Ground_Video_Packet));
                break;
            }

            size_t payload_size = video_packet_size - sizeof(Air2Ground_Video_Packet);
            Air2Ground_Video_Packet& air2ground_video_packet = *(Air2Ground_Video_Packet*)rx_data.data.data();
            uint8_t crc = air2ground_video_packet.crc;
            air2ground_video_packet.crc = 0;
            uint8_t computed_crc = crc8(0, rx_data.data.data(), sizeof(Air2Ground_Video_Packet));
            if (crc != computed_crc)
            {
                LOGE("Video frame {}, {} {}: crc mismatch: {} != {}", air2ground_video_packet.frame_index, (int)air2ground_video_packet.part_index, payload_size, crc, computed_crc);
                break;
            }

            if (air2ground_video_packet.pong == last_sent_ping)
            {
                last_sent_ping++;
                auto d = (Clock::now() - last_ping_sent_tp) / 2;
                ping_min = std::min(ping_min, d);
                ping_max = std::max(ping_max, d);
                ping_avg += d;
                ping_count++;
            }

            total_data += rx_data.size;
            min_rssi = std::min(min_rssi, rx_data.rssi);
            //LOGI("OK Video frame {}, {} {} - CRC OK {}. {}", air2ground_video_packet.frame_index, (int)air2ground_video_packet.part_index, payload_size, crc, rx_queue.size());

            if ((air2ground_video_packet.frame_index + 200 < video_frame_index) ||                 //frame from the distant past? TX was restarted
                (air2ground_video_packet.frame_index > video_frame_index)) //frame from the future and we still have other frames enqueued? Stale data
            {
                //if (video_next_part_index > 0) //incomplete frame
                //   s_decoder.decode_data(video_frame.data(), video_frame.size());

                //if (video_next_part_index > 0)
                //    LOGE("Aborting video frame {}, {}", video_frame_index, video_next_part_index);

                video_frame.clear();
                video_frame_index = air2ground_video_packet.frame_index;
                video_next_part_index = 0;
            }
            if (air2ground_video_packet.frame_index == video_frame_index && air2ground_video_packet.part_index == video_next_part_index)
            {
                video_next_part_index++;
                size_t offset = video_frame.size();
                video_frame.resize(offset + payload_size);
                memcpy(video_frame.data() + offset, rx_data.data.data() + sizeof(Air2Ground_Video_Packet), payload_size);

                if (video_next_part_index > 0 && air2ground_video_packet.last_part != 0)
                {
                    //LOGI("Received frame {}, {}, size {}", video_frame_index, video_next_part_index, video_frame.size());
                    s_decoder.decode_data(video_frame.data(), video_frame.size());
                    video_next_part_index = 0;
                    video_frame.clear();
                }
            }
        } 
        while (false);
#endif
    }
}

static inline ImVec2 operator+(const ImVec2& lhs, const ImVec2& rhs)
{
    return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y);
}
static inline ImVec2 ImRotate(const ImVec2& v, float cos_a, float sin_a)
{
    return ImVec2(v.x * cos_a - v.y * sin_a, v.x * sin_a + v.y * cos_a);
}
void ImageRotated(ImTextureID tex_id, ImVec2 center, ImVec2 size, float angle, float uvAngle)
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    ImVec2 pos[4] =
        {
            center + ImRotate(ImVec2(-size.x * 0.5f, -size.y * 0.5f), cos_a, sin_a),
            center + ImRotate(ImVec2(+size.x * 0.5f, -size.y * 0.5f), cos_a, sin_a),
            center + ImRotate(ImVec2(+size.x * 0.5f, +size.y * 0.5f), cos_a, sin_a),
            center + ImRotate(ImVec2(-size.x * 0.5f, +size.y * 0.5f), cos_a, sin_a)};

    cos_a = cosf(uvAngle);
    sin_a = sinf(uvAngle);
    ImVec2 uvCenter(0.5f, 0.5f);
    ImVec2 uvs[4] =
        {
            uvCenter + ImRotate(ImVec2(-0.5f, -0.5f), cos_a, sin_a),
            uvCenter + ImRotate(ImVec2(+0.5f, -0.5f), cos_a, sin_a),
            uvCenter + ImRotate(ImVec2(+0.5f, +0.5f), cos_a, sin_a),
            uvCenter + ImRotate(ImVec2(-0.5f, +0.5f), cos_a, sin_a)};

    draw_list->AddImageQuad(tex_id, pos[0], pos[1], pos[2], pos[3], uvs[0], uvs[1], uvs[2], uvs[3], IM_COL32_WHITE);
}

int run()
{
    HUD hud(*s_hal);


    ImGuiIO& io = ImGui::GetIO();


    s_decoder.init(*s_hal);

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    s_comms_thread = std::thread(&comms_thread_proc);

    Ground2Air_Config_Packet config;
    config.wifi_rate = WIFI_Rate::RATE_G_54M_ODFM;//RATE_G_18M_ODFM;

    config.camera.resolution = Resolution::SVGA;
    config.camera.fps_limit = 0;
    config.camera.quality = 63;

    size_t video_frame_count = 0;


    Clock::time_point last_stats_tp = Clock::now();
    Clock::time_point last_tp = Clock::now();
    while (true)
    {
        s_decoder.unlock_output();
        size_t count = s_decoder.lock_output();
        video_frame_count += count;
        for (size_t i = 0; i < 3; i++)
            s_hal->set_video_channel(i,s_decoder.get_video_texture_id(i));
           // ImGui_SetVideoTextureChannel(i, s_decoder.get_video_texture_id(i));

        s_hal->process();

        if (Clock::now() - last_stats_tp >= std::chrono::milliseconds(1000))
        {
            last_stats_tp = Clock::now();
            video_fps = video_frame_count;
            video_frame_count = 0;
        }

        Clock::time_point now = Clock::now();
        Clock::duration dt = now - last_tp;
        last_tp = now;
        io.DeltaTime = std::chrono::duration_cast<std::chrono::duration<float> >(dt).count();

    }

    return 0;
}

int main(int argc, const char* argv[])
{
    init_crc8_table();

    s_hal.reset(new PI_HAL());
    if (!s_hal->init())
        return -1;

#ifdef TEST_LATENCY
    gpioSetMode(17, PI_OUTPUT);
#endif

    Comms::RX_Descriptor rx_descriptor;
    rx_descriptor.coding_k = s_ground2air_config_packet.fec_codec_k;
    rx_descriptor.coding_n = s_ground2air_config_packet.fec_codec_n;
    rx_descriptor.mtu = s_ground2air_config_packet.fec_codec_mtu;
    rx_descriptor.interfaces = {"wlx00127b22ac39"};
    Comms::TX_Descriptor tx_descriptor;
    tx_descriptor.coding_k = 2;
    tx_descriptor.coding_n = 6;
    tx_descriptor.mtu = GROUND2AIR_DATA_MAX_SIZE;
    tx_descriptor.interface = "wlx00127b22ac39";
    if (!s_comms.init(rx_descriptor, tx_descriptor))
        return -1;

    for (const auto& itf: rx_descriptor.interfaces)
    {
        system(fmt::format("iwconfig {} channel 11", itf).c_str());
    }

    int result = run();

    s_hal->shutdown();

    return result;
}