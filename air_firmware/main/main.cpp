//#include <Arduino.h>
#include <algorithm>

#include "esp_camera.h"
//#include "EEPROM.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "esp_wifi_types.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
//#include "esp_wifi_internal.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_private/wifi.h"
#include "esp_task_wdt.h"
//#include "bt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "fec_codec.h"
#include "packets.h"
#include "safe_printf.h"
#include "structures.h"
#include "crc.h"
#include "driver/gpio.h"
#include "main.h"
#include "queue.h"
#include "circular_buffer.h"

#include "ll_cam.h" // cam_obj_t defination, used in camera_data_available

#include "wifi.h"
#include "nvs_args.h"

uint16_t g_wifi_channel;
static int s_stats_last_tp = -10000;



/////////////////////////////////////////////////////////////////////////

static size_t s_video_frame_data_size = 0;
static uint32_t s_video_frame_index = 0;
static uint8_t s_video_part_index = 0;
static bool s_video_frame_started = false;

static int64_t s_video_last_sent_tp = esp_timer_get_time();
static int64_t s_video_last_acquired_tp = esp_timer_get_time();
static bool s_video_skip_frame = false;
static int64_t s_video_target_frame_dt = 0;

/////////////////////////////////////////////////////////////////////////

static int s_uart_verbose = 1;

#define LOG(...) do { if (s_uart_verbose > 0) SAFE_PRINTF(__VA_ARGS__); } while (false) 

/////////////////////////////////////////////////////////////////////////

static constexpr gpio_num_t STATUS_LED_PIN = GPIO_NUM_33;
static constexpr uint8_t STATUS_LED_ON = 0;
static constexpr uint8_t STATUS_LED_OFF = 1;

void initialize_status_led()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << STATUS_LED_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
}

IRAM_ATTR uint64_t micros()
{
    return esp_timer_get_time();
}

IRAM_ATTR uint64_t millis()
{
    return esp_timer_get_time() / 1000ULL;
}

IRAM_ATTR void set_status_led_on()
{
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
}

IRAM_ATTR void update_status_led()
{
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
}

//////////////////////////////////////////////////////////////////////////////
static bool s_recv_ground2air_packet = false;
#ifdef DVR_SUPPORT

SemaphoreHandle_t s_sd_fast_buffer_mux = xSemaphoreCreateBinary();
SemaphoreHandle_t s_sd_slow_buffer_mux = xSemaphoreCreateBinary();

auto _init_result = []() -> bool
{
  xSemaphoreGive(s_sd_fast_buffer_mux);
  xSemaphoreGive(s_sd_slow_buffer_mux);
  return true;
}();

////////////////////////////////////////////////////////////////////////////////////

static TaskHandle_t s_sd_write_task = nullptr;
static TaskHandle_t s_sd_enqueue_task = nullptr;
static TaskHandle_t s_file_server_task = nullptr;
static bool s_sd_initialized = false;
static size_t s_sd_file_size = 0;
static uint32_t s_sd_next_session_id = 0;
static uint32_t s_sd_next_segment_id = 0;


//the fast buffer is RAM and used to transfer data quickly from the camera callback to the slow, SPIRAM buffer. 
//Writing directly to the SPIRAM buffer is too slow in the camera callback and causes lost frames, so I use this RAM buffer and a separate task (sd_enqueue_task) for that.
static constexpr size_t SD_FAST_BUFFER_SIZE = 10000;
Circular_Buffer s_sd_fast_buffer(new uint8_t[SD_FAST_BUFFER_SIZE], SD_FAST_BUFFER_SIZE);

//this slow buffer is used to buffer data that is about to be written to SD. The reason it's this big is because SD card write speed fluctuated a lot and 
// sometimes it pauses for a few hundred ms. So to avoid lost data, I have to buffer it into a big enoigh buffer.
//The data is written to the sd card by the sd_write_task, in chunks of SD_WRITE_BLOCK_SIZE.
static constexpr size_t SD_SLOW_BUFFER_SIZE = 3 * 1024 * 1024;
Circular_Buffer s_sd_slow_buffer((uint8_t*)heap_caps_malloc(SD_SLOW_BUFFER_SIZE, MALLOC_CAP_SPIRAM), SD_SLOW_BUFFER_SIZE);

//Cannot write to SD directly from the slow, SPIRAM buffer as that causes the write speed to plummet. So instead I read from the slow buffer into
// this RAM block and write from it directly. This results in several MB/s write speed performance which is good enough.
static constexpr size_t SD_WRITE_BLOCK_SIZE = 8192;


static void shutdown_sd()
{
    if (!s_sd_initialized)
        return;
    LOG("close sd card!\n");
    esp_vfs_fat_sdmmc_unmount();
    s_sd_initialized = false;

    //to turn the LED off
    gpio_set_pull_mode((gpio_num_t)4, GPIO_PULLDOWN_ONLY);    // D1, needed in 4-line mode only
}

static bool init_sd()
{
    if (s_sd_initialized)
        return true;

    sdmmc_card_t* card = nullptr;

    esp_vfs_fat_sdmmc_mount_config_t mount_config;
#ifdef CAMERA_MODEL_ESP_VTX
    mount_config.format_if_mount_failed = true;
#else
    mount_config.format_if_mount_failed = false;
#endif
    mount_config.max_files = 2;
    mount_config.allocation_unit_size = 0;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    //host.max_freq_khz = SDMMC_FREQ_PROBING;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    //host.flags = SDMMC_HOST_FLAG_1BIT;

    gpio_set_pull_mode((gpio_num_t)14, GPIO_PULLUP_ONLY);   // CLK, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode((gpio_num_t)2, GPIO_PULLUP_ONLY);    // D0, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    //slot_config.width = 1;

    LOG("Mounting SD card...\n");
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        LOG("Failed to mount SD card VFAT filesystem. Error: %s\n", esp_err_to_name(ret));
        //to turn the LED off
        gpio_set_pull_mode((gpio_num_t)4, GPIO_PULLDOWN_ONLY);    // D1, needed in 4-line mode only
        return false;
    }

    LOG("sd card inited!\n");
    s_sd_initialized = true;

    //find the latest file number
    char buffer[64];
    for (uint32_t i = 0; i < 100000; i++)
    {
        sprintf(buffer, "/sdcard/session%03d_segment000.mjpeg", i);
        FILE* f = fopen(buffer, "rb");
        if (f)
        {
            fclose(f);
            continue;
        }
        s_sd_next_session_id = i;
        s_sd_next_segment_id = 0;
        break;
    }
    return true;
}

static FILE* open_sd_file()
{
    char buffer[64];
    sprintf(buffer, "/sdcard/session%03d_segment%03d.mjpeg", s_sd_next_session_id, s_sd_next_segment_id);
    FILE* f = fopen(buffer, "wb");
    if (!f){
        LOG("error to open sdcard session %s!\n",buffer);
        return nullptr;
    }

    LOG("Opening session file '%s'\n", buffer);
    s_sd_file_size = 0;
    s_sd_next_segment_id++;

    return f;
}

//this will write data from the slow queue to file
static void sd_write_proc(void*)
{
    uint8_t* block = new uint8_t[SD_WRITE_BLOCK_SIZE];

    while (true)
    {
        if (!s_ground2air_config_packet.dvr_record)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        

        FILE* f = open_sd_file();
        if (!f)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS); 
            continue;
        }

        xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
        s_sd_slow_buffer.clear();
        xSemaphoreGive(s_sd_slow_buffer_mux);

        bool error = false; 
        bool done = false;
        while (!done)
        {
            ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS); //wait for notification

            while (true) //consume all the buffer
            {
                if (!s_ground2air_config_packet.dvr_record)
                {
                    LOG("Done recording, closing file\n", s_sd_file_size);
                    done = true;
                    break;
                }

                xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
                bool read = s_sd_slow_buffer.read(block, SD_WRITE_BLOCK_SIZE);
                xSemaphoreGive(s_sd_slow_buffer_mux);
                if (!read)
                    break; //not enough data, wait

                if (fwrite(block, SD_WRITE_BLOCK_SIZE, 1, f) == 0)
                {
                    LOG("Error while writing! Stopping session\n");
                    done = true;
                    error = true;
                    break;
                }
                s_stats.sd_data += SD_WRITE_BLOCK_SIZE;
                s_sd_file_size += SD_WRITE_BLOCK_SIZE;
                if (s_sd_file_size > 500 * 1024 * 1024)
                {
                    LOG("Max file size reached: %d. Restarting session\n", s_sd_file_size);
                    done = true;
                    break;
                }
            }
        }

        if (!error)
        {
            fflush(f);
            fsync(fileno(f));
            fclose(f);
        }else{
            shutdown_sd();
        }

    }
}

//this will move data from the fast queue to the slow queue
static void sd_enqueue_proc(void*)
{
    while (true)
    {
        if (!s_ground2air_config_packet.dvr_record)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
        s_sd_fast_buffer.clear();
        xSemaphoreGive(s_sd_fast_buffer_mux);

        while (true)
        {
            ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS); //wait for notification

            xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
            size_t size = s_sd_fast_buffer.size();
            if (size == 0)
            {
                xSemaphoreGive(s_sd_fast_buffer_mux);
                continue; //no data? wait some more
            }

            const void* buffer = s_sd_fast_buffer.start_reading(size);
            xSemaphoreGive(s_sd_fast_buffer_mux);

            xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
            s_sd_slow_buffer.write(buffer, size);
            xSemaphoreGive(s_sd_slow_buffer_mux);

            xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
            s_sd_fast_buffer.end_reading(size);
            xSemaphoreGive(s_sd_fast_buffer_mux);

            if (s_sd_write_task)
                xTaskNotifyGive(s_sd_write_task); //notify task

            if (!s_ground2air_config_packet.dvr_record)
                break;
        }
    }
}


IRAM_ATTR static void add_to_sd_fast_buffer(const void* data, size_t size)
{
    xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
    bool ok = s_sd_fast_buffer.write(data, size);
    xSemaphoreGive(s_sd_fast_buffer_mux);
    if (ok)
    {
        if (s_sd_enqueue_task)
            xTaskNotifyGive(s_sd_enqueue_task); //notify task
    }
    else
        s_stats.sd_drops += size;
}

#endif
/////////////////////////////////////////////////////////////////////

int16_t s_wlan_incoming_rssi = 0; //this is protected by the s_wlan_incoming_mux

///////////////////////////////////////////////////////////////////////////////////////////

IRAM_ATTR void packet_received_cb(void* buf, wifi_promiscuous_pkt_type_t type)
{
    if (type == WIFI_PKT_MGMT)
    {
        //LOG("management packet\n");
        return;
    }
    else if (type == WIFI_PKT_DATA)
    {
        //LOG("data packet\n");
    }
    else if (type == WIFI_PKT_MISC)
    {
        //LOG("misc packet\n");
        return;
    }

    wifi_promiscuous_pkt_t *pkt = reinterpret_cast<wifi_promiscuous_pkt_t*>(buf);

    uint16_t len = pkt->rx_ctrl.sig_len;
    //s_stats.wlan_data_received += len;
    //s_stats.wlan_data_sent += 1;

    if (len <= WLAN_IEEE_HEADER_SIZE)
        return;

    //LOG("Recv %d bytes\n", len);
    //LOG("Channel: %d\n", (int)pkt->rx_ctrl.channel);

    //uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    //LOG("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    uint8_t *data = pkt->payload;
    if (memcmp(data + 10, WLAN_IEEE_HEADER_GROUND2AIR + 10, 6) != 0)
        return;

    if (len <= WLAN_IEEE_HEADER_SIZE)
    {
        LOG("WLAN receive header error");
        s_stats.wlan_error_count++;
        return;
    }

    data += WLAN_IEEE_HEADER_SIZE;
    len -= WLAN_IEEE_HEADER_SIZE; //skip the 802.11 header

    len -= 4; //the received length has 4 more bytes at the end for some reason.

    int16_t rssi = pkt->rx_ctrl.rssi;

    size_t size = std::min<size_t>(len, WLAN_MAX_PAYLOAD_SIZE);

    xSemaphoreTake(s_wlan_incoming_mux, portMAX_DELAY);
    s_wlan_incoming_rssi = rssi;
    xSemaphoreGive(s_wlan_incoming_mux);

    s_fec_decoder.lock();
    if (!s_fec_decoder.decode_data(data, size, false))
        s_stats.wlan_received_packets_dropped++;
    s_fec_decoder.unlock();

    s_stats.wlan_data_received += len;
}

/////////////////////////////////////////////////////////////////////////

static void handle_ground2air_config_packet(Ground2Air_Config_Packet& src)
{
    Ground2Air_Config_Packet& dst = s_ground2air_config_packet;
    s_recv_ground2air_packet = true;
    if (dst.wifi_rate != src.wifi_rate)
    {
        LOG("Wifi rate changed from %d to %d\n", (int)dst.wifi_rate, (int)src.wifi_rate);
        ESP_ERROR_CHECK(set_wifi_fixed_rate(src.wifi_rate));
    }
    if (dst.wifi_power != src.wifi_power)
    {
        LOG("Wifi power changed from %d to %d\n", (int)dst.wifi_power, (int)src.wifi_power);
        ESP_ERROR_CHECK(set_wlan_power_dBm(src.wifi_power));
    }
    if (dst.fec_codec_k != src.fec_codec_k || dst.fec_codec_n != src.fec_codec_n || dst.fec_codec_mtu != src.fec_codec_mtu)
    {
        LOG("FEC codec changed from %d/%d/%d to %d/%d/%d\n", (int)dst.fec_codec_k, (int)dst.fec_codec_n, (int)dst.fec_codec_mtu, (int)src.fec_codec_k, (int)src.fec_codec_n, (int)src.fec_codec_mtu);
        {
            init_fec_codec(s_fec_encoder,src.fec_codec_k,src.fec_codec_n,src.fec_codec_mtu,true);
        }
    }
    if (dst.camera.resolution != src.camera.resolution)
    {
        LOG("Camera resolution changed from %d to %d\n", (int)dst.camera.resolution, (int)src.camera.resolution);
        sensor_t* s = esp_camera_sensor_get();
        switch (src.camera.resolution)
        {
            case Resolution::QVGA: s->set_framesize(s, FRAMESIZE_QVGA); break;
            case Resolution::CIF: s->set_framesize(s, FRAMESIZE_CIF); break;
            case Resolution::HVGA: s->set_framesize(s, FRAMESIZE_HVGA); break;
            case Resolution::VGA: s->set_framesize(s, FRAMESIZE_VGA); break;
            case Resolution::SVGA: s->set_framesize(s, FRAMESIZE_SVGA); break;
            case Resolution::XGA: s->set_framesize(s, FRAMESIZE_XGA); break;
            case Resolution::SXGA: s->set_framesize(s, FRAMESIZE_SXGA); break;
            case Resolution::UXGA: s->set_framesize(s, FRAMESIZE_UXGA); break;
        }
    }
    if (dst.camera.fps_limit != src.camera.fps_limit)
    {
        if (src.camera.fps_limit == 0)
            s_video_target_frame_dt = 0;
        else
            s_video_target_frame_dt = 1000000 / src.camera.fps_limit;
        LOG("Target FPS changed from %d to %d\n", (int)dst.camera.fps_limit, (int)src.camera.fps_limit);
    }

#define APPLY(n1, n2, type) \
    if (dst.camera.n1 != src.camera.n1) \
    { \
        LOG("Camera " #n1 " from %d to %d\n", (int)dst.camera.n1, (int)src.camera.n1); \
        sensor_t* s = esp_camera_sensor_get(); \
        s->set_##n2(s, (type)src.camera.n1); \
    }
    APPLY(quality, quality, int);
    APPLY(brightness, brightness, int);
    APPLY(contrast, contrast, int);
    APPLY(saturation, saturation, int);
    APPLY(sharpness, sharpness, int);
    APPLY(denoise, denoise, int);
    APPLY(gainceiling, gainceiling, gainceiling_t);
    APPLY(awb, whitebal, int);
    APPLY(awb_gain, awb_gain, int);
    APPLY(wb_mode, wb_mode, int);
    APPLY(agc, gain_ctrl, int);
    APPLY(agc_gain, agc_gain, int);
    APPLY(aec, exposure_ctrl, int);
    APPLY(aec_value, aec_value, int);
    APPLY(aec2, aec2, int);
    APPLY(ae_level, ae_level, int);
    APPLY(hmirror, hmirror, int);
    APPLY(vflip, vflip, int);
    APPLY(special_effect, special_effect, int);
    APPLY(dcw, dcw, int);
    APPLY(bpc, bpc, int);
    APPLY(wpc, wpc, int);
    APPLY(raw_gma, raw_gma, int);
    APPLY(lenc, lenc, int);
#undef APPLY

    dst = src;
}


IRAM_ATTR void send_air2ground_video_packet(bool last)
{
    if (last)
        s_stats.video_frames++;
    s_stats.video_data += s_video_frame_data_size;

    uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);

    if(!packet_data){
        LOG("no data buf!\n");
        return ;
    }

    Air2Ground_Video_Packet& packet = *(Air2Ground_Video_Packet*)packet_data;
    packet.type = Air2Ground_Header::Type::Video;
    packet.resolution = s_ground2air_config_packet.camera.resolution;
    packet.frame_index = s_video_frame_index;
    packet.part_index = s_video_part_index;
    packet.last_part = last ? 1 : 0;
    packet.size = s_video_frame_data_size + sizeof(Air2Ground_Video_Packet);
    packet.pong = s_ground2air_config_packet.ping;
    packet.crc = 0;
    packet.crc = crc8(0, &packet, sizeof(Air2Ground_Video_Packet));
    if (!s_fec_encoder.flush_encode_packet(true))
    {
        LOG("Fec codec busy\n");
        s_stats.wlan_error_count++;
    }
}

constexpr size_t PAYLOAD_SIZE = AIR2GROUND_MTU - sizeof(Air2Ground_Video_Packet);

IRAM_ATTR size_t camera_data_available(void * cam_obj,const uint8_t* data, size_t count, bool last)
{
    size_t stride=((cam_obj_t *)cam_obj)->dma_bytes_per_item;

    s_fec_encoder.lock();
    if (data == nullptr) //start frame
    {
        s_video_frame_started = true;        
    }
    else 
    {
        if (!s_video_skip_frame)
        {
            const uint8_t* src = (const uint8_t*)data + 2;  // jump to the sample1 of DMA element
            count /= stride;
            if (last) //find the end marker for JPEG. Data after that can be discarded
            {
                const uint8_t* dptr = src + (count - 2) * stride;
                while (dptr > src)
                {
                    if (dptr[0] == 0xFF && dptr[stride] == 0xD9)
                    {
                        count = (dptr - src) / stride + 2; //to include the 0xFFD9
                        if ((count & 0x1FF) == 0)
                            count += 1; 
                        if ((count % 100) == 0)
                            count += 1;
                        break;
                    }
                    dptr -= stride;
                }

            }

            while (count > 0)
            {
                if (s_video_frame_data_size >= PAYLOAD_SIZE) //flush prev data?
                {
                    //LOG("Flush: %d %d\n", s_video_frame_index, s_video_frame_data_size);
                    send_air2ground_video_packet(false);
                    s_video_frame_data_size = 0;
                    s_video_part_index++;
                }

                //LOG("Add: %d %d %d %d\n", s_video_frame_index, s_video_part_index, count, s_video_frame_data_size);

                //fill the buffer
                uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);
                uint8_t* start_ptr = packet_data + sizeof(Air2Ground_Video_Packet) + s_video_frame_data_size;
                uint8_t* ptr = start_ptr;
                size_t c = std::min(PAYLOAD_SIZE - s_video_frame_data_size, count);

                count -= c;
                s_video_frame_data_size += c;

                size_t c8 = c >> 3;
                for (size_t i = c8; i > 0; i--)
                {
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                }
                for (size_t i = c - (c8 << 3); i > 0; i--)
                {
                    *ptr++ = *src; src += stride;
                }
#ifdef DVR_SUPPORT
                if (s_ground2air_config_packet.dvr_record)
                    add_to_sd_fast_buffer(start_ptr, c);
#endif
            }
        }

        //////////////////

        if (last && s_video_frame_started)
        {
            s_video_frame_started = false;

            //frame pacing!
            int64_t now = esp_timer_get_time();

            int64_t acquire_dt = now - s_video_last_acquired_tp;
            s_video_last_acquired_tp = now;

            int64_t send_dt = now - s_video_last_sent_tp;
            if (send_dt < s_video_target_frame_dt) //limit fps
                s_video_skip_frame = true;
            else                
            {
                s_video_skip_frame = false;
                s_video_last_sent_tp += std::max(s_video_target_frame_dt, acquire_dt);
            }

            //////////////////

            //LOG("Finish: %d %d\n", s_video_frame_index, s_video_frame_data_size);
            if (s_video_frame_data_size > 0) //left over
                send_air2ground_video_packet(true);

            s_video_frame_data_size = 0;
            s_video_frame_index++;
            s_video_part_index = 0;
        }
    }

    s_fec_encoder.unlock();
    return count;
}

static void init_camera()
{
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 12000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 4;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.data_available_callback = camera_data_available;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        LOG("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_HQVGA);
    config.jpeg_quality = 12;
    s->set_saturation(s, 0);
}

//#define SHOW_CPU_USAGE

static void print_cpu_usage()
{
#ifdef SHOW_CPU_USAGE
    TaskStatus_t* pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime, ulStatsAsPercentage;

    // Take a snapshot of the number of tasks in case it changes while this
    // function is executing.
    uxArraySize = uxTaskGetNumberOfTasks();
    //LOG("%u tasks\n", uxArraySize);

    // Allocate a TaskStatus_t structure for each task.  An array could be
    // allocated statically at compile time.
    pxTaskStatusArray = (TaskStatus_t*)heap_caps_malloc(uxArraySize * sizeof(TaskStatus_t), MALLOC_CAP_SPIRAM);

    if (pxTaskStatusArray != NULL)
    {
        // Generate raw status information about each task.
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
        //LOG("%u total usage\n", ulTotalRunTime);

        // For percentage calculations.
        ulTotalRunTime /= 100UL;

        // Avoid divide by zero errors.
        if (ulTotalRunTime > 0)
        {
            // For each populated position in the pxTaskStatusArray array,
            // format the raw data as human readable ASCII data
            for (x = 0; x < uxArraySize; x++)
            {
                // What percentage of the total run time has the task used?
                // This will always be rounded down to the nearest integer.
                // ulTotalRunTimeDiv100 has already been divided by 100.
                ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

                if (ulStatsAsPercentage > 0UL)
                {
                    LOG("%s\t\t%u\t\t%u%%\r\n", pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter, ulStatsAsPercentage);
                }
                else
                {
                    // If the percentage is zero here then the task has
                    // consumed less than 1% of the total run time.
                    LOG("%s\t\t%u\t\t<1%%\r\n", pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter);
                }
            }
        }

        // The array is no longer needed, free the memory it consumes.
        free(pxTaskStatusArray);
    }
#endif
}


extern "C" void app_main()
{
    Ground2Air_Data_Packet& ground2air_data_packet = s_ground2air_data_packet;
    ground2air_data_packet.type = Ground2Air_Header::Type::Data;
    ground2air_data_packet.size = sizeof(ground2air_data_packet);

    Ground2Air_Config_Packet& ground2air_config_packet = s_ground2air_config_packet;
    ground2air_config_packet.type = Ground2Air_Header::Type::Config;
    ground2air_config_packet.size = sizeof(ground2air_config_packet);
    ground2air_config_packet.wifi_rate = WIFI_Rate::RATE_G_54M_ODFM;

    srand(esp_timer_get_time());

    printf("Initializing...\n");

    printf("MEMORY at start: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);

    nvs_args_init();
    g_wifi_channel = (uint16_t)nvs_args_read("channel");
    if(g_wifi_channel > 13){
        g_wifi_channel = DEFAULT_WIFI_CHANNEL;
        nvs_args_set("channel", g_wifi_channel);
        LOG("could not find the nvs store variable:channel, set wifi channel to default %d", g_wifi_channel);
    }

    initialize_status_led();

    setup_wifi(s_ground2air_config_packet.wifi_rate, g_wifi_channel, 20.0f, packet_received_cb);
    set_ground2air_config_packet_handler(handle_ground2air_config_packet);

    #ifdef DVR_SUPPORT
    init_sd();
    
    auto a = [](void * arg){
        int wait_seconds = 10;
        while(wait_seconds--){
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            LOG("wait for ground 2 air packet..");
         //   if(s_recv_ground2air_packet)
         {
                break;
            }
        }

        if(wait_seconds < 0){
            LOG("no ground 2 air packet for 10 seconds. setup file server...");
            vTaskSuspend(s_wifi_rx_task);
            vTaskSuspend(s_wifi_tx_task);
            setup_wifi_file_server();
        }
        vTaskDelete(NULL);
    };

    xTaskCreatePinnedToCore((TaskFunction_t)a, "file server", 4096, nullptr, 1, &s_file_server_task, tskNO_AFFINITY);

    {
        int core = tskNO_AFFINITY;
        BaseType_t res = xTaskCreatePinnedToCore(&sd_write_proc, "SD Write", 4096, nullptr, 1, &s_sd_write_task, core);
        if (res != pdPASS)
            LOG("Failed sd write task: %d\n", res);
    }
    {
        int core = tskNO_AFFINITY;
        BaseType_t res = xTaskCreatePinnedToCore(&sd_enqueue_proc, "SD Enq", 1536, nullptr, 1, &s_sd_enqueue_task, core);
        if (res != pdPASS)
            LOG("Failed sd enqueue task: %d\n", res);
    }
    #endif

    init_camera();
    printf("MEMORY Before Loop: \n");
    heap_caps_print_heap_info(MALLOC_CAP_8BIT);

    while (true)
    {
        if (s_uart_verbose > 0 && millis() - s_stats_last_tp >= 1000)
        {
            s_stats_last_tp = millis();
            LOG("WLAN S: %d, R: %d, E: %d, D: %d, %%: %d || FPS: %d, D: %d || D: %d, E: %d\n",
                s_stats.wlan_data_sent, s_stats.wlan_data_received, s_stats.wlan_error_count, s_stats.wlan_received_packets_dropped, s_wlan_outgoing_queue.size() * 100 / s_wlan_outgoing_queue.capacity(),
                (int)s_stats.video_frames, s_stats.video_data, s_stats.sd_data, s_stats.sd_drops);
            print_cpu_usage();

            s_stats = Stats();
        }

        vTaskDelay(10);
        esp_task_wdt_reset();

        update_status_led();
    }
}
