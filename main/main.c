#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_camera.h"
#include "img_converters.h"

#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "driver/i2s_pdm.h"

static const char *TAG = "CAM_SD";

typedef struct {
    FILE *f;
    size_t written;
} jpg_write_ctx_t;

static size_t jpg_write_cb(void *arg, size_t index, const void *data, size_t len)
{
    (void)index;
    jpg_write_ctx_t *ctx = (jpg_write_ctx_t *)arg;
    size_t w = fwrite(data, 1, len, ctx->f);
    ctx->written += w;
    return w;
}

// -------- Camera pins for Seeed XIAO ESP32S3 Sense --------
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39

#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// -------- SD card pins on Sense expansion board (SPI2) ----
#define SD_MOSI_GPIO   9
#define SD_MISO_GPIO   8
#define SD_SCLK_GPIO   7
#define SD_CS_GPIO     21

// -------- PDM microphone pins for Seeed XIAO ESP32S3 Sense --------
// Verify these with your board revision and adjust if needed.
#define MIC_PDM_CLK_GPIO  42
#define MIC_PDM_DIN_GPIO  41

#define MIC_SAMPLE_RATE_HZ 16000
#define MIC_BITS_PER_SAMPLE 16
#define MIC_CHANNELS 1

static sdmmc_card_t *sd_card = NULL;
static i2s_chan_handle_t mic_rx_handle = NULL;

typedef struct __attribute__((packed)) {
    char riff_id[4];
    uint32_t riff_size;
    char wave_id[4];
    char fmt_id[4];
    uint32_t fmt_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char data_id[4];
    uint32_t data_size;
} wav_header_t;

// ---------- SD card init over SPI (SDSPI) ----------
static esp_err_t init_sdcard(void)
{
    esp_err_t ret;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 10000; // 10 MHz for better card/reader stability

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_GPIO;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config,
                                  &mount_config, &sd_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_card_print_info(stdout, sd_card);
    ESP_LOGI(TAG, "SD card mounted at /sdcard");
    return ESP_OK;
}

// ---------- Camera init ----------
static esp_err_t init_camera(void)
{
    camera_config_t config = {
        .pin_pwdn     = PWDN_GPIO_NUM,
        .pin_reset    = RESET_GPIO_NUM,
        .pin_xclk     = XCLK_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,

        .pin_d7       = Y9_GPIO_NUM,
        .pin_d6       = Y8_GPIO_NUM,
        .pin_d5       = Y7_GPIO_NUM,
        .pin_d4       = Y6_GPIO_NUM,
        .pin_d3       = Y5_GPIO_NUM,
        .pin_d2       = Y4_GPIO_NUM,
        .pin_d1       = Y3_GPIO_NUM,
        .pin_d0       = Y2_GPIO_NUM,

        .pin_vsync    = VSYNC_GPIO_NUM,
        .pin_href     = HREF_GPIO_NUM,
        .pin_pclk     = PCLK_GPIO_NUM,

        // Use standard XCLK for OV3660; reduce only if unstable
        .xclk_freq_hz = 16000000,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        // Capture RAW and encode in software for stability
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size   = FRAMESIZE_SVGA,   // 800x600; reduce to VGA if needed
        .jpeg_quality = 12,               // unused for RGB565
        .fb_count     = 2,                // double buffer for smoother capture
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,   // block until buffer free
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Camera init done");
    return ESP_OK;
}

// ---------- Microphone init (PDM over I2S) ----------
static esp_err_t init_mic(void)
{
    if (MIC_PDM_CLK_GPIO < 0 || MIC_PDM_DIN_GPIO < 0) {
        ESP_LOGW(TAG, "Mic pins not set, skipping mic init");
        return ESP_OK;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &mic_rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to alloc I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = MIC_PDM_CLK_GPIO,
            .din = MIC_PDM_DIN_GPIO,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    ret = i2s_channel_init_pdm_rx_mode(mic_rx_handle, &pdm_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init PDM RX: %s", esp_err_to_name(ret));
        i2s_del_channel(mic_rx_handle);
        mic_rx_handle = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "Mic init done");
    return ESP_OK;
}

static void write_wav_header(FILE *f, uint32_t data_bytes)
{
    wav_header_t hdr = {
        .riff_id = { 'R', 'I', 'F', 'F' },
        .riff_size = 36 + data_bytes,
        .wave_id = { 'W', 'A', 'V', 'E' },
        .fmt_id = { 'f', 'm', 't', ' ' },
        .fmt_size = 16,
        .audio_format = 1,
        .num_channels = MIC_CHANNELS,
        .sample_rate = MIC_SAMPLE_RATE_HZ,
        .byte_rate = MIC_SAMPLE_RATE_HZ * MIC_CHANNELS * (MIC_BITS_PER_SAMPLE / 8),
        .block_align = MIC_CHANNELS * (MIC_BITS_PER_SAMPLE / 8),
        .bits_per_sample = MIC_BITS_PER_SAMPLE,
        .data_id = { 'd', 'a', 't', 'a' },
        .data_size = data_bytes,
    };

    fseek(f, 0, SEEK_SET);
    fwrite(&hdr, 1, sizeof(hdr), f);
}

// ---------- Record audio and save as /sdcard/<name>.wav ----------
static esp_err_t record_and_save_wav(const char *filename, uint32_t duration_ms)
{
    if (!mic_rx_handle) {
        ESP_LOGW(TAG, "Mic not initialized");
        return ESP_FAIL;
    }

    char path[64];
    snprintf(path, sizeof(path), "/sdcard/%s", filename);

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open audio file for writing");
        return ESP_FAIL;
    }

    // Placeholder header; we'll update sizes after capture.
    write_wav_header(f, 0);

    esp_err_t ret = i2s_channel_enable(mic_rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S RX: %s", esp_err_to_name(ret));
        fclose(f);
        return ret;
    }

    const uint32_t bytes_per_sample = (MIC_BITS_PER_SAMPLE / 8) * MIC_CHANNELS;
    const uint32_t total_bytes = (MIC_SAMPLE_RATE_HZ * bytes_per_sample * duration_ms) / 1000;
    uint32_t bytes_written = 0;

    int16_t buffer[512];
    const size_t buffer_bytes = sizeof(buffer);

    while (bytes_written < total_bytes) {
        size_t bytes_to_read = buffer_bytes;
        if (total_bytes - bytes_written < buffer_bytes) {
            bytes_to_read = total_bytes - bytes_written;
        }

        size_t bytes_read = 0;
        ret = i2s_channel_read(mic_rx_handle, buffer, bytes_to_read, &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S read failed: %s", esp_err_to_name(ret));
            break;
        }

        size_t w = fwrite(buffer, 1, bytes_read, f);
        bytes_written += w;
        if (w != bytes_read) {
            ESP_LOGE(TAG, "Audio write failed");
            ret = ESP_FAIL;
            break;
        }
    }

    i2s_channel_disable(mic_rx_handle);

    if (ret == ESP_OK) {
        write_wav_header(f, bytes_written);
        fflush(f);
        fsync(fileno(f));
        ESP_LOGI(TAG, "Saved audio to %s", path);
    }

    fflush(f);
    fsync(fileno(f));
    fclose(f);
    return ret;
}

// ---------- Take one picture and save as /sdcard/<name>.jpg ----------
static esp_err_t capture_and_save_jpeg(const char *filename)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return ESP_FAIL;
    }

    char path[64];
    snprintf(path, sizeof(path), "/sdcard/%s", filename);

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        esp_camera_fb_return(fb);
        return ESP_FAIL;
    }

    size_t written = 0;
    esp_err_t err = ESP_OK;
    if (fb->format == PIXFORMAT_JPEG) {
        written = fwrite(fb->buf, 1, fb->len, f);
    } else {
        jpg_write_ctx_t ctx = { .f = f, .written = 0 };
        bool ok = fmt2jpg_cb(fb->buf, fb->len, fb->width, fb->height,
                             fb->format, 12 /* quality */, jpg_write_cb, &ctx);
        if (!ok) {
            ESP_LOGE(TAG, "JPEG encode failed");
            err = ESP_FAIL;
        } else {
            written = ctx.written;
        }
    }

    fclose(f);
    esp_camera_fb_return(fb);

    if (err != ESP_OK) {
        return err;
    }

    if (fb->format == PIXFORMAT_JPEG && written != fb->len) {
        ESP_LOGE(TAG, "Write failed, wrote %u of %u bytes",
                 (unsigned)written, (unsigned)fb->len);
        return ESP_FAIL;
    } else if (fb->format != PIXFORMAT_JPEG && written == 0) {
        ESP_LOGE(TAG, "Write failed, no data written");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Saved image to %s", path);
    return ESP_OK;
}

void app_main(void)
{
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(init_sdcard());
    ESP_ERROR_CHECK(init_camera());
    ESP_ERROR_CHECK(init_mic());

    // Apply a conservative sensor config for stability
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, FRAMESIZE_SVGA);  // 800x600
        s->set_brightness(s, 1);
        s->set_contrast(s, 0);
    }

    // Throw away the first frame after init; sensors often need one cycle
    // to settle after PLL/XCLK is configured.
    camera_fb_t *fb_discard = esp_camera_fb_get();
    if (fb_discard) {
        esp_camera_fb_return(fb_discard);
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    int image_count = 0;

    while (1) {
        char name[32];
        snprintf(name, sizeof(name), "img%05d.jpg", image_count++);

        if (capture_and_save_jpeg(name) == ESP_OK) {
            ESP_LOGI(TAG, "Captured %s", name);
        } else {
            ESP_LOGE(TAG, "Capture failed");
        }

        char aud[32];
        snprintf(aud, sizeof(aud), "aud%05d.wav", image_count - 1);
        if (record_and_save_wav(aud, 3000) != ESP_OK) {
            ESP_LOGE(TAG, "Audio capture failed");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));   // 5 seconds between shots
    }
}
