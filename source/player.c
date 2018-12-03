#include "player.h"

#include <stdbool.h>
#include <stdint.h>
#include <term_io.h>
#include <usb_host.h>
#include "wm8994/wm8994.h"
#include <stm32746g_discovery_lcd.h>
#include <stm32746g_discovery_ts.h>
#include <stm32746g_discovery_audio.h>
#include <Middlewares/Third_Party/FatFs/src/ff.h>

#define LCD_X_SIZE RK043FN48H_WIDTH
#define LCD_Y_SIZE RK043FN48H_HEIGHT

#define AUDIO_OUT_BUFFER_SIZE 8192

enum {
    BUFFER_OFFSET_NONE = 0,
    BUFFER_OFFSET_HALF,
    BUFFER_OFFSET_FULL,
};

extern ApplicationTypeDef Appli_state;

static uint8_t audio_buffer[AUDIO_OUT_BUFFER_SIZE];
static uint8_t audio_buffer_offset = BUFFER_OFFSET_NONE;
static bool is_playing = false;
static FIL file;

static void StartPlaying(void);
static void StopPlaying(void);

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
    audio_buffer_offset = BUFFER_OFFSET_HALF;
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
    audio_buffer_offset = BUFFER_OFFSET_FULL;
}

void Player_Task(void) {
    xprintf("Waiting for USB mass storage ");
    while (Appli_state != APPLICATION_READY) {
        xprintf(".");
        vTaskDelay(250);
    }
    xprintf(" OK\r\n");

    for (;;) {
        TS_StateTypeDef ts_state;
        BSP_TS_GetState(&ts_state);
        if (ts_state.touchDetected && !is_playing) {
            StartPlaying();
        }

        BSP_LCD_Clear(LCD_COLOR_WHITE);

        if (is_playing) {
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
            BSP_LCD_FillCircle(LCD_X_SIZE / 2, LCD_Y_SIZE / 2, 40);

            if (audio_buffer_offset != BUFFER_OFFSET_NONE) {
                uint32_t offset = (audio_buffer_offset == BUFFER_OFFSET_HALF)
                                  ? 0
                                  : AUDIO_OUT_BUFFER_SIZE / 2;

                UINT bytes_read;

                if (f_read(&file, &audio_buffer[offset], AUDIO_OUT_BUFFER_SIZE / 2, &bytes_read) != FR_OK) {
                    xprintf("f_read error\r\n");
                    StopPlaying();
                    continue;
                }

                audio_buffer_offset = BUFFER_OFFSET_NONE;

                if (bytes_read < AUDIO_OUT_BUFFER_SIZE / 2) {
                    xprintf("stop at eof\r\n");
                    StopPlaying();
                }
            }

        }

        vTaskDelay(2);
    }
}

static void StartPlaying(void) {
    xprintf("StartPlaying\r\n");
    is_playing = true;

    xprintf("Opening wave file ...");
    FRESULT res = f_open(&file, "1:/test_1k.wav", FA_READ);
    if (!(res == FR_OK)) {
        xprintf(" ERROR, res = %d\r\n", res);
        while (1) {}
    }
    xprintf(" OK\r\n");

    xprintf("Initializing audio ...");
    //workaround: use 22K to play 44K
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE1, 60, AUDIO_FREQUENCY_22K) != 0) {
        xprintf(" ERROR\r\n");
        while (1) {}
    }
    xprintf(" OK\r\n");

    audio_buffer_offset = BUFFER_OFFSET_NONE;
    BSP_AUDIO_OUT_Play((uint16_t *) &audio_buffer[0], AUDIO_OUT_BUFFER_SIZE);
}

static void StopPlaying(void) {
    xprintf("StopPlaying\r\n");
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
    f_close(&file);
    is_playing = false;
}