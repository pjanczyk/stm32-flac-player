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
static bool player_is_playing = false;
static FIL file;
static TS_StateTypeDef TS_State;

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
    audio_buffer_offset = BUFFER_OFFSET_FULL;
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
    audio_buffer_offset = BUFFER_OFFSET_HALF;
}

void Player_Task() {
    xprintf("Waiting for USB mass storage ");
    while (Appli_state != APPLICATION_READY) {
        xprintf(".");
        vTaskDelay(250);
    }
    xprintf(" OK\r\n");

    for (;;) {
        BSP_TS_GetState(&TS_State);
        if (TS_State.touchDetected && !player_is_playing) {
            xprintf("play command...\r\n");
            player_is_playing = true;

            FRESULT res = f_open(&file, "1:/test_1k.wav", FA_READ);

            if (res == FR_OK) {
                xprintf("wave file open OK\r\n");
            } else {
                xprintf("wave file open ERROR, res = %d\r\n", res);
                while (1) {}
            }

            //workaround: use 22K to play 44K
            if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE1, 60, AUDIO_FREQUENCY_22K) == 0) {
                xprintf("audio init OK\r\n");
            } else {
                xprintf("audio init ERROR\r\n");
            }

            BSP_AUDIO_OUT_Play((uint16_t *) &audio_buffer[0], AUDIO_OUT_BUFFER_SIZE);
            audio_buffer_offset = BUFFER_OFFSET_NONE;
        }

        BSP_LCD_Clear(LCD_COLOR_WHITE);

        if (player_is_playing) {
            BSP_LCD_SetTextColor(0x40FF00FF);
            BSP_LCD_FillCircle(LCD_X_SIZE / 2, LCD_Y_SIZE / 2, 40);

            if (audio_buffer_offset != BUFFER_OFFSET_NONE) {
                uint8_t *begin = (audio_buffer_offset == BUFFER_OFFSET_HALF)
                                 ? &audio_buffer[0]
                                 : &audio_buffer[AUDIO_OUT_BUFFER_SIZE / 2];

                uint32_t bytes_read;

                if (f_read(&file, begin, AUDIO_OUT_BUFFER_SIZE / 2, (void *) &bytes_read) != FR_OK) {
                    BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
                    xprintf("f_read error\r\n");
                } else {
                    audio_buffer_offset = BUFFER_OFFSET_NONE;

                    if (bytes_read < AUDIO_OUT_BUFFER_SIZE / 2) {
                        xprintf("stop at eof\r\n");
                        BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
                        f_close(&file);
                        player_is_playing = false;
                    }
                }
            }

        }

        vTaskDelay(2);

    }
}