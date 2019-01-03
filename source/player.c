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
#include <source/stream/input_stream.h>
#include <source/flac/flac.h>
#include <source/flac_buffer/flac_buffer.h>

#define LCD_X_SIZE RK043FN48H_WIDTH
#define LCD_Y_SIZE RK043FN48H_HEIGHT

#define AUDIO_OUT_BUFFER_SIZE 131072

typedef enum {
    TransferEvent_None = 0,
    TransferEvent_TransferredFirstHalf,
    TransferEvent_TransferredSecondHalf,
} TransferEvent;

extern ApplicationTypeDef Appli_state;

static uint8_t audio_buffer[AUDIO_OUT_BUFFER_SIZE];
static uint8_t audio_transfer_event = TransferEvent_None;
static bool is_playing = false;
static FIL file;
static Flac *flac;
static InputStream input_stream;
static FlacBuffer flacBuffer;

static int last_audio_transfer_event_time = 0;

static void WaitForUsbStorage(void);
static void DrawPlayingState(void);
static bool IsTouchScreenTouched(void);
static TransferEvent GetTransferEvent(void);
static void StartPlaying(void);
static void StopPlaying(void);

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
    audio_transfer_event = TransferEvent_TransferredFirstHalf;
    int t = (int) xTaskGetTickCountFromISR();
    xprintf("[%d] TransferredFirstHalf (%d)\n", t, t - last_audio_transfer_event_time);
    last_audio_transfer_event_time = t;

}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
    audio_transfer_event = TransferEvent_TransferredSecondHalf;
    int t = (int) xTaskGetTickCountFromISR();
    xprintf("[%d] TransferredSecondHalf (%d)\n", t, t - last_audio_transfer_event_time);
    last_audio_transfer_event_time = t;
}

void Player_Task(void) {
    WaitForUsbStorage();

    StartPlaying();

    for (;;) {
        if (is_playing) {
            TransferEvent event = GetTransferEvent();
            if (event) {
                int t1 = (int) xTaskGetTickCount();
                xprintf("[%d] Handling event\n", t1);

                uint32_t offset = (event == TransferEvent_TransferredFirstHalf)
                                  ? 0
                                  : AUDIO_OUT_BUFFER_SIZE / 2;

                int bytes_read = FlacBuffer_Read(&flacBuffer, &audio_buffer[offset], AUDIO_OUT_BUFFER_SIZE / 2);

                if (bytes_read < AUDIO_OUT_BUFFER_SIZE / 2) {
                    xprintf("stop at eof\r\n");
                    StopPlaying();
                }

                int t2 = (int) xTaskGetTickCount();
                xprintf("[%d] Completed handling event in (%d)\n", t2, t2 - t1);
            }

        }
    }
}

static void WaitForUsbStorage(void) {
    xprintf("Waiting for USB mass storage ");
    while (Appli_state != APPLICATION_READY) {
        xprintf(".");
        vTaskDelay(250);
    }
    xprintf(" OK\r\n");
}

static void DrawPlayingState(void) {
    BSP_LCD_SelectLayer(0);
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    if (is_playing) {
        BSP_LCD_SetTextColor(0x40FF00FF);
        BSP_LCD_FillCircle(LCD_X_SIZE / 2, LCD_Y_SIZE / 2, 40);
    }
}

static bool IsTouchScreenTouched(void) {
    TS_StateTypeDef ts_state;
    BSP_TS_GetState(&ts_state);
    return ts_state.touchDetected > 0;
}

static TransferEvent GetTransferEvent(void) {
    TransferEvent event = audio_transfer_event;
    audio_transfer_event = TransferEvent_None;
    return event;
}

static void StartPlaying(void) {
    xprintf("StartPlaying\r\n");
    is_playing = true;

    xprintf("Opening FLAC file ...");
    FRESULT res = f_open(&file, "1:/test.flac", FA_READ);
    if (!(res == FR_OK)) {
        xprintf(" ERROR, res = %d\r\n", res);
        while (1) {}
    }

    input_stream = InputStream_InitWithFile(&file);
    flac = Flac_New(&input_stream);
    flacBuffer = FlacBuffer_New(flac);

    xprintf(" OK\r\n");

    xprintf("Reading FLAC metadata ... ");
    FlacInfo *flacInfo;
    if (!Flac_ReadMetadata(flac, &flacInfo)) {
        xprintf(" ERROR\r\n");
        while (1) {}
    }

    xprintf(" OK\r\n");

    xprintf("Initializing audio ...");
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE1, 60, AUDIO_FREQUENCY_44K) != 0) {
        xprintf(" ERROR\r\n");
        while (1) {}
    }
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

    xprintf(" OK\r\n");

    audio_transfer_event = TransferEvent_None;
    BSP_AUDIO_OUT_Play((uint16_t *) &audio_buffer[0], AUDIO_OUT_BUFFER_SIZE);

    xprintf("b\r\n");
}

static void StopPlaying(void) {
    xprintf("StopPlaying\r\n");
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
    f_close(&file);
    is_playing = false;
}