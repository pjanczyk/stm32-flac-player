#include "player.h"

#include <stdbool.h>
#include <stdint.h>
#include "wm8994/wm8994.h"
#include "log.h"
#include <stm32746g_discovery_audio.h>
#include <Middlewares/Third_Party/FatFs/src/ff.h>
#include <source/stream/input_stream.h>
#include <source/flac/flac.h>
#include <source/flac_buffer/flac_buffer.h>
#include <source/screen/screen.h>
#include <source/files/files.h>

#define AUDIO_OUT_BUFFER_SIZE 131072

typedef enum {
    TransferEvent_None = 0,
    TransferEvent_TransferredFirstHalf,
    TransferEvent_TransferredSecondHalf,
} TransferEvent;

static uint8_t audio_buffer[AUDIO_OUT_BUFFER_SIZE];
static uint8_t audio_transfer_event = TransferEvent_None;
static PlayerState state = PlayerState_Stopped;
static FIL file;
static Flac *flac;
static InputStream input_stream;
static FlacBuffer flacBuffer;

static int last_audio_transfer_event_time = 0;

static TransferEvent GetTransferEvent(void) {
    TransferEvent event = audio_transfer_event;
    audio_transfer_event = TransferEvent_None;
    return event;
}

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

void Player_Init(void) {
    xprintf("Initializing audio ...");
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE1, 60, AUDIO_FREQUENCY_44K) != 0) {
        xprintf(" ERROR\n");
        for (;;) {}
    }
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
    xprintf(" OK\n");
}

void Player_Update(void) {
    if (state == PlayerState_Playing) {
        TransferEvent event = GetTransferEvent();
        if (event) {
            int t1 = (int) xTaskGetTickCount();
            xprintf("[%d] Handling event\n", t1);

            uint32_t offset = (event == TransferEvent_TransferredFirstHalf)
                              ? 0
                              : AUDIO_OUT_BUFFER_SIZE / 2;

            int bytes_read = FlacBuffer_Read(&flacBuffer, &audio_buffer[offset], AUDIO_OUT_BUFFER_SIZE / 2);

            if (bytes_read < AUDIO_OUT_BUFFER_SIZE / 2) {
                xprintf("stop at eof\n");
                Player_Stop();
            }

            int t2 = (int) xTaskGetTickCount();
            xprintf("[%d] Completed handling event in (%d)\n", t2, t2 - t1);
        }
    }
}

void Player_Play(const char *filename) {
    xprintf("Player_Play\n");
    state = PlayerState_Playing;

    xprintf("Opening FLAC file ...");
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        xprintf(" ERROR, res = %d\n", res);
        while (1) {}
    }

    input_stream = InputStream_InitWithFile(&file);
    flac = Flac_New(&input_stream);
    flacBuffer = FlacBuffer_New(flac);

    xprintf(" OK\n");

    xprintf("Reading FLAC metadata ... ");
    FlacInfo *flacInfo;
    if (!Flac_ReadMetadata(flac, &flacInfo)) {
        xprintf(" ERROR\n");
        while (1) {}
    }
    xprintf(" OK\n");

    audio_transfer_event = TransferEvent_None;
    BSP_AUDIO_OUT_Play((uint16_t *) &audio_buffer[0], AUDIO_OUT_BUFFER_SIZE);
}

PlayerState Player_GetState(void) {
    return state;
}

int Player_GetProgress(void) {
    // TODO
    return 333;
}

void Player_Pause(void) {
    xprintf("Player_Pause\n");
    // TODO
}

void Player_Resume(void) {
    xprintf("Player_Resume\n");
    // TODO
}

void Player_Stop(void) {
    xprintf("Player_Stop\n");
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
    f_close(&file);
    state = PlayerState_Stopped;
}
