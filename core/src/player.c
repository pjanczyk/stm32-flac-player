#include "core/include/player.h"

#include <assert.h>
#include <stdint.h>

#include "core/include/flac.h"
#include "core/include/flac_buffer.h"
#include "core/include/log.h"
#include "Drivers/BSP/Components/wm8994/wm8994.h"
#include "Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_audio.h"

#define AUDIO_OUT_BUFFER_SIZE 131072

typedef enum {
    TransferEvent_None = 0,
    TransferEvent_TransferredFirstHalf,
    TransferEvent_TransferredSecondHalf,
} TransferEvent;

static uint8_t audio_buffer[AUDIO_OUT_BUFFER_SIZE];
static uint8_t audio_transfer_event = TransferEvent_None;
static unsigned last_audio_transfer_event_time = 0;

static PlayerState state = PlayerState_Stopped;
static FIL file;
static InputStream input_stream;
static FlacBuffer flac_buffer;
static Flac *flac;

static TransferEvent GetTransferEvent(void) {
    TransferEvent event = audio_transfer_event;
    audio_transfer_event = TransferEvent_None;
    return event;
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
    audio_transfer_event = TransferEvent_TransferredFirstHalf;
    unsigned t = osKernelSysTick();
    xprintf("[%u] TransferredFirstHalf (%u)\n", t, t - last_audio_transfer_event_time);
    last_audio_transfer_event_time = t;
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
    audio_transfer_event = TransferEvent_TransferredSecondHalf;
    unsigned t = osKernelSysTick();
    xprintf("[%u] TransferredSecondHalf (%u)\n", t, t - last_audio_transfer_event_time);
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
            unsigned t1 = osKernelSysTick();
            xprintf("[%u] Handling event\n", t1);

            uint32_t offset = (event == TransferEvent_TransferredFirstHalf)
                              ? 0
                              : AUDIO_OUT_BUFFER_SIZE / 2;

            int bytes_read = FlacBuffer_Read(&flac_buffer, &audio_buffer[offset], AUDIO_OUT_BUFFER_SIZE / 2);

            if (bytes_read < AUDIO_OUT_BUFFER_SIZE / 2) {
                xprintf("stop at eof\n");
                Player_Stop();
            }

            unsigned t2 = osKernelSysTick();
            xprintf("[%u] Completed handling event in (%u)\n", t2, t2 - t1);
        }
    }
}

PlayerState Player_GetState(void) {
    return state;
}

int Player_GetProgress(void) {
    // TODO
    return 333;
}

void Player_Play(const char *filename) {
    xprintf("Player_Play\n");

    assert(state == PlayerState_Stopped);

    state = PlayerState_Playing;

    xprintf("Opening FLAC file...\n");
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        xprintf(" ERROR, res = %d\n", res);
        while (1) {}
    }

    xprintf("Initializing decoder...\n");
    input_stream = InputStream_InitWithFile(&file);
    flac = Flac_New(&input_stream);
    flac_buffer = FlacBuffer_New(flac);

    xprintf("Reading FLAC metadata...\n");
    FlacInfo *flacInfo;
    if (!Flac_ReadMetadata(flac, &flacInfo)) {
        xprintf(" ERROR\n");
        while (1) {}
    }

    audio_transfer_event = TransferEvent_None;

    xprintf("Starting hardware playing...\n");
    BSP_AUDIO_OUT_Play((uint16_t *) &audio_buffer[0], AUDIO_OUT_BUFFER_SIZE);

    xprintf("Player_Play: Done\n");
}

void Player_Pause(void) {
    xprintf("Player_Pause\n");

    assert(state == PlayerState_Playing);

    state = PlayerState_Paused;
    BSP_AUDIO_OUT_Pause();
}

void Player_Resume(void) {
    xprintf("Player_Resume\n");

    assert(state == PlayerState_Paused);

    state = PlayerState_Playing;
    BSP_AUDIO_OUT_Resume();
}

void Player_Stop(void) {
    xprintf("Player_Stop\n");

    assert(state == PlayerState_Playing || state == PlayerState_Paused);

    state = PlayerState_Stopped;

    xprintf("Stopping hardware playing...\n");
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);

    xprintf("Destroying decoder...\n");
    FlacBuffer_Destroy(&flac_buffer);
    Flac_Destroy(flac);
    flac = NULL;
    InputStream_Destroy(&input_stream);

    xprintf("Closing file...\n");
    f_close(&file);

    xprintf("Player_Stop: Done\n");
}
