#include "core/include/player.h"

#include <assert.h>
#include <stdint.h>

#include "core/include/flac.h"
#include "core/include/flac_buffer.h"
#include "core/include/log.h"
#include "Drivers/BSP/Components/wm8994/wm8994.h"
#include "Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_audio.h"

#define AUDIO_OUT_BUFFER_SIZE 32768

typedef enum {
    TransferEvent_None = 0,
    TransferEvent_TransferredFirstHalf,
    TransferEvent_TransferredSecondHalf,
} TransferEvent;

static uint8_t audio_buffer[AUDIO_OUT_BUFFER_SIZE];
static uint8_t audio_transfer_event = TransferEvent_None;
static unsigned last_audio_transfer_event_time = 0;

static PlayerState state = PlayerState_Stopped;
static uint64_t samples_played;
static FIL file;
static InputStream input_stream;
static FlacBuffer flac_buffer;
static Flac *flac;
static FlacInfo flac_info;

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
    xprintf("Initializing audio...\n");
    if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE1, 60, AUDIO_FREQUENCY_44K) != 0) {
        log_fatal_and_die(" ERROR\n");
    }
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
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

            samples_played += bytes_read / flac_info.channels / (flac_info.bits_per_sample / 8);

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
    if (flac_info.total_samples == 0) {
        return 0;
    }
    uint64_t progress = samples_played * 1000 / flac_info.total_samples;
    if (progress > 1000) {
        progress = 1000;
    }
    return (int) progress;
}

void Player_Play(const char *filename) {
    xprintf("Player_Play '%s'\n", filename);

    assert(state == PlayerState_Stopped);

    state = PlayerState_Playing;

    xprintf("Opening FLAC file...\n");
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        log_fatal_and_die(" ERROR, res = %d\n", res);
    }

    xprintf("Initializing decoder...\n");
    input_stream = InputStream_InitWithFile(&file);
    flac = Flac_New(&input_stream);
    flac_buffer = FlacBuffer_New(flac);

    xprintf("Reading FLAC metadata...\n");
    if (!Flac_ReadMetadata(flac, &flac_info)) {
        log_fatal_and_die(" ERROR\n");
    }

    xprintf("Filling first half of buffer...\n");
    int bytes_read = FlacBuffer_Read(&flac_buffer, audio_buffer, AUDIO_OUT_BUFFER_SIZE / 2);
    if (bytes_read < AUDIO_OUT_BUFFER_SIZE / 2) {
        xprintf("stop at eof\n");
        Player_Stop();
        return;
    }

    audio_transfer_event = TransferEvent_TransferredSecondHalf;

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

    samples_played = 0;

    xprintf("Player_Stop: Done\n");
}
