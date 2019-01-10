#include "core/include/controller.h"

#include "core/include/files.h"
#include "core/include/log.h"
#include "core/include/player.h"
#include "core/include/screen.h"
#include "Inc/usb_host.h"
#include "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.h"

extern ApplicationTypeDef Appli_state;

static Files files;
static int current_file_index = 0;

static void WaitForUsbStorage(void) {
    xprintf("Waiting for USB mass storage...\n");
    while (Appli_state != APPLICATION_READY) {
        xprintf(".");
        osDelay(250);
    }
    xprintf("USB mass storage ready\n");
}

static const char *GetCurrentFilePath(void) {
    static char path[3 + MAX_FILE_PATH_LENGTH + 1];
    snprintf(path, sizeof(path), "1:/%s", files.files[current_file_index]);
    return path;
}

static void PlayOrPause(void) {
    switch (Player_GetState()) {
        case PlayerState_Stopped:
            Player_Play(GetCurrentFilePath());
            break;

        case PlayerState_Playing:
            Player_Pause();
            break;

        case PlayerState_Paused:
            Player_Resume();
            break;
    }
}

static void SkipPrevious(void) {
    if (Player_GetState() != PlayerState_Stopped) {
        Player_Stop();
    }

    current_file_index = (current_file_index - 1 + files.count) % files.count;
    Player_Play(GetCurrentFilePath());
}

static void SkipNext(void) {
    if (Player_GetState() != PlayerState_Stopped) {
        Player_Stop();
    }

    current_file_index = (current_file_index + 1) % files.count;
    Player_Play(GetCurrentFilePath());
}

void Controller_Task(void) {
    WaitForUsbStorage();
    FindFlacFiles("1:", &files);
    Player_Init();
    Player_Play("1:/test.flac");

    for (;;) {
        Screen_HandleTouch();
        Screen_Render(
            /*number_of_files*/ files.count,
            /*current_file_index*/ current_file_index,
            /*current_file_name*/ GetCurrentFilePath(),
            /*progress*/ Player_GetProgress(),
            /*is_playing*/ Player_GetState() == PlayerState_Playing
        );

        if (Screen_IsPlayPauseButtonTouched()) {
            PlayOrPause();
        } else if (Screen_IsBackButtonTouched()) {
            SkipPrevious();
        } else if (Screen_IsNextButtonTouched()) {
            SkipNext();
        }

        Player_Update();
    }
}
