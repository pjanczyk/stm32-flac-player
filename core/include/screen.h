#pragma once

#include <stdbool.h>

void Screen_Initialize(void);

void Screen_RenderInfo(const char *info);

void Screen_RenderPlayer(
    int number_of_files,
    int current_file_index,
    const char *current_file_name,
    int progress,
    bool is_playing
);

void Screen_HandleTouch(void);

bool Screen_IsBackButtonTouched(void);
bool Screen_IsPlayPauseButtonTouched(void);
bool Screen_IsNextButtonTouched(void);
