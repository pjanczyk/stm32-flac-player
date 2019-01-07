#pragma once

#include <stdbool.h>

void Screen_Render(bool is_playing, char *filename);
void Screen_HandleTouch(void);

bool Screen_IsBackButtonTouched(void);
bool Screen_IsPlayPauseButtonTouched(void);
bool Screen_IsNextButtonTouched(void);
