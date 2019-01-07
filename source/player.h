#pragma once

typedef enum {
    PlayerState_Stopped,
    PlayerState_Playing,
    PlayerState_Paused
} PlayerState;

void Player_Init(void);
void Player_Update(void);

PlayerState Player_GetState(void);
int Player_GetProgress(void);

void Player_Play(const char *filename);
void Player_Pause(void);
void Player_Resume(void);
void Player_Stop(void);