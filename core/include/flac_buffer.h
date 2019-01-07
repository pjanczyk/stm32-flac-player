#pragma once

#include "core/include/flac.h"

typedef struct {
    Flac* flac;
    FlacFrame* frame;
    int position;
} FlacBuffer;

FlacBuffer FlacBuffer_New(Flac* flac);
int FlacBuffer_Read(FlacBuffer* self, void* dest, int size);