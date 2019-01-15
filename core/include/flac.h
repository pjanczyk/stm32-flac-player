#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "core/include/input_stream.h"

typedef struct Flac Flac;

typedef struct {
    unsigned sample_rate;
    unsigned channels;
    unsigned bits_per_sample;
    uint64_t total_samples;
} FlacInfo;

typedef struct {
    uint8_t *buffer;
    int size;
    int samples;
} FlacFrame;

Flac *Flac_New(InputStream *input);

void Flac_Destroy(Flac *flac);

bool Flac_ReadMetadata(Flac *flac, /*out*/ FlacInfo *info);

bool Flac_ReadFrame(Flac *flac, /*out*/ FlacFrame **frame);

void FlacFrame_Destroy(FlacFrame *frame);
