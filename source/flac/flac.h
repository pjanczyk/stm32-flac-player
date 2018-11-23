#pragma once

#include "../stream/stream.h"

typedef struct Flac Flac;

typedef struct {
    unsigned sample_rate;
    unsigned channels;
    unsigned bits_per_sample;
    uint64_t total_samples;
} FlacInfo;

Flac *Flac_New(InputStream *input);

void Flac_Destroy(Flac *flac);

bool Flac_ReadMetadata(Flac *flac, FlacInfo *info);

bool Flac_ReadFrame(Flac *flac, OutputStream *output);
