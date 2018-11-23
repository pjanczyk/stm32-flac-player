#pragma once

#include "../stream/stream.h"
#include <stdint.h>
#include <stdbool.h>

bool Wave_WriteHeader(
    OutputStream *os,
    unsigned sample_rate,
    unsigned channels,
    unsigned bits_per_sample,
    uint64_t total_samples
);

bool Wave_WriteSample(
    OutputStream *os,
    unsigned bits_per_sample,
    int32_t sample
);
