#include "wave.h"
#include "../stream/stream.h"
#include <string.h>

static bool WriteUint16(OutputStream *os, uint16_t x) {
    uint8_t data[] = {
        (uint8_t) x,
        (uint8_t) (x >> 8)
    };
    return OutputStream_Write(os, data, sizeof(data));
}

static bool WriteUint24(OutputStream *os, uint32_t x) {
    uint8_t data[] = {
        (uint8_t) x,
        (uint8_t) (x >> 8),
        (uint8_t) (x >> 16),
    };
    return OutputStream_Write(os, data, sizeof(data));
}

static bool WriteUint32(OutputStream *os, uint32_t x) {
    uint8_t data[] = {
        (uint8_t) x,
        (uint8_t) (x >> 8),
        (uint8_t) (x >> 16),
        (uint8_t) (x >> 24),
    };
    return OutputStream_Write(os, data, sizeof(data));
}

static bool WriteInt16(OutputStream *os, int16_t x) {
    return WriteUint16(os, (uint16_t) x);
}

static bool WriteInt24(OutputStream *os, int32_t x) {
    return WriteUint24(os, (uint32_t) x);
}

static bool WriteString(OutputStream *os, const char *string) {
    int len = strlen(string);
    return OutputStream_Write(os, string, len);
}

bool Wave_WriteHeader(
    OutputStream *os,
    unsigned sample_rate,
    unsigned channels,
    unsigned bits_per_sample,
    uint64_t total_samples
) {
    uint32_t data_size = total_samples * channels * (bits_per_sample / 8);
    uint32_t chunk_size = data_size + 36;
    uint32_t byte_rate = sample_rate * channels * (bits_per_sample / 8);
    uint16_t block_align = channels * (bits_per_sample / 8);

    return WriteString(os, "RIFF")          // chunk ID
           && WriteUint32(os, chunk_size)      // chunk size
           && WriteString(os, "WAVE")          // format
           && WriteString(os, "fmt ")          // subchunk 1 ID
           && WriteUint32(os, 16)              // subchunk 1 size
           && WriteUint16(os, 1)               // audio format = PCM
           && WriteUint16(os, channels)        // channels
           && WriteUint32(os, sample_rate)     // sample rate
           && WriteUint32(os, byte_rate)       // byte rate
           && WriteUint16(os, block_align)     // block align
           && WriteUint16(os, bits_per_sample) // bits per sample
           && WriteString(os, "data")          // subchunk 2 ID
           && WriteUint32(os, data_size);      // subchunk 2 size
}

bool Wave_WriteSample(
    OutputStream *os,
    unsigned bits_per_sample,
    int32_t sample
) {
    if (bits_per_sample == 24) {
        return WriteInt24(os, sample);
    } else if (bits_per_sample == 16) {
        return WriteInt16(os, sample);
    } else {
        return false;
    }
}
