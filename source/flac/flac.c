#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include "flac.h"
#include "../log.h"
#include "FLAC/stream_decoder.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <source/stream/input_stream.h>


struct Flac {
    FLAC__StreamDecoder *decoder;
    InputStream *input;
    FlacInfo *info;
    FlacFrame *frame;
};

static FLAC__StreamDecoderReadStatus DecoderReadCallback(
    const FLAC__StreamDecoder *decoder,
    FLAC__byte *buffer,
    size_t *bytes,
    void *client_data
) {
    log_debug("Flac: Read callback\r\n");

    Flac *flac = (Flac *) client_data;

    if (*bytes <= 0) {
        return FLAC__STREAM_DECODER_READ_STATUS_ABORT;
    }

    int read_bytes = InputStream_Read(flac->input, buffer, *bytes);

    if (read_bytes > 0) {
        *bytes = (size_t) read_bytes;
        return FLAC__STREAM_DECODER_READ_STATUS_CONTINUE;
    } else if (read_bytes == 0) {
        *bytes = 0;
        return FLAC__STREAM_DECODER_READ_STATUS_END_OF_STREAM;
    } else {
        *bytes = 0;
        return FLAC__STREAM_DECODER_READ_STATUS_ABORT;
    }
}

static FLAC__StreamDecoderWriteStatus DecoderWriteCallback(
    const FLAC__StreamDecoder *decoder,
    const FLAC__Frame *frame,
    const FLAC__int32 *const *buffer,
    void *client_data
) {
    log_debug("Flac: Write callback\r\n");

    Flac *flac = (Flac *) client_data;

    for (int i = 0; i < frame->header.channels; i++) {
        if (buffer[i] == NULL) {
            log_error("ERROR: buffer[%d] is NULL\n", i);
            return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
        }
    }

    int samples = frame->header.blocksize;
    int channels = frame->header.channels;
    int bytes_per_sample = frame->header.bits_per_sample / 8;
    int size = samples * channels * bytes_per_sample;

    flac->frame = malloc(sizeof(FlacFrame));
    *flac->frame = (FlacFrame) {
        .size = size,
        .buffer = malloc(sizeof(size))
    };

    for (int sample = 0; sample < samples; sample++) {
        for (int channel = 0; channel < channels; channel++) {
            for (int byte = 0; byte < bytes_per_sample; byte++) {
                flac->frame->buffer[(sample * channels + channel) * bytes_per_sample + byte] =
                    (uint8_t) ((buffer[channel][sample] >> (byte * 8)) & 0xFF);
            }
        }
    }

    return FLAC__STREAM_DECODER_WRITE_STATUS_CONTINUE;
}

static void DecoderMetadataCallback(
    const FLAC__StreamDecoder *decoder,
    const FLAC__StreamMetadata *metadata,
    void *client_data
) {
    log_debug("Flac: Metadata callback\r\n");

    Flac *flac = (Flac *) client_data;

    if (metadata->type == FLAC__METADATA_TYPE_STREAMINFO) {
        flac->info = malloc(sizeof(FlacInfo));
        *flac->info = (FlacInfo) {
            .total_samples = metadata->data.stream_info.total_samples,
            .sample_rate = metadata->data.stream_info.sample_rate,
            .channels = metadata->data.stream_info.channels,
            .bits_per_sample = metadata->data.stream_info.bits_per_sample
        };
    }
}

static void DecoderErrorCallback(
    const FLAC__StreamDecoder *decoder,
    FLAC__StreamDecoderErrorStatus status,
    void *client_data
) {
    log_error("Got error callback: %s\n", FLAC__StreamDecoderErrorStatusString[status]);
}


Flac *Flac_New(InputStream *input) {
    Flac *flac = malloc(sizeof(Flac));
    *flac = (Flac) {
        .input = input
    };

    flac->decoder = FLAC__stream_decoder_new();
    if (flac->decoder == NULL) {
        log_error("ERROR: allocating decoder\n");
        Flac_Destroy(flac);
        return NULL;
    }

    FLAC__stream_decoder_set_md5_checking(flac->decoder, true);

    FLAC__StreamDecoderInitStatus init_status = FLAC__stream_decoder_init_stream(
        flac->decoder,
        &DecoderReadCallback,
        NULL,
        NULL,
        NULL,
        NULL,
        &DecoderWriteCallback,
        &DecoderMetadataCallback,
        &DecoderErrorCallback,
        flac
    );
    if (init_status != FLAC__STREAM_DECODER_INIT_STATUS_OK) {
        log_error("ERROR: initializing decoder: %s\n", FLAC__StreamDecoderInitStatusString[init_status]);
        Flac_Destroy(flac);
        return NULL;
    }

    return flac;
}

void Flac_Destroy(Flac *flac) {
    if (flac != NULL) {
        if (flac->decoder != NULL) {
            FLAC__stream_decoder_delete(flac->decoder);
        }
        free(flac);
    }
}

bool Flac_ReadMetadata(Flac *flac, /*out*/ FlacInfo **info) {
    if (FLAC__stream_decoder_process_until_end_of_metadata(flac->decoder)) {
        *info = flac->info;
        flac->info = NULL;
        return true;
    } else {
        log_error("ERROR: reading metadata: %s\n",
                  FLAC__StreamDecoderStateString[FLAC__stream_decoder_get_state(flac->decoder)]);
        return false;
    }
}

bool Flac_ReadFrame(Flac *flac, /*out*/ FlacFrame **frame) {
    unsigned int t = xTaskGetTickCount();

    if (FLAC__stream_decoder_process_single(flac->decoder)) {
        *frame = flac->frame;
        flac->frame = NULL;

        t = xTaskGetTickCount() - t;
        log_debug("Flac_ReadFrame: read frame with size: %d in %u ms\r\n", (*frame)->size, t);
        return true;
    } else {
        log_error("ERROR: reading frame: %s\n",
                  FLAC__StreamDecoderStateString[FLAC__stream_decoder_get_state(flac->decoder)]);
        return false;
    }
}

void FlacInfo_Destroy(FlacInfo *info) {
    free(info);
}

void FlacFrame_Destroy(FlacFrame *frame) {
    free(frame->buffer);
    free(frame);
}
