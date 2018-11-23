#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include "flac.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "share/compat.h"
#include "FLAC/stream_decoder.h"

struct Flac {
    FLAC__StreamDecoder *decoder;
    FlacInfo info;
    InputStream *input;
    OutputStream *output;
};

static FLAC__StreamDecoderReadStatus DecoderReadCallback(
    const FLAC__StreamDecoder *decoder,
    FLAC__byte *buffer,
    size_t *bytes,
    void *client_data
) {
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
    Flac *flac = (Flac *) client_data;
    FlacInfo *info = &flac->info;

    if (info->total_samples == 0) {
        fprintf(stderr,
                "ERROR: this example only works for FLAC files that have a total_samples count in STREAMINFO\n");
        return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
    }

    for (int i = 0; i < frame->header.channels; i++) {
        if (buffer[i] == NULL) {
            fprintf(stderr, "ERROR: buffer[%d] is NULL\n", i);
            return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
        }
    }

    /* write decoded PCM samples */
    for (size_t f = 0; f < frame->header.blocksize; f++) {
        for (size_t c = 0; c < info->channels; c++) {
            if (!OutputStream_Write(flac->output, &buffer[c][f], 4)) {
                fprintf(stderr, "ERROR: write error\n");
                return FLAC__STREAM_DECODER_WRITE_STATUS_ABORT;
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
    Flac *flac = (Flac *) client_data;
    FlacInfo *info = &flac->info;

    if (metadata->type == FLAC__METADATA_TYPE_STREAMINFO) {
        info->total_samples = metadata->data.stream_info.total_samples;
        info->sample_rate = metadata->data.stream_info.sample_rate;
        info->channels = metadata->data.stream_info.channels;
        info->bits_per_sample = metadata->data.stream_info.bits_per_sample;
    }
}

static void DecoderErrorCallback(
    const FLAC__StreamDecoder *decoder,
    FLAC__StreamDecoderErrorStatus status,
    void *client_data
) {
    fprintf(stderr, "Got error callback: %s\n", FLAC__StreamDecoderErrorStatusString[status]);
}


Flac *Flac_New(InputStream *input) {
    Flac *flac = malloc(sizeof(Flac));
    *flac = (Flac) {};

    flac->decoder = FLAC__stream_decoder_new();
    if (flac->decoder == NULL) {
        fprintf(stderr, "ERROR: allocating decoder\n");
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
        fprintf(stderr, "ERROR: initializing decoder: %s\n", FLAC__StreamDecoderInitStatusString[init_status]);
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

bool Flac_ReadMetadata(Flac *flac, FlacInfo *info) {
    if (FLAC__stream_decoder_process_until_end_of_metadata(flac->decoder)) {
        *info = flac->info;
        return true;
    } else {
        fprintf(stderr, "ERROR: reading metadata: %s\n",
                FLAC__StreamDecoderStateString[FLAC__stream_decoder_get_state(flac->decoder)]);
        return false;
    }
}

bool Flac_ReadFrame(Flac *flac, OutputStream *output) {
    flac->output = output;
    if (FLAC__stream_decoder_process_single(flac->decoder)) {
        return true;
    } else {
        fprintf(stderr, "ERROR: reading frame: %s\n",
                FLAC__StreamDecoderStateString[FLAC__stream_decoder_get_state(flac->decoder)]);
        return false;
    }
}
