#include "core/include/flac_buffer.h"

#include "core/include/log.h"

FlacBuffer FlacBuffer_New(Flac *flac) {
    return (FlacBuffer) {
        .flac = flac,
        .frame = NULL,
        .position = 0
    };
}

void FlacBuffer_Destroy(FlacBuffer *self) {
    free(self->frame);
    *self = (FlacBuffer) {
        .flac = NULL,
        .frame = NULL,
        .position = 0
    };
}

int FlacBuffer_Read(FlacBuffer *self, void *dest, int size) {
    log_debug("FlacBuffer_Read: begin\n");
    int written = 0;
    while (written < size) {
        log_debug("FlacBuffer_Read while: written: %d\n", written);

        if (self->frame == NULL) {
            self->position = 0;
            if (!Flac_ReadFrame(self->flac, &self->frame)) {
                log_debug("FlacBuffer_Read: done\n");
                return written;
            }
        }

        log_debug("Frame was read, frame size: %d\n", self->frame->size);

        int frame_left = self->frame->size - self->position;
        int dest_space_left = size - written;

        if (frame_left <= dest_space_left) {
            memcpy(dest + written, &self->frame->buffer[self->position], frame_left);
            written += frame_left;
            FlacFrame_Destroy(self->frame);
            self->frame = NULL;
            self->position = 0;
        } else {
            memcpy(dest + written, &self->frame->buffer[self->position], dest_space_left);
            written += dest_space_left;
            self->position += dest_space_left;
        }
    }

    log_debug("FlacBuffer_Read: end: written: %d\n", written);

    return written;
}
