#include "flac_buffer.h"
#include <stdlib.h>
#include <string.h>
#include <term_io.h>

FlacBuffer FlacBuffer_New(Flac *flac) {
    return (FlacBuffer) {
        .flac = flac,
        .frame = NULL,
        .position = 0
    };
}

int FlacBuffer_Read(FlacBuffer *self, void *dest, int size) {
    int written = 0;
    while (written < size) {
        xprintf("FlacBuffer_Read while: written: %d\r\n", written);

        if (self->frame == NULL) {
            self->position = 0;
            if (!Flac_ReadFrame(self->flac, &self->frame)) {
                xprintf("d\r\n");
                return written;
            }
        }

        xprintf("Frame was read, frame size: %d\r\n", self->frame->size);

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

    return written;
}