#include "fatfs_stream.h"

typedef struct {
    const InputStreamDef *def;
    FIL *file;
} FatFsInputStream;

int Read(InputStream *self, void *buf, int len) {
    FIL *file = ((FatFsInputStream *) self)->file;

    UINT bytes_read;
    FRESULT rc = f_read(file, buf, (UINT) len, &bytes_read);

    if (rc != FR_OK) {
        return -1;
    }

    return (int) bytes_read;
}

void Destroy(InputStream *self) {
    free(self);
}

static const InputStreamDef InputStreamDef_fatFs = {
    .read = &Read,
    .destroy = &Destroy
};

InputStream *InputStream_InitWithFatFs(FIL *file) {
    FatFsInputStream *stream = malloc(sizeof(FatFsInputStream));
    stream->def = &InputStreamDef_fatFs;
    stream->file = file;
    return stream;
}
