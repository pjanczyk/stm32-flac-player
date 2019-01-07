#include "core/include/input_stream.h"

#include "core/include/log.h"

int InputStream_Read(InputStream *self, void *buf, int len) {
    UINT bytes_read;
    FRESULT rc = f_read(self->file, buf, (UINT) len, &bytes_read);

    if (rc != FR_OK) {
        log_error("ERROR: f_read failed\r\n");
        return -1;
    }

    return (int) bytes_read;
}

void InputStream_Destroy(InputStream *self) {
    f_close(self->file);
    *self = (InputStream) {
        .file = NULL
    };
}

InputStream InputStream_InitWithFile(FIL *file) {
    return (InputStream) {
        .file = file
    };
}
