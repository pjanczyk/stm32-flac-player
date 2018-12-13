#include "input_stream.h"
#include <term_io.h>
#include <Middlewares/Third_Party/FatFs/src/ff.h>

int InputStream_Read(InputStream *self, void *buf, int len) {
    xprintf("FatFsInputStream.Read: self: %p %p %d\r\n", self, buf, len);

    UINT bytes_read;
    FRESULT rc = f_read(self->file, buf, (UINT) len, &bytes_read);

    if (rc != FR_OK) {
        xprintf("ERROR\r\n");
        return -1;
    }

    xprintf("Read: %d\r\n", (int) bytes_read);

    return (int) bytes_read;
}

void InputStream_Destroy(InputStream *self) {
    f_close(self->file);
}

InputStream InputStream_InitWithFile(FIL *file) {
    return (InputStream) {
        .file = file
    };
}
