#include "stream.h"

int InputStream_Read(InputStream *stream, void *buf, int len) {
    return ((InputStreamDef *) stream)->read(stream, buf, len);
}

void InputStream_destroy(InputStream *stream) {
    ((InputStreamDef *) stream)->destroy(stream);
}

bool OutputStream_Write(OutputStream *stream, const void *buf, int len) {
    return ((OutputStreamDef *) stream)->write(stream, buf, len);
}

void OutputStream_Destroy(OutputStream *stream) {
    ((OutputStreamDef *) stream)->destroy(stream);
}
