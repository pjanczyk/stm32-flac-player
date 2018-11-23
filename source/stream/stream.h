#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void InputStream;
typedef void OutputStream;

typedef struct {
    int (*read)(InputStream *self, void *buf, int len);
    void (*destroy)(InputStream *self);
} InputStreamDef;

typedef struct {
    bool (*write)(OutputStream *self, const void *buf, int len);
    void (*destroy)(OutputStream *self);
} OutputStreamDef;

int InputStream_Read(InputStream *stream, void *buf, int len);
void InputStream_Destory(InputStream *stream);

bool OutputStream_Write(OutputStream *stream, const void *buf, int len);
void OutputStream_Destroy(OutputStream *stream);
