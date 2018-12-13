#pragma once

#include <Middlewares/Third_Party/FatFs/src/ff.h>

typedef struct {
    FIL *file;
} InputStream;

InputStream InputStream_InitWithFile(FIL *file);
int InputStream_Read(InputStream *self, void *buf, int len);
void InputStream_Destroy(InputStream *self);
