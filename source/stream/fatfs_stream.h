#pragma once

#include "stream.h"
#include <Middlewares/Third_Party/FatFs/src/ff.h>

InputStream *InputStream_InitWithFatFs(FIL *file);
