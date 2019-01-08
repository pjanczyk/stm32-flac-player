#pragma once

#include "Inc/term_io.h"

#define log_debug(...)
#define log_error(...) xprintf(__VA_ARGS__)
#define log_fatal_and_die(...) { xprintf(__VA_ARGS__); while (1) {} }