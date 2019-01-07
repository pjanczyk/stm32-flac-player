#pragma once

#define MAX_FILES_COUNT 100

typedef struct {
    char *files[MAX_FILES_COUNT];
    int count;
} Files;

void FindFlacFiles(const char *path, Files *files);
