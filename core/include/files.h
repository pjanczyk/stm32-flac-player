#pragma once

#define MAX_FILES_COUNT 20
#define MAX_FILE_PATH_LENGTH 50

typedef struct {
    char files[MAX_FILES_COUNT][MAX_FILE_PATH_LENGTH + 1];
    int count;
} Files;

void FindFlacFiles(const char *path, Files *files);
