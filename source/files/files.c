#define _GNU_SOURCE
#include "files.h"
#include "../log.h"
#include <stdio.h>
#include <Middlewares/Third_Party/FatFs/src/ff.h>

static void FindFlacFilesRecursively(const char *path, Files *files) {
    FRESULT res;

    DIR dir;
    res = f_opendir(&dir, path);
    if (res != FR_OK) {
        log_error("Error: f_opendir\n");
        return;
    }

    for (;;) {
        FILINFO file_info;
        res = f_readdir(&dir, &file_info);
        if (res != FR_OK) {
            xprintf("Error: f_readdir\n");
            return;
        }

        if (file_info.fname[0] == '\0') {
            break;
        }

        char *child_path;
        asprintf(&child_path, "%s/%s", path, file_info.fname);

        if (file_info.fattrib & AM_DIR) { // directory
            FindFlacFiles(child_path, files);
            free(child_path);
        } else {
            int len = strlen(file_info.fname);
            if (len >= 5 && strcmp(&file_info.fname[len - 5], ".flac") == 0) {
                files->files[files->count] = child_path;
                files->count++;
            } else {
                free(child_path);
            }
        }
    }

    f_closedir(&dir);
}

void FindFlacFiles(const char *path, Files *files) {
    *files = (Files) {
        .count = 0
    };
    FindFlacFilesRecursively(path, files);
}
