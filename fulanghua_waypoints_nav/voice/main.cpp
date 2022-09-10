#include <stdio.h>
#include <dirent.h>
#include <iostream>
int main(int argc, const char**argv) {
    struct dirent *entry = nullptr;
    DIR *dp = nullptr;

    dp = opendir(argc > 1 ? argv[1] : "./");
    if (dp != nullptr) {
        while ((entry = readdir(dp)))
            printf ("%s\n", entry->d_name);
    }

    closedir(dp);
    return 0;
}
