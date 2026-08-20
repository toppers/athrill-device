#include "athrill_exdev.h"
#include "shared_library.h"

#include <stdio.h>

int main(int argc, char *argv[])
{
    AthrillSharedLibraryHandle library;
    AthrillExDeviceType *device;
    char error_message[512];

    if (argc != 2) {
        return 1;
    }
    library = athrill_shared_library_open(
        argv[1], error_message, sizeof(error_message));
    if (library == NULL) {
        (void)fprintf(stderr, "%s\n", error_message);
        return 2;
    }
    device = (AthrillExDeviceType *)athrill_shared_library_symbol(
        library, "athrill_ex_device",
        error_message, sizeof(error_message));
    if (device == NULL) {
        (void)fprintf(stderr, "%s\n", error_message);
        athrill_shared_library_close(library);
        return 3;
    }
    if ((device->header.magicno != ATHRILL_EXTERNAL_DEVICE_MAGICNO)
        || (device->header.version != ATHRILL_EXTERNAL_DEVICE_VERSION)
        || (device->header.memory_size <= 0)
        || (device->devinit == NULL)
        || (device->supply_clock == NULL)
        || (device->cleanup == NULL)) {
        athrill_shared_library_close(library);
        return 4;
    }
    athrill_shared_library_close(library);
    return 0;
}
