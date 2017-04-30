#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>


#include "spi.h"


int main(int argc, char *argv[]) {
    spi_t spi;
    uint8_t buf[4] = { 0xaa, 0xbb, 0xcc, 0xdd };
    int port = 509;
    char file[1000];

    if (argc == 2) {
        sscanf(argv[1], "%i", &port);
    }
    printf("port: %i\n", port);



    /* Open spidev1.0 with mode 0 and max speed 1MHz */
    snprintf(file, 1000, "/dev/spidev%i.0", port);
    if (spi_open(&spi, file, 0, 1000000) < 0) {
        fprintf(stderr, "spi_open(): %s\n", spi_errmsg(&spi));
        exit(1);
    }

    /* Shift out and in 4 bytes */
    if (spi_transfer(&spi, buf, buf, sizeof(buf)) < 0) {
        fprintf(stderr, "spi_transfer(): %s\n", spi_errmsg(&spi));
        exit(1);
    }

    printf("shifted in: 0x%02x 0x%02x 0x%02x 0x%02x\n", buf[0], buf[1], buf[2], buf[3]);

    spi_close(&spi);

    return 0;
}
