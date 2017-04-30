#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>


#include "gpio.h"
#include "spi.h"


int main(int argc, char *argv[]) {
    gpio_t gpio_out;
    bool value;
    int port = 509;
    spi_t spi;

    if (argc == 2) {
        sscanf(argv[1], "%i", &port);
    }

    printf("port: %i\n", port);

    /* Open GPIO with output direction */
    if (gpio_open(&gpio_out, port, GPIO_DIR_OUT) < 0) {
        fprintf(stderr, "gpio_open(): %s\n", gpio_errmsg(&gpio_out));
        exit(1);
    }


    /* Open spidev1.0 with mode 0 and max speed 1MHz */
    if (spi_open(&spi, "/dev/spidev10.0", 0, 1000000) < 0) {
        fprintf(stderr, "spi_open(): %s\n", spi_errmsg(&spi));
        exit(1);
    }



    while (1) { }

    return 0;
}
