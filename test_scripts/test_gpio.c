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

    if (argc == 2) {
        sscanf(argv[1], "%i", &port);
    }
    printf("port: %i\n", port);



    /* Open GPIO with output direction */
    if (gpio_open(&gpio_out, port, GPIO_DIR_OUT) < 0) {
        fprintf(stderr, "gpio_open(): %s\n", gpio_errmsg(&gpio_out));
        exit(1);
    }

    /* Read input GPIO into value */
    if (gpio_read(&gpio_out, &value) < 0) {
        fprintf(stderr, "gpio_read(): %s\n", gpio_errmsg(&gpio_out));
        exit(1);
    }
    printf("read: %i\n", value);

    /* Write output GPIO with !value */
    if (gpio_write(&gpio_out, !value) < 0) {
        fprintf(stderr, "gpio_write(): %s\n", gpio_errmsg(&gpio_out));
        exit(1);
    }

    gpio_close(&gpio_out);
    printf("done\n");

    return 0;
}
