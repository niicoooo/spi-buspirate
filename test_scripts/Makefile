LDFLAGS ?=

all:test

clean:
	find . -type f | xargs -n 5 touch
	rm -f *.elf *.o
	cd c-periphery && make clean
	
%.o: %.c
	$(CC) -c -Ic-periphery/src $< -o $@

test: test_spi.elf test_gpio.elf test_block.elf

test_spi.elf: test_spi.o c-periphery/periphery.a
	$(CC) -o $@ $(LDFLAGS) $^ -Lperiphery

test_gpio.elf: test_gpio.o c-periphery/periphery.a
	$(CC) -o $@ $(LDFLAGS) $^ -Lperiphery

test_block.elf: test_block.o c-periphery/periphery.a
	$(CC) -o $@ $(LDFLAGS) $^ -Lperiphery

c-periphery/periphery.a:
	cd c-periphery && make

.PHONY: test all clean
