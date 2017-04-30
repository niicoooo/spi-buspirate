# spi-buspirate

Linux kernel module for Bus Pirate / BusPirate as a SPI bus adapter

### Installation
Install the linux headers for your running kernel version, clone the repository and run:

```
$ make
$ sudo echo -n 'module bp +p' > /sys/kernel/debug/dynamic_debug/control    # enable debug mode
$ sudo insmod bp.ko
$ sudo ldattach GIGASET /dev/ttyUSB0                                       # use GIGASET ldisc!
```


### Usage

```
$ ls /sys/kernel/bp                                                        # list all Bus Pirate as SPI bus adapter available
$ cat /sys/kernel/bp/0001/status                                           # show driver status
$ echo -n 30000 > /sys/kernel/bp/0001/speed                                # set SPI speed
$ cat /sys/kernel/bp/0001/aux                                              # show AUX gpio pin number
```


### Debug

```
//#define ENABLEDEBUGIO 1
```

Uncomment this define to activate detailed debugging
