/*
 * A Linux kernel module for Bus Pirate / BusPirate as a SPI bus adapter
 *
 * Author: niicoooo <1niicoooo1@gmail.com>
 * Copyright (C) 2017 by niicoooo <1niicoooo1@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with keysniffer. If not, see <http://www.gnu.org/licenses/>.
 */


#define pr_fmt(fmt) "spi-bp: " fmt

#define DEBUG 1


#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/semaphore.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/tty.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/version.h>
#include <generated/utsrelease.h>



#define ENABLESPI 1
#define ENABLESPIDEV 1
#define ENABLEGPIO 1
// #define ENABLEDEBUGIO 1



#define RXTIMEOUTMS 100
#define RXBUFFERSIZE 1024
#define BOOTMSGSIZE 200



#define BPSPI_STATE_STARTING 0
#define BPSPI_STATE_OK 1
#define BPSPI_STATE_ERROR 2
#define BPSPI_STATE_STOP 3



static int gpio_base = -1;
static int spi_base = -1;



struct bp {
    struct list_head list;
    int id;
    char name[10];
    struct kobject kobj;

    atomic_t status;
    wait_queue_head_t status_queue;
    struct mutex busy;

    unsigned long BPconfig;
    unsigned long SPIconfig;
    unsigned long SPISpeedconfig;
    char bootmsg[BOOTMSGSIZE];

    char rx_buffer[RXBUFFERSIZE];
    atomic_t rx_len;
    struct semaphore rx_semaphore;
    wait_queue_head_t rx_queue;
    struct timer_list rx_timer;
    atomic_t rx_timeout;

    struct tty_struct *tty;
    struct task_struct *task;
#ifdef ENABLEGPIO
    struct gpio_chip gpio_chip;
#endif
#ifdef ENABLESPI
    struct spi_master *spi_master;
    char spi_name[32];
#ifdef ENABLESPIDEV
    struct spi_device *spi_device;
#endif
#endif
};



struct kobject module_kobj;
static DEFINE_MUTEX(list_lock); /* Must be locked to r/w list */
static LIST_HEAD(list);



static const int gpio_pin_offset[] = { // 3.6 0100wxyz - Configure peripherals w=power, x=pull-ups, y=AUX, z=CS
    0, 1, 3
};



/* ------ RX Timeout ------ */



static void rx_timout_reset(struct bp *bp) {
    atomic_set(&bp->rx_timeout, 0);
    wake_up(&bp->rx_queue);
    mod_timer(&bp->rx_timer, jiffies + msecs_to_jiffies(RXTIMEOUTMS));
}

static void rx_timeout_callback(unsigned long data) {
    struct bp *bp = (struct bp*)data;
    atomic_set(&bp->rx_timeout, 1);
    wake_up(&bp->rx_queue);
}



/* ------ TX & RX ------ */



static int bp_io_tx(struct bp *bp, char *tx_buf, int tx_len) /* Send data over serial line */
{
    int rc, i;
    BUG_ON(!mutex_is_locked(&bp->busy));

    if (atomic_read(&bp->status) > BPSPI_STATE_OK) {
        pr_debug("(%s) TX cancelled!\n", bp->tty->name);
        return -1;
    }
#ifdef ENABLEDEBUGIO
    char s[50];
    snprintf(s, 50, "BPSpi: (%s) -> ", bp->tty->name);
    print_hex_dump(KERN_DEBUG, s, DUMP_PREFIX_NONE, 16, 8, tx_buf, tx_len, true);
#endif
    i = 0;
    while (i < tx_len) {
        rc = bp->tty->ops->write(bp->tty, &tx_buf[i], tx_len - i);
        if (rc < 0) {
            pr_debug("(%s) tty->ops->write error: %i", bp->tty->name, rc);
            return -1;
        }
        i += rc;
        tty_wait_until_sent(bp->tty, 0);
    }
    return 0;
}

static int bp_io_tx_char(struct bp *bp, char c) /* Send data over serial line */
{
    char buffer[1];
    BUG_ON(!mutex_is_locked(&bp->busy));

    buffer[0] = c;
    return bp_io_tx(bp, buffer, 1);
}

static int bp_io_rx(struct bp *bp, int rx_expected) /* Wait for data from serial line */
{
    BUG_ON(!mutex_is_locked(&bp->busy));
    if (atomic_read(&bp->status) > BPSPI_STATE_OK) {
        pr_debug("(%s) RX cancelled!\n", bp->tty->name);
        return 0;
    }
    rx_timout_reset(bp);
    wait_event(bp->rx_queue, (atomic_read(&bp->rx_len) >= rx_expected) || (atomic_read(&bp->rx_timeout) != 0));
    return atomic_read(&bp->rx_len);
}

static void bp_io_rx_flush(struct bp *bp) /* Flush RX buffer */
{
    BUG_ON(!mutex_is_locked(&bp->busy));
    down(&bp->rx_semaphore);
    atomic_set(&bp->rx_len, 0);
    up(&bp->rx_semaphore);
}

static void bp_io_rx_get(struct bp *bp, char *buffer, int len) /* Get RX data from buffer */
{
    BUG_ON(!mutex_is_locked(&bp->busy));
    BUG_ON(len > atomic_read(&bp->rx_len));
    down(&bp->rx_semaphore);
    memmove(buffer, &bp->rx_buffer, len);
    atomic_sub(len, &bp->rx_len);
    memmove(&bp->rx_buffer, &(bp->rx_buffer[atomic_read(&bp->rx_len)]), len);
    up(&bp->rx_semaphore);
}

#define MSLEEP_INTERRUPTIBLE(A) if (msleep_interruptible(A)) { err = __LINE__; goto fail; }
#define BP_TX(BP, A, B) if (bp_io_tx(BP, A, B) < 0) { err = __LINE__; goto fail; }
#define BP_TX_CHAR(BP, A) if (bp_io_tx_char(BP, A) < 0) { err = __LINE__; goto fail; }
#define BP_RX(BP, A) if (bp_io_rx(BP, A) != A) { err = __LINE__; goto fail; }
#define BP_RX_GET_AND_MEMCMP(BP, A, B) { \
    char buffer[B + 1]; \
    bp_io_rx_get(BP, buffer, B); \
    if (memcmp(buffer, A, B)) { err = __LINE__; goto fail; } \
}



/* ------  SPI callbacks ------ */



#ifdef ENABLESPI
int bp_spi_transfer_one(struct spi_master *master, struct spi_device *spi, struct spi_transfer *transfer)
{
    struct bp *bp;
    int err, i;
    char buffer[10];
    const char *tx_buf;
    char *rx_buf;
    char s[50];

    bp = (struct bp *)spi_master_get_devdata(master);
    tx_buf = transfer->tx_buf;
    rx_buf = transfer->rx_buf;
    pr_debug("(%s) spi_transfer_one (len:%d tx_nbits:%d rx_nbits:%d)", bp->tty->name, transfer->len, transfer->tx_nbits, transfer->rx_nbits);
    snprintf(s, 50, "BPSpi: (%s) spi_transfer_one: -> ", bp->tty->name);
    print_hex_dump(KERN_DEBUG, s, DUMP_PREFIX_NONE, 16, 8, tx_buf, transfer->len, true);
    mutex_lock(&bp->busy);
    if (atomic_read(&bp->status) > BPSPI_STATE_OK) {
        mutex_unlock(&bp->busy);
        pr_debug("(%s) SPI cancelled!\n", bp->tty->name);
        return -1;
    }

    for (i = 0; i < transfer->len; i++) {
        BP_TX_CHAR(bp, 0x10);
        BP_RX(bp, 1);
        BP_RX_GET_AND_MEMCMP(bp, "\x01", 1);
        BP_TX_CHAR(bp, tx_buf[i]);
        BP_RX(bp, 1);
        bp_io_rx_get(bp, buffer, 1);
        rx_buf[i] = buffer[0];
    }
    mutex_unlock(&bp->busy);
    snprintf(s, 50, "BPSpi: (%s) spi_transfer_one: <- ", bp->tty->name);
    print_hex_dump(KERN_DEBUG, s, DUMP_PREFIX_NONE, 16, 8, rx_buf, transfer->len, true);
    return 0;

fail:
    pr_err("(%s) spi_transfer_one aborted (%s:%d)!\n", bp->tty->name, __FUNCTION__, err);
    if (atomic_read(&bp->status) != BPSPI_STATE_STOP) {
        atomic_set(&bp->status, BPSPI_STATE_ERROR);
    }
    mutex_unlock(&bp->busy);
    wake_up(&bp->status_queue);
    return -1;
}

static int bp_spi_setup(struct spi_device *spi)
{
    struct bp *bp;
    int err;

    bp = spi_master_get_devdata(spi->master);
    mutex_lock(&bp->busy);
    if (atomic_read(&bp->status) > BPSPI_STATE_OK) {
        pr_debug("(%s) bp_spi_setup aborted\n", bp->tty->name);
        mutex_unlock(&bp->busy);
        return -1;
    }

    if (spi->mode & SPI_CPHA) {
        set_bit(2, &bp->SPIconfig);
    } else {
        clear_bit(2, &bp->SPIconfig);
    }
    if (spi->mode & SPI_CPOL) {
        set_bit(1, &bp->SPIconfig);
    } else {
        clear_bit(1, &bp->SPIconfig);
    }
    pr_debug("(%s) bp_spi_setup(val:%x SPIconfig reg:%lx)\n", bp->tty->name, spi->mode, bp->SPIconfig);

    BP_TX_CHAR(bp, (0x80 | (0x0F & bp->SPIconfig)));
    BP_RX(bp, 1);
    BP_RX_GET_AND_MEMCMP(bp, "\x01", 1);
    mutex_unlock(&bp->busy);
    pr_debug("(%s) bp_spi_setup done\n", bp->tty->name);
    return 0;

fail:
    pr_err("(%s) bp_spi_setup aborted (%s:%d)!\n", bp->tty->name, __FUNCTION__, err);
    if (atomic_read(&bp->status) != BPSPI_STATE_STOP) {
        atomic_set(&bp->status, BPSPI_STATE_ERROR);
    }
    mutex_unlock(&bp->busy);
    wake_up(&bp->status_queue);
    return -1;
}
#endif


/* ------ GPIO callbacks ------ */


#ifdef ENABLEGPIO
static int bp_gpio_get_value(struct gpio_chip *gc, unsigned offset)
{
    struct bp *bp;
    int r = 0;

    BUG_ON(offset >= 3);
    bp = container_of(gc, struct bp, gpio_chip);
    mutex_lock(&bp->busy);
    if(test_bit(gpio_pin_offset[offset], &bp->BPconfig)) {
        r = 1;
    }
    mutex_unlock(&bp->busy);
    pr_debug("(%s) gpio_get_value (offset:%d val:%d)\n", bp->tty->name, offset, r);
    return r;
}

static void bp_gpio_set_value(struct gpio_chip *gc, unsigned offset, int val)
{
    struct bp *bp;
    int err;

    BUG_ON(offset >= 3);
    bp = container_of(gc, struct bp, gpio_chip);
    mutex_lock(&bp->busy);
    if (atomic_read(&bp->status) > BPSPI_STATE_OK) {
        pr_debug("(%s) gpio_set_value aborted\n", bp->tty->name);
        mutex_unlock(&bp->busy);
        return;
    }

    if (val) {
        set_bit(gpio_pin_offset[offset], &bp->BPconfig);
    } else {
        clear_bit(gpio_pin_offset[offset], &bp->BPconfig);
    }
    pr_debug("(%s) gpio_set_value (offset:%d val:%d BPconfig reg:%lx)\n", bp->tty->name, offset, val, bp->BPconfig);

    BP_TX_CHAR(bp, (0x40 | (0x0F & bp->BPconfig)));
    BP_RX(bp, 1);
    BP_RX_GET_AND_MEMCMP(bp, "\x01", 1);
    mutex_unlock(&bp->busy);
    pr_debug("(%s) gpio_set_value done\n", bp->tty->name);
    return;

fail:
    pr_err("(%s) gpio_set_value aborted (%s:%d)!\n", bp->tty->name, __FUNCTION__, err);
    if (atomic_read(&bp->status) != BPSPI_STATE_STOP) {
        atomic_set(&bp->status, BPSPI_STATE_ERROR);
    }
    mutex_unlock(&bp->busy);
    wake_up(&bp->status_queue);
    return;
}

static int bp_gpio_get_direction(struct gpio_chip *gc, unsigned offset)
{
    struct bp *bp;

    BUG_ON(offset >= 3);
    bp = container_of(gc, struct bp, gpio_chip);
    pr_debug("(%s) gpio_get_direction (offset:%d)\n", bp->tty->name, offset);
    return 0;
}

static int bp_gpio_direction_output(struct gpio_chip *gc, unsigned offset, int val)
{
    struct bp *bp;

    BUG_ON(offset >= 3);
    bp = container_of(gc, struct bp, gpio_chip);
    pr_debug("(%s) gpio_direction_output (offset:%d val:%d)\n", bp->tty->name, offset, val);
    return 0;
}

static int bp_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
    struct bp *bp;

    BUG_ON(offset >= 3);
    bp = container_of(gc, struct bp, gpio_chip);
    pr_debug("(%s) gpio_direction_input (offset:%d)\n", bp->tty->name, offset);
    return -EPERM;
}
#endif



/* ------ Sysfs ------ */


static void static_kobj_release(struct kobject *kobj)
{ 
    pr_info("(%s) kobj realeased\n", kobj->name);
}

static struct kobj_type static_kobj_ktype = {
    .release        = static_kobj_release,
    .sysfs_ops      = &kobj_sysfs_ops,
};

static const int bp_status_id[] = {
    BPSPI_STATE_STARTING,
    BPSPI_STATE_OK,
    BPSPI_STATE_ERROR,
    BPSPI_STATE_STOP,
};

static const char* bp_status_name[] = {
    "starting",
    "ok",
    "error",
    "stopping",
    "???",
};

static ssize_t status_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    int i;
    struct bp *bp = container_of(kobj , struct bp, kobj);
    int s = atomic_read(&bp->status);
    for (i = 0; i < (sizeof(bp_status_id) / sizeof(int)); i++) {
        if (bp_status_id[i] == s) {
            break;
        }
    }
    return scnprintf(buf, PAGE_SIZE, "%s",  bp_status_name[i]);
}

static struct kobj_attribute status_attribute = __ATTR_RO(status);

static const struct attribute *attrs1[] =
{
    &status_attribute.attr,
    NULL,
};

static ssize_t bootmsg_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    struct bp *bp = container_of(kobj , struct bp, kobj);
    return scnprintf(buf, PAGE_SIZE, "%s",  bp->bootmsg);
}

static struct kobj_attribute bootmsg_attribute = __ATTR_RO(bootmsg);

#ifdef ENABLEGPIO
#define SYSFS_SHOW(NAME, OFFSET) \
static ssize_t NAME##_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf) \
{ \
    struct bp *bp = container_of(kobj , struct bp, kobj); \
    return scnprintf(buf, PAGE_SIZE, "%i",  bp->gpio_chip.base + OFFSET); \
}

SYSFS_SHOW(cs, 0);
SYSFS_SHOW(aux, 1);
SYSFS_SHOW(pwr, 2);

static struct kobj_attribute cs_attribute = __ATTR_RO(cs);
static struct kobj_attribute aux_attribute = __ATTR_RO(aux);
static struct kobj_attribute pwr_attribute = __ATTR_RO(pwr);
#endif

#ifdef ENABLESPI
static ssize_t spi_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    struct bp *bp = container_of(kobj , struct bp, kobj);
    return scnprintf(buf, PAGE_SIZE, "%i",  bp->spi_master->bus_num);
}

static struct kobj_attribute spi_attribute = __ATTR_RO(spi);

static const int bp_speed[] = {
    30000,
    125000,
    250000,
    1000000,
    2000000,
    2600000,
    4000000,
    8000000,
};

static ssize_t speed_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
    struct bp *bp = container_of(kobj , struct bp, kobj);
    BUG_ON(bp->SPISpeedconfig >= 0x08);
    return scnprintf(buf, PAGE_SIZE, "%i", bp_speed[bp->SPISpeedconfig]);
}

static ssize_t speed_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t len)
{
    int s;
    int i;
    int err;
    struct bp *bp = container_of(kobj , struct bp, kobj);
    if(kstrtoint(buf, 10, &s)) {
        return -EINVAL;
    }

    for (i = 0; i < (sizeof(bp_speed) / sizeof(int)); i++) {
        if (bp_speed[i] == s) {
            break;
        }
    }
    if (i == sizeof(bp_speed) / sizeof(int)) {
        pr_debug("(%s) speed_store invalid value(%s)\n", bp->tty->name, buf);
        return -EINVAL;
    }
    bp->SPISpeedconfig = i;

    mutex_lock(&bp->busy);
    if (atomic_read(&bp->status) > BPSPI_STATE_OK) {
        pr_debug("(%s) speed_store aborted\n", bp->tty->name);
        mutex_unlock(&bp->busy);
        return -1;
    }

    pr_debug("(%s) speed_store(SPISpeedconfig reg:%lx speed: %i)\n", bp->tty->name, bp->SPISpeedconfig, s);

    BP_TX_CHAR(bp, (0x60 | (0x0F & bp->SPISpeedconfig)));
    BP_RX(bp, 1);
    BP_RX_GET_AND_MEMCMP(bp, "\x01", 1);
    mutex_unlock(&bp->busy);
    pr_debug("(%s) speed_store done\n", bp->tty->name);
    return len;

fail:
    pr_err("(%s) speed_store aborted (%s:%d)!\n", bp->tty->name, __FUNCTION__, err);
    if (atomic_read(&bp->status) != BPSPI_STATE_STOP) {
        atomic_set(&bp->status, BPSPI_STATE_ERROR);
    }
    mutex_unlock(&bp->busy);
    wake_up(&bp->status_queue);
    return -1;
}

static struct kobj_attribute speed_attribute = __ATTR_RW(speed);
#endif


static const struct attribute *attrs2[] =
{
    &bootmsg_attribute.attr,
#ifdef ENABLEGPIO
    &cs_attribute.attr,
    &aux_attribute.attr,
    &pwr_attribute.attr,
#endif
#ifdef ENABLESPI
    &spi_attribute.attr,
    &speed_attribute.attr,
#endif
    NULL,
};



/* ------ Task mngt, BP Init, Dev init & remove ------ */



static void __bp_dev_remove(struct bp *bp) /* Remove spi_master, spi_device, gpio_chip */
{
    sysfs_remove_files(&bp->kobj, attrs2);
#ifdef ENABLESPI
#ifdef ENABLESPIDEV
    spi_unregister_device(bp->spi_device);
#endif
    sysfs_remove_link(&bp->kobj, bp->spi_name);
    spi_unregister_master(bp->spi_master);
#endif
#ifdef ENABLEGPIO
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,17,8)
    WARN_ON(gpiochip_remove(&bp->gpio_chip));
#else
    gpiochip_remove(&bp->gpio_chip);
#endif
#endif
};

static int __bp_dev_add(struct bp *bp) /* Create spi_master, spi_device, gpio_chip */
{
    int rc;

    if (atomic_read(&bp->status) > BPSPI_STATE_OK) {
        return -1;
    }

    /* ------ gpio_chip create ------ */
#ifdef ENABLEGPIO
    bp->gpio_chip.label = "BPSpi GPIO";
    bp->gpio_chip.owner = THIS_MODULE;
    bp->gpio_chip.get_direction = bp_gpio_get_direction;
    bp->gpio_chip.direction_input = bp_gpio_direction_input;
    bp->gpio_chip.direction_output = bp_gpio_direction_output;
    bp->gpio_chip.get = bp_gpio_get_value;
    bp->gpio_chip.set = bp_gpio_set_value;
    bp->gpio_chip.can_sleep = true;
    bp->gpio_chip.ngpio = 3;
    bp->gpio_chip.base = gpio_base;
    if ((rc = gpiochip_add(&bp->gpio_chip))) {
        pr_err("(%s) gpiochip_add failed (rc = %d)\n", bp->tty->name, rc);
        return -1;
    }
#endif

    /* ------ spi_master create ------ */
#ifdef ENABLESPI
    bp->spi_master = spi_alloc_master(bp->tty->dev, 0);
    if (!bp->spi_master) {
        pr_err("(%s) spi_alloc_master failed\n", bp->tty->name);
        goto remove_gpio;
    }
    spi_master_set_devdata(bp->spi_master, bp);
    bp->spi_master->bus_num = spi_base;
    bp->spi_master->bits_per_word_mask = SPI_BIT_MASK(8);
    bp->spi_master->mode_bits = SPI_CPHA | SPI_CPOL;
    bp->spi_master->transfer_one = bp_spi_transfer_one;
    bp->spi_master->setup =  bp_spi_setup;
    bp->spi_master->min_speed_hz = 30000;
    bp->spi_master->max_speed_hz = 8000000;
    if ((rc = spi_register_master(bp->spi_master))) {
        pr_err("(%s) spi_register_master failed (rc = %d)\n", bp->tty->name, rc);
        spi_master_put(bp->spi_master);
        goto remove_gpio;
    }
    scnprintf(bp->spi_name, 32, "%s",  dev_name(&bp->spi_master->dev));
    if((rc = sysfs_create_link(&bp->kobj, &bp->spi_master->dev.kobj, bp->spi_name))) {
        pr_err("(%s) sysfs_create_link failed (rc = %d)\n", bp->tty->name, rc);
        spi_unregister_master(bp->spi_master);
        goto remove_gpio;
    }

    /* ------ spi_device create ------ */
#ifdef ENABLESPIDEV
    bp->spi_device = spi_alloc_device(bp->spi_master);
    if (!bp->spi_device) {
        pr_err("(%s) spi_alloc_device failed\n", bp->tty->name);
        goto remove_spi;
    }
    snprintf(bp->spi_device->modalias, SPI_NAME_SIZE, "spidev");
    if ((rc = spi_add_device(bp->spi_device))) {
        pr_err("(%s) spi_add_device failed (rc = %d)\n", bp->tty->name, rc);
        spi_dev_put(bp->spi_device);
        goto remove_spi;
    }
#endif
#endif


    /* ------ Sysfs setup ------ */
    if ((rc = sysfs_create_files(&bp->kobj, attrs2))) {
        pr_err("(%s) sysfs_create_files 2 failed (rc = %d)\n", bp->tty->name, rc);
        goto remove_spidev;
    }

    /* ------ Display result ------ */
#ifdef ENABLESPI
#ifdef ENABLESPIDEV
    pr_info("(%s) spi_device added: %s\n", bp->tty->name, dev_name(&bp->spi_device->dev));
#endif
    pr_info("(%s) spi_master added: spi%u\n", bp->tty->name, bp->spi_master->bus_num);
#endif
#ifdef ENABLEGPIO
    pr_info("(%s) gpiochip added: pins %d to %d\n", bp->tty->name, bp->gpio_chip.base, bp->gpio_chip.base + 2);
#endif
    return 0;

    /* ------ error mngt ------ */
remove_spidev:
#ifdef ENABLESPIDEV
    spi_unregister_device(bp->spi_device);
#endif
remove_spi:
#ifdef ENABLESPI
    sysfs_remove_link(&bp->kobj, bp->spi_name);
    spi_unregister_master(bp->spi_master);
#endif
remove_gpio:
#ifdef ENABLEGPIO
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,17,8)
    WARN_ON(gpiochip_remove(&bp->gpio_chip));
#else
    gpiochip_remove(&bp->gpio_chip);
#endif
#endif
    return -1;
}

static int __bp_reset(struct bp *bp) /* Reset Bus Pirate and set spi mode */
{
    int i, l, err;
    char buffer[BOOTMSGSIZE];
    BUG_ON(BOOTMSGSIZE <= 10);

    bp_io_rx_flush(bp);
    for (i = 0; i < 20; i++) { /* Send 0x00 20 times */
        BP_TX(bp, "\x00", 1);
        MSLEEP_INTERRUPTIBLE(10);
        bp_io_rx_flush(bp);
    }
    MSLEEP_INTERRUPTIBLE(10);
    BP_TX(bp, "\x0F", 1); /* Send 0x0F to reset */
    MSLEEP_INTERRUPTIBLE(500);
    bp_io_rx_flush(bp);
    for (i = 0; i < 10; i++) { /* Send <Enter> 10 times */
        BP_TX(bp, "\n", 1);
        MSLEEP_INTERRUPTIBLE(10);
        bp_io_rx_flush(bp);
    }
    BP_TX(bp, "#\n", 2); /* Send # to reset */
    MSLEEP_INTERRUPTIBLE(500);
    l = bp_io_rx(bp, RXBUFFERSIZE - 1);
    pr_debug("(%s) bootmsg len: %d\n", bp->tty->name, l);
    if (l > BOOTMSGSIZE - 1) {
        l = BOOTMSGSIZE - 1;
    }
    bp_io_rx_get(bp, buffer, l);
    buffer[l] = 0;
    strcpy(bp->bootmsg, buffer);
    bp_io_rx_flush(bp);
    for (i = 0; i < 25; i++) { /* Send 0x00 25 times */
        BP_TX(bp, "\x00", 1);
        if (bp_io_rx(bp, 5) >= 5) {
            bp_io_rx_get(bp, buffer, 5);
            if (!memcmp(buffer, "BBIO1", 5))
                break;
        }
        bp_io_rx_flush(bp);
    }
    if (i == 25) {
        err = __LINE__;
        goto fail;
    }
    MSLEEP_INTERRUPTIBLE(10);
    bp_io_rx_flush(bp);
    BP_TX(bp, "\x01", 1); /* Send 0x01 in bitbang mode to enter raw SPI mode */
    BP_RX(bp, 4);
    BP_RX_GET_AND_MEMCMP(bp, "SPI1", 4);

    BP_TX_CHAR(bp, (0x40 | (0x0F & bp->BPconfig)));
    BP_RX(bp, 1);
    BP_RX_GET_AND_MEMCMP(bp, "\x01", 1);
    BP_TX_CHAR(bp, (0x80 | (0x0F & bp->SPIconfig)));
    BP_RX(bp, 1);
    BP_RX_GET_AND_MEMCMP(bp, "\x01", 1);
    MSLEEP_INTERRUPTIBLE(10);
    bp_io_rx_flush(bp);
    return 0;
fail:
    pr_err("(%s) BP reset & initialization failed (%d)\n", bp->tty->name, err);
    return -1;
}


static int thread_function(void *data) /* Management thread */
{
    struct bp *bp = (struct bp*)data;

    atomic_set(&bp->status, BPSPI_STATE_STARTING);

    pr_debug("(%s) mngt task: started!\n", bp->tty->name);
    mutex_lock(&bp->busy);
    if (__bp_reset(bp) < 0) {
        mutex_unlock(&bp->busy);
        goto fail;
    }
    mutex_unlock(&bp->busy);
    if (__bp_dev_add(bp) < 0) {
        goto fail;
    }
    pr_debug("(%s) mngt task: initialization done!\n", bp->tty->name);
    atomic_set(&bp->status, BPSPI_STATE_OK);

    while(wait_event_interruptible(bp->status_queue, ((atomic_read(&bp->status) != BPSPI_STATE_OK) | kthread_should_stop())) != 0) { }

    __bp_dev_remove(bp);
    pr_debug("(%s) mngt task: stopped!\n", bp->tty->name);
    do_exit(0);
    return 0;

fail:
    pr_debug("(%s) mngt task: aborted\n", bp->tty->name);
    atomic_set(&bp->status, BPSPI_STATE_ERROR);
    wake_up(&bp->status_queue);
    do_exit(-1);
    return -1;
}



/* ------ LDISC part ------ */



static void bp_ldisc_receive(struct tty_struct *tty, const unsigned char *rx_buf, char *fp, int count) /* Receive data from BP */
{
    struct bp *bp = tty->disc_data;
    int l;

    if (unlikely(!bp)) {
        pr_err("(%s) ldisc_receive aborted\n", bp->tty->name);
        return;
    }
    down(&bp->rx_semaphore);
#ifdef ENABLEDEBUGIO
    char s[50];
    snprintf(s, 50, "BPSpi: (%s) <- ", tty->name);
    print_hex_dump(KERN_DEBUG, s, DUMP_PREFIX_NONE, 16, 8, rx_buf, count, true);
#endif
    l = min(count, RXBUFFERSIZE - atomic_read(&bp->rx_len));
    if (l != count) {
        pr_err("(%s) ldisc_receive overflow\n", bp->tty->name);
    }
    memcpy(&bp->rx_buffer[atomic_read(&bp->rx_len)], rx_buf, l);
    atomic_add(l, &bp->rx_len);
    rx_timout_reset(bp);
    up(&bp->rx_semaphore);
    wake_up(&bp->rx_queue);
    return;
}

static int __bp_getid(void)
{
    int id = 1;

    struct list_head *ptr;
    struct bp *entry;
    list_for_each(ptr, &list) {
        entry = list_entry(ptr, struct bp, list);
        if (id == entry->id) {
            id++;
            ptr = list.next;
        }
    }
    return id;
}

static int bp_ldisc_open(struct tty_struct *tty)
{
    struct bp *bp;
    struct ktermios ktermios;
    int rc;

    /* ------ Struct allocation ------ */
    pr_debug("(%s) ldisc_open\n", tty->name);
    bp = kzalloc(sizeof(struct bp), GFP_KERNEL);
    if (!bp) {
        pr_err("(%s) bp_ldisc_open: kzalloc failed\n", tty->name);
        return -ENOMEM;
    }
    tty->disc_data = bp;
    bp->tty = tty;
    bp->bootmsg[0] = 0;

    /* ------ Sysfs setup ------ */
    mutex_lock(&list_lock);
    list_add((struct list_head*)bp, &list);
    bp->id = __bp_getid();
    mutex_unlock(&list_lock);
    snprintf(bp->name, 10, "%04i", bp->id);
    memset(&bp->kobj, 0, sizeof (module_kobj));
    if((rc = kobject_init_and_add(&bp->kobj, &static_kobj_ktype, &module_kobj, bp->name))) {
        pr_err("(%s) thread kobject_init_and_add failed (rc = %d)\n", tty->name, rc);
        mutex_lock(&list_lock);
        list_del((struct list_head*)bp);
        mutex_unlock(&list_lock);
        goto remove_struct;
    }
    if((rc = sysfs_create_link(&bp->kobj, &tty->dev->kobj ,tty->name))) {
        pr_err("(%s) sysfs_create_link failed (rc = %d)\n", tty->name, rc);
        goto remove_kobj;
    }
    if ((rc = sysfs_create_files(&bp->kobj, attrs1))) {
        sysfs_remove_link(&bp->kobj, tty->name);
        pr_err("(%s) sysfs_create_files 1 failed (rc = %d)\n", bp->tty->name, rc);
        goto remove_kobj;
    }

    /* ------ Misc variables setup ------ */
    atomic_set(&bp->status, BPSPI_STATE_STARTING);
    mutex_init(&bp->busy);
    init_waitqueue_head(&bp->status_queue);
    bp->BPconfig = 0x00; /* Configure peripherals w=power, x=pull-ups, y=AUX, z=CS */
    bp->SPIconfig = 0x02; /* SPI config, w=HiZ/3.3v, x=CKP idle, y=CKE edge, z=SMP sample */
    bp->SPISpeedconfig = 0x00; /* SPI speed 000=30kHz, 001=125kHz, 010=250kHz, 011=1MHz, 100=2MHz, 101=2.6MHz,  110=4MHz, 111=8MHz */

    /* ------ RX variables setup ------ */
    init_waitqueue_head(&bp->rx_queue);
    sema_init(&bp->rx_semaphore, 1);
    atomic_set(&bp->rx_len, 0);
    atomic_set(&bp->rx_timeout, 0);
    setup_timer(&bp->rx_timer, rx_timeout_callback, (unsigned long)(bp));

    /* ------ TTY setup ------ */
    down_write(&tty->termios_rwsem);
    pr_debug("(%s) termios o:%d i:%d i:%x o:%x c:%x l:%x\n", tty->name, tty_termios_baud_rate(&tty->termios), tty_termios_input_baud_rate(&tty->termios), tty->termios.c_iflag, tty->termios.c_oflag, tty->termios.c_cflag, tty->termios.c_lflag);
    up_write(&tty->termios_rwsem);
    ktermios = tty->termios;
    ktermios.c_iflag = 0;
    ktermios.c_oflag = 0;
    ktermios.c_cflag = 0x800018b2; /* Magic! */
    ktermios.c_lflag = 0;
    tty_set_termios(tty, &ktermios);
    down_write(&tty->termios_rwsem);
    tty_termios_encode_baud_rate(&tty->termios, 115200, 115200);
    pr_debug("(%s) termios o:%d i:%d i:%x o:%x c:%x l:%x\n", tty->name, tty_termios_baud_rate(&tty->termios), tty_termios_input_baud_rate(&tty->termios), tty->termios.c_iflag, tty->termios.c_oflag, tty->termios.c_cflag, tty->termios.c_lflag);
    up_write(&tty->termios_rwsem);
    tty->receive_room = 65536;
    tty_driver_flush_buffer(tty);

    /* ------ Thread setup ------ */
    bp->task = kthread_run(&thread_function, (void*)bp, "bp");
    if (!bp->task) {
        pr_err("(%s) bp_ldisc_open: kthread_run failed\n", tty->name);
        goto remove_misc;
    }

    pr_info("(%s) line discipline opened\n", tty->name);
    return 0;

    /* ------ error mngt ------ */
remove_misc:
    mutex_destroy(&bp->busy);
    sysfs_remove_link(&bp->kobj, tty->name);
    sysfs_remove_files(&bp->kobj, attrs1);
remove_kobj:
    mutex_lock(&list_lock);
    list_del((struct list_head*)bp);
    mutex_unlock(&list_lock);
    kobject_del(&bp->kobj);
    kobject_put(&bp->kobj);
remove_struct:
    kfree(bp);
    return -ENOMEM;
}

static void bp_ldisc_close(struct tty_struct *tty)
{
    struct bp *bp = tty->disc_data;

    if (bp) {
        pr_debug("(%s) ldisc_close\n", tty->name);
        sysfs_remove_files(&bp->kobj, attrs1);
        atomic_set(&bp->status, BPSPI_STATE_STOP);
        wake_up(&bp->status_queue);
        kthread_stop(bp->task);
        del_timer_sync(&bp->rx_timer);
        sysfs_remove_link(&bp->kobj, bp->tty->name);
        mutex_destroy(&bp->busy);
        mutex_lock(&list_lock);
        list_del((struct list_head*)bp);
        mutex_unlock(&list_lock);
        kobject_del(&bp->kobj);
        kobject_put(&bp->kobj);
        kfree(bp);
        tty->disc_data = 0;
        pr_info("(%s) line discipline closed\n", tty->name);
    }
};

static int bp_ldisc_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{
    pr_debug("(%s) ldisc_ioctl: %x\n", tty->name, cmd);
    return 0;
}

static int bp_ldisc_hangup(struct tty_struct *tty)
{
    pr_debug("(%s) ldisc_hangup\n", tty->name);
    bp_ldisc_close(tty);
    return 0;
}

static struct tty_ldisc_ops bp_ldisc = {
    .owner = THIS_MODULE,
    .magic = TTY_LDISC_MAGIC,
    .name = "buspirate_spi",
    .open = bp_ldisc_open, /* This function is called when the line discipline is associated */
    .close = bp_ldisc_close, /* This function is called when the line discipline is being shutdown */
    .hangup = bp_ldisc_hangup, /* Called on a hangup. Tells the discipline that it should cease I/O to the tty driver */
    .ioctl = bp_ldisc_ioctl, /* This function is called when the user requests an ioctl which is not handled by the tty layer or the low-level tty driver */
    .receive_buf = bp_ldisc_receive, /* This function is called by the low-level tty driver to send characters received by the hardware to the line discpline for processing */
};



/* ------ MODULE part ------ */



module_param(gpio_base, int, 0644);
module_param(spi_base, int, 0644);
MODULE_PARM_DESC(base, "gpio pin number");

MODULE_AUTHOR("niicoooo <1niicoooo1@gmail.com>");
MODULE_DESCRIPTION("BusPirate Spi driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0");


static int __init buspirate_module_init(void)
{
    int rc;

    pr_debug("Let's go!\n");
    memset(&module_kobj, 0, sizeof (module_kobj));
    if((rc = kobject_init_and_add(&module_kobj, &static_kobj_ktype, kernel_kobj, "bp"))) {
        pr_err("kobject_init_and_add failed (rc = %d)\n", rc);
        return -ENOMEM;
    }
    if ((rc = tty_register_ldisc(N_GIGASET_M101, &bp_ldisc))) {
        pr_err("module tty_register_ldisc failed (rc = %d)\n", rc);
    } else {
        pr_info("module loaded\n");
    }
    return rc;
}

static void __exit buspirate_module_cleanup(void)
{
    int rc;

    if ((rc = tty_unregister_ldisc(N_GIGASET_M101))) {
        pr_err("can't unregister line discipline (rc = %d)\n", rc);
    } else {
        pr_info("module unloaded\n");
    }
    kobject_del(&module_kobj);
    kobject_put(&module_kobj);
}

module_init(buspirate_module_init);
module_exit(buspirate_module_cleanup);
