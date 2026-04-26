/*
 * TELE 7374 - Class Project
 * Kernel module for LED PWM control and button speed detection
 *
 * GPIO pins (BCM):
 *   P1 -> GPIO 17, P2 -> GPIO 27
 *   L1 -> GPIO 22, L2 -> GPIO 23
 *
 * Device file: /dev/project
 *   read:  returns "speed=<N>\n"  (presses per 10s)
 *   write: "L1=<0-100> L2=<0-100>"  (duty cycle %)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/gpio.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Heyang");
MODULE_DESCRIPTION("TELE7374 project - PWM LEDs + button speed");

/* BCM2711 GPIO base address (RPi 4) */
#define GPIO_BASE      0xFE200000UL
#define GPIO_MAP_SIZE  0xF4

/* Register offsets */
#define GPFSEL0   0x00
#define GPSET0    0x1C
#define GPCLR0    0x28
#define GPLEV0    0x34
#define GPEDS0    0x40
#define GPFEN0    0x58
#define GPPUPPDN0 0xE4
#define GPPUPPDN1 0xE8

#define FSEL_INPUT  0
#define FSEL_OUTPUT 1
#define PULL_UP     1

/* GPIO pin numbers */
#define PIN_P1  17
#define PIN_P2  27
#define PIN_L1  22
#define PIN_L2  23

static void __iomem *gpio_base;

/* set pin function: 3 bits per pin, 10 pins per GPFSEL register */
static void pin_set_func(unsigned int pin, unsigned int fn)
{
    unsigned int off = GPFSEL0 + (pin / 10) * 4;
    unsigned int shift = (pin % 10) * 3;
    u32 val = readl(gpio_base + off);
    val &= ~(0x7u << shift);
    val |=  (fn   << shift);
    writel(val, gpio_base + off);
}

/* set pull resistor: 2 bits per pin, 16 pins per GPPUPPDN register */
static void pin_set_pull(unsigned int pin, unsigned int pull)
{
    unsigned int off = GPPUPPDN0 + (pin / 16) * 4;
    unsigned int shift = (pin % 16) * 2;
    u32 val = readl(gpio_base + off);
    val &= ~(0x3u << shift);
    val |=  (pull << shift);
    writel(val, gpio_base + off);
}

static void pin_high(unsigned int pin)
{
    writel(1u << pin, gpio_base + GPSET0);
}

static void pin_low(unsigned int pin)
{
    writel(1u << pin, gpio_base + GPCLR0);
}

static void pin_enable_falling(unsigned int pin)
{
    u32 val = readl(gpio_base + GPFEN0);
    val |= (1u << pin);
    writel(val, gpio_base + GPFEN0);
}

static void pin_clear_event(unsigned int pin)
{
    writel(1u << pin, gpio_base + GPEDS0);
}

/* PWM: 100us tick, 100 ticks per period = 10ms period */
#define PWM_TICK_NS  (100 * 1000)
#define PWM_TICKS    100

static struct hrtimer pwm_timer;
static int pwm_tick = 0;
static int duty_l1  = 10;
static int duty_l2  = 0;
static DEFINE_SPINLOCK(pwm_lock);

static enum hrtimer_restart pwm_cb(struct hrtimer *timer)
{
    unsigned long flags;
    int d1, d2;

    spin_lock_irqsave(&pwm_lock, flags);
    d1 = duty_l1;
    d2 = duty_l2;
    spin_unlock_irqrestore(&pwm_lock, flags);

    if (pwm_tick < d1) pin_high(PIN_L1); else pin_low(PIN_L1);
    if (pwm_tick < d2) pin_high(PIN_L2); else pin_low(PIN_L2);

    pwm_tick = (pwm_tick + 1) % PWM_TICKS;
    hrtimer_forward_now(timer, ns_to_ktime(PWM_TICK_NS));
    return HRTIMER_RESTART;
}

/* Speed measurement: store timestamps of alternating P1/P2 presses */
#define MAX_TIMES  20

static ktime_t press_times[MAX_TIMES];
static int press_count = 0;
static DEFINE_SPINLOCK(speed_lock);
static atomic_t next_btn = ATOMIC_INIT(0); /* 0=P1, 1=P2 */
static int irq_p1 = -1, irq_p2 = -1;

static void record_press(void)
{
    unsigned long flags;
    spin_lock_irqsave(&speed_lock, flags);
    press_times[press_count % MAX_TIMES] = ktime_get();
    press_count++;
    spin_unlock_irqrestore(&speed_lock, flags);
}

static irqreturn_t isr_p1(int irq, void *dev_id)
{
    pin_clear_event(PIN_P1);
    if (atomic_read(&next_btn) == 0) {
        atomic_set(&next_btn, 1);
        record_press();
    }
    return IRQ_HANDLED;
}

static irqreturn_t isr_p2(int irq, void *dev_id)
{
    pin_clear_event(PIN_P2);
    if (atomic_read(&next_btn) == 1) {
        atomic_set(&next_btn, 0);
        record_press();
    }
    return IRQ_HANDLED;
}

/*
 * Compute speed in presses per 10s.
 * Uses the interval between the last two presses, but also factors in
 * how long ago the last press was. So if the user pauses, the speed
 * starts dropping immediately on each poll, not after a hard timeout.
 */
static int get_speed(void)
{
    unsigned long flags;
    ktime_t t_last, t_prev, now;
    int count;
    s64 interval, since_last, effective;
    u64 speed;

    spin_lock_irqsave(&speed_lock, flags);
    count = press_count;
    if (count < 2) {
        spin_unlock_irqrestore(&speed_lock, flags);
        return 0;
    }
    t_last = press_times[(count - 1) % MAX_TIMES];
    t_prev = press_times[(count - 2) % MAX_TIMES];
    spin_unlock_irqrestore(&speed_lock, flags);

    now        = ktime_get();
    since_last = ktime_to_ns(ktime_sub(now, t_last));
    interval   = ktime_to_ns(ktime_sub(t_last, t_prev));

    if (interval <= 0)
        return 0;

    /* use the larger of the two so speed drops naturally when pausing */
    effective = (since_last > interval) ? since_last : interval;

    /* cap at 10s - anything longer is considered fully stopped */
    if (effective >= 10000000000LL)
        return 0;

    speed = 10000000000ULL;
    do_div(speed, (u64)effective);
    return (int)speed;
}

/* Character device */
#define DEV_NAME   "project"
#define CLASS_NAME "project_class"

static int major;
static struct class  *dev_class;
static struct device *dev_device;
static struct cdev    dev_cdev;

static ssize_t dev_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
    char tmp[32];
    int n;

    if (*off > 0)
        return 0;

    memset(tmp, 0, sizeof(tmp));
    n = snprintf(tmp, sizeof(tmp), "speed=%d\n", get_speed());
    if (n <= 0 || (size_t)n > len)
        return -EINVAL;
    if (copy_to_user(buf, tmp, n))
        return -EFAULT;

    *off += n;
    return n;
}

static ssize_t dev_write(struct file *f, const char __user *buf, size_t len, loff_t *off)
{
    char tmp[64];
    int d1 = -1, d2 = -1;
    unsigned long flags;

    if (len >= sizeof(tmp))
        return -EINVAL;
    if (copy_from_user(tmp, buf, len))
        return -EFAULT;
    tmp[len] = '\0';

    if (sscanf(tmp, "L1=%d L2=%d", &d1, &d2) != 2)
        return -EINVAL;
    if (d1 < 0 || d1 > 100 || d2 < 0 || d2 > 100)
        return -EINVAL;

    spin_lock_irqsave(&pwm_lock, flags);
    duty_l1 = d1;
    duty_l2 = d2;
    spin_unlock_irqrestore(&pwm_lock, flags);

    return len;
}

static int dev_open(struct inode *i, struct file *f)    { return 0; }
static int dev_release(struct inode *i, struct file *f) { return 0; }

static const struct file_operations fops = {
    .owner   = THIS_MODULE,
    .open    = dev_open,
    .release = dev_release,
    .read    = dev_read,
    .write   = dev_write,
};

static int __init project_init(void)
{
    int ret;
    dev_t dev;

    pr_info("project: loading\n");

    gpio_base = ioremap(GPIO_BASE, GPIO_MAP_SIZE);
    if (!gpio_base) {
        pr_err("project: ioremap failed\n");
        return -ENOMEM;
    }

    /* setup LEDs as output */
    pin_set_func(PIN_L1, FSEL_OUTPUT); pin_low(PIN_L1);
    pin_set_func(PIN_L2, FSEL_OUTPUT); pin_low(PIN_L2);

    /* setup buttons as input with pull-up */
    pin_set_func(PIN_P1, FSEL_INPUT); pin_set_pull(PIN_P1, PULL_UP);
    pin_set_func(PIN_P2, FSEL_INPUT); pin_set_pull(PIN_P2, PULL_UP);

    /* enable falling edge detection */
    pin_enable_falling(PIN_P1);
    pin_enable_falling(PIN_P2);

    irq_p1 = gpio_to_irq(PIN_P1);
    irq_p2 = gpio_to_irq(PIN_P2);

    ret = request_irq(irq_p1, isr_p1, IRQF_TRIGGER_FALLING | IRQF_SHARED,
                      "project_p1", (void *)isr_p1);
    if (ret) { pr_err("project: irq p1 failed\n"); goto err_irq1; }

    ret = request_irq(irq_p2, isr_p2, IRQF_TRIGGER_FALLING | IRQF_SHARED,
                      "project_p2", (void *)isr_p2);
    if (ret) { pr_err("project: irq p2 failed\n"); goto err_irq2; }

    ret = alloc_chrdev_region(&dev, 0, 1, DEV_NAME);
    if (ret < 0) { pr_err("project: chrdev alloc failed\n"); goto err_chrdev; }
    major = MAJOR(dev);

    cdev_init(&dev_cdev, &fops);
    dev_cdev.owner = THIS_MODULE;
    ret = cdev_add(&dev_cdev, dev, 1);
    if (ret < 0) { pr_err("project: cdev_add failed\n"); goto err_cdev; }

    dev_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(dev_class)) { ret = PTR_ERR(dev_class); goto err_class; }

    dev_device = device_create(dev_class, NULL, dev, NULL, DEV_NAME);
    if (IS_ERR(dev_device)) { ret = PTR_ERR(dev_device); goto err_device; }

    hrtimer_init(&pwm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    pwm_timer.function = pwm_cb;
    hrtimer_start(&pwm_timer, ns_to_ktime(PWM_TICK_NS), HRTIMER_MODE_REL);

    pr_info("project: ready, /dev/%s created\n", DEV_NAME);
    return 0;

err_device:
    class_destroy(dev_class);
err_class:
    cdev_del(&dev_cdev);
err_cdev:
    unregister_chrdev_region(MKDEV(major, 0), 1);
err_chrdev:
    free_irq(irq_p2, (void *)isr_p2);
err_irq2:
    free_irq(irq_p1, (void *)isr_p1);
err_irq1:
    iounmap(gpio_base);
    return ret;
}

static void __exit project_exit(void)
{
    hrtimer_cancel(&pwm_timer);
    pin_low(PIN_L1);
    pin_low(PIN_L2);
    free_irq(irq_p2, (void *)isr_p2);
    free_irq(irq_p1, (void *)isr_p1);
    device_destroy(dev_class, MKDEV(major, 0));
    class_destroy(dev_class);
    cdev_del(&dev_cdev);
    unregister_chrdev_region(MKDEV(major, 0), 1);
    iounmap(gpio_base);
    pr_info("project: unloaded\n");
}

module_init(project_init);
module_exit(project_exit);
