#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/io.h>
#include <soc/base.h>
#include <soc/gpio.h>

#define GPIO_PORT_OFF    0x100
#define GPIO_SHADOW_OFF  0x700

#define PXPIN       0x00   /* PIN Level Register */
#define PXINT       0x10   /* Port Interrupt Register */
#define PXINTS      0x14   /* Port Interrupt Set Register */
#define PXINTC      0x18   /* Port Interrupt Clear Register */
#define PXMSK       0x20   /* Port Interrupt Mask Reg */
#define PXMSKS      0x24   /* Port Interrupt Mask Set Reg */
#define PXMSKC      0x28   /* Port Interrupt Mask Clear Reg */
#define PXPAT1      0x30   /* Port Pattern 1 Register. */
#define PXPAT1S     0x34   /* Port Pattern 1 Set Register. */
#define PXPAT1C     0x38   /* Port Pattern 1 Clear Register. */
#define PXPAT0      0x40   /* Port Pattern 0 Register */
#define PXPAT0S     0x44   /* Port Pattern 0 Set Register */
#define PXPAT0C     0x48   /* Port Pattern 0 Clear Register */
#define PXFLG       0x50   /* Port Flag Register */
#define PXFLGC      0x58   /* Port Flag clear Register */
#define PXDEG       0x70   /* Port Dual Edge Register */
#define PXDEGS      0x74   /* Port Dual Edge Set Register */
#define PXDEGC      0x78   /* Port Dual Edge Clear Register */
#define PXPEN       0x80   /* Port Pull Enable Register */
#define PXPENS      0x84   /* Port Pull Enable Set Register */
#define PXPENC      0x88   /* Port Pull Enable Clear Register */

#define PZGID2LD    0xF0   /* GPIOZ Group ID to load */

#define SHADOW 4

#define GPIO_FUNC_FLAG      0x0100
#define GPIO_FUNC_OFFSET    0
#define GPIO_FUNC_MASK      0x01ff

#define GPIO_PULL_FLAG      0x8000
#define GPIO_PULL_OFFSET    13
#define GPIO_PULL_MASK      0xe000

static const unsigned long gpiobase[] = {
    [0] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 0 * GPIO_PORT_OFF),
    [1] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 1 * GPIO_PORT_OFF),
    [2] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 2 * GPIO_PORT_OFF),
    [3] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + 3 * GPIO_PORT_OFF),

    [SHADOW] = (unsigned long)CKSEG1ADDR(GPIO_IOBASE + GPIO_SHADOW_OFF),
};

#define GPIO_ADDR(port, reg)            ((volatile unsigned long *)(gpiobase[port] + reg))

static inline void gpio_write(int port, unsigned int reg, int val)
{
    *GPIO_ADDR(port, reg) = val;
}

static inline unsigned int gpio_read(int port, unsigned int reg)
{
    return *GPIO_ADDR(port, reg);
}

static void hal_gpio_port_set_func(enum gpio_port port, unsigned int pins, enum gpio_function func)
{
    /* func option */
    if (func & GPIO_FUNC_FLAG) {
        /* No Shadows registers for EDG, set registers directly */
        if (func & 0x10)
            gpio_write(port, PXDEGS, pins);
        else
            gpio_write(port, PXDEGC, pins);


        if (func & 0x8)
            gpio_write(SHADOW, PXINTS, pins);
        else
            gpio_write(SHADOW, PXINTC, pins);

        if (func & 0x4)
            gpio_write(SHADOW, PXMSKS, pins);
        else
            gpio_write(SHADOW, PXMSKC, pins);

        if (func & 0x2)
            gpio_write(SHADOW, PXPAT1S, pins);
        else
            gpio_write(SHADOW, PXPAT1C, pins);

        if (func & 0x1)
            gpio_write(SHADOW, PXPAT0S, pins);
        else
            gpio_write(SHADOW, PXPAT0C, pins);

        /* configure PzGID2LD to specify which port group to load */
        gpio_write(SHADOW, PZGID2LD, port);
    }

    /* pull option */
    if (func & GPIO_PULL_FLAG) {
        if (func & 0x2000)
            gpio_write(port, PXPENS, pins);
        else
            gpio_write(port, PXPENC, pins);
    }

}

unsigned long ingenic_pinctrl_lock(int port);
void ingenic_pinctrl_unlock(int port, unsigned long flags);

int jzgpio_set_func(int port, enum gpio_function func, unsigned long pins)
{
    unsigned long flags;

    if (port < 0 || port > 3) {
        printk(KERN_ERR "gpio: invalid gpio port for x1600: %d\n", port);
        return -EINVAL;
    }

    flags = ingenic_pinctrl_lock(port);

    hal_gpio_port_set_func(port, pins, func);

    ingenic_pinctrl_unlock(port, flags);

    return 0;
}
EXPORT_SYMBOL(jzgpio_set_func);
