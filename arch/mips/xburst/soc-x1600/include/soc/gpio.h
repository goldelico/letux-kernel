#ifndef _SOC_GPIO_H_
#define _SOC_GPIO_H_

enum gpio_function {
    GPIO_FUNC_0         = 0x0100,  //00000, GPIO as function 0 / device 0
    GPIO_FUNC_1         = 0x0101,  //00001, GPIO as function 1 / device 1
    GPIO_FUNC_2         = 0x0102,  //00010, GPIO as function 2 / device 2
    GPIO_FUNC_3         = 0x0103,  //00011, GPIO as function 3 / device 3
    GPIO_OUTPUT0        = 0x0104,  //00100, GPIO output low  level
    GPIO_OUTPUT1        = 0x0105,  //00101, GPIO output high level
    GPIO_INPUT          = 0x0106,  //00110, GPIO as input.7 also.
    GPIO_INT_LO         = 0x0108,  //01000, Low  Level trigger interrupt
    GPIO_INT_HI         = 0x0109,  //01001, High Level trigger interrupt
    GPIO_INT_FE         = 0x010a,  //01010, Fall Edge trigger interrupt
    GPIO_INT_RE         = 0x010b,  //01011, Rise Edge trigger interrupt
    GPIO_INT_RE_FE      = 0x011b,  //11011, Dual Edge(both rise and fall) trigger interrupt
    GPIO_INT_MASK_LO    = 0x010c,  //01100, Port is low level triggered interrupt input. Interrupt is masked.
    GPIO_INT_MASK_HI    = 0x010d,  //01101, Port is high level triggered interrupt input. Interrupt is masked.
    GPIO_INT_MASK_FE    = 0x010e,  //01110, Port is fall edge triggered interrupt input. Interrupt is masked.
    GPIO_INT_MASK_RE    = 0x010f,  //01111, Port is rise edge triggered interrupt input. Interrupt is masked.
    GPIO_INT_MASK_RE_FE = 0x011f,  //11111, Port is dual edge(both rise and fall edge) triggered interrupt input. Interrupt is masked.

    GPIO_PULL_HIZ       = 0x8000,    //no pull
    GPIO_PULL           = 0xa000,    //pull, x1600只支持pull,是什么类型的pull 要看芯片手册
};

enum gpio_port {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,

    /* this must be last */
    GPIO_NR_PORTS,
};

#define GPIO_PA(n)      (0 * 32 + (n))
#define GPIO_PB(n)      (1 * 32 + (n))
#define GPIO_PC(n)      (2 * 32 + (n))
#define GPIO_PD(n)      (3 * 32 + (n))

int jzgpio_set_func(int port, enum gpio_function func, unsigned long pins);

#endif /* _SOC_GPIO_H_ */
