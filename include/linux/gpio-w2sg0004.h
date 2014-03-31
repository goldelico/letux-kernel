
/* Virtual gpio to allow ON/OFF control of w2sg0004 GPS receiver. */

struct gpio_w2sg_data {
	int	lna_gpio;		/* enable LNA power */
	int	on_off_gpio;	/* connected to the on-off input of the GPS module */
	int	rx_gpio;		/* the rx data we track to check for module activity */
	unsigned short	on_state;  /* Mux state when GPS is on */
	unsigned short	off_state; /* Mux state when GPS is off */
};
