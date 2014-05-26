
/* Virtual gpio to allow ON/OFF control of a WWAN module. */

struct gpio_w2sg_data {
	int	ctrl_gpio;		/* (not used by DT) - defines the gpio.base */
	int	on_off_gpio;	/* connected to the on-off input of the GPS module */
	int	feedback_gpio;	/* a status feedback to report module power state */
	/* to be added: a link to some USB PHY */
	/* to be removed */
	int	lna_gpio;		/* enable LNA power */
	unsigned short	on_state;  /* Mux state when GPS is on */
	unsigned short	off_state; /* Mux state when GPS is off */
};
