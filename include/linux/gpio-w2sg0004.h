
/* Virtual gpio to allow ON/OFF control of w2sg0004 GPS receiver. */

struct gpio_w2sg_data {
	int	ctrl_gpio;
	int	on_off_gpio;
	int	rx_gpio;
	short	on_state;  /* Mux state when GPS is on */
	short	off_state; /* Mux state when GPS is off */
};
