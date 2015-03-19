
struct tty_slave {
	struct device *tty_dev;
	struct tty_driver *tty_drv;
	struct tty_operations ops;
	struct device dev;
};

int tty_slave_finalize(struct tty_slave *slave);
int tty_slave_driver_register(struct device_driver *drv);
#if config_enabled(CONFIG_TTY_SLAVE)
void tty_slave_activate(struct tty_struct *tty);
int tty_slave_register(struct device *parent, struct device_node *node,
		       struct device *tty, struct tty_driver *drv);
#else
static inline void tty_slave_activate(struct tty_struct *tty)
{
}
static inline int tty_slave_register(struct device *parent,
				     struct device_node *node,
				     struct device *tty,
				     struct tty_driver *drv)
{
	return -ENODEV;
}
#endif
