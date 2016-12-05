#if defined(CONFIG_IIO_INPUT_BRIDGE)

extern int iio_device_register_inputbridge(struct iio_dev *indio_dev);
extern void iio_device_unregister_inputbridge(struct iio_dev *indio_dev);

#else

static inline int iio_device_register_inputbridge(struct iio_dev *indio_dev)
{
	return 0;
}

static inline void iio_device_unregister_inputbridge(struct iio_dev *indio_dev)
{
}

#endif
