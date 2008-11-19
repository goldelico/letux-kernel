#include <linux/resume-dependency.h>

extern void s3c24xx_serial_console_set_silence(int silence);
extern void s3c24xx_serial_register_resume_dependency(struct resume_dependency *
					     resume_dependency, int uart_index);
