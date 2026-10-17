#ifndef _LINUX_RWSEM_H
#define _LINUX_RWSEM_H

// surrogate definitions to avoid name clashes

#include <stdbool.h>
#include <linux/list.h>

#undef __user
#define __user

#undef IS_MODULE
#define IS_MODULE(x) (0)

#endif

