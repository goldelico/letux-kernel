/*
 * SW.LEE <hitchcar@samsung.com>
 *   
 * This file is subject to the terms and conditions of the GNU General Public
 * License 2. See the file COPYING in the main directory of this archive
 * for more details.
 */

#ifndef __Z_API_H_
#define __Z_API_H_

extern ssize_t z_read(struct file *f, char *buf, size_t count, loff_t *pos);
extern ssize_t z_write(struct file *f, const char *b, size_t c, loff_t *pos);



#endif

