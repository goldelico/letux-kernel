#ifndef __RESUME_DEPENDENCY_H__
#define __RESUME_DEPENDENCY_H__

/* Resume dependency framework
 *
 * (C) 2008 Openmoko, Inc.
 * Author: Andy Green <andy@openmoko.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; version 2.1.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <linux/list.h>

struct resume_dependency {
	struct list_head list;

	void (*callback)(void *); /* called with context as arg */
	void * context;
	int called_flag; /* set to 1 after called, use for multi dep */
};

/* if you are a driver accept to have other drivers as dependencies, you need to
 * instantiate a struct resume_dependency above, then initialize it by invoking
 * init_resume_dependency_list() on it
 */

#define init_resume_dependency_list(_head) \
	printk(KERN_DEBUG "##### init_resume_dependency_list(head=%p)\n", (_head)); \
	INIT_LIST_HEAD(&(_head)->list);


/* if your resume function depends on something else being resumed first, you
 * can register the dependency by calling this in your suspend function with
 * head being the list held by the thing you are dependent on, and dep being
 * your struct resume_dependency
 */

#define register_resume_dependency(_head, _dep) { \
	struct list_head *_pos, *_q; \
	struct resume_dependency *_d; \
\
	printk(KERN_DEBUG "##### register_resume_dependency(head=%p, dep=%p)\n", (_head), (_dep)); \
	(_dep)->called_flag = 1; \
	list_for_each_safe(_pos, _q, &((_head)->list)) { \
		_d = list_entry(_pos, struct resume_dependency, list); \
		if (_d == (_dep)) { \
			list_del(_pos); \
			printk(KERN_DEBUG "#####   duplicate dependency removed first\n"); \
		} \
	} \
	list_add(&(_dep)->list, &(_head)->list); \
}

/* In the resume function that things can be dependent on, at the end you
 * invoke this macro.  This calls back the dependent resumes now it is safe to
 * use the resumed thing they were dependent on.
 */

#define callback_all_resume_dependencies(_head) { \
	struct list_head *_pos, *_q; \
	struct resume_dependency *_dep; \
\
	printk(KERN_DEBUG "##### callback_all_resume_dependencies(head=%p)\n", (_head)); \
	list_for_each_safe(_pos, _q, &((_head)->list)) { \
		_dep = list_entry(_pos, struct resume_dependency, list); \
		printk(KERN_DEBUG "#####   callback list entry (head=%p, dep=%p)\n", (_head), (_dep)); \
		_dep->called_flag = 1; \
		printk(KERN_DEBUG "#####      callback=%p(context=%p))\n", (_dep->callback),(_dep->context)); \
		(_dep->callback)(_dep->context); \
		list_del(_pos); \
	} \
}

/* When a dependency is added, it is not actually active; the dependent resume
 * handler will function as normal.  The dependency is activated by the suspend
 * handler for the driver that will be doing the callbacks.  This ensures that
 * if the suspend is aborted for any reason (error, driver busy, etc), that all
 * suspended drivers will resume, even if the driver upon which they are dependent
 * did not suspend, and hence will not resume, and thus would be unable to perform
 * the callbacks.
 */

#define activate_all_resume_dependencies(_head) { \
	struct list_head *_pos, *_q; \
	struct resume_dependency *_dep; \
\
	printk(KERN_DEBUG "##### activate_all_resume_dependencies(head=%p)\n", (_head)); \
	list_for_each_safe(_pos, _q, &((_head)->list)) { \
		_dep = list_entry(_pos, struct resume_dependency, list); \
		printk(KERN_DEBUG "#####   activating callback list entry (head=%p, dep=%p)\n", (_head), (_dep)); \
		_dep->called_flag = 0; \
	} \
}

/* if your resume action is dependent on multiple drivers being resumed already,
 * register the same callback with each driver you are dependent on, and check
 * .called_flag for all of the struct resume_dependency.  When they are all 1
 * you know it is the last callback and you can resume, otherwise just return
 */

#endif
