/*
 *  nameserver.c
 *
 *  The nameserver module manages local name/value pairs that
 *  enables an application and other modules to store and retrieve
 *  values based on a name.
 *
 *  Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 *  This package is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 *  WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE.
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <syslink/atomic_linux.h>

#include <nameserver.h>
#include <multiproc.h>
#include <nameserver_remote.h>

#define NS_MAX_NAME_LEN  			32
#define NS_MAX_RUNTIME_ENTRY 		(~0)
#define NS_MAX_VALUE_LEN			4

/*
 *  The dynamic name/value table looks like the following. This approach allows
 *  each instance table to have different value and different name lengths.
 *  The names block is allocated on the create. The size of that block is
 *  (max_runtime_entries * max_name_en). That block is sliced and diced up and
 *  given to each table entry.
 *  The same thing is done for the values block.
 *
 *     names                    table                  values
 *   -------------           -------------          -------------
 *   |           |<-\        |   elem    |   /----->|           |
 *   |           |   \-------|   name    |  /       |           |
 *   |           |           |   value   |-/        |           |
 *   |           |           |   len     |          |           |
 *   |           |<-\        |-----------|          |           |
 *   |           |   \       |   elem    |          |           |
 *   |           |    \------|   name    |  /------>|           |
 *   |           |           |   value   |-/        |           |
 *   -------------           |   len     |          |           |
 *                           -------------          |           |
 *                                                  |           |
 *                                                  |           |
 *                                                  -------------
 *
 *  There is an optimization for small values (e.g. <= sizeof(UInt32).
 *  In this case, there is no values block allocated. Instead the value
  *  field is used directly.  This optimization occurs and is managed when
 *  obj->max_value_len <= sizeof(Us3232).
 *
 *  The static create is a little different. The static entries point directly
 *  to a name string (and value). Since it points directly to static items,
 *  this entries cannot be removed.
 *  If max_runtime_entries is non-zero, a names and values block is created.
 *  Here is an example of a table with 1 static entry and 2 dynamic entries
 *
 *                           ------------
 *  this entries cannot be removed.
 *  If max_runtime_entries is non-zero, a names and values block is created.
 *  Here is an example of a table with 1 static entry and 2 dynamic entries
 *
 *                           ------------
 *                           |   elem   |
 *      "myName" <-----------|   name   |----------> someValue
 *                           |   value  |
 *     names                 |   len    |              values
 *   -------------           -------------          -------------
 *   |           |<-\        |   elem    |   /----->|           |
 *   |           |   \-------|   name    |  /       |           |
 *   |           |           |   value   |-/        |           |
 *   |           |           |   len     |          |           |
 *   |           |<-\        |-----------|          |           |
 *   |           |   \       |   elem    |          |           |
 *   |           |    \------|   name    |  /------>|           |
 *   |           |           |   value   |-/        |           |
 *   -------------           |   len     |          |           |
 *                           -------------          |           |
 *                                                  |           |
 *                                                  |           |
 *                                                  -------------
 *
 *  NameServerD uses a freeList and namelist to maintain the empty
 *  and filled-in entries. So when a name/value pair is added, an entry
 *  is pulled off the freeList, filled-in and placed on the namelist.
 *  The reverse happens on a remove.
 *
 *  For static adds, the entries are placed on the namelist statically.
 *
 *  For dynamic creates, the freeList is populated in postInt and there are no
 *  entries placed on the namelist (this happens when the add is called).
 *
 */

/* Macro to make a correct module magic number with refCount */
#define NAMESERVER_MAKE_MAGICSTAMP(x) ((NAMESERVER_MODULEID << 12u) | (x))

/*
 *  A name/value table entry
 */
struct nameserver_entry {
	struct list_head elem; /* List element */
	u32 hash; /* Hash value */
	char *name; /* Name portion of name/value pair */
	u32 len; /* Length of the value field. */
	void *buf; /* Value portion of name/value entry */
	bool collide; /* Does the hash collides? */
	struct nameserver_entry *next; /* Pointer to the next entry,
					used incase of collision only */
};

/*
 * A nameserver instance object
 */
struct nameserver_object {
	struct list_head elem;
	char *name; /* Name of the instance */
	u32 count; /* Counter for entries */
	struct mutex *gate_handle; /* Gate for critical regions */
	struct list_head name_list; /* Filled entries list */
	struct nameserver_params params; /* The parameter structure */
};


/* nameserver module state object */
struct nameserver_module_object {
	struct list_head obj_list;
	struct mutex *list_lock;
	struct nameserver_remote_object **remote_handle_list;
	atomic_t ref_count;
};

/*
 * Variable for holding state of the nameserver module.
 */
struct nameserver_module_object nameserver_state = {
	.obj_list     = LIST_HEAD_INIT(nameserver_state.obj_list),
	.list_lock    = NULL,
	.remote_handle_list = NULL,
};

/*
 * Lookup table for CRC calculation.
 */
static const u32 string_crc_table[256u] = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
  0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
  0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
  0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
  0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
  0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
  0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
  0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
  0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
  0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
  0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
  0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
  0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
  0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
  0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
  0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
  0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
  0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
  0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
  0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
  0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
  0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
  0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
  0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
  0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
  0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
  0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
  0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
  0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
};

/*
 * ======== nameserver_setup ========
 *  Purpose:
 *  This will calculate hash for a string
 */
static u32 nameserver_string_hash(const char *string)
{
	u32 i;
	u32 hash ;
	u32 len = strlen(string);

	for (i = 0, hash = len; i < len; i++)
		hash = (hash >> 8) ^
			string_crc_table[(hash & 0xff)] ^ string[i];

	return hash;
}

/*
 * ======== nameserver_setup ========
 *  Purpose:
 *  This will setup the nameserver module
 */
int nameserver_setup(void)
{
	struct nameserver_remote_object **list = NULL;
	s32 retval = 0;
	u16 nr_procs = 0;

	/* This sets the ref_count variable if not initialized, upper 16 bits is
	*   written with module Id to ensure correctness of refCount variable
	*/
	atomic_cmpmask_and_set(&nameserver_state.ref_count,
				NAMESERVER_MAKE_MAGICSTAMP(0),
				NAMESERVER_MAKE_MAGICSTAMP(0));

	if (atomic_inc_return(&nameserver_state.ref_count)
				!= NAMESERVER_MAKE_MAGICSTAMP(1)) {
		return 1;
	}

	nr_procs = multiproc_get_max_processors();
	list = kmalloc(nr_procs * sizeof(struct nameserver_remote_object *),
					GFP_KERNEL);
	if (list == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	memset(list , 0, nr_procs * sizeof(struct nameserver_remote_object *));
	nameserver_state.remote_handle_list = list;
	nameserver_state.list_lock = kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (nameserver_state.list_lock == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	/* mutex is initialized with state = UNLOCKED */
	mutex_init(nameserver_state.list_lock);
	return 0;

error:
	kfree(list);
	printk(KERN_ERR "nameserver_setup failed, retval: %x\n", retval);
	return retval;
}
EXPORT_SYMBOL(nameserver_setup);

/*
 * ======== nameserver_destroy ========
 *  Purpose:
 *  This will destroy the nameserver module
 */
int nameserver_destroy(void)
{
	s32 retval = 0;
	struct mutex *lock = NULL;

	if (WARN_ON(atomic_cmpmask_and_lt(&(nameserver_state.ref_count),
				NAMESERVER_MAKE_MAGICSTAMP(0),
				NAMESERVER_MAKE_MAGICSTAMP(1)) == true)) {
		retval = -ENODEV;
		goto exit;
	}

	if (!(atomic_dec_return(&nameserver_state.ref_count)
					== NAMESERVER_MAKE_MAGICSTAMP(0))) {
		retval = 1;
		goto exit;
	}

	if (WARN_ON(nameserver_state.list_lock == NULL)) {
		retval = -ENODEV;
		goto exit;
	}

	/* If a nameserver instance exist, do not proceed  */
	if (!list_empty(&nameserver_state.obj_list)) {
		retval = -EBUSY;
		goto exit;
	}

	retval = mutex_lock_interruptible(nameserver_state.list_lock);
	if (retval)
		goto exit;

	lock = nameserver_state.list_lock;
	nameserver_state.list_lock = NULL;
	mutex_unlock(lock);
	kfree(lock);
	kfree(nameserver_state.remote_handle_list);
	nameserver_state.remote_handle_list = NULL;
	return 0;

exit:
	if (retval < 0) {
		printk(KERN_ERR "nameserver_destroy failed, retval: %x\n",
			retval);
	}
	return retval;
}
EXPORT_SYMBOL(nameserver_destroy);

/*!
 *  Purpose:
 *  Initialize this config-params structure with supplier-specified
 *  defaults before instance creation.
 */
int nameserver_params_init(struct nameserver_params *params)
{
	BUG_ON(params == NULL);
	params->check_existing = true;
	params->gate_handle = NULL;
	params->max_name_len = NS_MAX_NAME_LEN;
	params->max_runtime_entries = NS_MAX_RUNTIME_ENTRY;
	params->max_value_len = NS_MAX_VALUE_LEN;
	params->table_heap = NULL;
	return 0;
}
EXPORT_SYMBOL(nameserver_params_init);

/*
 * ======== nameserver_get_params ========
 *  Purpose:
 *  This will initialize config-params structure with
 *  supplier-specified defaults before instance creation
 */
int nameserver_get_params(void *handle,
		struct nameserver_params *params)
{
	struct nameserver_object *nshandle = NULL;

	BUG_ON(params == NULL);
	if (handle == NULL) {
		params->check_existing       = true;
		params->max_name_len         = NS_MAX_NAME_LEN;
		params->max_runtime_entries  = NS_MAX_RUNTIME_ENTRY;
		params->max_value_len        = NS_MAX_VALUE_LEN;
		params->gate_handle          = NULL;
		params->table_heap    	     = NULL;
	} else {
		nshandle = (struct nameserver_object *)handle;
		params->check_existing 	 = nshandle->params.check_existing;
		params->max_name_len 	 = nshandle->params.max_name_len;
		params->max_runtime_entries  =
					nshandle->params.max_runtime_entries;
		params->max_value_len    = nshandle->params.max_value_len;
		params->gate_handle      = nshandle->params.gate_handle;
		params->table_heap      = nshandle->params.table_heap;
	}
	return 0;
}
EXPORT_SYMBOL(nameserver_get_params);

/*
 * ======== nameserver_get_params ========
 *  Purpose:
 *  This will get the handle of a nameserver instance
 *  from name
 */
void *nameserver_get_handle(const char *name)
{
	struct nameserver_object *obj = NULL;

	BUG_ON(name == NULL);
	list_for_each_entry(obj, &nameserver_state.obj_list, elem) {
		if (strcmp(obj->name, name) == 0)
			goto succes;
	}
	return NULL;

succes:
	return (void *)obj;
}
EXPORT_SYMBOL(nameserver_get_handle);

/*
 * ======== nameserver_create ========
 *  Purpose:
 *  This will create a name server instance
 */
void *nameserver_create(const char *name,
			const struct nameserver_params *params)
{
	struct nameserver_object *new_obj = NULL;
	u32 name_len;
	s32 retval = 0;

	BUG_ON(name == NULL);
	BUG_ON(params == NULL);

	name_len = strlen(name) + 1;
	if (name_len > params->max_name_len) {
		retval = -E2BIG;
		goto exit;
	}

	retval = mutex_lock_interruptible(nameserver_state.list_lock);
	if (retval)
		goto exit;

	/* check if the name is already registered or not */
	new_obj = nameserver_get_handle(name);
	if (new_obj != NULL) {
		retval = -EEXIST;
		goto error_handle;
	}

	new_obj = kmalloc(sizeof(struct nameserver_object), GFP_KERNEL);
	if (new_obj == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	new_obj->name = kmalloc(name_len, GFP_ATOMIC);
	if (new_obj->name == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	strncpy(new_obj->name, name, name_len);
	memcpy(&new_obj->params, params,
				sizeof(struct nameserver_params));
	if (params->max_value_len < sizeof(u32))
		new_obj->params.max_value_len = sizeof(u32);
	else
		new_obj->params.max_value_len = params->max_value_len;

	new_obj->gate_handle =
				kmalloc(sizeof(struct mutex), GFP_KERNEL);
	if (new_obj->gate_handle == NULL) {
			retval = -ENOMEM;
			goto error_mutex;
	}

	mutex_init(new_obj->gate_handle);
	new_obj->count = 0;
	/* Put in the nameserver instance to local list */
	INIT_LIST_HEAD(&new_obj->name_list);
	list_add_tail(&new_obj->elem, &nameserver_state.obj_list);
	mutex_unlock(nameserver_state.list_lock);
	return (void *)new_obj;

error_mutex:
	kfree(new_obj->name);
error:
	kfree(new_obj);
error_handle:
	mutex_unlock(nameserver_state.list_lock);
exit:
	printk(KERN_ERR "nameserver_create failed retval:%x \n", retval);
	return NULL;
}
EXPORT_SYMBOL(nameserver_create);


/*
 * ======== nameserver_delete ========
 *  Purpose:
 *  This will delete a name server instance
 */
int nameserver_delete(void **handle)
{
	struct nameserver_object *temp_obj = NULL;
	struct mutex *gate_handle = NULL;
	bool localgate = false;
	s32 retval = 0;

	BUG_ON(handle == NULL);
	temp_obj = (struct nameserver_object *) (*handle);
	retval = mutex_lock_interruptible(temp_obj->gate_handle);
	if (retval)
		goto exit;

	/* Do not proceed if an entry in the in the table */
	if (temp_obj->count != 0) {
		retval = -EBUSY;
		goto error;
	}

	retval = mutex_lock_interruptible(nameserver_state.list_lock);
	if (retval)
		goto error;

	list_del(&temp_obj->elem);
	mutex_unlock(nameserver_state.list_lock);
	gate_handle = temp_obj->gate_handle;
	/* free the memory allocated for instance name */
	kfree(temp_obj->name);
	/* Delete the lock handle if created internally */
	if (temp_obj->params.gate_handle == NULL)
		localgate = true;

	/* Free the memory used for handle */
	kfree(temp_obj);
	*handle = NULL;
	mutex_unlock(gate_handle);
	if (localgate == true)
		kfree(gate_handle);
	return 0;

error:
	mutex_unlock(temp_obj->gate_handle);
exit:
	printk(KERN_ERR "nameserver_delete failed retval:%x \n", retval);
	return retval;
}
EXPORT_SYMBOL(nameserver_delete);

/*
 * ======== nameserver_is_entry_found ========
 *  Purpose:
 *  This will  return true if the entry fond in the table
 */
static bool nameserver_is_entry_found(const char *name, u32 hash,
				struct list_head *list,
				struct nameserver_entry **entry)
{
	struct nameserver_entry *node = NULL;
	bool hash_match = false;
	bool name_match = false;


	list_for_each_entry(node, list, elem) {
		/* Hash not matchs, take next node	*/
		if (node->hash == hash)
			hash_match = true;
		else
			continue;
		/* If the name matchs, incase hash is duplicate */
		if (strcmp(node->name, name) == 0)
			name_match = true;

		if (hash_match && name_match) {
			if (entry != NULL)
				*entry = node;
			return true;
		}

		hash_match = false;
		name_match = false;
	}
	return false;
}

/*
 * ======== nameserver_add ========
 *  Purpose:
 *  This will  add an entry into a nameserver instance
 */
void *nameserver_add(void *handle, const char *name,
		void  *buffer, u32 length)
{
	struct nameserver_entry *new_node = NULL;
	struct nameserver_object *temp_obj = NULL;
	bool found = false;
	u32 hash;
	u32 name_len;
	s32 retval = 0;

	BUG_ON(handle == NULL);
	BUG_ON(name == NULL);
	BUG_ON(buffer == NULL);
	if (WARN_ON(length == 0)) {
		retval = -EINVAL;
		goto exit;
	}

	temp_obj = (struct nameserver_object *)handle;
	retval = mutex_lock_interruptible(temp_obj->gate_handle);
	if (retval)
		goto exit;

	if (temp_obj->count >= temp_obj->params.max_runtime_entries) {
		retval = -ENOSPC;
		goto error;
	}

	/* make the null char in to account */
	name_len = strlen(name) + 1;
	if (name_len > temp_obj->params.max_name_len) {
		retval = -E2BIG;
		goto error;
	}

	/* TODO : hash and collide ?? */
	hash = nameserver_string_hash(name);
	found = nameserver_is_entry_found(name, hash,
					&temp_obj->name_list, &new_node);
	if (found == true) {
		retval = -EEXIST;
		goto error_entry;
	}

	new_node = kmalloc(sizeof(struct nameserver_entry), GFP_KERNEL);
	if (new_node == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	new_node->hash    = hash;
	new_node->collide = true;
	new_node->len     = length;
	new_node->next    = NULL;
	new_node->name    = kmalloc(name_len, GFP_KERNEL);
	if (new_node->name == NULL) {
		retval = -ENOMEM;
		goto error;
	}

	new_node->buf  = kmalloc(length, GFP_KERNEL);
	if (new_node->buf == NULL) {
		retval = -ENOMEM;
		goto error1;
	}

	strncpy(new_node->name, name, name_len);
	memcpy(new_node->buf, buffer, length);
	list_add_tail(&new_node->elem, &temp_obj->name_list);
	temp_obj->count++;
	mutex_unlock(temp_obj->gate_handle);
	return new_node;

error1:
	kfree(new_node->name);
error:
	kfree(new_node);
error_entry:
	mutex_unlock(temp_obj->gate_handle);
exit:
	printk(KERN_ERR "nameserver_add failed status: %x \n", retval);
	return NULL;

}
EXPORT_SYMBOL(nameserver_add);

/*
 * ======== nameserver_add_uint32 ========
 *  Purpose:
 *  This will  a Uint32 value into a nameserver instance
 */
void *nameserver_add_uint32(void *handle, const char *name,
			u32 value)
{
	struct nameserver_entry *new_node = NULL;
	BUG_ON(handle == NULL);
	BUG_ON(name == NULL);

	new_node = nameserver_add(handle, name, &value, sizeof(u32));
	return new_node;
}
EXPORT_SYMBOL(nameserver_add_uint32);

/*
 * ======== nameserver_remove ========
 *  Purpose:
 *  This will  remove a name/value pair from a name server
 */
int nameserver_remove(void *handle, const char *name)
{
	struct nameserver_object *temp_obj = NULL;
	struct nameserver_entry *entry = NULL;
	bool found = false;
	u32 hash;
	u32 name_len;
	s32 retval = 0;

	BUG_ON(handle == NULL);
	BUG_ON(name == NULL);

	temp_obj = (struct nameserver_object *)handle;
	name_len = strlen(name) + 1;
	if (name_len > temp_obj->params.max_name_len) {
		retval = -E2BIG;
		goto exit;
	}

	retval = mutex_lock_interruptible(temp_obj->gate_handle);
	if (retval)
		goto exit;

	/* TODO :check collide & hash usage */
	hash = nameserver_string_hash(name);
	found = nameserver_is_entry_found(name, hash,
					&temp_obj->name_list, &entry);
	if (found == false) {
		retval = -ENOENT;
		goto error;
	}

	kfree(entry->buf);
	kfree(entry->name);
	list_del(&entry->elem);
	kfree(entry);
	temp_obj->count--;
	mutex_unlock(temp_obj->gate_handle);
	return 0;

error:
	mutex_unlock(temp_obj->gate_handle);

exit:
	printk(KERN_ERR "nameserver_remove failed status:%x \n", retval);
	return retval;
}
EXPORT_SYMBOL(nameserver_remove);

/*
 * ======== nameserver_remove_entry ========
 *  Purpose:
 *  This will  remove a name/value pair from a name server
 */
int nameserver_remove_entry(void *nshandle, void *nsentry)
{
	struct nameserver_entry *node = NULL;
	struct nameserver_object *handle = NULL;
	s32 retval = 0;

	BUG_ON(nshandle == NULL);
	BUG_ON(nsentry == NULL);

	handle = (struct nameserver_object *)nshandle;
	node = (struct nameserver_entry *)nsentry;
	retval = mutex_lock_interruptible(handle->gate_handle);
	if (retval)
		goto exit;

	kfree(node->buf);
	kfree(node->name);
	list_del(&node->elem);
	kfree(node);
	handle->count--;
	mutex_unlock(handle->gate_handle);
	return 0;

exit:
	printk(KERN_ERR "nameserver_remove_entry failed status:%x \n", retval);
	return retval;
}
EXPORT_SYMBOL(nameserver_remove_entry);


/*
 * ======== nameserver_get_local ========
 *  Purpose:
 *  This will retrieve the value portion of a name/value
 *  pair from local table
 */
int nameserver_get_local(void *handle, const char *name,
			void *buffer, u32 length)
{
	struct nameserver_object *temp_obj = NULL;
	struct nameserver_entry *entry = NULL;
	bool found = false;
	u32 hash;
	s32 retval = 0;

	BUG_ON(handle == NULL);
	BUG_ON(name == NULL);
	BUG_ON(buffer == NULL);
	if (WARN_ON(length == 0)) {
		retval = -EINVAL;
		goto exit;
	}

	temp_obj = (struct nameserver_object *)handle;
	retval = mutex_lock_interruptible(temp_obj->gate_handle);
	if (retval)
		goto exit;

	/* TODO :check collide & hash usage */
	hash = nameserver_string_hash(name);
	found = nameserver_is_entry_found(name, hash,
					&temp_obj->name_list, &entry);
	if (found == false) {
		retval = -ENOENT;
		goto error;
	}

	if (entry->len >= length) {
		memcpy(buffer, entry->buf, length);
		retval = length;
	} else {
		memcpy(buffer, entry->buf, entry->len);
		retval = entry->len;
	}

	mutex_unlock(temp_obj->gate_handle);
	return retval;

error:
	mutex_unlock(temp_obj->gate_handle);

exit:
	printk(KERN_ERR "nameserver_get_local entry not found!\n");
	return retval;
}
EXPORT_SYMBOL(nameserver_get_local);

/*
 * ======== nameserver_get ========
 *  Purpose:
 *  This will etrieve the value portion of a name/value
 *  pair from local table
 */
int nameserver_get(void *handle, const char *name,
		void *buffer, u32 length, u16 proc_id[])
{
	struct nameserver_object *temp_obj = NULL;
	u16 max_proc_id;
	u16 local_proc_id;
	s32 retval = -ENOENT;
	u32 i;

	BUG_ON(handle == NULL);
	BUG_ON(name == NULL);
	BUG_ON(buffer == NULL);
	if (WARN_ON(length == 0)) {
		retval = -EINVAL;
		goto exit;
	}

	temp_obj = (struct nameserver_object *)handle;
	max_proc_id = multiproc_get_max_processors();
	local_proc_id = multiproc_get_id(NULL);
	if (proc_id == NULL) {
		retval = nameserver_get_local(temp_obj, name,
						buffer, length);
		if (retval > 0) /* Got the value */
			goto exit;

		for (i = 0; i < max_proc_id; i++) {
			/* Skip current processor */
			if (i == local_proc_id)
				continue;

			if (nameserver_state.remote_handle_list[i] == NULL)
				continue;

			retval = nameserver_remote_get(
					nameserver_state.remote_handle_list[i],
					temp_obj->name, name, buffer, length);
			if (retval > 0 || ((retval < 0) &&
				(retval != -ENOENT))) /* Got the value */
				break;
		}
		goto exit;
	}

	for (i = 0; i < max_proc_id; i++) {
		/* Skip processor with invalid id */
		if (proc_id[i] == MULTIPROC_INVALIDID)
			continue;

		if (i == local_proc_id) {
			retval = nameserver_get_local(temp_obj,
							name, buffer, length);
			if (retval > 0)
				break;

		} else {
			retval = nameserver_remote_get(
				nameserver_state.remote_handle_list[proc_id[i]],
				temp_obj->name,	name, buffer, length);
			if (retval > 0 || ((retval < 0) &&
				(retval != -ENOENT))) /* Got the value */
				break;
		}
	}

exit:
	if (retval < 0)
		printk(KERN_ERR "nameserver_get failed: status=%x \n", retval);
	return retval;
}
EXPORT_SYMBOL(nameserver_get);

/*
 * ======== nameserver_get ========
 *  Purpose:
 *  This will etrieve the value portion of a name/value
 *  pair from local table
 *
 *  Returns the number of characters that matched with an entry
 *  So if "abc" was an entry and you called match with "abcd", this
 *  function will have the "abc" entry. The return would be 3 since
 *  three characters matched
 *
 */
int nameserver_match(void *handle, const char *name, u32 *value)
{
	struct nameserver_object *temp_obj = NULL;
	struct nameserver_entry *node = NULL;
	s32 retval = 0;
	u32 hash;
	bool found = false;

	BUG_ON(handle == NULL);
	BUG_ON(name == NULL);
	BUG_ON(value == NULL);

	temp_obj = (struct nameserver_object *)handle;
	retval = mutex_lock_interruptible(temp_obj->gate_handle);
	if (retval)
		goto exit;

	hash = nameserver_string_hash(name);
	list_for_each_entry(node, &temp_obj->name_list, elem) {
		if (node->hash == hash) {
			*value = *(u32 *)node->buf;
			found = true;
		}
	}

	if (found == false)
		retval = -ENOENT;

	mutex_unlock(temp_obj->gate_handle);

exit:
	if (retval < 0)
		printk(KERN_ERR "nameserver_match failed status:%x \n", retval);
	return retval;
}
EXPORT_SYMBOL(nameserver_match);

/*
 * ======== nameserver_register_remote_driver ========
 *  Purpose:
 *  This will register a remote driver for a processor
 */
int nameserver_register_remote_driver(void *handle, u16 proc_id)
{
	struct nameserver_remote_object *temp = NULL;
	s32 retval = 0;
	u16 proc_count;

	BUG_ON(handle == NULL);
	proc_count = multiproc_get_max_processors();
	if (WARN_ON(proc_id >= proc_count)) {
		retval = -EINVAL;
		goto exit;
	}

	temp = (struct nameserver_remote_object *)handle;
	nameserver_state.remote_handle_list[proc_id] = temp;
	return 0;

exit:
	printk(KERN_ERR
			"nameserver_register_remote_driver failed status:%x \n",
			retval);
	return retval;
}
EXPORT_SYMBOL(nameserver_register_remote_driver);

/*
 * ======== nameserver_unregister_remote_driver ========
 *  Purpose:
 *  This will unregister a remote driver for a  processor
 */
int nameserver_unregister_remote_driver(u16 proc_id)
{
	s32 retval = 0;
	u16 proc_count;

	proc_count = multiproc_get_max_processors();
	if (WARN_ON(proc_id >= proc_count)) {
		retval = -EINVAL;
		goto exit;
	}

	nameserver_state.remote_handle_list[proc_id] = NULL;
	return 0;

exit:
	printk(KERN_ERR
		"nameserver_unregister_remote_driver failed status:%x \n",
		retval);
	return retval;
}
EXPORT_SYMBOL(nameserver_unregister_remote_driver);

