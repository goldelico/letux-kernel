/*
 * This file is part of UBIFS.
 *
 * Copyright (C) 2006-2008 Nokia Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Authors: Artem Bityutskiy (Битюцкий Артём)
 *          Adrian Hunter
 */

/* This file implements VFS superblock operations */

#include <linux/kthread.h>
#include <linux/seq_file.h>
#include <linux/mount.h>
#include "ubifs.h"

/**
 * validate_inode - validate inode.
 * @c: UBIFS file-system description object
 * @inode: the inode to validate
 *
 * This is a helper function for 'ubifs_iget()' which validates various fields
 * of a newly built inode to make sure they contain sane values and prevent
 * possible vulnerabilities. Returns zero if the inode is all right and
 * a non-zero error code if not.
 */
/* TODO: remove compatibility stuff as late as possible */
#ifndef UBIFS_COMPAT_USE_OLD_IGET
static int validate_inode(struct ubifs_info *c, const struct inode *inode)
#else
int validate_inode(struct ubifs_info *c, const struct inode *inode)
#endif
{
	int err;
	const struct ubifs_inode *ui = ubifs_inode(inode);

	if (inode->i_size > c->max_inode_sz) {
		ubifs_err("inode is too large (%lld)",
			  (long long)inode->i_size);
		return 1;
	}

	if (ui->compr_type < 0 || ui->compr_type >= UBIFS_COMPR_TYPES_CNT) {
		ubifs_err("unknown compression type %d", ui->compr_type);
		return 2;
	}

	if (ui->xattr_cnt < 0)
		return 3;

	if (ui->xattr_size < 0)
		return 4;

	if (ui->xattr_names < 0 ||
	    ui->xattr_names + ui->xattr_cnt > XATTR_LIST_MAX)
		return 5;

	if (ui->data_len < 0 || ui->data_len > UBIFS_MAX_INO_DATA)
		return 6;

	if (!ubifs_compr_present(ui->compr_type)) {
		ubifs_warn("inode %lu uses '%s' compression, but it was not "
			   "compiled in", inode->i_ino,
			   ubifs_compr_name(ui->compr_type));
	}

	err = dbg_check_dir_size(c, inode);
	return err;
}

/* TODO: remove compatibility stuff as late as possible */
#ifndef UBIFS_COMPAT_USE_OLD_IGET
struct inode *ubifs_iget(struct super_block *sb, unsigned long inum)
{
	int err;
	union ubifs_key key;
	struct ubifs_ino_node *ino;
	struct ubifs_info *c = sb->s_fs_info;
	struct inode *inode;
	struct ubifs_inode *ui;

	dbg_gen("inode %lu", inum);

	inode = iget_locked(sb, inum);
	if (!inode)
		return ERR_PTR(-ENOMEM);
	if (!(inode->i_state & I_NEW))
		return inode;
	ui = ubifs_inode(inode);

	ino = kmalloc(UBIFS_MAX_INO_NODE_SZ, GFP_NOFS);
	if (!ino) {
		err = -ENOMEM;
		goto out;
	}

	ino_key_init(c, &key, inode->i_ino);

	err = ubifs_tnc_lookup(c, &key, ino);
	if (err)
		goto out_ino;

	inode->i_flags |= (S_NOCMTIME | S_NOATIME);
	inode->i_nlink = le32_to_cpu(ino->nlink);
	inode->i_uid   = le32_to_cpu(ino->uid);
	inode->i_gid   = le32_to_cpu(ino->gid);
	inode->i_atime.tv_sec  = (int64_t)le64_to_cpu(ino->atime_sec);
	inode->i_atime.tv_nsec = le32_to_cpu(ino->atime_nsec);
	inode->i_mtime.tv_sec  = (int64_t)le64_to_cpu(ino->mtime_sec);
	inode->i_mtime.tv_nsec = le32_to_cpu(ino->mtime_nsec);
	inode->i_ctime.tv_sec  = (int64_t)le64_to_cpu(ino->ctime_sec);
	inode->i_ctime.tv_nsec = le32_to_cpu(ino->ctime_nsec);
	inode->i_mode  = le32_to_cpu(ino->mode);
	inode->i_size  = le64_to_cpu(ino->size);

	ui->data_len    = le32_to_cpu(ino->data_len);
	ui->flags       = le32_to_cpu(ino->flags);
	ui->compr_type  = le16_to_cpu(ino->compr_type);
	ui->creat_sqnum = le64_to_cpu(ino->creat_sqnum);
	ui->xattr_cnt   = le32_to_cpu(ino->xattr_cnt);
	ui->xattr_size  = le64_to_cpu(ino->xattr_size);
	ui->xattr_names = le32_to_cpu(ino->xattr_names);

	err = validate_inode(c, inode);
	if (err)
		goto out_invalid;

	switch (inode->i_mode & S_IFMT) {
	case S_IFREG:
		inode->i_mapping->a_ops = &ubifs_file_address_operations;
		inode->i_op = &ubifs_file_inode_operations;
		inode->i_fop = &ubifs_file_operations;
		if (ui->data_len != 0) {
			err = 10;
			goto out_invalid;
		}
		break;
	case S_IFDIR:
		inode->i_op  = &ubifs_dir_inode_operations;
		inode->i_fop = &ubifs_dir_operations;
		if (ui->data_len != 0) {
			err = 11;
			goto out_invalid;
		}
		break;
	case S_IFLNK:
		inode->i_op = &ubifs_symlink_inode_operations;
		if (ui->data_len <= 0 || ui->data_len > UBIFS_MAX_INO_DATA) {
			err = 12;
			goto out_invalid;
		}
		ui->data = kmalloc(ui->data_len + 1, GFP_NOFS);
		if (!ui->data) {
			err = -ENOMEM;
			goto out_ino;
		}
		memcpy(ui->data, ino->data, ui->data_len);
		((char *)ui->data)[ui->data_len] = '\0';
		break;
	case S_IFBLK:
	case S_IFCHR:
	{
		dev_t rdev;
		union ubifs_dev_desc *dev;

		ui->data = kmalloc(sizeof(union ubifs_dev_desc), GFP_NOFS);
		if (!ui->data) {
			err = -ENOMEM;
			goto out_ino;
		}

		dev = (union ubifs_dev_desc *)ino->data;
		if (ui->data_len == sizeof(dev->new))
			rdev = new_decode_dev(le32_to_cpu(dev->new));
		else if (ui->data_len == sizeof(dev->huge))
			rdev = huge_decode_dev(le64_to_cpu(dev->huge));
		else {
			err = 13;
			goto out_invalid;
		}
		memcpy(ui->data, ino->data, ui->data_len);
		inode->i_op = &ubifs_file_inode_operations;
		init_special_inode(inode, inode->i_mode, rdev);
		break;
	}
	case S_IFSOCK:
	case S_IFIFO:
		inode->i_op = &ubifs_file_inode_operations;
		init_special_inode(inode, inode->i_mode, 0);
		if (ui->data_len != 0) {
			err = 14;
			goto out_invalid;
		}
		break;
	default:
		err = 15;
		goto out_invalid;
	}

	kfree(ino);
	ubifs_set_inode_flags(inode);
	unlock_new_inode(inode);
	return inode;

out_invalid:
	ubifs_err("inode %lu validation failed, error %d", inode->i_ino, err);
	dbg_dump_node(c, ino);
	dbg_dump_inode(c, inode);
	err = -EINVAL;
out_ino:
	kfree(ino);
out:
	ubifs_err("failed to read inode %lu, error %d", inode->i_ino, err);
	iget_failed(inode);
	return ERR_PTR(err);
}

#endif /* UBIFS_COMPAT_USE_OLD_IGET */

static struct inode *ubifs_alloc_inode(struct super_block *sb)
{
	struct ubifs_inode *ui;

	ui = kmem_cache_alloc(ubifs_inode_slab, GFP_NOFS);
	if (!ui)
		return NULL;

	memset((void *)ui + sizeof(struct inode), 0,
	       sizeof(struct ubifs_inode) - sizeof(struct inode));
	mutex_init(&ui->budg_mutex);
	return &ui->vfs_inode;
};

static void ubifs_destroy_inode(struct inode *inode)
{
	struct ubifs_inode *ui = ubifs_inode(inode);

	kfree(ui->data);
	kmem_cache_free(ubifs_inode_slab, inode);
}

static void ubifs_put_super(struct super_block *sb)
{
	int i;
	struct ubifs_info *c = sb->s_fs_info;

	ubifs_msg("un-mount UBI device %d, volume %d", c->vi.ubi_num,
		  c->vi.vol_id);
	/*
	 * The following asserts are only valid if there has not been a failure
	 * of the media. For example, there will be dirty inodes if we failed
	 * to write them back because of I/O errors.
	 */
	ubifs_assert(atomic_long_read(&c->dirty_pg_cnt) == 0);
	ubifs_assert(atomic_long_read(&c->dirty_ino_cnt) == 0);
	ubifs_assert(c->budg_idx_growth == 0);
	ubifs_assert(c->budg_data_growth == 0);

	/*
	 * The 'c->umount_lock' prevents races between UBIFS memory shrinker
	 * and file system un-mount. Namely, it prevents the shrinker from
	 * picking this superblock for shrinking - it will be just skipped if
	 * the mutex is locked.
	 */
	mutex_lock(&c->umount_mutex);

	spin_lock(&ubifs_infos_lock);
	list_del(&c->infos_list);
	spin_unlock(&ubifs_infos_lock);

	if (!(c->vfs_sb->s_flags & MS_RDONLY)) {
		/*
		 * First of all kill the background thread to make sure it does
		 * not interfere with un-mounting and freeing resources.
		 */
		if (c->bgt) {
			kthread_stop(c->bgt);
			c->bgt = NULL;
		}

		/* Synchronize write-buffers */
		if (c->jheads)
			for (i = 0; i < c->jhead_cnt; i++) {
				ubifs_wbuf_sync(&c->jheads[i].wbuf);
				del_timer_sync(&c->jheads[i].wbuf.timer);
			}

		/*
		 * On fatal errors c->ro_media is set to 1, in which case we do
		 * not write the master node.
		 */
		if (!c->ro_media) {
			/*
			 * We are being cleanly unmounted which means the
			 * orphans were killed - indicate this in the master
			 * node. Also save the reserved GC LEB number.
			 */
			int err;

			c->mst_node->flags &= ~cpu_to_le32(UBIFS_MST_DIRTY);
			c->mst_node->flags |= cpu_to_le32(UBIFS_MST_NO_ORPHS);
			c->mst_node->gc_lnum = cpu_to_le32(c->gc_lnum);
			err = ubifs_write_master(c);
			if (err)
				/*
				 * Recovery will attempt to fix the master area
				 * next mount, so we just print a message and
				 * continue to unmount normally.
				 */
				ubifs_err("failed to write master node, "
					  "error %d", err);
		}
	}

	ubifs_umount(c);
	ubi_close_volume(c->ubi);
	mutex_unlock(&c->umount_mutex);
	kfree(c);
}

/*
 * Note, Linux write-back code calls this without 'i_mutex'.
 */
static int ubifs_write_inode(struct inode *inode, int wait)
{
	int err;
	struct ubifs_info *c = inode->i_sb->s_fs_info;
	struct ubifs_inode *ui = ubifs_inode(inode);
	struct ubifs_budget_req req = {.dd_growth = c->inode_budget,
				       .dirtied_ino_d = ui->data_len};

	ubifs_assert(!(c->vfs_sb->s_flags & MS_RDONLY));
	ubifs_assert(!ui->xattr);

	if (is_bad_inode(inode))
		return 0;

	mutex_lock(&ui->budg_mutex);

	/*
	 * Due to races between write-back forced by budgeting
	 * (see 'sync_some_inodes()') and pdflush write-back, the inode may
	 * have already been synchronized, do not do this again.
	 *
	 * This might also happen if it was synchronized in e.g. ubifs_link()',
	 * etc.
	 */
	if (!ui->dirty) {
		mutex_unlock(&ui->budg_mutex);
		return 0;
	}

	ubifs_assert(ui->budgeted);
	dbg_gen("inode %lu", inode->i_ino);

	err = ubifs_jrn_write_inode(c, inode, 0, IS_SYNC(inode));
	if (err)
		ubifs_err("can't write inode %lu, error %d", inode->i_ino, err);

	ui->dirty = 0;
	UBIFS_DBG(ui->budgeted = 0);
	atomic_long_dec(&c->dirty_ino_cnt);

	ubifs_release_budget(c, &req);
	mutex_unlock(&ui->budg_mutex);

	return err;
}

static void ubifs_delete_inode(struct inode *inode)
{
	struct ubifs_info *c = inode->i_sb->s_fs_info;
	struct ubifs_inode *ui = ubifs_inode(inode);
	struct ubifs_budget_req req = {.dd_growth = c->inode_budget,
				       .dirtied_ino_d = ui->data_len};
	int err;

	if (ui->xattr) {
		/*
		 * Extended attribute inode deletions are fully handled in
		 * 'ubifs_removexattr()'. These inodes are special and have
		 * limited usage, so there is nothing to do here.
		 */
		ubifs_assert(!ui->dirty);
		goto out;
	}

	dbg_gen("inode %lu", inode->i_ino);
	ubifs_assert(!atomic_read(&inode->i_count));
	ubifs_assert(inode->i_nlink == 0);

	truncate_inode_pages(&inode->i_data, 0);
	if (is_bad_inode(inode))
		goto out;

	mutex_lock(&ui->budg_mutex);

	inode->i_size = 0;

	err = ubifs_jrn_write_inode(c, inode, 1, IS_SYNC(inode));
	if (err)
		/*
		 * Worst case we have a lost orphan inode wasting space, so a
		 * simple error message is ok here.
		 */
		ubifs_err("can't write inode %lu, error %d", inode->i_ino, err);

	if (ui->dirty) {
		ubifs_assert(ui->budgeted);
		atomic_long_dec(&c->dirty_ino_cnt);
		ui->dirty = 0;
		UBIFS_DBG(ui->budgeted = 0);
		ubifs_release_budget(c, &req);
	}

	mutex_unlock(&ui->budg_mutex);
out:
	clear_inode(inode);
}

static void ubifs_dirty_inode(struct inode *inode)
{
	struct ubifs_inode *ui = ubifs_inode(inode);

	ubifs_assert(!(inode->i_sb->s_flags & MS_RDONLY));
	ubifs_assert(mutex_is_locked(&ui->budg_mutex));

	if (!ui->dirty) {
		struct ubifs_info *c = inode->i_sb->s_fs_info;

		ui->dirty = 1;
		atomic_long_inc(&c->dirty_ino_cnt);
		dbg_gen("inode %lu",  inode->i_ino);
	}
}

static int ubifs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	struct ubifs_info *c = dentry->d_sb->s_fs_info;
	unsigned long long free;

	free = ubifs_budg_get_free_space(c);
	dbg_gen("free space %lld bytes (%lld blocks)",
		free, free >> UBIFS_BLOCK_SHIFT);

	buf->f_type = UBIFS_SUPER_MAGIC;
	buf->f_bsize = UBIFS_BLOCK_SIZE;
	buf->f_blocks = c->block_cnt;
	buf->f_bfree = free >> UBIFS_BLOCK_SHIFT;
	if (free > c->report_rp_size)
		buf->f_bavail = (free - c->report_rp_size) >> UBIFS_BLOCK_SHIFT;
	else
		buf->f_bavail = 0;
	buf->f_files = 0;
	buf->f_ffree = 0;
	buf->f_namelen = UBIFS_MAX_NLEN;

	return 0;
}

static int ubifs_remount_fs(struct super_block *sb, int *flags, char *data)
{
	int err;
	struct ubifs_info *c = sb->s_fs_info;

	dbg_gen("old flags %#lx, new flags %#x", sb->s_flags, *flags);

	err = ubifs_parse_options(c, data, 1);
	if (err) {
		ubifs_err("invalid or unknown remount parameter");
		return err;
	}
	if ((sb->s_flags & MS_RDONLY) && !(*flags & MS_RDONLY)) {
		err = ubifs_remount_rw(c);
		if (err)
			return err;
	} else if (!(sb->s_flags & MS_RDONLY) && (*flags & MS_RDONLY))
		ubifs_remount_ro(c);

	return 0;
}

static int ubifs_show_options(struct seq_file *s, struct vfsmount *mnt)
{
	struct ubifs_info *c = mnt->mnt_sb->s_fs_info;

	if (c->mount_opts.unmount_mode == 2)
		seq_printf(s, ",fast_unmount");
	else if (c->mount_opts.unmount_mode == 1)
		seq_printf(s, ",norm_unmount");

	return 0;
}

static int ubifs_sync_fs(struct super_block *sb, int wait)
{
	struct ubifs_info *c = sb->s_fs_info;
	int i, ret = 0, err;

	if (c->jheads)
		for (i = 0; i < c->jhead_cnt; i++) {
			err = ubifs_wbuf_sync(&c->jheads[i].wbuf);
			if (err && !ret)
				ret = err;
		}
	/*
	 * We ought to call sync for c->ubi but it does not have one. If it had
	 * it would in turn call mtd->sync, however mtd operations are
	 * synchronous anyway, so we don't lose any sleep here.
	 */
	return ret;
}

struct super_operations ubifs_super_operations = {
/* TODO: remove compatibility stuff as late as possible */
#ifdef UBIFS_COMPAT_USE_OLD_IGET
	.read_inode    = ubifs_read_inode,
#endif
	.alloc_inode   = ubifs_alloc_inode,
	.destroy_inode = ubifs_destroy_inode,
	.put_super     = ubifs_put_super,
	.write_inode   = ubifs_write_inode,
	.delete_inode  = ubifs_delete_inode,
	.statfs        = ubifs_statfs,
	.dirty_inode   = ubifs_dirty_inode,
	.remount_fs    = ubifs_remount_fs,
	.show_options  = ubifs_show_options,
	.sync_fs       = ubifs_sync_fs,
};
