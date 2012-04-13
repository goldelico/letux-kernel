/* OMAP Identity file for OMAP5 boards.
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Based on mach-omap2/board-44xx-identity.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/hardware.h>
#include <mach/id.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include "control.h"

static ssize_t omap5_soc_family_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "OMAP%04x\n", GET_OMAP_TYPE);
}

static ssize_t omap5_soc_revision_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "ES%d.%d\n", (GET_OMAP_REVISION() >> 4) & 0xf,
		       GET_OMAP_REVISION() & 0xf);
}

static const char *omap_types[] = {
	[OMAP2_DEVICE_TYPE_TEST]	= "TST",
	[OMAP2_DEVICE_TYPE_EMU]		= "EMU",
	[OMAP2_DEVICE_TYPE_SEC]		= "HS",
	[OMAP2_DEVICE_TYPE_GP]		= "GP",
	[OMAP2_DEVICE_TYPE_BAD]		= "BAD",
};

static ssize_t omap5_soc_type_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", omap_types[omap_type()]);
}


static ssize_t omap5_prod_id_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct  omap_die_id opi;
	omap_get_production_id(&opi);
	return sprintf(buf, "%08X-%08X\n", opi.id_1, opi.id_0);
}

static ssize_t omap5_die_id_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct  omap_die_id opi;
	omap_get_die_id(&opi);
	return sprintf(buf, "%08X-%08X-%08X-%08X\n", opi.id_3,
					opi.id_2, opi.id_1, opi.id_0);
}

#define OMAP5_SOC_ATTR_RO(_name, _show) \
	struct kobj_attribute omap5_soc_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)

static OMAP5_SOC_ATTR_RO(family, omap5_soc_family_show);
static OMAP5_SOC_ATTR_RO(revision, omap5_soc_revision_show);
static OMAP5_SOC_ATTR_RO(type, omap5_soc_type_show);
static OMAP5_SOC_ATTR_RO(production_id, omap5_prod_id_show);
static OMAP5_SOC_ATTR_RO(die_id, omap5_die_id_show);


static struct attribute *omap5_soc_prop_attrs[] = {
	&omap5_soc_prop_attr_family.attr,
	&omap5_soc_prop_attr_revision.attr,
	&omap5_soc_prop_attr_type.attr,
	&omap5_soc_prop_attr_production_id.attr,
	&omap5_soc_prop_attr_die_id.attr,
	NULL,
};

static struct attribute_group omap5_soc_prop_attr_group = {
	.attrs = omap5_soc_prop_attrs,
};

#define OMAP5_BRD_ATTR_RO(_name, _show) \
	struct kobj_attribute omap5_brd_prop_attr_##_name = \
		__ATTR(_name, S_IRUGO, _show, NULL)

static char *omap5_bname;
static ssize_t omap5_brd_name_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", omap5_bname);
}
OMAP5_BRD_ATTR_RO(name, omap5_brd_name_show);

static char *omap5_brev;
static ssize_t omap5_brd_rev_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", omap5_brev);
}
OMAP5_BRD_ATTR_RO(revision, omap5_brd_rev_show);

static char *omap5_bvname;
static ssize_t omap5_brd_vname_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", omap5_bvname);
}
OMAP5_BRD_ATTR_RO(variant_name, omap5_brd_vname_show);

static struct attribute *omap5_brd_prop_attrs[] = {
	&omap5_brd_prop_attr_name.attr,
	&omap5_brd_prop_attr_revision.attr,
	&omap5_brd_prop_attr_variant_name.attr,
	NULL,
};
static struct attribute_group omap5_brd_prop_attr_group = {
	.attrs = omap5_brd_prop_attrs,
};

void __init omap5_create_board_props(char *bname, char *rev, char *vname)
{
	struct kobject *board_props_kobj;
	struct kobject *soc_kobj = NULL;
	int ret = 0;

	omap5_bname = kasprintf(GFP_KERNEL, "%s", bname);
	omap5_brev = kasprintf(GFP_KERNEL, "%s", rev);
	omap5_bvname = kasprintf(GFP_KERNEL, "%s", vname);

	board_props_kobj = kobject_create_and_add("board_properties", NULL);

	if (!board_props_kobj)
		goto err_board_obj;

	ret = sysfs_create_group(board_props_kobj, &omap5_brd_prop_attr_group);
	if (ret)
		goto err_brd_soc_sysfs_create;

	soc_kobj = kobject_create_and_add("soc", board_props_kobj);
	if (!soc_kobj)
		goto err_soc_obj;

	ret = sysfs_create_group(soc_kobj, &omap5_soc_prop_attr_group);
	if (ret)
		goto err_soc_sysfs_create;

	return;

err_soc_sysfs_create:
	sysfs_remove_group(soc_kobj, &omap5_soc_prop_attr_group);
err_soc_obj:
	kobject_put(soc_kobj);
err_brd_soc_sysfs_create:
	sysfs_remove_group(board_props_kobj, &omap5_brd_prop_attr_group);
err_board_obj:
	if (!board_props_kobj || !soc_kobj || ret)
		pr_err("failed to create board_properties\n");
}
