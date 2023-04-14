/*
 * Core driver access RICOH619 power management chip.
 *
 * Copyright (C) 2020 Ingenic Semiconductor Co., Ltd.
 * Author: cljiang <conglong.jiang@ingenic.com>
 *
 * Based on code
 *	drivers/mfd/ricoh619.c
 *	Copyright (c) 2011-2012, NVIDIA CORPORATION.  All rights reserved.
 *	Author: Laxman dewangan <ldewangan@nvidia.com>
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ricoh619.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <asm/delay.h>


typedef struct{
    int temp;    
    unsigned int R;
}R_T;

static R_T sheet[] ={
    {-40, 188500},
    {-39, 178600},
    {-38, 169200},
    {-37, 160400},
    {-36, 152100},
    {-35, 144300},
    {-34, 136900},
    {-33, 130000},
    {-32, 123400},
    {-31, 117200},
    {-30, 111300},
    {-29, 105800},
    {-28, 100600},
    {-27, 95640},
    {-26, 90970},
    {-25, 86560},
    {-24, 82380},
    {-23, 78430},
    {-22, 74690},
    {-21, 71140},
    {-20, 67790},
    {-19, 64610},
    {-18, 61600},
    {-17, 58740},
    {-16, 56030},
    {-15, 53460},
    {-14, 51030},
    {-13, 48710},
    {-12, 46520},
    {-11, 44430},
    {-10, 42450},
    {-9, 40570},
    {-8, 38780},
    {-7, 37080},
    {-6, 35460},
    {-5, 33930},
    {-4, 32460},
    {-3, 31070},
    {-2, 29750},
    {-1, 28490},
    {0, 27280},
    {1, 26140},
    {2, 25050},
    {3, 24010},
    {4, 23020},
    {5, 22070},
    {6, 21170},
    {7, 20310},
    {8, 19490},
    {9, 18710},
    {10, 17960},
    {11, 17250},
    {12, 16570},
    {13, 15910},
    {14, 15290},
    {15, 14700},
    {16, 14130},
    {17, 13590},
    {18, 13070},
    {19, 12570},
    {20, 12090},
    {21, 11640},
    {22, 11200},
    {23, 10780},
    {24, 10380},
    {25, 10000},
    {26, 9633},
    {27, 9282},
    {28, 8945},
    {29, 8622},
    {30, 8312},
    {31, 8015},
    {32, 7730},
    {33, 7456},
    {34, 7194},
    {35, 6942},
    {36, 6700},
    {37, 6468},
    {38, 6245},
    {39, 6031},
    {40, 5826},
    {41, 5628},
    {42, 5438},
    {43, 5255},
    {44, 5080},
    {45, 4911},
    {46, 4749},
    {47, 4592},
    {48, 4442},
    {49, 4297},
    {50, 4158},
    {51, 4024},
    {52, 3895},
    {53, 3771},
    {54, 3651},
    {55, 3536},
    {56, 3425},
    {57, 3318},
    {58, 3215},
    {59, 3115},
    {60, 3019},
    {61, 2927},
    {62, 2837},
    {63, 2751},
    {64, 2668},
    {65, 2588},
    {66, 2511},
    {67, 2436},
    {68, 2364},
    {69, 2295},
    {70, 2227},
    {71, 2163},
    {72, 2100},
    {73, 2039},
    {74, 1981},
    {75, 1924},
    {76, 1869},
    {77, 1817},
    {78, 1765},
    {79, 1716},
    {80, 1668},
    {81, 1622},
    {82, 1577},
    {83, 1534},
    {84, 1492},
    {85, 1451},
    {86, 1412},
    {87, 1374},
    {88, 1337},
    {89, 1302},
    {90, 1267},
    {91, 1234},
    {92, 1201},
    {93, 1170},
    {94, 1139},
    {95, 1110},
    {96, 1081},
    {97, 1054},
    {98, 1027},
    {99, 1001},
    {100, 975},
    {101, 951},
    {102, 927},
    {103, 904},
    {104, 881},
    {105, 860},
    {106, 838},
    {107, 818},
    {108, 798},
    {109, 779},
    {110, 760},
};

static unsigned int abs_subtract_val(int val_1, int val_2)
{
    if(val_1 > val_2)
        return (val_1 - val_2);
    else
        return (val_2 - val_1);
}

static int temp_foreach_sheet(unsigned int R)
{
    int index = 0;
    int i = 0;
    int num = sizeof(sheet) / sizeof(R_T);
    unsigned int abs = abs_subtract_val(R , sheet[0].R);
    int tmp;
    for(i = 1;i < num;i++){
        tmp = abs_subtract_val(R , sheet[i].R);
        if(tmp < abs){
            index = i; 
            abs = tmp;
            if(tmp < 10)
                break;
        }
    }
    return (int)sheet[index].temp;
}


static struct i2c_client *ricoh619_i2c = NULL;
static struct mutex ricoh61x_io_lock;

static const struct mfd_cell ricoh619_cells[] = {
	{ .name = "ricoh619-regulator" },
    { .name = "ricoh619-battery"},
	/*{ .name = "ricoh619-wdt" },
	  { .name = "ricoh619-pinctrl" },
	  { .name = "ricoh619-pm" }, */
};

static bool ricoh619_reg_hole(unsigned int reg)
{
#define RICOH619_REG_SINGLE_HOLE_NUM 16
	unsigned char single_hole[RICOH619_REG_SINGLE_HOLE_NUM] = {
		0x4, 0x8, 0x2b, 0x43, 0x49, 0x4b, 0x62, 0x63, 0x67, 0x8b,
            0x8f, 0x99, 0x9a, 0x9b, 0x9f, 0xb2, };
	int i;

	for (i = 0; i < RICOH619_REG_SINGLE_HOLE_NUM; i++)
		if ((unsigned char)reg == single_hole[i])
			return true;
	if (reg > RICOH619_MAX_REG)
		return true;
#undef RICOH619_REG_SINGLE_HOLE_NUM
	return false;
}

static bool ricoh619_opable_reg(struct device *dev, unsigned int reg)
{
	return !ricoh619_reg_hole(reg);
}

static bool ricoh619_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case RICOH619_WATCHDOGCNT:
	case RICOH619_DCIRQ:
	case RICOH619_IR_GPR:
	case RICOH619_IR_GPF:
	case RICOH619_MON_IOIN:
	case RICOH619_INTMON:
		return true;
	default:
		return false;
	}
}

static struct reg_default ricoh619_reg_default[RICOH619_REG_NUM];
static const struct regmap_config ricoh619_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.volatile_reg	= ricoh619_volatile_reg,
	.writeable_reg  = ricoh619_opable_reg,
	.readable_reg	= ricoh619_opable_reg,
	.max_register	= RICOH619_MAX_REG,
	.reg_defaults	= ricoh619_reg_default,
	.num_reg_defaults = ARRAY_SIZE(ricoh619_reg_default),
	.cache_type	= REGCACHE_RBTREE,
};
static inline int set_bank_ricoh61x(struct device *dev, int bank)
{
    int ret = 0;

    /*if (bank != (bank & 1))*/
        /*return -EINVAL;*/
    /*if (bank == ricoh61x->bank_num)*/
        /*return 0;*/
    /*ret = __ricoh61x_write(to_i2c_client(dev), RICOH61x_REG_BANKSEL, bank);*/
    /*if (!ret)*/
        /*ricoh61x->bank_num = bank;*/
    return ret;
}

static int __ricoh619_reg_read(struct i2c_client *i2c, u8 reg)
{
	return i2c_smbus_read_byte_data(i2c, reg);
}

static inline int __ricoh61x_bulk_reads(struct i2c_client *client, u8 reg,
        int len, uint8_t *val)
{
    int ret;
    int i;

    ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
    if (ret < 0) {
        dev_err(&client->dev, "failed reading from 0x%02x\n", reg);
        return ret;
    }
    for (i = 0; i < len; ++i) {
        dev_dbg(&client->dev, "ricoh61x: reg read  reg=%x, val=%x\n",
                reg + i, *(val + i));
    }
    return 0;
}

int ricoh61x_read(struct device* dev, u8 reg, uint8_t* val)
{
    int ret;
    mutex_lock(&ricoh61x_io_lock);
    if(ricoh619_i2c){
        *val =  __ricoh619_reg_read(ricoh619_i2c, reg);
        ret = 0;
    }else{
        ret = -1;
    }
    mutex_unlock(&ricoh61x_io_lock);
    return ret;
}
EXPORT_SYMBOL(ricoh61x_read);

int ricoh61x_bulk_reads(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
    int ret = 0;
    mutex_lock(&ricoh61x_io_lock);
    ret = set_bank_ricoh61x(dev, 0);
    if (!ret)
        ret = __ricoh61x_bulk_reads(ricoh619_i2c, reg, len, val);
    mutex_unlock(&ricoh61x_io_lock);
    return ret;
}
EXPORT_SYMBOL_GPL(ricoh61x_bulk_reads);
int ricoh61x_bulk_reads_bank1(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
    int ret = 0;

    mutex_lock(&ricoh61x_io_lock);
    ret = set_bank_ricoh61x(dev, 1);
    if (!ret)
        ret = __ricoh61x_bulk_reads(ricoh619_i2c, reg, len, val);
    mutex_unlock(&ricoh61x_io_lock);

    return ret;
}
EXPORT_SYMBOL_GPL(ricoh61x_bulk_reads_bank1);
static inline int __ricoh61x_bulk_writes(struct i2c_client *client, u8 reg,
        int len, uint8_t *val)
{
    int ret;
    int i;

    for (i = 0; i < len; ++i) {
        dev_dbg(&client->dev, "ricoh61x: reg write  reg=%x, val=%x\n",
                reg + i, *(val + i));
    }
    ret = i2c_smbus_write_i2c_block_data(client, reg, len, val);
    if (ret < 0) {
        dev_err(&client->dev, "failed writings to 0x%02x\n", reg);
        return ret;
    }
    return 0;
}

int ricoh61x_bulk_writes_bank1(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
    int ret = 0;

    mutex_lock(&ricoh61x_io_lock);
    ret = set_bank_ricoh61x(dev, 1);
    if (!ret)
        ret = __ricoh61x_bulk_writes(to_i2c_client(dev), reg, len, val);
    mutex_unlock(&ricoh61x_io_lock);

    return ret;
}
EXPORT_SYMBOL_GPL(ricoh61x_bulk_writes_bank1);
static int __ricoh619_reg_write(struct i2c_client *i2c, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(i2c, reg, val);
}

int ricoh61x_write(struct device* dev, u8 reg, uint8_t val)
{
    int ret;
    mutex_lock(&ricoh61x_io_lock);
    if(ricoh619_i2c)
        ret =  __ricoh619_reg_write(ricoh619_i2c, reg, val);
    else 
        ret = -1;
    mutex_unlock(&ricoh61x_io_lock);
    return ret;
}
EXPORT_SYMBOL(ricoh61x_write);
static inline int __ricoh61x_read(struct i2c_client *client,
        u8 reg, uint8_t *val)
{
    int ret;

    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0) {
        dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
        return ret;
    }

    *val = (uint8_t)ret;
    dev_dbg(&client->dev, "ricoh61x: reg read  reg=%x, val=%x\n",
            reg, *val);
    return 0;
}

int ricoh61x_read_bank1(struct device *dev, u8 reg, uint8_t *val)
{
    int ret = 0;

    mutex_lock(&ricoh61x_io_lock);
    ret = set_bank_ricoh61x(dev, 1);
    if (!ret)
        ret =  __ricoh61x_read(to_i2c_client(dev), reg, val);
    mutex_unlock(&ricoh61x_io_lock);

    return ret;
}
EXPORT_SYMBOL_GPL(ricoh61x_read_bank1);
int ricoh61x_bulk_writes(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
    int ret = 0;

    mutex_lock(&ricoh61x_io_lock);
    ret = set_bank_ricoh61x(dev, 0);
    if (!ret)
        ret = __ricoh61x_bulk_writes(to_i2c_client(dev), reg, len, val);
    mutex_unlock(&ricoh61x_io_lock);

    return ret;
}
EXPORT_SYMBOL_GPL(ricoh61x_bulk_writes);

int ricoh61x_set_bits(struct device *dev, u8 reg, uint8_t bit_mask)
{
    uint8_t reg_val;
    int ret = 0;

    mutex_lock(&ricoh61x_io_lock);
    if(ricoh619_i2c){
        reg_val = __ricoh619_reg_read(ricoh619_i2c, reg);

        if ((reg_val & bit_mask) != bit_mask) {
            reg_val |= bit_mask;
            ret = __ricoh619_reg_write(ricoh619_i2c, reg,
                    reg_val);
        }
    }
    mutex_unlock(&ricoh61x_io_lock);
    return ret;
}
EXPORT_SYMBOL_GPL(ricoh61x_set_bits);

int ricoh61x_clr_bits(struct device *dev, u8 reg, uint8_t bit_mask)
{
    uint8_t reg_val;
    int ret = 0;

    mutex_lock(&ricoh61x_io_lock);
    if(ricoh619_i2c){
        reg_val = __ricoh619_reg_read(ricoh619_i2c, reg);
        if (reg_val & bit_mask) {
            reg_val &= ~bit_mask;
            ret = __ricoh619_reg_write(ricoh619_i2c, reg,
                    reg_val);
        }
    }
    mutex_unlock(&ricoh61x_io_lock);
    return ret;
}
EXPORT_SYMBOL_GPL(ricoh61x_clr_bits);

int ricoh61x_write_bank1(struct device *dev, u8 reg, uint8_t val)
{
    int ret = 0;

    mutex_lock(&ricoh61x_io_lock);
    ret =  __ricoh619_reg_write(ricoh619_i2c, reg, val);
    mutex_unlock(&ricoh61x_io_lock);

    return ret;
}
EXPORT_SYMBOL_GPL(ricoh61x_write_bank1);


static void ricoh619_reg_default_init(struct i2c_client *i2c)
{
	unsigned int reg, def;
	int i;

	for (i = 0, reg = 0; i < RICOH619_REG_NUM &&
			reg <= RICOH619_MAX_REG; reg++) {
		if (ricoh619_reg_hole(reg) ||
				ricoh619_volatile_reg(NULL, reg))
			continue;
		ricoh619_reg_default[i].reg = reg;
		def = __ricoh619_reg_read(i2c, (u8)reg);
		if (def < 0) {
			dev_warn(&i2c->dev, "register %x read failed: %d\n", reg, def);
			ricoh619_reg_default[i++].def = 0;
		} else{
			ricoh619_reg_default[i++].def = def;
//            printk("ricoh619: reg: 0x%x , val: 0x%x\n", ricoh619_reg_default[i-1].reg, ricoh619_reg_default[i-1].def);
        }
	}
}

static int battery_ntc_R_val(int thermbat_val)
{
    thermbat_val *= 10;
#define R493 10000 
#define VCHGREGA 2500
    if(thermbat_val > 0)
        return thermbat_val * R493 / (VCHGREGA - thermbat_val);
    else
        return 0;
}

static void config_xxzA926_info(struct i2c_client *i2c)
{
    unsigned int reg_val;
    struct regulator *reg = devm_regulator_get(&i2c->dev, "ldo3_2v8");
    regulator_set_voltage(reg, 2800000, 2800000);
    regulator_enable(reg);


    reg = devm_regulator_get(&i2c->dev, "DC1_0v9");
    regulator_set_voltage(reg, 900000, 900000);
    regulator_enable(reg);

    /*regulator_enable(reg);*/
    reg = devm_regulator_get(&i2c->dev, "DC5_1v2");
    regulator_set_voltage(reg, 1200000, 1200000);
    regulator_enable(reg);
    printk("cljiang======================RICOH619_DC5CTL = 0x%x\n", __ricoh619_reg_read(i2c, RICOH619_DC5CTL));
    printk("cljiang======================RICOH619_DC5DAC = 0x%x\n", __ricoh619_reg_read(i2c, RICOH619_DC5DAC));

    reg = devm_regulator_get(&i2c->dev, "ldortc2_0v9");
    regulator_set_voltage(reg, 900000, 900000);
    regulator_enable(reg);

    /*Full-charging Voltage*/
    reg_val = __ricoh619_reg_read(i2c, RICOH619_BATSET2);
    printk("cljiang======================RICOH619_BATSET2 = 0x%x\n", reg_val);
    reg_val &= 0x84;
    reg_val |= 0x44; /*full-Charging volt 4.35v, Re-Charging volt 4.1v*/
    __ricoh619_reg_write(i2c, RICOH619_BATSET2, reg_val);
    printk("cljiang======================set RICOH619_BATSET2 = 0x%x\n", __ricoh619_reg_read(i2c, RICOH619_BATSET2));

    /*Over Voltage*/
    reg_val = __ricoh619_reg_read(i2c, RICOH619_BATSET1);
    reg_val &= 0xEF;
    reg_val |= 0x10;
    __ricoh619_reg_write(i2c, RICOH619_BATSET1, reg_val);
    printk("cljiang======================set RICOH619_BATSET1 = 0x%x\n", __ricoh619_reg_read(i2c, RICOH619_BATSET1));


    /*Charging current*/
        /*limited ADP*/
    __ricoh619_reg_write(i2c, RICOH619_REGISET1, 0x13); /* 2000ma*/ 
    printk("cljiang======================RICOH619_REGISET1 = 0x%x\n", __ricoh619_reg_read(i2c, RICOH619_REGISET1));

        /*limited USB */
    printk("cljiang======================RICOH619_REGISET1 = 0x%x\n", __ricoh619_reg_read(i2c, RICOH619_REGISET1));
    reg_val = __ricoh619_reg_read(i2c, RICOH619_REGISET2);
    reg_val &= 0xE0;
    reg_val |= 0x0E;/*1500ma*/
    __ricoh619_reg_write(i2c, RICOH619_REGISET2, reg_val);
    printk("cljiang======================RICOH619_REGISET2 = 0x%x\n", __ricoh619_reg_read(i2c, RICOH619_REGISET2));

        /*Charge Current Setting*/
    reg_val = __ricoh619_reg_read(i2c, RICOH619_CHGISET); 
    printk("cljiang======================RICOH619_CHGISET = 0x%x\n", reg_val);
    reg_val &= 0xE0;
    reg_val |= 0x0B;/*1200ma*/
    __ricoh619_reg_write(i2c, RICOH619_CHGISET, reg_val);
    printk("cljiang======================RICOH619_CHGISET = 0x%x\n", __ricoh619_reg_read(i2c, RICOH619_CHGISET));


    /*power off press timer*/
    reg_val = __ricoh619_reg_read(i2c, RICOH619_PWRONTIMSET); 
    reg_val &= 0x8F;
    reg_val |= 0x50; /*8 sec*/
    __ricoh619_reg_write(i2c, RICOH619_PWRONTIMSET, reg_val);


    /*battery temperature measure*/
        /*enable VTHMSEL bit*/
    reg_val = __ricoh619_reg_read(i2c, RICOH619_ADCCNT1); 
    reg_val &= 0xDF;
    reg_val |= 0x20;
    __ricoh619_reg_write(i2c, RICOH619_ADCCNT1, reg_val);

        /*set ADRQ auto-mode*/
    reg_val = __ricoh619_reg_read(i2c, RICOH619_ADCCNT3); 
    reg_val &= 0xCF;
    reg_val |= 0x20; /*auto-mode*/
    __ricoh619_reg_write(i2c, RICOH619_ADCCNT3, reg_val);

     
    
    udelay(100000);
        /*read voltage*/
    printk("cljiang======================VTHMDATAH: %d, VTHMDATAL: %d\n", __ricoh619_reg_read(i2c, RICOH619_VTHMDATAH), 
                                                __ricoh619_reg_read(i2c, RICOH619_VTHMDATAL));
    printk("cljiang======================Resistance = %d\n", battery_ntc_R_val(__ricoh619_reg_read(i2c, RICOH619_VTHMDATAH)));
    printk("cljiang======================temperature = %d\n", temp_foreach_sheet(188600));

    /*charger insertion logic*/
    reg_val = __ricoh619_reg_read(i2c, RICOH619_PONHIS);
    printk("cljiang======================RICOH619_PONHIS = 0x%x\n", reg_val);
#if 0
    if(reg_val & 0x4){
        /*power off*/
        __ricoh619_reg_write(i2c, RICOH619_SLPCNT, 0x1);
    }
#endif

}
static int ricoh619_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct regmap *regmap = NULL;
	int ret;
printk("jimgao test ricoh619_i2c_probe=======IIIIIIIIIIIIIII=====\n");
    ricoh619_i2c = i2c;
    mutex_init(&ricoh61x_io_lock);

	ricoh619_reg_default_init(i2c);

	regmap = devm_regmap_init_i2c(i2c, &ricoh619_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&i2c->dev, "regmap init failed: %d\n", ret);
		return ret;
	}
	i2c_set_clientdata(i2c, regmap);

	ret = ricoh619_irq_init(i2c, regmap);
	if (ret)
		return ret;


	ret = mfd_add_devices(&i2c->dev, -1, ricoh619_cells,
			ARRAY_SIZE(ricoh619_cells), NULL, 0, NULL);
	if (ret) {
		dev_err(&i2c->dev, "failed to add sub-devices: %d\n", ret);
		goto out_irq_domain_remove;
	}

	dev_info(&i2c->dev, "%s success\n", __func__);

    config_xxzA926_info(i2c);


	return 0;

out_irq_domain_remove:
	ricoh619_irq_deinit();
	return ret;
}

static int ricoh619_i2c_remove(struct i2c_client *i2c)
{
	mfd_remove_devices(&i2c->dev);
	return 0;
}

static const struct of_device_id ricoh619_of_match[] = {
	{ .compatible = "ricoh,rn5t619" },
	{ }
};
MODULE_DEVICE_TABLE(of, ricoh619_of_match);

static const struct i2c_device_id rc5t619_i2c_id[] = {
	{.name = "rc5t619", .driver_data = 0},
	{}
};


static struct i2c_driver ricoh619_i2c_driver = {
	.driver = {
		.name = "ricoh619",
		.of_match_table = of_match_ptr(ricoh619_of_match),
	},
	.probe = ricoh619_i2c_probe,
	.remove = ricoh619_i2c_remove,
	.id_table = rc5t619_i2c_id,
};

static int __init ricoh619_i2c_init(void)
{
	int ret = -ENODEV;

	ret = i2c_add_driver(&ricoh619_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);
	return ret;
}
subsys_initcall(ricoh619_i2c_init);

static void __exit ricoh619_i2c_exit(void)
{
	i2c_del_driver(&ricoh619_i2c_driver);
}

module_exit(ricoh619_i2c_exit);

MODULE_DESCRIPTION("Ricoh RICOH619 MFD driver");
MODULE_LICENSE("GPL v2");
