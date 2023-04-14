#include <linux/fb.h>
#include <video/ingenic_logo.h>


static struct _logo_info *logo_early = NULL;

static int __init logo_info_early(char *p)
{
	unsigned int pinfo = 0;
	pinfo = simple_strtoul(p, NULL, 0);
	logo_early = (struct _logo_info *)pinfo;
	logo_early->p8 = (unsigned char *)(pinfo + sizeof(struct _logo_info));
        return 0;
}
early_param("logo", logo_info_early);



void show_logo(struct fb_info *info)
{
	unsigned int i, j;
	int w, h, bpp;
	int blank_left, blank_top;
	void *buf;
	unsigned int *p32;
	unsigned int *p_logo;
	struct fb_var_screeninfo *var = &info->var;
	unsigned char *screen_base = info->screen_base;
	struct _logo_info *pinfo = &logo_info;	//Kernel Built-in logo.

	if(logo_early) {
		pinfo = logo_early;
	}

	w = var->xres;
	h = var->yres;
	bpp = info->var.bits_per_pixel;
	if ((w < pinfo->width) || (h < pinfo->height)) {
		printk("\033[33mERROR: LOGO shouldn't bigger than panel! skip.\033[0m\n");
		return;
	}

	buf = screen_base;
	if (pinfo->bpp == bpp) {
		blank_left = (w - pinfo->width) / 2;
		blank_top = (h - pinfo->height) / 2;
		// printk("\033[31m w = %d | pinfo->width = %d |\033[0m\n", w, pinfo->width);
		// printk("\033[31m blank_left = %d | blank_top = %d |\033[0m\n", blank_left, blank_top);
		if (bpp == 32) {
			if (blank_top) {
				p32 = (unsigned int *)buf;
				for (i = 0; i < blank_top * w; ++i) {
					*(p32 + i) = pinfo->background_color;
				}
				p32 = (unsigned int *)buf + (blank_top + pinfo->height) * w;
				for (i = 0; i < (h - pinfo->height - blank_top) * w; ++i) {
					*(p32 + i) = pinfo->background_color;
				}
			}
			if (blank_left) {
				for (i = 0; i < pinfo->height; ++i) {
					p32 = (unsigned int *)buf + (blank_top + i) * w;
					for (j = 0; j < blank_left; ++j) {
						*(p32 + j) = pinfo->background_color;
					}
					p32 = (unsigned int *)buf + (blank_top + i) * w + pinfo->width + blank_left;
					for (j = 0; j < w - pinfo->width - blank_left; ++j) {
						*(p32 + j) = pinfo->background_color;
					}
				}
			}
			for (i = 0; i < pinfo->height; ++i) {
				p32 = (unsigned int *)buf + (blank_top + i) * w + blank_left;
				p_logo = (unsigned int*)pinfo->p8 + i * pinfo->width;
				for (j = 0; j < pinfo->width; ++j) {
					*(p32 + j) = *(p_logo + j);
				}
			}
		} else if (bpp == 16) {
			unsigned int background_color = pinfo->background_color << 16 | pinfo->background_color;

			printk("\033[31m h = %d | blank_top = %d | pinfo->height = %d |\033[0m\n", h, blank_top, pinfo->height);
			if (blank_top) {
				int tmp = blank_top * ((w + 1) / 2);
				p32 = (unsigned int *)buf;
				for (i = 0; i < tmp; i++)
					*(p32++) = background_color;

				tmp = (h - blank_top - pinfo->height) * ((w + 1) / 2);
				p32 = (unsigned int *)buf + (blank_top + pinfo->height) * ((w + 1) / 2);
				for (i = 0; i < tmp; i++)
					*(p32++) = background_color;
			}

			if (blank_left) {
				int tmp;
				for (i = 0; i < pinfo->height; ++i) {
					tmp = (blank_left + 1) / 2;
					p32 = (unsigned int *)buf + (blank_top + i) * ((w + 1) / 2);
					for (j = 0; j < tmp; ++j) {
						*(p32 + j) = background_color;
					}

					p32 = (unsigned int *)buf + (blank_top + i) * ((w + 1) / 2) + (pinfo->width + blank_left) / 2;
					for (j = 0; j < tmp; ++j) {
						*(p32 + j) = background_color;
					}
				}
			}
			if (pinfo->width%2) {
				for (i = 0; i < pinfo->height; ++i) {
					p32 = (unsigned int *)buf + (blank_top + i) * ((w + 1) / 2) + blank_left / 2;
					p_logo = (unsigned int*)pinfo->p8 + i * pinfo->width/2;
					for (j = 0; j < pinfo->width/2; ++j) {
						*(p32 + j) = *(p_logo + j);
					}
					*(unsigned short*)(p32 + j) = *(unsigned short*)(p_logo + j);
				}
			} else {
				for (i = 0; i < pinfo->height; ++i) {
					p32 = (unsigned int *)buf + (blank_top + i) * ((w + 1) / 2) + blank_left / 2;
					p_logo = (unsigned int*)pinfo->p8 + i * pinfo->width/2;
					for (j = 0; j < pinfo->width/2; ++j) {
						*(p32 + j) = *(p_logo + j);
					}
				}
			}
		}
	}
}
EXPORT_SYMBOL(show_logo);
EXPORT_SYMBOL(logo_info);
EXPORT_SYMBOL(logo_buf_initdata);
