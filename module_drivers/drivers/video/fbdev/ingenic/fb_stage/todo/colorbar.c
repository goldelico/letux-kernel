
static void __maybe_unused ingenicfb_display_sread_v_color_bar(struct fb_info *info)
{
	int i, j;
	int w, h;
	int bpp;
	unsigned short *p16;
	unsigned int *p32;
	struct ingenicfb_device *fbdev = info->par;
	struct fb_videomode *mode = fbdev->panel->modes;


	if (!mode) {
		dev_err(fbdev->dev, "%s, video mode is NULL\n", __func__);
		return;
	}

	p16 = (unsigned short *)fbdev->sread_vidmem[0];
	p32 = (unsigned int *)fbdev->sread_vidmem[0];
	w = mode->xres;
	h = mode->yres;
	bpp = info->var.bits_per_pixel;


	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) {
			short c16;
			int c32 = 0;
			switch ((j / 10) % 4) {
				case 0:
					c16 = 0xF800;
					c32 = 0xFFFF0000;
					break;
				case 1:
					c16 = 0x07C0;
					c32 = 0xFF00FF00;
					break;
				case 2:
					c16 = 0x001F;
					c32 = 0xFF0000FF;
					break;
				default:
					c16 = 0xFFFF;
					c32 = 0xFFFFFFFF;
					break;
			}
			switch (bpp) {
				case 18:
				case 24:
				case 32:
					*p32++ = c32;
					break;
				default:
					*p16++ = c16;
			}
		}
		if (w % PIXEL_ALIGN) {
			switch (bpp) {
				case 18:
				case 24:
				case 32:
					p32 += (ALIGN(mode->xres, PIXEL_ALIGN) - w);
					break;
				default:
					p16 += (ALIGN(mode->xres, PIXEL_ALIGN) - w);
					break;
			}
		}
	}
}

static void ingenicfb_display_v_color_bar(struct fb_info *info)
{
	int i, j;
	int w, h;
	int bpp;
	unsigned short *p16;
	unsigned int *p32;
	struct ingenicfb_device *fbdev = info->par;
	struct fb_videomode *mode = fbdev->panel->modes;

	if (!mode) {
		dev_err(fbdev->dev, "%s, video mode is NULL\n", __func__);
		return;
	}
	if (!fbdev->vidmem_phys[fbdev->current_frm_desc][0]) {
		dev_err(fbdev->dev, "Not allocate frame buffer yet\n");
		return;
	}
	if (!fbdev->vidmem[fbdev->current_frm_desc][0])
		fbdev->vidmem[fbdev->current_frm_desc][0] =
			(void *)phys_to_virt(fbdev->vidmem_phys[fbdev->current_frm_desc][0]);
	p16 = (unsigned short *)fbdev->vidmem[fbdev->current_frm_desc][0];
	p32 = (unsigned int *)fbdev->vidmem[fbdev->current_frm_desc][0];
	w = mode->xres;
	h = mode->yres;
	bpp = info->var.bits_per_pixel;

	dev_info(info->dev,
			"LCD V COLOR BAR w,h,bpp(%d,%d,%d) fbdev->vidmem[0]=%p\n", w, h,
			bpp, fbdev->vidmem[fbdev->current_frm_desc][0]);

	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) {
			short c16;
			int c32 = 0;
			switch ((j / 10) % 4) {
				case 0:
					c16 = 0xF800;
					c32 = 0xFFFF0000;
					break;
				case 1:
					c16 = 0x07C0;
					c32 = 0xFF00FF00;
					break;
				case 2:
					c16 = 0x001F;
					c32 = 0xFF0000FF;
					break;
				default:
					c16 = 0xFFFF;
					c32 = 0xFFFFFFFF;
					break;
			}
			switch (bpp) {
				case 18:
				case 24:
				case 32:
					*p32++ = c32;
					break;
				default:
					*p16++ = c16;
			}
		}
		if (w % PIXEL_ALIGN) {
			switch (bpp) {
				case 18:
				case 24:
				case 32:
					p32 += (ALIGN(mode->xres, PIXEL_ALIGN) - w);
					break;
				default:
					p16 += (ALIGN(mode->xres, PIXEL_ALIGN) - w);
					break;
			}
		}
	}
}

static void ingenicfb_display_h_color_bar(struct fb_info *info)
{
	int i, j;
	int w, h;
	int bpp;
	unsigned short *p16;
	unsigned int *p32;
	struct ingenicfb_device *fbdev = info->par;
	struct fb_videomode *mode = fbdev->panel->modes;

	if (!mode) {
		dev_err(fbdev->dev, "%s, video mode is NULL\n", __func__);
		return;
	}
	if (!fbdev->vidmem_phys[fbdev->current_frm_desc][0]) {
		dev_err(fbdev->dev, "Not allocate frame buffer yet\n");
		return;
	}
	if (!fbdev->vidmem[fbdev->current_frm_desc][0])
		fbdev->vidmem[fbdev->current_frm_desc][0] =
			(void *)phys_to_virt(fbdev->vidmem_phys[fbdev->current_frm_desc][0]);
	p16 = (unsigned short *)fbdev->vidmem[fbdev->current_frm_desc][0];
	p32 = (unsigned int *)fbdev->vidmem[fbdev->current_frm_desc][0];
	w = mode->xres;
	h = mode->yres;
	bpp = info->var.bits_per_pixel;

	dev_info(info->dev,
			"LCD H COLOR BAR w,h,bpp(%d,%d,%d), fbdev->vidmem[0]=%p\n", w, h,
			bpp, fbdev->vidmem[fbdev->current_frm_desc][0]);

	for (i = 0; i < h; i++) {
		for (j = 0; j < w; j++) {
			short c16;
			int c32;
			switch ((i / 10) % 4) {
				case 0:
					c16 = 0xF800;
					c32 = 0x00FF0000;
					break;
				case 1:
					c16 = 0x07C0;
					c32 = 0x0000FF00;
					break;
				case 2:
					c16 = 0x001F;
					c32 = 0x000000FF;
					break;
				default:
					c16 = 0xFFFF;
					c32 = 0xFFFFFFFF;
					break;
			}
			switch (bpp) {
				case 18:
				case 24:
				case 32:
					*p32++ = c32;
					break;
				default:
					*p16++ = c16;
			}
		}
		if (w % PIXEL_ALIGN) {
			switch (bpp) {
				case 18:
				case 24:
				case 32:
					p32 += (ALIGN(mode->xres, PIXEL_ALIGN) - w);
					break;
				default:
					p16 += (ALIGN(mode->xres, PIXEL_ALIGN) - w);
					break;
			}
		}
	}
}

int test_pattern(struct ingenicfb_device *fbdev)
{
	int ret;

	ingenicfb_disable(fbdev->fb, QCK_STOP);
	ingenicfb_display_h_color_bar(fbdev->fb);
	//ingenicfb_display_sread_v_color_bar(fbdev->fb);
	fbdev->current_frm_desc = 0;
	ingenicfb_set_fix_par(fbdev->fb);
	ret = ingenicfb_set_par(fbdev->fb);
	if(ret) {
		dev_err(fbdev->dev, "Set par failed!\n");
		return ret;
	}
	ingenicfb_enable(fbdev->fb);

	return 0;
}


