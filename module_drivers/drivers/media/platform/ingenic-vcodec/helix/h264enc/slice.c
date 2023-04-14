#include "cabac.h"
#include "slice.h"

#ifdef __KERNEL__
#include <linux/printk.h>

#define printf printk
#endif



void xdump_slice_header(h264_slice_header_t *sh)
{
	printf("sh->i_type			: %d\n", sh->i_type);
	printf("sh->i_first_mb			: %d\n", sh->i_first_mb);
	printf("sh->i_last_mb			: %d\n", sh->i_last_mb);
	printf("sh->i_pps_id			: %d\n", sh->i_pps_id);
	printf("sh->i_frame_num			: %d\n", sh->i_frame_num);
	printf("sh->b_mbaff			: %d\n", sh->b_mbaff);
	printf("sh->b_field_pic			: %d\n", sh->b_field_pic);
	printf("sh->b_bottom_field		: %d\n", sh->b_bottom_field);
	printf("sh->i_idr_pic_id		: %d\n", sh->i_idr_pic_id);
	printf("sh->i_poc			: %d\n", sh->i_poc);
	printf("sh->i_delta_poc_bottom		: %d\n", sh->i_delta_poc_bottom);
	printf("sh->i_delta_poc[0]:		: %d\n", sh->i_delta_poc[0]);
	printf("sh->i_delta_poc[1]:		: %d\n", sh->i_delta_poc[1]);
	printf("sh->b_direct_spatial_mv_pred	: %d\n", sh->b_direct_spatial_mv_pred);
	printf("sh->b_num_ref_idx_override	: %d\n", sh->b_num_ref_idx_override);
	printf("sh->i_num_ref_idx_l0_active	: %d\n", sh->i_num_ref_idx_l0_active);
	printf("sh->i_num_ref_idx_l1_active	: %d\n", sh->i_num_ref_idx_l1_active);


	printf("sh->i_cabac_init_idc		: %d\n", sh->i_cabac_init_idc);
	printf("sh->i_qp			: %d\n", sh->i_qp);
	printf("sh->i_qp_delta			: %d\n", sh->i_qp_delta);
	printf("sh->b_sp_for_swidth		: %d\n", sh->b_sp_for_swidth);
	printf("sh->i_qs_delta			: %d\n", sh->i_qs_delta);


}

void h264e_slice_header_init(h264_slice_header_t *sh,
				h264_sps_t *sps, h264_pps_t *pps,
				int i_idr_pic_id, int i_frame, int i_qp)
{
	sh->sps = sps;
	sh->pps = pps;

	sh->i_first_mb = 0;
	sh->i_last_mb = sps->i_mb_width * sps->i_mb_height - 1;
	sh->i_pps_id = pps->i_id;
	sh->i_frame_num = i_frame;

	sh->b_mbaff = 0;
	sh->b_field_pic = 0;
	sh->b_bottom_field = 0;

	sh->i_idr_pic_id = i_idr_pic_id;

	sh->i_poc = 0;
	sh->i_delta_poc_bottom = 0;
	sh->i_delta_poc[0] = 0;
	sh->i_delta_poc[1] = 0;


	sh->i_redundant_pic_cnt = 0;

	sh->b_num_ref_idx_override = 0;
	sh->i_num_ref_idx_l0_active = 1;
	sh->i_num_ref_idx_l1_active = 1;

	sh->b_ref_pic_list_reordering[0] = 0;
	sh->b_ref_pic_list_reordering[1] = 0;


	sh->i_cabac_init_idc = 0;

	sh->i_qp = SPEC_QP(i_qp);
	sh->i_qp_delta = sh->i_qp - pps->i_pic_init_qp;

	sh->b_sp_for_swidth = 0;
	sh->i_qs_delta = 0;

	sh->i_disable_deblocking_filter_idc = 1;
	sh->i_alpha_c0_offset = 0;
	sh->i_beta_offset = 0;
}


void h264e_slice_header_write(bs_t *s, h264_slice_header_t *sh, int i_nal_ref_idc)
{
	/*Do not support mbaff*/
	bs_write_ue(s, sh->i_first_mb);
	bs_write_ue(s, sh->i_type + 5);
	bs_write_ue(s, sh->i_pps_id);
	bs_write(s, sh->sps->i_log2_max_frame_num, sh->i_frame_num & ((1 << sh->sps->i_log2_max_frame_num) - 1));

	/*mbs only*/

	if(sh->i_idr_pic_id >= 0)
		bs_write_ue(s, sh->i_idr_pic_id);


	if(sh->i_type == M_SLICE_TYPE_P || sh->i_type == M_SLICE_TYPE_B) {
		bs_write1(s, sh->b_num_ref_idx_override);
		if(sh->b_num_ref_idx_override) {
			bs_write_ue(s, sh->i_num_ref_idx_l0_active - 1);
			if(sh->i_type == M_SLICE_TYPE_B)
				bs_write_ue(s, sh->i_num_ref_idx_l1_active - 1);
		}

	}

	if(sh->i_type != M_SLICE_TYPE_I) {
		bs_write1(s, sh->b_ref_pic_list_reordering[0]);
		if(sh->b_ref_pic_list_reordering[0]) {
			int i;
			for(i = 0; i < sh->i_num_ref_idx_l0_active; i++) {
				bs_write_ue(s, sh->ref_pic_list_order[0][i].idc);
				bs_write_ue(s, sh->ref_pic_list_order[0][i].arg);
			}

			bs_write_ue(s, 3);
		}
	}

	if(i_nal_ref_idc != 0) {
		if(sh->i_idr_pic_id >= 0) {
			bs_write1(s, 0);
			bs_write1(s, 0);
		} else {
			bs_write1( s, sh->i_mmco_command_count > 0 ); /* adaptive_ref_pic_marking_mode_flag */
			if( sh->i_mmco_command_count > 0 )
			{
				int i = 0;
				for(i = 0; i < sh->i_mmco_command_count; i++ )
				{
					bs_write_ue( s, 1 ); /* mark short term ref as unused */
					bs_write_ue( s, sh->mmco[i].i_difference_of_pic_nums - 1 );
				}
				bs_write_ue( s, 0 ); /* end command list */
			}

		}

	}

	if( sh->pps->b_cabac && sh->i_type != M_SLICE_TYPE_I )
		bs_write_ue( s, sh->i_cabac_init_idc );

	bs_write_se( s, sh->i_qp_delta );      /* slice qp delta */

	if( sh->pps->b_deblocking_filter_control )
	{
		bs_write_ue( s, sh->i_disable_deblocking_filter_idc );
		if( sh->i_disable_deblocking_filter_idc != 1 )
		{
			bs_write_se( s, sh->i_alpha_c0_offset >> 1 );
			bs_write_se( s, sh->i_beta_offset >> 1 );
		}
	}
}
