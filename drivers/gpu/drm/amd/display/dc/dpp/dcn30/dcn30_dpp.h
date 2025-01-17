/* Copyright 2020 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: AMD
 *
 */

#ifndef __DCN30_DPP_H__
#define __DCN30_DPP_H__

#include "dcn20/dcn20_dpp.h"

#define TO_DCN30_DPP(dpp)\
	container_of(dpp, struct dcn3_dpp, base)

#define DPP_REG_LIST_DCN30_COMMON(id)\
	SRI(CM_DEALPHA, CM, id),\
	SRI(CM_MEM_PWR_STATUS, CM, id),\
	SRI(CM_BIAS_CR_R, CM, id),\
	SRI(CM_BIAS_Y_G_CB_B, CM, id),\
	SRI(PRE_DEGAM, CNVC_CFG, id),\
	SRI(CM_GAMCOR_CONTROL, CM, id),\
	SRI(CM_GAMCOR_LUT_CONTROL, CM, id),\
	SRI(CM_GAMCOR_LUT_INDEX, CM, id),\
	SRI(CM_GAMCOR_LUT_INDEX, CM, id),\
	SRI(CM_GAMCOR_LUT_DATA, CM, id),\
	SRI(CM_GAMCOR_RAMB_START_CNTL_B, CM, id),\
	SRI(CM_GAMCOR_RAMB_START_CNTL_G, CM, id),\
	SRI(CM_GAMCOR_RAMB_START_CNTL_R, CM, id),\
	SRI(CM_GAMCOR_RAMB_START_SLOPE_CNTL_B, CM, id),\
	SRI(CM_GAMCOR_RAMB_START_SLOPE_CNTL_G, CM, id),\
	SRI(CM_GAMCOR_RAMB_START_SLOPE_CNTL_R, CM, id),\
	SRI(CM_GAMCOR_RAMB_END_CNTL1_B, CM, id),\
	SRI(CM_GAMCOR_RAMB_END_CNTL2_B, CM, id),\
	SRI(CM_GAMCOR_RAMB_END_CNTL1_G, CM, id),\
	SRI(CM_GAMCOR_RAMB_END_CNTL2_G, CM, id),\
	SRI(CM_GAMCOR_RAMB_END_CNTL1_R, CM, id),\
	SRI(CM_GAMCOR_RAMB_END_CNTL2_R, CM, id),\
	SRI(CM_GAMCOR_RAMB_REGION_0_1, CM, id),\
	SRI(CM_GAMCOR_RAMB_REGION_32_33, CM, id),\
	SRI(CM_GAMCOR_RAMB_OFFSET_B, CM, id),\
	SRI(CM_GAMCOR_RAMB_OFFSET_G, CM, id),\
	SRI(CM_GAMCOR_RAMB_OFFSET_R, CM, id),\
	SRI(CM_GAMCOR_RAMB_START_BASE_CNTL_B, CM, id),\
	SRI(CM_GAMCOR_RAMB_START_BASE_CNTL_G, CM, id),\
	SRI(CM_GAMCOR_RAMB_START_BASE_CNTL_R, CM, id),\
	SRI(CM_GAMCOR_RAMA_START_CNTL_B, CM, id),\
	SRI(CM_GAMCOR_RAMA_START_CNTL_G, CM, id),\
	SRI(CM_GAMCOR_RAMA_START_CNTL_R, CM, id),\
	SRI(CM_GAMCOR_RAMA_START_SLOPE_CNTL_B, CM, id),\
	SRI(CM_GAMCOR_RAMA_START_SLOPE_CNTL_G, CM, id),\
	SRI(CM_GAMCOR_RAMA_START_SLOPE_CNTL_R, CM, id),\
	SRI(CM_GAMCOR_RAMA_END_CNTL1_B, CM, id),\
	SRI(CM_GAMCOR_RAMA_END_CNTL2_B, CM, id),\
	SRI(CM_GAMCOR_RAMA_END_CNTL1_G, CM, id),\
	SRI(CM_GAMCOR_RAMA_END_CNTL2_G, CM, id),\
	SRI(CM_GAMCOR_RAMA_END_CNTL1_R, CM, id),\
	SRI(CM_GAMCOR_RAMA_END_CNTL2_R, CM, id),\
	SRI(CM_GAMCOR_RAMA_REGION_0_1, CM, id),\
	SRI(CM_GAMCOR_RAMA_REGION_32_33, CM, id),\
	SRI(CM_GAMCOR_RAMA_OFFSET_B, CM, id),\
	SRI(CM_GAMCOR_RAMA_OFFSET_G, CM, id),\
	SRI(CM_GAMCOR_RAMA_OFFSET_R, CM, id),\
	SRI(CM_GAMCOR_RAMA_START_BASE_CNTL_B, CM, id),\
	SRI(CM_GAMCOR_RAMA_START_BASE_CNTL_G, CM, id),\
	SRI(CM_GAMCOR_RAMA_START_BASE_CNTL_R, CM, id),\
	SRI(CM_GAMUT_REMAP_CONTROL, CM, id),\
	SRI(CM_GAMUT_REMAP_C11_C12, CM, id),\
	SRI(CM_GAMUT_REMAP_C13_C14, CM, id),\
	SRI(CM_GAMUT_REMAP_C21_C22, CM, id),\
	SRI(CM_GAMUT_REMAP_C23_C24, CM, id),\
	SRI(CM_GAMUT_REMAP_C31_C32, CM, id),\
	SRI(CM_GAMUT_REMAP_C33_C34, CM, id),\
	SRI(CM_GAMUT_REMAP_B_C11_C12, CM, id),\
	SRI(CM_GAMUT_REMAP_B_C13_C14, CM, id),\
	SRI(CM_GAMUT_REMAP_B_C21_C22, CM, id),\
	SRI(CM_GAMUT_REMAP_B_C23_C24, CM, id),\
	SRI(CM_GAMUT_REMAP_B_C31_C32, CM, id),\
	SRI(CM_GAMUT_REMAP_B_C33_C34, CM, id),\
	SRI(DSCL_EXT_OVERSCAN_LEFT_RIGHT, DSCL, id), \
	SRI(DSCL_EXT_OVERSCAN_TOP_BOTTOM, DSCL, id), \
	SRI(OTG_H_BLANK, DSCL, id), \
	SRI(OTG_V_BLANK, DSCL, id), \
	SRI(SCL_MODE, DSCL, id), \
	SRI(LB_DATA_FORMAT, DSCL, id), \
	SRI(LB_MEMORY_CTRL, DSCL, id), \
	SRI(DSCL_AUTOCAL, DSCL, id), \
	SRI(DSCL_CONTROL, DSCL, id), \
	SRI(SCL_TAP_CONTROL, DSCL, id), \
	SRI(SCL_COEF_RAM_TAP_SELECT, DSCL, id), \
	SRI(SCL_COEF_RAM_TAP_DATA, DSCL, id), \
	SRI(DSCL_2TAP_CONTROL, DSCL, id), \
	SRI(MPC_SIZE, DSCL, id), \
	SRI(SCL_HORZ_FILTER_SCALE_RATIO, DSCL, id), \
	SRI(SCL_VERT_FILTER_SCALE_RATIO, DSCL, id), \
	SRI(SCL_HORZ_FILTER_SCALE_RATIO_C, DSCL, id), \
	SRI(SCL_VERT_FILTER_SCALE_RATIO_C, DSCL, id), \
	SRI(SCL_HORZ_FILTER_INIT, DSCL, id), \
	SRI(SCL_HORZ_FILTER_INIT_C, DSCL, id), \
	SRI(SCL_VERT_FILTER_INIT, DSCL, id), \
	SRI(SCL_VERT_FILTER_INIT_C, DSCL, id), \
	SRI(RECOUT_START, DSCL, id), \
	SRI(RECOUT_SIZE, DSCL, id), \
	SRI(PRE_DEALPHA, CNVC_CFG, id), \
	SRI(PRE_REALPHA, CNVC_CFG, id), \
	SRI(PRE_CSC_MODE, CNVC_CFG, id), \
	SRI(PRE_CSC_C11_C12, CNVC_CFG, id), \
	SRI(PRE_CSC_C33_C34, CNVC_CFG, id), \
	SRI(PRE_CSC_B_C11_C12, CNVC_CFG, id), \
	SRI(PRE_CSC_B_C33_C34, CNVC_CFG, id), \
	SRI(CM_POST_CSC_CONTROL, CM, id), \
	SRI(CM_POST_CSC_C11_C12, CM, id), \
	SRI(CM_POST_CSC_C33_C34, CM, id), \
	SRI(CM_POST_CSC_B_C11_C12, CM, id), \
	SRI(CM_POST_CSC_B_C33_C34, CM, id), \
	SRI(CM_MEM_PWR_CTRL, CM, id), \
	SRI(CM_CONTROL, CM, id), \
	SRI(CM_TEST_DEBUG_INDEX, CM, id), \
	SRI(CM_TEST_DEBUG_DATA, CM, id), \
	SRI(FORMAT_CONTROL, CNVC_CFG, id), \
	SRI(CNVC_SURFACE_PIXEL_FORMAT, CNVC_CFG, id), \
	SRI(CURSOR0_CONTROL, CNVC_CUR, id), \
	SRI(CURSOR0_COLOR0, CNVC_CUR, id), \
	SRI(CURSOR0_COLOR1, CNVC_CUR, id), \
	SRI(CURSOR0_FP_SCALE_BIAS, CNVC_CUR, id), \
	SRI(DPP_CONTROL, DPP_TOP, id), \
	SRI(CM_HDR_MULT_COEF, CM, id), \
	SRI(CURSOR_CONTROL, CURSOR0_, id), \
	SRI(ALPHA_2BIT_LUT, CNVC_CFG, id), \
	SRI(FCNV_FP_BIAS_R, CNVC_CFG, id), \
	SRI(FCNV_FP_BIAS_G, CNVC_CFG, id), \
	SRI(FCNV_FP_BIAS_B, CNVC_CFG, id), \
	SRI(FCNV_FP_SCALE_R, CNVC_CFG, id), \
	SRI(FCNV_FP_SCALE_G, CNVC_CFG, id), \
	SRI(FCNV_FP_SCALE_B, CNVC_CFG, id), \
	SRI(COLOR_KEYER_CONTROL, CNVC_CFG, id), \
	SRI(COLOR_KEYER_ALPHA, CNVC_CFG, id), \
	SRI(COLOR_KEYER_RED, CNVC_CFG, id), \
	SRI(COLOR_KEYER_GREEN, CNVC_CFG, id), \
	SRI(COLOR_KEYER_BLUE, CNVC_CFG, id), \
	SRI(CURSOR_CONTROL, CURSOR0_, id),\
	SRI(OBUF_MEM_PWR_CTRL, DSCL, id),\
	SRI(DSCL_MEM_PWR_STATUS, DSCL, id), \
	SRI(DSCL_MEM_PWR_CTRL, DSCL, id)

#define DPP_REG_LIST_DCN30(id)\
	DPP_REG_LIST_DCN30_COMMON(id), \
	TF_REG_LIST_DCN20_COMMON(id), \
	SRI(CM_BLNDGAM_CONTROL, CM, id), \
	SRI(CM_SHAPER_LUT_DATA, CM, id),\
	SRI(CM_MEM_PWR_CTRL2, CM, id), \
	SRI(CM_MEM_PWR_STATUS2, CM, id), \
	SRI(CM_BLNDGAM_RAMA_START_SLOPE_CNTL_B, CM, id),\
	SRI(CM_BLNDGAM_RAMA_START_SLOPE_CNTL_G, CM, id),\
	SRI(CM_BLNDGAM_RAMA_START_SLOPE_CNTL_R, CM, id),\
	SRI(CM_BLNDGAM_RAMB_START_SLOPE_CNTL_B, CM, id),\
	SRI(CM_BLNDGAM_RAMB_START_SLOPE_CNTL_G, CM, id),\
	SRI(CM_BLNDGAM_RAMB_START_SLOPE_CNTL_R, CM, id),\
	SRI(CM_BLNDGAM_LUT_CONTROL, CM, id)

#define DPP_REG_LIST_SH_MASK_DCN30_COMMON(mask_sh)\
	TF_SF(CM0_CM_MEM_PWR_STATUS, GAMCOR_MEM_PWR_STATE, mask_sh),\
	TF_SF(CM0_CM_DEALPHA, CM_DEALPHA_EN, mask_sh),\
	TF_SF(CM0_CM_DEALPHA, CM_DEALPHA_ABLND, mask_sh),\
	TF_SF(CM0_CM_BIAS_CR_R, CM_BIAS_CR_R, mask_sh),\
	TF_SF(CM0_CM_BIAS_Y_G_CB_B, CM_BIAS_Y_G, mask_sh),\
	TF_SF(CM0_CM_BIAS_Y_G_CB_B, CM_BIAS_CB_B, mask_sh),\
	TF_SF(CM0_CM_MEM_PWR_CTRL, GAMCOR_MEM_PWR_DIS, mask_sh),\
	TF_SF(CM0_CM_MEM_PWR_CTRL, GAMCOR_MEM_PWR_FORCE, mask_sh),\
	TF_SF(CNVC_CFG0_PRE_DEGAM, PRE_DEGAM_MODE, mask_sh),\
	TF_SF(CNVC_CFG0_PRE_DEGAM, PRE_DEGAM_SELECT, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_CONTROL, CM_GAMCOR_MODE, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_CONTROL, CM_GAMCOR_SELECT, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_CONTROL, CM_GAMCOR_PWL_DISABLE, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_CONTROL, CM_GAMCOR_MODE_CURRENT, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_CONTROL, CM_GAMCOR_SELECT_CURRENT, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_LUT_INDEX, CM_GAMCOR_LUT_INDEX, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_LUT_DATA, CM_GAMCOR_LUT_DATA, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_LUT_CONTROL, CM_GAMCOR_LUT_WRITE_COLOR_MASK, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_LUT_CONTROL, CM_GAMCOR_LUT_READ_COLOR_SEL, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_LUT_CONTROL, CM_GAMCOR_LUT_HOST_SEL, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_LUT_CONTROL, CM_GAMCOR_LUT_CONFIG_MODE, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_START_CNTL_B, CM_GAMCOR_RAMA_EXP_REGION_START_B, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_START_CNTL_B, CM_GAMCOR_RAMA_EXP_REGION_START_SEGMENT_B, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_START_SLOPE_CNTL_B, CM_GAMCOR_RAMA_EXP_REGION_START_SLOPE_B, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_START_BASE_CNTL_B, CM_GAMCOR_RAMA_EXP_REGION_START_BASE_B, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_END_CNTL1_B, CM_GAMCOR_RAMA_EXP_REGION_END_BASE_B, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_END_CNTL2_B, CM_GAMCOR_RAMA_EXP_REGION_END_B, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_END_CNTL2_B, CM_GAMCOR_RAMA_EXP_REGION_END_SLOPE_B, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_OFFSET_B, CM_GAMCOR_RAMA_OFFSET_B, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_REGION_0_1, CM_GAMCOR_RAMA_EXP_REGION0_LUT_OFFSET, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_REGION_0_1, CM_GAMCOR_RAMA_EXP_REGION0_NUM_SEGMENTS, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_REGION_0_1, CM_GAMCOR_RAMA_EXP_REGION1_LUT_OFFSET, mask_sh),\
	TF_SF(CM0_CM_GAMCOR_RAMA_REGION_0_1, CM_GAMCOR_RAMA_EXP_REGION1_NUM_SEGMENTS, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_CONTROL, CM_GAMUT_REMAP_MODE, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_CONTROL, CM_GAMUT_REMAP_MODE_CURRENT, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C11_C12, CM_GAMUT_REMAP_C11, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C11_C12, CM_GAMUT_REMAP_C12, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C13_C14, CM_GAMUT_REMAP_C13, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C13_C14, CM_GAMUT_REMAP_C14, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C21_C22, CM_GAMUT_REMAP_C21, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C21_C22, CM_GAMUT_REMAP_C22, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C23_C24, CM_GAMUT_REMAP_C23, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C23_C24, CM_GAMUT_REMAP_C24, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C31_C32, CM_GAMUT_REMAP_C31, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C31_C32, CM_GAMUT_REMAP_C32, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C33_C34, CM_GAMUT_REMAP_C33, mask_sh),\
	TF_SF(CM0_CM_GAMUT_REMAP_C33_C34, CM_GAMUT_REMAP_C34, mask_sh),\
	TF_SF(DSCL0_DSCL_EXT_OVERSCAN_LEFT_RIGHT, EXT_OVERSCAN_LEFT, mask_sh),\
	TF_SF(DSCL0_DSCL_EXT_OVERSCAN_LEFT_RIGHT, EXT_OVERSCAN_RIGHT, mask_sh),\
	TF_SF(DSCL0_DSCL_EXT_OVERSCAN_TOP_BOTTOM, EXT_OVERSCAN_BOTTOM, mask_sh),\
	TF_SF(DSCL0_DSCL_EXT_OVERSCAN_TOP_BOTTOM, EXT_OVERSCAN_TOP, mask_sh),\
	TF_SF(DSCL0_OTG_H_BLANK, OTG_H_BLANK_START, mask_sh),\
	TF_SF(DSCL0_OTG_H_BLANK, OTG_H_BLANK_END, mask_sh),\
	TF_SF(DSCL0_OTG_V_BLANK, OTG_V_BLANK_START, mask_sh),\
	TF_SF(DSCL0_OTG_V_BLANK, OTG_V_BLANK_END, mask_sh),\
	TF_SF(DSCL0_LB_DATA_FORMAT, INTERLEAVE_EN, mask_sh),\
	TF2_SF(DSCL0, LB_DATA_FORMAT__ALPHA_EN, mask_sh),\
	TF_SF(DSCL0_LB_MEMORY_CTRL, MEMORY_CONFIG, mask_sh),\
	TF_SF(DSCL0_LB_MEMORY_CTRL, LB_MAX_PARTITIONS, mask_sh),\
	TF_SF(DSCL0_DSCL_AUTOCAL, AUTOCAL_MODE, mask_sh),\
	TF_SF(DSCL0_DSCL_AUTOCAL, AUTOCAL_NUM_PIPE, mask_sh),\
	TF_SF(DSCL0_DSCL_CONTROL, SCL_BOUNDARY_MODE, mask_sh),\
	TF_SF(DSCL0_DSCL_AUTOCAL, AUTOCAL_PIPE_ID, mask_sh),\
	TF_SF(DSCL0_SCL_TAP_CONTROL, SCL_V_NUM_TAPS, mask_sh),\
	TF_SF(DSCL0_SCL_TAP_CONTROL, SCL_H_NUM_TAPS, mask_sh),\
	TF_SF(DSCL0_SCL_TAP_CONTROL, SCL_V_NUM_TAPS_C, mask_sh),\
	TF_SF(DSCL0_SCL_TAP_CONTROL, SCL_H_NUM_TAPS_C, mask_sh),\
	TF_SF(DSCL0_SCL_COEF_RAM_TAP_SELECT, SCL_COEF_RAM_TAP_PAIR_IDX, mask_sh),\
	TF_SF(DSCL0_SCL_COEF_RAM_TAP_SELECT, SCL_COEF_RAM_PHASE, mask_sh),\
	TF_SF(DSCL0_SCL_COEF_RAM_TAP_SELECT, SCL_COEF_RAM_FILTER_TYPE, mask_sh),\
	TF_SF(DSCL0_SCL_COEF_RAM_TAP_DATA, SCL_COEF_RAM_EVEN_TAP_COEF, mask_sh),\
	TF_SF(DSCL0_SCL_COEF_RAM_TAP_DATA, SCL_COEF_RAM_EVEN_TAP_COEF_EN, mask_sh),\
	TF_SF(DSCL0_SCL_COEF_RAM_TAP_DATA, SCL_COEF_RAM_ODD_TAP_COEF, mask_sh),\
	TF_SF(DSCL0_SCL_COEF_RAM_TAP_DATA, SCL_COEF_RAM_ODD_TAP_COEF_EN, mask_sh),\
	TF_SF(DSCL0_DSCL_2TAP_CONTROL, SCL_H_2TAP_HARDCODE_COEF_EN, mask_sh),\
	TF_SF(DSCL0_DSCL_2TAP_CONTROL, SCL_H_2TAP_SHARP_EN, mask_sh),\
	TF_SF(DSCL0_DSCL_2TAP_CONTROL, SCL_H_2TAP_SHARP_FACTOR, mask_sh),\
	TF_SF(DSCL0_DSCL_2TAP_CONTROL, SCL_V_2TAP_HARDCODE_COEF_EN, mask_sh),\
	TF_SF(DSCL0_DSCL_2TAP_CONTROL, SCL_V_2TAP_SHARP_EN, mask_sh),\
	TF_SF(DSCL0_DSCL_2TAP_CONTROL, SCL_V_2TAP_SHARP_FACTOR, mask_sh),\
	TF_SF(DSCL0_SCL_MODE, SCL_COEF_RAM_SELECT, mask_sh),\
	TF_SF(DSCL0_SCL_MODE, DSCL_MODE, mask_sh),\
	TF_SF(DSCL0_RECOUT_START, RECOUT_START_X, mask_sh),\
	TF_SF(DSCL0_RECOUT_START, RECOUT_START_Y, mask_sh),\
	TF_SF(DSCL0_RECOUT_SIZE, RECOUT_WIDTH, mask_sh),\
	TF_SF(DSCL0_RECOUT_SIZE, RECOUT_HEIGHT, mask_sh),\
	TF_SF(DSCL0_MPC_SIZE, MPC_WIDTH, mask_sh),\
	TF_SF(DSCL0_MPC_SIZE, MPC_HEIGHT, mask_sh),\
	TF_SF(DSCL0_SCL_HORZ_FILTER_SCALE_RATIO, SCL_H_SCALE_RATIO, mask_sh),\
	TF_SF(DSCL0_SCL_VERT_FILTER_SCALE_RATIO, SCL_V_SCALE_RATIO, mask_sh),\
	TF_SF(DSCL0_SCL_HORZ_FILTER_SCALE_RATIO_C, SCL_H_SCALE_RATIO_C, mask_sh),\
	TF_SF(DSCL0_SCL_VERT_FILTER_SCALE_RATIO_C, SCL_V_SCALE_RATIO_C, mask_sh),\
	TF_SF(DSCL0_SCL_HORZ_FILTER_INIT, SCL_H_INIT_FRAC, mask_sh),\
	TF_SF(DSCL0_SCL_HORZ_FILTER_INIT, SCL_H_INIT_INT, mask_sh),\
	TF_SF(DSCL0_SCL_HORZ_FILTER_INIT_C, SCL_H_INIT_FRAC_C, mask_sh),\
	TF_SF(DSCL0_SCL_HORZ_FILTER_INIT_C, SCL_H_INIT_INT_C, mask_sh),\
	TF_SF(DSCL0_SCL_VERT_FILTER_INIT, SCL_V_INIT_FRAC, mask_sh),\
	TF_SF(DSCL0_SCL_VERT_FILTER_INIT, SCL_V_INIT_INT, mask_sh),\
	TF_SF(DSCL0_SCL_VERT_FILTER_INIT_C, SCL_V_INIT_FRAC_C, mask_sh),\
	TF_SF(DSCL0_SCL_VERT_FILTER_INIT_C, SCL_V_INIT_INT_C, mask_sh),\
	TF_SF(DSCL0_SCL_MODE, SCL_CHROMA_COEF_MODE, mask_sh),\
	TF_SF(DSCL0_SCL_MODE, SCL_COEF_RAM_SELECT_CURRENT, mask_sh), \
	TF_SF(CNVC_CFG0_PRE_DEALPHA, PRE_DEALPHA_EN, mask_sh), \
	TF_SF(CNVC_CFG0_PRE_DEALPHA, PRE_DEALPHA_ABLND_EN, mask_sh), \
	TF_SF(CNVC_CFG0_PRE_REALPHA, PRE_REALPHA_EN, mask_sh), \
	TF_SF(CNVC_CFG0_PRE_REALPHA, PRE_REALPHA_ABLND_EN, mask_sh), \
	TF_SF(CNVC_CFG0_PRE_CSC_MODE, PRE_CSC_MODE, mask_sh), \
	TF_SF(CNVC_CFG0_PRE_CSC_MODE, PRE_CSC_MODE_CURRENT, mask_sh), \
	TF_SF(CNVC_CFG0_PRE_CSC_C11_C12, PRE_CSC_C11, mask_sh), \
	TF_SF(CNVC_CFG0_PRE_CSC_C11_C12, PRE_CSC_C12, mask_sh), \
	TF_SF(CNVC_CFG0_PRE_CSC_C33_C34, PRE_CSC_C33, mask_sh), \
	TF_SF(CNVC_CFG0_PRE_CSC_C33_C34, PRE_CSC_C34, mask_sh), \
	TF_SF(CM0_CM_POST_CSC_CONTROL, CM_POST_CSC_MODE, mask_sh), \
	TF_SF(CM0_CM_POST_CSC_CONTROL, CM_POST_CSC_MODE_CURRENT, mask_sh), \
	TF_SF(CM0_CM_POST_CSC_C11_C12, CM_POST_CSC_C11, mask_sh), \
	TF_SF(CM0_CM_POST_CSC_C11_C12, CM_POST_CSC_C12, mask_sh), \
	TF_SF(CM0_CM_POST_CSC_C33_C34, CM_POST_CSC_C33, mask_sh), \
	TF_SF(CM0_CM_POST_CSC_C33_C34, CM_POST_CSC_C34, mask_sh), \
	TF_SF(CM0_CM_TEST_DEBUG_INDEX, CM_TEST_DEBUG_INDEX, mask_sh), \
	TF_SF(CNVC_CFG0_FORMAT_CONTROL, CNVC_BYPASS, mask_sh), \
	TF2_SF(CNVC_CFG0, FORMAT_CONTROL__ALPHA_EN, mask_sh), \
	TF_SF(CNVC_CFG0_FORMAT_CONTROL, FORMAT_EXPANSION_MODE, mask_sh), \
	TF_SF(CNVC_CFG0_CNVC_SURFACE_PIXEL_FORMAT, CNVC_SURFACE_PIXEL_FORMAT, mask_sh), \
	TF_SF(CNVC_CFG0_CNVC_SURFACE_PIXEL_FORMAT, CNVC_ALPHA_PLANE_ENABLE, mask_sh), \
	TF_SF(CNVC_CUR0_CURSOR0_CONTROL, CUR0_MODE, mask_sh), \
	TF_SF(CNVC_CUR0_CURSOR0_CONTROL, CUR0_EXPANSION_MODE, mask_sh), \
	TF_SF(CNVC_CUR0_CURSOR0_CONTROL, CUR0_ENABLE, mask_sh), \
	TF_SF(CNVC_CUR0_CURSOR0_COLOR0, CUR0_COLOR0, mask_sh), \
	TF_SF(CNVC_CUR0_CURSOR0_COLOR1, CUR0_COLOR1, mask_sh), \
	TF_SF(CNVC_CUR0_CURSOR0_FP_SCALE_BIAS, CUR0_FP_BIAS, mask_sh), \
	TF_SF(CNVC_CUR0_CURSOR0_FP_SCALE_BIAS, CUR0_FP_SCALE, mask_sh), \
	TF_SF(DPP_TOP0_DPP_CONTROL, DPP_CLOCK_ENABLE, mask_sh), \
	TF_SF(CM0_CM_HDR_MULT_COEF, CM_HDR_MULT_COEF, mask_sh), \
	TF_SF(CM0_CM_CONTROL, CM_BYPASS, mask_sh), \
	TF_SF(CURSOR0_0_CURSOR_CONTROL, CURSOR_MODE, mask_sh), \
	TF_SF(CURSOR0_0_CURSOR_CONTROL, CURSOR_PITCH, mask_sh), \
	TF_SF(CURSOR0_0_CURSOR_CONTROL, CURSOR_LINES_PER_CHUNK, mask_sh), \
	TF_SF(CURSOR0_0_CURSOR_CONTROL, CURSOR_ENABLE, mask_sh), \
	TF_SF(CNVC_CFG0_FORMAT_CONTROL, FORMAT_CNV16, mask_sh), \
	TF_SF(CNVC_CFG0_FORMAT_CONTROL, CNVC_BYPASS_MSB_ALIGN, mask_sh), \
	TF_SF(CNVC_CFG0_FORMAT_CONTROL, CLAMP_POSITIVE, mask_sh), \
	TF_SF(CNVC_CFG0_FORMAT_CONTROL, CLAMP_POSITIVE_C, mask_sh), \
	TF_SF(CNVC_CFG0_FORMAT_CONTROL, FORMAT_CROSSBAR_R, mask_sh), \
	TF_SF(CNVC_CFG0_FORMAT_CONTROL, FORMAT_CROSSBAR_G, mask_sh), \
	TF_SF(CNVC_CFG0_FORMAT_CONTROL, FORMAT_CROSSBAR_B, mask_sh), \
	TF_SF(CNVC_CFG0_ALPHA_2BIT_LUT, ALPHA_2BIT_LUT0, mask_sh), \
	TF_SF(CNVC_CFG0_ALPHA_2BIT_LUT, ALPHA_2BIT_LUT1, mask_sh), \
	TF_SF(CNVC_CFG0_ALPHA_2BIT_LUT, ALPHA_2BIT_LUT2, mask_sh), \
	TF_SF(CNVC_CFG0_ALPHA_2BIT_LUT, ALPHA_2BIT_LUT3, mask_sh), \
	TF_SF(CNVC_CFG0_FCNV_FP_BIAS_R, FCNV_FP_BIAS_R, mask_sh), \
	TF_SF(CNVC_CFG0_FCNV_FP_BIAS_G, FCNV_FP_BIAS_G, mask_sh), \
	TF_SF(CNVC_CFG0_FCNV_FP_BIAS_B, FCNV_FP_BIAS_B, mask_sh), \
	TF_SF(CNVC_CFG0_FCNV_FP_SCALE_R, FCNV_FP_SCALE_R, mask_sh), \
	TF_SF(CNVC_CFG0_FCNV_FP_SCALE_G, FCNV_FP_SCALE_G, mask_sh), \
	TF_SF(CNVC_CFG0_FCNV_FP_SCALE_B, FCNV_FP_SCALE_B, mask_sh), \
	TF_SF(CNVC_CFG0_COLOR_KEYER_CONTROL, COLOR_KEYER_EN, mask_sh), \
	TF_SF(CNVC_CFG0_COLOR_KEYER_CONTROL, COLOR_KEYER_MODE, mask_sh), \
	TF_SF(CNVC_CFG0_COLOR_KEYER_ALPHA, COLOR_KEYER_ALPHA_LOW, mask_sh), \
	TF_SF(CNVC_CFG0_COLOR_KEYER_ALPHA, COLOR_KEYER_ALPHA_HIGH, mask_sh), \
	TF_SF(CNVC_CFG0_COLOR_KEYER_RED, COLOR_KEYER_RED_LOW, mask_sh), \
	TF_SF(CNVC_CFG0_COLOR_KEYER_RED, COLOR_KEYER_RED_HIGH, mask_sh), \
	TF_SF(CNVC_CFG0_COLOR_KEYER_GREEN, COLOR_KEYER_GREEN_LOW, mask_sh), \
	TF_SF(CNVC_CFG0_COLOR_KEYER_GREEN, COLOR_KEYER_GREEN_HIGH, mask_sh), \
	TF_SF(CNVC_CFG0_COLOR_KEYER_BLUE, COLOR_KEYER_BLUE_LOW, mask_sh), \
	TF_SF(CNVC_CFG0_COLOR_KEYER_BLUE, COLOR_KEYER_BLUE_HIGH, mask_sh), \
	TF_SF(CNVC_CUR0_CURSOR0_CONTROL, CUR0_PIX_INV_MODE, mask_sh), \
	TF_SF(CNVC_CUR0_CURSOR0_CONTROL, CUR0_PIXEL_ALPHA_MOD_EN, mask_sh), \
	TF_SF(CNVC_CUR0_CURSOR0_CONTROL, CUR0_ROM_EN, mask_sh),\
	TF_SF(DSCL0_OBUF_MEM_PWR_CTRL, OBUF_MEM_PWR_FORCE, mask_sh),\
	TF_SF(DSCL0_DSCL_MEM_PWR_CTRL, LUT_MEM_PWR_FORCE, mask_sh),\
	TF_SF(DSCL0_DSCL_MEM_PWR_STATUS, LUT_MEM_PWR_STATE, mask_sh)

#define DPP_REG_LIST_SH_MASK_DCN30_UPDATED(mask_sh)\
	TF_SF(CM0_CM_MEM_PWR_STATUS, BLNDGAM_MEM_PWR_STATE, mask_sh), \
	TF_SF(CM0_CM_MEM_PWR_CTRL2, HDR3DLUT_MEM_PWR_FORCE, mask_sh),\
	TF_SF(CM0_CM_MEM_PWR_CTRL2, SHAPER_MEM_PWR_FORCE, mask_sh),\
	TF_SF(CM0_CM_MEM_PWR_STATUS2, HDR3DLUT_MEM_PWR_STATE, mask_sh),\
	TF_SF(CM0_CM_MEM_PWR_STATUS2, SHAPER_MEM_PWR_STATE, mask_sh),\
	TF_SF(CM0_CM_BLNDGAM_CONTROL, CM_BLNDGAM_MODE, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_CONTROL, CM_BLNDGAM_MODE_CURRENT, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_CONTROL, CM_BLNDGAM_SELECT_CURRENT, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_CONTROL, CM_BLNDGAM_SELECT, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMB_START_SLOPE_CNTL_B, CM_BLNDGAM_RAMB_EXP_REGION_START_SLOPE_B, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMB_START_SLOPE_CNTL_G, CM_BLNDGAM_RAMB_EXP_REGION_START_SLOPE_G, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMB_START_SLOPE_CNTL_R, CM_BLNDGAM_RAMB_EXP_REGION_START_SLOPE_R, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMB_END_CNTL1_B, CM_BLNDGAM_RAMB_EXP_REGION_END_BASE_B, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMB_END_CNTL1_G, CM_BLNDGAM_RAMB_EXP_REGION_END_BASE_G, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMB_END_CNTL1_R, CM_BLNDGAM_RAMB_EXP_REGION_END_BASE_R, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMA_START_SLOPE_CNTL_B, CM_BLNDGAM_RAMA_EXP_REGION_START_SLOPE_B, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMA_START_SLOPE_CNTL_G, CM_BLNDGAM_RAMA_EXP_REGION_START_SLOPE_G, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMA_START_SLOPE_CNTL_R, CM_BLNDGAM_RAMA_EXP_REGION_START_SLOPE_R, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMA_END_CNTL1_B, CM_BLNDGAM_RAMA_EXP_REGION_END_BASE_B, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMA_END_CNTL1_G, CM_BLNDGAM_RAMA_EXP_REGION_END_BASE_G, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMA_END_CNTL1_R, CM_BLNDGAM_RAMA_EXP_REGION_END_BASE_R, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMA_END_CNTL2_B, CM_BLNDGAM_RAMA_EXP_REGION_END_B, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMA_END_CNTL2_G, CM_BLNDGAM_RAMA_EXP_REGION_END_G, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_RAMA_END_CNTL2_R, CM_BLNDGAM_RAMA_EXP_REGION_END_R, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_LUT_CONTROL, CM_BLNDGAM_LUT_WRITE_COLOR_MASK, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_LUT_CONTROL, CM_BLNDGAM_LUT_HOST_SEL, mask_sh), \
	TF_SF(CM0_CM_BLNDGAM_LUT_CONTROL, CM_BLNDGAM_LUT_CONFIG_MODE, mask_sh), \
	TF_SF(CM0_CM_3DLUT_MODE, CM_3DLUT_MODE_CURRENT, mask_sh), \
	TF_SF(CM0_CM_SHAPER_CONTROL, CM_SHAPER_MODE_CURRENT, mask_sh)


#define DPP_REG_LIST_SH_MASK_DCN30(mask_sh)\
	DPP_REG_LIST_SH_MASK_DCN30_COMMON(mask_sh), \
	TF_REG_LIST_SH_MASK_DCN20_COMMON(mask_sh), \
	DPP_REG_LIST_SH_MASK_DCN30_UPDATED(mask_sh)

#define DPP_REG_FIELD_LIST_DCN3(type) \
	TF_REG_FIELD_LIST_DCN2_0(type); \
	type FORMAT_CROSSBAR_R; \
	type FORMAT_CROSSBAR_G; \
	type FORMAT_CROSSBAR_B; \
	type CM_DEALPHA_EN;\
	type CM_DEALPHA_ABLND;\
	type CM_BIAS_Y_G;\
	type CM_BIAS_CB_B;\
	type CM_BIAS_CR_R;\
	type GAMCOR_MEM_PWR_DIS; \
	type GAMCOR_MEM_PWR_FORCE; \
	type HDR3DLUT_MEM_PWR_FORCE; \
	type SHAPER_MEM_PWR_FORCE; \
	type PRE_DEGAM_MODE;\
	type PRE_DEGAM_SELECT;\
	type CNVC_ALPHA_PLANE_ENABLE; \
	type PRE_DEALPHA_EN; \
	type PRE_DEALPHA_ABLND_EN; \
	type PRE_REALPHA_EN; \
	type PRE_REALPHA_ABLND_EN; \
	type PRE_CSC_MODE; \
	type PRE_CSC_MODE_CURRENT; \
	type PRE_CSC_C11; \
	type PRE_CSC_C12; \
	type PRE_CSC_C33; \
	type PRE_CSC_C34; \
	type CM_POST_CSC_MODE; \
	type CM_POST_CSC_MODE_CURRENT; \
	type CM_POST_CSC_C11; \
	type CM_POST_CSC_C12; \
	type CM_POST_CSC_C33; \
	type CM_POST_CSC_C34; \
	type CM_GAMCOR_MODE; \
	type CM_GAMCOR_SELECT; \
	type CM_GAMCOR_PWL_DISABLE; \
	type CM_GAMCOR_MODE_CURRENT; \
	type CM_GAMCOR_SELECT_CURRENT; \
	type CM_GAMCOR_LUT_INDEX; \
	type CM_GAMCOR_LUT_DATA; \
	type CM_GAMCOR_LUT_WRITE_COLOR_MASK; \
	type CM_GAMCOR_LUT_READ_COLOR_SEL; \
	type CM_GAMCOR_LUT_READ_DBG; \
	type CM_GAMCOR_LUT_HOST_SEL; \
	type CM_GAMCOR_LUT_CONFIG_MODE; \
	type CM_GAMCOR_LUT_STATUS; \
	type CM_GAMCOR_RAMA_EXP_REGION_START_B; \
	type CM_GAMCOR_RAMA_EXP_REGION_START_SEGMENT_B; \
	type CM_GAMCOR_RAMA_EXP_REGION_START_SLOPE_B; \
	type CM_GAMCOR_RAMA_EXP_REGION_START_BASE_B; \
	type CM_GAMCOR_RAMA_EXP_REGION_END_BASE_B; \
	type CM_GAMCOR_RAMA_EXP_REGION_END_B; \
	type CM_GAMCOR_RAMA_EXP_REGION_END_SLOPE_B; \
	type CM_GAMCOR_RAMA_OFFSET_B; \
	type CM_GAMCOR_RAMA_EXP_REGION0_LUT_OFFSET; \
	type CM_GAMCOR_RAMA_EXP_REGION0_NUM_SEGMENTS; \
	type CM_GAMCOR_RAMA_EXP_REGION1_LUT_OFFSET; \
	type CM_GAMCOR_RAMA_EXP_REGION1_NUM_SEGMENTS;\
	type CM_GAMUT_REMAP_MODE_CURRENT;\
	type CM_BLNDGAM_RAMB_EXP_REGION_START_SLOPE_B; \
	type CM_BLNDGAM_RAMB_EXP_REGION_START_SLOPE_G; \
	type CM_BLNDGAM_RAMB_EXP_REGION_START_SLOPE_R; \
	type CM_BLNDGAM_RAMA_EXP_REGION_START_SLOPE_B; \
	type CM_BLNDGAM_RAMA_EXP_REGION_START_SLOPE_G; \
	type CM_BLNDGAM_RAMA_EXP_REGION_START_SLOPE_R; \
	type CM_BLNDGAM_LUT_WRITE_COLOR_MASK; \
	type CM_BLNDGAM_LUT_HOST_SEL; \
	type CM_BLNDGAM_LUT_CONFIG_MODE; \
	type CM_3DLUT_MODE_CURRENT; \
	type CM_SHAPER_MODE_CURRENT; \
	type CM_BLNDGAM_MODE; \
	type CM_BLNDGAM_MODE_CURRENT; \
	type CM_BLNDGAM_SELECT_CURRENT; \
	type CM_BLNDGAM_SELECT; \
	type GAMCOR_MEM_PWR_STATE; \
	type BLNDGAM_MEM_PWR_STATE; \
	type HDR3DLUT_MEM_PWR_STATE; \
	type SHAPER_MEM_PWR_STATE

struct dcn3_dpp_shift {
	DPP_REG_FIELD_LIST_DCN3(uint8_t);
};

struct dcn3_dpp_mask {
	DPP_REG_FIELD_LIST_DCN3(uint32_t);
};

#define DPP_DCN3_REG_VARIABLE_LIST_COMMON \
	DPP_DCN2_REG_VARIABLE_LIST; \
	uint32_t CM_MEM_PWR_STATUS;\
	uint32_t CM_MEM_PWR_STATUS2;\
	uint32_t CM_MEM_PWR_CTRL2;\
	uint32_t CM_DEALPHA;\
	uint32_t CM_BIAS_CR_R;\
	uint32_t CM_BIAS_Y_G_CB_B;\
	uint32_t PRE_DEGAM;\
	uint32_t PRE_DEALPHA; \
	uint32_t PRE_REALPHA; \
	uint32_t PRE_CSC_MODE; \
	uint32_t PRE_CSC_C11_C12; \
	uint32_t PRE_CSC_C33_C34; \
	uint32_t PRE_CSC_B_C11_C12; \
	uint32_t PRE_CSC_B_C33_C34; \
	uint32_t CM_POST_CSC_CONTROL; \
	uint32_t CM_POST_CSC_C11_C12; \
	uint32_t CM_POST_CSC_C33_C34; \
	uint32_t CM_POST_CSC_B_C11_C12; \
	uint32_t CM_POST_CSC_B_C33_C34; \
	uint32_t CM_GAMUT_REMAP_B_C11_C12; \
	uint32_t CM_GAMUT_REMAP_B_C13_C14; \
	uint32_t CM_GAMUT_REMAP_B_C21_C22; \
	uint32_t CM_GAMUT_REMAP_B_C23_C24; \
	uint32_t CM_GAMUT_REMAP_B_C31_C32; \
	uint32_t CM_GAMUT_REMAP_B_C33_C34; \
	uint32_t CM_GAMCOR_CONTROL; \
	uint32_t CM_GAMCOR_LUT_CONTROL; \
	uint32_t CM_GAMCOR_LUT_INDEX; \
	uint32_t CM_GAMCOR_LUT_DATA; \
	uint32_t CM_GAMCOR_RAMB_START_CNTL_B; \
	uint32_t CM_GAMCOR_RAMB_START_CNTL_G; \
	uint32_t CM_GAMCOR_RAMB_START_CNTL_R; \
	uint32_t CM_GAMCOR_RAMB_START_SLOPE_CNTL_B; \
	uint32_t CM_GAMCOR_RAMB_START_SLOPE_CNTL_G; \
	uint32_t CM_GAMCOR_RAMB_START_SLOPE_CNTL_R; \
	uint32_t CM_GAMCOR_RAMB_END_CNTL1_B; \
	uint32_t CM_GAMCOR_RAMB_END_CNTL2_B; \
	uint32_t CM_GAMCOR_RAMB_END_CNTL1_G; \
	uint32_t CM_GAMCOR_RAMB_END_CNTL2_G; \
	uint32_t CM_GAMCOR_RAMB_END_CNTL1_R; \
	uint32_t CM_GAMCOR_RAMB_END_CNTL2_R; \
	uint32_t CM_GAMCOR_RAMB_REGION_0_1; \
	uint32_t CM_GAMCOR_RAMB_REGION_32_33; \
	uint32_t CM_GAMCOR_RAMB_OFFSET_B; \
	uint32_t CM_GAMCOR_RAMB_OFFSET_G; \
	uint32_t CM_GAMCOR_RAMB_OFFSET_R; \
	uint32_t CM_GAMCOR_RAMB_START_BASE_CNTL_B; \
	uint32_t CM_GAMCOR_RAMB_START_BASE_CNTL_G; \
	uint32_t CM_GAMCOR_RAMB_START_BASE_CNTL_R; \
	uint32_t CM_GAMCOR_RAMA_START_CNTL_B; \
	uint32_t CM_GAMCOR_RAMA_START_CNTL_G; \
	uint32_t CM_GAMCOR_RAMA_START_CNTL_R; \
	uint32_t CM_GAMCOR_RAMA_START_SLOPE_CNTL_B; \
	uint32_t CM_GAMCOR_RAMA_START_SLOPE_CNTL_G; \
	uint32_t CM_GAMCOR_RAMA_START_SLOPE_CNTL_R; \
	uint32_t CM_GAMCOR_RAMA_END_CNTL1_B; \
	uint32_t CM_GAMCOR_RAMA_END_CNTL2_B; \
	uint32_t CM_GAMCOR_RAMA_END_CNTL1_G; \
	uint32_t CM_GAMCOR_RAMA_END_CNTL2_G; \
	uint32_t CM_GAMCOR_RAMA_END_CNTL1_R; \
	uint32_t CM_GAMCOR_RAMA_END_CNTL2_R; \
	uint32_t CM_GAMCOR_RAMA_REGION_0_1; \
	uint32_t CM_GAMCOR_RAMA_REGION_32_33; \
	uint32_t CM_GAMCOR_RAMA_OFFSET_B; \
	uint32_t CM_GAMCOR_RAMA_OFFSET_G; \
	uint32_t CM_GAMCOR_RAMA_OFFSET_R; \
	uint32_t CM_GAMCOR_RAMA_START_BASE_CNTL_B; \
	uint32_t CM_GAMCOR_RAMA_START_BASE_CNTL_G; \
	uint32_t CM_GAMCOR_RAMA_START_BASE_CNTL_R; \
	uint32_t CM_BLNDGAM_RAMA_START_SLOPE_CNTL_B; \
	uint32_t CM_BLNDGAM_RAMA_START_SLOPE_CNTL_G; \
	uint32_t CM_BLNDGAM_RAMA_START_SLOPE_CNTL_R; \
	uint32_t CM_BLNDGAM_RAMB_START_SLOPE_CNTL_B; \
	uint32_t CM_BLNDGAM_RAMB_START_SLOPE_CNTL_G; \
	uint32_t CM_BLNDGAM_RAMB_START_SLOPE_CNTL_R; \
	uint32_t CM_BLNDGAM_LUT_CONTROL


struct dcn3_dpp_registers {
	DPP_DCN3_REG_VARIABLE_LIST_COMMON;
};


struct dcn3_dpp {
	struct dpp base;

	const struct dcn3_dpp_registers *tf_regs;
	const struct dcn3_dpp_shift *tf_shift;
	const struct dcn3_dpp_mask *tf_mask;

	const uint16_t *filter_v;
	const uint16_t *filter_h;
	const uint16_t *filter_v_c;
	const uint16_t *filter_h_c;
	int lb_pixel_depth_supported;
	int lb_memory_size;
	int lb_bits_per_entry;
	bool is_write_to_ram_a_safe;
	bool dispclk_r_gate_disable;
	struct scaler_data scl_data;
	struct pwl_params pwl_data;
};

bool dpp3_construct(struct dcn3_dpp *dpp3,
	struct dc_context *ctx,
	uint32_t inst,
	const struct dcn3_dpp_registers *tf_regs,
	const struct dcn3_dpp_shift *tf_shift,
	const struct dcn3_dpp_mask *tf_mask);

bool dpp3_program_gamcor_lut(
	struct dpp *dpp_base, const struct pwl_params *params);

void dpp3_program_CM_dealpha(
		struct dpp *dpp_base,
		uint32_t enable, uint32_t additive_blending);

void dpp30_read_state(struct dpp *dpp_base,
		struct dcn_dpp_state *s);

bool dpp3_get_optimal_number_of_taps(
		struct dpp *dpp,
		struct scaler_data *scl_data,
		const struct scaling_taps *in_taps);

void dpp3_cnv_setup(
		struct dpp *dpp_base,
		enum surface_pixel_format format,
		enum expansion_mode mode,
		struct dc_csc_transform input_csc_color_matrix,
		enum dc_color_space input_color_space,
		struct cnv_alpha_2bit_lut *alpha_2bit_lut);

void dpp3_program_CM_bias(
		struct dpp *dpp_base,
		struct CM_bias_params *bias_params);

void dpp3_set_hdr_multiplier(
		struct dpp *dpp_base,
		uint32_t multiplier);

void dpp3_cm_set_gamut_remap(
		struct dpp *dpp_base,
		const struct dpp_grph_csc_adjustment *adjust);

void dpp3_set_pre_degam(struct dpp *dpp_base,
		enum dc_transfer_func_predefined tr);

void dpp3_set_cursor_attributes(
		struct dpp *dpp_base,
		struct dc_cursor_attributes *cursor_attributes);

void dpp3_program_post_csc(
		struct dpp *dpp_base,
		enum dc_color_space color_space,
		enum dcn10_input_csc_select input_select,
		const struct out_csc_color_matrix *tbl_entry);

void dpp3_program_cm_bias(
	struct dpp *dpp_base,
	struct CM_bias_params *bias_params);

void dpp3_program_cm_dealpha(
		struct dpp *dpp_base,
	uint32_t enable, uint32_t additive_blending);

void dpp3_cm_get_gamut_remap(struct dpp *dpp_base,
			     struct dpp_grph_csc_adjustment *adjust);
#endif /* __DC_HWSS_DCN30_H__ */
