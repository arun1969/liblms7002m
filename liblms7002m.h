/*
 * lms7002m compact library header file
 * Copyright (c) 2018 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 * For more information, please visit: http://xtrx.io
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#ifndef LIBLMS7002MC_H
#define LIBLMS7002MC_H

#include <stdint.h>
#include <stdbool.h>
#ifdef __linux
#include <unistd.h>
#endif

#define MUX_BIAS_OUT			0x0084,12 << 4 | 11
#define RP_CALIB_BIAS			0x0084,10 << 4 | 6
#define PD_FRP_BIAS				0x0084,4 << 4 | 4
#define PD_F_BIAS				0x0084,3 << 4 | 3
#define PD_PTRP_BIAS			0x0084,2 << 4 | 2
#define PD_PT_BIAS				0x0084,1 << 4 | 1
#define PD_BIAS_MASTER			0x0084,0 << 4 | 0

#define EN_LOWBWLOMX_TMX_TRF	0x0100,15 << 4 | 15
#define EN_NEXTTX_TRF			0x0100,14 << 4 | 14
#define EN_AMPHF_PDET_TRF		0x0100,13 << 4 | 12
#define LOADR_PDET_TRF			0x0100,11 << 4 | 10
#define PD_PDET_TRF				0x0100,3 << 4 | 3
#define PD_TLOBUF_TRF			0x0100,2 << 4 | 2
#define PD_TXPAD_TRF			0x0100,1 << 4 | 1
#define EN_G_TRF				0x0100,0 << 4 | 0

#define F_TXPAD_TRF				0x0101,15 << 4 | 13
#define L_LOOPB_TXPAD_TRF		0x0101,12 << 4 | 11
#define LOSS_LIN_TXPAD_TRF		0x0101,10 << 4 | 6
#define LOSS_MAIN_TXPAD_TRF		0x0101,5 << 4 | 1
#define EN_LOOPB_TXPAD_TRF		0x0101,0 << 4 | 0

#define SEL_BAND1_TRF			0x0103,11 << 4 | 11
#define SEL_BAND2_TRF			0x0103,10 << 4 | 10
#define LOBIASN_TXM_TRF			0x0103,9 << 4 | 5
#define LOBIASP_TXX_TRF			0x0103,4 << 4 | 0

#define STATPULSE_TBB			0x0105,15 << 4 | 15
#define LOOPB_TBB				0x0105,14 << 4 | 12
#define PD_LPFH_TBB				0x0105,4 << 4 | 4
#define PD_LPFIAMP_TBB			0x0105,3 << 4 | 3
#define PD_LPFLAD_TBB			0x0105,2 << 4 | 2
#define PD_LPFS5_TBB			0x0105,1 << 4 | 1
#define EN_G_TBB				0x0105,0 << 4 | 0

#define CG_IAMP_TBB				0x0108,15 << 4 | 10
#define ICT_IAMP_FRP_TBB		0x0108,9 << 4 | 5
#define ICT_IAMP_GG_FRP_TBB		0x0108,4 << 4 | 0

#define RCAL_LPFH_TBB			0x0109,15 << 4 | 8
#define RCAL_LPFLAD_TBB			0x0109,7 << 4 | 0

#define TSTIN_TBB				0x010A,15 << 4 | 14
#define BYPLADDER_TBB			0x010A,13 << 4 | 13
#define CCAL_LPFLAD_TBB			0x010A,12 << 4 | 8
#define RCAL_LPFS5_TBB			0x010A,7 << 4 | 0

#define CDC_I_RFE				0x010C,15 << 4 | 12
#define CDC_Q_RFE				0x010C,11 << 4 | 8
#define PD_LNA_RFE				0x010C,7 << 4 | 7
#define PD_RLOOPB_1_RFE			0x010C,6 << 4 | 6
#define PD_RLOOPB_2_RFE			0x010C,5 << 4 | 5
#define PD_MXLOBUF_RFE			0x010C,4 << 4 | 4
#define PD_QGEN_RFE				0x010C,3 << 4 | 3
#define PD_RSSI_RFE				0x010C,2 << 4 | 2
#define PD_TIA_RFE				0x010C,1 << 4 | 1
#define EN_G_RFE				0x010C,0 << 4 | 0

#define SEL_PATH_RFE			0x010D,8 << 4 | 7
#define EN_DCOFF_RXFE_RFE		0x010D,6 << 4 | 6
#define EN_INSHSW_LB1_RFE		0x010D,4 << 4 | 4
#define EN_INSHSW_LB2_RFE		0x010D,3 << 4 | 3
#define EN_INSHSW_L_RFE			0x010D,2 << 4 | 2
#define EN_INSHSW_W_RFE			0x010D,1 << 4 | 1
#define EN_NEXTRX_RFE			0x010D,0 << 4 | 0

#define ICT_LOOPB_RFE			0x010F,14 << 4 | 10
#define ICT_TIAMAIN_RFE			0x010F,9 << 4 | 5
#define ICT_TIAOUT_RFE			0x010F,4 << 4 | 0

#define CCOMP_TIA_RFE			0x0112,15 << 4 | 12
#define CFB_TIA_RFE				0x0112,11 << 4 | 0

#define G_LNA_RFE				0x0113,9 << 4 | 6
#define G_RXLOOPB_RFE			0x0113,5 << 4 | 2
#define G_TIA_RFE				0x0113,1 << 4 | 0

#define RCOMP_TIA_RFE			0x0114,8 << 4 | 5
#define RFB_TIA_RFE				0x0114,4 << 4 | 0

#define EN_LB_LPFH_RBB			0x0115,15 << 4 | 15
#define EN_LB_LPFL_RBB			0x0115,14 << 4 | 14
#define PD_LPFH_RBB				0x0115,3 << 4 | 3
#define PD_LPFL_RBB				0x0115,2 << 4 | 2
#define PD_PGA_RBB				0x0115,1 << 4 | 1
#define EN_G_RBB				0x0115,0 << 4 | 0

#define R_CTL_LPF_RBB			0x0116,15 << 4 | 11
#define RCC_CTL_LPFH_RBB		0x0116,10 << 4 | 8
#define C_CTL_LPFH_RBB			0x0116,7 << 4 | 0

#define RCC_CTL_LPFL_RBB		0x0117,13 << 4 | 11
#define C_CTL_LPFL_RBB			0x0117,10 << 4 | 0

#define INPUT_CTL_PGA_RBB		0x0118,15 << 4 | 13
#define ICT_LPF_IN_RBB			0x0118,9 << 4 | 5
#define ICT_LPF_OUT_RBB			0x0118,4 << 4 | 0

#define OSW_PGA_RBB				0x0119,15 << 4 | 15
#define ICT_PGA_OUT_RBB			0x0119,14 << 4 | 10
#define ICT_PGA_IN_RBB			0x0119,9 << 4 | 5
#define G_PGA_RBB				0x0119,4 << 4 | 0

#define RCC_CTL_PGA_RBB			0x011A,13 << 4 | 9
#define C_CTL_PGA_RBB			0x011A,6 << 4 | 0

#define TSGFC_TXTSP				0x0200,9 << 4 | 9
#define TSGFCW_TXTSP			0x0200,8 << 4 | 7
#define TSGDCLDQ_TXTSP			0x0200,6 << 4 | 6
#define TSGDCLDI_TXTSP			0x0200,5 << 4 | 5
#define TSGSWAPIQ_TXTSP			0x0200,4 << 4 | 4
#define TSGMODE_TXTSP			0x0200,3 << 4 | 3
#define INSEL_TXTSP				0x0200,2 << 4 | 2
#define BSTART_TXTSP			0x0200,1 << 4 | 1
#define EN_TXTSP				0x0200,0 << 4 | 0

#define CMIX_GAIN_TXTSP			0x0208,15 << 4 | 14
#define CMIX_GAIN_TXTSP_R3		0x0208,12 << 4 | 12
#define CMIX_SC_TXTSP			0x0208,13 << 4 | 13
#define CMIX_BYP_TXTSP			0x0208,8 << 4 | 8
#define ISINC_BYP_TXTSP			0x0208,7 << 4 | 7
#define GFIR3_BYP_TXTSP			0x0208,6 << 4 | 6
#define GFIR2_BYP_TXTSP			0x0208,5 << 4 | 5
#define GFIR1_BYP_TXTSP			0x0208,4 << 4 | 4
#define DC_BYP_TXTSP			0x0208,3 << 4 | 3
#define GC_BYP_TXTSP			0x0208,1 << 4 | 1
#define PH_BYP_TXTSP			0x0208,0 << 4 | 0

#define DC_REG_TXTSP			0x020C,15 << 4 | 0

#define AGC_MODE_RXTSP			0x040A,13 << 4 | 12
#define AGC_AVG_RXTSP			0x040A,2 << 4 | 0

#define CMIX_GAIN_RXTSP			0x040C,15 << 4 | 14
#define CMIX_SC_RXTSP			0x040C,13 << 4 | 13
#define CMIX_GAIN_RXTSP_R3		0x040C,12 << 4 | 12
#define CMIX_BYP_RXTSP			0x040C,7 << 4 | 7
#define AGC_BYP_RXTSP			0x040C,6 << 4 | 6
#define GFIR3_BYP_RXTSP			0x040C,5 << 4 | 5
#define GFIR2_BYP_RXTSP			0x040C,4 << 4 | 4
#define GFIR1_BYP_RXTSP			0x040C,3 << 4 | 3
#define DC_BYP_RXTSP			0x040C,2 << 4 | 2
#define GC_BYP_RXTSP			0x040C,1 << 4 | 1
#define PH_BYP_RXTSP			0x040C,0 << 4 | 0

static inline int lms7_regs_default(uint16_t addr)
{
	if(addr == 0x0020) return(0xffff);
	if(addr == 0x0021) return(0xe9f);
	if(addr == 0x0022) return(0x7df);
	if(addr == 0x0023) return(0x5559);
	if(addr == 0x0024) return(0xe4e4);
	if(addr == 0x0025) return(0x101);
	if(addr == 0x0026) return(0x101);
	if(addr == 0x0027) return(0xe4e4);
	if(addr == 0x0028) return(0x101);
	if(addr == 0x0029) return(0x101);
	if(addr == 0x002A) return(0x86);
	if(addr == 0x002B) return(0x10);
	if(addr == 0x002C) return(0xffff);
	if(addr == 0x002E) return(0x0);
	if(addr == 0x002F) return(0x3840);
	if(addr == 0x0081) return(0x0);
	if(addr == 0x0082) return(0x800b);
	if(addr == 0x0084) return(0x400);
	if(addr == 0x0085) return(0x1);
	if(addr == 0x0086) return(0x4901);
	if(addr == 0x0087) return(0x400);
	if(addr == 0x0088) return(0x780);
	if(addr == 0x0089) return(0x20);
	if(addr == 0x008A) return(0x514);
	if(addr == 0x008B) return(0x2100);
	if(addr == 0x008C) return(0x67b);
	if(addr == 0x008D) return(0x0);
	if(addr == 0x0092) return(0x1);
	if(addr == 0x0093) return(0x0);
	if(addr == 0x0094) return(0x0);
	if(addr == 0x0095) return(0x0);
	if(addr == 0x0096) return(0x0);
	if(addr == 0x0097) return(0x0);
	if(addr == 0x0098) return(0x0);
	if(addr == 0x0099) return(0x6565);
	if(addr == 0x009A) return(0x658c);
	if(addr == 0x009B) return(0x6565);
	if(addr == 0x009C) return(0x658c);
	if(addr == 0x009D) return(0x6565);
	if(addr == 0x009E) return(0x658c);
	if(addr == 0x009F) return(0x658c);
	if(addr == 0x00A0) return(0x6565);
	if(addr == 0x00A1) return(0x6565);
	if(addr == 0x00A2) return(0x6565);
	if(addr == 0x00A3) return(0x6565);
	if(addr == 0x00A4) return(0x6565);
	if(addr == 0x00A5) return(0x6565);
	if(addr == 0x00A6) return(0xf);
	if(addr == 0x00A7) return(0x6565);
	if(addr == 0x00a8) return(0x0);
	if(addr == 0x00aa) return(0x0);
	if(addr == 0x00ab) return(0x0);
	if(addr == 0x00ad) return(0x3ff);
	if(addr == 0x00ae) return(0xf000);
	if(addr == 0x0100) return(0x3409);
	if(addr == 0x0101) return(0x7800);
	if(addr == 0x0102) return(0x3180);
	if(addr == 0x0103) return(0xa12);
	if(addr == 0x0104) return(0x88);
	if(addr == 0x0105) return(0x7);
	if(addr == 0x0106) return(0x318c);
	if(addr == 0x0107) return(0x318c);
	if(addr == 0x0108) return(0x9426);
	if(addr == 0x0109) return(0x61c1);
	if(addr == 0x010A) return(0x104c);
	if(addr == 0x010B) return(0x0);
	if(addr == 0x010C) return(0x88fd);
	if(addr == 0x010D) return(0x9e);
	if(addr == 0x010E) return(0x2040);
	if(addr == 0x010F) return(0x3042);
	if(addr == 0x0110) return(0xbf4);
	if(addr == 0x0111) return(0x83);
	if(addr == 0x0112) return(0xc0e6);
	if(addr == 0x0113) return(0x3c3);
	if(addr == 0x0114) return(0x8d);
	if(addr == 0x0115) return(0x9);
	if(addr == 0x0116) return(0x8180);
	if(addr == 0x0117) return(0x280c);
	if(addr == 0x0118) return(0x18c);
	if(addr == 0x0119) return(0x18cb);
	if(addr == 0x011A) return(0x2e02);
	if(addr == 0x011B) return(0x0);
	if(addr == 0x011C) return(0xad43);
	if(addr == 0x011D) return(0x400);
	if(addr == 0x011E) return(0x780);
	if(addr == 0x011F) return(0x3640);
	if(addr == 0x0120) return(0xb9ff);
	if(addr == 0x0121) return(0x3404);
	if(addr == 0x0122) return(0x33f);
	if(addr == 0x0123) return(0x67b);
	if(addr == 0x0124) return(0x0);
	if(addr == 0x0125) return(0x9400);
	if(addr == 0x0126) return(0x12ff);
	if(addr == 0x0200) return(0x81);
	if(addr == 0x0201) return(0x7ff);
	if(addr == 0x0202) return(0x7ff);
	if(addr == 0x0203) return(0x0);
	if(addr == 0x0204) return(0x0);
	if(addr == 0x0205) return(0x0);
	if(addr == 0x0206) return(0x0);
	if(addr == 0x0207) return(0x0);
	if(addr == 0x0208) return(0x0);
	if(addr == 0x0209) return(0x0);
	if(addr == 0x020a) return(0x0);
	if(addr == 0x020C) return(0x0);
	if(addr == 0x0240) return(0x20);
	if(addr == 0x0241) return(0x0);
	if(addr == 0x0242) return(0x0);
	if(addr == 0x0243) return(0x0);
	if(addr == 0x0400) return(0x81);
	if(addr == 0x0401) return(0x7ff);
	if(addr == 0x0402) return(0x7ff);
	if(addr == 0x0403) return(0x0);
	if(addr == 0x0404) return(0x0);
	if(addr == 0x0405) return(0x0);
	if(addr == 0x0406) return(0x0);
	if(addr == 0x0407) return(0x0);
	if(addr == 0x0408) return(0x0);
	if(addr == 0x0409) return(0x0);
	if(addr == 0x040A) return(0x0);
	if(addr == 0x040B) return(0x0);
	if(addr == 0x040C) return(0x0);
	if(addr == 0x040e) return(0x0);
	if(addr == 0x0440) return(0x20);
	if(addr == 0x0441) return(0x0);
	if(addr == 0x0442) return(0x0);
	if(addr == 0x0443) return(0x0);
	if(addr == 0x05c0) return(0x0);
	if(addr == 0x05c1) return(0x0);
	if(addr == 0x05c2) return(0x0);
	if(addr == 0x05c3) return(0x0);
	if(addr == 0x05c4) return(0x0);
	if(addr == 0x05c5) return(0x0);
	if(addr == 0x05c6) return(0x0);
	if(addr == 0x05c7) return(0x0);
	if(addr == 0x05c8) return(0x0);
	if(addr == 0x05c9) return(0x0);
	if(addr == 0x05ca) return(0x0);
	if(addr == 0x05cb) return(0x0);
	if(addr == 0x05cc) return(0x0);
	if(addr == 0x0600) return(0xf00);
	if(addr == 0x0601) return(0x0);
	if(addr == 0x0602) return(0x2000);
	if(addr == 0x0603) return(0x0);
	if(addr == 0x0604) return(0x0);
	if(addr == 0x0605) return(0x0);
	if(addr == 0x0606) return(0x0);
	if(addr == 0x0640) return(0xa0);
	if(addr == 0x0641) return(0x1020);

	return(-1);
}

enum lms7_error_codes {
	LMSE_OK = 0,
	LMSE_OUT_OF_RANGE = 1,
};

#ifndef LMS7_EXTERN_API
#define LMS7_EXTERN_API
#endif

#define LMS7_LOGGING

enum lms7_mac_mode {
	LMS7_CH_NONE = 0,
	LMS7_CH_A = 1,
	LMS7_CH_B = 2,
	LMS7_CH_AB = LMS7_CH_A | LMS7_CH_B,
};

//! direction constants
enum lms7_dir_t {
	LMS7_TX = 1,
	LMS7_RX = 2,
};

// State of xtsp block
struct lms7_tsp_state {
	uint16_t reg_0x0c;
};

struct lms7_filters_state {
	uint8_t rbb0_path:3;
	uint8_t rbb1_path:3;
};

struct lms7_state {
	// Global parameters
	uint32_t fref;

	// for calibration
	int rcal_lpflad_tbb;	// 0x0109
	int rcal_lpfh_tbb;		// 0x0109
	int ccal_lpflad_tbb;	// 0x010A
	int rcal_lpfs5_tbb;		// 0x010A
	int cg_iamp_tbb;		// 0x0108
	int g_pga_rbb;			// 0x0108

	int cfb_tia_rfe;		// 0x0112
	int ccomp_tia_rfe;		// 0x0112
	int rcomp_tia_rfe;		// 0x0114
	int rcc_ctl_lpfl_rbb;	// 0x0117
	int c_ctl_lpfl_rbb;
	int rcc_ctl_lpfh_rbb;	// 0x0116
	int c_ctl_lpfh_rbb;		// 0x0116

	double cgen_freq;		// last written CGEN frequency in Hz

	double sxr_freq;		// last written RX frequency in Hz
	double sxt_freq;		// last written TX frequency in Hz
	unsigned txdiv;

	uint32_t saved_regs[2][200];

	// Frequent registers cache
	uint16_t reg_0x0020;
	uint8_t reg_0x0124[2]; //EN_DIR for SXX/RBB/RFE/TBB/TRF

	// RBB & TBB major states
	struct lms7_filters_state xbbst;

	// Configuration for A&B channels
	struct lms7_tsp_state rxtsp;
	struct lms7_tsp_state txtsp;
};

/* General asyncronous task */
struct lms7_async_task {
	uint8_t task_id;
	uint8_t task_subtaskid;
	uint16_t task_param16;
	uint32_t task_param32;
};

enum lms7_async_tasks {
	LMS7_TASK_CGEN_TUNE,
	LMS7_TASK_SXX_RX_TUNE,
	LMS7_TASK_SXX_TX_TUNE,
};

LMS7_EXTERN_API int lms7_spi_transact(struct lms7_state* s, uint16_t ival, uint32_t* oval);
LMS7_EXTERN_API int lms7_spi_post(struct lms7_state* s, unsigned count, const uint32_t* regs);
#ifdef LMS7_LOGGING
LMS7_EXTERN_API void lms7_log_ex(struct lms7_state* s,
								 const char* function,
								 const char* file,
								 int line_no,
								 const char* fmt, ...) __attribute__ ((format (printf, 5, 6)));
#define lms7_log(s, ...) \
		lms7_log_ex(s, __FUNCTION__, __FILE__, __LINE__, __VA_ARGS__)
#else
#define lms7_log(s, fmt, ...)
#endif


// Initialize cached values
int lms7_reset(struct lms7_state* st);  // Reset internal logic
int lms7_disable(struct lms7_state* st);
int lms7_enable(struct lms7_state* st);

// MAC
int lms7_mac_set(struct lms7_state* st, enum lms7_mac_mode mode);

// CGEN functions
int lms7_cgen_disable(struct lms7_state* st);
int lms7_cgen_tune(struct lms7_state* st, unsigned outfreq, unsigned txdiv_ord);
int lms7_cgen_tune_sync(struct lms7_state* st, unsigned outfreq, unsigned txdiv_ord);


// LML functions
enum lml_mode {
	LML_NORMAL = 0,
	LML_LOOPBACK = 1,
	LML_RXLFSR = 2,
	LML_RD_FCLK = 4,
	LML_DS_HIGH = 8,
};

int lms7_lml_configure(struct lms7_state* st, bool rx_port_1,
					   unsigned txdiv, unsigned rxdiv, enum lml_mode mode);
enum lml_stream_map {
	LML_AI = 0,
	LML_AQ = 1,
	LML_BI = 2,
	LML_BQ = 3,
};
struct lml_map {
	uint8_t l[4];
};

int lms7_lml_set_map(struct lms7_state* st, struct lml_map l1m, struct lml_map l2m);

// RFE functions
enum rfe_path {
	RFE_NONE,
	RFE_LNAH,
	RFE_LNAL,
	RFE_LNAW,
	RFE_LBW,
	RFE_LBL,
};
int lms7_rfe_disable(struct lms7_state* st);
int lms7_rfe_set_path(struct lms7_state* st, enum rfe_path p, bool rfea_en, bool rfeb_en);
int lms7_rfe_set_lna(struct lms7_state* st, unsigned atten, unsigned *paout);
int lms7_rfe_set_lblna(struct lms7_state* st, unsigned attenx4, unsigned *paout);

// RBB functions
enum rbb_path {
	RBB_LBF,
	RBB_HBF,
	RBB_BYP,
	RBB_LB_LBF,
	RBB_LB_HBF,
	RBB_LB_BYP,
};
int lms7_rbb_disable(struct lms7_state* st);
int lms7_rbb_set_path(struct lms7_state* st, enum rbb_path path);
int lms7_rbb_set_pga(struct lms7_state* st, unsigned gain);
int lms7_rbb_set_bandwidth(struct lms7_state* st, unsigned bw);
int lms7_rbb_set_ext(struct lms7_state* st);

// AFE functions
int lms7_afe_ctrl(struct lms7_state* st, bool rxa, bool rxb, bool txa, bool txb);

// SXX
int lms7_sxx_disable(struct lms7_state* st, bool rx);
int lms7_sxx_tune_sync(struct lms7_state* st, bool rx, unsigned lofreq, bool lochen);

// LDO
int lms7_ldo_enable(struct lms7_state* st, bool enable);

// XBUF
int lms7_xbuf_enable(struct lms7_state* st, bool bias, bool enable);

// RXTSP
//  freq
//  decim
//  tsg_const
//  read_rssi
//  dc_corr
//  iq_corr

int lms7_rxtsp_get_rssi(struct lms7_state* st, unsigned mode, uint32_t *orssi);
int lms7_rxtsp_disable(struct lms7_state* st);
int lms7_rxtsp_init(struct lms7_state* st, unsigned decim_ord);
int lms7_rxtsp_cmix(struct lms7_state* st, int32_t freq);
int lms7_rxtsp_tsg_const(struct lms7_state* st, int16_t vi, int16_t vq);
int lms7_rxtsp_tsg_tone(struct lms7_state* st, bool fs, bool div4);
int lms7_rxtsp_dc_corr(struct lms7_state* st, unsigned wnd);

// TXTSP
int lms7_txtsp_disable(struct lms7_state* st);
int lms7_txtsp_init(struct lms7_state* st, unsigned interp_ord);
int lms7_txtsp_cmix(struct lms7_state* st, int32_t freq);
int lms7_txtsp_tsg_const(struct lms7_state* st, int16_t vi, int16_t vq);
int lms7_txtsp_tsg_tone(struct lms7_state* st, bool fs, bool div4);

// TBB
enum tbb_path {
	TBB_BYP,
	TBB_S5,
	TBB_LAD,
	TBB_LADS5,
	TBB_HBF,
};

int lms7_tbb_disable(struct lms7_state* st);
int lms7_tbb_set_path(struct lms7_state* st, enum tbb_path path);
int lms7_tbb_set_bandwidth(struct lms7_state* st, unsigned bw);

// TRF
int lms7_trf_disable(struct lms7_state* st);
int lms7_trf_enable(struct lms7_state* st, bool cha, bool chb);
int lms7_trf_set_pad(struct lms7_state* st, unsigned atten);
int lms7_trf_set_path(struct lms7_state* st, unsigned band);

// DC
int lms7_dc_init(struct lms7_state* st, bool rxaen, bool rxben, bool txaen, bool txben);
int lms7_dc_start(struct lms7_state* st, bool rxa, bool rxb, bool txa, bool txb);

// Helper functions

struct vco_nint_nfrac {
	unsigned nint;
	unsigned frac;
};
struct vco_nint_nfrac lms7_pll_calc(unsigned fref, unsigned vco);

// Calibration API
enum vco_cmp {
	VCO_CMP_LOW = 0,
	VCO_CMP_FAIL = 1,
	VCO_CMP_OK = 2,
	VCO_CMP_HIGH = 3,
};

int lms7_sxx_get_comp(struct lms7_state* st);
int lms7_cgen_get_comp(struct lms7_state* st);


int lms7_cgen_find_cap(struct lms7_state* st, unsigned start, uint8_t* phi, uint8_t* plo);


#define REG_COUNT(x) (sizeof(x) / sizeof(x[0]))

enum cgen_vco_params {
	CGEN_VCO_MIN = 2000000000U,
	CGEN_VCO_MAX = 2700000000U,
	CGEN_VCO_MID = CGEN_VCO_MIN / 2 + CGEN_VCO_MAX / 2,
	CGEN_VCO_RANGE = CGEN_VCO_MAX - CGEN_VCO_MIN,
};

enum {
	VCAL_LOW = 8,
	VCAL_NORM = 64,
	VCAL_HIGH = 8,
};

int lms7_cal_rxdc(struct lms7_state* st);

/* for calibration */
#ifdef UNUSED
uint16_t lms7_get_spi_bits(struct lms7_state *st,uint16_t address,uint8_t msb,uint8_t lsb);
int lms7_set_spi_modify_bits(struct lms7_state *st,uint16_t address,uint8_t msb,uint8_t lsb,uint16_t value);
#endif
int lms7_modify_spi_reg_bits(struct lms7_state *st,uint16_t addr,uint8_t bits,uint16_t new_bits_data);
uint16_t lms7_get_spi_reg_bits(struct lms7_state *st,uint16_t addr,uint8_t bits);
void lms7_store_register(struct lms7_state *st);
void lms7_restore_register(struct lms7_state *st);
int lms7_tbb_set_filter_bw(struct lms7_state *st,enum lms7_mac_mode mode,double bw);
void lms7_tbb_apply_calibration(struct lms7_state *st,int path);
int lms7_rbb_set_filter_bw(struct lms7_state *st,enum lms7_mac_mode mode,double bw);
void lms7_rbb_apply_calibration(struct lms7_state *st,int path);

#endif //LIBLMS7002MC_H
