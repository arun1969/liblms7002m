/*
 * lms7002m compact library source file
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
#include "liblms7002m.h"
#include "lms7002m_defs.h"
#include <math.h>

/************************************************************************
 * general helper functions
 ************************************************************************/

struct vco_nint_nfrac lms7_pll_calc(unsigned fref, unsigned vco)
{
	struct vco_nint_nfrac res;
	res.nint = vco / fref;
	res.frac = (vco - res.nint * fref) * ((uint64_t)1 << 20) / fref;
	return res;
}



// Two implimentation
// Internal definitions
// 
// Tune
// Best VCO sel
// [ MIN; MAX ]
// [ MIN/k; MAX/k ]  => [ (MIN+MAX)/2/k ; (MAX-MIN)/2/k ]
// distance => ((MIN+MAX)/2/k -x ) / (MAX-MIN)/2/k
//
// D = ((MIN + MAX) - 2*k*x ) / (MAX-MIN)
//
// 
// (MIM+MAX) - 2*k*x = -(MIN+MAX) + 2*(k+1)*x
// (MIN+MAX) = (2*k + 1)*x
//
// x = (MIN+MAX)/(2*k+1)
//
// z + 1 = (MIN + MAX) / x / 2
//
// z => (MIN + MAX) / x / 2 +- 2

/************************************************************************
 * Reset & initialization
 ************************************************************************/

int lms7_reset(struct lms7_state* st)
{
	int res;
	uint32_t reg_0x0020, reg_0x002e;

	// Reset all parts in LMS
	reg_0x0020 = MAKE_LMS7002_0x0020(0,0,0,0,0,0,0,0,0,0,0,0,0,0,LMS7_CH_AB);
	res = lms7_spi_post(st, 1, &reg_0x0020);
	if (res)
		return res;

	reg_0x0020 = MAKE_LMS7002_0x0020(1,1,1,1,1,1,1,1,1,1,1,1,1,1,LMS7_CH_AB);
	res = lms7_spi_post(st, 1, &reg_0x0020);
	if (res)
		return res;

	//Enable MIMO by default
	reg_0x002e = MAKE_LMS7002_0x002E(0);
	res = lms7_spi_post(st, 1, &reg_0x002e);
	if (res)
		return res;

	st->reg_0x0020 = (uint16_t)reg_0x0020;
	return 0;
}

int lms7_disable(struct lms7_state* st)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x0081( 0, 0, 0, 0 ),
		MAKE_LMS7002_0x00A6( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0),
	};

	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_enable(struct lms7_state* st)
{
	int res = lms7_reset(st);
	if (res)
		return res;

	res = lms7_ldo_enable(st, true);
	if (res)
		return res;

	res = lms7_xbuf_enable(st, false, true);
	if (res)
		return res;

	// ENDIR: All things are enabled!
	uint32_t regs[] = {
		MAKE_LMS7002_0x0081( 1, 1, 1, 1 ),
	};
	res = lms7_spi_post(st, REG_COUNT(regs), regs);
	if (res)
		return res;

	//Check lime version
	uint32_t ver;
	res = lms7_spi_transact(st, LMS7002M_0x002F, &ver);
	if (res)
		return res;

	lms7_log(st, "LMS VER:%d REV:%d MASK:%d (%04x)",
			 GET_LMS7002_LML_VER(ver),
			 GET_LMS7002_LML_REV(ver),
			 GET_LMS7002_LML_MASK(ver), ver);

	if (GET_LMS7002_LML_VER(ver) != 7)
		return -1;
	if (GET_LMS7002_LML_REV(ver) != 1)
		return -1;

	st->reg_0x0124[0] = 0;
	st->reg_0x0124[1] = 0;
	st->cbst[0].rfe_tia_g = 3;
	st->cbst[1].rfe_tia_g = 3;
	st->rxbw_calibrated[0] = false;
	st->rxbw_calibrated[1] = false;
	st->txbw_calibrated[0] = false;
	st->txbw_calibrated[1] = false;
	return 0;
}

#define MAKE_WR_REG(reg, val) \
	(((reg) << 16) | 0x80000000 | (val))

/************************************************************************
 * MAC configuration
 ************************************************************************/
int lms7_mac_set(struct lms7_state* st, enum lms7_mac_mode mode)
{
	uint32_t reg_0x0020;
	uint32_t macreg =
			(mode == LMS7_CH_AB) ? LMS7002_OPT_MAC_CHAB :
			(mode == LMS7_CH_B) ? LMS7002_OPT_MAC_CHB :
			(mode == LMS7_CH_A) ? LMS7002_OPT_MAC_CHA :
								  LMS7002_OPT_MAC_NONE;

	// Already in needed mode?
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) == macreg)
		return 0;

	st->reg_0x0020 = (uint16_t)((~(LMS7002_LML_MAC_MSK << LMS7002_LML_MAC_OFF) & st->reg_0x0020) |
			(macreg << LMS7002_LML_MAC_OFF));
	reg_0x0020 = MAKE_WR_REG(LMS7002M_0x0020, st->reg_0x0020);
	return lms7_spi_post(st, 1, &reg_0x0020);
}


/************************************************************************
 * LDO configuration
 ************************************************************************/
int lms7_ldo_enable(struct lms7_state* st, bool enable)
{
	unsigned e = (enable) ? 1 : 0;
	uint32_t lod_regs[] = {
		MAKE_LMS7002_0x0092( 0, 0, 0, 0, e, e, 0, e, 0, 0, 0, e, 0, e, 0, e),
		MAKE_LMS7002_0x0093( 0, 0, 0, 0, 0, 0, 0, e, e, 0, e, e, 0, 0, 0, e),
		MAKE_LMS7002_0x0095( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ),
		MAKE_LMS7002_0x0096( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ),
		MAKE_LMS7002_0x00A6( 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1),
		MAKE_LMS7002_0x00A1( 101, 101 ),
		MAKE_LMS7002_0x00A4( 101, 101 ),
	};
	return lms7_spi_post(st, REG_COUNT(lod_regs), lod_regs);
}

/************************************************************************
 * XBUF configuration
 ************************************************************************/
int lms7_xbuf_enable(struct lms7_state* st, bool bias, bool enable)
{
	unsigned e = (enable) ? 1 : 0;
	unsigned b = bias ? 1 : 0;
	uint32_t reg_0x0085 = MAKE_LMS7002_0x0085( b, b, 0, 0, e, e, 0, 0, e );

	return lms7_spi_post(st, 1, &reg_0x0085);
}

/************************************************************************
 * CGEN configuration
 ************************************************************************/
int lms7_cgen_disable(struct lms7_state* st)
{
	uint32_t reg_0x0086 = MAKE_LMS7002_0x0086(0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0);
	return lms7_spi_post(st, 1, &reg_0x0086);
}


int lms7_cgen_get_comp(struct lms7_state* st)
{
	uint32_t reg;
	int res = lms7_spi_transact(st, LMS7002M_0x008C, &reg);
	if (res)
		return res;

	return (int)((GET_LMS7002_CGEN_VCO_CMPHO_CGEN(reg) << 1) |
				  GET_LMS7002_CGEN_VCO_CMPLO_CGEN(reg));
}

int lms7_cgen_find_cap(struct lms7_state* st, unsigned start, uint8_t* phi, uint8_t* plo)
{
	int i;
	int lo = 0, hi = -1;
	int res;

	// Using binary search to find lowest range
	if (start > 255) {
		i = 128;
		for (int j = 6; j >= 0; j--) {
			uint32_t cgen_regs[] = { MAKE_LMS7002_0x008B(15, (unsigned)i, 0) };
			res = lms7_spi_post(st, REG_COUNT(cgen_regs), cgen_regs);
			if (res)
				return res;

			switch ((res = lms7_cgen_get_comp(st))) {
			case VCO_CMP_OK:
			case VCO_CMP_HIGH:
				i -= (1 << j);
				break;
			case VCO_CMP_LOW:
				i += (1 << j);
				break;
			case VCO_CMP_FAIL:
				return -1;
			default:
				return res;
			}
		}
		lo = i;
		lms7_log(st, "CGEN: binary result: %d", i);
	} else {
		i = (int)start;
	}

	for (; i < 256; i++) {
		uint32_t cgen_regs[] = { MAKE_LMS7002_0x008B(15, (unsigned)i, 0) };
		res = lms7_spi_post(st, REG_COUNT(cgen_regs), cgen_regs);
		if (res)
			return res;
		// delay for at least few uS

		switch ((res = lms7_cgen_get_comp(st))) {
		case VCO_CMP_OK:
			hi = i;
			break;
		case VCO_CMP_HIGH:
			if (hi == -1) {
				hi = (i == 0) ? 0 : i - 1;
			}
			goto find_high;
		case VCO_CMP_LOW:
			lo = i + 1;
			break;
		case VCO_CMP_FAIL:
			return -1;
		default:
			return res;
		}
	}

find_high:
	if (hi == -1)
		hi = 0;

	*phi = (uint8_t)hi;
	*plo = (uint8_t)lo;
	return 0;
}

int lms7_cgen_tune(struct lms7_state* st, unsigned outfreq, unsigned txdiv_ord)
{
	// VCO div selection
	unsigned kx2 = (CGEN_VCO_MID / outfreq + 1) >> 1;
	if (kx2 < 1)
		kx2 = 1;
	else if (kx2 > 256)
		kx2 = 256;

	unsigned vcox2 = kx2 * outfreq;
	struct vco_nint_nfrac vc = lms7_pll_calc(st->fref, vcox2 * 2);

	if ((CGEN_VCO_MIN/2 > vcox2) || (CGEN_VCO_MAX/2 < vcox2)) {
		// Out of range
		lms7_log(st, "CGEN: VCO/2=%u is out of range, VCO may not lock!",
				vcox2);
	}

	lms7_log(st, "CGEN: VCO/2=%u k/2=%u int=%u frac=%u",
			vcox2, kx2, vc.nint, vc.frac);

	uint32_t cgen_regs[] = {
		MAKE_LMS7002_0x0086(0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1),
		MAKE_LMS7002_0x0087(vc.frac),
		MAKE_LMS7002_0x0088(vc.nint - 1, vc.frac >> 16),
		MAKE_LMS7002_0x0089(0, 0, 0, txdiv_ord, kx2 - 1, 0),
		MAKE_LMS7002_0x008B(15, 128, 0),
	};

	int res = lms7_spi_post(st, REG_COUNT(cgen_regs), cgen_regs);
	if (res)
		return res;

	return 0;
}


int lms7_cgen_tune_sync(struct lms7_state* st, unsigned outfreq, unsigned txdiv_ord)
{
	int res = lms7_cgen_tune(st, outfreq, txdiv_ord);
	if (res < 0)
		return res;

	// 20uS settelment time
	usleep(20);

	res = lms7_cgen_get_comp(st);
	if (res < 0)
		return res;

	if (res != VCO_CMP_OK) {
		// Sweep over CAP values
		usleep(20);

		uint8_t hi, lo;
		res = lms7_cgen_find_cap(st, (unsigned)-1, &hi, &lo);
		if (res < 0)
			return res;

		if (hi <= lo) {
			lms7_log(st, "CGEN: Can't find sutable VCO cap!");
			res = -1;
			goto tune_failed;
		}

		lms7_log(st, "CGEN: Retuned [%d:%d] -> %d", lo, hi, (hi+lo)/2u);
		uint32_t cgen_regs[] = {
			MAKE_LMS7002_0x008B(15, (hi+lo)/2u, 0),
		};
		res = lms7_spi_post(st, REG_COUNT(cgen_regs), cgen_regs);
		if (res < 0)
			return res;
	}

	//TODO: Disable COMPARATOR power
	return 0;

tune_failed:
	lms7_cgen_disable(st);
	return res;

}

/************************************************************************
 * SXX configuration (RX -> CHA; TX -> CHB)
 ************************************************************************/
enum sxx_vco_params {
	SXX_VCOL_MIN = 1900000000U,
	SXX_VCOL_MAX = 2611000000U,
	SXX_VCOL_MID = SXX_VCOL_MIN / 2 + SXX_VCOL_MAX / 2,
	SXX_VCOL_RANGE = SXX_VCOL_MAX - SXX_VCOL_MIN,

	SXX_VCOM_MIN = 2481000000U,
	SXX_VCOM_MAX = 3377000000U,
	SXX_VCOM_MID = SXX_VCOM_MIN / 2 + SXX_VCOM_MAX / 2,
	SXX_VCOM_RANGE = SXX_VCOM_MAX - SXX_VCOM_MIN,

	SXX_VCOH_MIN = 3153000000U,
	SXX_VCOH_MAX = 3857000000U,
	SXX_VCOH_MID = SXX_VCOH_MIN / 2 + SXX_VCOH_MAX / 2,
	SXX_VCOH_RANGE = SXX_VCOH_MAX - SXX_VCOH_MIN,
};

int lms7_sxx_get_comp(struct lms7_state* st)
{
	uint32_t reg;
	int res = lms7_spi_transact(st, LMS7002M_0x0123, &reg);
	if (res)
		return res;

	return (int)((GET_LMS7002_SXX_VCO_CMPHO(reg) << 1) |
					GET_LMS7002_SXX_VCO_CMPLO(reg));
}

int lms7_sxx_find_cap(struct lms7_state* st, unsigned start, unsigned vcono,
					  uint8_t* phi, uint8_t* plo)
{
	int i;
	int lo = 0, hi = -1;
	int res;

	// Using binary search to find lowest range
	if (start > 255) {
		i = 128;
		for (int j = 6; j >= 0; j--) {
			uint32_t sxx_regs[] = { MAKE_LMS7002_0x0121(16, (unsigned)i, vcono, 0) };
			res = lms7_spi_post(st, REG_COUNT(sxx_regs), sxx_regs);
			if (res)
				return res;

			res = lms7_sxx_get_comp(st);
			switch (res) {
			case VCO_CMP_OK:
			case VCO_CMP_HIGH:
				i -= (1 << j);
				break;
			case VCO_CMP_LOW:
				i += (1 << j);
				break;
			case VCO_CMP_FAIL:
				return -1;
			default:
				return res;
			}
		}
		lo = i;
		lms7_log(st, "SXX: binary result: %d", i);
	} else {
		i = (int)start;
	}

	for (; i < 256; i++) {
		uint32_t sxx_regs[] = { MAKE_LMS7002_0x0121(16, (unsigned)i, vcono, 0) };
		res = lms7_spi_post(st, REG_COUNT(sxx_regs), sxx_regs);
		if (res)
			return res;
		// delay for at least few uS

		res = lms7_sxx_get_comp(st);
		switch (res) {
		case VCO_CMP_OK:
			hi = i;
			break;
		case VCO_CMP_HIGH:
			if (hi == -1) {
				hi = (i == 0) ? 0 : i - 1;
			}
			goto find_high;
		case VCO_CMP_LOW:
			lo = i + 1;
			break;
		case VCO_CMP_FAIL:
			return -1;
		default:
			return res;
		}
	}

find_high:
	if (hi == -1)
		hi = 0;

	*phi = (uint8_t)hi;
	*plo = (uint8_t)lo;
	return 0;
}

int lms7_sxx_disable(struct lms7_state* st, bool rx)
{
	st->reg_0x0124[rx ? 0 : 1] &= ~(1u << LMS7002_SXX_EN_DIR_SXX_OFF);
	uint32_t sxx_regs[] = {
		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) |
									 (rx ? LMS7002_OPT_MAC_CHA : LMS7002_OPT_MAC_CHB)),
		MAKE_WR_REG(LMS7002M_0x0124, st->reg_0x0124[rx ? 0 : 1]),
	};
	return lms7_spi_post(st, REG_COUNT(sxx_regs), sxx_regs);
}

static int _sxx_set_vcon(struct lms7_state* st, unsigned i,
						 unsigned sxx_vco, bool lochen)
{
	struct vco_nint_nfrac vc;
	vc = lms7_pll_calc(i == 3 ? st->fref / 2 : st->fref, sxx_vco);

	lms7_log(st, "SXX: VCO%u N=%d frac=%d", i, vc.nint, vc.frac);
	uint32_t sxx_pll_regs[] = {
		MAKE_LMS7002_0x011C(1, 0, 0, 0, 1, /*i == 0 ? 0 :*/ 1u,
							0, 1, 0,
							(lochen) ? 0 : 1u, //PD_LOCH_T2RBUF,
							0, 0, 0, 0, 0, 1),
		MAKE_LMS7002_0x011D(vc.frac),
		MAKE_LMS7002_0x011E(vc.nint - 4, vc.frac >> 16),
	};
	return lms7_spi_post(st, REG_COUNT(sxx_pll_regs), sxx_pll_regs);
}

int lms7_sxx_tune_sync(struct lms7_state* st, bool rx, unsigned lofreq,
					   bool lochen)
{
	unsigned divh = 0;
	unsigned sxx_vco = lofreq;
	int res = lms7_mac_set(st, (rx) ? LMS7_CH_A : LMS7_CH_B);
	if (res < 0)
		return res;

	if (sxx_vco > SXX_VCOH_MAX) {
		lms7_log(st, "SX%c: LO=%d is out of rnage",
				 (rx ? 'R' : 'T'), lofreq);
		return -1;
	}
	while (sxx_vco < SXX_VCOL_MIN) {
		if (divh >= 6) {
			// Unable to deliver frequency
			lms7_log(st, "SX%c: LO=%d is out of rnage (VCO=%u)",
					 (rx ? 'R' : 'T'), lofreq, sxx_vco);
			return -1;
		}
		divh++;
		sxx_vco <<= 1;
	}

	st->reg_0x0124[rx ? 0 : 1] |= 1u << LMS7002_SXX_EN_DIR_SXX_OFF;
	uint32_t sxx_regsq[] = {
		MAKE_WR_REG(LMS7002M_0x0124, st->reg_0x0124[rx ? 0 : 1]),
		MAKE_LMS7002_0x0120(204, 192),
		MAKE_LMS7002_0x0122(0, 20, 20) | (1u<<13),
		MAKE_LMS7002_0x011C(1, //RESET_N
							0, //SPDUP_VCO
							0, //BYPLDO_VCO
							0, //EN_COARSEPLL
							1, //CURLIM_VCO
							1u, //EN_DIV2_DIVPROG
							0, //EN_INTONLY_SDM
							1, //EN_SDM_CLK
							0, //PD_FBDIV
							(lochen) ? 0 : 1u, //PD_LOCH_T2RBUF
							0, //PD_CP
							0, //PD_FDIV
							0, //PD_SDM
							0, //PD_VCO_COMP
							0, //PD_VCO
							1),
		MAKE_LMS7002_0x011F(3, 3, 6, 0, 0, 0, 0),
	};
	res = lms7_spi_post(st, REG_COUNT(sxx_regsq), sxx_regsq);
	if (res)
		return res;

	//VCO selection
	bool vcol_ok = (SXX_VCOL_MIN < sxx_vco) && (sxx_vco < SXX_VCOL_MAX);
	bool vcom_ok = (SXX_VCOM_MIN < sxx_vco) && (sxx_vco < SXX_VCOM_MAX);
	bool vcoh_ok = (SXX_VCOH_MIN < sxx_vco) && (sxx_vco < SXX_VCOH_MAX);
	bool vcop_ok = (SXX_VCOH_MIN/2 < sxx_vco) && (sxx_vco < SXX_VCOH_MAX/2) && (divh < 6);

	lms7_log(st, "SX%c: initial VCO=%u DIVH=%u VCOs:%d%d%d%d",
			 (rx ? 'R' : 'T'), sxx_vco, divh,
			 vcol_ok, vcom_ok, vcoh_ok, vcop_ok);

	// TODO Calculate cap value using LUT and temperature
	static const unsigned vcono[4] = { 0, 1, 2, 2 };
	bool vcoit[4] = { vcol_ok, vcom_ok, vcoh_ok, vcop_ok };
	int pcap = -1;
	unsigned pvco_idx = 0;
	unsigned good = 0;

	for (unsigned t = 0; t < 8; t++) {
		pcap = -1;
		pvco_idx = 0;
		uint8_t plo = 0, phi = 0;
		for (unsigned i = 0; i < 4; i++) {
			if (!vcoit[i])
				continue;

			res = _sxx_set_vcon(st, i, sxx_vco, lochen);
			if (res)
				return res;

			res = lms7_sxx_find_cap(st, (unsigned)-1, vcono[i], &phi, &plo);
			if (res != 0)
				return res;

			if (phi < plo)
				continue;

			int mid = ((int)plo + phi) / 2;
			lms7_log(st, "SX%c: VCO%d [%d;%d] -> %d",
					 (rx ? 'R' : 'T'), i, plo, phi, mid);

			if (pcap == -1 || pcap > mid) {
				pcap = mid;
				pvco_idx = i;
			} if (i == 3 && pcap != -1 && pvco_idx != 3) {
				lms7_log(st, "SX%c: restore to VCO%d",
						 (rx ? 'R' : 'T'), pvco_idx);

				//Restore previous PLL settings
				res = _sxx_set_vcon(st, pvco_idx, sxx_vco, lochen);
				if (res)
					return res;
			}
		}
		if (pcap != -1 && !(plo == 0 && phi == 0)) {
			good++;

			if (good > 1)
				break;
		} else {
			usleep(1000);
		}
	}

	if (pcap == -1) {
		lms7_log(st, "SX%c: Unable to tune to VCO=%u",
				 (rx ? 'R' : 'T'), sxx_vco);
		return -2;
	}

	if (pvco_idx == 3) {
		divh++;
	}

	uint32_t sxx_regs[] = {
		MAKE_LMS7002_0x011F(3, 3, divh, 0, 0, 0, 0),
		MAKE_LMS7002_0x0121(16, (unsigned)pcap, vcono[pvco_idx], 0),
	};
	res = lms7_spi_post(st, REG_COUNT(sxx_regs), sxx_regs);
	if (res)
		return res;

	return 0;
}

/************************************************************************
 * LML configuration
 ************************************************************************/

int lms7_lml_configure(struct lms7_state* st, bool rx_port_1,
					   unsigned txdiv, unsigned rxdiv,
					   enum lml_mode mode)
{
	unsigned lml1_rx = (rx_port_1) ? 1 : 0;
	unsigned txmclk = (txdiv <= 1) ? LMS7002_OPT_MCLK1SRC_TXTSPCLKA : LMS7002_OPT_MCLK1SRC_TXTSPCLKA_DIV;
	unsigned rxmclk = (rxdiv <= 1) ? LMS7002_OPT_MCLK1SRC_RXTSPCLKA : LMS7002_OPT_MCLK1SRC_RXTSPCLKA_DIV;

	unsigned rxmux = ((mode & 0x3) == LML_NORMAL) ? LMS7002_OPT_RX_MUX_RXTSP :
					 ((mode & 0x3) == LML_LOOPBACK) ? LMS7002_OPT_RX_MUX_TXFIFO :
													  LMS7002_OPT_RX_MUX_LFSR;
	unsigned rdclk = ((mode & LML_RD_FCLK) || (mode == LML_LOOPBACK)) ?
				((lml1_rx) ? LMS7002_OPT_TXWRCLK_MUX_FCLK2 : LMS7002_OPT_TXWRCLK_MUX_FCLK1) :
				((lml1_rx) ? LMS7002_OPT_RXRDCLK_MUX_MCLK1 : LMS7002_OPT_RXRDCLK_MUX_MCLK2);

	uint32_t regs[] = {
		MAKE_LMS7002_0x0022((mode & LML_DS_HIGH) ? 1u : 0, //DIQ2_DS,
							0, //DIQ2_PE,
							0, //IQ_SET_EN_2_PE,
							0, //TXNRX2_PE,
							0, //FCLK2_PE,
							0, //MCLK2_PE,
							(mode & LML_DS_HIGH) ? 1u : 0, //DIQ1_DS,
							0, //DIQ1_PE,
							0, //IQ_SET_EN_1_PE,
							0, //TXNRX1_PE,
							0, //FCLK1_PE,
							0),
		MAKE_LMS7002_0x0023(0, LMS7002_OPT_DIQDIR2_INPUT, 0, LMS7002_OPT_ENABLEDIR2_INPUT,
							0, LMS7002_OPT_DIQDIR1_INPUT, 0, LMS7002_OPT_ENABLEDIR1_INPUT,
							1, //Enable LML
							0,
							(lml1_rx) ? 0 : 1u,
							LMS7002_OPT_LML2_MODE_TRXIQ,
							0,
							(lml1_rx) ? 1u : 0,
							LMS7002_OPT_LML1_MODE_TRXIQ ),

		MAKE_LMS7002_0x002A(rxmux,
							(lml1_rx) ? LMS7002_OPT_TX_MUX_PORT2 : LMS7002_OPT_TX_MUX_PORT1,
							LMS7002_OPT_TXRDCLK_MUX_TXTSPCLK,
							(lml1_rx) ? LMS7002_OPT_TXWRCLK_MUX_FCLK2 : LMS7002_OPT_TXWRCLK_MUX_FCLK1,
							rdclk, //(lml1_rx) ? LMS7002_OPT_RXRDCLK_MUX_MCLK1 : LMS7002_OPT_RXRDCLK_MUX_MCLK2,
							LMS7002_OPT_RXWRCLK_MUX_RXTSPCLK ),

		MAKE_LMS7002_0x002B(0, 0, 0, 0,
							(rx_port_1) ? txmclk : rxmclk,
							(rx_port_1) ? rxmclk : txmclk,
							(txdiv > 1) ? 1u : 0,
							(rxdiv > 1) ? 1u : 0),

		MAKE_LMS7002_0x002C( txdiv / 2u - 1u, rxdiv / 2u - 1u )
	};

	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

struct lml_positions {
	uint8_t aip;
	uint8_t aqp;
	uint8_t bip;
	uint8_t bqp;
};

static struct lml_positions _lms7_lml_fill_positions(struct lml_map m)
{
	struct lml_positions p = {0, 0, 0, 0};
	for (unsigned i = 0; i < 4; i++) {
		if (m.l[i] == LMS7002_OPT_LML1_S0S_AQ)
			p.aqp = i;
		else if (m.l[i] == LMS7002_OPT_LML1_S0S_AI)
			p.aip = i;
		else if (m.l[i] == LMS7002_OPT_LML1_S0S_BQ)
			p.bqp = i;
		else if (m.l[i] == LMS7002_OPT_LML1_S0S_BI)
			p.bip = i;
	}
	return p;
}

int lms7_lml_set_map(struct lms7_state* st, struct lml_map l1m, struct lml_map l2m)
{
	struct lml_positions l1p = _lms7_lml_fill_positions(l1m);
	struct lml_positions l2p = _lms7_lml_fill_positions(l2m);

	uint32_t regs[] = {
		MAKE_LMS7002_0x0024(l1m.l[3], l1m.l[2], l1m.l[1], l1m.l[0],
							l1p.bqp, l1p.bip, l1p.aqp, l1p.aip),
		MAKE_LMS7002_0x0027(l2m.l[3], l2m.l[2], l2m.l[1], l2m.l[0],
							l2p.bqp, l2p.bip, l2p.aqp, l2p.aip),
	};

	return lms7_spi_post(st, REG_COUNT(regs), regs);
}


static int _sxx_update_endir(struct lms7_state* st)
{
	lms7_log(st, "0x0124[%02x, %02x]", st->reg_0x0124[0], st->reg_0x0124[1]);

	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHA),
		MAKE_WR_REG(LMS7002M_0x0124, st->reg_0x0124[0]),

		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHB),
		MAKE_WR_REG(LMS7002M_0x0124, st->reg_0x0124[1]),

		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020)),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

/************************************************************************
 * RFE configuration
 ************************************************************************/
int lms7_rfe_disable(struct lms7_state* st)
{
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHA) {
		st->reg_0x0124[0] &= ~(1u << LMS7002_SXX_EN_DIR_RFE_OFF);
	}
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHB) {
		st->reg_0x0124[1] &= ~(1u << LMS7002_SXX_EN_DIR_RFE_OFF);
	}

	return _sxx_update_endir(st);
}

int lms7_rfe_set_path(struct lms7_state* st, enum rfe_path p, bool rfea_en, bool rfeb_en)
{
	unsigned path = (p == RFE_LNAH || p == RFE_LBH) ? LMS7002_OPT_SEL_PATH_RFE_LNAH :
					(p == RFE_LNAL || p == RFE_LBL) ? LMS7002_OPT_SEL_PATH_RFE_LNAL :
					(p == RFE_LNAW || p == RFE_LBW) ? LMS7002_OPT_SEL_PATH_RFE_LNAW :
													  LMS7002_OPT_SEL_PATH_RFE_NONE;
	unsigned mimo = ((GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHAB) == LMS7002_OPT_MAC_CHAB);
	unsigned chb = ((GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHAB) == LMS7002_OPT_MAC_CHB);

	bool do_en = false;
	if (rfea_en || rfeb_en) {
		if (!(st->reg_0x0124[0] & (1u << LMS7002_SXX_EN_DIR_RFE_OFF))) {
			st->reg_0x0124[0] |= 1u << LMS7002_SXX_EN_DIR_RFE_OFF;
			do_en = true;
		}
	}
	if (rfeb_en) {
		if (!(st->reg_0x0124[1] & (1u << LMS7002_SXX_EN_DIR_RFE_OFF))) {
			st->reg_0x0124[1] |= 1u << LMS7002_SXX_EN_DIR_RFE_OFF;
			do_en = true;
		}
	}
	if (do_en) {
		int res = _sxx_update_endir(st);
		if (res)
			return res;

		if (!rfea_en) {
			uint32_t regs[] = {
				MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHA),
				MAKE_LMS7002_0x010C(8, //CDC_I_RFE,
									8, //CDC_Q_RFE,
									1u, // PD_LNA_RFE,
									1u, // PD_RLOOPB_1_RFE,
									1u, // PD_RLOOPB_2_RFE,
									1, // PD_MXLOBUF_RFE,
									1, // PD_QGEN_RFE,
									1, // PD_RSSI_RFE,
									1, // PD_TIA_RFE,
									1u //EN_G_RFE
				),
				MAKE_LMS7002_0x010D(LMS7002_OPT_SEL_PATH_RFE_NONE, // SEL_PATH_RFE,
									0, //EN_DCOFF_RXFE_RFE,
									0, // EN_INSHSW_LB1_RFE,
									0, // EN_INSHSW_LB2_RFE,
									0, // EN_INSHSW_L_RFE,
									0, // EN_INSHSW_W_RFE,
									rfeb_en ? 1u : 0 //EN_NEXTRX_RFE
				),
				MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020)),
			};
			res = lms7_spi_post(st, REG_COUNT(regs), regs);
			if (res)
				return res;
		}
	}

	uint32_t regs[] = {
		//Channel A
		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHA),
		MAKE_LMS7002_0x010C( 8, //CDC_I_RFE,
							 8, //CDC_Q_RFE,
							 rfea_en && (p == RFE_LNAH || p == RFE_LNAW || p == RFE_LNAL) ? 0u : 1u, // PD_LNA_RFE,
							 rfea_en && (p == RFE_LBW || p == RFE_LBH) ? 0u : 1u, // PD_RLOOPB_1_RFE,
							 rfea_en && (p == RFE_LBL) ? 0u : 1u, // PD_RLOOPB_2_RFE,
							 (rfea_en) ? 0 : 1u, // PD_MXLOBUF_RFE,
							 (rfea_en) ? 0 : 1u, // PD_QGEN_RFE,
							 1, // PD_RSSI_RFE,
							 (rfea_en) ? 0 : 1u, // PD_TIA_RFE,
							 (p == RFE_NONE) ? 0u : 1u //EN_G_RFE
		),
		MAKE_LMS7002_0x010D( path, // SEL_PATH_RFE,
							 1, //EN_DCOFF_RXFE_RFE,
							 !rfea_en || (p == RFE_LBW || p == RFE_LBH) ? 0u : 1u, // EN_INSHSW_LB1_RFE,
							 !rfea_en || (p == RFE_LBL) ? 0u : 1u, // EN_INSHSW_LB2_RFE,
							 !rfea_en || (p == RFE_LNAL) ? 0u : 1u, // EN_INSHSW_L_RFE,
							 !rfea_en || (p == RFE_LNAW) ? 0u : 1u, // EN_INSHSW_W_RFE,
							 rfeb_en ? 1u : 0 //EN_NEXTRX_RFE
		),
		MAKE_LMS7002_0x010F((p == RFE_LBW || p == RFE_LBH || p == RFE_LBL) ? 3 : 0, 2, 2),
		MAKE_WR_REG(LMS7002M_0x0020, st->reg_0x0020),

		// Channel B
		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHB),
		MAKE_LMS7002_0x010C( 8, //CDC_I_RFE,
							 8, //CDC_Q_RFE,
							 (p == RFE_LNAH || p == RFE_LNAW || p == RFE_LNAL) ? 0u : 1u, // PD_LNA_RFE,
							 (p == RFE_LBW || p == RFE_LBH) ? 0u : 1u, // PD_RLOOPB_1_RFE,
							 (p == RFE_LBL) ? 0u : 1u, // PD_RLOOPB_2_RFE,
							 0, // PD_MXLOBUF_RFE,
							 0, // PD_QGEN_RFE,
							 1, // PD_RSSI_RFE,
							 0, // PD_TIA_RFE,
							 (p == RFE_NONE || !rfeb_en) ? 0u : 1u //EN_G_RFE
		),
		MAKE_LMS7002_0x010D( path, // SEL_PATH_RFE,
							 1, //EN_DCOFF_RXFE_RFE,
							 (p == RFE_LBW || p == RFE_LBH) ? 0u : 1u, // EN_INSHSW_LB1_RFE,
							 (p == RFE_LBL) ? 0u : 1u, // EN_INSHSW_LB2_RFE,
							 (p == RFE_LNAL) ? 0u : 1u, // EN_INSHSW_L_RFE,
							 (p == RFE_LNAW) ? 0u : 1u, // EN_INSHSW_W_RFE,
							 0 //EN_NEXTRX_RFE
		),
		MAKE_LMS7002_0x010F((p == RFE_LBW || p == RFE_LBH || p == RFE_LBL) ? 3 : 0, 2, 2),
		MAKE_WR_REG(LMS7002M_0x0020, st->reg_0x0020),
	};

	return lms7_spi_post(st,
						 (mimo) ? REG_COUNT(regs) : (REG_COUNT(regs)/2),
						 (chb) ? regs + (REG_COUNT(regs)/2) : regs);
}

static unsigned _lowerbound(uint8_t* a, uint8_t v, unsigned sz)
{
	unsigned i = sz / 2, j = sz / 4;
	for (; j != 0; j = j / 2) {
		if (a[i] < v) {
			i -= j;
		} else {
			i += j;
		}
	}
	if (a[i] < v) {
		i--;
	}
	return i;
}

static int _lms7_rfe_update_gains(struct lms7_state* st)
{
	unsigned mimo = ((GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHAB) == LMS7002_OPT_MAC_CHAB);
	unsigned chb = ((GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHAB) == LMS7002_OPT_MAC_CHB);

	uint32_t regs[] = {
		// Channel A
		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHA),
		MAKE_LMS7002_0x0113(st->cbst[0].rfe_lna_g, 0, st->cbst[0].rfe_tia_g),
		MAKE_WR_REG(LMS7002M_0x0020, st->reg_0x0020),

		// Channel B
		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHB),
		MAKE_LMS7002_0x0113(st->cbst[1].rfe_lna_g, 0, st->cbst[1].rfe_tia_g),
		MAKE_WR_REG(LMS7002M_0x0020, st->reg_0x0020),
	};

	return lms7_spi_post(st,
						 (mimo) ? REG_COUNT(regs) : (REG_COUNT(regs)/2),
						 (chb) ? regs + (REG_COUNT(regs)/2) : regs);
}

int lms7_rfe_set_lna(struct lms7_state* st, unsigned atten, unsigned *paout)
{
	uint8_t attens[] = {
		30, 30, 27, 24, 21, 18, 15, 12, 9, 6, 5, 4, 3, 2, 1, 0
	};
	unsigned i = _lowerbound(attens, atten, REG_COUNT(attens));
	if (paout) {
		*paout = attens[i];
	}

	lms7_log(st, "RFE: set_lna(%d -> %d) => %d", atten, attens[i], i);

	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHA) {
		st->cbst[0].rfe_lna_g = i;
	}
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHB) {
		st->cbst[1].rfe_lna_g = i;
	}

	return _lms7_rfe_update_gains(st);
}

int lms7_rfe_set_tia(struct lms7_state* st, unsigned atten, unsigned *paout)
{
	const uint8_t attens[] = {
		255, 12, 3, 0
	};
	unsigned i = _lowerbound(attens, atten, REG_COUNT(attens));
	if (paout) {
		*paout = attens[i];
	}

	lms7_log(st, "RFE: set_tia(%d -> %d) => %d", atten, attens[i], i);

	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHA) {
		st->cbst[0].rfe_tia_g = i;
	}
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHB) {
		st->cbst[1].rfe_tia_g = i;
	}

	return _lms7_rfe_update_gains(st);
}

int lms7_rfe_set_lblna(struct lms7_state* st, unsigned attenx4, unsigned *paout)
{
	uint8_t attens[] = {
		160, 96, 68, 56, 44, 36, 30, 25, 20, 16, 12, 10, 6, 4, 2, 0
	};
	unsigned i = _lowerbound(attens, attenx4, REG_COUNT(attens));
	if (paout) {
		*paout = attens[i];
	}

	lms7_log(st, "RFE: set_lblna(%d -> %d) => %d", attenx4, attens[i], i);

	unsigned mimo = ((GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHAB) == LMS7002_OPT_MAC_CHAB);
	unsigned chb = ((GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHAB) == LMS7002_OPT_MAC_CHB);

	uint32_t regs[] = {
		// Channel A
		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHA),
		MAKE_LMS7002_0x0113(1, i, st->cbst[0].rfe_tia_g),
		MAKE_WR_REG(LMS7002M_0x0020, st->reg_0x0020),

		// Channel B
		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHB),
		MAKE_LMS7002_0x0113(1, i, st->cbst[1].rfe_tia_g),
		MAKE_WR_REG(LMS7002M_0x0020, st->reg_0x0020),
	};

	return lms7_spi_post(st,
						 (mimo) ? REG_COUNT(regs) : (REG_COUNT(regs)/2),
						 (chb) ? regs + (REG_COUNT(regs)/2) : regs);
}

/************************************************************************
 * TRF configuration
 ************************************************************************/
int lms7_trf_disable(struct lms7_state* st)
{
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHA) {
		st->reg_0x0124[0] &= ~(1u << LMS7002_SXX_EN_DIR_TRF_OFF);
	}
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHB) {
		st->reg_0x0124[1] &= ~(1u << LMS7002_SXX_EN_DIR_TRF_OFF);
	}

	return _sxx_update_endir(st);
}

int lms7_trf_enable(struct lms7_state* st, bool cha, bool chb)
{
	bool do_en = false;
	if (cha || chb) {
		if (!(st->reg_0x0124[0] & (1u << LMS7002_SXX_EN_DIR_TRF_OFF))) {
			st->reg_0x0124[0] |= 1u << LMS7002_SXX_EN_DIR_TRF_OFF;
			do_en = true;
		}
	}
	if (chb) {
		if (!(st->reg_0x0124[1] & (1u << LMS7002_SXX_EN_DIR_TRF_OFF))) {
			st->reg_0x0124[1] |= 1u << LMS7002_SXX_EN_DIR_TRF_OFF;
			do_en = true;
		}
	}
	if (do_en) {
		int res = _sxx_update_endir(st);
		if (res)
			return res;
	}

	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHA),
		MAKE_LMS7002_0x0100(
			0, //EN_LOWBWLOMX_TMX_TRF,
			(chb /*|| cha*/) ? 1u : 0, //EN_NEXTTX_TRF,
			3, //EN_AMPHF_PDET_TRF,
			1, //LOADR_PDET_TRF,
			1, //PD_PDET_TRF,
			0, //PD_TLOBUF_TRF,
			0, //PD_TXPAD_TRF,
			1  //EN_G_TRF )
		),
		MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 & ~0x3u) | LMS7002_OPT_MAC_CHB),
		MAKE_LMS7002_0x0100(
			0, //EN_LOWBWLOMX_TMX_TRF,
			0, //EN_NEXTTX_TRF,
			3, //EN_AMPHF_PDET_TRF,
			1, //LOADR_PDET_TRF,
			1, //PD_PDET_TRF,
			0, //PD_TLOBUF_TRF,
			0, //PD_TXPAD_TRF,
			1  //EN_G_TRF )
		),
		MAKE_WR_REG(LMS7002M_0x0020, st->reg_0x0020),
	};

	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_trf_set_pad(struct lms7_state* st, unsigned atten)
{
	if (atten > 52)
		atten = 31;
	else if (atten > 10)
		atten = (atten + 10) / 2;

	uint32_t regs[] = {
		MAKE_LMS7002_0x0101(
			3,     //F_TXPAD_TRF,
			3,     //L_LOOPB_TXPAD_TRF,
			atten, //LOSS_LIN_TXPAD_TRF,
			atten, //LOSS_MAIN_TXPAD_TRF,
			0u     //EN_LOOPB_TXPAD_TRF
		),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_trf_set_path(struct lms7_state* st, unsigned band)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x0103(
			(band == 1) ? 1u : 0, //SEL_BAND1_TRF,
			(band == 1) ? 0 : 1u, //SEL_BAND2_TRF,
			16, //LOBIASN_TXM_TRF,
			18  //LOBIASP_TXX_TRF
		),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_trf_set_padlb(struct lms7_state* st, unsigned atten, unsigned lbloss)
{
	if (atten > 52)
		atten = 31;
	else if (atten > 10)
		atten = (atten + 10) / 2;

	if (lbloss > 3)
		lbloss = 3;

	uint32_t regs[] = {
		MAKE_LMS7002_0x0101(
			3,     //F_TXPAD_TRF,
			lbloss,     //L_LOOPB_TXPAD_TRF,
			atten, //LOSS_LIN_TXPAD_TRF,
			atten, //LOSS_MAIN_TXPAD_TRF,
			1u     //EN_LOOPB_TXPAD_TRF
		),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

/************************************************************************
 * TBB configuration
 ************************************************************************/
int lms7_tbb_disable(struct lms7_state* st)
{
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHA) {
		st->reg_0x0124[0] &= ~(1u << LMS7002_SXX_EN_DIR_TBB_OFF);
	}
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHB) {
		st->reg_0x0124[1] &= ~(1u << LMS7002_SXX_EN_DIR_TBB_OFF);
	}

	return _sxx_update_endir(st);
}

int lms7_tbb_set_path(struct lms7_state* st, enum tbb_path path)
{
	bool do_en = false;
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHA) {
		if (!(st->reg_0x0124[0] & (1u << LMS7002_SXX_EN_DIR_TBB_OFF))) {
			st->reg_0x0124[0] |= 1u << LMS7002_SXX_EN_DIR_TBB_OFF;
			do_en = true;
		}
	}
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHB) {
		if (!(st->reg_0x0124[1] & (1u << LMS7002_SXX_EN_DIR_TBB_OFF))) {
			st->reg_0x0124[1] |= 1u << LMS7002_SXX_EN_DIR_TBB_OFF;
			do_en = true;
		}
	}
	if (do_en) {
		int res = _sxx_update_endir(st);
		if (res)
			return res;
	}

	uint32_t regs[] = {
		MAKE_LMS7002_0x0105(
			0, //STATPULSE_TBB,
			0, //LOOPB_TBB,
			(path == TBB_HBF) ? 0 : 1u, //PD_LPFH_TBB,
			0, //PD_LPFIAMP_TBB,
			(path == TBB_LAD || path == TBB_LADS5) ? 0 : 1u, //PD_LPFLAD_TBB,
			(path == TBB_LAD || path == TBB_LADS5 || path == TBB_S5) ? 0 : 1u, //PD_LPFS5_TBB,
			1u //EN_G_TBB
		),
		MAKE_LMS7002_0x010A(
			0, //TSTIN_TBB,
			(path == TBB_S5) ? 1u : 0, //BYPLADDER_TBB,
			16, //CCAL_LPFLAD_TBB,
			76  //RCAL_LPFS5_TBB
		)
	};

	lms7_log(st, "TBB: path %d", path);

	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

// TODO: Improve this code!!!
static int lms7_tbb_set_bandwidth_lad(struct lms7_state* st, unsigned bw)
{
	if (bw < 1000000)
		bw = 1000000;
	else if (bw > 20000000)
		bw = 20000000;

	const double f = bw/1e6;
	const double p1 = 1.29858903647958E-16;
	const double p2 = -0.000110746929967704;
	const double p3 = 0.00277593485991029;
	const double p4 = 21.0384293169607;
	const double p5 = -48.4092606238297;
	int rcal_lpflad_tbb = (int)(f*f*f*f*p1 + f*f*f*p2 + f*f*p3 + f*p4 + p5);

	if (rcal_lpflad_tbb < 0)
		rcal_lpflad_tbb = 0;
	else if (rcal_lpflad_tbb > 255)
		rcal_lpflad_tbb = 255;

	uint32_t regs[] = {
		MAKE_LMS7002_0x0109(0, rcal_lpflad_tbb)
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_tbb_set_bandwidth_lpfh(struct lms7_state* st, unsigned bw)
{
	if (bw < 20000000)
		bw = 20000000;
	else if (bw > 80000000)
		bw = 80000000;

	const double f = bw/1e6;
	const double p1 = 1.10383261611112E-06;
	const double p2 = -0.000210800032517545;
	const double p3 = 0.0190494874803309;
	const double p4 = 1.43317445923528;
	const double p5 = -47.6950779298333;
	int rcal_lpfh_tbb = (int)(f*f*f*f*p1 + f*f*f*p2 + f*f*p3 + f*p4 + p5);

	if (rcal_lpfh_tbb < 0)
		rcal_lpfh_tbb = 0;
	else if (rcal_lpfh_tbb > 255)
		rcal_lpfh_tbb = 255;

	uint32_t regs[] = {
		MAKE_LMS7002_0x0109(rcal_lpfh_tbb, 0)
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_tbb_set_bandwidth(struct lms7_state* st, enum lms7_mac_mode mode, unsigned bw)
{
	int res = 0;
	unsigned cg_iamp_tbb;
	
	if(st->txbw_calibrated[(mode == LMS7_CH_A) ? 0 : 1] == false)
	{
		if(bw < 20000000)
		{
	//		cg_iamp_tbb = (bw + 1000000) / 2000000;
			cg_iamp_tbb = (bw / 1000000) * 2;
			res = lms7_tbb_set_path(st,TBB_LAD);
			if(res)
				return(res);

			res = lms7_tbb_set_bandwidth_lad(st,bw);
		} 
		else
		{
			cg_iamp_tbb = (bw + 500000) / 1000000;
			res = lms7_tbb_set_path(st,TBB_HBF);
			if(res)
				return res;

			res = lms7_tbb_set_bandwidth_lpfh(st, bw);
		}

		if (res)
			return res;

		if (cg_iamp_tbb < 1)
			cg_iamp_tbb = 1;
		else if (cg_iamp_tbb > 63)
			cg_iamp_tbb = 63;

		uint32_t regs[] = {
			MAKE_LMS7002_0x0108(
				cg_iamp_tbb, //CG_IAMP_TBB,
				12, //ICT_IAMP_FRP_TBB,
				12  //ICT_IAMP_GG_FRP_TBB
			),
		};

		res = lms7_spi_post(st, REG_COUNT(regs), regs);
	}

	if (bw < 20000000)
		lms7_tbb_apply_calibration(st,mode,TBB_LAD);
	else
		lms7_tbb_apply_calibration(st,mode,TBB_HBF);

	return(res);
}

/************************************************************************
 * RBB configuration
 ************************************************************************/
int lms7_rbb_disable(struct lms7_state* st)
{
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHA) {
		st->reg_0x0124[0] &= ~(1u << LMS7002_SXX_EN_DIR_RBB_OFF);
	}
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHB) {
		st->reg_0x0124[1] &= ~(1u << LMS7002_SXX_EN_DIR_RBB_OFF);
	}

	return _sxx_update_endir(st);
}

int lms7_rbb_set_path(struct lms7_state* st, enum rbb_path path)
{
	bool do_en = false;
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHA) {
		if (!(st->reg_0x0124[0] & (1u << LMS7002_SXX_EN_DIR_RBB_OFF))) {
			st->reg_0x0124[0] |= 1u << LMS7002_SXX_EN_DIR_RBB_OFF;
			do_en = true;
		}
	}
	if (GET_LMS7002_LML_MAC(st->reg_0x0020) & LMS7002_OPT_MAC_CHB) {
		if (!(st->reg_0x0124[1] & (1u << LMS7002_SXX_EN_DIR_RBB_OFF))) {
			st->reg_0x0124[1] |= 1u << LMS7002_SXX_EN_DIR_RBB_OFF;
			do_en = true;
		}
	}
	if (do_en) {
		int res = _sxx_update_endir(st);
		if (res)
			return res;
	}

	uint32_t regs[] = {
		MAKE_LMS7002_0x0115(
			(path == RBB_LB_HBF) ? 1u : 0,
			(path == RBB_LB_LBF) ? 1u : 0,
			(path == RBB_LB_HBF || path == RBB_HBF) ? 0 : 1u,
			(path == RBB_LB_LBF || path == RBB_LBF) ? 0 : 1u,
			0, 1),

		MAKE_LMS7002_0x0118(
			(path == RBB_LB_LBF || path == RBB_LBF) ? LMS7002_OPT_INPUT_CTL_PGA_RBB_LPFL :
			(path == RBB_LB_HBF || path == RBB_HBF) ? LMS7002_OPT_INPUT_CTL_PGA_RBB_LPFH :
			(path == RBB_LB_BYP) ? LMS7002_OPT_INPUT_CTL_PGA_RBB_TBB : LMS7002_OPT_INPUT_CTL_PGA_RBB_BYPASS,
			12, 12),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

static int lms7_rbb_set_lpfx_bandwidth(struct lms7_state* st, unsigned bw)
{
	enum rbb_path path;
	int res;

	path = RBB_LB_LBF;

	int rcc_ctl_lpfl_rbb = 0;
	int c_ctl_lpfl_rbb = (int)(2160000000U/bw - 103);
	if (c_ctl_lpfl_rbb < 0)
		c_ctl_lpfl_rbb = 0;
	else if (c_ctl_lpfl_rbb > 2047)
		c_ctl_lpfl_rbb = 2047;

	if (bw > 15000000)
		rcc_ctl_lpfl_rbb = 5;
	else if (bw > 10000000)
		rcc_ctl_lpfl_rbb = 4;
	else if (bw > 5000000)
		rcc_ctl_lpfl_rbb = 3;
	else if (bw > 3000000)
		rcc_ctl_lpfl_rbb = 2;
	else if (bw > 1400000)
		rcc_ctl_lpfl_rbb = 1;
	else
		rcc_ctl_lpfl_rbb = 0;

	int c_ctl_lpfh_rbb = (int)(6000.0e6/bw - 50);
	int rcc_ctl_lpfh_rbb = (int)(bw/10e6 - 3);
	if (c_ctl_lpfh_rbb < 0)
		c_ctl_lpfh_rbb = 0;
	else if (c_ctl_lpfh_rbb > 255)
		c_ctl_lpfh_rbb = 255;

	if (rcc_ctl_lpfh_rbb < 0)
		rcc_ctl_lpfh_rbb = 0;
	else if (rcc_ctl_lpfh_rbb > 7)
		rcc_ctl_lpfh_rbb = 7;


	uint32_t regs[] = {
		MAKE_LMS7002_0x0116(16,                  //R_CTL_LPF_RBB,
							rcc_ctl_lpfh_rbb,    //RCC_CTL_LPFH_RBB,
							c_ctl_lpfh_rbb),     //C_CTL_LPFH_RBB)
		MAKE_LMS7002_0x0117(rcc_ctl_lpfl_rbb,    //RCC_CTL_LPFL_RBB,
							c_ctl_lpfl_rbb),     //C_CTL_LPFL_RBB)
	};

	res = lms7_spi_post(st, REG_COUNT(regs), regs);
	if (res)
		return res;

	return lms7_rbb_set_path(st, path);
}

int lms7_rbb_set_bandwidth(struct lms7_state* st, enum lms7_mac_mode mode, unsigned bw)
{
	int res = 0;
	int path = (bw < 20e6) ? RBB_LBF : RBB_HBF;

	if(st->txbw_calibrated[(mode == LMS7_CH_A) ? 0 : 1] == false)
	{
		lms7_log(st,"RBB: Calibration was failed Apply default method");
		if (bw < 100000)
			bw = 100000;

		int cfb_tia_rfe = (int)(1680000000U/bw - 10);
		int ccomp_tia_rfe = cfb_tia_rfe/100;

		if (ccomp_tia_rfe > 15)
			ccomp_tia_rfe = 15;

		if (cfb_tia_rfe < 0)
			cfb_tia_rfe = 0;
		else if (cfb_tia_rfe > 4095)
			cfb_tia_rfe = 4095;

		int rcomp_tia_rfe = (int)(15 - 2*cfb_tia_rfe/100);
		if (rcomp_tia_rfe < 0)
			rcomp_tia_rfe = 0;

		lms7_log(st, "TIA: CCOMP=%d CFB=%d RCOMP=%d",
				ccomp_tia_rfe, cfb_tia_rfe, rcomp_tia_rfe);

		uint32_t regs[] = {
		//	MAKE_LMS7002_0x010F(0, 2, 2),
			MAKE_LMS7002_0x0112(ccomp_tia_rfe, //CCOMP_TIA_RFE,
								cfb_tia_rfe), //CFB_TIA_RFE),
			MAKE_LMS7002_0x0114(rcomp_tia_rfe, //RCOMP_TIA_RFE,
								16), //RFB_TIA_RFE),   //BIAS calibration
		};

		res = lms7_spi_post(st, REG_COUNT(regs), regs);
		if (res)
			return res;

		res = lms7_rbb_set_lpfx_bandwidth(st, bw);
	}

	if (res == 0)
		lms7_rbb_apply_calibration(st,mode,path);

	return res;
}

int lms7_rbb_set_pga(struct lms7_state* st, unsigned gain)
{
	if (gain > 31)
		gain = 31;

	const uint32_t pg_lut[] = { 2864504303u, 1449621641u, 573781061u, 4370u };
	unsigned rcc_ctl = ((pg_lut[gain / 8] >> (4 * (gain % 8))) & 0xf) | 0x10;
	unsigned c_ctl = (gain < 8) ? 3u :
					 (gain < 13) ? 2u :
					 (gain < 21) ? 1u : 0;

	lms7_log(st, "RBB: set_pga(%d) rcc_ctl -> %d, c_ctl -> %d", gain, rcc_ctl, c_ctl);

	uint32_t regs[] = {
		MAKE_LMS7002_0x0119(0, 20, 20, gain),
		MAKE_LMS7002_0x011A(rcc_ctl, c_ctl)
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_rbb_set_ext(struct lms7_state* st)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x0119(1, 20, 20, 1)
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}


/************************************************************************
 * AFE configuration
 ************************************************************************/
int lms7_afe_ctrl(struct lms7_state* st, bool rxa, bool rxb, bool txa, bool txb)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x0082( 4, //ISEL_DAC_AFE,
							 LMS7002_OPT_MODE_INTERLEAVE_AFE_2ADCS, //MODE_INTERLEAVE_AFE,
							 LMS7002_OPT_MUX_AFE_1_MUXOFF, //MUX_AFE_1,
							 LMS7002_OPT_MUX_AFE_2_MUXOFF, //MUX_AFE_2,
							 0, //PD_AFE,
							 rxa ? 0 : 1u, //PD_RX_AFE1
							 rxb ? 0 : 1u, //PD_RX_AFE2
							 txa ? 0 : 1u, //PD_TX_AFE1
							 txb ? 0 : 1u, //PD_TX_AFE2
							 rxa || rxb || txa || txb ? 1u : 0 //EN_G_AFE )
		)
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

/************************************************************************
 * RxTSP configuration
 ************************************************************************/

int lms7_rxtsp_get_rssi(struct lms7_state* st, unsigned mode, uint32_t *orssi)
{
	uint32_t i, q;
	uint32_t regs[] = {
		//MAKE_LMS7002_0x040A(mode, LMS7002_OPT_AGC_MODE_AGC, 3),
		MAKE_LMS7002_0x0400(0,
							LMS7002_OPT_CAPSEL_RSSI, //CAPSEL
							LMS7002_OPT_CAPSEL_ADC_RXTSP_INPUT, //CAPSEL_ADC
							LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_LML, //INSEL,
							0, //BSTART,
							1u),
		MAKE_LMS7002_0x0400(1,
							LMS7002_OPT_CAPSEL_RSSI, //CAPSEL
							LMS7002_OPT_CAPSEL_ADC_RXTSP_INPUT, //CAPSEL_ADC
							LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_LML, //INSEL,
							0, //BSTART,
							1u),
	};
	int res = lms7_spi_post(st, REG_COUNT(regs), regs);
	if (res)
		return res;

	res = lms7_spi_transact(st, 0x040E, &i);
	if (res)
		return res;

	res = lms7_spi_transact(st, 0x040F, &q);
	if (res)
		return res;

	*orssi = (i & 0x3) | ((q & 0xffff) << 2);
	return 0;
}

int lms7_rxtsp_disable(struct lms7_state* st)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x0400(0,
							LMS7002_OPT_CAPSEL_RSSI, //CAPSEL
							LMS7002_OPT_CAPSEL_ADC_RXTSP_INPUT, //CAPSEL_ADC
							LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_LML, //INSEL,
							0, //BSTART,
							0),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_rxtsp_init(struct lms7_state* st, unsigned decim_ord)
{
	st->rxtsp.reg_0x0c = (uint16_t)MAKE_LMS7002_0x040C(0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1);
	st->rxtsp.reg_0x03 = (uint16_t)MAKE_LMS7002_0x0403( (unsigned)(decim_ord - 1), 0);

	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x040C, st->rxtsp.reg_0x0c),
		MAKE_WR_REG(LMS7002M_0x0403, st->rxtsp.reg_0x03),
		MAKE_LMS7002_0x0400(0,
							LMS7002_OPT_CAPSEL_RSSI, //CAPSEL
							LMS7002_OPT_CAPSEL_ADC_RXTSP_INPUT, //CAPSEL_ADC
							LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_LML, //INSEL,
							0, //BSTART,
							1u),
		//MAKE_WR_REG(0x00ad, (3 << 6)), //TODO
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_rxtsp_cmix(struct lms7_state* st, int32_t freq)
{
	freq = -freq;
	if (freq == 0) {
		st->rxtsp.reg_0x0c |= (1u << LMS7002_RXTSP_CMIX_BYP_OFF);
	} else {
		st->rxtsp.reg_0x0c &= (~(1u << LMS7002_RXTSP_CMIX_BYP_OFF));
	}

	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x040C, st->rxtsp.reg_0x0c),
		MAKE_LMS7002_0x0442((uint32_t)freq >> 16),
		MAKE_LMS7002_0x0443((uint32_t)freq),
		MAKE_LMS7002_0x0440(1, 0, LMS7002_OPT_MODE_FCW),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_rxtsp_tsg_const(struct lms7_state* st, int16_t vi, int16_t vq)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x040B((uint32_t)vi),
		MAKE_LMS7002_0x0400(0,
							LMS7002_OPT_CAPSEL_RSSI, //CAPSEL
							LMS7002_OPT_CAPSEL_ADC_RXTSP_INPUT, //CAPSEL_ADC
							LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							1, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_TEST, //INSEL,
							0, //BSTART,
							1u),
		MAKE_LMS7002_0x040B((uint32_t)vq),
		MAKE_LMS7002_0x0400(0,
							LMS7002_OPT_CAPSEL_RSSI, //CAPSEL
							LMS7002_OPT_CAPSEL_ADC_RXTSP_INPUT, //CAPSEL_ADC
							LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							1, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_TEST, //INSEL,
							0, //BSTART,
							1u),
		MAKE_LMS7002_0x0400(0,
							LMS7002_OPT_CAPSEL_RSSI, //CAPSEL
							LMS7002_OPT_CAPSEL_ADC_RXTSP_INPUT, //CAPSEL_ADC
							LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_TEST, //INSEL,
							0, //BSTART,
							1u),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_rxtsp_tsg_tone(struct lms7_state* st, bool fs, bool div4)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x0400(0,
							LMS7002_OPT_CAPSEL_RSSI, //CAPSEL
							LMS7002_OPT_CAPSEL_ADC_RXTSP_INPUT, //CAPSEL_ADC
							(fs) ? LMS7002_OPT_TSGFC_FS : LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							(div4) ? LMS7002_OPT_TSGFCW_DIV4 : LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_NCO, //TSGMODE,
							LMS7002_OPT_INSEL_TEST, //INSEL,
							0, //BSTART,
							1u),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_rxtsp_dc_corr(struct lms7_state* st, unsigned wnd)
{
	st->rxtsp.reg_0x0c = st->rxtsp.reg_0x0c & (~(1u << LMS7002_RXTSP_DC_BYP_OFF));

	uint32_t regs[] = {
		MAKE_LMS7002_0x0404(0, wnd),
		MAKE_WR_REG(LMS7002M_0x040C, st->rxtsp.reg_0x0c),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_rxtsp_iq_gcorr(struct lms7_state* st, unsigned ig, unsigned qg)
{
	st->rxtsp.reg_0x0c = st->rxtsp.reg_0x0c & (~(1u << LMS7002_RXTSP_GC_BYP_OFF));

	uint32_t regs[] = {
		MAKE_LMS7002_0x0401(qg),
		MAKE_LMS7002_0x0402(ig),
		MAKE_WR_REG(LMS7002M_0x040C, st->rxtsp.reg_0x0c),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_rxtsp_iq_phcorr(struct lms7_state* st, int acorr)
{
	st->rxtsp.reg_0x0c = st->rxtsp.reg_0x0c & (~(1u << LMS7002_RXTSP_PH_BYP_OFF));
	st->rxtsp.reg_0x03 = (st->rxtsp.reg_0x03 & ~LMS7002_RXTSP_IQCORR_MSK) | (acorr & LMS7002_RXTSP_IQCORR_MSK);

	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x0403, st->rxtsp.reg_0x03),
		MAKE_WR_REG(LMS7002M_0x040C, st->rxtsp.reg_0x0c),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

/************************************************************************
 * TxTSP configuration
 ************************************************************************/
int lms7_txtsp_disable(struct lms7_state* st)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x0200(LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_LML, //INSEL,
							0, //BSTART,
							0),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}
int lms7_txtsp_init(struct lms7_state* st, unsigned interp_ord)
{
	st->txtsp.reg_0x0c = (uint16_t)MAKE_LMS7002_0x0208(0, 0, 1, 1, 1, 1, 1, 1, 1, 1);
	st->txtsp.reg_0x03 = (uint16_t)MAKE_LMS7002_0x0203( (unsigned)(interp_ord - 1), 0);

	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x0208, st->txtsp.reg_0x0c),
		MAKE_WR_REG(LMS7002M_0x0203, st->txtsp.reg_0x03),
		MAKE_LMS7002_0x0200(LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV4, //TSGFCW,
							0, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_LML, //INSEL,
							0, //BSTART,
							1u),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_txtsp_cmix(struct lms7_state* st, int32_t freq)
{
	if (freq == 0) {
		st->txtsp.reg_0x0c |= (1u << LMS7002_TXTSP_CMIX_BYP_OFF);
	} else {
		st->txtsp.reg_0x0c &= (~(1u << LMS7002_TXTSP_CMIX_BYP_OFF));
	}

	lms7_log(st, "TXTSP CMIX=%d", freq);

	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x0208, st->txtsp.reg_0x0c),
		MAKE_LMS7002_0x0242((uint32_t)freq >> 16),
		MAKE_LMS7002_0x0243((uint32_t)freq),
		MAKE_LMS7002_0x0240(1, 0, LMS7002_OPT_MODE_FCW),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_txtsp_tsg_const(struct lms7_state* st, int16_t vi, int16_t vq)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x020C((uint32_t)vi),
		MAKE_LMS7002_0x0200(LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							1, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_TEST, //INSEL,
							0, //BSTART,
							1u),
		MAKE_LMS7002_0x020C((uint32_t)vq),
		MAKE_LMS7002_0x0200(LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							1, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_TEST, //INSEL,
							0, //BSTART,
							1u),
		MAKE_LMS7002_0x0200(LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_DC, //TSGMODE,
							LMS7002_OPT_INSEL_TEST, //INSEL,
							0, //BSTART,
							1u),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_txtsp_tsg_tone(struct lms7_state* st, bool fs, bool div4)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x0200((fs) ? LMS7002_OPT_TSGFC_FS : LMS7002_OPT_TSGFC_NEG6DB, //TSGFC,
							(div4) ? LMS7002_OPT_TSGFCW_DIV4 : LMS7002_OPT_TSGFCW_DIV8, //TSGFCW,
							0, //TSGDCLDQ
							0, //TSGDCLDI
							0, //TSGSWAPIQ,
							LMS7002_OPT_TSGMODE_NCO, //TSGMODE,
							LMS7002_OPT_INSEL_TEST, //INSEL,
							0, //BSTART,
							1u),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_txtsp_iq_gcorr(struct lms7_state* st, unsigned ig, unsigned qg)
{
	st->txtsp.reg_0x0c = st->txtsp.reg_0x0c & (~(1u << LMS7002_TXTSP_GC_BYP_OFF));

	uint32_t regs[] = {
		MAKE_LMS7002_0x0201(qg),
		MAKE_LMS7002_0x0202(ig),
		MAKE_WR_REG(LMS7002M_0x0208, st->txtsp.reg_0x0c),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_txtsp_iq_phcorr(struct lms7_state* st, int acorr)
{
	st->txtsp.reg_0x0c = st->txtsp.reg_0x0c & (~(1u << LMS7002_TXTSP_PH_BYP_OFF));
	st->txtsp.reg_0x03 = (st->txtsp.reg_0x03 & ~LMS7002_TXTSP_IQCORR_MSK) | (acorr & LMS7002_TXTSP_IQCORR_MSK);

	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x0203, st->rxtsp.reg_0x03),
		MAKE_WR_REG(LMS7002M_0x0208, st->rxtsp.reg_0x0c),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

/************************************************************************
 * DC configuration
 ************************************************************************/
int lms7_dc_init(struct lms7_state* st,
				 bool rxaen, bool rxben, bool txaen, bool txben)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x05C0((rxaen || rxben || txaen || txben) ? 1u : 0, // DCMODE,
							rxben ? 0 : 1u, //PD_DCDAC_RXB,
							rxaen ? 0 : 1u, //PD_DCDAC_RXA,
							txben ? 0 : 1u, //PD_DCDAC_TXB,
							txaen ? 0 : 1u, //PD_DCDAC_TXA,
							rxben ? 0 : 1u, //PD_DCCMP_RXB,
							rxaen ? 0 : 1u, //PD_DCCMP_RXA,
							txben ? 0 : 1u, //PD_DCCMP_TXB,
							txaen ? 0 : 1u //PD_DCCMP_TXA)
		),
		MAKE_LMS7002_0x05C2(0, 0, 0, 0, 0, 0, 0, 0,
							0, 0, 0, 0, 0, 0, 0, 0),
		MAKE_LMS7002_0x05CB(255, 255),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_dc_rxa_set(struct lms7_state* st, int16_t i, int16_t q)
{
	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x05C7, i & 0x7f),
		MAKE_WR_REG(LMS7002M_0x05C7, (i & 0x7f) | 0x8000),
		MAKE_WR_REG(LMS7002M_0x05C8, q & 0x7f),
		MAKE_WR_REG(LMS7002M_0x05C8, (q & 0x7f) | 0x8000),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_dc_rxb_set(struct lms7_state* st, int16_t i, int16_t q)
{
	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x05C9, i & 0x7f),
		MAKE_WR_REG(LMS7002M_0x05C9, (i & 0x7f) | 0x8000),
		MAKE_WR_REG(LMS7002M_0x05CA, q & 0x7f),
		MAKE_WR_REG(LMS7002M_0x05CA, (q & 0x7f) | 0x8000),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_dc_txa_set(struct lms7_state* st, int16_t i, int16_t q)
{
	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x05C3, i & 0x7ff),
		MAKE_WR_REG(LMS7002M_0x05C3, (i & 0x7ff) | 0x8000),
		MAKE_WR_REG(LMS7002M_0x05C4, q & 0x7ff),
		MAKE_WR_REG(LMS7002M_0x05C4, (q & 0x7ff) | 0x8000),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_dc_txb_set(struct lms7_state* st, int16_t i, int16_t q)
{
	uint32_t regs[] = {
		MAKE_WR_REG(LMS7002M_0x05C5, i & 0x7ff),
		MAKE_WR_REG(LMS7002M_0x05C5, (i & 0x7ff) | 0x8000),
		MAKE_WR_REG(LMS7002M_0x05C6, q & 0x7ff),
		MAKE_WR_REG(LMS7002M_0x05C6, (q & 0x7ff) | 0x8000),
	};
	return lms7_spi_post(st, REG_COUNT(regs), regs);
}

int lms7_dc_start(struct lms7_state* st,
				  bool rxa, bool rxb, bool txa, bool txb)
{
	uint32_t regs[] = {
		MAKE_LMS7002_0x05C0(1, // DCMODE,
							rxb ? 0 : 1u, //PD_DCDAC_RXB,
							rxa ? 0 : 1u, //PD_DCDAC_RXA,
							txb ? 0 : 1u, //PD_DCDAC_TXB,
							txa ? 0 : 1u, //PD_DCDAC_TXA,
							rxb ? 0 : 1u, //PD_DCCMP_RXB,
							rxa ? 0 : 1u, //PD_DCCMP_RXA,
							txb ? 0 : 1u, //PD_DCCMP_TXB,
							txa ? 0 : 1u //PD_DCCMP_TXA)
		),
		MAKE_LMS7002_0x05C2(1, 1, 1, 1, 1, 1, 1, 1,
							(rxb) ? 1u : 0, //DCCAL_START_RXBQ,
							(rxb) ? 1u : 0, //DCCAL_START_RXBI,
							(rxa) ? 1u : 0, //DCCAL_START_RXAQ,
							(rxa) ? 1u : 0, //DCCAL_START_RXAI,
							(txb) ? 1u : 0, //DCCAL_START_TXBQ,
							(txb) ? 1u : 0, //DCCAL_START_TXBI,
							(txa) ? 1u : 0, //DCCAL_START_TXAQ,
							(txa) ? 1u : 0 //DCCAL_START_TXAI
		),
	};
	int res = lms7_spi_post(st, REG_COUNT(regs), regs);
	if (res)
		return res;

	for (unsigned i = 0; i < 10; i++) {
		uint32_t reg;
		int res = lms7_spi_transact(st, LMS7002M_0x05C1, &reg);
		if (res)
			return res;

		lms7_log(st, " 5c1=%04x", reg);
	}

	uint32_t reg[8];
	for (unsigned i = 0; i < 8; i++) {
		int res;
		uint32_t regs[] = {
			((0x8000 + LMS7002M_0x05C3 + i) << 16),
			((0x8000 + LMS7002M_0x05C3 + i) << 16) | 0x4000,
		};
		res = lms7_spi_post(st, REG_COUNT(regs), regs);
		if (res)
			return res;
		res = lms7_spi_transact(st, LMS7002M_0x05C3 + i, &reg[i]);
		if (res)
			return res;

		lms7_log(st, " %cX[%d]=%04x", i < 4 ? 'T' : 'R', i % 4, reg[i] & 0x1ff);
	}

	{
		uint32_t regs[] = {
			MAKE_LMS7002_0x05C2(0, 0, 0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0),
		};
		res = lms7_spi_post(st, REG_COUNT(regs), regs);
		if (res)
			return res;
	}
	return 0;
}


/************************************************************************
 * HELPERS
 ************************************************************************/
int lms7_cal_rxdc(struct lms7_state* st)
{
	int res;
	unsigned ai_min = 262144, aiv = 0;
	unsigned aq_min = 262144, aqv = 0;
	unsigned bi_min = 262144, biv = 0;
	unsigned bq_min = 262144, bqv = 0;
	uint32_t tmp;

	res = lms7_rxtsp_get_rssi(st, 0, &tmp);
	if (res)
		return res;

	for (unsigned c = 0; c < 128; c++) {
		uint32_t regs[] = {
			MAKE_LMS7002_0x05C7(0, 0, c),
			MAKE_LMS7002_0x05C8(0, 0, c),
			MAKE_LMS7002_0x05C9(0, 0, c),
			MAKE_LMS7002_0x05CA(0, 0, c),
			MAKE_LMS7002_0x05C7(1, 0, c),
			MAKE_LMS7002_0x05C8(1, 0, c),
			MAKE_LMS7002_0x05C9(1, 0, c),
			MAKE_LMS7002_0x05CA(1, 0, c),

			MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 | 0x3u)),
			MAKE_WR_REG(LMS7002M_0x040C, st->rxtsp.reg_0x0c & (~(1u << LMS7002_RXTSP_AGC_BYP_OFF))),

			MAKE_LMS7002_0x040A(LMS7002_OPT_RSSI_MODE_RSSI_I, LMS7002_OPT_AGC_MODE_RSSI, 6),
		};
		res = lms7_spi_post(st, REG_COUNT(regs), regs);
		if (res)
			return res;

		usleep(1000);

		res = lms7_mac_set(st, LMS7_CH_A);
		if (res)
			return res;
		res = lms7_rxtsp_get_rssi(st, LMS7002_OPT_RSSI_MODE_RSSI_I, &tmp);
		if (res)
			return res;

		if (ai_min > tmp) {
			ai_min = tmp;
			aiv = c;
		}

		res = lms7_mac_set(st, LMS7_CH_B);
		if (res)
			return res;
		res = lms7_rxtsp_get_rssi(st, LMS7002_OPT_RSSI_MODE_RSSI_I, &tmp);
		if (res)
			return res;

		if (bi_min > tmp) {
			bi_min = tmp;
			biv = c;
		}

		uint32_t regs2[] = {
			MAKE_WR_REG(LMS7002M_0x0020, (st->reg_0x0020 | 0x3u)),
			MAKE_LMS7002_0x040A(LMS7002_OPT_RSSI_MODE_RSSI_Q, LMS7002_OPT_AGC_MODE_RSSI, 6),
		};
		res = lms7_spi_post(st, REG_COUNT(regs2), regs2);
		if (res)
			return res;

		usleep(1000);

		res = lms7_mac_set(st, LMS7_CH_A);
		if (res)
			return res;
		res = lms7_rxtsp_get_rssi(st, LMS7002_OPT_RSSI_MODE_RSSI_Q, &tmp);
		if (res)
			return res;

		if (aq_min > tmp) {
			aq_min = tmp;
			aqv = c;
		}

		res = lms7_mac_set(st, LMS7_CH_B);
		if (res)
			return res;
		res = lms7_rxtsp_get_rssi(st, LMS7002_OPT_RSSI_MODE_RSSI_Q, &tmp);
		if (res)
			return res;

		if (bq_min > tmp) {
			bq_min = tmp;
			bqv = c;
		}
	}

	lms7_log(st, "AQ[%d] = %d AI[%d] = %d | BQ[%d] = %d BI[%d] = %d",
			 aqv, aq_min,
			 aiv, ai_min,
			 bqv, bq_min,
			 biv, bi_min);



	for (unsigned e = 0; e < 25; e++) {
		int vi = (e / 5);
		int vq = (e % 5);
		if (vi > 2)
			vi = -vi + 2;
		if (vq > 2)
			vq = -vq + 2;

		int vvi = ((int)aiv + vi);
		int vvq = ((int)aqv + vq);

		{
		uint32_t regs[] = {
			MAKE_LMS7002_0x05C7(0, 0, vvi),
			MAKE_LMS7002_0x05C8(0, 0, vvq),
		};
		res = lms7_spi_post(st, REG_COUNT(regs), regs);
		if (res)
			return res;
		}

		usleep(10000);

		res = lms7_mac_set(st, LMS7_CH_A);
		if (res)
			return res;
		res = lms7_rxtsp_get_rssi(st, LMS7002_OPT_RSSI_MODE_RSSI_NORM, &tmp);
		if (res)
			return res;

		lms7_log(st, "AQ[%d,%d] = %d",
				 vi, vq, tmp);
	}

	{
	uint32_t regs[] = {
		MAKE_LMS7002_0x05C7(0, 0, aiv),
		MAKE_LMS7002_0x05C8(0, 0, aqv),
		MAKE_LMS7002_0x05C9(0, 0, biv),
		MAKE_LMS7002_0x05CA(0, 0, bqv),
		MAKE_LMS7002_0x05C7(1, 0, aiv),
		MAKE_LMS7002_0x05C8(1, 0, aqv),
		MAKE_LMS7002_0x05C9(1, 0, biv),
		MAKE_LMS7002_0x05CA(1, 0, bqv),
	};
	res = lms7_spi_post(st, REG_COUNT(regs), regs);
	if (res)
		return res;
	}

	return 0;
}

//! Prevent calibration loops from getting stuck
#define MAX_CAL_LOOP_ITERS	512

uint16_t lms7regs_address[LMS7002M_IDX_COUNT] =
{
	LMS7002M_0x0020,
	LMS7002M_0x0021,
	LMS7002M_0x0022,
	LMS7002M_0x0023,
	LMS7002M_0x0024,
	LMS7002M_0x0025,
	LMS7002M_0x0026,
	LMS7002M_0x0027,
	LMS7002M_0x0028,
	LMS7002M_0x0029,
	LMS7002M_0x002A,
	LMS7002M_0x002B,
	LMS7002M_0x002C,
	LMS7002M_0x002E,
	LMS7002M_0x002F,
	LMS7002M_0x0081,
	LMS7002M_0x0082,
	LMS7002M_0x0084,
	LMS7002M_0x0085,
	LMS7002M_0x0086,
	LMS7002M_0x0087,
	LMS7002M_0x0088,
	LMS7002M_0x0089,
	LMS7002M_0x008A,
	LMS7002M_0x008B,
	LMS7002M_0x008C,
	LMS7002M_0x008D,
	LMS7002M_0x0092,
	LMS7002M_0x0093,
	LMS7002M_0x0094,
	LMS7002M_0x0095,
	LMS7002M_0x0096,
	LMS7002M_0x0097,
	LMS7002M_0x0098,
	LMS7002M_0x0099,
	LMS7002M_0x009A,
	LMS7002M_0x009B,
	LMS7002M_0x009C,
	LMS7002M_0x009D,
	LMS7002M_0x009E,
	LMS7002M_0x009F,
	LMS7002M_0x00A0,
	LMS7002M_0x00A1,
	LMS7002M_0x00A2,
	LMS7002M_0x00A3,
	LMS7002M_0x00A4,
	LMS7002M_0x00A5,
	LMS7002M_0x00A6,
	LMS7002M_0x00A7,
	LMS7002M_0x0100,
	LMS7002M_0x0101,
	LMS7002M_0x0102,
	LMS7002M_0x0103,
	LMS7002M_0x0104,
	LMS7002M_0x0105,
	LMS7002M_0x0106,
	LMS7002M_0x0107,
	LMS7002M_0x0108,
	LMS7002M_0x0109,
	LMS7002M_0x010A,
	LMS7002M_0x010C,
	LMS7002M_0x010D,
	LMS7002M_0x010E,
	LMS7002M_0x010F,
	LMS7002M_0x0110,
	LMS7002M_0x0111,
	LMS7002M_0x0112,
	LMS7002M_0x0113,
	LMS7002M_0x0114,
	LMS7002M_0x0115,
	LMS7002M_0x0116,
	LMS7002M_0x0117,
	LMS7002M_0x0118,
	LMS7002M_0x0119,
	LMS7002M_0x011A,
	LMS7002M_0x011B,
	LMS7002M_0x011C,
	LMS7002M_0x011D,
	LMS7002M_0x011E,
	LMS7002M_0x011F,
	LMS7002M_0x0120,
	LMS7002M_0x0121,
	LMS7002M_0x0122,
	LMS7002M_0x0123,
	LMS7002M_0x0124,
	LMS7002M_0x0200,
	LMS7002M_0x0201,
	LMS7002M_0x0202,
	LMS7002M_0x0203,
	LMS7002M_0x0204,
	LMS7002M_0x0205,
	LMS7002M_0x0206,
	LMS7002M_0x0207,
	LMS7002M_0x0208,
	LMS7002M_0x020C,
	LMS7002M_0x0240,
	LMS7002M_0x0241,
	LMS7002M_0x0242,
	LMS7002M_0x0243,
	LMS7002M_0x0400,
	LMS7002M_0x0401,
	LMS7002M_0x0402,
	LMS7002M_0x0403,
	LMS7002M_0x0404,
	LMS7002M_0x0405,
	LMS7002M_0x0406,
	LMS7002M_0x0407,
	LMS7002M_0x0408,
	LMS7002M_0x0409,
	LMS7002M_0x040A,
	LMS7002M_0x040B,
	LMS7002M_0x040C,
	LMS7002M_0x0440,
	LMS7002M_0x0441,
	LMS7002M_0x0442,
	LMS7002M_0x0443,
	LMS7002M_0x05C0,
	LMS7002M_0x05C1,
	LMS7002M_0x05C2,
	LMS7002M_0x05C3,
	LMS7002M_0x05C4,
	LMS7002M_0x05C5,
	LMS7002M_0x05C6,
	LMS7002M_0x05C7,
	LMS7002M_0x05C8,
	LMS7002M_0x05C9,
	LMS7002M_0x05CA,
	LMS7002M_0x05CB,
	LMS7002M_0x05CC,
};

#ifdef UNUSED
uint16_t lms7_get_spi_bits(struct lms7_state *st,uint16_t addr,uint8_t msb,uint8_t lsb)
{
	int res;
	uint32_t spiDataReg;

	res = lms7_spi_transact(st,addr,&spiDataReg);	// read current SPI reg data
	if(res)
		return(0);

	spiDataReg &= 0xFFFF;
	return((spiDataReg & (~(~0u << (msb + 1)))) >> lsb);	// shift bits to LSB
}

int lms7_set_spi_modify_bits(struct lms7_state *st,uint16_t addr,uint8_t msb,uint8_t lsb,uint16_t value)
{
	int res;
	uint32_t spiDataReg;

	res = lms7_spi_transact(st,addr,&spiDataReg);	// read current SPI reg data
	if(res)
		return(res);

	spiDataReg &= 0xFFFF;
	uint16_t spiMask = (~(~0u << (msb - lsb + 1))) << (lsb);				// creates bit mask
	spiDataReg = (spiDataReg & (~spiMask)) | ((value << lsb) & spiMask);	// clear bits
	uint32_t data = MAKE_WR_REG(addr,spiDataReg);

	return(lms7_spi_post(st,1,&data));	// write modified data back to SPI reg
}
#endif

int lms7_modify_spi_reg_bits(struct lms7_state *st,uint16_t addr,uint8_t bits,uint16_t new_bits_data)
{
	int res;
	uint32_t spiDataReg;

	res = lms7_spi_transact(st,addr,&spiDataReg);	// read current SPI reg data
	if(res)
		return(res);

	spiDataReg &= 0xFFFF;
	const uint16_t spiMask = (~(~0 << ((bits >> 4)-(bits & 0xF) + 1))) << (bits & 0xF);	// creates bit mask

	spiDataReg = (spiDataReg & (~spiMask)) | ((new_bits_data << (bits & 0xF)) & spiMask);	// clear bits
	uint32_t data = MAKE_WR_REG(addr,spiDataReg);

	return(lms7_spi_post(st,1,&data));	// write modified data back to SPI reg
}

uint16_t lms7_get_spi_reg_bits(struct lms7_state *st,uint16_t addr,uint8_t bits)
{
	int res;
	uint32_t spiDataReg;

	res = lms7_spi_transact(st,addr,&spiDataReg);	// read current SPI reg data
	if(res)
		return(0);

	spiDataReg &= 0xFFFF;
	return((spiDataReg & (~(~0 << ((bits >> 4)+1)))) >> (bits & 0xF));	//shift bits to LSB
}

static uint32_t lms7_spi_read(struct lms7_state *st,uint16_t addr)
{
	int res;
	uint32_t value;

	res = lms7_spi_transact(st,addr,&value);
	if(res)
		return(res);

	return(value & 0xFFFF);
}

static int lms7_spi_write(struct lms7_state *st,uint16_t addr,uint16_t value)
{
	uint32_t data = MAKE_WR_REG(addr,value);
	return(lms7_spi_post(st,1,&data));
}

static void set_addrs_to_default(struct lms7_state *st,enum lms7_mac_mode mode,int start_addr,int stop_addr)
{
	lms7_mac_set(st,mode);

	for(int addr = start_addr;addr <= stop_addr;addr++)
	{
		int value = lms7_regs_default(addr);
		if(value == -1)
			continue;	// not in map

		lms7_spi_write(st,addr,value);
	}
}

void lms7_store_register(struct lms7_state *st)
{
	int idx;

	// Save register map
	lms7_mac_set(st,LMS7_CH_A);
	for(idx = 0;idx < LMS7002M_IDX_COUNT;idx++)
	{
		st->saved_regs[0][idx] = lms7_spi_read(st,lms7regs_address[idx]);
	}

	lms7_mac_set(st,LMS7_CH_B);
	for(idx = 0;idx < LMS7002M_IDX_COUNT;idx++)
	{
		st->saved_regs[1][idx] = lms7_spi_read(st,lms7regs_address[idx]);
	}
}

void lms7_restore_register(struct lms7_state *st)
{
	int idx;

	// restore original register values
	lms7_mac_set(st,LMS7_CH_A);
	for(idx = 0;idx < LMS7002M_IDX_COUNT;idx++)
		lms7_spi_write(st,lms7regs_address[idx],st->saved_regs[0][idx]);

	lms7_mac_set(st,LMS7_CH_B);
	// ignore registers that do not depend on MAC
	for(idx = LMS7002M_IDX_0x0100;idx < LMS7002M_IDX_COUNT;idx++)
		lms7_spi_write(st,lms7regs_address[idx],st->saved_regs[1][idx]);
}

static void txtsp_tsg_const(struct lms7_state *st,enum lms7_mac_mode mode,int valI,int valQ)
{
	lms7_mac_set(st,mode);

	//muxes
	lms7_modify_spi_reg_bits(st,TSGFC_TXTSP,LMS7002_OPT_TSGFC_FS);
	lms7_modify_spi_reg_bits(st,TSGMODE_TXTSP,LMS7002_OPT_TSGMODE_DC);
	lms7_modify_spi_reg_bits(st,INSEL_TXTSP,LMS7002_OPT_INSEL_TEST);

	// load I
	lms7_modify_spi_reg_bits(st,DC_REG_TXTSP,valI);
	lms7_modify_spi_reg_bits(st,TSGDCLDI_TXTSP,0);
	lms7_modify_spi_reg_bits(st,TSGDCLDI_TXTSP,1);
	lms7_modify_spi_reg_bits(st,TSGDCLDI_TXTSP,0);

	// load Q
	lms7_modify_spi_reg_bits(st,DC_REG_TXTSP,valQ);
	lms7_modify_spi_reg_bits(st,TSGDCLDQ_TXTSP,0);
	lms7_modify_spi_reg_bits(st,TSGDCLDQ_TXTSP,1);
	lms7_modify_spi_reg_bits(st,TSGDCLDQ_TXTSP,0);
}

/* Re-tune the CORDICs based on the bandwidth */
static void setup_tx_cal_tone(struct lms7_state *st,enum lms7_mac_mode mode,double bw)
{
	lms7_mac_set(st,mode);

	double rel_freq;
	double dac_freq = st->cgen_freq / 4;	// hard coded
	int pfreq;

	rel_freq = bw / dac_freq;
	if((rel_freq > 0.5)
		|| (rel_freq < -0.5))
	{
		lms7_log(st,"NCO TX ouf of range,requested %.3f while DAC %.3f\n",rel_freq / 1000,dac_freq / 1000);
		return;
	}

	pfreq = rel_freq * 4294967296;
	lms7_txtsp_cmix(st,pfreq);

	rel_freq = (bw - 1e6) / dac_freq;
	if((rel_freq > 0.5)
		|| (rel_freq < -0.5))
	{
		lms7_log(st,"NCO RX ouf of range,requested %.3f while DAC %.3f\n",rel_freq / 1000,dac_freq / 1000);
		return;
	}

	pfreq = rel_freq * 4294967296;
	lms7_rxtsp_cmix(st,pfreq);
}

/* Re-tune the RX LO based on the bandwidth */
static int setup_rx_cal_tone(struct lms7_state *st,enum lms7_mac_mode mode,double bw)
{
	int res = 0;
	double rel_freq;
	int pfreq;

//	LMS7002M_sxx_enable(st,LMS7_RX,true);
//	LMS7002M_sxt_to_sxr(st,false);
	lms7_mac_set(st,mode);

	const double sxr_freq = st->sxt_freq - bw;

	res = lms7_sxx_tune_sync(st,true,sxr_freq,false);
	lms7_mac_set(st,mode);
	if(res != 0)
	{
		lms7_log(st,"lms7_sxx_tune_sync(LMS_RX,%f MHz)",sxr_freq / 1e6);
		goto done;
	}

	st->sxr_freq = sxr_freq;

	double dac_freq = st->cgen_freq / 4;

	rel_freq = (st->sxt_freq - sxr_freq) / dac_freq;
	if((rel_freq > 0.5)
		|| (rel_freq < -0.5))
	{
		lms7_log(st,"NCO RX ouf of range,requested %.3f while DAC %.3f\n",rel_freq / 1000,dac_freq / 1000);
		return(-1);
	}

	pfreq = rel_freq * 4294967296;
	lms7_rxtsp_cmix(st,pfreq);

	done:
	return(res);
}

static uint16_t cal_read_rssi(struct lms7_state *st,enum lms7_mac_mode mode)
{
	uint32_t rssi;

	usleep(1000);
	lms7_rxtsp_get_rssi(st,mode,&rssi);
	return(rssi);
}

static int cal_gain_selection(struct lms7_state *st,enum lms7_mac_mode mode)
{
	while(true)
	{
		const int rssi_value_50k = cal_read_rssi(st,mode);

		if(rssi_value_50k < 0x8400)
			break;

		st->cg_iamp_tbb++;
		if(st->cg_iamp_tbb > 63)
		{
			if(st->g_pga_rbb > 31)
				break;

			st->cg_iamp_tbb = 1;
			st->g_pga_rbb += 6;
		}

		lms7_modify_spi_reg_bits(st,CG_IAMP_TBB,st->cg_iamp_tbb);
		lms7_modify_spi_reg_bits(st,G_PGA_RBB,st->g_pga_rbb);
	}

	return(cal_read_rssi(st,mode));
}

/* Prepare for TX filter st-calibration */
static int tx_cal_init(struct lms7_state *st,enum lms7_mac_mode mode)
{
	lms7_mac_set(st,mode);

	//--- rfe ---
	lms7_modify_spi_reg_bits(st,EN_G_RFE,0);

	//--- rbb ---
	set_addrs_to_default(st,mode,LMS7002M_0x0115,LMS7002M_0x011B);

	lms7_modify_spi_reg_bits(st,PD_LPFL_RBB,1);
	lms7_modify_spi_reg_bits(st,INPUT_CTL_PGA_RBB,3);
	lms7_modify_spi_reg_bits(st,ICT_PGA_OUT_RBB,20);
	lms7_modify_spi_reg_bits(st,ICT_PGA_IN_RBB,20);
	lms7_modify_spi_reg_bits(st,C_CTL_PGA_RBB,3);

	//--- trf ---
	lms7_modify_spi_reg_bits(st,EN_G_TRF,0);

	//--- tbb ---
	set_addrs_to_default(st,mode,LMS7002M_0x0105,0x010B);

	lms7_modify_spi_reg_bits(st,CG_IAMP_TBB,1);
	st->rcal_lpflad_tbb = 193;
	st->rcal_lpfh_tbb = 97;

	st->ccal_lpflad_tbb = 16;
	st->rcal_lpfs5_tbb = 76;

	st->cg_iamp_tbb = 1;
	st->g_pga_rbb = 6;
	lms7_modify_spi_reg_bits(st,ICT_IAMP_FRP_TBB,st->cg_iamp_tbb);
	lms7_modify_spi_reg_bits(st,ICT_IAMP_GG_FRP_TBB,st->g_pga_rbb);

	//--- afe ---
	lms7_afe_ctrl(st,((mode == LMS7_CH_A) ? 1 : 0)
		,((mode == LMS7_CH_B) ? 1 : 0)
		,((mode == LMS7_CH_A) ? 1 : 0)
		,((mode == LMS7_CH_B) ? 1 : 0));

	lms7_mac_set(st,mode);

	//--- bias -- must write to chA ---//
	lms7_mac_set(st,LMS7_CH_A);

	const int rp_calib_bias = lms7_get_spi_reg_bits(st,RP_CALIB_BIAS);
	lms7_spi_write(st,LMS7002M_0x0084,0x400);	// set to default
	lms7_modify_spi_reg_bits(st,RP_CALIB_BIAS,rp_calib_bias);

	lms7_mac_set(st,mode);

	//--- TxTSP ---
	set_addrs_to_default(st,mode,LMS7002M_0x0200,LMS7002M_0x020C);

	lms7_modify_spi_reg_bits(st,TSGMODE_TXTSP,1);
	lms7_modify_spi_reg_bits(st,INSEL_TXTSP,1);

	lms7_modify_spi_reg_bits(st,GFIR3_BYP_TXTSP,1);
	lms7_modify_spi_reg_bits(st,GFIR2_BYP_TXTSP,1);
	lms7_modify_spi_reg_bits(st,GFIR1_BYP_TXTSP,1);

	txtsp_tsg_const(st,mode,0x7FFF,0x8000);

	//--- RxTSP ---
	set_addrs_to_default(st,mode,LMS7002M_0x0400,LMS7002M_0x040C);

	lms7_modify_spi_reg_bits(st,AGC_MODE_RXTSP,1);
	lms7_modify_spi_reg_bits(st,AGC_AVG_RXTSP,7);

	lms7_modify_spi_reg_bits(st,GFIR3_BYP_RXTSP,1);
	lms7_modify_spi_reg_bits(st,GFIR2_BYP_RXTSP,1);
	lms7_modify_spi_reg_bits(st,GFIR1_BYP_RXTSP,1);
	lms7_modify_spi_reg_bits(st,CMIX_GAIN_RXTSP,1);

	st->txtsp.reg_0x0c = lms7_spi_read(st,LMS7002M_0x0208);
	st->rxtsp.reg_0x0c = lms7_spi_read(st,LMS7002M_0x040C);

	//--- initial cal tone ---//
	setup_tx_cal_tone(st,mode,50e3);
	return(0);
}

/* Prepare for RX filter self-calibration */
static int rx_cal_init(struct lms7_state *st,enum lms7_mac_mode mode)
{
	int res = 0;

	lms7_mac_set(st,mode);
	const int g_tia_rfe_user = lms7_get_spi_reg_bits(st,G_TIA_RFE);

	//--- rfe ---
	set_addrs_to_default(st,mode,LMS7002M_0x010C,LMS7002M_0x0114);

	lms7_modify_spi_reg_bits(st,PD_RLOOPB_2_RFE,0);
	lms7_modify_spi_reg_bits(st,PD_MXLOBUF_RFE,0);
	lms7_modify_spi_reg_bits(st,PD_QGEN_RFE,0);

	lms7_modify_spi_reg_bits(st,SEL_PATH_RFE,2);
	lms7_modify_spi_reg_bits(st,EN_INSHSW_LB2_RFE,0);

	lms7_modify_spi_reg_bits(st,ICT_TIAMAIN_RFE,2);
	lms7_modify_spi_reg_bits(st,ICT_TIAOUT_RFE,2);

	lms7_modify_spi_reg_bits(st,G_RXLOOPB_RFE,8);
	lms7_modify_spi_reg_bits(st,G_TIA_RFE,g_tia_rfe_user);

	lms7_modify_spi_reg_bits(st,RFB_TIA_RFE,16);

	//--- rbb ---
	set_addrs_to_default(st,mode,LMS7002M_0x0115,LMS7002M_0x011B);
	lms7_modify_spi_reg_bits(st,ICT_PGA_OUT_RBB,20);
	lms7_modify_spi_reg_bits(st,ICT_PGA_IN_RBB,20);

	lms7_modify_spi_reg_bits(st,C_CTL_PGA_RBB,3);

	//--- trf ---
	set_addrs_to_default(st,mode,LMS7002M_0x0100,LMS7002M_0x0104);
	lms7_modify_spi_reg_bits(st,L_LOOPB_TXPAD_TRF,0);
	lms7_modify_spi_reg_bits(st,EN_LOOPB_TXPAD_TRF,1);

	lms7_modify_spi_reg_bits(st,SEL_BAND1_TRF,0);
	lms7_modify_spi_reg_bits(st,SEL_BAND2_TRF,1);

	//--- tbb ---
	set_addrs_to_default(st,mode,LMS7002M_0x0105,LMS7002M_0x010A);
	lms7_modify_spi_reg_bits(st,CG_IAMP_TBB,1);
	lms7_modify_spi_reg_bits(st,ICT_IAMP_FRP_TBB,1);
	lms7_modify_spi_reg_bits(st,ICT_IAMP_GG_FRP_TBB,6);

	//--- rfe and trf nextrx -- must write to chA ---//
	lms7_mac_set(st,LMS7_CH_A);

	lms7_modify_spi_reg_bits(st,EN_NEXTRX_RFE,(mode == LMS7_CH_A) ? 0 : 1);
	lms7_modify_spi_reg_bits(st,EN_NEXTTX_TRF,(mode == LMS7_CH_A) ? 0 : 1);
	lms7_mac_set(st,mode);

	//--- afe ---
	lms7_afe_ctrl(st,((mode == LMS7_CH_A) ? 1 : 0)
		,((mode == LMS7_CH_B) ? 1 : 0)
		,((mode == LMS7_CH_A) ? 1 : 0)
		,((mode == LMS7_CH_B) ? 1 : 0));

	lms7_mac_set(st,mode);

	//--- bias -- must write to chA ---//
	lms7_mac_set(st,LMS7_CH_A);

	const int rp_calib_bias = lms7_get_spi_reg_bits(st,RP_CALIB_BIAS);
	// set to default;
	lms7_spi_write(st,LMS7002M_0x0084,0x400);
	lms7_modify_spi_reg_bits(st,RP_CALIB_BIAS,rp_calib_bias);
	lms7_mac_set(st,mode);

	//--- sxt ---
	const double sxt_freq = 500e6;
	res = lms7_sxx_tune_sync(st,false,sxt_freq,false);
	lms7_mac_set(st,mode);
	if(res != 0)
	{
		lms7_log(st,"lms7_sxx_tune_sync(LMS_TX,%f MHz)",sxt_freq / 1e6);
		return(-1);
	}

	st->sxt_freq = sxt_freq;
	//--- TxTSP ---
	set_addrs_to_default(st,mode,LMS7002M_0x0200,LMS7002M_0x020C);

	lms7_modify_spi_reg_bits(st,TSGMODE_TXTSP,1);
	lms7_modify_spi_reg_bits(st,INSEL_TXTSP,1);

	lms7_modify_spi_reg_bits(st,CMIX_BYP_TXTSP,1);
	lms7_modify_spi_reg_bits(st,GFIR3_BYP_TXTSP,1);
	lms7_modify_spi_reg_bits(st,GFIR2_BYP_TXTSP,1);
	lms7_modify_spi_reg_bits(st,GFIR1_BYP_TXTSP,1);
	txtsp_tsg_const(st,mode,0x7fff,0x8000);

	//--- RxTSP ---
	set_addrs_to_default(st,mode,LMS7002M_0x0400,LMS7002M_0x040C);

	lms7_modify_spi_reg_bits(st,AGC_MODE_RXTSP,1);
	lms7_modify_spi_reg_bits(st,AGC_AVG_RXTSP,7);

	lms7_modify_spi_reg_bits(st,GFIR3_BYP_RXTSP,1);
	lms7_modify_spi_reg_bits(st,GFIR2_BYP_RXTSP,1);
	lms7_modify_spi_reg_bits(st,GFIR1_BYP_RXTSP,1);
	lms7_modify_spi_reg_bits(st,CMIX_GAIN_RXTSP,1);

	st->txtsp.reg_0x0c = lms7_spi_read(st,0x208);
	st->rxtsp.reg_0x0c = lms7_spi_read(st,0x40C);

	//--- initial cal tone ---//
	setup_rx_cal_tone(st,mode,50e3);
	return(0);
}

/* Tx calibration loop */
static int tx_cal_loop(struct lms7_state *st,enum lms7_mac_mode mode,double bw,int *reg_ptr,int path,char *reg_name)
{
	lms7_mac_set(st,mode);

	//--- cgen already set prior ---//

	//--- gain selection ---//
	const int rssi_value_50k = cal_gain_selection(st,mode);

	//--- setup calibration tone ---//
	setup_tx_cal_tone(st,mode,bw);

	//--- calibration loop ---//
	size_t iter = 0;
	uint16_t rssi_value = cal_read_rssi(st,mode);
	int adjust = (rssi_value < (rssi_value_50k * 0.7071)) ? -1 : +1;

	while(true)
	{
		if(iter++ == MAX_CAL_LOOP_ITERS)
		{
			lms7_log(st,"failed to converge when calibrating %s",reg_name);
			*reg_ptr = -1;
			return(-1);
		}

		st->ccal_lpflad_tbb += adjust;
		lms7_modify_spi_reg_bits(st,CCAL_LPFLAD_TBB,st->ccal_lpflad_tbb);

		rssi_value = cal_read_rssi(st,mode);
		if((rssi_value > (rssi_value_50k * 0.7071))
			&& (adjust < 0))
		{
			break;
		}

		if((rssi_value < (rssi_value_50k * 0.7071))
			&& (adjust > 0))
		{
			break;
		}

		if((st->ccal_lpflad_tbb != 0)
			&& (st->ccal_lpflad_tbb != 31))
		{
			continue;
		}

		*reg_ptr -= (adjust * 5);

		st->ccal_lpflad_tbb = 16;
		lms7_modify_spi_reg_bits(st,CCAL_LPFLAD_TBB,st->ccal_lpflad_tbb);

		if(path == TBB_S5)
			lms7_modify_spi_reg_bits(st,RCAL_LPFS5_TBB,*reg_ptr);
		else if(path == TBB_LAD)
			lms7_modify_spi_reg_bits(st,RCAL_LPFLAD_TBB,*reg_ptr);
		else if(path == TBB_HBF)
			lms7_modify_spi_reg_bits(st,RCAL_LPFH_TBB,*reg_ptr);
		else
			return(-1);

		if((*reg_ptr < 0)
			|| (*reg_ptr > 255))
		{
			lms7_log(st,"failed to cal %s -> %d",reg_name,*reg_ptr);
			*reg_ptr = -1;
			return(-1);
		}

		rssi_value = cal_read_rssi(st,mode);
		adjust = (rssi_value < (rssi_value_50k * 0.7071)) ?- 1 : +1;
	}

	st->txbw_calibrated[(mode == LMS7_CH_A) ? 0 : 1] = true;
	lms7_log(st,"%s = %d",reg_name,*reg_ptr);
	lms7_log(st,"ccal_lpflad_tbb = %d",st->ccal_lpflad_tbb);
	lms7_log(st,"cg_iamp_tbb = %d",st->cg_iamp_tbb);
	lms7_log(st,"g_pga_rbb = %d",st->g_pga_rbb);
	return(0);
}

static int rx_cal_loop(struct lms7_state *st,enum lms7_mac_mode mode,double bw,int *reg_ptr,int path,char *reg_name)
{
	int res;
	int reg_max = 4095;

	if(path == RBB_HBF)
		reg_max = 255;
	else if(path == RBB_LBF)
		reg_max = 2047;

	lms7_mac_set(st,mode);

	//--- cgen already set prior ---//

	//--- gain selection ---//
	const int rssi_value_50k = cal_gain_selection(st,mode);

	//--- setup calibration tone ---//
	res = setup_rx_cal_tone(st,mode,bw);
	if(res != 0)
		return(res);

	//--- calibration loop ---//
	size_t iter = 0;
	uint16_t rssi_value = cal_read_rssi(st,mode);
	const int adjust = (rssi_value < rssi_value_50k * 0.7071) ? -1 : +1;
	while(true)
	{
		if(iter++ == MAX_CAL_LOOP_ITERS)
		{
			lms7_log(st,"failed to converge when calibrating %s",reg_name);
			*reg_ptr = -1;
			return(-1);
		}

		*reg_ptr += adjust;

		if(path == RBB_LBF)
			lms7_modify_spi_reg_bits(st,C_CTL_LPFL_RBB,*reg_ptr);
		else if(path == RBB_HBF)
			lms7_modify_spi_reg_bits(st,C_CTL_LPFH_RBB,*reg_ptr);
		else
			lms7_modify_spi_reg_bits(st,CFB_TIA_RFE,*reg_ptr);

		rssi_value = cal_read_rssi(st,mode);
		if((rssi_value > rssi_value_50k * 0.7071)
			&& (adjust < 0))
		{
			break;
		}

		if((rssi_value < rssi_value_50k * 0.7071)
			&& (adjust > 0))
		{
			break;
		}

		if((*reg_ptr == 0)
			|| (*reg_ptr == reg_max))
		{
			lms7_log(st,"failed to cal %s -> %d",reg_name,*reg_ptr);
			*reg_ptr = -1;
			return(-1);
		}

	}

	st->rxbw_calibrated[(mode == LMS7_CH_A) ? 0 : 1] = true;
	lms7_log(st,"%s = %d",reg_name,*reg_ptr);
	return(0);
}

/* Perform TBB LPFLAD filter calibration */
static int tx_cal_tbb_lad(struct lms7_state *st,enum lms7_mac_mode mode,double bw)
{
	int res;

	lms7_mac_set(st,mode);
	//--- setup rcal,path ---//
	const double f = (16.0/20.0) * bw / 1e6;
	const double p1 = 1.29858903647958E-16;
	const double p2 = -0.000110746929967704;
	const double p3 = 0.00277593485991029;
	const double p4 = 21.0384293169607;
	const double p5 = -48.4092606238297;
	int rcal_lpflad_tbb = (int)(f*f*f*f*p1 + f*f*f*p2 + f*f*p3 + f*p4 + p5);

	if(rcal_lpflad_tbb < 0)
		rcal_lpflad_tbb = 0;
	else if(rcal_lpflad_tbb > 255)
		rcal_lpflad_tbb = 255;

	lms7_modify_spi_reg_bits(st,LOOPB_TBB,2);

	st->rcal_lpflad_tbb = rcal_lpflad_tbb;
	lms7_modify_spi_reg_bits(st,RCAL_LPFLAD_TBB,rcal_lpflad_tbb);

	//--- calibration ---//
	res = tx_cal_loop(st,mode,bw,&st->rcal_lpflad_tbb,TBB_LAD,"rcal_lpflad_tbb");
	return(res);
}

/* Perform TBB LPFH filter calibration */
static int tx_cal_tbb_lpfh(struct lms7_state *st,enum lms7_mac_mode mode,double bw)
{
	int res;

	lms7_mac_set(st,mode);

	//--- setup rcal,path ---//
	const double f = bw / 1e6;
	const double p1 = 1.10383261611112E-06;
	const double p2 = -0.000210800032517545;
	const double p3 = 0.0190494874803309;
	const double p4 = 1.43317445923528;
	const double p5 = -47.6950779298333;
	int rcal_lpfh_tbb = (int)(f*f*f*f*p1 + f*f*f*p2 + f*f*p3 + f*p4 + p5);

	if(rcal_lpfh_tbb < 0)
		rcal_lpfh_tbb = 0;
	else if(rcal_lpfh_tbb > 255)
		rcal_lpfh_tbb = 255;

	lms7_modify_spi_reg_bits(st,LOOPB_TBB,3);

	st->rcal_lpfh_tbb = rcal_lpfh_tbb;
	lms7_modify_spi_reg_bits(st,RCAL_LPFH_TBB,rcal_lpfh_tbb);

	//--- calibration ---//
	res = tx_cal_loop(st,mode,bw,&st->rcal_lpfh_tbb,TBB_HBF,"rcal_lpfh_tbb");
	return(res);
}

/* Perform RFE TIA filter calibration */
static int rx_cal_tia_rfe(struct lms7_state *st,enum lms7_mac_mode mode,double bw)
{
	int res;

	lms7_mac_set(st,mode);
	int g_tia_rfe_user = lms7_get_spi_reg_bits(st,G_TIA_RFE);

	//--- check filter bounds ---//
	if((bw < 0.5e6)
		|| (bw > 60e6))
	{
		lms7_log(st,"TIA bandwidth not in range[0.5 to 60 MHz]");
		return(-1);
	}

	//--- cfb_tia_rfe,ccomp_tia_rfe ---//
	st->cfb_tia_rfe = 0;
	st->ccomp_tia_rfe = 0;

	if((g_tia_rfe_user == 3)
		|| (g_tia_rfe_user == 2))
	{
		st->cfb_tia_rfe = (int)(1680e6 / bw - 10);
		st->ccomp_tia_rfe = st->cfb_tia_rfe / 100;
	}
	else if(g_tia_rfe_user == 1)
	{
		st->cfb_tia_rfe = (int)(5400e6 / bw - 10);
		st->ccomp_tia_rfe = (int)(st->cfb_tia_rfe/100 + 1);
	}
	else
	{
		lms7_log(st,"g_tia_rfe must be [1,2,or 3],got %d",g_tia_rfe_user);
		return(-1);
	}

	if(st->ccomp_tia_rfe > 15)
		st->ccomp_tia_rfe = 15;

	lms7_modify_spi_reg_bits(st,CFB_TIA_RFE,st->cfb_tia_rfe);
	lms7_modify_spi_reg_bits(st,CCOMP_TIA_RFE,st->ccomp_tia_rfe);

	//--- rcomp_tia_rfe ---//
	st->rcomp_tia_rfe = (int)(15 - 2 * st->cfb_tia_rfe / 100);
	if(st->rcomp_tia_rfe < 0)
		st->rcomp_tia_rfe = 0;

	lms7_modify_spi_reg_bits(st,RCOMP_TIA_RFE,st->rcomp_tia_rfe);

	//--- rbb path ---//
	lms7_modify_spi_reg_bits(st,INPUT_CTL_PGA_RBB,2);
	lms7_modify_spi_reg_bits(st,PD_LPFL_RBB,0);

	//--- calibration ---//
	res = rx_cal_loop(st,mode,bw,&st->cfb_tia_rfe,-1,"cfb_tia_rfe");
	return(res);
}

/***********************************************************************
 * Perform RBB LPFL filter calibration
 **********************************************************************/
static int rx_cal_rbb_lpfl(struct lms7_state *st,enum lms7_mac_mode mode,double bw)
{
	int res;

	lms7_mac_set(st,mode);

	//--- check filter bounds ---//
	if((bw < 0.5e6)
		|| (bw > 20e6))
	{
		lms7_log(st,"LPFL bandwidth not in range[0.5 to 20 MHz]");
		return(-1);
	}

	//--- c_ctl_lpfl_rbb,rcc_ctl_lpfl_rbb ---//
	st->c_ctl_lpfl_rbb = (int)(2160e6 / bw - 103);
	st->rcc_ctl_lpfl_rbb = 0;

	if(bw > 15e6)
		st->rcc_ctl_lpfl_rbb = 5;
	else if(bw > 10e6)
		st->rcc_ctl_lpfl_rbb = 4;
	else if(bw > 5e6)
		st->rcc_ctl_lpfl_rbb = 3;
	else if(bw > 3e6)
		st->rcc_ctl_lpfl_rbb = 2;
	else if(bw > 1.4e6)
		st->rcc_ctl_lpfl_rbb = 1;
	else
		st->rcc_ctl_lpfl_rbb = 0;

	lms7_modify_spi_reg_bits(st,RCC_CTL_LPFL_RBB,st->rcc_ctl_lpfl_rbb);

	//--- tia rfe registers ---//
	st->cfb_tia_rfe = 15;
	st->ccomp_tia_rfe = 1;
	lms7_modify_spi_reg_bits(st,CFB_TIA_RFE,st->cfb_tia_rfe);
	lms7_modify_spi_reg_bits(st,CCOMP_TIA_RFE,st->ccomp_tia_rfe);

	lms7_modify_spi_reg_bits(st,G_TIA_RFE,1);
	st->rcomp_tia_rfe = 15;
	lms7_modify_spi_reg_bits(st,RCOMP_TIA_RFE,st->rcomp_tia_rfe);

	//--- calibration ---//
	res = rx_cal_loop(st,mode,bw,&st->c_ctl_lpfl_rbb,RBB_LBF,"c_ctl_lpfl_rbb");
	return(res);
}

/***********************************************************************
 * Perform RBB LPFH filter calibration
 **********************************************************************/
static int rx_cal_rbb_lpfh(struct lms7_state *st,enum lms7_mac_mode mode,double bw)
{
	int res;

	lms7_mac_set(st,mode);

	//--- check filter bounds ---//
	if((bw < 20e6)
		|| (bw > 60e6))
	{
		lms7_log(st,"LPFH bandwidth not in range[0.5 to 60 MHz]");
		return(-1);
	}

	//--- c_ctl_lpfl_rbb,rcc_ctl_lpfl_rbb ---//
	st->c_ctl_lpfh_rbb = (int)(6000e6 / bw - 50);

	st->rcc_ctl_lpfh_rbb = (int)(bw / 10e6 - 3);
	if(st->rcc_ctl_lpfh_rbb < 0)
		st->rcc_ctl_lpfh_rbb = 0;

	lms7_modify_spi_reg_bits(st,RCC_CTL_LPFH_RBB,st->rcc_ctl_lpfh_rbb);

	//--- tia rfe registers and rbb ---//
	st->cfb_tia_rfe = 15;
	st->ccomp_tia_rfe = 1;
	lms7_modify_spi_reg_bits(st,CFB_TIA_RFE,st->cfb_tia_rfe);
	lms7_modify_spi_reg_bits(st,CCOMP_TIA_RFE,st->ccomp_tia_rfe);

	lms7_modify_spi_reg_bits(st,G_TIA_RFE,1);

	st->rcomp_tia_rfe = 15;
	lms7_modify_spi_reg_bits(st,RCOMP_TIA_RFE,st->rcomp_tia_rfe);

	lms7_modify_spi_reg_bits(st,PD_LPFH_RBB,0);
	lms7_modify_spi_reg_bits(st,PD_LPFL_RBB,1);

	lms7_modify_spi_reg_bits(st,INPUT_CTL_PGA_RBB,1);

	//--- calibration ---//
	res = rx_cal_loop(st,mode,bw,&st->c_ctl_lpfh_rbb,RBB_HBF,"c_ctl_lpfh_rbb");
	return(res);
}

/* Tx calibration dispatcher */
int lms7_tbb_set_filter_bw(struct lms7_state *st,enum lms7_mac_mode mode,double bw)
{
	int res = 0;

	// Save register map
	lms7_store_register(st);
	lms7_mac_set(st,mode);

	const int path = (bw < 20e6) ? TBB_LAD : TBB_HBF;

	// Clocking configuration
	res = lms7_cgen_tune_sync(st,st->cgen_freq,st->txdiv);
	if(res != 0)
	{
		lms7_log(st,"lms7_cgen_tune_sync failed %d",res);
		goto done;
	}

	// Load initial calibration state
	res = tx_cal_init(st,mode);
	if(res != 0)
	{
		lms7_log(st,"tx_cal_init failed %d",res);
		goto done;
	}

	lms7_tbb_set_path(st,path);

	// TBB LPF calibration
	if(path == TBB_LAD)
		res = tx_cal_tbb_lad(st,mode,bw);

	if(path == TBB_HBF)
		res = tx_cal_tbb_lpfh(st,mode,bw);

	if(res != 0)
	{
		lms7_log(st,"tx_cal_tbb_xxx() failed");
		goto done;
	}

	done:
	// stash tbb calibration results
	// restore original register values
	lms7_restore_register(st);
	lms7_mac_set(st,mode);

	// apply tbb calibration results
	if(res == 0)
		lms7_tbb_apply_calibration(st,mode,path);
	return(0);
}

void lms7_tbb_apply_calibration(struct lms7_state *st,enum lms7_mac_mode mode,int path)
{
	if(st->txbw_calibrated[(mode == LMS7_CH_A) ? 0 : 1] == true)
	{
		// set the filter selection
		lms7_tbb_set_path(st,path);
		// apply tbb calibration results
		lms7_modify_spi_reg_bits(st,CG_IAMP_TBB,st->cg_iamp_tbb);
		lms7_modify_spi_reg_bits(st,ICT_IAMP_FRP_TBB,1);
		lms7_modify_spi_reg_bits(st,ICT_IAMP_GG_FRP_TBB,6);

		lms7_modify_spi_reg_bits(st,RCAL_LPFLAD_TBB,st->rcal_lpflad_tbb);
		lms7_modify_spi_reg_bits(st,RCAL_LPFH_TBB,st->rcal_lpfh_tbb);

		lms7_modify_spi_reg_bits(st,CCAL_LPFLAD_TBB,st->ccal_lpflad_tbb);
//		lms7_modify_spi_reg_bits(st,RCAL_LPFS5_TBB,st->rcal_lpfs5_tbb);
	}
	
	lms7_log(st,"TBB: ccal_lpflad_tbb = %d",lms7_get_spi_reg_bits(st,CCAL_LPFLAD_TBB));
	lms7_log(st,"TBB: cg_iamp_tbb = %d",lms7_get_spi_reg_bits(st,CG_IAMP_TBB));
	lms7_log(st,"TBB: g_pga_rbb = %d",lms7_get_spi_reg_bits(st,G_PGA_RBB));
	if(path == TBB_LAD)
		lms7_log(st,"TBB: rcal_lpflad_tbb = %d",lms7_get_spi_reg_bits(st,RCAL_LPFLAD_TBB));

	if(path == TBB_HBF)
		lms7_log(st,"TBB: rcal_lpfh_tbb = %d",lms7_get_spi_reg_bits(st,RCAL_LPFH_TBB));
}

/* Rx calibration dispatcher */
int lms7_rbb_set_filter_bw(struct lms7_state *st,enum lms7_mac_mode mode,double bw)
{
	int res;
	int path = (bw < 20e6) ? RBB_LBF : RBB_HBF;

	// Save register map
	lms7_store_register(st);
	lms7_mac_set(st,mode);

	// Clocking configuration
	res = lms7_cgen_tune_sync(st,st->cgen_freq,st->txdiv);

	// Load initial calibration state
	res = rx_cal_init(st,mode);
	if(res)
	{
		lms7_log(st,"tx_cal_init failed %d",res);
		goto done;
	}

	// RFE TIA calibration
	res = rx_cal_tia_rfe(st,mode,bw);
	if(res != 0)
	{
		lms7_log(st,"rx_cal_tia_rfe() failed");
		goto done;
	}

	// Initialize calibration again for LPF
	res = rx_cal_init(st,mode);
	if(res != 0)
	{
		lms7_log(st,"rx_cal_init() failed");
		goto done;
	}

	// RBB LPF calibration
	if(path == RBB_LBF)
		res = rx_cal_rbb_lpfl(st,mode,bw);

	if(path == RBB_HBF)
		res = rx_cal_rbb_lpfh(st,mode,bw);

	if(res != 0)
	{
		lms7_log(st,"rx_cal_rbb_xxx() failed");
		goto done;
	}

	done:
	// stash tia + rbb calibration results
	// restore original register values
	lms7_restore_register(st);
	lms7_mac_set(st,mode);

	lms7_rbb_apply_calibration(st,mode,path);
	return(0);
}

void lms7_rbb_apply_calibration(struct lms7_state *st,enum lms7_mac_mode mode,int path)
{
	if(st->rxbw_calibrated[(mode == LMS7_CH_A) ? 0 : 1] == true)
	{
		// set the filter selection
		lms7_rbb_set_path(st,path);
		// apply tia calibration results
		lms7_modify_spi_reg_bits(st,ICT_TIAMAIN_RFE,2);
		lms7_modify_spi_reg_bits(st,ICT_TIAOUT_RFE,2);
		lms7_modify_spi_reg_bits(st,RFB_TIA_RFE,16);

		lms7_modify_spi_reg_bits(st,CFB_TIA_RFE,st->cfb_tia_rfe);
		lms7_modify_spi_reg_bits(st,CCOMP_TIA_RFE,st->ccomp_tia_rfe);
		lms7_modify_spi_reg_bits(st,RCOMP_TIA_RFE,st->rcomp_tia_rfe);

		// apply rbb calibration results
		lms7_modify_spi_reg_bits(st,RCC_CTL_LPFL_RBB,st->rcc_ctl_lpfl_rbb);
		lms7_modify_spi_reg_bits(st,C_CTL_LPFL_RBB,st->c_ctl_lpfl_rbb);
		lms7_modify_spi_reg_bits(st,RCC_CTL_LPFH_RBB,st->rcc_ctl_lpfh_rbb);
		lms7_modify_spi_reg_bits(st,C_CTL_LPFH_RBB,st->c_ctl_lpfh_rbb);

		lms7_modify_spi_reg_bits(st,ICT_PGA_OUT_RBB,20);
		lms7_modify_spi_reg_bits(st,ICT_PGA_IN_RBB,20);
		lms7_modify_spi_reg_bits(st,R_CTL_LPF_RBB,16);
	}
	
	lms7_log(st,"RBB: rcc_ctl_lpfh_rbb = %d",lms7_get_spi_reg_bits(st,RCC_CTL_LPFH_RBB));
	lms7_log(st,"RBB: rcc_ctl_lpfl_rbb = %d",lms7_get_spi_reg_bits(st,RCC_CTL_LPFL_RBB));
	lms7_log(st,"RBB: c_ctl_lpfh_rbb = %d",lms7_get_spi_reg_bits(st,C_CTL_LPFH_RBB));
	lms7_log(st,"RBB: c_ctl_lpfl_rbb = %d",lms7_get_spi_reg_bits(st,C_CTL_LPFL_RBB));

	lms7_log(st,"RBB: ccomp_tia_rfe = %d",lms7_get_spi_reg_bits(st,CCOMP_TIA_RFE));
	lms7_log(st,"RBB: cfb_tia_rfe = %d",lms7_get_spi_reg_bits(st,CFB_TIA_RFE));
	lms7_log(st,"RBB: rcomp_tia_rfe = %d",lms7_get_spi_reg_bits(st,RCOMP_TIA_RFE));
}

