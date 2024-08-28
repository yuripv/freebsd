/**
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2017 aQuantia Corporation. All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   (1) Redistributions of source code must retain the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer.
 *
 *   (2) Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 *
 *   (3) The name of the author may not be used to endorse or promote
 *   products derived from this software without specific prior
 *   written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @file aq_fw2x.c
 * Firmware v2.x specific functions.
 * @date 2017.12.11  @author roman.agafonov@aquantia.com
 */

#include "aq.h"
#include "aq_hw_llh.h"
#include "aq_hw_llh_internal.h"

enum {
	CAPS_LO_10BASET_HD = 0,
	CAPS_LO_10BASET_FD,
	CAPS_LO_100BASETX_HD,
	CAPS_LO_100BASET4_HD,
	CAPS_LO_100BASET2_HD,
	CAPS_LO_100BASETX_FD,
	CAPS_LO_100BASET2_FD,
	CAPS_LO_1000BASET_HD,
	CAPS_LO_1000BASET_FD,
	CAPS_LO_2P5GBASET_FD,
	CAPS_LO_5GBASET_FD,
	CAPS_LO_10GBASET_FD,
};

enum {
	CAPS_HI_RESERVED1 = 0,
	CAPS_HI_10BASET_EEE,
	CAPS_HI_RESERVED2,
	CAPS_HI_PAUSE,
	CAPS_HI_ASYMMETRIC_PAUSE,
	CAPS_HI_100BASETX_EEE,
	CAPS_HI_RESERVED3,
	CAPS_HI_RESERVED4,
	CAPS_HI_1000BASET_FD_EEE,
	CAPS_HI_2P5GBASET_FD_EEE,
	CAPS_HI_5GBASET_FD_EEE,
	CAPS_HI_10GBASET_FD_EEE,
	CAPS_HI_RESERVED5,
	CAPS_HI_RESERVED6,
	CAPS_HI_RESERVED7,
	CAPS_HI_RESERVED8,
	CAPS_HI_RESERVED9,
	CAPS_HI_CABLE_DIAG,
	CAPS_HI_TEMPERATURE,
	CAPS_HI_DOWNSHIFT,
	CAPS_HI_PTP_AVB_EN,
	CAPS_HI_MEDIA_DETECT,
	CAPS_HI_LINK_DROP,
	CAPS_HI_SLEEP_PROXY,
	CAPS_HI_WOL,
	CAPS_HI_MAC_STOP,
	CAPS_HI_EXT_LOOPBACK,
	CAPS_HI_INT_LOOPBACK,
	CAPS_HI_EFUSE_AGENT,
	CAPS_HI_WOL_TIMER,
	CAPS_HI_STATISTICS,
	CAPS_HI_TRANSACTION_ID,
};

typedef enum {
	FW2X_RATE_100M = 0x20,
	FW2X_RATE_1G = 0x100,
	FW2X_RATE_2G5 = 0x200,
	FW2X_RATE_5G = 0x400,
	FW2X_RATE_10G = 0x800,
} aq_fw2x_rate_t;


typedef struct {
	uint32_t	uprc;
	uint32_t	mprc;
	uint32_t	bprc;
	uint32_t	erpt;
	uint32_t	uptc;
	uint32_t	mptc;
	uint32_t	bptc;
	uint32_t	erpr;
	uint32_t	mbtc;
	uint32_t	bbtc;
	uint32_t	mbrc;
	uint32_t	bbrc;
	uint32_t	ubrc;
	uint32_t	ubtc;
	uint32_t	ptc;
	uint32_t	prc;
} fw2x_msm_statistics_t;

typedef struct {
	uint32_t	lane_data[4];
} fw2x_phy_cable_diag_data_t;

typedef struct {
	uint32_t	caps_lo;
	uint32_t	caps_hi;
} fw2x_caps_t;

typedef struct {
	uint32_t	version;
	uint32_t	transaction_id;
	int32_t		error;
	fw2x_msm_statistics_t msm;
	uint16_t	phy_h_bit;
	uint16_t	phy_fault_code;
	int16_t		phy_temperature;
	uint8_t		cable_len;
	uint8_t		reserved1;
	fw2x_phy_cable_diag_data_t diag_data;
	uint32_t	reserved[8];
	fw2x_caps_t	caps;
} fw2x_mailbox_t;

/* EEE caps */
#define FW2X_FW_CAP_EEE_100M	(1ull << (32 + CAPS_HI_100BASETX_EEE))
#define FW2X_FW_CAP_EEE_1G	(1ull << (32 + CAPS_HI_1000BASET_FD_EEE))
#define FW2X_FW_CAP_EEE_2G5	(1ull << (32 + CAPS_HI_2P5GBASET_FD_EEE))
#define FW2X_FW_CAP_EEE_5G	(1ull << (32 + CAPS_HI_5GBASET_FD_EEE))
#define FW2X_FW_CAP_EEE_10G	(1ull << (32 + CAPS_HI_10GBASET_FD_EEE))

/* Flow Control */
#define FW2X_FW_CAP_PAUSE	(1ull << (32 + CAPS_HI_PAUSE))
#define FW2X_FW_CAP_ASYM_PAUSE	(1ull << (32 + CAPS_HI_ASYMMETRIC_PAUSE))
/* Link Drop */
#define FW2X_CAP_LINK_DROP	(1ull << (32 + CAPS_HI_LINK_DROP))
/* MSM Statistics */
#define FW2X_CAP_STATISTICS	(1ull << (32 + CAPS_HI_STATISTICS))

#define	FW2X_RATE_MASK	\
	(FW2X_RATE_100M | FW2X_RATE_1G | FW2X_RATE_2G5 | \
	FW2X_RATE_5G | FW2X_RATE_10G)
#define	FW2X_EEE_MASK	\
	(FW2X_FW_CAP_EEE_100M | FW2X_FW_CAP_EEE_1G | FW2X_FW_CAP_EEE_2G5 | \
	FW2X_FW_CAP_EEE_5G | FW2X_FW_CAP_EEE_10G)

#define FW2X_MPI_LED_ADDR           0x31c
#define FW2X_MPI_CONTROL_ADDR       0x368
#define FW2X_MPI_STATE_ADDR         0x370

#define FW2X_FW_MIN_VER_LED 0x03010026U

#define FW2X_LED_BLINK    0x2U
#define FW2X_LED_DEFAULT  0x0U

// Firmware v2-3.x specific functions.
int fw2x_reset(aq_hw_t *);
int fw2x_set_mode(aq_hw_t *, aq_hw_fw_mpi_state_t, aq_fw_link_speed_t);
int fw2x_get_mode(aq_hw_t *, aq_hw_fw_mpi_state_t *, aq_fw_link_speed_t *,
    aq_fw_link_fc_t *);
int fw2x_get_mac_addr(aq_hw_t *, uint8_t *);
int fw2x_get_stats(aq_hw_t *, aq_hw_stats_t *);

static uint64_t
read64_(aq_hw_t *hw, uint32_t addr)
{
    uint64_t lo = AQ_READ_REG(hw, addr);
    uint64_t hi = AQ_READ_REG(hw, addr + 4);

    return (lo | (hi << 32));
}

static uint64_t
get_mpi_ctrl(aq_hw_t *hw)
{
	return (read64_(hw, FW2X_MPI_CONTROL_ADDR));
}

static uint64_t
get_mpi_state(aq_hw_t *hw)
{
	return (read64_(hw, FW2X_MPI_STATE_ADDR));
}

static void
set_mpi_ctrl(aq_hw_t *hw, uint64_t value)
{
	AQ_WRITE_REG(hw, FW2X_MPI_CONTROL_ADDR, (uint32_t)value);
	AQ_WRITE_REG(hw, FW2X_MPI_CONTROL_ADDR + 4, (uint32_t)(value >> 32));
}

int
fw2x_reset(aq_hw_t *hw)
{
	fw2x_caps_t caps = {0};

	if (aq_hw_fw_downld_dwords(hw, hw->mbox_addr +
	    offsetof(fw2x_mailbox_t, caps), (uint32_t *)&caps,
	    sizeof(caps) / sizeof(uint32_t)) == 0) {
		hw->fw_caps = caps.caps_lo | ((uint64_t)caps.caps_hi << 32);
	} else {
		device_printf(hw->dev, "failed to get fw caps mask\n");
	}

	return (0);
}

static aq_fw2x_rate_t
link_speed_mask_to_fw2x(uint32_t speed)
{
	uint32_t rate = 0;

	if (speed & AQ_FW_10G)
		rate |= FW2X_RATE_10G;
	if (speed & AQ_FW_5G)
		rate |= FW2X_RATE_5G;
	if (speed & AQ_FW_2G5)
		rate |= FW2X_RATE_2G5;
	if (speed & AQ_FW_1G)
		rate |= FW2X_RATE_1G;
	if (speed & AQ_FW_100M)
		rate |= FW2X_RATE_100M;

	return ((aq_fw2x_rate_t)rate);
}

int
fw2x_set_mode(aq_hw_t *hw, aq_hw_fw_mpi_state_t mode, aq_fw_link_speed_t speed)
{
	uint64_t mpi_ctrl = get_mpi_ctrl(hw);

	switch (mode) {
	case MPI_INIT:
		mpi_ctrl &= ~FW2X_RATE_MASK;
		mpi_ctrl |= link_speed_mask_to_fw2x(speed);
		mpi_ctrl &= ~FW2X_CAP_LINK_DROP;
		/* TODO */
#if 0
		if (pHal->pCfg->eee)
			mpi_ctrl |= FW2X_EEE_MASK;
#endif
		if (hw->fc.fc_rx)
			mpi_ctrl |= FW2X_FW_CAP_PAUSE;
		if (hw->fc.fc_tx)
			mpi_ctrl |= FW2X_FW_CAP_ASYM_PAUSE;
		break;
	case MPI_DEINIT:
		mpi_ctrl &= ~(FW2X_RATE_MASK | FW2X_EEE_MASK);
		mpi_ctrl &= ~(FW2X_FW_CAP_PAUSE | FW2X_FW_CAP_ASYM_PAUSE);
		break;
	default:
		device_printf(hw->dev, "unknown MPI state %d", mode);
		return (1);
	}

	set_mpi_ctrl(hw, mpi_ctrl);
	return (0);
}

int
fw2x_get_mode(aq_hw_t *hw, aq_hw_fw_mpi_state_t *mode,
    aq_fw_link_speed_t *link_speed, aq_fw_link_fc_t *fc)
{
	aq_fw_link_speed_t speed = AQ_FW_NONE;
	uint64_t mpi_state = get_mpi_state(hw);
	uint32_t rates = mpi_state & FW2X_RATE_MASK;

	if (mode != NULL) {
		uint64_t mpi_ctrl = get_mpi_ctrl(hw);
		if (mpi_ctrl & FW2X_RATE_MASK)
			*mode = MPI_INIT;
		else
			*mode = MPI_DEINIT;
	}

	if (rates & FW2X_RATE_10G)
		speed = AQ_FW_10G;
	else if (rates & FW2X_RATE_5G)
		speed = AQ_FW_5G;
	else if (rates & FW2X_RATE_2G5)
		speed = AQ_FW_2G5;
	else if (rates & FW2X_RATE_1G)
		speed = AQ_FW_1G;
	else if (rates & FW2X_RATE_100M)
		speed = AQ_FW_100M;

	if (link_speed != NULL)
		*link_speed = speed;
	*fc = (mpi_state & (FW2X_FW_CAP_PAUSE | FW2X_FW_CAP_ASYM_PAUSE)) >>
	    (32 + CAPS_HI_PAUSE);

	return (0);
}

int
fw2x_get_mac_addr(aq_hw_t *hw, uint8_t *mac)
{
	uint32_t mac_addr[2];
	uint32_t efuse_shadow_addr = AQ_READ_REG(hw, 0x364);

	if (efuse_shadow_addr == 0)
		return (1);

	if (aq_hw_fw_downld_dwords(hw, efuse_shadow_addr + (40 * 4),
	    mac_addr, ARRAY_SIZE(mac_addr)) != 0) {
		mac_addr[0] = 0;
		mac_addr[1] = 0;
		return (1);
	}

	mac_addr[0] = bswap32(mac_addr[0]);
	mac_addr[1] = bswap32(mac_addr[1]);

	memcpy(mac, (uint8_t*)mac_addr, ETH_MAC_LEN);

	return (0);
}

static inline void
fw2x_stats_to_fw_stats(aq_hw_stats_t *dst, const fw2x_msm_statistics_t *src)
{
    dst->uprc = src->uprc;
    dst->mprc = src->mprc;
    dst->bprc = src->bprc;
    dst->erpt = src->erpt;
    dst->uptc = src->uptc;
    dst->mptc = src->mptc;
    dst->bptc = src->bptc;
    dst->erpr = src->erpr;
    dst->mbtc = src->mbtc;
    dst->bbtc = src->bbtc;
    dst->mbrc = src->mbrc;
    dst->bbrc = src->bbrc;
    dst->ubrc = src->ubrc;
    dst->ubtc = src->ubtc;
    dst->ptc = src->ptc;
    dst->prc = src->prc;
}

static int
toggle_mpi_ctrl_and_wait(aq_hw_t *hw, uint64_t mask, uint32_t timeout_ms,
    uint32_t try_count)
{
	uint64_t ctrl = get_mpi_ctrl(hw);
	uint64_t state = get_mpi_state(hw);

	/* First, check that control and state values are consistent */
	if ((ctrl & mask) != (state & mask)) {
		device_printf(hw->dev, "MPI control (%#llx) and state (%#llx) "
		    "are not consistent for mask %#llx\n",
		    (unsigned long long)ctrl, (unsigned long long)state,
		    (unsigned long long)mask);
		return (1);
	}

	/* Invert bits (toggle) in control register */
	ctrl ^= mask;
	set_mpi_ctrl(hw, ctrl);
	/* Clear all bits except masked */
	ctrl &= mask;
	/* Wait for fw to reflect change in state register */
	while (try_count-- != 0) {
		if ((get_mpi_state(hw) & mask) == ctrl)
			return (0);
		msec_delay(timeout_ms);
	}

	device_printf(hw->dev, "timeout while waiting for response in state "
	    "register for bit %#llx\n", (unsigned long long)mask);
	return (1);
}

int
fw2x_get_stats(aq_hw_t *hw, aq_hw_stats_t *stats)
{
	fw2x_msm_statistics_t fw2x_stats = {0};

	if ((hw->fw_caps & FW2X_CAP_STATISTICS) == 0) {
		device_printf(hw->dev, "statistics not supported by fw\n");
		return (1);
	}

	/* Ask fw to update statistics */
	if (toggle_mpi_ctrl_and_wait(hw, FW2X_CAP_STATISTICS, 1, 25) != 0) {
		device_printf(hw->dev, "statistics update timeout\n");
		return (1);
	}

	if (aq_hw_fw_downld_dwords(hw, hw->mbox_addr +
	    offsetof(fw2x_mailbox_t, msm), (uint32_t *)&fw2x_stats,
	    sizeof(fw2x_stats) / sizeof(uint32_t)) != 0) {
		device_printf(hw->dev, "failed to download statistics data");
		return (1);
	}

	fw2x_stats_to_fw_stats(stats, &fw2x_stats);

	return (0);
}

static int
fw2x_led_control(aq_hw_t *hw, uint32_t onoff)
{
	if (FW2X_FW_MIN_VER_LED < hw->fw_version.raw) {
		AQ_WRITE_REG(hw, FW2X_MPI_LED_ADDR, onoff ?
		    FW2X_LED_BLINK | FW2X_LED_BLINK << 2 | FW2X_LED_BLINK << 4 :
		    FW2X_LED_DEFAULT);
	}

	return (0);
}

aq_fw_ops_t aq_fw2x_ops = {
	.reset = fw2x_reset,
	.set_mode = fw2x_set_mode,
	.get_mode = fw2x_get_mode,
	.get_mac_addr = fw2x_get_mac_addr,
	.get_stats = fw2x_get_stats,
	.led_control = fw2x_led_control,
};
