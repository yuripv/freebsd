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
 */

#include "aq.h"
#include "aq_hw_llh.h"
#include "aq_hw_llh_internal.h"

#define FW1X_MPI_CONTROL_ADR    0x368
#define FW1X_MPI_STATE_ADR      0x36C


typedef enum fw1x_mode {
    FW1X_MPI_DEINIT = 0,
    FW1X_MPI_RESERVED = 1,
    FW1X_MPI_INIT = 2,
    FW1X_MPI_POWER = 4,
} fw1x_mode;

typedef enum aq_fw1x_rate {
    FW1X_RATE_10G   = 1 << 0,
    FW1X_RATE_5G    = 1 << 1,
    FW1X_RATE_5GSR  = 1 << 2,
    FW1X_RATE_2G5   = 1 << 3,
    FW1X_RATE_1G    = 1 << 4,
    FW1X_RATE_100M  = 1 << 5,
    FW1X_RATE_INVALID = 1 << 6,
} aq_fw1x_rate;

typedef union fw1x_state_reg {
    uint32_t val;
    struct {
        uint8_t mode;
        uint8_t reserved1;
        uint8_t speed;
        uint8_t reserved2 : 1;
        uint8_t disableDirtyWake : 1;
        uint8_t reserved3 : 2;
        uint8_t downshift : 4;
    };
} fw1x_state_reg;

int fw1x_reset(struct aq_hw* hw);

int fw1x_set_mode(aq_hw_t *, aq_hw_fw_mpi_state_t, aq_fw_link_speed_t);
int fw1x_get_mode(aq_hw_t *, aq_hw_fw_mpi_state_t *, aq_fw_link_speed_t *,
    aq_fw_link_fc_t *);
int fw1x_get_mac_addr(aq_hw_t *, uint8_t *);
int fw1x_get_stats(aq_hw_t *, aq_hw_stats_t *);

static
fw1x_mode mpi_mode_to_fw1x_(aq_hw_fw_mpi_state_t mode)
{
    switch (mode) {
    case MPI_DEINIT:
        return (FW1X_MPI_DEINIT);

    case MPI_INIT:
        return (FW1X_MPI_INIT);

    case MPI_POWER:
        return (FW1X_MPI_POWER);

    case MPI_RESET:
        return (FW1X_MPI_RESERVED);
    }

    /*
     * We shouldn't get here.
     */

    return (FW1X_MPI_RESERVED);
}

static aq_fw1x_rate
link_speed_mask_to_fw1x(uint32_t speed)
{
	uint32_t rate = 0;

	if (speed & AQ_FW_10G)
		rate |= FW1X_RATE_10G;
	if (speed & AQ_FW_5G)
		rate |= FW1X_RATE_5G | FW1X_RATE_5GSR;
	if (speed & AQ_FW_2G5)
		rate |= FW1X_RATE_2G5;
	if (speed & AQ_FW_1G)
		rate |= FW1X_RATE_1G;
	if (speed & AQ_FW_100M)
		rate |= FW1X_RATE_100M;

	return ((aq_fw1x_rate)rate);
}

static aq_fw_link_speed_t
fw1x_rate_to_link_speed(aq_fw1x_rate rate)
{
	switch (rate) {
	case FW1X_RATE_10G:
		return (AQ_FW_10G);
	case FW1X_RATE_5G:
	case FW1X_RATE_5GSR:
		return (AQ_FW_5G);
	case FW1X_RATE_2G5:
		return (AQ_FW_2G5);
	case FW1X_RATE_1G:
		return (AQ_FW_1G);
	case FW1X_RATE_100M:
		return (AQ_FW_100M);
	case FW1X_RATE_INVALID:
	default:
		return (AQ_FW_NONE);
	}
}

int
fw1x_reset(aq_hw_t *hal)
{
    uint32_t tid0 = ~0u; /*< Initial value of MBOX transactionId. */
    aq_hw_fw_mbox_t mbox;
    const int retryCount = 1000;

    for (int i = 0; i < retryCount; ++i) {
        // Read the beginning of Statistics structure to capture the Transaction ID.
        aq_hw_fw_downld_dwords(hal, hal->mbox_addr, (uint32_t*)&mbox,
            (uint32_t)((char*)&mbox.stats - (char*)&mbox) / sizeof(uint32_t));

        // Successfully read the stats.
        if (tid0 == ~0U) {
            // We have read the initial value.
            tid0 = mbox.transaction_id;
            continue;
        } else if (mbox.transaction_id != tid0) {
            /*
             * Compare transaction ID to initial value.
             * If it's different means f/w is alive. We're done.
             */

            return (0);
        }

        /*
         * Transaction ID value haven't changed since last time.
         * Try reading the stats again.
         */
        usec_delay(10);
    }

    printf("F/W 1.x reset finalize timeout");
    return (-EBUSY);
}

int
fw1x_set_mode(aq_hw_t *hw, aq_hw_fw_mpi_state_t mode, aq_fw_link_speed_t speed)
{
    union fw1x_state_reg state = { 0 };

    state.mode = mpi_mode_to_fw1x_(mode);
    state.speed = link_speed_mask_to_fw1x(speed);

    AQ_WRITE_REG(hw, FW1X_MPI_CONTROL_ADR, state.val);

    return (0);
}

int
fw1x_get_mode(aq_hw_t *hw, aq_hw_fw_mpi_state_t *mode,
    aq_fw_link_speed_t *speed, aq_fw_link_fc_t *fc)
{
	aq_hw_fw_mpi_state_t md = MPI_DEINIT;
	union fw1x_state_reg state = {
		.val = AQ_READ_REG(hw, AQ_HW_MPI_STATE_ADR),
	};

	switch (state.mode) {
	case FW1X_MPI_DEINIT:
		md = MPI_DEINIT;
		break;
	case FW1X_MPI_RESERVED:
		md = MPI_RESET;
		break;
	case FW1X_MPI_INIT:
		md = MPI_INIT;
		break;
	case FW1X_MPI_POWER:
		md = MPI_POWER;
		break;
	}

	if (mode != NULL)
		*mode = md;
	if (speed != NULL)
		*speed = fw1x_rate_to_link_speed(state.speed);
	*fc = aq_fw_fc_none;

	return (0);
}

int
fw1x_get_mac_addr(aq_hw_t *hw, uint8_t *mac)
{
	uint32_t mac_addr[2];
	int err = -EFAULT;

    uint32_t efuse_shadow_addr = AQ_READ_REG(hw, 0x374);
    if (efuse_shadow_addr == 0) {
        printf("couldn't read eFUSE Shadow Address");
        return (-EFAULT);
    }

    err = aq_hw_fw_downld_dwords(hw, efuse_shadow_addr + (40 * 4),
        mac_addr, ARRAY_SIZE(mac_addr));
    if (err < 0) {
        mac_addr[0] = 0;
        mac_addr[1] = 0;
        return (err);
    }

    mac_addr[0] = bswap32(mac_addr[0]);
    mac_addr[1] = bswap32(mac_addr[1]);

    memcpy(mac, (uint8_t*)mac_addr, ETH_MAC_LEN);

    printf("fw1x> eFUSE MAC addr -> %02x-%02x-%02x-%02x-%02x-%02x",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return (0);
}

int
fw1x_get_stats(aq_hw_t *hw, aq_hw_stats_t *stats)
{
    int err = 0;

    err = aq_hw_fw_downld_dwords(hw, hw->mbox_addr, (uint32_t*)(void*)&hw->mbox,
        sizeof hw->mbox / sizeof(uint32_t));

    if (err >= 0) {
        if (stats != &hw->mbox.stats)
            memcpy(stats, &hw->mbox.stats, sizeof *stats);

        stats->dpc = reg_rx_dma_stat_counter7get(hw);
    }

    return (err);
}

aq_fw_ops_t aq_fw1x_ops =
{
	.reset = fw1x_reset,
	.set_mode = fw1x_set_mode,
	.get_mode = fw1x_get_mode,
	.get_mac_addr = fw1x_get_mac_addr,
	.get_stats = fw1x_get_stats,
};

