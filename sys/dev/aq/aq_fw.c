/*
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
 * @file aq_fw.c
 * Firmware-related functions implementation.
 * @date 2017.12.07  @author roman.agafonov@aquantia.com
 */

#include "aq.h"
#include "aq_hw_llh.h"
#include "aq_hw_llh_internal.h"

typedef enum {
	BOOT_MODE_UNKNOWN = 0,
	BOOT_MODE_FLB,
	BOOT_MODE_RBL_FLASH,
	BOOT_MODE_RBL_HOST_BOOTLOAD,
} aq_fw_bootloader_mode_t;

#define RBL_TIMEOUT_MS              10000
#define MAC_FW_START_TIMEOUT_MS     10000
#define FW_LOADER_START_TIMEOUT_MS  10000

const uint32_t NO_RESET_SCRATCHPAD_ADDRESS = 0;
const uint32_t NO_RESET_SCRATCHPAD_LEN_RES = 1;
const uint32_t NO_RESET_SCRATCHPAD_RBL_STATUS = 2;
const uint32_t NO_RESET_SCRATCHPAD_RBL_STATUS_2 = 3;
const uint32_t WRITE_DATA_COMPLETE = 0x55555555;
const uint32_t WRITE_DATA_CHUNK_DONE = 0xaaaaaaaa;
const uint32_t WRITE_DATA_FAIL_WRONG_ADDRESS = 0x66666666;

const uint32_t WAIT_WRITE_TIMEOUT = 1;
const uint32_t WAIT_WRITE_TIMEOUT_COUNT = 1000;

const uint32_t RBL_STATUS_SUCCESS = 0xabba;
const uint32_t RBL_STATUS_FAILURE = 0xbad;
const uint32_t RBL_STATUS_HOST_BOOT = 0xf1a7;

const uint32_t SCRATCHPAD_FW_LOADER_STATUS = (0x40 / sizeof(uint32_t));

extern aq_fw_ops_t aq_fw1x_ops;
extern aq_fw_ops_t aq_fw2x_ops;

static int mac_soft_reset(aq_hw_t *, aq_fw_bootloader_mode_t *);
static int mac_soft_reset_flb(aq_hw_t *);
static int mac_soft_reset_rbl(aq_hw_t *, aq_fw_bootloader_mode_t *);
static int wait_init_mac_firmware(aq_hw_t *);

int
aq_fw_reset(aq_hw_t *hw)
{
	aq_fw_bootloader_mode_t mode = BOOT_MODE_UNKNOWN;
	int		ver = AQ_READ_REG(hw, 0x18);
	uint32_t	bootExitCode = 0;
	int		k;

	for (k = 0; k < 1000; ++k) {
		uint32_t flbStatus = reg_glb_daisy_chain_status1_get(hw);
		bootExitCode = AQ_READ_REG(hw, 0x388);
		if (flbStatus != 0x06000000 || bootExitCode != 0)
			break;
	}

	if (k == 1000) {
		device_printf(hw->dev, "neither rbl nor flb started\n");
		return (1);
	}

	hw->rbl_enabled = bootExitCode != 0;

	/*
	 * FW version 0 indicates that cold start is in progress.
	 * This means the following:
	 * - driver has to wait for fw/hw to finish boot (500ms giveup)
	 * - driver may skip reset sequence and save time
	 */
	if (hw->fast_start_enabled && ver == 0) {
		/* Skip reset as it just completed */
		if (wait_init_mac_firmware(hw) == 0)
			return (0);
	}

	if (mac_soft_reset(hw, &mode) != 0) {
		device_printf(hw->dev, "mac reset failed\n");
		return (1);
	}

	switch (mode) {
	case BOOT_MODE_FLB:
		hw->flash_present = true;
		return (wait_init_mac_firmware(hw));
	case BOOT_MODE_RBL_FLASH:
		hw->flash_present = true;
		return (wait_init_mac_firmware(hw));
	case BOOT_MODE_RBL_HOST_BOOTLOAD:
		/* TODO Host Boot */
		device_printf(hw->dev, "rbl: hostboot not implemented\n");
		return (1);
	case BOOT_MODE_UNKNOWN:
		device_printf(hw->dev,
		    "fw bootload error: unknown bootloader type\n");
		return (1);
	}
}

int
aq_fw_ops_init(aq_hw_t *hw)
{
	if (hw->fw_version.raw == 0)
		hw->fw_version.raw = AQ_READ_REG(hw, 0x18);

	device_printf(hw->dev, "MAC FW version %d.%d.%d, ",
	    hw->fw_version.major_version, hw->fw_version.minor_version,
	    hw->fw_version.build_number);

	if (hw->fw_version.major_version == 1) {
		printf("using FW ops v1\n");
		hw->fw_ops = &aq_fw1x_ops;
		return (0);
	} else if (hw->fw_version.major_version >= 2) {
		printf("using FW ops v2\n");
		hw->fw_ops = &aq_fw2x_ops;
		return (0);
	}

	printf("invalid\n");
	return (1);
}

static int
mac_soft_reset(aq_hw_t *hw, aq_fw_bootloader_mode_t *mode)
{
	if (hw->rbl_enabled)
		return (mac_soft_reset_rbl(hw, mode));
	if (mode != NULL)
		*mode = BOOT_MODE_FLB;
	return (mac_soft_reset_flb(hw));
}

static int
mac_soft_reset_flb(aq_hw_t *hw)
{
    	bool restart_completed = false;
	int k;

	reg_global_ctl2_set(hw, 0x40e1);
	/*
	 * Let Felicity hardware to complete SMBUS transaction before Global
	 * software reset.
	 */
	msec_delay(50);

	/*
	 * If SPI burst transaction was interrupted (before running the script),
	 * global software reset may not clear SPI interface.  Clean it up
	 * manually before global reset.
	 */
	reg_glb_nvr_provisioning2_set(hw, 0xa0);
	reg_glb_nvr_interface1_set(hw, 0x9f);
	reg_glb_nvr_interface1_set(hw, 0x809f);
	msec_delay(50);

	reg_glb_standard_ctl1_set(hw,
	    (reg_glb_standard_ctl1_get(hw) & ~glb_reg_res_dis_msk) |
	    glb_soft_res_msk);

	/* Kickstart */
	reg_global_ctl2_set(hw, 0x80e0);
	reg_mif_power_gating_enable_control_set(hw, 0);
	if (!hw->fast_start_enabled)
		reg_glb_general_provisioning9_set(hw, 1);

	/*
	 * For the case SPI burst transaction was interrupted (by MCP reset
	 * above), wait until it is completed by hardware.
	 */
	msec_delay(50); /* sleep for 10ms */

	/* MAC Kickstart */
	if (!hw->fast_start_enabled) {
		uint32_t flb_status = 0;
		int k;

		reg_global_ctl2_set(hw, 0x180e0);

		for (k = 0; k < 1000; ++k) {
			flb_status = reg_glb_daisy_chain_status1_get(hw) & 0x10;
			if (flb_status != 0)
				break;
			msec_delay(10); /* sleep for 10 ms */
		}

		if (flb_status == 0) {
			device_printf(hw->dev,
			    "flb: mac kickstart timed out\n");
			return (1);
		}

		/* FW reset */
		reg_global_ctl2_set(hw, 0x80e0);
		/*
		 * Let Felicity hardware complete SMBUS transaction before
		 * Global software reset.
		 */
		msec_delay(50);
	}
	reg_glb_cpu_sem_set(hw, 1, 0);

	/* TODO PHY kickstart */

	/* Global software reset */
	rx_rx_reg_res_dis_set(hw, 0);
	tx_tx_reg_res_dis_set(hw, 0);
	mpi_tx_reg_res_dis_set(hw, 0);
	reg_glb_standard_ctl1_set(hw,
	    (reg_glb_standard_ctl1_get(hw) & ~glb_reg_res_dis_msk) |
	    glb_soft_res_msk);

	for (k = 0; k < 1000; ++k) {
		restart_completed = reg_glb_fw_image_id1_get(hw) != 0;
		if (restart_completed)
			break;
		msec_delay(10);
	}

	if (!restart_completed) {
		device_printf(hw->dev, "flb: global soft reset failed\n");
		return (1);
	}

	return (0);
}

static int
mac_soft_reset_rbl(aq_hw_t *hw, aq_fw_bootloader_mode_t *mode)
{
	uint16_t rbl_status = 0;

	reg_global_ctl2_set(hw, 0x40e1);
	reg_glb_cpu_sem_set(hw, 1, 0);
	reg_mif_power_gating_enable_control_set(hw, 0);

	/* TODO MAC FW will reload PHY FW if 1E.1000.3 was cleaned */

	reg_glb_cpu_no_reset_scratchpad_set(hw, 0xDEAD,
	    NO_RESET_SCRATCHPAD_RBL_STATUS);

	/* Global software reset */
	rx_rx_reg_res_dis_set(hw, 0);
	tx_tx_reg_res_dis_set(hw, 0);
	mpi_tx_reg_res_dis_set(hw, 0);
	reg_glb_standard_ctl1_set(hw,
	    (reg_glb_standard_ctl1_get(hw) & ~glb_reg_res_dis_msk) |
	    glb_soft_res_msk);

	reg_global_ctl2_set(hw, 0x40e0);

	/* Wait for RBL to finish boot process */
	for (int k = 0; k < RBL_TIMEOUT_MS; ++k) {
		rbl_status = LOWORD(reg_glb_cpu_no_reset_scratchpad_get(hw,
		    NO_RESET_SCRATCHPAD_RBL_STATUS));
		if (rbl_status != 0 && rbl_status != 0xdead)
			break;
		msec_delay(1);
	}

	if (rbl_status == 0 || rbl_status == 0xdead) {
		device_printf(hw->dev, "rbl: restart timed out\n");
		return (1);
	}

	if (rbl_status == RBL_STATUS_SUCCESS) {
		if (mode != NULL)
			*mode = BOOT_MODE_RBL_FLASH;
	} else if (rbl_status == RBL_STATUS_HOST_BOOT) {
		if (mode != NULL)
			*mode = BOOT_MODE_RBL_HOST_BOOTLOAD;
	} else {
		device_printf(hw->dev, "rbl: unknown status 0x%x", rbl_status);
		return (1);
	}

	return (0);
}

static int
wait_init_mac_firmware(aq_hw_t *hw)
{
	for (int i = 0; i < MAC_FW_START_TIMEOUT_MS; ++i) {
		if ((hw->fw_version.raw = AQ_READ_REG(hw, 0x18)) != 0)
			return (0);
		msec_delay(1);
	}

	device_printf(hw->dev,
	    "timed out waiting for reg 0x18; mac fw not ready\n");
	return (1);
}
