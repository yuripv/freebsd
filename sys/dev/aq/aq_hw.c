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
 *   (3)The name of the author may not be used to endorse or promote
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

#define AQ_CFG_FW_MIN_VER_EXPECTED 0x01050006U

int
aq_hw_err_from_flags(struct aq_hw *hw)
{
#if 0
	if (hw->flags & AQ_HW_FLAG_ERR_HW)
		return (EIO);
#endif

	return (0);
}

static void
aq_hw_chip_features_init(struct aq_hw *hw, uint32_t *p)
{
	uint32_t feat = 0;

	feat = AQ_HW_CHIP_MIPS | AQ_HW_CHIP_MPI_AQ;
	switch (reg_glb_mif_id_get(hw) & 0xf) {
	case 0x1:
		feat |= AQ_HW_CHIP_REVISION_A0;
		break;
	case 0x2:
		feat |= AQ_HW_CHIP_REVISION_B0 |
		    AQ_HW_CHIP_TPO2 |
		    AQ_HW_CHIP_RPF2;
		break;
	case 0xa:
		feat |= AQ_HW_CHIP_REVISION_B1 |
		    AQ_HW_CHIP_TPO2 |
		    AQ_HW_CHIP_RPF2;
	}

	*p = feat;
}

int
aq_hw_fw_downld_dwords(struct aq_hw *hw, uint32_t a, uint32_t *p, uint32_t cnt)
{
    int err = 0;

    AQ_HW_WAIT_FOR(reg_glb_cpu_sem_get(hw,
                       AQ_HW_FW_SM_RAM) == 1U,
                       1U, 10000U);

    if (err != 0) {
        bool is_locked;

        reg_glb_cpu_sem_set(hw, 1U, AQ_HW_FW_SM_RAM);
        is_locked = reg_glb_cpu_sem_get(hw, AQ_HW_FW_SM_RAM);
        if (!is_locked) {
            err = 1;
            goto err_exit;
        }
    }

    mif_mcp_up_mailbox_addr_set(hw, a);

    for (++cnt; --cnt && err == 0;) {
        mif_mcp_up_mailbox_execute_operation_set(hw, 1);

        if (IS_CHIP_FEATURE(hw, REVISION_B1))
            AQ_HW_WAIT_FOR(a != mif_mcp_up_mailbox_addr_get(hw), 1U, 1000U);
        else
            AQ_HW_WAIT_FOR(!mif_mcp_up_mailbox_busy_get(hw), 1, 1000U);

        *(p++) = mif_mcp_up_mailbox_data_get(hw);
    }

    reg_glb_cpu_sem_set(hw, 1U, AQ_HW_FW_SM_RAM);

err_exit:
    return (err);
}

static int
aq_hw_init_ucp(aq_hw_t *hw)
{
	int err;

	hw->fw_version.raw = 0;

	err = aq_fw_reset(hw);
	if (err != 0) {
		//device_printf(hw->dev, "failed to reset fw: %d", err);
		return (err);
	}

	aq_hw_chip_features_init(hw, &hw->chip_features);
	err = aq_fw_ops_init(hw);
	if (err < 0) {
		//aq_log_error("failed to initialize fw ops: %d", err);
		return (-1);
	}

	if (hw->fw_version.major_version == 1) {
		if (!AQ_READ_REG(hw, 0x370)) {
			unsigned int rnd;
			unsigned int ucp_0x370;

			rnd = arc4random();
			ucp_0x370 = 0x02020202 | (0xFEFEFEFE & rnd);
			AQ_WRITE_REG(hw, AQ_HW_UCP_0X370_REG, ucp_0x370);
		}
		reg_glb_cpu_scratch_scp_set(hw, 0, 25);
	}

	/* Check 10 times by 1ms */
	AQ_HW_WAIT_FOR((hw->mbox_addr = AQ_READ_REG(hw, 0x360)) != 0, 400, 20);

	if (AQ_CFG_FW_MIN_VER_EXPECTED < hw->fw_version.raw) {
		//aq_log_error("wrong fw version: expected %x actual %x",
		//    AQ_CFG_FW_MIN_VER_EXPECTED, hw->fw_version.raw);
	}

	return (err);
}

int
aq_hw_mpi_create(struct aq_hw *hw)
{
	return (aq_hw_init_ucp(hw));
}

int
aq_hw_mpi_read_stats(aq_hw_t *hw, aq_hw_fw_mbox_t *pmbox)
{
    int err = 0;

    if (hw->fw_ops && hw->fw_ops->get_stats) {
        err = hw->fw_ops->get_stats(hw, &pmbox->stats);
    } else {
        err = -ENOTSUP;
        //aq_log_error("get_stats() not supported by F/W");
    }

    if (err == 0) {
        pmbox->stats.dpc = reg_rx_dma_stat_counter7get(hw);
        pmbox->stats.cprc = stats_rx_lro_coalesced_pkt_count0_get(hw);
    }

    return (err);
}

static int
aq_hw_mpi_set(aq_hw_t *hw, aq_hw_fw_mpi_state_t state, uint32_t speed)
{
    int err = -ENOTSUP;

    if (hw->fw_ops && hw->fw_ops->set_mode) {
        err = hw->fw_ops->set_mode(hw, state, speed);
    } else {
        //aq_log_error("set_mode() not supported by F/W");
    }

    return (err);
}

int
aq_hw_set_link_speed(aq_hw_t *hw, uint32_t speed)
{
    return (aq_hw_mpi_set(hw, MPI_INIT, speed));
}

int
aq_hw_get_link_state(aq_hw_t *hw, uint32_t *link_speed, aq_hw_fc_info_t *fc_neg)
{
	aq_hw_fw_mpi_state_t mode;
	aq_fw_link_speed_t speed = AQ_FW_NONE;
	aq_fw_link_fc_t fc;

	if (hw->fw_ops == NULL || hw->fw_ops->get_mode == NULL ||
	    hw->fw_ops->get_mode(hw, &mode, &speed, &fc) != 0)
		return (1);

	*link_speed = 0;
	if (mode != MPI_INIT)
		return (0);

	switch (speed) {
	case AQ_FW_10G:
		*link_speed = 10000;
		break;
	case AQ_FW_5G:
		*link_speed = 5000;
		break;
	case AQ_FW_2G5:
		*link_speed = 2500;
		break;
	case AQ_FW_1G:
		*link_speed = 1000;
		break;
	case AQ_FW_100M:
		*link_speed = 100;
		break;
	default:
		*link_speed = 0;
		break;
	}

	fc_neg->fc_rx = !!(fc & aq_fw_fc_ENABLE_RX);
	fc_neg->fc_tx = !!(fc & aq_fw_fc_ENABLE_TX);

	return (0);
}

void
aq_hw_get_mac_permanent(struct aq_hw *hw, uint8_t *mac)
{
	if (hw->fw_ops != NULL && hw->fw_ops->get_mac_addr != NULL)
		hw->fw_ops->get_mac_addr(hw, mac);

	if ((mac[0] & 1) != 0 || (mac[0] | mac[1] | mac[2]) == 0) {
		/* Failed to get mac address from hw, generate one */
		uint16_t rnd;
		uint32_t h = 0;
		uint32_t l = 0;

		rnd = arc4random();
		/* chip revision */
		l = 0xe3000000 | (0xffff & rnd) | (0x00 << 16);
		h = 0x8001300e;

		mac[5] = (uint8_t)(0xff & l);
		l >>= 8;
		mac[4] = (uint8_t)(0xff & l);
		l >>= 8;
		mac[3] = (uint8_t)(0xff & l);
		l >>= 8;
		mac[2] = (uint8_t)(0xff & l);
		mac[1] = (uint8_t)(0xff & h);
		h >>= 8;
		mac[0] = (uint8_t)(0xff & h);
	}
}

void
aq_hw_deinit(aq_hw_t *hw)
{
	aq_hw_mpi_set(hw, MPI_DEINIT, 0);
}

int
aq_hw_set_power(aq_hw_t *hw, unsigned int power_state)
{
	aq_hw_mpi_set(hw, MPI_POWER, 0);

	return (0);
}


/* HW NIC functions */

int aq_hw_reset(struct aq_hw *hw)
{
    int err = 0;

    err = aq_fw_reset(hw);
    if (err < 0)
        goto err_exit;

    itr_irq_reg_res_dis_set(hw, 0);
    itr_res_irq_set(hw, 1);

    /* check 10 times by 1ms */
    AQ_HW_WAIT_FOR(itr_res_irq_get(hw) == 0, 1000, 10);
    if (err < 0) {
        printf("atlantic: IRQ reset failed: %d", err);
        goto err_exit;
    }

    if (hw->fw_ops && hw->fw_ops->reset)
        hw->fw_ops->reset(hw);

    err = aq_hw_err_from_flags(hw);

err_exit:
    return (err);
}

static int aq_hw_qos_set(struct aq_hw *hw)
{
    uint32_t tc = 0U;
    uint32_t buff_size = 0U;
    unsigned int i_priority = 0U;

    /* TPS Descriptor rate init */
    tps_tx_pkt_shed_desc_rate_curr_time_res_set(hw, 0x0U);
    tps_tx_pkt_shed_desc_rate_lim_set(hw, 0xA);

    /* TPS VM init */
    tps_tx_pkt_shed_desc_vm_arb_mode_set(hw, 0U);

    /* TPS TC credits init */
    tps_tx_pkt_shed_desc_tc_arb_mode_set(hw, 0U);
    tps_tx_pkt_shed_data_arb_mode_set(hw, 0U);

    tps_tx_pkt_shed_tc_data_max_credit_set(hw, 0xFFF, 0U);
    tps_tx_pkt_shed_tc_data_weight_set(hw, 0x64, 0U);
    tps_tx_pkt_shed_desc_tc_max_credit_set(hw, 0x50, 0U);
    tps_tx_pkt_shed_desc_tc_weight_set(hw, 0x1E, 0U);

    /* Tx buf size */
    buff_size = AQ_HW_TXBUF_MAX;

    tpb_tx_pkt_buff_size_per_tc_set(hw, buff_size, tc);
    tpb_tx_buff_hi_threshold_per_tc_set(hw,
                        (buff_size * (1024 / 32U) * 66U) /
                        100U, tc);
    tpb_tx_buff_lo_threshold_per_tc_set(hw,
                        (buff_size * (1024 / 32U) * 50U) /
                        100U, tc);

    /* QoS Rx buf size per TC */
    tc = 0;
    buff_size = AQ_HW_RXBUF_MAX;

    rpb_rx_pkt_buff_size_per_tc_set(hw, buff_size, tc);
    rpb_rx_buff_hi_threshold_per_tc_set(hw,
                        (buff_size *
                        (1024U / 32U) * 66U) /
                        100U, tc);
    rpb_rx_buff_lo_threshold_per_tc_set(hw,
                        (buff_size *
                        (1024U / 32U) * 50U) /
                        100U, tc);

	/* QoS 802.1p priority -> TC mapping */
	for (i_priority = 8U; i_priority--;)
		rpf_rpb_user_priority_tc_map_set(hw, i_priority, 0U);

	return (aq_hw_err_from_flags(hw));
}

static int aq_hw_offload_set(struct aq_hw *hw)
{
    int err = 0;

    /* TX checksums offloads*/
    tpo_ipv4header_crc_offload_en_set(hw, 1);
    tpo_tcp_udp_crc_offload_en_set(hw, 1);
    if (err < 0)
        goto err_exit;

    /* RX checksums offloads*/
    rpo_ipv4header_crc_offload_en_set(hw, 1);
    rpo_tcp_udp_crc_offload_en_set(hw, 1);
    if (err < 0)
        goto err_exit;

    /* LSO offloads*/
    tdm_large_send_offload_en_set(hw, 0xFFFFFFFFU);
    if (err < 0)
        goto err_exit;

/* LRO offloads */
    {
        uint32_t i = 0;
        uint32_t val = (8U < HW_ATL_B0_LRO_RXD_MAX) ? 0x3U :
            ((4U < HW_ATL_B0_LRO_RXD_MAX) ? 0x2U :
            ((2U < HW_ATL_B0_LRO_RXD_MAX) ? 0x1U : 0x0));

        for (i = 0; i < HW_ATL_B0_RINGS_MAX; i++)
            rpo_lro_max_num_of_descriptors_set(hw, val, i);

        rpo_lro_time_base_divider_set(hw, 0x61AU);
        rpo_lro_inactive_interval_set(hw, 0);
        /* the LRO timebase divider is 5 uS (0x61a),
         * to get a maximum coalescing interval of 250 uS,
         * we need to multiply by 50(0x32) to get
         * the default value 250 uS
         */
        rpo_lro_max_coalescing_interval_set(hw, 50);

        rpo_lro_qsessions_lim_set(hw, 1U);

        rpo_lro_total_desc_lim_set(hw, 2U);

        rpo_lro_patch_optimization_en_set(hw, 0U);

        rpo_lro_min_pay_of_first_pkt_set(hw, 10U);

        rpo_lro_pkt_lim_set(hw, 1U);

        rpo_lro_en_set(hw, (hw->lro_enabled ? 0xFFFFFFFFU : 0U));
    }


    err = aq_hw_err_from_flags(hw);

err_exit:
    return (err);
}

static int
aq_hw_init_tx_path(struct aq_hw *hw)
{
    int err = 0;

    /* Tx TC/RSS number config */
    tpb_tx_tc_mode_set(hw, 1U);

    thm_lso_tcp_flag_of_first_pkt_set(hw, 0x0FF6U);
    thm_lso_tcp_flag_of_middle_pkt_set(hw, 0x0FF6U);
    thm_lso_tcp_flag_of_last_pkt_set(hw, 0x0F7FU);

    /* Tx interrupts */
    tdm_tx_desc_wr_wb_irq_en_set(hw, 1U);

    /* misc */
    AQ_WRITE_REG(hw, 0x00007040U, 0x00010000U);//IS_CHIP_FEATURE(TPO2) ? 0x00010000U : 0x00000000U);
    tdm_tx_dca_en_set(hw, 0U);
    tdm_tx_dca_mode_set(hw, 0U);

    tpb_tx_path_scp_ins_en_set(hw, 1U);

    err = aq_hw_err_from_flags(hw);
    return (err);
}

static int
aq_hw_init_rx_path(struct aq_hw *hw)
{
    //struct aq_nic_cfg_s *cfg = hw->aq_nic_cfg;
    unsigned int control_reg_val = 0U;
    int i;
    int err;

    /* Rx TC/RSS number config */
    rpb_rpf_rx_traf_class_mode_set(hw, 1U);

    /* Rx flow control */
    rpb_rx_flow_ctl_mode_set(hw, 1U);

    /* RSS Ring selection */
    reg_rx_flr_rss_control1set(hw, 0xB3333333U);

    /* Multicast filters */
    for (i = AQ_HW_MAC_MAX; i--;) {
        rpfl2_uc_flr_en_set(hw, (i == 0U) ? 1U : 0U, i);
        rpfl2unicast_flr_act_set(hw, 1U, i);
    }

    reg_rx_flr_mcst_flr_msk_set(hw, 0x00000000U);
    reg_rx_flr_mcst_flr_set(hw, 0x00010FFFU, 0U);

    /* Vlan filters */
    rpf_vlan_outer_etht_set(hw, 0x88A8U);
    rpf_vlan_inner_etht_set(hw, 0x8100U);
	rpf_vlan_accept_untagged_packets_set(hw, true);
	rpf_vlan_untagged_act_set(hw, HW_ATL_RX_HOST);

    rpf_vlan_prom_mode_en_set(hw, 1);
 
    /* Rx Interrupts */
    rdm_rx_desc_wr_wb_irq_en_set(hw, 1U);

    /* misc */
    control_reg_val = 0x000F0000U; //RPF2

    /* RSS hash type set for IP/TCP */
    control_reg_val |= 0x1EU;

    AQ_WRITE_REG(hw, 0x00005040U, control_reg_val);

    rpfl2broadcast_en_set(hw, 1U);
    rpfl2broadcast_flr_act_set(hw, 1U);
    rpfl2broadcast_count_threshold_set(hw, 0xFFFFU & (~0U / 256U));

    rdm_rx_dca_en_set(hw, 0U);
    rdm_rx_dca_mode_set(hw, 0U);

    err = aq_hw_err_from_flags(hw);
    return (err);
}

int aq_hw_mac_addr_set(struct aq_hw *hw, uint8_t *mac_addr, uint8_t index)
{
    int err = 0;
    unsigned int h = 0U;
    unsigned int l = 0U;

    if (!mac_addr) {
        err = -EINVAL;
        goto err_exit;
    }
    h = (mac_addr[0] << 8) | (mac_addr[1]);
    l = (mac_addr[2] << 24) | (mac_addr[3] << 16) |
        (mac_addr[4] << 8) | mac_addr[5];

    rpfl2_uc_flr_en_set(hw, 0U, index);
    rpfl2unicast_dest_addresslsw_set(hw, l, index);
    rpfl2unicast_dest_addressmsw_set(hw, h, index);
    rpfl2_uc_flr_en_set(hw, 1U, index);

    err = aq_hw_err_from_flags(hw);

err_exit:
    return (err);
}

int
aq_hw_init(struct aq_hw *hw, uint8_t *mac_addr, uint8_t adm_irq, bool msix)
{
    int err = 0;
    uint32_t val = 0;

	/* Force limit MRRS on RDM/TDM to 2K */
	val = AQ_READ_REG(hw, AQ_HW_PCI_REG_CONTROL_6_ADR);
	AQ_WRITE_REG(hw, AQ_HW_PCI_REG_CONTROL_6_ADR, (val & ~0x707) | 0x404);

	/*
	 * TX DMA total request limit.  B0 hardware is not capable of
	 * handling more than (8K-MRRS) incoming DMA data.
	 * Value 24 in 256byte units.
	 */
	AQ_WRITE_REG(hw, AQ_HW_TX_DMA_TOTAL_REQ_LIMIT_ADR, 24);

	aq_hw_init_tx_path(hw);
	aq_hw_init_rx_path(hw);

	aq_hw_mac_addr_set(hw, mac_addr, AQ_HW_MAC);

	aq_hw_mpi_set(hw, MPI_INIT, hw->link_rate);

	aq_hw_qos_set(hw);

	err = aq_hw_err_from_flags(hw);
	if (err < 0)
		return (err);

	/* Disable clear-on-read for status */
	itr_irq_status_cor_en_set(hw, 0);
	/* Enable auto-mask clear */
	itr_irq_auto_mask_clr_en_set(hw, 1);
	if (msix) {
		/* MSIX + multi vector */
		itr_irq_mode_set(hw, 0x6);
	} else {
		/* MSI + multi vector */
		itr_irq_mode_set(hw, 0x5);
	}

	reg_gen_irq_map_set(hw, 0x80 | adm_irq, 3);
	aq_hw_offload_set(hw);

	return (0);
}

int
aq_hw_start(struct aq_hw *hw)
{
    	tpb_tx_buff_en_set(hw, 1);
	rpb_rx_buff_en_set(hw, 1);

	return (aq_hw_err_from_flags(hw));
}

int
aq_hw_interrupt_moderation_set(struct aq_hw *hw)
{
	static unsigned int tbl_rx[][2] = {
		{ 0x50, 0x78 },		/* 10Gbit */
		/* {0x6U, 0x38U}, 10Gbit */
		{ 0x0c, 0x70 },		/* 5Gbit */
		{ 0x0c, 0x70 },		/* 5Gbit 5GS */
		{ 0x18, 0xe0 },		/* 2.5Gbit */
		{ 0x30, 0x80 },		/* 1Gbit */
		{ 0x04, 0x50 },		/* 100Mbit */
	};
	static unsigned int tbl_tx[][2] = {
		{ 0x4f, 0x1ff },	/* 10Gbit */
		/* {0xffU, 0xffU}, * 10Gbit */
		{ 0x4f, 0xff },		/* 5Gbit */
		{ 0x4f, 0xff },		/* 5Gbit 5GS */
		{ 0x4f, 0xff },		/* 2.5Gbit */
		{ 0x4f, 0xff },		/* 1Gbit */
		{ 0x4f, 0xff },		/* 100Mbit */
	};

	uint32_t speed_index = 0;	/* itr settings for 10g */
	uint32_t itr_rx = 2;
	uint32_t itr_tx = 2;
	int custom_itr = hw->itr;
	int active = custom_itr != 0;

	if (custom_itr == -1) {
		itr_rx |= tbl_rx[speed_index][0] << 0x8; /* min timer value */
		itr_rx |= tbl_rx[speed_index][1] << 0x10; /* max timer value */

		itr_tx |= tbl_tx[speed_index][0] << 0x8; /* min timer value */
		itr_tx |= tbl_tx[speed_index][1] << 0x10; /* max timer value */
	} else {
		if (custom_itr > 0x1FF)
			custom_itr = 0x1FF;

		itr_rx |= (custom_itr / 2) << 0x8;	/* min timer value */
		itr_rx |= custom_itr << 0x10;		/* max timer value */

		itr_tx |= (custom_itr / 2) << 0x8;	/* min timer value */
		itr_tx |= custom_itr << 0x10;		/* max timer value */
	}

	tdm_tx_desc_wr_wb_irq_en_set(hw, !active);
	tdm_tdm_intr_moder_en_set(hw, active);
	rdm_rx_desc_wr_wb_irq_en_set(hw, !active);
	rdm_rdm_intr_moder_en_set(hw, active);

	for (int i = HW_ATL_B0_RINGS_MAX; i--;) {
		reg_tx_intr_moder_ctrl_set(hw,  itr_tx, i);
		reg_rx_intr_moder_ctrl_set(hw,  itr_rx, i);
	}

	return (aq_hw_err_from_flags(hw));
}

/**
 * @brief Set VLAN filter table
 * @details Configure VLAN filter table to accept (and assign the queue) traffic
 *  for the particular vlan ids.
 * Note: use this function under vlan promisc mode not to lost the traffic
 *
 * @param aq_hw_s
 * @param aq_rx_filter_vlan VLAN filter configuration
 * @return 0 - OK, <0 - error
 */
int
hw_atl_b0_hw_vlan_set(aq_hw_t *self, aq_rx_filter_vlan_t *aq_vlans)
{
	int i;

	for (i = 0; i < AQ_HW_VLAN_MAX_FILTERS; i++) {
		hw_atl_rpf_vlan_flr_en_set(self, 0, i);
		hw_atl_rpf_vlan_rxq_en_flr_set(self, 0, i);
		if (aq_vlans[i].enable) {
			hw_atl_rpf_vlan_id_flr_set(self, aq_vlans[i].vlan_id,
			    i);
			hw_atl_rpf_vlan_flr_act_set(self, 1, i);
			hw_atl_rpf_vlan_flr_en_set(self, 1, i);
			if (aq_vlans[i].queue != 0xff) {
				hw_atl_rpf_vlan_rxq_flr_set(self,
				    aq_vlans[i].queue, i);
				hw_atl_rpf_vlan_rxq_en_flr_set(self, 1, i);
			}
		}
	}

	return (aq_hw_err_from_flags(self));
}

int
hw_atl_b0_hw_vlan_promisc_set(aq_hw_t *self, bool promisc)
{
	hw_atl_rpf_vlan_prom_mode_en_set(self, promisc);
	return (aq_hw_err_from_flags(self));
}

void
aq_hw_set_promisc(aq_hw_t *self, bool l2_promisc, bool vlan_promisc,
    bool mc_promisc)
{
	rpfl2promiscuous_mode_en_set(self, l2_promisc);

	hw_atl_b0_hw_vlan_promisc_set(self, l2_promisc | vlan_promisc);

	rpfl2_accept_all_mc_packets_set(self, mc_promisc);
	rpfl2multicast_flr_en_set(self, mc_promisc, 0);
}

int
aq_hw_rss_hash_set(aq_hw_t *self, uint8_t rss_key[HW_ATL_RSS_HASHKEY_SIZE])
{
	uint32_t rss_key_dw[HW_ATL_RSS_HASHKEY_SIZE / 4];
	uint32_t addr = 0;
	uint32_t i = 0;
	int err = 0;

	memcpy(rss_key_dw, rss_key, HW_ATL_RSS_HASHKEY_SIZE);

	for (i = 10, addr = 0U; i--; ++addr) {
		uint32_t key_data = bswap32(rss_key_dw[i]);
		rpf_rss_key_wr_data_set(self, key_data);
		rpf_rss_key_addr_set(self, addr);
		rpf_rss_key_wr_en_set(self, 1U);
		AQ_HW_WAIT_FOR(rpf_rss_key_wr_en_get(self) == 0, 1000, 10);
		if (err < 0)
			goto err_exit;
	}

	err = aq_hw_err_from_flags(self);

err_exit:
	return (err);
}

int
aq_hw_rss_hash_get(aq_hw_t *self, uint8_t rss_key[HW_ATL_RSS_HASHKEY_SIZE])
{
	uint32_t rss_key_dw[HW_ATL_RSS_HASHKEY_SIZE / 4];
	uint32_t addr = 0U;
	uint32_t i = 0U;
	int err = 0;

	for (i = 10, addr = 0U; i--; ++addr) {
		rpf_rss_key_addr_set(self, addr);
		rss_key_dw[i] = bswap32(rpf_rss_key_rd_data_get(self));
	}
	memcpy(rss_key, rss_key_dw, HW_ATL_RSS_HASHKEY_SIZE);

	err = aq_hw_err_from_flags(self);

	return (err);
}

int
aq_hw_rss_set(aq_hw_t *self, uint8_t rss_table[HW_ATL_RSS_INDIRECTION_TABLE_MAX])
{
	uint16_t bitary[HW_ATL_RSS_INDIRECTION_TABLE_MAX * 3 / 16];
	int err = 0;
	uint32_t i = 0;

	memset(bitary, 0, sizeof(bitary));

	for (i = HW_ATL_RSS_INDIRECTION_TABLE_MAX; i--;) {
		(*(uint32_t *)(bitary + ((i * 3U) / 16U))) |=
			((rss_table[i]) << ((i * 3U) & 0xFU));
	}

	for (i = ARRAY_SIZE(bitary); i--;) {
		rpf_rss_redir_tbl_wr_data_set(self, bitary[i]);
		rpf_rss_redir_tbl_addr_set(self, i);
		rpf_rss_redir_wr_en_set(self, 1);
		AQ_HW_WAIT_FOR(rpf_rss_redir_wr_en_get(self) == 0,
			       1000, 10U);
		if (err < 0)
			goto err_exit;
	}

	err = aq_hw_err_from_flags(self);

err_exit:
	return (err);
}

int
aq_hw_udp_rss_enable(aq_hw_t *self, bool enable)
{
	if (!enable) {
		/*
		 * HW bug workaround:
		 * Disable RSS for UDP using rx flow filter 0.
		 * HW does not track RSS stream for fragmenged UDP,
		 * 0x5040 control reg does not work.
		 */
		hw_atl_rpf_l3_l4_enf_set(self, true, 0);
		hw_atl_rpf_l4_protf_en_set(self, true, 0);
		hw_atl_rpf_l3_l4_rxqf_en_set(self, true, 0);
		hw_atl_rpf_l3_l4_actf_set(self, L2_FILTER_ACTION_HOST, 0);
		hw_atl_rpf_l3_l4_rxqf_set(self, 0, 0);
		hw_atl_rpf_l4_protf_set(self, HW_ATL_RX_UDP, 0);
	} else {
		hw_atl_rpf_l3_l4_enf_set(self, false, 0);
	}

	return (aq_hw_err_from_flags(self));
}
