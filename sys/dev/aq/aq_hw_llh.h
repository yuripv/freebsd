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

/*
 * Declarations of bitfield and register access functions for
 * Atlantic registers.
 */

#ifndef _HW_ATL_LLH_H_
#define	_HW_ATL_LLH_H_

void reg_glb_fw_image_id1_set(aq_hw_t *, uint32_t);
uint32_t reg_glb_fw_image_id1_get(aq_hw_t *);
/* Set global microprocessor semaphore */
void reg_glb_cpu_sem_set(aq_hw_t *, uint32_t, uint32_t);
/* Get global microprocessor semaphore */
uint32_t reg_glb_cpu_sem_get(aq_hw_t *, uint32_t);
/* Get Global Standard Control 1 */
uint32_t reg_glb_standard_ctl1_get(aq_hw_t *);
/* Set Global Standard Control 1 */
void reg_glb_standard_ctl1_set(aq_hw_t *, uint32_t);
/* Set Global Control 2 */
void reg_global_ctl2_set(aq_hw_t *, uint32_t);
/* Get Global Control 2 */
uint32_t reg_global_ctl2_get(aq_hw_t *);
/* Set Global Daisy Chain Status 1 */
void reg_glb_daisy_chain_status1_set(aq_hw_t *, uint32_t);
/* Get Global Daisy Chain Status 1 */
uint32_t reg_glb_daisy_chain_status1_get(aq_hw_t *);
/* Set Global General Provisioning 9 */
void reg_glb_general_provisioning9_set(aq_hw_t *, uint32_t);
/* Get Global General Provisioning 9 */
uint32_t reg_glb_general_provisioning9_get(aq_hw_t *);
/* Set Global NVR Provisioning 2 */
void reg_glb_nvr_provisioning2_set(aq_hw_t *, uint32_t);
/* Get Global NVR Provisioning 2 */
uint32_t reg_glb_nvr_provisioning2_get(aq_hw_t *);
/* Set Global NVR Interface 1 */
void reg_glb_nvr_interface1_set(aq_hw_t *, uint32_t);
/* Get Global NVR Interface 1 */
uint32_t reg_glb_nvr_interface1_get(aq_hw_t *);
/* Set global register reset disable */
void glb_glb_reg_res_dis_set(aq_hw_t *, uint32_t);
/* Set soft reset */
void glb_soft_res_set(aq_hw_t *, uint32_t);
/* Get soft reset */
uint32_t glb_soft_res_get(aq_hw_t *);
/* Stats */
uint32_t rpb_rx_dma_drop_pkt_cnt_get(aq_hw_t *);
/* Get rx dma good octet counter lsw */
uint32_t stats_rx_dma_good_octet_counterlsw_get(aq_hw_t *);
/* Get rx dma good packet counter lsw */
uint32_t stats_rx_dma_good_pkt_counterlsw_get(aq_hw_t *);
/* Get tx dma good octet counter lsw */
uint32_t stats_tx_dma_good_octet_counterlsw_get(aq_hw_t *);
/* Get tx dma good packet counter lsw */
uint32_t stats_tx_dma_good_pkt_counterlsw_get(aq_hw_t *);
/* Get rx dma good octet counter msw */
uint32_t stats_rx_dma_good_octet_countermsw_get(aq_hw_t *);
/* Get rx dma good packet counter msw */
uint32_t stats_rx_dma_good_pkt_countermsw_get(aq_hw_t *);
/* Get tx dma good octet counter msw */
uint32_t stats_tx_dma_good_octet_countermsw_get(aq_hw_t *);
/* Get tx dma good packet counter msw */
uint32_t stats_tx_dma_good_pkt_countermsw_get(aq_hw_t *);
/* Get rx lro coalesced packet count lsw */
uint32_t stats_rx_lro_coalesced_pkt_count0_get(aq_hw_t *);
/* Get msm rx errors counter register */
uint32_t reg_mac_msm_rx_errs_cnt_get(aq_hw_t *);
/* Get msm rx unicast frames counter register */
uint32_t reg_mac_msm_rx_ucst_frm_cnt_get(aq_hw_t *);
/* Get msm rx multicast frames counter register */
uint32_t reg_mac_msm_rx_mcst_frm_cnt_get(aq_hw_t *);
/* Get msm rx broadcast frames counter register */
uint32_t reg_mac_msm_rx_bcst_frm_cnt_get(aq_hw_t *);
/* Get msm rx broadcast octets counter register 1 */
uint32_t reg_mac_msm_rx_bcst_octets_counter1get(aq_hw_t *);
/* Get msm rx unicast octets counter register 0 */
uint32_t reg_mac_msm_rx_ucst_octets_counter0get(aq_hw_t *);
/* Get rx dma statistics counter 7 */
uint32_t reg_rx_dma_stat_counter7get(aq_hw_t *);
/* Get msm tx errors counter register */
uint32_t reg_mac_msm_tx_errs_cnt_get(aq_hw_t *);
/* Get msm tx unicast frames counter register */
uint32_t reg_mac_msm_tx_ucst_frm_cnt_get(aq_hw_t *);
/* Get msm tx multicast frames counter register */
uint32_t reg_mac_msm_tx_mcst_frm_cnt_get(aq_hw_t *);
/* Get msm tx broadcast frames counter register */
uint32_t reg_mac_msm_tx_bcst_frm_cnt_get(aq_hw_t *);
/* Get msm tx multicast octets counter register 1 */
uint32_t reg_mac_msm_tx_mcst_octets_counter1get(aq_hw_t *);
/* Get msm tx broadcast octets counter register 1 */
uint32_t reg_mac_msm_tx_bcst_octets_counter1get(aq_hw_t *);
/* Get msm tx unicast octets counter register 0 */
uint32_t reg_mac_msm_tx_ucst_octets_counter0get(aq_hw_t *);
/* Get global mif identification */
uint32_t reg_glb_mif_id_get(aq_hw_t *);
/* Set Tx Register Reset Disable */
void mpi_tx_reg_res_dis_set(aq_hw_t *, uint32_t);
/* Get Tx Register Reset Disable */
uint32_t mpi_tx_reg_res_dis_get(aq_hw_t *);
/* Set interrupt auto mask lsw */
void itr_irq_auto_masklsw_set(aq_hw_t *, uint32_t);
/* Set interrupt mapping enable rx */
void itr_irq_map_en_rx_set(aq_hw_t *, uint32_t, uint32_t);
/* Set interrupt mapping enable tx */
void itr_irq_map_en_tx_set(aq_hw_t *, uint32_t, uint32_t);
/* Set interrupt mapping rx */
void itr_irq_map_rx_set(aq_hw_t *, uint32_t, uint32_t);
/* Set interrupt mapping tx */
void itr_irq_map_tx_set(aq_hw_t *, uint32_t, uint32_t);
/* Set interrupt mask clear lsw */
void itr_irq_msk_clearlsw_set(aq_hw_t *, uint32_t);
/* Set interrupt mask set lsw */
void itr_irq_msk_setlsw_set(aq_hw_t *, uint32_t);
/* Set interrupt register reset disable */
void itr_irq_reg_res_dis_set(aq_hw_t *, uint32_t);
/* Set interrupt status clear lsw */
void itr_irq_status_clearlsw_set(aq_hw_t *, uint32_t);
/* Get interrupt status lsw */
uint32_t itr_irq_statuslsw_get(aq_hw_t *);
/* Get reset interrupt */
uint32_t itr_res_irq_get(aq_hw_t *);
/* Set reset interrupt */
void itr_res_irq_set(aq_hw_t *, uint32_t);
void itr_irq_mode_set(aq_hw_t *, uint32_t);
/* Set Link Interrupt Mapping Enable */
void itr_link_int_map_en_set(aq_hw_t *, uint32_t);
/* Get Link Interrupt Mapping Enable */
uint32_t itr_link_int_map_en_get(aq_hw_t *);
/* Set Link Interrupt Mapping */
void itr_link_int_map_set(aq_hw_t *, uint32_t);
/* Get Link Interrupt Mapping */
uint32_t itr_link_int_map_get(aq_hw_t *);
/* Set MIF Interrupt Mapping Enable */
void itr_mif_int_map_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Get MIF Interrupt Mapping Enable */
uint32_t itr_mif_int_map_en_get(aq_hw_t *, uint32_t);
/* Set MIF Interrupt Mapping */
void itr_mif_int_map_set(aq_hw_t *, uint32_t, uint32_t);
/* Get MIF Interrupt Mapping */
uint32_t itr_mif_int_map_get(aq_hw_t *, uint32_t);
void itr_irq_status_cor_en_set(aq_hw_t *, uint32_t);
void itr_irq_auto_mask_clr_en_set(aq_hw_t *, uint32_t);
/* Set cpu id */
void rdm_cpu_id_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx dca enable */
void rdm_rx_dca_en_set(aq_hw_t *, uint32_t);
/* Set rx dca mode */
void rdm_rx_dca_mode_set(aq_hw_t *, uint32_t);
/* Set rx descriptor data buffer size */
void rdm_rx_desc_data_buff_size_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx descriptor dca enable */
void rdm_rx_desc_dca_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx descriptor enable */
void rdm_rx_desc_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx descriptor header splitting */
void rdm_rx_desc_head_splitting_set(aq_hw_t *, uint32_t, uint32_t);
/* Get rx descriptor head pointer */
uint32_t rdm_rx_desc_head_ptr_get(aq_hw_t *, uint32_t);
/* Set rx descriptor length */
void rdm_rx_desc_len_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx descriptor write-back interrupt enable */
void rdm_rx_desc_wr_wb_irq_en_set(aq_hw_t *, uint32_t);
/* Set rx header dca enable */
void rdm_rx_head_dca_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx payload dca enable */
void rdm_rx_pld_dca_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx descriptor header buffer size */
void rdm_rx_desc_head_buff_size_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx descriptor reset */
void rdm_rx_desc_res_set(aq_hw_t *, uint32_t, uint32_t);
/* Set RDM Interrupt Moderation Enable */
void rdm_rdm_intr_moder_en_set(aq_hw_t *, uint32_t);
/* Set general interrupt mapping register */
void reg_gen_irq_map_set(aq_hw_t *, uint32_t, uint32_t);
/* Get general interrupt status register */
uint32_t reg_gen_irq_status_get(aq_hw_t *);
/* Set interrupt global control register */
void reg_irq_glb_ctl_set(aq_hw_t *, uint32_t);
/* Set interrupt throttle register */
void reg_irq_thr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx dma descriptor base address lsw */
void reg_rx_dma_desc_base_addresslswset(aq_hw_t *, uint32_t, uint32_t);
/* Set rx dma descriptor base address msw */
void reg_rx_dma_desc_base_addressmswset(aq_hw_t *, uint32_t, uint32_t);
/* Get rx dma descriptor status register */
uint32_t reg_rx_dma_desc_status_get(aq_hw_t *, uint32_t);
/* Set rx dma descriptor tail pointer register */
void reg_rx_dma_desc_tail_ptr_set(aq_hw_t *, uint32_t, uint32_t);
/* Get rx dma descriptor tail pointer register */
uint32_t reg_rx_dma_desc_tail_ptr_get(aq_hw_t *, uint32_t);
/* Set rx filter multicast filter mask register */
void reg_rx_flr_mcst_flr_msk_set(aq_hw_t *, uint32_t);
/* Set rx filter multicast filter register */
void reg_rx_flr_mcst_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx filter rss control register 1 */
void reg_rx_flr_rss_control1set(aq_hw_t *, uint32_t);
/* Set RX Filter Control Register 2 */
void reg_rx_flr_control2_set(aq_hw_t *, uint32_t);
/* Set RX Interrupt Moderation Control Register */
void reg_rx_intr_moder_ctrl_set(aq_hw_t *, uint32_t, uint32_t);
/* Set tx dma debug control */
void reg_tx_dma_debug_ctl_set(aq_hw_t *, uint32_t);
/* Set tx dma descriptor base address lsw */
void reg_tx_dma_desc_base_addresslswset(aq_hw_t *, uint32_t, uint32_t);
/* Set tx dma descriptor base address msw */
void reg_tx_dma_desc_base_addressmswset(aq_hw_t *, uint32_t, uint32_t);
/* Set tx dma descriptor tail pointer register */
void reg_tx_dma_desc_tail_ptr_set(aq_hw_t *, uint32_t, uint32_t);
/* Get tx dma descriptor tail pointer register */
uint32_t reg_tx_dma_desc_tail_ptr_get(aq_hw_t *, uint32_t);
/* Set TX Interrupt Moderation Control Register */
void reg_tx_intr_moder_ctrl_set(aq_hw_t *, uint32_t, uint32_t);
/* Get global microprocessor scratch pad */
uint32_t reg_glb_cpu_scratch_scp_get(aq_hw_t *, uint32_t);
/* Set global microprocessor scratch pad */
void reg_glb_cpu_scratch_scp_set(aq_hw_t *, uint32_t, uint32_t);
/* Get global microprocessor no reset scratch pad */
uint32_t reg_glb_cpu_no_reset_scratchpad_get(aq_hw_t *, uint32_t);
/* Set global microprocessor no reset scratch pad */
void reg_glb_cpu_no_reset_scratchpad_set(aq_hw_t *, uint32_t, uint32_t);
/* Set dma system loopback */
void rpb_dma_sys_lbk_set(aq_hw_t *, uint32_t);
/* Set rx traffic class mode */
void rpb_rpf_rx_traf_class_mode_set(aq_hw_t *, uint32_t);
/* Set rx buffer enable */
void rpb_rx_buff_en_set(aq_hw_t *, uint32_t);
/* Set rx buffer high threshold (per tc) */
void rpb_rx_buff_hi_threshold_per_tc_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx buffer low threshold (per tc) */
void rpb_rx_buff_lo_threshold_per_tc_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx flow control mode */
void rpb_rx_flow_ctl_mode_set(aq_hw_t *, uint32_t);
/* Set rx packet buffer size (per tc) */
void rpb_rx_pkt_buff_size_per_tc_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rx xoff enable (per tc) */
void rpb_rx_xoff_en_per_tc_set(aq_hw_t *, uint32_t, uint32_t);
/* Set l2 broadcast count threshold */
void rpfl2broadcast_count_threshold_set(aq_hw_t *, uint32_t);
/* Set l2 broadcast enable */
void rpfl2broadcast_en_set(aq_hw_t *, uint32_t);
/* Set l2 broadcast filter action */
void rpfl2broadcast_flr_act_set(aq_hw_t *, uint32_t);
/* Set l2 multicast filter enable */
void rpfl2multicast_flr_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set l2 promiscuous mode enable */
void rpfl2promiscuous_mode_en_set(aq_hw_t *, uint32_t);
/* Set l2 unicast filter action */
void rpfl2unicast_flr_act_set(aq_hw_t *, uint32_t, uint32_t);
/* Set l2 unicast filter enable */
void rpfl2_uc_flr_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set l2 unicast destination address lsw */
void rpfl2unicast_dest_addresslsw_set(aq_hw_t *, uint32_t, uint32_t);
/* Set l2 unicast destination address msw */
void rpfl2unicast_dest_addressmsw_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L2 Accept all Multicast packets */
void rpfl2_accept_all_mc_packets_set(aq_hw_t *, uint32_t);
/* Set user-priority tc mapping */
void rpf_rpb_user_priority_tc_map_set(aq_hw_t *, uint32_t, uint32_t);
/* Set rss key address */
void rpf_rss_key_addr_set(aq_hw_t *, uint32_t);
/* Set rss key write data */
void rpf_rss_key_wr_data_set(aq_hw_t *, uint32_t);
/* Get rss key read data */
uint32_t rpf_rss_key_rd_data_get(aq_hw_t *);
/* Get rss key write enable */
uint32_t rpf_rss_key_wr_en_get(aq_hw_t *);
/* Set rss key write enable */
void rpf_rss_key_wr_en_set(aq_hw_t *, uint32_t);
/* Set rss redirection table address */
void rpf_rss_redir_tbl_addr_set(aq_hw_t *, uint32_t);
/* Set rss redirection table write data */
void rpf_rss_redir_tbl_wr_data_set(aq_hw_t *, uint32_t);
/* Get rss redirection write enable */
uint32_t rpf_rss_redir_wr_en_get(aq_hw_t *);
/* Set rss redirection write enable */
void rpf_rss_redir_wr_en_set(aq_hw_t *, uint32_t);
/* Set tpo to rpf system loopback */
void rpf_tpo_to_rpf_sys_lbk_set(aq_hw_t *, uint32_t);
/* Set vlan inner ethertype */
void hw_atl_rpf_vlan_inner_etht_set(aq_hw_t *, uint32_t);
/* Set vlan outer ethertype */
void hw_atl_rpf_vlan_outer_etht_set(aq_hw_t *, uint32_t);
/* Set vlan promiscuous mode enable */
void hw_atl_rpf_vlan_prom_mode_en_set(aq_hw_t *, uint32_t);
/* Set VLAN untagged action */
void hw_atl_rpf_vlan_untagged_act_set(aq_hw_t *, uint32_t);
/* Set VLAN accept untagged packets */
void hw_atl_rpf_vlan_accept_untagged_packets_set(aq_hw_t *, uint32_t);
/* Set VLAN filter enable */
void hw_atl_rpf_vlan_flr_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set VLAN Filter Action */
void hw_atl_rpf_vlan_flr_act_set(aq_hw_t *, uint32_t, uint32_t);
/* Set VLAN ID Filter */
void hw_atl_rpf_vlan_id_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set VLAN RX queue assignment enable */
void hw_atl_rpf_vlan_rxq_en_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set VLAN RX queue */
void hw_atl_rpf_vlan_rxq_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype filter enable */
void hw_atl_rpf_etht_flr_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype user-priority enable */
void hw_atl_rpf_etht_user_priority_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype rx queue enable */
void hw_atl_rpf_etht_rx_queue_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype rx queue */
void hw_atl_rpf_etht_rx_queue_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype user-priority */
void hw_atl_rpf_etht_user_priority_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype management queue */
void hw_atl_rpf_etht_mgt_queue_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype filter action */
void hw_atl_rpf_etht_flr_act_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype filter */
void hw_atl_rpf_etht_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 filter enable */
void hw_atl_rpf_l3_l4_enf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 IPv6 enable */
void hw_atl_rpf_l3_v6_enf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 source address enable */
void hw_atl_rpf_l3_saf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 destination address enable */
void hw_atl_rpf_l3_daf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 source port enable */
void hw_atl_rpf_l4_spf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 destination port enable */
void hw_atl_rpf_l4_dpf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 protocol enable */
void hw_atl_rpf_l4_protf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 ARP filter enable */
void hw_atl_rpf_l3_arpf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 rx queue enable */
void hw_atl_rpf_l3_l4_rxqf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 management queue */
void hw_atl_rpf_l3_l4_mng_rxqf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 filter action */
void hw_atl_rpf_l3_l4_actf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 rx queue */
void hw_atl_rpf_l3_l4_rxqf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 protocol value */
void hw_atl_rpf_l4_protf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 source port */
void hw_atl_rpf_l4_spd_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 destination port */
void hw_atl_rpf_l4_dpd_set(aq_hw_t *, uint32_t, uint32_t);
/* Set vlan inner ethertype */
void rpf_vlan_inner_etht_set(aq_hw_t *, uint32_t);
/* Set vlan outer ethertype */
void rpf_vlan_outer_etht_set(aq_hw_t *, uint32_t);
/* Set vlan promiscuous mode enable */
void rpf_vlan_prom_mode_en_set(aq_hw_t *, uint32_t);
/* Set VLAN untagged action */
void rpf_vlan_untagged_act_set(aq_hw_t *, uint32_t);
/* Set VLAN accept untagged packets */
void rpf_vlan_accept_untagged_packets_set(aq_hw_t *, uint32_t);
/* Set VLAN filter enable */
void rpf_vlan_flr_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set VLAN Filter Action */
void rpf_vlan_flr_act_set(aq_hw_t *, uint32_t, uint32_t);
/* Set VLAN ID Filter */
void rpf_vlan_id_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype filter enable */
void rpf_etht_flr_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype user-priority enable */
void rpf_etht_user_priority_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype rx queue enable */
void rpf_etht_rx_queue_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype rx queue */
void rpf_etht_rx_queue_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype user-priority */
void rpf_etht_user_priority_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype management queue */
void rpf_etht_mgt_queue_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype filter action */
void rpf_etht_flr_act_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype filter */
void rpf_etht_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 filter enable */
void hw_atl_rpf_l3_l4_enf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 IPv6 enable */
void hw_atl_rpf_l3_v6_enf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 source address enable */
void hw_atl_rpf_l3_saf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 destination address enable */
void hw_atl_rpf_l3_daf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 source port enable */
void hw_atl_rpf_l4_spf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 destination port enable */
void hw_atl_rpf_l4_dpf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 protocol enable */
void hw_atl_rpf_l4_protf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 ARP filter enable */
void hw_atl_rpf_l3_arpf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 rx queue enable */
void hw_atl_rpf_l3_l4_rxqf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 management queue */
void hw_atl_rpf_l3_l4_mng_rxqf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 filter action */
void hw_atl_rpf_l3_l4_actf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 rx queue */
void hw_atl_rpf_l3_l4_rxqf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 protocol value */
void hw_atl_rpf_l4_protf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 source port */
void hw_atl_rpf_l4_spd_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 destination port */
void hw_atl_rpf_l4_dpd_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ipv4 header checksum offload enable */
void rpo_ipv4header_crc_offload_en_set(aq_hw_t *, uint32_t);
/* Set rx descriptor vlan stripping */
void rpo_rx_desc_vlan_stripping_set(aq_hw_t *, uint32_t, uint32_t);
/* Set tcp/udp checksum offload enable */
void rpo_tcp_udp_crc_offload_en_set(aq_hw_t *, uint32_t);
/* Set LRO Patch Optimization Enable */
void rpo_lro_patch_optimization_en_set(aq_hw_t *, uint32_t);
/* Set Large Receive Offload Enable */
void rpo_lro_en_set(aq_hw_t *, uint32_t);
/* Set LRO Q Sessions Limit */
void rpo_lro_qsessions_lim_set(aq_hw_t *, uint32_t);
/* Set LRO Total Descriptor Limit */
void rpo_lro_total_desc_lim_set(aq_hw_t *, uint32_t);
/* Set LRO Min Payload of First Packet */
void rpo_lro_min_pay_of_first_pkt_set(aq_hw_t *, uint32_t);
/* Set LRO Packet Limit */
void rpo_lro_pkt_lim_set(aq_hw_t *, uint32_t);
/* Set LRO Max Number of Descriptors */
void rpo_lro_max_num_of_descriptors_set(aq_hw_t *, uint32_t, uint32_t);
/* Set LRO Time Base Divider */
void rpo_lro_time_base_divider_set(aq_hw_t *, uint32_t);
/* Set LRO Inactive Interval */
void rpo_lro_inactive_interval_set(aq_hw_t *, uint32_t);
/* Set LRO Max Coalescing Interval */
void rpo_lro_max_coalescing_interval_set(aq_hw_t *, uint32_t);
/* Set rx register reset disable */
void rx_rx_reg_res_dis_set(aq_hw_t *, uint32_t);
/* Set cpu id */
void tdm_cpu_id_set(aq_hw_t *, uint32_t, uint32_t);
/* Set large send offload enable */
void tdm_large_send_offload_en_set(aq_hw_t *, uint32_t);
/* Set tx descriptor enable */
void tdm_tx_desc_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set tx dca enable */
void tdm_tx_dca_en_set(aq_hw_t *, uint32_t);
/* Set tx dca mode */
void tdm_tx_dca_mode_set(aq_hw_t *, uint32_t);
/* Set tx descriptor dca enable */
void tdm_tx_desc_dca_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Get tx descriptor head pointer */
uint32_t tdm_tx_desc_head_ptr_get(aq_hw_t *, uint32_t);
/* Set tx descriptor length */
void tdm_tx_desc_len_set(aq_hw_t *, uint32_t, uint32_t);
/* Set tx descriptor write-back interrupt enable */
void tdm_tx_desc_wr_wb_irq_en_set(aq_hw_t *, uint32_t);
/* Set tx descriptor write-back threshold */
void tdm_tx_desc_wr_wb_threshold_set(aq_hw_t *, uint32_t, uint32_t);
/* Set TDM Interrupt Moderation Enable */
void tdm_tdm_intr_moder_en_set(aq_hw_t *, uint32_t);
/* Set lso tcp flag of first packet */
void thm_lso_tcp_flag_of_first_pkt_set(aq_hw_t *, uint32_t);
/* Set lso tcp flag of last packet */
void thm_lso_tcp_flag_of_last_pkt_set(aq_hw_t *, uint32_t);
/* Set lso tcp flag of middle packet */
void thm_lso_tcp_flag_of_middle_pkt_set(aq_hw_t *, uint32_t);
/* Set tx buffer enable */
void tpb_tx_buff_en_set(aq_hw_t *, uint32_t);
/* Set tx tc mode */
void tpb_tx_tc_mode_set(aq_hw_t *, uint32_t);
/* Set tx buffer high threshold (per tc) */
void tpb_tx_buff_hi_threshold_per_tc_set(aq_hw_t *, uint32_t, uint32_t);
/* Set tx buffer low threshold (per tc) */
void tpb_tx_buff_lo_threshold_per_tc_set(aq_hw_t *, uint32_t, uint32_t);
/* Set tx dma system loopback enable */
void tpb_tx_dma_sys_lbk_en_set(aq_hw_t *, uint32_t);
/* Set tx packet buffer size (per tc) */
void tpb_tx_pkt_buff_size_per_tc_set(aq_hw_t *, uint32_t, uint32_t);
/* Toggle rdm rx dma descriptor cache init */
void rdm_rx_dma_desc_cache_init_tgl(aq_hw_t *);
/* Set tx path pad insert enable */
void tpb_tx_path_scp_ins_en_set(aq_hw_t *, uint32_t);
/* Set ipv4 header checksum offload enable */
void tpo_ipv4header_crc_offload_en_set(aq_hw_t *, uint32_t);
/* Set tcp/udp checksum offload enable */
void tpo_tcp_udp_crc_offload_en_set(aq_hw_t *, uint32_t);
/* Set tx pkt system loopback enable */
void tpo_tx_pkt_sys_lbk_en_set(aq_hw_t *, uint32_t);
/* Set tx packet scheduler data arbitration mode */
void tps_tx_pkt_shed_data_arb_mode_set(aq_hw_t *, uint32_t);
/* Set tx packet scheduler descriptor rate current time reset */
void tps_tx_pkt_shed_desc_rate_curr_time_res_set(aq_hw_t *, uint32_t);
/* Set tx packet scheduler descriptor rate limit */
void tps_tx_pkt_shed_desc_rate_lim_set(aq_hw_t *, uint32_t);
/* Set tx packet scheduler descriptor tc arbitration mode */
void tps_tx_pkt_shed_desc_tc_arb_mode_set(aq_hw_t *, uint32_t);
/* Set tx packet scheduler descriptor tc max credit */
void tps_tx_pkt_shed_desc_tc_max_credit_set(aq_hw_t *, uint32_t, uint32_t);
/* Set tx packet scheduler descriptor tc weight */
void tps_tx_pkt_shed_desc_tc_weight_set(aq_hw_t *, uint32_t, uint32_t);
/* Set tx packet scheduler descriptor vm arbitration mode */
void tps_tx_pkt_shed_desc_vm_arb_mode_set(aq_hw_t *, uint32_t);
/* Set tx packet scheduler tc data max credit */
void tps_tx_pkt_shed_tc_data_max_credit_set(aq_hw_t *, uint32_t, uint32_t);
/* Set tx packet scheduler tc data weight */
void tps_tx_pkt_shed_tc_data_weight_set(aq_hw_t *, uint32_t, uint32_t);
/* Set tx register reset disable */
void tx_tx_reg_res_dis_set(aq_hw_t *, uint32_t);
/* Get register access status */
uint32_t msm_reg_access_status_get(aq_hw_t *);
/* Set register address for indirect address */
void msm_reg_addr_for_indirect_addr_set(aq_hw_t *, uint32_t);
/* Set register read strobe */
void msm_reg_rd_strobe_set(aq_hw_t *, uint32_t);
/* Get register read data */
uint32_t msm_reg_rd_data_get(aq_hw_t *);
/* Set register write data */
void msm_reg_wr_data_set(aq_hw_t *, uint32_t);
/* Set register write strobe */
void msm_reg_wr_strobe_set(aq_hw_t *, uint32_t);
/* Set pci register reset disable */
void pci_pci_reg_res_dis_set(aq_hw_t *, uint32_t);
/* Set MIF Power Gating Enable Control */
void reg_mif_power_gating_enable_control_set(aq_hw_t *, uint32_t);
/* Get MIF Power Gating Enable Control */
uint32_t reg_mif_power_gating_enable_control_get(aq_hw_t *);
/* Get mif up mailbox busy */
uint32_t mif_mcp_up_mailbox_busy_get(aq_hw_t *);
/* Set mif up mailbox execute operation */
void mif_mcp_up_mailbox_execute_operation_set(aq_hw_t *, uint32_t);
/* Get mif uP mailbox address */
uint32_t mif_mcp_up_mailbox_addr_get(aq_hw_t *);
/* Set mif uP mailbox address */
void mif_mcp_up_mailbox_addr_set(aq_hw_t *, uint32_t);
/* Get mif uP mailbox data */
uint32_t mif_mcp_up_mailbox_data_get(aq_hw_t *);
/* Clear ipv4 filter destination address */
void hw_atl_rpfl3l4_ipv4_dest_addr_clear(aq_hw_t *, uint8_t);
/* Clear ipv4 filter source address */
void hw_atl_rpfl3l4_ipv4_src_addr_clear(aq_hw_t *, uint8_t);
/* Clear command for filter l3-l4 */
void hw_atl_rpfl3l4_cmd_clear(aq_hw_t *, uint8_t);
/* Clear ipv6 filter destination address */
void hw_atl_rpfl3l4_ipv6_dest_addr_clear(aq_hw_t *, uint8_t);
/* Clear ipv6 filter source address */
void hw_atl_rpfl3l4_ipv6_src_addr_clear(aq_hw_t *, uint8_t);
/* Set ipv4 filter destination address */
void hw_atl_rpfl3l4_ipv4_dest_addr_set(aq_hw_t *, uint8_t, uint32_t);
/* Set ipv4 filter source address */
void hw_atl_rpfl3l4_ipv4_src_addr_set(aq_hw_t *, uint8_t, uint32_t);
/* Set command for filter l3-l4 */
void hw_atl_rpfl3l4_cmd_set(aq_hw_t *, uint8_t, uint32_t);
/* Set ipv6 filter source address */
void hw_atl_rpfl3l4_ipv6_src_addr_set(aq_hw_t *, uint8_t, uint32_t *);
/* Set ipv6 filter destination address */
void hw_atl_rpfl3l4_ipv6_dest_addr_set(aq_hw_t *, uint8_t, uint32_t *);
/* Set vlan inner ethertype */
void hw_atl_rpf_vlan_inner_etht_set(aq_hw_t *, uint32_t);
/* Set vlan outer ethertype */
void hw_atl_rpf_vlan_outer_etht_set(aq_hw_t *, uint32_t);
/* Set vlan promiscuous mode enable */
void hw_atl_rpf_vlan_prom_mode_en_set(aq_hw_t *, uint32_t);
/* Set VLAN untagged action */
void hw_atl_rpf_vlan_untagged_act_set(aq_hw_t *, uint32_t);
/* Set VLAN accept untagged packets */
void hw_atl_rpf_vlan_accept_untagged_packets_set(aq_hw_t *, uint32_t);
/* Set VLAN filter enable */
void hw_atl_rpf_vlan_flr_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set VLAN Filter Action */
void hw_atl_rpf_vlan_flr_act_set(aq_hw_t *, uint32_t, uint32_t);
/* Set VLAN ID Filter */
void hw_atl_rpf_vlan_id_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set VLAN RX queue assignment enable */
void hw_atl_rpf_vlan_rxq_en_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set VLAN RX queue */
void hw_atl_rpf_vlan_rxq_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype filter enable */
void hw_atl_rpf_etht_flr_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype user-priority enable */
void hw_atl_rpf_etht_user_priority_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype rx queue enable */
void hw_atl_rpf_etht_rx_queue_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype rx queue */
void hw_atl_rpf_etht_rx_queue_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype user-priority */
void hw_atl_rpf_etht_user_priority_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype management queue */
void hw_atl_rpf_etht_mgt_queue_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype filter action */
void hw_atl_rpf_etht_flr_act_set(aq_hw_t *, uint32_t, uint32_t);
/* Set ethertype filter */
void hw_atl_rpf_etht_flr_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 filter enable */
void hw_atl_rpf_l3_l4_enf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 IPv6 enable */
void hw_atl_rpf_l3_v6_enf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 source address enable */
void hw_atl_rpf_l3_saf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 destination address enable */
void hw_atl_rpf_l3_daf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 source port enable */
void hw_atl_rpf_l4_spf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 destination port enable */
void hw_atl_rpf_l4_dpf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 protocol enable */
void hw_atl_rpf_l4_protf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3 ARP filter enable */
void hw_atl_rpf_l3_arpf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 rx queue enable */
void hw_atl_rpf_l3_l4_rxqf_en_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 management queue */
void hw_atl_rpf_l3_l4_mng_rxqf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 filter action */
void hw_atl_rpf_l3_l4_actf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L3/L4 rx queue */
void hw_atl_rpf_l3_l4_rxqf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 protocol value */
void hw_atl_rpf_l4_protf_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 source port */
void hw_atl_rpf_l4_spd_set(aq_hw_t *, uint32_t, uint32_t);
/* Set L4 destination port */
void hw_atl_rpf_l4_dpd_set(aq_hw_t *, uint32_t, uint32_t);

#endif /* !_HW_ATL_LLH_H_ */
