/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2018 aQuantia Corporation. All rights reserved
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

/* iflib txrx interface prototypes */
static int aq_isc_txd_encap(void *, if_pkt_info_t);
static void aq_isc_txd_flush(void *, uint16_t, qidx_t);
static int aq_isc_txd_credits_update(void *, uint16_t, bool);
static void aq_ring_rx_refill(void *, if_rxd_update_t);
static void aq_isc_rxd_flush(void *, uint16_t, uint8_t, qidx_t);
static int aq_isc_rxd_available(void *, uint16_t, qidx_t, qidx_t);
static int aq_isc_rxd_pkt_get(void *, if_rxd_info_t);

struct if_txrx aq_txrx = {
	.ift_txd_encap = aq_isc_txd_encap,
	.ift_txd_flush = aq_isc_txd_flush,
	.ift_txd_credits_update = aq_isc_txd_credits_update,
	.ift_rxd_available = aq_isc_rxd_available,
	.ift_rxd_pkt_get = aq_isc_rxd_pkt_get,
	.ift_rxd_refill = aq_ring_rx_refill,
	.ift_rxd_flush = aq_isc_rxd_flush,
	.ift_legacy_intr = NULL
};

static inline uint32_t
aq_next(uint32_t i, uint32_t lim)
{
	return ((i == lim) ? 0 : i + 1);
}

int
aq_ring_rx_init(aq_hw_t *hw, aq_ring_t *ring)
{
	uint32_t dma_desc_addr_lsw = (uint32_t)ring->rx_descs_phys & 0xffffffff;
	uint32_t dma_desc_addr_msw = (uint32_t)(ring->rx_descs_phys >> 32);

	rdm_rx_desc_en_set(hw, false, ring->index);
	rdm_rx_desc_head_splitting_set(hw, 0, ring->index);
	reg_rx_dma_desc_base_addresslswset(hw, dma_desc_addr_lsw, ring->index);
	reg_rx_dma_desc_base_addressmswset(hw, dma_desc_addr_msw, ring->index);
	rdm_rx_desc_len_set(hw, ring->rx_size / 8, ring->index);
	rdm_rx_desc_data_buff_size_set(hw, ring->rx_max_frame_size / 1024,
	    ring->index);
	rdm_rx_desc_head_buff_size_set(hw, 0, ring->index);
	rdm_rx_desc_head_splitting_set(hw, 0, ring->index);
	rpo_rx_desc_vlan_stripping_set(hw, 0, ring->index);

	/* Rx ring set mode */

	/* Mapping interrupt vector */
	itr_irq_map_rx_set(hw, ring->msix, ring->index);
	itr_irq_map_en_rx_set(hw, true, ring->index);

	rdm_cpu_id_set(hw, 0, ring->index);
	rdm_rx_desc_dca_en_set(hw, 0, ring->index);
	rdm_rx_head_dca_en_set(hw, 0, ring->index);
	rdm_rx_pld_dca_en_set(hw, 0, ring->index);

	return (aq_hw_err_from_flags(hw));
}

int
aq_ring_tx_init(aq_hw_t *hw, aq_ring_t *ring)
{
	uint32_t dma_desc_addr_lsw = (uint32_t)ring->tx_descs_phys & 0xffffffff;
	uint32_t dma_desc_addr_msw = (uint64_t)(ring->tx_descs_phys >> 32);

	tdm_tx_desc_en_set(hw, 0U, ring->index);
	reg_tx_dma_desc_base_addresslswset(hw, dma_desc_addr_lsw, ring->index);
	reg_tx_dma_desc_base_addressmswset(hw, dma_desc_addr_msw, ring->index);
	tdm_tx_desc_len_set(hw, ring->tx_size / 8U, ring->index);
	aq_ring_tx_tail_update(hw, ring, 0U);

	/* Set Tx threshold */
	tdm_tx_desc_wr_wb_threshold_set(hw, 0U, ring->index);

	/* Mapping interrupt vector */
	itr_irq_map_tx_set(hw, ring->msix, ring->index);
	itr_irq_map_en_tx_set(hw, true, ring->index);

	tdm_cpu_id_set(hw, 0, ring->index);
	tdm_tx_desc_dca_en_set(hw, 0U, ring->index);

	return (aq_hw_err_from_flags(hw));
}

int
aq_ring_tx_tail_update(aq_hw_t *hw, aq_ring_t *ring, uint32_t tail)
{
	reg_tx_dma_desc_tail_ptr_set(hw, tail, ring->index);
	return (0);
}

int
aq_ring_tx_start(aq_hw_t *hw, aq_ring_t *ring)
{
	tdm_tx_desc_en_set(hw, 1, ring->index);
	return (aq_hw_err_from_flags(hw));
}

int
aq_ring_rx_start(aq_hw_t *hw, aq_ring_t *ring)
{
	rdm_rx_desc_en_set(hw, 1, ring->index);
	return (aq_hw_err_from_flags(hw));
}

int
aq_ring_tx_stop(aq_hw_t *hw, aq_ring_t *ring)
{
	tdm_tx_desc_en_set(hw, 0, ring->index);
	return (aq_hw_err_from_flags(hw));
}

int
aq_ring_rx_stop(aq_hw_t *hw, aq_ring_t *ring)
{
	rdm_rx_desc_en_set(hw, 0, ring->index);
	/*
	 * Invalidate Descriptor Cache to prevent writing to the cached
	 * descriptors and to the data pointer of those descriptors
	 */
	rdm_rx_dma_desc_cache_init_tgl(hw);
	return (aq_hw_err_from_flags(hw));
}

static void
aq_ring_rx_refill(void *arg, if_rxd_update_t iru)
{
	aq_dev_t *aq_dev = arg;
	aq_rx_desc_t *rx_desc;
	aq_ring_t *ring;
	qidx_t i, pidx;

	ring = aq_dev->rx_rings[iru->iru_qsidx];
	pidx = iru->iru_pidx;

	for (i = 0; i < iru->iru_count; i++) {
		rx_desc = (aq_rx_desc_t *) &ring->rx_descs[pidx];
		rx_desc->read.buf_addr = htole64(iru->iru_paddrs[i]);
		rx_desc->read.hdr_addr = 0;

		pidx=aq_next(pidx, ring->rx_size - 1);
	}
}

static void
aq_isc_rxd_flush(void *arg, uint16_t rxqid, uint8_t flid __unused, qidx_t pidx)
{
	aq_dev_t *aq_dev = arg;
	aq_ring_t *ring = aq_dev->rx_rings[rxqid];

	reg_rx_dma_desc_tail_ptr_set(&aq_dev->hw, pidx, ring->index);
}

static int
aq_isc_rxd_available(void *arg, uint16_t rxqid, qidx_t idx, qidx_t budget)
{
	aq_dev_t *aq_dev = arg;
	aq_ring_t *ring = aq_dev->rx_rings[rxqid];
	aq_rx_desc_t *rx_desc = (aq_rx_desc_t *) ring->rx_descs;
	int cnt, i, iter;

	for (iter = 0, cnt = 0, i = idx;
	    iter < ring->rx_size && cnt <= budget;) {
		if (!rx_desc[i].wb.dd)
			break;
		if (rx_desc[i].wb.eop) {
			iter++;
			i = aq_next(i, ring->rx_size - 1);

			cnt++;
		} else {
			/* LRO/Jumbo: wait for whole packet be in the ring */
			if (rx_desc[i].wb.rsc_cnt) {
				i = rx_desc[i].wb.next_desp;
				iter++;
				continue;
			} else {
				iter++;
				i = aq_next(i, ring->rx_size - 1);
				continue;
			}
		}
	}

	return (cnt);
}

static void
aq_rx_set_cso_flags(aq_rx_desc_t *rx_desc,  if_rxd_info_t ri)
{
	if ((rx_desc->wb.pkt_type & 0x3) == 0) { /* IPv4 */
		if (rx_desc->wb.rx_cntl & BIT(0)){ /* IPv4 cksum checked */
			ri->iri_csum_flags |= CSUM_IP_CHECKED;
			if (!(rx_desc->wb.rx_stat & BIT(1)))
				ri->iri_csum_flags |= CSUM_IP_VALID;
		}
	}
	if (rx_desc->wb.rx_cntl & BIT(1)) { /* TCP/UDP cksum checked */
		ri->iri_csum_flags |= CSUM_L4_CALC;
		if (!(rx_desc->wb.rx_stat & BIT(2)) && /* L4 cksum error */
			(rx_desc->wb.rx_stat & BIT(3))) {  /* L4 cksum valid */
			ri->iri_csum_flags |= CSUM_L4_VALID;
			ri->iri_csum_data = htons(0xffff);
		}
	}
}

static uint8_t bsd_rss_type[16] = {
	[AQ_RX_RSS_TYPE_IPV4] = M_HASHTYPE_RSS_IPV4,
	[AQ_RX_RSS_TYPE_IPV6] = M_HASHTYPE_RSS_IPV6,
	[AQ_RX_RSS_TYPE_IPV4_TCP] = M_HASHTYPE_RSS_TCP_IPV4,
	[AQ_RX_RSS_TYPE_IPV6_TCP] = M_HASHTYPE_RSS_TCP_IPV6,
	[AQ_RX_RSS_TYPE_IPV4_UDP] = M_HASHTYPE_RSS_UDP_IPV4,
	[AQ_RX_RSS_TYPE_IPV6_UDP] = M_HASHTYPE_RSS_UDP_IPV6,
};

static int
aq_isc_rxd_pkt_get(void *arg, if_rxd_info_t ri)
{
	aq_dev_t	*aq_dev = arg;
	aq_ring_t	*ring = aq_dev->rx_rings[ri->iri_qsidx];
	aq_rx_desc_t	*rx_desc;
	if_t		ifp;
	int		cidx, i;
	size_t		len, total_len;

	cidx = ri->iri_cidx;
	ifp = iflib_get_ifp(aq_dev->ctx);
	i = 0;

	do {
		rx_desc = (aq_rx_desc_t *) &ring->rx_descs[cidx];

		if ((rx_desc->wb.rx_stat & BIT(0)) != 0) {
			ring->stats.rx_err++;
			return (1);
		}

		if (!rx_desc->wb.eop) {
			len = ring->rx_max_frame_size;
		} else {
			total_len = le32toh(rx_desc->wb.pkt_len);
			len = total_len & (ring->rx_max_frame_size - 1);
		}
		ri->iri_frags[i].irf_flid = 0;
		ri->iri_frags[i].irf_idx = cidx;
		ri->iri_frags[i].irf_len = len;

		if ((rx_desc->wb.pkt_type & 0x60) != 0) {
			ri->iri_flags |= M_VLANTAG;
			ri->iri_vtag = le32toh(rx_desc->wb.vlan);
		}

		i++;
		cidx = aq_next(cidx, ring->rx_size - 1);
	} while (!rx_desc->wb.eop);

	if ((ifp->if_capenable & IFCAP_RXCSUM) != 0) {
		aq_rx_set_cso_flags(rx_desc, ri);
	}
	ri->iri_rsstype = bsd_rss_type[rx_desc->wb.rss_type & 0xF];
	if (ri->iri_rsstype != M_HASHTYPE_NONE) {
		ri->iri_flowid = le32toh(rx_desc->wb.rss_hash);
	}

	ri->iri_len = total_len;
	ri->iri_nfrags = i;

	ring->stats.rx_bytes += total_len;
	ring->stats.rx_pkts++;

	return (0);
}

static void
aq_setup_offloads(aq_dev_t *aq_dev, if_pkt_info_t pi, aq_tx_desc_t *txd,
    uint32_t tx_cmd)
{
	txd->cmd |= TX_DESC_CMD_FCS;
	txd->cmd |= (pi->ipi_csum_flags & (CSUM_IP|CSUM_TSO)) ?
	    TX_DESC_CMD_IPV4 : 0;
	txd->cmd |= (pi->ipi_csum_flags &
	    (CSUM_IP_TCP | CSUM_IP6_TCP | CSUM_IP_UDP | CSUM_IP6_UDP)) ?
	    TX_DESC_CMD_L4CS : 0;
	txd->cmd |= (pi->ipi_flags & IPI_TX_INTR) ? TX_DESC_CMD_WB : 0;
	txd->cmd |= tx_cmd;
}

static int
aq_ring_tso_setup(aq_dev_t *aq_dev, if_pkt_info_t pi, uint32_t *hdrlen,
    aq_txc_desc_t *txc)
{
	uint32_t tx_cmd = 0;

	if (pi->ipi_csum_flags & CSUM_TSO) {
		tx_cmd |= TX_DESC_CMD_LSO | TX_DESC_CMD_L4CS;

		if (pi->ipi_ipproto != IPPROTO_TCP)
			return (0);

		txc->cmd = 0x4; /* TCP */

		if (pi->ipi_csum_flags & CSUM_IP6_TCP)
		    txc->cmd |= 0x2;

		txc->l2_len = pi->ipi_ehdrlen;
		txc->l3_len = pi->ipi_ip_hlen;
		txc->l4_len = pi->ipi_tcp_hlen;
		txc->mss_len = pi->ipi_tso_segsz;
		*hdrlen = txc->l2_len + txc->l3_len + txc->l4_len;
	}

	/* Set VLAN tag */
	if (pi->ipi_mflags & M_VLANTAG) {
		tx_cmd |= TX_DESC_CMD_VLAN;
		txc->vlan_tag = htole16(pi->ipi_vtag);
	}

	if (tx_cmd) {
		txc->type = TX_DESC_TYPE_CTX;
		txc->idx = 0;
	}

	return (tx_cmd);
}

static int
aq_isc_txd_encap(void *arg, if_pkt_info_t pi)
{
	aq_dev_t *aq_dev = arg;
	aq_ring_t *ring;
	aq_txc_desc_t *txc;
	aq_tx_desc_t *txd = NULL;
	bus_dma_segment_t *segs;
	qidx_t pidx;
	uint32_t hdrlen=0, pay_len;
	uint8_t tx_cmd = 0;
	int i, desc_count = 0;

	ring = aq_dev->tx_rings[pi->ipi_qsidx];

	segs = pi->ipi_segs;
	pidx = pi->ipi_pidx;
	txc = (aq_txc_desc_t *)&ring->tx_descs[pidx];

	pay_len = pi->ipi_len;

	txc->flags1 = 0;
	txc->flags2 = 0;

	tx_cmd = aq_ring_tso_setup(aq_dev, pi, &hdrlen, txc);

	if (tx_cmd) {
		/* We've consumed the first desc, adjust counters */
		pidx = aq_next(pidx, ring->tx_size - 1);

		txd = &ring->tx_descs[pidx];
		txd->flags = 0;
	} else {
		txd = (aq_tx_desc_t *)txc;
	}

	txd->ct_en = !!tx_cmd;

	txd->type = TX_DESC_TYPE_DESC;

	aq_setup_offloads(aq_dev, pi, txd, tx_cmd);

	if (tx_cmd) {
		txd->ct_idx = 0;
	}

	pay_len -= hdrlen;

	txd->pay_len = pay_len;

	for (i = 0; i < pi->ipi_nsegs; i++) {
		if (desc_count > 0) {
			txd = &ring->tx_descs[pidx];
			txd->flags = 0;
		}

		txd->buf_addr = htole64(segs[i].ds_addr);

		txd->type = TX_DESC_TYPE_DESC;
		txd->len = segs[i].ds_len;
		txd->pay_len = pay_len;

		pidx = aq_next(pidx, ring->tx_size - 1);

		desc_count++;
	}
	/* Last descriptor requires EOP and WB */
	txd->eop = 1U;

	ring->tx_tail = pidx;

	ring->stats.tx_pkts++;
	ring->stats.tx_bytes += pay_len;

	pi->ipi_new_pidx = pidx;

	return (0);
}

static void
aq_isc_txd_flush(void *arg, uint16_t txqid, qidx_t pidx)
{
	aq_dev_t *aq_dev = arg;
	aq_ring_t *ring = aq_dev->tx_rings[txqid];

	/* Update the write pointer - submits packet for transmission */
	aq_ring_tx_tail_update(&aq_dev->hw, ring, pidx);
}

static inline unsigned int
aq_avail_desc(int a, int b, int size)
{
    return (b >= a ? size - b + a : a - b);
}

static int
aq_isc_txd_credits_update(void *arg, uint16_t txqid, bool clear)
{
	aq_dev_t *aq_dev = arg;
	aq_ring_t *ring = aq_dev->tx_rings[txqid];
	uint32_t head = tdm_tx_desc_head_ptr_get(&aq_dev->hw, ring->index);
	int avail;

	if (ring->tx_head == head) {
		/* ring->tx_size */
		return (0);
	}

	avail = aq_avail_desc(head, ring->tx_head, ring->tx_size);
	if (clear)
		ring->tx_head = head;
	return (avail);
}
