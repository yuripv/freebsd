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

int
aq_update_hw_stats(aq_dev_t *aq_dev)
{
	aq_hw_t		*hw = &aq_dev->hw;
	aq_hw_fw_mbox_t	mbox;

	aq_hw_mpi_read_stats(hw, &mbox);

#define	AQ_SDELTA(_N_) \
	(aq_dev->curr_stats._N_ += mbox.stats._N_ - aq_dev->last_stats._N_)
	if (aq_dev->linkup) {
		AQ_SDELTA(uprc);
		AQ_SDELTA(mprc);
		AQ_SDELTA(bprc);
		AQ_SDELTA(cprc);
		AQ_SDELTA(erpt);

		AQ_SDELTA(uptc);
		AQ_SDELTA(mptc);
		AQ_SDELTA(bptc);
		AQ_SDELTA(erpr);

		AQ_SDELTA(ubrc);
		AQ_SDELTA(ubtc);
		AQ_SDELTA(mbrc);
		AQ_SDELTA(mbtc);
		AQ_SDELTA(bbrc);
		AQ_SDELTA(bbtc);

		AQ_SDELTA(ptc);
		AQ_SDELTA(prc);

		AQ_SDELTA(dpc);

		aq_dev->curr_stats.brc =
		    aq_dev->curr_stats.ubrc +
		    aq_dev->curr_stats.mbrc +
		    aq_dev->curr_stats.bbrc;
		aq_dev->curr_stats.btc =
		    aq_dev->curr_stats.ubtc +
		    aq_dev->curr_stats.mbtc +
		    aq_dev->curr_stats.bbtc;
	}
#undef AQ_SDELTA

	memcpy(&aq_dev->last_stats, &mbox.stats, sizeof(mbox.stats));

	return (0);
}

void
aq_if_update_admin_status(if_ctx_t ctx)
{
	aq_dev_t *aq_dev = iflib_get_softc(ctx);
	aq_hw_t *hw = &aq_dev->hw;
	uint32_t link_speed;
	aq_hw_fc_info_t fc_neg;

	aq_hw_get_link_state(hw, &link_speed, &fc_neg);
	if (link_speed && !aq_dev->linkup) {
		aq_dev->linkup = 1;
		/* Disable TSO if link speed < 1G */
		if (link_speed < 1000 &&
		    (iflib_get_softc_ctx(ctx)->isc_capabilities &
		    (IFCAP_TSO4 | IFCAP_TSO6))) {
			iflib_get_softc_ctx(ctx)->isc_capabilities &=
			    ~(IFCAP_TSO4 | IFCAP_TSO6);
			device_printf(aq_dev->dev,
			    "TSO disabled for link speed < 1Gbit");
		} else {
			iflib_get_softc_ctx(ctx)->isc_capabilities |=
			    IFCAP_TSO4 | IFCAP_TSO6;
		}
		/* Turn on/off RX Pause in RPB */
		rpb_rx_xoff_en_per_tc_set(hw, fc_neg.fc_rx, 0);

		iflib_link_state_change(ctx, LINK_STATE_UP,
		    IF_Mbps(link_speed));
		aq_media_status_update(aq_dev, link_speed, &fc_neg);

		/* Update ITR settings according new link speed */
		aq_hw_interrupt_moderation_set(hw);
	} else if (link_speed == 0 && aq_dev->linkup) {
		aq_dev->linkup = 0;
		/* turn off RX Pause in RPB */
		rpb_rx_xoff_en_per_tc_set(hw, 0, 0);

		iflib_link_state_change(ctx, LINK_STATE_DOWN,  0);
		aq_media_status_update(aq_dev, link_speed, &fc_neg);
	}

	aq_update_hw_stats(aq_dev);
}

int aq_isr_rx(void *arg)
{
	aq_ring_t	*ring = arg;
	aq_dev_t	*aq_dev = ring->dev;
	aq_hw_t		*hw = &aq_dev->hw;

	/* clear interrupt status */
	itr_irq_status_clearlsw_set(hw, BIT(ring->msix));
	ring->stats.irq++;
	return (FILTER_SCHEDULE_THREAD);
}

int aq_linkstat_isr(void *arg)
{
	aq_dev_t	*aq_dev = arg;
	aq_hw_t		*hw = &aq_dev->hw;

	/* Clear interrupt status */
	itr_irq_status_clearlsw_set(hw, aq_dev->msix);

	iflib_admin_intr_deferred(aq_dev->ctx);

	return (FILTER_HANDLED);
}
