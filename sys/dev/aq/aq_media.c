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

#define	AQ_HW_SUPPORT_SPEED(softc, s) ((softc)->link_speeds & s)

void
aq_media_status_update(aq_dev_t *aq_dev, uint32_t link_speed,
    const aq_hw_fc_info_t *fc_neg)
{
	aq_hw_t *hw = &aq_dev->hw;

	aq_dev->media_active = 0;
	if (fc_neg->fc_rx)
	    aq_dev->media_active |= IFM_ETH_RXPAUSE;
	if (fc_neg->fc_tx)
	    aq_dev->media_active |= IFM_ETH_TXPAUSE;

	switch(link_speed) {
	case 100:
		aq_dev->media_active |= IFM_100_TX | IFM_FDX;
		break;
	case 1000:
		aq_dev->media_active |= IFM_1000_T | IFM_FDX;
		break;
	case 2500:
		aq_dev->media_active |= IFM_2500_T | IFM_FDX;
		break;
	case 5000:
		aq_dev->media_active |= IFM_5000_T | IFM_FDX;
		break;
	case 10000:
		aq_dev->media_active |= IFM_10G_T | IFM_FDX;
		break;
	case 0:
	default:
		aq_dev->media_active |= IFM_NONE;
		break;
	}

	if (hw->link_rate == AQ_FW_SPEED_AUTO)
		aq_dev->media_active |= IFM_AUTO;
}

void
aq_media_status(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	aq_dev_t *aq_dev = iflib_get_softc(ifp->if_softc);

	ifmr->ifm_active = IFM_ETHER;
	ifmr->ifm_status = IFM_AVALID;

	if (aq_dev->linkup)
		ifmr->ifm_status |= IFM_ACTIVE;

	ifmr->ifm_active |= aq_dev->media_active;
}

void
aq_media_change(struct ifnet *ifp)
{
	aq_dev_t          *aq_dev = iflib_get_softc(ifp->if_softc);
	struct aq_hw      *hw = &aq_dev->hw;
	int                old_media_rate = ifp->if_baudrate;
	int                old_link_speed = hw->link_rate;
	struct ifmedia    *ifm = iflib_get_media(aq_dev->ctx);
	int                user_media = IFM_SUBTYPE(ifm->ifm_media);
	uint64_t           media_rate;

	if ((ifm->ifm_media & IFM_ETHER) == 0) {
		/* Should not happen */
		device_printf(aq_dev->dev, "bad media: 0x%x\n", ifm->ifm_media);
		return;
	}

	switch (user_media) {
	case IFM_AUTO:
		hw->link_rate = AQ_FW_SPEED_AUTO;
		media_rate = -1;
		break;
	case IFM_NONE:
		media_rate = 0;
		hw->link_rate = 0;
		iflib_link_state_change(aq_dev->ctx, LINK_STATE_DOWN,  0);
		break;
	case IFM_100_TX:
		hw->link_rate = AQ_FW_100M;
		media_rate = 100 * 1000;
		break;
	case IFM_1000_T:
		hw->link_rate = AQ_FW_1G;
		media_rate = 1000 * 1000;
		break;
	case IFM_2500_T:
		hw->link_rate = AQ_FW_2G5;
		media_rate = 2500 * 1000;
		break;
	case IFM_5000_T:
		hw->link_rate = AQ_FW_5G;
		media_rate = 5000 * 1000;
		break;
	case IFM_10G_T:
		hw->link_rate = AQ_FW_10G;
		media_rate = 10000 * 1000;
		break;
	default:
		/* Should not happen */
		device_printf(aq_dev->dev, "unknown media: 0x%x\n", user_media);
		return;
	}
	hw->fc.fc_rx = (ifm->ifm_media & IFM_ETH_RXPAUSE) ? 1 : 0;
	hw->fc.fc_tx = (ifm->ifm_media & IFM_ETH_TXPAUSE) ? 1 : 0;

	/* In down state just remember new link speed */
	if (!(ifp->if_flags & IFF_UP))
		return;

	if (media_rate != old_media_rate || hw->link_rate != old_link_speed) {
		/* Re-initialize hardware with new parameters */
		aq_hw_set_link_speed(hw, hw->link_rate);
	}
}

static void
aq_media_add_types(aq_dev_t *aq_dev, int media_link_speed)
{
	ifmedia_add(aq_dev->media, IFM_ETHER | media_link_speed | IFM_FDX,
	    0, NULL);
	ifmedia_add(aq_dev->media, IFM_ETHER | media_link_speed | IFM_FDX |
	    IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE, 0, NULL);
	ifmedia_add(aq_dev->media, IFM_ETHER | media_link_speed | IFM_FDX |
	    IFM_ETH_RXPAUSE, 0, NULL);
	ifmedia_add(aq_dev->media, IFM_ETHER | media_link_speed | IFM_FDX |
	    IFM_ETH_TXPAUSE, 0, NULL);
}

void
aq_media_init(aq_dev_t *aq_dev)
{
	/* ifconfig eth0 none */
	ifmedia_add(aq_dev->media, IFM_ETHER | IFM_NONE, 0, NULL);

	/* ifconfig eth0 auto */
	aq_media_add_types(aq_dev, IFM_AUTO);

	if (AQ_HW_SUPPORT_SPEED(aq_dev, AQ_LINK_100M))
		aq_media_add_types(aq_dev, IFM_100_TX);
	if (AQ_HW_SUPPORT_SPEED(aq_dev, AQ_LINK_1G))
		aq_media_add_types(aq_dev, IFM_1000_T);
	if (AQ_HW_SUPPORT_SPEED(aq_dev, AQ_LINK_2G5))
		aq_media_add_types(aq_dev, IFM_2500_T);
	if (AQ_HW_SUPPORT_SPEED(aq_dev, AQ_LINK_5G))
		aq_media_add_types(aq_dev, IFM_5000_T);
	if (AQ_HW_SUPPORT_SPEED(aq_dev, AQ_LINK_10G))
		aq_media_add_types(aq_dev, IFM_10G_T);

	/* Link is initially autoselect */
	ifmedia_set(aq_dev->media,
	    IFM_ETHER | IFM_AUTO | IFM_FDX | IFM_ETH_RXPAUSE | IFM_ETH_TXPAUSE);
}
