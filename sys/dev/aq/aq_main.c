/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2019 aQuantia Corporation. All rights reserved
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

MALLOC_DEFINE(M_AQ, "aq", "Aquantia");

char aq_driver_version[] = AQ_VER;

#define AQUANTIA_VENDOR_ID	0x1d6a

#define AQ_DEVICE_ID_0001	0x0001
#define AQ_DEVICE_ID_D100	0xd100
#define AQ_DEVICE_ID_D107	0xd107
#define AQ_DEVICE_ID_D108	0xd108
#define AQ_DEVICE_ID_D109	0xd109

#define AQ_DEVICE_ID_AQC100	0x00b1
#define AQ_DEVICE_ID_AQC107	0x07b1
#define AQ_DEVICE_ID_AQC108	0x08b1
#define AQ_DEVICE_ID_AQC109	0x09b1
#define AQ_DEVICE_ID_AQC111	0x11b1
#define AQ_DEVICE_ID_AQC112	0x12b1

#define AQ_DEVICE_ID_AQC100S	0x80b1
#define AQ_DEVICE_ID_AQC107S	0x87b1
#define AQ_DEVICE_ID_AQC108S	0x88b1
#define AQ_DEVICE_ID_AQC109S	0x89b1
#define AQ_DEVICE_ID_AQC111S	0x91b1
#define AQ_DEVICE_ID_AQC112S	0x92b1

static pci_vendor_info_t aq_vendor_info_array[] = {
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_0001,
	    "Aquantia AQtion 10Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_D107,
	    "Aquantia AQtion 10Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_D108,
	    "Aquantia AQtion 5Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_D109,
	    "Aquantia AQtion 2.5Gbit Network Adapter"),

	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC107,
	    "Aquantia AQtion 10Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC108,
	    "Aquantia AQtion 5Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC109,
	    "Aquantia AQtion 2.5Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC100,
	    "Aquantia AQtion 10Gbit Network Adapter"),

	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC107S,
	    "Aquantia AQtion 10Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC108S,
	    "Aquantia AQtion 5Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC109S,
	    "Aquantia AQtion 2.5Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC100S,
	    "Aquantia AQtion 10Gbit Network Adapter"),

	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC111,
	    "Aquantia AQtion 5Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC112,
	    "Aquantia AQtion 2.5Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC111S,
	    "Aquantia AQtion 5Gbit Network Adapter"),
	PVID(AQUANTIA_VENDOR_ID, AQ_DEVICE_ID_AQC112S,
	    "Aquantia AQtion 2.5Gbit Network Adapter"),

	PVID_END
};

/* Device setup, teardown, etc */
static void *aq_register(device_t);
static int aq_if_attach_pre(if_ctx_t);
static int aq_if_attach_post(if_ctx_t);
static int aq_if_detach(if_ctx_t);
static int aq_if_shutdown(if_ctx_t);
static int aq_if_suspend(if_ctx_t);
static int aq_if_resume(if_ctx_t);

/* Soft queue setup and teardown */
static int aq_if_tx_queues_alloc(if_ctx_t, caddr_t *, uint64_t *, int, int);
static int aq_if_rx_queues_alloc(if_ctx_t, caddr_t *, uint64_t *, int, int);
static void aq_if_queues_free(if_ctx_t);

/* Device configuration */
static void aq_if_init(if_ctx_t);
static void aq_if_stop(if_ctx_t);
static void aq_if_multi_set(if_ctx_t);
static int aq_if_mtu_set(if_ctx_t, uint32_t);
static void aq_if_media_status(if_ctx_t, struct ifmediareq *);
static int aq_if_media_change(if_ctx_t);
static int aq_if_promisc_set(if_ctx_t, int);
static uint64_t aq_if_get_counter(if_ctx_t, ift_counter);
static void aq_if_timer(if_ctx_t, uint16_t);
//static int aq_if_priv_ioctl(if_ctx_t, u_long, caddr_t);
static int aq_hw_capabilities(aq_dev_t *);
static void aq_add_stats_sysctls(aq_dev_t *);

/* Interrupt enable/disable */
static void aq_if_enable_intr(if_ctx_t);
static void aq_if_disable_intr(if_ctx_t);
static int aq_if_rx_queue_intr_enable(if_ctx_t, uint16_t);
static int aq_if_msix_intr_assign(if_ctx_t, int);

/* VLAN support */
static bool aq_is_vlan_promisc_required(aq_dev_t *);
static void aq_update_vlan_filters(aq_dev_t *);
static void aq_if_vlan_register(if_ctx_t, uint16_t);
static void aq_if_vlan_unregister(if_ctx_t, uint16_t);

/* Informational/diagnostic */
//static void	aq_if_debug(if_ctx_t);
static void	aq_if_led_func(if_ctx_t, int);

static device_method_t aq_methods[] = {
	DEVMETHOD(device_register, aq_register),
	DEVMETHOD(device_probe, iflib_device_probe),
	DEVMETHOD(device_attach, iflib_device_attach),
	DEVMETHOD(device_detach, iflib_device_detach),
	DEVMETHOD(device_shutdown, iflib_device_shutdown),
	DEVMETHOD(device_suspend, iflib_device_suspend),
	DEVMETHOD(device_resume, iflib_device_resume),

	DEVMETHOD_END
};

static driver_t aq_driver = {
	"aq", aq_methods, sizeof(aq_dev_t),
};

DRIVER_MODULE(atlantic, pci, aq_driver, 0, 0);

MODULE_DEPEND(atlantic, pci, 1, 1, 1);
MODULE_DEPEND(atlantic, ether, 1, 1, 1);
MODULE_DEPEND(atlantic, iflib, 1, 1, 1);

IFLIB_PNP_INFO(pci, atlantic, aq_vendor_info_array);

static device_method_t aq_if_methods[] = {
	/* Device setup, teardown, etc */
	DEVMETHOD(ifdi_attach_pre, aq_if_attach_pre),
	DEVMETHOD(ifdi_attach_post, aq_if_attach_post),
	DEVMETHOD(ifdi_detach, aq_if_detach),

	DEVMETHOD(ifdi_shutdown, aq_if_shutdown),
	DEVMETHOD(ifdi_suspend, aq_if_suspend),
	DEVMETHOD(ifdi_resume, aq_if_resume),

	/* Soft queue setup and teardown */
	DEVMETHOD(ifdi_tx_queues_alloc, aq_if_tx_queues_alloc),
	DEVMETHOD(ifdi_rx_queues_alloc, aq_if_rx_queues_alloc),
	DEVMETHOD(ifdi_queues_free, aq_if_queues_free),

	/* Device configuration */
	DEVMETHOD(ifdi_init, aq_if_init),
	DEVMETHOD(ifdi_stop, aq_if_stop),
	DEVMETHOD(ifdi_multi_set, aq_if_multi_set),
	DEVMETHOD(ifdi_mtu_set, aq_if_mtu_set),
	DEVMETHOD(ifdi_media_status, aq_if_media_status),
	DEVMETHOD(ifdi_media_change, aq_if_media_change),
	DEVMETHOD(ifdi_promisc_set, aq_if_promisc_set),
	DEVMETHOD(ifdi_get_counter, aq_if_get_counter),
	DEVMETHOD(ifdi_update_admin_status, aq_if_update_admin_status),
	DEVMETHOD(ifdi_timer, aq_if_timer),
//	DEVMETHOD(ifdi_priv_ioctl, aq_if_priv_ioctl),

	/* Interrupt enable / disable */
	DEVMETHOD(ifdi_intr_enable, aq_if_enable_intr),
	DEVMETHOD(ifdi_intr_disable, aq_if_disable_intr),
	DEVMETHOD(ifdi_rx_queue_intr_enable, aq_if_rx_queue_intr_enable),
	DEVMETHOD(ifdi_tx_queue_intr_enable, aq_if_rx_queue_intr_enable),
	DEVMETHOD(ifdi_msix_intr_assign, aq_if_msix_intr_assign),

	/* VLAN support */
	DEVMETHOD(ifdi_vlan_register, aq_if_vlan_register),
	DEVMETHOD(ifdi_vlan_unregister, aq_if_vlan_unregister),

	/* Informational/diagnostic */
	DEVMETHOD(ifdi_led_func, aq_if_led_func),
//	DEVMETHOD(ifdi_debug, aq_if_debug),

	DEVMETHOD_END
};

static driver_t aq_if_driver = {
	"aq_if", aq_if_methods, sizeof(aq_dev_t)
};

static struct if_shared_ctx aq_sctx_init = {
	.isc_magic = IFLIB_MAGIC,
	.isc_q_align = PAGE_SIZE,
	.isc_tx_maxsize = HW_ATL_B0_TSO_SIZE,
	.isc_tx_maxsegsize = HW_ATL_B0_MTU_JUMBO,
	.isc_tso_maxsize = HW_ATL_B0_TSO_SIZE,
	.isc_tso_maxsegsize = HW_ATL_B0_MTU_JUMBO,
	.isc_rx_maxsize = HW_ATL_B0_MTU_JUMBO,
	.isc_rx_nsegments = 16,
	.isc_rx_maxsegsize = PAGE_SIZE,
	.isc_nfl = 1,
	.isc_nrxqs = 1,
	.isc_ntxqs = 1,
	.isc_admin_intrcnt = 1,
	.isc_vendor_info = aq_vendor_info_array,
	.isc_driver_version = aq_driver_version,
	.isc_driver = &aq_if_driver,
	.isc_flags = IFLIB_NEED_SCRATCH | IFLIB_TSO_INIT_IP |
	    IFLIB_NEED_ZERO_CSUM,

	.isc_nrxd_min = { HW_ATL_B0_MIN_RXD },
	.isc_ntxd_min = { HW_ATL_B0_MIN_TXD },
	.isc_nrxd_max = { HW_ATL_B0_MAX_RXD },
	.isc_ntxd_max = { HW_ATL_B0_MAX_TXD },
	.isc_nrxd_default = { PAGE_SIZE / sizeof(aq_txc_desc_t) * 4 },
	.isc_ntxd_default = { PAGE_SIZE / sizeof(aq_txc_desc_t) * 4 },
};

static SYSCTL_NODE(_hw, OID_AUTO, aq, CTLFLAG_RD, 0, "aq driver parameters");
/* UDP Receive-Side Scaling */
static int aq_enable_rss_udp = 1;
SYSCTL_INT(_hw_aq, OID_AUTO, enable_rss_udp, CTLFLAG_RDTUN, &aq_enable_rss_udp,
    0, "enable Receive-Side Scaling (RSS) for UDP");

static void *
aq_register(device_t dev)
{
	return (&aq_sctx_init);
}

static int
aq_if_attach_pre(if_ctx_t ctx)
{
	aq_dev_t	*softc = iflib_get_softc(ctx);
	aq_hw_t		*hw;
	if_softc_ctx_t	scctx;

	softc->ctx = ctx;
	softc->dev = iflib_get_dev(ctx);
	softc->media = iflib_get_media(ctx);
	softc->scctx = iflib_get_softc_ctx(ctx);
	softc->sctx = iflib_get_sctx(ctx);
	scctx = softc->scctx;

	softc->mmio_rid = PCIR_BAR(0);
	softc->mmio_res = bus_alloc_resource_any(softc->dev, SYS_RES_MEMORY,
	    &softc->mmio_rid, RF_ACTIVE | RF_SHAREABLE);
	if (softc->mmio_res == NULL) {
		device_printf(softc->dev,
		    "failed to allocate MMIO resources\n");
		goto fail;
	}

	softc->mmio_tag = rman_get_bustag(softc->mmio_res);
	softc->mmio_handle = rman_get_bushandle(softc->mmio_res);
	softc->mmio_size = rman_get_size(softc->mmio_res);
	softc->hw.hw_addr = (uint8_t *)softc->mmio_handle;
	hw = &softc->hw;
	hw->dev = iflib_get_dev(ctx);
	hw->link_rate = AQ_FW_SPEED_AUTO;
	hw->itr = -1;
	hw->fc.fc_rx = 1;
	hw->fc.fc_tx = 1;
	softc->linkup = false;

	/* Look up ops and caps */
	if (aq_hw_mpi_create(hw) != 0)
		goto fail;

	if (hw->fast_start_enabled) {
		if (hw->fw_ops && hw->fw_ops->reset)
			hw->fw_ops->reset(hw);
	} else
		aq_hw_reset(&softc->hw);
	aq_hw_capabilities(softc);
	aq_hw_get_mac_permanent(hw, hw->mac_addr);

	softc->admin_ticks = 0;
	iflib_set_mac(ctx, hw->mac_addr);

	/* TODO check */
#if 0
	/* since FreeBSD13 deadlock due to calling iflib_led_func() under CTX_LOCK() */
	iflib_led_create(ctx);
#endif

	scctx->isc_tx_csum_flags = CSUM_IP | CSUM_TCP | CSUM_UDP | CSUM_TSO;
	scctx->isc_capabilities = IFCAP_RXCSUM | IFCAP_TXCSUM | IFCAP_HWCSUM |
	    IFCAP_TSO | IFCAP_JUMBO_MTU | IFCAP_VLAN_HWFILTER | IFCAP_VLAN_MTU |
	    IFCAP_VLAN_HWTAGGING | IFCAP_VLAN_HWCSUM;
	scctx->isc_capenable = scctx->isc_capabilities;
	scctx->isc_tx_nsegments = 31,
	scctx->isc_tx_tso_segments_max = 31;
	scctx->isc_tx_tso_size_max = HW_ATL_B0_TSO_SIZE -
	    sizeof(struct ether_vlan_header);
	scctx->isc_tx_tso_segsize_max = HW_ATL_B0_MTU_JUMBO;
	scctx->isc_min_frame_size = 52;
	scctx->isc_txrx = &aq_txrx;

	scctx->isc_txqsizes[0] = sizeof(aq_tx_desc_t) * scctx->isc_ntxd[0];
	scctx->isc_rxqsizes[0] = sizeof(aq_rx_desc_t) * scctx->isc_nrxd[0];

	scctx->isc_ntxqsets_max = HW_ATL_B0_RINGS_MAX;
	scctx->isc_nrxqsets_max = HW_ATL_B0_RINGS_MAX;

	/* iflib will map and release this bar */
	scctx->isc_msix_bar = pci_msix_table_bar(softc->dev);

	softc->vlan_tags = bit_alloc(4096, M_AQ, M_NOWAIT);

	return (0);
fail:
	if (softc->mmio_res != NULL)
		bus_release_resource(softc->dev, SYS_RES_MEMORY,
		    softc->mmio_rid, softc->mmio_res);
	return (ENXIO);
}

static int
aq_if_attach_post(if_ctx_t ctx)
{
	aq_dev_t	*softc = iflib_get_softc(ctx);

	aq_update_hw_stats(softc);
	aq_media_init(softc);

	switch (softc->scctx->isc_intr) {
	case IFLIB_INTR_LEGACY:
		return (EOPNOTSUPP);
	case IFLIB_INTR_MSI:
		break;
	case IFLIB_INTR_MSIX:
		break;
	default:
		device_printf(softc->dev, "unknown interrupt mode\n");
		return (EOPNOTSUPP);
	}

	aq_add_stats_sysctls(softc);
	/* RSS */
	arc4rand(softc->rss_key, HW_ATL_RSS_HASHKEY_SIZE, 0);
	for (int i = ARRAY_SIZE(softc->rss_table); i--;)
		softc->rss_table[i] = i & (softc->rx_rings_count - 1);

	return (0);
}

static int
aq_if_detach(if_ctx_t ctx)
{
	aq_dev_t	*softc;
	int		i;

	softc = iflib_get_softc(ctx);

	aq_hw_deinit(&softc->hw);

	for (i = 0; i < softc->scctx->isc_nrxqsets; i++)
		iflib_irq_free(ctx, &softc->rx_rings[i]->irq);
	iflib_irq_free(ctx, &softc->irq);

	if (softc->mmio_res != NULL)
		bus_release_resource(softc->dev, SYS_RES_MEMORY,
		    softc->mmio_rid, softc->mmio_res);

	free(softc->vlan_tags, M_AQ);

	return (0);
}

static int
aq_if_shutdown(if_ctx_t ctx)
{
	return (0);
}

static int
aq_if_suspend(if_ctx_t ctx)
{
	return (0);
}

static int
aq_if_resume(if_ctx_t ctx)
{
	return (0);
}

/* Soft queue setup and teardown */
static int
aq_if_tx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs,
    int ntxqs, int ntxqsets)
{
	aq_dev_t	*softc = iflib_get_softc(ctx);
	aq_ring_t	*ring;
	int		i;

	for (i = 0; i < ntxqsets; i++) {
		/* XXX why nowait? */
		ring = softc->tx_rings[i] = malloc(sizeof(aq_ring_t), M_AQ,
		    M_NOWAIT | M_ZERO);
		if (ring == NULL) {
			device_printf(softc->dev, "failed to alloc tx ring\n");
			aq_if_queues_free(ctx);
			return (ENOMEM);
		}
		ring->tx_descs = (aq_tx_desc_t*)vaddrs[i];
		ring->tx_size = softc->scctx->isc_ntxd[0];
		ring->tx_descs_phys = paddrs[i];
		ring->tx_head = ring->tx_tail = 0;
		ring->index = i;
		ring->dev = softc;

		softc->tx_rings_count++;
	}

	return (0);
}

static int
aq_if_rx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs,
    int nrxqs, int nrxqsets)
{
	aq_dev_t	*softc = iflib_get_softc(ctx);
	aq_ring_t	*ring;
	int		i;

	for (i = 0; i < nrxqsets; i++) {
		/* XXX why nowait? */
		ring = softc->rx_rings[i] = malloc(sizeof(aq_ring_t), M_AQ,
		    M_NOWAIT | M_ZERO);
		if (ring == NULL) {
			device_printf(softc->dev, "failed to alloc rx ring\n");
			aq_if_queues_free(ctx);
			return (ENOMEM);
		}
		ring->rx_descs = (aq_rx_desc_t*)vaddrs[i];
		ring->rx_descs_phys = paddrs[i];
		ring->rx_size = softc->scctx->isc_nrxd[0];
		ring->index = i;
		ring->dev = softc;

		switch (MCLBYTES) {
		case 4 * 1024:
		case 8 * 1024:
		case 16 * 1024:
			ring->rx_max_frame_size = MCLBYTES;
			break;
		default:
			ring->rx_max_frame_size = 2048;
			break;
		}
		softc->rx_rings_count++;
	}

	return (0);
}

static void
aq_if_queues_free(if_ctx_t ctx)
{
	aq_dev_t	*softc = iflib_get_softc(ctx);
	int		i;


	for (i = 0; i < softc->tx_rings_count; i++) {
		if (softc->tx_rings[i]) {
			free(softc->tx_rings[i], M_AQ);
			softc->tx_rings[i] = NULL;
		}
	}
	softc->tx_rings_count = 0;
	for (i = 0; i < softc->rx_rings_count; i++) {
		if (softc->rx_rings[i]){
			free(softc->rx_rings[i], M_AQ);
			softc->rx_rings[i] = NULL;
		}
	}
	softc->rx_rings_count = 0;
}

/* Device configuration */
static void
aq_if_init(if_ctx_t ctx)
{
	aq_dev_t	*softc = iflib_get_softc(ctx);
	aq_hw_t		*hw = &softc->hw;
	struct ifmediareq ifmr;
	int		i, err;

	err = aq_hw_init(&softc->hw, softc->hw.mac_addr, softc->msix,
	    softc->scctx->isc_intr == IFLIB_INTR_MSIX);
	if (err != 0)
		device_printf(softc->dev, "aq_hw_init: %d", err);

	aq_if_media_status(ctx, &ifmr);

	aq_update_vlan_filters(softc);

	for (i = 0; i < softc->tx_rings_count; i++) {
		aq_ring_t *ring = softc->tx_rings[i];
		err = aq_ring_tx_init(&softc->hw, ring);
		if (err) {
			device_printf(softc->dev, "atlantic: aq_ring_tx_init: %d", err);
		}
		err = aq_ring_tx_start(hw, ring);
		if (err != 0) {
			device_printf(softc->dev, "atlantic: aq_ring_tx_start: %d", err);
		}
	}
	for (i = 0; i < softc->rx_rings_count; i++) {
		aq_ring_t *ring = softc->rx_rings[i];
		err = aq_ring_rx_init(&softc->hw, ring);
		if (err) {
			device_printf(softc->dev, "atlantic: aq_ring_rx_init: %d", err);
		}
		err = aq_ring_rx_start(hw, ring);
		if (err != 0) {
			device_printf(softc->dev, "atlantic: aq_ring_rx_start: %d", err);
		}
		aq_if_rx_queue_intr_enable(ctx, i);
	}

	aq_hw_start(hw);
	aq_if_enable_intr(ctx);
	aq_hw_rss_hash_set(&softc->hw, softc->rss_key);
	aq_hw_rss_set(&softc->hw, softc->rss_table);
	aq_hw_udp_rss_enable(hw, aq_enable_rss_udp);
	aq_hw_set_link_speed(hw, hw->link_rate);
}

static void
aq_if_stop(if_ctx_t ctx)
{
	aq_dev_t *softc;
	struct aq_hw *hw;
	int i;

	softc = iflib_get_softc(ctx);
	hw = &softc->hw;

	/* disable interrupt */
	aq_if_disable_intr(ctx);

	for (i = 0; i < softc->tx_rings_count; i++) {
		aq_ring_tx_stop(hw, softc->tx_rings[i]);
		softc->tx_rings[i]->tx_head = 0;
		softc->tx_rings[i]->tx_tail = 0;
	}
	for (i = 0; i < softc->rx_rings_count; i++) {
		aq_ring_rx_stop(hw, softc->rx_rings[i]);
	}

	aq_hw_reset(&softc->hw);
	memset(&softc->last_stats, 0, sizeof(softc->last_stats));
	softc->linkup = false;
	aq_if_update_admin_status(ctx);
}

static uint64_t
aq_if_get_counter(if_ctx_t ctx, ift_counter cnt)
{
	aq_dev_t	*softc;
	if_t		ifp;

	softc = iflib_get_softc(ctx);
	ifp = iflib_get_ifp(ctx);

	switch (cnt) {
	case IFCOUNTER_IERRORS:
		return (softc->curr_stats.erpr);
	case IFCOUNTER_IQDROPS:
		return (softc->curr_stats.dpc);
	case IFCOUNTER_OERRORS:
		return (softc->curr_stats.erpt);
	default:
		return (if_get_counter_default(ifp, cnt));
	}
}

static u_int
aq_mc_filter_apply(void *arg, struct sockaddr_dl *dl, u_int count)
{
	aq_dev_t *softc = arg;
	struct aq_hw *hw = &softc->hw;
	uint8_t *mac_addr = NULL;

	if (count == AQ_HW_MAC_MAX)
		return (0);

	mac_addr = LLADDR(dl);
	aq_hw_mac_addr_set(hw, mac_addr, count + 1);

	return (1);
}

static bool
aq_is_mc_promisc_required(aq_dev_t *softc)
{
	return (softc->mcnt >= AQ_HW_MAC_MAX);
}

static void
aq_if_multi_set(if_ctx_t ctx)
{
	aq_dev_t *softc = iflib_get_softc(ctx);
	struct ifnet  *ifp = iflib_get_ifp(ctx);
	struct aq_hw  *hw = &softc->hw;

	softc->mcnt = if_llmaddr_count(iflib_get_ifp(ctx));
	if (softc->mcnt >= AQ_HW_MAC_MAX) {
		aq_hw_set_promisc(hw, !!(ifp->if_flags & IFF_PROMISC),
		    aq_is_vlan_promisc_required(softc),
		    !!(ifp->if_flags & IFF_ALLMULTI) ||
		    aq_is_mc_promisc_required(softc));
	} else {
		if_foreach_llmaddr(iflib_get_ifp(ctx), &aq_mc_filter_apply,
		    softc);
	}
}

static int
aq_if_mtu_set(if_ctx_t ctx, uint32_t mtu)
{
	return (0);
}

static void
aq_if_media_status(if_ctx_t ctx, struct ifmediareq *ifmr)
{
	struct ifnet *ifp;

	ifp = iflib_get_ifp(ctx);

	aq_media_status(ifp, ifmr);
}

static int
aq_if_media_change(if_ctx_t ctx)
{
	struct ifnet	*ifp = iflib_get_ifp(ctx);

	/* Not allowed in UP state, since causes unsync of rings */
	if (ifp->if_flags & IFF_UP)
		return (EPERM);

	aq_media_change(ifp);

	return (0);
}

static int
aq_if_promisc_set(if_ctx_t ctx, int flags)
{
	aq_dev_t *softc = iflib_get_softc(ctx);

	aq_hw_set_promisc(&softc->hw, !!(flags & IFF_PROMISC),
	    aq_is_vlan_promisc_required(softc),
	    !!(flags & IFF_ALLMULTI) || aq_is_mc_promisc_required(softc));

	return (0);
}

static void
aq_if_timer(if_ctx_t ctx, uint16_t qid)
{
	aq_dev_t *softc;
	uint64_t ticks_now;

	softc = iflib_get_softc(ctx);
	ticks_now = ticks;

	/* Schedule aqc_if_update_admin_status() once per sec */
	if (ticks_now - softc->admin_ticks >= hz) {
		softc->admin_ticks = ticks_now;
		iflib_admin_intr_deferred(ctx);
	}
}

/* Interrupt enable / disable */
static void
aq_if_enable_intr(if_ctx_t ctx)
{
	aq_dev_t *softc = iflib_get_softc(ctx);
	struct aq_hw  *hw = &softc->hw;

	/* Enable interrupts */
	itr_irq_msk_setlsw_set(hw, BIT(softc->msix + 1) - 1);
}

static void
aq_if_disable_intr(if_ctx_t ctx)
{
	aq_dev_t	*softc = iflib_get_softc(ctx);
	aq_hw_t		*hw = &softc->hw;

	/* Disable interrupts */
	itr_irq_msk_clearlsw_set(hw, BIT(softc->msix + 1) - 1);
}

static int
aq_if_rx_queue_intr_enable(if_ctx_t ctx, uint16_t rxqid)
{
	aq_dev_t *softc = iflib_get_softc(ctx);
	struct aq_hw  *hw = &softc->hw;

	itr_irq_msk_setlsw_set(hw, BIT(softc->rx_rings[rxqid]->msix));

	return (0);
}

static int
aq_if_msix_intr_assign(if_ctx_t ctx, int msix)
{
	aq_dev_t	*softc = iflib_get_softc(ctx);
	char		irq_name[16];
	int		vector = 0;
	int		rx_vectors;
	int		ret;
	int		i;

	for (i = 0; i < softc->rx_rings_count; i++, vector++) {
		snprintf(irq_name, sizeof(irq_name), "rxq%d", i);
		ret = iflib_irq_alloc_generic(ctx, &softc->rx_rings[i]->irq,
		    vector + 1, IFLIB_INTR_RX, aq_isr_rx, softc->rx_rings[i],
			softc->rx_rings[i]->index, irq_name);
		if (ret != 0) {
			device_printf(iflib_get_dev(ctx),
			    "failed to set up rx handler %d\n", i);
			i--;
			goto fail;
		}
		softc->rx_rings[i]->msix = vector;
	}
	rx_vectors = vector;

	for (i = 0; i < softc->tx_rings_count; i++, vector++) {
		snprintf(irq_name, sizeof(irq_name), "txq%d", i);
		iflib_softirq_alloc_generic(ctx, &softc->rx_rings[i]->irq,
		    IFLIB_INTR_TX, softc->tx_rings[i], i, irq_name);
		softc->tx_rings[i]->msix = vector % softc->rx_rings_count;
	}

	ret = iflib_irq_alloc_generic(ctx, &softc->irq, rx_vectors + 1,
	    IFLIB_INTR_ADMIN, aq_linkstat_isr, softc, 0, "aq");
	if (ret != 0) {
		device_printf(iflib_get_dev(ctx),
		    "failed to register admin handler\n");
		i = softc->rx_rings_count;
		goto fail;
	}
	softc->msix = rx_vectors;
	return (0);
fail:
	for (; i >= 0; i--)
		iflib_irq_free(ctx, &softc->rx_rings[i]->irq);
	return (ret);
}

static bool
aq_is_vlan_promisc_required(aq_dev_t *softc)
{
	int vlan_tag_count;

	bit_count(softc->vlan_tags, 0, 4096, &vlan_tag_count);

	if (vlan_tag_count <= AQ_HW_VLAN_MAX_FILTERS)
		return (false);

	return (true);
}

static void
aq_update_vlan_filters(aq_dev_t *softc)
{
	aq_rx_filter_vlan_t aq_vlans[AQ_HW_VLAN_MAX_FILTERS];
	aq_hw_t		*hw = &softc->hw;
	int		bit_pos = 0;
	int		vlan_tag = -1;
	int		i;

	hw_atl_b0_hw_vlan_promisc_set(hw, true);
	for (i = 0; i < AQ_HW_VLAN_MAX_FILTERS; i++) {
		bit_ffs_at(softc->vlan_tags, bit_pos, 4096, &vlan_tag);
		if (vlan_tag != -1) {
			aq_vlans[i].enable = true;
			aq_vlans[i].location = i;
			aq_vlans[i].queue = 0xFF;
			aq_vlans[i].vlan_id = vlan_tag;
			bit_pos = vlan_tag;
		} else {
			aq_vlans[i].enable = false;
		}
	}

	hw_atl_b0_hw_vlan_set(hw, aq_vlans);
	hw_atl_b0_hw_vlan_promisc_set(hw, aq_is_vlan_promisc_required(softc));
}

/* VLAN support */
static void
aq_if_vlan_register(if_ctx_t ctx, uint16_t vtag)
{
	aq_dev_t *softc = iflib_get_softc(ctx);

	bit_set(softc->vlan_tags, vtag);

	aq_update_vlan_filters(softc);
}

static void
aq_if_vlan_unregister(if_ctx_t ctx, uint16_t vtag)
{
	aq_dev_t *softc = iflib_get_softc(ctx);

	bit_clear(softc->vlan_tags, vtag);

	aq_update_vlan_filters(softc);
}

static void
aq_if_led_func(if_ctx_t ctx, int onoff)
{
	aq_dev_t	*softc = iflib_get_softc(ctx);
	aq_hw_t		*hw = &softc->hw;

	if (hw->fw_ops && hw->fw_ops->led_control)
		hw->fw_ops->led_control(hw, onoff);
}

static int
aq_hw_capabilities(aq_dev_t *softc)
{
	if (pci_get_vendor(softc->dev) != AQUANTIA_VENDOR_ID)
		return (ENXIO);

	switch (pci_get_device(softc->dev)) {
	case AQ_DEVICE_ID_D100:
	case AQ_DEVICE_ID_AQC100:
	case AQ_DEVICE_ID_AQC100S:
		softc->media_type = AQ_MEDIA_TYPE_FIBRE;
		softc->link_speeds = AQ_LINK_ALL & ~AQ_LINK_10G;
		break;

	case AQ_DEVICE_ID_0001:
	case AQ_DEVICE_ID_D107:
	case AQ_DEVICE_ID_AQC107:
	case AQ_DEVICE_ID_AQC107S:
		softc->media_type = AQ_MEDIA_TYPE_TP;
		softc->link_speeds = AQ_LINK_ALL;
		break;

	case AQ_DEVICE_ID_D108:
	case AQ_DEVICE_ID_AQC108:
	case AQ_DEVICE_ID_AQC108S:
	case AQ_DEVICE_ID_AQC111:
	case AQ_DEVICE_ID_AQC111S:
		softc->media_type = AQ_MEDIA_TYPE_TP;
		softc->link_speeds = AQ_LINK_ALL & ~AQ_LINK_10G;
		break;

	case AQ_DEVICE_ID_D109:
	case AQ_DEVICE_ID_AQC109:
	case AQ_DEVICE_ID_AQC109S:
	case AQ_DEVICE_ID_AQC112:
	case AQ_DEVICE_ID_AQC112S:
		softc->media_type = AQ_MEDIA_TYPE_TP;
		softc->link_speeds = AQ_LINK_ALL & ~(AQ_LINK_10G | AQ_LINK_5G);
		break;

	default:
		return (ENXIO);
	}

	return (0);
}

static int
aq_sysctl_print_rss_config(SYSCTL_HANDLER_ARGS)
{
	aq_dev_t	*softc = (aq_dev_t *)arg1;
	device_t	dev = softc->dev;
	struct sbuf	*buf;

	buf = sbuf_new_for_sysctl(NULL, NULL, 256, req);
	if (buf == NULL) {
		device_printf(dev, "failed to alloc output sbuf\n");
		return (ENOMEM);
	}

	/* Print out the redirection table */
	sbuf_cat(buf, "\nRSS Indirection table:\n");
	for (int i = 0; i < HW_ATL_RSS_INDIRECTION_TABLE_MAX; i++) {
		sbuf_printf(buf, "%d ", softc->rss_table[i]);
		if ((i+1) % 10 == 0)
			sbuf_printf(buf, "\n");
	}

	sbuf_cat(buf, "\nRSS Key:\n");
	for (int i = 0; i < HW_ATL_RSS_HASHKEY_SIZE; i++)
		sbuf_printf(buf, "0x%02x ", softc->rss_key[i]);
	sbuf_printf(buf, "\n");

	if (sbuf_finish(buf) != 0)
		device_printf(dev, "failed to finish sbuf\n");
	sbuf_delete(buf);

	return (0);
}

static int
aq_sysctl_print_tx_head(SYSCTL_HANDLER_ARGS)
{
	aq_ring_t  *ring = arg1;
	int             error = 0;
	unsigned int   val;

	if (!ring)
		return (0);

	val = tdm_tx_desc_head_ptr_get(&ring->dev->hw, ring->index);

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || !req->newptr)
		return (error);

	return (0);
}

static int aq_sysctl_print_tx_tail(SYSCTL_HANDLER_ARGS)
{
	aq_ring_t  *ring = arg1;
	int             error = 0;
	unsigned int   val;

	if (!ring)
		return (0);

	val = reg_tx_dma_desc_tail_ptr_get(&ring->dev->hw, ring->index);

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || !req->newptr)
		return (error);

	return (0);
}

static int
aq_sysctl_print_rx_head(SYSCTL_HANDLER_ARGS)
{
	aq_ring_t	*ring = arg1;
	unsigned int	val;
	int		ret;

	if (ring == NULL)
		return (0);

	val = rdm_rx_desc_head_ptr_get(&ring->dev->hw, ring->index);
	ret = sysctl_handle_int(oidp, &val, 0, req);
	if (ret != 0 || req->newptr == NULL)
		return (ret);

	return (0);
}

static int
aq_sysctl_print_rx_tail(SYSCTL_HANDLER_ARGS)
{
	aq_ring_t	*ring = arg1;
	unsigned int	val;
	int		ret;

	if (ring == NULL)
		return (0);

	val = reg_rx_dma_desc_tail_ptr_get(&ring->dev->hw, ring->index);
	ret = sysctl_handle_int(oidp, &val, 0, req);
	if (ret != 0 || req->newptr == NULL)
		return (ret);

	return (0);
}

static void
aq_add_stats_sysctls(aq_dev_t *softc)
{
	device_t	dev = softc->dev;
	aq_stats_t	*stats = &softc->curr_stats;
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(dev);
	struct sysctl_oid_list *child = SYSCTL_CHILDREN(tree);
	struct sysctl_oid *stat_node, *queue_node;
	struct sysctl_oid_list *stat_list, *queue_list;
#define	QUEUE_NAME_LEN 32
	char		namebuf[QUEUE_NAME_LEN];
	/* RSS configuration */
	SYSCTL_ADD_PROC(ctx, child, OID_AUTO, "print_rss_config",
	    CTLTYPE_STRING | CTLFLAG_RD, softc, 0,
	    aq_sysctl_print_rss_config, "A", "Prints RSS Configuration");

	/* Driver Statistics */
	for (int i = 0; i < softc->tx_rings_count; i++) {
		aq_ring_t *ring = softc->tx_rings[i];
		snprintf(namebuf, QUEUE_NAME_LEN, "tx_queue%d", i);
		queue_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, namebuf,
		    CTLFLAG_RD, NULL, "Queue Name");
		queue_list = SYSCTL_CHILDREN(queue_node);

		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "tx_pkts",
		    CTLFLAG_RD, &(ring->stats.tx_pkts), "TX Packets");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "tx_bytes",
		    CTLFLAG_RD, &(ring->stats.tx_bytes), "TX Octets");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "tx_drops",
		    CTLFLAG_RD, &(ring->stats.tx_drops), "TX Drops");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "tx_queue_full",
		    CTLFLAG_RD, &(ring->stats.tx_queue_full), "TX Queue Full");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "tx_head",
		    CTLTYPE_UINT | CTLFLAG_RD, ring, 0,
		    aq_sysctl_print_tx_head, "IU", "ring head pointer");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "tx_tail",
		    CTLTYPE_UINT | CTLFLAG_RD, ring, 0,
		    aq_sysctl_print_tx_tail, "IU", "ring tail pointer");
	}

	for (int i = 0; i < softc->rx_rings_count; i++) {
		aq_ring_t *ring = softc->rx_rings[i];
		snprintf(namebuf, QUEUE_NAME_LEN, "rx_queue%d", i);
		queue_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, namebuf,
		    CTLFLAG_RD, NULL, "Queue Name");
		queue_list = SYSCTL_CHILDREN(queue_node);

		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "rx_pkts",
		    CTLFLAG_RD, &(ring->stats.rx_pkts), "RX Packets");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "rx_bytes",
		    CTLFLAG_RD, &(ring->stats.rx_bytes), "TX Octets");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "jumbo_pkts",
		    CTLFLAG_RD, &(ring->stats.jumbo_pkts), "Jumbo Packets");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "rx_err",
		    CTLFLAG_RD, &(ring->stats.rx_err), "RX Errors");
		SYSCTL_ADD_UQUAD(ctx, queue_list, OID_AUTO, "irq",
		    CTLFLAG_RD, &(ring->stats.irq), "RX interrupts");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "rx_head",
		    CTLTYPE_UINT | CTLFLAG_RD, ring, 0,
		    aq_sysctl_print_rx_head, "IU", "ring head pointer");
		SYSCTL_ADD_PROC(ctx, queue_list, OID_AUTO, "rx_tail",
		    CTLTYPE_UINT | CTLFLAG_RD, ring, 0,
		    aq_sysctl_print_rx_tail, "IU", " ring tail pointer");
	}

	stat_node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "mac",
	    CTLFLAG_RD, NULL, "Statistics (read from HW registers)");
	stat_list = SYSCTL_CHILDREN(stat_node);

	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_pkts_rcvd",
	    CTLFLAG_RD, &stats->prc, "Good Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "ucast_pkts_rcvd",
	    CTLFLAG_RD, &stats->uprc, "Unicast Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "mcast_pkts_rcvd",
	    CTLFLAG_RD, &stats->mprc, "Multicast Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "bcast_pkts_rcvd",
	    CTLFLAG_RD, &stats->bprc, "Broadcast Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "rsc_pkts_rcvd",
	    CTLFLAG_RD, &stats->cprc, "Coalesced Packets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "err_pkts_rcvd",
	    CTLFLAG_RD, &stats->erpr, "Errors of Packet Receive");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "drop_pkts_dma",
	    CTLFLAG_RD, &stats->dpc, "Dropped Packets in DMA");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_octets_rcvd",
	    CTLFLAG_RD, &stats->brc, "Good Octets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "ucast_octets_rcvd",
	    CTLFLAG_RD, &stats->ubrc, "Unicast Octets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "mcast_octets_rcvd",
	    CTLFLAG_RD, &stats->mbrc, "Multicast Octets Received");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "bcast_octets_rcvd",
	    CTLFLAG_RD, &stats->bbrc, "Broadcast Octets Received");

	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_pkts_txd",
	    CTLFLAG_RD, &stats->ptc, "Good Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "ucast_pkts_txd",
	    CTLFLAG_RD, &stats->uptc, "Unicast Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "mcast_pkts_txd",
	    CTLFLAG_RD, &stats->mptc, "Multicast Packets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "bcast_pkts_txd",
	    CTLFLAG_RD, &stats->bptc, "Broadcast Packets Transmitted");

	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "err_pkts_txd",
	    CTLFLAG_RD, &stats->erpt, "Errors of Packet Transmit");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "good_octets_txd",
	    CTLFLAG_RD, &stats->btc, "Good Octets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "ucast_octets_txd",
	    CTLFLAG_RD, &stats->ubtc, "Unicast Octets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "mcast_octets_txd",
	    CTLFLAG_RD, &stats->mbtc, "Multicast Octets Transmitted");
	SYSCTL_ADD_UQUAD(ctx, stat_list, OID_AUTO, "bcast_octets_txd",
	    CTLFLAG_RD, &stats->bbtc, "Broadcast Octets Transmitted");
}
