.PATH:		${SRCTOP}/sys/dev/aq

KMOD=		if_aq

SRCS+=		aq_fw.c
SRCS+=		aq_fw1x.c
SRCS+=		aq_fw2x.c
SRCS+=		aq_hw.c
SRCS+=		aq_hw_llh.c
SRCS+=		aq_irq.c
SRCS+=		aq_main.c
SRCS+=		aq_media.c
SRCS+=		aq_ring.c

SRCS+=		bus_if.h
SRCS+=		device_if.h
SRCS+=		ifdi_if.h
SRCS+=		pci_if.h

SRCS+=		opt_inet.h
SRCS+=		opt_inet6.h
SRCS+=		opt_rss.h

.include	<bsd.kmod.mk>
