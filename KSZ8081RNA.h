
#ifndef __LOCCIONI_KSZ8081RNA_H
#define __LOCCIONI_KSZ8081RNA_H

#include "libohiboard.h"
#include "lwip/netif.h"

typedef struct _KSZ8081RNA_status
{
	uint32_t	renegCount;
	bool		autoNegEnd;		/* 0->NO		1->YES*/
	bool		remoteAuto;		/* 0->NO		1->YES*/
	bool		remoteFault;	/* 0->NO		1->YES*/
	bool		acknoledge;		/* 0->NO		1->YES*/
	bool		linkStatus; 	/* 0->down		1->up */
	bool		cableX;			/* 0->straight	1->cross*/
	bool		cablePlugged;	/* 0->NO		1->YES */
	bool		cableUnplugged;	/* 0->NO		1->YES */
	bool		oldcablePlug;
	uint8_t		speed;
	uint16_t	RXErrors;

} KSZ8081_stat,*ptrKSZ8081_stat;

/**
 *  Initializes the PHY device.
 *
 *  @param[in] dev The ENET device of libohiboard
 */
void KSZ8081RNA_init(Ethernet_DeviceHandle dev);
void KSZ8081RNA_status(ptrKSZ8081_stat S, struct netif * N);


#endif /* __LOCCIONI_KSZ8081RNA_H */
