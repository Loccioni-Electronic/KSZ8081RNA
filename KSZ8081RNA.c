#include "KSZ8081RNA.h"

#define KSZ8081RNA_MAX_TIMEOUT 0xFFFF

// PHY Register Mnemonics
#define  KSZ8081RNA_REGISTER_CTRL    0     /*!< PHY control register.*/
#define  KSZ8081RNA_REGISTER_STATUS  1     /*!< PHY status register.*/
#define  KSZ8081RNA_REGISTER_ID1     2     /*!< PHY identification register 1.*/
#define  KSZ8081RNA_REGISTER_ID2     3     /*!< PHY identification register 2.*/
#define  KSZ8081RNA_REGISTER_ANLPA   5     /*!< Auto-Neg Link Partner Ability.*/
#define  KSZ8081RNA_REGISTER_ANE     6     /*!< Auto-Neg Expansion */
#define  KSZ8081RNA_REGISTER_RXERR   15    /*!< Receive Frame errors */
#define  KSZ8081RNA_REGISTER_EXCTRL  0x18    /*!< Receive Frame errors */
#define  KSZ8081RNA_REGISTER_CT1     0x1E  /*!< PHY control1 register.*/
#define  KSZ8081RNA_REGISTER_CT2     0x1F  /*!< PHY control2 register.*/

// PHY Control Register Masks
#define KSZ8081RNA_CTRL_REG_COLLISION              0x0080  /*!< ENET PHY collision detects.*/
#define KSZ8081RNA_CTRL_REG_DUPLEX                 0x0100  /*!< ENET PHY set half/duplex.*/
#define KSZ8081RNA_CTRL_REG_RESTART_AUTO_NEG       0x0200  /*!< ENET PHY restart auto neg.*/
#define KSZ8081RNA_CTRL_REG_ISOLATE                0x0400  /*!< ENET PHY isolate.*/
#define KSZ8081RNA_CTRL_REG_POWER_DOWN             0x0800  /*!< ENET PHY power down.*/
#define KSZ8081RNA_CTRL_REG_AUTO_NEG_MASK          0x1000  /*!< ENET PHY auto negotiation control.*/
#define KSZ8081RNA_CTRL_REG_PHY_HI_SPEED_MASK      0x2000  /*!< ENET PHY speed control.*/
#define KSZ8081RNA_CTRL_REG_LOOP_MASK              0x4000  /*!< ENET PHY loop control.*/
#define KSZ8081RNA_CTRL_REG_RESET_MASK             0x8000  /*!< ENET PHY reset control.*/

#define KSZ8081RNA_CTRL_REG_10M_HALF_DUPLEX_MASK   0x01    /*!< ENET PHY 10M half duplex.*/
#define KSZ8081RNA_CTRL_REG_100M_HALF_DUPLEX_MASK  0x02    /*!< ENET PHY 100M half duplex.*/
#define KSZ8081RNA_CTRL_REG_10M_FULL_DUPLEX_MASK   0x05    /*!< ENET PHY 10M full duplex.*/
#define KSZ8081RNA_CTRL_REG_100M_FULL_DUPLEX_MASK  0x06    /*!< ENET PHY 100M full duplex.*/

// PHY Status Register Masks
#define KSZ8081RNA_STATUS_REG_EXTENDED_CAP_MASK       0x0001 /*!< ENET PHY link status bit.*/
#define KSZ8081RNA_STATUS_REG_JABBER_MASK             0x0002 /*!< ENET PHY extended capability.*/
#define KSZ8081RNA_STATUS_REG_LINK_STATUS_MASK        0x0004 /*!< ENET PHY link status bit.*/
#define KSZ8081RNA_STATUS_REG_AUTO_NEG_ABLE_MASK      0x0008 /*!< ENET PHY auto neg capable.*/
#define KSZ8081RNA_STATUS_REG_REMOTE_FAULT_MASK       0x0010 /*!< ENET PHY remote fault bit.*/
#define KSZ8081RNA_STATUS_REG_AUTO_NEG_COMPLETE_MASK  0x0020 /*!< ENET PHY auto negotiation complete.*/
#define KSZ8081RNA_STATUS_REG_PREAMBLE_MASK           0x0040 /*!< ENET PHY preamble suppression.*/
#define KSZ8081RNA_STATUS_REG_10BASE_T_HD             0x0800 /*!< ENET PHY 10 base t hd.*/
#define KSZ8081RNA_STATUS_REG_10BASE_T_FD             0x1000 /*!< ENET PHY 10 base t fd.*/
#define KSZ8081RNA_STATUS_REG_100BASE_TX_HD           0x2000 /*!< ENET PHY 100 base tx hd.*/
#define KSZ8081RNA_STATUS_REG_100BASE_TX_FD           0x4000 /*!< ENET PHY 100 base tx fd.*/
#define KSZ8081RNA_STATUS_REG_100BASE_T4              0x8000 /*!< ENET PHY 100 base t4.*/

#define  KSZ8081RNA_REGISTER_ANLPA_REM_FLT			  0x2000
#define  KSZ8081RNA_REGISTER_ANLPA_ACK_REC			  0x4000

#define  KSZ8081RNA_REGISTER_ANE_REM_AUTO			  0x0001

#define  KSZ8081RNA_REGISTER_CT1_LINK_STAT			  0x0100 /* link status */
#define  KSZ8081RNA_REGISTER_CT1_CABLE_CROSS		  0x0020 /* cable type */
#define  KSZ8081RNA_REGISTER_CT1_CABLE_PLUGGED		  0x0010 /* energy on cable */
#define  KSZ8081RNA_REGISTER_CT1_SPEED				  0x0007 /* speed bits */
#define	 KSZ8081RNA_REGISTER_CT2_DIS_PAIR_SWP		  0x2000
#define  KSZ8081RNA_REGISTER_CT2_MDI_STRAIGHT		  0x4000
#define  KSZ8081RNA_REGISTER_CT2_FORCE_LINK			  0x0800

#define KSZ8081RNA_REGISTER_EXCTRL_EDPD				  0x0800

// PHY Constants
#define KSZ8081RNA_MAX_ADDRESS     0x1FU /*!< The maximum value for PHY device address*/

#undef DISABLE_PAIR_SWAP

Ethernet_DeviceHandle 	lCopyDev = 0;
uint32_t 				KSZ8081Reg[32];
bool					auto_neg_set = 0; /* enable auto_neg */
bool					speed_hi	 = 1; /* 1->HI  0->LOW */
bool					duplex		 = 1; /* 1->DUP 0->HALF */
static	uint32_t		noLinkCount = 0;

void KSZ8081RNA_init(Ethernet_DeviceHandle dev)
{
    uint32_t 	phyInstruction, phyRegValue;
    uint32_t 	counter = 0;
    uint32_t 	cond1,cond2;

    lCopyDev = dev;

    phyInstruction = KSZ8081RNA_CTRL_REG_RESET_MASK;

    // Reset Phy
    Ethernet_smiOperation(dev,
                          0,
                          KSZ8081RNA_REGISTER_CTRL,
                          ETHERNET_SMI_WRITE_FRAME_NOT_COMPLIANT,
                          &phyInstruction);

    do
    {
        counter++;
        /* Wait for complete */
        Ethernet_smiOperation(dev,
                              0,
                              KSZ8081RNA_REGISTER_CTRL,
                              ETHERNET_SMI_READ_FRAME_COMPLIANT,
                              &phyRegValue);
    } while((phyRegValue & KSZ8081RNA_CTRL_REG_RESET_MASK) &&
    		(counter < KSZ8081RNA_MAX_TIMEOUT));
/*
 *
 * **/

    phyInstruction = KSZ8081RNA_REGISTER_EXCTRL_EDPD;
    // Disable power mode
    Ethernet_smiOperation(dev,
                          0,
						  KSZ8081RNA_REGISTER_EXCTRL,
                          ETHERNET_SMI_WRITE_FRAME_NOT_COMPLIANT,
                          &phyInstruction);

    do
    {
        counter++;
        /* Wait for complete */
        Ethernet_smiOperation(dev,
                              0,
							  KSZ8081RNA_REGISTER_EXCTRL,
                              ETHERNET_SMI_READ_FRAME_COMPLIANT,
                              &phyRegValue);
    } while((phyRegValue & KSZ8081RNA_REGISTER_EXCTRL_EDPD) &&
    		(counter < KSZ8081RNA_MAX_TIMEOUT));

/*
 *
 * **/


    // Check for timeout
    if (counter < KSZ8081RNA_MAX_TIMEOUT)
    { /* Reset OK */

		Ethernet_smiOperation(dev,
							  0,
							  KSZ8081RNA_REGISTER_STATUS,
							  ETHERNET_SMI_READ_FRAME_COMPLIANT,
							  &phyRegValue);

		cond1 = phyRegValue & KSZ8081RNA_STATUS_REG_AUTO_NEG_ABLE_MASK;
		cond2 = phyRegValue & KSZ8081RNA_STATUS_REG_AUTO_NEG_COMPLETE_MASK;
		if (cond1 != 0 && cond2 == 0 && auto_neg_set)
		{
				phyInstruction = KSZ8081RNA_CTRL_REG_AUTO_NEG_MASK;
				Ethernet_smiOperation(dev,
									  0,
									  KSZ8081RNA_REGISTER_CTRL,
									  ETHERNET_SMI_WRITE_FRAME_COMPLIANT,
									  &phyInstruction);

				for (counter = 0; counter < KSZ8081RNA_MAX_TIMEOUT; ++counter)
				{
					Ethernet_smiOperation(dev,
										  0,
										  KSZ8081RNA_REGISTER_STATUS,
										  ETHERNET_SMI_READ_FRAME_COMPLIANT,
										  &phyRegValue);

					if ((phyRegValue & KSZ8081RNA_STATUS_REG_AUTO_NEG_COMPLETE_MASK) != 0)
						break;
				}
#ifdef DISABLE_PAIR_SWAP
			/* Disable Pair Swap */
			Ethernet_smiOperation(dev,
								  0,
								  KSZ8081RNA_REGISTER_CT2,
								  ETHERNET_SMI_READ_FRAME_COMPLIANT,
								  &phyRegValue);

			phyInstruction = phyRegValue & (~KSZ8081RNA_REGISTER_CT2_MDI_STRAIGHT);
			phyInstruction = phyInstruction | KSZ8081RNA_REGISTER_CT2_DIS_PAIR_SWP;
			Ethernet_smiOperation(dev,
								  0,
								  KSZ8081RNA_REGISTER_CT2,
								  ETHERNET_SMI_WRITE_FRAME_NOT_COMPLIANT,
								  &phyInstruction);
#endif
		} else {
			/* No Auto neg able*/
			phyInstruction = 0;
			if(speed_hi) {phyInstruction |=	KSZ8081RNA_CTRL_REG_PHY_HI_SPEED_MASK;}
			if(duplex)   {phyInstruction |=	KSZ8081RNA_CTRL_REG_DUPLEX;}

			Ethernet_smiOperation(dev,
								  0,
								  KSZ8081RNA_REGISTER_CTRL,
								  ETHERNET_SMI_WRITE_FRAME_COMPLIANT,
								  &phyInstruction);
#ifdef DISABLE_PAIR_SWAP
			/* Disable Pair Swap */
			Ethernet_smiOperation(dev,
								  0,
								  KSZ8081RNA_REGISTER_CT2,
								  ETHERNET_SMI_READ_FRAME_COMPLIANT,
								  &phyRegValue);

			phyInstruction = phyRegValue & (~KSZ8081RNA_REGISTER_CT2_MDI_STRAIGHT);
			phyInstruction = phyInstruction | KSZ8081RNA_REGISTER_CT2_DIS_PAIR_SWP;
			Ethernet_smiOperation(dev,
								  0,
								  KSZ8081RNA_REGISTER_CT2,
								  ETHERNET_SMI_WRITE_FRAME_NOT_COMPLIANT,
								  &phyInstruction);
#endif
		}

    }
    for(counter = 0; counter < 32; counter++)
    {
       Ethernet_smiOperation(dev,
                             0,
                             counter,
                             ETHERNET_SMI_READ_FRAME_COMPLIANT,
                             &KSZ8081Reg[counter]);
    }
}
void KSZ8081RNA_reneg(Ethernet_DeviceHandle dev)
{
    uint32_t 	phyInstruction, phyRegValue;
    uint32_t 	counter = 0;
    uint32_t 	cond;

    lCopyDev = dev;
    phyInstruction = KSZ8081RNA_CTRL_REG_RESTART_AUTO_NEG;

    Ethernet_smiOperation(dev,
                          0,
                          KSZ8081RNA_REGISTER_CTRL,
                          ETHERNET_SMI_WRITE_FRAME_NOT_COMPLIANT,
                          &phyInstruction);
	for (counter = 0; counter < KSZ8081RNA_MAX_TIMEOUT; ++counter)
	{
		Ethernet_smiOperation(dev,
							  0,
							  KSZ8081RNA_REGISTER_STATUS,
							  ETHERNET_SMI_READ_FRAME_COMPLIANT,
							  &phyRegValue);

		cond = phyRegValue & KSZ8081RNA_STATUS_REG_AUTO_NEG_COMPLETE_MASK;
		if (cond != 0)
			break;
	}
}

void KSZ8081RNA_status(ptrKSZ8081_stat S, struct netif * N)
{
    uint32_t 	phyReg01,
				phyReg05,
				phyReg06,
				phyReg15,
				phyReg1E,
				phyReg1F;
    bool		cableenergy;

    S->remoteAuto = 0;
    S->autoNegEnd = 0;
    S->acknoledge = 0;
    S->remoteFault = 0;
	S->cableX = 0;
	S->linkStatus = 0;
	S->speed = 0;
	S->RXErrors = 0;
	if(!lCopyDev) return;

	Ethernet_smiOperation(lCopyDev,
    					  0,
						  KSZ8081RNA_REGISTER_STATUS,
                          ETHERNET_SMI_READ_FRAME_COMPLIANT,
                          &phyReg01);

	if(phyReg01 & KSZ8081RNA_STATUS_REG_AUTO_NEG_COMPLETE_MASK)
	{
		S->autoNegEnd = 1;
	}
	Ethernet_smiOperation(lCopyDev,
    					  0,
						  KSZ8081RNA_REGISTER_ANLPA,
                          ETHERNET_SMI_READ_FRAME_COMPLIANT,
                          &phyReg05);
	if(phyReg05 & KSZ8081RNA_REGISTER_ANLPA_ACK_REC)
	{
		S->acknoledge = 1;
	}
	if(phyReg05 & KSZ8081RNA_REGISTER_ANLPA_REM_FLT)
	{
		S->remoteFault = 1;
	}

	Ethernet_smiOperation(lCopyDev,
    					  0,
						  KSZ8081RNA_REGISTER_ANE,
                          ETHERNET_SMI_READ_FRAME_COMPLIANT,
                          &phyReg06);

	if(phyReg06 & KSZ8081RNA_REGISTER_ANE_REM_AUTO)
	{
		S->remoteAuto = 1;
	}

	Ethernet_smiOperation(lCopyDev,
    					  0,
						  KSZ8081RNA_REGISTER_RXERR,
                          ETHERNET_SMI_READ_FRAME_COMPLIANT,
                          &phyReg15);
	S->RXErrors = phyReg15;

	Ethernet_smiOperation(lCopyDev,
    					  0,
    		   	   	   	  KSZ8081RNA_REGISTER_CT2,
                          ETHERNET_SMI_READ_FRAME_COMPLIANT,
                          &phyReg1F);

	Ethernet_smiOperation(lCopyDev,
    					  0,
    		   	   	   	  KSZ8081RNA_REGISTER_CT1,
                          ETHERNET_SMI_READ_FRAME_COMPLIANT,
                          &phyReg1E);

	if(phyReg1E & KSZ8081RNA_REGISTER_CT1_CABLE_PLUGGED)
	{
		cableenergy = 1 ;
	} else {
		cableenergy = 0 ;
	}
	if(phyReg1E & KSZ8081RNA_REGISTER_CT1_CABLE_CROSS)	{ S->cableX = 1;}
	if(phyReg1E & KSZ8081RNA_REGISTER_CT1_LINK_STAT) 	{ S->linkStatus = 1;}
	S->speed = (uint8_t)(phyReg1E & KSZ8081RNA_REGISTER_CT1_SPEED);

	if(S->linkStatus)
	{
		netif_set_link_up(N);
	} else {
		netif_set_link_down(N);
	}

	if(cableenergy == 1 && S->oldcablePlug == 0)
	{
		S->cablePlugged = 1;
		S->cableUnplugged = 0;
	}

	if(cableenergy == 0 && S->oldcablePlug == 1)
	{
		S->cablePlugged = 0;
		S->cableUnplugged = 1;
	}

	if(S->linkStatus == 0 && S->cablePlugged == 1)
	{
		noLinkCount++;
		if(noLinkCount > 10)
		{
			noLinkCount = 0;
			if(S->remoteAuto && S->autoNegEnd){
				S->renegCount++;
				KSZ8081RNA_reneg(lCopyDev);
			}
		}
	}

	S->oldcablePlug = cableenergy;

	return;
}
