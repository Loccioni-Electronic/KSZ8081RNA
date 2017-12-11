/******************************************************************************
 * KSZ8081RNA Library
 * Copyright (C) 2015-2017 AEA s.r.l. Loccioni Group - Elctronic Design Dept.
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@loccioni.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ******************************************************************************/

/**
 * @file KSZ8081RNA.c
 * @author Marco Giammarini <m.giammarini@loccioni.com>
 * @brief KSZ8081RNA implementation
 */

#include "KSZ8081RNA.h"

#define KSZ8081RNA_MAX_TIMEOUT 0xFFFF

// PHY Register Mnemonics
#define  KSZ8081RNA_REGISTER_CTRL    0     /*!< PHY control register.*/
#define  KSZ8081RNA_REGISTER_STATUS  1     /*!< PHY status register.*/
#define  KSZ8081RNA_REGISTER_ID1     2     /*!< PHY identification register 1.*/
#define  KSZ8081RNA_REGISTER_ID2     3     /*!< PHY identification register 2.*/
#define  KSZ8081RNA_REGISTER_CT1     0x1E  /*!< PHY control1 register.*/
#define  KSZ8081RNA_REGISTER_CT2     0x1F  /*!< PHY control2 register.*/

// PHY Control Register Masks
#define KSZ8081RNA_CTRL_REG_AUTO_NEG_MASK          0x1000  /*!< ENET PHY auto negotiation control.*/
#define KSZ8081RNA_CTRL_REG_PHY_SPEED_MASK         0x2000  /*!< ENET PHY speed control.*/
#define KSZ8081RNA_CTRL_REG_LOOP_MASK              0x4000  /*!< ENET PHY loop control.*/
#define KSZ8081RNA_CTRL_REG_RESET_MASK             0x8000  /*!< ENET PHY reset control.*/
#define KSZ8081RNA_CTRL_REG_10M_HALF_DUPLEX_MASK   0x01    /*!< ENET PHY 10M half duplex.*/
#define KSZ8081RNA_CTRL_REG_100M_HALF_DUPLEX_MASK  0x02    /*!< ENET PHY 100M half duplex.*/
#define KSZ8081RNA_CTRL_REG_10M_FULL_DUPLEX_MASK   0x05    /*!< ENET PHY 10M full duplex.*/
#define KSZ8081RNA_CTRL_REG_100M_FULL_DUPLEX_MASK  0x06    /*!< ENET PHY 100M full duplex.*/

// PHY Status Register Masks
#define KSZ8081RNA_STATUS_REG_LINK_STATUS_MASK        0x04 /*!< ENET PHY link status bit.*/
#define KSZ8081RNA_STATUS_REG_AUTO_NEG_ABLE_MASK      0x08 /*!< ENET PHY auto negotiation ability.*/
#define KSZ8081RNA_STATUS_REG_SPEED_DUPLEX_MASK       0x07 /*!< ENET PHY speed mask on status register 2.*/
#define KSZ8081RNA_STATUS_REG_AUTO_NEG_COMPLETE_MASK  0x20 /*!< ENET PHY auto negotiation complete.*/

// PHY Constants
#define KSZ8081RNA_MAX_ADDRESS     0x1FU /*!< The maximum value for PHY device address*/

void KSZ8081RNA_init(Ethernet_DeviceHandle dev)
{
    uint32_t phyInstruction, phyRegValue;
    uint32_t counter;

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
    } while((phyRegValue & 0x8000) && (counter < KSZ8081RNA_MAX_TIMEOUT));

    // Check for timeout
    if (counter >= KSZ8081RNA_MAX_TIMEOUT) return;

    Ethernet_smiOperation(dev,
                          0,
                          KSZ8081RNA_REGISTER_STATUS,
                          ETHERNET_SMI_READ_FRAME_COMPLIANT,
                          &phyRegValue);

    phyInstruction = KSZ8081RNA_CTRL_REG_AUTO_NEG_MASK;

    if (((phyRegValue & KSZ8081RNA_STATUS_REG_AUTO_NEG_ABLE_MASK) != 0) &&
        ((phyRegValue & KSZ8081RNA_STATUS_REG_AUTO_NEG_COMPLETE_MASK) == 0))
    {
        // Set Auto-negotiation
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
    }
}
