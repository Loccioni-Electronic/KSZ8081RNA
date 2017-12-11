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
 * @file KSZ8081RNA.h
 * @author Marco Giammarini <m.giammarini@loccioni.com>
 * @brief KSZ8081RNA definitions
 */

#ifndef __LOCCIONI_KSZ8081RNA_H
#define __LOCCIONI_KSZ8081RNA_H

#define LOCCIONI_KSZ8081RNA_LIBRARY_NAME        "KSZ8081RNA Library"
#define LOCCIONI_KSZ8081RNA_LIBRARY_VERSION     "1.0"
#define LOCCIONI_KSZ8081RNA_LIBRARY_VERSION_M   1
#define LOCCIONI_KSZ8081RNA_LIBRARY_VERSION_m   0
#define LOCCIONI_KSZ8081RNA_LIBRARY_TIME        1512990749

#include "libohiboard.h"

/**
 *  Initializes the PHY device.
 *
 *  @param[in] dev The ENET device of libohiboard
 */
void KSZ8081RNA_init(Ethernet_DeviceHandle dev);


#endif /* __LOCCIONI_KSZ8081RNA_H */
