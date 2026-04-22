/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

#pragma once

#if defined(__PX4_FREERTOS)


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Userspace SPI device interface compatible with Linux spidev
 */

#define SPI_CPHA	0x01
#define SPI_CPOL	0x02
#define SPI_MODE_0	(0 | 0)
#define SPI_MODE_1	(0 | SPI_CPHA)
#define SPI_MODE_2	(SPI_CPOL | 0)
#define SPI_MODE_3	(SPI_CPOL | SPI_CPHA)
#define SPI_CS_HIGH	0x04
#define SPI_LSB_FIRST	0x08
#define SPI_3WIRE	0x10
#define SPI_LOOP	0x20
#define SPI_NO_CS	0x40
#define SPI_READY	0x80
#define SPI_TX_DUAL	0x100
#define SPI_TX_QUAD	0x200
#define SPI_RX_DUAL	0x400
#define SPI_RX_QUAD	0x800

#define SPI_IOC_MAGIC	'k'

struct spi_ioc_transfer {
	uint64_t tx_buf;
	uint64_t rx_buf;
	uint32_t len;
	uint32_t speed_hz;
	uint16_t delay_usecs;
	uint8_t bits_per_word;
	uint8_t cs_change;
	uint8_t tx_nbits;
	uint8_t rx_nbits;
	uint16_t pad;
};

/* ioctl command encoding compatible with Linux _IOC macros */
#ifndef _IOC_NRBITS
#define _IOC_NRBITS	8
#endif
#ifndef _IOC_TYPEBITS
#define _IOC_TYPEBITS	8
#endif
#ifndef _IOC_SIZEBITS
#define _IOC_SIZEBITS	14
#endif
#ifndef _IOC_DIRBITS
#define _IOC_DIRBITS	2
#endif

#ifndef _IOC_NRMASK
#define _IOC_NRMASK	((1 << _IOC_NRBITS) - 1)
#endif
#ifndef _IOC_TYPEMASK
#define _IOC_TYPEMASK	((1 << _IOC_TYPEBITS) - 1)
#endif
#ifndef _IOC_SIZEMASK
#define _IOC_SIZEMASK	((1 << _IOC_SIZEBITS) - 1)
#endif
#ifndef _IOC_DIRMASK
#define _IOC_DIRMASK	((1 << _IOC_DIRBITS) - 1)
#endif

#ifndef _IOC_NRSHIFT
#define _IOC_NRSHIFT	0
#endif
#ifndef _IOC_TYPESHIFT
#define _IOC_TYPESHIFT	(_IOC_NRSHIFT + _IOC_NRBITS)
#endif
#ifndef _IOC_SIZESHIFT
#define _IOC_SIZESHIFT	(_IOC_TYPESHIFT + _IOC_TYPEBITS)
#endif
#ifndef _IOC_DIRSHIFT
#define _IOC_DIRSHIFT	(_IOC_SIZESHIFT + _IOC_SIZEBITS)
#endif

#ifndef _IOC_NONE
#define _IOC_NONE	0U
#endif
#ifndef _IOC_WRITE
#define _IOC_WRITE	1U
#endif
#ifndef _IOC_READ
#define _IOC_READ	2U
#endif

#ifndef _IOC
#define _IOC(dir,type,nr,size)	(((dir) << _IOC_DIRSHIFT) | \
	((type) << _IOC_TYPESHIFT) | ((nr) << _IOC_NRSHIFT) | ((size) << _IOC_SIZESHIFT))
#endif

#ifndef _IOC_DIR
#define _IOC_DIR(cmd)	(((cmd) >> _IOC_DIRSHIFT) & _IOC_DIRMASK)
#endif
#ifndef _IOC_TYPE
#define _IOC_TYPE(cmd)	(((cmd) >> _IOC_TYPESHIFT) & _IOC_TYPEMASK)
#endif
#ifndef _IOC_NR
#define _IOC_NR(cmd)	(((cmd) >> _IOC_NRSHIFT) & _IOC_NRMASK)
#endif
#ifndef _IOC_SIZE
#define _IOC_SIZE(cmd)	(((cmd) >> _IOC_SIZESHIFT) & _IOC_SIZEMASK)
#endif

#ifndef _IO
#define _IO(type,nr)		_IOC(_IOC_NONE,(type),(nr),0)
#endif
#ifndef _IOR
#define _IOR(type,nr,size)	_IOC(_IOC_READ,(type),(nr),sizeof(size))
#endif
#ifndef _IOW
#define _IOW(type,nr,size)	_IOC(_IOC_WRITE,(type),(nr),sizeof(size))
#endif
#ifndef _IOWR
#define _IOWR(type,nr,size)	_IOC(_IOC_READ|_IOC_WRITE,(type),(nr),sizeof(size))
#endif

#define SPI_IOC_RD_MODE		_IOR(SPI_IOC_MAGIC, 1, uint8_t)
#define SPI_IOC_WR_MODE		_IOW(SPI_IOC_MAGIC, 1, uint8_t)
#define SPI_IOC_RD_BITS_PER_WORD	_IOR(SPI_IOC_MAGIC, 3, uint8_t)
#define SPI_IOC_WR_BITS_PER_WORD	_IOW(SPI_IOC_MAGIC, 3, uint8_t)
#define SPI_IOC_RD_MAX_SPEED_HZ	_IOR(SPI_IOC_MAGIC, 4, uint32_t)
#define SPI_IOC_WR_MAX_SPEED_HZ	_IOW(SPI_IOC_MAGIC, 4, uint32_t)

#define SPI_MSGSIZE(N)	(((N) * sizeof(struct spi_ioc_transfer) < (1 << _IOC_SIZEBITS)) ? \
	((N) * sizeof(struct spi_ioc_transfer)) : 0)

#define SPI_IOC_MESSAGE(N)	_IOW(SPI_IOC_MAGIC, 0, struct spi_ioc_transfer[(N)])

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
