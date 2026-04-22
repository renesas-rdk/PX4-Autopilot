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

#define I2C_M_TEN	0x0010
#define I2C_M_RD	0x0001
#define I2C_M_STOP	0x8000
#define I2C_M_NOSTART	0x4000
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_IGNORE_NAK	0x1000
#define I2C_M_NO_RD_ACK	0x0800
#define I2C_M_RECV_LEN	0x0400

struct i2c_msg {
	uint16_t addr;
	uint16_t flags;
	uint16_t len;
	uint8_t *buf;
};

struct i2c_rdwr_ioctl_data {
	struct i2c_msg *msgs;
	uint32_t nmsgs;
};

#define I2C_IOC_MAGIC	'i'

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

#define I2C_RDWR		_IOWR(I2C_IOC_MAGIC, 0x07, struct i2c_rdwr_ioctl_data)

#ifdef __cplusplus
}

#endif /* __PX4_FREERTOS */

#endif
