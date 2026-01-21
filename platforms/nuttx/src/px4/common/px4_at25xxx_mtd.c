/************************************************************************************
 * Driver for AT25xxxx-style SPI EEPROMs.
 *
 * Driver for SPI-based at25xxx EEPROM
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_mtd.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

#include <board_config.h>

// #define PX4_AT25XXX_MTD_DEBUG   /* for debug */

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_AT25XXX_SPIMODE
#  define CONFIG_AT25XXX_SPIMODE 0
#endif

/* Instructions:
 *      Command          Value       N Description         Addr Dummy Data
 */

#define AT25XXX_WREN      0x06    /* 1 Write Enable          0   0     0 */
#define AT25XXX_WRDI      0x04    /* 1 Write Disable         0   0     0 */
#define AT25XXX_RDSR      0x05    /* 1 Read Status Register  0   0     >=1 */
#define AT25XXX_WRSR      0x01    /* 1 Write Status Register 0   0     1 */
#define AT25XXX_READ      0x03    /* 1 Read Data Bytes       A   0     >=1 */
#define AT25XXX_WRITE     0x02    /* 1 Write                 A   0     1-256 */

/* SR bits definitions */

#define AT25XXX_SR_WIP  0x01 /* Write in Progress */
#define AT25XXX_SR_WEL  0x02 /* Write Enable Latch */
#define AT25XXX_SR_BP0  0x04 /* First Block Protect bit */
#define AT25XXX_SR_BP1  0x08 /* Second Block Protect bit */
#define AT25XXX_SR_WPEN 0x80 /* Write Protect Enable */

#define AT25XXX_DUMMY   0xFF


#define AT25XXX_INIT_CLK_MAX    10000000UL

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct at25xxx_dev_s.
 */

struct at25xxx_dev_s {
	struct mtd_dev_s      mtd;      /* MTD interface */
	FAR struct spi_dev_s *dev;      /* Saved SPI interface instance */
	uint32_t              size;     /* in bytes, expanded from geometry */
	uint16_t              pgsize;   /* write block size, in bytes, expanded from geometry */
	uint16_t              addrlen;  /* number of BITS in data addresses */
	uint32_t              speed;    /* Overridable via ioctl */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static void at25xxx_lock(FAR struct at25xxx_dev_s *priv);
static inline void at25xxx_unlock(FAR struct at25xxx_dev_s *priv);
static void at25xxx_sendcmd(FAR struct at25xxx_dev_s *priv, uint8_t cmd,
							uint8_t addrlen, uint32_t addr);
static inline void at25xxx_waitwritecomplete(struct at25xxx_dev_s *priv);
static inline void at25xxx_writeenable(struct at25xxx_dev_s *priv);
static inline int at25xxx_pagewrite(struct at25xxx_dev_s *priv, FAR const uint8_t *buffer,
									off_t page, size_t pagesize);

/* MTD driver methods */

static int at25xxx_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t at25xxx_bread(FAR struct mtd_dev_s *dev, off_t startblock,
							 size_t nblocks, FAR uint8_t *buf);
static ssize_t at25xxx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
							  size_t nblocks, FAR const uint8_t *buf);
static ssize_t at25xxx_read(FAR struct mtd_dev_s *dev, off_t offset, 
							size_t nbytes, FAR uint8_t *buffer);
static int at25xxx_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);


#ifdef PX4_AT25XXX_MTD_DEBUG
/* for debug */
static void debug_at25xxx_test(void);
#endif /* of #ifdef PX4_AT25XXX_MTD_DEBUG */

/************************************************************************************
 * Private Data
 ************************************************************************************/

static uint8_t number_of_instances = 0u;
static struct at25xxx_dev_s g_at25xxx[BOARD_MTD_NUM_EEPROM];

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/****************************************************************************
 * Name: at25xxx_lock
 ****************************************************************************/

static void at25xxx_lock(FAR struct at25xxx_dev_s *priv)
{
	/* On SPI buses where there are multiple devices, it will be necessary to
	 * lock SPI to have exclusive access to the buses for a sequence of
	 * transfers.  The bus should be locked before the chip is selected.
	 *
	 * This is a blocking call and will not return until we have exclusive
	 * access to the SPI bus.  We will retain that exclusive access until the
	 * bus is unlocked.
	 */

	SPI_LOCK(priv->dev, true);

	/* After locking the SPI bus, the we also need call the setfrequency,
	 * setbits, and setmode methods to make sure that the SPI is properly
	 * configured for the device.  If the SPI bus is being shared, then it may
	 * have been left in an incompatible state.
	 */

	SPI_SETMODE(priv->dev, CONFIG_AT25XXX_SPIMODE);
	SPI_SETBITS(priv->dev, 8);
	SPI_HWFEATURES(priv->dev, 0);
	SPI_SETFREQUENCY(priv->dev, priv->speed);
}

/****************************************************************************
 * Name: at25xxx_unlock
 ****************************************************************************/

static inline void at25xxx_unlock(FAR struct at25xxx_dev_s *priv)
{
	SPI_LOCK(priv->dev, false);
}

/****************************************************************************
 * Name: at25xxx_sendcmd
 *
 * Description: Send command and address as one transaction to take advantage
 * of possible faster DMA transfers. Sending byte per byte is FAR FAR slower.
 *
 ****************************************************************************/

static void at25xxx_sendcmd(FAR struct at25xxx_dev_s *priv, uint8_t cmd,
							uint8_t addrlen, uint32_t addr)
{
	uint8_t buf[4];
	int     cmdlen = 1;

	/* Store command */

	buf[0] = cmd;

	/* Store address according to its length */

	if (addrlen == 9) {
		buf[0] |= (((addr >> 8) & 1) << 3);
	}

	if (addrlen > 16) {
		buf[cmdlen++] = (addr >> 16) & 0xff;
	}

	if (addrlen > 9) {
		buf[cmdlen++] = (addr >>  8) & 0xff;
	}

	buf[cmdlen++] = addr & 0xff;

	SPI_SNDBLOCK(priv->dev, buf, cmdlen);
}

/****************************************************************************
 * Name: at25xxx_waitwritecomplete
 *
 * Description: loop until the write operation is done.
 *
 ****************************************************************************/

static inline void at25xxx_waitwritecomplete(struct at25xxx_dev_s *priv)
{
	uint8_t status;

	/* Loop as long as the memory is busy with a write cycle */

	do {
		/* Select this FLASH part */

		SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), true);

		/* Send "Read Status Register (RDSR)" command */

		SPI_SEND(priv->dev, AT25XXX_RDSR);

		/* Send a dummy byte to generate the clock needed to shift out the
		 * status
		 */

		status = SPI_SEND(priv->dev, AT25XXX_DUMMY);

		/* Deselect the FLASH */

		SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), false);

		/* Given that writing could take up to a few milliseconds,
		 * the following short delay in the "busy" case will allow
		 * other peripherals to access the SPI bus.
		 */

		if ((status & AT25XXX_SR_WIP) != 0) {
			at25xxx_unlock(priv);
			px4_usleep(1000);
			at25xxx_lock(priv);
		}
	}
	while ((status & AT25XXX_SR_WIP) != 0);
}

/****************************************************************************
 * Name:  at25xxx_writeenable
 ****************************************************************************/

static inline void at25xxx_writeenable(struct at25xxx_dev_s *priv)
{
	/* Select this FLASH part */

	SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), true);

	/* Send "Write Enable (WREN)" command */

	SPI_SEND(priv->dev, AT25XXX_WREN);

	/* Deselect the FLASH */

	SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), false);
	finfo("Enabled\n");
}

/****************************************************************************
 * Name:  at25xxx_pagewrite
 ****************************************************************************/

static inline int at25xxx_pagewrite(struct at25xxx_dev_s *priv, FAR const uint8_t *buffer,
									off_t page, size_t pagesize)
{
	off_t offset = page * pagesize;

	finfo("page: %08lx offset: %08lx\n", (long)page, (long)offset);

	/* Enable the write access to the FLASH */

	at25xxx_writeenable(priv);

	/* Select this FLASH part */

	SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), true);

	/* Send "Write from Memory " instruction */

	at25xxx_sendcmd(priv, AT25XXX_WRITE, priv->addrlen, offset);

	/* Then write the specified number of bytes */

	SPI_SNDBLOCK(priv->dev, buffer, pagesize);

	/* Deselect the FLASH: Chip Select high */

	SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), false);

	/* Wait write complete */

	at25xxx_waitwritecomplete(priv);

	finfo("Written\n");

	return OK;
}

/************************************************************************************
 * Name: at25xxx_erase
 ************************************************************************************/

static int at25xxx_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
	finfo("startblock: %08lx nblocks: %d\n", (unsigned long)startblock, (int)nblocks);
	finfo("On RAMTRON devices erasing makes no sense, returning as OK\n");

	return (int)nblocks;
}

/************************************************************************************
 * Name: at25xxx_bread
 ************************************************************************************/

static ssize_t at25xxx_bread(FAR struct mtd_dev_s *dev, off_t startblock,
							 size_t nblocks, FAR uint8_t *buffer)
{
	FAR struct at25xxx_dev_s *priv = (FAR struct at25xxx_dev_s *)dev;
	ssize_t nbytes;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

	/* On this device, we can handle the block read just like the byte-oriented
	 * read
	 */

	nbytes = at25xxx_read(dev, startblock *  priv->pgsize,
						  nblocks *  priv->pgsize, buffer);

	if (nbytes > 0) {
		return nbytes / priv->pgsize;
	}

	return (int)nbytes;
}

/************************************************************************************
 * Name: at25xxx_bwrite
 ************************************************************************************/

static ssize_t at25xxx_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
							  size_t nblocks, FAR const uint8_t *buffer)
{
	FAR struct at25xxx_dev_s *priv = (FAR struct at25xxx_dev_s *)dev;

	size_t blocksleft = nblocks;

	finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
	DEBUGASSERT(priv != NULL && buffer != NULL);

	/* Lock the SPI bus and write each page to FLASH */

	at25xxx_lock(priv);

	while (blocksleft-- > 0) {
		if (at25xxx_pagewrite(priv, buffer, startblock, priv->pgsize)) {
			nblocks = 0;
			break;
		}

		startblock++;
	}

	at25xxx_unlock(priv);

	return nblocks;
}

/****************************************************************************
 * Name: at25xxx_read
 ****************************************************************************/

static ssize_t at25xxx_read(FAR struct mtd_dev_s *dev, off_t offset,
							size_t nbytes, FAR uint8_t *buffer)
{
	FAR struct at25xxx_dev_s *priv = (FAR struct at25xxx_dev_s *)dev;

	finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

	/* Lock the SPI bus NOW because the ramtron_waitwritecomplete call must be
	 * executed with the bus locked.
	 */

	at25xxx_lock(priv);

	/* Select this FLASH part */

	SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), true);

	/* Send "Read from Memory " instruction */

	at25xxx_sendcmd(priv, AT25XXX_READ, priv->addrlen, offset);

	/* Then read all of the requested bytes */

	SPI_RECVBLOCK(priv->dev, buffer, nbytes);

	/* Deselect the FLASH and unlock the SPI bus */

	SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), false);

	at25xxx_unlock(priv);

	finfo("return nbytes: %d\n", (int)nbytes);

	return nbytes;
}

/************************************************************************************
 * Name: at25xxx_ioctl
 ************************************************************************************/

static int at25xxx_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
	FAR struct at25xxx_dev_s *priv = (FAR struct at25xxx_dev_s *)dev;
	int ret = -EINVAL; /* Assume good command with bad parameters */

	finfo("cmd: %d \n", cmd);

	switch (cmd) {
	case MTDIOC_GEOMETRY: {
			FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);

			if (geo) {
				/* Populate the geometry structure with information need to know
				 * the capacity and how to access the device.
				 *
				 * NOTE: that the device is treated as though it where just an array
				 * of fixed size blocks.  That is most likely not true, but the client
				 * will expect the device logic to do whatever is necessary to make it
				 * appear so.
				 */

				geo->blocksize    = priv->pgsize;
				geo->erasesize    = priv->pgsize;
				geo->neraseblocks = priv->size / priv->pgsize;
				ret               = OK;

				finfo("blocksize: %" PRId32 " erasesize: %" PRId32 " neraseblocks: %" PRId32 "\n",
				      geo->blocksize, geo->erasesize, geo->neraseblocks);
			}
		}
		break;

	case MTDIOC_BULKERASE:
		finfo("BULDERASE: Makes no sense in at25xxx.\n");
		finfo("BULDERASE: Let's confirm operation as OK\n");
		ret = OK;
		break;

	case MTDIOC_SETSPEED:
		if (arg > 0 && arg <= AT25XXX_INIT_CLK_MAX) {
			priv->speed = arg;
			finfo("set bus speed to %lu\n", priv->speed);
			ret = OK;
		}
		break;

	default:
		ret = -ENOTTY; /* Bad command */
		break;
	}

	finfo("return %d\n", ret);

	return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: at25xxx_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/
int px4_at25xxx_initialize(FAR struct spi_dev_s *dev, FAR struct mtd_dev_s **mtd_dev)
{
	if (number_of_instances >= BOARD_MTD_NUM_EEPROM) {
		return -ENOMEM;
	}

	FAR struct at25xxx_dev_s *priv;

	finfo("dev: %p, mtd_dev %p\n", dev, mtd_dev);

	/* Allocate a state structure (we allocate the structure instead of using
	 * a fixed, static allocation so that we can handle multiple FLASH devices.
	 * The current implementation would handle only one FLASH part per SPI
	 * device (only because of the SPIDEV_FLASH(0) definition) and so would
	 * have to be extended to handle multiple FLASH parts on the same SPI bus.
	 */

	priv = &g_at25xxx[number_of_instances];

	if (priv) {
		/* Initialize the allocated structure */

		priv->mtd.erase  = at25xxx_erase;
		priv->mtd.bread  = at25xxx_bread;
		priv->mtd.bwrite = at25xxx_bwrite;
		priv->mtd.read   = at25xxx_read;
		priv->mtd.ioctl  = at25xxx_ioctl;
		priv->mtd.name   = "at25xxx";
		priv->dev        = dev;

		priv->size       = 64L * 1024L;
		priv->pgsize     = 128;
		priv->addrlen    = 16;
		priv->speed      = 10000000;

		/* Deselect the FLASH */

		SPI_SELECT(dev, SPIDEV_EEPROM(0), false);

		/* attempt to read to validate device is present */

		at25xxx_lock(priv);

		/* Send "Write Enable (WREN)" command */

		SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), true);

		SPI_SEND(priv->dev, AT25XXX_WREN);

		SPI_SELECT(dev, SPIDEV_EEPROM(0), false);

		/* Send "Read Status Register (RDSR)" command */

		SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), true);

		SPI_SEND(priv->dev, AT25XXX_RDSR);

		/* Send a dummy byte to generate the clock needed to shift out the
		 * status
		 */

		uint8_t status = SPI_SEND(priv->dev, AT25XXX_DUMMY);

		SPI_SELECT(dev, SPIDEV_EEPROM(0), false);

		if ((status & AT25XXX_SR_WEL) == 0x00) {
			at25xxx_unlock(priv);
			return -ENODEV;
		}

		/* Send "Write Disable (WRDI)" command */

		SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), true);

		SPI_SEND(priv->dev, AT25XXX_WRDI);

		SPI_SELECT(dev, SPIDEV_EEPROM(0), false);

		/* Send "Read Status Register (RDSR)" command */

		SPI_SELECT(priv->dev, SPIDEV_EEPROM(0), true);

		SPI_SEND(priv->dev, AT25XXX_RDSR);

		/* Send a dummy byte to generate the clock needed to shift out the
		 * status
		 */

		status = SPI_SEND(priv->dev, AT25XXX_DUMMY);

		SPI_SELECT(dev, SPIDEV_EEPROM(0), false);

		if ((status & AT25XXX_SR_WEL) != 0x00) {
			at25xxx_unlock(priv);
			return -ENODEV;
		}

		at25xxx_unlock(priv);
	}

	else {
		return -ENOMEM;
	}

	*mtd_dev = (FAR struct mtd_dev_s *)priv;
	++number_of_instances;

#ifdef PX4_AT25XXX_MTD_DEBUG
	debug_at25xxx_test();
#endif /* of #ifdef PX4_AT25XXX_MTD_DEBUG */

	return OK;
}

#ifdef PX4_AT25XXX_MTD_DEBUG
/************************************************************************************
 * Name: debug_at25xxx_test
 ************************************************************************************/

static void debug_at25xxx_test(void)
{
	uint8_t buf[128];
	unsigned count = 0;
	unsigned errors = 0;

	for (count = 0; count < 10; count++) {

		memset(buf, count, sizeof(buf));
		ssize_t result = at25xxx_bwrite(&g_at25xxx[0].mtd, 1, 1, buf);

		if (result == ERROR) {
			if (errors++ > 2) {
				syslog(LOG_INFO, "too many errors\n");
				return;
			}

		} else if (result != 1) {
			syslog(LOG_INFO, "unexpected %zu\n", result);
		}

		memset(buf, 0x55, sizeof(buf));
		result = at25xxx_bread(&g_at25xxx[0].mtd, 1, 1, buf);

		if (result == ERROR) {
			if (errors++ > 2) {
				syslog(LOG_INFO, "too many errors\n");
				return;
			}

		} else if (result != 1) {
			syslog(LOG_INFO, "unexpected %zu\n", result);

		} else {
			syslog(LOG_INFO, "Read Test 0x%x \n", buf[0]);
		}
	}

	syslog(LOG_INFO, "test %u errors %u\n", count, errors);
}

#endif /* of #ifdef PX4_AT25XXX_MTD_DEBUG */

