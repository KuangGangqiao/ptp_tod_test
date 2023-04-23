#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/sockios.h>
#include <linux/mdio.h>

#ifndef __GLIBC__
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#endif
#include "mii.h"
#include "mdio-tool.h"

//you can find kernel/driver/net/phy/phy.c : phy_mii_ioctl
#define MII_ADDR_C45	(1 << 30)
#define PHY_IS_C45	0x8000
#define PHY_C45_MASK	0x03e0
#define PHY_C45_ADDR	1

static struct ifreq ifr;
static int skfd = -1;

static int mdio_read(int skfd, int location)
{
	struct mii_data *mii = (struct mii_data *)&ifr.ifr_data;
	mii->phy_id = PHY_IS_C45 | (PHY_C45_MASK & (PHY_C45_ADDR << 5));
	mii->reg_num = location;

	if (ioctl(skfd, SIOCGMIIREG, &ifr) < 0) {
		fprintf(stderr, "SIOCGMIIREG on %s failed: %s\n", ifr.ifr_name,
			strerror(errno));
		return -1;
	}
	return mii->val_out;
}

static int mdio_write(int skfd, int location, int value)
{
	struct mii_data *mii = (struct mii_data *)&ifr.ifr_data;
	mii->phy_id = PHY_IS_C45 | (PHY_C45_MASK & (PHY_C45_ADDR << 5));
	mii->reg_num = location;
	mii->val_in = value;

	if (ioctl(skfd, SIOCSMIIREG, &ifr) < 0) {
		fprintf(stderr, "SIOCSMIIREG on %s failed: %s\n", ifr.ifr_name,
		strerror(errno));
		return -1;
	}

	return 0;
}

static int mdio45_read(int skfd, int dev, int addr)
{
	int regnum = (MII_ADDR_C45 | (dev << 16) | (addr & 0xffff));
	return mdio_read(skfd, regnum);
}

static int mdio45_write(int skfd, int dev, int addr, int val)
{
	int regnum = (MII_ADDR_C45 | (dev << 16) | (addr & 0xffff));
	return mdio_write(skfd, regnum, val);
}

static void phy_write(int skfd, int argc, char **argv)
{
	int dev, addr, val;

	if (argc == 6) {
		dev = strtol(argv[3], NULL, 0);
		addr = strtol(argv[4], NULL, 0);
		val = strtol(argv[5], NULL, 0);
		mdio45_write(skfd, dev, addr, val);
	} else {
		printf("Usage: ./mdio-tool [w] [eth_device] [dev] [addr] [val]\n");
	}
}

static int phy_read(int skfd, int argc, char **argv)
{
	int retval, dev, addr;

	if (argc == 5) {
		dev = strtol(argv[3], NULL, 0);
		addr = strtol(argv[4], NULL, 0);
		retval = mdio45_read(skfd, dev, addr);
		printf("dev: %d addr: %d val: 0x%.4x\n", dev, addr, retval);
	} else {
		printf("Usage: ./mdio-tool [r] [eth_device] [dev] [addr]\n");
	}

	return retval;
}

int phy_c45_read(char *eth_name, int dev, int addr)
{
	int retval;

	/* Open a basic socket. */
	if ((skfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket");
		return -1;
	}
	/* Get the vitals from the interface. */
	strncpy(ifr.ifr_name, eth_name, IFNAMSIZ);
	if (ioctl(skfd, SIOCGMIIPHY, &ifr) < 0) {
		fprintf(stderr, "SIOCGMIIPHY on '%s' failed: %s\n",
			eth_name, strerror(errno));
		return -1;
	}
	retval = mdio45_read(skfd, dev, addr);
	if (retval < 0) {
		printf("Unexcept error for phy read!\n");
		return -1;
	}
	close(skfd);
	return retval;
}

int phy_c45_write(char *eth_name, int dev, int addr, int val)
{
	int retval;

	/* Open a basic socket. */
	if ((skfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket");
		return -1;
	}
	/* Get the vitals from the interface. */
	strncpy(ifr.ifr_name, eth_name, IFNAMSIZ);
	if (ioctl(skfd, SIOCGMIIPHY, &ifr) < 0) {
		fprintf(stderr, "SIOCGMIIPHY on '%s' failed: %s\n",
			eth_name, strerror(errno));
		return -1;
	}
	retval = mdio45_write(skfd, dev, addr, val);
	if (retval < 0) {
		printf("Unexcept error for phy read!\n");
		return -1;
	}
	close(skfd);
	return retval;
}

#if 0
int main(int argc, char **argv)
{
	int retval;

	if(6 < argc || argc < 2 || !argv[2]) {
		printf("Usage mii [r/w] [eth_device] [dev] [addr] [val]\n");
		return 0;
	}

	/* Open a basic socket. */
	if ((skfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket");
		return -1;
	}

	/* Get the vitals from the interface. */
	strncpy(ifr.ifr_name, argv[2], IFNAMSIZ);
	if (ioctl(skfd, SIOCGMIIPHY, &ifr) < 0) {
		fprintf(stderr, "SIOCGMIIPHY on '%s' failed: %s\n",
			argv[2], strerror(errno));
		return -1;
	}

	if(argv[1][0] == 'r') {
		retval = phy_read(skfd, argc, argv);
		if (retval < 0) {
			printf("Unexcept error for phy read!\n");
			return -1;
		}
	} else if(argv[1][0] == 'w') {
		phy_write(skfd, argc, argv);
	} else {
		printf("Usage mii [r/w] [eth_device] [dev] [addr] [val]\n");
	}

	close(skfd);
}
#endif
