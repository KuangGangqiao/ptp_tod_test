#ifndef _MDIO_TOOL_H
#define _MDIO_TOOL_H

int phy_c45_read(char *eth_name, int dev, int addr);
int phy_c45_write(char *eth_name, int dev, int addr, int val);

#endif
