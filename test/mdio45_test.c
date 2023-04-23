#include <stdio.h>
#include <stdbool.h>

#define MDIO_PHY_ID_C45			0x8000
#define MDIO_PHY_ID_PRTAD		0x03e0
#define MDIO_PHY_ID_DEVAD		0x001f
#define MDIO_PHY_ID_C45_MASK						\
	(MDIO_PHY_ID_C45 | MDIO_PHY_ID_PRTAD | MDIO_PHY_ID_DEVAD)


static inline int mdio_phy_id_prtad(int phy_id)
{
	return (phy_id & MDIO_PHY_ID_PRTAD) >> 5;
}

static inline int mdio_phy_id_devad(int phy_id)
{
	return phy_id & MDIO_PHY_ID_DEVAD;
}

static inline bool mdio_phy_id_is_c45(int phy_id)
{
	return (phy_id & MDIO_PHY_ID_C45) && !(phy_id & ~MDIO_PHY_ID_C45_MASK);
}

int main(void)
{
	int phy_id = 0x8020;
	printf("phy_addr: %d\n", mdio_phy_id_prtad(phy_id));
	printf("regnum_xxx: %d\n", mdio_phy_id_devad(phy_id));
	printf("is_c45: %d\n", mdio_phy_id_is_c45(phy_id));
	return 0;
}
