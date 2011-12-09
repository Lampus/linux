#ifndef __LINUX_SPI_TLV_H
#define __LINUX_SPI_TLV_H

struct tlv320aic23b_board_info {
	int		ssc_id;
	struct clk	*dac_clk;
	char		shortname[32];
};

#endif
