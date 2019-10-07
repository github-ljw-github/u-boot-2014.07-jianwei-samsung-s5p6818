/*
 * (C) Copyright 2012 SAMSUNG Electronics
 * Jaehoon Chung <jh80.chung@samsung.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dwmmc.h>
#include <fdtdec.h>
#include <libfdt.h>
#include <malloc.h>
#include <asm/arch/dwmmc.h>
#include <asm/arch/clk.h>
#include <asm/arch/pinmux.h>
#include <asm/gpio.h>
#include <asm-generic/errno.h>

#define	DWMMC_MAX_CH_NUM		4
#define	DWMMC_MAX_FREQ			52000000
#define	DWMMC_MIN_FREQ			400000
#define	DWMMC_MMC0_CLKSEL_VAL		0x03030001
#define	DWMMC_MMC2_CLKSEL_VAL		0x03020001
#ifndef CONFIG_JIANWEI_S5P6818
/*
 * Function used as callback function to initialise the
 * CLKSEL register for every mmc channel.
 */
static void exynos_dwmci_clksel(struct dwmci_host *host)
{
	dwmci_writel(host, DWMCI_CLKSEL, host->clksel_val);
}

unsigned int exynos_dwmci_get_clk(struct dwmci_host *host)
{
	unsigned long sclk;
	int8_t clk_div;

	/*
	 * Since SDCLKIN is divided inside controller by the DIVRATIO
	 * value set in the CLKSEL register, we need to use the same output
	 * clock value to calculate the CLKDIV value.
	 * as per user manual:cclk_in = SDCLKIN / (DIVRATIO + 1)
	 */
	clk_div = ((dwmci_readl(host, DWMCI_CLKSEL) >> DWMCI_DIVRATIO_BIT)
			& DWMCI_DIVRATIO_MASK) + 1;
	sclk = get_mmc_clk(host->dev_index);

	/*
	 * Assume to know divider value.
	 * When clock unit is broken, need to set "host->div"
	 */
	return sclk / clk_div / (host->div + 1);
}

static void exynos_dwmci_board_init(struct dwmci_host *host)
{
	if (host->quirks & DWMCI_QUIRK_DISABLE_SMU) {
		dwmci_writel(host, EMMCP_MPSBEGIN0, 0);
		dwmci_writel(host, EMMCP_SEND0, 0);
		dwmci_writel(host, EMMCP_CTRL0,
			     MPSCTRL_SECURE_READ_BIT |
			     MPSCTRL_SECURE_WRITE_BIT |
			     MPSCTRL_NON_SECURE_READ_BIT |
			     MPSCTRL_NON_SECURE_WRITE_BIT | MPSCTRL_VALID);
	}
}

static int exynos_dwmci_core_init(struct dwmci_host *host, int index)
{
	unsigned int div;
	unsigned long freq, sclk;

	if (host->bus_hz)
		freq = host->bus_hz;
	else
		freq = DWMMC_MAX_FREQ;

	/* request mmc clock vlaue of 52MHz.  */
	sclk = get_mmc_clk(index);
	div = DIV_ROUND_UP(sclk, freq);
	/* set the clock divisor for mmc */
	set_mmc_clk(index, div);

	host->name = "EXYNOS DWMMC";
#ifdef CONFIG_EXYNOS5420
	host->quirks = DWMCI_QUIRK_DISABLE_SMU;
#endif
	host->board_init = exynos_dwmci_board_init;

	if (!host->clksel_val) {
		if (index == 0)
			host->clksel_val = DWMMC_MMC0_CLKSEL_VAL;
		else if (index == 2)
			host->clksel_val = DWMMC_MMC2_CLKSEL_VAL;
	}

	host->caps = MMC_MODE_DDR_52MHz;
	host->clksel = exynos_dwmci_clksel;
	host->dev_index = index;
	host->get_mmc_clk = exynos_dwmci_get_clk;
	/* Add the mmc channel to be registered with mmc core */
	if (add_dwmci(host, DWMMC_MAX_FREQ, DWMMC_MIN_FREQ)) {
		debug("dwmmc%d registration failed\n", index);
		return -1;
	}
	return 0;
}

/*
 * This function adds the mmc channel to be registered with mmc core.
 * index -	mmc channel number.
 * regbase -	register base address of mmc channel specified in 'index'.
 * bus_width -	operating bus width of mmc channel specified in 'index'.
 * clksel -	value to be written into CLKSEL register in case of FDT.
 *		NULL in case od non-FDT.
 */
int exynos_dwmci_add_port(int index, u32 regbase, int bus_width, u32 clksel)
{
	struct dwmci_host *host = NULL;

	host = malloc(sizeof(struct dwmci_host));
	if (!host) {
		error("dwmci_host malloc fail!\n");
		return -ENOMEM;
	}

	host->ioaddr = (void *)regbase;
	host->buswidth = bus_width;

	if (clksel)
		host->clksel_val = clksel;

	return exynos_dwmci_core_init(host, index);
}
#ifdef CONFIG_OF_CONTROL
static struct dwmci_host dwmci_host[DWMMC_MAX_CH_NUM];

static int do_dwmci_init(struct dwmci_host *host)
{
	int index, flag, err;

	index = host->dev_index;

	flag = host->buswidth == 8 ? PINMUX_FLAG_8BIT_MODE : PINMUX_FLAG_NONE;
	err = exynos_pinmux_config(host->dev_id, flag);
	if (err) {
		debug("DWMMC not configure\n");
		return err;
	}

	return exynos_dwmci_core_init(host, index);
}

static int exynos_dwmci_get_config(const void *blob, int node,
					struct dwmci_host *host)
{
	int err = 0;
	u32 base, clksel_val, timing[3];

	/* Extract device id for each mmc channel */
	host->dev_id = pinmux_decode_periph_id(blob, node);

	/* Get the bus width from the device node */
	host->buswidth = fdtdec_get_int(blob, node, "samsung,bus-width", 0);
	if (host->buswidth <= 0) {
		debug("DWMMC: Can't get bus-width\n");
		return -EINVAL;
	}

	host->dev_index = fdtdec_get_int(blob, node, "index", host->dev_id);
	if (host->dev_index == host->dev_id)
		host->dev_index = host->dev_id - PERIPH_ID_SDMMC0;

	/* Set the base address from the device node */
	base = fdtdec_get_addr(blob, node, "reg");
	if (!base) {
		debug("DWMMC: Can't get base address\n");
		return -EINVAL;
	}
	host->ioaddr = (void *)base;

	/* Extract the timing info from the node */
	err =  fdtdec_get_int_array(blob, node, "samsung,timing", timing, 3);
	if (err) {
		debug("Can't get sdr-timings for devider\n");
		return -EINVAL;
	}

	clksel_val = (DWMCI_SET_SAMPLE_CLK(timing[0]) |
			DWMCI_SET_DRV_CLK(timing[1]) |
			DWMCI_SET_DIV_RATIO(timing[2]));
	if (clksel_val)
		host->clksel_val = clksel_val;

	host->fifoth_val = fdtdec_get_int(blob, node, "fifoth_val", 0);
	host->bus_hz = fdtdec_get_int(blob, node, "bus_hz", 0);
	host->div = fdtdec_get_int(blob, node, "div", 0);

	return 0;
}

static int exynos_dwmci_process_node(const void *blob,
					int node_list[], int count)
{
	struct dwmci_host *host;
	int i, node, err;

	for (i = 0; i < count; i++) {
		node = node_list[i];
		if (node <= 0)
			continue;
		host = &dwmci_host[i];
		err = exynos_dwmci_get_config(blob, node, host);
		if (err) {
			debug("%s: failed to decode dev %d\n", __func__, i);
			return err;
		}

		do_dwmci_init(host);
	}
	return 0;
}
int exynos_dwmmc_init(const void *blob)
{
	int compat_id;
	int node_list[DWMMC_MAX_CH_NUM];
	int err = 0, count;

	compat_id = COMPAT_SAMSUNG_EXYNOS_DWMMC;

	count = fdtdec_find_aliases_for_id(blob, "mmc",
				compat_id, node_list, DWMMC_MAX_CH_NUM);
	err = exynos_dwmci_process_node(blob, node_list, count);

	return err;
}
#endif
#endif/*CONFIG_JIANWEI_S5P6818*/

#ifndef __S5P6818_REG_GPIO_H__
#define __S5P6818_REG_GPIO_H__

#define S5P6818_GPIOA_BASE				(0xC001A000)
#define S5P6818_GPIOB_BASE				(0xC001B000)
#define S5P6818_GPIOC_BASE				(0xC001C000)
#define S5P6818_GPIOD_BASE				(0xC001D000)
#define S5P6818_GPIOE_BASE				(0xC001E000)

#define GPIO_OUT						(0x00)
#define GPIO_OUTENB						(0x04)
#define GPIO_DETMODE0					(0x08)
#define GPIO_DETMODE1					(0x0C)
#define GPIO_INTENB						(0x10)
#define GPIO_DET						(0x14)
#define GPIO_PAD						(0x18)
#define GPIO_ALTFN0						(0x20)
#define GPIO_ALTFN1						(0x24)
#define GPIO_DETMODEEX					(0x28)
#define GPIO_DETENB						(0x3C)
#define GPIO_SLEW						(0x40)
#define GPIO_SLEW_DISABLE_DEFAULT		(0x44)
#define GPIO_DRV1						(0x48)
#define GPIO_DRV1_DISABLE_DEFAULT		(0x4C)
#define GPIO_DRV0						(0x50)
#define GPIO_DRV0_DISABLE_DEFAULT		(0x54)
#define GPIO_PULLSEL					(0x58)
#define GPIO_PULLSEL_DISABLE_DEFAULT	(0x5C)
#define GPIO_PULLENB					(0x60)
#define GPIO_PULLENB_DISABLE_DEFAULT	(0x64)

#endif /* __S5P6818_REG_GPIO_H__ */

enum s5p6818_altfn {
	SDCLK0 = 1,
	SDCMD0 = 1,
	SDDAT0_0 = 1,
	SDDAT0_1 = 1,
	SDDAT0_2 = 1,
	SDDAT0_3 = 1,
	SDCLK1 = 1,
	SDCMD1 = 1,
	SDDAT1_0 = 1,
	SDDAT1_1 = 1,
	SDDAT1_2 = 1,
	SDDAT1_3 = 1,
	SDCLK2 = 2,
	SDCMD2 = 2,
	SDDAT2_0 = 2,
	SDDAT2_1 = 2,
	SDDAT2_2 = 2,
	SDDAT2_3 = 2,

};
struct s5p6818_altfn_data{
	unsigned altfn_regaddr;
	unsigned altfn_sift;
	unsigned altfn_width;
	enum s5p6818_altfn altfn;
};
static struct s5p6818_altfn_data sdmmc_pinmux_d[] = {
	{/*MCU_SD2_CLK, SA18/GPIOC18/SDCLK2/VID2[1], AB12*/
		.altfn_regaddr = (S5P6818_GPIOC_BASE+GPIO_ALTFN1),
		.altfn_sift = 4,
		.altfn_width = 2,
		.altfn = SDCLK2,
	},
	{/*MCU_SD2_CMD, SA19/GPIOC19/SDCMD2/VID2[2], AB14*/
		.altfn_regaddr = (S5P6818_GPIOC_BASE+GPIO_ALTFN1),
		.altfn_sift = 6,
		.altfn_width = 2,
		.altfn = SDCMD2,
	},
	{/*MCU_SD2_D0, SA20/GPIOC20/SDDAT2[0]/VID2[3], AB11*/
		.altfn_regaddr = (S5P6818_GPIOC_BASE+GPIO_ALTFN1),
		.altfn_sift = 8,
		.altfn_width = 2,
		.altfn = SDDAT2_0,
	},
	{/*MCU_SD2_D1, SA21/GPIOC21/SDDAT2[1]/VID2[4], AA12*/
		.altfn_regaddr = (S5P6818_GPIOC_BASE+GPIO_ALTFN1),
		.altfn_sift = 10,
		.altfn_width = 2,
		.altfn = SDDAT2_1,
	},
	{/*MCU_SD2_D2, SA22/GPIOC22/SDDAT2[2]/VID2[5], AC14*/
		.altfn_regaddr = (S5P6818_GPIOC_BASE+GPIO_ALTFN1),
		.altfn_sift = 12,
		.altfn_width = 2,
		.altfn = SDDAT2_2,
	},
	{/*MCU_SD2_D3, SA23/GPIOC23/SDDAT2[3]/VID2[6], AC12*/
		.altfn_regaddr = (S5P6818_GPIOC_BASE+GPIO_ALTFN1),
		.altfn_sift = 14,
		.altfn_width = 2,
		.altfn = SDDAT2_3,
	},
	{/*MCU_SD1_CLK, GPIOD22/SDCLK1, AA20*/
		.altfn_regaddr = (S5P6818_GPIOD_BASE+GPIO_ALTFN1),
		.altfn_sift = 12,
		.altfn_width = 2,
		.altfn = SDCLK1,
	},
	{/*MCU_SD1_CMD, GPIOD23/SDCMD1, AA19*/
		.altfn_regaddr = (S5P6818_GPIOD_BASE+GPIO_ALTFN1),
		.altfn_sift = 14,
		.altfn_width = 2,
		.altfn = SDCMD1,
	},
	{/*MCU_SD1_D0, GPIOD24/SDDAT1[0], AA18*/
		.altfn_regaddr = (S5P6818_GPIOD_BASE+GPIO_ALTFN1),
		.altfn_sift = 16,
		.altfn_width = 2,
		.altfn = SDDAT1_0,
	},
	{/*MCU_SD1_D1, GPIOD25/SDDAT1[1], AA17*/
		.altfn_regaddr = (S5P6818_GPIOD_BASE+GPIO_ALTFN1),
		.altfn_sift = 18,
		.altfn_width = 2,
		.altfn = SDDAT1_1,
	},
	{/*MCU_SD1_D2, GPIOD27/SDDAT1[2], Y15*/
		.altfn_regaddr = (S5P6818_GPIOD_BASE+GPIO_ALTFN1),
		.altfn_sift = 22,
		.altfn_width = 2,
		.altfn = SDDAT1_2,
	},
	{/*MCU_SD1_D3, GPIOD26/SDDAT1[3], Y14*/
		.altfn_regaddr = (S5P6818_GPIOD_BASE+GPIO_ALTFN1),
		.altfn_sift = 20,
		.altfn_width = 2,
		.altfn = SDDAT1_3,
	},
	{0, 0, 0, 0},
};
static unsigned in32(unsigned addr)
{
	return (*((volatile unsigned*)addr));
}
static void out32(unsigned addr, unsigned data)
{
	(*((volatile unsigned*)addr)) = data;
}
static void sr32(unsigned addr, unsigned start_bit, unsigned bit_num, unsigned data)
{
	unsigned mask = ~(((1<<bit_num) - 1)<<start_bit);
	out32(addr, ((in32(addr))&mask) | (data<<start_bit));
}
static void s5p6818_sdmmmc_pinmux_init(void)
{
	int i;
	struct s5p6818_altfn_data *d = sdmmc_pinmux_d;
	for(i=0; ; i++){
		if(d[i].altfn_regaddr == 0) break;
		sr32(d[i].altfn_regaddr, d[i].altfn_sift, d[i].altfn_width, d[i].altfn);
	}
}
static unsigned int s5p6818_dw_mci_get_clk(struct dwmci_host *host)
{
	return 0;
}

static unsigned long s5p6818_dw_mci_set_clk(int dev_index, unsigned  rate)
{
	unsigned val;
	if(dev_index == 0){
		val = readl(0xc00c5004);
		val &= ~(0x07   << 2);
		val |=  (1    << 2);
		val	&= ~(0xFF   << 5);
		val	|=  (200-1) << 5;
		writel(val, 0xc00c5004);
		writel(readl(0xc00c5000)|(1<<0)|(1<<2)|(1<<3), 0xc00c5000);
	}
	else if(dev_index == 1){
		val = readl(0xc00cc004);
		val &= ~(0x07   << 2);
		val |=  (1    << 2);
		val	&= ~(0xFF   << 5);
		val	|=  (200-1) << 5;
		writel(val, 0xc00cc004);
		writel(readl(0xc00cc000)|(1<<0)|(1<<2)|(1<<3), 0xc00cc000);
	}
	else if(dev_index == 2){
		val = readl(0xc00cd004);
		val &= ~(0x07   << 2);
		val |=  (1    << 2);
		val	&= ~(0xFF   << 5);
		val	|=  (200-1) << 5;
		writel(val, 0xc00cd004);
		writel(readl(0xc00cd000)|(1<<0)|(1<<2)|(1<<3), 0xc00cd000);
	}
	return rate;
}

static void s5p6818_dw_mci_clksel(struct dwmci_host *host)
{
	if(host->dev_index == 0){

	}
	else if(host->dev_index == 1){

	}
	else if(host->dev_index == 2){

	}
}

static void s5p6818_dw_mci_clk_delay(int val ,int regbase )
{
	writel(val, regbase + 0x114);
}
static int s5p6818_dw_mci_init(u32 regbase, int dev_index)
{
	struct dwmci_host *host = NULL;
	int  fifo_size = 0x20;
	void *host_ioaddr = 0;

	host = malloc(sizeof(struct dwmci_host));

	if(dev_index == 0){
		s5p6818_dw_mci_set_clk(dev_index, 50000000 * 4);
		host_ioaddr = (void *)0xC0062000;
	}
	else if(dev_index == 1){
		s5p6818_dw_mci_set_clk(dev_index, 50000000 * 4);
		host_ioaddr = (void *)0xC0068000;
	}
	else if(dev_index == 2){
		s5p6818_dw_mci_set_clk(dev_index, 50000000 * 4);
		host_ioaddr = (void *)0xC0069000;
	}

	host->ioaddr = host_ioaddr;
	host->dev_index = dev_index;
	host->name = "JIANWEI";
	host->buswidth = 4;
	host->clksel = s5p6818_dw_mci_clksel;
	host->get_mmc_clk = s5p6818_dw_mci_get_clk;
	host->fifoth_val = (0x2<<28) | (fifo_size/2 -1)<<16 | (fifo_size/2);

	add_dwmci(host, 50000000, 400000);

	return 0;
}
int exynos_dwmmc_init(const void *blob)
{
	/* CLK DELAY SHIFT Register*/
	#define DW_MMC_DRIVE_DELAY(n)       ((n & 0xFF) << 0)   // write
	#define DW_MMC_DRIVE_PHASE(n)       ((n & 0x03) <<16)   // write
	#define DW_MMC_SAMPLE_DELAY(n)      ((n & 0xFF) << 8)   // read
	#define DW_MMC_SAMPLE_PHASE(n)      ((n & 0x03) <<24)   // read/* CLK DELAY SHIFT Register*/
	#define CONFIG_MMC2_CLK_DELAY       DW_MMC_DRIVE_DELAY(0) | DW_MMC_SAMPLE_DELAY(0x1c) | DW_MMC_DRIVE_PHASE(2)| DW_MMC_SAMPLE_PHASE(1)
	#define CONFIG_MMC1_CLK_DELAY		CONFIG_MMC2_CLK_DELAY
	#define CONFIG_MMC0_CLK_DELAY		CONFIG_MMC2_CLK_DELAY

	s5p6818_sdmmmc_pinmux_init();

	writel(readl(0xC0012004) | (1<<9), 0xC0012004);
	writel(readl(0xC0012004) | (1<<8), 0xC0012004);
	writel(readl(0xC0012004) | (1<<7), 0xC0012004);

	s5p6818_dw_mci_init(0xC0062000, 0);
	s5p6818_dw_mci_clk_delay(CONFIG_MMC0_CLK_DELAY, 0xC0069000);
	s5p6818_dw_mci_init(0xC0068000, 1);
	s5p6818_dw_mci_clk_delay(CONFIG_MMC1_CLK_DELAY, 0xC0069000);
	s5p6818_dw_mci_init(0xC0069000, 2);
	s5p6818_dw_mci_clk_delay(CONFIG_MMC2_CLK_DELAY, 0xC0069000);

	return 0;
}


