/*
 * tegra210_apbif_alt.h - Tegra210 APBIF registers
 *
 * Copyright (c) 2014 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TEGRA210_ADMAIF_ALT_H__
#define __TEGRA210_ADMAIF_ALT_H__

#define TEGRA210_ADMAIF_CHANNEL_REG_STRIDE	0x40
#define TEGRA210_ADMAIF_CHANNEL_COUNT		10

#define TEGRA210_ADMAIF_XBAR_RX_ENABLE					0x0
#define TEGRA210_ADMAIF_XBAR_RX_SOFT_RESET				0x4
#define TEGRA210_ADMAIF_XBAR_RX_STATUS					0xc
#define TEGRA210_ADMAIF_XBAR_RX_INT_STATUS				0x10
#define TEGRA210_ADMAIF_XBAR_RX_INT_MASK				0x14
#define TEGRA210_ADMAIF_XBAR_RX_INT_SET					0x18
#define TEGRA210_ADMAIF_XBAR_RX_INT_CLEAR				0x1c
#define TEGRA210_ADMAIF_CHAN_ACIF_RX_CTRL				0x20
#define TEGRA210_ADMAIF_XBAR_RX_FIFO_CTRL				0x28
#define TEGRA210_ADMAIF_XBAR_RX_FIFO_READ				0x2c

#define TEGRA210_ADMAIF_XBAR_TX_ENABLE					0x300
#define TEGRA210_ADMAIF_XBAR_TX_SOFT_RESET				0x304
#define TEGRA210_ADMAIF_XBAR_TX_STATUS					0x30c
#define TEGRA210_ADMAIF_XBAR_TX_INT_STATUS				0x310
#define TEGRA210_ADMAIF_XBAR_TX_INT_MASK				0x314
#define TEGRA210_ADMAIF_XBAR_TX_INT_SET					0x318
#define TEGRA210_ADMAIF_XBAR_TX_INT_CLEAR				0x31c
#define TEGRA210_ADMAIF_CHAN_ACIF_TX_CTRL				0x320
#define TEGRA210_ADMAIF_XBAR_TX_FIFO_CTRL				0x328
#define TEGRA210_ADMAIF_XBAR_TX_FIFO_WRITE				0x32c

#define TEGRA210_ADMAIF_GLOBAL_ENABLE					0x700
#define TEGRA210_ADMAIF_GLOBAL_CG_0						0x708

#define TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK8_EN_SHIFT		31
#define TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK8_EN_MASK			(1 << TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK8_EN_SHIFT)
#define TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK8_EN			(1 << TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK8_EN_SHIFT)
#define TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK16_EN_SHIFT		30
#define TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK16_EN_MASK		(1 << TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK16_EN_SHIFT)
#define TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK16_EN			(1 << TEGRA210_ADMAIF_CHAN_ACIF_CTRL_PACK16_EN_SHIFT)

#define TEGRA210_ADMAIF_XBAR_TX_ENABLE_SHIFT		0
#define TEGRA210_ADMAIF_XBAR_TX_EN			(1 << TEGRA210_ADMAIF_XBAR_TX_ENABLE_SHIFT)
#define TEGRA210_ADMAIF_XBAR_TX_ENABLE_MASK		(1 << TEGRA210_ADMAIF_XBAR_TX_ENABLE_SHIFT)

#define TEGRA210_ADMAIF_XBAR_RX_ENABLE_SHIFT		0
#define TEGRA210_ADMAIF_XBAR_RX_EN			(1 << TEGRA210_ADMAIF_XBAR_RX_ENABLE_SHIFT)
#define TEGRA210_ADMAIF_XBAR_RX_ENABLE_MASK		(1 << TEGRA210_ADMAIF_XBAR_RX_ENABLE_SHIFT)

#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT	0
#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_MASK	(0x1f << TEGRA210_ADMAIF_XBAR_DMA_FIFO_START_ADDR_SHIFT)

#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_SHIFT	8
#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_MASK		(0x1f << TEGRA210_ADMAIF_XBAR_DMA_FIFO_SIZE_SHIFT)

#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_SHIFT	20
#define TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_MASK	(0x2ff << TEGRA210_ADMAIF_XBAR_DMA_FIFO_THRESHOLD_SHIFT)

#define TEGRA210_ADMAIF_XBAR_STATUS_TRANS_EN_SHIFT	0
#define TEGRA210_ADMAIF_XBAR_STATUS_TRANS_EN_MASK	(0x1 << TEGRA210_ADMAIF_XBAR_STATUS_TRANS_EN_SHIFT)

#define TEGRA210_ADMAIF_XBAR_CIF_CTRL_UNPACK8_ENABLE_SHIFT	31
#define TEGRA210_ADMAIF_XBAR_CIF_CTRL_UNPACK8_ENABLE		BIT(TEGRA210_ADMAIF_XBAR_CIF_CTRL_UNPACK8_ENABLE_SHIFT)

#define TEGRA210_ADMAIF_XBAR_CIF_CTRL_UNPACK16_ENABLE_SHIFT	30
#define TEGRA210_ADMAIF_XBAR_CIF_CTRL_UNPACK16_ENABLE		BIT(TEGRA210_ADMAIF_XBAR_CIF_CTRL_UNPACK16_ENABLE_SHIFT)

#define TEGRA210_ADMAIF_LAST_REG			0x75f

enum {
	DATA_8BIT,
	DATA_16BIT,
	DATA_32BIT
};

struct tegra210_admaif_soc_data {
	unsigned int num_ch;
	unsigned int clk_list_mask;
	void (*set_audio_cif)(struct regmap *map,
			unsigned int reg,
			struct tegra210_xbar_cif_conf *cif_conf);
};

struct tegra210_admaif {
	/* regmap for admaif */
	struct regmap *regmap;
	int refcnt;
	struct tegra_alt_pcm_dma_params *capture_dma_data;
	struct tegra_alt_pcm_dma_params *playback_dma_data;
	const struct tegra210_admaif_soc_data *soc_data;
	int override_channels[TEGRA210_ADMAIF_CHANNEL_COUNT];
};

#endif
