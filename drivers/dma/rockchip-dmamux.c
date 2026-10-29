// SPDX-License-Identifier: GPL-2.0-only
/*
 * DMA request-line mux of the Rockchip RK3506 DMAC0
 *
 * Each of the request lines of the first PL330 on the RK3506 can be driven by
 * one of up to four peripherals; a two-bit field per line in the GRF selects
 * which. Model it as a DMA router so that a peripheral names its request line
 * and the select value in its dmas property and the mux is programmed when the
 * channel is requested.
 *
 * Copyright (C) 2026 Bas Vermeulen <bas.vermeulen@blackstar.nl>
 *
 * Based on the LPC18xx DMA MUX driver by Joachim Eastwood.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#define RK3506_GRF_DMAC0_CON0		0x00a8
#define RK3506_DMAMUX_LINES_PER_REG	8
#define RK3506_DMAMUX_MAX_VAL		0x3
#define RK3506_DMAMUX_DEFAULT_LINES	12

/* GRF registers take a write-enable mask in the upper half word */
#define RK3506_DMAMUX_REG(n)	(RK3506_GRF_DMAC0_CON0 + \
				 4 * ((n) / RK3506_DMAMUX_LINES_PER_REG))
#define RK3506_DMAMUX_SHIFT(n)	(2 * ((n) % RK3506_DMAMUX_LINES_PER_REG))
#define RK3506_DMAMUX_WRITE(n, v) \
	(((RK3506_DMAMUX_MAX_VAL << 16) | (v)) << RK3506_DMAMUX_SHIFT(n))

struct rockchip_dmamux {
	u32 value;
	bool busy;
};

struct rockchip_dmamux_data {
	struct dma_router dmarouter;
	struct rockchip_dmamux *muxes;
	u32 dma_requests;
	struct regmap *grf;
	spinlock_t lock;
};

static void rockchip_dmamux_free(struct device *dev, void *route_data)
{
	struct rockchip_dmamux_data *dmamux = dev_get_drvdata(dev);
	struct rockchip_dmamux *mux = route_data;
	unsigned long flags;

	spin_lock_irqsave(&dmamux->lock, flags);
	mux->busy = false;
	spin_unlock_irqrestore(&dmamux->lock, flags);
}

static void *rockchip_dmamux_reserve(struct of_phandle_args *dma_spec,
				     struct of_dma *ofdma)
{
	struct platform_device *pdev = of_find_device_by_node(ofdma->of_node);
	struct rockchip_dmamux_data *dmamux = platform_get_drvdata(pdev);
	unsigned long flags;
	unsigned int line;
	int ret = -EINVAL;

	if (dma_spec->args_count != 2) {
		dev_err(&pdev->dev, "invalid number of dma mux args\n");
		goto err_put_pdev;
	}

	line = dma_spec->args[0];
	if (line >= dmamux->dma_requests) {
		dev_err(&pdev->dev, "invalid request line: %u\n", line);
		goto err_put_pdev;
	}

	if (dma_spec->args[1] > RK3506_DMAMUX_MAX_VAL) {
		dev_err(&pdev->dev, "invalid mux value: %u\n", dma_spec->args[1]);
		goto err_put_pdev;
	}

	/* The of_node_put() will be done in the core for the node */
	dma_spec->np = of_parse_phandle(ofdma->of_node, "dma-masters", 0);
	if (!dma_spec->np) {
		dev_err(&pdev->dev, "can't get dma master\n");
		goto err_put_pdev;
	}

	spin_lock_irqsave(&dmamux->lock, flags);
	if (dmamux->muxes[line].busy) {
		spin_unlock_irqrestore(&dmamux->lock, flags);
		dev_err(&pdev->dev, "request line %u busy with select %u\n",
			line, dmamux->muxes[line].value);
		of_node_put(dma_spec->np);
		ret = -EBUSY;
		goto err_put_pdev;
	}

	dmamux->muxes[line].busy = true;
	dmamux->muxes[line].value = dma_spec->args[1];
	regmap_write(dmamux->grf, RK3506_DMAMUX_REG(line),
		     RK3506_DMAMUX_WRITE(line, dmamux->muxes[line].value));
	spin_unlock_irqrestore(&dmamux->lock, flags);

	/* the PL330 takes a single cell: the request line */
	dma_spec->args_count = 1;

	dev_dbg(&pdev->dev, "request line %u driven by peripheral %u\n", line,
		dmamux->muxes[line].value);

	put_device(&pdev->dev);
	return &dmamux->muxes[line];

err_put_pdev:
	put_device(&pdev->dev);
	return ERR_PTR(ret);
}

static int rockchip_dmamux_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct rockchip_dmamux_data *dmamux;

	dmamux = devm_kzalloc(&pdev->dev, sizeof(*dmamux), GFP_KERNEL);
	if (!dmamux)
		return -ENOMEM;

	dmamux->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(dmamux->grf))
		return dev_err_probe(&pdev->dev, PTR_ERR(dmamux->grf),
				     "failed to get the GRF\n");

	dmamux->dma_requests = RK3506_DMAMUX_DEFAULT_LINES;
	of_property_read_u32(np, "dma-requests", &dmamux->dma_requests);

	dmamux->muxes = devm_kcalloc(&pdev->dev, dmamux->dma_requests,
				     sizeof(*dmamux->muxes), GFP_KERNEL);
	if (!dmamux->muxes)
		return -ENOMEM;

	spin_lock_init(&dmamux->lock);
	platform_set_drvdata(pdev, dmamux);
	dmamux->dmarouter.dev = &pdev->dev;
	dmamux->dmarouter.route_free = rockchip_dmamux_free;

	return of_dma_router_register(np, rockchip_dmamux_reserve,
				      &dmamux->dmarouter);
}

static const struct of_device_id rockchip_dmamux_match[] = {
	{ .compatible = "rockchip,rk3506-dmamux" },
	{},
};

static struct platform_driver rockchip_dmamux_driver = {
	.probe	= rockchip_dmamux_probe,
	.driver = {
		.name = "rockchip-dmamux",
		.of_match_table = rockchip_dmamux_match,
	},
};

static int __init rockchip_dmamux_init(void)
{
	return platform_driver_register(&rockchip_dmamux_driver);
}
arch_initcall(rockchip_dmamux_init);
