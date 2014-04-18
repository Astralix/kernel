/*$_FOR_ROCKCHIP_RBOX_$*/
/*$_rbox_$_modify_$_huangzhibao for spdif output*/

/*
 * smdk_spdif.c  --  S/PDIF audio for SMDK
 *
 * Copyright 2010 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "card_info.h"
#include "rk_pcm.h"


#if 0
#define RK_SPDIF_DBG(x...) printk(KERN_INFO "rk_hdmi_spdif:"x)
#else
#define RK_SPDIF_DBG(x...) do { } while (0)
#endif


static int set_audio_clock_rate(unsigned long pll_rate,
				unsigned long audio_rate)
{
	struct clk *sclk_spdif;
#if defined (CONFIG_ARCH_RK30) || defined (CONFIG_ARCH_RK3188)
	struct clk *hclk_spdif;
#endif

#if defined (CONFIG_ARCH_RK30) || defined (CONFIG_ARCH_RK3188)
	hclk_spdif = clk_get(NULL, "hclk_spdif");
	if (IS_ERR(hclk_spdif)) {
		printk(KERN_ERR "spdif:failed to get hclk_spdif\n");
		return -ENOENT;
	}

	clk_set_rate(hclk_spdif, pll_rate);
	clk_put(hclk_spdif);
#endif

	sclk_spdif = clk_get(NULL, "spdif");
	if (IS_ERR(sclk_spdif)) {
		printk(KERN_ERR "spdif:failed to get sclk_spdif\n");
		return -ENOENT;
	}

	clk_set_rate(sclk_spdif, audio_rate);
	clk_put(sclk_spdif);

	return 0;
}

static int rk_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned long pll_out, rclk_rate, dai_fmt = rtd->dai_link->dai_fmt;
	int ret, ratio;

	RK_SPDIF_DBG("spdif:Entered %s\n", __func__);

	return 0;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_fmt);
	if (ret < 0) {
		printk("%s():failed to set the format for codec side\n", __func__);
		return ret;
	}

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_fmt);
	if (ret < 0) {
		printk("%s():failed to set the format for cpu side\n", __func__);
		return ret;
	}
  
	switch (params_rate(params)) {
	case 44100:
		pll_out = 11289600;
		break;
	case 32000:
		pll_out = 8192000;
		break;
	case 48000:
		pll_out = 12288000;
		break;
	case 96000:
		pll_out = 24576000;
		break;
	default:
		printk("rk_spdif: params not support\n");
		return -EINVAL;
	}

	ratio = 256;
	rclk_rate = params_rate(params) * ratio;

	/* Set audio source clock rates */
	ret = set_audio_clock_rate(pll_out, rclk_rate);
	if (ret < 0)
		return ret;

	/* Set S/PDIF uses internal source clock */
	//ret = snd_soc_dai_set_sysclk(cpu_dai, SND_SOC_SPDIF_INT_MCLK,
					//rclk_rate, SND_SOC_CLOCK_IN);
	//if (ret < 0)
		//return ret;

	return ret;
}

static struct snd_soc_ops rk_spdif_ops = {
	.hw_params = rk_hw_params,
};

static struct snd_soc_dai_link rk_dai = {
	.name = "SPDIF",
	.stream_name = "SPDIF PCM Playback",
	.codec_dai_name = "rk-hdmi-spdif-hifi",
	.ops = &rk_spdif_ops,
};

static struct snd_soc_card rockchip_hdmi_spdif_snd_card = {
	.name = "RK-HDMI-SPDIF",
	.dai_link = &rk_dai,
	.num_links = 1,
};

static int rockchip_hdmi_spdif_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &rockchip_hdmi_spdif_snd_card;

	card->dev = &pdev->dev;

	ret = rockchip_of_get_sound_card_info_(card, false);
	if (ret) {
		printk("%s() get sound card info failed:%d\n", __FUNCTION__, ret);
		return ret;
	}

	ret = snd_soc_register_card(card);

	if (ret)
		printk("%s() register card failed:%d\n", __FUNCTION__, ret);

	return ret;
}

static int rockchip_hdmi_spdif_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rockchip_hdmi_spdif_of_match[] = {
	{ .compatible = "rockchip-hdmi-spdif"},
	{},
};
MODULE_DEVICE_TABLE(of, rockchip_hdmi_spdif_of_match);
#endif /* CONFIG_OF */

static struct platform_driver rockchip_hdmi_spdif_audio_driver = {
	.driver         = {
		.name   = "rockchip-hdmi-spdif",
		.owner  = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(rockchip_hdmi_spdif_of_match),
	},
	.probe          = rockchip_hdmi_spdif_audio_probe,
	.remove         = rockchip_hdmi_spdif_audio_remove,
};

module_platform_driver(rockchip_hdmi_spdif_audio_driver);

MODULE_AUTHOR("hzb, <hzb@rock-chips.com>");
MODULE_DESCRIPTION("ALSA SoC RK+S/PDIF");
MODULE_LICENSE("GPL");
