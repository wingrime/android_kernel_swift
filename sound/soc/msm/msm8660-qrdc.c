/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */


#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pmic8901.h>
#include <linux/platform_device.h>
#include <linux/mfd/wm8994/gpio.h>
#include <linux/pmic8058-othc.h>
#include <mach/board.h>
#include <mach/mpp.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/dai.h>
#include "../codecs/wm8994.h"
#include "msm8660-pcm.h"

#define PM8058_GPIO_BASE			NR_MSM_GPIOS
#define PM8901_GPIO_BASE			(PM8058_GPIO_BASE + \
						PM8058_GPIOS + PM8058_MPPS)
#define PM8901_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8901_GPIO_BASE)
#define GPIO_EXPANDER_GPIO_BASE \
	(PM8901_GPIO_BASE + PM8901_MPPS)

struct clk *wm8994_mclk;
/* should use atomic ? */
u8 mclk_cnt;
struct clk *rx_bit_clk;
struct clk *tx_bit_clk;

static int rx_hw_param_status;
static int tx_hw_param_status;
/* Platform specific logic */

#define MSM8660_SPK_ON	1
#define MSM8660_SPK_OFF 0

#define MSM_CDC_RX_MCLK_GPIO (109)
#define WM8994_GPIO_BASE (501)

static int msm8660_spk_func;

static int msm_cdc_mclk_gpio_enable(void)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	ret = gpio_request(MSM_CDC_RX_MCLK_GPIO, "CDC MCLK");
	if (ret != 0) {
		pr_err("%s: cdc mclk gpio request"
			"failed\n", __func__);
		goto done;
	}
done:
	return ret;
}

static int msm_cdc_mclk_gpio_disable(void)
{
	pr_debug("%s\n", __func__);

	gpio_free(MSM_CDC_RX_MCLK_GPIO);
	return 0;
}

static int msm8660_mclk_enable_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	pr_debug("%s: Event=%d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		msm_cdc_mclk_gpio_enable();
	}

	return 0;
}
static int codec_mclk_deacquire(void);
static int msm8660_mclk_disable_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	pr_debug("%s: Event=%d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMD:
		codec_mclk_deacquire();
		msm_cdc_mclk_gpio_disable();
	}

	return 0;
}

#define PM8901_MPP_3 (2) /* PM8901 MPP starts from 0 */
/* This function should be made as widget */
static void classd_amp_pwr(int enable)
{
	int rc;

	if (enable) {
		rc = pm8901_mpp_config_digital_out(PM8901_MPP_3,
			PM8901_MPP_DIG_LEVEL_MSMIO, 1);

		if (rc) {
			pr_err("%s: CLASS_D0_EN failed\n", __func__);
			return;
		}

		rc = gpio_request(PM8901_GPIO_PM_TO_SYS(PM8901_MPP_3),
			"CLASSD0_EN");

		if (rc) {
			pr_err("%s: spkr pamp gpio pm8901 mpp3 request"
			"failed\n", __func__);
			pm8901_mpp_config_digital_out(PM8901_MPP_3,
			PM8901_MPP_DIG_LEVEL_MSMIO, 0);
			return;
		}

		gpio_direction_output(PM8901_GPIO_PM_TO_SYS(PM8901_MPP_3), 1);
		gpio_set_value(PM8901_GPIO_PM_TO_SYS(PM8901_MPP_3), 1);

	} else {
		pm8901_mpp_config_digital_out(PM8901_MPP_3,
		PM8901_MPP_DIG_LEVEL_MSMIO, 0);
		gpio_set_value(PM8901_GPIO_PM_TO_SYS(PM8901_MPP_3), 0);
		gpio_free(PM8901_GPIO_PM_TO_SYS(PM8901_MPP_3));
	}
}
static void msm_snddev_enable_qrdc_mic_power(void)
{
	pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_ALWAYS_ON);
}
static void msm_snddev_disable_qrdc_mic_power(void)
{
	pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_OFF);
}

static void wm_poweramp_on(void)
{
	pr_debug("%s: enable stereo spkr amp\n", __func__);
	classd_amp_pwr(1);
}
static void wm_poweramp_off(void)
{
	pr_debug("%s: enable stereo spkr amp\n", __func__);
	classd_amp_pwr(0);
	msleep(30);
}
static void msm8660_ext_control(struct snd_soc_codec *codec)
{
	pr_debug("%s: msm8660_spk_func = %d", __func__, msm8660_spk_func);
	/* set the endpoints to their new connection states */
	if (msm8660_spk_func == MSM8660_SPK_ON)
		snd_soc_dapm_enable_pin(codec, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(codec, "Ext Spk");

	/* Signal a DAPM event*/
	snd_soc_dapm_sync(codec);
}
static int msm8660_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm8660_spk_func = %d", __func__, msm8660_spk_func);
	ucontrol->value.integer.value[0] = msm8660_spk_func;
	return 0;
}
static int msm8660_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	pr_debug("%s()\n", __func__);
	if (msm8660_spk_func == ucontrol->value.integer.value[0])
		return 0;

	msm8660_spk_func = ucontrol->value.integer.value[0];
	msm8660_ext_control(codec);
	return 1;
}
static int msm8660_spkramp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	pr_debug("%s() %x\n", __func__, SND_SOC_DAPM_EVENT_ON(event));
	if (SND_SOC_DAPM_EVENT_ON(event))
		wm_poweramp_on();
	else
		wm_poweramp_off();
	return 0;
}
static const struct snd_soc_dapm_widget msm8660_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Ext Spk", msm8660_spkramp_event),
	SND_SOC_DAPM_PRE("CODEC MCLK", msm8660_mclk_enable_event),
	SND_SOC_DAPM_POST("CODEC MCLK", msm8660_mclk_disable_event),
};

/* Corgi machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Match with timpani codec line out pin */
	{"Ext Spk", NULL, "LINEOUT1P"},
	{"Ext Spk", NULL, "LINEOUT1N"},
};

static const char *spk_function[] = {"Off", "On"};
static const struct soc_enum msm8660_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new timpani_msm8660_controls[] = {
	SOC_ENUM_EXT("Speaker Function", msm8660_enum[0], msm8660_get_spk,
		msm8660_set_spk),
};

static int msm8660_audrx_init(struct snd_soc_codec *codec)
{
	int err;

	pr_debug("%s()\n", __func__);

	snd_soc_dapm_enable_pin(codec, "Ext Spk");

	/* Add poodle specific controls */
	err = snd_soc_add_controls(codec, timpani_msm8660_controls,
				ARRAY_SIZE(timpani_msm8660_controls));
	if (err < 0)
		return err;

	/* Add poodle specific widgets */
	snd_soc_dapm_new_controls(codec, msm8660_dapm_widgets,
				ARRAY_SIZE(msm8660_dapm_widgets));

	/* Set up poodle specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(codec);

	return 0;
}
#define SAMPLE_RATE_GUESS_48k 48000
static int codec_mclk_acquire(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret = 0;
	pr_debug("%s\n", __func__);

	if (mclk_cnt)
		goto done;

	wm8994_mclk = clk_get(NULL, "i2s_spkr_osr_clk");

	if (IS_ERR(wm8994_mclk)) {
		pr_err("Failed to get MCLK\n");
		ret = PTR_ERR(wm8994_mclk);
		goto done;
	}

	clk_set_rate(wm8994_mclk, SAMPLE_RATE_GUESS_48k * 256);
	/* Guess the clock rate to avoid kernel warnings of enabling
	clock before setting rate */

	ret = clk_enable(wm8994_mclk);
	if (ret != 0) {
		pr_err("Unable to enable MCLK\n");
		goto fail_clk;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_CBM_CFM |
	SND_SOC_DAIFMT_I2S);

	if (ret < 0) {
		pr_err("can't set codec DAI configuration\n");
		goto fail_codec_dai;
	}
	mclk_cnt++;

	return ret;

fail_codec_dai:
	clk_disable(wm8994_mclk);
fail_clk:
	clk_put(wm8994_mclk);
done:
	return ret;
}
static int codec_mclk_set_rate(struct snd_pcm_substream *substream, int rate)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret = 0;

	clk_set_rate(wm8994_mclk, rate * 256); /* codec to run @ OSR 256 */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_MCLK1,
		rate * 256, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec clk configuration\n");
		goto fail_codec_dai;
	}
fail_codec_dai:
	return ret;
}

static int codec_mclk_deacquire(void)
{
	if (mclk_cnt) {
		mclk_cnt--;
		if (!mclk_cnt) {
			clk_disable(wm8994_mclk);
			clk_put(wm8994_mclk);
			wm8994_mclk = NULL;
		}
	}

	return 0;
}

static int msm8660_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	int ret;
	int rate = params_rate(params);
	pr_info("%s\n", __func__);
	/* MCLK for both playback/capture */
	ret = codec_mclk_set_rate(substream, rate);
	if (ret)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (rx_hw_param_status)
			return 0;
		clk_set_rate(rx_bit_clk, 0);
		rx_hw_param_status++;
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (tx_hw_param_status)
			return 0;
		clk_set_rate(tx_bit_clk, 0);
		tx_hw_param_status++;
	}
	return 0;
}
static int msm8660_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	pr_info("%s\n", __func__);

	/* MCLK for both playback/capture */
	ret = codec_mclk_acquire(substream);

	if (ret)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Need to set WM8994 GPIO1 to enable ADCBCLK */
		rx_bit_clk = clk_get(NULL, "i2s_spkr_bit_clk");
		if (IS_ERR(rx_bit_clk)) {
			pr_debug("Failed to get i2s_spkr_bit_clk\n");
			return PTR_ERR(rx_bit_clk);
		}
		clk_set_rate(rx_bit_clk, 0); /* Set to slave mode */
		ret = clk_enable(rx_bit_clk);
		if (ret != 0) {
			pr_err("Unable to enable spkr BCLK\n");
			/*clk_put(tx_bit_clk);
			clk_disable(tx_osr_clk);
			clk_put(tx_osr_clk);*/
			return ret;
		}

		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_CBM_CFM);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		/* Need to set WM8994 GPIO1 to enable ADCBCLK */
		tx_bit_clk = clk_get(NULL, "i2s_mic_bit_clk");
		if (IS_ERR(tx_bit_clk)) {
			pr_debug("Failed to get i2s_mic_bit_clk\n");
			return PTR_ERR(tx_bit_clk);
		}
		clk_set_rate(tx_bit_clk, 0); /* Set to slave mode */
		ret = clk_enable(tx_bit_clk);
		if (ret != 0) {
			pr_err("Unable to enable MIC BCLK\n");
			/*clk_put(tx_bit_clk);
			clk_disable(tx_osr_clk);
			clk_put(tx_osr_clk);*/
			return ret;
		}

		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_CBM_CFM);

		if (ret < 0) {
			printk(KERN_ERR "can't set cpu DAI configuration\n");
			return ret;
		}
		msm_snddev_enable_qrdc_mic_power();
	}

	return ret;
}

static void msm8660_shutdown(struct snd_pcm_substream *substream)
{
	pr_info("%s\n", __func__);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		rx_hw_param_status = 0;
		if (rx_bit_clk) {
			clk_disable(rx_bit_clk);
			clk_put(rx_bit_clk);
			rx_bit_clk = NULL;
		}
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		tx_hw_param_status = 0;
		msm_snddev_disable_qrdc_mic_power();
		if (tx_bit_clk) {
			clk_disable(tx_bit_clk);
			clk_put(tx_bit_clk);
			tx_bit_clk = NULL;
		}
	}
}

static struct snd_soc_ops machine_ops  = {
	.startup	= msm8660_startup,
	.shutdown	= msm8660_shutdown,
	.hw_params	= msm8660_hw_params,
};

/* Digital audio interface glue - connects codec <---> CPU */
static struct snd_soc_dai_link msm8660_dai[] = {
	{
		.name		= "WM8994 I2S RX",
		.stream_name	= "MSM8660 QRDC Playback",
		.cpu_dai	= &msm_cpu_dai[0],
		.codec_dai	= &wm8994_dai[0],
		.ops		= &machine_ops,
		.init		= &msm8660_audrx_init,
	},
	{
		.name		= "WM8994 I2S TX ",
		.stream_name	= "MSM8660 QRDC Capture",
		.cpu_dai	= &msm_cpu_dai[5],
		.codec_dai	= &wm8994_dai[0],
		.ops		= &machine_ops,
	},
};

struct snd_soc_card snd_soc_card_msm8660 = {
	.name		= "msm8660_qrdc",
	.dai_link	= msm8660_dai,
	.num_links	= ARRAY_SIZE(msm8660_dai),
	.platform = &msm8660_soc_platform,
};

/* msm_audio audio subsystem */
static struct snd_soc_device msm_snd_devdata = {
	.card = &snd_soc_card_msm8660,
	.codec_dev = &soc_codec_dev_wm8994,
};

static struct platform_device *msm_snd_device;

static int __init msm_audio_init(void)
{
	int ret;

	msm_snd_device = platform_device_alloc("soc-audio", 0);
	if (!msm_snd_device) {
		pr_err("Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(msm_snd_device, &msm_snd_devdata);

	msm_snd_devdata.dev = &msm_snd_device->dev;
	ret = platform_device_add(msm_snd_device);
	if (ret) {
		platform_device_put(msm_snd_device);
		return ret;
	}

	return ret;
}
module_init(msm_audio_init);

static void __exit msm_audio_exit(void)
{
	platform_device_unregister(msm_snd_device);
}
module_exit(msm_audio_exit);

MODULE_DESCRIPTION("ALSA SoC MSM8660 QRDC");
MODULE_LICENSE("GPL v2");
