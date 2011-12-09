/*	tlv320aic23b - driver version 0.0.2	 */

/*#define DEBUG*/

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>

//#include <sound/driver.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/pcm.h>

#include <linux/atmel-ssc.h>

#include <linux/spi/spi.h>

#define BITRATE	48000
#define SAMPLINGRATE	0  // 48 kHz

struct tlv320aic23b_board_info{
	int		ssc_id;
	struct clk	*dac_clk;
	char		shortname[32];
};

//=======================================================================================================================================
struct snd_tlv320aic23b {
	struct snd_card			*card;
	struct snd_pcm			*pcm;
	struct snd_pcm_substream	*substream_playback;
	struct snd_pcm_substream	*substream_capture;
	struct tlv320aic23b_board_info	*board;
	int				irq;
	int				period_playback;
	int				period_capture;
	unsigned long			bitrate;
	struct clk			*bitclk;
	struct ssc_device		*ssc;
	struct spi_device		*spi;
	u8				spi_wbuffer[2];
	u8				spi_rbuffer[2];
	/* Image of the SPI registers. */
	u8				reg_image[18];
	/* Protect registers against concurrent access. */
	spinlock_t			lock;
};

#define get_chip(card) ((struct snd_tlv320aic23b *)card->private_data)
//=======================================================================================================================================
static int
snd_tlv320aic23b_write_reg(struct snd_tlv320aic23b *chip, u8 reg, u8 val){
	struct spi_message msg;
	struct spi_transfer msg_xfer = {
		.len		= 2,
		.cs_change	= 0,
	};
	int retval;

	spi_message_init(&msg);

	reg = reg << 1;

	chip->spi_wbuffer[0] = reg;
	chip->spi_wbuffer[1] = val;

	msg_xfer.tx_buf = chip->spi_wbuffer;
	msg_xfer.rx_buf = chip->spi_rbuffer;
	spi_message_add_tail(&msg_xfer, &msg);

	retval = spi_sync(chip->spi, &msg);

	if (!retval)
		chip->reg_image[reg] = val;

	return retval;
}
//=======================================================================================================================================
static struct snd_pcm_hardware snd_tlv320aic23b_playback_hw = {
	.info		= SNDRV_PCM_INFO_INTERLEAVED |
			  SNDRV_PCM_INFO_BLOCK_TRANSFER,
#ifdef __BIG_ENDIAN
	.formats	= SNDRV_PCM_FMTBIT_S16_BE,
#else
	.formats	= SNDRV_PCM_FMTBIT_S16_LE,
#endif
	.rates		= SNDRV_PCM_RATE_CONTINUOUS,
	.rate_min	= 8000,  /* Replaced by chip->bitrate later. */
	.rate_max	= 50000, /* Replaced by chip->bitrate later. */
	.channels_min	= 2,
	.channels_max	= 2,
	.buffer_bytes_max = 64 * 1024 - 1,
	.period_bytes_min = 512,
	.period_bytes_max = 64 * 1024 - 1,
	.periods_min	= 4,
	.periods_max	= 1024,
};
static struct snd_pcm_hardware snd_tlv320aic23b_capture_hw = {
	.info		= SNDRV_PCM_INFO_INTERLEAVED |
			  SNDRV_PCM_INFO_BLOCK_TRANSFER,
#ifdef __BIG_ENDIAN
	.formats	= SNDRV_PCM_FMTBIT_S16_BE,
#else
	.formats	= SNDRV_PCM_FMTBIT_S16_LE,
#endif
	.rates		= SNDRV_PCM_RATE_CONTINUOUS,
	.rate_min	= 8000,  /* Replaced by chip->bitrate later. */
	.rate_max	= 50000, /* Replaced by chip->bitrate later. */
	.channels_min	= 2,
	.channels_max	= 2,
	.buffer_bytes_max = 64 * 1024 - 1,
	.period_bytes_min = 512,
	.period_bytes_max = 64 * 1024 - 1,
	.periods_min	= 4,
	.periods_max	= 1024,
};

//=======================================================================================================================================
/* Calculate and set bitrate and divisions.	*/
static int snd_tlv320aic23b_set_bitrate(struct snd_tlv320aic23b *chip){
	unsigned long ssc_rate = clk_get_rate(chip->ssc->clk);
	unsigned long ssc_div, status;

	ssc_div = ssc_rate / (BITRATE * 2 * 16);

	/* ssc_div must be a power of 2 for hardware dividing */
	ssc_div = (ssc_div + 1) & ~1UL;

	status = clk_set_rate(chip->board->dac_clk, 12000000);
	if (status < 0){
		printk(KERN_INFO "ERROR clk_set_rate\n");
		return status;
	}

	status = clk_get_rate(chip->board->dac_clk);

	/* Set divider in SSC device. */
	ssc_writel(chip->ssc->regs, CMR, ssc_div/2);

	/* SSC clock / (ssc divider * 16-bit * stereo). */
	chip->bitrate = ssc_rate / (ssc_div * 16 * 2);

	dev_info(&chip->spi->dev,"tlv320aic23b: supported bitrate is %lu (%lu divider)\n",chip->bitrate, ssc_div);

	return 0;
}
//=========================CMR==============================================================================================================
static int snd_tlv320aic23b_pcm_open_playback(struct snd_pcm_substream *substream){
	struct snd_tlv320aic23b *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	unsigned long ssc_rate = clk_get_rate(chip->ssc->clk);
	unsigned long ssc_div=0;

	ssc_div = ssc_rate / (BITRATE * 2 * 16);
	ssc_div = (ssc_div + 1) & ~1UL;
	chip->bitrate = ssc_rate / (ssc_div * 16 * 2);

	snd_tlv320aic23b_playback_hw.rate_min = chip->bitrate;
	snd_tlv320aic23b_playback_hw.rate_max = chip->bitrate;
	runtime->hw = snd_tlv320aic23b_playback_hw;
	chip->substream_playback = substream;

	printk(KERN_INFO "tlv320aic23b: playback bitrate is %lu (%lu divider)\n", chip->bitrate, ssc_div);

	return 0;
}
static int snd_tlv320aic23b_pcm_open_capture(struct snd_pcm_substream *substream){
	struct snd_tlv320aic23b *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	unsigned long ssc_rate = clk_get_rate(chip->ssc->clk);
	unsigned long ssc_div=0;

	ssc_div = ssc_rate / (BITRATE * 2 * 16);
	ssc_div = (ssc_div + 1) & ~1UL;
	chip->bitrate = ssc_rate / (ssc_div * 16 * 2);

	snd_tlv320aic23b_capture_hw.rate_min = chip->bitrate;
	snd_tlv320aic23b_capture_hw.rate_max = chip->bitrate;
	runtime->hw = snd_tlv320aic23b_capture_hw;
	chip->substream_capture = substream;

	printk(KERN_INFO "tlv320aic23b: capture bitrate is %lu (%lu divider)\n", chip->bitrate, ssc_div);

	return 0;
}
//=======================================================================================================================================
static int snd_tlv320aic23b_pcm_close_playback(struct snd_pcm_substream *substream){
	struct snd_tlv320aic23b *chip = snd_pcm_substream_chip(substream);
	chip->substream_playback = NULL;
	return 0;
}
static int snd_tlv320aic23b_pcm_close_capture(struct snd_pcm_substream *substream){
	struct snd_tlv320aic23b *chip = snd_pcm_substream_chip(substream);
	chip->substream_capture = NULL;
	return 0;
}
//=======================================================================================================================================
static int snd_tlv320aic23b_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *hw_params){
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int snd_tlv320aic23b_pcm_hw_free(struct snd_pcm_substream *substream){
	return snd_pcm_lib_free_pages(substream);
}
//=======================================================================================================================================
static int snd_tlv320aic23b_pcm_prepare_playback(struct snd_pcm_substream *substream){
	struct snd_tlv320aic23b *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int block_size;

	block_size = frames_to_bytes(runtime, runtime->period_size);
	chip->period_playback = 0;

	ssc_writel(chip->ssc->regs, PDC_TPR, (long)runtime->dma_addr);
	ssc_writel(chip->ssc->regs, PDC_TCR, runtime->period_size * 2);
	ssc_writel(chip->ssc->regs, PDC_TNPR, (long)runtime->dma_addr + block_size);
	ssc_writel(chip->ssc->regs, PDC_TNCR, runtime->period_size * 2);

	return 0;
}
static int snd_tlv320aic23b_pcm_prepare_capture(struct snd_pcm_substream *substream){
	struct snd_tlv320aic23b *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int block_size;

	block_size = frames_to_bytes(runtime, runtime->period_size);
	chip->period_capture = 0;

	ssc_writel(chip->ssc->regs, PDC_RPR, (long)runtime->dma_addr);
	ssc_writel(chip->ssc->regs, PDC_RCR, runtime->period_size * 2);
	ssc_writel(chip->ssc->regs, PDC_RNPR, (long)runtime->dma_addr + block_size);
	ssc_writel(chip->ssc->regs, PDC_RNCR, runtime->period_size * 2);

	return 0;
}
//=======================================================================================================================================
static int snd_tlv320aic23b_pcm_trigger_playback(struct snd_pcm_substream *substream, int cmd){
	struct snd_tlv320aic23b *chip = snd_pcm_substream_chip(substream);
	int retval = 0;

	spin_lock(&chip->lock);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ssc_writel(chip->ssc->regs, IER, SSC_BIT(IER_ENDTX));
		ssc_writel(chip->ssc->regs, PDC_PTCR, SSC_BIT(PDC_PTCR_TXTEN));
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ssc_writel(chip->ssc->regs, PDC_PTCR, SSC_BIT(PDC_PTCR_TXTDIS));
		ssc_writel(chip->ssc->regs, IDR, SSC_BIT(IDR_ENDTX));
		break;
	default:
		dev_dbg(&chip->spi->dev, "spurious command %x\n", cmd);
		retval = -EINVAL;
		break;
	}

	spin_unlock(&chip->lock);

	return retval;
}
static int snd_tlv320aic23b_pcm_trigger_capture(struct snd_pcm_substream *substream, int cmd){
	struct snd_tlv320aic23b *chip = snd_pcm_substream_chip(substream);
	int retval = 0;

	spin_lock(&chip->lock);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ssc_writel(chip->ssc->regs, PDC_PTCR, 1);
		ssc_writel(chip->ssc->regs, IER, SSC_BIT(IER_ENDRX));
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ssc_writel(chip->ssc->regs, PDC_PTCR, SSC_BIT(PDC_PTCR_RXTDIS));
		ssc_writel(chip->ssc->regs, IDR, SSC_BIT(IDR_ENDRX));
		break;
	default:
		dev_dbg(&chip->spi->dev, "spurious command %x\n", cmd);
		retval = -EINVAL;
		break;
	}

	spin_unlock(&chip->lock);

	return retval;
}
//=======================================================================================================================================
static snd_pcm_uframes_t
snd_tlv320aic23b_pcm_pointer_playback(struct snd_pcm_substream *substream){
	struct snd_tlv320aic23b *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t pos;
	unsigned long bytes;

	bytes = ssc_readl(chip->ssc->regs, PDC_TPR) - (unsigned long)runtime->dma_addr;
	pos = bytes_to_frames(runtime, bytes);
	if (pos >= runtime->buffer_size)
		pos -= runtime->buffer_size;

	return pos;
}
static snd_pcm_uframes_t
snd_tlv320aic23b_pcm_pointer_capture(struct snd_pcm_substream *substream){
	struct snd_tlv320aic23b *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t pos;
	unsigned long bytes;

	bytes = ssc_readl(chip->ssc->regs, PDC_RPR) - (unsigned long)runtime->dma_addr;
	pos = bytes_to_frames(runtime, bytes);
	if (pos >= runtime->buffer_size)
		pos -= runtime->buffer_size;

	return pos;
}
//=======================================================================================================================================
static struct snd_pcm_ops tlv320aic23b_playback_ops = {
	.open		= snd_tlv320aic23b_pcm_open_playback,
	.close		= snd_tlv320aic23b_pcm_close_playback,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= snd_tlv320aic23b_pcm_hw_params,
	.hw_free	= snd_tlv320aic23b_pcm_hw_free,
	.prepare	= snd_tlv320aic23b_pcm_prepare_playback,
	.trigger	= snd_tlv320aic23b_pcm_trigger_playback,
	.pointer	= snd_tlv320aic23b_pcm_pointer_playback,
};
static struct snd_pcm_ops tlv320aic23b_capture_ops = {
	.open		= snd_tlv320aic23b_pcm_open_capture,
	.close		= snd_tlv320aic23b_pcm_close_capture,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= snd_tlv320aic23b_pcm_hw_params,
	.hw_free	= snd_tlv320aic23b_pcm_hw_free,
	.prepare	= snd_tlv320aic23b_pcm_prepare_capture,
	.trigger	= snd_tlv320aic23b_pcm_trigger_capture,
	.pointer	= snd_tlv320aic23b_pcm_pointer_capture,
};
//=======================================================================================================================================
static void snd_tlv320aic23b_pcm_free(struct snd_pcm *pcm){
	struct snd_tlv320aic23b *chip = snd_pcm_chip(pcm);
	if (chip->pcm) {
		snd_pcm_lib_preallocate_free_for_all(chip->pcm);
		chip->pcm = NULL;
	}
}
static int __devinit snd_tlv320aic23b_pcm_new(struct snd_tlv320aic23b *chip, int device){
	struct snd_pcm *pcm;
	int retval;

	retval = snd_pcm_new(chip->card, chip->card->shortname, device, 1, 1, &pcm);
	if (retval < 0)
		goto out;

	pcm->private_data = chip;
	pcm->private_free = snd_tlv320aic23b_pcm_free;
	pcm->info_flags = SNDRV_PCM_INFO_BLOCK_TRANSFER;
	strcpy(pcm->name, "tlv320aic23b");
	chip->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &tlv320aic23b_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &tlv320aic23b_capture_ops);

	retval = snd_pcm_lib_preallocate_pages_for_all(chip->pcm,
			SNDRV_DMA_TYPE_DEV, &chip->ssc->pdev->dev,
			64 * 1024, 64 * 1024);
out:
	return retval;
}
//=======================================================================================================================================
static irqreturn_t snd_tlv320aic23b_interrupt(int irq, void *dev_id){
	struct snd_tlv320aic23b *chip = dev_id;
	struct snd_pcm_runtime *runtime;
	u32 status, mask;
	int offset;
	int block_size;
	int next_period;
	int retval = IRQ_NONE;
	int count = 0;

	spin_lock(&chip->lock);

	status = ssc_readl(chip->ssc->regs, SR);
	mask = ssc_readl(chip->ssc->regs, IMR);

#ifdef	DEBUG
	printk("status=%x, mask=%x\n", status & (SSC_BIT(SR_ENDTX) | SSC_BIT(SR_ENDRX)), mask);
#endif

	if (status & mask & SSC_BIT(SR_ENDTX))
	{
#ifdef	DEBUG
		printk("END_TX in SR\n");
#endif
		count++;
		runtime = chip->substream_playback->runtime;

		chip->period_playback++;
		if (chip->period_playback == runtime->periods)
			chip->period_playback = 0;
		next_period = chip->period_playback + 1;
		if (next_period == runtime->periods)
			next_period = 0;

		block_size = frames_to_bytes(runtime, runtime->period_size);
		offset = block_size * next_period;

		ssc_writel(chip->ssc->regs, PDC_TNPR, (long)runtime->dma_addr + offset);
		ssc_writel(chip->ssc->regs, PDC_TNCR, runtime->period_size * 2);
		retval = IRQ_HANDLED;

		spin_unlock(&chip->lock);
		snd_pcm_period_elapsed(chip->substream_playback);
	}

	if (status & mask & SSC_BIT(SR_ENDRX))
	{
#ifdef	DEBUG
		printk("END_RX in SR\n");
#endif
		count++;
		runtime = chip->substream_capture->runtime;

		chip->period_capture++;
		if (chip->period_capture == runtime->periods)
			chip->period_capture = 0;
		next_period = chip->period_capture + 1;
		if (next_period == runtime->periods)
			next_period = 0;

		block_size  = frames_to_bytes(runtime, runtime->period_size);
		offset = block_size * next_period;

		ssc_writel(chip->ssc->regs, PDC_RNPR, (long)runtime->dma_addr + offset);
		ssc_writel(chip->ssc->regs, PDC_RNCR, runtime->period_size * 2);
		retval = IRQ_HANDLED;

		spin_unlock(&chip->lock);
		snd_pcm_period_elapsed(chip->substream_capture);
	}

	return retval;
}
//=======================================================================================================================================
static int snd_tlv320aic23b_stereo_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo){
	int mask = (kcontrol->private_value >> 24) & 0xff;

	if (mask == 1)
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = mask;

	return 0;
}
//=======================================================================================================================================
static int snd_tlv320aic23b_stereo_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol){
	struct snd_tlv320aic23b *chip = snd_kcontrol_chip(kcontrol);
	int left_reg = kcontrol->private_value & 0xff;
	int right_reg = (kcontrol->private_value >> 8) & 0xff;
	int shift_left = (kcontrol->private_value >> 16) & 0x07;
	int shift_right = (kcontrol->private_value >> 19) & 0x07;
	int mask = (kcontrol->private_value >> 24) & 0xff;
	int invert = (kcontrol->private_value >> 22) & 1;

	spin_lock_irq(&chip->lock);

	ucontrol->value.integer.value[0] = (chip->reg_image[left_reg] >> shift_left) & mask;
	ucontrol->value.integer.value[1] = (chip->reg_image[right_reg] >> shift_right) & mask;

	if (invert) {
		ucontrol->value.integer.value[0] = mask - ucontrol->value.integer.value[0];
		ucontrol->value.integer.value[1] = mask - ucontrol->value.integer.value[1];
	}

	spin_unlock_irq(&chip->lock);

	return 0;
}
//=======================================================================================================================================
static int snd_tlv320aic23b_stereo_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol){
	struct snd_tlv320aic23b *chip = snd_kcontrol_chip(kcontrol);
	int left_reg = kcontrol->private_value & 0xff;
	int right_reg = (kcontrol->private_value >> 8) & 0xff;
	int shift_left = (kcontrol->private_value >> 16) & 0x07;
	int shift_right = (kcontrol->private_value >> 19) & 0x07;
	int mask = (kcontrol->private_value >> 24) & 0xff;
	int invert = (kcontrol->private_value >> 22) & 1;
	int change;
	unsigned short val1, val2;

	val1 = ucontrol->value.integer.value[0] & mask;
	val2 = ucontrol->value.integer.value[1] & mask;
	if (invert) {
		val1 = mask - val1;
		val2 = mask - val2;
	}
	val1 <<= shift_left;
	val2 <<= shift_right;

	spin_lock_irq(&chip->lock);

	val1 = (chip->reg_image[left_reg] & ~(mask << shift_left)) | val1;
	val2 = (chip->reg_image[right_reg] & ~(mask << shift_right)) | val2;
	change = val1 != chip->reg_image[left_reg] || val2 != chip->reg_image[right_reg];


	snd_tlv320aic23b_write_reg(chip, left_reg, val1);
	snd_tlv320aic23b_write_reg(chip, right_reg, val2);

	spin_unlock_irq(&chip->lock);

	return change;
}
//=======================================================================================================================================
static int snd_tlv320aic23b_line_capture_volume_info(
		struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *uinfo){
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	// When inverted will give values 0x10001 => 0.
	uinfo->value.integer.min = 14;
	uinfo->value.integer.max = 31;

	return 0;
}
//=======================================================================================================================================
#define TLV320AIC23B_STEREO(xname, xindex, left_reg, right_reg, shift_left, shift_right, mask, invert) \
{									\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,				\
	.name = xname,							\
	.index = xindex,						\
	.info = snd_tlv320aic23b_stereo_info,				\
	.get = snd_tlv320aic23b_stereo_get,					\
	.put = snd_tlv320aic23b_stereo_put,					\
	.private_value = (left_reg | (right_reg << 8)			\
			| (shift_left << 16) | (shift_right << 19)	\
			| (mask << 24) | (invert << 22))		\
}
//=======================================================================================================================================
static struct snd_kcontrol_new snd_tlv320aic23b_controls[] __devinitdata = {
TLV320AIC23B_STEREO("Master Playback Volume", 0, 0x02, 0x03, 0, 0, 0x7F, 0),
{
	.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
	.name	= "Line Playback Volume",
	.index	= 0,
	.info	= snd_tlv320aic23b_line_capture_volume_info,
	.get	= snd_tlv320aic23b_stereo_get,
	.put	= snd_tlv320aic23b_stereo_put,
	.private_value	= (0x00 | (0x01 << 8) | (0x7F << 24) ),
},
{
	.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
	.name	= "Line Capture Volume",
	.index	= 0,
	.info	= snd_tlv320aic23b_line_capture_volume_info,
	.get	= snd_tlv320aic23b_stereo_get,
	.put	= snd_tlv320aic23b_stereo_put,
	.private_value	= (0x00 | (0x01 << 8) | (0x7F << 24) ),
},
};
//=======================================================================================================================================
static int __devinit snd_tlv320aic23b_mixer(struct snd_tlv320aic23b *chip){
	struct snd_card *card;
	int errval, idx;

	if (chip == NULL || chip->pcm == NULL)
		return -EINVAL;

	card = chip->card;

	strcpy(card->mixername, chip->pcm->name);

	for (idx = 0; idx < ARRAY_SIZE(snd_tlv320aic23b_controls); idx++) {
		errval = snd_ctl_add(card,
				snd_ctl_new1(&snd_tlv320aic23b_controls[idx],
					chip));
		if (errval < 0)
			goto cleanup;
	}

	return 0;

cleanup:
	for (idx = 1; idx < ARRAY_SIZE(snd_tlv320aic23b_controls) + 1; idx++) {
		struct snd_kcontrol *kctl;
		kctl = snd_ctl_find_numid(card, idx);
		if (kctl)
			snd_ctl_remove(card, kctl);
	}
	return errval;
}
//=======================================================================================================================================
/*  Device functions */
static int snd_tlv320aic23b_ssc_init(struct snd_tlv320aic23b *chip){

	/* Restart and disable SSC */
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_SWRST));
	ssc_writel(chip->ssc->regs, CMR, 0);
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_TXDIS) | SSC_BIT(CR_RXDIS));

	/*
	 * Continuous clock output at TK.
	 * Clock source is divided clock.
	 * Starts on falling TF.
	 * No clock inversion - write bit on falling TK.
	 * Delay 1 cycle (1 bit).
	 * Periode is 32 bit (32/2 - 1).
	 */
	ssc_writel(chip->ssc->regs, TCMR,
			SSC_BF(TCMR_CKO, 1)
			| SSC_BF(TCMR_CKS, 0)
			| SSC_BF(TCMR_CKI, 0)
			| SSC_BF(TCMR_START, 4)
			| SSC_BF(TCMR_STTDLY, 1)
			| SSC_BF(TCMR_PERIOD, 32/2 - 1));
	/*
	 * RK - input.
	 * Clock source is TK.
	 * Clock inversion - read bit on rising TK.
	 * Starts with transmit start.
	 * Delay 1 cycle (1 bit).
	 * Periode is 32 bit (32/2 - 1).
	 */
	ssc_writel(chip->ssc->regs, RCMR,
			SSC_BF(RCMR_CKO, 0)
			| SSC_BF(RCMR_CKS, 1)
			| SSC_BF(RCMR_CKI, 1)
			| SSC_BF(RCMR_START, 1)
			| SSC_BF(RCMR_STTDLY, 1)
			| SSC_BF(RCMR_PERIOD, 32/2 - 1));


	/*
	 * Data length is 16 bit (16 - 1).
	 * Transmit MSB first.
	 * Transmit 2 word each transfer.
	 * Frame sync length is 16 bit (16 - 1).
	 * Frame output is negative pulse.
	 */
	ssc_writel(chip->ssc->regs, TFMR,
			SSC_BF(TFMR_DATLEN, 16 - 1)
			| SSC_BIT(TFMR_MSBF)
			| SSC_BF(TFMR_DATNB, 2 - 1)
			| SSC_BF(TFMR_FSLEN, 16 - 1)
			| SSC_BF(TFMR_FSOS, 1));
	ssc_writel(chip->ssc->regs, RFMR,
			SSC_BF(RFMR_DATLEN, 16 - 1)
			| SSC_BIT(RFMR_MSBF)
			| SSC_BF(RFMR_DATNB, 2 - 1)
			| SSC_BF(RFMR_FSLEN, 0)
			| SSC_BF(RFMR_FSOS, 0));
	return 0;
}
//=======================================================================================================================================
static int snd_tlv320aic23b_chip_init(struct snd_tlv320aic23b *chip)
{
	int retval,a;

	retval = snd_tlv320aic23b_set_bitrate(chip);
	if (retval)
		return retval;

	/* Enable DAC master clock. */
	clk_enable(chip->board->dac_clk);

	snd_tlv320aic23b_write_reg(chip, 0x0F, 0x00);//RESET
	for(a=0;a<0x1000;a++)a=a;
	snd_tlv320aic23b_write_reg(chip, 0x0, 0x17);//L line in set gain +0 db
	snd_tlv320aic23b_write_reg(chip, 0x1, 0x17);//R line in set gain +0 db
	snd_tlv320aic23b_write_reg(chip, 0x2, 0xF9);//L head phone set gain +0 db
	snd_tlv320aic23b_write_reg(chip, 0x3, 0xF9);//R head phone set gain +0 db
	snd_tlv320aic23b_write_reg(chip, 0x4, 0x39);//analog audio path control, MIC not mute +20dB, lineside -6dB, line in select, bypass
//	snd_tlv320aic23b_write_reg(chip, 0x4, 0x3D);//analog audio path control, MIC not mute +20dB, lineside -6dB, mic in select, bypass
	snd_tlv320aic23b_write_reg(chip, 0x5, 0x00);//digital
	snd_tlv320aic23b_write_reg(chip, 0x6, 0x40|0x20);//turn off osc and clock
	snd_tlv320aic23b_write_reg(chip, 0x7, 0x02);//I2S, 16-bit, slave mode
	snd_tlv320aic23b_write_reg(chip, 0x8, 0x01 | (SAMPLINGRATE << 2));//DAC/ADC sample rate 48000, USB mode
	snd_tlv320aic23b_write_reg(chip, 0x9, 0x01);

	/* Enable I2S device, i.e. clock output. */
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_TXEN) | SSC_BIT(CR_RXEN));

	//!!! clk_disable(chip->board->dac_clk);

	return retval;
}
//=======================================================================================================================================
static int snd_tlv320aic23b_dev_free(struct snd_device *device){
	struct snd_tlv320aic23b *chip = device->device_data;

	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_TXDIS) | SSC_BIT(CR_RXDIS));
	if (chip->irq >= 0) {
		free_irq(chip->irq, chip);
		chip->irq = -1;
	}

	return 0;
}
//=======================================================================================================================================
static int __devinit snd_tlv320aic23b_dev_init(struct snd_card *card,
					 struct spi_device *spi){
	static struct snd_device_ops ops = {
		.dev_free	= snd_tlv320aic23b_dev_free,
	};
	struct snd_tlv320aic23b *chip = get_chip(card);
	int irq, retval;

	irq = chip->ssc->irq;
	if (irq < 0)
		return irq;

	spin_lock_init(&chip->lock);
	chip->card = card;
	chip->irq = -1;

	retval = request_irq(irq, snd_tlv320aic23b_interrupt, 0, "tlv320aic23b", chip);
	if (retval) {
		dev_dbg(&chip->spi->dev, "unable to request irq %d\n", irq);
		goto out;
	}
	chip->irq = irq;

	retval = snd_tlv320aic23b_ssc_init(chip);
	if (retval){
		printk(KERN_INFO "ERROR snd_tlv320aic23b_ssc_init\n");
		goto out_irq;
	}

	retval = snd_tlv320aic23b_chip_init(chip);
	if (retval){
		printk(KERN_INFO "ERROR snd_tlv320aic23b_chip_init\n");
		goto out_irq;
	}

	retval = snd_tlv320aic23b_pcm_new(chip, 0);
	if (retval){
		printk(KERN_INFO "ERROR snd_tlv320aic23b_pcm_new\n");
		goto out_irq;
	}

	retval = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (retval){
		printk(KERN_INFO "ERROR snd_device_new\n");
		goto out_irq;
	}

	retval = snd_tlv320aic23b_mixer(chip);
	if (retval){
		printk(KERN_INFO "ERROR snd_tlv320aic23b_mixer\n");
		goto out_snd_dev;
	}

	snd_card_set_dev(card, &spi->dev);

	goto out;

out_snd_dev:
	snd_device_free(card, chip);
out_irq:
	free_irq(chip->irq, chip);
	chip->irq = -1;
out:
	return retval;
}
//=======================================================================================================================================
static int snd_tlv320aic23b_probe(struct spi_device *spi){
	struct snd_card			*card;
	struct snd_tlv320aic23b		*chip;
	struct tlv320aic23b_board_info	*board;
	int				retval;
	char				id[16];

	board = spi->dev.platform_data;
	if (!board) {
		dev_dbg(&spi->dev, "no platform_data\n");
		return -ENXIO;
	}

	if (!board->dac_clk) {
		dev_dbg(&spi->dev, "no DAC clk\n");
		return -ENXIO;
	}

	if (IS_ERR(board->dac_clk)) {
		dev_dbg(&spi->dev, "no DAC clk\n");
		return PTR_ERR(board->dac_clk);
	}

	retval = -ENOMEM;

	/* Allocate "card" using some unused identifiers. */
	snprintf(id, sizeof id, "tlv320aic23b_%d", board->ssc_id);
	//card = snd_card_new(-1, id, THIS_MODULE, sizeof(struct snd_tlv320aic23b));
	//if (!card){
	//	printk(KERN_INFO " !card\n");
	//	goto out;
	//}
	retval = snd_card_create(-1, id, THIS_MODULE, sizeof(struct snd_tlv320aic23b), &card);
	if (retval < 0) {
	    printk(KERN_INFO, "tlv320 Card fail!\n");
	    goto out;
	}

	chip = card->private_data;
	chip->spi = spi;
	chip->board = board;

	chip->ssc = ssc_request(board->ssc_id);
	if (IS_ERR(chip->ssc)) {
		dev_dbg(&spi->dev, "could not get ssc%d device\n", board->ssc_id);
		retval = PTR_ERR(chip->ssc);
		goto out_card;
	}

	retval = snd_tlv320aic23b_dev_init(card, spi);
	if (retval){
		printk(KERN_INFO " snd_tlv320aic23b_dev_init ERROR\n");
		goto out_ssc;
	}

	strcpy(card->driver, "tlv320aic23b");
	strcpy(card->shortname, board->shortname);
	sprintf(card->longname, "%s on irq %d", card->shortname, chip->irq);

	retval = snd_card_register(card);
	if (retval){
		printk(KERN_INFO " snd_card_register(card); ERROR\n");
		goto out_ssc;
	}

	dev_set_drvdata(&spi->dev, card);

	goto out;

out_ssc:
	ssc_free(chip->ssc);
out_card:
	snd_card_free(card);
out:
	return retval;
}
//=======================================================================================================================================
static int __devexit snd_tlv320aic23b_remove(struct spi_device *spi){
	struct snd_card *card = dev_get_drvdata(&spi->dev);
	struct snd_tlv320aic23b *chip = card->private_data;

	/* Stop playback and capture. */
	ssc_writel(chip->ssc->regs, CR, SSC_BIT(CR_TXDIS) | SSC_BIT(CR_RXDIS));

	/* Stop DAC master clock. */
	clk_disable(chip->board->dac_clk);

	ssc_free(chip->ssc);
	snd_card_free(card);
	dev_set_drvdata(&spi->dev, NULL);

	return 0;
}
//=======================================================================================================================================
#define snd_tlv320aic23b_suspend NULL
#define snd_tlv320aic23b_resume NULL
//=======================================================================================================================================
static struct spi_driver tlv320aic23b_driver = {
	.driver		= {
		.name	= "tlv320aic23b",
	},
	.probe		= snd_tlv320aic23b_probe,
	.suspend	= snd_tlv320aic23b_suspend,
	.resume		= snd_tlv320aic23b_resume,
	.remove		= __devexit_p(snd_tlv320aic23b_remove),
};
//=======================================================================================================================================
static int __init tlv320aic23b_init(void){
	return spi_register_driver(&tlv320aic23b_driver);
}
module_init(tlv320aic23b_init);

static void __exit tlv320aic23b_exit(void){
	spi_unregister_driver(&tlv320aic23b_driver);
}
module_exit(tlv320aic23b_exit);

MODULE_DESCRIPTION("tlv320aic23b driver");
MODULE_AUTHOR("RW9UAO");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA, TLV320AIC23b sound codec}}");
