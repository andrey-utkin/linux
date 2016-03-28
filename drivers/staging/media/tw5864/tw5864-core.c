/*
 *  TW5864 driver - core functions
 *
 *  Copyright (C) 2015 Bluecherry, LLC <maintainers@bluecherrydvr.com>
 *  Author: Andrey Utkin <andrey.utkin@corp.bluecherry.net>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/kmod.h>
#include <linux/sound.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pm.h>
#include <linux/pci_ids.h>
#include <linux/jiffies.h>
#include <asm/dma.h>
#include <media/v4l2-dev.h>

#include "tw5864.h"
#include "tw5864-reg.h"

MODULE_DESCRIPTION("V4L2 driver module for tw5864-based multimedia capture & encoding devices");
MODULE_AUTHOR("Bluecherry Maintainers <maintainers@bluecherrydvr.com>");
MODULE_AUTHOR("Andrey Utkin <andrey.utkin@corp.bluecherry.net>");
MODULE_LICENSE("GPL");

/* take first free /dev/videoX indexes by default */
static unsigned int video_nr[] = {[0 ... (TW5864_INPUTS - 1)] = -1 };

module_param_array(video_nr, int, NULL, 0444);
MODULE_PARM_DESC(video_nr, "video devices numbers array");

/*
 * Please add any new PCI IDs to: http://pci-ids.ucw.cz.  This keeps
 * the PCI ID database up to date.  Note that the entries must be
 * added under vendor 0x1797 (Techwell Inc.) as subsystem IDs.
 */
static const struct pci_device_id tw5864_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_TECHWELL, PCI_DEVICE_ID_TECHWELL_5864)},
	{0,}
};

void tw_indir_writeb(struct tw5864_dev *dev, u16 addr, u8 data)
{
	int retries = 30000;

	addr <<= 2;

	while ((tw_readl(TW5864_IND_CTL) >> 31) && (retries--))
		;
	if (!retries)
		dev_err(&dev->pci->dev,
			"tw_indir_writel() retries exhausted before writing\n");

	tw_writel(TW5864_IND_DATA, data);
	tw_writel(TW5864_IND_CTL, addr | TW5864_RW | TW5864_ENABLE);
}

u8 tw_indir_readb(struct tw5864_dev *dev, u16 addr)
{
	int retries = 30000;
	u32 data = 0;

	addr <<= 2;

	while ((tw_readl(TW5864_IND_CTL) >> 31) && (retries--))
		;
	if (!retries)
		dev_err(&dev->pci->dev,
			"tw_indir_readl() retries exhausted before reading\n");

	tw_writel(TW5864_IND_CTL, addr | TW5864_ENABLE);

	retries = 30000;
	while ((tw_readl(TW5864_IND_CTL) >> 31) && (retries--))
		;
	if (!retries)
		dev_err(&dev->pci->dev,
			"tw_indir_readl() retries exhausted at reading\n");

	data = tw_readl(TW5864_IND_DATA);
	return data & 0xff;
}

void tw5864_irqmask_apply(struct tw5864_dev *dev)
{
	tw_writel(TW5864_INTR_ENABLE_L, dev->irqmask & 0xffff);
	tw_writel(TW5864_INTR_ENABLE_H, (dev->irqmask >> 16));
}

static void tw5864_interrupts_disable(struct tw5864_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->slock, flags);
	dev->irqmask = 0;
	tw5864_irqmask_apply(dev);
	spin_unlock_irqrestore(&dev->slock, flags);
}

static void tw5864_timer_isr(struct tw5864_dev *dev);
static void tw5864_h264_isr(struct tw5864_dev *dev);

static irqreturn_t tw5864_isr(int irq, void *dev_id)
{
	struct tw5864_dev *dev = dev_id;
	u32 status;

	status = tw_readl(TW5864_INTR_STATUS_L)
		| (tw_readl(TW5864_INTR_STATUS_H) << 16);
	if (!status)
		return IRQ_NONE;

	tw_writel(TW5864_INTR_CLR_L, 0xffff);
	tw_writel(TW5864_INTR_CLR_H, 0xffff);

	if (status & TW5864_INTR_VLC_DONE) {
		tw5864_h264_isr(dev);
		tw_writel(TW5864_VLC_DSP_INTR, 0x00000001);
		tw_writel(TW5864_PCI_INTR_STATUS, TW5864_VLC_DONE_INTR);
	}

	if (status & TW5864_INTR_TIMER) {
		tw5864_timer_isr(dev);
		tw_writel(TW5864_PCI_INTR_STATUS, TW5864_TIMER_INTR);
	}

	if (!(status & (TW5864_INTR_TIMER | TW5864_INTR_VLC_DONE))) {
		dev_dbg(&dev->pci->dev, "Unknown interrupt, status 0x%08X\n",
			status);
	}

	return IRQ_HANDLED;
}

static void tw5864_h264_isr(struct tw5864_dev *dev)
{
	int channel = tw_readl(TW5864_DSP) & TW5864_DSP_ENC_CHN;
	struct tw5864_input *input = &dev->inputs[channel];
	int cur_frame_index, next_frame_index;
	struct tw5864_h264_frame *cur_frame, *next_frame;
	unsigned long flags;

	spin_lock_irqsave(&dev->slock, flags);

	cur_frame_index = dev->h264_buf_w_index;
	next_frame_index = (cur_frame_index + 1) % H264_BUF_CNT;
	cur_frame = &dev->h264_buf[cur_frame_index];
	next_frame = &dev->h264_buf[next_frame_index];

	dma_sync_single_for_cpu(&dev->pci->dev, cur_frame->vlc.dma_addr,
				H264_VLC_BUF_SIZE, DMA_FROM_DEVICE);
	dma_sync_single_for_cpu(&dev->pci->dev, cur_frame->mv.dma_addr,
				H264_MV_BUF_SIZE, DMA_FROM_DEVICE);

	if (next_frame_index != dev->h264_buf_r_index) {
		cur_frame->vlc_len = tw_readl(TW5864_VLC_LENGTH) << 2;
		cur_frame->checksum = tw_readl(TW5864_VLC_CRC_REG);
		cur_frame->input = input;
		cur_frame->timestamp = ktime_get_ns();

		dev->h264_buf_w_index = next_frame_index;
		tasklet_schedule(&dev->tasklet);

		cur_frame = next_frame;
	} else {
		dev_err(&dev->pci->dev,
			"Skipped frame on input %d because all buffers busy\n",
			channel);
	}

	dev->encoder_busy = 0;

	spin_unlock_irqrestore(&dev->slock, flags);

	input->frame_seqno++;

	dma_sync_single_for_device(&dev->pci->dev,
				   cur_frame->vlc.dma_addr,
				   H264_VLC_BUF_SIZE, DMA_FROM_DEVICE);
	dma_sync_single_for_device(&dev->pci->dev,
				   cur_frame->mv.dma_addr,
				   H264_MV_BUF_SIZE, DMA_FROM_DEVICE);

	tw_writel(TW5864_VLC_STREAM_BASE_ADDR, cur_frame->vlc.dma_addr);
	tw_writel(TW5864_MV_STREAM_BASE_ADDR, cur_frame->mv.dma_addr);
}

static void tw5864_input_deadline_update(struct tw5864_input *input)
{
	input->new_frame_deadline = jiffies + msecs_to_jiffies(1000);
}

static void tw5864_timer_isr(struct tw5864_dev *dev)
{
	unsigned long flags;
	int i;
	int encoder_busy;

	spin_lock_irqsave(&dev->slock, flags);
	encoder_busy = dev->encoder_busy;
	spin_unlock_irqrestore(&dev->slock, flags);

	if (encoder_busy)
		return;

	/*
	 * Traversing inputs in round-robin fashion, starting from next to the
	 * last processed one
	 */
	for (i = 0; i < TW5864_INPUTS; i++) {
		int next_input = (i + dev->next_i) % TW5864_INPUTS;
		struct tw5864_input *input = &dev->inputs[next_input];
		int raw_buf_id; /* id of internal buf with last raw frame */

		spin_lock_irqsave(&input->slock, flags);
		if (!input->enabled)
			goto next;

		raw_buf_id = tw_mask_shift_readl(TW5864_SENIF_ORG_FRM_PTR1, 0x3,
						 2 * input->input_number);

		/* Check if new raw frame is available */
		if (input->buf_id == raw_buf_id) {
			if (time_is_after_jiffies(input->new_frame_deadline)) {
				tw_mask_shift_writel(TW5864_ENC_BUF_PTR_REC1,
						0x3, 2 * input->input_number,
						input->buf_id + 3);
				tw5864_input_deadline_update(input);
			}
			goto next;
		}

		input->new_frame_deadline = jiffies + msecs_to_jiffies(1000);
		input->buf_id = raw_buf_id;
		spin_unlock_irqrestore(&input->slock, flags);

		spin_lock_irqsave(&dev->slock, flags);
		dev->encoder_busy = 1;
		spin_unlock_irqrestore(&dev->slock, flags);
		tw5864_request_encoded_frame(input);
		tw5864_input_deadline_update(input);
		break;
next:
		spin_unlock_irqrestore(&input->slock, flags);
		continue;
	}
}

static int tw5864_initdev(struct pci_dev *pci_dev,
			  const struct pci_device_id *pci_id)
{
	struct tw5864_dev *dev;
	int err;

	dev = devm_kzalloc(&pci_dev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	snprintf(dev->name, sizeof(dev->name), "tw5864:%s", pci_name(pci_dev));

	err = v4l2_device_register(&pci_dev->dev, &dev->v4l2_dev);
	if (err)
		goto v4l2_reg_fail;

	/* pci init */
	dev->pci = pci_dev;
	if (pci_enable_device(pci_dev)) {
		err = -EIO;
		goto pci_enable_fail;
	}

	pci_set_master(pci_dev);

	err = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(&dev->pci->dev, "32bit PCI DMA is not supported\n");
		goto req_mem_fail;
	}

	/* get mmio */
	if (!request_mem_region(pci_resource_start(pci_dev, 0),
				pci_resource_len(pci_dev, 0), dev->name)) {
		err = -EBUSY;
		dev_err(&dev->pci->dev, "can't get MMIO memory @ 0x%llx\n",
			(unsigned long long)pci_resource_start(pci_dev, 0));
		goto req_mem_fail;
	}
	dev->mmio = ioremap_nocache(pci_resource_start(pci_dev, 0),
				    pci_resource_len(pci_dev, 0));
	if (!dev->mmio) {
		err = -EIO;
		dev_err(&dev->pci->dev, "can't ioremap() MMIO memory\n");
		goto ioremap_fail;
	}

	spin_lock_init(&dev->slock);

	err = tw5864_video_init(dev, video_nr);
	if (err)
		goto video_init_fail;

	/* get irq */
	err = devm_request_irq(&pci_dev->dev, pci_dev->irq, tw5864_isr,
			       IRQF_SHARED, "tw5864", dev);
	if (err < 0) {
		dev_err(&dev->pci->dev, "can't get IRQ %d\n", pci_dev->irq);
		goto irq_req_fail;
	}

	return 0;

irq_req_fail:
	tw5864_video_fini(dev);
video_init_fail:
	iounmap(dev->mmio);
ioremap_fail:
	release_mem_region(pci_resource_start(pci_dev, 0),
			   pci_resource_len(pci_dev, 0));
req_mem_fail:
	pci_disable_device(pci_dev);
pci_enable_fail:
	v4l2_device_unregister(&dev->v4l2_dev);
v4l2_reg_fail:
	devm_kfree(&pci_dev->dev, dev);
	return err;
}

static void tw5864_finidev(struct pci_dev *pci_dev)
{
	struct v4l2_device *v4l2_dev = pci_get_drvdata(pci_dev);
	struct tw5864_dev *dev =
		container_of(v4l2_dev, struct tw5864_dev, v4l2_dev);

	/* shutdown subsystems */
	tw5864_interrupts_disable(dev);

	/* unregister */
	tw5864_video_fini(dev);

	/* release resources */
	iounmap(dev->mmio);
	release_mem_region(pci_resource_start(pci_dev, 0),
			   pci_resource_len(pci_dev, 0));

	v4l2_device_unregister(&dev->v4l2_dev);
	devm_kfree(&pci_dev->dev, dev);
}

static struct pci_driver tw5864_pci_driver = {
	.name = "tw5864",
	.id_table = tw5864_pci_tbl,
	.probe = tw5864_initdev,
	.remove = tw5864_finidev,
};

module_pci_driver(tw5864_pci_driver);
