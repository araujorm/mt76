// SPDX-License-Identifier: ISC
/* Copyright (C) 2019 MediaTek Inc.
 *
 * Author: Ryder Lee <ryder.lee@mediatek.com>
 *         Felix Fietkau <nbd@nbd.name>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#include "mt7615.h"
#include "mac.h"

static const struct pci_device_id mt7615_pci_device_table[] = {
	{ PCI_DEVICE(0x14c3, 0x7615) },
	{ },
};

u32 mt7615_reg_map(struct mt7615_dev *dev, u32 addr)
{
	u32 base = addr & MT_MCU_PCIE_REMAP_2_BASE;
	u32 offset = addr & MT_MCU_PCIE_REMAP_2_OFFSET;

	mt76_wr(dev, MT_MCU_PCIE_REMAP_2, base);

	return MT_PCIE_REMAP_BASE_2 + offset;
}

static void
mt7615_rx_poll_complete(struct mt76_dev *mdev, enum mt76_rxq_id q)
{
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);

	mt7615_irq_enable(dev, MT_INT_RX_DONE(q));
}

static irqreturn_t mt7615_irq_handler(int irq, void *dev_instance)
{
	struct mt7615_dev *dev = dev_instance;
	u32 intr;

	intr = mt76_rr(dev, MT_INT_SOURCE_CSR);
	mt76_wr(dev, MT_INT_SOURCE_CSR, intr);

	if (!test_bit(MT76_STATE_INITIALIZED, &dev->mt76.state))
		return IRQ_NONE;

	intr &= dev->mt76.mmio.irqmask;

	if (intr & MT_INT_TX_DONE_ALL) {
		mt7615_irq_disable(dev, MT_INT_TX_DONE_ALL);
		napi_schedule(&dev->mt76.tx_napi);
	}

	if (intr & MT_INT_RX_DONE(0)) {
		mt7615_irq_disable(dev, MT_INT_RX_DONE(0));
		napi_schedule(&dev->mt76.napi[0]);
	}

	if (intr & MT_INT_RX_DONE(1)) {
		mt7615_irq_disable(dev, MT_INT_RX_DONE(1));
		napi_schedule(&dev->mt76.napi[1]);
	}

	return IRQ_HANDLED;
}

static int mt7615_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	static const struct mt76_driver_ops drv_ops = {
		/* txwi_size = txd size + txp size */
		.txwi_size = MT_TXD_SIZE + sizeof(struct mt7615_txp),
		.drv_flags = MT_DRV_TXWI_NO_FREE,
		.survey_flags = SURVEY_INFO_TIME_TX |
				SURVEY_INFO_TIME_RX |
				SURVEY_INFO_TIME_BSS_RX,
		.tx_prepare_skb = mt7615_tx_prepare_skb,
		.tx_complete_skb = mt7615_tx_complete_skb,
		.rx_skb = mt7615_queue_rx_skb,
		.rx_poll_complete = mt7615_rx_poll_complete,
		.sta_ps = mt7615_sta_ps,
		.sta_add = mt7615_mac_sta_add,
		.sta_remove = mt7615_mac_sta_remove,
		.update_survey = mt7615_update_channel,
	};
	struct mt7615_dev *dev;
	struct mt76_dev *mdev;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	ret = pcim_iomap_regions(pdev, BIT(0), pci_name(pdev));
	if (ret)
		return ret;

	pci_set_master(pdev);

	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	mdev = mt76_alloc_device(&pdev->dev, sizeof(*dev), &mt7615_ops,
				 &drv_ops);
	if (!mdev)
		return -ENOMEM;

	dev = container_of(mdev, struct mt7615_dev, mt76);
	mt76_mmio_init(&dev->mt76, pcim_iomap_table(pdev)[0]);

	mdev->rev = (mt76_rr(dev, MT_HW_CHIPID) << 16) |
		    (mt76_rr(dev, MT_HW_REV) & 0xff);
	dev_dbg(mdev->dev, "ASIC revision: %04x\n", mdev->rev);

	ret = devm_request_irq(mdev->dev, pdev->irq, mt7615_irq_handler,
			       IRQF_SHARED, KBUILD_MODNAME, dev);
	if (ret)
		goto error;

	ret = mt7615_register_device(dev);
	if (ret)
		goto error;

	return 0;
error:
	ieee80211_free_hw(mt76_hw(dev));
	return ret;
}

static void mt7615_pci_remove(struct pci_dev *pdev)
{
	struct mt76_dev *mdev = pci_get_drvdata(pdev);
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);

	mt7615_unregister_device(dev);
}

#ifdef CONFIG_PM
static int mt7615_pci_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct mt76_dev *mdev = pci_get_drvdata(pdev);
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);
	bool hif_suspend;
	int i, err;

	hif_suspend = !test_bit(MT76_STATE_SUSPEND, &dev->mphy.state) &&
		      mt7615_firmware_offload(dev);
	if (hif_suspend) {
		err = mt7615_mcu_set_hif_suspend(dev, true);
		if (err)
			return err;
	}

	napi_disable(&mdev->tx_napi);
	tasklet_kill(&mdev->tx_tasklet);

	mt76_for_each_q_rx(mdev, i) {
		napi_disable(&mdev->napi[i]);
	}
	tasklet_kill(&dev->irq_tasklet);

	mt7615_dma_reset(dev);

	err = mt7615_wait_pdma_busy(dev);
	if (err)
		goto restore;

	if (is_mt7663(mdev)) {
		mt76_set(dev, MT_PDMA_SLP_PROT, MT_PDMA_AXI_SLPPROT_ENABLE);
		if (!mt76_poll_msec(dev, MT_PDMA_SLP_PROT,
				    MT_PDMA_AXI_SLPPROT_RDY,
				    MT_PDMA_AXI_SLPPROT_RDY, 1000)) {
			dev_err(mdev->dev, "PDMA sleep protection failed\n");
			err = -EIO;
			goto restore;
		}
	}

	pci_enable_wake(pdev, pci_choose_state(pdev, state), true);
	pci_save_state(pdev);
	err = pci_set_power_state(pdev, pci_choose_state(pdev, state));
	if (err)
		goto restore;

	err = mt7615_firmware_own(dev);
	if (err)
		goto restore;

	return 0;

restore:
	mt76_for_each_q_rx(mdev, i) {
		napi_enable(&mdev->napi[i]);
	}
	napi_enable(&mdev->tx_napi);
	if (hif_suspend)
		mt7615_mcu_set_hif_suspend(dev, false);

	return err;
}

static int mt7615_pci_resume(struct pci_dev *pdev)
{
	struct mt76_dev *mdev = pci_get_drvdata(pdev);
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);
	bool pdma_reset;
	int i, err;

	err = mt7615_driver_own(dev);
	if (err < 0)
		return err;

	err = pci_set_power_state(pdev, PCI_D0);
	if (err)
		return err;

	pci_restore_state(pdev);

	if (is_mt7663(&dev->mt76)) {
		mt76_clear(dev, MT_PDMA_SLP_PROT, MT_PDMA_AXI_SLPPROT_ENABLE);
		mt76_wr(dev, MT_PCIE_IRQ_ENABLE, 1);
	}

	pdma_reset = !mt76_rr(dev, MT_WPDMA_TX_RING0_CTRL0) &&
		     !mt76_rr(dev, MT_WPDMA_TX_RING0_CTRL1);
	if (pdma_reset)
		dev_err(mdev->dev, "PDMA engine must be reinitialized\n");

	mt76_for_each_q_rx(mdev, i) {
		napi_enable(&mdev->napi[i]);
		napi_schedule(&mdev->napi[i]);
	}
	napi_enable(&mdev->tx_napi);
	napi_schedule(&mdev->tx_napi);

	if (!test_bit(MT76_STATE_SUSPEND, &dev->mphy.state) &&
	    mt7615_firmware_offload(dev))
		err = mt7615_mcu_set_hif_suspend(dev, false);

	return err;
}
#endif /* CONFIG_PM */

struct pci_driver mt7615_pci_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= mt7615_pci_device_table,
	.probe		= mt7615_pci_probe,
	.remove		= mt7615_pci_remove,
};

module_pci_driver(mt7615_pci_driver);

MODULE_DEVICE_TABLE(pci, mt7615_pci_device_table);
MODULE_FIRMWARE(MT7615_FIRMWARE_CR4);
MODULE_FIRMWARE(MT7615_FIRMWARE_N9);
MODULE_FIRMWARE(MT7615_ROM_PATCH);
MODULE_LICENSE("Dual BSD/GPL");
