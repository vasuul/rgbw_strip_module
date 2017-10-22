#include <linux/module.h>
#include <linux/kernel.h>
//#include <linux/types.h>
#include <linux/interrupt.h>
//#include <linux/fcntl.h>
#include <linux/init.h>
//#include <linux/proc_fs.h>
//#include <linux/sysfs.h>
//#include <linux/mutex.h>
//#include <linux/sysctl.h>
//#include <linux/fs.h>
//#include <linux/cdev.h>
#include <linux/platform_device.h>
//#include <linux/slab.h>
//#include <linux/spi/spi.h>
//#include <asm/uaccess.h>
//#include <linux/semaphore.h>
//#include <linux/ioport.h>
//#include <asm/io.h>
#include <linux/delay.h>
#include <linux/clk.h>
//#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/dmaengine.h>

//#include <linux/clk-provider.h>
//#include <linux/err.h>
//#include <linux/wait.h>
//#include <linux/kthread.h>
//#include <linux/poll.h>
//#include <asm/gpio.h>
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maxwell Walter");
MODULE_DESCRIPTION("Driver for RGBW light strip using PWM");
#define DRIVER_NAME "RGBW_strip"

#include "rgbw_strip.h"

#define USE_DMA 0

struct rgbw_strip_platform_data {
  void __iomem *pwm_base;

#if USE_DMA
  struct dma_chan *dma_chan;
#endif

  struct clk *clk;

  uint32_t num_leds;
  uint32_t *leds;
};

static int rgbw_strip_remove(struct platform_device *dev) {
  printk(KERN_INFO "RGBW strip module removed\n");
  
  return 0;
}

#define CTL_OFFSET    0x00
#define PWM_EN_1      (1<<0)
#define MODE_SERIAL_1 (1<<1)
#define USE_FIFO_1    (1<<5)

#define RNG1_OFFSET   0x10
#define DAT1_OFFSET   0x14
#define FIFO_OFFSET   0x18
static int setup_strip(struct rgbw_strip_platform_data *pdata) {
  // Control register
  uint32_t ctl_reg = MODE_SERIAL_1 | USE_FIFO_1;
  uint32_t range_reg = 32;

  // Write the range and the control values we set up
  writel(range_reg, pdata->pwm_base + RNG1_OFFSET);
  writel(ctl_reg, pdata->pwm_base + CTL_OFFSET);

  // Then enable it after a small wait TODO: Is the wait needed?
  udelay(10);
  writel(ctl_reg | PWM_EN_1, pdata->pwm_base + CTL_OFFSET);

  return 0;
}

#if USE_DMA
static void rgbw_strip_dma_callback(void *arg) {

}

struct void rgbw_strip_dma_write(struct device *dev,
                                 struct rgbw_strip_platform_data *pdata) {
  int res;
  struct dma_async_tx_descriptor *txdesc;
  pdata->dma_buf = dma_map_single(pdata->dma_chan->dev,
                                  pdata->leds, pdata->num_leds * sizeof(uint32_t),
                                  DMA_TO_DEVICE);
  if(dma_mapping_error(pdata->dma_chan->dev, pdata->dma_buf)) {
    printk(KERN_ERR "Failed to map dma buffer\n");
    return -EINVAL;
  }

  txdesc = dmaengine_prep_slave_single(pdata->dma_chan, pdata->dma_buf, 
}
#endif
    
static int rgbw_strip_probe(struct platform_device *pdev) {
  int ret = 0;
  struct resource *addr = 0;

#if USE_DMA
  dma_addr_t phy_addr;
  struct dma_slave_config dma_sconfig;
#endif
  
  struct rgbw_strip_platform_data *pdata = NULL;
  struct device_node *np = pdev->dev.of_node;
  
  if(!of_have_populated_dt()) {
    dev_err(&pdev->dev, "Devicetree not populated!\n");
    return -ENOENT;
  }

  // Allocate space for our platorm data structure
  pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
  if(!pdata) return -ENOMEM;

  // Get the parameters we want from the device tree
  ret = of_property_read_u32(np, "num-leds", &pdata->num_leds);
  if(ret < 0) {
    printk(KERN_ERR "Could not get number of leds from device tree\n");
    pdata->num_leds = 32;
    ret = 0;
  }
  printk(KERN_INFO "Initialized with %d LEDS\n", pdata->num_leds);
  pdata->leds = devm_kzalloc(&pdev->dev, sizeof(uint32_t) * pdata->num_leds, GFP_KERNEL);
  if(!pdata->leds) {
    printk(KERN_ERR "Could not allocate space for %d leds\n", pdata->num_leds);
    return -ENOMEM;
  }
  
  // Get pointers to the PWM peripheral memory
  addr = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  pdata->pwm_base = devm_ioremap_resource(&pdev->dev, addr);
  if(IS_ERR(pdata->pwm_base)) {
    printk(KERN_ERR "Failed to remap PWM memory\n");
    return PTR_ERR(pdata->pwm_base);
  }

  #if USE_DMA
  // Get pointers to the DMA stuff we need
  pdata->dma_chan = dma_request_slave_channel(&pdev->dev, "fifo-dma");
  if(IS_ERR(pdata->dma_chan)) {
    printk(KERN_ERR "Could not get fifo dma channel\n");
    return PTR_ERR(pdata->dma_chan);
  }
  phy_addr = (dma_addr_t)addr.start;
  dma_sconfig_dst_addr = phy_addr + FIFO_OFFSET;
  dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
  dma_sconfig.dst_maxburst = 1;
  dma_sconfig.direction = DMA_MEM_TO_DEV;
  ret = dmaengine_slave_config(pdata->dma_chan, &dma_sconfig);
  if(ret < 0) {
    printk(KERN_ERR "DMA config failed: %d\n", ret);
    dma_release(pdata->dma_chan);
    return ret;
  }
  #endif

  setup_strip(pdata);
  
  // Get the clock and start it up (prepare and enable)
  pdata->clk = devm_clk_get(&pdev->dev, NULL);
  if(IS_ERR(pdata->clk)) {
    printk(KERN_ERR "Failed to get clock\n");
    return PTR_ERR(pdata->clk);
  }
  ret = clk_prepare_enable(pdata->clk);
  if(ret) return ret;

  // finally set the data and return
  platform_set_drvdata(pdev, pdata);
  printk(KERN_INFO "finished probing RGBW strip driver\n");  
  return ret;
}

static struct of_device_id rgbw_strip_of_match[] = {
  {.compatible = "xulne,rgbw-strip",},
  {},
};
static struct platform_driver rgbw_strip_platform_driver = {
  .probe = rgbw_strip_probe,
  .remove = rgbw_strip_remove,
  .driver = {
    .name = DRIVER_NAME,
    .owner = THIS_MODULE,
    .of_match_table = rgbw_strip_of_match,
  },
};
module_platform_driver(rgbw_strip_platform_driver);
