#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/dmaengine.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maxwell Walter");
MODULE_DESCRIPTION("Driver for RGBW light strip using PWM");
#define DRIVER_NAME "RGBW_strip"

#include "rgbw_strip.h"

#define USE_DMA 0

struct rgbw_strip_platform_data {
  dev_t            number;
  char             sw_revision;
  char             sw_rev_major;
  char             sw_rev_minor;
  uint32_t         major;
  struct cdev      cdev;
  struct device   *device;
  struct device   *fdev;
  struct class    *class;
  void __iomem    *pwm_base;

#if USE_DMA
  struct dma_chan *dma_chan;
#endif

  struct clk      *clk;

  uint32_t         num_leds;
  uint32_t        *leds;
};
#define CTL_OFFSET    0x00
#define PWM_EN_1      (1<<0)
#define MODE_SERIAL_1 (1<<1)
#define REPEAT_1      (1<<2)
#define USE_FIFO_1    (1<<5)

#define STA_OFFSET    0x04
#define RNG1_OFFSET   0x10
#define DAT1_OFFSET   0x14
#define FIFO_OFFSET   0x18

// Each LED is 20 mA according to the documentation
//  And our power supply is 30 amp, so say 25 to be safe
//  We also have to power the pi and we want some margin
// TODO: Make the power supply a parameter to set in the device tree
//       That makes it easier to change
#define MA_LED 20
#define MAX_CURRENT_MA (25 * 1000)

static int send_pwm_buf(struct rgbw_strip_platform_data *dat) {
  // Basically just keep pumping data into the FIFO until there is no
  //  more data left
  int i, buf_len = (dat->num_leds * 4) + 1;
  uint32_t sr, arr_size, ma_used = 0;
  dat->leds[buf_len - 1] = 0; // Just to make sure

  // Check power consumption
  arr_size = dat->num_leds * 4;  
  for(i = 0; i < arr_size;) {
    if(dat->leds[i++] != bit_values[0]) ma_used += MA_LED;
  }
  if(ma_used > MAX_CURRENT_MA) {
    // No good
    printk(KERN_DEBUG "Too much current (%d > %d)", ma_used, MAX_CURRENT_MA);
    return -EINVAL;
  } else {
    printk(KERN_INFO "Current used %d", ma_used);
  }

  // Send it off it we are good
  for(i = 0; i < buf_len;) {
    sr = readl(dat->pwm_base + STA_OFFSET);
    if(sr & 0xC) {
      // This is an error...
      printk(KERN_ERR "Error in status register: 0x%X\n", sr);
      return -EIO;
    } else if(sr & 0x1) {
      // FIFO is full so we have to wait
      //  It takes ~1.2us per 4 bits, so ~9.6us per word and the FIFO is 16 deep
      //  Delay between 2 and 10 FIFO elements
      // For whatever reason this causes hiccups in the transmission so we just
      //  busy wait instead for now
      //usleep_range(10, 50);
    } else {
      // Put something in to the FIFO
      writel(dat->leds[i++], dat->pwm_base + FIFO_OFFSET);
    }
  }
}

// We keep this around and global so our file handlers have something
//  to find.
static struct rgbw_strip_platform_data g_dev;

static int rgbw_strip_open(struct inode *inode, struct file *fp) {
  fp->private_data = &g_dev;  
  return 0;
}

static int rgbw_strip_release(struct inode *inode, struct file *fp) {
    fp->private_data = NULL;
    return 0;
}

static uint8_t bit_values[] = {0x88, 0x8C, 0xC8, 0xCC};
// 0xC = 1100
// 0x8 = 1000
// LED 1 = 1100 = C
// LED 0 = 1000 = 8
static inline uint32_t make_pwm_bits(uint8_t val) {
  // MSB first
  uint32_t tmp =
    ((bit_values[(val>>0) & 0x3] << 0 ) |
     (bit_values[(val>>2) & 0x3] << 8 ) |
     (bit_values[(val>>4) & 0x3] << 16) |
     (bit_values[(val>>6) & 0x3] << 24));
  return tmp;
}

static long rgbw_strip_unlocked_ioctl(struct file *fp, unsigned int cmd, unsigned long arg) {
  long ret = -ENOTTY;
  int i, indx;
  struct rgbw_strip_platform_data *dat = fp->private_data;
    
  switch(cmd) {
  case RGBW_STRIP_GET_NUM_LEDS:
  {
    ret = copy_to_user((void*)arg, &dat->num_leds, sizeof(dat->num_leds));
    if(ret) ret = -EIO;
  }
  break;

  case RGBW_STRIP_RENDER_ALL:
  {
    rgbw_led_t *payload = devm_kmalloc(dat->device, sizeof(rgbw_led_t) * dat->num_leds, GFP_KERNEL);
    if(!payload) {
      ret = -ENOMEM;
      break;
    }

    ret = copy_from_user((void*)payload, (void*)arg, sizeof(rgbw_led_t) * dat->num_leds);
    if(ret) {
      devm_kfree(dat->device, payload);
      ret = -EIO;
      break;
    }
    
    // Then process the color data into PWM data
    indx = 0;
    for(i = 0; i < cmd->count; i++) {
      // R and G seem switched based on what is in the datasheet
      dat->leds[indx++] = make_pwm_bits(payload[i].g);
      dat->leds[indx++] = make_pwm_bits(payload[i].r);
      dat->leds[indx++] = make_pwm_bits(payload[i].b);
      dat->leds[indx++] = make_pwm_bits(payload[i].w);
    }
    devm_kfree(dat->device, payload);
  
    // Then send the buffer to the PWM
    send_pwm_buf(dat);
  }
  break;

  case RGBW_STRIP_RENDER:
  {
    rgbw_render_t *cmd = devm_kmalloc(dat->device, sizeof(rgbw_render_t), GFP_KERNEL);
    rgbw_led_t *payload = NULL;
    uint arr_start = 0, arr_end = 0, arr_size = 0;
    uint ma_used;
    if(!cmd) {
      ret = -ENOMEM;
      break;
    }

    // Get the offset and count
    ret = copy_from_user((void*)cmd, (void*)arg, sizeof(rgbw_render_t));
    if(ret) {
      devm_kfree(dat->device, cmd);
      ret = -EIO;
      break;
    }

    if(cmd->count > 0) {
      
      // Make sure it won't run us past the end of the buffer
      if(cmd->offset + cmd->count > dat->num_leds) {
        devm_kfree(dat->device, cmd);
        ret = -EINVAL;
        break;
      }
      
      payload = devm_kmalloc(dat->device, sizeof(rgbw_led_t) * cmd->count, GFP_KERNEL);
      if(!payload) {
        devm_kfree(dat->device, cmd);
        ret = -ENOMEM;
        break;      
      }
      
      ret = copy_from_user((void*)payload, (void*)arg + offsetof(rgbw_render_t, leds),
                           sizeof(rgbw_led_t) * cmd->count);
      if(ret) {
        devm_kfree(dat->device, cmd);
        devm_kfree(dat->device, payload);
        ret = -EIO;
        break;
      }

      // Then process the color data into PWM data
      arr_start = cmd->offset * 4;
      indx = arr_start;
      for(i = 0; i < cmd->count; i++) {
        // R and G seem switched based on what is in the datasheet
        dat->leds[indx++] = make_pwm_bits(payload[i].g);
        dat->leds[indx++] = make_pwm_bits(payload[i].r);
        dat->leds[indx++] = make_pwm_bits(payload[i].b);
        dat->leds[indx++] = make_pwm_bits(payload[i].w);
      }
      devm_kfree(dat->device, payload);
    }
    devm_kfree(dat->device, cmd);

    // Then send the buffer to the PWM
    send_pwm_buf(dat);
  }
  break;

  default:
    ret = -ENODEV;
    break;
  }
  
  return ret;
}

static struct file_operations rgbw_strip_fops = {
  .owner          = THIS_MODULE,
  //.llseek = NULL,
  //.read           = rgbw_strip_read,
  //.write          = rgbw_strip.write,
  .unlocked_ioctl = rgbw_strip_unlocked_ioctl,
  .open           = rgbw_strip_open,
  .release        = rgbw_strip_release,
};

static void unsetup_dev(struct rgbw_strip_platform_data *dev) {
  printk(KERN_INFO "Unsetting up RGBW\n");
  
  // Destroy the device which destroys the device node
  device_destroy(dev->class, dev->number);

  // delete the cdev to unmap stuff
  cdev_del(&dev->cdev);

  // and destroy the class
  class_destroy(dev->class);

  // Finally give our major/minor back to the kernel
  unregister_chrdev_region(dev->number, 1);
}

static void unsetup_strip(struct rgbw_strip_platform_data *dev) {
  // Disable the PWM
  writel(0, dev->pwm_base + CTL_OFFSET);
}

static int rgbw_strip_remove(struct platform_device *dev) {
  printk(KERN_INFO "RGBW strip module removed\n");

  unsetup_dev(&g_dev);
  unsetup_strip(&g_dev);
  clk_disable_unprepare(g_dev.clk);

  // device resources released automagically
  
  return 0;
}

static int setup_strip(struct rgbw_strip_platform_data *pdata) {
  // Control register
  uint32_t ctl_reg = MODE_SERIAL_1 | REPEAT_1 | USE_FIFO_1;
  uint32_t range_reg = 32;
  
  // Write the range and the control values we set up
  writel(0, pdata->pwm_base + FIFO_OFFSET);
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

  txdesc = dmaengine_prep_slave_single(pdata->dma_chan, pdata->dma_buf);
}
#endif
  
static int setup_dev(struct rgbw_strip_platform_data *dev) {
  int ret = 0;
  unsigned int major = 0;
  
  // request one chardev device starting at 0
  ret = alloc_chrdev_region(&dev->number, 0, 1, DRIVER_NAME);
  if(ret < 0) {
    printk(KERN_ERR "Could not allocate chardev_region: %d\n", ret);
    goto error_alloc;
  }
  major = MAJOR(dev->number);
  
  // Create the device class
  dev->class = class_create(THIS_MODULE, DRIVER_NAME);
  if(IS_ERR(dev->class)) {
    ret = PTR_ERR(dev->class);
    printk(KERN_ERR "Could not create class: %d\n", ret);
    goto error_class_create;
  }

  //Setup up cdev 
  cdev_init(&dev->cdev, &rgbw_strip_fops);
  dev->cdev.owner = THIS_MODULE;
  dev->cdev.ops = &rgbw_strip_fops;

  // Only one minor device
  if((ret = cdev_add(&dev->cdev, dev->number, 1))<0  ) {
      printk(KERN_ERR "could not add %s, error: %d\n",
	      DRIVER_NAME, ret);
      goto error_cdev_add;
  } else {
    printk(KERN_ALERT "%s , Major/Minor #: %d:%d\n",
	    DRIVER_NAME, MAJOR(dev->number), MINOR(dev->number));
  }
  
  // Create the device node
  dev->fdev = device_create(dev->class, NULL, dev->number, NULL, "rgbws");
  if(dev->fdev == NULL) {
    printk(KERN_ERR "Failed to create device node\n");
    goto error_create;
  }
    
  return ret;

 error_create:
  cdev_del(&dev->cdev);
 error_cdev_add:
  class_destroy(dev->class);
 error_class_create:
  unregister_chrdev_region(dev->number, 1);
 error_alloc:
  return ret;
}

static int rgbw_strip_probe(struct platform_device *pdev) {
  int ret = 0, i, indx;
  struct resource *addr = 0;  
  uint32_t rc[5], num_lit = 0;
#if USE_DMA
  dma_addr_t phy_addr;
  struct dma_slave_config dma_sconfig;
#endif
  
  struct device_node *np = pdev->dev.of_node;
  
  if(!of_have_populated_dt()) {
    dev_err(&pdev->dev, "Devicetree not populated!\n");
    return -ENOENT;
  }
  g_dev.device = &pdev->dev;

  // Get the clock and start it up (prepare and enable)
  g_dev.clk = devm_clk_get(&pdev->dev, NULL);
  if(IS_ERR(g_dev.clk)) {
    printk(KERN_ERR "Failed to get clock\n");
    return PTR_ERR(g_dev.clk);
  }
  ret = clk_prepare_enable(g_dev.clk);
  if(ret) return ret;

  // Get the parameters we want from the device tree
  ret = of_property_read_u32(np, "num-leds", &g_dev.num_leds);
  if(ret < 0) {
    printk(KERN_ERR "Could not get number of leds from device tree\n");
    g_dev.num_leds = 32;
    ret = 0;
  }
  ret = of_property_read_u32_array(np, "reset-color", rc, 5);
  if(ret < 0) {
    printk(KERN_ERR "Could not get reset color\n");
    rc[0] = rc[1] = rc[2] = rc[3] = rc[4] = 0;
    ret = 0;
  }
  printk(KERN_INFO "Initialized with %d of %d LEDS color(%u %u %u %u)\n", rc[4], g_dev.num_leds, rc[0], rc[1], rc[2], rc[3]);

  num_lit = 0;
  if(rc[0] != 0) num_lit++;
  if(rc[1] != 0) num_lit++;
  if(rc[2] != 0) num_lit++;
  if(rc[3] != 0) num_lit++;
  if(num_lit * rc[4] * MA_LED > MAX_CURRENT_MA) {
    printk(KERN_WARNING "Default LEDS requre too much current (%d > %d)", num_lit * rc[4] * MA_LED, MAX_CURRENT_MA);
    rc[0] = rc[1] = rc[2] = rc[3] = rc[4] = 0;    
  }
  printk(KERN_INFO "Estimated default current usage: %dmA of %dMa", num_lit * rc[4] * MA_LED, MAX_CURRENT_MA);
      
  // Allocate enough space for +1 so that the last byte can always be zero
  //  *4 here because every bit in the led color takes 4 bits in pwm
  g_dev.leds = devm_kzalloc(&pdev->dev, sizeof(uint32_t) * (g_dev.num_leds * 4) + 1, GFP_KERNEL);
  if(!g_dev.leds) {
    printk(KERN_ERR "Could not allocate space for %d leds\n", g_dev.num_leds);
    return -ENOMEM;
  }
  indx = 0;
  for(i = 0; i < g_dev.num_leds; i++) {
    if(i < rc[4]) {
      g_dev.leds[indx++] = make_pwm_bits(rc[0]);
      g_dev.leds[indx++] = make_pwm_bits(rc[1]);
      g_dev.leds[indx++] = make_pwm_bits(rc[2]);
      g_dev.leds[indx++] = make_pwm_bits(rc[3]);
    } else {
      g_dev.leds[indx++] = make_pwm_bits(0);
      g_dev.leds[indx++] = make_pwm_bits(0);
      g_dev.leds[indx++] = make_pwm_bits(0);
      g_dev.leds[indx++] = make_pwm_bits(0);
    }
  }
  // do the last one which needs to be zero
  g_dev.leds[indx++] = 0;

  // Get pointers to the PWM peripheral memory
  addr = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  g_dev.pwm_base = devm_ioremap_resource(&pdev->dev, addr);
  if(IS_ERR(g_dev.pwm_base)) {
    printk(KERN_ERR "Failed to remap PWM memory\n");
    return PTR_ERR(g_dev.pwm_base);
  }

#if USE_DMA
  // Get pointers to the DMA stuff we need
  g_dev.dma_chan = dma_request_slave_channel(&pdev->dev, "fifo-dma");
  if(IS_ERR(g_dev.dma_chan)) {
    printk(KERN_ERR "Could not get fifo dma channel\n");
    return PTR_ERR(g_dev.dma_chan);
  }
  phy_addr = (dma_addr_t)addr.start;
  dma_sconfig_dst_addr = phy_addr + FIFO_OFFSET;
  dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
  dma_sconfig.dst_maxburst = 1;
  dma_sconfig.direction = DMA_MEM_TO_DEV;
  ret = dmaengine_slave_config(g_dev.dma_chan, &dma_sconfig);
  if(ret < 0) {
    printk(KERN_ERR "DMA config failed: %d\n", ret);
    dma_release(g_dev.dma_chan);
    return ret;
  }
#endif

  // Initialize the hardware
  setup_strip(&g_dev);

  // Setup our device node
  setup_dev(&g_dev);
  
  // finally set the data and return
  platform_set_drvdata(pdev, &g_dev);

  send_pwm_buf(&g_dev);
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
