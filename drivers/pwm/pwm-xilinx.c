/**
  ******************************************************************************
  ******************************************************************************
  *
  * @file    pwm-xilinx.c
  * @author  R. Bush
  * @email   bush@krtkl.com
  * @version v1.0
  * @date    2016 April 17
  * @brief   Xilinx pwm driver for axi_timer IP 
  * @license GNU GPL v3
  *
  ******************************************************************************
  *
  * Copyright (c) krtkl inc., 2016
  * 
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * any later version.
  *   
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *  
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
  *
  ******************************************************************************
  */
  
#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>


/**
  * @defgroup xilinx_pwm_defines
  * @brief Xilinx PWM driver register definitions and macros
  * @{
  */
  
/*----------------------- Register offset definitions ------------------------*/
#define XPWM_TCSR0_OFFSET           0x00        /**< Timer 0 Control and Status Register */
#define XPWM_TLR0_OFFSET            0x04        /**< Timer 0 Load Register */
#define XPWM_TCR0_OFFSET            0x08        /**< Timer 0 Counter Register */

#define XPWM_TCSR1_OFFSET           0x10        /**< Timer 1 Control and Status Register */
#define XPWM_TLR1_OFFSET            0x14        /**< Timer 1 Load Register */
#define XPWM_TCR1_OFFSET            0x18        /**< Timer 1 Counter Register */

/*----------------------- TCSR Register bit definitions ----------------------*/
#define XPWM_TCSR_CASC              (1 << 11)   /**< Cascade enable */
#define XPWM_TCSR_ENALL             (1 << 10)   /**< Enable all */
#define XPWM_TSCR_PWMA              (1 << 9)    /**< PWM enable */
#define XPWM_TSCR_T0INT             (1 << 8)    /**< Timer interrupt */
#define XPWM_TSCR_ENT               (1 << 7)    /**< Enable timer 0 */
#define XPWM_TSCR_ENIT              (1 << 6)    /**< Enable interrupt */
#define XPWM_TSCR_LOAD              (1 << 5)    /**< Load timer with TLR0 */
#define XPWM_TSCR_ARHT              (1 << 4)    /**< Autoload/Hold */
#define XPWM_TSCR_CAPT              (1 << 3)    /**< External capture trigger enable */
#define XPWM_TSCR_GENT              (1 << 2)    /**< External generate signal enable */
#define XPWM_TSCR_UDT               (1 << 1)    /**< Up/Down count (0 = up, 1 = down) */
#define XPWM_TSCR_MDT               (1 << 0)    /**< Timer mode (0 = generate, 1 = capture) */

/**
  * @}
  */


/**
  * @defgroup xilinx_pwm_types
  * @{
  */

/**
  * struct xilinx_pwm_chip
  */
struct xilinx_pwm_chip {
    struct pwm_chip chip;
    struct clk *clk;
    void __iomem *mmio_base;
    int scaler;
    
    struct mutex isr_lock;
    
    void (*config)(struct pwm_chip *chip, struct pwm_device *pwm,
        unsigned long duty_ns, unsigned long period_ns);
    void (*set_enable)(struct pwm_chip *chip, bool enable);
};

/**
  * @}
  */


/**
  * @defgroup xilinx_pwm_io_functions
  * @{
  */

static unsigned int xilinx_pwm_readreg(void __iomem *addr)
{
#ifdef CONFIG_ARCH_ZYNQ
    return readl(addr);
#else
    return __raw_readl(addr);
#endif
}


static void xilinx_pwm_writereg(u32 val, void __iomem *addr)
{
#ifdef CONFIG_ARCH_ZYNQ
    writel(val, addr);
#else
    __raw_writel(val, addr);
#endif
}

/**
  * @}
  */


/**
  * @brief  to_xilinx_pwm_chip
  *
  * @param  chip:           PWM chip data
  * @retval pwm_chip structure
  */
static inline struct xilinx_pwm_chip *to_xilinx_pwm_chip(struct pwm_chip *chip)
{
    return container_of(chip, struct xilinx_pwm_chip, chip);
}



/**
  * @defgroup xilinx_pwm_driver_functions
  */

/**
  * @brief  xilinx_pwm_config - Configure the PWM device
  *
  * @param  chip:           PWM chip data
  * @param  pwm_device:     PWM device data
  * @param  duty_ns:        Duty cycle to initialize in nanoseconds
  * @param  period_ns:      PWM period in nanoseconds
  * @retval 0 on success, negative value otherwise
  */
static int xilinx_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
                int duty_ns, int period_ns)
{
    struct xilinx_pwm_chip *xilinx_pwm = to_xilinx_pwm_chip(chip);
    
    /* Write to necessary registers to set the duty cycle and period */
    
    /*
     *  Write the duty to timer 0 load register
     *  Write the period to timer 1 load register 
     */
    xilinx_pwm_writereg((duty_ns/xilinx_pwm->scaler) - 2, xilinx_pwm->mmio_base + XPWM_TLR1_OFFSET);
    xilinx_pwm_writereg((period_ns/xilinx_pwm->scaler) - 2, xilinx_pwm->mmio_base + XPWM_TLR0_OFFSET);

    return 0;    
}

/**
  * @brief  xilinx_pwm_set_polarity
  *
  * @param  chip:           PWM chip data
  * @param  pwm:            PWM device data
  * @param  polarity:       PWM polarity to set
  * @retval 0 on success, negative value otherwise
  */
static int xilinx_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
                enum pwm_polarity polarity)
{
    /* @note Changes in polarity are not supported by AXI timer IP after
     *       synthesis. As a result, this function does nothing, however always
     *       returns a successful retult
     */ 

    return 0;
}

/**
  * @brief  xilinx_pwm_enable - Enable the PWM interface
  *
  * @param  chip:           PWM chip data
  * @param  pwm:            PWM device data
  * @retval 0 on success, negative value otherwise
  */
static int xilinx_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
    struct xilinx_pwm_chip *xilinx_pwm = to_xilinx_pwm_chip(chip);
    unsigned long csr;
    
    csr = xilinx_pwm_readreg(xilinx_pwm->mmio_base + XPWM_TCSR0_OFFSET);
    /*
     *  Write to to enable all timers and enable the PWM generation
     */
    csr |= (XPWM_TCSR_ENALL | XPWM_TSCR_PWMA | XPWM_TSCR_GENT | XPWM_TSCR_UDT);
    csr &= ~(XPWM_TCSR_CASC | XPWM_TSCR_MDT | XPWM_TSCR_LOAD);
    xilinx_pwm_writereg(csr, xilinx_pwm->mmio_base + XPWM_TCSR0_OFFSET);

    return 0;
}


/**
  * @brief  xilinx_pwm_disable - Disable the PWM interface
  *
  * @param  chip:           PWM chip data
  * @param  pwm:            PWM device data
  * @retval None
  */
static void xilinx_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
    struct xilinx_pwm_chip *xilinx_pwm = to_xilinx_pwm_chip(chip);
    unsigned long csr = 0;
    
    /*
     * Write to disable all timers. This requires resetting the enable bits
     * on both timer control registers as re-setting the ENALL bit(s) will not
     * affect the enable bits for the individual timers.
     */
    xilinx_pwm_writereg(csr, xilinx_pwm->mmio_base + XPWM_TCSR0_OFFSET);
    xilinx_pwm_writereg(csr, xilinx_pwm->mmio_base + XPWM_TCSR1_OFFSET);
}

/**
  * @}
  */


static const struct pwm_ops xilinx_pwm_ops = {
    .config = xilinx_pwm_config,
    .set_polarity = xilinx_pwm_set_polarity,
    .enable = xilinx_pwm_enable,
    .disable = xilinx_pwm_disable,
    .owner = THIS_MODULE,
};


/**
  * @brief xilinx_pwm_probe - Probe method for the timer device.
  * @np: pointer to device tree node
  *
  * This function probes the timer device in the device tree. It initializes the
  * driver data structure.
  *
  * Return:
  * It returns 0, if the driver is bound to the timer device, or a negative
  * value if there is an error.
  */
static int xilinx_pwm_probe(struct platform_device *pdev)
{
    struct xilinx_pwm_chip *xilinx_pwm; 
    struct resource *r;
    int ret;
    unsigned long csr;
    
    xilinx_pwm = devm_kzalloc(&pdev->dev, sizeof(*xilinx_pwm), GFP_KERNEL);
    if (!xilinx_pwm)
        return -ENOMEM;
        
    /**
      * @todo   Set parameters for PWM signal from properties specified in 
      *         devicetree using of_property_read_u32()
      */
    
    xilinx_pwm->clk = devm_clk_get(&pdev->dev, NULL);
    if (IS_ERR(xilinx_pwm->clk)) {
        dev_err(&pdev->dev, "failure to find clock: %ld\n", 
                PTR_ERR(xilinx_pwm->clk));
        return PTR_ERR(xilinx_pwm->clk);
    }
    
    xilinx_pwm->scaler = (int)1000000000/clk_get_rate(xilinx_pwm->clk);
    
    xilinx_pwm->chip.ops = &xilinx_pwm_ops;
    xilinx_pwm->chip.dev = &pdev->dev;
    xilinx_pwm->chip.base = (int)&pdev->id;
    xilinx_pwm->chip.npwm = 1;
    xilinx_pwm->chip.can_sleep = true;
    
    r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    xilinx_pwm->mmio_base = devm_ioremap_resource(&pdev->dev, r);
    if (IS_ERR(xilinx_pwm->mmio_base))
        return PTR_ERR(xilinx_pwm->mmio_base);    
    
    /* Update timer state shadow register with default value */
    ret = pwmchip_add(&xilinx_pwm->chip);
    if (ret < 0) {
        dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
        return ret;
    }
    
    csr = (XPWM_TSCR_PWMA | XPWM_TSCR_GENT | XPWM_TSCR_UDT);
    
    xilinx_pwm_writereg(csr, xilinx_pwm->mmio_base + XPWM_TCSR0_OFFSET);
    xilinx_pwm_writereg(csr, xilinx_pwm->mmio_base + XPWM_TCSR1_OFFSET);

    platform_set_drvdata(pdev, xilinx_pwm);
    return 0;
}

static const struct of_device_id xilinx_pwm_of_match[] = {
    { .compatible = "xlnx,pwm-1.00", },
    { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, xilinx_pwm_of_match);

static struct platform_driver xilinx_pwm_driver = {
    .probe = xilinx_pwm_probe,
    .driver = {
        .name = "xilinx-pwm",
        .of_match_table = xilinx_pwm_of_match,
    },
};

module_platform_driver(xilinx_pwm_driver);

MODULE_AUTHOR("krtkl inc.");
MODULE_DESCRIPTION("Xilinx PWM driver");
MODULE_LICENSE("GPL v2.0");

