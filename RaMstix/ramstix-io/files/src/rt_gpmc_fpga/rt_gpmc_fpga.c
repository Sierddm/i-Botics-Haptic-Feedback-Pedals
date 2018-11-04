/**
 * @file rt_gpmc_fpga.c
 * @brief Real-Time General Purpose Memory Controller driver for usage on the RaMStix board.
 * @author Jan Jaap Kempenaar, University of Twente
 * @author Jos Vos, University of Twente
 * @author Wilbert van de Ridder, University of Twente
 */

#include "rt_gpmc_fpga.h"
#include "gpmc_fpga_user.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("J.S. Vos, Jan Jaap Kempenaar, Wilbert van de Ridder");
MODULE_DESCRIPTION("Access FPGA hardware on the RaMstix baseboard for Overo modules.");

// @todo variables/internals.
static void __iomem *gpmc_fpga_base;
static void __iomem *gpmc_fpga_cs6_base = 0;
unsigned long cs6_mem_base;

static struct class *rt_gpmc_fpga_class;
static struct device *rt_gpmc_fpga_device;
static int rt_gpmc_fpga_major;

/**
 * @brief RTDM device structure according to Xenomai 3 RTDM
 *
 */
static struct rtdm_driver gpmc_fpga_driver = {
        .profile_info           =       RTDM_PROFILE_INFO(rt_gpmc_fpga,
                                                          RTDM_CLASS_EXPERIMENTAL,
                                                          DEVICE_SUBCLASS,
                                                          42),
        .device_flags           =       RTDM_NAMED_DEVICE|RTDM_EXCLUSIVE,
        .device_count           =       1,
        .ops = {
                .open           =       nrt_gpmc_fpga_open,
                .close          =	nrt_gpmc_fpga_close,
                .ioctl_rt       =       rt_gpmc_fpga_ioctl,
		.ioctl_nrt	= 	rt_gpmc_fpga_ioctl,
        },
};

static struct rtdm_device gpmc_fgpa_device = {
    .driver = &gpmc_fpga_driver,
    .label = "rt_gpmc_fpga",
};

u32 read32_from_fpga(int offset)
{
    return __raw_readl(gpmc_fpga_cs6_base + offset * 2);
}

void write32_to_fpga(int offset, u32 val)
{
    __raw_writel(val, gpmc_fpga_cs6_base + offset * 2);
}

u16 read16_from_fpga(int offset)
{
    return __raw_readw(gpmc_fpga_cs6_base + offset * 2);
}

void write16_to_fpga(int offset, u16 val)
{
    __raw_writew(val, gpmc_fpga_cs6_base + offset * 2);
}

static void rt_gpmc_write_reg(int idx, u32 val)
{
    __raw_writel(val, gpmc_fpga_base + idx);
}

void rt_gpmc_cs_write_reg(int cs, int idx, u32 val)
{
    void __iomem *reg_addr;

    reg_addr = gpmc_fpga_base + GPMC_CS0_OFFSET + (cs * GPMC_CS_SIZE) + idx;
    __raw_writel(val, reg_addr);
}

static int nrt_gpmc_fpga_open(struct rtdm_fd *fd, int oflags)
{
    TRACE_OBJECT(__FUNCTION__);

    return rt_gpmc_valid_fpga();
}

static void nrt_gpmc_fpga_close(struct rtdm_fd *fd)
{
    TRACE_OBJECT(__FUNCTION__);

    // Make sure next time we have a VALID check:
    write16_to_fpga(VALIDATE_REG1,0x0);
    write16_to_fpga(VALIDATE_REG2,0x0);
}

static int nrt_gpmc_fpga_ioctl(struct rtdm_fd *fd, unsigned int request, void __user *arg) 
{
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: This RT driver does not support calls from NRT userspace.\n");
    return -ENOSYS;
}

static int rt_gpmc_fpga_ioctl(struct rtdm_fd *fd, unsigned int request, void __user *arg)
{
    TRACE_OBJECT(__FUNCTION__);
    // DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: rt_gpmc_fpga_ioctl(%p,%u,%lu)\n", file, ioctl_num, ioctl_param);

    //DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: context.context_flags = 0x%08lx\n", context->context_flags);
    //DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: context.fd = 0x%04x\n", context->fd);
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: request = 0x%04x", request);
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg: 0x%p\n", arg);
    switch (request)
    {
        case IOCTL_SET_U64:
            rtdm_printk(KERN_INFO "rt_gpmc_fpga: 64 write action is not supported.\n");
            /*
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->offset = %i\n", ((struct gpmc_fpga_data64*)arg)->offset);
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Setting value(64).\n");
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->data = %li\n", ((struct gpmc_fpga_data64*)arg)->data.longval);
            if ((((struct gpmc_fpga_data64 *) arg)->offset % 2))
            {
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Uneven offset value.\n");
            write16_to_fpga(((struct gpmc_fpga_data64 *) arg)->offset, (((struct gpmc_fpga_data64 *) arg)->data.shorts[0]));
            write16_to_fpga(((struct gpmc_fpga_data64 *) arg)->offset+1, (((struct gpmc_fpga_data64 *) arg)->data.shorts[1]));
            write16_to_fpga(((struct gpmc_fpga_data64 *) arg)->offset+2, (((struct gpmc_fpga_data64 *) arg)->data.shorts[2]));
            write16_to_fpga(((struct gpmc_fpga_data64 *) arg)->offset+3, (((struct gpmc_fpga_data64 *) arg)->data.shorts[3]));
            }
            else
            {
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Even offset value.\n");
            write32_to_fpga(((struct gpmc_fpga_data64 *) arg)->offset ,((struct gpmc_fpga_data64 *) arg)->data.ints[0]);
            write32_to_fpga(((struct gpmc_fpga_data64 *) arg)->offset+2 ,(((struct gpmc_fpga_data64 *) arg)->data.ints[1]));
            }
            */
            break;

        case IOCTL_GET_U64:
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->offset = %i\n", ((struct gpmc_fpga_data64*)arg)->offset);
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Reading value(64).\n");
            ((struct gpmc_fpga_data64 *) arg)->data.ints[0] = (read32_from_fpga(
            ((struct gpmc_fpga_data64 *) arg)->offset + 2));
            ((struct gpmc_fpga_data64 *) arg)->data.ints[1] = (read32_from_fpga(
            ((struct gpmc_fpga_data64 *) arg)->offset));
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->data = %i\n", ((struct gpmc_fpga_data*)arg)->data);
            break;

        case IOCTL_SET_U32:
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->offset = %i\n", ((struct gpmc_fpga_data*)arg)->offset);
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Setting value(32).\n");
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->data = %i\n", ((struct gpmc_fpga_data*)arg)->data);
            if ((((struct gpmc_fpga_data *) arg)->offset % 2))
            {
                DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Uneven offset value.\n");
                write16_to_fpga(((struct gpmc_fpga_data *) arg)->offset,
                (((struct gpmc_fpga_data *) arg)->data));
                write16_to_fpga(((struct gpmc_fpga_data *) arg)->offset + 1,
                ((struct gpmc_fpga_data *) arg)->data >> 16);
            }
            else
            {
                DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Even offset value.\n");
                write32_to_fpga(((struct gpmc_fpga_data *) arg)->offset, ((struct gpmc_fpga_data *) arg)->data);
            }
            break;

        case IOCTL_GET_U32:
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->offset = %i\n", ((struct gpmc_fpga_data*)arg)->offset);
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Reading value(32).\n");
            ((struct gpmc_fpga_data *) arg)->data = read32_from_fpga(((struct gpmc_fpga_data *) arg)->offset);
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->data = %i\n", ((struct gpmc_fpga_data*)arg)->data);
            break;

        case IOCTL_SET_U16:
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->offset = %i\n", ((struct gpmc_fpga_data*)arg)->offset);
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Setting value(16).\n");
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->data = %i\n", ((struct gpmc_fpga_data*)arg)->data);
            write16_to_fpga(((struct gpmc_fpga_data *) arg)->offset,
            ((struct gpmc_fpga_data *) arg)->data);
            break;

        case IOCTL_GET_U16:
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->offset = %i\n", ((struct gpmc_fpga_data*)arg)->offset);
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Reading value(16).\n");
            ((struct gpmc_fpga_data *) arg)->data = read16_from_fpga(((struct gpmc_fpga_data *) arg)->offset);
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: arg->data = %i\n", ((struct gpmc_fpga_data*)arg)->data);
            break;

        default:
            DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Default Action Performed.\n");
            return -1;
            break;
    }

    return SUCCESS;
}

int __init rt_gpmc_fpga_init(void)
{
    int ret;
    void* ptr_err;
    TRACE_OBJECT(__FUNCTION__);
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: DEBUG_MODULE = on\n");
    rt_gpmc_fpga_major = rtdm_dev_register(&gpmc_fgpa_device);

    if (0 > rt_gpmc_fpga_major)
    {
        rtdm_printk(KERN_ALERT "rt_gpmc_fpga: Registering failed, err: %i\n", rt_gpmc_fpga_major);
        return rt_gpmc_fpga_major;
    }

    // Map base address of the GPMC:
    gpmc_fpga_base = ioremap(OMAP34XX_GPMC_BASE, SZ_4K);
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: gpmc_fpga_base = %p", gpmc_fpga_base);
    rt_gpmc_write_reg(GPMC_SYSCONFIG, 0x00);
    rt_gpmc_write_reg(GPMC_CONFIG, 0x12);
    rt_gpmc_cs_write_reg(6, GPMC_CS_CONFIG1, 0x00001000); // Turn off all clocking and such.
    //  rt_gpmc_cs_write_reg(6, GPMC_CS_CONFIG1,0x28001000); // This is with clocks.
    if (gpmc_cs_request(6, SZ_16M, &cs6_mem_base) < 0)
    {
        rtdm_printk(KERN_ALERT "rt_gpmc_fgpa: Failed to request GPMC mem for cs6n\n");
        return -1;
    }
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: cs6_mem_base:%lx\n",cs6_mem_base);
    gpmc_fpga_cs6_base = ioremap(cs6_mem_base, SZ_16M);
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: gpmc_fpga_cs6_base: %p\n", gpmc_fpga_cs6_base);
    rtdm_printk(KERN_INFO "rt_gpmc_fpga: Loaded module.\n");
    //printk(KERN_INFO "run the next statement to create the device\n");
    //rtdm_printk(KERN_INFO "rt_gpmc_fpga: mknod /dev/%s c %d 0\n", DEVICE_NAME_RT, GPMC_FPGA_MAJOR);
    // before validation, create a device.
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Create a device class and file.\n");

    rt_gpmc_fpga_class = class_create(THIS_MODULE, DEVICE_NAME_RT);
    if (IS_ERR(ptr_err = rt_gpmc_fpga_class))
    {
        printk(KERN_ALERT "rt_gpmc_fpga: Failed to create class.\n");
        class_destroy(rt_gpmc_fpga_class);
    }
    else
    {
        rt_gpmc_fpga_device = device_create(rt_gpmc_fpga_class, NULL, MKDEV(rt_gpmc_fpga_major,0), NULL, DEVICE_NAME_RT);
        if (IS_ERR(ptr_err = rt_gpmc_fpga_device))
        {
            printk(KERN_ALERT "rt_gpmc_fpga: Failed to create device.\n");
            class_destroy(rt_gpmc_fpga_class);
        }
    }
    
#if !defined(CONFIG_LEDS_GPIO) && !defined(CONFIG_LEDS_GPIO_MODULE)
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Obtain and configure LEDs.\n");
    rtdm_printk(KERN_INFO "rt_gpmc_fpga: Setting leds... \n");
    if (0 == gpio_request(22, "gpmc_green"))
    {
        DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Obtained LED 22\n");
        gpio_direction_output(22, LED_OFF);
    }
    if (0 == gpio_request(21, "gpmc_green"))
    {
        DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: Obtained LED 21\n");
        gpio_direction_output(21, LED_OFF);
    }
#endif
    ret = rt_gpmc_valid_fpga();
    if (0 != ret)
    {
        // Keep the module loaded.
    }
    else
    {
        ret = read16_from_fpga(VALIDATE_VERSION);
        DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: version return: %x\n", ret);
        rtdm_printk(KERN_INFO "rt_gpmc_fpga: FPGA hardware version: %i.%i.%i\n", GET_MAJOR(ret), GET_MINOR(ret), GET_BUILD(ret));
    }
    return 0;
}

static int rt_gpmc_valid_fpga(void)
{
    int ret;
    // initially should be zero.
    ret = read16_from_fpga(VALIDATE_REG1);
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: ret: 0x%x, expect 0x0\n", ret);
    if (ret != 0x0)
    {
        rtdm_printk(KERN_ALERT "rt_gpmc_fpga: FPGA appears to not contain the proper application.\n");
#if !defined(CONFIG_LEDS_GPIO) && !defined(CONFIG_LEDS_GPIO_MODULE)
        gpio_direction_output(21, LED_OFF);
        gpio_direction_output(22, LED_ON);
#endif
        return -1;
    }
    write16_to_fpga(VALIDATE_REG1,0x5566);
    write16_to_fpga(VALIDATE_REG2,0x1122);
    ret = read16_from_fpga(VALIDATE_REG1);
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: (7)ret: 0x%x, expect 0x5566\n", ret);
    if (ret != 0x5566)
    {
        rtdm_printk(KERN_ALERT "rt_gpmc_fpga: FPGA appears to not contain the proper application.\n");
#if !defined(CONFIG_LEDS_GPIO) && !defined(CONFIG_LEDS_GPIO_MODULE)
        gpio_direction_output(21, LED_OFF);
        gpio_direction_output(22, LED_ON);
#endif
        return -1;
    }
    
    
    ret = read16_from_fpga(VALIDATE_REG2);
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: (11)ret: 0x%x, expect 0x1122\n", ret);
    if (ret != 0x1122)
    {
        rtdm_printk(KERN_ALERT "rt_gpmc_fpga: FPGA appears to not contain the proper application.\n");
#if !defined(CONFIG_LEDS_GPIO) && !defined(CONFIG_LEDS_GPIO_MODULE)
        gpio_direction_output(21, LED_OFF);
        gpio_direction_output(22, LED_ON);
#endif
        return -1;
    }
    write16_to_fpga(VALIDATE_REG1,0x0);
    write16_to_fpga(VALIDATE_REG2,0x0);
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: VALID\n");
#if !defined(CONFIG_LEDS_GPIO) && !defined(CONFIG_LEDS_GPIO_MODULE)
    gpio_direction_output(21, LED_ON);
    gpio_direction_output(22, LED_OFF);
#endif        
    return 0;
    
}
/**
 * @brief Cleanup and close the GPMC driver module.
 */
void __exit rt_gpmc_fpga_exit(void)
{
    TRACE_OBJECT(__FUNCTION__);
    rtdm_printk(KERN_INFO "rt_gpmc_fpga: Exit module\n");
    gpmc_cs_free(6);
    DPRINTK(KERN_DEBUG "DEBUG: gpmc_fpga: Delete device file.\n");
    device_destroy(rt_gpmc_fpga_class, MKDEV(rt_gpmc_fpga_major, 0));
    class_unregister(rt_gpmc_fpga_class);
    class_destroy(rt_gpmc_fpga_class);
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: unregister device.\n");
    rtdm_dev_unregister(&gpmc_fgpa_device);
#if !defined(CONFIG_LEDS_GPIO) && !defined(CONFIG_LEDS_GPIO_MODULE)
    DPRINTK(KERN_DEBUG "DEBUG: rt_gpmc_fpga: free LEDs.\n");
    gpio_free(21);
    gpio_free(22);
#endif
}

module_init(rt_gpmc_fpga_init);
module_exit(rt_gpmc_fpga_exit);
