#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/io.h> 
#include <linux/fs.h>
//#include <asm/segment.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <asm/arch/regs-gpio.h>
#include <asm/arch/regs-irq.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <asm/uaccess.h>        /* copy_to_user() */
#include <linux/delay.h>    /* mdelay() */

#include <linux/irq.h>
#include <linux/interrupt.h> 
#include <asm/arch/regs-irq.h>
#include <asm/arch/regs-gpio.h>
#include <linux/fs.h>
#include <asm/arch/regs-gpio.h>
#include <linux/mm.h>

#include <linux/errno.h>

#include <asm/segment.h>
MODULE_LICENSE("GPL");
#define CHDRV_MAJOR 42
char* CHDRV_NAME = "c8255";
#define PORTA 0x08000000
#define PORTB 0x08000001
#define PORTC 0x08000002
#define CTL   0x08000003
#define LENGTH 0x04
#define MODE1AIBO 0xB4 //A mode 1 input, C high Output, B Mode 1 output, low C output
//#define MODE1AIBO 0xBC//A mode 1 input, C high input, B Mode 1 output, low C output
//#define MODE1AIBO 0x90//A mode 0 input, C high input, B Mode 0 output, low C output



void* io_base;

static irqreturn_t c8255_intsvc(int irq, void *dev_id, struct pt_regs *regs);
struct timer_list c8255_timer;
struct resource *c8255_resource;

static inline int verify_area(int type, const void * addr, unsigned long size)
{
    return access_ok(type,addr,size) ? 0 : -EFAULT;
}

int c8255_chdev_open( struct inode *inode, struct file *file )
{
    unsigned int pinvalue = 0;
    printk("c8255_open 1\n");
    //io_base = ioremap_nocache(PORTA,LENGTH);
    s3c2410_gpio_setpin(S3C2410_GPF6, 0);
    pinvalue = s3c2410_gpio_getpin(S3C2410_GPF6);
    printk("power TP1 mdelay:%d\n", pinvalue);
    printk("c8255_open\n");
    return 0;
}

int c8255_chdev_release( struct inode *inode, struct file *file )
{
	printk("release\n~");
    //iounmap(io_base);
        return 0;
}

static ssize_t c8255_chdev_read(struct file *filp, char *buf, size_t count,loff_t *offset)
{
	unsigned int ix = 1;
    char buff;
    char* buffer2 = kmalloc(1,GFP_KERNEL);
	if(verify_area(VERIFY_WRITE,buf,count)==-EFAULT)
		return -EFAULT;
    //printk("adb ");
 //   iowrite8(0xb4,io_base + 3);
    //printk("bdd ");
	//buff = ioread8(io_base +2);
  //  msleep(1);
    //printk("cdd ");
	//printk("c8255_read~~\n");
	*(buffer2)=ioread8(io_base);
	copy_to_user(buf,buffer2,ix);
	kfree(buffer2);
	return ix;
}

int printstr(char* str, int* count,int len);
static ssize_t c8255_chdev_write(struct file *filp,const char *buf,size_t count, loff_t *offset)
{
	unsigned int iy=0;
    int cnt = 0;
#if 1
	unsigned char datax;
    if(count <= 0 )
        return 0;
	char* buffer1 = kmalloc(count,GFP_KERNEL);
    char* pbuf = buffer1;
	//printk("c8255_write..\n");
	copy_from_user(buffer1,buf,count);
    for(iy = 0 ;iy < count; iy++)
    {
        datax=*(pbuf++);
    //iowrite8(0xb4,io_base + 3);
    //printk("bdd ");
	//buff = ioread8(io_base +2);
    //msleep(1);
        printk("c8255_write.datax:0x%x .%d\n",datax,iy);
        iowrite8(datax, io_base+1);
        udelay(100);
    }
    kfree(buffer1);	
#else
	char* buffer1 = kmalloc(count,GFP_KERNEL);
	copy_from_user(buffer1,buf,count);
    printstr(buffer1, &cnt, count);
	printk("c8255_write..count %d , write successfule cnt:%d\n", count, cnt);
    kfree(buffer1);	
#endif
	return count;

}
#if 0 //io mode0 control PORTB, no use any more
void PBOutData(u8 datax)
{
	printk("0x%x  ", datax);
	iowrite8(datax, io_base+1);
}

int PBStateRead(void)
{
    //BUSY  1 pin11 R3
    //ERROR 0 pin15 R56
    int pinvalue;
    pinvalue = s3c2410_gpio_getpin(S3C2410_GPG6);
    if(pinvalue == 1){
        printk("Port B is busy\n");
        return PRINTER_BUSY;
    }
    return PRINTER_OK;
}
void setStrobe(void)
{
    
#if 1
    //Port C bi set/reset control word
    iowrite8((unsigned char)0x2, io_base+3);
#else
    //direct write port C 
    iowrite8((unsigned char)0x0, io_base+2);
#endif
}
void clrStrobe(void)
{
    
#if 1
    
    //Port C bi set/reset control word
    iowrite8((unsigned char)0x3, io_base+3);
#else
    iowrite8((unsigned char)0x2, io_base+2);
#endif

}

void mydelay(int delay);
int my_mdelays = 30;
int printchr(u8 data)
{
    int k = 0, state;
    setStrobe();
    state = PBStateRead(); 
    while(state != PRINTER_OK)
    {
        msleep(1);
        if(++k > 100)
        {
            if(state == PRINTER_BUSY)
            {
                printk("it busy!!!\n");
                return PRINTER_BUSY;
            }
        }
        state = PBStateRead(); 
    }
    PBOutData(data);
    msleep(1);
    clrStrobe();
    msleep(1);
    //mydelay(my_mdelays);
    setStrobe();
    msleep(1);
    return PRINTER_OK;
}

int printstr(char* str, int* count, int len)
{
    int status;
    while(*count < len)
    {
        status = printchr(*str++);
        *count = *count + 1;
        if(status != PRINTER_OK)
            return status;
    }
    return PRINTER_OK;
}

void setPortAPC5()
{
    unsigned char buf;
#if 0
    //Port C bi set/reset control word
    iowrite8((unsigned char)0xb, io_base+3);
#else
 //   iowrite8((unsigned char)0x0B, io_base+3);
    buf=ioread8(io_base + 2);
    buf|=(1<<5);
    iowrite8(buf, io_base+2);
#endif
}

void clrPortAPC5()
{
    unsigned char buf;
#if 0
    //Port C bi set/reset control word
    iowrite8((unsigned char)0xa, io_base+3);
#else
    buf=ioread8(io_base + 2);
    buf&=~(1<<5);
    iowrite8(buf, io_base+2);
#endif
}
void mydelay(int delay)
{
    int i;
    for(i = delay; i > 0; i--);
}
void set_ack()
{
    int pinvalue;
    int i = 0;
    //  s3c2410_gpio_setpin(S3C2410_GPG5, 0);
    //       pinvalue = s3c2410_gpio_getpin(S3C2410_GPG5);
    //         printk("power GPG5 value:%d\n", pinvalue);
   // iowrite8(0x90, io_base+3);
    mdelay(10);
    clrPortAPC5();
    mydelay(my_mdelays);
    setPortAPC5();
    
 //   iowrite8(0xb4, io_base+3);
    // s3c2410_gpio_setpin(S3C2410_GPG5, 1);
    //pinvalue = s3c2410_gpio_getpin(S3C2410_GPG5);
    //printk("power GPG5 value:%d\n", pinvalue);
}
#endif

static ssize_t c8255_chdev_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
    unsigned char buf,bufc;
    unsigned int pinvalue;
    unsigned int pinstate;
 
    int ret;
    switch(cmd) {
        case 0:
        case 1:
        case 2:
        case 3:
            printk("c8255_ioctl cfg addr:%d, value:%x\n",cmd, arg);
            iowrite8((unsigned char)arg, io_base+cmd);
            break;
        case 4:
        case 5:
        case 6:
        case 7:
            buf=ioread8(io_base + cmd - 4);
            printk("c8255_ioctl read cfg addr:%d, value:%x\n",(cmd-4),buf);
            break;
        case 8:
            //1st. check PortC status register
            bufc=ioread8(io_base + 2);
            printk("\tC%x,", bufc);
            if(bufc&0x20) //INTRA is set, there some thing coming
            {
                //read port A
                buf=ioread8(io_base + 0);
                printk("r%x,", buf);
                msleep(1);
                // buf=ioread8(io_base + 2);
                // printk("Cb%x,", buf);
                if(bufc&0x01) //IBFA and INTRA is set, there some thing coming
                {  //write to port B
                    printk("w%x,", buf);
                    iowrite8(buf, io_base + 1);
                    msleep(1);
                }
            }
            break;
        case 9:
            bufc=ioread8(io_base + 2);
            printk("Ca%x,", bufc);
            if(bufc&0x20) //IBFA is set, there some thing coming
            {
                //read port A
                buf=ioread8(io_base + 0);
                printk("r%x,", buf);
                msleep(1);
            }
            break;
        case 10:
            printk("power ACK GPG5/EINT13 TP11 set:%d\n", arg);
            s3c2410_gpio_setpin(S3C2410_GPG5, arg);
            pinvalue = s3c2410_gpio_getpin(S3C2410_GPG5);
            printk("power GPG5 value:%d\n", pinvalue);
            break;
        case 11:
            printk("power GPG6/EINT14 TP12 set:%d\n", arg);
            s3c2410_gpio_setpin(S3C2410_GPG6, arg);
            pinvalue = s3c2410_gpio_getpin(S3C2410_GPG6);
            printk("power GPG6/EINT14 TP12 value:%d\n", pinvalue);
            break;
#if 0
        case 12:
            setPortAPC5();
            break;
        case 13:
            set_ack();
            break;
        case 14:
            clrPortAPC5();
            break;
        case 15:
            my_mdelays = arg;
            printk("my_mdelays:%d\n", my_mdelays);
            break;
#endif
        case 16:
            s3c2410_gpio_cfgpin(S3C2410_GPG9,S3C2410_GPG9_INP);
            set_irq_type(IRQ_EINT17, IRQT_RISING);
            ret = request_irq(IRQ_EINT17, c8255_intsvc, SA_INTERRUPT, "c8255_driver", NULL);
            if(ret)
            {   
                printk(KERN_WARNING "buttons:can't get irq no.%d, ret:%d\n", IRQ_EINT17, ret);
                return -1;
            }       
            iowrite8((u8)0x09,io_base+3);
            iowrite8((u8)0x05,io_base+3);
            break;
        case 17:
            s3c2410_gpio_setpin(S3C2410_GPF6, 0);
            break;
        case 18:
            pinvalue = s3c2410_gpio_getpin(S3C2410_GPF6);
            printk("power GPF6 mdelay:%d\n", pinvalue);
            pinstate = s3c2410_gpio_getcfg(S3C2410_GPF6);
            printk("pinstate GPF6 set:0x%x\n", pinstate);
            break;
        case 19:
            disable_irq(IRQ_EINT6);
            break;
        case 20:
            free_irq(IRQ_EINT6, NULL);
            break;
        case 21:
            free_irq(IRQ_EINT15, NULL);
            break;
        default:
            return -EINVAL;
    }
    return 0;

}
static struct file_operations c8255_chdrv_fops = {
    .read = c8255_chdev_read, /* read */
    .write = c8255_chdev_write, /* write */
    .ioctl = c8255_chdev_ioctl, /* ioctl */
    .open = c8255_chdev_open, /* open */
    .release = c8255_chdev_release, /* release */
};

static irqreturn_t c8255_intsvc(int irq, void *dev_id, struct pt_regs *regs)
{
    char buf;
    spinlock_t local_lock;
    unsigned long flags;
    spin_lock_irqsave(&local_lock,flags);
 //   disable_irq(IRQ_EINT17);
	buf=ioread8(io_base);
    //printk("A=0x%x,%c ", buf, buf);
	iowrite8(buf, io_base+1);
    //add_timer(&c8255_timer); // enable_irq(IRQ_EINT17);
    spin_unlock_irqrestore(&local_lock,flags);
    return IRQ_HANDLED;
}

static irqreturn_t c8255_intsvc_tx(int irq, void *dev_id, struct pt_regs *regs)
{
    char buf = 0x30;
    spinlock_t local_lock;
    unsigned long flags;
   // spin_lock_irqsave(&local_lock,flags);
    printk("B=0x%x,%c ", buf, buf);
	iowrite8(buf, io_base+1);
    //spin_unlock_irqrestore(&local_lock,flags);
    return IRQ_HANDLED;
}



static void c8255_timer_cb()
{
#if 0
    char* buffer3 = kmalloc(1,GFP_KERNEL);
    /* --------read data from 8255 PORT A---------------------- */
    io_base=ioremap_nocache(PORTA, LENGTH);
    iowrite8(0xb4,io_base + 3);
    *buffer3 = ioread8(io_base);
    printk("interrupted !!!!! portA=%x", *buffer3);
    iowrite8(*buffer3, io_base + 1);
    *buffer3 = ioread8(io_base + 2);
    printk(" status = %x", *buffer3);
    kfree(buffer3);
    enable_irq(IRQ_EINT17);
#endif
}
static int __init module_frame_init( void )
{
    int ret;
    if(register_chrdev(CHDRV_MAJOR, CHDRV_NAME, &c8255_chdrv_fops)) {
        printk(KERN_ALERT "module_frame is now loaded.\n");
      //  return -EIO;
    }
    #if 1
    c8255_resource = request_mem_region(PORTA, LENGTH, "c8255");
    if(NULL == c8255_resource )
    {
        
        printk(KERN_ALERT "request mem for c8255 failed\n");
      //  return -ENOMEM;
    }
    #endif
    io_base = ioremap_nocache(PORTA,LENGTH);
    do {
        volatile void *bwscon;
        volatile void *bankcon1;
        void* sfreg_base;
        int reg_data = 0;
        unsigned long pinstate = 0;
        unsigned int pinvalue = 0;
        unsigned char u8255cfg = MODE1AIBO;
#define BWSCON (0x48000000)   
#define BANKCON1 (0x48000008)   
        bwscon = ioremap_nocache(BWSCON, LENGTH);
        bankcon1 = ioremap_nocache(BANKCON1, LENGTH);
        reg_data = readl(bwscon);
        printk(KERN_ALERT "reg_data bwscon:%x\n", reg_data);
        reg_data &= ~(0x03<<4);
        printk(KERN_ALERT "reg_data bwscon:%x\n", reg_data);
        writel(reg_data, bwscon);

        reg_data = readl(bankcon1);
        printk(KERN_ALERT "reg_data bankcon1:%x\n", reg_data);
        reg_data &= ~(0x05<<4);
        reg_data |= (0x05<<4);
        printk(KERN_ALERT "reg_data bankcon1:%x\n", reg_data);
        writel(reg_data, bankcon1);

        s3c2410_gpio_cfgpin(S3C2410_GPF6, S3C2410_GPF6_OUTP);
        s3c2410_gpio_setpin(S3C2410_GPF6, 0);
        pinvalue = s3c2410_gpio_getpin(S3C2410_GPF6);
        printk("power GPF6 mdelay:%d\n", pinvalue);
        pinstate = s3c2410_gpio_getcfg(S3C2410_GPF6);
        printk("pinstate GPG6 set:0x%x\n", pinstate);
       // s3c2410_gpio_cfgpin(S3C2410_GPF6, S3C2410_GPF6_OUTP);

        s3c2410_gpio_cfgpin(S3C2410_GPA12, S3C2410_GPA12_nGCS1);
        pinstate = s3c2410_gpio_getcfg(S3C2410_GPA12);
        printk("pinstate GPA12 8255 :0x%x\n", pinstate);

        pinstate = s3c2410_gpio_getcfg(S3C2410_GPA14);
        printk("pinstate GPA14 dm9000 :0x%x\n", pinstate);

        //s3c2410_gpio_cfgpin(S3C2410_GPG5, S3C2410_GPG5_OUTP);
        //s3c2410_gpio_setpin(S3C2410_GPG5, 0);
        //disable_irq(IRQ_EINT13);
        //disable_irq(IRQ_EINT6);
        //free_irq(IRQ_EINT6, NULL);
       // free_irq(IRQ_EINT13,NULL);
        //EINT14 PortB busy
      //  s3c2410_gpio_cfgpin(S3C2410_GPG6, S3C2410_GPG6_INP);
        //s3c2410_gpio_setpin(S3C2410_GPG6, 0);
#if 0
        //interrupt init EINT17
        sfreg_base=ioremap_nocache(INTMOD, LENGTH);
        printk("remapped INTMOD address= 0x%8x\n", sfreg_base);
        reg_data=readl(sfreg_base);
        reg_data &= 0xffffffdf;
        writel(reg_data, sfreg_base);

        sfreg_base=ioremap_nocache(INTMASK, LENGTH);
        printk("remapped INTMASK address= 0x%8x\n", sfreg_base);
        reg_data=readl(sfreg_base);
        reg_data &= 0xffffffdf;
        writel(reg_data, sfreg_base);

        sfreg_base=ioremap_nocache(EXTINT2, LENGTH);
        printk("remapped EXTINT2 address= 0x%8x\n", sfreg_base);
        reg_data=readl(sfreg_base);
        reg_data &= 0xffffff40;
        writel(reg_data, sfreg_base);

        sfreg_base=ioremap_nocache(EINTMASK, LENGTH);
        printk("remapped EINTMASK address= 0x%8x\n", sfreg_base);
        reg_data=readl(sfreg_base);
        reg_data &= ~(1<<17);
        writel(reg_data, sfreg_base);
        request_irq(5, c8255_intsvc, SA_INTERRUPT, "c8255_driver",NULL);
#else 
#if 1 //EINT17 interrupt
        iowrite8(u8255cfg , io_base+3);
        printk("write 0x%x to 8255 config reg",u8255cfg);
        //INTA PC4, port A input INT enable 
        iowrite8((u8)0x09,io_base+3);
        s3c2410_gpio_cfgpin(S3C2410_GPG9,S3C2410_GPG9_INP);
        set_irq_type(IRQ_EINT17, IRQT_RISING);
        ret = request_irq(IRQ_EINT17, c8255_intsvc, SA_INTERRUPT, "c8255_driver", NULL);
        if(ret)
        {   
            printk(KERN_WARNING "Port A:can't get irq no.%d, ret:%d\n", IRQ_EINT17, ret);
            return -1;
        }       
        //INTA PC2, port B output INT enable 
        iowrite8(0x05, io_base+3);
        s3c2410_gpio_cfgpin(S3C2410_GPG7,S3C2410_GPG7_INP);
        set_irq_type(IRQ_EINT15, IRQT_RISING);
        ret = request_irq(IRQ_EINT15, c8255_intsvc_tx, SA_INTERRUPT, "c8255_driver_tx", NULL);
        if(ret)
        {   
            printk(KERN_WARNING "Port B:can't get irq no.%d, ret:%d\n", IRQ_EINT15, ret);
            return -1;
        }
#endif
#endif
        c8255_timer.function = c8255_timer_cb;
        init_timer(&c8255_timer);
#if 0
        reg_data =ioread8(io_base+3);
        printk(KERN_ALERT "reg_data CTL:%x\n", reg_data);
        reg_data =ioread8(io_base+3);
        printk(KERN_ALERT "reg_data CTL:%x\n", reg_data);
#endif  
#if 0
        reg_data =ioread8(io_base+3);
        printk(KERN_ALERT "reg_data CTL:%x\n", reg_data);
        reg_data =ioread8(io_base+3);
        printk(KERN_ALERT "reg_data CTL:%x\n", reg_data);
#endif
    }while(0);
    printk(KERN_ALERT "module_frame is now loaded.return 0\n");
    return 0;
}//module_frame_init()

static void __exit module_frame_exit( void )
{
    del_timer(&c8255_timer);
    release_mem_region(PORTA, LENGTH);
    free_irq(IRQ_EINT17, NULL);
    free_irq(IRQ_EINT15, NULL);
    iounmap(io_base);
    unregister_chrdev(CHDRV_MAJOR, CHDRV_NAME);
    printk(KERN_ALERT "module_frame is now unloaded.\n");
    return;
}//module_frame_exit()

module_init( module_frame_init );
module_exit( module_frame_exit );

