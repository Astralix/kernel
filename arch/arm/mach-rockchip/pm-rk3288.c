
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <linux/rockchip/cpu.h>
//#include <linux/rockchip/cru.h>
#include <linux/rockchip/grf.h>
#include <linux/rockchip/iomap.h>
#include "pm.h"
#include <linux/irqchip/arm-gic.h>

#define CPU 3288
//#include "sram.h"
#include "pm-pie.c"

/*************************cru define********************************************/
#define RK3288_CRU_UNGATING_OPS(id) cru_writel(CRU_W_MSK_SETBITS(0,id%16,0x1),RK3288_CRU_GATEID_CONS(id))
#define RK3288_CRU_GATING_OPS(id) cru_writel(CRU_W_MSK_SETBITS(1,id%16,0x1),RK3288_CRU_GATEID_CONS(id))

/*******************************gpio define **********************************************/

/* GPIO control registers */
#define GPIO_SWPORT_DR		0x00
#define GPIO_SWPORT_DDR		0x04
#define GPIO_INTEN			0x30
#define GPIO_INTMASK		0x34
#define GPIO_INTTYPE_LEVEL	0x38
#define GPIO_INT_POLARITY	0x3c
#define GPIO_INT_STATUS		0x40
#define GPIO_INT_RAWSTATUS	0x44
#define GPIO_DEBOUNCE		0x48
#define GPIO_PORTS_EOI		0x4c
#define GPIO_EXT_PORT		0x50
#define GPIO_LS_SYNC		0x60

/***********************************sleep func*********************************************/

// sys resume data in boot ram
#define SLP_DATA_SAVE_PHY  (RK3288_BOOTRAM_PHYS+PM_BOOT_CODE_OFFSET+PM_BOOT_CODE_SIZE)
#define SLP_DATA_SAVE_BASE  (RK_BOOTRAM_VIRT+PM_BOOT_CODE_OFFSET+PM_BOOT_CODE_SIZE)

// ddr resume data in boot ram
#define SLP_DDR_DATA_SAVE_PHY  (RK3288_BOOTRAM_PHYS + PM_BOOT_DDR_CODE_OFFSET)
#define SLP_DDR_DATA_SAVE_BASE  (RK_BOOTRAM_VIRT+PM_BOOT_DDR_CODE_OFFSET)

#define PM_BOOT_DDR_CODE_OFFSET	(((PM_BOOT_CODE_OFFSET+PM_BOOT_CODE_SIZE+PM_BOOT_DATA_SIZE)/4+2)*4)
#define PM_BOOT_CODE_SP (RK3288_BOOTRAM_PHYS+((RK3288_BOOTRAM_SIZE-1)&~0x7))


#define BOOT_RAM_SIZE	(4*1024)
#define INT_RAM_SIZE		(64*1024)

static char boot_ram_data[BOOT_RAM_SIZE+4*10];
static char int_ram_data[INT_RAM_SIZE];


// the value is used to control cpu resume flow
static u32 sleep_resume_data[SLPDATA_SLEEP_RES_CON_CNT];
static char *resume_data_base=(char *)(SLP_DATA_SAVE_BASE);
static char *resume_data_phy=  (char *)(SLP_DATA_SAVE_PHY);

static void sram_code_reset(char *data, char *save, char* sram_base,char* sram_phy,u32 _size)
{
  char *addr_d=(char *)sram_base;

  if(save)
	  memcpy(save,addr_d, _size);
	  
  memcpy((char *)addr_d,(char *)data, _size);
  flush_icache_range((unsigned long)addr_d, (unsigned long)addr_d + _size);
  outer_clean_range((phys_addr_t) sram_phy, _size);
}

/**
ddr code and data

*** code start
---data offset-- 
---code----
---data----
*/
static void sram_data_for_sleep(char *boot_save, char *int_save)
{	
 	
	char *addr_base,*addr_phy,*data_src,*data_dst;
	u32 sr_size,data_size;

	addr_base=(char *)RK_BOOTRAM_VIRT;
	addr_phy=(char *)RK3288_BOOTRAM_PHYS;
	sr_size=RK3288_BOOTRAM_SIZE;

 	// save boot sram
	 if(boot_save)
		 memcpy(boot_save,addr_base, sr_size);

	// move resume code and date to boot sram
	// move sys code
	data_dst=(char *)RK_BOOTRAM_VIRT+PM_BOOT_CODE_OFFSET;
	data_src=(char *)rkpm_slp_cpu_resume;
	data_size=PM_BOOT_CODE_SIZE;
	memcpy((char *)data_dst,(char *)data_src, data_size);

	// move sys data
	data_dst=(char *)resume_data_base;
	data_src=(char *)sleep_resume_data;
	data_size=sizeof(sleep_resume_data);
	memcpy((char *)data_dst,(char *)data_src, data_size);
#if 0
	/*************************ddr code cpy*************************************/
	// ddr code
	data_dst=(char *)(char *)SLP_DDR_DATA_SAVE_BASE;
	data_src=(char *)ddr_get_resume_code_info(&data_size);
	
	data_size=(data_size/4+1)*4;
	
	memcpy((char *)data_dst,(char *)data_src, data_size);

	// ddr data
	data_dst=(char *)(data_dst+data_size);
	
	data_src=(char *)ddr_get_resume_data_info(&data_size);
	data_size=(data_size/4+1)*4;
	memcpy((char *)data_dst,(char *)data_src, data_size);
	
	/*************************ddr code cpy  end*************************************/
#endif	
	flush_icache_range((unsigned long)addr_base, (unsigned long)addr_base + sr_size);
	 outer_clean_range((phys_addr_t) addr_phy, (phys_addr_t)(addr_phy)+sr_size);
#if 0	 
	 /*************************int mem bak*************************************/
	// int mem
	addr_base=(char *)rockchip_sram_virt;
	//addr_phy=(char *)RK319X_IMEM_PHYS;
	sr_size=rockchip_sram_size;
        //mmap
	if(int_save)
	  memcpy(int_save,addr_base, sr_size);

	 flush_icache_range((unsigned long)addr_base, (unsigned long)addr_base + sr_size);
	// outer_clean_range((phys_addr_t) addr_phy, sr_size);
#endif
 }

static void sram_data_resume(char *boot_save, char *int_save)
{  
 
    char *addr_base,*addr_phy;
    u32 sr_size;

    addr_base=(char *)RK_BOOTRAM_VIRT;
    addr_phy=(char *)RK3288_BOOTRAM_PHYS;
    sr_size=RK3288_BOOTRAM_SIZE;
    // save boot sram
    if(boot_save)
        memcpy(addr_base,boot_save, sr_size);

    flush_icache_range((unsigned long)addr_base, (unsigned long)addr_base + sr_size);
    outer_clean_range((phys_addr_t) addr_phy, (phys_addr_t)addr_phy+sr_size);
     
    #if 0
    // int mem
    addr_base=(char *)RK319X_IMEM_BASE;
    addr_phy=(char *)RK319X_IMEM_PHYS;
    sr_size=RK319X_IMEM_SIZE;

     if(int_save)
       memcpy(addr_base, int_save,sr_size);

      flush_icache_range((unsigned long)addr_base, (unsigned long)addr_base + sr_size);
      outer_clean_range((phys_addr_t) addr_phy, sr_size);
      #endif
}

/**************************************gic save and resume**************************/
#define  RK_GICD_BASE (RK_GIC_VIRT)
#define RK_GICC_BASE (RK_GIC_VIRT+RK3288_GIC_DIST_SIZE)

#define PM_IRQN_START 32
#define PM_IRQN_END	107//107
static void pm_gic_enable(u32 irqs)
{

        int irqstart=0;
        u32 bit_off;
        void __iomem *reg_off;
        unsigned int gic_irqs;

        gic_irqs = PM_IRQN_END;
        irqstart=PM_IRQN_START;//PM_IRQN_START;

        reg_off=(irqs/32)*4+GIC_DIST_ENABLE_SET+RK_GICD_BASE;
        bit_off=irqs%32;
        writel_relaxed(readl_relaxed(reg_off)|(1<<bit_off),reg_off);

        dsb();
}
  
static void rkpm_gic_disable(u32 irqs)
{
        int irqstart=0;
        u32 bit_off;    
        void __iomem *reg_off;
        unsigned int gic_irqs;

        gic_irqs = PM_IRQN_END;
        irqstart=PM_IRQN_START;//PM_IRQN_START;

        reg_off=(irqs/32)*4+GIC_DIST_ENABLE_CLEAR+RK_GICD_BASE;
        bit_off=irqs%32;
        writel_relaxed(readl_relaxed(reg_off)&~(1<<bit_off),reg_off);
        dsb();
}
  
#define gic_reg_dump(a,b,c)  {}//reg_dump((a),(b),(c))
  
static u32 slp_gic_save[260+50];

static void rkpm_gic_dist_save(u32 *context)
{
     int i = 0,j,irqstart=0;
     unsigned int gic_irqs;
     
     gic_irqs = readl_relaxed(RK_GICD_BASE + GIC_DIST_CTR) & 0x1f;
     gic_irqs = (gic_irqs + 1) * 32;
     if (gic_irqs > 1020)
     gic_irqs = 1020;
     //printk("gic_irqs=%d\n",gic_irqs);
     //gic_irqs = PM_IRQN_END;
     irqstart=PM_IRQN_START;//PM_IRQN_START;
     
     i = 0;
     //level
     for (j = irqstart; j < gic_irqs; j += 16)
      context[i++]=readl_relaxed(RK_GICD_BASE + GIC_DIST_CONFIG + (j * 4) / 16);
     gic_reg_dump("gic level",j,RK_GICD_BASE + GIC_DIST_CONFIG);

     /*
     * Set all global interrupts to this CPU only.
     */
     for(j = 0; j < gic_irqs; j += 4)
    	 context[i++]=readl_relaxed(RK_GICD_BASE + GIC_DIST_TARGET +	(j * 4) / 4);
     
     gic_reg_dump("gic trig",j,RK_GICD_BASE + GIC_DIST_TARGET);

     //pri
     for (j = 0; j < gic_irqs; j += 4)
    	 context[i++]=readl_relaxed(RK_GICD_BASE+ GIC_DIST_PRI + (j * 4) / 4);
     gic_reg_dump("gic pri",j,RK_GICD_BASE + GIC_DIST_PRI);	 


     

     //secure
     for (j = 0; j < gic_irqs; j += 32)
    	 context[i++]=readl_relaxed(RK_GICD_BASE + 0x80 + (j * 4) / 32);
     gic_reg_dump("gic secure",j,RK_GICD_BASE + 0x80); 
     	 
     for (j = irqstart; j < gic_irqs; j += 32)
    	 context[i++]=readl_relaxed(RK_GICD_BASE + GIC_DIST_PENDING_SET + (j * 4) / 32);
     
     gic_reg_dump("gic PENDING",j,RK_GICD_BASE + GIC_DIST_PENDING_SET);	 


     
    #if 0
     //disable
     for (j = 0; j < gic_irqs; j += 32)
    	 context[i++]=readl_relaxed(RK_GICD_BASE + GIC_DIST_ENABLE_CLEAR + (j * 4) / 32);
     
     gic_reg_dump("gic dis",j,RK_GICD_BASE + GIC_DIST_ENABLE_CLEAR);
    #endif
     //enable
     for (j = 0; j < gic_irqs; j += 32)
    	 context[i++]=readl_relaxed(RK_GICD_BASE + GIC_DIST_ENABLE_SET + (j * 4) / 32);

    	//sram_printhex(j);
     gic_reg_dump("gic en",j,RK_GICD_BASE + GIC_DIST_ENABLE_SET);  

     
     
     gic_reg_dump("gicc",0x1c,RK_GICC_BASE);	 
     gic_reg_dump("giccfc",0,RK_GICC_BASE+0xfc);

     context[i++]=readl_relaxed(RK_GICC_BASE + GIC_CPU_PRIMASK);
     context[i++]=readl_relaxed(RK_GICC_BASE + GIC_CPU_CTRL);
     context[i++]=readl_relaxed(RK_GICD_BASE + GIC_DIST_CTRL);

     
    #if 0
     context[i++]=readl_relaxed(RK_GICC_BASE + GIC_CPU_BINPOINT);
     context[i++]=readl_relaxed(RK_GICC_BASE + GIC_CPU_PRIMASK);
     context[i++]=readl_relaxed(RK_GICC_BASE + GIC_DIST_SOFTINT);
     context[i++]=readl_relaxed(RK_GICC_BASE + GIC_CPU_CTRL);
     context[i++]=readl_relaxed(RK_GICD_BASE + GIC_DIST_CTRL);
    #endif	
    #if 0  //rk319x is not need
     for (j = irqstart; j < gic_irqs; j += 32)
     {
    	 writel_relaxed(0xffffffff, RK_GICD_BASE + GIC_DIST_ENABLE_CLEAR + j * 4 / 32);
    	 dsb();

     }
    #endif  

}

static void rkpm_gic_dist_resume(u32 *context)
{

         int i = 0,j,irqstart=0;
         unsigned int gic_irqs;
         
         gic_irqs = readl_relaxed(RK_GICD_BASE + GIC_DIST_CTR) & 0x1f;
         gic_irqs = (gic_irqs + 1) * 32;
         if (gic_irqs > 1020)
        	 gic_irqs = 1020;
         
         
         //gic_irqs = PM_IRQN_END;
         irqstart=PM_IRQN_START;//PM_IRQN_START;

         writel_relaxed(0,RK_GICC_BASE + GIC_CPU_CTRL);
         dsb();
         writel_relaxed(0,RK_GICD_BASE + GIC_DIST_CTRL);
         dsb();
         for (j = irqstart; j < gic_irqs; j += 32)
         {
        	 writel_relaxed(0xffffffff, RK_GICD_BASE + GIC_DIST_ENABLE_CLEAR + j * 4 / 32);
        	 dsb();
         }


         i = 0;

         //trig
         for (j = irqstart; j < gic_irqs; j += 16)
         {
        	 writel_relaxed(context[i++],RK_GICD_BASE + GIC_DIST_CONFIG + j * 4 / 16);
        	 dsb();
         }
         gic_reg_dump("gic level",j,RK_GICD_BASE + GIC_DIST_CONFIG);	 

         /*
         * Set all global interrupts to this CPU only.
         */
         for (j = 0; j < gic_irqs; j += 4)
         {
        	 writel_relaxed(context[i++],RK_GICD_BASE + GIC_DIST_TARGET +  (j * 4) / 4);
        	 dsb();
         }
         gic_reg_dump("gic target",j,RK_GICD_BASE + GIC_DIST_TARGET);  

         //pri
         for (j = 0; j < gic_irqs; j += 4)
         {
        	 writel_relaxed(context[i++],RK_GICD_BASE+ GIC_DIST_PRI + (j * 4) / 4);
        	 
        	 dsb();
         }
         gic_reg_dump("gic pri",j,RK_GICD_BASE + GIC_DIST_PRI);	 

         
         //secu
         for (j = 0; j < gic_irqs; j += 32)
         {
        	 writel_relaxed(context[i++],RK_GICD_BASE + 0x80 + (j * 4 )/ 32);
        	 #if 0
        	 sram_printhex((j * 4 )/ 32);
        	 
        	 sram_printch('_');
        	 sram_printhex(temp);
        	 
        	 sram_printch('_');
        	 sram_printhex(readl_relaxed(RK_GICD_BASE + 0x80 + (j * 4 )/ 32));
        	 sram_printch('\n');
#endif
        	 
        	 dsb();
         }

         gic_reg_dump("gic secu",j,RK_GICD_BASE + 0x80);	 


         //pending
         for (j = irqstart; j < gic_irqs; j += 32)
         {
        	 //writel_relaxed(context[i++],RK_GICD_BASE + GIC_DIST_PENDING_SET + j * 4 / 32);
        	 i++;
        	 dsb();
         }
         gic_reg_dump("gic pending",j,RK_GICD_BASE + GIC_DIST_PENDING_SET);	 


         //disable
#if 0
         for (j = 0; j < gic_irqs; j += 32)
         {
        	 writel_relaxed(context[i++],RK_GICD_BASE + GIC_DIST_ENABLE_CLEAR + j * 4 / 32);
        	 
        	 dsb();
         }
         gic_reg_dump("gic disable",j,RK_GICD_BASE + GIC_DIST_ENABLE_CLEAR);	 
         
#else
        //for (j = 0; j < gic_irqs; j += 32)
        	// writel_relaxed(0xffffffff,RK_GICD_BASE + GIC_DIST_ENABLE_CLEAR + j * 4 / 32);
#endif
         
        	 
         //enable
         for (j = 0; j < gic_irqs; j += 32)
         {
        	 writel_relaxed(context[i++],RK_GICD_BASE + GIC_DIST_ENABLE_SET + (j * 4) / 32);
        	 
        	 dsb();
         }
        // sram_printhex(j);
         gic_reg_dump("gic enable",j,RK_GICD_BASE + GIC_DIST_ENABLE_SET);  

         writel_relaxed(context[i++],RK_GICC_BASE + GIC_CPU_PRIMASK);
         
         writel_relaxed(context[i++],RK_GICC_BASE + GIC_CPU_CTRL);

         writel_relaxed(context[i++],RK_GICD_BASE + GIC_DIST_CTRL);

         gic_reg_dump("gicc",0x1c,RK_GICC_BASE);	 
         gic_reg_dump("giccfc",0,RK_GICC_BASE+0xfc);	 
 
}


 
/**************************************regs save and resume**************************/
void slp_regs_save(u32 *data,void __iomem * base,u32 st_offset,u32 end_offset)
{
     u32 i;
         u32 cnt=(end_offset-st_offset)/4+1;
     for(i=0;i<cnt;i++)
     {
    	 data[i]=readl_relaxed(base+st_offset+i*4);
     }	 
}

void slp_regs_resume(u32 *data,void __iomem * base,u32 st_offset,u32 end_offset,u32 w_msk)
{
     u32 i;
     u32 cnt=(end_offset-st_offset)/4+1;
     for(i=0;i<cnt;i++)
     {		 
    	 reg_writel(data[i]|w_msk,(base+st_offset+i*4));
     }	 
}

void slp_regs_w_msk_resume(u32 *data,void __iomem * base,u32 st_offset,u32 end_offset,u32 *w_msk)
{
        u32 i;
        u32 cnt=(end_offset-st_offset)/4+1;
         for(i=0;i<cnt;i++)
	 {		 
		 reg_writel(data[i]|w_msk[i],(base+st_offset+i*4));
	 }	 
}

/**************************************uarts save and resume**************************/

#define RK3288_UART_NUM (4)

static void __iomem *slp_uart_base[RK3288_UART_NUM]={NULL};
static u32 slp_uart_phy[RK3288_UART_NUM]={(0xff180000),(0xff190000),(0xff690000),(0xff1b0000)};

static u32 slp_uart_data[RK3288_UART_NUM][10];
 
#define UART_DLL	0	/* Out: Divisor Latch Low */
#define UART_DLM	1	/* Out: Divisor Latch High */

#define UART_IER	1
#define UART_FCR	2
 
#define UART_LCR	3	/* Out: Line Control Register */
#define UART_MCR	4

 void slp_uart_save(int ch)
 {
	 int i=0;
	void __iomem *b_addr=slp_uart_base[ch];
	 int idx=RK3288_CLKGATE_PCLK_UART0+ch;
	 u32 gate_reg;
	 if(b_addr==NULL || ch>=RK3288_UART_NUM)
	 	return;	
        if(ch==2)
        {
            idx=RK3288_CLKGATE_PCLK_UART2;
            b_addr=RK_DEBUG_UART_VIRT;
        }

        
	gate_reg=cru_readl(RK3288_CRU_GATEID_CONS(idx));     
        RK3288_CRU_UNGATING_OPS(idx); 
         i=0;
	 slp_uart_data[ch][i++]=readl_relaxed(b_addr+UART_LCR*4); 
	 writel_relaxed(readl_relaxed(b_addr+UART_LCR*4)|0x80,b_addr+UART_LCR*4);
	 
	 slp_uart_data[ch][i++]=readl_relaxed(b_addr+UART_DLL*4);
	 slp_uart_data[ch][i++]=readl_relaxed(b_addr+UART_DLM*4);
	 
	 writel_relaxed(readl_relaxed(b_addr+UART_LCR*4)&(~0x80),b_addr+UART_LCR*4);
	 slp_uart_data[ch][i++]=readl_relaxed(b_addr+UART_IER*4);
	 slp_uart_data[ch][i++]=readl_relaxed(b_addr+UART_FCR*4);
	 slp_uart_data[ch][i++]=readl_relaxed(b_addr+UART_MCR*4);
	 
        cru_writel(gate_reg|CRU_W_MSK(idx%16,0x1),RK3288_CRU_GATEID_CONS(idx));         
 
 }
 
 void slp_uart_resume(int ch)
 {	 
        int i=0;

        u32 temp;
        void __iomem *b_addr=slp_uart_base[ch];
        int idx=RK3288_CLKGATE_PCLK_UART0+ch;
        u32 gate_reg;
        
        if(b_addr==NULL || ch>=RK3288_UART_NUM)
            return;	
        
        if(ch==2)
            idx=RK3288_CLKGATE_PCLK_UART2;


        gate_reg=cru_readl(RK3288_CRU_GATEID_CONS(idx));     
        RK3288_CRU_UNGATING_OPS(idx); 
 
         i=0;
	 temp=slp_uart_data[ch][i++];
	 writel_relaxed(readl_relaxed(b_addr+UART_LCR*4)|0x80,b_addr+UART_LCR*4);
	 
	 writel_relaxed(slp_uart_data[ch][i++],b_addr+UART_DLL*4);
	 writel_relaxed(slp_uart_data[ch][i++],b_addr+UART_DLM*4);
	 
	 writel_relaxed(readl_relaxed(b_addr+UART_LCR*4)&(~0x80),b_addr+UART_LCR*4);
 
	 writel_relaxed(slp_uart_data[ch][i++],b_addr+UART_IER*4);
	 writel_relaxed(slp_uart_data[ch][i++],b_addr+UART_FCR*4);
	 writel_relaxed(slp_uart_data[ch][i++],b_addr+UART_MCR*4);
	 
	 writel_relaxed(temp,b_addr+UART_LCR*4);
	 
         cru_writel(gate_reg|CRU_W_MSK(idx%16,0x1),RK3288_CRU_GATEID_CONS(idx));         
 }
 
/**************************************i2c save and resume**************************/

//#define RK3288_I2C_REG_DUMP
#define RK3288_I2C_NUM (6)
static u32 slp_i2c_phy[RK3288_I2C_NUM]={(0xff650000),(0xff140000),(0xff660000),(0xff150000),(0xff160000),(0xff170000)};
static void __iomem *slp_i2c_base[RK3288_I2C_NUM]={NULL};

static u32 slp_i2c_data[RK3288_I2C_NUM][10];

void slp_i2c_save(int ch)
{

	void __iomem *b_addr=slp_i2c_base[ch];
	int idx= (ch>1) ? (RK3288_CLKGATE_PCLK_I2C2+ch-2):(RK3288_CLKGATE_PCLK_I2C0+ch);
	u32 gate_reg;

	if(!b_addr)
		return;
    
        gate_reg=cru_readl(RK3288_CRU_GATEID_CONS(idx));     
        RK3288_CRU_UNGATING_OPS(idx); 
        
        #ifdef RK3288_I2C_REG_DUMP
        rkpm_ddr_printascii("i2c save");
        rkpm_ddr_printhex(ch);
        rkpm_ddr_printch('\n');        
        rkpm_ddr_regs_dump(b_addr,0x0,0xc);
        #endif
        
        slp_regs_save(&slp_i2c_data[ch][0],b_addr,0x0,0xc);  
        

        cru_writel(gate_reg|CRU_W_MSK(idx%16,0x1),RK3288_CRU_GATEID_CONS(idx));         

}
void slp_i2c_resume(int ch)
{
        void __iomem *b_addr=slp_i2c_base[ch];
        int idx= (ch>1) ? (RK3288_CLKGATE_PCLK_I2C2+ch-2):(RK3288_CLKGATE_PCLK_I2C0+ch);
	u32 gate_reg;
	
	if(!b_addr)
		return;
        gate_reg=cru_readl(RK3288_CRU_GATEID_CONS(idx));     
        RK3288_CRU_UNGATING_OPS(idx); 

        slp_regs_resume(&slp_i2c_data[ch][0],b_addr,0x0,0xc,0x0);  

        #ifdef RK3288_I2C_REG_DUMP
        rkpm_ddr_printascii("i2c resume");
        rkpm_ddr_printhex(ch);
        rkpm_ddr_printch('\n');        
        rkpm_ddr_regs_dump(b_addr,0x0,0xc);
        #endif
  
        cru_writel(gate_reg|CRU_W_MSK(idx%16,0x1),RK3288_CRU_GATEID_CONS(idx));         
}

/**************************************gpios save and resume**************************/
#define RK3288_GPIO_CH (9)
static u32 slp_gpio_data[RK3288_GPIO_CH][10]; 
static u32 slp_grf_iomux_data[RK3288_GPIO_CH*4];
static u32 slp_grf_io_pull_data[RK3288_GPIO_CH*4];

static void gpio_ddr_dump_reg(int ports)
{
    void __iomem *b_addr=RK_GPIO_VIRT(ports);
    
    rkpm_ddr_printascii("gpio-");
    rkpm_ddr_printhex(ports);
    rkpm_ddr_printhex('\n');      
    
    rkpm_ddr_reg_offset_dump(b_addr,GPIO_SWPORT_DR);
    rkpm_ddr_reg_offset_dump(b_addr,GPIO_SWPORT_DDR);      
    rkpm_ddr_reg_offset_dump(b_addr,GPIO_INTEN);  
    rkpm_ddr_reg_offset_dump(b_addr,GPIO_INTMASK);     
    rkpm_ddr_reg_offset_dump(b_addr,GPIO_INTTYPE_LEVEL);  
    rkpm_ddr_reg_offset_dump(b_addr,GPIO_INT_POLARITY);   
    rkpm_ddr_reg_offset_dump(b_addr,GPIO_DEBOUNCE);   
    rkpm_ddr_reg_offset_dump(b_addr,GPIO_LS_SYNC);    
    rkpm_ddr_printhex('\n');      

    rkpm_ddr_printascii("iomux\n");
    rkpm_ddr_regs_dump(RK_GRF_VIRT,0x0+ports*4*4,0x0+ports*4*4+3*4);

    rkpm_ddr_printascii("iomux\n");
    rkpm_ddr_regs_dump(RK_GRF_VIRT,0x130+ports*4*4,ports*4*4+3*4);

}

 static void slp_pin_gpio_save(int ports)
 {
        int i;
        void __iomem *b_addr=RK_GPIO_VIRT(ports);
        int idx=RK3288_CLKGATE_PCLK_GPIO1+ports-1;
        u32 gate_reg;

	if(ports==0||ports>=RK3288_GPIO_CH)
		return;
	
         gate_reg=cru_readl(RK3288_CRU_GATEID_CONS(idx));     
         RK3288_CRU_UNGATING_OPS(idx); 
         
         //gpio_ddr_dump_reg(ports);          
	 i=0;
	 slp_gpio_data[ports][i++]=readl_relaxed(b_addr+GPIO_SWPORT_DR);
	 slp_gpio_data[ports][i++]=readl_relaxed(b_addr+GPIO_SWPORT_DDR);
	 slp_gpio_data[ports][i++]=readl_relaxed(b_addr+GPIO_INTEN);	 
	 slp_gpio_data[ports][i++]=readl_relaxed(b_addr+GPIO_INTMASK);  
	 slp_gpio_data[ports][i++]=readl_relaxed(b_addr+GPIO_INTTYPE_LEVEL);	 
	 slp_gpio_data[ports][i++]=readl_relaxed(b_addr+GPIO_INT_POLARITY);
	 slp_gpio_data[ports][i++]=readl_relaxed(b_addr+GPIO_DEBOUNCE);
	 slp_gpio_data[ports][i++]=readl_relaxed(b_addr+GPIO_LS_SYNC); 

        if(ports>0)
        {
            slp_regs_save(&slp_grf_iomux_data[ports*4],RK_GRF_VIRT,0x0+ports*4*4,0x0+ports*4*4+3*4);  
            slp_regs_save(&slp_grf_io_pull_data[ports*4],RK_GRF_VIRT,0x130+ports*4*4,ports*4*4+3*4);
         }

     
        cru_writel(gate_reg|CRU_W_MSK(idx%16,0x1),RK3288_CRU_GATEID_CONS(idx));         
 
 }

 static void slp_pin_gpio_resume (int ports)
 {
	 int i;
        void __iomem *b_addr=RK_GPIO_VIRT(ports);
        int idx=RK3288_CLKGATE_PCLK_GPIO1+ports-1;
	 u32 gate_reg;
	 
	 if(ports==0||ports>=RK3288_GPIO_CH)
		return;
	  gate_reg=cru_readl(RK3288_CRU_GATEID_CONS(idx));     
         RK3288_CRU_UNGATING_OPS(idx); 


        if(ports>0)
        {
            slp_regs_resume(&slp_grf_iomux_data[ports*4],RK_GRF_VIRT,0x0+ports*4*4,0x0+ports*4*4+3*4,0xffff0000);  
            slp_regs_resume(&slp_grf_io_pull_data[ports*4],RK_GRF_VIRT,0x130+ports*4*4,ports*4*4+3*4,0xffff0000);
        }
 
        i=0;
        writel_relaxed(slp_gpio_data[ports][i++],b_addr+GPIO_SWPORT_DR);
        writel_relaxed(slp_gpio_data[ports][i++],b_addr+GPIO_SWPORT_DDR);
        writel_relaxed(slp_gpio_data[ports][i++],b_addr+GPIO_INTEN);	 
        writel_relaxed(slp_gpio_data[ports][i++],b_addr+GPIO_INTMASK); 
        writel_relaxed(slp_gpio_data[ports][i++],b_addr+GPIO_INTTYPE_LEVEL);	 
        writel_relaxed(slp_gpio_data[ports][i++],b_addr+GPIO_INT_POLARITY);
        writel_relaxed(slp_gpio_data[ports][i++],b_addr+GPIO_DEBOUNCE);
        writel_relaxed(slp_gpio_data[ports][i++],b_addr+GPIO_LS_SYNC);	    
        
        //gpio_ddr_dump_reg(ports);	
        cru_writel(gate_reg|CRU_W_MSK(idx%16,0x1),RK3288_CRU_GATEID_CONS(idx));         
 
 }

/**************************************sleep func**************************/

void ddr_reg_save(uint32_t *pArg);
void fiq_glue_resume(void);
void rk30_cpu_resume(void);
void rk30_l2_cache_init_pm(void);
//static void rk319x_pm_set_power_domain(enum pmu_power_domain pd, bool state);
void ddr_cfg_to_lp_mode(void);
void l2x0_inv_all_pm(void);
void rk30_cpu_while_tst(void);

#if 0
static u32 slp_grf_soc_con_data[5];
static u32 slp_grf_soc_con_w_msk[5]={0x70000,0x40ff0000,0xffff0000,0xffff0000,0xffff0000};

static u32 slp_grf_cpu_con_data[5];
static u32 slp_grf_cpu_con_w_msk[5]={0xefff0000,0xffff0000,0xcfff0000,0xffff0000,0x7fff0000};

static u32 slp_grf_uoc0_con_data[4];
static u32 slp_grf_uoc0_con_w_msk[4]={0xffff0000,0xffff0000,0x7dff0000,0x7fff0000};// uoc0_con4 bit 15?? 

static u32 slp_grf_uoc1_con_data[2];
static u32 slp_grf_uoc1_con_w_msk[2]={0x1fdc0000,0x047f0000};

static u32 slp_grf_uoc2_con_data[2];
static u32 slp_grf_uoc2_con_w_msk[2]={0x7fff0000,0x1f0000};

static u32 slp_grf_uoc3_con_data[2];
static u32 slp_grf_uoc3_con_w_msk[2]={0x3ff0000,0x0fff0000};

#endif
static u32 slp_pmu_pwrmode_con_data[1];


static u32 slp_nandc_data[8];
static void __iomem *rk30_nandc_base=NULL;

#define MS_37K (37)
#define US_24M (24)

void inline pm_io_base_map(void)
{
        int i;
        for(i=0;i<RK3288_I2C_NUM;i++)
            slp_i2c_base[i]  = ioremap(slp_i2c_phy[i], 0x1000);

        for(i=0;i<RK3288_UART_NUM;i++)
            {
                if(i!=CONFIG_RK_DEBUG_UART)
                    slp_uart_base[i]  = ioremap(slp_uart_phy[i], 0x1000);
                else
                    slp_uart_base[i] = RK_DEBUG_UART_VIRT;
            }
	
}	
#if 0
void pm_gpio_setting(void)
{

	//gpio wake up setting
	gpio0_lp_wakeup_en(1,3);
	gpio0s_lp_wakeup_type(1);
	gpio0_lp_wakeup_en(6,3);
	
}
#endif
 enum rk3288_pwr_mode_con {

        pmu_pwr_mode_en=0,
        pmu_clk_core_src_gate_en,
        pmu_global_int_disable,
        pmu_l2flush_en,
        
        pmu_bus_pd_en,
        pmu_a12_0_pd_en,
        pmu_scu_en,
        pmu_pll_pd_en,
        
        pmu_chip_pd_en, // power off pin enable
        pmu_pwroff_comb,
        pmu_alive_use_lf,
        pmu_pmu_use_lf,
        
        pmu_osc_24m_dis,
        pmu_input_clamp_en,
        pmu_wakeup_reset_en,
        pmu_sref0_enter_en,
        
        pmu_sref1_enter_en,       
        pmu_ddr0io_ret_en,
        pmu_ddr1io_ret_en,
        pmu_ddr0_gating_en,
        
        pmu_ddr1_gating_en,
        pmu_ddr0io_ret_de_req,
        pmu_ddr1io_ret_de_req

};
 enum rk3288_pwr_mode_con1 {

        pmu_clr_bus=0,
        pmu_clr_core,
        pmu_clr_cpup,
        pmu_clr_alive,
        
        pmu_clr_dma,
        pmu_clr_peri,
        pmu_clr_gpu,
        pmu_clr_video,
        pmu_clr_hevc,
        pmu_clr_vio
  
};

static void ddr_pin_set_fun(u8 port,u8 bank,u8 b_gpio,u8 fun);

static u32 sgrf_soc_con0,pmu_pwr_mode_con0,pmu_pwr_mode_con1;

static void  rkpm_slp_mode_set(u32 val)
{
    u32 mode_set,mode_set1;
    
    // setting gpio0_a0 arm off pin
    ddr_pin_set_fun(0x0,0xa,0x0,0x1);

    sgrf_soc_con0=reg_readl(RK_SGRF_VIRT+RK3288_SGRF_SOC_CON0);
    pmu_pwr_mode_con0=pmu_readl(RK3288_PMU_PWRMODE_CON);  
    pmu_pwr_mode_con1=pmu_readl(RK3288_PMU_PWRMODE_CON1);
    
    mode_set1=pmu_pwr_mode_con1;
    mode_set=pmu_pwr_mode_con0;
    

    pmu_writel(0x1<<3,RK3188_PMU_WAKEUP_CFG1);  

    // enable boot ram    
    reg_writel((0x1<<8)|(0x1<<(8+16)),RK_SGRF_VIRT+RK3288_SGRF_SOC_CON0);
    reg_writel(RK3288_BOOTRAM_PHYS,RK_SGRF_VIRT+RK3288_SGRF_FAST_BOOT_ADDR);

    mode_set|=  BIT(pmu_pwr_mode_en)|BIT(pmu_global_int_disable)
                        | BIT(pmu_l2flush_en)
                        |BIT(pmu_sref0_enter_en)|BIT(pmu_sref1_enter_en) |BIT(pmu_ddr0_gating_en)|BIT(pmu_ddr1_gating_en);


    if(rkpm_chk_val_ctrbit(val,RKPM_CTR_IDLEAUTO_MD))
    {
        rkpm_ddr_printascii("-autoidle-");
        mode_set|=BIT(pmu_clk_core_src_gate_en);
    }
    else if(rkpm_chk_val_ctrbit(val,RKPM_CTR_ARMDP_LPMD))
    {
        rkpm_ddr_printascii("-armdp-");            
        //rkpm_ddr_printhex(cru_readl(RK3288_CRU_MODE_CON));       
        
       // pmu_writel(0x1<<3,RK3188_PMU_WAKEUP_CFG1);  
        mode_set|=BIT(pmu_a12_0_pd_en);
    }
    else if(rkpm_chk_val_ctrbit(val,RKPM_CTR_ARMOFF_LPMD))
    {
    
        rkpm_ddr_printascii("-armoff-");            
        //rkpm_ddr_printhex(cru_readl(RK3288_CRU_MODE_CON)); 
        
        mode_set|=BIT(pmu_scu_en)|BIT(pmu_chip_pd_en);
        mode_set1|=BIT(pmu_clr_core)|BIT(pmu_clr_cpup);
    } 
    else
    {
        mode_set=pmu_pwr_mode_con0;
        mode_set1=pmu_pwr_mode_con1;
    }

   
    pmu_writel(mode_set,RK3288_PMU_PWRMODE_CON);  
    pmu_writel(mode_set1,RK3288_PMU_PWRMODE_CON1);  

}

static inline void  rkpm_slp_mode_set_resume(void)
{

    pmu_writel(pmu_pwr_mode_con0,RK3288_PMU_PWRMODE_CON);  
    pmu_writel(pmu_pwr_mode_con1,RK3288_PMU_PWRMODE_CON1);  
    reg_writel(sgrf_soc_con0|(0x1<<(8+16)),RK_SGRF_VIRT+RK3288_SGRF_SOC_CON0);
    
}

static inline u32 rkpm_l2_config(void)
{
	u32 l2ctlr;
	asm("mrc p15, 1, %0, c9, c0, 2" : "=r" (l2ctlr));
        return l2ctlr;
}


static int sleep_resume_all=0;
void pm_sleep_func_save(void)
{
	//char *data_src;
	//u32 data_size;
	//u32 *p;
	
	sleep_resume_data[SLPDATA_L2_CON]=rkpm_l2_config();// in sys resume ,ddr is need resume	
	sleep_resume_data[SLPDATA_SP_ADDR]=PM_BOOT_CODE_SP;// in sys resume ,ddr is need resume	
	sleep_resume_data[SLPDATA_SP_CPU_RESUME]=virt_to_phys(cpu_resume);// in sys resume ,ddr is need resume
	sleep_resume_data[SLPDATA_DDR_NEED_RES]=0;// in sys resume ,ddr is need resume
	sleep_resume_data[SLPDATA_DPLL_NEED_RES]=0;// in ddr resume ,dpll is need resume
	//data_src=(char *)ddr_get_resume_code_info(&data_size);
	//sleep_resume_data[SLPDATA_DDR_CODE_PHY]=SLP_DDR_DATA_SAVE_PHY;
	//sleep_resume_data[SLPDATA_DDR_DATA_PHY]=SLP_DDR_DATA_SAVE_PHY+(data_size/4+1)*4;	
	
	//ddr_reg_save(resume_data_phy+SLP_DPLL_NEED_RES*4);
	sram_data_for_sleep(boot_ram_data,int_ram_data);
       rkpm_gic_dist_save(&slp_gic_save[0]);
	flush_cache_all();
	outer_flush_all();
	local_flush_tlb_all();

	//slp_regs_save(slp_grf_iomux_data,(u32)RK_GRF_VIRT+0x10,16);
	//slp_regs_save(slp_nandc_data,(u32)rk30_nandc_base,8);
 
	//slp_pin_gpio_save(1);
	//slp_pin_gpio_save(2);
	//slp_pin_gpio_save(3);
	//slp_pin_gpio_save(4);
	//slp_uart_save(2);
	#if 0
    	rkpm_ddr_printascii("l2-");
        rkpm_ddr_printhex(sleep_resume_data[SLPDATA_L2_CON]);
    	rkpm_ddr_printascii("\n");
        #endif
        
	sleep_resume_all=0;
}
void pm_sleep_func_rusume_first(void)
{
    
       // rkpm_ddr_printhex(cru_readl(RK3288_CRU_MODE_CON));
        #if 0
	//rk319x_pm_set_power_domain(PD_PERI,true);
	//slp_regs_resume(slp_grf_io_pull_data,(u32)RK_GRF_VIRT+0x144,16,0xffff0000);
	slp_pin_gpio_resume(1);
	slp_pin_gpio_resume(2);
	slp_pin_gpio_resume(3);
	slp_pin_gpio_resume(4);

	#if 0
	slp_regs_w_msk_resume(slp_grf_soc_con_data,(u32)RK_GRF_VIRT+0x60,5,slp_grf_soc_con_w_msk);
	slp_regs_w_msk_resume(slp_grf_cpu_con_data,(u32)RK_GRF_VIRT+0x9c,5,slp_grf_cpu_con_w_msk);

	slp_regs_w_msk_resume(slp_grf_uoc0_con_data,(u32)RK_GRF_VIRT+0xc4,4,slp_grf_uoc0_con_w_msk);
	slp_regs_w_msk_resume(slp_grf_uoc1_con_data,(u32)RK_GRF_VIRT+0xd4,2,slp_grf_uoc1_con_w_msk);
	slp_regs_w_msk_resume(slp_grf_uoc2_con_data,(u32)RK_GRF_VIRT+0xe4,2,slp_grf_uoc2_con_w_msk);
	slp_regs_w_msk_resume(slp_grf_uoc3_con_data,(u32)RK_GRF_VIRT+0xec,2,slp_grf_uoc3_con_w_msk);
	#endif
	//sram_printch_uart_enable();
	slp_uart_resume(2);
    #endif
    
    sleep_resume_all=1;
    

}

void pm_sleep_func_rusume_last(void)
{
    if(sleep_resume_all)
    {
  
        // slp_uart_resume(0);
        // slp_uart_resume(1);
        // slp_uart_resume(3);
        // slp_regs_resume(slp_nandc_data,(u32)rk30_nandc_base,8,0);

        //rkpm_ddr_printch('g');
        rkpm_gic_dist_resume(&slp_gic_save[0]);

        fiq_glue_resume();
        #if 0
        rkpm_ddr_printascii("l2-");
        rkpm_ddr_printhex(rkpm_l2_config());
        rkpm_ddr_printascii("\n");
        #endif
        
    }
    rkpm_slp_mode_set_resume();
    sram_data_resume(boot_ram_data,int_ram_data);  
}

/*******************************common code  for rkxxx*********************************/
static void  inline uart_printch(char byte)
{
        u32 reg_save[2];
        u32 u_clk_id=(RK3288_CLKGATE_UART0_SRC+CONFIG_RK_DEBUG_UART);
        u32 u_pclk_id=(RK3288_CLKGATE_PCLK_UART0+CONFIG_RK_DEBUG_UART);
        
        if(CONFIG_RK_DEBUG_UART==4)
            u_clk_id=RK3288_CLKGATE_UART4_SRC;
        if(CONFIG_RK_DEBUG_UART==2)
            u_pclk_id=RK3288_CLKGATE_PCLK_UART2;
            
        reg_save[0]=cru_readl(RK3288_CRU_GATEID_CONS(u_clk_id));
        reg_save[1]=cru_readl(RK3288_CRU_GATEID_CONS(u_pclk_id));
        RK3288_CRU_UNGATING_OPS(u_clk_id);
        RK3288_CRU_UNGATING_OPS(u_pclk_id);
        
        rkpm_udelay(1);
        
	writel_relaxed(byte, RK_DEBUG_UART_VIRT);
	dsb();

	/* loop check LSR[6], Transmitter Empty bit */
	while (!(readl_relaxed(RK_DEBUG_UART_VIRT + 0x14) & 0x40))
		barrier();
    
         cru_writel(reg_save[0]|CRU_W_MSK(u_clk_id%16,0x1),RK3288_CRU_GATEID_CONS(u_clk_id));         
         cru_writel(reg_save[1]|CRU_W_MSK(u_pclk_id%16,0x1),RK3288_CRU_GATEID_CONS(u_pclk_id));
        
	if (byte == '\n')
		uart_printch('\r');
}

void PIE_FUNC(sram_printch)(char byte)
{
	uart_printch(byte);
}

static void pll_udelay(u32 udelay);

static void  ddr_printch(char byte)
{
	uart_printch(byte);
        pll_udelay(2);
}
/*******************************gpio func*******************************************/
//#define RK3288_PMU_GPIO0_A_IOMUX	0x0084
//#define RK3288_PMU_GPIO0_B_IOMUX	0x0088
//#define RK3288_PMU_GPIO0_C_IOMUX	0x008c
//#define RK3288_PMU_GPIO0_D_IOMUX	0x0090


//pin=0x0a21  gpio0a2,port=0,bank=a,b_gpio=2,fun=1
static inline void pin_set_fun(u8 port,u8 bank,u8 b_gpio,u8 fun)
{ 
    u8 off_set;
    bank-=0xa;

    if(port==0)
    { 
        if(bank>2)
            return;
            
        off_set=RK3288_PMU_GPIO0_A_IOMUX+bank*4;
        pmu_writel(RKPM_VAL_SETBITS(pmu_readl(off_set),fun,b_gpio*2,0x3),off_set);
    }
    else
    {    
    
        off_set=port*(4*4)+bank*4;
        //form RK3288_GRF_GPIO1D_IOMUX
         reg_writel(RKPM_W_MSK_SETBITS(fun,b_gpio*2,0x3),RK_GRF_VIRT+0+off_set);
    }
}

static inline u8 pin_get_funset(u8 port,u8 bank,u8 b_gpio)
{ 
    u8 off_set;
    bank-=0xa;

    if(port==0)
    { 
      if(bank>2)
            return 0;
        off_set=RK3288_PMU_GPIO0_A_IOMUX+bank*4;
        return (pmu_readl(off_set)>>(b_gpio*2))&0x3;
    }
    else
    {    
    
        off_set=port*(4*4)+bank*4;
        //form RK3288_GRF_GPIO1D_IOMUX
         return (reg_readl(RK_GRF_VIRT+0+off_set)>>(b_gpio*2))&0x3;
    }
}

static inline void pin_set_pull(u8 port,u8 bank,u8 b_gpio,u8 pull)
{ 
    u8 off_set;
    
    bank-=0xa;

    if(port > 0)
    {
        //gpio1_d st
        if(port==1&&bank<3)
         return;   
        //gpio1_d==0x14c ,form gpio0_a to gpio1_d offset 1*16+3*4= 0x1c
        off_set=0x14c-0x1c+port*(4*4)+bank*4;    
        reg_writel(RKPM_W_MSK_SETBITS(pull,b_gpio*2,0x3),RK_GRF_VIRT+off_set);

    }
    else
    {
        if(bank>2)// gpio0_d is not support
            return; 
        pmu_writel(RKPM_VAL_SETBITS(pmu_readl(0x64+bank*4),pull,b_gpio*2,0x3),0x64+bank*4);
    }
        
}

static inline u8 pin_get_pullset(u8 port,u8 bank,u8 b_gpio)
{ 
    u8 off_set;
    
    bank-=0xa;

    if(port > 0)
    {
        //gpio1_d st
        if(port==1&&bank<3)
            return 0;   
        //gpio1_d==0x14c ,form gpio0_a to gpio1_d offset 1*16+3*4= 0x1c
        off_set=0x14c-0x1c+port*(4*4)+bank*4;    
        return RKPM_GETBITS(reg_readl(RK_GRF_VIRT+off_set),b_gpio*2,0x3);

    }
    else
    {
        if(bank>2)// gpio0_d is not support
            return 0;         
        return RKPM_GETBITS(pmu_readl(0x64+bank*4),b_gpio*2,0x3);
    }
        
}


//RKPM_GPIOS_INPUT
static inline void gpio_set_in_output(u8 port,u8 bank,u8 b_gpio,u8 type)
{
    u8 val;    
    
    bank-=0xa;
    b_gpio=bank*8+b_gpio;//

    val=reg_readl(RK_GPIO_VIRT(port)+GPIO_SWPORT_DDR);

    if(type==RKPM_GPIO_OUTPUT)
        val|=(0x1<<b_gpio);
    else
        val&=~(0x1<<b_gpio);
    
    reg_writel(val,RK_GPIO_VIRT(port)+GPIO_SWPORT_DDR);
}

static inline u8 gpio_get_in_outputset(u8 port,u8 bank,u8 b_gpio)
{
    bank-=0xa;
    b_gpio=bank*8+b_gpio;
    return reg_readl(RK_GPIO_VIRT(port)+GPIO_SWPORT_DDR)&(0x1<<b_gpio);
}

//RKPM_GPIOS_OUT_L   RKPM_GPIOS_OUT_H
static inline void gpio_set_output_level(u8 port,u8 bank,u8 b_gpio,u8 level)
{
    u8 val;    

    bank-=0xa;
    b_gpio=bank*8+b_gpio;
        
    val=reg_readl(RK_GPIO_VIRT(port)+GPIO_SWPORT_DR);

    if(level==RKPM_GPIO_OUT_H)
        val|=(0x1<<b_gpio);
    else //
        val&=~(0x1<<b_gpio);

     reg_writel(val,RK_GPIO_VIRT(port)+GPIO_SWPORT_DR);
}

static inline u8 gpio_get_output_levelset(u8 port,u8 bank,u8 b_gpio)
{     
    bank-=0xa;
    b_gpio=bank*8+b_gpio;
    return reg_readl(RK_GPIO_VIRT(port)+GPIO_SWPORT_DR)&(0x1<<b_gpio);
}

static inline u8 gpio_get_input_level(u8 port,u8 bank,u8 b_gpio)
{

    bank-=0xa;
    b_gpio=bank*8+b_gpio;

    return (reg_readl(RK_GPIO_VIRT(port)+GPIO_EXT_PORT)>>b_gpio)&0x1;
}

static void __sramfunc sram_pin_set_fun(u8 port,u8 bank,u8 b_gpio,u8 fun)
{ 
        pin_set_fun(port,bank,b_gpio,fun); 
}
static u8 __sramfunc sram_pin_get_funset(u8 port,u8 bank,u8 b_gpio)
{ 
    return pin_get_funset(port,bank,b_gpio); 
}

static void __sramfunc sram_pin_set_pull(u8 port,u8 bank,u8 b_gpio,u8 fun)
{ 
        pin_set_pull(port,bank,b_gpio,fun); 
}
static u8 __sramfunc sram_pin_get_pullset(u8 port,u8 bank,u8 b_gpio)
{ 
    return pin_get_pullset(port,bank,b_gpio); 
}

static void __sramfunc sram_gpio_set_in_output(u8 port,u8 bank,u8 b_gpio,u8 type)
{
    gpio_set_in_output(port,bank,b_gpio,type);
}

static u8 __sramfunc sram_gpio_get_in_outputset(u8 port,u8 bank,u8 b_gpio)
{
    return gpio_get_in_outputset(port,bank,b_gpio);
}

static void __sramfunc sram_gpio_set_output_level(u8 port,u8 bank,u8 b_gpio,u8 level)
{
    
    gpio_set_output_level(port,bank,b_gpio,level);

}

static u8 __sramfunc sram_gpio_get_output_levelset(u8 port,u8 bank,u8 b_gpio)
{
    return gpio_get_output_levelset(port,bank,b_gpio);
}

static u8 __sramfunc sram_gpio_get_input_level(u8 port,u8 bank,u8 b_gpio)
{
    return gpio_get_input_level(port,bank,b_gpio);
}
//ddr
static void ddr_pin_set_fun(u8 port,u8 bank,u8 b_gpio,u8 fun)
{ 
        pin_set_fun(port,bank,b_gpio,fun); 
}
static u8 ddr_pin_get_funset(u8 port,u8 bank,u8 b_gpio)
{ 
    return pin_get_funset(port,bank,b_gpio); 
}

static void ddr_pin_set_pull(u8 port,u8 bank,u8 b_gpio,u8 fun)
{ 
        pin_set_pull(port,bank,b_gpio,fun); 
}
static u8 ddr_pin_get_pullset(u8 port,u8 bank,u8 b_gpio)
{ 
    return pin_get_pullset(port,bank,b_gpio); 
}

static void ddr_gpio_set_in_output(u8 port,u8 bank,u8 b_gpio,u8 type)
{
    gpio_set_in_output(port,bank,b_gpio,type);
}

static u8 ddr_gpio_get_in_outputset(u8 port,u8 bank,u8 b_gpio)
{
    return gpio_get_in_outputset(port,bank,b_gpio);
}

static void ddr_gpio_set_output_level(u8 port,u8 bank,u8 b_gpio,u8 level)
{   
    gpio_set_output_level(port,bank,b_gpio,level);
}

static u8 ddr_gpio_get_output_levelset(u8 port,u8 bank,u8 b_gpio)
{
    return gpio_get_output_levelset(port,bank,b_gpio);
}

static u8 ddr_gpio_get_input_level(u8 port,u8 bank,u8 b_gpio)
{
    return gpio_get_input_level(port,bank,b_gpio);
}



static  void __sramfunc rkpm_pin_gpio_config_sram(u32 pin_gpio_bits,u32 *save_bits)
{
    
    u32 pins;
    u8 port,bank,b_gpio,fun,in_out, level, pull;
   
    pins=RKPM_PINGPIO_BITS_PIN(pin_gpio_bits);      
    in_out=RKPM_PINGPIO_BITS_INOUT(pin_gpio_bits);       
    pull=RKPM_PINGPIO_BITS_PULL(pin_gpio_bits);          
    level=RKPM_PINGPIO_BITS_LEVEL(pin_gpio_bits);     

    port=RKPM_PINBITS_PORT(pins);
    bank=RKPM_PINBITS_BANK(pins);
    b_gpio=RKPM_PINBITS_BGPIO(pins);
    fun=RKPM_PINBITS_FUN(pins);
    
    //save pins info
    if(save_bits)
    {
        pins=RKPM_PINBITS_SET_FUN(pins,sram_pin_get_funset(port,bank,b_gpio));
       *save_bits=RKPM_PINGPIO_BITS(pins,sram_pin_get_pullset(port,bank,b_gpio),sram_gpio_get_in_outputset(port,bank,b_gpio),
                                                                                        sram_gpio_get_output_levelset(port,bank,b_gpio));
    }
    if(!fun&&(in_out==RKPM_GPIO_OUTPUT))
   {
        if(level==RKPM_GPIO_OUT_L)
            pull=RKPM_GPIO_PULL_DN;
        else
            pull=RKPM_GPIO_PULL_UP;
        
        sram_gpio_set_output_level(port,bank,b_gpio,level);
    }
        
    sram_pin_set_pull(port,bank,b_gpio,pull);                
    sram_pin_set_fun(port,bank,b_gpio,fun);
    
    if(!fun)
    {
        sram_gpio_set_in_output(port,bank,b_gpio,in_out);
    }      
    
}

static inline void rkpm_pin_gpio_config_ddr(u32 pin_gpio_bits,u32 *save_bits)
{
    
    u32 pins;
    u8 port,bank,b_gpio,fun,in_out, level, pull;
   
    pins=RKPM_PINGPIO_BITS_PIN(pin_gpio_bits);      
    in_out=RKPM_PINGPIO_BITS_INOUT(pin_gpio_bits);       
    pull=RKPM_PINGPIO_BITS_PULL(pin_gpio_bits);          
    level=RKPM_PINGPIO_BITS_LEVEL(pin_gpio_bits);     

    port=RKPM_PINBITS_PORT(pins);
    bank=RKPM_PINBITS_BANK(pins);
    b_gpio=RKPM_PINBITS_BGPIO(pins);
    fun=RKPM_PINBITS_FUN(pins);
    
    //save pins info
    if(save_bits)
    {
        pins=RKPM_PINBITS_SET_FUN(pins,ddr_pin_get_funset(port,bank,b_gpio));
       *save_bits=RKPM_PINGPIO_BITS(pins,ddr_pin_get_pullset(port,bank,b_gpio),ddr_gpio_get_in_outputset(port,bank,b_gpio),
                                                                                        ddr_gpio_get_output_levelset(port,bank,b_gpio));
    }
    if(!fun&&(in_out==RKPM_GPIO_OUTPUT))
   {
        if(level==RKPM_GPIO_OUT_L)
            pull=RKPM_GPIO_PULL_DN;
        else
            pull=RKPM_GPIO_PULL_UP;
        
        ddr_gpio_set_output_level(port,bank,b_gpio,level);
    }
        
    ddr_pin_set_pull(port,bank,b_gpio,pull);                
    ddr_pin_set_fun(port,bank,b_gpio,fun);
    
    if(!fun)
    {
        ddr_gpio_set_in_output(port,bank,b_gpio,in_out);
    }      
    
}


#define GPIO_DTS_NUM 10

static  u32 gpio_dts_save[GPIO_DTS_NUM];
static  u32 gpio_dts[GPIO_DTS_NUM];

#define PMICGPIO_DTS_NUM 3


u32 DEFINE_PIE_DATA(pmicgpio_dts[PMICGPIO_DTS_NUM]);
static u32 *p_pmicgpio_dts;
static __sramdata u32 pmicgpio_dts_save[PMICGPIO_DTS_NUM];

static void __sramfunc pmic_gpio_suspend(void)
{
       int i;   
       for(i=0;;i++)
       {
            if(DATA(pmicgpio_dts[i]))
                rkpm_pin_gpio_config_sram(DATA(pmicgpio_dts[i]),& pmicgpio_dts_save[i]);
            else
            {
                    pmicgpio_dts_save[i]=0; 
                    break;
             }
       }
    #if 0       
         for(i=0;i<6;i++)
        {
            rkpm_sram_reg_dump(RK_GPIO_VIRT(i),0,0x4); 
        }
        //
        rkpm_sram_reg_dump(RK_GRF_VIRT,0xc,0x84); 
        rkpm_sram_reg_dump(RK_GRF_VIRT,0x14c,0x1b4);     
        rkpm_sram_reg_dump(RK_PMU_VIRT,0x64,0x6c);   
        rkpm_sram_reg_dump(RK_PMU_VIRT,0x84,0x9c); 
    #endif

}

static void  __sramfunc pmic_gpio_resume(void)
{
       int i;   
       for(i=0;;i++)
       {
            if(pmicgpio_dts_save[i])
                rkpm_pin_gpio_config_sram(pmicgpio_dts_save[i],NULL);     
       }

}

void PIE_FUNC(pmic_suspend)(void)
{
    pmic_gpio_suspend();

}

void PIE_FUNC(pmic_resume)(void)
{
    pmic_gpio_resume();
}


static void  rkpm_gpio_suspend(void)
{
       int i;   
       for(i=0;;i++)
       {
            if(DATA(pmicgpio_dts[i]))
                rkpm_pin_gpio_config_ddr(DATA(pmicgpio_dts[i]),& pmicgpio_dts_save[i]);
            else
            {
                    pmicgpio_dts_save[i]=0; 
                    break;
             }
       }
    #if 0       
         for(i=0;i<6;i++)
        {
            rkpm_ddr_reg_dump(RK_GPIO_VIRT(i),0,0x4); 
        }
        //
        rkpm_ddr_reg_dump(RK_GRF_VIRT,0xc,0x84); 
        rkpm_ddr_reg_dump(RK_GRF_VIRT,0x14c,0x1b4);     
        rkpm_ddr_reg_dump(RK_PMU_VIRT,0x64,0x6c);   
        rkpm_ddr_reg_dump(RK_PMU_VIRT,0x84,0x9c); 
    #endif

}

static void  rkpm_gpio_resume(void)
{
       int i;   
       for(i=0;;i++)
       {
            if(pmicgpio_dts_save[i])
                rkpm_pin_gpio_config_ddr(pmicgpio_dts_save[i],NULL);     
       }

}
static void gpio_get_dts_info(struct device_node *parent)
{
        int i;

        for(i=0;i<PMICGPIO_DTS_NUM;i++)
            p_pmicgpio_dts[i]=0;

        for(i=0;i<GPIO_DTS_NUM;i++)
            gpio_dts[i]=0;

        
        p_pmicgpio_dts= kern_to_pie(rockchip_pie_chunk, &DATA(pmicgpio_dts[0]));
        
       if(of_property_read_u32_array(parent,"rockchip,pmic-gpios",p_pmicgpio_dts,PMICGPIO_DTS_NUM))
       {
                p_pmicgpio_dts[0]=0;
               PM_ERR("%s:get pm ctr error\n",__FUNCTION__);
       }
       
       for(i=0;i<PMICGPIO_DTS_NUM;i++)
            printk("%s:pmic gpio(%x)\n",__FUNCTION__,p_pmicgpio_dts[i]);

        if(of_property_read_u32_array(parent,"rockchip,pm-gpios",gpio_dts,GPIO_DTS_NUM))
        {
                 gpio_dts[0]=0;
                PM_ERR("%s:get pm ctr error\n",__FUNCTION__);
        }
        for(i=0;i<GPIO_DTS_NUM;i++)
         printk("%s:pmic gpio(%x)\n",__FUNCTION__,gpio_dts[i]);

    rkpm_set_ops_gpios(rkpm_gpio_suspend,rkpm_gpio_resume);
    rkpm_set_sram_ops_gtclks(fn_to_pie(rockchip_pie_chunk, &FUNC(pmic_suspend)), 
                  fn_to_pie(rockchip_pie_chunk, &FUNC(pmic_resume)));

}


/*******************************clk gating config*******************************************/
#define CLK_MSK_GATING(msk, con) cru_writel((msk << 16) | 0xffff, con)
#define CLK_MSK_UNGATING(msk, con) cru_writel(((~msk) << 16) | 0xffff, con)


static u32 clk_ungt_msk[RK3288_CRU_CLKGATES_CON_CNT];// first clk gating setting
static u32 clk_ungt_msk_1[RK3288_CRU_CLKGATES_CON_CNT];// first clk gating setting
static u32 clk_ungt_save[RK3288_CRU_CLKGATES_CON_CNT]; //first clk gating value saveing


u32 DEFINE_PIE_DATA(rkpm_clkgt_last_set[RK3288_CRU_CLKGATES_CON_CNT]);
static u32 *p_rkpm_clkgt_last_set;

static __sramdata u32 rkpm_clkgt_last_save[RK3288_CRU_CLKGATES_CON_CNT];

void PIE_FUNC(gtclks_sram_suspend)(void)
{
    int i;
   // u32 u_clk_id=(RK3188_CLKGATE_UART0_SRC+CONFIG_RK_DEBUG_UART);
   // u32 u_pclk_id=(RK3188_CLKGATE_PCLK_UART0+CONFIG_RK_DEBUG_UART);

    for(i=0;i<RK3288_CRU_CLKGATES_CON_CNT;i++)
    {
        rkpm_clkgt_last_save[i]=cru_readl(RK3288_CRU_CLKGATES_CON(i));     
        CLK_MSK_UNGATING( DATA(rkpm_clkgt_last_set[i]), RK3288_CRU_CLKGATES_CON(i));      
        #if 0
        rkpm_sram_printch('\n');   
        rkpm_sram_printhex(DATA(rkpm_clkgt_last_save[i]));
        rkpm_sram_printch('-');   
        rkpm_sram_printhex(DATA(rkpm_clkgt_last_set[i]));
        rkpm_sram_printch('-');   
        rkpm_sram_printhex(cru_readl(RK3188_CRU_CLKGATES_CON(i)));
        if(i==(RK3288_CRU_CLKGATES_CON_CNT-1))         
        rkpm_sram_printch('\n');   
        #endif
    }
    
        //RK3288_CRU_UNGATING_OPS(u_clk_id);
        //RK3288_CRU_UNGATING_OPS(u_pclk_id);
 
}

void PIE_FUNC(gtclks_sram_resume)(void)
{
    int i;
    for(i=0;i<RK3288_CRU_CLKGATES_CON_CNT;i++)
    {
        cru_writel(rkpm_clkgt_last_save[i]|0xffff0000, RK3288_CRU_CLKGATES_CON(i));
    }
}

static void gtclks_suspend(void)
{
    int i;

  // rkpm_ddr_regs_dump(RK_CRU_VIRT,RK3288_CRU_CLKGATES_CON(0)
                                          //          ,RK3288_CRU_CLKGATES_CON(RK3288_CRU_CLKGATES_CON_CNT-1));
    for(i=0;i<RK3288_CRU_CLKGATES_CON_CNT;i++)
    {
            clk_ungt_save[i]=cru_readl(RK3288_CRU_CLKGATES_CON(i));    
           //if(RK3288_CRU_CLKGATES_CON(i)<0x170||RK3288_CRU_CLKGATES_CON(i)>0x194)
            {
                CLK_MSK_UNGATING(clk_ungt_msk[i],RK3288_CRU_CLKGATES_CON(i));
            
            }
           #if 0
            rkpm_ddr_printch('\n');   
            rkpm_ddr_printhex(RK3288_CRU_CLKGATES_CON(i));
            rkpm_ddr_printch('-');   
            rkpm_ddr_printhex(clk_ungt_msk[i]);
            rkpm_ddr_printch('-');   
            rkpm_ddr_printhex(cru_readl(RK3288_CRU_CLKGATES_CON(i))) ;  
            if(i==(RK3288_CRU_CLKGATES_CON_CNT-1))            
            rkpm_ddr_printch('\n');   
            #endif
    }

}

static void gtclks_resume(void)
{
    int i;
     for(i=0;i<RK3288_CRU_CLKGATES_CON_CNT;i++)
    {
       cru_writel(clk_ungt_save[i]|0xffff0000,RK3288_CRU_CLKGATES_CON(i));       
     }
     //rkpm_ddr_regs_dump(RK_CRU_VIRT,RK3288_CRU_CLKGATES_CON(0)
                                                 //   ,RK3288_CRU_CLKGATES_CON(RK3288_CRU_CLKGATES_CON_CNT-1));
    
}
/********************************pll power down***************************************/

enum rk_plls_id {
	APLL_ID = 0,
	DPLL_ID,
	CPLL_ID,
	GPLL_ID,
	NPLL_ID,
	END_PLL_ID,
};

#define RK3288_PLL_PWR_DN_MSK (0x1<<1)
#define RK3288_PLL_PWR_DN CRU_W_MSK_SETBITS(1,1,0x1)
#define RK3288_PLL_PWR_ON CRU_W_MSK_SETBITS(0,1,0x1)


#define RK3288_PLL_RESET		CRU_W_MSK_SETBITS(1,5,0x1)
#define RK3288_PLL_RESET_RESUME CRU_W_MSK_SETBITS(0,5,0x1)

#define RK3288_PLL_BYPASS_MSK (0x1<<0)
#define RK3288_PLL_BYPASS CRU_W_MSK_SETBITS(1,0,0x1)
#define RK3288_PLL_NO_BYPASS CRU_W_MSK_SETBITS(0,0,0x1)

static void pm_pll_wait_lock(u32 pll_idx)
{
	u32 delay = 600000U;
       // u32 mode;
     //  mode=cru_readl(RK3288_CRU_MODE_CON);
	dsb();
	dsb();
	dsb();
	dsb();
	dsb();
	dsb();
	while (delay > 0) {
		if ((cru_readl(RK3288_PLL_CONS(pll_idx,1))&(0x1<<31)))
			break;
		delay--;
	}
	if (delay == 0) {
		rkpm_ddr_printascii("unlock-pll:");
		rkpm_ddr_printhex(pll_idx);
		rkpm_ddr_printch('\n');
	}
    //cru_writel(mode|(RK3288_PLL_MODE_MSK(pll_idx)<<16), RK3288_CRU_MODE_CON);
}	

static void pll_udelay(u32 udelay)
{
    u32 mode;
    mode=cru_readl(RK3288_CRU_MODE_CON);
    // delay in 24m
    cru_writel(RK3288_PLL_MODE_SLOW(APLL_ID), RK3288_CRU_MODE_CON);
    
    rkpm_udelay(udelay*5);
    
    cru_writel(mode|(RK3288_PLL_MODE_MSK(APLL_ID)<<16), RK3288_CRU_MODE_CON);
}

static u32 plls_con0_save[END_PLL_ID];
static u32 plls_con1_save[END_PLL_ID];
static u32 plls_con2_save[END_PLL_ID];
static u32 plls_con3_save[END_PLL_ID];

static u32 cru_mode_con;

static inline void plls_suspend(u32 pll_id)
{
    plls_con0_save[pll_id]=cru_readl(RK3288_PLL_CONS((pll_id), 0));
    plls_con1_save[pll_id]=cru_readl(RK3288_PLL_CONS((pll_id), 1));
    plls_con2_save[pll_id]=cru_readl(RK3288_PLL_CONS((pll_id), 2));
    plls_con3_save[pll_id]=cru_readl(RK3288_PLL_CONS((pll_id), 3));
 
    cru_writel(RK3288_PLL_PWR_DN, RK3288_PLL_CONS((pll_id), 3));
    
}
static inline void plls_resume(u32 pll_id)
{
        u32 pllcon0, pllcon1, pllcon2;

        if((plls_con3_save[pll_id]&RK3288_PLL_PWR_DN_MSK))
            return ;
         
        //enter slowmode
        cru_writel(RK3288_PLL_MODE_SLOW(pll_id), RK3288_CRU_MODE_CON);      
        
        cru_writel(RK3288_PLL_PWR_ON, RK3288_PLL_CONS((pll_id),3));
        cru_writel(RK3288_PLL_NO_BYPASS, RK3288_PLL_CONS((pll_id),3));
        
        pllcon0 =plls_con0_save[pll_id];// cru_readl(RK3288_PLL_CONS((pll_id),0));
        pllcon1 = plls_con1_save[pll_id];//cru_readl(RK3288_PLL_CONS((pll_id),1));
        pllcon2 = plls_con2_save[pll_id];//cru_readl(RK3288_PLL_CONS((pll_id),2));

        //enter rest
        cru_writel(RK3288_PLL_RESET, RK3288_PLL_CONS(pll_id,3));
        cru_writel(pllcon0|CRU_W_MSK(0,0xf)|CRU_W_MSK(8,0x3f), RK3288_PLL_CONS(pll_id,0));
        cru_writel(pllcon1, RK3288_PLL_CONS(pll_id,1));
        cru_writel(pllcon2, RK3288_PLL_CONS(pll_id,2));
        
        pll_udelay(5);
        //udelay(5); //timer7 delay

        //return form rest
        cru_writel(RK3288_PLL_RESET_RESUME, RK3288_PLL_CONS(pll_id,3));

        //wating lock state
        pll_udelay(168);
        pm_pll_wait_lock(pll_id);
        
        cru_writel(plls_con3_save[pll_id]|(RK3288_PLL_BYPASS_MSK<<16),RK3288_PLL_CONS(pll_id,3));

}

static u32 clk_sel0,clk_sel1, clk_sel10,clk_sel26,clk_sel33,clk_sel36, clk_sel37;

static void pm_plls_suspend(void)
{

   // rkpm_ddr_regs_dump(RK_CRU_VIRT,RK3288_PLL_CONS((0), 0),RK3288_PLL_CONS((4), 3)); 
   // rkpm_ddr_regs_dump(RK_CRU_VIRT,RK3288_CRU_MODE_CON,RK3288_CRU_MODE_CON);   
   // rkpm_ddr_regs_dump(RK_CRU_VIRT,RK3288_CRU_CLKSELS_CON(0),RK3288_CRU_CLKSELS_CON(42));
    
    clk_sel0=cru_readl(RK3288_CRU_CLKSELS_CON(0));
    clk_sel1=cru_readl(RK3288_CRU_CLKSELS_CON(1));
    clk_sel10=cru_readl(RK3288_CRU_CLKSELS_CON(10));
    clk_sel26=cru_readl(RK3288_CRU_CLKSELS_CON(26));    
    clk_sel33=cru_readl(RK3288_CRU_CLKSELS_CON(33));
    clk_sel36=cru_readl(RK3288_CRU_CLKSELS_CON(36));
    clk_sel37=cru_readl(RK3288_CRU_CLKSELS_CON(37));
    
    cru_mode_con = cru_readl(RK3288_CRU_MODE_CON);


    cru_writel(RK3288_PLL_MODE_SLOW(NPLL_ID), RK3288_CRU_MODE_CON);  
    plls_suspend(NPLL_ID);
    
// cpll
    cru_writel(RK3288_PLL_MODE_SLOW(CPLL_ID), RK3288_CRU_MODE_CON);
  
// gpll 
    cru_writel(RK3288_PLL_MODE_SLOW(GPLL_ID), RK3288_CRU_MODE_CON); 

    // set 1,pdbus pll is gpll
    cru_writel(CRU_W_MSK_SETBITS(1,15,0x1), RK3288_CRU_CLKSELS_CON(1)); // 0 cpll 1gpll

    // pd_bus clk 
    cru_writel(0
                        |CRU_W_MSK_SETBITS(0,0,0x7)  //  1  aclk
                        |CRU_W_MSK_SETBITS(0,3,0x1f) //  1   aclk src
                        |CRU_W_MSK_SETBITS(0,8,0x3) // 1   hclk 0~1 1 2 4
                        |CRU_W_MSK_SETBITS(0,12,0x7) //  3   pclk
                     , RK3288_CRU_CLKSELS_CON(1));
    
    //crypto for pd_bus
    cru_writel(CRU_W_MSK_SETBITS(3,6,0x3), RK3288_CRU_CLKSELS_CON(26));

    // peri aclk hclk pclk
    cru_writel(0
                        |CRU_W_MSK_SETBITS(0,0,0x1f) // 1 aclk
                        |CRU_W_MSK_SETBITS(0,8,0x3) // 2   hclk 0 1:1,1 2:1 ,2 4:1
                        |CRU_W_MSK_SETBITS(0,12,0x3)// 2     0~3  1 2 4 8 div
                        , RK3288_CRU_CLKSELS_CON(10));

    plls_suspend(CPLL_ID);
    plls_suspend(GPLL_ID);

//apll 
   cru_writel(RK3288_PLL_MODE_SLOW(APLL_ID), RK3288_CRU_MODE_CON);
     // core_m0 core_mp a12_core
    cru_writel(0
                        |CRU_W_MSK_SETBITS(0,0,0xf) // 1   axi_mo
                        |CRU_W_MSK_SETBITS(0,4,0xf) // 3  axi mp
                        |CRU_W_MSK_SETBITS(0,8,0x1f) // 0 a12 core div
                      , RK3288_CRU_CLKSELS_CON(0));
    // core0 core1 core2 core3
    cru_writel(0
                        |CRU_W_MSK_SETBITS(0,0,0x7) //core 0 div
                        |CRU_W_MSK_SETBITS(0,4,0x7) // core 1
                        |CRU_W_MSK_SETBITS(0,8,0x7) // core2
                        |CRU_W_MSK_SETBITS(0,12,0x7)//core3
                      , RK3288_CRU_CLKSELS_CON(36));
    // l2ram atclk pclk
    cru_writel(0
                    |CRU_W_MSK_SETBITS(3,0,0x7) // l2ram
                    |CRU_W_MSK_SETBITS(0xf,4,0x1f) // atclk
                     |CRU_W_MSK_SETBITS(0xf,9,0x1f) // pclk dbg
                     , RK3288_CRU_CLKSELS_CON(37));
    plls_suspend(APLL_ID);

}

static void pm_plls_resume(void)
{

        plls_resume(APLL_ID);    

        // core_m0 core_mp a12_core
        cru_writel(clk_sel0|(CRU_W_MSK(0,0xf)|CRU_W_MSK(4,0xf)|CRU_W_MSK(8,0xf)),RK3288_CRU_CLKSELS_CON(0));
        // core0 core1 core2 core3
        cru_writel(clk_sel36|(CRU_W_MSK(0,0x7)|CRU_W_MSK(4,0x7)|CRU_W_MSK(8,0x7)|CRU_W_MSK(12,0x7))
                        , RK3288_CRU_CLKSELS_CON(36));
        // l2ram atclk pclk
        cru_writel(clk_sel37|(CRU_W_MSK(0,0x7)|CRU_W_MSK(4,0x1f)|CRU_W_MSK(9,0x1f)) , RK3288_CRU_CLKSELS_CON(37));

        cru_writel(cru_mode_con|(RK3288_PLL_MODE_MSK(APLL_ID)<<16), RK3288_CRU_MODE_CON);

        
        plls_resume(GPLL_ID);       
        plls_resume(CPLL_ID);      

        // peri aclk hclk pclk
        cru_writel(clk_sel10|(CRU_W_MSK(0,0x1f)|CRU_W_MSK(8,0x3)|CRU_W_MSK(12,0x3))
                                                                            , RK3288_CRU_CLKSELS_CON(10));
        //pd bus gpll sel
        cru_writel(clk_sel1|CRU_W_MSK(15,0x1), RK3288_CRU_CLKSELS_CON(1));
        // pd_bus clk 
        cru_writel(clk_sel1|(CRU_W_MSK(0,0x7)|CRU_W_MSK(3,0x1f)|CRU_W_MSK(8,0x3)|CRU_W_MSK(12,0x7))
                    , RK3288_CRU_CLKSELS_CON(1));
                
        // crypto
        cru_writel(clk_sel26|CRU_W_MSK(6,0x3), RK3288_CRU_CLKSELS_CON(26));
    
        
          // pmu alive 
        cru_writel(clk_sel33|CRU_W_MSK(0,0x1f)|CRU_W_MSK(8,0x1f), RK3288_CRU_CLKSELS_CON(33));
          
        cru_writel(cru_mode_con|(RK3288_PLL_MODE_MSK(GPLL_ID)<<16), RK3288_CRU_MODE_CON);       
        cru_writel(cru_mode_con|(RK3288_PLL_MODE_MSK(CPLL_ID)<<16), RK3288_CRU_MODE_CON);
        
        plls_resume(NPLL_ID);       
        cru_writel(cru_mode_con|(RK3288_PLL_MODE_MSK(NPLL_ID)<<16), RK3288_CRU_MODE_CON);

       // rkpm_ddr_regs_dump(RK_CRU_VIRT,RK3288_PLL_CONS((0), 0),RK3288_PLL_CONS((4), 3)); 
       // rkpm_ddr_regs_dump(RK_CRU_VIRT,RK3288_CRU_MODE_CON,RK3288_CRU_MODE_CON);   
       // rkpm_ddr_regs_dump(RK_CRU_VIRT,RK3288_CRU_CLKSELS_CON(0),RK3288_CRU_CLKSELS_CON(42));
        
}

static __sramdata u32  sysclk_clksel0_con,sysclk_clksel1_con,sysclk_clksel10_con,sysclk_mode_con;

void PIE_FUNC(sysclk_suspend)(u32 sel_clk)
{

    int div;  
    sysclk_clksel0_con = cru_readl(RK3288_CRU_CLKSELS_CON(0));
    sysclk_clksel1_con = cru_readl(RK3288_CRU_CLKSELS_CON(1));
    sysclk_clksel10_con= cru_readl(RK3288_CRU_CLKSELS_CON(10));


    if(sel_clk&(RKPM_CTR_SYSCLK_32K))
    {
        div=3;
        sysclk_mode_con= cru_readl(RK3288_CRU_MODE_CON);
        cru_writel(0
                |RK3288_PLL_MODE_DEEP(APLL_ID)| RK3288_PLL_MODE_DEEP(CPLL_ID)
                | RK3288_PLL_MODE_DEEP(GPLL_ID)|RK3288_PLL_MODE_DEEP(NPLL_ID)
                            , RK3288_CRU_MODE_CON);
    }
    else if(sel_clk&(RKPM_CTR_SYSCLK_DIV))
    {      
        div=31;
    }

    cru_writel(CRU_W_MSK_SETBITS(div,8,0x1f), RK3188_CRU_CLKSELS_CON(0)); //pd core
    cru_writel(CRU_W_MSK_SETBITS(div,3,0x1f), RK3188_CRU_CLKSELS_CON(1));//pd bus
    cru_writel(CRU_W_MSK_SETBITS(div,0,0x1f), RK3188_CRU_CLKSELS_CON(10));//pd peri
    
}

void PIE_FUNC(sysclk_resume)(u32 sel_clk)
{
    
    cru_writel(sysclk_clksel0_con|CRU_W_MSK(8,0x1f), RK3188_CRU_CLKSELS_CON(0)); //pd core
    cru_writel(sysclk_clksel1_con|CRU_W_MSK(3,0x1f), RK3188_CRU_CLKSELS_CON(1));//pd bus
    cru_writel(sysclk_clksel10_con|CRU_W_MSK(0,0x1f), RK3188_CRU_CLKSELS_CON(10));//pd peri
    cru_writel(sysclk_mode_con|(RK3288_PLL_MODE_MSK(APLL_ID)<<16)
                            |(RK3288_PLL_MODE_MSK(CPLL_ID)<<16)
                            |(RK3288_PLL_MODE_MSK(GPLL_ID)<<16)
                            |(RK3288_PLL_MODE_MSK(NPLL_ID)<<16), RK3288_CRU_MODE_CON);

}


static void clks_gating_suspend_init(void)
{
    // get clk gating info
    if(rockchip_pie_chunk)
        p_rkpm_clkgt_last_set= kern_to_pie(rockchip_pie_chunk, &DATA(rkpm_clkgt_last_set[0]));
    else
        p_rkpm_clkgt_last_set=&clk_ungt_msk_1[0];
    if(clk_suspend_clkgt_info_get(clk_ungt_msk,p_rkpm_clkgt_last_set, RK3288_CRU_CLKGATES_CON_CNT) 
        ==RK3288_CRU_CLKGATES_CON(0))
    {
        rkpm_set_ops_gtclks(gtclks_suspend,gtclks_resume);
        if(rockchip_pie_chunk)
            rkpm_set_sram_ops_gtclks(fn_to_pie(rockchip_pie_chunk, &FUNC(gtclks_sram_suspend)), 
                                fn_to_pie(rockchip_pie_chunk, &FUNC(gtclks_sram_resume)));
        
        PM_LOG("%s:clkgt info ok\n",__FUNCTION__);

    }
    if(rockchip_pie_chunk)
        rkpm_set_sram_ops_sysclk(fn_to_pie(rockchip_pie_chunk, &FUNC(sysclk_suspend))
                                                ,fn_to_pie(rockchip_pie_chunk, &FUNC(sysclk_resume))); 
}

/***************************prepare and finish reg_pread***********************************/



#define GIC_DIST_PENDING_SET		0x200
#define DUMP_GPIO_INT_STATUS(ID) \
do { \
	if (irq_gpio & (1 << ID)) \
		printk("wakeup gpio" #ID ": %08x\n", readl_relaxed(RK_GPIO_VIRT(ID) + GPIO_INT_STATUS)); \
} while (0)
static noinline void rk30_pm_dump_irq(void)
{
	u32 irq_gpio = (readl_relaxed(RK_GIC_VIRT + GIC_DIST_PENDING_SET + 12) >> 17) & 0x1FF;
	printk("wakeup irq: %08x %08x %08x %08x\n",
		readl_relaxed(RK_GIC_VIRT + GIC_DIST_PENDING_SET + 4),
		readl_relaxed(RK_GIC_VIRT + GIC_DIST_PENDING_SET + 8),
		readl_relaxed(RK_GIC_VIRT + GIC_DIST_PENDING_SET + 12),
		readl_relaxed(RK_GIC_VIRT + GIC_DIST_PENDING_SET + 16));
        DUMP_GPIO_INT_STATUS(0);
        DUMP_GPIO_INT_STATUS(1);
        DUMP_GPIO_INT_STATUS(2);
        DUMP_GPIO_INT_STATUS(3);
        DUMP_GPIO_INT_STATUS(4);
        DUMP_GPIO_INT_STATUS(5);
        DUMP_GPIO_INT_STATUS(6);
        DUMP_GPIO_INT_STATUS(7);
        DUMP_GPIO_INT_STATUS(8);
        
}

#define DUMP_GPIO_INTEN(ID) \
do { \
	u32 en = readl_relaxed(RK_GPIO_VIRT(ID) + GPIO_INTEN); \
	if (en) { \
		rkpm_ddr_printascii("GPIO" #ID "_INTEN: "); \
		rkpm_ddr_printhex(en); \
		rkpm_ddr_printch('\n'); \
		printk(KERN_DEBUG "GPIO%d_INTEN: %08x\n", ID, en); \
	} \
} while (0)

//dump while irq is enable
static noinline void rk30_pm_dump_inten(void)
{
	DUMP_GPIO_INTEN(0);
	DUMP_GPIO_INTEN(1);
	DUMP_GPIO_INTEN(2);
	DUMP_GPIO_INTEN(3);
    	DUMP_GPIO_INTEN(4);
	DUMP_GPIO_INTEN(5);
	DUMP_GPIO_INTEN(6);
	DUMP_GPIO_INTEN(7);    
	DUMP_GPIO_INTEN(8);
}

static  void rkpm_prepare(void)
{   
        #if 0
        u32 temp =reg_readl(RK_GPIO_VIRT(0)+0x30);

       // rkpm_ddr_printhex(temp);
        reg_writel(temp|0x1<<4,RK_GPIO_VIRT(0)+0x30);
        temp =reg_readl(RK_GPIO_VIRT(0)+0x30);
       // rkpm_ddr_printhex(temp);
        #endif             
	// dump GPIO INTEN for debug
	rk30_pm_dump_inten();
}

static void rkpm_finish(void)
{
	rk30_pm_dump_irq();
}


static  void interface_ctr_reg_pread(void)
{
	//u32 addr;
	flush_cache_all();
	outer_flush_all();
	local_flush_tlb_all();
        #if 0  // do it in ddr suspend 
	for (addr = (u32)SRAM_CODE_OFFSET; addr < (u32)(SRAM_CODE_OFFSET+rockchip_sram_size); addr += PAGE_SIZE)
		readl_relaxed(addr);
        #endif
        readl_relaxed(RK_PMU_VIRT);
        readl_relaxed(RK_GRF_VIRT);
        readl_relaxed(RK_DDR_VIRT);
        readl_relaxed(RK_GPIO_VIRT(0));     
        //readl_relaxed(RK30_I2C1_BASE+SZ_4K);
        //readl_relaxed(RK_GPIO_VIRT(3));
}
void PIE_FUNC(ddr_leakage_tst)(void)
{
    cru_writel(RK3288_PLL_MODE_SLOW(DPLL_ID), RK3288_CRU_MODE_CON);    
    rkpm_sram_printch('\n');   
    rkpm_sram_printch('t');   
    rkpm_sram_printch('e');   
    rkpm_sram_printch('s');
    rkpm_sram_printch('t');   
    while(1);               
}

static void __init  rk3288_suspend_init(void)
{
    struct device_node *parent;
    u32 pm_ctrbits;

    PM_LOG("%s enter\n",__FUNCTION__);

    parent = of_find_node_by_name(NULL, "rockchip_suspend");    

    if (IS_ERR_OR_NULL(parent)) {
		PM_ERR("%s dev node err\n", __func__);
		return;
	}


    if(of_property_read_u32_array(parent,"rockchip,ctrbits",&pm_ctrbits,1))
    {
            PM_ERR("%s:get pm ctr error\n",__FUNCTION__);
            return ;
    }
    PM_LOG("%s: pm_ctrbits =%x\n",__FUNCTION__,pm_ctrbits);
    
    rkpm_set_ctrbits(pm_ctrbits);
    
    clks_gating_suspend_init();

    rkpm_set_ops_plls(pm_plls_suspend,pm_plls_resume);
    
    //rkpm_set_sram_ops_ddr(fn_to_pie(rockchip_pie_chunk, &FUNC(ddr_leakage_tst)),NULL);
    rkpm_set_ops_prepare_finish(rkpm_prepare,rkpm_finish);
    //rkpm_set_ops_regs_pread(interface_ctr_reg_pread);  
    
    rkpm_set_ops_regs_sleep(pm_sleep_func_save,rkpm_slp_mode_set,pm_sleep_func_rusume_first,pm_sleep_func_rusume_last);
    if(rockchip_pie_chunk)
        rkpm_set_sram_ops_printch(fn_to_pie(rockchip_pie_chunk, &FUNC(sram_printch)));
    
    rkpm_set_ops_printch(ddr_printch); 	
}
