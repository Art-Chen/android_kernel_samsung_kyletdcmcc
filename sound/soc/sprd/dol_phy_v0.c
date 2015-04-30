#include "audio_dev_phy.h"
#include "vbc-codec.h"
#include "sc88xx-asoc.h"

#define VBPMR2_RLGOD_SHIFT 4
#define VBPMR2_RLGOD_MASK  (0x3)

#define VBCGR8_LRGO_SHIFT 6
#define VBCGR8_LRGO_MASK    (0x3)

#define VBCGR1_GODL_SHIFT 0
#define VBCGR1_GODL_MASK    (0xf)

#define VBCGR8_GOL_SHIFT 0
#define VBCGR8_GOL_MASK    (0x1f)

#define VBCGR1_GODR_SHIFT 4
#define VBCGR1_GODR_MASK    (0xf)

#define VBCGR9_GOR_SHIFT 0
#define VBCGR9_GOR_MASK    (0x1f)

PUBLIC void CODEC_PHY_SetDACPGA(CODEC_DAC_OUTPUT_PGA_T pga)
{
	vbc_reg_write(VBPMR2,0,0,VBPMR2_RLGOD_MASK);
	vbc_reg_write(VBCGR8,0,0,VBCGR8_LRGO_MASK);	
//	printk("1CODEC_PHY_SetDACPGA n.......hp_pga_l:[0x%x] hp_pga_r:[0x%x] cgr8:[0x%x] \n",pga.hp_pga_l,pga.hp_pga_r,vbc_reg_read(VBCGR8,VBCGR9_GOR_SHIFT,VBCGR9_GOR_MASK));

	vbc_reg_write(VBCGR1,VBCGR1_GODL_SHIFT,pga.dac_pga_l,VBCGR1_GODL_MASK);
	vbc_reg_write(VBCGR8,VBCGR9_GOR_SHIFT,pga.hp_pga_r,VBCGR9_GOR_MASK);
//	printk("2CODEC_PHY_SetDACPGA set left..... cgr8:[0x%x]\n ",vbc_reg_read(VBCGR8,VBCGR9_GOR_SHIFT,VBCGR9_GOR_MASK));

//	printk("2.5 CODEC_PHY_SetDACPGA set right in cgr8:[0x%x] \n",vbc_reg_read(VBCGR8,VBCGR9_GOR_SHIFT,VBCGR9_GOR_MASK));
	vbc_reg_write(VBCGR1,VBCGR1_GODR_SHIFT,pga.dac_pga_r,VBCGR1_GODR_MASK);
	vbc_reg_write(VBCGR9,VBCGR9_GOR_SHIFT,pga.hp_pga_r,VBCGR9_GOR_MASK);
//	printk("3CODEC_PHY_SetDACPGA after set right ...cgr8:[0x%x]\n ",vbc_reg_read(VBCGR8,VBCGR9_GOR_SHIFT,VBCGR9_GOR_MASK));

//	printk("4CODEC_PHY_SetDACPGA out.......cgr8:[0x%x]\n ",vbc_reg_read(VBCGR8,VBCGR9_GOR_SHIFT,VBCGR9_GOR_MASK));

}
