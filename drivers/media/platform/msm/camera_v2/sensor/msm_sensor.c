/* Copyright (c) 2011-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include <linux/regulator/rpm-smd-regulator.h>
#include <linux/regulator/consumer.h>
#include<linux/delay.h> 
#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

extern struct otp_struct otp_ptr;
static void msm_sensor_adjust_mclk(struct msm_camera_power_ctrl_t *ctrl)
{
	int idx;
	struct msm_sensor_power_setting *power_setting;
	for (idx = 0; idx < ctrl->power_setting_size; idx++) {
		power_setting = &ctrl->power_setting[idx];
		if (power_setting->seq_type == SENSOR_CLK &&
			power_setting->seq_val ==  SENSOR_CAM_MCLK) {
			if (power_setting->config_val == 24000000) {
				power_setting->config_val = 23880000;
				CDBG("%s MCLK request adjusted to 23.88MHz\n"
							, __func__);
			}
			break;
		}
	}

	return;
}

static void msm_sensor_misc_regulator(
	struct msm_sensor_ctrl_t *sctrl, uint32_t enable)
{
	int32_t rc = 0;
	if (enable) {
		sctrl->misc_regulator = (void *)rpm_regulator_get(
			&sctrl->pdev->dev, sctrl->sensordata->misc_regulator);
		if (sctrl->misc_regulator) {
			rc = rpm_regulator_set_mode(sctrl->misc_regulator,
				RPM_REGULATOR_MODE_HPM);
			if (rc < 0) {
				pr_err("%s: Failed to set for rpm regulator on %s: %d\n",
					__func__,
					sctrl->sensordata->misc_regulator, rc);
				rpm_regulator_put(sctrl->misc_regulator);
			}
		} else {
			pr_err("%s: Failed to vote for rpm regulator on %s: %d\n",
				__func__,
				sctrl->sensordata->misc_regulator, rc);
		}
	} else {
		if (sctrl->misc_regulator) {
			rc = rpm_regulator_set_mode(
				(struct rpm_regulator *)sctrl->misc_regulator,
				RPM_REGULATOR_MODE_AUTO);
			if (rc < 0)
				pr_err("%s: Failed to set for rpm regulator on %s: %d\n",
					__func__,
					sctrl->sensordata->misc_regulator, rc);
			rpm_regulator_put(sctrl->misc_regulator);
		}
	}
}

int32_t msm_sensor_free_sensor_data(struct msm_sensor_ctrl_t *s_ctrl)
{
	if (!s_ctrl->pdev && !s_ctrl->sensor_i2c_client->client)
		return 0;
	kfree(s_ctrl->sensordata->slave_info);
	kfree(s_ctrl->sensordata->cam_slave_info);
	kfree(s_ctrl->sensordata->actuator_info);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->gpio_num_info);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_req_tbl);
	kfree(s_ctrl->sensordata->power_info.gpio_conf);
	kfree(s_ctrl->sensordata->power_info.cam_vreg);
	kfree(s_ctrl->sensordata->power_info.power_setting);
	kfree(s_ctrl->sensordata->power_info.power_down_setting);
	kfree(s_ctrl->sensordata->csi_lane_params);
	kfree(s_ctrl->sensordata->sensor_info);
	if (s_ctrl->sensor_device_type == MSM_CAMERA_I2C_DEVICE) {
		msm_camera_i2c_dev_put_clk_info(
			&s_ctrl->sensor_i2c_client->client->dev,
			&s_ctrl->sensordata->power_info.clk_info,
			&s_ctrl->sensordata->power_info.clk_ptr,
			s_ctrl->sensordata->power_info.clk_info_size);
	} else {
		msm_camera_put_clk_info(s_ctrl->pdev,
			&s_ctrl->sensordata->power_info.clk_info,
			&s_ctrl->sensordata->power_info.clk_ptr,
			s_ctrl->sensordata->power_info.clk_info_size);
	}

	kfree(s_ctrl->sensordata);
	return 0;
}
void delay(unsigned int xms) // xms代表需要延时的毫秒数
{
   unsigned int x;
   for(x=xms;x>0;x--);
}
#define DD_PARAM_QTY 		200
#define WINDOW_WIDTH  		0x0a30 //2608 max effective pixels
#define WINDOW_HEIGHT 		0x079c //1948
#define REG_ROM_START 		0x62

#define RG_TYPICAL    		0x0400
#define BG_TYPICAL			0x0400
#define INFO_ROM_START		0x01
#define INFO_WIDTH       	0x08
#define WB_ROM_START      	0x11
#define WB_WIDTH          	0x05
#define GOLDEN_ROM_START  	0x1c
#define GOLDEN_WIDTH      	0x05



typedef struct otp_gc5025a
{
	uint16_t dd_param_x[DD_PARAM_QTY];
	uint16_t dd_param_y[DD_PARAM_QTY];
	uint16_t dd_param_type[DD_PARAM_QTY];
	uint16_t dd_cnt;
	uint16_t dd_flag;
	uint16_t reg_addr[10];	
	uint16_t reg_value[10];	
	uint16_t reg_num;
			
    uint16_t infowbvalid;
	uint16_t module_id;
	uint16_t lens_id;
	uint16_t year;
	uint16_t month;
	uint16_t day;
	uint16_t rg_gain;
	uint16_t bg_gain;
	uint16_t golden_rg;
	uint16_t golden_bg;
    uint16_t afvalid;
    uint16_t af_inf;
    uint16_t af_macro;	
}gc5025a_otp;
static uint8_t gc5025a_otp_read;
static gc5025a_otp gc5025a_otp_info;
static void write_cmos_sensor(struct msm_sensor_ctrl_t *s_ctrl,uint8_t addr,uint8_t value)
{
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,addr,value,MSM_CAMERA_I2C_BYTE_ADDR);
}
static void write_cmos_sensor_u16(struct msm_sensor_ctrl_t *s_ctrl,uint16_t addr,uint8_t value)
{
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,addr,value,MSM_CAMERA_I2C_BYTE_DATA);
}
int read_cmos_sensor_u16(struct msm_sensor_ctrl_t *s_ctrl,int addr)
{
	uint16_t data;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, addr,&data,MSM_CAMERA_I2C_BYTE_DATA);
	return data;
}
int read_cmos_sensor(struct msm_sensor_ctrl_t *s_ctrl,int addr)
{
	uint16_t data;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, addr,&data,MSM_CAMERA_I2C_BYTE_ADDR);
	return data;
}
static void gc5025a_gcore_initial_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	write_cmos_sensor(s_ctrl,0xfe,0x00);
	write_cmos_sensor(s_ctrl,0xfe,0x00);
	write_cmos_sensor(s_ctrl,0xfe,0x00);
	write_cmos_sensor(s_ctrl,0xf7,0x01);
	write_cmos_sensor(s_ctrl,0xf8,0x11);
	write_cmos_sensor(s_ctrl,0xf9,0x00);
	write_cmos_sensor(s_ctrl,0xfa,0xa0);
	write_cmos_sensor(s_ctrl,0xfc,0x2e);
}
static uint8_t gc5025a_read_otp(struct msm_sensor_ctrl_t *s_ctrl,uint8_t addr)
{
	uint8_t value;
	uint8_t regd4;
	uint16_t realaddr = addr * 8;
	regd4 = read_cmos_sensor(s_ctrl,0xd4);

	write_cmos_sensor(s_ctrl,0xfe,0x00);
	write_cmos_sensor(s_ctrl,0xd4,(regd4&0xfc)+((realaddr>>8)&0x03));
	write_cmos_sensor(s_ctrl,0xd5,realaddr&0xff);
	write_cmos_sensor(s_ctrl,0xf3,0x20);
	value = read_cmos_sensor(s_ctrl,0xd7);

	return value;
}
static void gc5025a_read_otp_group(struct msm_sensor_ctrl_t *s_ctrl,uint8_t addr,uint8_t* buff,int size)
{
	uint8_t i;
	uint8_t regd4;
	uint16_t realaddr = addr * 8;
	regd4 = read_cmos_sensor(s_ctrl,0xd4);	

	write_cmos_sensor(s_ctrl,0xfe,0x00);
	write_cmos_sensor(s_ctrl,0xd4,(regd4&0xfc)+((realaddr>>8)&0x03));
	write_cmos_sensor(s_ctrl,0xd5,realaddr);
	write_cmos_sensor(s_ctrl,0xf3,0x20);
	write_cmos_sensor(s_ctrl,0xf3,0x88);
	
	for(i=0;i<size;i++)
	{
		buff[i] = read_cmos_sensor(s_ctrl,0xd7);
	}
}
static void gc5025a_select_page_otp(struct msm_sensor_ctrl_t *s_ctrl,uint8_t otp_select_page)
{
	uint8_t page;
	write_cmos_sensor(s_ctrl,0xfe,0x00);
	page = read_cmos_sensor(s_ctrl,0xd4);

	switch(otp_select_page)
	{
	case 0:
		page = page & 0xfb;
		break;
	case 1:
		page = page | 0x04;
		break;
	default:
		break;
	}

	mdelay(5);
	write_cmos_sensor(s_ctrl,0xd4,page);	

}

static void gc5025a_gcore_enable_otp(struct msm_sensor_ctrl_t *s_ctrl,uint8_t state)
{
	uint8_t otp_clk,otp_en;
	otp_clk = read_cmos_sensor(s_ctrl,0xfa);
	otp_en= read_cmos_sensor(s_ctrl,0xd4);	
	if(state)	
	{ 
		otp_clk = otp_clk | 0x10;
		otp_en = otp_en | 0x80;
		mdelay(5);
		write_cmos_sensor(s_ctrl,0xfa,otp_clk);// 0xfa[6]:OTP_CLK_en
		write_cmos_sensor(s_ctrl,0xd4,otp_en);	// 0xd4[7]:OTP_en	
	
		pr_err("GC5025A_OTP: Enable OTP!\n");		
	}
	else			
	{
		otp_en = otp_en & 0x7f;
		otp_clk = otp_clk & 0xef;
		mdelay(5);
		write_cmos_sensor(s_ctrl,0xd4,otp_en);
		write_cmos_sensor(s_ctrl,0xfa,otp_clk);

		pr_err("GC5025A_OTP: Disable OTP!\n");
	}

}
static void gc5025a_gcore_read_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t flagdd,flag_chipversion;
	uint8_t i,j,cnt=0;
	uint16_t check;
	uint8_t total_number=0; 
	uint8_t ddtempbuff[50*4];
	uint8_t ddchecksum;

	uint8_t infowb_flag,af_flag;
	uint8_t info[14];
    uint8_t afcode[4];
    uint8_t InfHigh,MacHigh;
	//uint8_t wb[5];
	//uint8_t golden[5];

	memset(&gc5025a_otp_info,0,sizeof(gc5025a_otp));

	/*TODO*/
	gc5025a_select_page_otp(s_ctrl,0);
	flagdd = gc5025a_read_otp(s_ctrl,0x00);
	pr_err("GC5025A_OTP_DD : flag_dd = 0x%x\n",flagdd);
	flag_chipversion= gc5025a_read_otp(s_ctrl,0x7f);

	//DD
	switch(flagdd&0x03)
	{
	case 0x00:
		pr_err("GC5025A_OTP_DD is Empty !!\n");
		gc5025a_otp_info.dd_flag = 0x00;
		break;
	case 0x01:	
		pr_err("GC5025A_OTP_DD is Valid!!\n");
		total_number = gc5025a_read_otp(s_ctrl,0x01) + gc5025a_read_otp(s_ctrl,0x02);
		pr_err("GC5025A_OTP : total_number = %d\n",total_number);
		
		if (total_number <= 31)
		{
			gc5025a_read_otp_group(s_ctrl,0x03, &ddtempbuff[0], total_number * 4);
		}
		else
		{
			gc5025a_read_otp_group(s_ctrl,0x03, &ddtempbuff[0], 31 * 4);
			gc5025a_select_page_otp(s_ctrl,1);
			gc5025a_read_otp_group(s_ctrl,0x29, &ddtempbuff[31 * 4], (total_number - 31) * 4);
		}

		/*DD check sum*/
		gc5025a_select_page_otp(s_ctrl,1);
		ddchecksum = gc5025a_read_otp(s_ctrl,0x61);
		check = total_number;
		for(i = 0; i < 4 * total_number; i++)
		{
			check += ddtempbuff[i];
		}
		if((check % 255 + 1) == ddchecksum)
		{
			pr_err("GC5025A_OTP_DD : DD check sum correct! checksum = 0x%x\n", ddchecksum);
		}
		else
		{
			pr_err("GC5025A_OTP_DD : DD check sum error! otpchecksum = 0x%x, sum = 0x%x\n", ddchecksum, (check % 255 + 1));
		}

		for(i=0; i<total_number; i++)
		{
			pr_err("GC5025A_OTP_DD:index = %d, data[0] = %x , data[1] = %x , data[2] = %x ,data[3] = %x \n",\
				i, ddtempbuff[4 * i], ddtempbuff[4 * i + 1], ddtempbuff[4 * i + 2], ddtempbuff[4 * i + 3]);
			
			if (ddtempbuff[4 * i + 3] & 0x10)
			{
				switch (ddtempbuff[4 * i + 3] & 0x0f)
				{
				case 3:
					for (j = 0; j < 4; j++)
					{
						gc5025a_otp_info.dd_param_x[cnt] = (((uint16_t)ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
						gc5025a_otp_info.dd_param_y[cnt] = ((uint16_t)ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4) + j;
						gc5025a_otp_info.dd_param_type[cnt++] = 2;
					}
					break;
				case 4:
					for (j = 0; j < 2; j++)
					{
						gc5025a_otp_info.dd_param_x[cnt] = (((uint16_t)ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
						gc5025a_otp_info.dd_param_y[cnt] = ((uint16_t)ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4) + j;
						gc5025a_otp_info.dd_param_type[cnt++] = 2;
					}
					break;
				default:
					gc5025a_otp_info.dd_param_x[cnt] = (((uint16_t)ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
					gc5025a_otp_info.dd_param_y[cnt] = ((uint16_t)ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4);
					gc5025a_otp_info.dd_param_type[cnt++] = ddtempbuff[4 * i + 3] & 0x0f;
					break;
				}
			}
			else
			{
				pr_err("GC5025A_OTP_DD:check_id[%d] = %x,checkid error!!\n", i, ddtempbuff[4 * i + 3] & 0xf0);
			}
		}

		gc5025a_otp_info.dd_cnt = cnt;
		gc5025a_otp_info.dd_flag = 0x01;
		break;
	case 0x02:
	case 0x03:	
		pr_err("GC5025A_OTP_DD is Invalid !!\n");
		gc5025a_otp_info.dd_flag = 0x02;
		break;
	default :
		break;
	}

/*For Chip Version*/
	pr_err("GC5025A_OTP_CHIPVESION : flag_chipversion = 0x%x\n",flag_chipversion);

	switch((flag_chipversion>>4)&0x03)
	{
	case 0x00:
		pr_err("GC5025A_OTP_CHIPVERSION is Empty !!\n");
		break;
	case 0x01:
		pr_err("GC5025A_OTP_CHIPVERSION is Valid !!\n");
		gc5025a_select_page_otp(s_ctrl,1);		
		i = 0;
		do{
			gc5025a_otp_info.reg_addr[i] = gc5025a_read_otp(s_ctrl,REG_ROM_START + i*2 ) ;
			gc5025a_otp_info.reg_value[i] = gc5025a_read_otp(s_ctrl,REG_ROM_START + i*2 + 1 ) ;
			pr_err("GC5025A_OTP_CHIPVERSION reg_addr[%d] = 0x%x,reg_value[%d] = 0x%x\n",i,gc5025a_otp_info.reg_addr[i],i,gc5025a_otp_info.reg_value[i]);			
			i++;			
		}while((gc5025a_otp_info.reg_addr[i-1]!=0)&&(i<10));
		gc5025a_otp_info.reg_num = i - 1;
		break;
	case 0x02:
		pr_err("GC5025A_OTP_CHIPVERSION is Invalid !!\n");
		break;
	default :
		break;
	}
	

	gc5025a_select_page_otp(s_ctrl,1);
	infowb_flag = gc5025a_read_otp(s_ctrl,0x00);//read InfoWBFlag
	af_flag =     gc5025a_read_otp(s_ctrl,0x1d);//read AFFlag
	pr_err("GC5025A_OTP : infowb_flag = 0x%x , af_flag = 0x%x\n",infowb_flag,af_flag);
//INFO&WB
        if(infowb_flag == 0x01) 
        {   
            gc5025a_otp_info.infowbvalid = 1;
            gc5025a_read_otp_group(s_ctrl,1, &info[0], 14);       
        }
        else if(infowb_flag == 0x07)
        {
            gc5025a_otp_info.infowbvalid = 2;
            gc5025a_read_otp_group(s_ctrl,15, &info[0], 14);
        }
        else
        {   
            gc5025a_otp_info.infowbvalid = 0;
            pr_err("infowb_flag  invalid !!\n");
        }
        if (gc5025a_otp_info.infowbvalid) 
        {   
            check = 0;
            for (i=0;i<13;i++)
            {
                pr_err("infowb  Group 1 value =%x !!\n",info[i]);
                check += info[i];
            }
            if ((check % 255 + 1) == info[13])
            {
                pr_err("AWB checksum is ok !!\n");
                gc5025a_otp_info.module_id = info[0];
                gc5025a_otp_info.lens_id = info[1];
                gc5025a_otp_info.year = info[2];
                gc5025a_otp_info.month = info[3];
                gc5025a_otp_info.day = info[4];
                gc5025a_otp_info.rg_gain = info[5]<<8|info[6];
                gc5025a_otp_info.bg_gain = info[7]<<8|info[8];
                gc5025a_otp_info.golden_rg = info[9]<<8|info[10];
                gc5025a_otp_info.golden_bg = info[11]<<8|info[12];
            }
            else
            {   gc5025a_otp_info.infowbvalid = 0;
                pr_err("GC5025A_OTP_INFO Check sum %d Error !!\n", info[13]);
            }

        }
        if(af_flag == 0x01) 
        {   
            gc5025a_otp_info.afvalid = 1;
            gc5025a_read_otp_group(s_ctrl,30, &afcode[0], 4);       
        }
        else if(af_flag == 0x07)
        {   
            gc5025a_otp_info.afvalid = 2;
            gc5025a_read_otp_group(s_ctrl,34, &afcode[0], 4);
        }
        else
        {   
            gc5025a_otp_info.afvalid = 0;
            pr_err("af_flag  invalid !!\n");
        }
        if (gc5025a_otp_info.afvalid) 
        {
            check = 0;
            for (i=0;i<3;i++)
            {
                pr_err("af Group 1 value =%x !!\n",afcode[i]);
                check += afcode[i];
            }
            if ((check % 255 + 1) == afcode[3])
            {   
                pr_err("AF checksum is ok !!\n");
                InfHigh=  afcode[0]>>4;
                MacHigh = afcode[0]&0x0f;
                gc5025a_otp_info.af_inf =   InfHigh<<8|afcode[1];
                gc5025a_otp_info.af_macro = MacHigh<<8|afcode[2];
            }
            else
            {
                gc5025a_otp_info.afvalid=0;
                pr_err("GC5025A_OTP_AF Check sum %d Error !!\n", info[3]);
            }
               
        }
        gc5025a_otp_read = 1;
	/*print otp information*/
	pr_err("GC5025A_OTP_INFO:module_id=0x%x\n",gc5025a_otp_info.module_id);
	pr_err("GC5025A_OTP_INFO:lens_id=0x%x\n",gc5025a_otp_info.lens_id);
	pr_err("GC5025A_OTP_INFO:date=%d-%d-%d\n",gc5025a_otp_info.year,gc5025a_otp_info.month,gc5025a_otp_info.day);
	pr_err("GC5025A_OTP_WB:r/g=0x%x\n",gc5025a_otp_info.rg_gain);
	pr_err("GC5025A_OTP_WB:b/g=0x%x\n",gc5025a_otp_info.bg_gain);
	pr_err("GC5025A_OTP_GOLDEN:golden_rg=0x%x\n",gc5025a_otp_info.golden_rg);
	pr_err("GC5025A_OTP_GOLDEN:golden_bg=0x%x\n",gc5025a_otp_info.golden_bg);
    pr_err("GC5025A_OTP_GOLDEN:af_inf=0x%d\n",gc5025a_otp_info.af_inf);
	pr_err("GC5025A_OTP_GOLDEN:af_macro=0x%d\n",gc5025a_otp_info.af_macro);
		

}
//#define IMAGE_NORMAL_MIRROR
static void gc5025a_gcore_update_dd(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t i=0,j=0,n=0,m=0,s=0,e=0;
	uint16_t temp_x=0,temp_y=0;
	uint8_t flag=0;
	uint8_t temp_type=0;
	uint8_t temp_val0,temp_val1,temp_val2;
	//TODO
	pr_err("%s stoneadd gc5025a_gcore_update_dd \n",__func__);
	if(0x01 ==gc5025a_otp_info.dd_flag)
	{
#if defined(IMAGE_NORMAL_MIRROR)
	//do nothing
#elif defined(IMAGE_H_MIRROR)
	for(i=0; i<gc5025a_otp_info.dd_cnt; i++)
	{
		if(gc5025a_otp_info.dd_param_type[i]==0)
		{	gc5025a_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025a_otp_info.dd_param_x[i]+1;	}
		else if(gc5025a_otp_info.dd_param_type[i]==1)
		{	gc5025a_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025a_otp_info.dd_param_x[i]-1;	}
		else
		{	gc5025a_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025a_otp_info.dd_param_x[i] ;	}
	}
#elif defined(IMAGE_V_MIRROR)
		for(i=0; i<gc5025a_otp_info.dd_cnt; i++)
		{	gc5025a_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025a_otp_info.dd_param_y[i] + 1;	}

#elif defined(IMAGE_HV_MIRROR)
	for(i=0; i<gc5025a_otp_info.dd_cnt; i++)
		{
			if(gc5025a_otp_info.dd_param_type[i]==0)
			{	
				gc5025a_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025a_otp_info.dd_param_x[i]+1;
				gc5025a_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025a_otp_info.dd_param_y[i]+1;
			}
			else if(gc5025a_otp_info.dd_param_type[i]==1)
			{
				gc5025a_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025a_otp_info.dd_param_x[i]-1;
				gc5025a_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025a_otp_info.dd_param_y[i]+1;
			}
			else
			{
				gc5025a_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025a_otp_info.dd_param_x[i] ;
				gc5025a_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025a_otp_info.dd_param_y[i] + 1;
			}
		}
#endif

		//y
		for(i=0 ; i< gc5025a_otp_info.dd_cnt-1; i++) 
		{
			for(j = i+1; j < gc5025a_otp_info.dd_cnt; j++) 
			{  
				if(gc5025a_otp_info.dd_param_y[i] > gc5025a_otp_info.dd_param_y[j])  
				{  
					temp_x = gc5025a_otp_info.dd_param_x[i] ; gc5025a_otp_info.dd_param_x[i] = gc5025a_otp_info.dd_param_x[j] ;  gc5025a_otp_info.dd_param_x[j] = temp_x;
					temp_y = gc5025a_otp_info.dd_param_y[i] ; gc5025a_otp_info.dd_param_y[i] = gc5025a_otp_info.dd_param_y[j] ;  gc5025a_otp_info.dd_param_y[j] = temp_y;
					temp_type = gc5025a_otp_info.dd_param_type[i] ; gc5025a_otp_info.dd_param_type[i] = gc5025a_otp_info.dd_param_type[j]; gc5025a_otp_info.dd_param_type[j]= temp_type;
				} 
			}
		
		}
		
		//x
		for(i=0; i<gc5025a_otp_info.dd_cnt; i++)
		{
			if(gc5025a_otp_info.dd_param_y[i]==gc5025a_otp_info.dd_param_y[i+1])
			{
				s=i++;
				while((gc5025a_otp_info.dd_param_y[s] == gc5025a_otp_info.dd_param_y[i+1])&&(i<gc5025a_otp_info.dd_cnt-1))
					i++;
				e=i;

				for(n=s; n<e; n++)
				{
					for(m=n+1; m<e+1; m++)
					{
						if(gc5025a_otp_info.dd_param_x[n] > gc5025a_otp_info.dd_param_x[m])
						{
							temp_x = gc5025a_otp_info.dd_param_x[n] ; gc5025a_otp_info.dd_param_x[n] = gc5025a_otp_info.dd_param_x[m] ;  gc5025a_otp_info.dd_param_x[m] = temp_x;
							temp_y = gc5025a_otp_info.dd_param_y[n] ; gc5025a_otp_info.dd_param_y[n] = gc5025a_otp_info.dd_param_y[m] ;  gc5025a_otp_info.dd_param_y[m] = temp_y;
							temp_type = gc5025a_otp_info.dd_param_type[n] ; gc5025a_otp_info.dd_param_type[n] = gc5025a_otp_info.dd_param_type[m]; gc5025a_otp_info.dd_param_type[m]= temp_type;
						}
					}
				}

			}

		}

		
		//write SRAM
		write_cmos_sensor(s_ctrl,0xfe, 0x01);
		write_cmos_sensor(s_ctrl,0xa8, 0x00);
		write_cmos_sensor(s_ctrl,0x9d, 0x04);
		write_cmos_sensor(s_ctrl,0xbe, 0x00);
		write_cmos_sensor(s_ctrl,0xa9, 0x01);

		for(i=0; i<gc5025a_otp_info.dd_cnt; i++)
		{
			temp_val0 = gc5025a_otp_info.dd_param_x[i]& 0x00ff;
			temp_val1 = (((gc5025a_otp_info.dd_param_y[i])<<4)& 0x00f0) + ((gc5025a_otp_info.dd_param_x[i]>>8)&0X000f);
			temp_val2 = (gc5025a_otp_info.dd_param_y[i]>>4) & 0xff;
			write_cmos_sensor(s_ctrl,0xaa,i);
			write_cmos_sensor(s_ctrl,0xac,temp_val0);
			write_cmos_sensor(s_ctrl,0xac,temp_val1);
			write_cmos_sensor(s_ctrl,0xac,temp_val2);
			while((i < gc5025a_otp_info.dd_cnt - 1) && (gc5025a_otp_info.dd_param_x[i]==gc5025a_otp_info.dd_param_x[i+1]) && (gc5025a_otp_info.dd_param_y[i]==gc5025a_otp_info.dd_param_y[i+1]))
				{
					flag = 1;
					i++;
				}
			if(flag)
				write_cmos_sensor(s_ctrl,0xac,0x02);
			else
				write_cmos_sensor(s_ctrl,0xac,gc5025a_otp_info.dd_param_type[i]);
			
			pr_err("GC5025A_OTP_GC val0 = 0x%x , val1 = 0x%x , val2 = 0x%x \n",temp_val0,temp_val1,temp_val2);
			pr_err("GC5025A_OTP_GC x = %d , y = %d \n",((temp_val1&0x0f)<<8) + temp_val0,(temp_val2<<4) + ((temp_val1&0xf0)>>4));	
		}

		write_cmos_sensor(s_ctrl,0xbe,0x01);
		write_cmos_sensor(s_ctrl,0xfe,0x00);
	}

}
static void gc5025a_gcore_update_wb(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t r_gain_current = 0 , g_gain_current = 0 , b_gain_current = 0 , base_gain = 0;
	uint16_t r_gain = 1024 , g_gain = 1024 , b_gain = 1024 ;
	uint16_t rg_typical,bg_typical;	 
	pr_err("%s stoneadd gc5025a_gcore_update_wb \n",__func__);
        if (gc5025a_otp_info.infowbvalid) 
        {
        rg_typical=gc5025a_otp_info.golden_rg;
		bg_typical=gc5025a_otp_info.golden_bg;

		pr_err("GC5025A_OTP_UPDATE_AWB:rg_typical = 0x%x , bg_typical = 0x%x\n",rg_typical,bg_typical);
		r_gain_current = 1024 * rg_typical/gc5025a_otp_info.rg_gain;
		b_gain_current = 1024 * bg_typical/gc5025a_otp_info.bg_gain;
		g_gain_current = 1024;

		base_gain = (r_gain_current<b_gain_current) ? r_gain_current : b_gain_current;
		base_gain = (base_gain<g_gain_current) ? base_gain : g_gain_current;
		pr_err("GC5025A_OTP_UPDATE_AWB:r_gain_current = 0x%x , b_gain_current = 0x%x , base_gain = 0x%x \n",r_gain_current,b_gain_current,base_gain);

		r_gain = 0x400 * r_gain_current / base_gain;
		g_gain = 0x400 * g_gain_current / base_gain;
		b_gain = 0x400 * b_gain_current / base_gain;
		pr_err("GC5025A_OTP_UPDATE_AWB:r_gain = 0x%x , g_gain = 0x%x , b_gain = 0x%x \n",r_gain,g_gain,b_gain);

		//TODO
		write_cmos_sensor(s_ctrl,0xfe,0x00);
		write_cmos_sensor(s_ctrl,0xc6,g_gain>>3);
		write_cmos_sensor(s_ctrl,0xc7,r_gain>>3);
		write_cmos_sensor(s_ctrl,0xc8,b_gain>>3);
		write_cmos_sensor(s_ctrl,0xc9,g_gain>>3);
		write_cmos_sensor(s_ctrl,0xc4,((g_gain&0x07) << 4) + (r_gain&0x07));
		write_cmos_sensor(s_ctrl,0xc5,((b_gain&0x07) << 4) + (g_gain&0x07));

        }

}

static void gc5025a_gcore_identify_otp(struct msm_sensor_ctrl_t *s_ctrl)
{	
    pr_err("%s stoneadd gc5025a_gcore_identify_otp \n",__func__);
	gc5025a_gcore_initial_otp(s_ctrl);
	gc5025a_gcore_enable_otp(s_ctrl,1);
	gc5025a_gcore_read_otp_info(s_ctrl);
	gc5025a_gcore_enable_otp(s_ctrl,0);
}


int read_otp(struct msm_sensor_ctrl_t *s_ctrl,struct otp_struct *otp_ptr)
{
    int addr,i,af_addr;
    uint16_t otp_flag;
    uint16_t data1,data2,temp;
    int err;
         s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x0100,0x01,MSM_CAMERA_I2C_BYTE_ADDR);
       err= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x3d84,0xC0,MSM_CAMERA_I2C_BYTE_ADDR);
        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x3d88,0x70,MSM_CAMERA_I2C_BYTE_ADDR);
        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x3d89,0x0C,MSM_CAMERA_I2C_BYTE_ADDR);
        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x3d8A,0x70,MSM_CAMERA_I2C_BYTE_ADDR);
        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x3d8B,0x1B,MSM_CAMERA_I2C_BYTE_ADDR);
         s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x3d81,0x01,MSM_CAMERA_I2C_BYTE_ADDR);
       delay(5);
       err=s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                s_ctrl->sensor_i2c_client, 0x700c,
                               &otp_flag, MSM_CAMERA_I2C_BYTE_ADDR);

        addr = 0;
       if((otp_flag & 0xc0) == 0x40) {
       addr = 0x700D; // base address of WB Calibration group 1
       }
       else if((otp_flag & 0x0c) == 0x04) {
       addr = 0x7014; // base address of WB Calibration group 2
       }
       printk("SP_EEPROM====OTP_FLAG %x\n",otp_flag);
       /*af addr*/
       if((otp_flag & 0x30)==0x10){
       af_addr=0x7011;
       }
       else if((otp_flag&0x03)==0x01){
       af_addr=0x7018;
       } 
                                  
       if(addr != 0) {
           (*otp_ptr).flag |= 0x80;
           s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                    s_ctrl->sensor_i2c_client, addr,
                                   &((*otp_ptr).module_integrator_id),MSM_CAMERA_I2C_BYTE_ADDR);
           s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                    s_ctrl->sensor_i2c_client, addr+3,
                                   &temp,MSM_CAMERA_I2C_BYTE_ADDR);
           s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                    s_ctrl->sensor_i2c_client, addr+1,
                                   &data1,MSM_CAMERA_I2C_BYTE_ADDR);

            (*otp_ptr).rg_ratio = (data1<<2) + ((temp>>6) & 0x03);
           s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                    s_ctrl->sensor_i2c_client, addr+2,
                                   &data2,MSM_CAMERA_I2C_BYTE_ADDR);
           (*otp_ptr).bg_ratio = (data2<<2) + ((temp>>4) & 0x03);

           pr_err("%s,%d,r = %d,b = %d",__func__,__LINE__,otp_ptr->rg_ratio,otp_ptr->rg_ratio);
       }
       else {
	       (*otp_ptr).flag = 0x00; // not info in OTP
	       (*otp_ptr).module_integrator_id = 0;
	       (*otp_ptr).lens_id = 0;
	       (*otp_ptr).rg_ratio = 0;
	       (*otp_ptr).bg_ratio = 0;
       }
       for(i=0x700C;i<=0x701A;i++) {
           // OV5695_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelarate
          s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                     s_ctrl->sensor_i2c_client,i,0,MSM_CAMERA_I2C_BYTE_ADDR);
       }
        s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x0100,0x00,MSM_CAMERA_I2C_BYTE_ADDR);

        otp_ptr->enable = 1;
       return (*otp_ptr).flag;
}


int apply_otp(struct msm_sensor_ctrl_t *s_ctrl,struct otp_struct *otp_ptr)
{
    int rg, bg, R_gain, G_gain, B_gain, Base_gain;
    int RG_Ratio_Typical=269;
    int BG_Ratio_Typical=348;
    uint16_t err;
    //printk("sbing=================flag=%u\n",(*otp_ptr).flag);
    //printk("sbing=================rg_ratio=%u\n",(*otp_ptr).rg_ratio);
    //printk("sbing=================bg_ratio=%u\n",(*otp_ptr).bg_ratio);
    // apply OTP WB Calibration
    if ((*otp_ptr).flag & 0x80) {
    rg = (*otp_ptr).rg_ratio;
    bg = (*otp_ptr).bg_ratio;
    }
    //calculate G gain
    R_gain = (RG_Ratio_Typical*1000) / rg;
    B_gain = (BG_Ratio_Typical*1000) / bg;
    G_gain = 1000;

	pr_err("%s,%d,rg = %d,bg = %d,rg_golden = %d,bg_golden = %d",
		__func__,__LINE__,rg,bg,RG_Ratio_Typical,BG_Ratio_Typical);
	
    if (R_gain < 1000 || B_gain < 1000)
    {
       if (R_gain < B_gain)
       Base_gain = R_gain;
       else
       Base_gain = B_gain;
    }
    else
    {
       Base_gain = G_gain;
    }
   R_gain = 0x400 * R_gain / (Base_gain);
   B_gain = 0x400 * B_gain / (Base_gain);
   G_gain = 0x400 * G_gain / (Base_gain);
   //printk("sbing==========r_gain=%x\n",R_gain);
   //printk("sbing==========b_gain=%x\n",B_gain);
   //printk("sbing==========g_gain=%x\n",G_gain);
   pr_err("%s,%d,R_gain = %d,B_gain = %d,G_gain = %d",__func__,__LINE__,R_gain,B_gain,G_gain);
   // update sensor WB gain
   if (R_gain>0x400) {
   s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x5019,R_gain>>8,MSM_CAMERA_I2C_BYTE_ADDR);
   s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x501a,R_gain & 0x00ff,MSM_CAMERA_I2C_BYTE_ADDR);
   s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                                s_ctrl->sensor_i2c_client, 0x501a,
                               &err,MSM_CAMERA_I2C_BYTE_ADDR);
    printk("SP_EEPROM,%s,%d,0x501a = %d\n",__func__,__LINE__,err);

 /*  OV5695_write_i2c(0x5019, R_gain>>8);
   OV5695_write_i2c(0x501a, R_gain & 0x00ff);*/
  }
   if (G_gain>0x400) {
   s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x501b,G_gain>>8,MSM_CAMERA_I2C_BYTE_ADDR);
   s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x501c,G_gain & 0x00ff,MSM_CAMERA_I2C_BYTE_ADDR);
 /*  OV5695_write_i2c(0x501b, G_gain>>8);
   OV5695_write_i2c(0x501c, G_gain & 0x00ff);*/
 }
   if (B_gain>0x400) {
   s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x501d, B_gain>>8,MSM_CAMERA_I2C_BYTE_ADDR);
   s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                                        s_ctrl->sensor_i2c_client,0x501e,B_gain & 0x00ff,MSM_CAMERA_I2C_BYTE_ADDR);

  /* OV5695_write_i2c(0x501d, B_gain>>8);*/
 }

  return (*otp_ptr).flag;
}
#define IICWRITE_BYTE(addr,data)    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,addr, data,MSM_CAMERA_I2C_BYTE_DATA);
#define IICREAD_BYTE(addr,data)    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, addr,data, MSM_CAMERA_I2C_BYTE_DATA);

int msm_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_power_ctrl_t *power_info;
	enum msm_camera_device_type_t sensor_device_type;
	struct msm_camera_i2c_client *sensor_i2c_client;

	if (!s_ctrl) {
		pr_err("%s:%d failed: s_ctrl %pK\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}

	if (s_ctrl->is_csid_tg_mode)
		return 0;

	power_info = &s_ctrl->sensordata->power_info;
	sensor_device_type = s_ctrl->sensor_device_type;
	sensor_i2c_client = s_ctrl->sensor_i2c_client;

	if (!power_info || !sensor_i2c_client) {
		pr_err("%s:%d failed: power_info %pK sensor_i2c_client %pK\n",
			__func__, __LINE__, power_info, sensor_i2c_client);
		return -EINVAL;
	}
	return msm_camera_power_down(power_info, sensor_device_type,
		sensor_i2c_client);
}

int msm_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;
	struct msm_camera_power_ctrl_t *power_info;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;
	uint32_t retry = 0;

	if (!s_ctrl) {
		pr_err("%s:%d failed: %pK\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}

	if (s_ctrl->is_csid_tg_mode)
		return 0;

	power_info = &s_ctrl->sensordata->power_info;
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!power_info || !sensor_i2c_client || !slave_info ||
		!sensor_name) {
		pr_err("%s:%d failed: %pK %pK %pK %pK\n",
			__func__, __LINE__, power_info,
			sensor_i2c_client, slave_info, sensor_name);
		return -EINVAL;
	}

	if (s_ctrl->set_mclk_23880000)
		msm_sensor_adjust_mclk(power_info);

	for (retry = 0; retry < 3; retry++) {
		rc = msm_camera_power_up(power_info, s_ctrl->sensor_device_type,
			sensor_i2c_client);
		if (rc < 0)
			return rc;
		rc = msm_sensor_check_id(s_ctrl);
		if (rc < 0) {
			msm_camera_power_down(power_info,
				s_ctrl->sensor_device_type, sensor_i2c_client);
			msleep(20);
			continue;
		} else {
			break;
		}
	}

	return rc;
}

static uint16_t msm_sensor_id_by_mask(struct msm_sensor_ctrl_t *s_ctrl,
	uint16_t chipid)
{
	uint16_t sensor_id = chipid;
	int16_t sensor_id_mask = s_ctrl->sensordata->slave_info->sensor_id_mask;

	if (!sensor_id_mask)
		sensor_id_mask = ~sensor_id_mask;

	sensor_id &= sensor_id_mask;
	sensor_id_mask &= -sensor_id_mask;
	sensor_id_mask -= 1;
	while (sensor_id_mask) {
		sensor_id_mask >>= 1;
		sensor_id >>= 1;
	}
	return sensor_id;
}
#define OTP_START 0x0401
#define OTP_END 0x04b1
uint8_t otp_data[180];
int msm_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t chipid = 0;
  
    
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;

	if (!s_ctrl) {
		pr_err("%s:%d failed: %pK\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!sensor_i2c_client || !slave_info || !sensor_name) {
		pr_err("%s:%d failed: %pK %pK %pK\n",
			__func__, __LINE__, sensor_i2c_client, slave_info,
			sensor_name);
		return -EINVAL;
	}

	rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		sensor_i2c_client, slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__, sensor_name);
		return rc;
	}

	pr_debug("%s: read id: 0x%x expected id 0x%x:\n",
			__func__, chipid, slave_info->sensor_id);
	if (msm_sensor_id_by_mask(s_ctrl, chipid) != slave_info->sensor_id) {
		pr_err("%s chip id %x does not match %x\n",
				__func__, chipid, slave_info->sensor_id);
		return -ENODEV;
	}
    else
    {
        if (chipid == 0x5695 && !otp_ptr.enable)
        {
            read_otp(s_ctrl,&otp_ptr);
        }
        else if (chipid == 0x5025&& (!gc5025a_otp_read)) 
		{
			pr_err("%s stoneadd chip id %x  match!\n",__func__, chipid);
			gc5025a_gcore_identify_otp(s_ctrl);
        }
        else if (chipid == 0x0556) 
        {
         pr_err("%s stoneadd chip id HI556 %x  match!\n",__func__, chipid);
            

        }
    }
	return rc;
}

static struct msm_sensor_ctrl_t *get_sctrl(struct v4l2_subdev *sd)
{
	return container_of(container_of(sd, struct msm_sd_subdev, sd),
		struct msm_sensor_ctrl_t, msm_sd);
}

static void msm_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	mutex_lock(s_ctrl->msm_sensor_mutex);
	if (s_ctrl->sensor_state == MSM_SENSOR_POWER_UP) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &s_ctrl->stop_setting);
		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;

		if (s_ctrl->func_tbl->sensor_power_down) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 0);

			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			pr_err("s_ctrl->func_tbl NULL\n");
		}
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return;
}

static int msm_sensor_get_af_status(struct msm_sensor_ctrl_t *s_ctrl,
			void __user *argp)
{
	/* TO-DO: Need to set AF status register address and expected value
	We need to check the AF status in the sensor register and
	set the status in the *status variable accordingly*/
	return 0;
}

static long msm_sensor_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	void __user *argp = (void __user *)arg;
	if (!s_ctrl) {
		pr_err("%s s_ctrl NULL\n", __func__);
		return -EBADF;
	}
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_CFG:
#ifdef CONFIG_COMPAT
		if (is_compat_task())
			rc = s_ctrl->func_tbl->sensor_config32(s_ctrl, argp);
		else
#endif
			rc = s_ctrl->func_tbl->sensor_config(s_ctrl, argp);
		return rc;
	case VIDIOC_MSM_SENSOR_GET_AF_STATUS:
		return msm_sensor_get_af_status(s_ctrl, argp);
	case VIDIOC_MSM_SENSOR_RELEASE:
	case MSM_SD_SHUTDOWN:
		msm_sensor_stop_stream(s_ctrl);
		return 0;
	case MSM_SD_NOTIFY_FREEZE:
		return 0;
	case MSM_SD_UNNOTIFY_FREEZE:
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static long msm_sensor_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_CFG32:
		cmd = VIDIOC_MSM_SENSOR_CFG;
	default:
		return msm_sensor_subdev_ioctl(sd, cmd, arg);
	}
}

long msm_sensor_subdev_fops_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_sensor_subdev_do_ioctl);
}
static uint8_t hi556_otpread;
static int msm_sensor_config32(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data32 *cdata = (struct sensorb_cfg_data32 *)argp;
	int32_t rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
			cdata->cfg.sensor_info.subdev_intf[i] =
				s_ctrl->sensordata->sensor_info->subdev_intf[i];
		}
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_info.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
			CDBG("%s:%d subdev_intf[%d] %d\n", __func__, __LINE__,
				i, cdata->cfg.sensor_info.subdev_intf[i]);
		}
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_WRITE_I2C_ARRAY:
	case CFG_WRITE_I2C_ARRAY_SYNC:
	case CFG_WRITE_I2C_ARRAY_SYNC_BLOCK:
	case CFG_WRITE_I2C_ARRAY_ASYNC: {
		struct msm_camera_i2c_reg_setting32 conf_array32;
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array32,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		conf_array.addr_type = conf_array32.addr_type;
		conf_array.data_type = conf_array32.data_type;
		conf_array.delay = conf_array32.delay;
		conf_array.size = conf_array32.size;
		conf_array.reg_setting = compat_ptr(conf_array32.reg_setting);

		if (!conf_array.size ||
			conf_array.size > I2C_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
			(void *)(conf_array.reg_setting),
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;

		if (CFG_WRITE_I2C_ARRAY == cdata->cfgtype)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table(s_ctrl->sensor_i2c_client,
				&conf_array);
		else if (CFG_WRITE_I2C_ARRAY_ASYNC == cdata->cfgtype)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_async(s_ctrl->sensor_i2c_client,
				&conf_array);
		else if (CFG_WRITE_I2C_ARRAY_SYNC_BLOCK == cdata->cfgtype)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_sync_block(
				s_ctrl->sensor_i2c_client,
				&conf_array);
		else
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_sync(s_ctrl->sensor_i2c_client,
				&conf_array);

		if (reg_setting[conf_array.size - 1].reg_addr == 0x4009 && !strcmp(s_ctrl->sensordata->sensor_name,"ov5695_qtech"))
		{
			uint16_t value = 0;
			apply_otp(s_ctrl,&otp_ptr);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,0x501a,
				&value, MSM_CAMERA_I2C_BYTE_DATA);
			pr_err("SP_EEPROM,%s,%d,0x501a = %d\n",__func__,__LINE__,value);

		}
        pr_err("stoneadd writei2c size = %d\n",conf_array.size);
		if (!strcmp(s_ctrl->sensordata->sensor_name, "gc5025") && conf_array.size == 115)
		{
			pr_err("stoneadd now apply dd& wb to sensor \n");
            if (gc5025a_otp_read ==1) 
			{
				gc5025a_gcore_update_dd(s_ctrl);
				gc5025a_gcore_update_wb(s_ctrl);
            }
			else
			{
				pr_err("stoneadd  dd& wb otp not ready \n");
			}


		}
        else if ((strcmp(s_ctrl->sensordata->sensor_name, "hi556") ==0)&& conf_array.size == 222)
        {
            if (!hi556_otpread) 
            {

            
            pr_err("%s stoneadd chip id HI556  match!\n", __func__);
            write_cmos_sensor_u16(s_ctrl,0x0a02, 0x01); //Fast sleep on
            write_cmos_sensor_u16(s_ctrl,0x0a00, 0x00); // stand by on
            msleep(10);
            write_cmos_sensor_u16(s_ctrl,0x0f02, 0x00); // pll disable
            write_cmos_sensor_u16(s_ctrl,0x011a, 0x01); // CP TRIM_H
            write_cmos_sensor_u16(s_ctrl,0x011b, 0x09); // IPGM TRIM_H
            write_cmos_sensor_u16(s_ctrl,0x0d04, 0x01); // Fsync(OTP busy) Output Enable
            write_cmos_sensor_u16(s_ctrl,0x0d00, 0x07); // Fsync(OTP busy) Output Drivability
            write_cmos_sensor_u16(s_ctrl,0x003e, 0x10); // OTP R/W mode
            write_cmos_sensor_u16(s_ctrl,0x0a00, 0x01); // stand by off



            pr_err("stoneadd otp_data:\n");
            write_cmos_sensor_u16(s_ctrl,0x10a, ((OTP_START + i) >> 8) & 0xff); // start address H
            write_cmos_sensor_u16(s_ctrl,0x10b, (OTP_START + i) & 0xff); // start address L
            write_cmos_sensor_u16(s_ctrl,0x102, 0x01); // single read
            for (i = 0; i <= (OTP_END - OTP_START); i++) 
            {
                otp_data[i] = read_cmos_sensor_u16(s_ctrl,0x108);
                
            }
            for (i = 0; i <= (OTP_END - OTP_START); i++) 
            {
               pr_err("otp_data:0x%x\n",otp_data[i]);
            }
            hi556_otpread = 1;
            }
            else
            {
                pr_err("hi556 OTP already read\n");
            }
           // 
          

        }
        kfree(reg_setting);

		break;
	}
	case CFG_SLAVE_READ_I2C: {
		struct msm_camera_i2c_read_config read_config;
		struct msm_camera_i2c_read_config *read_config_ptr = NULL;
		uint16_t local_data = 0;
		uint16_t orig_slave_addr = 0, read_slave_addr = 0;
		uint16_t orig_addr_type = 0, read_addr_type = 0;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		read_config_ptr =
			(struct msm_camera_i2c_read_config *)
			compat_ptr(cdata->cfg.setting);

		if (copy_from_user(&read_config, read_config_ptr,
			sizeof(struct msm_camera_i2c_read_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		read_slave_addr = read_config.slave_addr;
		read_addr_type = read_config.addr_type;

		CDBG("%s:CFG_SLAVE_READ_I2C:", __func__);
		CDBG("%s:slave_addr=0x%x reg_addr=0x%x, data_type=%d\n",
			__func__, read_config.slave_addr,
			read_config.reg_addr, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				read_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				read_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				read_slave_addr >> 1);

		orig_addr_type = s_ctrl->sensor_i2c_client->addr_type;
		s_ctrl->sensor_i2c_client->addr_type = read_addr_type;

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				read_config.reg_addr,
				&local_data, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		}
		s_ctrl->sensor_i2c_client->addr_type = orig_addr_type;

		pr_debug("slave_read %x %x %x\n", read_slave_addr,
			read_config.reg_addr, local_data);

		if (rc < 0) {
			pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
			break;
		}
		read_config_ptr->data = local_data;
		break;
	}
	case CFG_SLAVE_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_array_write_config32 write_config32;
		struct msm_camera_i2c_array_write_config write_config;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		uint16_t orig_slave_addr = 0, write_slave_addr = 0;
		uint16_t orig_addr_type = 0, write_addr_type = 0;
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (copy_from_user(&write_config32,
				(void *)compat_ptr(cdata->cfg.setting),
				sizeof(
				struct msm_camera_i2c_array_write_config32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		write_config.slave_addr = write_config32.slave_addr;
		write_config.conf_array.addr_type =
			write_config32.conf_array.addr_type;
		write_config.conf_array.data_type =
			write_config32.conf_array.data_type;
		write_config.conf_array.delay =
			write_config32.conf_array.delay;
		write_config.conf_array.size =
			write_config32.conf_array.size;
		write_config.conf_array.reg_setting =
			compat_ptr(write_config32.conf_array.reg_setting);

		pr_debug("%s:CFG_SLAVE_WRITE_I2C_ARRAY:\n", __func__);
		pr_debug("%s:slave_addr=0x%x, array_size=%d addr_type=%d data_type=%d\n",
			__func__,
			write_config.slave_addr,
			write_config.conf_array.size,
			write_config.conf_array.addr_type,
			write_config.conf_array.data_type);

		if (!write_config.conf_array.size ||
			write_config.conf_array.size > I2C_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(write_config.conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
				(void *)(write_config.conf_array.reg_setting),
				write_config.conf_array.size *
				sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		write_config.conf_array.reg_setting = reg_setting;
		write_slave_addr = write_config.slave_addr;
		write_addr_type = write_config.conf_array.addr_type;

		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				write_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				write_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.",
				__func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		pr_debug("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x\n",
				__func__, orig_slave_addr,
				write_slave_addr >> 1);
		orig_addr_type = s_ctrl->sensor_i2c_client->addr_type;
		s_ctrl->sensor_i2c_client->addr_type = write_addr_type;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &(write_config.conf_array));

		s_ctrl->sensor_i2c_client->addr_type = orig_addr_type;
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		} else {
			pr_err("%s: error: no i2c/cci client found.\n",
				__func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting32 conf_array32;
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array32,
			(void *)compat_ptr(cdata->cfg.setting),
			sizeof(struct msm_camera_i2c_seq_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		conf_array.addr_type = conf_array32.addr_type;
		conf_array.delay = conf_array32.delay;
		conf_array.size = conf_array32.size;
		conf_array.reg_setting = compat_ptr(conf_array32.reg_setting);

		if (!conf_array.size ||
			conf_array.size > I2C_SEQ_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (s_ctrl->func_tbl->sensor_power_up) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 1);

			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;
	case CFG_POWER_DOWN:
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_down) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 0);

			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;
	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting32 stop_setting32;
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (copy_from_user(&stop_setting32,
				(void *)compat_ptr((cdata->cfg.setting)),
			sizeof(struct msm_camera_i2c_reg_setting32))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		stop_setting->addr_type = stop_setting32.addr_type;
		stop_setting->data_type = stop_setting32.data_type;
		stop_setting->delay = stop_setting32.delay;
		stop_setting->size = stop_setting32.size;

		reg_setting = compat_ptr(stop_setting32.reg_setting);

		if (!stop_setting->size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting,
			stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}

	case CFG_SET_I2C_SYNC_PARAM: {
		struct msm_camera_cci_ctrl cci_ctrl;
		s_ctrl->sensor_i2c_client->cci_client->cid =
			cdata->cfg.sensor_i2c_sync_params.cid;
		s_ctrl->sensor_i2c_client->cci_client->id_map =
			cdata->cfg.sensor_i2c_sync_params.csid;

		CDBG("I2C_SYNC_PARAM CID:%d, line:%d delay:%d, cdid:%d\n",
			s_ctrl->sensor_i2c_client->cci_client->cid,
			cdata->cfg.sensor_i2c_sync_params.line,
			cdata->cfg.sensor_i2c_sync_params.delay,
			cdata->cfg.sensor_i2c_sync_params.csid);

		cci_ctrl.cmd = MSM_CCI_SET_SYNC_CID;
		cci_ctrl.cfg.cci_wait_sync_cfg.line =
			cdata->cfg.sensor_i2c_sync_params.line;
		cci_ctrl.cfg.cci_wait_sync_cfg.delay =
			cdata->cfg.sensor_i2c_sync_params.delay;
		cci_ctrl.cfg.cci_wait_sync_cfg.cid =
			cdata->cfg.sensor_i2c_sync_params.cid;
		cci_ctrl.cfg.cci_wait_sync_cfg.csid =
			cdata->cfg.sensor_i2c_sync_params.csid;
		rc = v4l2_subdev_call(s_ctrl->sensor_i2c_client->
				cci_client->cci_subdev,
				core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
		if (rc < 0) {
			pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
			rc = -EFAULT;
			break;
		}
		break;
	}

	default:
		rc = -EFAULT;
		break;
	}

DONE:
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}
#endif

int msm_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	int32_t rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
			cdata->cfg.sensor_info.subdev_intf[i] =
				s_ctrl->sensordata->sensor_info->subdev_intf[i];
		}
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_info.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++) {
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
			CDBG("%s:%d subdev_intf[%d] %d\n", __func__, __LINE__,
				i, cdata->cfg.sensor_info.subdev_intf[i]);
		}
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;

	case CFG_WRITE_I2C_ARRAY:
	case CFG_WRITE_I2C_ARRAY_SYNC:
	case CFG_WRITE_I2C_ARRAY_SYNC_BLOCK:
	case CFG_WRITE_I2C_ARRAY_ASYNC: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size ||
			conf_array.size > I2C_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		if (cdata->cfgtype == CFG_WRITE_I2C_ARRAY)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table(s_ctrl->sensor_i2c_client,
					&conf_array);
		else if (CFG_WRITE_I2C_ARRAY_ASYNC == cdata->cfgtype)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_async(s_ctrl->sensor_i2c_client,
					&conf_array);
		else if (CFG_WRITE_I2C_ARRAY_SYNC_BLOCK == cdata->cfgtype)
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_sync_block(
					s_ctrl->sensor_i2c_client,
					&conf_array);
		else
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_table_sync(s_ctrl->sensor_i2c_client,
					&conf_array);

		kfree(reg_setting);
		break;
	}
	case CFG_SLAVE_READ_I2C: {
		struct msm_camera_i2c_read_config read_config;
		struct msm_camera_i2c_read_config *read_config_ptr = NULL;
		uint16_t local_data = 0;
		uint16_t orig_slave_addr = 0, read_slave_addr = 0;
		uint16_t orig_addr_type = 0, read_addr_type = 0;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		read_config_ptr =
			(struct msm_camera_i2c_read_config *)cdata->cfg.setting;
		if (copy_from_user(&read_config, read_config_ptr,
			sizeof(struct msm_camera_i2c_read_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		read_slave_addr = read_config.slave_addr;
		read_addr_type = read_config.addr_type;
		CDBG("%s:CFG_SLAVE_READ_I2C:", __func__);
		CDBG("%s:slave_addr=0x%x reg_addr=0x%x, data_type=%d\n",
			__func__, read_config.slave_addr,
			read_config.reg_addr, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				read_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				read_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				read_slave_addr >> 1);

		orig_addr_type = s_ctrl->sensor_i2c_client->addr_type;
		s_ctrl->sensor_i2c_client->addr_type = read_addr_type;

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				read_config.reg_addr,
				&local_data, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		}
		s_ctrl->sensor_i2c_client->addr_type = orig_addr_type;

		if (rc < 0) {
			pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
			break;
		}
		read_config_ptr->data = local_data;
		break;
	}
	case CFG_SLAVE_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_array_write_config write_config;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		uint16_t orig_slave_addr = 0,  write_slave_addr = 0;
		uint16_t orig_addr_type = 0, write_addr_type = 0;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (copy_from_user(&write_config,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_array_write_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:CFG_SLAVE_WRITE_I2C_ARRAY:", __func__);
		CDBG("%s:slave_addr=0x%x, array_size=%d\n", __func__,
			write_config.slave_addr,
			write_config.conf_array.size);

		if (!write_config.conf_array.size ||
			write_config.conf_array.size > I2C_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(write_config.conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
				(void *)(write_config.conf_array.reg_setting),
				write_config.conf_array.size *
				sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		write_config.conf_array.reg_setting = reg_setting;
		write_slave_addr = write_config.slave_addr;
		write_addr_type = write_config.conf_array.addr_type;
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				write_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				write_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				write_slave_addr >> 1);
		orig_addr_type = s_ctrl->sensor_i2c_client->addr_type;
		s_ctrl->sensor_i2c_client->addr_type = write_addr_type;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &(write_config.conf_array));
		s_ctrl->sensor_i2c_client->addr_type = orig_addr_type;
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size ||
			conf_array.size > I2C_SEQ_REG_DATA_MAX) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_up) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 1);

			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_down) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 0);

			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %d\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			CDBG("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->is_csid_tg_mode)
			goto DONE;

		if (copy_from_user(stop_setting,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;

		if (!stop_setting->size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting,
			stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}

	case CFG_SET_I2C_SYNC_PARAM: {
		struct msm_camera_cci_ctrl cci_ctrl;

		s_ctrl->sensor_i2c_client->cci_client->cid =
			cdata->cfg.sensor_i2c_sync_params.cid;
		s_ctrl->sensor_i2c_client->cci_client->id_map =
			cdata->cfg.sensor_i2c_sync_params.csid;

		CDBG("I2C_SYNC_PARAM CID:%d, line:%d delay:%d, cdid:%d\n",
			s_ctrl->sensor_i2c_client->cci_client->cid,
			cdata->cfg.sensor_i2c_sync_params.line,
			cdata->cfg.sensor_i2c_sync_params.delay,
			cdata->cfg.sensor_i2c_sync_params.csid);

		cci_ctrl.cmd = MSM_CCI_SET_SYNC_CID;
		cci_ctrl.cfg.cci_wait_sync_cfg.line =
			cdata->cfg.sensor_i2c_sync_params.line;
		cci_ctrl.cfg.cci_wait_sync_cfg.delay =
			cdata->cfg.sensor_i2c_sync_params.delay;
		cci_ctrl.cfg.cci_wait_sync_cfg.cid =
			cdata->cfg.sensor_i2c_sync_params.cid;
		cci_ctrl.cfg.cci_wait_sync_cfg.csid =
			cdata->cfg.sensor_i2c_sync_params.csid;
		rc = v4l2_subdev_call(s_ctrl->sensor_i2c_client->
				cci_client->cci_subdev,
				core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
		if (rc < 0) {
			pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
			rc = -EFAULT;
			break;
		}
		break;
	}

	default:
		rc = -EFAULT;
		break;
	}

DONE:
	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int msm_sensor_check_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0)
		pr_err("%s:%d match id failed rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static int msm_sensor_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	mutex_lock(s_ctrl->msm_sensor_mutex);
	if (!on && s_ctrl->sensor_state == MSM_SENSOR_POWER_UP) {
		s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return rc;
}

static int msm_sensor_v4l2_enum_fmt(struct v4l2_subdev *sd,
	unsigned int index, enum v4l2_mbus_pixelcode *code)
{
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);

	if ((unsigned int)index >= s_ctrl->sensor_v4l2_subdev_info_size)
		return -EINVAL;

	*code = s_ctrl->sensor_v4l2_subdev_info[index].code;
	return 0;
}

static struct v4l2_subdev_core_ops msm_sensor_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops msm_sensor_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops msm_sensor_subdev_ops = {
	.core = &msm_sensor_subdev_core_ops,
	.video  = &msm_sensor_subdev_video_ops,
};

static struct msm_sensor_fn_t msm_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
#ifdef CONFIG_COMPAT
	.sensor_config32 = msm_sensor_config32,
#endif
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_write_conf_tbl = msm_camera_cci_i2c_write_conf_tbl,
	.i2c_write_table_async = msm_camera_cci_i2c_write_table_async,
	.i2c_write_table_sync = msm_camera_cci_i2c_write_table_sync,
	.i2c_write_table_sync_block = msm_camera_cci_i2c_write_table_sync_block,

};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_write_conf_tbl = msm_camera_qup_i2c_write_conf_tbl,
	.i2c_write_table_async = msm_camera_qup_i2c_write_table,
	.i2c_write_table_sync = msm_camera_qup_i2c_write_table,
	.i2c_write_table_sync_block = msm_camera_qup_i2c_write_table,
};

int32_t msm_sensor_init_default_params(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_cci_client *cci_client = NULL;
	unsigned long mount_pos = 0;

	/* Validate input parameters */
	if (!s_ctrl) {
		pr_err("%s:%d failed: invalid params s_ctrl %pK\n", __func__,
			__LINE__, s_ctrl);
		return -EINVAL;
	}

	if (!s_ctrl->sensor_i2c_client) {
		pr_err("%s:%d failed: invalid params sensor_i2c_client %pK\n",
			__func__, __LINE__, s_ctrl->sensor_i2c_client);
		return -EINVAL;
	}

	/* Initialize cci_client */
	s_ctrl->sensor_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!s_ctrl->sensor_i2c_client->cci_client) {
		pr_err("%s:%d failed: no memory cci_client %pK\n", __func__,
			__LINE__, s_ctrl->sensor_i2c_client->cci_client);
		return -ENOMEM;
	}

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = s_ctrl->sensor_i2c_client->cci_client;

		/* Get CCI subdev */
		cci_client->cci_subdev = msm_cci_get_subdev();

		/* Update CCI / I2C function table */
		if (!s_ctrl->sensor_i2c_client->i2c_func_tbl)
			s_ctrl->sensor_i2c_client->i2c_func_tbl =
				&msm_sensor_cci_func_tbl;
	} else {
		if (!s_ctrl->sensor_i2c_client->i2c_func_tbl) {
			CDBG("%s:%d\n", __func__, __LINE__);
			s_ctrl->sensor_i2c_client->i2c_func_tbl =
				&msm_sensor_qup_func_tbl;
		}
	}

	/* Update function table driven by ioctl */
	if (!s_ctrl->func_tbl)
		s_ctrl->func_tbl = &msm_sensor_func_tbl;

	/* Update v4l2 subdev ops table */
	if (!s_ctrl->sensor_v4l2_subdev_ops)
		s_ctrl->sensor_v4l2_subdev_ops = &msm_sensor_subdev_ops;

	/* Update sensor mount angle and position in media entity flag */
	mount_pos = s_ctrl->sensordata->sensor_info->position << 16;
	mount_pos = mount_pos | ((s_ctrl->sensordata->sensor_info->
					sensor_mount_angle / 90) << 8);
	s_ctrl->msm_sd.sd.entity.flags = mount_pos | MEDIA_ENT_FL_DEFAULT;

	return 0;
}
