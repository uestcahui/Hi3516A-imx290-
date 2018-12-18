/******************************************************************************

  Copyright (C), 2015-2018, AIC. Co., Ltd.

 ******************************************************************************
  File Name     : imx290_sensor_ctl.c
  Author        : 曹宪辉（fendouhui@live.com）
  Created       : 2018/12/12
  Description   : Sony IMX290 sensor driver
                  只针对I2C MIPI接口，WDR只支持Liner Mode
  Note          : 以Hi3516A提供imx178驱动为蓝本，参考hi3519提供的imx290驱动，修改得到
******************************************************************************/

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "hi_comm_video.h"
#include "hi_i2c.h"

const unsigned char sensor_i2c_addr     =    0x34; /* I2C Address of IMX178 */
const unsigned int  sensor_addr_byte    =    2;
const unsigned int  sensor_data_byte    =    1;
static int g_fd = -1; // Hi3516 ISP_NUM 1;

extern HI_U8 gu8SensorImageMode;
extern HI_BOOL bSensorInit;
extern WDR_MODE_E genSensorMode;

//sensor fps mode  ---??  gu8SensorImageMode 的取值？？？？
#define IMX290_SENSOR_1080P_30FPS_LINEAR_MODE      (1)
//#define IMX290_SENSOR_1080P_30FPS_3t1_WDR_MODE     (2) //不支持
//#define IMX290_SENSOR_1080P_30FPS_2t1_WDR_MODE     (3) // 不支持


int sensor_i2c_init(void)
{
    if (g_fd >= 0)
    {
        return 0;
    }    

    int ret;
    g_fd = open("/dev/i2c-0", O_RDWR);
    if(g_fd < 0)
    {
        printf("Open /dev/i2c-0 error!\n");
        return -1;
    }
   
   /*
    关于 sensor_i2c_addr>>1 的问题
    ov4689 在Hi3519 和 Hi3516上都支持，在3519上也作了移位处理，但3516上没有；这里倾向认为是海思芯片的差异
   */
    ret = ioctl(g_fd, I2C_SLAVE_FORCE, sensor_i2c_addr);
    //ret = ioctl(g_fd[IspDev], I2C_SLAVE_FORCE, (sensor_i2c_addr>>1));
    if (ret < 0)
    {
        printf("CMD_SET_DEV error!\n");
        return ret;
    }

    return 0;
}

int sensor_i2c_exit(void)
{
    if (g_fd >= 0)
    {
        close(g_fd);
        g_fd = -1;
        return 0;
    }
    return -1;
}

int sensor_read_register(int addr)
{
    // TODO: 
    
    return 0;
}

int sensor_write_register(int addr, int data)
{
    if (0 > g_fd)
    {
        return 0;
    }

    int idx = 0;
    int ret;
    char buf[8];

    /*
    3519 的imx290驱动 有地址右移八位的操纵
    buf[idx] = (addr >> 8) & 0xff;
    idx++;
    同样参考OV4689驱动在这方面的差异，认为是海思芯片不通，保持不移位
    */
    buf[idx++] = addr & 0xFF;
    if (sensor_addr_byte == 2)
    {
        ret = ioctl(g_fd, I2C_16BIT_REG, 1);
        buf[idx++] = addr >> 8;
    }
    else
    {
        ret = ioctl(g_fd, I2C_16BIT_REG, 0);
    }

    if (ret < 0)
    {
        printf("CMD_SET_REG_WIDTH error!\n");
        return -1;
    }

    buf[idx++] = data;
    if (sensor_data_byte == 2)
    {
        ret = ioctl(g_fd, I2C_16BIT_DATA, 1);
        buf[idx++] = data >> 8;
    }
    else
    {
        ret = ioctl(g_fd, I2C_16BIT_DATA, 0);
    }

    if (ret)
    {
        printf("hi_i2c write faild!\n");
        return -1;
    }

    ret = write(g_fd, buf, idx);
    if(ret < 0)
    {
        printf("I2C_WRITE error!\n");
        return -1;
    }
    return 0;
}

static void delay_ms(int ms) { 
    usleep(ms*1000);
}

void sensor_prog(int* rom) 
{
    int i = 0;
    while (1) {
        int lookup = rom[i++];
        int addr = (lookup >> 16) & 0xFFFF;
        int data = lookup & 0xFFFF;
        if (addr == 0xFFFE) {
            delay_ms(data);
        } else if (addr == 0xFFFF) {
            return;
        } else {
            sensor_write_register(addr, data);
        }
    }
}

// 暂时只支持 liner mode
//void imx290_wdr_1080p30_2to1_init(ISP_DEV IspDev); 
//void imx290_wdr_1080p60_2to1_init(ISP_DEV IspDev);
//void imx290_wdr_1080p120_2to1_init(ISP_DEV IspDev);
//void imx290_wdr_720p60_2to1_init(ISP_DEV IspDev);
//void imx290_wdr_1080p30_3to1_init(ISP_DEV IspDev);
//void imx290_wdr_1080p120_3to1_init(ISP_DEV IspDev);
//void imx290_wdr_720p60_3to1_init(ISP_DEV IspDev);
void imx290_linear_1080p30_init(void);

void sensor_init()
{
    sensor_i2c_init();
    /* When sensor first init, config all registers */
    // if(HI_FALSE == bSensorInit){
    //     if(genSensorMode == WDR_MODE_NONE){
            imx290_linear_1080p30_init();
    //     }else{
    //         printf("WDR Mode暂时只支持 WDR_MODE_NONE，其他模式请自行移植\n");
    //     }
    // }
    // /* When sensor switch mode(linear<->WDR or resolution), config different registers(if possible) */
    // else 
    // {
    //     // 仅支持linear_1080p30一种模式，其他模式请自行移植
    // }
    /*
    for (i=0; i<g_astimx290[IspDev].astRegsInfo[0].u32RegNum; i++)
	{
		sensor_write_register(  g_astimx290[IspDev].astRegsInfo[0].astI2cData[i].u32RegAddr, g_astimx290[IspDev].astRegsInfo[0].astI2cData[i].u32Data);
	}
    */
    bSensorInit = HI_TRUE;
}

void sensor_exit()
{
    sensor_i2c_exit();

    return;
}

/* 1080P30 and 1080P25 */
void imx290_linear_1080p30_init(void)
{
    sensor_write_register ( 0x3000, 0x01); /* standby */
    sensor_write_register ( 0x3002, 0x01); /* XTMSTA */
    
    sensor_write_register ( 0x3005, 0x01); //ADBIT
    sensor_write_register ( 0x3129, 0x00); //ADBIT1
    sensor_write_register ( 0x317c, 0x00); //ADBIT2
    sensor_write_register ( 0x31ec, 0x0e); //ADBIT3
    sensor_write_register ( 0x3441, 0x0c); //CSI_DT_FMT
    sensor_write_register ( 0x3442, 0x0c); //CSI_DT_FMT
    
    sensor_write_register ( 0x3007, 0x00);  
    sensor_write_register ( 0x300c, 0x00);
    sensor_write_register ( 0x300f, 0x00);
    sensor_write_register ( 0x3010, 0x21);
    sensor_write_register ( 0x3012, 0x64);
    sensor_write_register ( 0x3016, 0x09);
    sensor_write_register ( 0x3017, 0x00);

#if 1
    sensor_write_register ( 0x3020, 0x01);  /* SHS1 */
    sensor_write_register ( 0x3021, 0x00);
    sensor_write_register ( 0x3024, 0x00);  /* SHS2 */ 
    sensor_write_register ( 0x3025, 0x00);
    sensor_write_register ( 0x3028, 0x00);  /* SHS3 */
    sensor_write_register ( 0x3029, 0x00);
    sensor_write_register ( 0x3030, 0x00);  /* RHS1 */
    sensor_write_register ( 0x3031, 0x00);
    sensor_write_register ( 0x3034, 0x00);  /* RHS2 */ 
    sensor_write_register ( 0x3035, 0x00);
#else
    sensor_write_register ( 0x3020, 0x02);
#endif
    
    sensor_write_register ( 0x305c, 0x18);
    sensor_write_register ( 0x305d, 0x03); 
    sensor_write_register ( 0x305e, 0x20); 
    sensor_write_register ( 0x305f, 0x01);
    sensor_write_register ( 0x3070, 0x02);
    sensor_write_register ( 0x3071, 0x11);
    sensor_write_register ( 0x309b, 0x10);
    sensor_write_register ( 0x309c, 0x22);
    sensor_write_register ( 0x30a2, 0x02);
    sensor_write_register ( 0x30a6, 0x20);
    sensor_write_register ( 0x30a8, 0x20);
    sensor_write_register ( 0x30aa, 0x20);
    sensor_write_register ( 0x30ac, 0x20);
    
    sensor_write_register ( 0x30b0, 0x43);
    sensor_write_register ( 0x3119, 0x9e);
    sensor_write_register ( 0x311c, 0x1e);
    sensor_write_register ( 0x311e, 0x08);
    sensor_write_register ( 0x3128, 0x05);
    sensor_write_register ( 0x313d, 0x83);
    sensor_write_register ( 0x3150, 0x03);

    sensor_write_register ( 0x317e, 0x00);
    sensor_write_register ( 0x315e, 0x1a);
    sensor_write_register ( 0x3164, 0x1a);
    sensor_write_register ( 0x32b8, 0x50);

    sensor_write_register ( 0x32b9, 0x10);
    sensor_write_register ( 0x32ba, 0x00);
    sensor_write_register ( 0x32bb, 0x04);
    sensor_write_register ( 0x32c8, 0x50);
    sensor_write_register ( 0x32c9, 0x10);
    sensor_write_register ( 0x32ca, 0x00);
    sensor_write_register ( 0x32cb, 0x04);
    sensor_write_register ( 0x332c, 0xd3);
    sensor_write_register ( 0x332d, 0x10);
    sensor_write_register ( 0x332e, 0x0d);
    sensor_write_register ( 0x3358, 0x06);
    sensor_write_register ( 0x3359, 0xe1);
    sensor_write_register ( 0x335a, 0x11);
    sensor_write_register ( 0x3360, 0x1e);
    
    sensor_write_register ( 0x3361, 0x61);
    sensor_write_register ( 0x3362, 0x10);
    sensor_write_register ( 0x33b0, 0x50);
    sensor_write_register ( 0x33b2, 0x1a);
    sensor_write_register ( 0x33b3, 0x04);
    sensor_write_register ( 0x3414, 0x0a);
    sensor_write_register ( 0x3418, 0x49);
    sensor_write_register ( 0x3419, 0x04);
    sensor_write_register ( 0x3444, 0x20);
    sensor_write_register ( 0x3445, 0x25);

    sensor_write_register ( 0x3446, 0x47);
    sensor_write_register ( 0x3447, 0x0);
    sensor_write_register ( 0x3448, 0x1f);
    sensor_write_register ( 0x3449, 0x0);
    sensor_write_register ( 0x344a, 0x17);
    sensor_write_register ( 0x344b, 0x0);
    sensor_write_register ( 0x344c, 0x0f);
    sensor_write_register ( 0x344d, 0x0);
    sensor_write_register ( 0x344e, 0x17);
    sensor_write_register ( 0x344f, 0x0);
    sensor_write_register ( 0x3450, 0x47);
    sensor_write_register ( 0x3451, 0x0);
    sensor_write_register ( 0x3452, 0x0f);
    sensor_write_register ( 0x3453, 0x0);
    sensor_write_register ( 0x3454, 0x0f);
    sensor_write_register ( 0x3455, 0x0);
    sensor_write_register ( 0x3480, 0x49);

    sensor_write_register ( 0x3000, 0x00); /* standby */
    delay_ms(20);
    sensor_write_register ( 0x3002, 0x00); /* master mode start */
    sensor_write_register ( 0x304b, 0x0a); /* XVSOUTSEL XHSOUTSEL */
    
    printf("===IMX290 1080P 30fps 12bit LINE Init OK!===\n");    
    return;
}
