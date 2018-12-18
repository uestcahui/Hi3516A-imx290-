#if !defined(__IMX290_CMOS_H_)
#define __IMX290_CMOS_H_

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "hi_comm_sns.h"
#include "hi_comm_video.h"
#include "hi_sns_ctrl.h"
#include "mpi_isp.h"
#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_af.h"

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* End of #ifdef __cplusplus */

#define IMX290_ID 290

/* 不使用 cmos_cfg.ini 初始化配置 .*/
/****************************************************************************
 * global variables                                                            *
 ****************************************************************************/

//------------ 待定 --------------------------------------------
//static ISP_FSWDR_MODE_E genFSWDRMode = ISP_FSWDR_NORMAL_MODE;
//static HI_U32 gu32MaxTimeGetCnt = 0;

static HI_U32 g_au32InitExposure = 0;
//static HI_U32 g_au32LinesPer500ms = 0;
//static HI_U16 g_au16InitWBGain[3] = {0};
//static HI_U16 g_au16SampleRgain = 0;
//static HI_U16 g_au16SampleBgain = 0;

typedef struct hiIMX290_STATE_S
{
    HI_U8       u8Hcg;
    HI_U32      u32BRL;
    HI_U32      u32RHS1_MAX;
    HI_U32      u32RHS2_MAX;
} IMX290_STATE_S;

IMX290_STATE_S g_astimx290State = {0};
//-----------------以上全局变量在imx178中未定义，是否使用看情况定-------------------------------------

extern void sensor_init();
extern void sensor_exit();
extern int sensor_write_register(int addr, int data);
extern int sensor_read_register(int addr);

#define IMX290_FULL_LINES_MAX  (0x3FFFF)
//#define IMX290_FULL_LINES_MAX_2TO1_WDR  (0x8AA)    // considering the YOUT_SIZE and bad frame
//#define IMX290_FULL_LINES_MAX_3TO1_WDR  (0x7FC)

/*****Imx290 Register Address*****/
#define IMX290_SHS1_ADDR (0x3020) 
#define IMX290_SHS2_ADDR (0x3024) 
#define IMX290_SHS3_ADDR (0x3028) 
#define IMX290_GAIN_ADDR (0x3014)
#define IMX290_HCG_ADDR  (0x3009)
#define IMX290_VMAX_ADDR (0x3018)
#define IMX290_HMAX_ADDR (0x301c)
#define IMX290_RHS1_ADDR (0x3030) 
#define IMX290_RHS2_ADDR (0x3034)
#define IMX290_Y_OUT_SIZE_ADDR (0x3418)

#define IMX290_INCREASE_LINES (1) /* make real fps less than stand fps because NVR require*/

#define IMX290_VMAX_1080P30_LINEAR  (1125+IMX290_INCREASE_LINES)
//#define IMX290_VMAX_1080P60TO30_WDR (1220+IMX290_INCREASE_LINES)
//#define IMX290_VMAX_1080P120TO30_WDR (1125+IMX290_INCREASE_LINES)

//sensor fps mode--目前仅支持linear mode
#define IMX290_SENSOR_1080P_30FPS_LINEAR_MODE      (1)
//#define IMX290_SENSOR_1080P_30FPS_3t1_WDR_MODE     (2)
//#define IMX290_SENSOR_1080P_30FPS_2t1_WDR_MODE     (3)

/****************************************************************************
 * local variables                                                            *
 ****************************************************************************/

extern const unsigned int sensor_i2c_addr;
extern unsigned int sensor_addr_byte;
extern unsigned int sensor_data_byte;

HI_U8 gu8SensorImageMode = IMX290_SENSOR_1080P_30FPS_LINEAR_MODE;
WDR_MODE_E genSensorMode = WDR_MODE_NONE;

static HI_U32 gu32FullLinesStd = IMX290_VMAX_1080P30_LINEAR; 
static HI_U32 gu32FullLines = IMX290_VMAX_1080P30_LINEAR;

static HI_BOOL bInit = HI_FALSE;
HI_BOOL bSensorInit = HI_FALSE; 

ISP_SNS_REGS_INFO_S g_stSnsRegsInfo = {0};
ISP_SNS_REGS_INFO_S g_stPreSnsRegsInfo = {0};


/* AE default parameter and function */
static HI_S32 cmos_get_ae_default(AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    if (HI_NULL == pstAeSnsDft)
    {
        printf("null pointer when get ae default value!\n");
        return - 1;
    }
    
    memset(&pstAeSnsDft->stAERouteAttr, 0, sizeof(ISP_AE_ROUTE_S));
    
    pstAeSnsDft->u32LinesPer500ms =gu32FullLinesStd*30/2;
    pstAeSnsDft->u32FullLinesStd = gu32FullLinesStd;
    pstAeSnsDft->u32FlickerFreq = 50*256;
    pstAeSnsDft->u32FullLinesMax = IMX290_FULL_LINES_MAX;

    pstAeSnsDft->stIntTimeAccu.enAccuType = AE_ACCURACY_LINEAR;
    pstAeSnsDft->stIntTimeAccu.f32Accuracy = 1;
    pstAeSnsDft->stIntTimeAccu.f32Offset = 0;

    pstAeSnsDft->stAgainAccu.enAccuType = AE_ACCURACY_TABLE;
    pstAeSnsDft->stAgainAccu.f32Accuracy = 1;

    pstAeSnsDft->stDgainAccu.enAccuType = AE_ACCURACY_TABLE;
    pstAeSnsDft->stDgainAccu.f32Accuracy = 1;
    
    pstAeSnsDft->u32ISPDgainShift = 8;
    pstAeSnsDft->u32MinISPDgainTarget = 1 << pstAeSnsDft->u32ISPDgainShift;
    pstAeSnsDft->u32MaxISPDgainTarget = 2 << pstAeSnsDft->u32ISPDgainShift;

    pstAeSnsDft->enMaxIrisFNO = ISP_IRIS_F_NO_1_0;
    pstAeSnsDft->enMinIrisFNO = ISP_IRIS_F_NO_32_0;

    pstAeSnsDft->bAERouteExValid = HI_FALSE;
    pstAeSnsDft->stAERouteAttr.u32TotalNum = 0;
    pstAeSnsDft->stAERouteAttrEx.u32TotalNum = 0;

    switch(genSensorMode)
    {
        default:
        case WDR_MODE_NONE:   /*linear mode*/
            pstAeSnsDft->au8HistThresh[0] = 0xd;
            pstAeSnsDft->au8HistThresh[1] = 0x28;
            pstAeSnsDft->au8HistThresh[2] = 0x60;
            pstAeSnsDft->au8HistThresh[3] = 0x80;

            pstAeSnsDft->u32MaxAgain = 62564; 
            pstAeSnsDft->u32MinAgain = 1024;
            pstAeSnsDft->u32MaxAgainTarget = pstAeSnsDft->u32MaxAgain;
            pstAeSnsDft->u32MinAgainTarget = pstAeSnsDft->u32MinAgain;

            pstAeSnsDft->u32MaxDgain = 38577;  
            pstAeSnsDft->u32MinDgain = 1024;
            pstAeSnsDft->u32MaxDgainTarget = 20013;
            pstAeSnsDft->u32MinDgainTarget = pstAeSnsDft->u32MinDgain;
            
            pstAeSnsDft->u8AeCompensation = 0x38;
            //pstAeSnsDft->enAeExpMode = AE_EXP_HIGHLIGHT_PRIOR; // not support in 3516A

            pstAeSnsDft->u32InitExposure = g_au32InitExposure ? g_au32InitExposure : 148859;
            
            pstAeSnsDft->u32MaxIntTime = gu32FullLinesStd - 2;
            pstAeSnsDft->u32MinIntTime = 1;
            pstAeSnsDft->u32MaxIntTimeTarget = 65535;
            pstAeSnsDft->u32MinIntTimeTarget = 1;
            break;
        /*
        仅支持WDR_MODE_NONE，其他请自行移植
        case WDR_MODE_2To1_LINE:
        case WDR_MODE_3To1_LINE:
        */
    }
    return 0;
}


/* the function of sensor set fps */
static HI_VOID cmos_fps_set(HI_FLOAT f32Fps, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    // 这里只支持 Liner mode 1080p 30fps，其他自行移植
    /*fix fps to 30fps*/
    HI_U32 u32VMAX = IMX290_VMAX_1080P30_LINEAR;
    if ((f32Fps <= 30) && (f32Fps >= 0.5))                                                                            
    {
        u32VMAX = IMX290_VMAX_1080P30_LINEAR * 30 / f32Fps;  
    }
    else                                                                                                              
    {                                                                                                                 
        printf("Not support Fps: %f\n", f32Fps);                                                                      
        return;                                                                                                       
    } 
    u32VMAX = (u32VMAX > IMX290_FULL_LINES_MAX) ? IMX290_FULL_LINES_MAX : u32VMAX;

    
    g_stSnsRegsInfo.astI2cData[5].u32Data = (u32VMAX & 0xFF);                                                         
    g_stSnsRegsInfo.astI2cData[6].u32Data = ((u32VMAX & 0xFF00) >> 8);                                                
    g_stSnsRegsInfo.astI2cData[7].u32Data = ((u32VMAX & 0xF0000) >> 16);
    
    /* 上面三行摘自3519的290驱动
       下面两行是 3516的178驱动
       现在不确定用哪个？
    */
   /*
    g_stSnsRegsInfo.astI2cData[4].u32Data = (gu32FullLinesStd & 0xFF);
    g_stSnsRegsInfo.astI2cData[5].u32Data = (gu32FullLinesStd & 0xFF00) >> 8;
   */
    gu32FullLinesStd = u32VMAX; 

    pstAeSnsDft->f32Fps = f32Fps;                                    
    pstAeSnsDft->u32LinesPer500ms = gu32FullLinesStd * f32Fps / 2;                                                        
    pstAeSnsDft->u32FullLinesStd = gu32FullLinesStd;  
    pstAeSnsDft->u32MaxIntTime = gu32FullLinesStd - 2;   
    gu32FullLines = gu32FullLinesStd;
    //pstAeSnsDft->u32FullLines = gu32FullLines; //SDK_V1.0.5.0 没有这个属性，在SDK_V1.0.6.0中添加了这个属性

    return;
}

static HI_VOID cmos_slow_framerate_set(HI_U32 u32FullLines,
    AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    // 这里只支持 Liner mode 1080p 30fps，其他自行移植
    u32FullLines = (u32FullLines > IMX290_FULL_LINES_MAX) ? IMX290_FULL_LINES_MAX : u32FullLines;
    gu32FullLines = u32FullLines;  

    
    g_stSnsRegsInfo.astI2cData[5].u32Data = (gu32FullLines & 0xFF);
    g_stSnsRegsInfo.astI2cData[6].u32Data = ((gu32FullLines & 0xFF00) >> 8);
    g_stSnsRegsInfo.astI2cData[7].u32Data = ((gu32FullLines & 0xF0000) >> 16);
    
     /* 上面三行摘自3519的290驱动
        下面两行是 3516的178驱动
        现在不确定用哪个？
    
    g_stSnsRegsInfo.astI2cData[4].u32Data = (gu32FullLines & 0xFF);
    g_stSnsRegsInfo.astI2cData[5].u32Data = (gu32FullLines & 0xFF00) >> 8;
    */
    //pstAeSnsDft->u32FullLines = gu32FullLines; //SDK_V1.0.5.0 没有这个属性，在SDK_V1.0.6.0中添加了这个属性
    pstAeSnsDft->u32MaxIntTime = gu32FullLines - 2; 
    
    return;
}

/* while isp notify ae to update sensor regs, ae call these funcs. */
static HI_VOID cmos_inttime_update(HI_U32 u32IntTime)
{
    // only 1080p_30fps _linear mode
    HI_U32 u32Value = gu32FullLines - u32IntTime - 1; 
    /*
     上面是imx290，下面是imx178， 减不减1 ，和上面 IMX290_INCREASE_LINES 的定义应该也有关系
    */
    /*
    HI_U32 u32Value = gu32FullLines - u32IntTime; // 是否减一
    */

    
    g_stSnsRegsInfo.astI2cData[0].u32Data = (u32Value & 0xFF);                                    
    g_stSnsRegsInfo.astI2cData[1].u32Data = ((u32Value & 0xFF00) >> 8);                           
    g_stSnsRegsInfo.astI2cData[2].u32Data = ((u32Value & 0x30000) >> 16); 
    
    /*
     上面是imx290，下面是imx178， 减不减1 ，和上面 IMX290_INCREASE_LINES 的定义应该也有关系
    
    g_stSnsRegsInfo.astI2cData[0].u32Data = (u32Value & 0xFF);
    g_stSnsRegsInfo.astI2cData[1].u32Data = ((u32Value & 0xFF00) >> 8);
    */

    return;
}


static HI_U32 gain_table[262]=
{
    1024,1059,1097,1135,1175,1217,1259,1304,1349,1397,1446,1497,1549,1604,1660,1719,1779,1842,1906,
    1973,2043,2048,2119,2194,2271,2351,2434,2519,2608,2699,2794,2892,2994,3099,3208,3321,3438,3559,
    3684,3813,3947,4086,4229,4378,4532,4691,4856,5027,5203,5386,5576,5772,5974,6184,6402,6627,6860,
    7101,7350,7609,7876,8153,8439,8736,9043,9361,9690,10030,10383,10748,11125,11516,11921,12340,12774,
    13222,13687,14168,14666,15182,15715,16267,16839,17431,18043,18677,19334,20013,20717,21445,22198,
    22978,23786,24622,25487,26383,27310,28270,29263,30292,31356,32458,33599,34780,36002,37267,38577,
    39932,41336,42788,44292,45849,47460,49128,50854,52641,54491,56406,58388,60440,62564,64763,67039,
    69395,71833,74358,76971,79676,82476,85374,88375,91480,94695,98023,101468,105034,108725,112545,
    116501,120595,124833,129220,133761,138461,143327,148364,153578,158975,164562,170345,176331,182528,
    188942,195582,202455,209570,216935,224558,232450,240619,249074,257827,266888,276267,285976,296026,
    306429,317197,328344,339883,351827,364191,376990,390238,403952,418147,432842,448053,463799,480098,
    496969,514434,532512,551226,570597,590649,611406,632892,655133,678156,701988,726657,752194,778627,
    805990,834314,863634,893984,925400,957921,991585,1026431,1062502,1099841,1138491,1178500,1219916,
    1262786,1307163,1353100,1400651,1449872,1500824,1553566,1608162,1664676,1723177,1783733,1846417,
    1911304,1978472,2048000,2119971,2194471,2271590,2351418,2434052,2519590,2608134,2699789,2794666,
    2892876,2994538,3099773,3208706,3321467,3438190,3559016,3684087,3813554,3947571,4086297,4229898,
    4378546,4532417,4691696,4856573,5027243,5203912,5386788,5576092,5772048,5974890,6184861,6402210,
    6627198,6860092,7101170,7350721,7609041,7876439,8153234
};

static HI_VOID cmos_again_calc_table(HI_U32 *pu32AgainLin, HI_U32 *pu32AgainDb)
{
    int i;

    if (*pu32AgainLin >= gain_table[120])
    {
         *pu32AgainLin = gain_table[120];
         *pu32AgainDb = 120;
         return ;
    }
    
    for (i = 1; i < 121; i++)
    {
        if (*pu32AgainLin < gain_table[i])
        {
            *pu32AgainLin = gain_table[i - 1];
            *pu32AgainDb = i - 1;
            break;
        }
    }
    return;
}

static HI_VOID cmos_dgain_calc_table(HI_U32 *pu32DgainLin, HI_U32 *pu32DgainDb)
{
    int i;

    if((HI_NULL == pu32DgainLin) ||(HI_NULL == pu32DgainDb))
    {
        printf("null pointer when get ae sensor gain info value!\n");
        return;
    }

    if (*pu32DgainLin >= gain_table[106])
    {
         *pu32DgainLin = gain_table[106];
         *pu32DgainDb = 106;
         return ;
    }
    
    for (i = 1; i < 106; i++)
    {
        if (*pu32DgainLin < gain_table[i])
        {
            *pu32DgainLin = gain_table[i - 1];
            *pu32DgainDb = i - 1;
            break;
        }
    }

    return;
}


static HI_VOID cmos_gains_update(HI_U32 u32Again, HI_U32 u32Dgain)
{  
    HI_U32 u32HCG = g_astimx290State.u8Hcg;
    HI_U32 u32Tmp;
    
    if(u32Again >= 21)
    {
        u32HCG = u32HCG | 0x10;  // bit[4] HCG  .Reg0x3009[7:0]
        u32Again = u32Again - 21;
    }

    u32Tmp=u32Again+u32Dgain;
       
    g_stSnsRegsInfo.astI2cData[3].u32Data = (u32Tmp & 0xFF);
    g_stSnsRegsInfo.astI2cData[4].u32Data = (u32HCG & 0xFF);
    
    /* 上面三行摘自3519的290驱动
        下面两行是 3516的178驱动
        现在不确定用哪个？
    
    g_stSnsRegsInfo.astI2cData[2].u32Data = (u32Tmp & 0xFF);
    g_stSnsRegsInfo.astI2cData[3].u32Data = ((u32Tmp & 0x100) >> 8); 
    */
    return;
}

/* Only used in WDR_MODE_2To1_LINE and WDR_MODE_2To1_FRAME mode */
static HI_VOID cmos_get_inttime_max(HI_U32 u32Ratio, HI_U32 *pu32IntTimeMax)
{
    return;
}


HI_S32 cmos_init_ae_exp_function(AE_SENSOR_EXP_FUNC_S *pstExpFuncs)
{
    memset(pstExpFuncs, 0, sizeof(AE_SENSOR_EXP_FUNC_S));

    pstExpFuncs->pfn_cmos_get_ae_default    = cmos_get_ae_default;
    pstExpFuncs->pfn_cmos_fps_set           = cmos_fps_set;
    pstExpFuncs->pfn_cmos_slow_framerate_set= cmos_slow_framerate_set;    
    pstExpFuncs->pfn_cmos_inttime_update    = cmos_inttime_update;
    pstExpFuncs->pfn_cmos_gains_update      = cmos_gains_update;
    pstExpFuncs->pfn_cmos_again_calc_table  = cmos_again_calc_table;
    pstExpFuncs->pfn_cmos_dgain_calc_table  = cmos_dgain_calc_table;
    pstExpFuncs->pfn_cmos_get_inttime_max   = cmos_get_inttime_max; 

    return 0;
}


/* AWB default parameter and function */
static AWB_CCM_S g_stAwbCcm =
{  
   4900,
   {
      0x1CE, 0x80BF, 0x800F,
      0x8052,0x184,  0x8032,
      0x5,   0x80D1, 0x1CC
   },
   3770,
   {
      0x1CB,  0x80AF, 0x801C,
      0x806D, 0x18C,  0x801F,
      0x11,   0x80EE, 0x1DD
   }, 
   2640,
   {     
      0x1B4,  0x80AA, 0x800A,
      0x8091, 0x1A1,  0x8010,
      0x0,    0x80DB, 0x1DB
   }     
};

static AWB_CCM_S g_stAwbCcmWDR =
{  
   4900,
   {
      0x1CE, 0x80BF, 0x800F,
      0x8052,0x184,  0x8032,
      0x5,   0x80D1, 0x1CC
   },
   3770,
   {
      0x1CB,  0x80AF, 0x801C,
      0x806D, 0x18C,  0x801F,
      0x11,   0x80EE, 0x1DD
   }, 
   2640,
   {     
      0x1B4,  0x80AA, 0x800A,
      0x8091, 0x1A1,  0x8010,
      0x0,    0x80DB, 0x1DB
   }     
};

static AWB_AGC_TABLE_S g_stAwbAgcTable =
{
    /* bvalid */
    1,
    
    /*1,  2,  4,  8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768*/
    /* saturation */   
    {0x80,0x7a,0x78,0x74,0x68,0x60,0x58,0x50,0x48,0x40,0x38,0x38,0x38,0x38,0x38,0x38}
};

static AWB_AGC_TABLE_S g_stAwbAgcTableFSWDR =
{
    /* bvalid */
    1,

    /* saturation */ 
    {0x78,0x78,0x6e,0x64,0x5E,0x58,0x50,0x48,0x40,0x38,0x38,0x38,0x38,0x38,0x38,0x38}
};

static HI_S32 cmos_get_awb_default(AWB_SENSOR_DEFAULT_S *pstAwbSnsDft)
{
    if (HI_NULL == pstAwbSnsDft)
    {
        printf("null pointer when get awb default value!\n");
        return -1;
    }

    memset(pstAwbSnsDft, 0, sizeof(AWB_SENSOR_DEFAULT_S));
    pstAwbSnsDft->u16WbRefTemp = 4900;

    pstAwbSnsDft->au16GainOffset[0] = 0x1C3;
    pstAwbSnsDft->au16GainOffset[1] = 0x100;
    pstAwbSnsDft->au16GainOffset[2] = 0x100;
    pstAwbSnsDft->au16GainOffset[3] = 0x1D4;

    pstAwbSnsDft->as32WbPara[0] = -37;
    pstAwbSnsDft->as32WbPara[1] = 293;
    pstAwbSnsDft->as32WbPara[2] = 0;
    pstAwbSnsDft->as32WbPara[3] = 179537;
    pstAwbSnsDft->as32WbPara[4] = 128;
    pstAwbSnsDft->as32WbPara[5] = -123691;
    

     memcpy(&pstAwbSnsDft->stAgcTbl, &g_stAwbAgcTable, sizeof(AWB_AGC_TABLE_S));
     memcpy(&pstAwbSnsDft->stCcm, &g_stAwbCcm, sizeof(AWB_CCM_S));

    return 0;
}


HI_S32 cmos_init_awb_exp_function(AWB_SENSOR_EXP_FUNC_S *pstExpFuncs)
{   

    memset(pstExpFuncs, 0, sizeof(AWB_SENSOR_EXP_FUNC_S));

    pstExpFuncs->pfn_cmos_get_awb_default = cmos_get_awb_default;

    return 0;
}


/* ISP default parameter and function */

static ISP_CMOS_AGC_TABLE_S g_stIspAgcTable =
{
    /* bvalid */
    1,
    
    /* 100, 200, 400, 800, 1600, 3200, 6400, 12800, 25600, 51200, 102400, 204800, 409600, 819200, 1638400, 3276800 */

    /* sharpen_alt_d */
    {0x3a,0x3a,0x38,0x34,0x30,0x2e,0x28,0x24,0x20,0x1b,0x18,0x14,0x12,0x10,0x10,0x10},
        
    /* sharpen_alt_ud */
    {0x60,0x60,0x58,0x50,0x48,0x40,0x38,0x28,0x20,0x20,0x18,0x12,0x10,0x10,0x10,0x10},
        
    /* snr_thresh Max=0x54 */
    //{0x08,0x0a,0x0f,0x12,0x16,0x1a,0x22,0x28,0x2e,0x36,0x3a,0x40,0x40,0x40,0x40,0x40},
    {0x08,0x0c,0x10,0x14,0x18,0x20,0x28,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30}, 

    /* demosaic_lum_thresh */
    {0x50,0x50,0x4e,0x49,0x45,0x45,0x40,0x3a,0x3a,0x30,0x30,0x2a,0x20,0x20,0x20,0x20},
        
    /* demosaic_np_offset */
    //{0x00,0x0a,0x12,0x1a,0x20,0x28,0x30,0x32,0x34,0x36,0x38,0x38,0x38,0x38,0x38,0x38},
    {0x0,0xa,0x12,0x1a,0x20,0x28,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30},   

    /* ge_strength */
    //{0x55,0x55,0x55,0x55,0x55,0x55,0x37,0x37,0x37,0x35,0x35,0x35,0x35,0x35,0x35,0x35},
    {0x55,0x55,0x55,0x55,0x55,0x55,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37},

    /* rgbsharp_strength */
    {0x88,0x86,0x84,0x78,0x6a,0x60,0x58,0x50,0x40,0x30,0x20,0x16,0x12,0x12,0x12,0x12}
};

static ISP_CMOS_AGC_TABLE_S g_stIspAgcTableFSWDR =
{
    /* bvalid */
    1,
    
    /* 100, 200, 400, 800, 1600, 3200, 6400, 12800, 25600, 51200, 102400, 204800, 409600, 819200, 1638400, 3276800 */

    /* sharpen_alt_d */
    {0x40,0x40,0x3e,0x3a,0x35,0x32,0x2e,0x28,0x24,0x20,0x1e,0x18,0x16,0x12,0x12,0x12},
        
    /* sharpen_alt_ud */
    {0x60,0x60,0x58,0x50,0x48,0x40,0x38,0x28,0x20,0x20,0x18,0x12,0x10,0x10,0x10,0x10},
        
    /* snr_thresh Max=0x54 */
    //{0x08,0x0a,0x0f,0x12,0x16,0x1a,0x22,0x28,0x2e,0x36,0x3a,0x40,0x40,0x40,0x40,0x40},
    {0x8,0xC,0x10,0x14,0x18,0x20,0x28,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30},

    /* demosaic_lum_thresh */
    {0x50,0x50,0x4e,0x49,0x45,0x45,0x40,0x3a,0x3a,0x30,0x30,0x2a,0x20,0x20,0x20,0x20},
        
    /* demosaic_np_offset */
    //{0x00,0x0a,0x12,0x1a,0x20,0x28,0x30,0x32,0x34,0x36,0x38,0x38,0x38,0x38,0x38,0x38},
    {0x0,0xa,0x12,0x1a,0x20,0x28,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30},
        
    /* ge_strength */
    //{0x55,0x55,0x55,0x55,0x55,0x55,0x37,0x37,0x37,0x35,0x35,0x35,0x35,0x35,0x35,0x35},
    {0x55,0x55,0x55,0x55,0x55,0x55,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37,0x37},

    /* rgbsharp_strength */
    {0x88,0x86,0x84,0x78,0x6a,0x60,0x58,0x50,0x40,0x30,0x20,0x16,0x12,0x12,0x12,0x12}
};

static ISP_CMOS_NOISE_TABLE_S g_stIspNoiseTable =
{
    /* bvalid */
    1,
    
    /* nosie_profile_weight_lut */
    {
      0,0,0,0,0,0,0,8,14,18,21,24,26,27,28,30,31,32,33,34,34,36,36,37,38,38,39,39,
      40,40,41,41,42,42,42,43,43,44,44,44,45,45,45,46,46,46,46,47,47,47,48,48,48,48,49,49,49,
      49,49,50,50,50,50,51,51,51,51,51,51,52,52,52,52,52,53,53,53,53,53,53,53,54,54,54,54,54,
      54,55,55,55,55,55,55,55,55,56,56,56,56,56,56,56,56,57,57,57,57,57,57,57,57,57,58,58,58,
      58,58,58,58,58,58,59,59,59,59,59,59,59
    },

    /* demosaic_weight_lut */
    {
      0,8,14,18,21,24,26,27,28,30,31,32,33,34,34,36,36,37,38,38,39,39,40,40,41,41,42,42,42,
      43,43,44,44,44,45,45,45,46,46,46,46,47,47,47,48,48,48,48,49,49,49,49,49,50,50,50,50,
      51,51,51,51,51,51,52,52,52,52,52,53,53,53,53,53,53,53,54,54,54,54,54,54,55,55,55,55,
      55,55,55,55,56,56,56,56,56,56,56,56,57,57,57,57,57,57,57,57,57,58,58,58,58,58,58,58,
      58,58,59,59,59,59,59,59,59,59,59,59,59,59,59
    }
};

static ISP_CMOS_NOISE_TABLE_S g_stIspNoiseTableFSWDR =
{
    /* bvalid */
    1,
    
    /* nosie_profile_weight_lut */
    {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,5,7,10,13,15,18,21,23,26,29,31,34,37,39,42,45
    },

    /* demosaic_weight_lut */
    {
        0,2,15,20,24,27,30,32,33,35,36,37,38,39,40,41,42,42,43,44,44,45,45,46,
        46,47,47,48,48,49,49,49,50,50,50,51,51,51,52,52,52,53,53,53,53,54,54,54,54,55,55,
        55,55,56,56,56,56,56,57,57,57,57,57,58,58,58,58,58,59,59,59,59,59,59,59,60,60,60,
        60,60,60,61,61,61,61,61,61,61,62,62,62,62,62,62,62,62,63,63,63,63,63,63,63,63,63,
        64,64,64,64,64,64,64,64,64,65,65,65,65,65,65,65,65,65,65,65,65,65,65
    }
};

static ISP_CMOS_DEMOSAIC_S g_stIspDemosaic =
{
    /* bvalid */
    1,
    
    /*vh_slope*/
    0xac,

    /*aa_slope*/
    0xaa,

    /*va_slope*/
    0xa8,

    /*uu_slope*/
    0x55,

    /*sat_slope*/
    0x5d,

    /*ac_slope*/
    0xa0,
    
    /*fc_slope*/
    0x80,

    /*vh_thresh*/
    0x00,

    /*aa_thresh*/
    0x00,

    /*va_thresh*/
    0x00,

    /*uu_thresh*/
    0x08,

    /*sat_thresh*/
    0x00,

    /*ac_thresh*/
    0x1b3

};

static ISP_CMOS_DEMOSAIC_S g_stIspDemosaicFSWDR =
{
    /* bvalid */
    1,
    
    /*vh_slope*/
    0xac,

    /*aa_slope*/
    0xaa,

    /*va_slope*/
    0xa8,

    /*uu_slope*/
    0x55,

    /*sat_slope*/
    0x5d,

    /*ac_slope*/
    0xa0,
    
    /*fc_slope*/
    0x80,

    /*vh_thresh*/
    0x00,

    /*aa_thresh*/
    0x00,

    /*va_thresh*/
    0x00,

    /*uu_thresh*/
    0x00,

    /*sat_thresh*/
    0x00,

    /*ac_thresh*/
    0x1b3
};


static ISP_CMOS_RGBSHARPEN_S g_stIspRgbSharpen =
{   
    /* bvalid */   
    1,   
    
    /*lut_core*/   
    192,  
    
    /*lut_strength*/  
    127, 
    
    /*lut_magnitude*/   
    6      
};

static ISP_CMOS_GAMMA_S g_stIspGamma =
{
    /* bvalid */
    1,
    
    {0   ,120 ,220 ,310 ,390 ,470 ,540 ,610 ,670 ,730 ,786 ,842 ,894 ,944 ,994 ,1050,    
    1096,1138,1178,1218,1254,1280,1314,1346,1378,1408,1438,1467,1493,1519,1543,1568,    
    1592,1615,1638,1661,1683,1705,1726,1748,1769,1789,1810,1830,1849,1869,1888,1907,    
    1926,1945,1963,1981,1999,2017,2034,2052,2069,2086,2102,2119,2136,2152,2168,2184,    
    2200,2216,2231,2247,2262,2277,2292,2307,2322,2337,2351,2366,2380,2394,2408,2422,    
    2436,2450,2464,2477,2491,2504,2518,2531,2544,2557,2570,2583,2596,2609,2621,2634,    
    2646,2659,2671,2683,2696,2708,2720,2732,2744,2756,2767,2779,2791,2802,2814,2825,    
    2837,2848,2859,2871,2882,2893,2904,2915,2926,2937,2948,2959,2969,2980,2991,3001,    
    3012,3023,3033,3043,3054,3064,3074,3085,3095,3105,3115,3125,3135,3145,3155,3165,    
    3175,3185,3194,3204,3214,3224,3233,3243,3252,3262,3271,3281,3290,3300,3309,3318,    
    3327,3337,3346,3355,3364,3373,3382,3391,3400,3409,3418,3427,3436,3445,3454,3463,    
    3471,3480,3489,3498,3506,3515,3523,3532,3540,3549,3557,3566,3574,3583,3591,3600,    
    3608,3616,3624,3633,3641,3649,3657,3665,3674,3682,3690,3698,3706,3714,3722,3730,    
    3738,3746,3754,3762,3769,3777,3785,3793,3801,3808,3816,3824,3832,3839,3847,3855,    
    3862,3870,3877,3885,3892,3900,3907,3915,3922,3930,3937,3945,3952,3959,3967,3974,    
    3981,3989,3996,4003,4010,4018,4025,4032,4039,4046,4054,4061,4068,4075,4082,4089,4095}
};


static ISP_CMOS_GAMMA_S g_stIspGammaFSWDR =
{
    /* bvalid */
    1,
    
    {0   ,120 ,220 ,310 ,390 ,470 ,540 ,610 ,670 ,730 ,786 ,842 ,894 ,944 ,994 ,1050,    
    1096,1138,1178,1218,1254,1280,1314,1346,1378,1408,1438,1467,1493,1519,1543,1568,    
    1592,1615,1638,1661,1683,1705,1726,1748,1769,1789,1810,1830,1849,1869,1888,1907,    
    1926,1945,1963,1981,1999,2017,2034,2052,2069,2086,2102,2119,2136,2152,2168,2184,    
    2200,2216,2231,2247,2262,2277,2292,2307,2322,2337,2351,2366,2380,2394,2408,2422,    
    2436,2450,2464,2477,2491,2504,2518,2531,2544,2557,2570,2583,2596,2609,2621,2634,    
    2646,2659,2671,2683,2696,2708,2720,2732,2744,2756,2767,2779,2791,2802,2814,2825,    
    2837,2848,2859,2871,2882,2893,2904,2915,2926,2937,2948,2959,2969,2980,2991,3001,    
    3012,3023,3033,3043,3054,3064,3074,3085,3095,3105,3115,3125,3135,3145,3155,3165,    
    3175,3185,3194,3204,3214,3224,3233,3243,3252,3262,3271,3281,3290,3300,3309,3318,    
    3327,3337,3346,3355,3364,3373,3382,3391,3400,3409,3418,3427,3436,3445,3454,3463,    
    3471,3480,3489,3498,3506,3515,3523,3532,3540,3549,3557,3566,3574,3583,3591,3600,    
    3608,3616,3624,3633,3641,3649,3657,3665,3674,3682,3690,3698,3706,3714,3722,3730,    
    3738,3746,3754,3762,3769,3777,3785,3793,3801,3808,3816,3824,3832,3839,3847,3855,    
    3862,3870,3877,3885,3892,3900,3907,3915,3922,3930,3937,3945,3952,3959,3967,3974,    
    3981,3989,3996,4003,4010,4018,4025,4032,4039,4046,4054,4061,4068,4075,4082,4089,4095}

};

static ISP_CMOS_GAMMAFE_S g_stGammafeFSWDR = 
{
    /* bvalid */
    1,

    /* gamma_fe0 */
    {
        0, 38406, 39281, 40156, 41031, 41907, 42782, 43657, 44532, 45407, 46282, 47158, 48033, 48908, 49783, 50658, 51533, 52409, 53284, 54159, 55034, 55909, 56784, 57660, 58535, 59410, 60285, 61160, 62035, 62911, 63786, 64661, 65535
    },

    /* gamma_fe1 */
    {
        0, 72, 145, 218, 293, 369, 446, 524, 604, 685, 767, 851, 937, 1024, 1113, 1204, 1297, 1391, 1489, 1590, 1692, 1798, 1907, 2020, 2136, 2258, 2383, 2515, 2652, 2798, 2952, 3116, 3295, 3490, 3708, 3961, 4272, 4721, 5954, 6407, 6719, 6972, 7190, 7386, 7564, 7729, 7884, 8029, 8167, 8298, 8424, 8545, 8662, 8774, 8883, 8990, 9092, 9192, 9289, 9385, 9478, 9569, 9658, 9745, 9831, 9915, 9997, 10078, 10158, 10236, 10313, 10389, 10463, 10538, 10610, 10682, 10752, 10823, 10891, 10959, 11026, 11094, 11159, 11224, 11289, 11352, 11415, 11477, 11539, 11600, 11660, 11720, 11780, 11838, 11897, 11954, 12012, 12069, 12125, 12181, 12236, 12291, 12346, 12399, 12453, 12507, 12559, 12612, 12664, 12716, 12768, 12818, 12869, 12919, 12970, 13020, 13069, 13118, 13166, 13215, 13263, 13311, 13358, 13405, 13453, 13500, 13546, 13592, 13638, 13684, 13730, 13775, 13820, 13864, 13909, 13953, 13997, 14041, 14085, 14128, 14172, 14214, 14257, 14299, 14342, 14384, 14426, 14468, 14509, 14551, 14592, 16213, 17654, 18942, 20118, 21208, 22227, 23189, 24101, 24971, 25804, 26603, 27373, 28118, 28838, 29538, 30219, 30881, 31527, 32156, 32772, 33375, 33964, 34541, 35107, 35663, 36208, 36745, 37272, 37790, 38301, 38803, 39298, 39785, 40267, 40741, 41210, 41672, 42128, 42580, 43026, 43466, 43901, 44332, 44757, 45179, 45596, 46008, 46417, 46821, 47222, 47619, 48011, 48400, 48785, 49168, 49547, 49924, 50296, 50666, 51033, 51397, 51758, 52116, 52472, 52825, 53175, 53522, 53868, 54211, 54551, 54889, 55225, 55558, 55889, 56218, 56545, 56870, 57193, 57514, 57833, 58150, 58465, 58778, 59090, 59399, 59708, 60014, 60318, 60621, 60922, 61222, 61520, 61816, 62111, 62403, 62695, 62985, 63275, 63562, 63848, 64132, 64416, 64698, 64978, 65258, 65535
    }
};

HI_U32 cmos_get_isp_default(ISP_CMOS_DEFAULT_S *pstDef)
{   
    if (HI_NULL == pstDef)
    {
        printf("null pointer when get isp default value!\n");
        return -1;
    }

    memset(pstDef, 0, sizeof(ISP_CMOS_DEFAULT_S));

    
    pstDef->stDrc.bEnable               = HI_FALSE;
    pstDef->stDrc.u32BlackLevel         = 0x00;
    pstDef->stDrc.u32WhiteLevel         = 0xD0000; 
    pstDef->stDrc.u32SlopeMax           = 0x30;
    pstDef->stDrc.u32SlopeMin           = 0x0;
    pstDef->stDrc.u32VarianceSpace      = 0x4;
    pstDef->stDrc.u32VarianceIntensity  = 0x2;
    pstDef->stDrc.u32Asymmetry          = 0x08;
    pstDef->stDrc.u32BrightEnhance      = 0xE6;     
    
    
    memcpy(&pstDef->stAgcTbl, &g_stIspAgcTable, sizeof(ISP_CMOS_AGC_TABLE_S));
    memcpy(&pstDef->stNoiseTbl, &g_stIspNoiseTable, sizeof(ISP_CMOS_NOISE_TABLE_S));            
    memcpy(&pstDef->stDemosaic, &g_stIspDemosaic, sizeof(ISP_CMOS_DEMOSAIC_S));
    memcpy(&pstDef->stGamma, &g_stIspGamma, sizeof(ISP_CMOS_GAMMA_S));
    memcpy(&pstDef->stRgbSharpen, &g_stIspRgbSharpen, sizeof(ISP_CMOS_RGBSHARPEN_S));

    pstDef->stSensorMaxResolution.u32MaxWidth  = 1920;
    pstDef->stSensorMaxResolution.u32MaxHeight = 1080;
    return 0;
}


HI_U32 cmos_get_isp_black_level(ISP_CMOS_BLACK_LEVEL_S *pstBlackLevel)
{
    HI_S32  i;
    
    if (HI_NULL == pstBlackLevel)
    {
        printf("null pointer when get isp black level value!\n");
        return -1;
    }

    /* Don't need to update black level when iso change */
    pstBlackLevel->bUpdate = HI_FALSE;

    /* black level of linear mode */
    for (i=0; i<4; i++)
    {
        pstBlackLevel->au16BlackLevel[i] = 0xF0;    // 240
    }
    return 0;  
}

HI_VOID cmos_set_pixel_detect(HI_BOOL bEnable)
{
    HI_U32 u32FullLines_5Fps, u32MaxIntTime_5Fps;
    
    u32FullLines_5Fps = (IMX290_VMAX_1080P30_LINEAR * 30) / 5;
      
    u32MaxIntTime_5Fps = 4;

    if (bEnable) /* setup for ISP pixel calibration mode */
    {
        sensor_write_register (IMX290_GAIN_ADDR,0x00);
        
        sensor_write_register (IMX290_VMAX_ADDR, u32FullLines_5Fps & 0xFF); 
        sensor_write_register (IMX290_VMAX_ADDR + 1, (u32FullLines_5Fps & 0xFF00) >> 8); 
        sensor_write_register (IMX290_VMAX_ADDR + 2, (u32FullLines_5Fps & 0xF0000) >> 16);

        sensor_write_register (IMX290_SHS1_ADDR, u32MaxIntTime_5Fps & 0xFF);
        sensor_write_register (IMX290_SHS1_ADDR + 1,  (u32MaxIntTime_5Fps & 0xFF00) >> 8); 
        sensor_write_register (IMX290_SHS1_ADDR + 2, (u32MaxIntTime_5Fps & 0xF0000) >> 16); 
    }
    else /* setup for ISP 'normal mode' */
    {
        gu32FullLinesStd = (gu32FullLinesStd > 0x1FFFF) ? 0x1FFFF : gu32FullLinesStd;
        sensor_write_register (IMX290_VMAX_ADDR, gu32FullLinesStd & 0xFF); 
        sensor_write_register (IMX290_VMAX_ADDR + 1, (gu32FullLinesStd & 0xFF00) >> 8); 
        sensor_write_register (IMX290_VMAX_ADDR + 2, (gu32FullLinesStd & 0xF0000) >> 16);
        bInit = HI_FALSE;
    }

    return;
}

HI_VOID cmos_set_wdr_mode(HI_U8 u8Mode)
{
    bInit = HI_FALSE;                                                         
                                                                                 
    switch(u8Mode)                                                               
    {                                                                            
        case WDR_MODE_NONE:                                                      
            genSensorMode = WDR_MODE_NONE;                                       
            gu32FullLinesStd = IMX290_VMAX_1080P30_LINEAR;
            gu8SensorImageMode = IMX290_SENSOR_1080P_30FPS_LINEAR_MODE;
            g_astimx290State.u8Hcg = 0x2;
            printf("linear mode\n");                                             
        break;       

        default:                                                                 
            printf("NOT support this mode!\n");                                  
            return;                                                              
        break;                                                                   
    }  

    gu32FullLines= gu32FullLinesStd;    
    return;                                                  
}

static HI_S32 cmos_set_image_mode(ISP_CMOS_SENSOR_IMAGE_MODE_S *pstSensorImageMode)
{
    HI_U8 u8SensorImageMode = gu8SensorImageMode;
    
    bInit = HI_FALSE;   
        
    if (HI_NULL == pstSensorImageMode )
    {
        printf("null pointer when set image mode\n");
        return -1;
    }

    if ((pstSensorImageMode->u16Width <= 1920) && (pstSensorImageMode->u16Height <= 1080)) 
    {
    }  
    else
    {
        printf("Not support! Width:%d, Height:%d, Fps:%f, WDRMode:%d\n", 
            pstSensorImageMode->u16Width, 
            pstSensorImageMode->u16Height,
            pstSensorImageMode->f32Fps,
            genSensorMode);

        return -1;
    }

    if ((HI_TRUE ==bSensorInit) && (u8SensorImageMode ==gu8SensorImageMode))     
    {                                                                              
        /* Don't need to switch SensorImageMode */                                 
        return -1;                                                                 
    }                                                                              

    return 0;
}

HI_U32 cmos_get_sns_regs_info(ISP_SNS_REGS_INFO_S *pstSnsRegsInfo)
{
    HI_S32 i;

    if (HI_NULL == pstSnsRegsInfo)
    {
        printf("null pointer when get sns reg info!\n");
        return -1;
    }

    if (HI_FALSE == bInit)
    {
        g_stSnsRegsInfo.enSnsType = ISP_SNS_I2C_TYPE;
        g_stSnsRegsInfo.u8Cfg2ValidDelayMax = 2;
        g_stSnsRegsInfo.u32RegNum = 8;

        for (i=0; i<g_stSnsRegsInfo.u32RegNum; i++)
        {
            g_stSnsRegsInfo.astI2cData[i].bUpdate = HI_TRUE;
            g_stSnsRegsInfo.astI2cData[i].u8DevAddr = sensor_i2c_addr;
            g_stSnsRegsInfo.astI2cData[i].u32AddrByteNum = sensor_addr_byte;
            g_stSnsRegsInfo.astI2cData[i].u32DataByteNum = sensor_data_byte;
        }

        //Linear Mode Regs
       g_stSnsRegsInfo.astI2cData[0].u8DelayFrmNum = 0;
       g_stSnsRegsInfo.astI2cData[0].u32RegAddr = IMX290_SHS1_ADDR;       
       g_stSnsRegsInfo.astI2cData[1].u8DelayFrmNum = 0;
       g_stSnsRegsInfo.astI2cData[1].u32RegAddr = IMX290_SHS1_ADDR + 1;        
       g_stSnsRegsInfo.astI2cData[2].u8DelayFrmNum = 0;
       g_stSnsRegsInfo.astI2cData[2].u32RegAddr = IMX290_SHS1_ADDR + 2;   
     
       g_stSnsRegsInfo.astI2cData[3].u8DelayFrmNum = 0;       //make shutter and gain effective at the same time
       g_stSnsRegsInfo.astI2cData[3].u32RegAddr = IMX290_GAIN_ADDR;  //gain     
       g_stSnsRegsInfo.astI2cData[4].u8DelayFrmNum = 1;       
       g_stSnsRegsInfo.astI2cData[4].u32RegAddr = IMX290_HCG_ADDR;    
        
       g_stSnsRegsInfo.astI2cData[5].u8DelayFrmNum = 0;
       g_stSnsRegsInfo.astI2cData[5].u32RegAddr = IMX290_VMAX_ADDR;
       g_stSnsRegsInfo.astI2cData[6].u8DelayFrmNum = 0;
       g_stSnsRegsInfo.astI2cData[6].u32RegAddr = IMX290_VMAX_ADDR + 1;
       g_stSnsRegsInfo.astI2cData[7].u8DelayFrmNum = 0;
       g_stSnsRegsInfo.astI2cData[7].u32RegAddr = IMX290_VMAX_ADDR + 2;

        //DOL 2t1 Mode Regs
        if (WDR_MODE_2To1_LINE == genSensorMode)                                                               
        {
         }  

        //DOL 3t1 Mode Regs
        else if (WDR_MODE_3To1_LINE == genSensorMode)                                                               
        {
         } 
        
        bInit = HI_TRUE;                                                                                                       
    }

    else
    {
        for (i=0; i<g_stSnsRegsInfo.u32RegNum; i++)
        {                                                
            if (g_stSnsRegsInfo.astI2cData[i].u32Data == g_stPreSnsRegsInfo.astI2cData[i].u32Data)
            {
                g_stSnsRegsInfo.astI2cData[i].bUpdate = HI_FALSE;
            }
            
            else
            {
                g_stSnsRegsInfo.astI2cData[i].bUpdate = HI_TRUE;
            }
        }
    }

    memcpy(pstSnsRegsInfo, &g_stSnsRegsInfo, sizeof(ISP_SNS_REGS_INFO_S)); 
    memcpy(&g_stPreSnsRegsInfo, &g_stSnsRegsInfo, sizeof(ISP_SNS_REGS_INFO_S)); 

    return 0;
}

int  sensor_set_inifile_path(const char *pcPath)
{
    return 0;
}

HI_VOID sensor_global_init()
{   
    bSensorInit = HI_FALSE; 
    bInit = HI_FALSE;
    gu8SensorImageMode = IMX290_SENSOR_1080P_30FPS_LINEAR_MODE;
    genSensorMode = WDR_MODE_NONE;

    gu32FullLinesStd = IMX290_VMAX_1080P30_LINEAR;
    gu32FullLines = IMX290_VMAX_1080P30_LINEAR;

    memset(&g_stSnsRegsInfo, 0, sizeof(ISP_SNS_REGS_INFO_S));
    memset(&g_stPreSnsRegsInfo, 0, sizeof(ISP_SNS_REGS_INFO_S));    
}

HI_S32 cmos_init_sensor_exp_function(ISP_SENSOR_EXP_FUNC_S *pstSensorExpFunc)
{
    memset(pstSensorExpFunc, 0, sizeof(ISP_SENSOR_EXP_FUNC_S));

    pstSensorExpFunc->pfn_cmos_sensor_init = sensor_init;
    pstSensorExpFunc->pfn_cmos_sensor_exit = sensor_exit;
    pstSensorExpFunc->pfn_cmos_sensor_global_init = sensor_global_init;
    pstSensorExpFunc->pfn_cmos_set_image_mode = cmos_set_image_mode;
    pstSensorExpFunc->pfn_cmos_set_wdr_mode = cmos_set_wdr_mode;
    
    pstSensorExpFunc->pfn_cmos_get_isp_default = cmos_get_isp_default;
    pstSensorExpFunc->pfn_cmos_get_isp_black_level = cmos_get_isp_black_level;
    pstSensorExpFunc->pfn_cmos_set_pixel_detect = cmos_set_pixel_detect;
    pstSensorExpFunc->pfn_cmos_get_sns_reg_info = cmos_get_sns_regs_info;

    return 0;
}

/****************************************************************************
 * callback structure                                                       *
 ****************************************************************************/

int sensor_register_callback(void)
{
    ISP_DEV IspDev = 0;
    HI_S32 s32Ret;
    ALG_LIB_S stLib;
    ISP_SENSOR_REGISTER_S stIspRegister;
    AE_SENSOR_REGISTER_S  stAeRegister;
    AWB_SENSOR_REGISTER_S stAwbRegister;

    cmos_init_sensor_exp_function(&stIspRegister.stSnsExp);
    s32Ret = HI_MPI_ISP_SensorRegCallBack(IspDev, IMX290_ID, &stIspRegister);
    if (s32Ret)
    {
        printf("sensor register callback function failed!\n");
        return s32Ret;
    }
    
    stLib.s32Id = 0;
    strncpy(stLib.acLibName, HI_AE_LIB_NAME, sizeof(HI_AE_LIB_NAME));
    cmos_init_ae_exp_function(&stAeRegister.stSnsExp);
    s32Ret = HI_MPI_AE_SensorRegCallBack(IspDev, &stLib, IMX290_ID, &stAeRegister);
    if (s32Ret)
    {
        printf("sensor register callback function to ae lib failed!\n");
        return s32Ret;
    }

    stLib.s32Id = 0;
    strncpy(stLib.acLibName, HI_AWB_LIB_NAME, sizeof(HI_AWB_LIB_NAME));
    cmos_init_awb_exp_function(&stAwbRegister.stSnsExp);
    s32Ret = HI_MPI_AWB_SensorRegCallBack(IspDev, &stLib, IMX290_ID, &stAwbRegister);
    if (s32Ret)
    {
        printf("sensor register callback function to awb lib failed!\n");
        return s32Ret;
    }
    
    return 0;
}

int sensor_unregister_callback(void)
{
    ISP_DEV IspDev = 0;
    HI_S32 s32Ret;
    ALG_LIB_S stLib;

    s32Ret = HI_MPI_ISP_SensorUnRegCallBack(IspDev, IMX290_ID);
    if (s32Ret)
    {
        printf("sensor unregister callback function failed!\n");
        return s32Ret;
    }
    
    stLib.s32Id = 0;
    strncpy(stLib.acLibName, HI_AE_LIB_NAME, sizeof(HI_AE_LIB_NAME));
    s32Ret = HI_MPI_AE_SensorUnRegCallBack(IspDev, &stLib, IMX290_ID);
    if (s32Ret)
    {
        printf("sensor unregister callback function to ae lib failed!\n");
        return s32Ret;
    }

    stLib.s32Id = 0;
    strncpy(stLib.acLibName, HI_AWB_LIB_NAME, sizeof(HI_AWB_LIB_NAME));
    s32Ret = HI_MPI_AWB_SensorUnRegCallBack(IspDev, &stLib, IMX290_ID);
    if (s32Ret)
    {
        printf("sensor unregister callback function to awb lib failed!\n");
        return s32Ret;
    }
    
    return 0;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif 
