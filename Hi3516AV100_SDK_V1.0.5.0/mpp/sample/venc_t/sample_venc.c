#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

#include "sample_comm.h"

// SONY_IMX290_MIPI_1080P_30FPS

VIDEO_NORM_E gs_enNorm = VIDEO_ENCODING_MODE_NTSC;

HI_S32 SAMPLE_VENC_1080P_CLASSIC(HI_VOID)
{
    PAYLOAD_TYPE_E enPayLoad[3] = {PT_H264, PT_H264, PT_H264};
    PIC_SIZE_E enSize[3] = {PIC_HD1080, PIC_HD720, PIC_VGA};
    HI_U32 u32Profile = 0;

    VB_CONF_S stVbConf;
    SAMPLE_VI_CONFIG_S stViConfig = {0};

    VPSS_GRP VpssGrp;
    VPSS_CHN VpssChn;
    VPSS_GRP_ATTR_S stVpssGrpAttr;
    VPSS_CHN_ATTR_S stVpssChnAttr;
    VPSS_CHN_MODE_S stVpssChnMode;

    VENC_CHN VencChn;
    SAMPLE_RC_E enRcMode = SAMPLE_RC_CBR;

    HI_S32 s32ChnNum=2;

    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32BlkSize;
    SIZE_S stSize;
    //char c;
    
    /******************************************
     step  1: init sys variable
    ******************************************/
    memset(&stVbConf, 0, sizeof(VB_CONF_S));

    SAMPLE_COMM_VI_GetSizeBySensor(&enSize[0]);

    stVbConf.u32MaxPoolCnt = 128;

    /*video buffer*/
    if(s32ChnNum >= 1){
        u32BlkSize = SAMPLE_COMM_SYS_CalcPicVbBlkSize(gs_enNorm, \
                    enSize[0], SAMPLE_PIXEL_FORMAT, SAMPLE_SYS_ALIGN_WIDTH);
        stVbConf.astCommPool[0].u32BlkSize = u32BlkSize;
        stVbConf.astCommPool[0].u32BlkCnt = 10;
    }

    if(s32ChnNum >= 2)
    {
        u32BlkSize = SAMPLE_COMM_SYS_CalcPicVbBlkSize(gs_enNorm, \
                    enSize[1], SAMPLE_PIXEL_FORMAT, SAMPLE_SYS_ALIGN_WIDTH);
        stVbConf.astCommPool[1].u32BlkSize = u32BlkSize;
        stVbConf.astCommPool[1].u32BlkCnt = 10;
    }
    
    // if(s32ChnNum >= 3){
    //     u32BlkSize = SAMPLE_COMM_SYS_CalcPicVbBlkSize(gs_enNorm, \
    //              enSize[2], SAMPLE_PIXEL_FORMAT, SAMPLE_SYS_ALIGN_WIDTH);
    //     stVbConf.astCommPool[2].u32BlkSize = u32BlkSize;
    //     stVbConf.astCommPool[2].u32BlkCnt = 10;
    // }

    /******************************************
     step 2: mpp system init.
    ******************************************/
    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("system init failed with %d!\n", s32Ret);
        goto END_VENC_1080P_CLASSIC_0;
    }

    /******************************************
     step 3: start vi dev & chn to capture
    ******************************************/
    printf("sensot type:%s",SENSOR_TYPE==SONY_IMX290_MIPI_1080P_30FPS?"imx290":"不是290");

    stViConfig.enViMode   = SENSOR_TYPE;
    stViConfig.enRotate   = ROTATE_NONE;
    stViConfig.enNorm     = VIDEO_ENCODING_MODE_AUTO;
    stViConfig.enViChnSet = VI_CHN_SET_NORMAL;
    stViConfig.enWDRMode  = WDR_MODE_NONE;
    s32Ret = SAMPLE_COMM_VI_StartVi(&stViConfig);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("start vi failed!\n");
        goto END_VENC_1080P_CLASSIC_1;
    }

    /******************************************
     step 4: start vpss and vi bind vpss
    ******************************************/
    s32Ret = SAMPLE_COMM_SYS_GetPicSize(gs_enNorm, enSize[0], &stSize);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("SAMPLE_COMM_SYS_GetPicSize failed!\n");
        goto END_VENC_1080P_CLASSIC_1;
    }

    if(s32ChnNum >= 1){
        VpssGrp = 0;
        stVpssGrpAttr.u32MaxW = stSize.u32Width;
        stVpssGrpAttr.u32MaxH = stSize.u32Height;
        stVpssGrpAttr.bIeEn = HI_FALSE;
        stVpssGrpAttr.bNrEn = HI_TRUE;
        stVpssGrpAttr.bHistEn = HI_FALSE;
        stVpssGrpAttr.bDciEn = HI_FALSE;
        stVpssGrpAttr.enDieMode = VPSS_DIE_MODE_NODIE;
        stVpssGrpAttr.enPixFmt = SAMPLE_PIXEL_FORMAT;

        s32Ret = SAMPLE_COMM_VPSS_StartGroup(VpssGrp, &stVpssGrpAttr);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("Start Vpss failed!\n");
            goto END_VENC_1080P_CLASSIC_2;
        }

        s32Ret = SAMPLE_COMM_VI_BindVpss(stViConfig.enViMode);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("Vi bind Vpss failed!\n");
            goto END_VENC_1080P_CLASSIC_3;
        }

        VpssChn = 0;
        stVpssChnMode.enChnMode      = VPSS_CHN_MODE_USER;
        stVpssChnMode.bDouble        = HI_FALSE;
        stVpssChnMode.enPixelFormat  = SAMPLE_PIXEL_FORMAT;
        stVpssChnMode.u32Width       = stSize.u32Width;
        stVpssChnMode.u32Height      = stSize.u32Height;
        stVpssChnMode.enCompressMode = COMPRESS_MODE_SEG;
        memset(&stVpssChnAttr, 0, sizeof(stVpssChnAttr));
        stVpssChnAttr.s32SrcFrameRate = -1;
        stVpssChnAttr.s32DstFrameRate = -1;
        s32Ret = SAMPLE_COMM_VPSS_EnableChn(VpssGrp, VpssChn, &stVpssChnAttr, &stVpssChnMode, HI_NULL);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("Enable vpss chn failed!\n");
            goto END_VENC_1080P_CLASSIC_4;
        }
    }

     if(s32ChnNum >= 2){
        VpssChn = 1;
        stVpssChnMode.enChnMode       = VPSS_CHN_MODE_USER;
        stVpssChnMode.bDouble         = HI_FALSE;
        stVpssChnMode.enPixelFormat   = SAMPLE_PIXEL_FORMAT;
        stVpssChnMode.u32Width        = stSize.u32Width;
        stVpssChnMode.u32Height       = stSize.u32Height;
        stVpssChnMode.enCompressMode  = COMPRESS_MODE_SEG;
        stVpssChnAttr.s32SrcFrameRate = -1;
        stVpssChnAttr.s32DstFrameRate = -1;
        s32Ret = SAMPLE_COMM_VPSS_EnableChn(VpssGrp, VpssChn, &stVpssChnAttr, &stVpssChnMode, HI_NULL);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("Enable vpss chn failed!\n");
            goto END_VENC_1080P_CLASSIC_4;
        }
    }
   
    // if ((SONY_IMX178_LVDS_5M_30FPS != SENSOR_TYPE)
    //     && (APTINA_AR0330_MIPI_1536P_25FPS != SENSOR_TYPE)
    //     && (APTINA_AR0330_MIPI_1296P_25FPS != SENSOR_TYPE))
    // {

    //     VpssChn = 2;
    //     stVpssChnMode.enChnMode 	= VPSS_CHN_MODE_USER;
    //     stVpssChnMode.bDouble		= HI_FALSE;
    //     stVpssChnMode.enPixelFormat = SAMPLE_PIXEL_FORMAT;
    //     stVpssChnMode.u32Width		= 720;
    //     stVpssChnMode.u32Height 	= (VIDEO_ENCODING_MODE_PAL == gs_enNorm) ? 576 : 480;;
    //     stVpssChnMode.enCompressMode = COMPRESS_MODE_NONE;

    //     stVpssChnAttr.s32SrcFrameRate = -1;
    //     stVpssChnAttr.s32DstFrameRate = -1;

    //     s32Ret = SAMPLE_COMM_VPSS_EnableChn(VpssGrp, VpssChn, &stVpssChnAttr, &stVpssChnMode, HI_NULL);
    //     if (HI_SUCCESS != s32Ret)
    //     {
    //         SAMPLE_PRT("Enable vpss chn failed!\n");
    //         goto END_VENC_1080P_CLASSIC_4;
    //     }
    // }
    /******************************************
     step 5: start stream venc
    ******************************************/
    /*** HD1080P **/
    enRcMode = SAMPLE_RC_CBR;
    if(s32ChnNum >= 1){
        VpssGrp = 0;
        VpssChn = 0;
        VencChn = 0;
        s32Ret = SAMPLE_COMM_VENC_Start(VencChn, enPayLoad[0], \
                                        gs_enNorm, enSize[0], enRcMode, u32Profile);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("Start Venc failed!\n");
            goto END_VENC_1080P_CLASSIC_5;
        }

        s32Ret = SAMPLE_COMM_VENC_BindVpss(VencChn, VpssGrp, VpssChn);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("Start Venc failed!\n");
            goto END_VENC_1080P_CLASSIC_5;
        }
    }
    
    /*** 1080p **/
    if(s32ChnNum >= 2){
        VpssChn = 1;
        VencChn = 1;
        s32Ret = SAMPLE_COMM_VENC_Start(VencChn, enPayLoad[1], \
                                        gs_enNorm, enSize[1], enRcMode, u32Profile);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("Start Venc failed!\n");
            goto END_VENC_1080P_CLASSIC_5;
        }

        s32Ret = SAMPLE_COMM_VENC_BindVpss(VencChn, VpssGrp, VpssChn);
        if (HI_SUCCESS != s32Ret)
        {
            SAMPLE_PRT("Start Venc failed!\n");
            goto END_VENC_1080P_CLASSIC_5;
        }
    }
    

    // /*** D1 **/
    // if (SONY_IMX178_LVDS_5M_30FPS != SENSOR_TYPE)
    // {
    //     VpssChn = 2;
    //     VencChn = 2;
    //     s32Ret = SAMPLE_COMM_VENC_Start(VencChn, enPayLoad[2], \
    //                                     gs_enNorm, enSize[2], enRcMode, u32Profile);
    //     if (HI_SUCCESS != s32Ret)
    //     {
    //         SAMPLE_PRT("Start Venc failed!\n");
    //         goto END_VENC_1080P_CLASSIC_5;
    //     }

    //     s32Ret = SAMPLE_COMM_VENC_BindVpss(VencChn, VpssGrp, VpssChn);
    //     if (HI_SUCCESS != s32Ret)
    //     {
    //         SAMPLE_PRT("Start Venc failed!\n");
    //         goto END_VENC_1080P_CLASSIC_5;
    //     }
    // }
    /******************************************
     step 6: stream venc process -- get stream, then save it to file.
    ******************************************/
    s32Ret = SAMPLE_COMM_VENC_StartGetStream(s32ChnNum);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("Start Venc failed!\n");
        goto END_VENC_1080P_CLASSIC_5;
    }

    printf("please press twice ENTER to exit this sample\n");
    getchar();
    getchar();

    /******************************************
     step 7: exit process
    ******************************************/
    SAMPLE_COMM_VENC_StopGetStream();

END_VENC_1080P_CLASSIC_5:
    VpssGrp = 0;

    VpssChn = 0;
    VencChn = 0;
    SAMPLE_COMM_VENC_UnBindVpss(VencChn, VpssGrp, VpssChn);
    SAMPLE_COMM_VENC_Stop(VencChn);

    VpssChn = 1;
    VencChn = 1;
    SAMPLE_COMM_VENC_UnBindVpss(VencChn, VpssGrp, VpssChn);
    SAMPLE_COMM_VENC_Stop(VencChn);


    if (SONY_IMX178_LVDS_5M_30FPS != SENSOR_TYPE)
    {
        VpssChn = 2;
        VencChn = 2;
        SAMPLE_COMM_VENC_UnBindVpss(VencChn, VpssGrp, VpssChn);
        SAMPLE_COMM_VENC_Stop(VencChn);
    }

    SAMPLE_COMM_VI_UnBindVpss(stViConfig.enViMode);
END_VENC_1080P_CLASSIC_4:	//vpss stop
    VpssGrp = 0;
    VpssChn = 0;
    SAMPLE_COMM_VPSS_DisableChn(VpssGrp, VpssChn);
    VpssChn = 1;
    SAMPLE_COMM_VPSS_DisableChn(VpssGrp, VpssChn);
    if (SONY_IMX178_LVDS_5M_30FPS != SENSOR_TYPE)
    {
        VpssChn = 2;
        SAMPLE_COMM_VPSS_DisableChn(VpssGrp, VpssChn);
    }
END_VENC_1080P_CLASSIC_3:    //vpss stop
    SAMPLE_COMM_VI_UnBindVpss(stViConfig.enViMode);
END_VENC_1080P_CLASSIC_2:    //vpss stop
    SAMPLE_COMM_VPSS_StopGroup(VpssGrp);
END_VENC_1080P_CLASSIC_1:	//vi stop
    SAMPLE_COMM_VI_StopVi(&stViConfig);
END_VENC_1080P_CLASSIC_0:	//system exit
    SAMPLE_COMM_SYS_Exit();

    return s32Ret;
}

void SAMPLE_VENC_HandleSig(HI_S32 signo)
{
    if (SIGINT == signo || SIGTERM == signo)
    {
    	SAMPLE_COMM_VENC_StopGetStream();
        SAMPLE_COMM_ISP_Stop();
        SAMPLE_COMM_SYS_Exit();
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
    }
    exit(-1);
}

void SAMPLE_VENC_StreamHandleSig(HI_S32 signo)
{

    if (SIGINT == signo || SIGTERM == signo)
    {
        SAMPLE_COMM_SYS_Exit();
        printf("\033[0;31mprogram exit abnormally!\033[0;39m\n");
    }

    exit(0);
}


int main(int argc, char* argv[])
{
    HI_S32 s32Ret;


    signal(SIGINT, SAMPLE_VENC_HandleSig);
    signal(SIGTERM, SAMPLE_VENC_HandleSig);


    s32Ret = SAMPLE_VENC_1080P_CLASSIC();
      

    if (HI_SUCCESS == s32Ret)
    { printf("program exit normally!\n"); }
    else
    { printf("program exit abnormally!\n"); }
    exit(s32Ret);
}


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
