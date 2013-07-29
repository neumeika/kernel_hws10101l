/*
 **************************************************************************************
 *
 *       Filename:   vpp_inter.c
 *    Description:   source file
 *
 *        Version:  1.0
 *        Created:  2011-07-8 16:20:00
 *         Author:  j00140427 
 *
 *       Revision:  initial draft;
 **************************************************************************************
 */
#include "vpp_inter.h"

#define LOG_TAG "VPP_INTER"
#include "vpp_log.h"

#define ZME_H_COEFF_SIZE (288)
#define ZME_V_COEFF_SIZE (240)

/**************************************************************
全局变量定义
*/

//VPP 通道:0 vsd, 1 vhd
static VPP_VPP_S s_stVPPLayer[HAL_LAYER_BUTT];

//VPP ZME 系数
static VPP_ZOOM_COEF_ADDR_S s_stZoomCoef;

////6-tap vcoeff
static int s_coef6_cubic[17][6] = {{0,0,511,0,0,0},{3,-12,511,13,-3,0},{6,-22,507,28,-7,0},
                                               {8,-32,502,44,-11,1},{10,-40,495,61,-15,1},{11,-47,486,79,-19,2},
                                               {12,-53,476,98,-24,3},{13,-58,464,117,-28,4},{14,-62,451,137,-33,5},
                                               {15,-65,437,157,-38,6},{15,-67,420,179,-42,7},{15,-68,404,200,-46,7},
                                               {14,-68,386,221,-50,9},{14,-68,367,243,-54,10},{14,-67,348,264,-58,11},
                                               {13,-66,328,286,-61,12},{13,-63,306,306,-63,12}};

////2-tap vcoeff
static int s_coef2_cubic[17][4] = {{0,511,0,0},{0,511,1,0},{0,506,6,0},{0,499,13,0},
                             {0,490,22,0},{0,478,34,0},{0,465,47,0},{0,449,63,0},{0,432,80,0},
                             {0,413,99,0},{0,393,119,0},{0,372,140,0},{0,350,162,0},{0,328,184,0},
                             {0,304,208,0},{0,280,232,0},{0,256,256,0}};

//8-tap hcoeff
static int s_coef8_cubic[17][8] = {{0,0,0,511,0,0,0,0},{-1,3,-12,511,14,-4,1,0},
                             {-2,6,-23,509,28,-8,2,0},{-2,9,-33,503,44,-12,3,0},{-3,11,-41,496,61,-16,4,0},
                             {-3,13,-48,488,79,-21,5,-1},{-3,14,-54,477,98,-25,7,-2},
                             {-4,16,-59,465,118,-30,8,-2},{-4,17,-63,451,138,-35,9,-1},
                             {-4,18,-66,437,158,-39,10,-2},{-4,18,-68,421,180,-44,11,-2},
                             {-4,18,-69,404,201,-48,13,-3},{-4,18,-70,386,222,-52,14,-2},
                             {-4,18,-70,368,244,-56,15,-3},{-4,18,-69,348,265,-59,16,-3},
                             {-4,18,-67,329,286,-63,16,-3},{-3,17,-65,307,307,-65,17,-3}};

////4-tap hcoeff
static int s_coef4_cubic[17][4] = {{0,511,0,0},{-19,511,21,-1},{-37,509,42,-2},  {-51,504,64,-5}, 
                                               {-64,499,86,-9},{-74,492,108,-14}, {-82,484,129,-19}, {-89,474,152,-25},
                                               {-94,463,174,-31},{-97,451,196,-38},{-98,438,217,-45}, {-98,424,238,-52},
                                               {-98,409,260,-59},    {-95,392,280,-65},{-92,376,300,-72},{-88,358,320,-78},
                                               {-83,339,339,-83}};

/**************************************************************
宏功能单元定义
*/
//获取VPP Layer句柄
#define VPP_GET_HANDLE(enLayer, pstVpp) \
do{                                  \
    pstVpp = &(s_stVPPLayer[(int)enLayer]); \
}while(0)


/**************************************************************
基本功能函数定义
*/

// 初始化视频层(当前仅支持视频层1)
static void inter_init_layer(HAL_LAYER_E enLayer)
{
    VPP_VPP_S *pstVpp = NULL;
    int s32Coef[8] = {0,13,-63,306,306,-63,13,0};

    logd();
    
    VPP_GET_HANDLE(enLayer, pstVpp);
    
    pstVpp->bopened = false;

    pstVpp->enLayer = enLayer;
    pstVpp->boffline_mode_en = false;

    pstVpp->st_in_rect.s32X = 0;
    pstVpp->st_in_rect.s32Y = 0;
    pstVpp->st_in_rect.s32Width = 320;
    pstVpp->st_in_rect.s32Height = 240;

    pstVpp->st_out_rect.s32X = 0;
    pstVpp->st_out_rect.s32Y = 0;
    pstVpp->st_out_rect.s32Width = 320;
    pstVpp->st_out_rect.s32Height = 240;

    pstVpp->bcsc_enable = false;
    pstVpp->bzoom_enable = false;

    hal_set_rd_bus_id(1);
    //1 as default outstanding value
    if( 0 == pstVpp->axi_ostd_r0 )
    {
        pstVpp->axi_ostd_r0 = 1;
    }
    
    if( 0 == pstVpp->axi_ostd_r1 )
    {
        pstVpp->axi_ostd_r1 = 1;
    }
    
    if( 0 == pstVpp->axi_ostd_wr )
    {
        pstVpp->axi_ostd_wr = 1;
    }
    hal_set_rd_out_std(0, pstVpp->axi_ostd_r0);  /* AXI总线读ID0的outstanding。*/
    hal_set_rd_out_std(1, pstVpp->axi_ostd_r1);  /* resident */
    hal_set_wr_out_std(pstVpp->axi_ostd_wr);

    //1 set video layer ifir enable
    hal_set_ifir_mode(HAL_LAYER_VIDEO1, HAL_IFIRMODE_DOUBLE);
    
    hal_set_ifir_coef(HAL_LAYER_VIDEO1, s32Coef);

    hal_set_die_def_thd(HAL_LAYER_VIDEO1);
    hal_set_pd_defthd (HAL_LAYER_VIDEO1);  /* pull down */

    //1 HAL Clock gate ctrl enable
    hal_set_clk_gate_en(1);

    hal_set_lnk_lst_node_int_en(1); /*wbc1 int enable */

    hal_set_zme_enable (HAL_LAYER_VIDEO1, HAL_ZMEMODE_ALL, 0);        
    hal_set_zme_fir_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_ALL, 0);        
    hal_set_mid_enable (HAL_LAYER_VIDEO1, HAL_ZMEMODE_ALL, 0);

    hal_set_die_enable (HAL_LAYER_VIDEO1, 0, 0);
}

// 去初始化视频层(目前仅支持视频层1)
static void inter_deinit_layer(HAL_LAYER_E enLayer)
{
    logd();
    hal_set_clk_gate_en(0);
    hal_set_lnk_lst_node_int_en(0); /*wbc1 deint disable */
}

// 检查输出窗口合法性
static int inter_check_rect(RECT_S *pstRect)
{
    BUG_ON( NULL == pstRect);
    
    if ( (pstRect->s32X < VPP_RECT_MIN_X) 
        || (pstRect->s32X > VPP_RECT_MAX_X)
        || (pstRect->s32Y < VPP_RECT_MIN_Y)
        || (pstRect->s32Y > VPP_RECT_MAX_Y)
        || (pstRect->s32Width < VPP_RECT_MIN_W)
        || (pstRect->s32Width > VPP_RECT_MAX_W)
        || (pstRect->s32Height < VPP_RECT_MIN_H) 
        || (pstRect->s32Height > VPP_RECT_MAX_H) )
    {
        loge("check rect fail( x: 0~1856, y:0~1016, w:64~1920, h:64~1080), x= %d, y=%d, w = %d, h = %d\n",
                pstRect->s32X, pstRect->s32Y, pstRect->s32Width, pstRect->s32Height);
        return K3_FAILURE;
    }

    return K3_SUCCESS;
}

// 对于负数的处理是，取绝对值，然后最高位置1 
static __inline int inter_bitvalue(int s32Value)
{
    //return s32Value < 0 ? (512 - s32Value) : s32Value;
    return s32Value < 0 ? s32Value : s32Value;
}

static void inter_translate_zoomcoef(int *ps32Coef, VPP_ZOOM_TAP_E enTap,
                                  HAL_ZOOM_BITARRAY_S *pBitCoef)
{
    unsigned int i = 0, u32Cnt = 0, u32Tap = 0, u32Tmp = 0;

    BUG_ON(NULL == ps32Coef );
    BUG_ON(enTap >= VOU_ZOOM_TAP_BUTT);
    BUG_ON(NULL == pBitCoef );
    
    switch (enTap)
    {
        case VOU_ZOOM_TAP_8LH:
        {
            u32Tap = 8;
            break;
        }
        
        case VOU_ZOOM_TAP_6LV:
        {
            u32Tap = 6;
            break;
        }
        
        case VOU_ZOOM_TAP_4CH:
        {
            u32Tap = 4;
            break;
        }
        
        case VOU_ZOOM_TAP_4CV:
        {
            u32Tap = 4;
            break;
        }
        
        case VOU_ZOOM_TAP_2CV:
        {
            u32Tap = 4;
            break;
        }
        
        default:
        {
            u32Tap = 4;
            break;
        }
    }

    u32Cnt = 18 * u32Tap / 12;
    for (i = 0; i < u32Cnt; i++)
    {
        pBitCoef->stBit[i].bits_0 = inter_bitvalue(*ps32Coef++);
        pBitCoef->stBit[i].bits_1 = inter_bitvalue(*ps32Coef++);
        pBitCoef->stBit[i].bits_2 = inter_bitvalue(*ps32Coef++);

        u32Tmp = inter_bitvalue(*ps32Coef++);
        pBitCoef->stBit[i].bits_32 = u32Tmp;
        pBitCoef->stBit[i].bits_38 = (u32Tmp >> 2);

        pBitCoef->stBit[i].bits_4 = inter_bitvalue(*ps32Coef++);
        pBitCoef->stBit[i].bits_5 = inter_bitvalue(*ps32Coef++);

        u32Tmp = inter_bitvalue(*ps32Coef++);
        pBitCoef->stBit[i].bits_64 = u32Tmp;
        pBitCoef->stBit[i].bits_66 = u32Tmp >> 4;

        pBitCoef->stBit[i].bits_7 = inter_bitvalue(*ps32Coef++);
        pBitCoef->stBit[i].bits_8 = inter_bitvalue(*ps32Coef++);

        u32Tmp = inter_bitvalue(*ps32Coef++);
        pBitCoef->stBit[i].bits_96 = u32Tmp;
        pBitCoef->stBit[i].bits_94 = u32Tmp >> 6;

        pBitCoef->stBit[i].bits_10 = inter_bitvalue(*ps32Coef++);
        pBitCoef->stBit[i].bits_11 = inter_bitvalue(*ps32Coef++);
        pBitCoef->stBit[i].bits_12 = 0;
    }

    pBitCoef->u32Size = u32Cnt * sizeof(HAL_ZOOM_BIT_S);

}

static void inter_load_zoomcoef_h(unsigned char *pu8Addr)
{
    HAL_ZOOM_BITARRAY_S stArray;
    memset(&stArray, 0 , sizeof(HAL_ZOOM_BITARRAY_S));

    BUG_ON(NULL == pu8Addr);
    
    // 亮度
    inter_translate_zoomcoef((int*)s_coef8_cubic ,VOU_ZOOM_TAP_8LH, &stArray);
    memcpy(pu8Addr, stArray.stBit, stArray.u32Size );

    logd("H_LUMA addr: 0x%0x\n", (unsigned int)pu8Addr );
    pu8Addr += stArray.u32Size;
    
    logd("H_CHROMA addr: 0x%0x\n", (unsigned int)pu8Addr  );
    inter_translate_zoomcoef((int*)s_coef4_cubic, VOU_ZOOM_TAP_4CH, &stArray);
    memcpy(pu8Addr, stArray.stBit, stArray.u32Size);

    return;
}

static void inter_load_zoomcoef_v(unsigned char *pu8Addr)
{
    HAL_ZOOM_BITARRAY_S stArray;
    memset(&stArray, 0 , sizeof(HAL_ZOOM_BITARRAY_S));

    BUG_ON(NULL == pu8Addr);
    
    // 亮度
    inter_translate_zoomcoef((int*)s_coef6_cubic, VOU_ZOOM_TAP_6LV, &stArray);
    memcpy(pu8Addr, stArray.stBit, stArray.u32Size);

    logd("V_LUMA addr: 0x%0x\n", (unsigned int)pu8Addr );
    pu8Addr += stArray.u32Size;

    // 色度
    logd("V_CHROMA addr: 0x%0x\n", (unsigned int)pu8Addr  );
    inter_translate_zoomcoef((int*)s_coef2_cubic, VOU_ZOOM_TAP_2CV, &stArray);
    memcpy(pu8Addr, stArray.stBit, stArray.u32Size);
    
    return;
}


static void inter_load_zoomcoef( unsigned char *pZoomVirtAddr )
{
    unsigned char * horaddr = (unsigned char *)pZoomVirtAddr;          //hor addr
    unsigned char * veraddr = (unsigned char *)(pZoomVirtAddr + ZME_H_COEFF_SIZE);  //ver addr

    BUG_ON(NULL == pZoomVirtAddr);

    logd("inter_load_zoomcoef hor_addr = 0x%0x,ver_addr = 0x%0x \n"
         ,(unsigned int)horaddr, (unsigned int)veraddr);
         
    inter_load_zoomcoef_h(horaddr);

    inter_load_zoomcoef_v(veraddr);

    return;
}

static int inter_init_zoomcoef(VPP_ZOOM_COEF_ADDR_S *pstZmeModule)
{
    BUG_ON(NULL == pstZmeModule);
    
    memset((VPP_ZOOM_COEF_S *)pstZmeModule, 0, sizeof(VPP_ZOOM_COEF_S));

    pstZmeModule->pStartVirAddr  = (int *)kmalloc(ZME_H_COEFF_SIZE + ZME_V_COEFF_SIZE, 
                    GFP_KERNEL);
    if (NULL == pstZmeModule->pStartVirAddr)
    {
        logi("Get zme_buf failed\n");
        return K3_FAILURE;
    }
    memset(pstZmeModule->pStartVirAddr,0,ZME_H_COEFF_SIZE + ZME_V_COEFF_SIZE);
    pstZmeModule->pStartPhyAddr = virt_to_phys(pstZmeModule->pStartVirAddr);
    logd("coef addr, pStartPhyAddr = %#x, pStartVirAddr = %#x\n"
       , (unsigned int)pstZmeModule->pStartPhyAddr, (unsigned int)pstZmeModule->pStartVirAddr);

    inter_load_zoomcoef((unsigned char*)pstZmeModule->pStartVirAddr );
    
    return K3_SUCCESS;
}

//去初始化ZME模块
static void inter_deinit_zoomcoef(VPP_ZOOM_COEF_ADDR_S *pstZmeModule)
{
    BUG_ON(NULL == pstZmeModule);
    
    //释放ZME_INFO缓冲区
    if (pstZmeModule->pStartVirAddr != NULL)
    {
        kfree(pstZmeModule->pStartVirAddr); 
        pstZmeModule->pStartVirAddr = NULL;
        pstZmeModule->pStartPhyAddr = NULL;
    }

    return;
}

/*计算分量的地址和stride*/
static void  inter_calc_inputaddrinfo( VPP_INIMAGE_S *pInImg ,VPP_ADDRINFO_S * pVppAddrInfo)
{
    int stride = 0; 

    BUG_ON(NULL == pInImg);
    BUG_ON(NULL == pVppAddrInfo);
    BUG_ON(pInImg->pixformat >= VPP_PIXELFORMAT_BUTT);

    stride = pInImg->width; 
 
    pVppAddrInfo->vhd_lum_str = stride;
    
    switch(pInImg->pixformat)
    {
        case VPP_INPUTFMT_YCBCR_PLANAR_444:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_400:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_420:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_420S:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_422:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_422S:
        {
            pVppAddrInfo->vhd_chm_str = stride;
            break;
        }

        case VPP_INPUTFMT_YCBCR_PLANAR_420:
        case VPP_INPUTFMT_YCBCR_PLANAR_422:
        {
            pVppAddrInfo->vhd_chm_str = stride/2;
            break;
        }
        
        case VPP_INPUTFMT_CBYCRY_PACKAGE_422:
        case VPP_INPUTFMT_YCBYCR_PACKAGE_422:
        case VPP_INPUTFMT_YCRYCB_PACKAGE_422:
        case VPP_INPUTFMT_CRYCBY_PACKAGE_422:
        {
            if ( 0 == pInImg->stride)
	        {
	            logi("stride is zero,so we should computer the stride \n");
	            pVppAddrInfo->vhd_lum_str = 2*stride;
                pVppAddrInfo->vhd_chm_str = 2*stride;   
	        }
	        else 
	        {
	            pVppAddrInfo->vhd_lum_str = pInImg->stride;
                pVppAddrInfo->vhd_chm_str = pInImg->stride; 
            }	
           
            break;
        }
        
        default:
        {
            loge("Input pixformat is not supported, 0x%x\n", pInImg->pixformat);
            break;
        }
    }

    switch(pInImg->pixformat)
    {
        case VPP_INPUTFMT_CBYCRY_PACKAGE_422:
        case VPP_INPUTFMT_YCBYCR_PACKAGE_422:
        case VPP_INPUTFMT_YCRYCB_PACKAGE_422:
        case VPP_INPUTFMT_CRYCBY_PACKAGE_422:
        {
            pVppAddrInfo->vhd_clum_addr = pInImg->buffer_bus_addr;
            break;
        }
        
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_400:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_420:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_422:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_444:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_420S:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_422S:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_444S:
        {
            pVppAddrInfo->vhd_clum_addr = pInImg->buffer_bus_addr;
            pVppAddrInfo->vhd_cchm_addr = pInImg->buffer_bus_addr+pVppAddrInfo->vhd_lum_str*
                                    pInImg->height;
            break;
        }
        
        case VPP_INPUTFMT_YCBCR_PLANAR_444:
        case VPP_INPUTFMT_YCBCR_PLANAR_422:
        {
            pVppAddrInfo->vhd_clum_addr = pInImg->buffer_bus_addr;
            pVppAddrInfo->vhd_cchm_addr = pVppAddrInfo->vhd_clum_addr
                                          + pVppAddrInfo->vhd_lum_str*pInImg->height;
            pVppAddrInfo->vhd_lchm_addr = pVppAddrInfo->vhd_cchm_addr
                                          + pVppAddrInfo->vhd_chm_str*pInImg->height;
            break;
        }
        
        case VPP_INPUTFMT_YCBCR_PLANAR_420:
        {
            pVppAddrInfo->vhd_clum_addr = pInImg->buffer_bus_addr; 
            pVppAddrInfo->vhd_cchm_addr = pVppAddrInfo->vhd_clum_addr
                                          + pVppAddrInfo->vhd_lum_str*pInImg->height;
            pVppAddrInfo->vhd_lchm_addr = pVppAddrInfo->vhd_cchm_addr
                                          + pVppAddrInfo->vhd_chm_str*pInImg->height/2;
            break;
        }
        
        default:
        {
            loge("Input pixformat is not supported, 0x%x\n", pInImg->pixformat);
            BUG_ON(NULL == NULL);
            break;
        }
    }
    
    return;
}

/***********************************************************************************
* Function:       inter_set_inputaddr
* Description:   设置输入图像地址
* Data Accessed:
* Data Updated:
* Input:          HAL_LAYER_E enLayer,
*                 bool bEnable,
*                 VPP_OUTIMAGE_S vppOutImage
* Output:        
* Return:        K3_SUCCESS/errorcode
* Others:
***********************************************************************************/
static void inter_set_inputaddr(VPP_PIXELFORMAT_E pixformat, VPP_ADDRINFO_S *pVppAddrInfo,HAL_LAYER_E enLayer)
{
    BUG_ON(pixformat >= VPP_PIXELFORMAT_BUTT);
    BUG_ON(NULL == pVppAddrInfo);
    BUG_ON(enLayer != HAL_LAYER_VIDEO1);

    logd();
    
    switch(pixformat)
    {
        case  VPP_INPUTFMT_YCBCR_SEMIPLANAR_400:
        case  VPP_INPUTFMT_YCBCR_SEMIPLANAR_420:
        case  VPP_INPUTFMT_YCBCR_SEMIPLANAR_422:
        case  VPP_INPUTFMT_YCBCR_SEMIPLANAR_444:
        case  VPP_INPUTFMT_YCBCR_SEMIPLANAR_420S:
        case  VPP_INPUTFMT_YCBCR_SEMIPLANAR_422S:
        case  VPP_INPUTFMT_YCBCR_SEMIPLANAR_444S:
        case  VPP_INPUTFMT_CBYCRY_PACKAGE_422:
        case  VPP_INPUTFMT_YCBYCR_PACKAGE_422:
        case  VPP_INPUTFMT_YCRYCB_PACKAGE_422:
        case  VPP_INPUTFMT_CRYCBY_PACKAGE_422:
        {
            hal_set_layer_addr(enLayer,0,
                                  pVppAddrInfo->vhd_clum_addr,pVppAddrInfo->vhd_cchm_addr,pVppAddrInfo->vhd_lum_str, pVppAddrInfo->vhd_chm_str);
            break;
        }

        case  VPP_INPUTFMT_YCBCR_PLANAR_420:
        case  VPP_INPUTFMT_YCBCR_PLANAR_422:
        case  VPP_INPUTFMT_YCBCR_PLANAR_444:
        {
            hal_set_layer_addr(enLayer,0,
                                  pVppAddrInfo->vhd_clum_addr,pVppAddrInfo->vhd_cchm_addr,pVppAddrInfo->vhd_lum_str, pVppAddrInfo->vhd_chm_str);
            hal_set_layer_addr(enLayer,1,
                                  pVppAddrInfo->vhd_llum_addr,pVppAddrInfo->vhd_lchm_addr,pVppAddrInfo->vhd_lum_str, pVppAddrInfo->vhd_chm_str);
            break;
        }
        default:
        {
            loge("Input format is not supported, 0x%x\n", pixformat);
            BUG_ON(NULL == NULL);
            break;
        }
    }

    return;
}

//根据窗口偏移量，计算内存中图像显示起始地址相对基地址的偏移量
//当格式为非SEMIPLANAR格式时，参数bLuma无效
static unsigned int inter_calc_addrdiff(VPP_PIXELFORMAT_E enFormat, unsigned int X, unsigned int Y, unsigned int Stride, 
    bool bLuma)
{
    unsigned int AddrDiff = 0;

    BUG_ON(enFormat >= VPP_PIXELFORMAT_BUTT);

    logd();

    switch(enFormat)
    {
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_420:
        {
            //计算真正的地址偏移
            if (true == bLuma)
            {
                AddrDiff = X + (Stride * Y);
            }
            else
            {
                AddrDiff = X + (Stride * Y / 2);
            }
            break;
        }
        
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_422:
        {
            //计算真正的地址偏移
            if (true == bLuma)
            {
                AddrDiff = X + (Stride * Y);
            }
            else
            {
                AddrDiff = (X * 2) + (Stride * Y);
            }
            break;
        }
        
        case VPP_INPUTFMT_CBYCRY_PACKAGE_422:
        case VPP_INPUTFMT_YCBYCR_PACKAGE_422:
        case VPP_INPUTFMT_YCRYCB_PACKAGE_422:
        case VPP_INPUTFMT_CRYCBY_PACKAGE_422:
        {
            //计算真正的地址偏移
            AddrDiff = (X * 2) + (Stride * Y);
            break;
        }
        //TODO
        default:
        {
            AddrDiff = 0;
            break;
        }
    }

    return AddrDiff;
}

/***********************************************************************************
* Function:       inter_set_wbcconfig
* Description:   设置输出图像的信息
* Data Accessed:
* Data Updated:
* Input:          HAL_LAYER_E enLayer,
*                 bool bEnable,
*                 VPP_OUTIMAGE_S vppOutImage
* Output:        
* Return:        K3_SUCCESS/errorcode
* Others:
***********************************************************************************/
static void inter_set_wbcconfig(HAL_LAYER_E enLayer, bool bEnable, VPP_OUTIMAGE_S *pVppOutImage)
{
    BUG_ON(NULL == pVppOutImage);
    BUG_ON(HAL_LAYER_WBC1 != enLayer);

    logd();

    /*enable wbc1*/
    hal_set_wbc_enable(enLayer, bEnable);

    /*set wbc busaddr*/
    hal_set_wbc_addr(enLayer, pVppOutImage->buffer_bus_addr);

    /*set wbc1 stride*/
    hal_set_wbc_stride(enLayer, pVppOutImage->stride);

    /*set wbc1 mode, only surport off line*/
    hal_set_wbc_md(HAL_WBC_RESOSEL_OFL); 

    hal_set_wbc_wr_mode( enLayer, 0);

    /*set wbc1 pixformat*/
    hal_set_wbc_out_fmt(enLayer, pVppOutImage->pixformat);  

}

static int inter_check_para( VPP_INIMAGE_S * pInImg, VPP_ADDRINFO_S *pVppAddrInfo)
{
    BUG_ON(NULL == pInImg);
    BUG_ON( NULL == pVppAddrInfo);
    
    /*数据格式相关检测*/
    switch(pInImg->pixformat)
    {
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_400:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_420:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_422:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_444:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_420S:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_422S:
        case VPP_INPUTFMT_YCBCR_SEMIPLANAR_444S:
        {
            if(0 == pVppAddrInfo->vhd_clum_addr )
            {
                loge("SEMIPALNAR dataformat not config Y addr");
                return K3_FAILURE;
            }

            if( 0 == pVppAddrInfo->vhd_cchm_addr )
            {
                loge("SEMIPALNAR dataformat not config C addr");
                return K3_FAILURE;
            }

            if( 0 == pVppAddrInfo->vhd_lum_str )    
            {
                loge("SEMIPALNAR dataformat not config Y stride");
                return K3_FAILURE;
            }

            if( 0 == pVppAddrInfo->vhd_chm_str )  
            {
                loge("SEMIPALNAR dataformat not config C stride");
                return K3_FAILURE;
            }
            break;
        }
        case VPP_INPUTFMT_YCBCR_PLANAR_420:
        case VPP_INPUTFMT_YCBCR_PLANAR_422:
        case VPP_INPUTFMT_YCBCR_PLANAR_444:
        {
            if( 0 == pVppAddrInfo->vhd_clum_addr )
            {
                loge("PALNAR dataformat not config Y addr");
                return K3_FAILURE;
            }

            if( 0 == pVppAddrInfo->vhd_cchm_addr )
            {
                loge("PALNAR dataformat not config U addr");
                return K3_FAILURE;
            }

            if( 0 == pVppAddrInfo->vhd_lchm_addr )
            {
                loge("PALNAR dataformat not config V addr");
                return K3_FAILURE;
            }

            if( 0 == pVppAddrInfo->vhd_lum_str )
            {
                loge("PALNAR dataformat not config Y stride");
                return K3_FAILURE;
            }

            if( 0 == pVppAddrInfo->vhd_chm_str ) 
            {
                loge("PALNAR dataformat not config C stride");
                return K3_FAILURE;
            }
            break;
        }
        case VPP_INPUTFMT_CBYCRY_PACKAGE_422:
        case VPP_INPUTFMT_YCBYCR_PACKAGE_422:
        case VPP_INPUTFMT_YCRYCB_PACKAGE_422:
        case VPP_INPUTFMT_CRYCBY_PACKAGE_422:
        {
            if( 0 == pVppAddrInfo->vhd_clum_addr )
            {
                loge("PACKAGE dataformat not config addr");
                return K3_FAILURE;
            }

            if( 0 == pVppAddrInfo->vhd_lum_str )
            {
                loge("PACKAGE dataformat not config stride");
                return K3_FAILURE;
            }
            break;
        }
        default:
        {
            loge("Input format is not supported, 0x%x\n", pInImg->pixformat);
            return K3_FAILURE;
        }
    }
    
    return K3_SUCCESS;
}

/***********************************************************************************
* Function:       inter_set_cscconfig
* Description:   设置CSC功能
* Data Accessed:
* Data Updated:
* Input:          HAL_LAYER_E enLayer,
*                 bool bEnable,
*                 VPP_OUTIMAGE_S vppOutImage
* Output:        
* Return:        K3_SUCCESS/errorcode
* Others:
***********************************************************************************/

static void inter_set_cscconfig(HAL_LAYER_E enLayer,VPP_CSCCOEF_E cscMode, bool bEnable)
{
    HAL_CSCCOEF_S Coef;
    memset(&Coef, 0, sizeof(HAL_CSCCOEF_S));

    BUG_ON(cscMode >= VPP_CSC_BUTT );
    BUG_ON( HAL_LAYER_VIDEO1 != enLayer);

    logd("bEnable = %d\n",bEnable);
    
    hal_set_layer_csc(enLayer, bEnable);
    
    if( VPP_CSC_YUV2YUV_709_601 == cscMode )
    {
        Coef.csc_coef00  = (int)(     1*256);
        Coef.csc_coef01  = (int)(-0.116*256);
        Coef.csc_coef02  = (int)( 0.208*256);

        Coef.csc_coef10  = (int)(     0*256);
        Coef.csc_coef11  = (int)( 1.017*256);
        Coef.csc_coef12  = (int)( 0.114*256);

        Coef.csc_coef20  = (int)(     0*256);
        Coef.csc_coef21  = (int)( 0.075*256);
        Coef.csc_coef22  = (int)( 1.025*256);

        Coef.csc_in_dc2  = -16;
        Coef.csc_in_dc1  = -128;
        Coef.csc_in_dc0  = -128;

        Coef.csc_out_dc2 =   16;
        Coef.csc_out_dc1 =  128;
        Coef.csc_out_dc0 =  128;
    }
    else if( VPP_CSC_YUV2YUV_601_709 == cscMode )
    {
        Coef.csc_coef00  = (int)(     1*256);
        Coef.csc_coef01  = (int)(-0.116*256);
        Coef.csc_coef02  = (int)( 0.208*256);

        Coef.csc_coef10  = (int)(     0*256);
        Coef.csc_coef11  = (int)( 1.017*256);
        Coef.csc_coef12  = (int)( 0.114*256);

        Coef.csc_coef20  = (int)(     0*256);
        Coef.csc_coef21  = (int)( 0.075*256);
        Coef.csc_coef22  = (int)( 1.025*256);

        Coef.csc_in_dc2  = -16;
        Coef.csc_in_dc1  = -128;
        Coef.csc_in_dc0  = -128;

        Coef.csc_out_dc2 =   16;
        Coef.csc_out_dc1 =  128;
        Coef.csc_out_dc0 =  128;
    }
    else
    {
        Coef.csc_coef00  = 1*256;
        Coef.csc_coef01  = 0*256;
        Coef.csc_coef02  = 0*256;

        Coef.csc_coef10  = 0*256;
        Coef.csc_coef11  = 1*256;
        Coef.csc_coef12  = 0*256;

        Coef.csc_coef20  = 0*256;
        Coef.csc_coef21  = 0*256;
        Coef.csc_coef22  = 1*256;

        Coef.csc_in_dc2  = -16;
        Coef.csc_in_dc1  = -128;
        Coef.csc_in_dc0  = -128;

        Coef.csc_out_dc2 =  16;
        Coef.csc_out_dc1 = 128;
        Coef.csc_out_dc0 = 128;
    }

    if ( true == bEnable )
    {
        hal_set_intf_csc_coef( enLayer, Coef);
    }
}

/***********************************************************************************
* Function:       inter_init
* Description:   初始化VPP模块，支持重复打开
* Data Accessed:
* Data Updated:
* Input:          NA
* Output:        
* Return:        K3_SUCCESS/errorcode
* Others:
***********************************************************************************/
int inter_init(void)
{
    logd("enter inter Inited!\n");

    //创建公共组件
    //创建ZME系数表
    if (inter_init_zoomcoef(&s_stZoomCoef) != K3_SUCCESS)
    {
        loge("VPP malloc zme coef buf failed.\n");
        return K3_FAILURE;
    }

    hal_init();

    inter_init_layer(HAL_LAYER_VIDEO1);

    return K3_SUCCESS;
}

/***********************************************************************************
* Function:       inter_deinit
* Description:   去初始化VPP模块，支持重复去初始化
* Data Accessed:
* Data Updated:
* Input:          NA
* Output:        
* Return:        K3_SUCCESS/errorcode
* Others:
***********************************************************************************/
void inter_deinit(void)
{
    logd("enter deinit\n");
    
    inter_deinit_layer(HAL_LAYER_VIDEO1);

    hal_deinit();

    //释放ZME系数表
    inter_deinit_zoomcoef(&s_stZoomCoef);
    
    return;
}

/***********************************************************************************
* Function:      inter_open
* Description:   关闭视频层
* Data Accessed:
* Data Updated:
* Input:         enVo --视频层ID
* Output:        
* Return:        K3_SUCCESS/errorcode
* Others:        
***********************************************************************************/
void inter_open(HAL_LAYER_E enLayer)
{
    VPP_VPP_S  *pstVpp = NULL;

    logd("enter open enLayer = %d!\n",enLayer);
    
    VPP_GET_HANDLE(enLayer, pstVpp);

    pstVpp->bopened = true;
    
    return;
}

/***********************************************************************************
* Function:      inter_close
* Description:    关闭视频层
* Data Accessed:
* Data Updated:
* Input:         enVo --视频层ID
* Output:        
* Return:        K3_SUCCESS/errorcode
* Others:        
**********************************************************************************/
void inter_close(HAL_LAYER_E enLayer)
{
    VPP_VPP_S     *pstVpp = NULL;
    logd("enter close enLayer = %d!\n",enLayer);

    //获取VPP句柄
    VPP_GET_HANDLE(enLayer, pstVpp);

    if (pstVpp->bopened != true)
    {
        logi("the device isn't open\n");
        return;
    }

    /*close video1 layer*/
    hal_enable_layer(enLayer, false);

    pstVpp->bopened = false;

    return;
}

/***********************************************************************************
* Function:      inter_start
* Description:   启动配置VPP
* Data Accessed:
* Data Updated:
* Input:        vppConfig 输入VPP配置参数
* Output:        
* Return:        K3_SUCCESS/errorcode
* Others:
***********************************************************************************/
int inter_start(VPP_CONFIG_S* pVppConfig)
{
    VPP_INIMAGE_S  *pInImage = NULL;
    VPP_OUTIMAGE_S *pOutImage = NULL;
    VPP_INCROPPING_S *pInCrop = NULL;
    VPP_ADDRINFO_S addrInfo = {0};
    HAL_RECT_S vhdrect = {0};
    RECT_S  inRect = {0};

    unsigned int u32YAddrDiff = 0;
    unsigned int u32CAddrDiff = 0;

    logd("enter inter start \n");
    BUG_ON(NULL == pVppConfig);
    
    pInImage = &(pVppConfig->vpp_in_img);
    pOutImage = &(pVppConfig->vpp_out_img);
    pInCrop = &(pVppConfig->vpp_in_crop);

    inRect.s32Height = (int)pInImage->height;
    inRect.s32Width = (int)pInImage->width;
    
    logd("enter start pixformat = %d\n", pInImage->pixformat);

    vhdrect.s32DXL = pOutImage->width;   /*显示列结束坐标, 以帧高度为参考，以行为单位*/
    vhdrect.s32DYL = pOutImage->height;
    vhdrect.u32InWidth   = pInImage->width;
    vhdrect.u32InHeight  = pInImage->height;
    vhdrect.u32OutWidth  = pOutImage->width;
    vhdrect.u32OutHeight = pOutImage->height;

    inter_calc_inputaddrinfo(pInImage, &addrInfo);
    if (K3_SUCCESS != inter_check_para( pInImage, &addrInfo))
    {
        loge("inter_check_para fail\n");
        return K3_FAILURE;
    }

    hal_enable_layer(HAL_LAYER_VIDEO1, true);
    hal_set_layer_data_fmt( HAL_LAYER_VIDEO1, pInImage->pixformat);
    hal_set_layer_rect(HAL_LAYER_VIDEO1,vhdrect );
    if ( true == pVppConfig->bincrop_enable )
    {
        logi("crop enable\n");
        vhdrect.u32InWidth   = pInCrop->width;
        vhdrect.u32InHeight  = pInCrop->height;
        u32YAddrDiff = inter_calc_addrdiff(pInImage->pixformat, pInCrop->cropX, pInCrop->cropY, 
                   addrInfo.vhd_lum_str, true);
        u32CAddrDiff = inter_calc_addrdiff(pInImage->pixformat, pInCrop->cropX, pInCrop->cropY, 
                   addrInfo.vhd_lum_str, false);
    }

    inter_set_inputaddr(pInImage->pixformat, &addrInfo,HAL_LAYER_VIDEO1);
    
    hal_set_zme_reso(HAL_LAYER_VIDEO1,vhdrect );

    hal_set_read_mode(HAL_LAYER_VIDEO1,HAL_PROGRESSIVE,HAL_PROGRESSIVE);

    inter_set_wbcconfig(HAL_LAYER_WBC1,true,pOutImage);

    if (pOutImage->width != pInImage->width)
    {
        hal_set_zme_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_HOR, 1);
        hal_set_zme_fir_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_HOR, 1);    
        hal_set_mid_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_HOR, 1);
        hal_set_zme_phase (HAL_LAYER_VIDEO1,HAL_ZMEMODE_HOR,0);

        if ( pOutImage->width > VPP_RECT_MAX_W )
        {
            hal_set_hfir_order(HAL_LAYER_VIDEO1, 1);
        }
        else 
        {
            hal_set_hfir_order(HAL_LAYER_VIDEO1, 0);
        }
    }
    
    if ( pOutImage->height != pInImage->height )
    {
        hal_set_zme_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_VER, 1);
        hal_set_zme_fir_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_VER, 1);    
        hal_set_mid_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_VER, 1);
        hal_set_zme_phase (HAL_LAYER_VIDEO1, HAL_ZMEMODE_VER,0);
    }
#if 1
    if ( k3_true == pVppConfig->bscal_enable)
    {
        logi("Zme Fir Enable!\n");
        hal_set_zme_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_HOR, 1);
        hal_set_zme_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_VER, 1);

        hal_set_zme_fir_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_HOR, 1);
        hal_set_zme_fir_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_VER, 1);

        hal_set_mid_enable   (HAL_LAYER_VIDEO1, HAL_ZMEMODE_HOR, 1);
        hal_set_mid_enable   (HAL_LAYER_VIDEO1, HAL_ZMEMODE_VER, 1);

        hal_set_zme_phase (HAL_LAYER_VIDEO1,HAL_ZMEMODE_HOR,0);
        hal_set_zme_phase (HAL_LAYER_VIDEO1,HAL_ZMEMODE_VER,0);

        hal_set_hfir_order(HAL_LAYER_VIDEO1, 0);

        hal_set_coef_addr (HAL_LAYER_VIDEO1, HAL_COEFMODE_HOR, 
                        (unsigned int)s_stZoomCoef.pStartPhyAddr);
        hal_set_coef_addr (HAL_LAYER_VIDEO1, HAL_COEFMODE_VER, (unsigned int)(s_stZoomCoef.pStartPhyAddr+
                            ZME_H_COEFF_SIZE*4));

        hal_set_layer_para_upd(HAL_LAYER_VIDEO1, HAL_COEFMODE_HOR);
        hal_set_layer_para_upd(HAL_LAYER_VIDEO1, HAL_COEFMODE_VER);
    }
    else
    {
        logi("Zme Fir Disable!\n");
        hal_set_zme_fir_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_HOR, 0);
        hal_set_zme_fir_enable(HAL_LAYER_VIDEO1, HAL_ZMEMODE_VER, 0);

        hal_set_mid_enable   (HAL_LAYER_VIDEO1, HAL_ZMEMODE_HOR, 0);
        hal_set_mid_enable   (HAL_LAYER_VIDEO1, HAL_ZMEMODE_VER, 0);

        hal_set_zme_phase (HAL_LAYER_VIDEO1,HAL_ZMEMODE_HOR,0);
        hal_set_zme_phase (HAL_LAYER_VIDEO1,HAL_ZMEMODE_VER,0);

        hal_set_hfir_order  (HAL_LAYER_VIDEO1, 0);
    }   
#endif

    inter_set_cscconfig(HAL_LAYER_VIDEO1,pVppConfig->encsc_mode, pVppConfig->bcsc_enable);

    hal_set_die_enable(HAL_LAYER_VIDEO1, 0,0);
    
    hal_set_acm_enable(HAL_ACMBLK_ID0, 0);
    hal_set_acm_enable(HAL_ACMBLK_ID1, 0);
    hal_set_acm_enable(HAL_ACMBLK_ID2, 0);
    hal_set_acm_enable(HAL_ACMBLK_ID3, 0);
    
    hal_set_acc_ctrl(HAL_LAYER_VIDEO1, 0, 0);
    
    log_reg_value();
    hal_set_zme_ratio();
    hal_set_regup(HAL_LAYER_VIDEO1);
    
    return K3_SUCCESS;
}


