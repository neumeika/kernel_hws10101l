
#include <linux/kernel.h>
#include <mach/product_feature_sel.h>

#define HW_VERSION_VOLTAGE_LVEL0	0
#define HW_VERSION_VOLTAGE_LVEL1	381
#define HW_VERSION_VOLTAGE_LVEL2	807
#define HW_VERSION_VOLTAGE_LVEL3	1185
#define HW_VERSION_VOLTAGE_LVEL4	1573
#define HW_VERSION_VOLTAGE_LVEL5	1973
#define HW_VERSION_VOLTAGE_LVEL6	2395
#define HW_VERSION_VOLTAGE_LVEL7	3300
#define VOL_TOLERANCE 				30

extern void get_hwversion_num(int* version1, int* version2, int* min_version1,int* min_version2);

unsigned int g_product_feature_array[PROD_FEATURE_MAX_NUM] = {0};

unsigned int get_product_feature(enum PROD_FEATURE_SEL_TYPE feature_type)
{
	if((feature_type < 0) || (feature_type >= PROD_FEATURE_MAX_NUM))
		return -1;
	printk("[product feature]%s:%d\n",__func__,g_product_feature_array[feature_type]);
	return g_product_feature_array[feature_type];
}

static bool init_lte_hardware_detect(void)
{
	bool is_lte = 0;
	unsigned int version1 = 0, version2 =0, min_version1 = 0, min_version2 = 0;
	get_hwversion_num(&version1, &version2, &min_version1,&min_version2);
	if((version2 >= HW_VERSION_VOLTAGE_LVEL4 - VOL_TOLERANCE) &&
		(version2 <= HW_VERSION_VOLTAGE_LVEL4 + VOL_TOLERANCE))
	{
		if(((version1>=HW_VERSION_VOLTAGE_LVEL2- VOL_TOLERANCE)&&
			(version1<=HW_VERSION_VOLTAGE_LVEL2 + VOL_TOLERANCE))||
                  ((version1>=HW_VERSION_VOLTAGE_LVEL3- VOL_TOLERANCE)&&
			(version1<=HW_VERSION_VOLTAGE_LVEL3 + VOL_TOLERANCE))||
		    ((version1>=HW_VERSION_VOLTAGE_LVEL5- VOL_TOLERANCE)&&
			(version1<=HW_VERSION_VOLTAGE_LVEL5 + VOL_TOLERANCE))||
		    ((version1>=HW_VERSION_VOLTAGE_LVEL6- VOL_TOLERANCE)&&
			(version1<=HW_VERSION_VOLTAGE_LVEL6 + VOL_TOLERANCE))||
		    ((version1>=HW_VERSION_VOLTAGE_LVEL7- VOL_TOLERANCE)&&
			(version1<=HW_VERSION_VOLTAGE_LVEL7 + VOL_TOLERANCE)))
		is_lte = 1;
	}
	printk("[product feature]%s:%d (0: no LTE ; 1: LTE)\n",__func__,is_lte);
	return is_lte;
}


/*
	func	: choose the usb out 5v supply or not by hardware version
	input	: void
	output	: 0 : ES/CS before V4
			  1	: CS V4 or later
*/
static int init_usb_out_5v_supply()
{
	unsigned int version1 = 0, version2 =0, min_version1 = 0, min_version2 = 0;
	int supply = 0;
	get_hwversion_num(&version1, &version2, &min_version1,&min_version2);
	if(min_version2 == 1)
	{
		supply = 1;
	}
	else
	{
		supply = 0;
	}
	printk("[product feature]%s:%d (0: ES/CS before V4; 1: CS V4 or later)\n",__func__,supply);
	return supply;
}

/*
	func	: detect whether have mhl chip or not by hardware version
	input	: void
	output	: 0 : no mhl chip
			  1	: have mhl chip
*/
static int init_mhl_chip_detect()
{
	unsigned int version1 = 0, version2 =0, min_version1 = 0, min_version2 = 0;
	int supply = 0;
	get_hwversion_num(&version1, &version2, &min_version1,&min_version2);

	if((version2 >= HW_VERSION_VOLTAGE_LVEL6 - VOL_TOLERANCE) 
		&&(version2 <= HW_VERSION_VOLTAGE_LVEL6 + VOL_TOLERANCE))
	{
		if(((version1>=HW_VERSION_VOLTAGE_LVEL3- VOL_TOLERANCE)
			&&(version1<=HW_VERSION_VOLTAGE_LVEL3 + VOL_TOLERANCE))
			||((version1>=HW_VERSION_VOLTAGE_LVEL4- VOL_TOLERANCE)
			&&(version1<=HW_VERSION_VOLTAGE_LVEL4 + VOL_TOLERANCE)))//202u 不带mhl
			{
				supply = 1;
			}
	}
	else if((version2 >= HW_VERSION_VOLTAGE_LVEL7 - VOL_TOLERANCE) 
			&&(version2 <= HW_VERSION_VOLTAGE_LVEL7 + VOL_TOLERANCE))
	{
		if(((version1>=HW_VERSION_VOLTAGE_LVEL4 - VOL_TOLERANCE)
			&&(version1<=HW_VERSION_VOLTAGE_LVEL4 + VOL_TOLERANCE))
			||((version1>=HW_VERSION_VOLTAGE_LVEL2- VOL_TOLERANCE)
			&&(version1<=HW_VERSION_VOLTAGE_LVEL2 + VOL_TOLERANCE)))//201u 不带mhl
			{
				supply = 1;
			}		
	}	
	else if((version2 >= HW_VERSION_VOLTAGE_LVEL3 - VOL_TOLERANCE)
		 	&& (version1 <= HW_VERSION_VOLTAGE_LVEL3 + VOL_TOLERANCE))//201w,201u,201L
	{
		if((version1 >= HW_VERSION_VOLTAGE_LVEL2 - VOL_TOLERANCE)
			&&(version1 <= HW_VERSION_VOLTAGE_LVEL2 + VOL_TOLERANCE))//不带mhl
		{
			supply = 1;
		}
	}
	printk("[product feature]%s:%d (0: no mhl chip; 1: have mhl chip)\n",__func__,supply);
	return supply;
}

/*
	func	: choose the gpu dcdc supply or not by hardware version
	input	: void
	output	: 0 : not supply gpu dcdc
			  1	: supply gpu dcdc
*/
static int init_gpu_dcdc_supply()
{
	unsigned int version1 = 0, version2 =0, min_version1 = 0, min_version2 = 0;
	int supply = 0;

	get_hwversion_num(&version1, &version2, &min_version1,&min_version2);

	if((version2 >= HW_VERSION_VOLTAGE_LVEL4 - VOL_TOLERANCE)
		&& (version2 <= HW_VERSION_VOLTAGE_LVEL4 + VOL_TOLERANCE)
		&& (version1 >= HW_VERSION_VOLTAGE_LVEL5 - VOL_TOLERANCE)) // LTE 101L  103L
	{
		supply = 1;
	}
	else if((version2 <= HW_VERSION_VOLTAGE_LVEL3 + VOL_TOLERANCE)
		&& (version2 >= HW_VERSION_VOLTAGE_LVEL3 - VOL_TOLERANCE)
		&& (version1 >= HW_VERSION_VOLTAGE_LVEL7 - VOL_TOLERANCE))  //LTE 101wf
	{
		supply = 1;
	}
	else if((version2 >= HW_VERSION_VOLTAGE_LVEL5 - VOL_TOLERANCE)
		&& (version1 <= HW_VERSION_VOLTAGE_LVEL5 + VOL_TOLERANCE)
		&& (version1 >= HW_VERSION_VOLTAGE_LVEL5 - VOL_TOLERANCE)) //101w, 102u, 101u : V4 & higher than v4
	{
			supply = 1;
	}
	else if((version2 >= HW_VERSION_VOLTAGE_LVEL5 - VOL_TOLERANCE)
		&& (version2 <= HW_VERSION_VOLTAGE_LVEL5 + VOL_TOLERANCE)
		&& (version1 >= HW_VERSION_VOLTAGE_LVEL4 - VOL_TOLERANCE)
		&& (version1 <= HW_VERSION_VOLTAGE_LVEL4 + VOL_TOLERANCE)) // 101wt
	{
			supply = 1;
	}
#if 0
	else if((version2 >= HW_VERSION_VOLTAGE_LVEL7 - VOL_TOLERANCE) &&
		(version2 <= HW_VERSION_VOLTAGE_LVEL7 + VOL_TOLERANCE))
	{
		if(version1 <= HW_VERSION_VOLTAGE_LVEL3 + VOL_TOLERANCE) //201u
			supply = 1;
	}
#endif
	else if(((int)version2 >= (int)(HW_VERSION_VOLTAGE_LVEL0 - VOL_TOLERANCE)) &&
			(((int)version2 <=(int)(HW_VERSION_VOLTAGE_LVEL0 + VOL_TOLERANCE))))
	{
		if((version1 >= HW_VERSION_VOLTAGE_LVEL3 - VOL_TOLERANCE) && version1 <= HW_VERSION_VOLTAGE_LVEL3 + VOL_TOLERANCE) // temp verb
			supply = 1;
	}
	printk("[product feature]%s:%d (0: not supply gpu dcdc; 1: supply gpu dcdc)\n",__func__,supply);
	return supply;
}

/*
	func	: detect dcm battery by hardware version
	input	: void
	output	: 0 : not dcm battery
			  1	: dcm battery
*/
static int init_dcm_battery_detect()
{
	unsigned int version1 = 0, version2 =0, min_version1 = 0, min_version2 = 0;
	int supply = 0;

	get_hwversion_num(&version1, &version2, &min_version1,&min_version2);

	if((version2 >= HW_VERSION_VOLTAGE_LVEL3 - VOL_TOLERANCE)
        && (version2 <= HW_VERSION_VOLTAGE_LVEL3 + VOL_TOLERANCE)
        && (version1 >= HW_VERSION_VOLTAGE_LVEL3 - VOL_TOLERANCE)
        && (version1 >= HW_VERSION_VOLTAGE_LVEL3 + VOL_TOLERANCE)) 
	{
			supply = 1;
	}
	else
	{
			supply = 0;
	}

	printk("[product feature]%s:%d (0: not dcm hardware; 1: dcm hardware)\n",__func__,supply);

	return supply;
}

/*
	func	: set the sdp charging current by hardware version
	input	: void
	output	: 0 : charger current 2A
			  1	: charger current 500mA
*/
static int init_sdp_charger_current()
{
	unsigned int version1 = 0, version2 =0, min_version1 = 0, min_version2 = 0;
	int supply = 0;

	get_hwversion_num(&version1, &version2, &min_version1,&min_version2);
	if(version2 >= HW_VERSION_VOLTAGE_LVEL5 - VOL_TOLERANCE) // 101w, 102u, 101u
	{
		if(version1 >= HW_VERSION_VOLTAGE_LVEL6 - VOL_TOLERANCE) // only V3 or before
			supply = 0;
		else
			supply = 1;
	}
	else
	{
			supply = 1;
	}

	printk("[product feature]%s:%d (0: charger current 2A; 1: charger current 500mA)\n",__func__,supply);
	return supply;
}

unsigned int init_product_feature_by_type(enum PROD_FEATURE_SEL_TYPE feature_type)
{
	if((feature_type < 0) || (feature_type >= PROD_FEATURE_MAX_NUM))
		return -1;

	switch(feature_type)
	{
		case PROD_FEATURE_GPU_DCDC_SUPPLY:
			g_product_feature_array[feature_type] = init_gpu_dcdc_supply();
			break;
		case PROD_FEATURE_SDP_CHARGER_CURRENT:
			g_product_feature_array[feature_type] = init_sdp_charger_current();
			break;
		case PROD_FEATURE_USB_OUT_5V_SUPPLY:
			g_product_feature_array[feature_type] = init_usb_out_5v_supply();
			break;
		case PROD_FEATURE_DETECT_MHL_CHIP:
			g_product_feature_array[feature_type] = init_mhl_chip_detect();
			break;
		case PROD_FEATURE_DETECT_LTE_HARDWARE:
			g_product_feature_array[feature_type] = init_lte_hardware_detect();
			break;
        case PROD_FEATURE_DETECT_DCM_HARDWARE:
            g_product_feature_array[feature_type] = init_dcm_battery_detect();
			break;
		default:
			printk("[product feature]unknown feature_type\n");
			return -1;
	}
	return 0;
}

static int init_type_array[] =
{
	PROD_FEATURE_GPU_DCDC_SUPPLY,
	PROD_FEATURE_SDP_CHARGER_CURRENT,
	PROD_FEATURE_USB_OUT_5V_SUPPLY,
	PROD_FEATURE_DETECT_MHL_CHIP,
    PROD_FEATURE_DETECT_LTE_HARDWARE,
    PROD_FEATURE_DETECT_DCM_HARDWARE,
};
#ifndef ARRAY_SIZE
 #define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif
void init_product_feature_array()
{
	int i = 0;
	enum PROD_FEATURE_SEL_TYPE feature_type = 0;
	printk("[product feature]init_product_feature_array\n");
	for (i = 0;  i < ARRAY_SIZE(init_type_array); i++) {
		feature_type = init_type_array[i];
		init_product_feature_by_type(feature_type);
    }
}

