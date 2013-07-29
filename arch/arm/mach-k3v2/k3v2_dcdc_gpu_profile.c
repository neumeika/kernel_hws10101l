
/* DO NOT EDIT - Generated automatically by k3v2_dcdc_gpu_profile_gen.pl */


#include "k3v2_dcdc_gpu.h"

struct gpu_profile_info gpu_profile[] = {
	/*freq, gpu_clk_profile, gpu_vol_profile, dcdc_vol*/
	{58000, 0x00060C18, 0x00000232, 1054000},
	{120000, 0x0002C58B, 0x00000232, 1054000},
	{240000, 0x00014285, 0x00000238, 1097000},
	{360000, 0x0000C183, 0x00000247, 1203000},
	{480000, 0x0000C182, 0x0000025A, 1331000},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_powersave[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 2, 95, 30},
	{1, 3, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 100, 12},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_normal[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 2, 95, 30},
	{1, 3, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 100, 12},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_performance[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 2, 95, 30},
	{1, 3, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 100, 12},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special01[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 95, 30},
	{1, 1, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 100, 12},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special02[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 95, 30},
	{1, 1, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 100, 12},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special03[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 95, 30},
	{1, 1, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 100, 12},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special04[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 95, 30},
	{1, 1, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 80, 30},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special05[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 95, 30},
	{1, 1, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 80, 30},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special06[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 95, 30},
	{1, 1, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 80, 30},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special07[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 95, 30},
	{1, 1, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 80, 30},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special08[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 95, 30},
	{1, 1, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 80, 30},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special09[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 95, 30},
	{1, 1, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 80, 30},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special0A[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 95, 30},
	{1, 1, 99, 40},
	{1, 1, 100, 50},
	{1, 1, 80, 30},
	{0, 0, 0, 0},
};
struct gpu_policy_info gpu_policy_special0B[] = {
	/*uptimes, downtimes, up_threshold, down_threshold*/
	{1, 1, 75, 30},
	{1, 1, 80, 30},
	{1, 1, 80, 30},
	{1, 1, 80, 30},
	{1, 1, 23, 7},
	{0, 0, 0, 0},
};
struct gpu_policy_info *policy_table[] = {
	[NORMAL_POLICY] = gpu_policy_normal,
	[POWERSAVE_POLICY] = gpu_policy_powersave,
	[PERF_POLICY] = gpu_policy_performance,
	[SPEC01_POLICY] = gpu_policy_special01,
	[SPEC02_POLICY] = gpu_policy_special02,
	[SPEC03_POLICY] = gpu_policy_special03,
	[SPEC04_POLICY] = gpu_policy_special04,
	[SPEC05_POLICY] = gpu_policy_special05,
	[SPEC06_POLICY] = gpu_policy_special06,
	[SPEC07_POLICY] = gpu_policy_special07,
	[SPEC08_POLICY] = gpu_policy_special08,
	[SPEC09_POLICY] = gpu_policy_special09,
	[SPEC0A_POLICY] = gpu_policy_special0A,
	[SPEC0B_POLICY] = gpu_policy_special0B,
};
