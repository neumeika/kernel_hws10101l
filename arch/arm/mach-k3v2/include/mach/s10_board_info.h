#ifndef __MACH_K3V2_S10_BOARD_H
#define __MACH_K3V2_S10_BOARD_H
/* New definitions for supporting multiple sub-projects */
#define  S10_HWID_L1(X1)    			(defined(CONFIG_MACH_K3V200_##X1))
#define  S10_HWID_L2(X1, X2)    		(defined(CONFIG_MACH_K3V200_##X1) \
										 && defined(CONFIG_MACH_K3V200_##X1##_##X2))
#define  S10_HWID_L3H(X1, X2, X3)    (defined(CONFIG_MACH_K3V200_##X1) \
										 && defined(CONFIG_MACH_K3V200_##X1##_##X2) \
    									 && (  CONFIG_MACH_K3V200_##X1##_##X2##_HWVER \
    									    >= CONFIG_MACH_K3V200_##X1##_##X2##_HWVER_##X3))
#define  S10_HWID_L3L(X1, X2, X3)    (defined(CONFIG_MACH_K3V200_##X1) \
                                         && defined(CONFIG_MACH_K3V200_##X1##_##X2) \
                                         && (  CONFIG_MACH_K3V200_##X1##_##X2##_HWVER \
                                                <= CONFIG_MACH_K3V200_##X1##_##X2##_HWVER_##X3))
#define  S10_HWID_L3D(X1, X2, X3, X4) (S10_HWID_L3H(X1, X2, X3) && S10_HWID_L3L(X1, X2, X4))

/* New definitions for supporting multiple sub-projects */
#define  CONFIG_MACH_K3V200_S10
#define  CONFIG_MACH_K3V200_S10_S10101
#define  CONFIG_MACH_K3V200_S10_S10101_HWVER_A  1
#define  CONFIG_MACH_K3V200_S10_S10101_HWVER  CONFIG_MACH_K3V200_S10_S10101_HWVER_A

#endif