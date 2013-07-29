/* AUTHOR: huyouhua 00136760
 * DESC: add file for control log print by tag at debugging.
 * HISTORY: 1. add file at 2012.07.05
 */
#ifndef __SII_9244_LOG_H__
#define __SII_9244_LOG_H__


extern int gMHLprintTag;

#define TX_DEBUG_PRINT(x) \
    do { \
        if (gMHLprintTag) \
        { \
            printk x ; \
        } \
    }while(0)

#define TX_API_PRINT(x) \
    do { \
        if (gMHLprintTag) \
        { \
            printk x; \
        } \
    }while(0)

#endif
/*#ifndef __SII_9244_LOG_H__*/
