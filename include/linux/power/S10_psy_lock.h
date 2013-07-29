/*
 * Copyright 2010 S10 Tech. Co., Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 *  S10 tablet standard power module driver
 *  author: wufan w00163571 2012-08-13
 *
 */

/*=============================================================================
修订历史

问题单号              修改人            日期                           原因
==============================================================================*/
#ifndef _S10_PSY_LOCK_H_
#define _S10_PSY_LOCK_H_
#include <linux/semaphore.h>

enum S10_power_lock_type
{
    S10_POWER_LOCK_LIGHT,
    S10_POWER_LOCK_HEAVY,
    S10_POWER_LOCK_SEMP,
    S10_POWER_LOCK_UNDEF,
};

struct S10_power_lock
{
    int           lock_type;
    unsigned long irq_flags;
    union
    {
        struct spinlock  light_lock;
        struct mutex     heavy_lock;
        struct semaphore semp_lock;
    } lock;
};

int  S10_power_lock_init(enum S10_power_lock_type lock_type, struct S10_power_lock* lock);
void S10_power_lock_deinit(struct S10_power_lock* lock);
int  S10_power_lock_lock(struct S10_power_lock* lock);
int  S10_power_lock_unlock(struct S10_power_lock* lock);
#endif
