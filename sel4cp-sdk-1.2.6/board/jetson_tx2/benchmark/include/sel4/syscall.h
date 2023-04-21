/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */


/* This header was generated by kernel/tools/syscall_header_gen.py.
 *
 * To add a system call number, edit kernel/libsel4/include/api/syscall.xml
 *
 */
#pragma once

#include <autoconf.h>

typedef enum {
       seL4_SysCall = -1,
       seL4_SysReplyRecv = -2,
       seL4_SysNBSendRecv = -3,
       seL4_SysNBSendWait = -4,
       seL4_SysSend = -5,
       seL4_SysNBSend = -6,
       seL4_SysRecv = -7,
       seL4_SysNBRecv = -8,
       seL4_SysWait = -9,
       seL4_SysNBWait = -10,
       seL4_SysYield = -11,
#if defined(CONFIG_PRINTING)
       seL4_SysDebugPutChar = -12,
       seL4_SysDebugDumpScheduler = -13,
#endif /* defined(CONFIG_PRINTING) */
#if defined(CONFIG_DEBUG_BUILD)
       seL4_SysDebugHalt = -14,
       seL4_SysDebugCapIdentify = -15,
       seL4_SysDebugSnapshot = -16,
       seL4_SysDebugNameThread = -17,
#endif /* defined(CONFIG_DEBUG_BUILD) */
#if (defined(CONFIG_DEBUG_BUILD) && defined(CONFIG_ENABLE_SMP_SUPPORT))
       seL4_SysDebugSendIPI = -18,
#endif /* (defined(CONFIG_DEBUG_BUILD) && defined(CONFIG_ENABLE_SMP_SUPPORT)) */
#if defined(CONFIG_DANGEROUS_CODE_INJECTION)
       seL4_SysDebugRun = -19,
#endif /* defined(CONFIG_DANGEROUS_CODE_INJECTION) */
#if defined(CONFIG_ENABLE_BENCHMARKS)
       seL4_SysBenchmarkFlushCaches = -20,
       seL4_SysBenchmarkResetLog = -21,
       seL4_SysBenchmarkFinalizeLog = -22,
       seL4_SysBenchmarkSetLogBuffer = -23,
       seL4_SysBenchmarkNullSyscall = -24,
#endif /* defined(CONFIG_ENABLE_BENCHMARKS) */
#if defined(CONFIG_BENCHMARK_TRACK_UTILISATION)
       seL4_SysBenchmarkGetThreadUtilisation = -25,
       seL4_SysBenchmarkResetThreadUtilisation = -26,
#endif /* defined(CONFIG_BENCHMARK_TRACK_UTILISATION) */
#if (defined(CONFIG_DEBUG_BUILD) && defined(CONFIG_BENCHMARK_TRACK_UTILISATION))
       seL4_SysBenchmarkDumpAllThreadsUtilisation = -27,
       seL4_SysBenchmarkResetAllThreadsUtilisation = -28,
#endif /* (defined(CONFIG_DEBUG_BUILD) && defined(CONFIG_BENCHMARK_TRACK_UTILISATION)) */
#if defined(CONFIG_KERNEL_X86_DANGEROUS_MSR)
       seL4_SysX86DangerousWRMSR = -29,
       seL4_SysX86DangerousRDMSR = -30,
#endif /* defined(CONFIG_KERNEL_X86_DANGEROUS_MSR) */
#if defined(CONFIG_VTX)
       seL4_SysVMEnter = -31,
#endif /* defined(CONFIG_VTX) */
#if defined(CONFIG_SET_TLS_BASE_SELF)
       seL4_SysSetTLSBase = -32,
#endif /* defined(CONFIG_SET_TLS_BASE_SELF) */
    SEL4_FORCE_LONG_ENUM(seL4_Syscall_ID)
} seL4_Syscall_ID;
