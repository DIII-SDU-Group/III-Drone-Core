// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XCABLEDRUMBRIDGE_H
#define XCABLEDRUMBRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xcabledrumbridge_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u32 Axi_cpu_BaseAddress;
} XCabledrumbridge_Config;
#endif

typedef struct {
    u64 Axi_cpu_BaseAddress;
    u32 IsReady;
} XCabledrumbridge;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XCabledrumbridge_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XCabledrumbridge_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XCabledrumbridge_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XCabledrumbridge_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XCabledrumbridge_Initialize(XCabledrumbridge *InstancePtr, u16 DeviceId);
XCabledrumbridge_Config* XCabledrumbridge_LookupConfig(u16 DeviceId);
int XCabledrumbridge_CfgInitialize(XCabledrumbridge *InstancePtr, XCabledrumbridge_Config *ConfigPtr);
#else
int XCabledrumbridge_Initialize(XCabledrumbridge *InstancePtr, const char* InstanceName);
int XCabledrumbridge_Release(XCabledrumbridge *InstancePtr);
#endif


void XCabledrumbridge_Set_mode_CPU(XCabledrumbridge *InstancePtr, u32 Data);
u32 XCabledrumbridge_Get_mode_CPU(XCabledrumbridge *InstancePtr);
void XCabledrumbridge_Set_duty_cycle_CPU(XCabledrumbridge *InstancePtr, u32 Data);
u32 XCabledrumbridge_Get_duty_cycle_CPU(XCabledrumbridge *InstancePtr);
void XCabledrumbridge_Set_man_dir_CPU(XCabledrumbridge *InstancePtr, u32 Data);
u32 XCabledrumbridge_Get_man_dir_CPU(XCabledrumbridge *InstancePtr);
void XCabledrumbridge_Set_ref_flex_CPU(XCabledrumbridge *InstancePtr, u32 Data);
u32 XCabledrumbridge_Get_ref_flex_CPU(XCabledrumbridge *InstancePtr);
u32 XCabledrumbridge_Get_sensor_flex_CPU(XCabledrumbridge *InstancePtr);
u32 XCabledrumbridge_Get_sensor_flex_CPU_vld(XCabledrumbridge *InstancePtr);
void XCabledrumbridge_Set_gain_CPU(XCabledrumbridge *InstancePtr, u32 Data);
u32 XCabledrumbridge_Get_gain_CPU(XCabledrumbridge *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
