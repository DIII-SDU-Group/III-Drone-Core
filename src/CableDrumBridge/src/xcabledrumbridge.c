// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xcabledrumbridge.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XCabledrumbridge_CfgInitialize(XCabledrumbridge *InstancePtr, XCabledrumbridge_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Axi_cpu_BaseAddress = ConfigPtr->Axi_cpu_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XCabledrumbridge_Set_mode_CPU(XCabledrumbridge *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XCabledrumbridge_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_MODE_CPU_DATA, Data);
}

u32 XCabledrumbridge_Get_mode_CPU(XCabledrumbridge *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XCabledrumbridge_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_MODE_CPU_DATA);
    return Data;
}

void XCabledrumbridge_Set_duty_cycle_CPU(XCabledrumbridge *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XCabledrumbridge_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_DUTY_CYCLE_CPU_DATA, Data);
}

u32 XCabledrumbridge_Get_duty_cycle_CPU(XCabledrumbridge *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XCabledrumbridge_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_DUTY_CYCLE_CPU_DATA);
    return Data;
}

void XCabledrumbridge_Set_man_dir_CPU(XCabledrumbridge *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XCabledrumbridge_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_MAN_DIR_CPU_DATA, Data);
}

u32 XCabledrumbridge_Get_man_dir_CPU(XCabledrumbridge *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XCabledrumbridge_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_MAN_DIR_CPU_DATA);
    return Data;
}

void XCabledrumbridge_Set_ref_flex_CPU(XCabledrumbridge *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XCabledrumbridge_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_REF_FLEX_CPU_DATA, Data);
}

u32 XCabledrumbridge_Get_ref_flex_CPU(XCabledrumbridge *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XCabledrumbridge_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_REF_FLEX_CPU_DATA);
    return Data;
}

u32 XCabledrumbridge_Get_sensor_flex_CPU(XCabledrumbridge *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XCabledrumbridge_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_SENSOR_FLEX_CPU_DATA);
    return Data;
}

u32 XCabledrumbridge_Get_sensor_flex_CPU_vld(XCabledrumbridge *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XCabledrumbridge_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_SENSOR_FLEX_CPU_CTRL);
    return Data & 0x1;
}

void XCabledrumbridge_Set_gain_CPU(XCabledrumbridge *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XCabledrumbridge_WriteReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_GAIN_CPU_DATA, Data);
}

u32 XCabledrumbridge_Get_gain_CPU(XCabledrumbridge *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XCabledrumbridge_ReadReg(InstancePtr->Axi_cpu_BaseAddress, XCABLEDRUMBRIDGE_AXI_CPU_ADDR_GAIN_CPU_DATA);
    return Data;
}

