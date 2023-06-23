// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xcabledrumbridge.h"

extern XCabledrumbridge_Config XCabledrumbridge_ConfigTable[];

XCabledrumbridge_Config *XCabledrumbridge_LookupConfig(u16 DeviceId) {
	XCabledrumbridge_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XCABLEDRUMBRIDGE_NUM_INSTANCES; Index++) {
		if (XCabledrumbridge_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XCabledrumbridge_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XCabledrumbridge_Initialize(XCabledrumbridge *InstancePtr, u16 DeviceId) {
	XCabledrumbridge_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XCabledrumbridge_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XCabledrumbridge_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

