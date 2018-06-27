#pragma once
#include "device_table.h"

void blePrintAdvPacketData(uint8_t *data,int len,char *pad);
void clear_terminal();
void printDeviceOneLine(scan_device_t *device, uint32_t index);
void printDeviceTableOneLine();
void printDeviceTableMultiLine();
void printRecentFilterData();
void printMostRecentFilterData();

void decrementPageNum_m();
void incrementPageNum_m();
void decrementPageNum_r();
void incrementPageNum_r();
void reset_tables();
