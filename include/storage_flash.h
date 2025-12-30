#ifndef __STORAGE_FLASH_H__
#define __STORAGE_FLASH_H__

#include "stdio.h"
#include "main.h"

void storageInit(void);
void storageLocalConfig(robot_local_configs_t params);
robot_local_configs_t getFromStorageLocalConfig(void);

#endif