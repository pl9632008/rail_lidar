#ifndef LADAR_H
#define LADAR_H

#include "manager/ConfigReader.h"
#include "xp_lidar.h"
#include "livox_lidar_def.h"
#include "livox_lidar_sdk.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <QTime>

int Livox2Init();
void LoadConfig();
void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data);
void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data);
void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data);
void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data);
void QueryInternalInfoCallback(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data);
void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data);
void G_JudegeLidarStatus();


#endif // LADAR_H
