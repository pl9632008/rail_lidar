#include "Ladar.h"

BinaryPoint *receive_point = nullptr;
int pos = 0;
int max_limit = 120000;
//int max_limit = 90000;

bool changeFlag = false;
LadarConf m_ladarConf;
LadarConf m_ladarConf_BACK;

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
    if (data == nullptr) {
        return;
    }

    if (changeFlag) {
        delete [] receive_point;
        receive_point = nullptr;
        receive_point = new BinaryPoint[max_limit];
        pos=0;
        changeFlag = false;
    }

    if(!(g_GPSDrection4Ladar && handle==m_ladarConf.handle)&&
            !(g_GPSDrection4Ladar && handle==m_ladarConf_BACK.handle)){
        return;
    }

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        if (receive_point == nullptr) {
            receive_point = new BinaryPoint[max_limit];
        }

        LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
        uint32_t pNum = data->dot_num;
        //BinaryPoint *tmp=new BinaryPoint[pNum];
        for (uint32_t i = 0; i < pNum; i++) {
            receive_point[pos].x=p_point_data[i].x*0.001;
            receive_point[pos].y=p_point_data[i].y*0.001;
            receive_point[pos].z=p_point_data[i].z*0.001;
            uint8_t ref = p_point_data[i].reflectivity;
            receive_point[pos].r=ref;
            receive_point[pos].g=0;
            receive_point[pos].b=255-ref;
            pos++;
            if(pos==max_limit){
                pos=0;
            }
        }

        if(LidarSave){
            LIDAR_TIME = QTime::currentTime();
            xp_ExtraPoints(receive_point, max_limit);
            delete [] receive_point;
            receive_point = nullptr;
            receive_point = new BinaryPoint[max_limit];
            LidarSave=false;
        }
    }
    else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
        LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)data->data;
    } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
        LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;
    }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,    LivoxLidarEthernetPacket* data, void* client_data) {
    if (data == nullptr) {
        return;
    }
    printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
           handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (response == nullptr) {
        return;
    }
    printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
           status, handle, response->ret_code, response->error_key);

}


void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
    if (response == nullptr) {
        return;
    }
    printf("RebootCallback, status:%u, handle:%u, ret_code:%u",
           status, handle, response->ret_code);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (response == nullptr) {
        return;
    }
    printf("LivoxLidarIpInfoCallback, status:%u, handle:%u, ret_code:%u, error_key:%u",
           status, handle, response->ret_code, response->error_key);

    if (response->ret_code == 0 && response->error_key == 0) {
        LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
    }
}


void QueryInternalInfoCallback(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
    if (status != kLivoxLidarStatusSuccess) {
        printf("Query lidar internal info failed.\n");
        QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
        return;
    }

    if (response == nullptr) {
        return;
    }

    uint8_t host_point_ipaddr[4];
    uint16_t host_point_port;
    uint16_t lidar_point_port;

    uint8_t host_imu_ipaddr[4];
    uint16_t host_imu_data_port;
    uint16_t lidar_imu_data_port;

    uint16_t off = 0;
    for (uint8_t i = 0; i < response->param_num; ++i) {
        LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
        if (kv->key == kKeyLidarPointDataHostIPCfg) {
            memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
            memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
            memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
        } else if (kv->key == kKeyLidarImuHostIPCfg) {
            memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
            memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
            memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
        }
        off += sizeof(uint16_t) * 2;
        off += kv->length;
    }
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
    if (info == nullptr) {
        printf("lidar info change callback failed, the info is nullptr.\n");
        return;
    }
    printf("LidarInfoChangeCallback Lidar handle: %d SN: %s\n", handle, info->sn);
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);

    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

void LoadConfig()
{
    QString path = QCoreApplication::applicationDirPath() + QString(CONFIG_FILE_NAME);
    INIConfigReader reader = INIConfigReader(path);

    m_ladarConf.ip = reader.GetConfigByKey("Ladar","ip").toString().toStdString();
    m_ladarConf.handle = (uint32_t)reader.GetConfigByKey("Ladar","handle").toString().toUInt();
    m_ladarConf_BACK.ip = reader.GetConfigByKey("Ladar_Back","ip").toString().toStdString();
    m_ladarConf_BACK.handle = (uint32_t)reader.GetConfigByKey("Ladar_Back","handle").toString().toUInt();

    lidar_height_1 = reader.GetConfigByKey("Ladar","height").toFloat();
    lidar_offset_1 = reader.GetConfigByKey("Ladar","offset").toFloat();
    lidar_pitch_1 = reader.GetConfigByKey("Ladar","pitch").toFloat();
    lidar_roll_1 = reader.GetConfigByKey("Ladar","roll").toFloat();
    lidar_yaw_1 = reader.GetConfigByKey("Ladar","yaw").toFloat();

    lidar_height_2 = reader.GetConfigByKey("Ladar_Back","height").toFloat();
    lidar_offset_2 = reader.GetConfigByKey("Ladar_Back","offset").toFloat();
    lidar_pitch_2 = reader.GetConfigByKey("Ladar_Back","pitch").toFloat();
    lidar_roll_2 = reader.GetConfigByKey("Ladar_Back","roll").toFloat();
    lidar_yaw_2 = reader.GetConfigByKey("Ladar_Back","yaw").toFloat();


    EDGE_RAIL = reader.GetConfigByKey("Ladar","EDGE_RAIL").toFloat();
    EDGE_DANGER = reader.GetConfigByKey("Ladar","EDGE_DANGER").toFloat();
    EDGE_WARNING = reader.GetConfigByKey("Ladar","EDGE_WARNING").toFloat();
    EDGE_HEIGHT = reader.GetConfigByKey("Ladar","EDGE_HEIGHT").toFloat();




}

int Livox2Init()
{
    LoadConfig();
    DEVICE_MODEL = 3;
    std::string path = "livox_lidar_config.json";
    while(true){
        if (!LivoxLidarSdkInit(path.c_str())) {//set ip unworking
            printf("Livox Init Failed!\n");
            LivoxLidarSdkUninit();
            g_lidarStatus = false;
            continue;
        }
        printf("connect lidar success.\n");
        bool temp_direction=g_GPSDrection4Ladar;
        if(temp_direction){//192.168.1.100
            lidar_yaw = lidar_yaw_1;
            lidar_pitch = lidar_pitch_1;
            lidar_roll = lidar_roll_1;
            lidar_offset = lidar_offset_1;
            lidar_height = lidar_height_1;
        }else {//192.168.1.101
            lidar_yaw = lidar_yaw_2;
            lidar_pitch = lidar_pitch_2;
            lidar_roll = lidar_roll_2;
            lidar_offset = lidar_offset_2;
            lidar_height = lidar_height_2;
        }
        receive_point = new BinaryPoint[max_limit];
        SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
        SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
        SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
        g_lidarStatus = true;
        while (true) {
            if(g_GPSDrection4Ladar!=temp_direction){//||g_lidarStatus==false
                if(g_GPSDrection4Ladar){
                    if(!temp_direction){
                        lidar_yaw_2 = lidar_yaw;
                        lidar_pitch_2 = lidar_pitch;
                        lidar_roll_2 = lidar_roll;
                        lidar_offset_2 = lidar_offset;
                        lidar_height_2 = lidar_height;
                    }
                    lidar_yaw = lidar_yaw_1;
                    lidar_pitch = lidar_pitch_1;
                    lidar_roll = lidar_roll_1;
                    lidar_offset = lidar_offset_1;
                    lidar_height = lidar_height_1;
                }else {
                    if(temp_direction){
                        lidar_yaw_1 = lidar_yaw;
                        lidar_pitch_1 = lidar_pitch;
                        lidar_roll_1 = lidar_roll;
                        lidar_offset_1 = lidar_offset;
                        lidar_height_1 = lidar_height;
                    }
                    lidar_yaw = lidar_yaw_2;
                    lidar_pitch = lidar_pitch_2;
                    lidar_roll = lidar_roll_2;
                    lidar_offset = lidar_offset_2;
                    lidar_height = lidar_height_2;
                }
                temp_direction=g_GPSDrection4Ladar;
                changeFlag = true;
            }
            usleep(1 * 1000 * 1000);
        }
        LivoxLidarSdkUninit();
        g_lidarStatus = false;
        printf("Livox Uninit!\n");
    }
    return 0;
}

unsigned long G_GetTickCount(QTime firstTime, QTime secondTime)
{
    unsigned long elapsed=firstTime.msecsTo(secondTime);
    return elapsed;
}


void G_JudegeLidarStatus()
{
    QTime currTime = QTime::currentTime();
    long spandTime = G_GetTickCount(LIDAR_TIME, currTime);

    if (spandTime>=1000)
    {
        LIDAR_PCD.clear();
        LIDAR_PCL.clear();
        LIDAR_TARGETS_MOVE.clear();
        g_lidarStatus = false;
    }else {
        g_lidarStatus = true;
    }
}
