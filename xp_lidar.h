#ifndef __XP_LIDAR_H
#define __XP_LIDAR_H

#ifdef _WIN32
#include <io.h>
#define Delay(x) Sleep(1000*x);
#define Delayms(x) Sleep(x);
#ifdef _WIN64
//define something for Windows (64-bit only)
#else
//define something for Windows (32-bit only)
#endif
#elif defined  __linux__ || defined __APPLE__
#define Delay(x) sleep(x);
#define Delayms(x) usleep(x*1000);
#include <signal.h>
#else
#   error "Unknown compiler"
#endif

#include <QDateTime>
#include <QVector>
#include <QtCore>
#include <QMatrix4x4>
#include <math.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv4/opencv2/opencv.hpp>

#ifdef ARM
//#include <neuv_defs.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <unistd.h>
#include <sys/time.h>
#elif __linux__
#include <neuv_defs.hpp>
#endif


//20G
//#define CAR_SIZE 20000


//Innvusion雷达相关定义
typedef double inno_timestamp_us_t;
#ifdef _WIN32
#pragma pack(1)
struct  inno_point {
    float x;                 /* in meter           */
    float y;                 /* in meter           */
    float z;                 /* in meter           */
    float radius;            /* in meter           */
    double ts_us;            /* in microsecond     */
    unsigned char ref;       /* reflectance, 1-255 */
    unsigned char flags;     /* which channel      */
};
#pragma pack()

#pragma pack(1)
struct  inno_frame {
    uint64_t idx;            /* frame index, start from 0                     */
    unsigned int sub_idx;    /* sub-frame index, start from 0 for every frame */
    inno_timestamp_us_t ts_us;   /* in microsecond                            */
    unsigned int points_number;  /* number of points                          */
    struct inno_point points[0];
};
#pragma pack()
#elif defined  __linux__ || defined __APPLE__
    //define struct in inno_lidar_api.h
#endif


//RFANs雷达相关定义

struct Definition
{
    float H_Beta;
    float V_theta;
    float Dt;
};

struct myPoint
{
    uint8_t range[2];//为 2 个字节的 unsigned integer，单位为 4.0mm
    uint8_t intensity;//取值范围为 1~255。表示经过检较的回波强度
};

struct Group
{
    uint16_t flag;//标志
    uint16_t AzimuthAgl;//水平角度  单位为 0.01°，取值范围为 0~35999
    myPoint point[32];
};
struct PCD
{
    // uint8_t udp_head[42];
    Group group[12];
    uint8_t Timestamp[4]; //4 个字节的 unsigned integer，取值范围为 0~3,599,999,999,微秒数
    uint8_t Reserved[2];

};

/*			Reserved
    0x37**
    */

//基础点结构体，用于存储解析完成的点数据
struct pointattrs
{
    QVector4D xyz;//x,y,z,添加1维(距离原点距离或其他作用),第四位添加路口ID
    QVector4D rgba;//1种作颜色属性，2种作长宽高，视具体使用处解释
    //r:处存放路口数据；1左，2右，3十字，4左车站，5右车站
    //g:处存放角度数据
    //b:目标宽度
    //a:站台标志
    QVector4D type;//类别，到目前的状态，目标沿x轴速度，其他（待定义）
    //type[0]:  0:other;1:people; 2:car
    //type[1]: 0：静止，1：运动
    //type[2]: 目标沿x轴速度
    //type[3]: 目标ID

};
//存取日志文件的二进制点结构体
struct BinaryPoint
{
    float x;
    float y;
    float z;
    uint8_t reflectivity;
    uint8_t r,g,b;
};
//线路锚点，存储关键点相关的信息
struct Anchor
{
    glm::vec4 position;//dis,侧向位移,角度,种类（无目标0，杆子1，车站2）
    pointattrs point;//X,Y,Z,L,W,H,距离->锚点参数
    glm::mat4 offset;//适用于回放DB模式下的mat4参数
};
//隧道壁截面结构体，用于Opengl绘图
struct  Edge
{
    pointattrs anchor;//锚点
    pointattrs rails[2];//铁轨
    pointattrs danger[2];//危险限界
    pointattrs warning[29];//警告限界
};
//线路定位使用的截面直方图结构体
struct histogram{
    uint8_t p[8];
};
extern std::map<int,histogram> scene_histogram;
extern std::map<int,Anchor> rail_scene[2];//距离，左右标识

extern bool g_isFrame;
extern bool g_lidarStatus;

extern QTime LIDAR_TIME;

extern glm::mat4 INS_OFST;
extern glm::vec3 INS_DIR,INS_POSITION;
extern float INS_MOVEMENT;
extern float INS_SPEED;

extern int SYSTEM_STATUS;//设备状态（-1：未知，0：正常启动，1：运行中，2：掉电重启）
extern int DEVICE_MODEL;
extern int DEVICE_STATUS;//设备状态（-1：未知，0：正常）

extern bool G_IndepentTest;

extern float lidar_yaw;
extern float lidar_pitch;
extern float lidar_roll;
extern float lidar_offset;
extern float lidar_height;

extern float lidar_yaw_1;
extern float lidar_pitch_1;
extern float lidar_roll_1;
extern float lidar_offset_1;
extern float lidar_height_1;
extern float lidar_yaw_2;
extern float lidar_pitch_2;
extern float lidar_roll_2;
extern float lidar_offset_2;
extern float lidar_height_2;

//界限尺寸
extern float EDGE_RAIL;
extern float EDGE_DANGER;
extern float EDGE_WARNING;
extern float EDGE_HEIGHT;

extern QVector<pointattrs> GPU_RULERS;//标尺
extern QVector<Edge> Edges;//线阶
extern QVector<pointattrs> LIDAR_TARGETS,LIDAR_TARGETS_last,GPU_TARGETS;//雷达输出目标
extern QVector<QVector<pointattrs>> LIDAR_TARGETS_last_all;
extern QVector<pointattrs> LIDAR_TARGETS_ROAD,LIDAR_TARGETS_MOVE;
extern QVector<glm::mat4> LIDAR_TARGETS_OFFSET;
extern QVector<pointattrs> LIDAR_RAILS,GPU_RAILS;//线路点云数据
extern QVector<pointattrs> LIDAR_PCD,LIDAR_PCL,LIDAR_UDPSEND,LIDAR_PCDTARGET;//点云数据
extern QVector<pointattrs> Global_Rail;
extern QVector<pointattrs> LIDAR_RAILS_INTERSECTION;
extern QVector<int> INTERSECTION;
extern float intersection_with;
extern float intersection_length;
extern float intersection_height;
//掉电启动时的配置参数
extern float REBOOT_MOVEMENT;
extern float REBOOT_V;
extern int REBOOT_T;

extern cv::Mat cameraMatrix;
extern cv::Mat disCoeffs;
extern cv::Mat rtMatrix;
extern cv::Mat mjpgMat;

//大小端转换
int  swap_endian(int val);
uint32_t  swap_endian(uint32_t val);
uint16_t swap_endian(uint16_t val);
//字符转hex
uint32_t StrToHex(char *src);//将8*2个字符转uint32_t
static uint8_t CharToHex(char *src);//将2个字符装uint8_t
char * HexToStr(char *ptr,int len);//hex转字符串
uint32_t FloatToHex(float src);//float转hex
float HexToFloat(uint32_t src);
void HSVtoRGB(float *r, float *g, float *b, float h, float s, float v );//色域转换，r,g,b目标rgb地址，h（色调）[0-360]，s（饱和度)[0-1]，v（明度)[0-1]
void gettime(char *des);//本地化获取时间

QVector<pointattrs> Genrate_DirVec();//点云方向向量坐标系
void Generate_Edge(QVector<pointattrs> points, QVector<pointattrs> ps_road);//绘制隧道
void Generate_Targets(QVector<pointattrs> points=QVector<pointattrs>(),QVector<glm::mat4> offsets=QVector<glm::mat4>());//绘制目标
void Genrate_Staff(int num=50,int step=5,float height=0.0f);//生成标尺和坐标轴，线数，步进，相对于原点o-z高度

void Thread_updsend();
extern int PCD_BATCH;//点云迭代批次
extern int Line_Mode;//测线模式
void xp_ExtraPoints(BinaryPoint *tmp, uint32_t len, int batch=PCD_BATCH);//解析点云数据，L键存储日志

//stand:独立模式，为0时需控制DEVICE_MODEL以启用对应雷达,0:Innovusion雷达,1:Invz雷达,2:RFans
void xp_InnovusionInit(int stand=1);//Innovusion雷达
void xp_InvzInit(int stand=1);//Invz雷达，需调用api
void xp_RFansInit(int stand=1);//RFans雷达
void xp_NeuvInit(int stand,char *ip);//Neuv 雷达//int stand=1
void xp_LidarInit(void);
void saveFile(cv::Mat mat);
//bool GetDirSize(int d);
void G_JudegeLidarStatus();

extern bool LidarSend;
extern bool LidarSave;
extern bool g_GPSDrection4Ladar;

struct Position
{
    float x,y,z, lenth, width,height;
};

struct LadarConf
{
    std::string ip;
    uint32_t handle;
    float height;
    float offset;
    float pitch;
    float roll;
    float yaw;
};

extern LadarConf m_ladarConf;

#endif
