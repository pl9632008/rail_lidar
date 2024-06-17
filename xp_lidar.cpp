#include "xp_lidar.h"
#include "xp_opengl.h"
#include <memory>
#include <string>
#include <ctime>
#include <cstdlib>

#include <vector>
#include <map>

#include <QTextCodec>
#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <thread>//myd
#include <fstream>//myd
#include "Ladar.h"

#ifdef _WINDOWS

#else // !_Win
#include <unistd.h>
#include <sys/time.h>

#endif

using namespace std;

int g_isLidar = 0;
bool g_isFrame = false;
bool G_IndepentTest = false;
bool g_lidarStatus = true;

glm::mat4 INS_OFST(1.0f);
glm::vec3 INS_POSITION;
float INS_MOVEMENT=0;//累加距离
glm::vec3 INS_DIR;//角度-//INS_DIR[0]:平滑后的角度
float INS_SPEED=0;//列车车速（由毫米波雷达处理，即WaveAveSpeed）


//注:采用右手坐标系，右向为X轴正，前向为Y轴正，上向为z轴正

//全局数据
int DEVICE_MODEL=0;//设备型号，0:Innovusion,1:Invz,2:RFans
int DEVICE_STATUS=-1;//设备状态（-1：未知，0：正常）
int SYSTEM_STATUS=-1;//状态（-1：未知，0：正常启动，1：运行中，2：掉电重启）
int PCD_BATCH=1;//点云迭代批次
int Line_Mode=0;//测线模式，会关闭降采样
QTime LIDAR_TIME=QTime::currentTime();
QVector<pointattrs> Global_Rail;
QVector<pointattrs> GPU_RULERS;//标尺
QVector<Edge> Edges;//线界
QVector<pointattrs> LIDAR_TARGETS,LIDAR_TARGETS_last,GPU_TARGETS;//雷达目标
QVector<QVector<pointattrs>> LIDAR_TARGETS_last_all;  //前n帧目标和集
QVector<pointattrs> LIDAR_TARGETS_ROAD,LIDAR_TARGETS_MOVE;//路口目标
QVector<glm::mat4> LIDAR_TARGETS_OFFSET;//目标偏移mat4
QVector<pointattrs> LIDAR_RAILS,GPU_RAILS;//线路点云数据
QVector<pointattrs> LIDAR_PCD,LIDAR_PCL,LIDAR_UDPSEND,LIDAR_PCDTARGET;//点云数据
QVector<int> INTERSECTION;  // 路口信息
QVector<pointattrs> LIDAR_RAILS_INTERSECTION;  // 路口路线
float intersection_with=15.0f;  //路口宽
float intersection_length=10.0f;  //路口长
float intersection_height=2.5f;  //路口高
std::map<int,histogram> scene_histogram;//直方图定位，见xp_track.cpp/xp_pcl.cpp
std::map<int,Anchor> rail_scene[2];//距离，左右标识
//雷达安装信息
float lidar_yaw=0.0f; //偏航角
float lidar_pitch=0.0f; //俯仰角
float lidar_roll=0.0f; //翻滚角
float lidar_offset=0.0f;//左右偏移
float lidar_height=0.0f;//高度

float lidar_yaw_1=0.0f; //偏航角
float lidar_pitch_1=0.0f; //俯仰角
float lidar_roll_1=0.0f; //翻滚角
float lidar_offset_1=0.0f;//左右偏移
float lidar_height_1=0.0f;//高度
float lidar_yaw_2=0.0f; //偏航角
float lidar_pitch_2=0.0f; //俯仰角
float lidar_roll_2=0.0f; //翻滚角
float lidar_offset_2=0.0f;//左右偏移
float lidar_height_2=0.0f;//高度

//界限尺寸
float EDGE_RAIL=1.435f/2;
float EDGE_DANGER=1.3f;
float EDGE_WARNING=0.8f;
float EDGE_HEIGHT=4.9f;

//掉电启动时的配置参数
float REBOOT_MOVEMENT=0,REBOOT_V=0;//位移，速度
int REBOOT_T=0;//时间

cv::Mat cameraMatrix;
cv::Mat disCoeffs;
cv::Mat rtMatrix;
cv::Mat mjpgMat;

//点云过滤信息
float rail_range[]={-3.0f,3.0f,0.2f,3.0f,150.0f};//单侧铁轨左边界，右边界，高MIN边界,高MAX边界,远边界


bool LidarSend = false;
bool LidarSave = false;

bool g_GPSDrection4Ladar = true;

void HSVtoRGB(float *r, float *g, float *b, float h, float s, float v )
{
    int i;
    float f, p, q, t;
    if( fabsf(s)<=0.0001f ) {
        *r = *g = *b = v;
        return;
    }
    if(h>360)
        h-=360;
    h /= 60;            // sector 0 to 5
    i = (int)floor( h );
    f = h - i;          // factorial part of h
    p = v * ( 1 - s );
    q = v * ( 1 - s * f );
    t = v * ( 1 - s * ( 1 - f ) );
    switch( i ) {
    case 0:
        *r = v;
        *g = t;
        *b = p;
        break;
    case 1:
        *r = q;
        *g = v;
        *b = p;
        break;
    case 2:
        *r = p;
        *g = v;
        *b = t;
        break;
    case 3:
        *r = p;
        *g = q;
        *b = v;
        break;
    case 4:
        *r = t;
        *g = p;
        *b = v;
        break;
    default:
        *r = v;
        *g = p;
        *b = q;
        break;
    }
}
#ifdef _WIN32
int gettimeofday(struct timeval * tp, struct timezone * tzp)
{
    static const uint64_t EPOCH = ((uint64_t) 116444736000000000ULL);
    SYSTEMTIME  system_time;
    FILETIME    file_time;
    uint64_t    time;
    GetSystemTime( &system_time );
    SystemTimeToFileTime( &system_time, &file_time );
    time =  ((uint64_t)file_time.dwLowDateTime );
    time += ((uint64_t)file_time.dwHighDateTime) << 32;
    tp->tv_sec  = (long) ((time - EPOCH) / 10000000L);
    tp->tv_usec = (long) (system_time.wMilliseconds * 1000);
    return 0;
}
#endif
void gettime(char *des)//本地化获取时间
{
    struct tm *t;
    struct timeval tv;
    time_t nowtime;
    gettimeofday(&tv, NULL);
    nowtime=tv.tv_sec;
    t =localtime(&nowtime);
    if(des!=NULL)
        sprintf(des,"%4d%02d%02d%02d%02d_%02d%03d",t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec,tv.tv_usec/1000);
}

int  swap_endian(int val)//大小端转换
{
    val = ((val << 8)&0xFF00FF00) | ((val >> 8)&0x00FF00FF);
    return (val << 16)|(val >> 16);
}

uint32_t  swap_endian(uint32_t val)//大小端转换
{
    val = ((val << 8)&0xFF00FF00) | ((val >> 8)&0x00FF00FF);
    return (val << 16)|(val >> 16);
}

uint16_t swap_endian(uint16_t val)
{
    return (val << 8)|(val >> 8);
}

uint32_t StrToHex(char *src)
{
    long dst=0x0;
    char byte[3]={'\0'};
    for(int i=0;i<4;i++)
    {
        memcpy(byte,src+i*2,2);
        dst|=strtol(byte,nullptr,16)<<(3-i)*8;
    }
    return (uint32_t)dst;
}

static uint8_t CharToHex(char *src)
{
    uint8_t dst=0x0;
    char byte[3]={'\0'};
    memcpy(byte,src,2);
    dst|=strtol(byte,nullptr,16);
    return dst;
}

char * HexToStr(char *ptr,int len)//16进制转字符串
{
    char *tmp=new char[len*2+1];
    for(int i=0;i<len;i++)
    {
        sprintf(tmp+i*2,"%02X",(uint8_t)*(ptr+i));
    }
    *(tmp+len*2)='\0';
    return tmp;
}

uint32_t FloatToHex(float src)
{
    union{
        float src;
        uint32_t dst;
    }FloatToHex;
    FloatToHex.src=src;
    return FloatToHex.dst;
}

float HexToFloat(uint32_t src)
{
    union{
        float dst;
        uint32_t src;
    }FloatToHex;
    FloatToHex.src=src;
    return FloatToHex.dst;
}


void xp_ExtraPoints(BinaryPoint *tmp, uint32_t len, int batch)
{
    static int index=0;
    static QVector<pointattrs> *PCD = new QVector<pointattrs>[10]();
    QVector<pointattrs> LIDAR_POINTS;
    QVector<pointattrs> PCD_OUT;
    static int movement=-1;
    char buff[250];
    QMatrix4x4 model;//变换矩阵
    model.translate(QVector3D(lidar_offset,0.0f, lidar_height));//平移
    model.rotate(lidar_pitch, 1.0f, 0.0f, 0.0f);//旋转
    model.rotate(lidar_yaw, 0.0f, 0.0f, 1.0f);
    model.rotate(lidar_roll,0.0f,1.0f,0.0f);
    if(DEVICE_MODEL==1){
        model.rotate(90.0f, 0.0f, 0.0f, 1.0f);
    }else if(DEVICE_MODEL==3){
        model.rotate(90.0f, 0.0f, 0.0f, 1.0f);
    }
    pointattrs point;

    {
        movement=-1;
    }

#ifdef C_CDebug2PCD
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
#endif
    for(auto i=0;i<len;i++)//遍历点
    {


        point.xyz=QVector4D(model*QVector3D(tmp[i].x,tmp[i].y,tmp[i].z));//进行坐标系转换和相对位置偏移
        //HSVtoRGB(&point.rgba[0],&point.rgba[1],&point.rgba[2],(tmp[i].reflectivity/255.0f)*240.0f+0.0f,1,1);//依据反射率得到rgb颜色
        //        std::cout<<"RGB:"<<point.rgba[0]<<" "<<point.rgba[1]<<" "<<point.rgba[2]<<std::endl;
        point.rgba[0]=tmp[i].r/255.0f;
        point.rgba[1]=tmp[i].g/255.0f;
        point.rgba[2]=tmp[i].b/255.0f;
        //std::cout<<point.rgba[0]<<point.rgba[1]<<point.rgba[2]<<std::endl;

#ifdef C_CDebug2PCD
        {
            pcl::PointXYZRGB pcds;
            pcds.x=tmp[i].x;
            pcds.y=tmp[i].y;
            pcds.z=tmp[i].z;
            pcds.r=point.rgba[0]*255;
            pcds.g=point.rgba[1]*255;
            pcds.b=point.rgba[2]*255;
            cloud.push_back(pcds);
        }

#endif

        point.rgba[3]=0.8f;//alpha通道
//        qDebug()<<"x:"<<point.xyz[0]<<"y:"<<point.xyz[1]<<"z:"<<point.xyz[2];
        if(point.xyz[2]>=0 && point.xyz[1]>1)//保存地面以上点,1m之外的数据
        {
            LIDAR_POINTS.push_back(point);//传递给GPU_PCD进行绘图
        }

    }
#ifdef C_CDebug2PCD
    //QString dName = QString("PCD/%1%2%3.pcd").arg(g_cameraFNum, 6, 10, QLatin1Char('0')).arg("_").arg(pcdnum);
    //pcdnum++;
    if (cloud.size())
        G_NeedSavePCL = cloud;
        //pcl::io::savePCDFileBinary(dName.toStdString(),*cloud.makeShared());//save
#endif

    PCD[(index>=(batch-1)?index=0:++index)].swap(LIDAR_POINTS);
    if((LIDAR_PCD.size()==0 || LIDAR_PCL.size()==0)&&LidarSend)//仅当处理线程空闲才赋值，优化性能
    {
        for(auto i=0;i<batch;i++)
        {
            PCD_OUT.append(PCD[i]);
        }
        if(LIDAR_PCD.size()==0)
            LIDAR_PCD=PCD_OUT;
        if(LIDAR_PCL.size()==0 && Key_T==1)
            LIDAR_PCL.swap(PCD_OUT);
        LidarSend=false;
    }

}


Definition RFans32A[32]={
    6.01,-20,0,         -4.068,-19,6.25,        3.377,-18,12.5,         -6.713,-17,18.75,
    6.01,-16,1.5625,    -4.068,-15,7.8125,      3.377,-14,14.0625,      -6.713,-13,20.3125,
    6.01,-12,3.125,     -4.068,-11,9.375,       3.377,-10,15.625,       -6.713,-9,21.875,
    6.01,-8,4.6875,     -4.068,-7,10.9375,      3.377,-6,17.1875,       -6.713,-5,23.4375,
    6.01,-4,25,         -4.068,-3,31.25,        3.377,-2,37.5,          -6.713,-1,43.75,
    6.01,0,26.5625,     -4.068,1,32.8125,       3.377,2,39.0625,        -6.713,3,45.3125,
    6.01,4,28.125,      -4.068,5,34.375,        3.377,6,40.625,         -6.713,7,46.875,
    6.01,8,29.6875,     -4.068,9,35.9375,       3.377,10,42.1875,       -6.713,11,48.4375,
};

float get_timestamp(uint8_t lines,uint16_t time,uint8_t group)
{
    float DT=0;
    if(lines==16)
    {
        return (float)(time+3.125f*32*group+DT);
    }
    else if(lines==32)
        return (float)(time+1.5625f*32*group+DT);
}

#if !defined ARM && !defined __APPLE__
bool xp_InvzConnect(invz::DeviceInterface *invzapi) {
    try {
        invzapi->Connect();
        return true;
    }
    catch (std::exception e) {
        return false;
    }
    return false;
}

bool xp_InvzDisconnect(invz::DeviceInterface *invzapi) {
    try {
        invzapi->Disconnect();
    }
    catch (std::exception e) {
        return false;
    }
    return true;
}

//10.1.1.112，255.255.0.0
void xp_InvzInit(int stand)
{
    if(stand==1)//独立模式
        DEVICE_MODEL=1;//设备号
    while(DEVICE_MODEL==1)
    {
        DEVICE_STATUS=-1;
        invz::DeviceInterface *invzapi = invz::DeviceInit("10.1.1.112");//连接指定Ip
        if(!xp_InvzConnect(invzapi))//连接失败则2s后重试
        {
            invzapi->~DeviceInterface ();
            //            cout<<"Try to connect Invz after 2s..."<<endl;
            Delay(2);
            continue;
        }
        try {
            while(invzapi->IsConnected() && DEVICE_MODEL==1)//连接成功
            {
                auto len = invzapi->GetPixelsPerFrame();//获取Measurement结构体长度
                invz::Measurement *mesurements = new invz::Measurement[len];//申请内存
                invz::FrameMetaData meta;//帧元数据
                invz::vector3 *directions = new invz::vector3[len];//申请vector3向量内存
                invzapi->GetFrame(mesurements, len, meta);//获取一帧数据
                invzapi->GetDirections(directions,len);//获取当前的方向信息
                BinaryPoint *tmp=new BinaryPoint[len];//申请自定义结构体BinaryPoint内存
                for(unsigned long i=0;i<len;i++)//遍历转换至自定义结构体中
                {
                    tmp[i].x=-directions[i].y*mesurements[i].distance/100.0f;
                    tmp[i].y=directions[i].x*mesurements[i].distance/100.0f;
                    tmp[i].z=directions[i].z*mesurements[i].distance/100.0f;
                    tmp[i].reflectivity=mesurements[i].reflectivity;
                }
                xp_ExtraPoints(tmp,len);//调用解析函数
                delete [] tmp;
                tmp = nullptr;
                delete [] mesurements;
                mesurements = nullptr;
                delete [] directions;
                directions = nullptr;
                 LIDAR_TIME=QTime::currentTime();//更新全局时间
            }
        } catch (...) {
            xp_InvzDisconnect(invzapi);//断开连接
        }
        invzapi->~DeviceInterface ();
    }
}
#endif



void xp_LidarInit(void)
{



    while(true)//以非独立模式启动雷达，雷达函数受MODEL_DEVICE控制
    {
        Livox2Init();
    }
}


void trans_model(QMatrix4x4 matrix,pointattrs *point,int size=1)//点集变换
{
    for (int i=0;i<size;i++)
        (point+i)->xyz=matrix*QVector3D((point+i)->xyz[0],(point+i)->xyz[1],(point+i)->xyz[2]);
}
//绘制隧道
void Generate_Edge(QVector<pointattrs> points, QVector<pointattrs> ps_road)
{
    Edges.clear();//清空vector
    LIDAR_RAILS.clear();
//    INTERSECTION.clear();
    if(points.size()==0)
        return;

    //路线添加到全局变量
    LIDAR_RAILS = points;
    pointattrs tmp_p={QVector4D(0,0,0.02,0),QVector4D(0,0,0,0),QVector4D(0,0,0,0)};
    points.prepend(tmp_p);

    Edge edge;//初始化结构体
    pointattrs p={QVector4D(0,0,0.03,0),QVector4D(0,0.2f,0.2f,1.0f),QVector4D(0,0,0,0)};//定义点
    edge.anchor=p;//锚点赋值
    //轨道限界
    p.xyz[0]=EDGE_RAIL;
    edge.rails[0]=p;
    p.xyz[0]=-EDGE_RAIL;
    edge.rails[1]=p;
    //危险限界
    p={QVector4D(EDGE_DANGER,0,0.02,0),QVector4D(0.6f,0.6f,0.4f,1.0f),QVector4D(0,0,0,0)};
    edge.danger[0]=p;
    p.xyz[0]=-EDGE_DANGER;
    edge.danger[1]=p;
    //告警限界
    p={QVector4D(EDGE_WARNING,0,0.02,0),QVector4D(0.5f,0.5f,0.5f,0.3f),QVector4D(0,0,0,0)};
    edge.warning[0]=p;
    for (auto i=0;i<(sizeof(Edge::warning)/sizeof(pointattrs)-2);i++) {
        p.xyz[0]=EDGE_WARNING*cos(M_PI*i/(sizeof(Edge::warning)/sizeof(pointattrs)-3));
        p.xyz[2]=EDGE_HEIGHT+EDGE_WARNING*abs(sin(M_PI*i/(sizeof(Edge::warning)/sizeof(pointattrs)-3)));
        edge.warning[i+1]=p;
    }
    p.xyz[2]=0;
    edge.warning[sizeof(Edge::warning)/sizeof(pointattrs)-1]=p;
    Edges.push_back(edge);//添加默认edge至vector中
    //开始索引vector，计算角度，偏移模型
    for(int i=1;i<points.size();i++)
    {
        //std::cout<<"has point:"<<points.size()<<std::endl;
        QMatrix4x4 matrix;
        Edge edge_target=edge;
        QVector2D OY=QVector2D(1.0f,0.0f);//（1，0）单位向量
        QVector2D OB=QVector2D(points[i].xyz[0]-points[i-1].xyz[0],points[i].xyz[1]-points[i-1].xyz[1]);//OB为两锚点相减
        float YOB=OB.length()?acos((OY.x()*OB.x()+OY.y()*OB.y())/(OY.length()*OB.length()))/M_PI*180:0;//得到向量夹角
        YOB=YOB-90;//与模型法向量夹角90°
        matrix.translate(points[i].xyz[0],points[i].xyz[1],points[i].xyz[2]);
        matrix.rotate(YOB,0,0,1);
        trans_model(matrix,(pointattrs *)(&edge_target),sizeof(Edge)/sizeof(pointattrs));//转换点集
        Edges.push_back(edge_target);//得到转换后的模型，添加至vector，待绘图线程调用
    }
}

void Generate_Targets(QVector<pointattrs> points,QVector<glm::mat4> offsets)
{//points x,y,z,h,l,w,number*1000+angle,speed
    QVector<pointattrs> targets;
    QMatrix4x4 model;
    pointattrs p={QVector4D(0,0,0,0),QVector4D(0,0,0,0.8f),QVector4D(0,0,0,0)};
    //索引长方体顶点
    if(points.size()>0){
        for (int i=0;i<points.size();i++) {
            //            if(points[i].rgba[2]/1000<3 && Line_Mode==0)
            //                continue;
            //            if(points[i].rgba[0]<0.5f && Line_Mode==1)
            //                continue;
            //        if(points[i].xyz[3]<2)
            //        qDebug()<<points[i].xyz<<points[i].rgba;
            if(offsets.size()==points.size())
            {
                for (int j=0;j<16;j++)
                    model.data()[j]=glm::value_ptr(offsets[i])[j];
            }
            HSVtoRGB(&p.rgba[0],&p.rgba[1],&p.rgba[2],points[i].rgba[3],1,1);
            p.xyz=model*QVector3D(points[i].xyz[0]-points[i].rgba[0]/2,points[i].xyz[1]-points[i].rgba[1]/2,points[i].xyz[2]);
            targets.push_back(p);
            p.xyz=model*QVector3D(points[i].xyz[0]+points[i].rgba[0]/2,points[i].xyz[1]-points[i].rgba[1]/2,points[i].xyz[2]);
            targets.push_back(p);
            p.xyz=model*QVector3D(points[i].xyz[0]-points[i].rgba[0]/2,points[i].xyz[1]+points[i].rgba[1]/2,points[i].xyz[2]);
            targets.push_back(p);
            p.xyz=model*QVector3D(points[i].xyz[0]+points[i].rgba[0]/2,points[i].xyz[1]+points[i].rgba[1]/2,points[i].xyz[2]);
            targets.push_back(p);
            p.xyz=model*QVector3D(points[i].xyz[0]-points[i].rgba[0]/2,points[i].xyz[1]-points[i].rgba[1]/2,points[i].xyz[2]+points[i].xyz[3]);
            targets.push_back(p);
            p.xyz=model*QVector3D(points[i].xyz[0]+points[i].rgba[0]/2,points[i].xyz[1]-points[i].rgba[1]/2,points[i].xyz[2]+points[i].xyz[3]);
            targets.push_back(p);
            p.xyz=model*QVector3D(points[i].xyz[0]-points[i].rgba[0]/2,points[i].xyz[1]+points[i].rgba[1]/2,points[i].xyz[2]+points[i].xyz[3]);
            targets.push_back(p);
            p.xyz=model*QVector3D(points[i].xyz[0]+points[i].rgba[0]/2,points[i].xyz[1]+points[i].rgba[1]/2,points[i].xyz[2]+points[i].xyz[3]);
            p.xyz[3]=points[i].xyz[2];
            targets.push_back(p);
        }
    }
    //    if(points.size()==0)
    //        GPU_TARGETS.swap(targets);//实时模式，刷新
    //    else
    //        GPU_TARGETS.append(targets);//追加模式
    if(targets.size()!=0)
        GPU_TARGETS.swap(targets);//实时模式，刷新
    else
        GPU_TARGETS.append(targets);//追加模式
}

QVector<pointattrs> Genrate_DirVec()//生成方向向量
{
    QVector<pointattrs> points;
    QMatrix4x4 model;
    model.translate(QVector3D(lidar_offset,0.0f, lidar_height));
    model.rotate(lidar_yaw, 0.0f, 0.0f, 1.0f);
    model.rotate(lidar_pitch, 1.0f, 0.0f, 0.0f);
    model.rotate(lidar_roll,0.0f,1.0f,0.0f);
    pointattrs p={QVector4D(0,0,0,0),QVector4D(1,0,0,0.5f),QVector4D(0,0,0,0)};
    p.xyz=model*QVector3D(p.xyz);
    points.push_back(p);
    p={QVector4D(0.5f,0,0,0),QVector4D(1,0,0,0.5f),QVector4D(0,0,0,0)};
    p.xyz=model*QVector3D(p.xyz);
    points.push_back(p);
    p={QVector4D(0,0,0,0),QVector4D(0,1,0,0.5f),QVector4D(0,0,0,0)};
    p.xyz=model*QVector3D(p.xyz);
    points.push_back(p);
    p={QVector4D(0,0.5f,0,0),QVector4D(0,1,0,0.5f),QVector4D(0,0,0,0)};
    p.xyz=model*QVector3D(p.xyz);
    points.push_back(p);
    p={QVector4D(0,0,0,0),QVector4D(0,0,1,0.5f),QVector4D(0,0,0,0)};
    p.xyz=model*QVector3D(p.xyz);
    points.push_back(p);
    p={QVector4D(0,0,0.5f,0),QVector4D(0,0,1,0.5f),QVector4D(0,0,0,0)};
    p.xyz=model*QVector3D(p.xyz);
    points.push_back(p);
    return points;
}

void Genrate_Staff(int num,int step,float height)//生成标尺和坐标轴，线数，步进，相对于原点o-z高度
{
    QVector<pointattrs> points;
    pointattrs p={QVector4D(0,0,-height,0),QVector4D(0.2f,0.2f,0.2f,0.5f),QVector4D(0,0,0,0)};//标尺
    for(int i=0;i<=num*2;i++)
    {
        p.xyz[0]=(i-num)*step;
        p.xyz[1]=num*step;
        points.push_back(p);
        p.xyz[1]=-p.xyz[1];
        points.push_back(p);
    }
    for(int i=0;i<=num*2;i++)
    {
        p.xyz[1]=(i-num)*step;
        p.xyz[0]=num*step;
        points.push_back(p);
        p.xyz[0]=-p.xyz[0];
        points.push_back(p);
    }
    //地平面填充
    p={QVector4D(0,0,-height,0),QVector4D(0.3f,0.3f,0.6f,0.2f),QVector4D(0,0,0,0)};
    p.xyz[0]=num*step;
    p.xyz[1]=-num*step;
    points.push_back(p);
    p={QVector4D(0,0,-height,0),QVector4D(0.3f,0.3f,0.6f,0.2f),QVector4D(0,0,0,0)};
    p.xyz[0]=num*step;
    p.xyz[1]=num*step;
    points.push_back(p);
    p={QVector4D(0,0,-height,0),QVector4D(0.3f,0.3f,0.6f,0.2f),QVector4D(0,0,0,0)};
    p.xyz[0]=-num*step;
    p.xyz[1]=-num*step;
    points.push_back(p);
    p={QVector4D(0,0,-height,0),QVector4D(0.3f,0.3f,0.6f,0.2f),QVector4D(0,0,0,0)};
    p.xyz[0]=-num*step;
    p.xyz[1]=num*step;
    points.push_back(p);
    GPU_RULERS.swap(points);
}


// NEUV雷达入口
enum NEU_LASER_RATE {NEU_200KHZ, NEU_300KHZ, NEU_400KHZ, NEU_500KHZ, NEU_750KHZ, NEU_1P5MHZ};
const int CONNECT_STATE_CONNECTING = 0;
const int CONNECT_STATE_CONNECTED = 1;
const int CONNECT_STATE_DISCONNECTED = -1;

 void showretval(int ret) { if(ret == 0) return; std::cout<<"ret:"<<ret<<std::endl;}


#define UNUSED(var) (void)(var)
const uint32_t coloredge[6] = {0xffff00, 0xff0000, 0xff00ff, 0x0000ff, 0x00ffff, 0x00ff00};
int custom_ffs(uint32_t x)
{
    if (x == 0)
    {
        return 0;
    }
    int num = 1;
    if ((x & 0xffff) == 0)
    {
        num += 16;
        x >>= 16;
    }
    if ((x & 0xff) == 0)
    {
        num += 8;
        x >>= 8;
    }
    if ((x & 0xf) == 0)
    {
        num += 4;
        x >>= 4;
    }
    if ((x & 0x3) == 0)
    {
        num += 2;
        x >>= 2;
    }
    if ((x & 0x1) == 0)
    {
        num += 1;
    }
    return num;
}

