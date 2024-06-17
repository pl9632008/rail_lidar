#include "xp_pcl.h"

#include <vtkAutoInit.h>
#include <vtkRenderWindow.h>
#include <vtkOutputWindow.h>

#include <chrono>

#include "xp_lidar.h"
#include "xp_opengl.h"
#include <map>
#include <iostream>

#include "filter_ground.h"

#ifdef _WIN32
VTK_MODULE_INIT(vtkRenderingOpenGL2);
#endif
VTK_MODULE_INIT(vtkInteractionStyle);

xp_pcl *xp_Pcl;

#define XP_LOCATION_MODE 0//定位方法
int POINTS_MAX_LENGTH = 150;
//点云线界
float PCXmin=-10.0f;
float PCXmax=10.0f;
float PCYmin=1.0f;
float PCYmax=200.0f;
float PCZmin=0.0f;
float PCZmax=2.0f;
xp_pcl::xp_pcl()
{
    //std::thread t(&xp_pcl::run,this);
    std::thread t(&xp_pcl::new_run,this);
    pthread_setname_np(t.native_handle(), "xp_pcl new_run");
    t.detach();
}
xp_pcl::~xp_pcl()
{

}

/**
 * @brief 点云处理（刘 new）
 *
 */
void xp_pcl::new_run()
{
    //    char buff[1024];
    //    clock_t time_start;//时间戳
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();//创建变换矩阵

    bool attitude_flag = false;
    std::vector<float> cus_ans;

    while(1)
    {
        auto start = std::chrono::steady_clock::now();
        auto end = std::chrono::steady_clock::now();
        //        continue;
        //如果不是下行的开始和过桥洞那段，
        QVector<pointattrs> temp_LIDAR_PCL;
        temp_LIDAR_PCL.swap(LIDAR_PCL);
        if (temp_LIDAR_PCL.size()>0 && Key_T==1)
        {
            //计算循环一次消耗时间，并刷新当前帧的进入时间

            pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>);//初始点云
            mycloud->width = temp_LIDAR_PCL.size();
            mycloud->height = 1;
            mycloud->is_dense = false;
            mycloud->points.resize(mycloud->width * mycloud->height);//定义尺寸
            for ( int i = 0; i < temp_LIDAR_PCL.size(); ++i)//遍历赋值//; i < LIDAR_PCL.size()-1;
            {
                mycloud->points[i].x = temp_LIDAR_PCL[i].xyz[0];
                mycloud->points[i].y = temp_LIDAR_PCL[i].xyz[1];
                mycloud->points[i].z = temp_LIDAR_PCL[i].xyz[2];
            }
            temp_LIDAR_PCL.clear();//清空输入数据
            

            if(!attitude_flag){
                cus_ans = getAttitude(mycloud);// height x y z
                attitude_flag = true;
                if(cus_ans[0] != -1){
                    lidar_roll= cus_ans[2];
                    lidar_pitch = cus_ans[1];
                    lidar_yaw = cus_ans[3];
                }
            }
            if(ans[0] != -1){
                mycloud = filterGround(mycloud, ans[0]);
            }

            
            //旋转平移
            glm::mat4 model(1.0f);//定义四阶矩阵
            model*=INS_OFST;//矩阵偏移
            for(int i=0;i<4;i++)
            {
                for(int j=0;j<4;j++)
                {
                    transform(j,i)=glm::value_ptr(model)[i*4+j];//glm::mat4转换至Eigen::Matrix4f
                }
            }
            pcl::transformPointCloud (*mycloud, *mycloud, transform);//偏移点云，修正偏移量

            end = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::micro> elapsed = end-start;
//            std::cout <<"the runtime of new_run font process: "<< (double)elapsed.count() / 1000000 << "s" <<std::endl;
            start = std::chrono::steady_clock::now();

            pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

            //根据线界分割点云
            //std::cout<<"LIDAR_RAILS.size():"<<LIDAR_RAILS.size()<<std::endl;
            QVector<pointattrs> tmp_lidar_rails(LIDAR_RAILS);
            if(tmp_lidar_rails.size()>1){
                //qDebug()<<"lidar_rails is not empty!";
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cut (new pcl::PointCloud<pcl::PointXYZ>());
                float edge_width;
                float edge_bottom = 0.15;
                if(G_IndepentTest){
                    edge_width = EDGE_DANGER;
                }
                else
                {
//                    edge_width = EDGE_RAIL+0.3;
                    edge_width = EDGE_DANGER;
                }
                for (int i=0;(i<tmp_lidar_rails.size()-1)&&(tmp_lidar_rails[i].xyz[1]<POINTS_MAX_LENGTH);++i){//for (int i=0;tmp_lidar_rails[i].xyz[1]<POINTS_MAX_LENGTH;++i){
                    float dat_y = abs(tmp_lidar_rails[i].xyz[1] - tmp_lidar_rails[i+1].xyz[1]);  // 和下一点之间的y的距离
                    pcl::CropBox<pcl::PointXYZ> boxFilter;
                    if(tmp_lidar_rails[i].rgba[0]==4){  //如果是左侧站台
                        boxFilter.setMin(Eigen::Vector4f(tmp_lidar_rails[i].xyz[0]-0.5, tmp_lidar_rails[i].xyz[1], edge_bottom, 1.0));
//                        qDebug()<<"左侧站台"<<LIDAR_RAILS[i].xyz[0]<<","<<LIDAR_RAILS[i].xyz[1];
                    }
                    else {
                        //std::cout<<"left: ("<<LIDAR_RAILS[i].xyz[0]-EDGE_WARNING<<", "<<LIDAR_RAILS[i].xyz[1]<<", "<<0<<", "<<1.0<<")"<<std::endl;
                        boxFilter.setMin(Eigen::Vector4f(tmp_lidar_rails[i].xyz[0]-edge_width, tmp_lidar_rails[i].xyz[1], edge_bottom, 1.0));
//                        boxFilter.setMin(Eigen::Vector4f(LIDAR_RAILS[i].xyz[0]-2, LIDAR_RAILS[i].xyz[1], 0, 1.0));
//                        qDebug()<<"no左侧站台"<<LIDAR_RAILS[i].xyz[0]<<","<<LIDAR_RAILS[i].xyz[1];
                    }
                    if(tmp_lidar_rails[i].rgba[0]==5){  //如果是右侧站台
                        boxFilter.setMax(Eigen::Vector4f(tmp_lidar_rails[i].xyz[0]+0.5, tmp_lidar_rails[i].xyz[1]+dat_y, EDGE_HEIGHT, 1.0));
//                        qDebug()<<"右侧站台"<<LIDAR_RAILS[i].xyz[0]<<","<<LIDAR_RAILS[i].xyz[1];
                    }
                    else {
                        //std::cout<<"right:("<<LIDAR_RAILS[i].xyz[0]+EDGE_WARNING<<", "<<LIDAR_RAILS[i].xyz[1]+dat_y<<", "<<EDGE_HEIGHT<<", "<<1.0<<")"<<std::endl;
                        boxFilter.setMax(Eigen::Vector4f(tmp_lidar_rails[i].xyz[0]+edge_width, tmp_lidar_rails[i].xyz[1]+dat_y, EDGE_HEIGHT, 1.0));
//                        boxFilter.setMax(Eigen::Vector4f(LIDAR_RAILS[i].xyz[0]+2, LIDAR_RAILS[i].xyz[1]+dat_y, EDGE_HEIGHT, 1.0));
//                        qDebug()<<"no右侧站台"<<LIDAR_RAILS[i].xyz[0]<<","<<LIDAR_RAILS[i].xyz[1];
                    }
                    boxFilter.setInputCloud(mycloud);
                    boxFilter.setNegative(false);//myd
                    boxFilter.filter(*cloud_cut);
                    *no_ground_cloud_ptr += *cloud_cut;
                }
            }
            else{
                //qDebug()<<"Lidar_rails is empty!";
                LIDAR_TARGETS.clear();
                //                LIDAR_TARGETS_MOVE.clear();
                LIDAR_TARGETS_ROAD.clear();
            }
            if(no_ground_cloud_ptr->size()==0){
                LIDAR_TARGETS_MOVE.clear();
                Delayms(2);
                continue;
            }


            //qDebug()<<"mycloud size: " << mycloud->size() << "no ground cloud size: "<< no_ground_cloud_ptr->size();

            //聚类
            QVector<pointattrs> points;//目标集合
            points.clear();
            std::vector<Detected_Obj> targetList;//聚类舞台集合
            std::vector<std::vector<Detected_Obj>> Allkinds;
            targetList.clear();
            Allkinds.clear();


            pcl::PointCloud<pcl::PointXYZ> rail_cloud = getRail(no_ground_cloud_ptr);
            if(!rail_cloud->empty()){
                no_ground_cloud_ptr = clusterOnRail(rail_cloud, no_ground_cloud_ptr, 0);
            }

            
            cluster(no_ground_cloud_ptr,targetList,Allkinds);//聚类


            //cluster(mycloud, targetList, Allkinds);

            end = std::chrono::steady_clock::now();
            elapsed = end-start;
            //std::cout <<"the runtime of new_run font cluster: "<< (double)elapsed.count() / 1000000 << "s" <<std::endl;//need
            start = std::chrono::steady_clock::now();

#ifdef GRID_CLUSTER
            for (size_t i=0;i<Allkinds.size();i++)
            {
                for (size_t j=0;i<Allkinds[i].size();j++)
                {
#ifdef XP_SHOWVTK
                    viewer->addCube(Allkinds[i][j].min_point_.x, Allkinds[i][j].max_point_.x, Allkinds[i][j].min_point_.y,Allkinds[i][j].max_point_.y, Allkinds[i][j].min_point_.z,Allkinds[i][j].max_point_.z,1.0, 1.0, 0.0);
#endif
                }
            }
#else
            for (size_t i=0;i<targetList.size();i++)
            {
                pointattrs point={QVector4D((targetList[i].max_point_.x+targetList[i].min_point_.x)/2,(targetList[i].max_point_.y+targetList[i].min_point_.y)/2, \
                                  std::min(targetList[i].max_point_.z,targetList[i].min_point_.z),abs(targetList[i].max_point_.z-targetList[i].min_point_.z)),\
                                  QVector4D(abs(targetList[i].min_point_.x-targetList[i].max_point_.x),abs(targetList[i].min_point_.y-targetList[i].max_point_.y),0.0f,0.0f),\
                                  QVector4D(0.0f,0.0f,0.0f,0.0f)};//定义目标参数
                points.push_back(point);
                //viewer->addCube (targetList[i].min_point_.x, targetList[i].max_point_.x, targetList[i].min_point_.y,targetList[i].max_point_.y, targetList[i].min_point_.z,targetList[i].max_point_.z,1.0, 1.0, 0.0);//添加包络长方体
            }
#endif


            QVector<pointattrs> TARGETS_BACK;//保存上帧的目标集合

           TARGETS_BACK=points;

           LIDAR_TARGETS.swap(TARGETS_BACK);

            LIDAR_TARGETS_OFFSET=QVector<glm::mat4>();



            //删除静态物体
            QVector<pointattrs> LIDAR_TARGETS_linshi;
            LIDAR_TARGETS_linshi = LIDAR_TARGETS;
            {
                LIDAR_TARGETS_MOVE = LIDAR_TARGETS_linshi;
            }

            LIDAR_TARGETS.clear();
            LIDAR_TARGETS_ROAD.clear();

            clock_t end = clock();
        }

#ifdef XP_SHOWVTK
        viewer->spinOnce(30);//延迟30ms
#else
        Delayms(2);
#endif
        end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::micro> elapsed = end-start;
    }
}


void xp_pcl::m_MLogic()
{

}

//聚类入口函数
/**
 * @brief 聚类入口函数
 *
 * @param current_pc_ptr 输入点云
 * @param global_obj_list 聚类后的目标列表
 * @param Allkinds
 */
void xp_pcl::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr,std::vector<Detected_Obj> &global_obj_list, std::vector<std::vector<Detected_Obj>> &Allkinds)
{

    seg_distance_ = {20, 30, 40, 60, 80, 100, 150};
    cluster_distance_ = {0.1, 0.2, 0.3, 0.6, 1.0, 1.6, 2.5};
//    cluster_distance_ = {0.05, 0.2, 0.3, 0.6, 1.0, 1.6, 2.5};
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    //降采样
    voxel_grid_filer(current_pc_ptr, filtered_pc_ptr, LEAF_SIZE);

    cluster_by_distance(filtered_pc_ptr, global_obj_list, Allkinds);
}

// 根据不同的距离进行聚类
/**
 * @brief 根据不同的距离进行聚类
 *
 * @param in_pc 输入点云
 * @param obj_list 聚类出的类别
 * @param Allkinds
 */
void xp_pcl::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list,std::vector<std::vector<Detected_Obj>> &Allkinds)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(7);
    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[i] = tmp;
    }
    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        // 如果点的距离大于150m, 忽略该点
        if (origin_distance >= 150)
        {
            continue;
        }
        //将点云按照距离进行分割
        if (origin_distance < seg_distance_[0])
        {
            segment_pc_array[0]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[1])
        {
            segment_pc_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2])
        {
            segment_pc_array[2]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[3])
        {
            segment_pc_array[3]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[4])
        {
            segment_pc_array[4]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[5])
        {
            segment_pc_array[5]->points.push_back(current_point);
        }
        else
        {
            segment_pc_array[6]->points.push_back(current_point);
        }
    }
    std::vector<pcl::PointIndices> final_indices;
    std::vector<pcl::PointIndices> tmp_indices;

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        //对不同距离的点云进行聚类
        cluster_segment(segment_pc_array[i], cluster_distance_[i], obj_list, Allkinds);
    }

}

// 点云分割函数
/**
 * @brief 点云聚类函数
 *
 * @param in_pc 输入点云
 * @param in_max_cluster_distance 最大聚类阈值
 * @param obj_list 聚类后的目标
 * @param Allkinds
 */
void xp_pcl::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                                    double in_max_cluster_distance, std::vector<Detected_Obj> &obj_list,std::vector<std::vector<Detected_Obj>> &Allkinds)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //创建2D点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    // 将点云压缩到地面
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }
    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> local_indices;
    //欧式聚类
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(cloud_2d);
    // 设置聚类距离
    euclid.setClusterTolerance(in_max_cluster_distance);
    if(G_IndepentTest)
    {
        //设置最小聚类点数
        euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    }
    else
    {
        //设置最小聚类点数
        euclid.setMinClusterSize(10);
    }
    //设置最大聚类点数
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);



    for (size_t i=0; i<local_indices.size();i++)
    {
        Detected_Obj obj_info;

#ifdef GRID_CLUSTER
        float leafSize=0.5f;
        //将每类中的点云降采样到0.5*0.5*0.5的小方格
        pcl::PointCloud<pcl::PointXYZ>::Ptr VoxOut(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr FilterOut(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extractPoint;
        extractPoint.setInputCloud(in_pc);
        extractPoint.setIndices(boost::make_shared<pcl::PointIndices>(local_indices[i]));
        extractPoint.setNegative(false);
        extractPoint.filter(*VoxOut);
        voxel_grid_filer(VoxOut,FilterOut,leafSize);
        //每类中存储的网格
        std::vector<Detected_Obj> oneKind;
        for(size_t i=0;i<FilterOut->points.size();i++)
        {
            pcl::PointXYZ MaxAABB;
            pcl::PointXYZ MinAABB;
            MinAABB.x = FilterOut->points[i].x-leafSize/2;
            MinAABB.y = FilterOut->points[i].y-leafSize/2;
            MinAABB.z = FilterOut->points[i].z-leafSize/2;
            MaxAABB.x = FilterOut->points[i].x+leafSize/2;
            MaxAABB.y = FilterOut->points[i].y+leafSize/2;
            MaxAABB.z = FilterOut->points[i].z+leafSize/2;
            obj_info.min_point_=MinAABB;
            obj_info.max_point_=MaxAABB;
            oneKind.push_back(obj_info);
        }
        Allkinds.push_back(oneKind);
#else
        //计算聚类点云的外包框

        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            //fill new colored cluster point by point
            pcl::PointXYZ p;
            p.x = in_pc->points[*pit].x;
            p.y = in_pc->points[*pit].y;
            p.z = in_pc->points[*pit].z;

            obj_info.centroid_.x += p.x;
            obj_info.centroid_.y += p.y;
            obj_info.centroid_.z += p.z;

            if (p.x < min_x)
                min_x = p.x;
            if (p.y < min_y)
                min_y = p.y;
            if (p.z < min_z)
                min_z = p.z;
            if (p.x > max_x)
                max_x = p.x;
            if (p.y > max_y)
                max_y = p.y;
            if (p.z > max_z)
                max_z = p.z;
        }
        // qopengl界面 坐标系 坐标轴
        //    z
        //    ↑  x
        // y ←o↗
        //过滤杨絮、浮尘、雨雪、虚点
        if(min_x<20){//20米内
            if((max_y-min_y)<0.15&&(max_z-min_z)<0.15){//过滤杨絮、浮尘、雨雪等小物体
                continue;
            }
}
        //min, max points
        obj_info.min_point_.x = min_x;
        obj_info.min_point_.y = min_y;
        obj_info.min_point_.z = min_z;

        obj_info.max_point_.x = max_x;
        obj_info.max_point_.y = max_y;
        obj_info.max_point_.z = max_z;

        //calculate centroid, average
        if (local_indices[i].indices.size() > 0)
        {
            obj_info.centroid_.x /= local_indices[i].indices.size();
            obj_info.centroid_.y /= local_indices[i].indices.size();
            obj_info.centroid_.z /= local_indices[i].indices.size();
        }


        obj_list.push_back(obj_info);
#endif
    }
}

// 滤波函数
/**
 * @brief 滤波函数
 *
 * @param in 输入点云
 * @param out 输出点云
 * @param leaf_size 体素滤波大小
 */
void xp_pcl::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)
{
    //滤波
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_near(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_far(new pcl::PointCloud<pcl::PointXYZ>);
    //切分近处点云
    pass.setInputCloud (in);
    pass.setFilterFieldName ("y");//y轴，正前方为正
    pass.setFilterLimits (0, 30);//0-50m
    pass.filter (*cloud_near);//过滤结果存储至cloud_near
    //切分远处点云
    pass.setInputCloud (in);
    pass.setFilterFieldName ("y");//y轴，正前方为正
    pass.setFilterLimits (30, 200);//50-1000m
    pass.filter (*cloud_far);//过滤结果存储至cloud_far
    //体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud_near);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);

    *out += *cloud_far;
}


//提取地面函数
/**
 * @brief 提取地面函数
 *
 * @param no_ground_cloud_ptr 分割地面后的点云
 * @param current_pc_ptr 输入点云
 */
void xp_pcl::ExtractNoGround(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr)
{
    //pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointXYZ Up(1,60,1.5);
    //pcl::PointXYZ Down(-5,1,-1);
    //设置分割点云范围
    pcl::PointXYZ Up(PCXmax,PCYmax,PCZmax);
    pcl::PointXYZ Down(PCXmin,PCYmin,PCZmin);

    //根据范围分割点云
    filterByXYZ(Up,Down,current_pc_ptr,cliped_pc_ptr);

//     for (int i=0;i<cliped_pc_ptr->points.size();i++)
//     {
//         cout<<"处理后x"<<cliped_pc_ptr->points[i].x<<"y"<<cliped_pc_ptr->points[i].y<<"z"<<cliped_pc_ptr->points[i].z<<endl;
//     }

    PointCloudXYZRTColor origanized_points;
    std::vector<pcl::PointIndices> radial_division_indices;
    std::vector<pcl::PointIndices> closest_indices;
    std::vector<PointCloudXYZRTColor> radial_ordered_clouds;

    //RAY_ground地面分割算法
    radial_dividers_num_=ceil(360/RADIAL_DIVIDER_ANGLE);
    XYZ_to_RTZColor(cliped_pc_ptr,origanized_points, radial_division_indices,radial_ordered_clouds);

    pcl::PointIndices ground_indices, no_ground_indices;

    classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ExtractIndices<pcl::PointXYZ> extract_ground;
    extract_ground.setInputCloud(cliped_pc_ptr);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

    extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
    extract_ground.filter(*ground_cloud_ptr);

    extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
    extract_ground.filter(*no_ground_cloud_ptr);

}

void xp_pcl::classify_pc(std::vector<PointCloudXYZRTColor> &in_radial_ordered_clouds, pcl::PointIndices &out_ground_indices, pcl::PointIndices &out_no_ground_indices)
{
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();
#pragma omp parallel for
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
    {
        float prev_radius = 0.f;
        float prev_height = -SENSOR_HEIGHT;
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j<in_radial_ordered_clouds[i].size(); j++)
        {
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
            float height_threshold = tan(DEG2RAD(local_max_slope_))*points_distance;
            float current_height = in_radial_ordered_clouds[i][j].point.z;
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;
            if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
            {
                height_threshold = min_height_threshold_;
            }
            //check current point height against the LOCAL threshold (previous point)
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                //Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground)
                {
                    if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else
                {
                    current_ground = true;
                }
            }
            else
            {
                //check if previous point is too far from previous one, if so classify again
                if (points_distance > reclass_distance_threshold_ &&
                        (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold)))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }

            if (current_ground)
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }

            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}

void xp_pcl::XYZ_to_RTZColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, PointCloudXYZRTColor &out_origanized_points, std::vector<pcl::PointIndices> &out_radial_divided_indices, std::vector<PointCloudXYZRTColor> &out_radial_ordered_clouds)
{
    out_origanized_points.resize(in_cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    for (size_t i = 0; i<in_cloud->points.size();i++)
    {
        PointXYZRTColor new_point;
        auto radius = (float)sqrt(in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180/M_PI;
        if (theta < 0)
        {
            theta += 360;
        }
        //角度的微分
        auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);
        //半径的微分
        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i;

        out_origanized_points[i] = new_point;

        out_radial_divided_indices[radial_div].indices.push_back(i);
        out_radial_ordered_clouds[radial_div].push_back(new_point);

    }

    //将同一根射线上的点按照半径(距离)排序
#pragma omp parallel for
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(),out_radial_ordered_clouds[i].end(),
                  [](const PointXYZRTColor &a, const PointXYZRTColor &b){ return a.radius < b.radius;});
    }
}

/**
 * @brief 直通滤波
 *
 * @param Up 分割上限
 * @param Down 分割下限
 * @param in 输入点云
 * @param out 输出点云
 */
void xp_pcl::filterByXYZ(pcl::PointXYZ Up, pcl::PointXYZ Down,const pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZ> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
//    cout<<"up:"<<Up.x<<" "<<Up.y<<" "<<Up.z<<endl;
//    cout<<"Down:"<<Down.x<<" "<<Down.y<<" "<<Down.z<<endl;
#pragma omp parallel for
    for (size_t i=0; i < in->points.size(); i++)
    {
        if ((in->points[i].x>Down.x)&&(in->points[i].x<Up.x)&&(in->points[i].y>Down.y)&&(in->points[i].y<Up.y)&&(in->points[i].z>Down.z)&&(in->points[i].z<Up.z))
        //if((in->points[i].x>-1)&&(in->points[i].x<1)&&(in->points[i].y>0)&&(in->points[i].y<1)&&(in->points[i].z>0)&&(in->points[i].z<1))
        {
            indices.indices.push_back(i);
            //cout<<"x:"<<in->points[i].x<<"y:"<<in->points[i].y<<"z:"<<in->points[i].z<<endl;
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(false);
    cliper.filter(*out);

}


