#ifndef XIPENG_PCL
#define XIPENG_PCL


#include <thread>
#include <QDebug>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "xp_lidar.h"
#include <ctime>
#include <time.h>
//pcl
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/common/transforms.h>
//模型系数
#include <pcl/ModelCoefficients.h>
//样本共识
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
//分割
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//过滤器
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
#include <pcl/filters/crop_box.h>
//定位
#include <pcl/registration/icp.h>
//边界提取
#include <pcl/features/moment_of_inertia_estimation.h>

#define CLIP_HEIGHT 5 //截取掉高于雷达自身0.2米的点
#define MIN_DISTANCE 1
#define RADIAL_DIVIDER_ANGLE 0.01
#define SENSOR_HEIGHT -0.15

#define concentric_divider_distance_ 0.01 //0.1 meters default
#define min_height_threshold_ 0.05
#define local_max_slope_ 8  //max slope of the ground between points, degree
#define general_max_slope_ 5 //max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.2

#define LEAF_SIZE 0.1 //定义降采样的leaf size，聚类是一个费时运算，为了减少计算量，我们通常先进行降采样
#define MIN_CLUSTER_SIZE 2 //聚类最小阈值
#define MAX_CLUSTER_SIZE 600 //聚类最大阈值

class xp_pcl
{
public:
    xp_pcl();
    ~xp_pcl();
protected:
    void run();
    void new_run();
private:
    struct PointXYZRTColor
        {
            pcl::PointXYZ point;

            float radius;
            float theta;

            size_t radial_div;
            size_t concentric_div;

            size_t original_index;
        };
        typedef std::vector<PointXYZRTColor> PointCloudXYZRTColor;

        struct Detected_Obj
        {

          pcl::PointXYZ min_point_;
          pcl::PointXYZ max_point_;
          pcl::PointXYZ centroid_;
        };

        size_t radial_dividers_num_;
        size_t concentric_dividers_num_;

    std::map<int,Anchor> current_scene[2];//当前场景
    void m_MLogic();

    void ExtractNoGround(pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr);//提取地面
    void filterByXYZ(pcl::PointXYZ Up,pcl::PointXYZ Down,const pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out);
    void XYZ_to_RTZColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,PointCloudXYZRTColor &out_origanized_points, std::vector<pcl::PointIndices> &out_radial_divided_indices, std::vector<PointCloudXYZRTColor> &out_radial_ordered_clouds);
    void classify_pc(std::vector<PointCloudXYZRTColor> &in_radial_ordered_clouds, pcl::PointIndices &out_ground_indices,pcl::PointIndices &out_no_ground_indices);

    std::vector<double> seg_distance_, cluster_distance_;

    void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr,std::vector<Detected_Obj> &global_obj_list,std::vector<std::vector<Detected_Obj>> &Allkinds);
    //降采样
    void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size);
    void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list,std::vector<std::vector<Detected_Obj>> &Allkinds);
    void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,double in_max_cluster_distance, std::vector<Detected_Obj> &obj_list, std::vector<std::vector<Detected_Obj>> &Allkinds);
};

extern xp_pcl *xp_Pcl;

#endif // XIPENG_PCL
