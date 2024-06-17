

#include "ground_segmentation.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <random>

pcl::PointCloud<pcl::PointXYZ>::Ptr tttest (new pcl::PointCloud<pcl::PointXYZ>());
float height = 0;
float slope = 0;

void sampleConsensus2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1.2, 0);  
    pass.filter(*left_cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(0, 1.2);  
    pass.filter(*right_cloud);

    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/left.pcd", *left_cloud);
    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/right.pcd", *right_cloud);

    //pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_left (new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(left_cloud));


    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_left (new pcl::SampleConsensusModelLine<pcl::PointXYZ>(left_cloud));
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_right(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(right_cloud));

  
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_left (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_right (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_total (new pcl::PointCloud<pcl::PointXYZ>);


    std::vector<int> inliers_left;
    std::vector<int> inliers_right;
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_left(model_left);
    ransac_left.setDistanceThreshold(0.05);
    ransac_left.computeModel();
    ransac_left.getInliers(inliers_left);
    
    
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_right(model_right);
    ransac_right.setDistanceThreshold(0.05);
    ransac_right.computeModel();
    ransac_right.getInliers(inliers_right);

    pcl::copyPointCloud (*left_cloud, inliers_left, *final_left);
    pcl::copyPointCloud (*right_cloud, inliers_right, *final_right);


    pcl::concatenate(*final_left, *final_right, *final_total);


    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/left_out.pcd", *final_left);
    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/right_out.pcd", *final_right);
    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/total_out.pcd", *final_total);
    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/tttest.pcd", *tttest);


    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);  // 平面方程 ax + by + cz + d = 0
    coefficients->values[0] = 0.0;   // a
    coefficients->values[1] = 0.0;   // b
    coefficients->values[2] = 1.0;   // c
    coefficients->values[3] = 0.0;   // d


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(final_total);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    // 保存投影后的点云数据
    pcl::io::savePCDFile("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/cloud_projected.pcd", *cloud_projected);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud_projected);
    chull.setDimension(2);
    chull.reconstruct(*cloud_hull);

    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/cloud_hull.pcd", *cloud_hull);


    pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_check(new pcl::PointCloud<pcl::PointXYZ>());

    // 初始化随机数生成器
    std::random_device rd;
    
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis_x(-10, 50);
    std::uniform_real_distribution<float> dis_y(-4, 4);
    std::uniform_real_distribution<float> dis_z(-2, 2);

    // 生成随机点云
    for (int i = 0; i < 10000; ++i) {
        pcl::PointXYZ point;
        point.x = dis_x(gen);
        point.y = dis_y(gen);
        point.z = 0; // 如果需要生成3D点，可以对z轴进行同样的操作
        points_to_check->points.push_back(point);
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr points_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_out(new pcl::PointCloud<pcl::PointXYZ>());


    std::vector<bool> inside(points_to_check->points.size(), false);


    for (size_t i = 0; i < points_to_check->points.size(); ++i) {


        bool flag =  pcl::isPointIn2DPolygon(points_to_check->points[i], *cloud_hull);
        if(flag == true){
            points_in->points.push_back(points_to_check->points[i] );

        }else{

          points_out->points.push_back(points_to_check->points[i] );

        }
    }


    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(points_in);


    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2);  // 聚类容差
    ec.setMinClusterSize(2);     // 最小聚类大小
    ec.setMaxClusterSize(100);   // 最大聚类大小
    ec.setSearchMethod(kd_tree);
    ec.setInputCloud(points_in);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Convex Hull Viewer"));

    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& idx : indices.indices) {
            cluster->points.push_back(points_in->points[idx]);
        }

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color(cluster, rand() % 256, rand() % 256, rand() % 256);
        viewer->addPointCloud<pcl::PointXYZ>(cluster, cluster_color, "cluster_" + std::to_string(cluster_id));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cluster_" + std::to_string(cluster_id));

    
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);

            viewer->addCube(min_pt[0],max_pt[0],min_pt[1], max_pt[1], min_pt[2], max_pt[2], 160, 32, 240, "cube_" + std::to_string(cluster_id));


          viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "cube_" + std::to_string(cluster_id)); // 设置边界框透明度为0.5

        ++cluster_id;
    }


    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_projected, "original cloud");
    viewer->addPointCloud<pcl::PointXYZ>(cloud_hull, "convex hull");
    viewer->addPointCloud<pcl::PointXYZ>(points_in,"points_in");
    viewer->addPointCloud<pcl::PointXYZ>(points_out,"points_out");


    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "points_in");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "points_out");


    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "convex hull");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "convex hull");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points_in");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points_out");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterGround2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/cloud_filtering.pcd", *cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(-10, 10);  
    // pass.filter(*cloud_filtered_x);

    // pass.setInputCloud(cloud_filtered_x);  
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-10, 10);  
    // pass.filter(*cloud_filtered_y);



      pass.setInputCloud(cloud);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(-0.5,3);
      pass.filter(*cloud_filtered_y);


      pass.setInputCloud(cloud_filtered_y);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-10,10);
      pass.filter(*cloud_filtered);


    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/cloud_xz.pcd", *cloud_filtered);



    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    seg.setDistanceThreshold (0.05);

    seg.setInputCloud (cloud_filtered);

    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");

        std::cout<<"inliers empty!!!!!!!!!"<<std::endl;

            pcl::PointCloud<pcl::PointXYZ>::Ptr ssss(new pcl::PointCloud<pcl::PointXYZ>);
        return ssss;
    }
 


    // 提取地面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setNegative(false);
    extract.setIndices (inliers);
    extract.filter (*cloud_filtered_xyz);

    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/cloud_filter_complete.pcd", *cloud_filtered_xyz);


  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;


    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];
    
    float norm = std::sqrt(a*a + b*b + c*c);

    float theta_x = std::acos(a/norm)*180.0/M_PI;
    float theta_y = std::acos(b/norm)*180.0/M_PI;
    float theta_z = std::acos(c/norm)*180.0/M_PI;
    
  

    std::cout<<"theta_x = "<<theta_x <<" theta_y = " << theta_y<< " theta_z = "<<theta_z<<std::endl;

    float distance = std::abs(a * 0 + b * 0 + c * 0 + d) / std::sqrt(a * a + b * b + c * c);
    std::cout<<"distance = " <<distance<<std::endl;


    float h =  distance / (std::cos(theta_z*M_PI/180.0));
    std::cout<<"h = "<<h<<std::endl;
    height = distance;
    slope = theta_z * M_PI/180.0;
    std::cout<<"slope = "<<slope<<std::endl;
    return cloud_filtered_xyz;

}





//-----------------------------------------



void clusterOnRail(pcl::PointCloud<pcl::PointXYZ>::Ptr rail_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr org_cloud){

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);  // 平面方程 ax + by + cz + d = 0
    coefficients->values[0] = 0.0;   // a
    coefficients->values[1] = 0.0;   // b
    coefficients->values[2] = 1.0;   // c
    coefficients->values[3] = 0.0;   // d


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(rail_cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud_projected);
    chull.setDimension(2);
    chull.reconstruct(*cloud_hull);


    pcl::PointCloud<pcl::PointXYZ>::Ptr points_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_out(new pcl::PointCloud<pcl::PointXYZ>());



    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_hull, centroid);


    float expansion_distance = 1.0;  
    for (auto& point : cloud_hull->points) {
        if (point.y > centroid[1]) {
            point.y += expansion_distance;
        } else {
            point.y -= expansion_distance;
        }
    }


    std::vector<bool> inside(org_cloud->points.size(), false);

    for (size_t i = 0; i < org_cloud->points.size(); ++i) {
        bool flag =  pcl::isPointIn2DPolygon(org_cloud->points[i], *cloud_hull);
        if(flag == true){
          points_in->points.push_back(org_cloud->points[i] );
        }else{
          points_out->points.push_back(org_cloud->points[i]);
        }
    }


    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(points_in);


    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2);  // 聚类容差
    ec.setMinClusterSize(3);     // 最小聚类大小
    ec.setMaxClusterSize(1000);   // 最大聚类大小
    ec.setSearchMethod(kd_tree);
    ec.setInputCloud(points_in);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Convex Hull Viewer"));

    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& idx : indices.indices) {
            cluster->points.push_back(points_in->points[idx]);
        }

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_color(cluster, rand() % 256, rand() % 256, rand() % 256);
        viewer->addPointCloud<pcl::PointXYZ>(cluster, cluster_color, "cluster_" + std::to_string(cluster_id));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cluster_" + std::to_string(cluster_id));

    
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        viewer->addCube(min_pt[0],max_pt[0],min_pt[1], max_pt[1], min_pt[2], max_pt[2], 160, 32, 240, "cube_" + std::to_string(cluster_id));
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, "cube_" + std::to_string(cluster_id)); // 设置边界框透明度为0.5
        ++cluster_id;
    }


    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_projected, "rail cloud");
    viewer->addPointCloud<pcl::PointXYZ>(cloud_hull, "convex hull");
    viewer->addPointCloud<pcl::PointXYZ>(points_in,"points_in");
    viewer->addPointCloud<pcl::PointXYZ>(points_out,"points_out");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "points_in");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "points_out");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "convex hull");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rail cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "convex hull");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points_in");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points_out");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}



pcl::PointCloud<pcl::PointXYZ>::Ptr sampleConsensus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line (new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
    std::vector<int> inliers;
    pcl::RandomSampleConsensus<pcl::PointXYZ> ran(model_line);
    ran.setDistanceThreshold(0.05);
    ran.computeModel();
    ran.getInliers(inliers);
    pcl::copyPointCloud (*cloud, inliers, *final_cloud);
    return final_cloud;

}



pcl::PointCloud<pcl::PointXYZ>::Ptr getRail(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1.2, 0);  
    pass.filter(*left_cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(0, 1.2);  
    pass.filter(*right_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_left  = sampleConsensus(left_cloud) ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_right = sampleConsensus(right_cloud) ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_total (new pcl::PointCloud<pcl::PointXYZ>());

    pcl::concatenate(*final_left, *final_right, *final_total);

    return final_total;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float & height){

    GroundSegmentationParams params;
    params.visualize = false;
    params.n_bins = 120;
    params.n_segments = 360;
    params.max_slope = 0.3;
    params.long_threshold = 1.0;
    params.max_long_height = 0.1;
    params.max_start_height =0.2;
    params.line_search_angle = 0.1;
    params.n_threads = 4;
    params.max_dist_to_line = 0.1;
    float r_min = 0.5;
    float r_max = 50;
    float max_fit_error= 0.05;
    params.r_min_square = r_min*r_min;
    params.r_max_square = r_max*r_max;
    params.max_error_square = max_fit_error * max_fit_error;

    // params.sensor_height = 0.2;
    params.sensor_height = height;

    GroundSegmentation segmenter(params);
    std::vector<int> labels;

    segmenter.segment(*cloud, &labels);


    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    ground_cloud->header = cloud->header;
    obstacle_cloud->header = cloud->header;
    for (int i = 0; i < cloud->size(); ++i) {
      if (labels[i] == 1) {
          ground_cloud->push_back(cloud->at(i));
      } else{
          obstacle_cloud->push_back(cloud->at(i));
      }
    }
    
    return obstacle_cloud;
}

std::vector<float> getAttitude(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    std::vector<float> ans(4, -1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2, 2);  
    pass.filter(*test_cloud);

    pass.setInputCloud(test_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-5, 10);  
    pass.filter(*test_cloud);

    pass.setInputCloud(test_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-3, 1);  
    pass.filter(*test_cloud);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);

    // 距离阈值 单位m
    seg.setDistanceThreshold (0.05);
    seg.setInputCloud (test_cloud);
    seg.segment (*inliers, *coefficients);


    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");

        std::cout<<"inliers empty!!!"<<std::endl;

        return ans;
    }
 

    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];
    float height = std::abs(a * 0 + b * 0 + c * 0 + d) / std::sqrt(a * a + b * b + c * c);

    float norm = std::sqrt(a*a + b*b + c*c);
    float theta_x = std::acos(a/norm)*180.0/M_PI;
    float theta_y = std::acos(b/norm)*180.0/M_PI;
    float theta_z = std::acos(c/norm)*180.0/M_PI;

    std::cout<<"height = "<<height<<" theta_x = "<<theta_x <<" theta_y = " << theta_y<< " theta_z = "<<theta_z<<std::endl;

    ans[0] = height;
    ans[1] = theta_x;
    ans[2] = theta_y;
    ans[3] = theta_z;
    
    return ans;


}

void run( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    std::vector<float> ans = getAttitude(cloud);
    if(ans[0] == -1){
      return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud = filterGround(cloud, ans[0]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rail_cloud = getRail(obstacle_cloud);
    clusterOnRail(rail_cloud, cloud);

}


int main(int argc, char** argv) {

    std::string cloud_file = "/home/nvidia/wjd/pcds/056_1631639436.731886000.pcd";

    pcl::PointCloud<pcl::PointXYZ> cloud;

    std::string str = cloud_file;
    int idx = str.find_last_of(".");
    std::string ext = str.substr(idx, str.size()-idx);

    if(ext == ".ply")
    pcl::io::loadPLYFile(cloud_file, cloud);
    
    if(ext == ".pcd")
    pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_file, cloud);


    pcl::copyPointCloud(cloud, *tttest);


    run(tttest);


    GroundSegmentationParams params;
    params.visualize = false;
    params.n_bins = 120;
    params.n_segments = 360;
    params.max_slope = 0.3;
    params.long_threshold = 1.0;
    params.max_long_height = 0.1;
    params.max_start_height =0.2;
    params.line_search_angle = 0.1;
    params.n_threads = 4;
    params.max_dist_to_line = 0.1;
    float r_min = 0.5;
    float r_max = 50;
    float max_fit_error= 0.05;
    params.r_min_square = r_min*r_min;
    params.r_max_square = r_max*r_max;
    params.max_error_square = max_fit_error * max_fit_error;

    params.sensor_height = 0.2;



    pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/test_ascii.pcd", cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr abcd (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(cloud, *abcd);

    auto ddd = getAttitude(abcd);

  
    // pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/test_22222.pcd", *ddd);

    // *ddd = cloud;

    // pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/test_3333.pcd", *ddd);


    pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud (new pcl::PointCloud<pcl::PointXYZ>(cloud));

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(test_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1.2, 1.2);  
    pass.filter(*test_cloud);


    pass.setInputCloud(test_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, 50);  
    pass.filter(*test_cloud);

    
    pass.setInputCloud(test_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-4, 1.5);  
    pass.filter(*test_cloud);




    auto cloud_filtered_xyz = filterGround2(test_cloud);
    
    if(!cloud_filtered_xyz->empty()){
      Eigen::Vector4d centroid;
      pcl::compute3DCentroid(*cloud_filtered_xyz , centroid);
      std::cout<<centroid[0] <<" "<<centroid[1]<<" "<<centroid[2]<<" "<<centroid[3]<<std::endl;

      // params.sensor_height = std::abs(centroid[2]);

      params.sensor_height = height;
      // params.max_slope = slope;
      pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/cloud_filtered_xyz.pcd", *cloud_filtered_xyz);

      std::cout<<"cloud_filtered_xyz not empty!"<<std::endl;

    }else{

      std::cout<<"empty!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    }





    GroundSegmentation segmenter(params);
    std::vector<int> labels;

    segmenter.segment(*test_cloud, &labels);


    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
    ground_cloud.header = test_cloud->header;
    obstacle_cloud.header = test_cloud->header;
    for (size_t i = 0; i < test_cloud->size(); ++i) {
      if (labels[i] == 1) ground_cloud.push_back(test_cloud->at(i));
      else obstacle_cloud.push_back(test_cloud->at(i));
    }
    
    if(!ground_cloud.empty()){
      pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/ground_cloud.pcd", ground_cloud);

    }
    if(!obstacle_cloud.empty()){

      pcl::io::savePCDFileASCII ("/home/nvidia/wjd/linefit/src/linefit_ground_segmentation/doc/obstacle_cloud.pcd", obstacle_cloud);
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr ttt (new pcl::PointCloud<pcl::PointXYZ>(obstacle_cloud));
      
      sampleConsensus2(ttt);

    }
    if(ground_cloud.empty()){
      std::cout<<"ground_cloud empty!"<<std::endl;
    }
    if(obstacle_cloud.empty()){
      std::cout<<"obstacle_cloud empty!"<<std::endl;

    }



  

}
