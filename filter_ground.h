

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

pcl::PointCloud<pcl::PointXYZ>::Ptr clusterOnRail(pcl::PointCloud<pcl::PointXYZ>::Ptr rail_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr org_cloud, const float & expansion_distance );
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleConsensus();
pcl::PointCloud<pcl::PointXYZ>::Ptr getRail(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float & height);
std::vector<float> getAttitude(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void run( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
