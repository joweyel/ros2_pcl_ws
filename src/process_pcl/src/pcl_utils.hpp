#ifndef PCL_UTILS_HPP
#define PCL_UTILS_HPP

// general includes
#include <cmath>
#include <exception>
#include <vector>

// PCL Includes
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/search/kdtree.h>


// PCL -> ROS Headers
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/camera_info.hpp> 

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Eigen
#include <Eigen/Dense>

void getPCLDimensions(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

// Downsamplint pointcloud
void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

// filter points by depth
void filter_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float maxZ);

// in-place nan-removal
void removeNaN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void remove_outlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void remove_bad_clusters(pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

std::vector<float> l2_norm_points(const std::vector<pcl::PointXYZ>& centroids,
                                  const pcl::PointXYZ camera_origin);

#endif // PCL_UTILS_HPP