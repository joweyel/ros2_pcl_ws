#include "pcl_utils.hpp"

void getPCLDimensions(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    float length = max_pt.x - min_pt.x;
    float width  = max_pt.y - min_pt.y;
    float height = max_pt.z - min_pt.z;

    std::cout << "Bounding Box:" << std::endl;
    std::cout << "Min Point: (" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << ")" << std::endl;
    std::cout << "Max Point: (" << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << ")" << std::endl;
    std::cout << "Dimensions (LxWxH): " << length << " x " << width << " x " << height << std::endl;
}


void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());

    float leaf_size = 0.01f;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
              << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

    *cloud = *cloud_filtered;
}

void filter_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float maxZ)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ>());

    range_cond->addComparison(
        pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (
            new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LE, maxZ)
        )
    );
    
    // Create filter object and apply condition
    pcl::ConditionalRemoval<pcl::PointXYZ> cond_rem;
    cond_rem.setCondition(range_cond);
    cond_rem.setInputCloud(cloud);
    cond_rem.filter(*cloud_filtered);

    std::cerr << "PointCloud after distance filtering: " 
              << cloud_filtered->width * cloud_filtered->height 
              << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." 
              << std::endl;

    *cloud = *cloud_filtered;
}

void removeNaN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
}

void remove_outlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.1);
    // sor.setStddevMulThresh(0.2); // Standard deviation multiplier threshold 
    sor.filter(*cloud_filtered);
    *cloud = *cloud_filtered;
}


void remove_bad_clusters(pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    // Extracting valid cluster-indices and saving them for filtering
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>());

    // obtain all indices of a (valid) cluster
    for (const auto& indices : cluster_indices)
    {
        // Iterate over all indices and put relevant points in resulting cloud
        for (const auto& index : indices.indices)
        {
            filtered_cloud->points.push_back(cloud->points[index]);
        }
    }
    *cloud = *filtered_cloud;
}

std::vector<float> l2_norm_points(const std::vector<pcl::PointXYZ>& centroids,
                           const pcl::PointXYZ camera_origin)
{
    unsigned int n = centroids.size();
    std::vector<float> distances (n);
    for (unsigned int i = 0; i < n; i++)
    {
        distances.at(i) = std::sqrt(
            std::pow(centroids.at(i).x - camera_origin.x, 2) + 
            std::pow(centroids.at(i).y - camera_origin.y, 2) +
            std::pow(centroids.at(i).z - camera_origin.z, 2)
        );
    }
    return distances;
}