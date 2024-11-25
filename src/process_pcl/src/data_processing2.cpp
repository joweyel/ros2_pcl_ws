#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp> 
#include <sensor_msgs/msg/image.hpp> 
#include <sensor_msgs/msg/camera_info.hpp> 
#include <cv_bridge/cv_bridge.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h> 
#include <message_filters/subscriber.h> 
#include <message_filters/sync_policies/approximate_time.h> 
#include <message_filters/synchronizer.h>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Geometry>

#include "pcl_utils.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> SyncPolicies;

class SyncCloudHandler : public rclcpp::Node
{
    public:
        SyncCloudHandler() : 
            Node("data_handler"), 
            counter_(0),
            cloud_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
            tree_(std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>())

        {
            // Use topics as parameter
            this->declare_parameter<std::string>("pcl_topic", "/oak/points");
            this->declare_parameter<std::string>("img_topic", "/oak/rgb/image_raw");
            this->declare_parameter<std::string>("camera_info_topic", "/oak/right/camera_info");
            // Get the name-parameter of the relevant topics
            std::string pcl_topic = this->get_parameter("pcl_topic").as_string();
            std::string img_topic = this->get_parameter("img_topic").as_string();
            std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();

            // Quality of service object to be able to access pointclouds (best effort required)
            auto qos_pcl = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);
            auto qos_default = rclcpp::QoS(10);

            pcl_filter_.subscribe(this, pcl_topic, qos_pcl.get_rmw_qos_profile());
            img_filter_.subscribe(this, img_topic, qos_default.get_rmw_qos_profile());
            camera_info_filter_.subscribe(this, camera_info_topic, qos_default.get_rmw_qos_profile());

            synchronizer_.reset(new message_filters::Synchronizer<SyncPolicies>(
                SyncPolicies(10), pcl_filter_, img_filter_, camera_info_filter_
            ));
            synchronizer_->registerCallback(std::bind(
                &SyncCloudHandler::callback, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
            );
            this->cloud_output_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_output", 10);

            // Debugging publisher for tf of camera
            this->camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info_depth", 10);
        }

        void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr img_msg,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg)
        {
            RCLCPP_INFO(this->get_logger(), "\n\n[%d] Sync-Callback called!", this->counter_++);

            // obtain pcl data
            pcl::fromROSMsg(*pcl_msg, *cloud_);


            // obtain image data
            this->img_ = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8)->image;

            // obtain camera info data
            this->camera_info_ = *camera_info_msg;

            RCLCPP_INFO(this->get_logger(), "PointCloud size: %lu", cloud_->points.size());
            RCLCPP_INFO(this->get_logger(), "Image size: [%d x %d]", img_.rows, img_.cols); 
            RCLCPP_INFO(this->get_logger(), "Camera Info - Height: %d, Width: %d", camera_info_.height, camera_info_.width);

            filter_distance(this->cloud_, 1.2f);
            getPCLDimensions(this->cloud_);
            downsample(this->cloud_);
            removeNaN(this->cloud_);
            remove_outlier(this->cloud_);
            remove_bad_clusters(this->tree_, this->cloud_);

            // publish processed cloud
            sensor_msgs::msg::PointCloud2 out_cloud_msg;
            pcl::toROSMsg(*this->cloud_, out_cloud_msg);
            out_cloud_msg.header.stamp = this->now();
            out_cloud_msg.header.frame_id = "oak_right_camera_frame"; // "oak-d-base-frame";
            this->cloud_output_pub_->publish(out_cloud_msg);

            this->camera_info_pub_->publish(this->camera_info_);
        }

    private:
        unsigned int counter_;

        // subscriber
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_filter_;
        message_filters::Subscriber<sensor_msgs::msg::Image> img_filter_;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_filter_;

        // publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_output_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;


        // for syncronizing msg's by time
        std::shared_ptr<message_filters::Synchronizer<SyncPolicies>> synchronizer_;

        // data storage of received messages
        cv::Mat img_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        sensor_msgs::msg::CameraInfo camera_info_;
        // Datastructures
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncCloudHandler>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}