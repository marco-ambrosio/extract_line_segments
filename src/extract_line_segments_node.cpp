#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef boost::geometry::model::d2::point_xy<double> xy;
laser_geometry::LaserProjection lp;

class ExtractLineSegmentsNode : public rclcpp::Node
{
public:
    ExtractLineSegmentsNode()
    : Node("extract_line_segments_node")
    {
        this->declare_parameter("douglas_peucker_distance", 0.1);
        this->declare_parameter("neighbor_distance", 0.5);
        this->declare_parameter("min_cluster_size", 5);

        this->get_parameter("douglas_peucker_distance", douglas_peucker_distance_);
        this->get_parameter("neighbor_distance", neighbor_distance_);
        this->get_parameter("min_cluster_size", min_cluster_size_);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("line_segments", 2);
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 2, std::bind(&ExtractLineSegmentsNode::laser_cb, this, std::placeholders::_1)
        );
    }

private:
    void laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create marker for RViz
        visualization_msgs::msg::Marker line_segments;
        line_segments.header.frame_id = msg->header.frame_id;
        line_segments.header.stamp = msg->header.stamp;
        line_segments.ns = "line_segments";
        line_segments.action = visualization_msgs::msg::Marker::ADD;
        line_segments.type = visualization_msgs::msg::Marker::LINE_LIST;
        line_segments.pose.orientation.w = 1.0;
        line_segments.id = 42;
        line_segments.scale.x = 0.1;
        line_segments.color.r = 1.0;
        line_segments.color.a = 1.0;

        // Convert laser data to PointCloud2
        sensor_msgs::msg::PointCloud2 cloud;
        lp.projectLaser(*msg, cloud);

        // Convert PointCloud2 to pcl::PointCloud for easier processing
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(cloud, pcl_cloud);

        // Cluster the point cloud
        boost::geometry::model::linestring<xy> cluster;
        for (size_t i = 0; i < pcl_cloud.points.size(); i++)
        {
            boost::geometry::append(cluster, xy(pcl_cloud.points[i].x, pcl_cloud.points[i].y));
            bool is_cluster_ready = false;

            // If this is the last point, complete the cluster
            if (i == pcl_cloud.points.size() - 1)
            {
                is_cluster_ready = true;
            }
            else
            {
                // Find if the point is in the cluster using Euclidean distance
                double x1 = pcl_cloud.points[i].x;
                double y1 = pcl_cloud.points[i].y;
                double x2 = pcl_cloud.points[i + 1].x;
                double y2 = pcl_cloud.points[i + 1].y;
                is_cluster_ready = (pow(x1 - x2, 2) + pow(y1 - y2, 2) > pow(neighbor_distance_, 2));
            }

            if (is_cluster_ready)
            {
                if (cluster.size() >= min_cluster_size_)
                {
                    // Simplify process using Douglas-Peucker algorithm
                    boost::geometry::model::linestring<xy> simplified;
                    boost::geometry::simplify(cluster, simplified, douglas_peucker_distance_);

                    // Parse results from "simplified" for visualization purposes
                    for (size_t j = 0; j < simplified.size() - 1; j++)
                    {
                        geometry_msgs::msg::Point p1, p2;
                        p1.x = simplified[j].x();
                        p1.y = simplified[j].y();
                        p2.x = simplified[j + 1].x();
                        p2.y = simplified[j + 1].y();

                        line_segments.points.push_back(p1);
                        line_segments.points.push_back(p2);
                    }
                    cluster.clear();
                }
                else
                {
                    cluster.clear();
                }
            }
        }

        marker_pub_->publish(line_segments);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    double douglas_peucker_distance_;
    double neighbor_distance_;
    int min_cluster_size_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExtractLineSegmentsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
