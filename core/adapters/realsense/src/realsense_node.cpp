// realsense_node.cpp
//
// Standalone ROS2 node for the RealSense D4xx depth camera.
// Completely independent of the control loop — never touches /lowcmd.
// Publishes:
//   /camera/depth/image_rect_raw  (sensor_msgs/Image, 16UC1)
//   /camera/depth/camera_info     (sensor_msgs/CameraInfo)
//   /camera/depth/points          (sensor_msgs/PointCloud2)
//   /tf_static  torso_link → realsense_depth_link

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include <librealsense2/rs.hpp>

using namespace std::chrono_literals;

class RealSenseNode : public rclcpp::Node {
public:
    explicit RealSenseNode() : rclcpp::Node("realsense_node") {
        int width  = declare_parameter("width",  480);
        int height = declare_parameter("height", 270);
        fps_       = declare_parameter("fps",     60);
        publish_cloud_ = declare_parameter("publish_cloud", true);

        cfg_.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps_);
        auto profile = pipeline_.start(cfg_);
        depth_scale_ = profile.get_device()
                              .first<rs2::depth_sensor>()
                              .get_depth_scale();

        auto vsp = profile.get_stream(RS2_STREAM_DEPTH)
                          .as<rs2::video_stream_profile>();
        auto intr = vsp.get_intrinsics();
        fx_ = intr.fx; fy_ = intr.fy;
        cx_ = intr.ppx; cy_ = intr.ppy;
        width_  = width;
        height_ = height;

        pipeline_.wait_for_frames(1000);  // warm-up

        depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/depth/image_rect_raw", rclcpp::SensorDataQoS());
        info_pub_  = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/depth/camera_info",    rclcpp::SensorDataQoS());
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", 2);

        static_tf_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_static_tf();

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / fps_),
            [this]{ capture_and_publish(); });

        RCLCPP_INFO(get_logger(), "RealSense ready %dx%d @ %d Hz", width, height, fps_);
    }

    ~RealSenseNode() override { try { pipeline_.stop(); } catch (...) {} }

private:
    rs2::pipeline     pipeline_;
    rs2::config       cfg_;
    float             depth_scale_{0.001f};
    float             fx_{0}, fy_{0}, cx_{0}, cy_{0};
    int               width_{480}, height_{270}, fps_{60};
    bool              publish_cloud_{true};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster>        static_tf_bc_;
    rclcpp::TimerBase::SharedPtr                                timer_;

    void capture_and_publish() {
        rs2::frameset frames;
        if (!pipeline_.try_wait_for_frames(&frames, (1000 / fps_) * 2))
            return;
        auto depth = frames.get_depth_frame();
        if (!depth) return;

        auto stamp = now();

        // Depth image
        sensor_msgs::msg::Image img;
        img.header.stamp    = stamp;
        img.header.frame_id = "realsense_depth_link";
        img.height    = height_;
        img.width     = width_;
        img.encoding  = "16UC1";
        img.step      = width_ * sizeof(uint16_t);
        img.data.resize(img.step * img.height);
        std::memcpy(img.data.data(), depth.get_data(), img.data.size());
        depth_pub_->publish(img);

        // CameraInfo
        sensor_msgs::msg::CameraInfo ci;
        ci.header = img.header;
        ci.width  = width_;
        ci.height = height_;
        ci.distortion_model = "plumb_bob";
        ci.d  = {0,0,0,0,0};
        ci.k  = {fx_,0,cx_, 0,fy_,cy_, 0,0,1};
        ci.p  = {fx_,0,cx_,0, 0,fy_,cy_,0, 0,0,1,0};
        info_pub_->publish(ci);

        // PointCloud2
        if (publish_cloud_) {
            sensor_msgs::msg::PointCloud2 cloud;
            cloud.header = img.header;
            cloud.height = height_;
            cloud.width  = width_;
            cloud.is_dense = false;
            cloud.point_step = 12;
            cloud.row_step   = 12 * width_;
            for (auto& n : {"x","y","z"}) {
                sensor_msgs::msg::PointField pf;
                pf.name   = n;
                pf.offset = (&n - &"x") * 4;
                pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
                pf.count  = 1;
                cloud.fields.push_back(pf);
            }
            cloud.data.resize(cloud.row_step * height_);
            float* ptr = reinterpret_cast<float*>(cloud.data.data());
            const uint16_t* raw = reinterpret_cast<const uint16_t*>(depth.get_data());
            for (int v = 0; v < height_; ++v) {
                for (int u = 0; u < width_; ++u) {
                    float z = raw[v*width_+u] * depth_scale_;
                    *ptr++ = z > 0 ? (u - cx_) * z / fx_ : std::numeric_limits<float>::quiet_NaN();
                    *ptr++ = z > 0 ? (v - cy_) * z / fy_ : std::numeric_limits<float>::quiet_NaN();
                    *ptr++ = z > 0 ? z : std::numeric_limits<float>::quiet_NaN();
                }
            }
            cloud_pub_->publish(cloud);
        }
    }

    void publish_static_tf() {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = now();
        tf.header.frame_id = "torso_link";
        tf.child_frame_id  = "realsense_depth_link";
        tf.transform.translation.x =  0.0513;
        tf.transform.translation.y =  0.015;
        tf.transform.translation.z =  0.4571;
        tf.transform.rotation.w    =  0.9136;
        tf.transform.rotation.x    =  0.0044;
        tf.transform.rotation.y    =  0.4067;
        tf.transform.rotation.z    =  0.0;
        static_tf_bc_->sendTransform(tf);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealSenseNode>());
    rclcpp::shutdown();
}
