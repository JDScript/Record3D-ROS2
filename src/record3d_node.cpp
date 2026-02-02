#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <record3d/Record3DStream.h>
#include <mutex>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>

class Record3DNode : public rclcpp::Node
{
public:
    Record3DNode() : Node("record3d_node")
    {
        // Declare parameters
        this->declare_parameter<int>("device_index", 0);
        this->declare_parameter<std::string>("frame_id", "");
        this->declare_parameter<double>("connection_retry_interval", 2.0);
        this->declare_parameter<std::string>("world_frame", "world");
        this->declare_parameter<bool>("apply_transform", true);
        
        device_index_ = this->get_parameter("device_index").as_int();
        connection_retry_interval_ = this->get_parameter("connection_retry_interval").as_double();
        world_frame_ = this->get_parameter("world_frame").as_string();
        apply_transform_ = this->get_parameter("apply_transform").as_bool();
        
        // Set frame_id based on device_index if not provided
        std::string frame_id_param = this->get_parameter("frame_id").as_string();
        if (frame_id_param.empty()) {
            frame_id_ = "device_" + std::to_string(device_index_) + "_camera_link";
        } else {
            frame_id_ = frame_id_param;
        }

        // Create topic prefix with device ID
        topic_prefix_ = "device_" + std::to_string(device_index_) + "/";

        // Setup Record3D stream
        stream_ = std::make_unique<Record3D::Record3DStream>();
        
        // Setup TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // Set callbacks
        stream_->onNewFrame = [this](const Record3D::BufferRGB &rgb_frame,
                                      const Record3D::BufferDepth &depth_frame,
                                      const Record3D::BufferConfidence &conf_frame,
                                      const Record3D::BufferMisc &misc_data,
                                      uint32_t rgb_width,
                                      uint32_t rgb_height,
                                      uint32_t depth_width,
                                      uint32_t depth_height,
                                      uint32_t conf_width,
                                      uint32_t conf_height,
                                      Record3D::DeviceType device_type,
                                      Record3D::IntrinsicMatrixCoeffs K,
                                      Record3D::CameraPose camera_pose)
        {
            this->onNewFrame(rgb_frame, depth_frame, conf_frame, misc_data,
                           rgb_width, rgb_height, depth_width, depth_height,
                           conf_width, conf_height, device_type, K, camera_pose);
        };

        stream_->onStreamStopped = [this]() {
            RCLCPP_WARN(this->get_logger(), "Stream stopped! Will retry connection...");
            connected_ = false;
            destroyPublishers();
        };

        // Create connection monitoring timer
        connection_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(connection_retry_interval_),
            std::bind(&Record3DNode::checkAndConnect, this));

        RCLCPP_INFO(this->get_logger(), "Record3D node started. Monitoring for device %d...", device_index_);
        RCLCPP_INFO(this->get_logger(), "World frame: %s, Camera frame: %s", 
                   world_frame_.c_str(), frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Coordinate transform (Y-up to Z-up): %s", 
                   apply_transform_ ? "enabled" : "disabled");
        
        // Try to connect immediately instead of waiting for first timer tick
        checkAndConnect();
    }

    ~Record3DNode()
    {
        if (stream_ && connected_) {
            stream_->Disconnect();
        }
    }

private:
    void checkAndConnect()
    {
        // If already connected, do nothing
        if (connected_) {
            return;
        }

        auto devices = Record3D::Record3DStream::GetConnectedDevices();
        
        if (devices.empty()) {
            if (!connection_warning_shown_) {
                RCLCPP_WARN(this->get_logger(), 
                    "No iOS devices found. Waiting for device connection...\n"
                    "  Please ensure:\n"
                    "    1. iPhone is connected via USB\n"
                    "    2. Record3D app is installed and opened\n"
                    "    3. USB Streaming is enabled in app settings");
                connection_warning_shown_ = true;
            }
            return;
        }

        // Device list changed, show available devices
        if (devices.size() != last_device_count_) {
            RCLCPP_INFO(this->get_logger(), "Found %zu device(s):", devices.size());
            for (size_t i = 0; i < devices.size(); i++) {
                RCLCPP_INFO(this->get_logger(), "  [%zu] Product ID: %u, UDID: %s", 
                           i, devices[i].productId, devices[i].udid.c_str());
            }
            last_device_count_ = devices.size();
        }

        if (device_index_ >= static_cast<int>(devices.size())) {
            if (!connection_warning_shown_) {
                RCLCPP_WARN(this->get_logger(), 
                    "Device index %d is out of range (0-%zu). Waiting for more devices...",
                    device_index_, devices.size() - 1);
                connection_warning_shown_ = true;
            }
            return;
        }

        // Try to connect
        const auto &selected_device = devices[device_index_];
        RCLCPP_INFO(this->get_logger(), 
            "Attempting to connect to device %d (Product ID: %u)...", 
            device_index_, selected_device.productId);

        bool success = stream_->ConnectToDevice(selected_device);
        
        if (success) {
            connected_ = true;
            connection_warning_shown_ = false;
            
            // Create publishers now that we're connected
            createPublishers();
            
            RCLCPP_INFO(this->get_logger(), "✓ Connected! Press RECORD in Record3D app to start streaming.");
        } else {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to connect to device. Will retry in %.1f seconds...",
                connection_retry_interval_);
            connection_warning_shown_ = false;
        }
    }

    void createPublishers()
    {
        std::lock_guard<std::mutex> lock(publisher_mutex_);
        
        if (publishers_created_) {
            return;  // Already created
        }

        // Use BEST_EFFORT QoS for lower latency (no retransmissions)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // Only publish compressed images (better performance)
        rgb_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            topic_prefix_ + "rgb/image_raw/compressed", qos);
        
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_prefix_ + "depth/image_raw", qos);
        confidence_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_prefix_ + "confidence/image_raw", qos);
        rgb_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_prefix_ + "rgb/camera_info", qos);
        depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic_prefix_ + "depth/camera_info", qos);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic_prefix_ + "camera_pose", qos);
        pose_delta_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic_prefix_ + "camera_pose_delta", qos);

        publishers_created_ = true;
        initial_pose_set_ = false;  // Reset delta pose computation
        
        RCLCPP_DEBUG(this->get_logger(), "Publishers created for device %d", device_index_);
    }

    void destroyPublishers()
    {
        std::lock_guard<std::mutex> lock(publisher_mutex_);
        
        if (!publishers_created_) {
            return;  // Already destroyed
        }

        rgb_compressed_pub_.reset();
        depth_pub_.reset();
        confidence_pub_.reset();
        rgb_info_pub_.reset();
        depth_info_pub_.reset();
        pose_pub_.reset();
        pose_delta_pub_.reset();

        publishers_created_ = false;
        
        RCLCPP_DEBUG(this->get_logger(), "Publishers destroyed for device %d", device_index_);
    }

    void onNewFrame(const Record3D::BufferRGB &rgb_frame,
                    const Record3D::BufferDepth &depth_frame,
                    const Record3D::BufferConfidence &/*conf_frame*/,
                    const Record3D::BufferMisc &/*misc_data*/,
                    uint32_t rgb_width,
                    uint32_t rgb_height,
                    uint32_t depth_width,
                    uint32_t depth_height,
                    uint32_t /*conf_width*/,
                    uint32_t /*conf_height*/,
                    Record3D::DeviceType /*device_type*/,
                    Record3D::IntrinsicMatrixCoeffs K,
                    Record3D::CameraPose camera_pose)
    {
        std::lock_guard<std::mutex> lock(publisher_mutex_);
        
        if (!publishers_created_) {
            return;  // Not ready to publish
        }

        auto timestamp = this->now();
        auto frame_start_time = std::chrono::steady_clock::now();

        // Print message on first frame
        if (!first_frame_received_) {
            RCLCPP_INFO(this->get_logger(), "✓ Receiving data! Streaming started.");
            first_frame_received_ = true;
        }

        // Publish RGB image (most important)
        publishRGBCompressed(rgb_frame, rgb_width, rgb_height, timestamp);

        // Publish depth image
        publishDepthImage(depth_frame, depth_width, depth_height, timestamp);

        // Publish TF transform (camera pose in world frame)
        publishTF(camera_pose, timestamp);
        
        // Publish camera pose message
        publishCameraPose(camera_pose, timestamp);

        // Publish camera info at reduced rate (every 10th frame)
        if (frame_count_ % 10 == 0) {
            publishCameraInfo(K, rgb_width, rgb_height, depth_width, depth_height, timestamp);
            publishDeltaPose(camera_pose, timestamp);
        }

        // Warn if processing is too slow
        frame_count_++;
        auto frame_end_time = std::chrono::steady_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            frame_end_time - frame_start_time).count();
        
        if (processing_time > 50) {  // Warn if >50ms
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Frame processing slow: %ld ms", processing_time);
        }
    }

    void publishRGBCompressed(const Record3D::BufferRGB &rgb_frame, 
                              uint32_t width, uint32_t height,
                              rclcpp::Time timestamp)
    {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = timestamp;
        msg->header.frame_id = frame_id_;
        msg->format = "jpeg";
        
        // Create OpenCV Mat from RGB data (no copy, just wrap)
        cv::Mat rgb_image(height, width, CV_8UC3, (void*)rgb_frame.data());
        
        // Convert RGB to BGR for OpenCV
        cv::Mat bgr_image;
        cv::cvtColor(rgb_image, bgr_image, cv::COLOR_RGB2BGR);
        
        // JPEG encode with quality parameter (90 = high quality, good compression)
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
        cv::imencode(".jpg", bgr_image, msg->data, params);
        
        rgb_compressed_pub_->publish(std::move(msg));
    }

    void publishTF(const Record3D::CameraPose &pose, rclcpp::Time timestamp)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = timestamp;
        transform.header.frame_id = world_frame_;
        transform.child_frame_id = frame_id_;
        
        if (apply_transform_) {
            // iPhone uses Y-up coordinate system (ARKit convention)
            // ROS uses Z-up coordinate system
            // Transform to device-centric frame (Z always points up)
            
            // Original iPhone pose quaternion
            tf2::Quaternion iphone_orientation(pose.qx, pose.qy, pose.qz, pose.qw);
            
            // Rotation to convert Y-up to Z-up: -90° around X axis
            // This aligns iPhone Y-axis (up) with ROS Z-axis (up)
            // Device frame: Z always points up, regardless of device orientation
            tf2::Quaternion frame_correction;
            frame_correction.setRPY(-M_PI_2, 0, 0);  // Only roll -90°
            
            // Apply frame correction (pre-multiply to rotate the frame)
            tf2::Quaternion ros_orientation = frame_correction * iphone_orientation;
            ros_orientation.normalize();
            
            // Transform position: iPhone (X, Y, Z) -> ROS (X, Z, -Y)
            transform.transform.translation.x = pose.tx;
            transform.transform.translation.y = -pose.tz;  // iPhone Z -> ROS -Y
            transform.transform.translation.z = pose.ty;   // iPhone Y -> ROS Z
            
            transform.transform.rotation.x = ros_orientation.x();
            transform.transform.rotation.y = ros_orientation.y();
            transform.transform.rotation.z = ros_orientation.z();
            transform.transform.rotation.w = ros_orientation.w();
        } else {
            // No transformation - use raw iPhone pose
            transform.transform.translation.x = pose.tx;
            transform.transform.translation.y = pose.ty;
            transform.transform.translation.z = pose.tz;
            
            transform.transform.rotation.x = pose.qx;
            transform.transform.rotation.y = pose.qy;
            transform.transform.rotation.z = pose.qz;
            transform.transform.rotation.w = pose.qw;
        }
        
        tf_broadcaster_->sendTransform(transform);
    }

    void publishDepthImage(const Record3D::BufferDepth &depth_frame,
                          uint32_t width, uint32_t height,
                          rclcpp::Time timestamp)
    {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = timestamp;
        msg->header.frame_id = frame_id_;
        msg->height = height;
        msg->width = width;
        msg->encoding = "32FC1";  // 32-bit float, single channel
        msg->is_bigendian = false;
        msg->step = width * sizeof(float);
        
        // Use resize + memcpy for performance
        size_t data_size = width * height * sizeof(float);
        msg->data.resize(data_size);
        std::memcpy(msg->data.data(), depth_frame.data(), data_size);
        
        depth_pub_->publish(std::move(msg));
    }

    void publishConfidenceImage(const Record3D::BufferConfidence &conf_frame,
                                uint32_t width, uint32_t height,
                                rclcpp::Time timestamp)
    {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = timestamp;
        msg->header.frame_id = frame_id_;
        msg->height = height;
        msg->width = width;
        msg->encoding = "8UC1";  // 8-bit unsigned, single channel
        msg->is_bigendian = false;
        msg->step = width;
        msg->data.assign(conf_frame.begin(), conf_frame.begin() + width * height);
        
        confidence_pub_->publish(std::move(msg));
    }

    void publishCameraInfo(Record3D::IntrinsicMatrixCoeffs K,
                          uint32_t rgb_width, uint32_t rgb_height,
                          uint32_t depth_width, uint32_t depth_height,
                          rclcpp::Time timestamp)
    {
        // Publish RGB camera info
        auto rgb_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
        rgb_info->header.stamp = timestamp;
        rgb_info->header.frame_id = frame_id_;
        rgb_info->width = rgb_width;
        rgb_info->height = rgb_height;
        rgb_info->distortion_model = "plumb_bob";
        rgb_info->d = {0.0, 0.0, 0.0, 0.0, 0.0};  // No distortion
        rgb_info->k = {K.fx, 0.0, K.tx,
                       0.0, K.fy, K.ty,
                       0.0, 0.0, 1.0};
        rgb_info->r = {1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0};
        rgb_info->p = {K.fx, 0.0, K.tx, 0.0,
                       0.0, K.fy, K.ty, 0.0,
                       0.0, 0.0, 1.0, 0.0};
        rgb_info_pub_->publish(std::move(rgb_info));

        // Publish depth camera info (same intrinsics for aligned depth)
        auto depth_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
        depth_info->header.stamp = timestamp;
        depth_info->header.frame_id = frame_id_;
        depth_info->width = depth_width;
        depth_info->height = depth_height;
        depth_info->distortion_model = "plumb_bob";
        depth_info->d = {0.0, 0.0, 0.0, 0.0, 0.0};
        depth_info->k = {K.fx, 0.0, K.tx,
                        0.0, K.fy, K.ty,
                        0.0, 0.0, 1.0};
        depth_info->r = {1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0};
        depth_info->p = {K.fx, 0.0, K.tx, 0.0,
                        0.0, K.fy, K.ty, 0.0,
                        0.0, 0.0, 1.0, 0.0};
        depth_info_pub_->publish(std::move(depth_info));
    }

    void publishCameraPose(Record3D::CameraPose camera_pose, rclcpp::Time timestamp)
    {
        auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        msg->header.stamp = timestamp;
        msg->header.frame_id = "world";
        
        msg->pose.position.x = camera_pose.tx;
        msg->pose.position.y = camera_pose.ty;
        msg->pose.position.z = camera_pose.tz;
        
        msg->pose.orientation.x = camera_pose.qx;
        msg->pose.orientation.y = camera_pose.qy;
        msg->pose.orientation.z = camera_pose.qz;
        msg->pose.orientation.w = camera_pose.qw;
        
        pose_pub_->publish(std::move(msg));
    }

    void publishDeltaPose(Record3D::CameraPose camera_pose, rclcpp::Time timestamp)
    {
        // Initialize the initial pose on first frame
        if (!initial_pose_set_) {
            initial_pose_.tx = camera_pose.tx;
            initial_pose_.ty = camera_pose.ty;
            initial_pose_.tz = camera_pose.tz;
            initial_pose_.qx = camera_pose.qx;
            initial_pose_.qy = camera_pose.qy;
            initial_pose_.qz = camera_pose.qz;
            initial_pose_.qw = camera_pose.qw;
            initial_pose_set_ = true;
            
            // Delta pose is identity for the first frame
            auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
            msg->header.stamp = timestamp;
            msg->header.frame_id = "device_" + std::to_string(device_index_) + "_initial";
            msg->pose.position.x = 0.0;
            msg->pose.position.y = 0.0;
            msg->pose.position.z = 0.0;
            msg->pose.orientation.x = 0.0;
            msg->pose.orientation.y = 0.0;
            msg->pose.orientation.z = 0.0;
            msg->pose.orientation.w = 1.0;
            pose_delta_pub_->publish(std::move(msg));
            return;
        }

        // Compute delta pose: T_delta = T_initial^-1 * T_current
        tf2::Transform initial_transform;
        initial_transform.setOrigin(tf2::Vector3(initial_pose_.tx, initial_pose_.ty, initial_pose_.tz));
        initial_transform.setRotation(tf2::Quaternion(initial_pose_.qx, initial_pose_.qy, 
                                                      initial_pose_.qz, initial_pose_.qw));

        tf2::Transform current_transform;
        current_transform.setOrigin(tf2::Vector3(camera_pose.tx, camera_pose.ty, camera_pose.tz));
        current_transform.setRotation(tf2::Quaternion(camera_pose.qx, camera_pose.qy,
                                                      camera_pose.qz, camera_pose.qw));

        tf2::Transform delta_transform = initial_transform.inverse() * current_transform;

        auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        msg->header.stamp = timestamp;
        msg->header.frame_id = "device_" + std::to_string(device_index_) + "_initial";
        
        msg->pose.position.x = delta_transform.getOrigin().x();
        msg->pose.position.y = delta_transform.getOrigin().y();
        msg->pose.position.z = delta_transform.getOrigin().z();
        
        msg->pose.orientation.x = delta_transform.getRotation().x();
        msg->pose.orientation.y = delta_transform.getRotation().y();
        msg->pose.orientation.z = delta_transform.getRotation().z();
        msg->pose.orientation.w = delta_transform.getRotation().w();
        
        pose_delta_pub_->publish(std::move(msg));
    }

    // Member variables
    int device_index_;
    std::string frame_id_;
    std::string world_frame_;
    std::string topic_prefix_;
    bool apply_transform_;
    double connection_retry_interval_;
    
    std::unique_ptr<Record3D::Record3DStream> stream_;
    rclcpp::TimerBase::SharedPtr connection_timer_;
    
    bool connected_ = false;
    bool connection_warning_shown_ = false;
    bool publishers_created_ = false;
    bool first_frame_received_ = false;
    size_t last_device_count_ = 0;
    
    std::mutex publisher_mutex_;
    
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr confidence_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_delta_pub_;
    
    bool initial_pose_set_ = false;
    Record3D::CameraPose initial_pose_;
    
    uint64_t frame_count_ = 0;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<Record3DNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
