/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    this->declare_parameter("headless", false); // Enable headless mode (no GUI)
    this->declare_parameter("use_camera", false); // Use camera input instead of python driver
    this->declare_parameter("camera_topic", "camera/mono"); // Camera image topic name  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";
    headlessMode = false;
    useCameraInput = false;
    cameraImgTopicName = "camera/mono";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();
    rclcpp::Parameter param4 = this->get_parameter("headless");
    headlessMode = param4.as_bool();
    rclcpp::Parameter param5 = this->get_parameter("use_camera");
    useCameraInput = param5.as_bool();
    rclcpp::Parameter param6 = this->get_parameter("camera_topic");
    cameraImgTopicName = param6.as_string();
    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "headless mode %s", headlessMode ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "use camera input %s", useCameraInput ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(), "camera topic %s", cameraImgTopicName.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages

    //* Create map visualization publishers
    mapPoints_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_points", 10);
    cameraPose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera_pose", 10);
    keyframePath_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/keyframe_path", 10);
    trackingState_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/tracking_state", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    //* Initialize path message
    keyframe_path_.header.frame_id = "map";
    last_publish_time_ = this->get_clock()->now();
    //* Setup input subscriptions based on mode
    if (useCameraInput) {
        // Camera mode: subscribe to camera topic directly
        cameraImg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            cameraImgTopicName, 1, std::bind(&MonocularMode::cameraImg_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Camera mode: subscribing to %s", cameraImgTopicName.c_str());
        
        // Initialize SLAM with default configuration for camera mode
        std::string defaultConfig = "EuRoC"; // Default configuration
        initializeVSLAM(defaultConfig);
    } else {
        // Python driver mode: subscribe to python driver topics
        expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(
            subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));
        subImgMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));
        subTimestepMsg_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Python driver mode: waiting for handshake...");
    }
    
    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);
    
    RCLCPP_INFO(this->get_logger(), "MonocularMode node initialization completed");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    
    // Set GUI options based on headless mode
    if (headlessMode) {
        enablePangolinWindow = false; // Disable Pangolin window in headless mode
        enableOpenCVWindow = false; // Disable OpenCV window in headless mode
        RCLCPP_INFO(this->get_logger(), "Running in headless mode - GUI disabled");
    } else {
        enablePangolinWindow = true; // Shows Pangolin window output
        enableOpenCVWindow = true; // Shows OpenCV window output
        RCLCPP_INFO(this->get_logger(), "Running with GUI enabled");
    }
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use
    
    //* Publish map visualization data
    if (pAgent != nullptr) {
        publishTrackingState();
        publishCameraPose(Tcw);
        publishTF(Tcw);
        
        // Publish map data at reduced frequency (e.g., every 1 second)
        auto current_time = this->get_clock()->now();
        if ((current_time - last_publish_time_).seconds() >= 1.0) {
            publishMapPoints();
            publishKeyframePath();
            last_publish_time_ = current_time;
        }
    }

}

//* Callback to process camera images directly (bypasses Python driver)
void MonocularMode::cameraImg_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr;
    
    //* Convert ROS image to openCV image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading camera image: %s", e.what());
        return;
    }
    
    //* Use timestamp from image message or generate one
    double timestamp;
    if (useTimestamp) {
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    } else {
        timestamp = this->get_clock()->now().seconds();
    }
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timestamp);
    
    //* Publish map visualization data
    if (pAgent != nullptr) {
        publishTrackingState();
        publishCameraPose(Tcw);
        publishTF(Tcw);
        
        // Publish map data at reduced frequency (e.g., every 1 second)
        auto current_time = this->get_clock()->now();
        if ((current_time - last_publish_time_).seconds() >= 1.0) {
            publishMapPoints();
            publishKeyframePath();
            last_publish_time_ = current_time;
        }
    }
}

//* Map visualization helper methods
void MonocularMode::publishMapPoints()
{
    if (!pAgent) return;
    
    std::vector<ORB_SLAM3::MapPoint*> mapPoints = pAgent->GetMap()->GetAllMapPoints();
    
    if (mapPoints.empty()) return;
    
    sensor_msgs::msg::PointCloud2 pointcloud_msg = createPointCloud2(mapPoints);
    pointcloud_msg.header.stamp = this->get_clock()->now();
    pointcloud_msg.header.frame_id = "map";
    
    mapPoints_publisher_->publish(pointcloud_msg);
}

void MonocularMode::publishCameraPose(const Sophus::SE3f& Tcw)
{
    if (!pAgent) return;
    
    // Convert camera pose to world pose
    Sophus::SE3f Twc = Tcw.inverse();
    
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "map";
    
    // Extract translation
    Eigen::Vector3f translation = Twc.translation();
    pose_msg.pose.position.x = translation.x();
    pose_msg.pose.position.y = translation.y();
    pose_msg.pose.position.z = translation.z();
    
    // Extract rotation (quaternion)
    Eigen::Quaternionf quaternion = Twc.unit_quaternion();
    pose_msg.pose.orientation.x = quaternion.x();
    pose_msg.pose.orientation.y = quaternion.y();
    pose_msg.pose.orientation.z = quaternion.z();
    pose_msg.pose.orientation.w = quaternion.w();
    
    cameraPose_publisher_->publish(pose_msg);
}

void MonocularMode::publishKeyframePath()
{
    if (!pAgent) return;
    
    std::vector<ORB_SLAM3::KeyFrame*> keyframes = pAgent->GetMap()->GetAllKeyFrames();
    
    if (keyframes.empty()) return;
    
    // Sort keyframes by timestamp
    std::sort(keyframes.begin(), keyframes.end(), 
              [](ORB_SLAM3::KeyFrame* a, ORB_SLAM3::KeyFrame* b) {
                  return a->mTimeStamp < b->mTimeStamp;
              });
    
    keyframe_path_.poses.clear();
    keyframe_path_.header.stamp = this->get_clock()->now();
    keyframe_path_.header.frame_id = "map";
    
    for (ORB_SLAM3::KeyFrame* kf : keyframes) {
        if (kf->isBad()) continue;
        
        Sophus::SE3f Twc = kf->GetPoseInverse();
        
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->get_clock()->now();
        pose_stamped.header.frame_id = "map";
        
        // Extract translation
        Eigen::Vector3f translation = Twc.translation();
        pose_stamped.pose.position.x = translation.x();
        pose_stamped.pose.position.y = translation.y();
        pose_stamped.pose.position.z = translation.z();
        
        // Extract rotation (quaternion)
        Eigen::Quaternionf quaternion = Twc.unit_quaternion();
        pose_stamped.pose.orientation.x = quaternion.x();
        pose_stamped.pose.orientation.y = quaternion.y();
        pose_stamped.pose.orientation.z = quaternion.z();
        pose_stamped.pose.orientation.w = quaternion.w();
        
        keyframe_path_.poses.push_back(pose_stamped);
    }
    
    keyframePath_publisher_->publish(keyframe_path_);
}

void MonocularMode::publishTrackingState()
{
    if (!pAgent) return;
    
    std_msgs::msg::Int32 tracking_state_msg;
    tracking_state_msg.data = static_cast<int32_t>(pAgent->GetTrackingState());
    
    trackingState_publisher_->publish(tracking_state_msg);
}

void MonocularMode::publishTF(const Sophus::SE3f& Tcw)
{
    if (!pAgent) return;
    
    // Convert camera pose to world pose
    Sophus::SE3f Twc = Tcw.inverse();
    
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "camera_link";
    
    // Extract translation
    Eigen::Vector3f translation = Twc.translation();
    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    
    // Extract rotation (quaternion)
    Eigen::Quaternionf quaternion = Twc.unit_quaternion();
    transform_stamped.transform.rotation.x = quaternion.x();
    transform_stamped.transform.rotation.y = quaternion.y();
    transform_stamped.transform.rotation.z = quaternion.z();
    transform_stamped.transform.rotation.w = quaternion.w();
    
    tf_broadcaster_->sendTransform(transform_stamped);
}

sensor_msgs::msg::PointCloud2 MonocularMode::createPointCloud2(const std::vector<ORB_SLAM3::MapPoint*>& mapPoints)
{
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    
    // Set up the PointCloud2 message
    pointcloud_msg.height = 1;
    pointcloud_msg.width = mapPoints.size();
    pointcloud_msg.is_dense = true;
    
    // Set up the fields
    sensor_msgs::msg::PointField field_x, field_y, field_z;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;
    
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;
    
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;
    
    pointcloud_msg.fields.push_back(field_x);
    pointcloud_msg.fields.push_back(field_y);
    pointcloud_msg.fields.push_back(field_z);
    
    pointcloud_msg.point_step = 12; // 3 floats * 4 bytes
    pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width;
    
    // Resize the data array
    pointcloud_msg.data.resize(pointcloud_msg.row_step);
    
    // Fill the data
    float* data_ptr = reinterpret_cast<float*>(&pointcloud_msg.data[0]);
    
    for (size_t i = 0; i < mapPoints.size(); ++i) {
        ORB_SLAM3::MapPoint* mp = mapPoints[i];
        if (mp && !mp->isBad()) {
            Eigen::Vector3f pos = mp->GetWorldPos();
            data_ptr[i * 3 + 0] = pos.x();
            data_ptr[i * 3 + 1] = pos.y();
            data_ptr[i * 3 + 2] = pos.z();
        }
    }
    
    return pointcloud_msg;
}


