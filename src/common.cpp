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
    packagePath = "ros2_ws/src/ros2_orb_slam3/"; // !HARDCODED, change it as necessary
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    this->declare_parameter("headless", false); // Enable headless mode (no GUI)  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";
    headlessMode = false;

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    rclcpp::Parameter param4 = this->get_parameter("headless");
    // headlessMode = param4.as_bool();
    headlessMode = true;

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
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* Map visualization publishers
    mapPoints_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/orb_slam3/map_points", 10);
    cameraPose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/camera_pose", 10);
    keyframePath_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/orb_slam3/keyframe_trajectory", 10);
    trackingState_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/orb_slam3/tracking_state", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //* Initialize path message
    keyframe_path_.header.frame_id = "map";
    last_publish_time_ = this->get_clock()->now();

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
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
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

}

//* Map visualization helper functions implementation

void MonocularMode::publishMapPoints() {
    if (pAgent == nullptr) return;
    
    std::vector<ORB_SLAM3::MapPoint*> mapPoints = pAgent->GetAllMapPoints();
    if (mapPoints.empty()) return;
    
    sensor_msgs::msg::PointCloud2 cloud_msg = createPointCloud2(mapPoints);
    cloud_msg.header.stamp = this->get_clock()->now();
    cloud_msg.header.frame_id = "map";
    
    mapPoints_publisher_->publish(cloud_msg);
}

void MonocularMode::publishCameraPose(const Sophus::SE3f& Tcw) {
    if (pAgent == nullptr) return;
    
    // Convert camera pose to world coordinate frame
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

void MonocularMode::publishKeyframePath() {
    if (pAgent == nullptr) return;
    
    std::vector<Sophus::SE3f> keyframe_poses = pAgent->GetAllKeyframePoses();
    if (keyframe_poses.empty()) return;
    
    keyframe_path_.poses.clear();
    keyframe_path_.header.stamp = this->get_clock()->now();
    
    for (const auto& pose : keyframe_poses) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = this->get_clock()->now();
        
        // Extract translation
        Eigen::Vector3f translation = pose.translation();
        pose_stamped.pose.position.x = translation.x();
        pose_stamped.pose.position.y = translation.y();
        pose_stamped.pose.position.z = translation.z();
        
        // Extract rotation (quaternion)
        Eigen::Quaternionf quaternion = pose.unit_quaternion();
        pose_stamped.pose.orientation.x = quaternion.x();
        pose_stamped.pose.orientation.y = quaternion.y();
        pose_stamped.pose.orientation.z = quaternion.z();
        pose_stamped.pose.orientation.w = quaternion.w();
        
        keyframe_path_.poses.push_back(pose_stamped);
    }
    
    keyframePath_publisher_->publish(keyframe_path_);
}

void MonocularMode::publishTrackingState() {
    if (pAgent == nullptr) return;
    
    std_msgs::msg::Int32 state_msg;
    state_msg.data = pAgent->GetTrackingState();
    trackingState_publisher_->publish(state_msg);
}

void MonocularMode::publishTF(const Sophus::SE3f& Tcw) {
    if (pAgent == nullptr) return;
    
    // Convert camera pose to world coordinate frame
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

sensor_msgs::msg::PointCloud2 MonocularMode::createPointCloud2(const std::vector<ORB_SLAM3::MapPoint*>& mapPoints) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    
    // Set up the PointCloud2 message
    cloud_msg.height = 1;
    cloud_msg.width = 0;
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;
    
    // Define point cloud fields
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
    
    cloud_msg.fields = {field_x, field_y, field_z};
    cloud_msg.point_step = 12; // 3 floats * 4 bytes each
    
    // Count valid map points
    std::vector<Eigen::Vector3f> valid_points;
    for (ORB_SLAM3::MapPoint* pMP : mapPoints) {
        if (pMP && !pMP->isBad()) {
            Eigen::Vector3f pos = pMP->GetWorldPos();
            valid_points.push_back(pos);
        }
    }
    
    cloud_msg.width = valid_points.size();
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step);
    
    // Fill point data
    float* data_ptr = reinterpret_cast<float*>(cloud_msg.data.data());
    for (size_t i = 0; i < valid_points.size(); ++i) {
        data_ptr[i * 3 + 0] = valid_points[i].x();
        data_ptr[i * 3 + 1] = valid_points[i].y();
        data_ptr[i * 3 + 2] = valid_points[i].z();
    }
    
    return cloud_msg;
}


