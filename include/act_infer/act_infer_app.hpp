#ifndef ACT_INFER_APP_HPP
#define ACT_INFER_APP_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "airbot_controller_msgs/msg/gripper_control.hpp"

#include <vector>
#include <numeric>
#include <chrono>
#include <mutex>
#include <thread>
#include <queue>
#include <fmt/core.h>
#include <iostream>

#include "dnn/hb_dnn.h"
#include "utils/conversion.hpp"
#include "utils/postprocess.hpp"
#include "utils/preprocess.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Act_Infer_APP : public rclcpp_lifecycle::LifecycleNode {
public:
    Act_Infer_APP();

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

private:
    std::string namespace_;
    std::string app_name_;
    std::string config_path_;
    bool readonly_;
    std::vector<double> init_joint_pos_;
    std::vector<std::string> activated_controllers_;
    std::vector<std::string> activated_hardwares_;

    std::vector<double> jointCache, deepcp_jointCache;
    std::vector<std::vector<uint8_t>> imageCache, deepcp_imageCache;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam1_sub_, cam2_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
    rclcpp::Publisher<airbot_controller_msgs::msg::GripperControl>::SharedPtr gripper_publisher_;
    
    // rclcpp::Client<service_msgs::srv::ServoCommandType>::SharedPtr servo_mode_client_;

    /*
     * hbdnn inference
     */
    hbPackedDNNHandle_t packed_dnn_handle;
    hbDNNHandle_t dnn_handle;
    std::vector<hbDNNTensor> input_tensors;
    std::vector<hbDNNTensor> output_tensors;
    hbDNNTaskHandle_t task_handle = nullptr;
    
    int chunk_size_;
    int max_timesteps_;
    bool temporal_agg_;
    int action_dim_;
    int query_period_;
    std::vector<std::vector<double>> raw_actions;
    std::vector<std::vector<std::vector<double>>> all_time_actions_;
    int time_step_=0;
    bool infer_done = false;
    std::thread running_thread_;
    std::mutex mtx;
    std::vector<double> action_mean = {
        0.24963677, -0.65107054, 0.65631634, 1.6214081, -1.1599095, -1.1657934, 0.43080807
    };
    std::vector<double> action_std = {
        0.42494634, 0.21923615, 0.11953888, 0.08032797, 0.12769431, 0.29900876, 0.39818874
    };
    std::vector<double> qpos_mean = {
        0.24775764, -0.65712106, 0.6457556, 1.6196821, -1.1749103, -1.1686785, 0.44967297
    };
    std::vector<double> qpos_std = {
        0.42606193, 0.22065982, 0.11921931, 0.07671923, 0.12728566, 0.30039334, 0.32951903
    };
    std::vector<double> init_velocties = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
    std::vector<double> init_joints = {-0.26836803555488586, -0.33588922023773193, 0.648699164390564, 1.5287632942199707, -1.260585904121399, -1.5543221235275269};

    std::vector<std::vector<std::vector<double>>> create_3d_vector(int dim1, int dim2, int dim3, double init_value = 0.0);
    std::vector<std::vector<double>> create_2d_vector(int dim1, int dim2, double init_value = 0.0);

    void images_callback(const sensor_msgs::msg::Image::SharedPtr msg, int id);
    void joints_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publish_joints(const std::vector<double>& joints);
    void loadModel(std::string model_path);
    void unloadModel();
    void run();
    bool infer(Act_Infer_APP* self);


};

#endif // ACT_INFER_APP_HPP
