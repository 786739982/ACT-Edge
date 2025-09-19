#include "act_infer/act_infer_app.hpp"

using namespace std::chrono_literals;

Act_Infer_APP::Act_Infer_APP()
    : rclcpp_lifecycle::LifecycleNode("act_infer_app") {
    RCLCPP_INFO(get_logger(), "Lifecycle Node Constructor");

    this->declare_parameter<std::string>("app_name", "ActInfer");
    this->declare_parameter<std::string>("config_path", "/home/sunrise/act_config");
    this->declare_parameter<bool>("readonly", true);
    this->declare_parameter<std::vector<double>>("init_joint_pos", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<std::string>>("activated_controllers", {"controller1", "controller2"});
    this->declare_parameter<std::vector<std::string>>("activated_hardwares", {"hardware1", "hardware2"});
    this->declare_parameter<std::string>("namespace", "hongrui");
    this->declare_parameter<bool>("temporal_agg", true);
    this->declare_parameter<int>("chunk_size", 40);
    this->declare_parameter<int>("max_timesteps", 220);
    this->declare_parameter<int>("action_dim", 7);


    imageCache = std::vector<std::vector<uint8_t>>(2, std::vector<uint8_t>(640 * 480 * 3, 0));
}

CallbackReturn Act_Infer_APP::on_configure(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Configuring...");

    this->get_parameter("app_name", app_name_);
    this->get_parameter("config_path", config_path_);
    this->get_parameter("readonly", readonly_);
    this->get_parameter("init_joint_pos", init_joint_pos_);
    this->get_parameter("activated_controllers", activated_controllers_);
    this->get_parameter("activated_hardwares", activated_hardwares_);
    this->get_parameter("namespace", namespace_);
    this->get_parameter("temporal_agg", temporal_agg_);
    this->get_parameter("chunk_size", chunk_size_);
    this->get_parameter("max_timesteps", max_timesteps_);
    this->get_parameter("action_dim", action_dim_);

    query_period_ = temporal_agg_ ? 1 : chunk_size_;
    std::cout << "Init--Query period: " << query_period_ << std::endl;
    std::cout << "Init--Chunk size: " << chunk_size_ << std::endl;
    std::cout << "Init--Max timesteps: " << max_timesteps_ << std::endl;
    std::cout << "Init--Action dim: " << action_dim_ << std::endl;
    std::cout << "Init--Temporal agg: " << temporal_agg_ << std::endl;
    all_time_actions_ = create_3d_vector(max_timesteps_, max_timesteps_ + chunk_size_, action_dim_);
    raw_actions = create_2d_vector(chunk_size_, action_dim_);

    RCLCPP_INFO(get_logger(), "Configuration Done. App Name: %s, Config Path: %s",
                app_name_.c_str(), config_path_.c_str());
                
    loadModel(std::string("/home/sunrise/act_config/act.bin"));

    return CallbackReturn::SUCCESS;
}

CallbackReturn Act_Infer_APP::on_activate(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Activating Node...");

    joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        fmt::format("/{}/joint_states", namespace_), 10, std::bind(&Act_Infer_APP::joints_callback, this, std::placeholders::_1));
    
    joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(fmt::format("/{}/servo_node/joint_pos_cmds", namespace_), 10);
    
    gripper_publisher_ = this->create_publisher<airbot_controller_msgs::msg::GripperControl>(fmt::format("/{}/gripper_control", namespace_), 10);
  
    
    auto callback_1 = [this](std::shared_ptr<sensor_msgs::msg::Image> msg) {
            this->images_callback(msg, 0);  // 传递 0 作为第二个参数
        };
    cam1_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/usbcam1/color/image_bgr8", 10, callback_1);
    auto callback_2 = [this](std::shared_ptr<sensor_msgs::msg::Image> msg) {
            this->images_callback(msg, 1);  // 传递 1 作为第二个参数
        };
    cam2_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/usbcam2/color/image_bgr8", 10, callback_2);

    running_thread_ = std::thread(&Act_Infer_APP::run, this);
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn Act_Infer_APP::on_deactivate(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Deactivating Node...");
    return CallbackReturn::SUCCESS;
}

CallbackReturn Act_Infer_APP::on_cleanup(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    return CallbackReturn::SUCCESS;
}

CallbackReturn Act_Infer_APP::on_shutdown(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    return CallbackReturn::SUCCESS;
}

void Act_Infer_APP::images_callback(const sensor_msgs::msg::Image::SharedPtr msg, int id) {
    // RCLCPP_INFO(this->get_logger(), "Id %d Received image at time: %d.%09d", id,
    //             msg->header.stamp.sec, msg->header.stamp.nanosec);
    // RCLCPP_INFO(this->get_logger(), "Image height: %d, width: %d", msg->height, msg->width);
    // RCLCPP_INFO(this->get_logger(), "Image encoding: %s", msg->encoding.c_str());
    // RCLCPP_INFO(this->get_logger(), "Is big endian: %d", msg->is_bigendian);
    // RCLCPP_INFO(this->get_logger(), "Step (bytes per row): %d", msg->step);
    // RCLCPP_INFO(this->get_logger(), "Image data length: %zu", msg->data.size());
    imageCache[id] = std::vector<uint8_t>(msg->data.begin(), msg->data.end());

}

void Act_Infer_APP::joints_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    jointCache.clear();
    for (int i = 1; i <= 6; ++i) {
        std::string joint_name = "joint" + std::to_string(i);
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
        if (it != msg->name.end()) {
            size_t index = std::distance(msg->name.begin(), it);
            jointCache.push_back(msg->position[index]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Joint %s not found in JointState message", joint_name.c_str());
            jointCache.push_back(0.0); // 默认值
        }
    }
    // std::cout << "Rece Joints: ";
    // for (auto &joint : jointCache) {
    //     std::cout << joint << std::endl;
    // }
    // std::cout << std::endl;

    auto it = std::find(msg->name.begin(), msg->name.end(), "gripper_mapping_controller");
    if (it != msg->name.end()) {
        size_t index = std::distance(msg->name.begin(), it);
        jointCache.push_back(msg->position[index]);
    } else {
        RCLCPP_WARN(this->get_logger(), "Gripper not found in JointState message");
        jointCache.push_back(0.0); // 默认值
    }

}

void Act_Infer_APP::publish_joints(const std::vector<double>& joints) {
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    joint_msg.position = joints;
    // TODO: Add velocity and effort
    joint_msg.velocity = {};
    joint_msg.effort = {};
    joint_publisher_->publish(joint_msg);

    auto gripper_msg = airbot_controller_msgs::msg::GripperControl();
    gripper_msg.name.push_back("eef_gripper_joint");
    gripper_msg.pos.push_back(joints[6]);
    gripper_publisher_->publish(gripper_msg);
}

std::vector<std::vector<std::vector<double>>> Act_Infer_APP::create_3d_vector(int dim1, int dim2, int dim3, double init_value) {
    return std::vector<std::vector<std::vector<double>>>(dim1, std::vector<std::vector<double>>(dim2, std::vector<double>(dim3, init_value)));
}

std::vector<std::vector<double>> Act_Infer_APP::create_2d_vector(int dim1, int dim2, double init_value) {
    return std::vector<std::vector<double>>(dim1, std::vector<double>(dim2, init_value));
}

void Act_Infer_APP::run()
{
    bool is_approch_init_position = false;
    RCLCPP_INFO(this->get_logger(), "Waiting to approch init position");
    while (!is_approch_init_position && rclcpp::ok())
    {   
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        publish_joints(init_joints);
        std::vector<double> joint_copy = jointCache;
        // std::cout << "JointCache: " <<std::endl;
        // for (int i=0; i<7; i++)
        //     std::cout << jointCache[i] << " ";
        // std::cout << std::endl;

        is_approch_init_position = compareDoubleArrays(init_joints, joint_copy);
    }

    while(!infer(this) && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool Act_Infer_APP::infer(Act_Infer_APP* self)
{
    for (auto& image : imageCache) {
        if (image.empty()) {
            // RCLCPP_WARN(this->get_logger(), "Image empty");
            return infer_done;
        }
    }
    if (jointCache.empty()) {
        // RCLCPP_WARN(this->get_logger(), "Joint empty");
        return infer_done;
    }

    if (time_step_ < max_timesteps_) {
        
        if (time_step_ % query_period_ == 0) {
            auto start_time = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(self->mtx);
                deepcp_jointCache = jointCache;
                deepcp_imageCache = imageCache;
            }
            //TODO: Dynamic define image number
            cv::Mat image_1 = cv::Mat(480, 640, CV_8UC3, const_cast<uchar*>(deepcp_imageCache[0].data()));
            cv::Mat image_2 = cv::Mat(480, 640, CV_8UC3, const_cast<uchar*>(deepcp_imageCache[1].data()));
            std::cout << "Infer Begin" << std::endl;
            BGR2BGRTensor(
                image_1, 
                480,   // TODO: Get image height from input_tensors infos
                640, 
                (float*)self->input_tensors[0].sysMem[0].virAddr
                );
            BGR2BGRTensor(
                image_2, 
                480, // TODO: Get image height from input_tensors infos
                640, 
                (float*)self->input_tensors[1].sysMem[0].virAddr
                );
            
            std::cout << "Receive Joints: " ;
            for (int i = 0; i < deepcp_jointCache.size(); i++) {
                std::cout << deepcp_jointCache[i] << " ";
            }
            std::cout << std::endl;
            std::vector<double> current_joints = pre_process(deepcp_jointCache, qpos_mean, qpos_std);
            // std::cout << "Vector Double Joints: ";
            // for (double joint : current_joints) {
            //     std::cout << joint << " ";
            // }
            // std::cout << std::endl;
            std::vector<float> joints_float(current_joints.size());
            std::transform(current_joints.begin(), current_joints.end(), joints_float.begin(),
                [](double d) -> float {
                    return static_cast<float>(d);
                });
            
            // std::cout << "Vector Float Joints: ";
            // for (float joint : joints_float) {
            //     std::cout << joint << " ";
            // }
            // std::cout << std::endl;

            GetJointTensor(
                joints_float, 
                self->input_tensors[2].properties, 
                (float*)self->input_tensors[2].sysMem[0].virAddr
                );
            
            // Flush input tensors
            for (size_t i = 0; i < self->input_tensors.size(); i++) {
                hbSysFlushMem(&self->input_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
            }
            std::cout << "Flush Done!" << std::endl;
            hbDNNInferCtrlParam infer_ctrl_param;
            HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
            auto output = self->output_tensors.data();
            self->task_handle = nullptr;
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            std::cout << "Preprocess time: " << duration_ms << " ms" << std::endl;
            
            std::cout << "Infer ing!" << std::endl;
            start_time = std::chrono::high_resolution_clock::now();
            hbDNNInfer(&self->task_handle,
                &output,
                self->input_tensors.data(),
                self->dnn_handle,
                &infer_ctrl_param);
            hbDNNWaitTaskDone(self->task_handle, 0);
            end_time = std::chrono::high_resolution_clock::now();
            duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            std::cout << "Infer time: " << duration_ms << " ms" << std::endl;

            for (int i = 0; i < self->output_tensors.size(); i++) {
                hbSysFlushMem(&self->output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
            }
            start_time = std::chrono::high_resolution_clock::now();
            auto *data = reinterpret_cast<float *>(output_tensors[0].sysMem[0].virAddr);
            for (int n = 0; n < chunk_size_; ++n) {
                for (int i = 0; i < action_dim_; ++i) {
                    raw_actions[n][i] = data[i+n*7];
                }
                // Populate all_time_actions_ at timestep time_step_ with raw_actions
                all_time_actions_[time_step_][time_step_ + n] = raw_actions[n];
            }
            hbDNNReleaseTask(self->task_handle);
            
        }
        
        std::vector<double> new_action(action_dim_, 0.0);
        std::vector<double> new_velocities(action_dim_, 0.3);

        if (temporal_agg_) {
            new_action = smooth_joints(all_time_actions_, time_step_, chunk_size_, action_dim_);
        }
        else {
            new_action = raw_actions[time_step_ % query_period_];
        }

        // Denormalize Joints
        new_action = post_process(new_action, action_mean, action_std);
        // Publish the joints reuslt of inference
        publish_joints(new_action);
        for (auto& image : imageCache) {
            image.clear();
        }
        jointCache.clear();
        time_step_++;
        if (time_step_ == max_timesteps_) {
            std::cout << "Inference finished" << std::endl;
            infer_done = true;
            return infer_done;
        } else {
            return infer_done;
        }
    }
    return infer_done;
}

void Act_Infer_APP::loadModel(std::string model_path)
{
    /*
     * Note: For now, the method only supports loading 1 model
     */

    // TODO : get cameras info from input_tensors infos to dynamicly init 
    const char** model_name_list;
    int model_count = 0;
    auto model_path_cstr = model_path.c_str();

    int32_t ret = 0;
    ret = hbDNNInitializeFromFiles(&packed_dnn_handle, &model_path_cstr, 1);
    ret = hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle);
    ret = hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]);
    // ret = hbDNNRelease(packed_dnn_handle);

    int input_count = 0, output_count = 0;
    hbDNNGetInputCount(&input_count, dnn_handle);
    input_tensors.resize(input_count);
    hbDNNGetOutputCount(&output_count, dnn_handle);
    output_tensors.resize(output_count);

    for (size_t i = 0; i < input_count; i++) {
        hbDNNGetInputTensorProperties(&input_tensors[i].properties, dnn_handle, i);
        int input_memSize = input_tensors[i].properties.alignedByteSize;
        hbSysAllocCachedMem(&input_tensors[i].sysMem[0], input_memSize);

        /** Tips:
         * For input tensor, aligned shape should always be equal to the real
         * shape of the user's data. If you are going to set your input data with
         * padding, this step is not necessary.
         * */
        input_tensors[i].properties.alignedShape = input_tensors[i].properties.validShape;

        const char* input_name;
        hbDNNGetInputName(&input_name, dnn_handle, i);
        std::cerr << "input[" << i << "] name is " << input_name << std::endl;
    }

    for (size_t i = 0; i < output_count; i++) {
        hbDNNGetOutputTensorProperties(&output_tensors[i].properties, dnn_handle, i);
        int output_memSize = output_tensors[i].properties.alignedByteSize;
        hbSysAllocCachedMem(&output_tensors[i].sysMem[0], output_memSize);

        const char* output_name;
        hbDNNGetOutputName(&output_name, dnn_handle, i);
        std::cerr << "output[" << i << "] name is " << output_name << std::endl;
    }
}

void Act_Infer_APP::unloadModel()
{
    for (size_t i = 0; i < input_tensors.size(); i++) {
        hbSysFreeMem(&(input_tensors[i].sysMem[0]));
    }
    for (size_t i = 0; i < output_tensors.size(); i++) {
        hbSysFreeMem(&(output_tensors[i].sysMem[0]));
    }
    hbDNNRelease(packed_dnn_handle);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Act_Infer_APP>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    RCLCPP_INFO(node->get_logger(), "Spinning Lifecycle Node...");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

// RCLCPP_COMPONENTS_REGISTER_NODE(Act_Infer_APP)