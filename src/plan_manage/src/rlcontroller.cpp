#include <ros/ros.h>
#include <torch/script.h> // LibTorch JIT模块
#include <vector>
#include <string>
#include <iostream>

// ROS消息类型
#include <geometry_msgs/Twist.h>    // 用于订阅 /cmd_vel
#include <nav_msgs/Odometry.h>      // 用于订阅 /odom
#include <sensor_msgs/JointState.h> // 如果需要订阅关节状态
#include <std_msgs/Float32MultiArray.h> // 用于发布actions (一个例子)

// TF2 用于坐标变换 (如果需要更复杂的变换)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// 辅助函数：将世界系速度转换为本体坐标系速度
geometry_msgs::Vector3 world_to_body_velocity(const geometry_msgs::Vector3& world_vel, const geometry_msgs::Quaternion& orientation) {
    tf2::Quaternion q_world_to_body;
    tf2::fromMsg(orientation, q_world_to_body);
    q_world_to_body = q_world_to_body.inverse(); // 从世界到本体的旋转

    tf2::Vector3 v_world(world_vel.x, world_vel.y, world_vel.z);
    tf2::Vector3 v_body = tf2::quatRotate(q_world_to_body, v_world);

    geometry_msgs::Vector3 body_vel;
    body_vel.x = v_body.x();
    body_vel.y = v_body.y();
    body_vel.z = v_body.z();
    return body_vel;
}

// 辅助函数：计算投影重力
std::vector<float> calculate_projected_gravity(const geometry_msgs::Quaternion& orientation, float gravity_magnitude = 9.81) {
    tf2::Quaternion q_world_to_body;
    tf2::fromMsg(orientation, q_world_to_body);
    q_world_to_body = q_world_to_body.inverse();

    tf2::Vector3 gravity_world(0, 0, -gravity_magnitude); // 世界坐标系下的重力向量
    tf2::Vector3 gravity_body = tf2::quatRotate(q_world_to_body, gravity_world);

    return {(float)gravity_body.x(), (float)gravity_body.y(), (float)gravity_body.z()};
}


class ControllerNode {
public:
    ControllerNode(ros::NodeHandle& nh) : nh_(nh), gen_(rd_()), dist_(-1.0, 1.0) { // dist_用于生成随机数，如果需要
        // 1. 加载 TorchScript 模型
        std::string model_path;
        nh_.param<std::string>("model_path", model_path, "model.pt"); // 从参数服务器获取模型路径
        try {
            module_ = torch::jit::load(model_path);
            module_.to(torch::kCUDA); // 如果有GPU并且模型训练在GPU上
            ROS_INFO("TorchScript model loaded successfully from: %s", model_path.c_str());
        } catch (const c10::Error& e) {
            ROS_FATAL("Error loading the model: %s", e.what());
            ros::shutdown();
            return;
        }

        // 2. 加载观测和动作的缩放参数 (从ROS参数服务器)
        // 这些参数必须与Python训练时的配置完全一致！
        load_scales_from_params();

        // 初始化上一时刻的观测和动作 (根据你的观测维度和动作维度)
        // 假设观测维度 num_observations_, 动作维度 num_actions_
        // 你需要从配置文件中获取 num_observations_ 和 num_actions_
        nh_.param<int>("num_observations", num_observations_, 24); // 示例值
        nh_.param<int>("num_actions", num_actions_, 2);         // 示例值

        last_actions_.resize(num_actions_, 0.0f);
        last_base_lin_vel_body_.resize(3, 0.0f);
        last_base_ang_vel_body_.resize(3, 0.0f);

        // 3. 初始化订阅者
        cmd_vel_sub_ = nh_.subscribe("/Dftpav_path", 1, &ControllerNode::cmdVelCallback, this);
        odom_sub_ = nh_.subscribe("/ugv/odometry", 1, &ControllerNode::odomCallback, this);
        joint_state_sub_ = nh_.subscribe("/motor_state", 1, &ControllerNode::jointStateCallback, this);

        // 4. 初始化发布者 (用于发布模型的原始actions或处理后的控制指令)
        actions_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/controller/actions", 1);
        // control_command_pub_ = nh_.advertise<YourCustomMotorMsg>("/motor_commands", 1); // 发布给电机驱动

        // 5. 设置定时器，用于固定频率执行模型推理和控制
        double control_frequency;
        nh_.param<double>("control_frequency", control_frequency, 25.0); // Hz (对应policy_dt)
        if (control_frequency > 0) {
            inference_timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency), &ControllerNode::inferenceCallback, this);
        }

        ROS_INFO("Controller node initialized. Model input dim: %d, output dim: %d", num_observations_, num_actions_);
    }

private:
    void load_scales_from_params() {
        // --- 观测缩放 obs_scales (这些值是你训练时obs_scales对象的属性) ---
        // command = [lin_vel_cmd_scale, ang_vel_cmd_scale]
        if (!nh_.getParam("obs_scales/command", obs_scales_command_) || obs_scales_command_.size() != 2) {
            ROS_ERROR("Failed to get 'obs_scales/command' or size is not 2. Using defaults.");
            obs_scales_command_ = {1.0f, 1.0f}; // 提供默认值
        }
        // lin_vel = [vx_scale, vy_scale, vz_scale]
        if (!nh_.getParam("obs_scales/lin_vel", obs_scales_lin_vel_) || obs_scales_lin_vel_.size() != 3) {
            ROS_ERROR("Failed to get 'obs_scales/lin_vel' or size is not 3. Using defaults.");
            obs_scales_lin_vel_ = {1.0f, 1.0f, 1.0f};
        }
        // ang_vel = [wx_scale, wy_scale, wz_scale]
        if (!nh_.getParam("obs_scales/ang_vel", obs_scales_ang_vel_) || obs_scales_ang_vel_.size() != 3) {
            ROS_ERROR("Failed to get 'obs_scales/ang_vel' or size is not 3. Using defaults.");
            obs_scales_ang_vel_ = {1.0f, 1.0f, 1.0f};
        }
        // dof_pos (scalar for steering)
        nh_.param<float>("obs_scales/dof_pos", obs_scales_dof_pos_, 1.0f);
        // dof_vel = [main_vel_scale, steering_vel_scale]
        if (!nh_.getParam("obs_scales/dof_vel", obs_scales_dof_vel_) || obs_scales_dof_vel_.size() != 2) {
            ROS_ERROR("Failed to get 'obs_scales/dof_vel' or size is not 2. Using defaults.");
            obs_scales_dof_vel_ = {1.0f, 1.0f};
        }
        // projected_gravity (scalar, e.g. 1.0/9.81)
        nh_.param<float>("obs_scales/projected_gravity", obs_scales_projected_gravity_, 1.0f);
        // actions (scalar or list, for previous actions observation)
        // 假设 previous_actions 的 obs_scale 是1.0，或者你需要从参数加载它
        obs_scales_actions_ = 1.0f; // 示例

        // --- 动作缩放 action_scale (这些值是你训练时 cfg.control 的属性) ---
        nh_.param<float>("action_scales/first_actionScale", first_action_scale_, 1.0f);
        nh_.param<float>("action_scales/second_actionScale", second_action_scale_, 1.0f);

        // --- 如果需要 default_dof_pos ---
        // 假设只有一个转向关节需要default_dof_pos
        nh_.param<float>("robot_params/default_dof_pos_steering", default_dof_pos_steering_, 0.0f);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        current_cmd_vel_ = *msg;
        cmd_vel_received_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom_ = *msg;
        odom_received_ = true;
    }

    // void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    //     // 你需要根据关节名称找到对应的索引并存储位置和速度
    //     // current_dof_pos_[0] = ...; current_dof_vel_[0] = ...; (主轴)
    //     // current_dof_pos_[1] = ...; current_dof_vel_[1] = ...; (副轴/转向)
    //     // current_dof_pos_steering_ = msg->position[steering_joint_idx_];
    //     // current_dof_vel_steering_ = msg->velocity[steering_joint_idx_];
    //     // current_dof_vel_main_ = msg->velocity[main_drive_joint_idx_];
    //     joint_states_received_ = true;
    // }

    void inferenceCallback(const ros::TimerEvent& event) {
        if (!cmd_vel_received_ || !odom_received_ /*|| !joint_states_received_ 如果你需要这个*/) {
            ROS_WARN_THROTTLE(1.0, "Waiting for all sensor data to be received...");
            return;
        }

        // 1. 准备观测向量 (std::vector<float>)
        // 顺序必须与训练时 obs_buf 的拼接顺序完全一致！
        std::vector<float> obs_vec;
        obs_vec.reserve(num_observations_);

        // a. commands (2个)
        obs_vec.push_back(current_cmd_vel_.linear.x * obs_scales_command_[0]);
        obs_vec.push_back(current_cmd_vel_.angular.z * obs_scales_command_[1]);

        // b. base_quat (4个) - 通常不直接乘以obs_scale，但模型可能期望归一化
        obs_vec.push_back(current_odom_.pose.pose.orientation.x);
        obs_vec.push_back(current_odom_.pose.pose.orientation.y);
        obs_vec.push_back(current_odom_.pose.pose.orientation.z);
        obs_vec.push_back(current_odom_.pose.pose.orientation.w);

        // c. base_lin_vel (本体坐标系, 3个)
        geometry_msgs::Vector3 body_lin_vel = world_to_body_velocity(current_odom_.twist.twist.linear, current_odom_.pose.pose.orientation);
        obs_vec.push_back(body_lin_vel.x * obs_scales_lin_vel_[0]);
        obs_vec.push_back(body_lin_vel.y * obs_scales_lin_vel_[1]);
        obs_vec.push_back(body_lin_vel.z * obs_scales_lin_vel_[2]);

        // d. base_ang_vel (本体坐标系, 3个)
        // Odom的twist.angular可能已经是本体坐标系(如imu_link)，也可能是世界坐标系下的角速度关于本体轴的分量
        // 假设 odom.twist.angular 就是本体角速度 (你需要确认这一点!)
        geometry_msgs::Vector3 body_ang_vel = current_odom_.twist.twist.angular; // 假设已经是本体角速度
        // 如果不是, 你可能需要像lin_vel那样转换:
        // geometry_msgs::Vector3 body_ang_vel = world_to_body_velocity(current_odom_.twist.twist.angular, current_odom_.pose.pose.orientation);
        obs_vec.push_back(body_ang_vel.x * obs_scales_ang_vel_[0]);
        obs_vec.push_back(body_ang_vel.y * obs_scales_ang_vel_[1]);
        obs_vec.push_back(body_ang_vel.z * obs_scales_ang_vel_[2]);

        // e. last_base_lin_vel (3个)
        obs_vec.push_back(last_base_lin_vel_body_[0] * obs_scales_lin_vel_[0]);
        obs_vec.push_back(last_base_lin_vel_body_[1] * obs_scales_lin_vel_[1]);
        obs_vec.push_back(last_base_lin_vel_body_[2] * obs_scales_lin_vel_[2]);

        // f. last_base_ang_vel (3个)
        obs_vec.push_back(last_base_ang_vel_body_[0] * obs_scales_ang_vel_[0]);
        obs_vec.push_back(last_base_ang_vel_body_[1] * obs_scales_ang_vel_[1]);
        obs_vec.push_back(last_base_ang_vel_body_[2] * obs_scales_ang_vel_[2]);

        // g. dof_pos (副轴电机角度, 1个) - 你需要从 /joint_states 获取这个值
        // float current_steering_dof_pos_ = ... ; // 从 joint_states_msg_ 获取
        // obs_vec.push_back((current_steering_dof_pos_ - default_dof_pos_steering_) * obs_scales_dof_pos_);
        // 临时用0填充，你需要替换
        obs_vec.push_back(0.0f * obs_scales_dof_pos_);


        // h. dof_vel (电机速度, 2个) - 你需要从 /joint_states 获取这些值
        // float current_main_drive_dof_vel_ = ...;
        // float current_steering_dof_vel_ = ...;
        // obs_vec.push_back(current_main_drive_dof_vel_ * obs_scales_dof_vel_[0]);
        // obs_vec.push_back(current_steering_dof_vel_ * obs_scales_dof_vel_[1]);
        // 临时用0填充，你需要替换
        obs_vec.push_back(0.0f * obs_scales_dof_vel_[0]);
        obs_vec.push_back(0.0f * obs_scales_dof_vel_[1]);


        // i. projected_gravity (3个)
        std::vector<float> proj_gravity = calculate_projected_gravity(current_odom_.pose.pose.orientation);
        obs_vec.push_back(proj_gravity[0] * obs_scales_projected_gravity_);
        obs_vec.push_back(proj_gravity[1] * obs_scales_projected_gravity_);
        obs_vec.push_back(proj_gravity[2] * obs_scales_projected_gravity_);

        // j. actions (上一时刻的动作, 2个)
        obs_vec.push_back(last_actions_[0] * obs_scales_actions_); // 假设 obs_scales_actions_ 是标量1.0
        obs_vec.push_back(last_actions_[1] * obs_scales_actions_);


        if (obs_vec.size() != num_observations_) {
            ROS_ERROR_STREAM_THROTTLE(1.0, "Observation vector size mismatch! Expected " << num_observations_ << " but got " << obs_vec.size());
            return;
        }

        // 2. 将观测向量转换为 Torch 张量
        at::Tensor obs_tensor = torch::from_blob(obs_vec.data(), {1, num_observations_}, torch::kFloat32);
        obs_tensor = obs_tensor.to(torch::kCUDA); // 如果模型在GPU上

        // 3. 模型推理
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(obs_tensor);
        at::Tensor output_actions_tensor;
        try {
            output_actions_tensor = module_.forward(inputs).toTensor();
            output_actions_tensor = output_actions_tensor.to(torch::kCPU); // 转回CPU处理
        } catch (const c10::Error& e) {
            ROS_ERROR("Error during model inference: %s", e.what());
            return;
        }


        // 4. 提取原始动作 (通常在 [-1, 1] 范围)
        std::vector<float> raw_actions(num_actions_);
        for (int i = 0; i < num_actions_; ++i) {
            raw_actions[i] = output_actions_tensor[0][i].item<float>();
        }

        // 更新 last_actions (用的是原始的、未乘以action_scale的actions)
        last_actions_ = raw_actions;

        // 更新上一时刻速度 (用于下一次观测)
        last_base_lin_vel_body_ = {body_lin_vel.x, body_lin_vel.y, body_lin_vel.z};
        last_base_ang_vel_body_ = {body_ang_vel.x, body_ang_vel.y, body_ang_vel.z};


        // 5. 将原始动作乘以 action_scale 得到物理指令
        float physical_action_dof0 = raw_actions[0] * first_action_scale_;
        float physical_action_dof1 = raw_actions[1] * second_action_scale_;

        // 6. 发布处理后的动作或直接用于控制
        std_msgs::Float32MultiArray actions_msg;
        actions_msg.data.push_back(physical_action_dof0);
        actions_msg.data.push_back(physical_action_dof1);
        actions_pub_.publish(actions_msg);

        // ROS_INFO("Published actions: [%.2f, %.2f]", physical_action_dof0, physical_action_dof1);

        // 在这里，你可以将 physical_action_dof0 (例如目标速度) 和
        // physical_action_dof1 (例如目标位置) 发送给你的底层电机控制器
        // (例如通过PD控制器转换为力矩)
    }


private:
    ros::NodeHandle nh_;
    torch::jit::script::Module module_; // TorchScript 模型

    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber joint_state_sub_; // 如果需要
    ros::Publisher actions_pub_;      // 发布模型输出的动作 (或处理后的物理指令)
    // ros::Publisher control_command_pub_; // 发布给电机驱动

    ros::Timer inference_timer_;

    // 存储最新的传感器数据
    geometry_msgs::Twist current_cmd_vel_;
    nav_msgs::Odometry current_odom_;
    // sensor_msgs::JointState current_joint_states_; // 如果需要
    bool cmd_vel_received_ = false;
    bool odom_received_ = false;
    bool joint_states_received_ = false; // 如果需要

    // 存储上一时刻的状态 (用于观测)
    std::vector<float> last_actions_;
    std::vector<float> last_base_lin_vel_body_;
    std::vector<float> last_base_ang_vel_body_;

    // 观测和动作的维度
    int num_observations_;
    int num_actions_;

    // 缩放参数
    std::vector<float> obs_scales_command_;
    std::vector<float> obs_scales_lin_vel_;
    std::vector<float> obs_scales_ang_vel_;
    float obs_scales_dof_pos_;
    std::vector<float> obs_scales_dof_vel_;
    float obs_scales_projected_gravity_;
    float obs_scales_actions_; // 用于上一时刻动作的观测缩放

    float first_action_scale_;  // 用于将模型输出的action[0]转换为物理值
    float second_action_scale_; // 用于将模型输出的action[1]转换为物理值

    float default_dof_pos_steering_; // 副轴/转向电机的默认位置

    // 用于生成随机数 (如果需要初始化某些值或进行随机扰动测试)
    std.random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<float> dist_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh("~"); // 使用私有节点句柄，方便从launch文件传递参数

    ControllerNode controller(nh);

    ros::spin(); // 处理回调

    return 0;
}