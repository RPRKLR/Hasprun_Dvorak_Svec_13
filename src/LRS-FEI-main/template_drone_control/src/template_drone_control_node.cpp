#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <sstream>
#include <fstream>
#include <string>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

struct TaskPoint{
    float x;
    float y;
    float z;
    std::string precision;
    std::string task;
};

class TemplateDroneControl : public rclcpp::Node
{
public:
    TemplateDroneControl() : Node("template_drone_control_node")
    {
        // Set up ROS publishers, subscribers and service clients
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&TemplateDroneControl::state_cb, this, std::placeholders::_1));
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        custom_qos.depth = 1;
        custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(custom_qos.history, 1), custom_qos);
        local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/mavros/local_position/pose", qos, std::bind(&TemplateDroneControl::local_pos_cb, this, std::placeholders::_1));

        // Wait for MAVROS SITL connection
        while (rclcpp::ok() && !current_state_.connected)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }

        mavros_msgs::srv::SetMode::Request guided_set_mode_req;
        guided_set_mode_req.custom_mode = "GUIDED";
        while (!set_mode_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }
        auto result = set_mode_client_->async_send_request(std::make_shared<mavros_msgs::srv::SetMode::Request>(guided_set_mode_req));

        // TODO: Test if drone state really changed to GUIDED

        // TODO: Arm and Take Off
        RCLCPP_INFO(this->get_logger(), "Sending position command");
        // TODO: Implement position controller and mission commands here
    }

private:

    void getInputFromFile()
    {
        std::fstream file;
        file.open("path/to/commands", std::fstream::in);
        std::string line;

        while (getline(file, line))
        {
            std::istringstream line_stream(line);
            std::string tmp_container;
            TaskPoint tmp_task_point;
            getline(line_stream, tmp_container, ',');
            std::stringstream ss(tmp_container);
            ss >> tmp_task_point.x;
            
            // getline(line_stream, tmp_container, ',');
            // std::stringstream ss(tmp_container);
            // ss >> tmp_task_point.y;

            // getline(line_stream, tmp_container, ',');
            // std::stringstream ss(tmp_container);
            // ss >> tmp_task_point.z;

            // getline(line_stream, tmp_container, ',');
            // std::stringstream ss(tmp_container);
            // ss >> tmp_task_point.precision;

            // getline(line_stream, tmp_container, ',');
            // std::stringstream ss(tmp_container);
            // ss >> tmp_task_point.task;
        
            task_points_.push_back(tmp_task_point);
        }
    }

    void setDroneOrientation(float &x, float &y)
    {
        // Delta x
        double d_x = drone_position_.pose.pose.position.x - x;
        // Delta y
        double d_y = drone_position_.pose.pose.position.y - y;
        float yaw = atan2(d_y, d_x);
        float pitch = 0.0, roll = 0.0;
    
        float cy = cos(yaw * 0.5);
        float sy = sin(yaw * 0.5);
        float cr = cos(roll * 0.5);
        float sr = sin(roll * 0.5);
        float cp = cos(pitch * 0.5);
        float sp = sin(pitch * 0.5);

        float qw = cy * cr * cp + sy * sr * sp;
        float qx = cy * sr * cp - sy * cr * sp;
        float qy = cy * cr * sp + sy * sr * cp;
        float qz = sy * cr * cp - cy * sr * sp;
        
        drone_goal_pose.pose.orientation.w = qw;
        drone_goal_pose.pose.orientation.x = qx;
        drone_goal_pose.pose.orientation.y = qy;
        drone_goal_pose.pose.orientation.z = qz;
        drone_goal_pose.pose.position.x = drone_position_.pose.pose.position.x;
        drone_goal_pose.pose.position.y = drone_position_.pose.pose.position.y;
        drone_goal_pose.pose.position.z = drone_position_.pose.pose.position.z;
        RCLCPP_INFO(this->get_logger(), "Drone heading angle has been set to target");
        RCLCPP_INFO(this->get_logger(), "Orientation(rad): %f", yaw);
    }

    void setDroneGoalDestination(float &x, float &y, float &z)
    {
        drone_goal_pose.pose.position.x = x;
        drone_goal_pose.pose.position.y = y;
        drone_goal_pose.pose.position.z = z;
    }

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped current_local_pos_ = *msg;

        // To obtain the position of the drone use this data fields withing the message, please note, that this is the local position of the drone in the NED frame so it is different to the map frame
        // current_local_pos_.pose.position.x
        // current_local_pos_.pose.position.y
        // current_local_pos_.pose.position.z
        // you can do the same for orientation, but you will not need it for this seminar


        RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f", current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
    }
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
    }

    std::vector<TaskPoint> task_points_;
    nav_msgs::msg::Odometry drone_position_;
    geometry_msgs::msg::PoseStamped drone_goal_pose;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;


    mavros_msgs::msg::State current_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateDroneControl>());
    rclcpp::shutdown();
    return 0;
}