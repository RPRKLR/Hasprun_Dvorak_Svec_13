#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <fstream>
#include <cstdio>
#include <iostream>

using namespace std::chrono_literals;

class TemplateDroneControl : public rclcpp::Node
{
public:
    TemplateDroneControl() : Node("template_drone_control_node")
    {
        // Set up ROS publishers, subscribers and service clients
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&TemplateDroneControl::state_cb, this, std::placeholders::_1));
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        position_target_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
        land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");
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

        read_mission_csv("/home/lrs-ubuntu/LRS/Hasprun_Dvorak_13/src/LRS-FEI-main/resources/points_example.csv");

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
        mode_check("GUIDED");
        
        current_point = 0;
        // start_x = task_points_[0].x;
        // start_y = task_points_[0].y;
        previous_x = start_x;
        previous_y = start_y;
        TaskPoint current_task_point;
        float precision;
        float altitude;
        std::string current_map;
        while (current_point < (int) task_points_.size())
        {
            if (!first_check)
            {
                current_task_point = task_points_[current_point];
                if (current_task_point.precision == "hard")
                {
                    precision = precision_hard;
                } else if (current_task_point.precision == "soft")
                {
                    precision = precision_soft;
                }
                altitude = current_task_point.z;
                if (abs(altitude - current_local_pos_.pose.position.z) <= precision)
                {
                    is_at_altitude = true;
                }
                first_check = true;
            }
            if (current_task_point.task == "takeoff" && !is_at_altitude)
            {
                RCLCPP_INFO(this->get_logger(), "Received takeoff command");
                RCLCPP_INFO(this->get_logger(), "Arming drone");
                arm(true);
                RCLCPP_INFO(this->get_logger(), "Drone armed");
                takeoff(0, yaw, altitude);
                check_altitude(altitude, precision);
                is_at_altitude = true;
                // is_at_position = true;
                current_map = choose_map(altitude);

            }
            if (current_task_point.task == "landtakeoff")
            {
                RCLCPP_INFO(this->get_logger(), "Received landtakeoff command");
                std::string trajectory_path = generate_trajectory(current_map, previous_x, previous_y, current_task_point.x, current_task_point.y);
                read_points_csv(trajectory_path);
                move_through_points(precision, current_task_point.x, current_task_point.y);
                is_at_position = true;
                land();
                check_land();
                RCLCPP_INFO(this->get_logger(), "Arming drone");
                arm(true);
                RCLCPP_INFO(this->get_logger(), "Drone armed");
                takeoff(0, yaw, altitude);
                check_altitude(altitude, precision);
                is_at_altitude = true;
                current_map = choose_map(altitude);
            }
            if (current_task_point.task == "land")
            {
                RCLCPP_INFO(this->get_logger(), "Received land command");
                std::string trajectory_path = generate_trajectory(current_map, previous_x, previous_y, current_task_point.x, current_task_point.y);
                read_points_csv(trajectory_path);
                move_through_points(precision, current_task_point.x, current_task_point.y);
                is_at_position = true;                
                land();
                check_land();
                arm(false);
                is_at_altitude = true;
            }
            if (!is_at_altitude)
            {
                move(current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_task_point.z, yaw);
                check_altitude(current_task_point.z, precision);
                is_at_altitude = true;
                current_map = choose_map(altitude);
            } 
            if (!is_at_position)
            {
                std::string trajectory_path = generate_trajectory(current_map, previous_x, previous_y, current_task_point.x, current_task_point.y);
                read_points_csv(trajectory_path);
                move_through_points(precision, current_task_point.x, current_task_point.y);
                is_at_position = true;
            }
            if (is_at_altitude && is_at_position)
            {
                is_at_altitude = false;
                is_at_position = false;
                first_check = false;
                previous_x = current_task_point.x;
                previous_y = current_task_point.y;
                current_point++;
                mid_points_.clear();
                RCLCPP_INFO(this->get_logger(), "Current point: %d", current_point+1);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Mission complete!");
    }

private:

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_local_pos_ = *msg;

        // To obtain the position of the drone use this data fields withing the message, please note, that this is the local position of the drone in the NED frame so it is different to the map frame
        // current_local_pos_.pose.position.x
        // current_local_pos_.pose.position.y
        // current_local_pos_.pose.position.z
        // you can do the same for orientation, but you will not need it for this seminar


        // RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f", current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
    }
    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        // RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
    }
    void mode_check(std::string mode)
    {
        while (rclcpp::ok() && current_state_.mode != mode)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(500ms);
            RCLCPP_INFO(this->get_logger(), "Waiting for mode to be set to %s", mode.c_str());
        }
    }
    void arm(bool value)
    {
        mavros_msgs::srv::CommandBool::Request arm_req;
        arm_req.value = value;
        while (!arming_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the arming_client service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for arming_client service...");
        }
        auto arm_result = arming_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandBool::Request>(arm_req));

        while (rclcpp::ok() && current_state_.armed != value)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(1000ms);
            RCLCPP_INFO(this->get_logger(), "Waiting for drone to be armed=%d", value);
        }
    }
    void takeoff(float min_pitch, float yaw, float altitude)
    {
        mavros_msgs::srv::CommandTOL::Request takeoff_req;
        takeoff_req.min_pitch = min_pitch;
        takeoff_req.yaw = yaw;
        takeoff_req.altitude = altitude;
        while (!takeoff_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the takeoff_client service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for takeoff_client service...");
        }
        RCLCPP_INFO(this->get_logger(), "Sending takeoff request");
        auto takeoff_result = takeoff_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(takeoff_req));
    }
    void land()
    {
        mavros_msgs::srv::CommandTOL::Request land_req;
        while (!land_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the land_client service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for land_client service...");
        }
        RCLCPP_INFO(this->get_logger(), "Sending land request");
        auto land_result = land_client_->async_send_request(std::make_shared<mavros_msgs::srv::CommandTOL::Request>(land_req));
    }
    void move(float x, float y, float z, float yaw)
    {
        auto message = mavros_msgs::msg::PositionTarget();
        message.header.stamp.sec = 0;
        message.header.stamp.nanosec = 0;
        message.header.frame_id = "";
        message.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        message.type_mask = 0b0000101111111000;
        message.position.x = x;
        message.position.y = y;
        message.position.z = z;
        message.yaw = yaw;
        RCLCPP_INFO(this->get_logger(), "Moving to x=%f, y=%f, z=%f", x, y, z);
        position_target_pub_->publish(message);
    }
    void check_altitude(float altitude, float precision)
    {
        while (rclcpp::ok() && abs(altitude - current_local_pos_.pose.position.z) > precision)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for drone to reach altitude=%f with precision %f", altitude, precision);
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(1000ms);
        }
        RCLCPP_INFO(this->get_logger(), "Drone reached altitude=%f with precision %f", altitude, precision);
    }
    void check_land()
    {
        float land_val = current_local_pos_.pose.position.z;
        bool is_landed = false;
        mode_check("LAND");
        while (rclcpp::ok() && current_state_.mode == "LAND" && !is_landed)
        {
            float curr_land_val = current_local_pos_.pose.position.z;
            if(abs(land_val - curr_land_val) > 0.05)
            {
                land_val = curr_land_val;
            }
            else
            {
                is_landed = true;
            }
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(1000ms);
            RCLCPP_INFO(this->get_logger(), "Waiting for drone to land and disarm");
        }
        RCLCPP_INFO(this->get_logger(), "Drone landed");
    }
    void check_position(float x, float y, float precision)
    {
        while (rclcpp::ok() && euclid_distance(x, current_local_pos_.pose.position.x, y, current_local_pos_.pose.position.y) > precision)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for drone to reach position x=%f, y=%f with precision %f", x, y, precision);
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(1000ms);
        }
        RCLCPP_INFO(this->get_logger(), "Drone reached position x=%f, y=%f with precision %f", x, y, precision);
    }
    float euclid_distance(float x1, float x2, float y1, float y2)
    {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }
    std::string choose_map(float altitude)
    {
        // std::string path = "/home/lrs-ubuntu/LRS/Hasprun_Dvorak_13/src/LRS-FEI-main/scripts/";
        std::string path = "";
        if (altitude * 100 >= 0 && altitude * 100 < 75) 
        {
            path = path + "025";
        } else if (altitude * 100 >= 75 && altitude * 100 < 100)
        {
            path = path + "080";
        } else if (altitude * 100 >= 100 && altitude * 100 < 125)
        {
            path = path + "125";
        } else if (altitude * 100 >= 125 && altitude * 100 < 175)
        {
            path = path + "150";
        } else if (altitude * 100 >= 175 && altitude * 100 < 200)
        {
            path = path + "180";
        } else 
        {
            path = path + "225";
        }

        RCLCPP_INFO(this->get_logger(), "Selected map: %s", path.c_str());
        return path;
    }
    std::string generate_trajectory(std::string altitude, float x_start, float y_start, float x_end, float y_end)
    {
        char command[1024];
        RCLCPP_INFO(this->get_logger(), "Python INSIDE x_start=%f, y_start=%f, x_end=%f, y_end=%f", x_start, y_start, x_end, y_end);
        snprintf(command, sizeof(command), "python3 /home/lrs-ubuntu/LRS/Hasprun_Dvorak_13/src/LRS-FEI-main/scripts/map_loader.py %s %s %s %s %s %s %s",
                                                "map_",
                                                altitude.c_str(),
                                                std::to_string((int) (x_start*100/5)).c_str(), 
                                                std::to_string((int) (y_start*100/5)).c_str(),
                                                std::to_string((int) (x_end*100/5)).c_str(),
                                                std::to_string((int) (y_end*100/5)).c_str(),
                                                "trajectory_points.csv");
        
        int return_code = system(command);
        if(return_code == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Python script executed successfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Error executing the python script");
        }
        
        return "/home/lrs-ubuntu/LRS/Hasprun_Dvorak_13/trajectory_points.csv";
    }
    void read_mission_csv(std::string path)
    {
        std::ifstream file;
        file.open(path);
        std::string line;
        // getline(file, line);
        RCLCPP_INFO(this->get_logger(), "Loading file %s", path.c_str());
        while (getline(file, line)) 
        {
            std::istringstream ss(line);

            std::vector<std::string> split_parts;
            std::string token;

            while (getline(ss, token, ',')) 
            {
                split_parts.push_back(token);
            }

            TaskPoint tmp_task_point;
            tmp_task_point.x = std::stof(split_parts[1]);
            tmp_task_point.y = std::stof(split_parts[0]);
            tmp_task_point.z = std::stof(split_parts[2]);

            tmp_task_point.precision = split_parts[3];
            std::string::iterator end_pos = std::remove(tmp_task_point.precision.begin(), tmp_task_point.precision.end(), ' ');
            tmp_task_point.precision.erase(end_pos, tmp_task_point.precision.end());

            tmp_task_point.task = split_parts[4];
            end_pos = std::remove(tmp_task_point.task.begin(), tmp_task_point.task.end(), ' ');
            tmp_task_point.task.erase(end_pos, tmp_task_point.task.end());

            task_points_.push_back(tmp_task_point);
        }
        file.close();
    }
    void read_points_csv(std::string path)
    {
        std::ifstream file;
        file.open(path);
        std::string line;
        // getline(file, line);
        RCLCPP_INFO(this->get_logger(), "Loading file %s", path.c_str());
        while (getline(file, line)) 
        {
            std::istringstream ss(line);

            std::vector<std::string> split_parts;
            std::string token;

            while (getline(ss, token, ',')) 
            {
                split_parts.push_back(token);
            }

            MidPoints tmp_mid_point;
            tmp_mid_point.x = std::stof(split_parts[0])*5/100;
            tmp_mid_point.y = std::stof(split_parts[1])*5/100;

            mid_points_.push_back(tmp_mid_point);
        }
        file.close();
    }
    void move_through_points(float precision, float current_task_point_x, float current_task_point_y)
    {
        for(int k = 1; k < (int) mid_points_.size(); k++)
        {
            RCLCPP_INFO(this->get_logger(), "k: %d", k);
            RCLCPP_INFO(this->get_logger(), "x: %f", mid_points_[k].x);
            RCLCPP_INFO(this->get_logger(), "y: %f", mid_points_[k].y);
        }
        if((int) mid_points_.size() > 1)
        {   
            int point_iteration = 1;
            while(point_iteration < (int) mid_points_.size())
            {
                if(point_iteration == (int)(mid_points_.size())-1)
                {
                    move(current_task_point_x - start_x, current_task_point_y - start_y, current_local_pos_.pose.position.z, yaw);
                    check_position(current_task_point_x - start_x, current_task_point_y - start_y, precision);
                }
                else
                {
                    move(mid_points_[point_iteration].x - start_x, mid_points_[point_iteration].y - start_y, current_local_pos_.pose.position.z, yaw);
                    check_position(mid_points_[point_iteration].x - start_x, mid_points_[point_iteration].y - start_y, precision);
                }
                point_iteration++;
            }
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr position_target_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_local_pos_;

    struct TaskPoint{
        float x;
        float y;
        float z;
        std::string precision;
        std::string task;
    };

    struct MidPoints{
        float x;
        float y;
    };

    std::vector<TaskPoint> task_points_;
    std::vector<MidPoints> mid_points_;
    int current_point = -1;
    bool is_at_altitude = false;
    bool is_at_position = false;
    bool is_landed = false;
    float precision_hard = 0.05;
    float precision_soft = 0.1;
    float precision_default = 0;
    // float yaw = 90;
    float yaw = 0;
    // 285 240
    float start_x = 250*5/100;
    float start_y = 300*5/100;
    float previous_x = 0;
    float previous_y = 0;
    bool first_check = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateDroneControl>());
    rclcpp::shutdown();
    return 0;
}