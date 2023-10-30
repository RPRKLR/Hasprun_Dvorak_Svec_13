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
#include <cstdlib>
// #include <unistd.h>

using namespace std::chrono_literals;

struct TaskPoint{
    std::vector<float> x;
    std::vector<float> y;
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

        RCLCPP_INFO(this->get_logger(), "Current state is connected.");

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
        RCLCPP_INFO(this->get_logger(), "Set mode Client is available, setting mode to GUIDED");
        auto result = set_mode_client_->async_send_request(std::make_shared<mavros_msgs::srv::SetMode::Request>(guided_set_mode_req));
        // TODO: Test if drone state really changed to GUIDED
        while(rclcpp::ok() && !current_state_.guided)
        {
            rclcpp::Rate r{10};
            RCLCPP_INFO(this->get_logger(), "Waiting for GUIDED mode request");
            r.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "Mode set to GUIDED");
        auto arm_set = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_set->value = true;
        while (!arming_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }
        
        auto arm_future = arming_client_->async_send_request(arm_set);
        RCLCPP_INFO(this->get_logger(), "Request is sent for ARM");
        RCLCPP_INFO(this->get_logger(), "Waiting for result");
        if(rclcpp::spin_until_future_complete(shared_from_this(), arm_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Got request result from arming: %d", arm_future.get()->result);
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Failed to call service for arming");
        }
        
        // Take off control
        // Creating service message for the takeoff client
        // auto takeoff_set = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        // takeoff_set->min_pitch = 0;
        // takeoff_set->yaw = 90;
        // takeoff_set->altitude = 2;
        // // Check if service is responding
        // while(!takeoff_client_->wait_for_service(1s))
        // {
        //     if (!rclcpp::ok())
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
        //         return;
        //     }
        //     RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        // }

        // auto takeoff_result = takeoff_client_->async_send_request(takeoff_set);
        // RCLCPP_INFO(this->get_logger(), "Request sent for takeoff");
        
        // if(rclcpp::spin_until_future_complete(shared_from_this(), takeoff_result) == rclcpp::FutureReturnCode::SUCCESS)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Got request result: %d", takeoff_result.get()->result);
        // }
        // else 
        // {
        //     RCLCPP_INFO(this->get_logger(), "Failed to call service for takeoff");
        // }

        RCLCPP_INFO(this->get_logger(), "Getting position commands from csv file.");
        getInputFromFile();
        createGoalPointsBasedOnAltitudeAndMaps("map_");

        // TODO: Implement position controller and mission commands here
        // Position controller 
        while(position_count_ < task_points_.size() && rclcpp::ok())
        {
            while(filtered_position_count_ < task_points_[position_count_].x.size())
            {
                if(task_points_[position_count_].precision == "hard")
                    precision_ = 0.05;
                else if(task_points_[position_count_].precision == "soft")
                    precision_ = 0.1;
                // geometry_msgs::msg::PoseStamped goal_pos;
                // goal_pos.pose.position.x = task_points_[position_count_].x;
                // goal_pos.pose.position.y = task_points_[position_count_].y;
                // goal_pos.pose.position.z = task_points_[position_count_].z;
                

                if(task_points_[position_count_].task == "landtakeoff")
                { 
                    landTakeOff(task_points_[position_count_].z);
                }

                // If the task is take off, and the drone is not in motion, and also not at altitude, it will 
                // send the service message to take off to the given task altitude level.
                if(task_points_[position_count_].task == "takeoff" && !is_moving_ && !is_on_altitude_)
                {
                    is_moving_ = true;
                    std::shared_ptr<mavros_msgs::srv::CommandTOL::Request> takeoff_srv;
                    RCLCPP_INFO(this->get_logger(), "AAAA %f", task_points_[position_count_].z);
                    takeoff_srv->altitude = task_points_[position_count_].z;
                    takeOffDrone(takeoff_srv);
                }

                // Set the robot goal position for the actual goal position. 
                if(!is_on_altitude_ && !is_moving_)
                {
                    is_moving_ = true;
                    setDroneGoalDestination(task_points_[position_count_].x[filtered_position_count_], task_points_[position_count_].y[filtered_position_count_], task_points_[position_count_].z);
                    local_pos_pub_->publish(drone_goal_pose);
                }

                // Check that if the drone is at altitude, if true the is_on_altitude variable is set as true, and we can start 
                // the navigation toward the goal position.
                if(!is_on_altitude_ && is_moving_ && sqrt(pow(drone_position_.pose.pose.position.z - drone_goal_pose.pose.position.z, 2)) < precision_)
                {
                    is_moving_ = false;
                    is_on_altitude_ = true;
                    RCLCPP_INFO(this->get_logger(), "Drone is at altitude: %f", drone_position_.pose.pose.position.z);
                }

                // If the drone is on altitude level, but not at the goal position first, we will rotate the drone towards the goal position
                // After the drone is facing the goal position, the drone will fly forward to the goal position.
                if(!is_moving_ && is_on_altitude_ && !is_at_goal_)
                {
                    is_moving_ = true;
                    setDroneOrientation(task_points_[position_count_].x[filtered_position_count_], task_points_[position_count_].y[filtered_position_count_]);
                    local_pos_pub_->publish(drone_goal_pose);
                    // sleep(3);
                    // because sleep(3) does not work in unix
                    std::this_thread::sleep_for(3000ms);
                    setDroneGoalDestination(task_points_[position_count_].x[filtered_position_count_], task_points_[position_count_].y[filtered_position_count_], task_points_[position_count_].z);
                    local_pos_pub_->publish(drone_goal_pose);
                }

                // Check if the robot reached the given goal position with the defined precision
                if(is_moving_ && euclidDistance(drone_position_.pose.pose.position.x, drone_position_.pose.pose.position.y,drone_position_.pose.pose.position.z,  drone_goal_pose.pose.position.x, drone_goal_pose.pose.position.y, drone_goal_pose.pose.position.z) < precision_ && is_on_altitude_ && !is_at_goal_)
                {
                    if (filtered_position_count_ == task_points_[position_count_].x.size() - 1)
                        is_at_goal_ = true;
                    is_moving_ = false;
                    is_at_small_goal_ = true;
                    RCLCPP_INFO(this->get_logger(), "Drone is at position x: %f y: %f z: %f", drone_position_.pose.pose.position.x, drone_position_.pose.pose.position.y, drone_position_.pose.pose.position.z);
                    RCLCPP_INFO(this->get_logger(), "Reached with precision: %f Distance from goal position: %f", precision_, euclidDistance(drone_position_.pose.pose.position.x, drone_position_.pose.pose.position.y,drone_position_.pose.pose.position.z,  drone_goal_pose.pose.position.x, drone_goal_pose.pose.position.y, drone_goal_pose.pose.position.z));
                }

                // if the drone is not moving, and the drone is at goal, we initiate the landing protocol.
                if(task_points_[position_count_].task == "land" && !is_moving_ && is_at_goal_)
                {
                    is_moving_ = true;
                    std::shared_ptr<mavros_msgs::srv::CommandTOL::Request> land_srv;
                    land_srv->altitude = 0.0;
                    landDrone(land_srv);
                }

                // Stop the drone if the current state is not armed, and the task is land
                if(task_points_[position_count_].task == "land" && is_moving_ && !current_state_.armed)
                {
                    is_moving_ = false;
                }

                // if the drone is at the goal position, and it is not moving, we reset the boolean values, and go for the next goal point position.
                if(is_at_goal_ && !is_moving_)
                {
                    is_on_altitude_ = false;
                    is_at_goal_ = false;
                    is_at_small_goal_ = false;
                    filtered_position_count_ = 0;
                    position_count_++;
                    RCLCPP_INFO(this->get_logger(), "Task done, Moving to next goal position");
                    break;
                }
                if(is_at_small_goal_ && !is_moving_)
                {
                    is_on_altitude_ = false;
                    is_at_small_goal_ = false;
                    filtered_position_count_++;
                    RCLCPP_INFO(this->get_logger(), "Reached small goal");
                }

            }
        }   
        RCLCPP_INFO(this->get_logger(), "Went trough all goal position, shuting down");
        rclcpp::shutdown();
    
    }

private:

    /**
     * @brief Get the path positions form the commands file.
     * 
     */
    void getInputFromFile()
    {
        std::fstream file;
        // file.open("path/to/commands", std::fstream::in);
        file.open("/home/pdvorak/school/ros2_ws_hasprun_dvorak_13/src/LRS-FEI-main/resources/points_example.csv", std::fstream::in);
        std::string line;

        while (getline(file, line))
        {
            float temp_x, temp_y, temp_z;
            // Getting the task point x value 
            std::istringstream line_stream(line);
            std::string tmp_container;
            TaskPoint tmp_task_point;
            std::stringstream ss;
            getline(line_stream, tmp_container, ',');
            // std::stringstream ss(tmp_container);
            temp_x = std::stof(tmp_container);
            // ss >> tmp_task_point.x[0];
            tmp_task_point.x.push_back(temp_x);

            // Getting the task point y value
            getline(line_stream, tmp_container, ',');
            temp_y = std::stof(tmp_container);
            
            tmp_task_point.y.push_back(temp_y);

            // Getting the task point z value

            getline(line_stream, tmp_container, ',');
            temp_z = std::stof(tmp_container);
            tmp_task_point.z = temp_z;

            // Getting the task precision
            std::string temp_str;
            ss.clear();
            getline(line_stream, tmp_container, ',');
            ss << tmp_container;
            ss >> temp_str;
            tmp_task_point.precision = temp_str;


            // Getting the task 
            ss.clear();
            getline(line_stream, tmp_container, ',');
            ss << tmp_container;
            ss >> temp_str;
            tmp_task_point.task = temp_str;

            task_points_.push_back(tmp_task_point);
        }
    }

    /// TODO: Implement function to deal with the maps and the path findings
    void createGoalPointsBasedOnAltitudeAndMaps(std::string map_name)
    {
        geometry_msgs::msg::PoseStamped tmp_pos;
        double precision{0.05};
        tmp_pos.pose.position.x = std::round(task_points_[0].x[0] / precision) * precision;
        tmp_pos.pose.position.y = std::round(task_points_[0].y[0] / precision) * precision;
        tmp_pos.pose.position.z = drone_position_.pose.pose.position.z;
        std::vector<float> x_vec, y_vec;
        // for(int j = 0; j < task_points_.size(); j++)
        // std::size_t because otherwise there was a comparison between int and unsigned int
        for(std::size_t j = 0; j < task_points_.size(); j++)
        {
            float x,y;
            x = std::round(task_points_[j].x[0] / precision) * precision;
            y = std::round(task_points_[j].y[0] / precision) * precision;
            char command[1024];
            std::string altitude;
            if(std::round(tmp_pos.pose.position.z*100) >= 0 && std::round(tmp_pos.pose.position.z*100) <75)
                altitude = "025";
            else if(std::round(tmp_pos.pose.position.z*100) >= 75 && std::round(tmp_pos.pose.position.z*100) <100)
                altitude = "080";
            else if(std::round(tmp_pos.pose.position.z*100) >= 100 && std::round(tmp_pos.pose.position.z*100) <125)
                altitude = "125";
            else if(std::round(tmp_pos.pose.position.z*100) >= 125 && std::round(tmp_pos.pose.position.z*100) <175)
                altitude = "150";
            else if(std::round(tmp_pos.pose.position.z*100) >= 175 && std::round(tmp_pos.pose.position.z*100) <200)
                altitude = "180";
            else 
                altitude = "225";
            snprintf(command, sizeof(command), "python3 /home/pdvorak/school/ros2_ws_hasprun_dvorak_13/src/LRS-FEI-main/scripts/map_loader.py %s %s %s %s %s %s %s",
                                                map_name.c_str(),
                                                altitude.c_str(),
                                                std::to_string((int)(tmp_pos.pose.position.x / precision)).c_str(), 
                                                std::to_string((int)(tmp_pos.pose.position.y / precision)).c_str(),
                                                std::to_string((int)(x / precision)).c_str(),
                                                std::to_string((int)(y / precision)).c_str(),
                                                "simplified_points.csv");

            int return_code = system(command);

            if(return_code == 0)
                RCLCPP_INFO(this->get_logger(), "Python script executed successfully");
            else 
                RCLCPP_ERROR(this->get_logger(), "Error executing the python script");
            std::fstream file;
            file.open("/home/pdvorak/school/ros2_ws_hasprun_dvorak_13/src/LRS-FEI-main/scripts/simplified_points.csv", std::fstream::in);
            // file.open("/home/lrs-ubuntu/LRS/Hasprun_Dvorak_13/src/LRS-FEI-main/scripts/simplified_points.csv", std::fstream::in);
            std::string line;
            std::vector<float> tmp_vec;
            while (getline(file, line))
            {
                float tmp_val;
                std::istringstream line_stream(line);
                std::string token;

                std::getline(line_stream, token, ',');
                std::stringstream ss(token);
                ss >> tmp_val;
                x_vec.push_back(tmp_val * precision);


                std::getline(line_stream, token, ',');
                ss.clear();
                ss << token;
                ss >> tmp_val;
                y_vec.push_back(tmp_val * precision);

            }
            task_points_[j].x = x_vec;
            task_points_[j].y = y_vec;
            tmp_pos.pose.position.x = task_points_[j].x[task_points_[j].x.size() - 1];
            tmp_pos.pose.position.y = task_points_[j].y[task_points_[j].y.size() - 1];
            tmp_pos.pose.position.z = task_points_[j].z;
        }

    }

    /**
     * @brief Set the drone orientation
     * 
     * @param x x coordinate of the goal position.
     * @param y y coordinate of the goal position.
     */
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

    /**
     * @brief Set the goal position to navigate
     * 
     * @param x X coordinate of the goal position
     * @param y Y coordinate of the goal position
     * @param z Z coordinate of the goal position
     */
    void setDroneGoalDestination(float &x, float &y, float &z)
    {
        drone_goal_pose.pose.position.x = x;
        drone_goal_pose.pose.position.y = y;
        drone_goal_pose.pose.position.z = z;
    }

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        drone_position_.pose.pose.position.x = msg->pose.position.x;
        drone_position_.pose.pose.position.y = msg->pose.position.y;
        drone_position_.pose.pose.position.z = msg->pose.position.z;
    }

    /**
     * @brief Calculate the euclid distance from one 3D point to another 3D point
     * 
     * @param x1 First x coordinate
     * @param y1 First y coordinate
     * @param z1 First z coordinate
     * @param x2 Second x coordinate
     * @param y2 Second y coordinate
     * @param z2 Second z coordinate
     * @return double Returns the distance between two points.
     */
    double euclidDistance(float x1, float y1, float z1, float x2, float y2, float z2)
    {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2)); 
    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
    }

    /**
     * @brief Sends the request for the takeoff client, waits until the server sends back the result.
     * if the landing failed, the node shuts down.
     * 
     * @param srv 
     */
    void landDrone(std::shared_ptr<mavros_msgs::srv::CommandTOL::Request> srv)
    {

        auto land_future = takeoff_client_->async_send_request(srv);
        auto result = land_future.get();
        
        if(result->success)
        {
            RCLCPP_INFO(this->get_logger(), "Land sent");
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Land failed");
            rclcpp::shutdown();
        }
    }

    /**
     * @brief Sends the request for the takeoff server, gets the result from the future.
     * If the result is successful the drone will fly to the given altitude, else it will do nothing.
     * 
     * @param srv 
     */
    void takeOffDrone(std::shared_ptr<mavros_msgs::srv::CommandTOL::Request> srv)
    {
        auto takeoff_future = takeoff_client_->async_send_request(srv);
        auto result = takeoff_future.get();
        
        if(result->success)
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff sent to altitude: %f", srv->altitude);
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff failed");
        }

    }

    /**
     * @brief Land the drone, and after the landing, the drone will take off to the given altitude.
     * 
     * @param altitude 
     */
    void landTakeOff(float altitude)
    {
        std::shared_ptr<mavros_msgs::srv::CommandTOL::Request> srv;
        landDrone(srv);
        srv->altitude = altitude;
        takeOffDrone(srv);
        position_count_++;
    }

    std::vector<TaskPoint> task_points_;
    nav_msgs::msg::Odometry drone_position_;
    geometry_msgs::msg::PoseStamped drone_goal_pose;

    bool is_on_altitude_{false};
    bool is_at_goal_{false};
    bool is_at_small_goal_{false};
    bool is_moving_{false};
    // Soft precision is 10 cm, hard is 5 cm.
    double precision_{0.1};
    // int position_count_{0};
    // std::size_t because otherwise there was a comparison between int and unsigned int
    std::size_t position_count_{0};
    // int filtered_position_count_{0};
    std::size_t filtered_position_count_{0};

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