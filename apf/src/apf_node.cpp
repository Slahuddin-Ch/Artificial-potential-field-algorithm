#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <cmath>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>


double position_x = 0;
double position_y = 0;
double position_z = 0;

// Define the polynomial function for the path (cubic polynomial)
double pathFunction(double x, const std::vector<double>& coefficients) {
    return coefficients[0] * std::pow(x, 3) + coefficients[1] * std::pow(x, 2) +
           coefficients[2] * x + coefficients[3];
}

// Calculate the attractive force towards the path
std::vector<double> attractiveForce(double x, double y, double pathX, const std::vector<double>& coefficients) {
    double pathY = pathFunction(pathX, coefficients);
    double forceX = -1.0 * (x - pathX); // Attractive force towards the path in x
    double forceY = -1.0 * (y - pathY); // Attractive force towards the path in y
    return {forceX, forceY};
}

// Calculate the repulsive force away from obstacles
std::vector<double> repulsiveForce(double x, double y, const std::vector<std::vector<double>>& obstacle_positions, double obstacle_radius) {
    double forceX = 0;
    double forceY = 0;

    for (const auto& obstacle : obstacle_positions) {
        double distance = std::hypot(x - obstacle[0], y - obstacle[1]);
        if (distance < obstacle_radius && distance > 0) { // Check to avoid division by zero
            double repulsionStrength = 1.0 / (distance * distance); // Repulsion decreases with distance squared
            forceX += repulsionStrength * (x - obstacle[0]);
            forceY += repulsionStrength * (y - obstacle[1]);
        }
    }

    return {forceX, forceY};
}

// Main function to move the vehicle along the path towards the goal
void followPath(ros::ServiceClient& client, double startX, double startY, const std::vector<double>& coefficients, const std::vector<std::vector<double>>& obstacle_positions, double obstacle_radius, double goalX, double goalY) {
    double x = startX;  // Vehicle's current x position
    double y = startY;  // Vehicle's current y position

    ros::Rate rate(1); // Control loop rate
    while (ros::ok()) {


        double pathX = x;   // This will be used to calculate the current target on the path
        // Update the target point on the path
        pathX = std::min(pathX + 0.1, goalX);  // Increment pathX towards goalX, but do not exceed goalX
        double pathY = pathFunction(pathX, coefficients); // Calculate the corresponding y on the path

        // Calculate the attractive force towards the current target point on the path
        auto attr_force = attractiveForce(x, y, pathX, coefficients);

        // Check for obstacles and calculate repulsive force if any are within the obstacle radius
        std::vector<double> rep_force = {0.0, 0.0};
        for (const auto& obstacle : obstacle_positions) {
            double distance = std::hypot(x - obstacle[0], y - obstacle[1]);
            if (distance < obstacle_radius) {
                auto temp_rep_force = repulsiveForce(x, y, obstacle_positions, obstacle_radius);
                rep_force[0] += temp_rep_force[0];
                rep_force[1] += temp_rep_force[1];
            }
        }

        // Compute the resultant force
        double forceX = attr_force[0] + rep_force[0];
        double forceY = attr_force[1] + rep_force[1];

        // Normalize and apply the forces to the vehicle's current position
        double norm = std::hypot(forceX, forceY);
        if (norm > 0) {
            forceX /= norm;  // Normalize the force vector
            forceY /= norm;
        }

        // Apply a speed factor to determine the velocity of the vehicle
        double speed = 1; // You can adjust this speed as necessary
        forceX *= speed;
        forceY *= speed;

        // Update the vehicle's position based on the command issued



        x += 0.1 * forceX;
        y += 0.1 * forceY; 


        // Define the desired state
        gazebo_msgs::SetModelState set_state;
        set_state.request.model_state.model_name = "mr_robot"; // Replace with your robot's name
        set_state.request.model_state.pose.position.x = x; // Replace with desired X
        set_state.request.model_state.pose.position.y = y; // Replace with desired Y
        set_state.request.model_state.pose.position.z = 0; // Z is typically 0 for ground robots
        set_state.request.model_state.pose.orientation.w = 1; // No rotation; facing forward
        // Set other states as needed, like velocity, if you want to give an initial motion

        // Call the service
        if (client.call(set_state)) {
            ROS_INFO("Robot state updated successfully.");
        } else {
            ROS_ERROR("Failed to update robot state.");
        }

        // Log for debugging
        ROS_INFO("");
        ROS_INFO("Attractive force: (%f, %f) X %f , Y %f ", attr_force[0], attr_force[1], pathX, pathY);
        ROS_INFO("Repulsive force: (%f, %f)", rep_force[0], rep_force[1]);
        ROS_INFO("Vehicle position: (%f, %f)", x, y);

        // Check if the vehicle has reached the goal area
        if (std::hypot(x - goalX, y - goalY) < obstacle_radius) {
            ROS_INFO("Vehicle has reached the goal.");
            break; // Exit the loop if the goal is reached
        }

        ros::spinOnce();
        rate.sleep(); // Sleep to maintain the loop rate
    }
}


void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    int index = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "mr_robot") {  // Replace 'robot_name' with your robot's name
            index = i;
            break;
        }
    }

    if (index != -1) {
         position_x = msg->pose[index].position.x;
         position_y = msg->pose[index].position.y;
         position_z = msg->pose[index].position.z;

        // Here you can process the position data as needed
        // ROS_INFO("Robot position Callback: x=%f, y=%f, z=%f", position_x, position_y, position_z);
    } else {
        // ROS_WARN("Robot not found in model states!");
    }
}





int main(int argc, char** argv) {
    ros::init(argc, argv, "apf_controller");
    ros::NodeHandle nh;

    // Define the robot client
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");


    ros::NodeHandle nh1;
    ros::Subscriber sub = nh1.subscribe("/gazebo/model_states", 10, modelStatesCallback);



    // Define the polynomial coefficients
    std::vector<double> coefficients = {1, -6, 9, -2}; // For y = x^3

    // Define the starting position
    double startX = 0;
    double startY = -2;

    // Define obstacle positions
    std::vector<std::vector<double>> obstacle_positions = {{1, -3}, {2, 3}, {5, -3}};

    //std::vector<std::vector<double>> obstacle_positions = {};

    // Define the obstacle radius
    double obstacle_radius = 0.1;

    // Define the goal X position (Y is calculated from the polynomial)
    double goalX = 4;
    double goalY = pathFunction(goalX, coefficients);

    // Call the function to follow the path
    followPath(client, startX, startY, coefficients, obstacle_positions, obstacle_radius, goalX, goalY);

    return 0;
}
